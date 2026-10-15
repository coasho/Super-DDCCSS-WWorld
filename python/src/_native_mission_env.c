/* Mission episodes over the C ABI (fsim/fsim_mission_env.h; docs/sdk/mission-training.md, section 3; M5 core): three
 * types - the range's episode, an actor's endpoint, the referee's documents - each one handle and nothing else. A
 * document read is the C ABI's two-call protocol; the text crosses as str, so identities stay exact. Reset and step
 * run without the GIL: another thread's call meanwhile finds the episode busy (fsim.Error) and never waits.
 *
 * Lifetime. An episode and every handle it opened share one record: the episode's native handle, and a count of its
 * resets and steps running without the GIL. close() never waits: it marks the record closed at once - the episode and
 * all its handles answer "closed" from then on - and destroys the native episode now, or, while a reset or step still
 * runs on it, lets that call destroy it when it returns. No call reads the native handle from its object once it has
 * released the GIL. The record also says whose an endpoint is (MissionEpisode._owns): its own episode's handle, not an
 * equal service id or episode key. */
#define FSIM_PY_NO_BUFFER_TYPE
#include "_native_world.h"

#include "fsim/fsim_mission_env.h"

#include <stdatomic.h>
#include <stdlib.h>

/* One episode's lifetime, shared by its object and its handles' objects. `refs` changes under the GIL; `state` is 2 per
 * reset or step running without the GIL, plus 1 once closed, and changes without it too. */
typedef struct {
    Py_ssize_t refs;
    atomic_long state;
    fsim_mission_episode* episode;
} MissionLifetime;

static MissionLifetime* lifetime_new(fsim_mission_episode* episode) {
    MissionLifetime* life = (MissionLifetime*)malloc(sizeof(MissionLifetime));
    if (!life) return NULL;
    life->refs = 1;
    atomic_init(&life->state, 0);
    life->episode = episode;
    return life;
}
static void lifetime_retain(MissionLifetime* life) { ++life->refs; }
static void lifetime_release(MissionLifetime* life) {
    if (life && --life->refs == 0) free(life);
}
static int lifetime_closed(MissionLifetime* life) { return !life || (atomic_load(&life->state) & 1) != 0; }
/* Closed once: the native episode goes now, unless a reset or step runs on it - then that call ends it on return. */
static void lifetime_close(MissionLifetime* life) {
    if (!life) return;
    const long before = atomic_fetch_or(&life->state, 1);
    if (before == 0) fsim_mission_episode_destroy(life->episode);
}
/* Under the GIL, before releasing it: the native handle this call uses, or none when the episode is closed. */
static fsim_mission_episode* lifetime_enter(MissionLifetime* life) {
    if (lifetime_closed(life)) return NULL;
    atomic_fetch_add(&life->state, 2);
    return life->episode;
}
/* Without the GIL, after the native call: the last running call of a closed episode ends it. */
static void lifetime_leave(MissionLifetime* life, fsim_mission_episode* episode) {
    if (atomic_fetch_sub(&life->state, 2) == 3) fsim_mission_episode_destroy(episode);
}

typedef struct {
    PyObject_HEAD
    MissionLifetime* life;
} MissionEpisodeObject;

typedef struct {
    PyObject_HEAD
    fsim_mission_endpoint* endpoint;
    uint64_t service_id;
    MissionLifetime* life; /* its episode's */
} MissionEndpointObject;

typedef struct {
    PyObject_HEAD
    fsim_mission_referee* referee;
    MissionLifetime* life; /* its episode's */
} MissionRefereeObject;

static PyTypeObject* MissionEndpointType;
static PyTypeObject* MissionRefereeType;

static PyObject* closed(const char* what) {
    char text[96];
    PyOS_snprintf(text, sizeof text, "%s is closed", what);
    fsim_set_last_error(text);
    return fail();
}

/* A document's text from a two-call read; read again if it grew between the calls. */
typedef int (*document_read)(void* handle, char* buffer, size_t capacity, size_t* needed);
static PyObject* read_document(void* handle, document_read read) {
    for (int attempt = 0; attempt < 4; ++attempt) {
        size_t needed = 0;
        if (read(handle, NULL, 0, &needed) != FSIM_OK) return fail();
        char* buffer = (char*)malloc(needed);
        if (!buffer) return PyErr_NoMemory();
        size_t got = needed;
        const int rc = read(handle, buffer, needed, &got);
        if (rc == FSIM_OK) {
            PyObject* text = PyUnicode_DecodeUTF8(buffer, (Py_ssize_t)(got - 1), "strict");
            free(buffer);
            return text;
        }
        free(buffer);
        if (got <= needed) return fail();
    }
    fsim_set_last_error("the document kept changing while it was read");
    return fail();
}

/* str (UTF-8) or bytes, as a pointer and a length. */
static int as_document(PyObject* o, const char** data, Py_ssize_t* size) {
    if (PyUnicode_Check(o)) {
        *data = PyUnicode_AsUTF8AndSize(o, size);
        return *data != NULL;
    }
    if (PyBytes_Check(o)) {
        char* p = NULL;
        if (PyBytes_AsStringAndSize(o, &p, size) < 0) return 0;
        *data = p;
        return 1;
    }
    PyErr_SetString(PyExc_TypeError, "a document is str or bytes");
    return 0;
}

/* --- MissionEpisode ------------------------------------------------------------------------------------------------ */

static PyObject* episode_new(PyTypeObject* type, PyObject* args, PyObject* kwargs) {
    PyObject* config = NULL;
    if (kwargs && PyDict_Size(kwargs)) {
        PyErr_SetString(PyExc_TypeError, "MissionEpisode(config_json)");
        return NULL;
    }
    if (!PyArg_ParseTuple(args, "O:MissionEpisode", &config)) return NULL;
    const char* data = NULL;
    Py_ssize_t size = 0;
    if (!as_document(config, &data, &size)) return NULL;
    fsim_mission_episode* episode = NULL;
    if (fsim_mission_episode_create(data, (size_t)size, &episode) != FSIM_OK) return fail();
    MissionLifetime* life = lifetime_new(episode);
    if (!life) {
        fsim_mission_episode_destroy(episode);
        return PyErr_NoMemory();
    }
    allocfunc alloc = (allocfunc)PyType_GetSlot(type, Py_tp_alloc);
    MissionEpisodeObject* self = (MissionEpisodeObject*)alloc(type, 0);
    if (!self) {
        lifetime_close(life);
        lifetime_release(life);
        return NULL;
    }
    self->life = life;
    return (PyObject*)self;
}

static void episode_dealloc(PyObject* o) {
    PyTypeObject* tp = Py_TYPE(o);
    MissionEpisodeObject* self = (MissionEpisodeObject*)o;
    lifetime_close(self->life);
    lifetime_release(self->life);
    freefunc f = (freefunc)PyType_GetSlot(tp, Py_tp_free);
    f(o);
    Py_DECREF(tp);
}

static PyObject* episode_reset(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    MissionLifetime* life = ((MissionEpisodeObject*)o)->life;
    uint64_t index = 0;
    if (!check_args(n, 1, 1, "reset") || !as_u64(args[0], &index)) return NULL;
    fsim_mission_episode* episode = lifetime_enter(life);
    if (!episode) return closed("this MissionEpisode");
    int rc;
    Py_BEGIN_ALLOW_THREADS
    rc = fsim_mission_episode_reset(episode, index);
    lifetime_leave(life, episode);
    Py_END_ALLOW_THREADS
    if (rc != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

static PyObject* episode_step(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    MissionLifetime* life = ((MissionEpisodeObject*)o)->life;
    (void)args;
    if (!check_args(n, 0, 0, "step")) return NULL;
    fsim_mission_episode* episode = lifetime_enter(life);
    if (!episode) return closed("this MissionEpisode");
    int rc;
    Py_BEGIN_ALLOW_THREADS
    rc = fsim_mission_episode_step(episode);
    lifetime_leave(life, episode);
    Py_END_ALLOW_THREADS
    if (rc != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

static PyObject* episode_endpoint(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    MissionLifetime* life = ((MissionEpisodeObject*)o)->life;
    uint64_t service = 0;
    if (!check_args(n, 1, 1, "endpoint") || !as_u64(args[0], &service)) return NULL;
    if (lifetime_closed(life)) return closed("this MissionEpisode");
    fsim_mission_endpoint* endpoint = NULL;
    if (fsim_mission_endpoint_open(life->episode, service, &endpoint) != FSIM_OK) return fail();
    allocfunc alloc = (allocfunc)PyType_GetSlot(MissionEndpointType, Py_tp_alloc);
    MissionEndpointObject* e = (MissionEndpointObject*)alloc(MissionEndpointType, 0);
    if (!e) {
        fsim_mission_endpoint_destroy(endpoint);
        return NULL;
    }
    e->endpoint = endpoint;
    e->service_id = service;
    lifetime_retain(life);
    e->life = life;
    return (PyObject*)e;
}

static PyObject* episode_referee(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    MissionLifetime* life = ((MissionEpisodeObject*)o)->life;
    (void)args;
    if (!check_args(n, 0, 0, "referee")) return NULL;
    if (lifetime_closed(life)) return closed("this MissionEpisode");
    fsim_mission_referee* referee = NULL;
    if (fsim_mission_referee_open(life->episode, &referee) != FSIM_OK) return fail();
    allocfunc alloc = (allocfunc)PyType_GetSlot(MissionRefereeType, Py_tp_alloc);
    MissionRefereeObject* r = (MissionRefereeObject*)alloc(MissionRefereeType, 0);
    if (!r) {
        fsim_mission_referee_destroy(referee);
        return NULL;
    }
    r->referee = referee;
    lifetime_retain(life);
    r->life = life;
    return (PyObject*)r;
}

static PyObject* episode_close(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    (void)args;
    if (!check_args(n, 0, 0, "close")) return NULL;
    lifetime_close(((MissionEpisodeObject*)o)->life);
    Py_RETURN_NONE;
}

/* _owns(endpoint): whether the endpoint was opened by this episode, which is not closed. */
static PyObject* episode_owns(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    MissionLifetime* life = ((MissionEpisodeObject*)o)->life;
    if (!check_args(n, 1, 1, "_owns")) return NULL;
    const int endpoint = PyObject_IsInstance(args[0], (PyObject*)MissionEndpointType);
    if (endpoint < 0) return NULL;
    const MissionEndpointObject* e = (const MissionEndpointObject*)args[0];
    return PyBool_FromLong(endpoint && e->endpoint && e->life == life && !lifetime_closed(life));
}

#define FAST(name, fn, doc) {name, (PyCFunction)(void (*)(void))fn, METH_FASTCALL, doc}

static PyMethodDef episode_methods[] = {
    FAST("reset", episode_reset, "reset(local_episode_index): a fresh World; every earlier handle stale"),
    FAST("step", episode_step, "step(): close the decision tick and freeze the next snapshot"),
    FAST("endpoint", episode_endpoint, "endpoint(service_id) -> MissionEndpoint of the current generation"),
    FAST("referee", episode_referee, "referee() -> MissionReferee of the current generation"),
    FAST("close", episode_close, "close(): end the episode at once; its handles answer no more"),
    FAST("_owns", episode_owns, "_owns(endpoint) -> bool: the endpoint is this open episode's"),
    {NULL, NULL, 0, NULL}};

static PyType_Slot episode_slots[] = {
    {Py_tp_new, (void*)episode_new},
    {Py_tp_dealloc, (void*)episode_dealloc},
    {Py_tp_methods, (void*)episode_methods},
    {Py_tp_doc, (void*)"MissionEpisode(config_json) over fsim_mission_episode (see fsim.mission)"},
    {0, NULL}};
static PyType_Spec episode_spec = {"fsim._native.MissionEpisode", sizeof(MissionEpisodeObject), 0, Py_TPFLAGS_DEFAULT, episode_slots};

/* --- MissionEndpoint ----------------------------------------------------------------------------------------------- */

static void endpoint_dealloc(PyObject* o) {
    PyTypeObject* tp = Py_TYPE(o);
    MissionEndpointObject* self = (MissionEndpointObject*)o;
    fsim_mission_endpoint_destroy(self->endpoint);
    lifetime_release(self->life);
    freefunc f = (freefunc)PyType_GetSlot(tp, Py_tp_free);
    f(o);
    Py_DECREF(tp);
}

/* Its handle, or the reason it answers no more: its own close, or its episode's. */
static fsim_mission_endpoint* live_endpoint(MissionEndpointObject* self) {
    if (!self->endpoint) {
        closed("this MissionEndpoint");
        return NULL;
    }
    if (lifetime_closed(self->life)) {
        closed("this MissionEndpoint's episode");
        return NULL;
    }
    return self->endpoint;
}

static int endpoint_observation(void* h, char* b, size_t c, size_t* n) { return fsim_mission_endpoint_observation_json((fsim_mission_endpoint*)h, b, c, n); }
static int endpoint_receipts(void* h, char* b, size_t c, size_t* n) { return fsim_mission_endpoint_receipts_json((fsim_mission_endpoint*)h, b, c, n); }

static PyObject* endpoint_observation_json(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    (void)args;
    if (!check_args(n, 0, 0, "observation_json")) return NULL;
    fsim_mission_endpoint* endpoint = live_endpoint((MissionEndpointObject*)o);
    return endpoint ? read_document(endpoint, endpoint_observation) : NULL;
}

static PyObject* endpoint_receipts_json(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    (void)args;
    if (!check_args(n, 0, 0, "receipts_json")) return NULL;
    fsim_mission_endpoint* endpoint = live_endpoint((MissionEndpointObject*)o);
    return endpoint ? read_document(endpoint, endpoint_receipts) : NULL;
}

static PyObject* endpoint_submit_json(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    const char* data = NULL;
    Py_ssize_t size = 0;
    if (!check_args(n, 1, 1, "submit_json") || !as_document(args[0], &data, &size)) return NULL;
    fsim_mission_endpoint* endpoint = live_endpoint((MissionEndpointObject*)o);
    if (!endpoint) return NULL;
    if (fsim_mission_endpoint_submit(endpoint, data, (size_t)size) != FSIM_OK) return fail();
    return read_document(endpoint, endpoint_receipts);
}

static PyObject* endpoint_service_id(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    (void)args;
    if (!check_args(n, 0, 0, "service_id")) return NULL;
    return PyLong_FromUnsignedLongLong(((MissionEndpointObject*)o)->service_id);
}

static PyObject* endpoint_close(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    MissionEndpointObject* self = (MissionEndpointObject*)o;
    (void)args;
    if (!check_args(n, 0, 0, "close")) return NULL;
    fsim_mission_endpoint_destroy(self->endpoint);
    self->endpoint = NULL;
    Py_RETURN_NONE;
}

static PyMethodDef endpoint_methods[] = {
    FAST("observation_json", endpoint_observation_json, "observation_json() -> str: this tick's ma.observation.v1, consuming nothing"),
    FAST("submit_json", endpoint_submit_json, "submit_json(actions: str | bytes) -> str: its ma.receipts.v1 acknowledgment"),
    FAST("receipts_json", endpoint_receipts_json, "receipts_json() -> str: the latest acknowledgment and receipts, again"),
    FAST("service_id", endpoint_service_id, "service_id() -> int"),
    FAST("close", endpoint_close, "close()"),
    {NULL, NULL, 0, NULL}};

static PyType_Slot endpoint_slots[] = {
    {Py_tp_dealloc, (void*)endpoint_dealloc},
    {Py_tp_methods, (void*)endpoint_methods},
    {Py_tp_doc, (void*)"MissionEndpoint over fsim_mission_endpoint: one service's, of one generation (see fsim.mission)"},
    {0, NULL}};
static PyType_Spec endpoint_spec = {"fsim._native.MissionEndpoint", sizeof(MissionEndpointObject), 0, Py_TPFLAGS_DEFAULT, endpoint_slots};

/* --- MissionReferee ------------------------------------------------------------------------------------------------ */

static void referee_dealloc(PyObject* o) {
    PyTypeObject* tp = Py_TYPE(o);
    MissionRefereeObject* self = (MissionRefereeObject*)o;
    fsim_mission_referee_destroy(self->referee);
    lifetime_release(self->life);
    freefunc f = (freefunc)PyType_GetSlot(tp, Py_tp_free);
    f(o);
    Py_DECREF(tp);
}

static fsim_mission_referee* live_referee(MissionRefereeObject* self) {
    if (!self->referee) {
        closed("this MissionReferee");
        return NULL;
    }
    if (lifetime_closed(self->life)) {
        closed("this MissionReferee's episode");
        return NULL;
    }
    return self->referee;
}

static int referee_outcome(void* h, char* b, size_t c, size_t* n) { return fsim_mission_referee_outcome_json((fsim_mission_referee*)h, b, c, n); }
static int referee_truth(void* h, char* b, size_t c, size_t* n) { return fsim_mission_referee_truth_json((fsim_mission_referee*)h, b, c, n); }

static PyObject* referee_outcome_json(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    (void)args;
    if (!check_args(n, 0, 0, "outcome_json")) return NULL;
    fsim_mission_referee* referee = live_referee((MissionRefereeObject*)o);
    return referee ? read_document(referee, referee_outcome) : NULL;
}

static PyObject* referee_truth_json(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    (void)args;
    if (!check_args(n, 0, 0, "truth_json")) return NULL;
    fsim_mission_referee* referee = live_referee((MissionRefereeObject*)o);
    return referee ? read_document(referee, referee_truth) : NULL;
}

static PyObject* referee_close(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    MissionRefereeObject* self = (MissionRefereeObject*)o;
    (void)args;
    if (!check_args(n, 0, 0, "close")) return NULL;
    fsim_mission_referee_destroy(self->referee);
    self->referee = NULL;
    Py_RETURN_NONE;
}

static PyMethodDef referee_methods[] = {
    FAST("outcome_json", referee_outcome_json, "outcome_json() -> str: this tick's ma.outcome.v1"),
    FAST("truth_json", referee_truth_json, "truth_json() -> str: this tick's ma.truth.v1"),
    FAST("close", referee_close, "close()"),
    {NULL, NULL, 0, NULL}};

static PyType_Slot referee_slots[] = {
    {Py_tp_dealloc, (void*)referee_dealloc},
    {Py_tp_methods, (void*)referee_methods},
    {Py_tp_doc, (void*)"MissionReferee over fsim_mission_referee: the range's documents (see fsim.mission)"},
    {0, NULL}};
static PyType_Spec referee_spec = {"fsim._native.MissionReferee", sizeof(MissionRefereeObject), 0, Py_TPFLAGS_DEFAULT, referee_slots};

int fsim_py_add_mission_types(PyObject* m) {
    PyTypeObject* episode = (PyTypeObject*)PyType_FromSpec(&episode_spec);
    MissionEndpointType = (PyTypeObject*)PyType_FromSpec(&endpoint_spec);
    MissionRefereeType = (PyTypeObject*)PyType_FromSpec(&referee_spec);
    if (!episode || !MissionEndpointType || !MissionRefereeType) return -1;
    const int rc = PyModule_AddObjectRef(m, "MissionEpisode", (PyObject*)episode) < 0 ||
                           PyModule_AddObjectRef(m, "MissionEndpoint", (PyObject*)MissionEndpointType) < 0 ||
                           PyModule_AddObjectRef(m, "MissionReferee", (PyObject*)MissionRefereeType) < 0
                       ? -1
                       : 0;
    Py_DECREF(episode);
    return rc;
}
