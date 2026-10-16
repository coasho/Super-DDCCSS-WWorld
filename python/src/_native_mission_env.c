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
typedef void (*handle_destroy)(void* handle);
typedef struct {
    Py_ssize_t refs;
    atomic_long state;
    void* handle;           /* an fsim_mission_episode or (M7) an fsim_mission_batch */
    handle_destroy destroy;
} MissionLifetime;

static void destroy_episode(void* h) { fsim_mission_episode_destroy((fsim_mission_episode*)h); }
static void destroy_batch(void* h) { fsim_mission_batch_destroy((fsim_mission_batch*)h); }

static MissionLifetime* lifetime_new(void* handle, handle_destroy destroy) {
    MissionLifetime* life = (MissionLifetime*)malloc(sizeof(MissionLifetime));
    if (!life) return NULL;
    life->refs = 1;
    atomic_init(&life->state, 0);
    life->handle = handle;
    life->destroy = destroy;
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
    if (before == 0) life->destroy(life->handle);
}
/* Under the GIL, before releasing it: the native handle this call uses, or none when the episode is closed. */
static void* lifetime_enter(MissionLifetime* life) {
    if (lifetime_closed(life)) return NULL;
    atomic_fetch_add(&life->state, 2);
    return life->handle;
}
/* Without the GIL, after the native call: the last running call of a closed episode ends it. */
static void lifetime_leave(MissionLifetime* life, void* handle) {
    if (atomic_fetch_sub(&life->state, 2) == 3) life->destroy(handle);
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
static PyTypeObject* MissionBatchType;

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

/* A handle's object, sharing the lifetime of the episode or batch that opened it. */
static PyObject* endpoint_object(fsim_mission_endpoint* endpoint, uint64_t service, MissionLifetime* life) {
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

static PyObject* referee_object(fsim_mission_referee* referee, MissionLifetime* life) {
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
    MissionLifetime* life = lifetime_new(episode, destroy_episode);
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
    fsim_mission_episode* episode = (fsim_mission_episode*)lifetime_enter(life);
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
    fsim_mission_episode* episode = (fsim_mission_episode*)lifetime_enter(life);
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
    if (fsim_mission_endpoint_open((fsim_mission_episode*)life->handle, service, &endpoint) != FSIM_OK) return fail();
    return endpoint_object(endpoint, service, life);
}

static PyObject* episode_referee(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    MissionLifetime* life = ((MissionEpisodeObject*)o)->life;
    (void)args;
    if (!check_args(n, 0, 0, "referee")) return NULL;
    if (lifetime_closed(life)) return closed("this MissionEpisode");
    fsim_mission_referee* referee = NULL;
    if (fsim_mission_referee_open((fsim_mission_episode*)life->handle, &referee) != FSIM_OK) return fail();
    return referee_object(referee, life);
}

static PyObject* episode_close(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    (void)args;
    if (!check_args(n, 0, 0, "close")) return NULL;
    lifetime_close(((MissionEpisodeObject*)o)->life);
    Py_RETURN_NONE;
}

/* remove_platform(name) -> bool (M7 core): the range removes a platform; False when it is already gone. */
static PyObject* episode_remove_platform(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    MissionLifetime* life = ((MissionEpisodeObject*)o)->life;
    const char* name = NULL;
    Py_ssize_t size = 0;
    if (!check_args(n, 1, 1, "remove_platform")) return NULL;
    if (!PyUnicode_Check(args[0]) || !(name = PyUnicode_AsUTF8AndSize(args[0], &size))) {
        if (!PyErr_Occurred()) PyErr_SetString(PyExc_TypeError, "remove_platform(name: str)");
        return NULL;
    }
    if (lifetime_closed(life)) return closed("this MissionEpisode");
    int removed = 0;
    if (fsim_mission_episode_remove_platform((fsim_mission_episode*)life->handle, name, (size_t)size, &removed) != FSIM_OK) return fail();
    return PyBool_FromLong(removed);
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
    FAST("remove_platform", episode_remove_platform, "remove_platform(name): the range removes a platform (M7 core)"),
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
static int referee_costs(void* h, char* b, size_t c, size_t* n) { return fsim_mission_referee_costs_json((fsim_mission_referee*)h, b, c, n); }

static PyObject* referee_costs_json(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    (void)args;
    if (!check_args(n, 0, 0, "costs_json")) return NULL;
    fsim_mission_referee* referee = live_referee((MissionRefereeObject*)o);
    return referee ? read_document(referee, referee_costs) : NULL;
}

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
    FAST("costs_json", referee_costs_json, "costs_json() -> str: this tick's ma.costs.v1 (M7 core)"),
    FAST("close", referee_close, "close()"),
    {NULL, NULL, 0, NULL}};

static PyType_Slot referee_slots[] = {
    {Py_tp_dealloc, (void*)referee_dealloc},
    {Py_tp_methods, (void*)referee_methods},
    {Py_tp_doc, (void*)"MissionReferee over fsim_mission_referee: the range's documents (see fsim.mission)"},
    {0, NULL}};
static PyType_Spec referee_spec = {"fsim._native.MissionReferee", sizeof(MissionRefereeObject), 0, Py_TPFLAGS_DEFAULT, referee_slots};

/* --- MissionBatch (M7 core) --------------------------------------------------------------------------------------- */

typedef struct {
    PyObject_HEAD
    MissionLifetime* life;
} MissionBatchObject;

/* MissionBatch(configs: sequence of str | bytes, autoreset: int, workers: int, first_local_episode_index: int) */
static PyObject* batch_new(PyTypeObject* type, PyObject* args, PyObject* kwargs) {
    PyObject* configs = NULL;
    unsigned int autoreset = FSIM_MISSION_AUTORESET_NEXT_STEP, workers = 1;
    unsigned long long first = 0;
    if (kwargs && PyDict_Size(kwargs)) {
        PyErr_SetString(PyExc_TypeError, "MissionBatch(configs, autoreset, workers, first_local_episode_index)");
        return NULL;
    }
    if (!PyArg_ParseTuple(args, "O|IIK:MissionBatch", &configs, &autoreset, &workers, &first)) return NULL;
    PyObject* list = PySequence_List(configs);
    if (!list) return NULL;
    const Py_ssize_t count = PyList_Size(list);
    const char** texts = (const char**)calloc((size_t)(count ? count : 1), sizeof(char*));
    size_t* sizes = (size_t*)calloc((size_t)(count ? count : 1), sizeof(size_t));
    if (!texts || !sizes) {
        free(texts), free(sizes);
        Py_DECREF(list);
        return PyErr_NoMemory();
    }
    for (Py_ssize_t i = 0; i < count; ++i) {
        Py_ssize_t size = 0;
        if (!as_document(PyList_GetItem(list, i), &texts[i], &size)) {
            free(texts), free(sizes);
            Py_DECREF(list);
            return NULL;
        }
        sizes[i] = (size_t)size;
    }
    fsim_mission_batch_options options;
    fsim_mission_batch_options_init(&options);
    options.autoreset = autoreset, options.workers = workers, options.first_local_episode_index = first;
    fsim_mission_batch* batch = NULL;
    const int rc = fsim_mission_batch_create(texts, sizes, (size_t)count, &options, &batch);
    free(texts), free(sizes);
    Py_DECREF(list);
    if (rc != FSIM_OK) return fail();
    MissionLifetime* life = lifetime_new(batch, destroy_batch);
    if (!life) {
        fsim_mission_batch_destroy(batch);
        return PyErr_NoMemory();
    }
    allocfunc alloc = (allocfunc)PyType_GetSlot(type, Py_tp_alloc);
    MissionBatchObject* self = (MissionBatchObject*)alloc(type, 0);
    if (!self) {
        lifetime_close(life);
        lifetime_release(life);
        return NULL;
    }
    self->life = life;
    return (PyObject*)self;
}

static void batch_dealloc(PyObject* o) {
    PyTypeObject* tp = Py_TYPE(o);
    MissionBatchObject* self = (MissionBatchObject*)o;
    lifetime_close(self->life);
    lifetime_release(self->life);
    freefunc f = (freefunc)PyType_GetSlot(tp, Py_tp_free);
    f(o);
    Py_DECREF(tp);
}

static fsim_mission_batch* live_batch(MissionBatchObject* self) {
    if (lifetime_closed(self->life)) {
        closed("this MissionBatch");
        return NULL;
    }
    return (fsim_mission_batch*)self->life->handle;
}

static int as_slot(PyObject* o, size_t* slot) {
    uint64_t v = 0;
    if (!as_u64(o, &v)) return 0;
    *slot = (size_t)v;
    return 1;
}

/* reset() / step() / reset_slot(slot): without the GIL, as an episode's reset and step. */
static PyObject* batch_run(PyObject* o, int which, size_t slot) {
    MissionLifetime* life = ((MissionBatchObject*)o)->life;
    fsim_mission_batch* batch = (fsim_mission_batch*)lifetime_enter(life);
    if (!batch) return closed("this MissionBatch");
    int rc;
    Py_BEGIN_ALLOW_THREADS
    rc = which == 0 ? fsim_mission_batch_reset(batch) : which == 1 ? fsim_mission_batch_step(batch) : fsim_mission_batch_reset_slot(batch, slot);
    lifetime_leave(life, batch);
    Py_END_ALLOW_THREADS
    if (rc != FSIM_OK) return fail();
    Py_RETURN_NONE;
}
static PyObject* batch_reset(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    (void)args;
    return check_args(n, 0, 0, "reset") ? batch_run(o, 0, 0) : NULL;
}
static PyObject* batch_step(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    (void)args;
    return check_args(n, 0, 0, "step") ? batch_run(o, 1, 0) : NULL;
}
static PyObject* batch_reset_slot(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    size_t slot = 0;
    return check_args(n, 1, 1, "reset_slot") && as_slot(args[0], &slot) ? batch_run(o, 2, slot) : NULL;
}

static PyObject* batch_size(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    (void)args;
    if (!check_args(n, 0, 0, "size")) return NULL;
    fsim_mission_batch* batch = live_batch((MissionBatchObject*)o);
    size_t size = 0;
    if (!batch) return NULL;
    if (fsim_mission_batch_size(batch, &size) != FSIM_OK) return fail();
    return PyLong_FromSize_t(size);
}

/* status(slot) -> (generation, local_episode_index, tick, flags) */
static PyObject* batch_status(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    size_t slot = 0;
    if (!check_args(n, 1, 1, "status") || !as_slot(args[0], &slot)) return NULL;
    fsim_mission_batch* batch = live_batch((MissionBatchObject*)o);
    if (!batch) return NULL;
    fsim_mission_slot_status st;
    memset(&st, 0, sizeof st);
    st.struct_size = sizeof st;
    if (fsim_mission_batch_status(batch, slot, &st) != FSIM_OK) return fail();
    return Py_BuildValue("(KKKI)", (unsigned long long)st.generation, (unsigned long long)st.local_episode_index, (unsigned long long)st.tick, st.flags);
}

static PyObject* batch_endpoint(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    size_t slot = 0;
    uint64_t service = 0;
    if (!check_args(n, 2, 2, "endpoint") || !as_slot(args[0], &slot) || !as_u64(args[1], &service)) return NULL;
    MissionBatchObject* self = (MissionBatchObject*)o;
    fsim_mission_batch* batch = live_batch(self);
    fsim_mission_endpoint* endpoint = NULL;
    if (!batch) return NULL;
    if (fsim_mission_batch_endpoint_open(batch, slot, service, &endpoint) != FSIM_OK) return fail();
    return endpoint_object(endpoint, service, self->life);
}

static PyObject* batch_referee(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    size_t slot = 0;
    if (!check_args(n, 1, 1, "referee") || !as_slot(args[0], &slot)) return NULL;
    MissionBatchObject* self = (MissionBatchObject*)o;
    fsim_mission_batch* batch = live_batch(self);
    fsim_mission_referee* referee = NULL;
    if (!batch) return NULL;
    if (fsim_mission_batch_referee_open(batch, slot, &referee) != FSIM_OK) return fail();
    return referee_object(referee, self->life);
}

/* final_*_json(slot[, service]): a two-call read of the slot's final documents */
static PyObject* batch_final(PyObject* o, PyObject* const* args, Py_ssize_t n, int which) {
    size_t slot = 0, needed = 0;
    uint64_t service = 0;
    if (!check_args(n, which == 0 ? 2 : 1, which == 0 ? 2 : 1, "final_json") || !as_slot(args[0], &slot)) return NULL;
    if (which == 0 && !as_u64(args[1], &service)) return NULL;
    fsim_mission_batch* batch = live_batch((MissionBatchObject*)o);
    if (!batch) return NULL;
    for (int attempt = 0; attempt < 4; ++attempt) {
        int rc = which == 0 ? fsim_mission_batch_final_observation_json(batch, slot, service, NULL, 0, &needed)
                 : which == 1 ? fsim_mission_batch_final_outcome_json(batch, slot, NULL, 0, &needed)
                 : which == 2 ? fsim_mission_batch_final_costs_json(batch, slot, NULL, 0, &needed)
                              : fsim_mission_batch_final_truth_json(batch, slot, NULL, 0, &needed);
        if (rc != FSIM_OK) return fail();
        char* buffer = (char*)malloc(needed);
        if (!buffer) return PyErr_NoMemory();
        size_t got = needed;
        rc = which == 0 ? fsim_mission_batch_final_observation_json(batch, slot, service, buffer, needed, &got)
             : which == 1 ? fsim_mission_batch_final_outcome_json(batch, slot, buffer, needed, &got)
             : which == 2 ? fsim_mission_batch_final_costs_json(batch, slot, buffer, needed, &got)
                          : fsim_mission_batch_final_truth_json(batch, slot, buffer, needed, &got);
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
static PyObject* batch_final_observation(PyObject* o, PyObject* const* a, Py_ssize_t n) { return batch_final(o, a, n, 0); }
static PyObject* batch_final_outcome(PyObject* o, PyObject* const* a, Py_ssize_t n) { return batch_final(o, a, n, 1); }
static PyObject* batch_final_costs(PyObject* o, PyObject* const* a, Py_ssize_t n) { return batch_final(o, a, n, 2); }
static PyObject* batch_final_truth(PyObject* o, PyObject* const* a, Py_ssize_t n) { return batch_final(o, a, n, 3); }

static PyObject* batch_remove_platform(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    size_t slot = 0;
    const char* name = NULL;
    Py_ssize_t size = 0;
    if (!check_args(n, 2, 2, "remove_platform") || !as_slot(args[0], &slot)) return NULL;
    if (!PyUnicode_Check(args[1]) || !(name = PyUnicode_AsUTF8AndSize(args[1], &size))) {
        if (!PyErr_Occurred()) PyErr_SetString(PyExc_TypeError, "remove_platform(slot, name: str)");
        return NULL;
    }
    fsim_mission_batch* batch = live_batch((MissionBatchObject*)o);
    if (!batch) return NULL;
    int removed = 0;
    if (fsim_mission_batch_remove_platform(batch, slot, name, (size_t)size, &removed) != FSIM_OK) return fail();
    return PyBool_FromLong(removed);
}

static PyObject* batch_close(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    (void)args;
    if (!check_args(n, 0, 0, "close")) return NULL;
    lifetime_close(((MissionBatchObject*)o)->life);
    Py_RETURN_NONE;
}

static PyMethodDef batch_methods[] = {
    FAST("size", batch_size, "size() -> int"),
    FAST("reset", batch_reset, "reset(): a fresh generation in every slot"),
    FAST("reset_slot", batch_reset_slot, "reset_slot(slot)"),
    FAST("step", batch_step, "step(): one decision tick in every slot, with its autoreset"),
    FAST("status", batch_status, "status(slot) -> (generation, local_episode_index, tick, flags)"),
    FAST("endpoint", batch_endpoint, "endpoint(slot, service_id) -> MissionEndpoint"),
    FAST("referee", batch_referee, "referee(slot) -> MissionReferee"),
    FAST("final_observation_json", batch_final_observation, "final_observation_json(slot, service_id) -> str"),
    FAST("final_outcome_json", batch_final_outcome, "final_outcome_json(slot) -> str"),
    FAST("final_costs_json", batch_final_costs, "final_costs_json(slot) -> str"),
    FAST("final_truth_json", batch_final_truth, "final_truth_json(slot) -> str"),
    FAST("remove_platform", batch_remove_platform, "remove_platform(slot, name)"),
    FAST("close", batch_close, "close(): end every slot at once; their handles answer no more"),
    {NULL, NULL, 0, NULL}};

static PyType_Slot batch_slots[] = {
    {Py_tp_new, (void*)batch_new},
    {Py_tp_dealloc, (void*)batch_dealloc},
    {Py_tp_methods, (void*)batch_methods},
    {Py_tp_doc, (void*)"MissionBatch(configs, autoreset, workers, first_local_episode_index) over fsim_mission_batch (see fsim.mission)"},
    {0, NULL}};
static PyType_Spec batch_spec = {"fsim._native.MissionBatch", sizeof(MissionBatchObject), 0, Py_TPFLAGS_DEFAULT, batch_slots};

int fsim_py_add_mission_types(PyObject* m) {
    PyTypeObject* episode = (PyTypeObject*)PyType_FromSpec(&episode_spec);
    MissionEndpointType = (PyTypeObject*)PyType_FromSpec(&endpoint_spec);
    MissionRefereeType = (PyTypeObject*)PyType_FromSpec(&referee_spec);
    MissionBatchType = (PyTypeObject*)PyType_FromSpec(&batch_spec);
    if (!episode || !MissionEndpointType || !MissionRefereeType || !MissionBatchType) return -1;
    const int rc = PyModule_AddObjectRef(m, "MissionEpisode", (PyObject*)episode) < 0 ||
                           PyModule_AddObjectRef(m, "MissionEndpoint", (PyObject*)MissionEndpointType) < 0 ||
                           PyModule_AddObjectRef(m, "MissionReferee", (PyObject*)MissionRefereeType) < 0 ||
                           PyModule_AddObjectRef(m, "MissionBatch", (PyObject*)MissionBatchType) < 0
                       ? -1
                       : 0;
    Py_DECREF(episode);
    return rc;
}
