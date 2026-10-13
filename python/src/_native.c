/* fsim._native: the Python SDK's native core (docs/sdk/python.md).
 *
 * A thin layer over the C ABI (include/fsim/fsim_c.h), written for the least
 * cost per call rather than for convenience - the fsim package's Python code
 * provides that on top:
 *
 *   - METH_FASTCALL throughout, positional arguments only;
 *   - memory the platform owns (VecEnv buffers, vehicle states, recordings)
 *     is exported through the buffer protocol and viewed by numpy without a
 *     copy, once, and stays valid across steps;
 *   - actions, command rows and state gathers are read from and written to
 *     the caller's arrays in place;
 *   - the GIL is released for anything that simulates (steps, resets,
 *     loading vehicles), so other Python threads run meanwhile;
 *   - many-vehicle operations are one call (fsim_world_command_batch,
 *     fsim_world_gather_states).
 *
 * Stable ABI (abi3) for Python 3.11+: one binary for every interpreter from
 * 3.11 on. Built with the project's own GCC for the python.org / conda
 * CPython (MSVC) - the interface between them is C and both use the UCRT. */
#include "_native_world.h"
#include "_native_world_options.h"
#include "_native_vecenv.h"
#include "_native_recording.h"

#include <math.h>


static PyObject* Error;            /* fsim.Error */
static PyTypeObject* BufferType;
static PyTypeObject* VecEnvType;
static PyTypeObject* WorldType;
static PyTypeObject* ScenarioType;
static PyTypeObject* RecordingType;

PyObject* fail(void) {
    const char* text = fsim_last_error();
    PyErr_SetString(Error, text && *text ? text : "fsim call failed");
    return NULL;
}

int check_args(Py_ssize_t n, Py_ssize_t lo, Py_ssize_t hi, const char* name) {
    if (n < lo || n > hi) {
        if (lo == hi) PyErr_Format(PyExc_TypeError, "%s() takes %zd argument(s) (%zd given)", name, lo, n);
        else PyErr_Format(PyExc_TypeError, "%s() takes %zd to %zd arguments (%zd given)", name, lo, hi, n);
        return 0;
    }
    return 1;
}

int as_u32(PyObject* o, uint32_t* out) {
    const unsigned long v = PyLong_AsUnsignedLong(o);
    if (v == (unsigned long)-1 && PyErr_Occurred()) return 0;
    *out = (uint32_t)v;
    return 1;
}

int as_int(PyObject* o, int* out) {
    const long v = PyLong_AsLong(o);
    if (v == -1 && PyErr_Occurred()) return 0;
    *out = (int)v;
    return 1;
}

const char* as_str(PyObject* o, const char* what) {
    const char* s = PyUnicode_Check(o) ? PyUnicode_AsUTF8AndSize(o, NULL) : NULL;
    if (!s && !PyErr_Occurred()) PyErr_Format(PyExc_TypeError, "%s must be a string", what);
    return s;
}

/* A C-contiguous buffer of exactly `count` items of one of the formats in
 * `formats` (single characters, optionally with a byte-order prefix). */
int get_array(PyObject* o, Py_buffer* b, int writable, const char* formats, Py_ssize_t itemsize, const char* what) {
    if (PyObject_GetBuffer(o, b, PyBUF_C_CONTIGUOUS | PyBUF_FORMAT | (writable ? PyBUF_WRITABLE : 0)) < 0) {
        PyErr_Clear(); /* numpy says ValueError for a strided array: here it means "convert it first" */
        PyErr_Format(PyExc_TypeError, "%s: expected a C-contiguous array of '%s' items", what, formats);
        return 0;
    }
    const char* f = b->format ? b->format : "B";
    if (*f == '<' || *f == '=' || *f == '@') ++f;
    if (b->itemsize != itemsize || f[0] == '\0' || f[1] != '\0' || !strchr(formats, f[0])) {
        PyErr_Format(PyExc_TypeError, "%s: expected a contiguous array of '%s' items of %zd bytes, got '%s'", what, formats,
                     itemsize, b->format ? b->format : "B");
        PyBuffer_Release(b);
        return 0;
    }
    return 1;
}

/* ==========================================================================
 * VecEnv
 * ======================================================================== */
/* The environment's buffers, as read-only memory to view once: they are
 * allocated when the environment is and never move. */
static PyObject* vecenv_buffers(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    VecEnvObject* self = (VecEnvObject*)o;
    (void)args;
    if (!check_args(n, 0, 0, "buffers")) return NULL;
    fsim_buffers b;
    memset(&b, 0, sizeof b);
    b.struct_size = sizeof b;
    if (fsim_vecenv_buffers(self->env, &b) != FSIM_OK) return fail();
    const Py_ssize_t nv = (Py_ssize_t)b.num_envs * b.vehicles_per_env, no = b.observation_size;
    return Py_BuildValue(
        "{s:N,s:N,s:N,s:N,s:N,s:N,s:I,s:I,s:I,s:I,s:d}",
        "observations", fsim_py_buffer_new(BufferType, o, b.observations, nv * no * 4, 1),
        "rewards", fsim_py_buffer_new(BufferType, o, b.rewards, nv * 4, 1),
        "terminated", fsim_py_buffer_new(BufferType, o, b.terminated, nv, 1),
        "truncated", fsim_py_buffer_new(BufferType, o, b.truncated, nv, 1),
        "final_observations", fsim_py_buffer_new(BufferType, o, b.final_observations, nv * no * 4, 1),
        "episode_steps", fsim_py_buffer_new(BufferType, o, b.episode_steps, (Py_ssize_t)b.num_envs * 4, 1),
        "num_envs", b.num_envs, "vehicles_per_env", b.vehicles_per_env, "observation_size", b.observation_size,
        "action_size", b.action_size, "agent_step_seconds", b.agent_step_seconds);
}


static PyObject* world_borrowed(PyObject* owner, fsim_world* world, int* busy);

/* The batch's world through the object model; keeps this VecEnv alive. */
static PyObject* vecenv_world(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    VecEnvObject* self = (VecEnvObject*)o;
    (void)args;
    if (!check_args(n, 0, 0, "world")) return NULL;
    fsim_world* w = fsim_vecenv_world(self->env);
    if (!w) return fail();
    return world_borrowed(o, w, &self->busy);
}


#define FAST(name, fn, doc) {name, (PyCFunction)(void (*)(void))fn, METH_FASTCALL, doc}
static PyMethodDef vecenv_methods[] = {
    FAST("step", vecenv_step, "step(actions): M*K*A float32 (or float64) values; results land in the buffers"),
    FAST("reset", vecenv_reset, "reset(seed=0)"),
    FAST("buffers", vecenv_buffers, "the environment's result buffers as read-only memory, and their sizes"),
    FAST("names", vecenv_names, "(observation names, action names)"),
    FAST("set_action_ranges", vecenv_set_action_ranges, "set_action_ranges(\"fixed\" | \"aircraft\")"),
    FAST("vehicle_steps", vecenv_vehicle_steps, "FDM vehicle-steps so far"),
    FAST("set_autoreset", vecenv_set_autoreset, "set_autoreset(AUTORESET_NEXT_STEP | AUTORESET_SAME_STEP)"),
    FAST("autoreset", vecenv_autoreset, "the current auto-reset mode"),
    FAST("vehicle_ids", vecenv_vehicle_ids, "world vehicle ids in batch order, as uint32 bytes"),
    FAST("world", vecenv_world, "the batch's world (a borrowed World)"),
    FAST("_capsule", vecenv_capsule, "for fsim._vision"),
    {NULL, NULL, 0, NULL}};

static PyType_Slot vecenv_slots[] = {
    {Py_tp_new, (void*)vecenv_new},
    {Py_tp_dealloc, (void*)vecenv_dealloc},
    {Py_tp_methods, (void*)vecenv_methods},
    {Py_tp_doc, (void*)"VecEnv(options: dict) over fsim_vecenv (see fsim.VecEnv)"},
    {0, NULL}};
static PyType_Spec vecenv_spec = {"fsim._native.VecEnv", sizeof(VecEnvObject), 0, Py_TPFLAGS_DEFAULT, vecenv_slots};

/* ==========================================================================
 * World
 * ======================================================================== */
static PyObject* world_new(PyTypeObject* type, PyObject* args, PyObject* kwargs) {
    PyObject* options = NULL;
    if (kwargs && PyDict_Size(kwargs)) {
        PyErr_SetString(PyExc_TypeError, "World(options): options is a dict");
        return NULL;
    }
    if (!PyArg_ParseTuple(args, "|O:World", &options)) return NULL;
    fsim_world_options o;
    fsim_world_options_init(&o);
    if (fsim_py_set_fields(options, &o, world_fields, COUNT(world_fields), "World") < 0) return NULL;
    allocfunc alloc = (allocfunc)PyType_GetSlot(type, Py_tp_alloc);
    WorldObject* self = (WorldObject*)alloc(type, 0);
    if (!self) return NULL;
    self->busy = &self->own_busy;
    int rc;
    fsim_world* w = NULL;
    Py_BEGIN_ALLOW_THREADS
    rc = fsim_world_create(&o, &w);
    Py_END_ALLOW_THREADS
    if (rc != FSIM_OK) {
        Py_DECREF(self);
        return fail();
    }
    self->world = w;
    self->ref.handle = w;
    self->ref.busy = self->busy;
    return (PyObject*)self;
}

static PyObject* world_borrowed(PyObject* owner, fsim_world* world, int* busy) {
    allocfunc alloc = (allocfunc)PyType_GetSlot(WorldType, Py_tp_alloc);
    WorldObject* self = (WorldObject*)alloc(WorldType, 0);
    if (!self) return NULL;
    self->world = world;
    self->owner = Py_NewRef(owner);
    self->busy = busy;
    self->ref.handle = world;
    self->ref.busy = busy;
    return (PyObject*)self;
}

static void world_dealloc(PyObject* o) {
    WorldObject* self = (WorldObject*)o;
    PyTypeObject* tp = Py_TYPE(o);
    if (self->world && (!self->owner || self->agent || self->successor)) fsim_world_destroy(self->world);
    Py_CLEAR(self->owner);
    freefunc f = (freefunc)PyType_GetSlot(tp, Py_tp_free);
    f(o);
    Py_DECREF(tp);
}


/* ADR-31: an agent's handle for one of this world's aircraft - a World object answering an agent's calls alone. */
static PyObject* world_create_agent(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t vehicle = 0;
    const char* name = "";
    if (!check_args(n, 1, 2, "create_agent") || !as_u32(args[0], &vehicle)) return NULL;
    if (n == 2 && !(name = as_str(args[1], "create_agent: name"))) return NULL;
    fsim_world* a = NULL;
    if (fsim_agent_create(self->world, vehicle, name, &a) != FSIM_OK) return fail();
    allocfunc alloc = (allocfunc)PyType_GetSlot(WorldType, Py_tp_alloc);
    WorldObject* agent = (WorldObject*)alloc(WorldType, 0);
    if (!agent) {
        fsim_world_destroy(a);
        return NULL;
    }
    agent->world = a;
    agent->owner = Py_NewRef(o); /* (the range's world lives while its agents do) */
    agent->agent = 1;
    agent->busy = self->busy;
    agent->ref.handle = a;
    agent->ref.busy = self->busy;
    return (PyObject*)agent;
}

/* AB-2d: seal this handle; the range goes on through the one returned (keeping this object, and its busy flag, alive). */
static PyObject* world_seal(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    fsim_world* next = NULL;
    (void)args;
    if (!check_args(n, 0, 0, "seal")) return NULL;
    if (fsim_world_seal(self->world, &next) != FSIM_OK) return fail();
    allocfunc alloc = (allocfunc)PyType_GetSlot(WorldType, Py_tp_alloc);
    WorldObject* range = (WorldObject*)alloc(WorldType, 0);
    if (!range) {
        fsim_world_destroy(next);
        return NULL;
    }
    range->world = next;
    range->owner = Py_NewRef(o);
    range->successor = 1;
    range->busy = self->busy;
    range->ref.handle = next;
    range->ref.busy = self->busy;
    return (PyObject*)range;
}



/* state_buffer(id, sensed): the vehicle's live state struct, without a copy.
 * Writable only so that ctypes can map a struct over it; it is the
 * platform's snapshot, rewritten every step. */
static PyObject* world_state_buffer(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    int sensed = 0;
    if (!check_args(n, 1, 2, "state_buffer") || !as_u32(args[0], &id) || (n == 2 && !as_int(args[1], &sensed))) return NULL;
    const fsim_vehicle_state* s = sensed ? fsim_vehicle_sensed_ptr(self->world, id) : fsim_vehicle_state_ptr(self->world, id);
    if (!s) {
        PyErr_Format(PyExc_KeyError, "no vehicle with id %u", (unsigned)id);
        return NULL;
    }
    return fsim_py_buffer_new(BufferType, o, s, (Py_ssize_t)sizeof(fsim_vehicle_state), 0);
}


int as_u64(PyObject* o, uint64_t* out) {
    const unsigned long long v = PyLong_AsUnsignedLongLong(o);
    if (v == (unsigned long long)-1 && PyErr_Occurred()) return 0;
    *out = (uint64_t)v;
    return 1;
}





static PyObject* world_apply_scenario(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    if (!check_args(n, 1, 1, "apply_scenario") || !WORLD_IDLE(self)) return NULL;
    if (!PyObject_TypeCheck(args[0], ScenarioType)) {
        PyErr_SetString(PyExc_TypeError, "apply_scenario: expected a Scenario");
        return NULL;
    }
    const fsim_scenario* sc = ((ScenarioObject*)args[0])->scenario;
    const uint32_t capacity = fsim_scenario_vehicle_count(sc);
    uint32_t* ids = (uint32_t*)PyMem_Malloc(sizeof(uint32_t) * (capacity ? capacity : 1));
    if (!ids) return PyErr_NoMemory();
    size_t count = 0;
    int rc;
    *self->busy = 1;
    Py_BEGIN_ALLOW_THREADS
    rc = fsim_scenario_apply(self->world, sc, ids, capacity, &count);
    Py_END_ALLOW_THREADS
    *self->busy = 0;
    if (rc != FSIM_OK) {
        PyMem_Free(ids);
        return fail();
    }
    PyObject* bytes = PyBytes_FromStringAndSize((const char*)ids, (Py_ssize_t)(count * 4));
    PyMem_Free(ids);
    return bytes;
}



/* Mission Systems (ADR-30): defined further down */

static PyMethodDef world_methods[] = {
    FAST("step", world_step, "step(n=1): advance every vehicle by n world steps"),
    FAST("info", world_info, "time, step seconds, vehicle steps, published, vehicle count"),
    FAST("create_agent", world_create_agent, "create_agent(vehicle, name='') -> an agent's handle (ADR-31)"),
    FAST("agent_identity", world_agent_identity, "agent_identity() -> (vehicle, controller, name), None for the range's handle"),
    FAST("seal", world_seal, "seal() -> the range's handle from now on; this one answers nothing (AB-2d)"),
    FAST("agent_audit", world_agent_audit, "agent_audit() -> every agent's calls and refusals (the referee's; AB-2d)"),
    FAST("time", world_time, "simulation seconds since creation"),
    FAST("create_vehicle", world_create_vehicle, "create_vehicle(spec: dict) -> id"),
    FAST("remove_vehicle", world_remove_vehicle, "remove_vehicle(id)"),
    FAST("focus_viewer", world_focus_viewer, "focus_viewer(id)"),
    FAST("reset_vehicle", world_reset_vehicle, "reset_vehicle(id, spec=None)"),
    FAST("find_vehicle", world_find_vehicle, "find_vehicle(name) -> id, 0 if none"),
    FAST("vehicle_ids", world_vehicle_ids, "every vehicle id, as uint32 bytes"),
    FAST("vehicle_info", world_vehicle_info, "vehicle_info(id) -> (name, type)"),
    FAST("state_buffer", world_state_buffer, "state_buffer(id, sensed=0): the live state struct, zero-copy"),
    FAST("gather_states", world_gather_states, "gather_states(ids, out, sensed=0)"),
    FAST("command", world_command, "command(id, level, values)"),
    FAST("command_batch", world_command_batch, "command_batch(level, ids, values)"),
    FAST("command_behavior", world_command_behavior, "command_behavior(id, behavior, target=0, params=None, points=None)"),
    FAST("submit", world_submit, "submit(id, level, values, source, axes, range, min_version, envelope) -> result"),
    FAST("submit_batch", world_submit_batch,
         "submit_batch(id, items) -> [result]: each item (kind, code, values, behavior, waypoints, segments, options[, loiters[, states[, paths]]])"),
    FAST("last_findings", world_last_findings, "last_findings() -> [(reason, index, constraint, from, to, associated, description)]"),
    FAST("last_adjustments", world_last_adjustments, "last_adjustments() -> [(index, field, constraint, requested, adjusted)]"),
    FAST("last_endurance", world_last_endurance, "last_endurance() -> (energy, remaining, required, remaining_s, required_s) or None"),
    FAST("last_terrain", world_last_terrain, "last_terrain() -> (index, latitude_rad, longitude_rad, altitude_msl_m, ground_m, time_s) or None"),
    FAST("terrain", world_terrain, "terrain(latitudes_rad, longitudes_rad) -> [height_m or None]: the ground's height above the WGS-84 ellipsoid"),
    FAST("submit_behavior", world_submit_behavior, "submit_behavior(id, behavior, target, params, points, source, axes, range, min_version) -> result"),
    FAST("submit_support", world_submit_support, "submit_support(id, kind, values, source, axes, range, min_version) -> result"),
    FAST("submit_mode", world_submit_mode, "submit_mode(id, mode, values, source, axes, range, min_version) -> result"),
    FAST("submit_route", world_submit_route,
         "submit_route(id, values, waypoints, source, axes, range, min_version, envelope, loiters, states, paths) -> result"),
    FAST("activity_update_route", world_activity_update_route,
         "activity_update_route(activity, values, waypoints, source=0, controller=0, loiters=None, states=None, paths=None) -> result"),
    FAST("submit_curve", world_submit_curve, "submit_curve(id, values, segments, source, axes, range, min_version) -> result"),
    FAST("activity_update_curve", world_activity_update_curve, "activity_update_curve(activity, values, segments, source=0) -> result"),
    FAST("submit_nurbs", world_submit_nurbs, "submit_nurbs(id, values, segments, source, axes, range, min_version) -> result"),
    FAST("activity_update_nurbs", world_activity_update_nurbs, "activity_update_nurbs(activity, values, segments, source=0) -> result"),
    FAST("activity_update", world_activity_update, "activity_update(activity, values, source=0) -> result"),
    FAST("activity_update_behavior", world_activity_update_behavior, "activity_update_behavior(activity, params, source=0, controller=0) -> result"),
    FAST("activity_update_batch", world_activity_update_batch, "activity_update_batch(activities uint64, values float64, stride[, fields])"),
    FAST("activity_cancel", world_activity_cancel, "activity_cancel(activity, source=0) -> result"),
    FAST("activity_command", world_activity_command, "activity_command(activity, command, priority, precedence, source=0) -> result"),
    FAST("activity_command_branch", world_activity_command_branch,
         "activity_command_branch(activity, branch, commanded=1, source=0, controller=0) -> result"),
    FAST("store_task", world_store_task, "store_task(id, task_id, item, attempts, interval_s) -> reason"),
    FAST("command_task", world_command_task, "command_task(id, task_id, source, axes, range, min_version, envelope) -> result"),
    FAST("cancel_task", world_cancel_task, "cancel_task(id, task_id, source=0) -> result"),
    FAST("remove_task", world_remove_task, "remove_task(id, task_id) -> reason"),
    FAST("task_status", world_task_status, "task_status(id, task_id) -> status, or None"),
    FAST("tasks", world_tasks, "tasks(id) -> [status]"),
    FAST("publish_plan", world_publish_plan,
         "publish_plan(id, plan_id, version, for_planning_use_only, detailed, remarks_name, remarks, item, points, paths) -> reason"),
    FAST("plan_command", world_plan_command, "plan_command(id, plan_id, command, source, axes, range, min_version, envelope) -> result"),
    FAST("abort_plan", world_abort_plan, "abort_plan(id, plan_id, reason=0) -> result: FA's own deactivation"),
    FAST("remove_plan", world_remove_plan, "remove_plan(id, plan_id) -> reason"),
    FAST("plan_status", world_plan_status, "plan_status(id, plan_id) -> status or None"),
    FAST("intercept_status", world_intercept_status, "intercept_status(activity) -> status or None"),
    FAST("plans", world_plans, "plans(id) -> [status]"),
    FAST("get_plan", world_get_plan, "get_plan(id, plan_id) -> plan tuple or None"),
    FAST("load_plan", world_load_plan, "load_plan(id, ...publish_plan's) -> reason: FA's own plan"),
    FAST("validate_plan", world_validate_plan, "validate_plan(id, ...publish_plan's, validation) -> (valid, result)"),
    FAST("validate_stored_plan", world_validate_stored_plan, "validate_stored_plan(id, plan_id, validation) -> (valid, result)"),
    FAST("load_airfield", world_load_airfield, "load_airfield(id, airfield_id, icao, qnh_pa, runways) -> reason"),
    FAST("airfields", world_airfields, "airfields(id) -> [airfield tuple]"),
    FAST("get_airfield", world_get_airfield, "get_airfield(id, airfield_id) -> airfield tuple or None"),
    FAST("activity_info", world_activity_info, "activity_info(activity) -> info or None"),
    FAST("activity_setpoint", world_activity_setpoint,
         "activity_setpoint(activity) -> (kind, code, fields, behavior, waypoints, segments, loiters, states, paths) or None"),
    FAST("activity_end_points", world_activity_end_points, "activity_end_points(activity, max) -> [end point]"),
    FAST("activity_progress", world_activity_progress, "activity_progress(activity) -> progress or None"),
    FAST("commanded", world_commanded, "commanded(id) -> what the cascade asked for in its last update"),
    FAST("navigation_report", world_navigation_report, "navigation_report(id) -> A-GRA's navigation report, 14 items"),
    FAST("set_navigation", world_set_navigation, "set_navigation(id, recovery, latitude_deg, longitude_deg, altitude_msl_m, reserve_fraction)"),
    FAST("navigation", world_navigation, "navigation(id) -> (recovery, latitude_deg, longitude_deg, altitude_msl_m, reserve_fraction)"),
    FAST("set_qnh", world_set_qnh, "set_qnh(id, qnh_pa): what its barometric altimeter is set to"),
    FAST("qnh", world_qnh, "qnh(id) -> its altimeter's setting, Pa"),
    FAST("state_data", world_state_data, "state_data(id) -> its altimeter's reading, the air, its orientation's rates, the wind: 15 items"),
    FAST("ownship", world_ownship, "ownship(id) -> its own ship as its navigation and air data report it (ADR-31)"),
    FAST("designate", world_designate, "designate(id, payload, track) -> a target id, 0 for no such track (AB-2b)"),
    FAST("undesignate", world_undesignate, "undesignate(id, target)"),
    FAST("designations", world_designations, "designations(id) -> [(target, payload, track, valid)]"),
    FAST("designation_truth", world_designation_truth, "designation_truth(target) -> the entity behind it (the referee's)"),
    FAST("magnetic_year", world_magnetic_year, "magnetic_year() -> the decimal year the World Magnetic Model is read at for this world now"),
    FAST("create_frame", world_create_frame, "create_frame(origin, vehicle, latitude_rad, longitude_rad, altitude_msl_m, yaw_rad, pitch_rad, roll_rad, north_ms, east_ms, down_ms, time_s) -> id"),
    FAST("remove_frame", world_remove_frame, "remove_frame(id) -> bool"),
    FAST("frame_point", world_frame_point, "frame_point(id, rotation, offsets, x, y, z, time_s) -> (latitude_rad, longitude_rad, altitude_msl_m) or None"),
    FAST("set_op_point", world_set_op_point, "set_op_point(id, latitude_rad, longitude_rad, altitude_m, altitude_reference, frame, frame_rotation, frame_offsets, frame_x_m, frame_y_m, frame_z_m, ingress_min_rad, ingress_max_rad) -> reason"),
    FAST("op_points", world_op_points, "op_points() -> [op point tuple]"),
    FAST("op_point", world_op_point, "op_point(id) -> op point tuple or None"),
    FAST("remove_op_point", world_remove_op_point, "remove_op_point(id) -> bool"),
    FAST("set_op_zone", world_set_op_zone, "set_op_zone(zone tuple) -> reason"),
    FAST("op_zones", world_op_zones, "op_zones() -> [zone tuple]"),
    FAST("op_zone", world_op_zone, "op_zone(id) -> zone tuple or None"),
    FAST("remove_op_zone", world_remove_op_zone, "remove_op_zone(id) -> bool"),
    FAST("submit_must_fly", world_submit_must_fly, "submit_must_fly(id, values, zone or None, source, axes, range, min_version, envelope) -> result"),
    FAST("activity_update_must_fly", world_activity_update_must_fly, "activity_update_must_fly(activity, values, zone or None, source, controller) -> result"),
    FAST("set_op_line", world_set_op_line, "set_op_line(line tuple) -> reason"),
    FAST("op_lines", world_op_lines, "op_lines() -> [line tuple]"),
    FAST("op_line", world_op_line, "op_line(id) -> line tuple or None"),
    FAST("remove_op_line", world_remove_op_line, "remove_op_line(id) -> bool"),
    FAST("submit_must_fly_line", world_submit_must_fly_line, "submit_must_fly_line(id, values, line, source, axes, range, min_version, envelope) -> result"),
    FAST("activity_update_must_fly_line", world_activity_update_must_fly_line, "activity_update_must_fly_line(activity, values, line, source, controller) -> result"),
    FAST("set_op_volume", world_set_op_volume, "set_op_volume(volume tuple) -> reason"),
    FAST("op_volumes", world_op_volumes, "op_volumes() -> [volume tuple]"),
    FAST("op_volume", world_op_volume, "op_volume(id) -> volume tuple or None"),
    FAST("remove_op_volume", world_remove_op_volume, "remove_op_volume(id) -> bool"),
    FAST("submit_must_fly_volume", world_submit_must_fly_volume, "submit_must_fly_volume(id, values, volume, source, axes, range, min_version, envelope) -> result"),
    FAST("activity_update_must_fly_volume", world_activity_update_must_fly_volume, "activity_update_must_fly_volume(activity, values, volume, source, controller) -> result"),
    FAST("performance_profile", world_performance_profile, "performance_profile(id, mode) -> (reason, profile or None)"),
    FAST("vehicle_activities", world_vehicle_activities, "vehicle_activities(id) -> [info]"),
    FAST("capabilities", world_capabilities, "capabilities(id) -> [capability]"),
    FAST("capability_status", world_capability_status, "capability_status(id, capability) -> (availability, reason)"),
    FAST("payloads", world_payloads, "payloads(id) -> [(index, name, card, designation, summary, _, family, state, count, source)]"),
    FAST("ms_capabilities", world_ms_capabilities, "ms_capabilities(id) -> [(index, payload, type, id, support, stage)]"),
    FAST("ms_capability_status", world_ms_capability_status, "ms_capability_status(id, capability) -> (availability, reason, next_available_s)"),
    FAST("payload_state", world_payload_state, "payload_state(id, payload, state, source=0, controller=0, command_id=0) -> result"),
    FAST("payload_bit", world_payload_bit, "payload_bit(id, payload, source=0, controller=0, command_id=0) -> result"),
    FAST("ms_cancel", world_ms_cancel, "ms_cancel(activity, source=0, controller=0) -> result"),
    FAST("ms_activity", world_ms_activity, "ms_activity(activity) -> tuple or None"),
    FAST("ms_activities", world_ms_activities, "ms_activities(id) -> [tuple]"),
    FAST("payload_fail", world_payload_fail, "payload_fail(id, payload, failed)"),
    FAST("launch_weapon", world_launch_weapon, "launch_weapon(launcher, type, name, ox, oy, oz, ex, ey, ez) -> id"),
    FAST("dispense", world_dispense, "dispense(vehicle, flare) -> None"),
    FAST("countermeasures", world_countermeasures, "countermeasures(vehicle) -> chaff, flare"),
    FAST("engagement_events", world_engagement_events, "engagement_events(since) -> referee events"),
    FAST("decoys", world_decoys, "decoys() -> decoy states"),
    FAST("launch_zone", world_launch_zone, "launch_zone(vehicle, store, target) -> maximum, no_escape, minimum"),
    FAST("seeker_target", world_seeker_target, "seeker_target(vehicle, store) -> observation id or zero"),
    FAST("weapons", world_weapons,
         "weapons() -> [(id, launcher, name, type, launch_time, end_time, end, lat, lon, height, speed, store, station, jettisoned, "
         "activity, target)]"),
    FAST("stores", world_stores, "stores(id) -> [(index, payload, station, station_name, store, mnemonic, class, carriage, powerable, "
                                 "jettisonable, flyable, status, mass_kg, weapon)]"),
    FAST("load_store", world_load_store, "load_store(id, payload, station, store, count) -> reason"),
    FAST("load_loadout", world_load_loadout, "load_loadout(id, name) -> reason"),
    FAST("strike", world_strike, "strike(id, payload, store, quantity, stores, target, consent, ripple_s, source=0, controller=0, "
                                 "command_id=0) -> result"),
    FAST("jettison", world_jettison, "jettison(id, payload, stores, emergency, source=0, controller=0, command_id=0) -> result"),
    FAST("strike_consent", world_strike_consent, "strike_consent(activity, granted) -> reason"),
    FAST("strike_status", world_strike_status, "strike_status(activity) -> (phase, jettison, target, stores, weapons) or None"),
    FAST("mass_kg", world_mass_kg, "mass_kg(id) -> float"),
    FAST("vehicle_role", world_vehicle_role, "vehicle_role(id) -> (role, launcher)"),
    FAST("amti_search", world_amti_search,
         "amti_search(id, payload, mode, az_centre, az_half_width, el_centre, bars, range_scale, signal_level, source=0, controller=0, "
         "command_id=0) -> result"),
    FAST("radar_status", world_radar_status, "radar_status(id, payload) -> tuple, or None for a payload that is not a radar"),
    FAST("amti_tracks", world_amti_tracks, "amti_tracks(id, payload) -> [(id, payload, time, last_update, updates, truth, lat, lon, h, vn, ve, vd, sp, sv, range, az, el, rate)]"),
    FAST("contacts", world_contacts, "contacts(id, payload) -> [(id, payload, kind, mode, warning, truth, first, time, az, el, sigma, range, rate, lat, lon, h, strength, frequency, size)]"),
    FAST("amti_detections", world_amti_detections, "amti_detections(id, payload, since) -> [(serial, payload, time, truth, range, az, el, rate, snr, lat, lon, h)]"),
    FAST("radar_cross_section", world_radar_cross_section,
         "radar_cross_section(observer, target, frequency_hz, tx_rad, rx_rad) -> (modelled, sigma_m2, fluctuation, az_rad, el_rad)"),
    FAST("radio_path", world_radio_path, "radio_path(from, to, frequency_hz, polarization, time_percent, rain_mmh, rain_height_m, itm) -> tuple"),
    FAST("performance", world_performance, "performance(id) -> (revision, hovers, 23 floats in fsim_performance's order)"),
    FAST("control_revision", world_control_revision, "control_revision(id) -> int"),
    FAST("set_control_mode", world_set_control_mode, "set_control_mode(id, mode)"),
    FAST("control_mode", world_control_mode, "control_mode(id) -> mode"),
    FAST("request_control", world_request_control, "request_control(id, capability) -> reason, 0 if granted"),
    FAST("release_control", world_release_control, "release_control(id, capability)"),
    FAST("revoke_control", world_revoke_control, "revoke_control(id, capability, reason)"),
    FAST("set_allowed", world_set_allowed, "set_allowed(id, capability, allowed)"),
    FAST("control_status", world_control_status, "control_status(id, capability) -> (allowed, granted)"),
    FAST("set_capability_precedence", world_set_capability_precedence, "set_capability_precedence(id, capability, precedence)"),
    FAST("capability_precedence", world_capability_precedence, "capability_precedence(id, capability) -> precedence"),
    FAST("set_availability", world_set_availability, "set_availability(id, capability, availability, reason)"),
    FAST("set_availability_ex", world_set_availability_ex, "set_availability_ex(id, capability, availability, reason, associated, next_available_s)"),
    FAST("capability_status_info", world_capability_status_info,
         "capability_status_info(id, capability) -> (availability, reason, reasons, description, associated, next_available_s, ranges)"),
    FAST("support", world_support, "support(id, feature) -> (feature, support, rules, stage, capability, missing, evidence)"),
    FAST("profile_value", world_profile_value, "profile_value(id, path) -> float, NaN if unknown"),
    FAST("profile_section", world_profile_section, "profile_section(id, section) -> (version, provenance)"),
    FAST("set_vehicle_default", world_set_vehicle_default, "set_vehicle_default(id, mode) -> reason, 0 if set"),
    FAST("set_protection", world_set_protection, "set_protection(id, mode)"),
    FAST("protection", world_protection, "protection(id) -> mode"),
    FAST("envelope", world_envelope, "envelope(id) -> (mode, [(limited_updates, exceeded_updates, exceeded_s, worst_excess)])"),
    FAST("vehicle_default", world_vehicle_default, "vehicle_default(id) -> mode"),
    FAST("active_level", world_active_level, "active_level(id)"),
    FAST("behavior_finished", world_behavior_finished, "behavior_finished(id)"),
    FAST("use_controller", world_use_controller, "use_controller(id, level, controller_id)"),
    FAST("set_controller_parameter", world_set_controller_parameter, "set_controller_parameter(id, level, name, value)"),
    FAST("controller_parameter", world_controller_parameter, "controller_parameter(id, level, name) -> float"),
    FAST("get_property", world_get_property, "get_property(id, path)"),
    FAST("set_property", world_set_property, "set_property(id, path, value)"),
    FAST("get_environment", world_get_environment, "the environment as a dict"),
    FAST("set_environment", world_set_environment, "set_environment(changes: dict)"),
    FAST("add_effect", world_add_effect, "add_effect(id or 0 for all, effect_id, params=None)"),
    FAST("clear_effects", world_clear_effects, "clear_effects(id)"),
    FAST("comm_create_node", world_comm_create_node, "comm_create_node(address)"),
    FAST("create_c2_node", world_create_c2_node, "create_c2_node(name, latitude_rad, longitude_rad, altitude_msl_m) -> id (AB-4)"),
    FAST("remove_c2_node", world_remove_c2_node, "remove_c2_node(id) -> bool"),
    FAST("create_net", world_create_net, "create_net(name, {field: number}) -> net id (AB-4)"),
    FAST("join_net", world_join_net, "join_net(entity, net) -> bool"),
    FAST("leave_net", world_leave_net, "leave_net(entity, net) -> bool"),
    FAST("assign_system", world_assign_system, "assign_system(entity, system, tail_number) -> SystemID (0: refused)"),
    FAST("dms_system", world_dms_system, "dms_system(entity) -> its SystemID (0: on no net)"),
    FAST("dms_nets", world_dms_nets, "dms_nets(entity) -> [(net, name)]"),
    FAST("dms_send", world_dms_send, "dms_send(entity, topic, data, destinations) -> message number"),
    FAST("dms_subscribe", world_dms_subscribe, "dms_subscribe(entity, topic, on) -> bool"),
    FAST("dms_subscribe_all", world_dms_subscribe_all, "dms_subscribe_all(entity, on) -> bool"),
    FAST("dms_subscriptions", world_dms_subscriptions, "dms_subscriptions(entity) -> ([topics], all)"),
    FAST("dms_inbox", world_dms_inbox, "dms_inbox(entity) -> [(message, source, topic, net, time_sent, time_delivered, bytes)]"),
    FAST("dms_inbox_take", world_dms_inbox_take, "dms_inbox_take(entity, count): the first count taken"),
    FAST("dms_transmission", world_dms_transmission, "dms_transmission(entity, message) -> its status, or None"),
    FAST("system_entity", world_system_entity, "system_entity(system) -> entity (the referee's)"),
    FAST("entity_system", world_entity_system, "entity_system(entity) -> SystemID (the referee's)"),
    FAST("net_count", world_net_count, "net_count() -> nets (numbered 1 .. count)"),
    FAST("net_stats", world_net_stats, "net_stats(net) -> its traffic (the referee's)"),
    FAST("set_dissemination", world_set_dissemination, "set_dissemination(entity, position_s, status_s, tracks_s, peer_timeout_s, track_timeout_s) (AB-4b)"),
    FAST("dms_publish", world_dms_publish, "dms_publish(entity, topic, payload, destinations) -> (first, count): native messages (AB-4b)"),
    FAST("dms_peers", world_dms_peers, "dms_peers(entity) -> its picture's peers"),
    FAST("dms_link_tracks", world_dms_link_tracks, "dms_link_tracks(entity) -> the tracks it received"),
    FAST("designate_link", world_designate_link, "designate_link(id, source, payload, track) -> target"),
    FAST("designation_source", world_designation_source, "designation_source(id, target) -> its source system (0: its own radar)"),
    FAST("link_track_truth", world_link_track_truth, "link_track_truth(source, payload, track) -> entity (the referee's)"),
    FAST("set_rbac", world_set_rbac, "set_rbac(entity, [(id, name, permissions, approves_strikes)]) (AB-5a)"),
    FAST("rbac", world_rbac, "rbac(entity) -> its roles"),
    FAST("bind_ma_service", world_bind_ma_service,
         "bind_ma_service(service_id, platform, controller, capabilities, owns_dms_ingress, owns_dms_egress) (MA services)"),
    FAST("ma_services", world_ma_services, "ma_services() -> [(service_id, platform, controller, capabilities, ingress, egress)]"),
    FAST("ma_capability_owner", world_ma_capability_owner, "ma_capability_owner(platform, capability) -> controller (0: none)"),
    FAST("agent_authority", world_agent_authority, "agent_authority(entity) -> the authority over its platform (AB-5a)"),
    FAST("agent_report", world_agent_report,
         "agent_report(entity, kind, stale_after_s) -> (availability, [(source, validity, system, net, payload, sample, receive, age)])"),
    FAST("agent_operator_callsign", world_agent_operator_callsign, "agent_operator_callsign(entity, index) -> str"),
    FAST("c2_declare_role", world_c2_declare_role, "c2_declare_role(entity, role_id, callsign, destinations)"),
    FAST("c2_designate_control", world_c2_designate_control, "c2_designate_control(entity, ma, modes) -> command id"),
    FAST("c2_command_hsa", world_c2_command_hsa, "c2_command_hsa(entity, ma, 6 HSA fields) -> command id"),
    FAST("c2_command_route", world_c2_command_route, "c2_command_route(entity, ma, points) -> command id"),
    FAST("c2_cancel_command", world_c2_cancel_command, "c2_cancel_command(entity, ma, command id)"),
    FAST("c2_command_statuses", world_c2_command_statuses, "c2_command_statuses(entity) -> its commands' last statuses"),
    FAST("set_strike_policy", world_set_strike_policy, "set_strike_policy(entity, policy) (AB-5b)"),
    FAST("strike_policy", world_strike_policy, "strike_policy(entity) -> its policy"),
    FAST("agent_c2_tasks", world_agent_c2_tasks, "agent_c2_tasks(entity) -> C2's tasks (AB-5b)"),
    FAST("agent_answer_task", world_agent_answer_task, "agent_answer_task(entity, from, task_id, state, code, percent) -> bool"),
    FAST("agent_overrides", world_agent_overrides, "agent_overrides(entity) -> the overrides in force"),
    FAST("c2_send_task", world_c2_send_task, "c2_send_task(entity, ma, ...) -> task id"),
    FAST("c2_set_override", world_c2_set_override, "c2_set_override(entity, ma, kind, cancel, value, expiry_s) -> command id"),
    FAST("c2_consents", world_c2_consents, "c2_consents(entity) -> the consent requests it received"),
    FAST("c2_answer_consent", world_c2_answer_consent, "c2_answer_consent(entity, ma, request_id, granted) -> bool"),
    FAST("agent_comms", world_agent_comms, "agent_comms(entity) -> its lost-communication state and team (AB-5c)"),
    FAST("c2_set_lost_comms", world_c2_set_lost_comms, "c2_set_lost_comms(entity, ma, threshold_s, plan_id) -> command id"),
    FAST("c2_package", world_c2_package, "c2_package(entity, package_id, leader, systems, dissolve) -> command id"),
    FAST("comm_send", world_comm_send, "comm_send(from, to, channel, format, data)"),
    FAST("comm_inbox", world_comm_inbox, "comm_inbox(node) -> messages"),
    FAST("comm_set_medium", world_comm_set_medium, "comm_set_medium(medium_id, params=None)"),
    FAST("comm_attach_protocol", world_comm_attach_protocol, "comm_attach_protocol(node, protocol_id, params=None)"),
    FAST("comm_attach_udp_bridge", world_comm_attach_udp_bridge, "comm_attach_udp_bridge(node, local_port, host, port)"),
    FAST("apply_scenario", world_apply_scenario, "apply_scenario(scenario) -> ids as uint32 bytes"),
    FAST("borrowed", world_borrowed_flag, "True for a VecEnv's world"),
    FAST("_capsule", world_capsule, "for fsim._vision"),
    {NULL, NULL, 0, NULL}};

static PyType_Slot world_slots[] = {
    {Py_tp_new, (void*)world_new},
    {Py_tp_dealloc, (void*)world_dealloc},
    {Py_tp_methods, (void*)world_methods},
    {Py_tp_doc, (void*)"World(options: dict) over fsim_world (see fsim.World)"},
    {0, NULL}};
static PyType_Spec world_spec = {"fsim._native.World", sizeof(WorldObject), 0, Py_TPFLAGS_DEFAULT, world_slots};

/* ==========================================================================
 * Scenario
 * ======================================================================== */

static PyMethodDef scenario_methods[] = {
    FAST("world_options", scenario_world_options, "the scenario's world section as World options"),
    FAST("vehicle_count", scenario_vehicle_count, "vehicle instances, counting 'count'"),
    {NULL, NULL, 0, NULL}};

static PyType_Slot scenario_slots[] = {
    {Py_tp_new, (void*)scenario_new},
    {Py_tp_dealloc, (void*)scenario_dealloc},
    {Py_tp_methods, (void*)scenario_methods},
    {Py_tp_doc, (void*)"Scenario(path) or Scenario(None, json, source) over fsim_scenario"},
    {0, NULL}};
static PyType_Spec scenario_spec = {"fsim._native.Scenario", sizeof(ScenarioObject), 0, Py_TPFLAGS_DEFAULT, scenario_slots};

/* ==========================================================================
 * Recording
 * ======================================================================== */

/* samples(frame): the frame's fsim_recorded_sample array, zero-copy. */
static PyObject* recording_samples(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    uint32_t frame, count = 0;
    if (!check_args(n, 1, 1, "samples") || !as_u32(args[0], &frame)) return NULL;
    const fsim_recorded_sample* s = fsim_recording_samples(((RecordingObject*)o)->recording, frame, &count);
    if (!s || !count) return PyBytes_FromStringAndSize(NULL, 0);
    return fsim_py_buffer_new(BufferType, o, s, (Py_ssize_t)count * (Py_ssize_t)sizeof(fsim_recorded_sample), 1);
}


static PyMethodDef recording_methods[] = {
    FAST("info", recording_info, "frame count, world name, dt, frame skip"),
    FAST("frame_time", recording_frame_time, "frame_time(frame) -> seconds"),
    FAST("samples", recording_samples, "samples(frame): fsim_recorded_sample array, zero-copy"),
    FAST("events", recording_events, "events(frame) -> list of dicts"),
    {NULL, NULL, 0, NULL}};

static PyType_Slot recording_slots[] = {
    {Py_tp_new, (void*)recording_new},
    {Py_tp_dealloc, (void*)recording_dealloc},
    {Py_tp_methods, (void*)recording_methods},
    {Py_tp_doc, (void*)"Recording(path) over fsim_recording"},
    {0, NULL}};
static PyType_Spec recording_spec = {"fsim._native.Recording", sizeof(RecordingObject), 0, Py_TPFLAGS_DEFAULT, recording_slots};

/* ==========================================================================
 * Module
 * ======================================================================== */
static PyObject* mod_version(PyObject* m, PyObject* const* args, Py_ssize_t n) {
    (void)m;
    (void)args;
    if (!check_args(n, 0, 0, "version")) return NULL;
    return Py_BuildValue("(sI)", fsim_version(), fsim_abi_version());
}

static PyObject* mod_registered_ids(PyObject* m, PyObject* const* args, Py_ssize_t n) {
    int registry;
    (void)m;
    if (!check_args(n, 1, 1, "registered_ids") || !as_int(args[0], &registry)) return NULL;
    PyObject* list = PyList_New(0);
    if (!list) return NULL;
    for (uint32_t i = 0;; ++i) {
        const char* id = fsim_registered_id(registry, i);
        if (!id || !*id) break;
        PyObject* s = PyUnicode_FromString(id);
        if (!s || PyList_Append(list, s) < 0) {
            Py_XDECREF(s);
            Py_DECREF(list);
            return NULL;
        }
        Py_DECREF(s);
    }
    return list;
}


int as_doubles(PyObject* const* args, Py_ssize_t from, Py_ssize_t count, double* out) {
    for (Py_ssize_t i = 0; i < count; ++i) {
        out[i] = PyFloat_AsDouble(args[from + i]);
        if (out[i] == -1.0 && PyErr_Occurred()) return 0;
    }
    return 1;
}

/* layout(): the C structs' sizes and field offsets, so the Python mirrors can
 * be checked against the library they are about to read. */
#define OFS(T, f) #f, (unsigned long long)offsetof(T, f)
static PyObject* mod_layout(PyObject* m, PyObject* const* args, Py_ssize_t n) {
    (void)m;
    (void)args;
    if (!check_args(n, 0, 0, "layout")) return NULL;
    PyObject* state = Py_BuildValue(
        "{s:K,s:K,s:K,s:K,s:K,s:K,s:K,s:K,s:K,s:K,s:K,s:K,s:K,s:K,s:K,s:K,s:K,s:K,s:K}", "size", (unsigned long long)sizeof(fsim_vehicle_state),
        OFS(fsim_vehicle_state, position_ecef), OFS(fsim_vehicle_state, latitude_rad), OFS(fsim_vehicle_state, euler_rad),
        OFS(fsim_vehicle_state, velocity_ned_ms), OFS(fsim_vehicle_state, airspeed_true_ms), OFS(fsim_vehicle_state, load_factor),
        OFS(fsim_vehicle_state, gear_position), OFS(fsim_vehicle_state, engine_count), OFS(fsim_vehicle_state, throttle_position),
        OFS(fsim_vehicle_state, fuel_kg), OFS(fsim_vehicle_state, step_count), OFS(fsim_vehicle_state, on_ground),
        OFS(fsim_vehicle_state, rotation_body_to_ecef), OFS(fsim_vehicle_state, engine_rpm),
        OFS(fsim_vehicle_state, afterburner), OFS(fsim_vehicle_state, leading_edge_flap_rad),
        OFS(fsim_vehicle_state, wheel_count), OFS(fsim_vehicle_state, wheel_speed_ms));
    PyObject* inputs = Py_BuildValue("{s:K,s:K,s:K}", "size", (unsigned long long)sizeof(fsim_control_inputs),
                                     OFS(fsim_control_inputs, throttle), OFS(fsim_control_inputs, brake_right));
    PyObject* sample = Py_BuildValue("{s:K,s:K,s:K}", "size", (unsigned long long)sizeof(fsim_recorded_sample),
                                     OFS(fsim_recorded_sample, state), OFS(fsim_recorded_sample, inputs));
    if (!state || !inputs || !sample) {
        Py_XDECREF(state);
        Py_XDECREF(inputs);
        Py_XDECREF(sample);
        return NULL;
    }
    return Py_BuildValue("{s:N,s:N,s:N}", "vehicle_state", state, "control_inputs", inputs, "recorded_sample", sample);
}
#undef OFS

static PyObject* mod_set_log_level(PyObject* m, PyObject* const* args, Py_ssize_t n) {
    int level;
    (void)m;
    if (!check_args(n, 1, 1, "set_log_level") || !as_int(args[0], &level)) return NULL;
    fsim_set_log_level(level);
    Py_RETURN_NONE;
}

static PyObject* mod_log_level(PyObject* m, PyObject* const* args, Py_ssize_t n) {
    (void)m;
    (void)args;
    if (!check_args(n, 0, 0, "log_level")) return NULL;
    return PyLong_FromLong(fsim_log_level());
}

static PyMethodDef module_methods[] = {
    FAST("version", mod_version, "(library version, ABI version)"),
    FAST("set_log_level", mod_set_log_level, "set_log_level(0 trace .. 4 error, 5 off)"),
    FAST("log_level", mod_log_level, "the platform log's level"),
    FAST("registered_ids", mod_registered_ids, "registered_ids(REGISTRY_TASK | REGISTRY_OBSERVATION | REGISTRY_ACTION)"),
    FAST("command_field_count", mod_command_field_count, "command_field_count(level)"),
    FAST("command_field_count_full", mod_command_field_count_full, "command_field_count_full(level)"),
    FAST("reason_name", mod_reason_name, "reason_name(code): why a command was refused or an activity ended"),
    FAST("ms_reason_name", mod_ms_reason_name, "ms_reason_name(code): a Mission Systems reason"),
    FAST("role_name", mod_role_name, "role_name(code): aircraft or weapon"),
    FAST("weapon_end_name", mod_weapon_end_name, "weapon_end_name(code): in_flight, impact or removed"),
    FAST("store_status_name", mod_store_status_name, "store_status_name(code): loaded, powering_up, ready, releasing, released, jettisoned, powering_down"),
    FAST("strike_phase_name", mod_strike_phase_name, "strike_phase_name(code): powering_up, awaiting_consent, releasing, done"),
    FAST("amti_mode_name", mod_amti_mode_name,
         "amti_mode_name(code): range_while_search, velocity_search, track_while_scan, surveillance or single_target_track"),
    FAST("amti_task_name", mod_amti_task_name, "amti_task_name(code): volume, cued or track"),
    FAST("amti_acm", mod_amti_acm, "amti_acm(pattern) -> (mode, az_centre, az_half_width, el_centre, bars, range_scale, el_half_width)"),
    FAST("ms_reason_description", mod_ms_reason_description, "ms_reason_description(code)"),
    FAST("payload_family_name", mod_payload_family_name, "payload_family_name(code): \"air_radar\" ..."),
    FAST("payload_state_name", mod_payload_state_name, "payload_state_name(code): \"off\" ..."),
    FAST("ms_capability_type_id", mod_ms_capability_type_id, "ms_capability_type_id(code): \"fsim.ms.amti\" ..."),
    FAST("activity_state_name", mod_activity_state_name, "activity_state_name(code)"),
    FAST("limit_name", mod_limit_name, "limit_name(i): an envelope limit, \"load_factor_max\" ..."),
    FAST("constraint_name", mod_constraint_name, "constraint_name(i): a performance limit a value broke, \"max_airspeed\" ..."),
    FAST("flight_mode_name", mod_flight_mode_name, "flight_mode_name(i): an A-GRA flight capability type, \"hsa_csa\" ..."),
    FAST("support_features", mod_support_features, "support_features(): every public feature identifier"),
    FAST("support_name", mod_support_name, "support_name(i): \"supported\", \"partial\", \"not_implemented\", \"not_supported\""),
    FAST("rule_name", mod_rule_name, "rule_name(i): an applicability rule, \"R1\" ..."),
    FAST("rule_description", mod_rule_description, "rule_description(i): what the rule says"),
    FAST("availability_name", mod_availability_name, "availability_name(i): \"available\", \"temporarily_unavailable\" ..."),
    FAST("reason_description", mod_reason_description, "reason_description(code): the reason in words"),
    FAST("layout", mod_layout, "C struct sizes and offsets"),
    FAST("magnetic_field", mod_magnetic_field, "magnetic_field(latitude_rad, longitude_rad, height_m, decimal_year): the World Magnetic Model 2025"),
    FAST("decimal_year", mod_decimal_year, "decimal_year(unix_seconds)"),
    {NULL, NULL, 0, NULL}};

static struct PyModuleDef module = {PyModuleDef_HEAD_INIT, "fsim._native", "fsim's native core over the C ABI", -1,
                                    module_methods, NULL, NULL, NULL, NULL};

static int add_type(PyObject* m, PyType_Spec* spec, PyTypeObject** out, const char* name) {
    *out = (PyTypeObject*)PyType_FromSpec(spec);
    if (!*out) return -1;
    return PyModule_AddObjectRef(m, name, (PyObject*)*out);
}

PyMODINIT_FUNC PyInit__native(void) {
    PyObject* m = PyModule_Create(&module);
    if (!m) return NULL;
    Error = PyErr_NewExceptionWithDoc("fsim.Error", "A call into the platform failed; the message says why.", PyExc_RuntimeError, NULL);
    if (!Error || PyModule_AddObjectRef(m, "Error", Error) < 0 ||
        add_type(m, &fsim_py_buffer_spec, &BufferType, "Buffer") < 0 || add_type(m, &vecenv_spec, &VecEnvType, "VecEnv") < 0 ||
        add_type(m, &world_spec, &WorldType, "World") < 0 || add_type(m, &scenario_spec, &ScenarioType, "Scenario") < 0 ||
        add_type(m, &recording_spec, &RecordingType, "Recording") < 0 ||
        PyModule_AddIntConstant(m, "LEVEL_ACTUATOR", FSIM_LEVEL_ACTUATOR) < 0 ||
        PyModule_AddIntConstant(m, "LEVEL_ATTITUDE", FSIM_LEVEL_ATTITUDE) < 0 ||
        PyModule_AddIntConstant(m, "LEVEL_ACCELERATION", FSIM_LEVEL_ACCELERATION) < 0 ||
        PyModule_AddIntConstant(m, "LEVEL_VELOCITY", FSIM_LEVEL_VELOCITY) < 0 ||
        PyModule_AddIntConstant(m, "LEVEL_POSITION", FSIM_LEVEL_POSITION) < 0 ||
        PyModule_AddIntConstant(m, "LEVEL_BEHAVIOR", FSIM_LEVEL_BEHAVIOR) < 0 ||
        PyModule_AddIntConstant(m, "AUTORESET_NEXT_STEP", FSIM_AUTORESET_NEXT_STEP) < 0 ||
        PyModule_AddIntConstant(m, "AUTORESET_SAME_STEP", FSIM_AUTORESET_SAME_STEP) < 0 ||
        PyModule_AddIntConstant(m, "REGISTRY_TASK", FSIM_REGISTRY_TASK) < 0 ||
        PyModule_AddIntConstant(m, "REGISTRY_OBSERVATION", FSIM_REGISTRY_OBSERVATION) < 0 ||
        PyModule_AddIntConstant(m, "REGISTRY_ACTION", FSIM_REGISTRY_ACTION) < 0) {
        Py_DECREF(m);
        return NULL;
    }
    PyObject* hold = PyFloat_FromDouble(fsim_hold());
    if (!hold || PyModule_AddObjectRef(m, "HOLD", hold) < 0) {
        Py_XDECREF(hold);
        Py_DECREF(m);
        return NULL;
    }
    Py_DECREF(hold);
    return m;
}
