/* Control and vehicle-interface bindings for fsim._native. */
#define FSIM_PY_NO_BUFFER_TYPE
#include "_native_world.h"

#include <math.h>

/* command(id, level, values): one vehicle, the level's fields as a sequence of floats. */
PyObject* world_command(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    int level;
    if (!check_args(n, 3, 3, "command") || !as_u32(args[0], &id) || !as_int(args[1], &level) || !WORLD_IDLE(self)) return NULL;
    const uint32_t fields = fsim_command_field_count(level);
    double row[FSIM_PY_VALUES];
    PyObject* seq = PySequence_Fast(args[2], "command: values must be a sequence of numbers");
    if (!seq) return NULL;
    if (!fields || PySequence_Size(seq) != (Py_ssize_t)fields) {
        PyErr_Format(PyExc_ValueError, "command: level %d takes %u values", level, (unsigned)fields);
        Py_DECREF(seq);
        return NULL;
    }
    for (uint32_t i = 0; i < fields; ++i) {
        PyObject* item = PySequence_GetItem(seq, (Py_ssize_t)i);
        row[i] = item ? PyFloat_AsDouble(item) : -1.0;
        Py_XDECREF(item);
        if (PyErr_Occurred()) {
            Py_DECREF(seq);
            return NULL;
        }
    }
    Py_DECREF(seq);
    if (fsim_world_command_batch(self->world, level, &id, 1, row, 0) != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

/* command_batch(level, ids uint32[n], values float64[n][fields]) */
PyObject* world_command_batch(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    int level;
    if (!check_args(n, 3, 3, "command_batch") || !as_int(args[0], &level) || !WORLD_IDLE(self)) return NULL;
    const uint32_t fields = fsim_command_field_count(level);
    if (!fields) {
        PyErr_Format(PyExc_ValueError, "command_batch: level %d cannot be batched", level);
        return NULL;
    }
    Py_buffer ids, values;
    if (!get_array(args[1], &ids, 0, "IL", 4, "command_batch ids")) return NULL;
    if (!get_array(args[2], &values, 0, "d", 8, "command_batch values")) {
        PyBuffer_Release(&ids);
        return NULL;
    }
    const Py_ssize_t count = ids.len / 4;
    if (values.len != count * (Py_ssize_t)fields * 8) {
        PyErr_Format(PyExc_ValueError, "command_batch: %zd vehicles at level %d need %zd values, got %zd", count, level,
                     count * (Py_ssize_t)fields, values.len / 8);
        PyBuffer_Release(&ids);
        PyBuffer_Release(&values);
        return NULL;
    }
    const int rc = fsim_world_command_batch(self->world, level, (const uint32_t*)ids.buf, (uint32_t)count, (const double*)values.buf, fields);
    PyBuffer_Release(&ids);
    PyBuffer_Release(&values);
    if (rc != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

/* A route: rows of (latitude_rad, longitude_rad, altitude_msl_m, airspeed_ms,
 * capture_radius_m). *out is PyMem-allocated (free it); 0, or -1 with an error set. */
static int read_points(PyObject* o, fsim_position_command** out, uint32_t* count) {
    *out = NULL;
    *count = 0;
    PyObject* seq = PySequence_Fast(o, "points must be a sequence of 5-number rows");
    if (!seq) return -1;
    const uint32_t point_count = (uint32_t)PySequence_Size(seq);
    fsim_position_command* points = (fsim_position_command*)PyMem_Malloc(sizeof(fsim_position_command) * (point_count ? point_count : 1));
    for (uint32_t i = 0; points && i < point_count; ++i) {
        PyObject* row = PySequence_GetItem(seq, (Py_ssize_t)i);
        double v[5] = {0, 0, 0, 0, 0};
        PyObject* r = row ? PySequence_Fast(row, "each point must be 5 numbers") : NULL;
        if (r && PySequence_Size(r) == 5)
            for (int k = 0; k < 5; ++k) {
                PyObject* item = PySequence_GetItem(r, k);
                v[k] = item ? PyFloat_AsDouble(item) : 0.0;
                Py_XDECREF(item);
            }
        else if (r) PyErr_SetString(PyExc_ValueError, "each point must be (latitude_rad, longitude_rad, altitude_msl_m, airspeed_ms, capture_radius_m)");
        Py_XDECREF(r);
        Py_XDECREF(row);
        if (PyErr_Occurred()) break;
        points[i].latitude_rad = v[0];
        points[i].longitude_rad = v[1];
        points[i].altitude_msl_m = v[2];
        points[i].airspeed_ms = v[3];
        points[i].capture_radius_m = v[4];
    }
    Py_DECREF(seq);
    if (!points) PyErr_NoMemory();
    if (PyErr_Occurred()) {
        PyMem_Free(points);
        return -1;
    }
    *out = points;
    *count = point_count;
    return 0;
}

/* command_behavior(id, behavior_id, target, params dict, points) - points: rows of 5 floats */
PyObject* world_command_behavior(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id, target = 0;
    if (!check_args(n, 2, 5, "command_behavior") || !as_u32(args[0], &id) || !WORLD_IDLE(self)) return NULL;
    const char* bid = as_str(args[1], "behaviour id");
    if (!bid || (n >= 3 && args[2] != Py_None && !as_u32(args[2], &target))) return NULL;
    fsim_py_params p;
    if (fsim_py_params_read(n >= 4 ? args[3] : NULL, &p) < 0) {
        fsim_py_params_free(&p);
        return NULL;
    }
    fsim_position_command* points = NULL;
    uint32_t point_count = 0;
    if (n >= 5 && args[4] != Py_None && read_points(args[4], &points, &point_count) < 0) {
        fsim_py_params_free(&p);
        return NULL;
    }
    fsim_behavior_command c;
    memset(&c, 0, sizeof c);
    c.id = bid;
    c.target = target;
    c.param_names = p.names;
    c.param_values = p.values;
    c.param_count = p.count;
    c.points = points;
    c.point_count = point_count;
    const int rc = fsim_vehicle_command_behavior(self->world, id, &c);
    PyMem_Free(points);
    fsim_py_params_free(&p);
    if (rc != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

/* --- Capabilities and activities (ABI 1.4) ------------------------------------------
 * Results are tuples (status, reason, activity, other, clamped); activity
 * infos (id, vehicle, capability, source, axes, state, reason, by,
 * constraints, constraints_seen, start_time, end_time). fsim/world.py wraps
 * them. */

/* (status, reason, activity, other, clamped, index, constraint, from, to, new_activity, command_id, associated,
 * description, finding_count, adjustment_count): from index 5 what the answer was about (ABI 1.6 and 1.8,
 * fsim_last_command_detail) - the field, route point or curve segment, the limit, a curve's section; then the
 * command envelope's, and how many findings and adjustments last_findings() and last_adjustments() have */
static PyObject* result_tuple_with(const fsim_command_result* r, const fsim_command_detail* dp);

static PyObject* result_tuple(const fsim_world* world, const fsim_command_result* r) {
    fsim_command_detail d;
    fsim_command_detail_init(&d);
    fsim_last_command_detail(world, &d);
    return result_tuple_with(r, &d);
}

/* As result_tuple, with its detail given (a batch item's). */
static PyObject* result_tuple_with(const fsim_command_result* r, const fsim_command_detail* dp) {
    const fsim_command_detail d = *dp;
    return Py_BuildValue("(iiKKOiiddOKKsIIOKO)", r->status, r->reason, (unsigned long long)r->activity, (unsigned long long)r->other,
                         (r->flags & FSIM_COMMAND_CLAMPED) ? Py_True : Py_False, d.index, d.constraint, d.from, d.to, d.new_activity ? Py_True : Py_False,
                         (unsigned long long)d.command_id, (unsigned long long)d.associated, d.description ? d.description : "", d.finding_count,
                         d.adjustment_count, (r->flags & FSIM_COMMAND_DEFERRED) ? Py_True : Py_False, (unsigned long long)d.suggestion,
                         (r->flags & FSIM_COMMAND_OVERRIDDEN) ? Py_True : Py_False);
}

/* (id, vehicle, capability, source, axes, state, reason, by, constraints, constraints_seen, start_time, end_time,
 * command_id, interactive, ((kind, id) per requirement it traces to), waiting, basis, (priority, precedence),
 * precedence, waiting_for, interrupt, (start_not_before, start_not_after, end_not_before, end_not_after, criticality)) */
static PyObject* info_tuple(const fsim_world* world, const fsim_activity_info* a) {
    fsim_activity_envelope e;
    fsim_activity_envelope_init(&e);
    fsim_activity_get_envelope(world, a->id, &e);
    Py_ssize_t count = 0;
    for (int i = 0; i < FSIM_MAX_REQUIREMENTS; ++i)
        if (e.trace[i].kind != FSIM_REQUIREMENT_NONE) ++count;
    PyObject* trace = PyTuple_New(count);
    for (int i = 0, k = 0; trace && i < FSIM_MAX_REQUIREMENTS; ++i) {
        if (e.trace[i].kind == FSIM_REQUIREMENT_NONE) continue;
        PyObject* item = Py_BuildValue("(iK)", e.trace[i].kind, (unsigned long long)e.trace[i].id);
        if (!item || PyTuple_SetItem(trace, k++, item) < 0) { /* (SetItem takes the item, even when it fails) */
            Py_CLEAR(trace);
            break;
        }
    }
    if (!trace) return NULL;
    return Py_BuildValue("(KIIiIiiKIIddKONii(II)IKO(ddddi)KIII)", (unsigned long long)a->id, a->vehicle, a->capability, a->source, a->axes, a->state,
                         a->reason, (unsigned long long)a->by, a->constraints, a->constraints_seen, a->start_time, a->end_time,
                         (unsigned long long)e.command_id, e.interactive ? Py_True : Py_False, trace, e.waiting, e.basis,
                         (unsigned int)e.rank_priority, (unsigned int)e.rank_precedence, e.precedence, (unsigned long long)e.waiting_for,
                         e.interrupt ? Py_True : Py_False, e.start_not_before, e.start_not_after, e.end_not_before, e.end_not_after, e.criticality,
                         (unsigned long long)e.suggestion, e.run, e.runs, e.controller);
}

/* How a command is arbitrated and scheduled (ABI 1.9): (interrupt, override_rejection, (priority, precedence),
 * precedence_override or None, (start_not_before, start_not_after, end_not_before, end_not_after), criticality). */
static int read_schedule(PyObject* const* item, fsim_command_options* o) {
    int ok = 1;
    const int interrupt = PyObject_IsTrue(item[0]), override = PyObject_IsTrue(item[1]);
    if (interrupt < 0 || override < 0) return 0;
    o->interrupt = interrupt > 0, o->override_rejection = override > 0;
    PyObject* priority = PySequence_Check(item[2]) && PySequence_Size(item[2]) == 2 ? PySequence_GetItem(item[2], 0) : NULL;
    PyObject* precedence = priority ? PySequence_GetItem(item[2], 1) : NULL;
    uint32_t p = 0, q = 0;
    ok = precedence && as_u32(priority, &p) && as_u32(precedence, &q) && p <= 0xFFFF && q <= 0xFFFF;
    if (!ok && !PyErr_Occurred()) PyErr_SetString(PyExc_ValueError, "a rank is (priority, precedence), each 0 .. 65535");
    o->rank_priority = (uint16_t)p, o->rank_precedence = (uint16_t)q;
    Py_XDECREF(precedence);
    Py_XDECREF(priority);
    if (ok && item[3] != Py_None) ok = as_u32(item[3], &o->precedence_override);
    if (ok) {
        double* bounds[] = {&o->start_not_before, &o->start_not_after, &o->end_not_before, &o->end_not_after};
        ok = PySequence_Check(item[4]) && PySequence_Size(item[4]) == 4;
        for (Py_ssize_t i = 0; ok && i < 4; ++i) {
            PyObject* t = PySequence_GetItem(item[4], i);
            ok = t != NULL;
            if (ok) *bounds[i] = PyFloat_AsDouble(t), ok = !PyErr_Occurred();
            Py_XDECREF(t);
        }
        if (!ok && !PyErr_Occurred()) PyErr_SetString(PyExc_ValueError, "a window is four times (NaN: no bound)");
    }
    if (ok) ok = as_int(item[5], &o->criticality);
    return ok;
}

/* The command envelope (ABI 1.8): (command_id, ((kind, id), ...), interactive, validate_only), then (1.9) how it is
 * arbitrated and scheduled (read_schedule's six), then (1.12) a policy's controller. */
static int read_envelope(PyObject* e, fsim_command_options* o) {
    static const char* shape = "the envelope must be (command_id, trace, interactive, validate_only[, the schedule's six[, controller]])";
    const Py_ssize_t size = PySequence_Check(e) ? PySequence_Size(e) : -1;
    if (size != 4 && size != 10 && size != 11) {
        if (!PyErr_Occurred()) PyErr_SetString(PyExc_ValueError, shape);
        return 0;
    }
    if (size == 11) {
        PyObject* controller = PySequence_GetItem(e, 10);
        const int ok = controller && as_u32(controller, &o->controller);
        Py_XDECREF(controller);
        if (!ok) return 0;
    }
    if (size >= 10) {
        PyObject* item[6] = {NULL, NULL, NULL, NULL, NULL, NULL};
        int ok = 1;
        for (Py_ssize_t i = 0; i < 6; ++i) ok = (item[i] = PySequence_GetItem(e, 4 + i)) != NULL && ok;
        if (ok) ok = read_schedule(item, o);
        for (Py_ssize_t i = 0; i < 6; ++i) Py_XDECREF(item[i]);
        if (!ok) return 0;
    }
    PyObject* id = PySequence_GetItem(e, 0);
    PyObject* trace = PySequence_GetItem(e, 1);
    PyObject* interactive = PySequence_GetItem(e, 2);
    PyObject* validate = PySequence_GetItem(e, 3);
    int ok = id && trace && interactive && validate && as_u64(id, &o->command_id);
    const Py_ssize_t count = ok ? PySequence_Size(trace) : 0;
    if (ok && (count < 0 || count > FSIM_MAX_REQUIREMENTS)) {
        if (!PyErr_Occurred()) PyErr_Format(PyExc_ValueError, "a command traces to at most %d requirements", FSIM_MAX_REQUIREMENTS);
        ok = 0;
    }
    for (Py_ssize_t i = 0; ok && i < count; ++i) {
        PyObject* pair = PySequence_GetItem(trace, i);
        PyObject* kind = pair && PySequence_Check(pair) && PySequence_Size(pair) == 2 ? PySequence_GetItem(pair, 0) : NULL;
        PyObject* rid = kind ? PySequence_GetItem(pair, 1) : NULL;
        ok = rid && as_int(kind, &o->trace[i].kind) && as_u64(rid, &o->trace[i].id);
        if (!ok && !PyErr_Occurred()) PyErr_SetString(PyExc_ValueError, "each requirement must be (kind, id)");
        Py_XDECREF(rid);
        Py_XDECREF(kind);
        Py_XDECREF(pair);
    }
    if (ok) {
        const int yes = PyObject_IsTrue(interactive), check = PyObject_IsTrue(validate);
        ok = yes >= 0 && check >= 0;
        o->interactive = yes > 0, o->validate_only = check > 0;
    }
    Py_XDECREF(validate);
    Py_XDECREF(interactive);
    Py_XDECREF(trace);
    Py_XDECREF(id);
    return ok;
}

/* source, axes, range, min_version, envelope: optional, None = the default */
static int read_options(PyObject* const* args, Py_ssize_t n, Py_ssize_t first, fsim_command_options* o) {
    int v;
    uint32_t u;
    fsim_command_options_init(o);
    if (n > first && args[first] != Py_None) {
        if (!as_int(args[first], &v)) return 0;
        o->source = v;
    }
    if (n > first + 1 && args[first + 1] != Py_None) {
        if (!as_u32(args[first + 1], &u)) return 0;
        o->axes = u;
    }
    if (n > first + 2 && args[first + 2] != Py_None) {
        if (!as_int(args[first + 2], &v)) return 0;
        o->range = v;
    }
    if (n > first + 3 && args[first + 3] != Py_None) {
        if (!as_u32(args[first + 3], &u)) return 0;
        o->min_version = u;
    }
    if (n > first + 4 && args[first + 4] != Py_None && !read_envelope(args[first + 4], o)) return 0;
    return 1;
}

/* A sequence of at most FSIM_PY_VALUES numbers into row; returns how many, or -1. */
static Py_ssize_t read_values(PyObject* o, double* row, const char* what) {
    PyObject* seq = PySequence_Fast(o, "values must be a sequence of numbers");
    if (!seq) return -1;
    const Py_ssize_t count = PySequence_Size(seq);
    if (count > FSIM_PY_VALUES) {
        PyErr_Format(PyExc_ValueError, "%s: at most %d values", what, FSIM_PY_VALUES);
        Py_DECREF(seq);
        return -1;
    }
    for (Py_ssize_t i = 0; i < count; ++i) {
        PyObject* item = PySequence_GetItem(seq, i);
        row[i] = item ? PyFloat_AsDouble(item) : -1.0;
        Py_XDECREF(item);
        if (PyErr_Occurred()) {
            Py_DECREF(seq);
            return -1;
        }
    }
    Py_DECREF(seq);
    return count;
}

/* submit(id, level, values, source=None, axes=None, range=None, min_version=None) -> result */
PyObject* world_submit(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    int level;
    double row[FSIM_PY_VALUES];
    fsim_command_options opt;
    fsim_command_result r;
    if (!check_args(n, 3, 8, "submit") || !as_u32(args[0], &id) || !as_int(args[1], &level) || !WORLD_IDLE(self)) return NULL;
    const Py_ssize_t count = read_values(args[2], row, "submit");
    if (count < 0 || !read_options(args, n, 3, &opt)) return NULL;
    if (fsim_vehicle_submit(self->world, id, level, row, (uint32_t)count, &opt, &r) != FSIM_OK) return fail();
    return result_tuple(self->world, &r);
}

/* submit_behavior(id, behavior, target=0, params=None, points=None, source=None, axes=None, range=None, min_version=None) */
PyObject* world_submit_behavior(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id, target = 0;
    fsim_command_options opt;
    fsim_command_result r;
    if (!check_args(n, 2, 10, "submit_behavior") || !as_u32(args[0], &id) || !WORLD_IDLE(self)) return NULL;
    const char* bid = as_str(args[1], "behaviour id");
    if (!bid || (n >= 3 && args[2] != Py_None && !as_u32(args[2], &target)) || !read_options(args, n, 5, &opt)) return NULL;
    fsim_py_params p;
    if (fsim_py_params_read(n >= 4 ? args[3] : NULL, &p) < 0) {
        fsim_py_params_free(&p);
        return NULL;
    }
    fsim_position_command* points = NULL;
    uint32_t point_count = 0;
    if (n >= 5 && args[4] != Py_None && read_points(args[4], &points, &point_count) < 0) {
        fsim_py_params_free(&p);
        return NULL;
    }
    fsim_behavior_command c;
    memset(&c, 0, sizeof c);
    c.id = bid;
    c.target = target;
    c.param_names = p.names;
    c.param_values = p.values;
    c.param_count = p.count;
    c.points = points;
    c.point_count = point_count;
    const int rc = fsim_vehicle_submit_behavior(self->world, id, &c, &opt, &r);
    PyMem_Free(points);
    fsim_py_params_free(&p);
    if (rc != FSIM_OK) return fail();
    return result_tuple(self->world, &r);
}

/* submit_mode(id, mode, values, source=None, axes=None, range=None, min_version=None) -> result */
PyObject* world_submit_mode(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    int mode;
    double row[FSIM_PY_VALUES];
    fsim_command_options opt;
    fsim_command_result r;
    if (!check_args(n, 3, 8, "submit_mode") || !as_u32(args[0], &id) || !as_int(args[1], &mode) || !WORLD_IDLE(self)) return NULL;
    const Py_ssize_t count = read_values(args[2], row, "submit_mode");
    if (count < 0 || !read_options(args, n, 3, &opt)) return NULL;
    if (fsim_vehicle_submit_mode(self->world, id, mode, row, (uint32_t)count, &opt, &r) != FSIM_OK) return fail();
    return result_tuple(self->world, &r);
}

/* A route's waypoints: rows of (latitude_rad, longitude_rad, altitude_m, altitude_reference, speed,
 * speed_reference, turn, max_bank_rad, climb_rate_ms, id) and, from ABI 1.26, (altitude_min_m, altitude_max_m,
 * kind, waypoint_type, frame, frame_rotation, frame_offsets, frame_x_m, frame_y_m, frame_z_m) - NaN leaves one
 * out. *out is PyMem-allocated (free it); the count, or -1 with an error set. */
static Py_ssize_t read_waypoints(PyObject* o, fsim_waypoint** out) {
    *out = NULL;
    PyObject* seq = PySequence_Fast(o, "waypoints must be a sequence of 10-, 20-, 22-, 25-, 27-, 28-, 29- or 30-number rows");
    if (!seq) return -1;
    const Py_ssize_t count = PySequence_Size(seq);
    fsim_waypoint* points = (fsim_waypoint*)PyMem_Malloc(sizeof(fsim_waypoint) * (size_t)(count ? count : 1));
    for (Py_ssize_t i = 0; points && i < count; ++i) {
        PyObject* row = PySequence_GetItem(seq, i);
        PyObject* r = row ? PySequence_Fast(row, "each waypoint must be 10, 20, 22, 25, 27, 28, 29 or 30 numbers") : NULL;
        fsim_waypoint* w = &points[i];
        fsim_waypoint_init(w);
        const Py_ssize_t size = r ? PySequence_Size(r) : 0;
        if (r && (size == 10 || size == 20 || size == 22 || size == 25 || size == 27 || size == 28 || size == 29 || size == 30)) {
            double v[29];
            for (int k = 0; k < 29; ++k) v[k] = fsim_hold();
            for (Py_ssize_t k = 0; k < size && !PyErr_Occurred(); ++k) {
                if (k == 9) continue; /* (its id, below) */
                PyObject* item = PySequence_GetItem(r, k);
                v[k < 9 ? k : k - 1] = item ? PyFloat_AsDouble(item) : 0.0;
                Py_XDECREF(item);
            }
            PyObject* id = PySequence_GetItem(r, 9);
            if (id && !PyErr_Occurred()) {
                w->id = (uint64_t)PyLong_AsUnsignedLongLong(id);
                if (PyErr_Occurred()) PyErr_SetString(PyExc_ValueError, "a waypoint's id must be a whole number from 0");
            }
            Py_XDECREF(id);
            w->latitude_rad = v[0], w->longitude_rad = v[1], w->altitude_m = v[2], w->altitude_reference = v[3];
            w->speed = v[4], w->speed_reference = v[5], w->turn = v[6], w->max_bank_rad = v[7], w->climb_rate_ms = v[8];
            w->altitude_min_m = v[9], w->altitude_max_m = v[10], w->kind = v[11], w->waypoint_type = v[12], w->frame = v[13];
            w->frame_rotation = v[14], w->frame_offsets = v[15], w->frame_x_m = v[16], w->frame_y_m = v[17], w->frame_z_m = v[18];
            w->course_rad = v[19], w->turn_radius_m = v[20];
            w->speed_optimization = v[21], w->climb_optimization = v[22], w->acceleration_ms2 = v[23];
            w->arrival_begin_s = v[24], w->arrival_end_s = v[25];
            w->rnp_m = v[26], w->next = v[27], w->terminator = v[28];
        } else if (r) {
            PyErr_SetString(PyExc_ValueError, "each waypoint must be (latitude_rad, longitude_rad, altitude_m, altitude_reference, speed, "
                                              "speed_reference, turn, max_bank_rad, climb_rate_ms, id), and from ABI 1.26 its A-GRA fields");
        }
        Py_XDECREF(r);
        Py_XDECREF(row);
        if (PyErr_Occurred()) break;
    }
    Py_DECREF(seq);
    if (!points) PyErr_NoMemory();
    if (PyErr_Occurred()) {
        PyMem_Free(points);
        return -1;
    }
    *out = points;
    return count;
}

/* A route's loiters (ABI 1.28): rows of 37 numbers - point, the pattern's 35 fields in FSIM_MODE_PATTERN's order,
 * end_time_s - NaN leaves one out. *out is PyMem-allocated (free it); the count, or -1 with an error set. */
static Py_ssize_t read_loiters(PyObject* o, fsim_route_loiter** out) {
    *out = NULL;
    PyObject* seq = PySequence_Fast(o, "loiters must be a sequence of 37-number rows");
    if (!seq) return -1;
    const Py_ssize_t count = PySequence_Size(seq);
    fsim_route_loiter* loiters = (fsim_route_loiter*)PyMem_Malloc(sizeof(fsim_route_loiter) * (size_t)(count ? count : 1));
    for (Py_ssize_t i = 0; loiters && i < count; ++i) {
        PyObject* row = PySequence_GetItem(seq, i);
        PyObject* r = row ? PySequence_Fast(row, "each loiter must be 37 numbers") : NULL;
        fsim_route_loiter* l = &loiters[i];
        fsim_route_loiter_init(l);
        if (r && PySequence_Size(r) == 37) {
            PyObject* point = PySequence_GetItem(r, 0);
            if (point) {
                const unsigned long v = PyLong_AsUnsignedLong(point);
                if (PyErr_Occurred() || v > 0xFFFFFFFFul) PyErr_SetString(PyExc_ValueError, "a loiter's point must be a whole number from 0");
                else l->point = (uint32_t)v;
            }
            Py_XDECREF(point);
            for (Py_ssize_t k = 1; k < 37 && !PyErr_Occurred(); ++k) {
                PyObject* item = PySequence_GetItem(r, k);
                const double v = item ? PyFloat_AsDouble(item) : 0.0;
                Py_XDECREF(item);
                if (k < 36) l->fields[k - 1] = v;
                else l->end_time_s = v;
            }
        } else if (r) {
            PyErr_SetString(PyExc_ValueError, "each loiter must be (point, the pattern's 35 fields, end_time_s)");
        }
        Py_XDECREF(r);
        Py_XDECREF(row);
        if (PyErr_Occurred()) break;
    }
    Py_DECREF(seq);
    if (!loiters) PyErr_NoMemory();
    if (PyErr_Occurred()) {
        PyMem_Free(loiters);
        return -1;
    }
    *out = loiters;
    return count;
}

/* A loiter as read_loiters's row. */
static PyObject* loiter_row(const fsim_route_loiter* l) {
    PyObject* row = PyTuple_New(37);
    if (!row) return NULL;
    PyObject* point = PyLong_FromUnsignedLong(l->point);
    if (!point || PyTuple_SetItem(row, 0, point) < 0) {
        Py_DECREF(row);
        return NULL;
    }
    for (Py_ssize_t k = 1; k < 37; ++k) {
        PyObject* v = PyFloat_FromDouble(k < 36 ? l->fields[k - 1] : l->end_time_s);
        if (!v || PyTuple_SetItem(row, k, v) < 0) {
            Py_DECREF(row);
            return NULL;
        }
    }
    return row;
}

/* A route's planned states (ABI 1.31): rows of 30 numbers - point, then fsim_route_state's 29 fields in order - NaN leaves
 * one out. *out is PyMem-allocated (free it); the count, or -1 with an error set. */
static Py_ssize_t read_states(PyObject* o, fsim_route_state** out) {
    *out = NULL;
    PyObject* seq = PySequence_Fast(o, "states must be a sequence of 30-number rows");
    if (!seq) return -1;
    const Py_ssize_t count = PySequence_Size(seq);
    fsim_route_state* states = (fsim_route_state*)PyMem_Malloc(sizeof(fsim_route_state) * (size_t)(count ? count : 1));
    for (Py_ssize_t i = 0; states && i < count; ++i) {
        PyObject* row = PySequence_GetItem(seq, i);
        PyObject* r = row ? PySequence_Fast(row, "each state must be 30 numbers") : NULL;
        fsim_route_state* s = &states[i];
        fsim_route_state_init(s);
        if (r && PySequence_Size(r) == 30) {
            PyObject* point = PySequence_GetItem(r, 0);
            if (point) {
                const unsigned long v = PyLong_AsUnsignedLong(point);
                if (PyErr_Occurred() || v > 0xFFFFFFFFul) PyErr_SetString(PyExc_ValueError, "a state's point must be a whole number from 0");
                else s->point = (uint32_t)v;
            }
            Py_XDECREF(point);
            for (Py_ssize_t k = 1; k < 30 && !PyErr_Occurred(); ++k) {
                PyObject* item = PySequence_GetItem(r, k);
                s->fields[k - 1] = item ? PyFloat_AsDouble(item) : 0.0;
                Py_XDECREF(item);
            }
        } else if (r) {
            PyErr_SetString(PyExc_ValueError, "each state must be (point, fsim_route_state's 29 fields)");
        }
        Py_XDECREF(r);
        Py_XDECREF(row);
        if (PyErr_Occurred()) break;
    }
    Py_DECREF(seq);
    if (!states) PyErr_NoMemory();
    if (PyErr_Occurred()) {
        PyMem_Free(states);
        return -1;
    }
    *out = states;
    return count;
}

/* A state as read_states's row. */
static PyObject* state_row(const fsim_route_state* s) {
    PyObject* row = PyTuple_New(30);
    if (!row) return NULL;
    PyObject* point = PyLong_FromUnsignedLong(s->point);
    if (!point || PyTuple_SetItem(row, 0, point) < 0) {
        Py_DECREF(row);
        return NULL;
    }
    for (Py_ssize_t k = 1; k < 30; ++k) {
        PyObject* v = PyFloat_FromDouble(s->fields[k - 1]);
        if (!v || PyTuple_SetItem(row, k, v) < 0) {
            Py_DECREF(row);
            return NULL;
        }
    }
    return row;
}

/* A route's paths (ABI 1.33): rows of 4 numbers - id, type (NaN: primary), first, count. *out is PyMem-allocated (free it);
 * the count, or -1 with an error set. */
static Py_ssize_t read_paths(PyObject* o, fsim_route_path** out) {
    *out = NULL;
    PyObject* seq = PySequence_Fast(o, "paths must be a sequence of 4-number rows");
    if (!seq) return -1;
    const Py_ssize_t count = PySequence_Size(seq);
    fsim_route_path* paths = (fsim_route_path*)PyMem_Malloc(sizeof(fsim_route_path) * (size_t)(count ? count : 1));
    for (Py_ssize_t i = 0; paths && i < count; ++i) {
        PyObject* row = PySequence_GetItem(seq, i);
        PyObject* r = row ? PySequence_Fast(row, "each path must be (id, type, first, count)") : NULL;
        fsim_route_path* p = &paths[i];
        fsim_route_path_init(p);
        if (r && PySequence_Size(r) == 4) {
            PyObject* id = PySequence_GetItem(r, 0);
            PyObject* type = id ? PySequence_GetItem(r, 1) : NULL;
            PyObject* first = type ? PySequence_GetItem(r, 2) : NULL;
            PyObject* n = first ? PySequence_GetItem(r, 3) : NULL;
            if (n) {
                p->id = (uint64_t)PyLong_AsUnsignedLongLong(id);
                p->type = PyFloat_AsDouble(type);
                const unsigned long f = PyLong_AsUnsignedLong(first), c = PyLong_AsUnsignedLong(n);
                if (!PyErr_Occurred() && (f > 0xFFFFFFFFul || c > 0xFFFFFFFFul)) PyErr_SetString(PyExc_ValueError, "a path's first and count must fit");
                p->first = (uint32_t)f, p->count = (uint32_t)c;
            }
            Py_XDECREF(id), Py_XDECREF(type), Py_XDECREF(first), Py_XDECREF(n);
        } else if (r) {
            PyErr_SetString(PyExc_ValueError, "each path must be (id, type, first, count)");
        }
        Py_XDECREF(r);
        Py_XDECREF(row);
        if (PyErr_Occurred()) break;
    }
    Py_DECREF(seq);
    if (!paths) PyErr_NoMemory();
    if (PyErr_Occurred()) {
        PyMem_Free(paths);
        return -1;
    }
    *out = paths;
    return count;
}

/* A route's conditional branches (ABI 1.34): rows of 16 numbers - point, then fsim_route_branch's 15 fields. *out is
 * PyMem-allocated (free it); the count, or -1 with an error set. */
static Py_ssize_t read_branches(PyObject* o, fsim_route_branch** out) {
    *out = NULL;
    PyObject* seq = PySequence_Fast(o, "branches must be a sequence of 16-number rows");
    if (!seq) return -1;
    const Py_ssize_t count = PySequence_Size(seq);
    fsim_route_branch* branches = (fsim_route_branch*)PyMem_Malloc(sizeof(fsim_route_branch) * (size_t)(count ? count : 1));
    for (Py_ssize_t i = 0; branches && i < count; ++i) {
        PyObject* row = PySequence_GetItem(seq, i);
        PyObject* r = row ? PySequence_Fast(row, "each branch must be 16 numbers") : NULL;
        fsim_route_branch* b = &branches[i];
        fsim_route_branch_init(b);
        if (r && PySequence_Size(r) == 16) {
            PyObject* point = PySequence_GetItem(r, 0);
            if (point) {
                const unsigned long v = PyLong_AsUnsignedLong(point);
                if (PyErr_Occurred() || v > 0xFFFFFFFFul) PyErr_SetString(PyExc_ValueError, "a branch's point must be a whole number from 0");
                else b->point = (uint32_t)v;
            }
            Py_XDECREF(point);
            for (Py_ssize_t k = 1; k < 16 && !PyErr_Occurred(); ++k) {
                PyObject* item = PySequence_GetItem(r, k);
                b->fields[k - 1] = item ? PyFloat_AsDouble(item) : 0.0;
                Py_XDECREF(item);
            }
        } else if (r) {
            PyErr_SetString(PyExc_ValueError, "each branch must be (point, fsim_route_branch's 15 fields)");
        }
        Py_XDECREF(r);
        Py_XDECREF(row);
        if (PyErr_Occurred()) break;
    }
    Py_DECREF(seq);
    if (!branches) PyErr_NoMemory();
    if (PyErr_Occurred()) {
        PyMem_Free(branches);
        return -1;
    }
    *out = branches;
    return count;
}

/* A branch as read_branches's row. */
static PyObject* branch_row(const fsim_route_branch* b) {
    PyObject* row = PyTuple_New(16);
    if (!row) return NULL;
    PyObject* point = PyLong_FromUnsignedLong(b->point);
    if (!point || PyTuple_SetItem(row, 0, point) < 0) {
        Py_DECREF(row);
        return NULL;
    }
    for (Py_ssize_t k = 1; k < 16; ++k) {
        PyObject* v = PyFloat_FromDouble(b->fields[k - 1]);
        if (!v || PyTuple_SetItem(row, k, v) < 0) {
            Py_DECREF(row);
            return NULL;
        }
    }
    return row;
}

/* A route's civil path terminators' data (ABI 1.35): rows of 14 numbers - point, then fsim_route_terminator's 13 fields.
 * *out is PyMem-allocated (free it); the count, or -1 with an error set. */
static Py_ssize_t read_terminators(PyObject* o, fsim_route_terminator** out) {
    *out = NULL;
    PyObject* seq = PySequence_Fast(o, "terminators must be a sequence of 14-number rows");
    if (!seq) return -1;
    const Py_ssize_t count = PySequence_Size(seq);
    fsim_route_terminator* terminators = (fsim_route_terminator*)PyMem_Malloc(sizeof(fsim_route_terminator) * (size_t)(count ? count : 1));
    for (Py_ssize_t i = 0; terminators && i < count; ++i) {
        PyObject* row = PySequence_GetItem(seq, i);
        PyObject* r = row ? PySequence_Fast(row, "each terminator must be 14 numbers") : NULL;
        fsim_route_terminator* t = &terminators[i];
        fsim_route_terminator_init(t);
        if (r && PySequence_Size(r) == 14) {
            PyObject* point = PySequence_GetItem(r, 0);
            if (point) {
                const unsigned long v = PyLong_AsUnsignedLong(point);
                if (PyErr_Occurred() || v > 0xFFFFFFFFul) PyErr_SetString(PyExc_ValueError, "a terminator's point must be a whole number from 0");
                else t->point = (uint32_t)v;
            }
            Py_XDECREF(point);
            for (Py_ssize_t k = 1; k < 14 && !PyErr_Occurred(); ++k) {
                PyObject* item = PySequence_GetItem(r, k);
                t->fields[k - 1] = item ? PyFloat_AsDouble(item) : 0.0;
                Py_XDECREF(item);
            }
        } else if (r) {
            PyErr_SetString(PyExc_ValueError, "each terminator must be (point, fsim_route_terminator's 13 fields)");
        }
        Py_XDECREF(r);
        Py_XDECREF(row);
        if (PyErr_Occurred()) break;
    }
    Py_DECREF(seq);
    if (!terminators) PyErr_NoMemory();
    if (PyErr_Occurred()) {
        PyMem_Free(terminators);
        return -1;
    }
    *out = terminators;
    return count;
}

/* A terminator as read_terminators's row. */
static PyObject* terminator_row(const fsim_route_terminator* t) {
    PyObject* row = PyTuple_New(14);
    if (!row) return NULL;
    PyObject* point = PyLong_FromUnsignedLong(t->point);
    if (!point || PyTuple_SetItem(row, 0, point) < 0) {
        Py_DECREF(row);
        return NULL;
    }
    for (Py_ssize_t k = 1; k < 14; ++k) {
        PyObject* v = PyFloat_FromDouble(t->fields[k - 1]);
        if (!v || PyTuple_SetItem(row, k, v) < 0) {
            Py_DECREF(row);
            return NULL;
        }
    }
    return row;
}

/* submit_route(id, values, waypoints, source=None, axes=None, range=None, min_version=None, envelope=None, loiters=None,
 * states=None, paths=None, branches=None, terminators=None) -> result; loiters (ABI 1.28): read_loiters's rows; states (ABI
 * 1.31): read_states's; paths (ABI 1.33): read_paths's; branches (ABI 1.34): read_branches's; terminators (ABI 1.35):
 * read_terminators's */
PyObject* world_submit_route(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    double row[FSIM_PY_VALUES];
    fsim_command_options opt;
    fsim_command_result r;
    fsim_waypoint* points = NULL;
    fsim_route_loiter* loiters = NULL;
    fsim_route_state* states = NULL;
    fsim_route_path* paths = NULL;
    fsim_route_branch* branches = NULL;
    fsim_route_terminator* terminators = NULL;
    if (!check_args(n, 3, 13, "submit_route") || !as_u32(args[0], &id) || !WORLD_IDLE(self)) return NULL;
    const Py_ssize_t count = read_values(args[1], row, "submit_route");
    if (count < 0 || !read_options(args, n > 8 ? 8 : n, 3, &opt)) return NULL;
    const Py_ssize_t np = read_waypoints(args[2], &points);
    if (np < 0) return NULL;
    const Py_ssize_t nl = n > 8 && args[8] != Py_None ? read_loiters(args[8], &loiters) : 0;
    const Py_ssize_t ns = nl >= 0 && n > 9 && args[9] != Py_None ? read_states(args[9], &states) : 0;
    const Py_ssize_t nr = nl >= 0 && ns >= 0 && n > 10 && args[10] != Py_None ? read_paths(args[10], &paths) : 0;
    const Py_ssize_t nb = nl >= 0 && ns >= 0 && nr >= 0 && n > 11 && args[11] != Py_None ? read_branches(args[11], &branches) : 0;
    const Py_ssize_t nt = nl >= 0 && ns >= 0 && nr >= 0 && nb >= 0 && n > 12 && args[12] != Py_None ? read_terminators(args[12], &terminators) : 0;
    if (nl < 0 || ns < 0 || nr < 0 || nb < 0 || nt < 0) {
        PyMem_Free(points);
        PyMem_Free(loiters);
        PyMem_Free(states);
        PyMem_Free(paths);
        PyMem_Free(branches);
        return NULL;
    }
    fsim_route_extras extras;
    fsim_route_extras_init(&extras);
    extras.loiters = loiters, extras.loiter_count = (uint32_t)nl, extras.states = states, extras.state_count = (uint32_t)ns;
    extras.paths = paths, extras.path_count = (uint32_t)nr, extras.branches = branches, extras.branch_count = (uint32_t)nb;
    extras.terminators = terminators, extras.terminator_count = (uint32_t)nt;
    const int rc = fsim_vehicle_submit_route_extras(self->world, id, row, (uint32_t)count, points, (uint32_t)np, &extras, &opt, &r);
    PyMem_Free(points);
    PyMem_Free(loiters);
    PyMem_Free(states);
    PyMem_Free(paths);
    PyMem_Free(branches);
    PyMem_Free(terminators);
    if (rc != FSIM_OK) return fail();
    return result_tuple(self->world, &r);
}

/* activity_update_route(activity, values, waypoints, source=0, controller=0, loiters=None, states=None, paths=None,
 * branches=None, terminators=None) -> result; waypoints may be empty: the route's own, and their loiters, states, paths,
 * branches and terminators; given, with `loiters` (ABI 1.28: read_loiters's rows), `states` (ABI 1.31: read_states's),
 * `paths` (ABI 1.33: read_paths's), `branches` (ABI 1.34: read_branches's) and `terminators` (ABI 1.35:
 * read_terminators's) */
PyObject* world_activity_update_route(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint64_t activity;
    double row[FSIM_PY_VALUES];
    fsim_command_result r;
    fsim_waypoint* points = NULL;
    fsim_route_loiter* loiters = NULL;
    fsim_route_state* states = NULL;
    fsim_route_path* paths = NULL;
    fsim_route_branch* branches = NULL;
    fsim_route_terminator* terminators = NULL;
    int source = 0;
    uint32_t controller = 0;
    if (!check_args(n, 3, 10, "activity_update_route") || !as_u64(args[0], &activity) || (n > 3 && !as_int(args[3], &source)) ||
        (n > 4 && !as_u32(args[4], &controller)) || !WORLD_IDLE(self))
        return NULL;
    const Py_ssize_t count = read_values(args[1], row, "activity_update_route");
    if (count < 0) return NULL;
    const Py_ssize_t np = read_waypoints(args[2], &points);
    if (np < 0) return NULL;
    const Py_ssize_t nl = n > 5 && args[5] != Py_None ? read_loiters(args[5], &loiters) : 0;
    const Py_ssize_t ns = nl >= 0 && n > 6 && args[6] != Py_None ? read_states(args[6], &states) : 0;
    const Py_ssize_t nr = nl >= 0 && ns >= 0 && n > 7 && args[7] != Py_None ? read_paths(args[7], &paths) : 0;
    const Py_ssize_t nb = nl >= 0 && ns >= 0 && nr >= 0 && n > 8 && args[8] != Py_None ? read_branches(args[8], &branches) : 0;
    const Py_ssize_t nt = nl >= 0 && ns >= 0 && nr >= 0 && nb >= 0 && n > 9 && args[9] != Py_None ? read_terminators(args[9], &terminators) : 0;
    if (nl < 0 || ns < 0 || nr < 0 || nb < 0 || nt < 0) {
        PyMem_Free(points);
        PyMem_Free(loiters);
        PyMem_Free(states);
        PyMem_Free(paths);
        PyMem_Free(branches);
        return NULL;
    }
    fsim_route_extras extras;
    fsim_route_extras_init(&extras);
    extras.loiters = loiters, extras.loiter_count = (uint32_t)nl, extras.states = states, extras.state_count = (uint32_t)ns;
    extras.paths = paths, extras.path_count = (uint32_t)nr, extras.branches = branches, extras.branch_count = (uint32_t)nb;
    extras.terminators = terminators, extras.terminator_count = (uint32_t)nt;
    const int rc = fsim_activity_update_route_extras(self->world, activity, source, controller, row, (uint32_t)count, points, (uint32_t)np, &extras, &r);
    PyMem_Free(points);
    PyMem_Free(loiters);
    PyMem_Free(states);
    PyMem_Free(paths);
    PyMem_Free(branches);
    PyMem_Free(terminators);
    if (rc != FSIM_OK) return fail();
    return result_tuple(self->world, &r);
}

/* A curve's segments: rows of 18 numbers, the control points north[6], east[6], down[6]. *out is
 * PyMem-allocated (free it); the count, or -1 with an error set. */
static Py_ssize_t read_segments(PyObject* o, fsim_bezier_segment** out) {
    *out = NULL;
    PyObject* seq = PySequence_Fast(o, "segments must be a sequence of 18-number rows");
    if (!seq) return -1;
    const Py_ssize_t count = PySequence_Size(seq);
    fsim_bezier_segment* segments = (fsim_bezier_segment*)PyMem_Malloc(sizeof(fsim_bezier_segment) * (size_t)(count ? count : 1));
    for (Py_ssize_t i = 0; segments && i < count; ++i) {
        PyObject* row = PySequence_GetItem(seq, i);
        PyObject* r = row ? PySequence_Fast(row, "each segment must be 18 numbers") : NULL;
        fsim_bezier_segment* s = &segments[i];
        fsim_bezier_segment_init(s);
        if (r && PySequence_Size(r) == 18) {
            for (int k = 0; k < 18 && !PyErr_Occurred(); ++k) {
                PyObject* item = PySequence_GetItem(r, k);
                const double v = item ? PyFloat_AsDouble(item) : 0.0;
                Py_XDECREF(item);
                if (k < 6) s->north[k] = v;
                else if (k < 12) s->east[k - 6] = v;
                else s->down[k - 12] = v;
            }
        } else if (r) {
            PyErr_SetString(PyExc_ValueError, "each segment must be its control points north[6], east[6], down[6]");
        }
        Py_XDECREF(r);
        Py_XDECREF(row);
        if (PyErr_Occurred()) break;
    }
    Py_DECREF(seq);
    if (!segments) PyErr_NoMemory();
    if (PyErr_Occurred()) {
        PyMem_Free(segments);
        return -1;
    }
    *out = segments;
    return count;
}

/* A curve's segments as A-GRA's schema gives them (ABI 1.24): rows of 59 numbers - points, knots, north[10], east[10],
 * down[10], weight[10], knot[14], curvature, first_index, last_index. *out is PyMem-allocated (free it); the count, or
 * -1 with an error set. */
static Py_ssize_t read_nurbs(PyObject* o, fsim_nurbs_segment** out) {
    *out = NULL;
    PyObject* seq = PySequence_Fast(o, "segments must be a sequence of 59-number rows");
    if (!seq) return -1;
    const Py_ssize_t count = PySequence_Size(seq);
    fsim_nurbs_segment* segments = (fsim_nurbs_segment*)PyMem_Malloc(sizeof(fsim_nurbs_segment) * (size_t)(count ? count : 1));
    for (Py_ssize_t i = 0; segments && i < count; ++i) {
        PyObject* row = PySequence_GetItem(seq, i);
        PyObject* r = row ? PySequence_Fast(row, "each segment must be 59 numbers") : NULL;
        fsim_nurbs_segment* s = &segments[i];
        fsim_nurbs_segment_init(s);
        if (r && PySequence_Size(r) == 59) {
            double v[59];
            for (int k = 0; k < 59 && !PyErr_Occurred(); ++k) {
                PyObject* item = PySequence_GetItem(r, k);
                v[k] = item ? PyFloat_AsDouble(item) : 0.0;
                Py_XDECREF(item);
            }
            if (!PyErr_Occurred()) {
                const double points = v[0], knots = v[1];
                if (!(points >= 0.0 && points <= 10.0 && knots >= 0.0 && knots <= 14.0))
                    PyErr_SetString(PyExc_ValueError, "a segment has at most 10 control points and 14 knots");
                s->points = (uint32_t)points, s->knots = (uint32_t)knots;
                for (int k = 0; k < 10; ++k) s->north[k] = v[2 + k], s->east[k] = v[12 + k], s->down[k] = v[22 + k], s->weight[k] = v[32 + k];
                for (int k = 0; k < 14; ++k) s->knot[k] = v[42 + k];
                s->curvature = v[56], s->first_index = v[57], s->last_index = v[58];
            }
        } else if (r) {
            PyErr_SetString(PyExc_ValueError, "each segment must be points, knots, north[10], east[10], down[10], weight[10], knot[14], curvature, "
                                              "first_index, last_index");
        }
        Py_XDECREF(r);
        Py_XDECREF(row);
        if (PyErr_Occurred()) break;
    }
    Py_DECREF(seq);
    if (!segments) PyErr_NoMemory();
    if (PyErr_Occurred()) {
        PyMem_Free(segments);
        return -1;
    }
    *out = segments;
    return count;
}

/* A segment as A-GRA's schema gives it, as the 59 numbers read_nurbs reads. */
static PyObject* nurbs_row(const fsim_nurbs_segment* s) {
    PyObject* row = PyTuple_New(59);
    double v[59];
    v[0] = s->points, v[1] = s->knots;
    for (int k = 0; k < 10; ++k) v[2 + k] = s->north[k], v[12 + k] = s->east[k], v[22 + k] = s->down[k], v[32 + k] = s->weight[k];
    for (int k = 0; k < 14; ++k) v[42 + k] = s->knot[k];
    v[56] = s->curvature, v[57] = s->first_index, v[58] = s->last_index;
    for (int k = 0; row && k < 59; ++k) {
        PyObject* x = PyFloat_FromDouble(v[k]);
        if (!x || PyTuple_SetItem(row, k, x) < 0) Py_CLEAR(row); /* (SetItem takes the item, even when it fails) */
    }
    return row;
}

/* submit_curve(id, values, segments, source=None, axes=None, range=None, min_version=None) -> result */
PyObject* world_submit_curve(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    double row[FSIM_PY_VALUES];
    fsim_command_options opt;
    fsim_command_result r;
    fsim_bezier_segment* segments = NULL;
    if (!check_args(n, 3, 8, "submit_curve") || !as_u32(args[0], &id) || !WORLD_IDLE(self)) return NULL;
    const Py_ssize_t count = read_values(args[1], row, "submit_curve");
    if (count < 0 || !read_options(args, n, 3, &opt)) return NULL;
    const Py_ssize_t ns = read_segments(args[2], &segments);
    if (ns < 0) return NULL;
    const int rc = fsim_vehicle_submit_curve(self->world, id, row, (uint32_t)count, segments, (uint32_t)ns, &opt, &r);
    PyMem_Free(segments);
    if (rc != FSIM_OK) return fail();
    return result_tuple(self->world, &r);
}

/* activity_update_curve(activity, values, segments) -> result; segments may be empty: how it is flown alone */
PyObject* world_activity_update_curve(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint64_t activity;
    double row[FSIM_PY_VALUES];
    fsim_command_result r;
    fsim_bezier_segment* segments = NULL;
    int source = 0;
    uint32_t controller = 0;
    if (!check_args(n, 3, 5, "activity_update_curve") || !as_u64(args[0], &activity) || (n > 3 && !as_int(args[3], &source)) ||
        (n > 4 && !as_u32(args[4], &controller)) || !WORLD_IDLE(self))
        return NULL;
    const Py_ssize_t count = read_values(args[1], row, "activity_update_curve");
    if (count < 0) return NULL;
    const Py_ssize_t ns = read_segments(args[2], &segments);
    if (ns < 0) return NULL;
    const int rc = fsim_activity_update_curve_by(self->world, activity, source, controller, row, (uint32_t)count, segments, (uint32_t)ns, &r);
    PyMem_Free(segments);
    if (rc != FSIM_OK) return fail();
    return result_tuple(self->world, &r);
}

/* submit_nurbs(id, values, segments, source=None, axes=None, range=None, min_version=None) -> result: segments as
 * A-GRA's schema gives them (read_nurbs's rows) */
PyObject* world_submit_nurbs(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    double row[FSIM_PY_VALUES];
    fsim_command_options opt;
    fsim_command_result r;
    fsim_nurbs_segment* segments = NULL;
    if (!check_args(n, 3, 8, "submit_nurbs") || !as_u32(args[0], &id) || !WORLD_IDLE(self)) return NULL;
    const Py_ssize_t count = read_values(args[1], row, "submit_nurbs");
    if (count < 0 || !read_options(args, n, 3, &opt)) return NULL;
    const Py_ssize_t ns = read_nurbs(args[2], &segments);
    if (ns < 0) return NULL;
    const int rc = fsim_vehicle_submit_nurbs(self->world, id, row, (uint32_t)count, segments, (uint32_t)ns, &opt, &r);
    PyMem_Free(segments);
    if (rc != FSIM_OK) return fail();
    return result_tuple(self->world, &r);
}

/* activity_update_nurbs(activity, values, segments, source=0, controller=0) -> result */
PyObject* world_activity_update_nurbs(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint64_t activity;
    double row[FSIM_PY_VALUES];
    fsim_command_result r;
    fsim_nurbs_segment* segments = NULL;
    int source = 0;
    uint32_t controller = 0;
    if (!check_args(n, 3, 5, "activity_update_nurbs") || !as_u64(args[0], &activity) || (n > 3 && !as_int(args[3], &source)) ||
        (n > 4 && !as_u32(args[4], &controller)) || !WORLD_IDLE(self))
        return NULL;
    const Py_ssize_t count = read_values(args[1], row, "activity_update_nurbs");
    if (count < 0) return NULL;
    const Py_ssize_t ns = read_nurbs(args[2], &segments);
    if (ns < 0) return NULL;
    const int rc = fsim_activity_update_nurbs_by(self->world, activity, source, controller, row, (uint32_t)count, segments, (uint32_t)ns, &r);
    PyMem_Free(segments);
    if (rc != FSIM_OK) return fail();
    return result_tuple(self->world, &r);
}

/* submit_support(id, kind, values, source=None, axes=None, range=None, min_version=None) -> result */
PyObject* world_submit_support(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    int kind;
    double row[FSIM_PY_VALUES];
    fsim_command_options opt;
    fsim_command_result r;
    if (!check_args(n, 3, 8, "submit_support") || !as_u32(args[0], &id) || !as_int(args[1], &kind) || !WORLD_IDLE(self)) return NULL;
    const Py_ssize_t count = read_values(args[2], row, "submit_support");
    if (count < 0 || !read_options(args, n, 3, &opt)) return NULL;
    if (fsim_vehicle_submit_support(self->world, id, kind, row, (uint32_t)count, &opt, &r) != FSIM_OK) return fail();
    return result_tuple(self->world, &r);
}

/* activity_update(activity, values, source=0, controller=0) -> result */
PyObject* world_activity_update(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint64_t activity;
    double row[FSIM_PY_VALUES];
    fsim_command_result r;
    int source = 0;
    uint32_t controller = 0;
    if (!check_args(n, 2, 4, "activity_update") || !as_u64(args[0], &activity) || (n > 2 && !as_int(args[2], &source)) ||
        (n > 3 && !as_u32(args[3], &controller)) || !WORLD_IDLE(self))
        return NULL;
    const Py_ssize_t count = read_values(args[1], row, "activity_update");
    if (count < 0) return NULL;
    if (fsim_activity_update_by(self->world, activity, source, controller, row, (uint32_t)count, &r) != FSIM_OK) return fail();
    return result_tuple(self->world, &r);
}

/* activity_update_behavior(activity, params, source=0, controller=0) -> result: a behaviour's updatable parameters (ABI 1.45) */
PyObject* world_activity_update_behavior(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint64_t activity;
    fsim_command_result r;
    int source = 0;
    uint32_t controller = 0;
    if (!check_args(n, 2, 4, "activity_update_behavior") || !as_u64(args[0], &activity) || (n > 2 && !as_int(args[2], &source)) ||
        (n > 3 && !as_u32(args[3], &controller)) || !WORLD_IDLE(self))
        return NULL;
    fsim_py_params p;
    if (fsim_py_params_read(args[1] == Py_None ? NULL : args[1], &p) < 0) {
        fsim_py_params_free(&p);
        return NULL;
    }
    fsim_behavior_command c;
    memset(&c, 0, sizeof c);
    c.param_names = p.names;
    c.param_values = p.values;
    c.param_count = p.count;
    const int rc = fsim_activity_update_behavior_by(self->world, activity, source, controller, &c, &r);
    fsim_py_params_free(&p);
    if (rc != FSIM_OK) return fail();
    return result_tuple(self->world, &r);
}

/* activity_update_batch(activities uint64[n], values float64[n][stride], stride[, fields]): with `fields`, each
 * row holds that many (a level's fsim_command_field_count or fsim_command_field_count_full) */
PyObject* world_activity_update_batch(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t stride, fields = 0;
    Py_buffer acts, values;
    if (!check_args(n, 3, 4, "activity_update_batch") || !as_u32(args[2], &stride) || (n > 3 && !as_u32(args[3], &fields)) ||
        !WORLD_IDLE(self))
        return NULL;
    if (!get_array(args[0], &acts, 0, "Q", 8, "activity_update_batch activities")) return NULL;
    if (!get_array(args[1], &values, 0, "d", 8, "activity_update_batch values")) {
        PyBuffer_Release(&acts);
        return NULL;
    }
    const Py_ssize_t count = acts.len / 8;
    if (stride == 0 || values.len != count * (Py_ssize_t)stride * 8) {
        PyErr_Format(PyExc_ValueError, "activity_update_batch: %zd activities at a stride of %u need %zd values, got %zd", count,
                     (unsigned)stride, count * (Py_ssize_t)stride, values.len / 8);
        PyBuffer_Release(&acts);
        PyBuffer_Release(&values);
        return NULL;
    }
    const int rc = fields ? fsim_activity_update_batch_n(self->world, (const fsim_activity_id*)acts.buf, (uint32_t)count,
                                                         (const double*)values.buf, stride, fields)
                          : fsim_activity_update_batch(self->world, (const fsim_activity_id*)acts.buf, (uint32_t)count,
                                                       (const double*)values.buf, stride);
    PyBuffer_Release(&acts);
    PyBuffer_Release(&values);
    if (rc != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

/* activity_cancel(activity, source=0, controller=0) -> result */
PyObject* world_activity_cancel(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint64_t activity;
    fsim_command_result r;
    int source = 0;
    uint32_t controller = 0;
    if (!check_args(n, 1, 3, "activity_cancel") || !as_u64(args[0], &activity) || (n > 1 && !as_int(args[1], &source)) ||
        (n > 2 && !as_u32(args[2], &controller)) || !WORLD_IDLE(self))
        return NULL;
    if (fsim_activity_cancel_by(self->world, activity, source, controller, &r) != FSIM_OK) return fail();
    return result_tuple(self->world, &r);
}

/* activity_command(activity, command, priority, precedence, source=0, controller=0) -> result */
PyObject* world_activity_command(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint64_t activity;
    uint32_t priority = 0, precedence = 0, controller = 0;
    fsim_command_result r;
    int command = 0, source = 0;
    if (!check_args(n, 4, 6, "activity_command") || !as_u64(args[0], &activity) || !as_int(args[1], &command) || !as_u32(args[2], &priority) ||
        !as_u32(args[3], &precedence) || (n > 4 && !as_int(args[4], &source)) || (n > 5 && !as_u32(args[5], &controller)) || !WORLD_IDLE(self))
        return NULL;
    if (fsim_activity_command(self->world, activity, command, priority, precedence, source, controller, &r) != FSIM_OK) return fail();
    return result_tuple(self->world, &r);
}

/* activity_command_branch(activity, branch, commanded=1, source=0, controller=0) -> result: the operator's input to a
 * route's conditional branch (ABI 1.34) */
PyObject* world_activity_command_branch(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint64_t activity;
    uint32_t branch = 0, controller = 0;
    int commanded = 1, source = 0;
    fsim_command_result r;
    if (!check_args(n, 2, 5, "activity_command_branch") || !as_u64(args[0], &activity) || !as_u32(args[1], &branch) ||
        (n > 2 && !as_int(args[2], &commanded)) || (n > 3 && !as_int(args[3], &source)) || (n > 4 && !as_u32(args[4], &controller)) ||
        !WORLD_IDLE(self))
        return NULL;
    if (fsim_activity_command_branch(self->world, activity, source, controller, branch, commanded, &r) != FSIM_OK) return fail();
    return result_tuple(self->world, &r);
}

/* activity_info(activity) -> info tuple, or None if the vehicle does not remember it */
PyObject* world_activity_info(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint64_t activity;
    fsim_activity_info a;
    if (!check_args(n, 1, 1, "activity_info") || !as_u64(args[0], &activity)) return NULL;
    if (fsim_activity_get(self->world, activity, &a) != FSIM_OK) Py_RETURN_NONE;
    return info_tuple(self->world, &a);
}

/* vehicle_activities(id) -> [info tuple]: live, then ended (newest first) */
PyObject* world_vehicle_activities(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    fsim_activity_info a;
    if (!check_args(n, 1, 1, "vehicle_activities") || !as_u32(args[0], &id)) return NULL;
    const uint32_t count = fsim_vehicle_activity_count(self->world, id);
    PyObject* list = PyList_New(0);
    for (uint32_t i = 0; list && i < count; ++i) {
        if (fsim_vehicle_activity(self->world, id, i, &a) != FSIM_OK) continue;
        PyObject* t = info_tuple(self->world, &a);
        if (!t || PyList_Append(list, t) < 0) {
            Py_XDECREF(t);
            Py_DECREF(list);
            return NULL;
        }
        Py_DECREF(t);
    }
    return list;
}

/* capabilities(id) -> [(id, version, kind, interactions, level, axes, terminating, needs_target, behavior,
 *                       [(name, unit, min, max, default, optional, supported)], axis_groups, flight_mode, accepted,
 *                       superseded)] */
PyObject* world_capabilities(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    fsim_capability_info c;
    fsim_parameter_info p;
    if (!check_args(n, 1, 1, "capabilities") || !as_u32(args[0], &id)) return NULL;
    const uint32_t count = fsim_vehicle_capability_count(self->world, id);
    PyObject* list = PyList_New(0);
    for (uint32_t i = 0; list && i < count; ++i) {
        if (fsim_vehicle_capability(self->world, id, i, &c) != FSIM_OK) continue;
        PyObject* params = PyList_New(0);
        for (uint32_t k = 0; params && k < c.parameter_count; ++k) {
            if (fsim_vehicle_capability_parameter(self->world, id, i, k, &p) != FSIM_OK) continue;
            PyObject* t = Py_BuildValue("(ssdddOO)", p.name, p.unit, p.min, p.max, p.default_value, p.optional ? Py_True : Py_False,
                                        p.unsupported ? Py_False : Py_True);
            if (!t || PyList_Append(params, t) < 0) {
                Py_XDECREF(t);
                Py_CLEAR(params);
                break;
            }
            Py_DECREF(t);
        }
        const char* superseded = fsim_vehicle_capability_superseded(self->world, id, i);
        PyObject* t = params ? Py_BuildValue("(sIiIiIOOsNIiis)", c.id, c.version, c.kind, c.interactions, c.level, c.axes,
                                             c.terminating ? Py_True : Py_False, c.needs_target ? Py_True : Py_False, c.behavior, params,
                                             c.axis_groups, fsim_vehicle_capability_flight_mode(self->world, id, i),
                                             fsim_vehicle_capability_accepted(self->world, id, i), superseded ? superseded : "")
                             : NULL;
        if (!t || PyList_Append(list, t) < 0) {
            Py_XDECREF(t);
            Py_DECREF(list);
            return NULL;
        }
        Py_DECREF(t);
    }
    return list;
}

/* activity_progress(activity) -> (segment, segments, laps, segment_id, percent, segment_percent, distance_to_go_m,
 * time_to_go_s, cross_track_m, course_rad, heading_rad, altitude_msl_m, speed_ms, speed_reference, arrival_s,
 * arrival_delta_s), or None */
PyObject* world_activity_progress(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint64_t activity;
    fsim_activity_progress p;
    if (!check_args(n, 1, 1, "activity_progress") || !as_u64(args[0], &activity)) return NULL;
    fsim_activity_progress_init(&p);
    if (fsim_activity_get_progress(self->world, activity, &p) != FSIM_OK) Py_RETURN_NONE;
    return Py_BuildValue("(IIIKdddddddddddd)", p.segment, p.segments, p.laps, (unsigned long long)p.segment_id, p.percent, p.segment_percent,
                         p.distance_to_go_m, p.time_to_go_s, p.cross_track_m, p.course_rad, p.heading_rad, p.altitude_msl_m, p.speed_ms,
                         p.speed_reference, p.arrival_s, p.arrival_delta_s);
}

/* commanded(id) -> (top_level, latitude_rad, longitude_rad, altitude_msl_m, heading_rad, turn_rate_rad_s, airspeed_ms,
 * vertical_speed_ms, north_ms, east_ms, roll_rad, pitch_rad, load_factor_g, roll_rate_rad_s, pitch_rate_rad_s,
 * yaw_rate_rad_s, throttle, north_acceleration_ms2, east_acceleration_ms2, down_acceleration_ms2, altitude_m,
 * altitude_reference) */
PyObject* world_commanded(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    fsim_commanded_state c;
    if (!check_args(n, 1, 1, "commanded") || !as_u32(args[0], &id)) return NULL;
    fsim_commanded_state_init(&c);
    if (fsim_vehicle_commanded(self->world, id, &c) != FSIM_OK) return fail();
    return Py_BuildValue("(iddddddddddddddddddddd)", c.top_level, c.latitude_rad, c.longitude_rad, c.altitude_msl_m, c.heading_rad,
                         c.turn_rate_rad_s, c.airspeed_ms, c.vertical_speed_ms, c.north_ms, c.east_ms, c.roll_rad, c.pitch_rad,
                         c.load_factor_g, c.roll_rate_rad_s, c.pitch_rate_rad_s, c.yaw_rate_rad_s, c.throttle, c.north_acceleration_ms2,
                         c.east_acceleration_ms2, c.down_acceleration_ms2, c.altitude_m, c.altitude_reference);
}

/* navigation_report(id) -> (energy, fuel_kg, remaining, capacity, percent, consumption, endurance_s, reserve, playtime_s,
 * return_distance_m, return_tas_ms, return_consumption, contingency, starved) */
PyObject* world_navigation_report(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    fsim_navigation_report r;
    if (!check_args(n, 1, 1, "navigation_report") || !as_u32(args[0], &id)) return NULL;
    fsim_navigation_report_init(&r);
    if (fsim_vehicle_navigation_report(self->world, id, &r) != FSIM_OK) return fail();
    return Py_BuildValue("(idddddddddddii)", r.energy, r.fuel_kg, r.remaining, r.capacity, r.percent, r.consumption, r.endurance_s,
                         r.reserve, r.playtime_s, r.return_distance_m, r.return_tas_ms, r.return_consumption, r.contingency, r.starved);
}

/* `count` records of `fields` doubles each, as a list of tuples */
static PyObject* double_records(const void* data, uint32_t count, int fields) {
    const double* d = (const double*)data;
    PyObject* list = PyList_New((Py_ssize_t)count);
    if (!list) return NULL;
    for (uint32_t i = 0; i < count; ++i) {
        PyObject* t = PyTuple_New(fields);
        if (!t) {
            Py_DECREF(list);
            return NULL;
        }
        for (int f = 0; f < fields; ++f) {
            PyObject* x = PyFloat_FromDouble(d[(size_t)i * (size_t)fields + (size_t)f]);
            if (!x || PyTuple_SetItem(t, f, x) < 0) { /* (it takes x, set or not) */
                Py_DECREF(t);
                Py_DECREF(list);
                return NULL;
            }
        }
        if (PyList_SetItem(list, (Py_ssize_t)i, t) < 0) { /* (it takes t, set or not) */
            Py_DECREF(list);
            return NULL;
        }
    }
    return list;
}

/* performance_profile(id, mode) -> (reason, None) where it has none, else (0, (mode, energy, clean, flaps_out,
 * gear_down, time_s, altitude_msl_m, weight_kg, tas_ms, min_altitude_msl_m, max_altitude_msl_m, max_turn_rate_rad_s,
 * max_climb_rate_ms, min_airspeed, max_airspeed, best_endurance_airspeed, best_range_airspeed, min_acceleration,
 * max_acceleration, max_deceleration, excess_power, max_descent_rate, burn, max_orientation, max_orientation_rate)),
 * each of the last twelve a list of tuples */
PyObject* world_performance_profile(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    int32_t why = 0;
    fsim_performance_profile p;
    if (!check_args(n, 2, 2, "performance_profile") || !as_u32(args[0], &id) || !WORLD_IDLE(self)) return NULL;
    const long mode = PyLong_AsLong(args[1]);
    if (mode == -1 && PyErr_Occurred()) return NULL;
    fsim_performance_profile_init(&p);
    if (fsim_vehicle_performance_profile(self->world, id, (int32_t)mode, &p, &why) != FSIM_OK) {
        if (why) return Py_BuildValue("(iO)", why, Py_None);
        return fail();
    }
    PyObject* lists[12] = {
        double_records(p.min_airspeed, p.min_airspeed_count, 4),
        double_records(p.max_airspeed, p.max_airspeed_count, 4),
        double_records(p.best_endurance_airspeed, p.best_endurance_airspeed_count, 4),
        double_records(p.best_range_airspeed, p.best_range_airspeed_count, 4),
        double_records(p.min_acceleration, p.min_acceleration_count, 7),
        double_records(p.max_acceleration, p.max_acceleration_count, 7),
        double_records(p.max_deceleration, p.max_deceleration_count, 7),
        double_records(p.excess_power, p.excess_power_count, 5),
        double_records(p.max_descent_rate, p.max_descent_rate_count, 4),
        double_records(p.burn, p.burn_count, 4),
        double_records(p.max_orientation, p.max_orientation_count, 7),
        double_records(p.max_orientation_rate, p.max_orientation_rate_count, 4),
    };
    for (int i = 0; i < 12; ++i)
        if (!lists[i]) {
            for (int j = 0; j < 12; ++j) Py_XDECREF(lists[j]);
            return NULL;
        }
    return Py_BuildValue("(i(iiiiiddddddddNNNNNNNNNNNN))", 0, p.mode, p.energy, p.clean, p.flaps_out, p.gear_down, p.time_s, p.altitude_msl_m,
                         p.weight_kg, p.tas_ms, p.min_altitude_msl_m, p.max_altitude_msl_m, p.max_turn_rate_rad_s, p.max_climb_rate_ms, lists[0],
                         lists[1], lists[2], lists[3], lists[4], lists[5], lists[6], lists[7], lists[8], lists[9], lists[10], lists[11]);
}

/* set_navigation(id, recovery, latitude_deg, longitude_deg, altitude_msl_m, reserve_fraction) */
PyObject* world_set_navigation(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    fsim_navigation_settings s;
    if (!check_args(n, 6, 6, "set_navigation") || !as_u32(args[0], &id)) return NULL;
    fsim_navigation_settings_init(&s);
    const int recovery = PyObject_IsTrue(args[1]);
    if (recovery < 0) return NULL;
    s.recovery = recovery;
    s.latitude_deg = PyFloat_AsDouble(args[2]);
    s.longitude_deg = PyFloat_AsDouble(args[3]);
    s.altitude_msl_m = PyFloat_AsDouble(args[4]);
    s.reserve_fraction = PyFloat_AsDouble(args[5]);
    if (PyErr_Occurred()) return NULL;
    if (fsim_vehicle_set_navigation(self->world, id, &s) != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

/* navigation(id) -> (recovery, latitude_deg, longitude_deg, altitude_msl_m, reserve_fraction) */
PyObject* world_navigation(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    fsim_navigation_settings s;
    if (!check_args(n, 1, 1, "navigation") || !as_u32(args[0], &id)) return NULL;
    fsim_navigation_settings_init(&s);
    if (fsim_vehicle_get_navigation(self->world, id, &s) != FSIM_OK) return fail();
    return Py_BuildValue("(Ndddd)", PyBool_FromLong(s.recovery), s.latitude_deg, s.longitude_deg, s.altitude_msl_m, s.reserve_fraction);
}

/* set_qnh(id, qnh_pa): what its barometric altimeter is set to (ABI 1.18); an error outside 850 to 1,100 hPa */
PyObject* world_set_qnh(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    if (!check_args(n, 2, 2, "set_qnh") || !as_u32(args[0], &id)) return NULL;
    const double qnh = PyFloat_AsDouble(args[1]);
    if (PyErr_Occurred()) return NULL;
    if (fsim_vehicle_set_qnh(self->world, id, qnh) != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

/* qnh(id) -> its altimeter's setting, Pa (ABI 1.18) */
PyObject* world_qnh(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    double qnh = 0.0;
    if (!check_args(n, 1, 1, "qnh") || !as_u32(args[0], &id)) return NULL;
    if (fsim_vehicle_qnh(self->world, id, &qnh) != FSIM_OK) return fail();
    return PyFloat_FromDouble(qnh);
}

/* state_data(id) -> (indicated_altitude_m, indicated_altitude_rate_ms, kollsman_hpa, static_pressure_pa, static_temperature_k,
 * yaw_rate_rad_s, pitch_rate_rad_s, roll_rate_rad_s, yaw_acceleration_rad_s2, pitch_acceleration_rad_s2,
 * roll_acceleration_rad_s2, wander_angle_rad, wind_north_ms, wind_east_ms, wind_down_ms) (ABI 1.18, 1.19) */
PyObject* world_state_data(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    fsim_state_data d;
    if (!check_args(n, 1, 1, "state_data") || !as_u32(args[0], &id)) return NULL;
    fsim_state_data_init(&d);
    if (fsim_vehicle_state_data(self->world, id, &d) != FSIM_OK) return fail();
    return Py_BuildValue("(ddddddddddddddddd)", d.indicated_altitude_m, d.indicated_altitude_rate_ms, d.kollsman_hpa, d.static_pressure_pa,
                         d.static_temperature_k, d.yaw_rate_rad_s, d.pitch_rate_rad_s, d.roll_rate_rad_s, d.yaw_acceleration_rad_s2,
                         d.pitch_acceleration_rad_s2, d.roll_acceleration_rad_s2, d.wander_angle_rad, d.wind_north_ms, d.wind_east_ms,
                         d.wind_down_ms, d.magnetic_heading_rad, d.declination_rad);
}

/* ownship(id) -> its own ship as its navigation and air data report it (ABI 1.57; ADR-31): 23 items */
PyObject* world_ownship(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    fsim_ownship d;
    if (!check_args(n, 1, 1, "ownship") || !as_u32(args[0], &id)) return NULL;
    fsim_ownship_init(&d);
    if (fsim_vehicle_ownship(self->world, id, &d) != FSIM_OK) return fail();
    return Py_BuildValue("(ddddd(ddd)(ddd)(ddd)(ddd)dddddddiiid)", d.time_s, d.latitude_rad, d.longitude_rad, d.altitude_msl_m,
                         d.altitude_agl_m, d.velocity_ned_ms[0], d.velocity_ned_ms[1], d.velocity_ned_ms[2], d.euler_rad[0],
                         d.euler_rad[1], d.euler_rad[2], d.angular_rate_body_rad_s[0], d.angular_rate_body_rad_s[1],
                         d.angular_rate_body_rad_s[2], d.acceleration_body_ms2[0], d.acceleration_body_ms2[1],
                         d.acceleration_body_ms2[2], d.airspeed_true_ms, d.airspeed_calibrated_ms, d.mach, d.alpha_rad, d.beta_rad,
                         d.load_factor, d.fuel_kg, (int)d.on_ground, (int)d.gnss_valid, (int)d.air_data_valid, d.throttle);
}

/* create_frame(origin, vehicle, latitude_rad, longitude_rad, altitude_msl_m, yaw_rad, pitch_rad, roll_rad, north_ms, east_ms,
 * down_ms, time_s) -> its id (ABI 1.19); an error where refused */
PyObject* world_create_frame(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    fsim_frame_spec spec;
    uint32_t vehicle;
    if (!check_args(n, 12, 12, "create_frame") || !as_u32(args[1], &vehicle)) return NULL;
    fsim_frame_spec_init(&spec);
    spec.origin = (int32_t)PyLong_AsLong(args[0]);
    spec.vehicle = vehicle;
    double* fields[] = {&spec.latitude_rad, &spec.longitude_rad, &spec.altitude_msl_m, &spec.yaw_rad, &spec.pitch_rad, &spec.roll_rad,
                        &spec.north_ms, &spec.east_ms, &spec.down_ms, &spec.time_s};
    for (int k = 0; k < 10 && !PyErr_Occurred(); ++k) *fields[k] = PyFloat_AsDouble(args[k + 2]);
    if (PyErr_Occurred()) return NULL;
    uint64_t id = 0;
    if (fsim_world_create_frame(self->world, &spec, &id) != FSIM_OK) return fail();
    return PyLong_FromUnsignedLongLong(id);
}

/* remove_frame(id) -> bool (ABI 1.19) */
PyObject* world_remove_frame(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint64_t id;
    if (!check_args(n, 1, 1, "remove_frame") || !as_u64(args[0], &id)) return NULL;
    return PyBool_FromLong(fsim_world_remove_frame(self->world, id) == FSIM_OK);
}

/* frame_point(id, rotation, offsets, x, y, z, time_s) -> (latitude_rad, longitude_rad, altitude_msl_m), or None for an
 * unknown frame or one whose vehicle is gone (ABI 1.19) */
PyObject* world_frame_point(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint64_t id;
    fsim_frame_offset offset;
    if (!check_args(n, 7, 7, "frame_point") || !as_u64(args[0], &id)) return NULL;
    fsim_frame_offset_init(&offset);
    offset.rotation = (int32_t)PyLong_AsLong(args[1]);
    offset.offsets = (int32_t)PyLong_AsLong(args[2]);
    offset.x = PyFloat_AsDouble(args[3]);
    offset.y = PyFloat_AsDouble(args[4]);
    offset.z = PyFloat_AsDouble(args[5]);
    const double time = PyFloat_AsDouble(args[6]);
    if (PyErr_Occurred()) return NULL;
    double lat = 0.0, lon = 0.0, alt = 0.0;
    if (fsim_world_frame_point(self->world, id, &offset, time, &lat, &lon, &alt) != FSIM_OK) Py_RETURN_NONE;
    return Py_BuildValue("(ddd)", lat, lon, alt);
}

/* An operational point as a tuple (ABI 1.39): (id, revision, latitude_rad, longitude_rad, altitude_m, altitude_reference,
 * frame, frame_rotation, frame_offsets, frame_x_m, frame_y_m, frame_z_m, ingress_min_rad, ingress_max_rad) */
static PyObject* op_point_tuple(const fsim_op_point* p) {
    return Py_BuildValue("(KIdddddddddddd)", (unsigned long long)p->op_point_id, (unsigned int)p->revision, p->latitude_rad, p->longitude_rad,
                         p->altitude_m, p->altitude_reference, p->frame, p->frame_rotation, p->frame_offsets, p->frame_x_m, p->frame_y_m,
                         p->frame_z_m, p->ingress_min_rad, p->ingress_max_rad);
}

/* set_op_point(id, latitude_rad, longitude_rad, altitude_m, altitude_reference, frame, frame_rotation, frame_offsets,
 * frame_x_m, frame_y_m, frame_z_m, ingress_min_rad, ingress_max_rad) -> reason (0: kept) (ABI 1.39) */
PyObject* world_set_op_point(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    fsim_op_point p;
    if (!check_args(n, 13, 13, "set_op_point")) return NULL;
    fsim_op_point_init(&p);
    if (!as_u64(args[0], &p.op_point_id)) return NULL;
    double* fields[] = {&p.latitude_rad, &p.longitude_rad, &p.altitude_m, &p.altitude_reference, &p.frame, &p.frame_rotation, &p.frame_offsets,
                        &p.frame_x_m, &p.frame_y_m, &p.frame_z_m, &p.ingress_min_rad, &p.ingress_max_rad};
    for (int k = 0; k < 12 && !PyErr_Occurred(); ++k) *fields[k] = PyFloat_AsDouble(args[k + 1]);
    if (PyErr_Occurred()) return NULL;
    int32_t reason = 0;
    if (fsim_world_set_op_point(self->world, &p, &reason) != FSIM_OK) return fail();
    return PyLong_FromLong(reason);
}

/* op_points() -> [op_point tuple], by id (ABI 1.39) */
PyObject* world_op_points(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    (void)args;
    if (!check_args(n, 0, 0, "op_points")) return NULL;
    const uint32_t count = fsim_world_op_point_count(self->world);
    PyObject* list = PyList_New(0);
    for (uint32_t i = 0; list && i < count; ++i) {
        fsim_op_point p;
        fsim_op_point_init(&p);
        if (fsim_world_get_op_point_at(self->world, i, &p) != FSIM_OK) continue;
        PyObject* t = op_point_tuple(&p);
        if (!t || PyList_Append(list, t) < 0) {
            Py_XDECREF(t);
            Py_DECREF(list);
            return NULL;
        }
        Py_DECREF(t);
    }
    return list;
}

/* op_point(id) -> op_point tuple, or None for one not kept (ABI 1.39) */
PyObject* world_op_point(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint64_t id;
    fsim_op_point p;
    if (!check_args(n, 1, 1, "op_point") || !as_u64(args[0], &id)) return NULL;
    fsim_op_point_init(&p);
    if (fsim_world_get_op_point(self->world, id, &p) != FSIM_OK) Py_RETURN_NONE;
    return op_point_tuple(&p);
}

/* remove_op_point(id) -> bool (ABI 1.39) */
PyObject* world_remove_op_point(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint64_t id;
    if (!check_args(n, 1, 1, "remove_op_point") || !as_u64(args[0], &id)) return NULL;
    return PyBool_FromLong(fsim_world_remove_op_point(self->world, id) == FSIM_OK);
}

/* An operational zone (ABI 1.40) as its tuple: (id, shape, vertices, holes, latitude_rad, longitude_rad, x_m, y_m, semi_major_m,
 * semi_minor_m, width_m, height_m, range_min_m, range_max_m, azimuth_min_rad, azimuth_max_rad, orientation_rad, altitude_min_m,
 * altitude_max_m, altitude_reference, frame, frame_rotation, north_ms, east_ms, time_s, revision) - vertices a sequence of
 * (latitude_rad, longitude_rad, x_m, y_m), holes a sequence of such sequences. read_zone fills `z` and its arrays (PyMem:
 * zone_free frees them). */
typedef struct {
    fsim_op_zone z;
    fsim_zone_vertex* vertices;
    fsim_zone_vertex** holes;
    uint32_t* sizes;
} ZoneArrays;

static void zone_free(ZoneArrays* a) {
    if (a->holes)
        for (uint32_t k = 0; k < a->z.hole_count; ++k) PyMem_Free(a->holes[k]);
    PyMem_Free(a->vertices), PyMem_Free(a->holes), PyMem_Free(a->sizes);
    a->vertices = NULL, a->holes = NULL, a->sizes = NULL;
}

/* A ring's vertices from a sequence of 4-number rows, into a new array (its count into `count`); NULL with an error set. */
static fsim_zone_vertex* read_ring(PyObject* o, uint32_t* count) {
    PyObject* rows = PySequence_Tuple(o);
    if (!rows) return NULL;
    const Py_ssize_t n = PyTuple_Size(rows);
    fsim_zone_vertex* out = (fsim_zone_vertex*)PyMem_Calloc((size_t)(n ? n : 1), sizeof(fsim_zone_vertex));
    int ok = out != NULL;
    if (!ok) PyErr_NoMemory();
    for (Py_ssize_t i = 0; ok && i < n; ++i) {
        PyObject* row = PySequence_Tuple(PyTuple_GetItem(rows, i));
        ok = row && PyTuple_Size(row) == 4;
        if (ok) {
            out[i].latitude_rad = PyFloat_AsDouble(PyTuple_GetItem(row, 0)), out[i].longitude_rad = PyFloat_AsDouble(PyTuple_GetItem(row, 1));
            out[i].x_m = PyFloat_AsDouble(PyTuple_GetItem(row, 2)), out[i].y_m = PyFloat_AsDouble(PyTuple_GetItem(row, 3));
            ok = !PyErr_Occurred();
        } else if (!PyErr_Occurred()) {
            PyErr_SetString(PyExc_ValueError, "a zone's vertex is 4 numbers: latitude_rad, longitude_rad, x_m, y_m");
        }
        Py_XDECREF(row);
    }
    Py_DECREF(rows);
    if (!ok) {
        PyMem_Free(out);
        return NULL;
    }
    *count = (uint32_t)n;
    return out;
}

static int read_zone(PyObject* o, ZoneArrays* a) {
    fsim_op_zone_init(&a->z);
    a->vertices = NULL, a->holes = NULL, a->sizes = NULL;
    PyObject* t = PySequence_Tuple(o);
    if (!t) return 0;
    int ok = PyTuple_Size(t) >= 25;
    if (!ok) PyErr_SetString(PyExc_ValueError, "a zone is 25 items (fsim.OpZone)");
    if (ok) ok = as_u64(PyTuple_GetItem(t, 0), &a->z.op_zone_id);
    if (ok) {
        a->z.shape = PyFloat_AsDouble(PyTuple_GetItem(t, 1));
        ok = !PyErr_Occurred();
    }
    if (ok) {
        a->vertices = read_ring(PyTuple_GetItem(t, 2), &a->z.vertex_count);
        ok = a->vertices != NULL;
    }
    if (ok) {
        PyObject* holes = PySequence_Tuple(PyTuple_GetItem(t, 3));
        ok = holes != NULL;
        const Py_ssize_t h = ok ? PyTuple_Size(holes) : 0;
        if (ok) {
            a->holes = (fsim_zone_vertex**)PyMem_Calloc((size_t)(h ? h : 1), sizeof(fsim_zone_vertex*));
            a->sizes = (uint32_t*)PyMem_Calloc((size_t)(h ? h : 1), sizeof(uint32_t));
            ok = a->holes && a->sizes;
            if (!ok) PyErr_NoMemory();
        }
        for (Py_ssize_t k = 0; ok && k < h; ++k) {
            a->holes[k] = read_ring(PyTuple_GetItem(holes, k), &a->sizes[k]);
            ok = a->holes[k] != NULL;
            a->z.hole_count = (uint32_t)(k + 1);
        }
        Py_XDECREF(holes);
    }
    double* fields[] = {&a->z.latitude_rad, &a->z.longitude_rad, &a->z.x_m, &a->z.y_m, &a->z.semi_major_m, &a->z.semi_minor_m, &a->z.width_m,
                        &a->z.height_m, &a->z.range_min_m, &a->z.range_max_m, &a->z.azimuth_min_rad, &a->z.azimuth_max_rad,
                        &a->z.orientation_rad, &a->z.altitude_min_m, &a->z.altitude_max_m, &a->z.altitude_reference, &a->z.frame,
                        &a->z.frame_rotation, &a->z.north_ms, &a->z.east_ms, &a->z.time_s};
    for (int k = 0; ok && k < 21; ++k) {
        *fields[k] = PyFloat_AsDouble(PyTuple_GetItem(t, 4 + k));
        ok = !PyErr_Occurred();
    }
    Py_DECREF(t);
    if (ok) {
        a->z.vertices = a->z.vertex_count ? a->vertices : NULL;
        a->z.holes = a->z.hole_count ? (const fsim_zone_vertex* const*)a->holes : NULL;
        a->z.hole_sizes = a->z.hole_count ? a->sizes : NULL;
    } else {
        zone_free(a);
    }
    return ok;
}

static PyObject* ring_list(const fsim_zone_vertex* v, uint32_t n) {
    PyObject* list = PyList_New(0);
    for (uint32_t i = 0; list && i < n; ++i) {
        PyObject* row = Py_BuildValue("(dddd)", v[i].latitude_rad, v[i].longitude_rad, v[i].x_m, v[i].y_m);
        if (!row || PyList_Append(list, row) < 0) {
            Py_XDECREF(row);
            Py_DECREF(list);
            return NULL;
        }
        Py_DECREF(row);
    }
    return list;
}

static PyObject* zone_tuple(const fsim_op_zone* z) {
    PyObject* vertices = ring_list(z->vertices, z->vertex_count);
    PyObject* holes = vertices ? PyList_New(0) : NULL;
    for (uint32_t k = 0; holes && k < z->hole_count; ++k) {
        PyObject* ring = ring_list(z->holes[k], z->hole_sizes[k]);
        if (!ring || PyList_Append(holes, ring) < 0) {
            Py_XDECREF(ring);
            Py_CLEAR(holes);
            break;
        }
        Py_DECREF(ring);
    }
    if (!vertices || !holes) {
        Py_XDECREF(vertices);
        Py_XDECREF(holes);
        return NULL;
    }
    return Py_BuildValue("(KdNNdddddddddddddddddddddI)", (unsigned long long)z->op_zone_id, z->shape, vertices, holes, z->latitude_rad,
                         z->longitude_rad, z->x_m, z->y_m, z->semi_major_m, z->semi_minor_m, z->width_m, z->height_m, z->range_min_m,
                         z->range_max_m, z->azimuth_min_rad, z->azimuth_max_rad, z->orientation_rad, z->altitude_min_m, z->altitude_max_m,
                         z->altitude_reference, z->frame, z->frame_rotation, z->north_ms, z->east_ms, z->time_s, (unsigned int)z->revision);
}

/* set_op_zone(zone tuple) -> reason (0: kept) (ABI 1.40) */
PyObject* world_set_op_zone(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    ZoneArrays a;
    if (!check_args(n, 1, 1, "set_op_zone") || !read_zone(args[0], &a)) return NULL;
    int32_t reason = 0;
    const int rc = fsim_world_set_op_zone(self->world, &a.z, &reason);
    zone_free(&a);
    if (rc != FSIM_OK) return fail();
    return PyLong_FromLong(reason);
}

/* op_zones() -> [zone tuple], by id (ABI 1.40) */
PyObject* world_op_zones(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    (void)args;
    if (!check_args(n, 0, 0, "op_zones")) return NULL;
    const uint32_t count = fsim_world_op_zone_count(self->world);
    PyObject* list = PyList_New(0);
    for (uint32_t i = 0; list && i < count; ++i) {
        fsim_op_zone z;
        fsim_op_zone_init(&z);
        if (fsim_world_get_op_zone_at(self->world, i, &z) != FSIM_OK) continue;
        PyObject* t = zone_tuple(&z);
        if (!t || PyList_Append(list, t) < 0) {
            Py_XDECREF(t);
            Py_DECREF(list);
            return NULL;
        }
        Py_DECREF(t);
    }
    return list;
}

/* op_zone(id) -> zone tuple, or None for one not kept (ABI 1.40) */
PyObject* world_op_zone(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint64_t id;
    fsim_op_zone z;
    if (!check_args(n, 1, 1, "op_zone") || !as_u64(args[0], &id)) return NULL;
    fsim_op_zone_init(&z);
    if (fsim_world_get_op_zone(self->world, id, &z) != FSIM_OK) Py_RETURN_NONE;
    return zone_tuple(&z);
}

/* remove_op_zone(id) -> bool (ABI 1.40) */
PyObject* world_remove_op_zone(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint64_t id;
    if (!check_args(n, 1, 1, "remove_op_zone") || !as_u64(args[0], &id)) return NULL;
    return PyBool_FromLong(fsim_world_remove_op_zone(self->world, id) == FSIM_OK);
}

/* submit_must_fly(id, values, zone tuple or None, source=None, axes=None, range=None, min_version=None, envelope=None) -> result
 * (ABI 1.40) */
PyObject* world_submit_must_fly(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    double row[FSIM_PY_VALUES];
    fsim_command_options opt;
    fsim_command_result r;
    ZoneArrays a;
    if (!check_args(n, 3, 8, "submit_must_fly") || !as_u32(args[0], &id) || !WORLD_IDLE(self)) return NULL;
    const Py_ssize_t count = read_values(args[1], row, "submit_must_fly");
    if (count < 0 || !read_options(args, n, 3, &opt)) return NULL;
    const int zoned = args[2] != Py_None;
    if (zoned && !read_zone(args[2], &a)) return NULL;
    const int rc = fsim_vehicle_submit_must_fly(self->world, id, row, (uint32_t)count, zoned ? &a.z : NULL, &opt, &r);
    if (zoned) zone_free(&a);
    if (rc != FSIM_OK) return fail();
    return result_tuple(self->world, &r);
}

/* activity_update_must_fly(activity, values, zone tuple or None, source, controller) -> result (ABI 1.40) */
PyObject* world_activity_update_must_fly(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint64_t activity;
    double row[FSIM_PY_VALUES];
    fsim_command_result r;
    int source = 0;
    uint32_t controller = 0;
    ZoneArrays a;
    if (!check_args(n, 3, 5, "activity_update_must_fly") || !as_u64(args[0], &activity) || (n > 3 && !as_int(args[3], &source)) ||
        (n > 4 && !as_u32(args[4], &controller)) || !WORLD_IDLE(self))
        return NULL;
    const Py_ssize_t count = read_values(args[1], row, "activity_update_must_fly");
    if (count < 0) return NULL;
    const int zoned = args[2] != Py_None;
    if (zoned && !read_zone(args[2], &a)) return NULL;
    const int rc = fsim_activity_update_must_fly_by(self->world, activity, source, controller, row, (uint32_t)count, zoned ? &a.z : NULL, &r);
    if (zoned) zone_free(&a);
    if (rc != FSIM_OK) return fail();
    return result_tuple(self->world, &r);
}

/* An operational line (ABI 1.41) as its tuple: (id, vertices, projection, left_width_m, right_width_m, altitude_min_m,
 * altitude_max_m, altitude_reference, frame, frame_rotation, north_ms, east_ms, time_s, revision) - vertices a sequence of
 * (latitude_rad, longitude_rad, x_m, y_m, altitude_m, altitude_min_m, altitude_max_m, altitude_reference, left_width_m,
 * right_width_m). read_line fills `l` and its vertices (PyMem: line_free frees them). */
typedef struct {
    fsim_op_line l;
    fsim_line_vertex* vertices;
} LineArrays;

static void line_free(LineArrays* a) {
    PyMem_Free(a->vertices);
    a->vertices = NULL;
}

static int read_line(PyObject* o, LineArrays* a) {
    fsim_op_line_init(&a->l);
    a->vertices = NULL;
    PyObject* t = PySequence_Tuple(o);
    if (!t) return 0;
    int ok = PyTuple_Size(t) >= 13;
    if (!ok) PyErr_SetString(PyExc_ValueError, "a line is 13 items (fsim.OpLine)");
    if (ok) ok = as_u64(PyTuple_GetItem(t, 0), &a->l.op_line_id);
    PyObject* rows = ok ? PySequence_Tuple(PyTuple_GetItem(t, 1)) : NULL;
    ok = ok && rows != NULL;
    const Py_ssize_t n = ok ? PyTuple_Size(rows) : 0;
    if (ok) {
        a->vertices = (fsim_line_vertex*)PyMem_Calloc((size_t)(n ? n : 1), sizeof(fsim_line_vertex));
        ok = a->vertices != NULL;
        if (!ok) PyErr_NoMemory();
    }
    for (Py_ssize_t i = 0; ok && i < n; ++i) {
        PyObject* row = PySequence_Tuple(PyTuple_GetItem(rows, i));
        ok = row && PyTuple_Size(row) == 10;
        if (ok) {
            fsim_line_vertex* v = &a->vertices[i];
            double* f[] = {&v->latitude_rad, &v->longitude_rad, &v->x_m, &v->y_m, &v->altitude_m, &v->altitude_min_m, &v->altitude_max_m,
                           &v->altitude_reference, &v->left_width_m, &v->right_width_m};
            for (int k = 0; ok && k < 10; ++k) {
                *f[k] = PyFloat_AsDouble(PyTuple_GetItem(row, k));
                ok = !PyErr_Occurred();
            }
        } else if (!PyErr_Occurred()) {
            PyErr_SetString(PyExc_ValueError, "a line's vertex is 10 numbers (fsim.LineVertex)");
        }
        Py_XDECREF(row);
    }
    Py_XDECREF(rows);
    double* fields[] = {&a->l.projection, &a->l.left_width_m, &a->l.right_width_m, &a->l.altitude_min_m, &a->l.altitude_max_m,
                        &a->l.altitude_reference, &a->l.frame, &a->l.frame_rotation, &a->l.north_ms, &a->l.east_ms, &a->l.time_s};
    for (int k = 0; ok && k < 11; ++k) {
        *fields[k] = PyFloat_AsDouble(PyTuple_GetItem(t, 2 + k));
        ok = !PyErr_Occurred();
    }
    Py_DECREF(t);
    if (ok) {
        a->l.vertices = n ? a->vertices : NULL;
        a->l.vertex_count = (uint32_t)n;
    } else {
        line_free(a);
    }
    return ok;
}

static PyObject* line_tuple(const fsim_op_line* l) {
    PyObject* vertices = PyList_New(0);
    for (uint32_t i = 0; vertices && i < l->vertex_count; ++i) {
        const fsim_line_vertex* v = &l->vertices[i];
        PyObject* row = Py_BuildValue("(dddddddddd)", v->latitude_rad, v->longitude_rad, v->x_m, v->y_m, v->altitude_m, v->altitude_min_m,
                                      v->altitude_max_m, v->altitude_reference, v->left_width_m, v->right_width_m);
        if (!row || PyList_Append(vertices, row) < 0) {
            Py_XDECREF(row);
            Py_CLEAR(vertices);
            break;
        }
        Py_DECREF(row);
    }
    if (!vertices) return NULL;
    return Py_BuildValue("(KNdddddddddddI)", (unsigned long long)l->op_line_id, vertices, l->projection, l->left_width_m, l->right_width_m,
                         l->altitude_min_m, l->altitude_max_m, l->altitude_reference, l->frame, l->frame_rotation, l->north_ms, l->east_ms,
                         l->time_s, (unsigned int)l->revision);
}

/* set_op_line(line tuple) -> reason (0: kept) (ABI 1.41) */
PyObject* world_set_op_line(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    LineArrays a;
    if (!check_args(n, 1, 1, "set_op_line") || !read_line(args[0], &a)) return NULL;
    int32_t reason = 0;
    const int rc = fsim_world_set_op_line(self->world, &a.l, &reason);
    line_free(&a);
    if (rc != FSIM_OK) return fail();
    return PyLong_FromLong(reason);
}

/* op_lines() -> [line tuple], by id (ABI 1.41) */
PyObject* world_op_lines(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    (void)args;
    if (!check_args(n, 0, 0, "op_lines")) return NULL;
    const uint32_t count = fsim_world_op_line_count(self->world);
    PyObject* list = PyList_New(0);
    for (uint32_t i = 0; list && i < count; ++i) {
        fsim_op_line l;
        fsim_op_line_init(&l);
        if (fsim_world_get_op_line_at(self->world, i, &l) != FSIM_OK) continue;
        PyObject* t = line_tuple(&l);
        if (!t || PyList_Append(list, t) < 0) {
            Py_XDECREF(t);
            Py_DECREF(list);
            return NULL;
        }
        Py_DECREF(t);
    }
    return list;
}

/* op_line(id) -> line tuple, or None for one not kept (ABI 1.41) */
PyObject* world_op_line(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint64_t id;
    fsim_op_line l;
    if (!check_args(n, 1, 1, "op_line") || !as_u64(args[0], &id)) return NULL;
    fsim_op_line_init(&l);
    if (fsim_world_get_op_line(self->world, id, &l) != FSIM_OK) Py_RETURN_NONE;
    return line_tuple(&l);
}

/* remove_op_line(id) -> bool (ABI 1.41) */
PyObject* world_remove_op_line(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint64_t id;
    if (!check_args(n, 1, 1, "remove_op_line") || !as_u64(args[0], &id)) return NULL;
    return PyBool_FromLong(fsim_world_remove_op_line(self->world, id) == FSIM_OK);
}

/* submit_must_fly_line(id, values, line tuple, source=None, axes=None, range=None, min_version=None, envelope=None) -> result
 * (ABI 1.41) */
PyObject* world_submit_must_fly_line(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    double row[FSIM_PY_VALUES];
    fsim_command_options opt;
    fsim_command_result r;
    LineArrays a;
    if (!check_args(n, 3, 8, "submit_must_fly_line") || !as_u32(args[0], &id) || !WORLD_IDLE(self)) return NULL;
    const Py_ssize_t count = read_values(args[1], row, "submit_must_fly_line");
    if (count < 0 || !read_options(args, n, 3, &opt) || !read_line(args[2], &a)) return NULL;
    const int rc = fsim_vehicle_submit_must_fly_line(self->world, id, row, (uint32_t)count, &a.l, &opt, &r);
    line_free(&a);
    if (rc != FSIM_OK) return fail();
    return result_tuple(self->world, &r);
}

/* activity_update_must_fly_line(activity, values, line tuple, source, controller) -> result (ABI 1.41) */
PyObject* world_activity_update_must_fly_line(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint64_t activity;
    double row[FSIM_PY_VALUES];
    fsim_command_result r;
    int source = 0;
    uint32_t controller = 0;
    LineArrays a;
    if (!check_args(n, 3, 5, "activity_update_must_fly_line") || !as_u64(args[0], &activity) || (n > 3 && !as_int(args[3], &source)) ||
        (n > 4 && !as_u32(args[4], &controller)) || !WORLD_IDLE(self))
        return NULL;
    const Py_ssize_t count = read_values(args[1], row, "activity_update_must_fly_line");
    if (count < 0 || !read_line(args[2], &a)) return NULL;
    const int rc = fsim_activity_update_must_fly_line_by(self->world, activity, source, controller, row, (uint32_t)count, &a.l, &r);
    line_free(&a);
    if (rc != FSIM_OK) return fail();
    return result_tuple(self->world, &r);
}

/* An operational volume (ABI 1.42) as its tuple: (id, shape, latitude_rad, longitude_rad, x_m, y_m, altitude_m, altitude_reference,
 * radius_m, semi_axis_a_m, semi_axis_b_m, semi_axis_c_m, length_m, half_angle_rad, length_half_angle_rad, width_half_angle_rad,
 * range_m, yaw_rad, pitch_rad, roll_rad, latitude_min_rad, latitude_max_rad, longitude_min_rad, longitude_max_rad, altitude_min_m,
 * altitude_max_m, frame, frame_rotation, north_ms, east_ms, down_ms, time_s, revision). */
static int read_volume(PyObject* o, fsim_op_volume* a) {
    fsim_op_volume_init(a);
    PyObject* t = PySequence_Tuple(o);
    if (!t) return 0;
    int ok = PyTuple_Size(t) >= 32;
    if (!ok) PyErr_SetString(PyExc_ValueError, "a volume is 32 items (fsim.OpVolume)");
    if (ok) ok = as_u64(PyTuple_GetItem(t, 0), &a->op_volume_id);
    double* fields[] = {&a->shape, &a->latitude_rad, &a->longitude_rad, &a->x_m, &a->y_m, &a->altitude_m, &a->altitude_reference, &a->radius_m, &a->semi_axis_a_m, &a->semi_axis_b_m, &a->semi_axis_c_m, &a->length_m, &a->half_angle_rad, &a->length_half_angle_rad, &a->width_half_angle_rad, &a->range_m, &a->yaw_rad, &a->pitch_rad, &a->roll_rad, &a->latitude_min_rad, &a->latitude_max_rad, &a->longitude_min_rad, &a->longitude_max_rad, &a->altitude_min_m, &a->altitude_max_m, &a->frame, &a->frame_rotation, &a->north_ms, &a->east_ms, &a->down_ms, &a->time_s};
    for (int k = 0; ok && k < 31; ++k) {
        *fields[k] = PyFloat_AsDouble(PyTuple_GetItem(t, 1 + k));
        ok = !PyErr_Occurred();
    }
    Py_DECREF(t);
    return ok;
}

static PyObject* volume_tuple(const fsim_op_volume* v) {
    return Py_BuildValue("(KdddddddddddddddddddddddddddddddI)", (unsigned long long)v->op_volume_id, v->shape, v->latitude_rad, v->longitude_rad, v->x_m, v->y_m, v->altitude_m, v->altitude_reference, v->radius_m, v->semi_axis_a_m, v->semi_axis_b_m, v->semi_axis_c_m, v->length_m, v->half_angle_rad, v->length_half_angle_rad, v->width_half_angle_rad, v->range_m, v->yaw_rad, v->pitch_rad, v->roll_rad, v->latitude_min_rad, v->latitude_max_rad, v->longitude_min_rad, v->longitude_max_rad, v->altitude_min_m, v->altitude_max_m, v->frame, v->frame_rotation, v->north_ms, v->east_ms, v->down_ms, v->time_s, (unsigned int)v->revision);
}

/* set_op_volume(volume tuple) -> reason (0: kept) (ABI 1.42) */
PyObject* world_set_op_volume(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    fsim_op_volume v;
    if (!check_args(n, 1, 1, "set_op_volume") || !read_volume(args[0], &v)) return NULL;
    int32_t reason = 0;
    if (fsim_world_set_op_volume(self->world, &v, &reason) != FSIM_OK) return fail();
    return PyLong_FromLong(reason);
}

/* op_volumes() -> [volume tuple], by id (ABI 1.42) */
PyObject* world_op_volumes(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    (void)args;
    if (!check_args(n, 0, 0, "op_volumes")) return NULL;
    const uint32_t count = fsim_world_op_volume_count(self->world);
    PyObject* list = PyList_New(0);
    for (uint32_t i = 0; list && i < count; ++i) {
        fsim_op_volume v;
        fsim_op_volume_init(&v);
        if (fsim_world_get_op_volume_at(self->world, i, &v) != FSIM_OK) continue;
        PyObject* t = volume_tuple(&v);
        if (!t || PyList_Append(list, t) < 0) {
            Py_XDECREF(t);
            Py_DECREF(list);
            return NULL;
        }
        Py_DECREF(t);
    }
    return list;
}

/* op_volume(id) -> volume tuple, or None for one not kept (ABI 1.42) */
PyObject* world_op_volume(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint64_t id;
    fsim_op_volume v;
    if (!check_args(n, 1, 1, "op_volume") || !as_u64(args[0], &id)) return NULL;
    fsim_op_volume_init(&v);
    if (fsim_world_get_op_volume(self->world, id, &v) != FSIM_OK) Py_RETURN_NONE;
    return volume_tuple(&v);
}

/* remove_op_volume(id) -> bool (ABI 1.42) */
PyObject* world_remove_op_volume(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint64_t id;
    if (!check_args(n, 1, 1, "remove_op_volume") || !as_u64(args[0], &id)) return NULL;
    return PyBool_FromLong(fsim_world_remove_op_volume(self->world, id) == FSIM_OK);
}

/* submit_must_fly_volume(id, values, volume tuple, source=None, axes=None, range=None, min_version=None, envelope=None) -> result
 * (ABI 1.42) */
PyObject* world_submit_must_fly_volume(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    double row[FSIM_PY_VALUES];
    fsim_command_options opt;
    fsim_command_result r;
    fsim_op_volume v;
    if (!check_args(n, 3, 8, "submit_must_fly_volume") || !as_u32(args[0], &id) || !WORLD_IDLE(self)) return NULL;
    const Py_ssize_t count = read_values(args[1], row, "submit_must_fly_volume");
    if (count < 0 || !read_options(args, n, 3, &opt) || !read_volume(args[2], &v)) return NULL;
    if (fsim_vehicle_submit_must_fly_volume(self->world, id, row, (uint32_t)count, &v, &opt, &r) != FSIM_OK) return fail();
    return result_tuple(self->world, &r);
}

/* activity_update_must_fly_volume(activity, values, volume tuple, source, controller) -> result (ABI 1.42) */
PyObject* world_activity_update_must_fly_volume(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint64_t activity;
    double row[FSIM_PY_VALUES];
    fsim_command_result r;
    int source = 0;
    uint32_t controller = 0;
    fsim_op_volume v;
    if (!check_args(n, 3, 5, "activity_update_must_fly_volume") || !as_u64(args[0], &activity) || (n > 3 && !as_int(args[3], &source)) ||
        (n > 4 && !as_u32(args[4], &controller)) || !WORLD_IDLE(self))
        return NULL;
    const Py_ssize_t count = read_values(args[1], row, "activity_update_must_fly_volume");
    if (count < 0 || !read_volume(args[2], &v)) return NULL;
    if (fsim_activity_update_must_fly_volume_by(self->world, activity, source, controller, row, (uint32_t)count, &v, &r) != FSIM_OK) return fail();
    return result_tuple(self->world, &r);
}

/* activity_setpoint(activity) -> (kind, code, fields, behavior, waypoints, segments), or None for one not live:
 * `behavior` (id, target, {name: value}, [(latitude_rad, longitude_rad, altitude_msl_m, airspeed_ms, capture_radius_m)])
 * or None, `waypoints` [waypoint row] or None, `segments` [18 floats: north, east, down] or None - a curve's as A-GRA's
 * schema gives them (kind FSIM_BATCH_NURBS) [59 floats: read_nurbs's] */
static PyObject* batch_tuple(const fsim_batch_command* item) {
    const fsim_batch_command b = *item;
    PyObject* fields = PyTuple_New(b.count);
    for (uint32_t i = 0; fields && i < b.count; ++i) {
        PyObject* v = PyFloat_FromDouble(b.fields[i]);
        if (!v || PyTuple_SetItem(fields, i, v) < 0) Py_CLEAR(fields); /* (SetItem takes the item, even when it fails) */
    }
    PyObject* behavior = NULL;
    if (fields && b.behavior) {
        PyObject* params = PyDict_New();
        for (uint32_t i = 0; params && i < b.behavior->param_count; ++i) {
            PyObject* v = PyFloat_FromDouble(b.behavior->param_values[i]);
            if (!v || PyDict_SetItemString(params, b.behavior->param_names[i], v) < 0) Py_CLEAR(params);
            Py_XDECREF(v);
        }
        PyObject* points = params ? PyList_New(0) : NULL;
        for (uint32_t i = 0; points && i < b.behavior->point_count; ++i) {
            const fsim_position_command* p = &b.behavior->points[i];
            PyObject* row = Py_BuildValue("(ddddd)", p->latitude_rad, p->longitude_rad, p->altitude_msl_m, p->airspeed_ms, p->capture_radius_m);
            if (!row || PyList_Append(points, row) < 0) Py_CLEAR(points);
            Py_XDECREF(row);
        }
        behavior = points ? Py_BuildValue("(sINN)", b.behavior->id, b.behavior->target, params, points) : NULL;
        if (!points) Py_XDECREF(params);
        if (!behavior) Py_CLEAR(fields);
    } else {
        behavior = Py_NewRef(Py_None);
    }
    PyObject* waypoints = NULL;
    if (fields && b.waypoints) {
        waypoints = PyList_New(0);
        for (uint32_t i = 0; waypoints && i < b.waypoint_count; ++i) {
            const fsim_waypoint* w = &b.waypoints[i];
            PyObject* row = Py_BuildValue("(dddddddddKdddddddddddddddddddd)", w->latitude_rad, w->longitude_rad, w->altitude_m, w->altitude_reference,
                                          w->speed, w->speed_reference, w->turn, w->max_bank_rad, w->climb_rate_ms, (unsigned long long)w->id,
                                          w->altitude_min_m, w->altitude_max_m, w->kind, w->waypoint_type, w->frame, w->frame_rotation,
                                          w->frame_offsets, w->frame_x_m, w->frame_y_m, w->frame_z_m, w->course_rad, w->turn_radius_m,
                                          w->speed_optimization, w->climb_optimization, w->acceleration_ms2, w->arrival_begin_s, w->arrival_end_s,
                                          w->rnp_m, w->next, w->terminator);
            if (!row || PyList_Append(waypoints, row) < 0) Py_CLEAR(waypoints);
            Py_XDECREF(row);
        }
    } else if (fields) {
        waypoints = Py_NewRef(Py_None);
    }
    PyObject* segments = NULL;
    if (waypoints && b.kind == FSIM_BATCH_NURBS && b.nurbs) { /* (ABI 1.24: read_nurbs's rows) */
        segments = PyList_New(0);
        for (uint32_t i = 0; segments && i < b.segment_count; ++i) {
            PyObject* row = nurbs_row(&b.nurbs[i]);
            if (!row || PyList_Append(segments, row) < 0) Py_CLEAR(segments);
            Py_XDECREF(row);
        }
    } else if (waypoints && b.segments) {
        segments = PyList_New(0);
        for (uint32_t i = 0; segments && i < b.segment_count; ++i) {
            const fsim_bezier_segment* s = &b.segments[i];
            PyObject* row = Py_BuildValue("(dddddddddddddddddd)", s->north[0], s->north[1], s->north[2], s->north[3], s->north[4], s->north[5],
                                          s->east[0], s->east[1], s->east[2], s->east[3], s->east[4], s->east[5], s->down[0], s->down[1],
                                          s->down[2], s->down[3], s->down[4], s->down[5]);
            if (!row || PyList_Append(segments, row) < 0) Py_CLEAR(segments);
            Py_XDECREF(row);
        }
    } else if (waypoints) {
        segments = Py_NewRef(Py_None);
    }
    PyObject* loiters = NULL; /* (a route's: ABI 1.28, read_loiters's rows) */
    if (segments && b.loiters) {
        loiters = PyList_New(0);
        for (uint32_t i = 0; loiters && i < b.loiter_count; ++i) {
            PyObject* row = loiter_row(&b.loiters[i]);
            if (!row || PyList_Append(loiters, row) < 0) Py_CLEAR(loiters);
            Py_XDECREF(row);
        }
    } else if (segments) {
        loiters = Py_NewRef(Py_None);
    }
    PyObject* states = NULL; /* (a route's: ABI 1.31, read_states's rows) */
    if (loiters && b.states) {
        states = PyList_New(0);
        for (uint32_t i = 0; states && i < b.state_count; ++i) {
            PyObject* row = state_row(&b.states[i]);
            if (!row || PyList_Append(states, row) < 0) Py_CLEAR(states);
            Py_XDECREF(row);
        }
    } else if (loiters) {
        states = Py_NewRef(Py_None);
    }
    PyObject* paths = NULL; /* (a route's: ABI 1.33, read_paths's rows) */
    if (states && b.paths) {
        paths = PyList_New(0);
        for (uint32_t i = 0; paths && i < b.path_count; ++i) {
            const fsim_route_path* p = &b.paths[i];
            PyObject* row = Py_BuildValue("(KdII)", (unsigned long long)p->id, p->type, p->first, p->count);
            if (!row || PyList_Append(paths, row) < 0) Py_CLEAR(paths);
            Py_XDECREF(row);
        }
    } else if (states) {
        paths = Py_NewRef(Py_None);
    }
    PyObject* branches = NULL; /* (a route's: ABI 1.34, read_branches's rows) */
    if (paths && b.branches) {
        branches = PyList_New(0);
        for (uint32_t i = 0; branches && i < b.branch_count; ++i) {
            PyObject* row = branch_row(&b.branches[i]);
            if (!row || PyList_Append(branches, row) < 0) Py_CLEAR(branches);
            Py_XDECREF(row);
        }
    } else if (paths) {
        branches = Py_NewRef(Py_None);
    }
    PyObject* terminators = NULL; /* (a route's: ABI 1.35, read_terminators's rows) */
    if (branches && b.terminators) {
        terminators = PyList_New(0);
        for (uint32_t i = 0; terminators && i < b.terminator_count; ++i) {
            PyObject* row = terminator_row(&b.terminators[i]);
            if (!row || PyList_Append(terminators, row) < 0) Py_CLEAR(terminators);
            Py_XDECREF(row);
        }
    } else if (branches) {
        terminators = Py_NewRef(Py_None);
    }
    if (!fields || !behavior || !waypoints || !segments || !loiters || !states || !paths || !branches || !terminators) {
        Py_XDECREF(fields), Py_XDECREF(behavior), Py_XDECREF(waypoints), Py_XDECREF(segments), Py_XDECREF(loiters), Py_XDECREF(states);
        Py_XDECREF(paths), Py_XDECREF(branches), Py_XDECREF(terminators);
        return NULL;
    }
    return Py_BuildValue("(iiNNNNNNNNN)", b.kind, b.code, fields, behavior, waypoints, segments, loiters, states, paths, branches, terminators);
}

PyObject* world_activity_setpoint(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint64_t activity;
    fsim_batch_command b;
    if (!check_args(n, 1, 1, "activity_setpoint") || !as_u64(args[0], &activity)) return NULL;
    memset(&b, 0, sizeof b);
    b.struct_size = sizeof b;
    if (fsim_activity_get_setpoint(self->world, activity, &b) != FSIM_OK) Py_RETURN_NONE;
    return batch_tuple(&b);
}

/* activity_end_points(activity, max) -> [(kind, latitude_rad, longitude_rad, altitude_m, altitude_reference, turn, id, index)] */
PyObject* world_activity_end_points(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint64_t activity;
    uint32_t max, count = 0;
    if (!check_args(n, 2, 2, "activity_end_points") || !as_u64(args[0], &activity) || !as_u32(args[1], &max)) return NULL;
    fsim_end_point* points = max ? (fsim_end_point*)PyMem_Calloc(max, sizeof(fsim_end_point)) : NULL;
    if (max && !points) return PyErr_NoMemory();
    for (uint32_t i = 0; i < max; ++i) fsim_end_point_init(&points[i]);
    const int status = fsim_activity_end_points(self->world, activity, points, max, &count);
    PyObject* out = status == FSIM_OK ? PyList_New(0) : NULL;
    for (uint32_t i = 0; out && i < count; ++i) {
        const fsim_end_point* p = &points[i];
        PyObject* row = Py_BuildValue("(idddddKi)", p->kind, p->latitude_rad, p->longitude_rad, p->altitude_m, p->altitude_reference, p->turn,
                                      (unsigned long long)p->id, p->index);
        if (!row || PyList_Append(out, row) < 0) Py_CLEAR(out);
        Py_XDECREF(row);
    }
    PyMem_Free(points);
    return status == FSIM_OK ? out : fail();
}

/* capability_status(id, capability) -> (availability, reason) */
PyObject* world_capability_status(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    int32_t availability = 0, reason = 0;
    if (!check_args(n, 2, 2, "capability_status") || !as_u32(args[0], &id)) return NULL;
    const char* cap = as_str(args[1], "capability id");
    if (!cap) return NULL;
    if (fsim_vehicle_capability_status(self->world, id, cap, &availability, &reason) != FSIM_OK) return fail();
    return Py_BuildValue("(ii)", availability, reason);
}

/* capability_status_info(id, capability) -> (availability, reason, reasons, description, associated, next_available_s,
 * [(parameter, min, max)]) */
PyObject* world_capability_status_info(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    fsim_capability_status s;
    fsim_parameter_limit limits[8];
    uint32_t count = 0;
    if (!check_args(n, 2, 2, "capability_status_info") || !as_u32(args[0], &id)) return NULL;
    const char* cap = as_str(args[1], "capability id");
    if (!cap) return NULL;
    fsim_capability_status_init(&s);
    if (fsim_vehicle_capability_status_info(self->world, id, cap, &s) != FSIM_OK) return fail();
    if (fsim_vehicle_capability_limits(self->world, id, cap, limits, 8, &count) != FSIM_OK) return fail();
    PyObject* ranges = PyList_New(0);
    for (uint32_t i = 0; ranges && i < count && i < 8; ++i) {
        PyObject* t = Py_BuildValue("(Idd)", limits[i].parameter, limits[i].min, limits[i].max);
        if (!t || PyList_Append(ranges, t) < 0) {
            Py_XDECREF(t);
            Py_CLEAR(ranges);
            break;
        }
        Py_DECREF(t);
    }
    if (!ranges) return NULL;
    return Py_BuildValue("(iiKsKdN)", s.availability, s.reason, (unsigned long long)s.reasons, s.description ? s.description : "",
                         (unsigned long long)s.associated, s.next_available_s, ranges);
}

/* support(id, feature) -> (feature, support, rules, stage, capability, missing, evidence) */
PyObject* world_support(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    fsim_support_info s;
    if (!check_args(n, 2, 2, "support") || !as_u32(args[0], &id)) return NULL;
    const char* feature = as_str(args[1], "feature id");
    if (!feature) return NULL;
    fsim_support_info_init(&s);
    if (fsim_vehicle_support(self->world, id, feature, &s) != FSIM_OK) return fail();
    return Py_BuildValue("(siIIsss)", s.feature, s.support, s.rules, s.stage, s.capability, s.missing, s.evidence);
}

/* set_availability_ex(id, capability, availability, reason, associated, next_available_s) */
PyObject* world_set_availability_ex(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    int availability, reason;
    uint64_t associated;
    if (!check_args(n, 6, 6, "set_availability_ex") || !as_u32(args[0], &id) || !as_int(args[2], &availability) || !as_int(args[3], &reason) ||
        !as_u64(args[4], &associated) || !WORLD_IDLE(self))
        return NULL;
    const char* cap = as_str(args[1], "capability id");
    if (!cap) return NULL;
    const double next = PyFloat_AsDouble(args[5]);
    if (next == -1.0 && PyErr_Occurred()) return NULL;
    if (fsim_vehicle_set_availability_ex(self->world, id, cap, availability, reason, associated, next) != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

/* performance(id) -> (revision, hovers, 26 floats in fsim_performance's order) */
PyObject* world_performance(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    fsim_performance p;
    if (!check_args(n, 1, 1, "performance") || !as_u32(args[0], &id)) return NULL;
    fsim_performance_init(&p);
    if (fsim_vehicle_performance(self->world, id, &p) != FSIM_OK) return fail();
    return Py_BuildValue("(Iidddddddddddddddddddddddddd)", p.revision, p.hovers, p.min_cas_ms, p.max_cas_ms, p.max_mach, p.max_tas_ms, p.cruise_tas_ms,
                         p.max_ground_speed_ms, p.ceiling_m, p.max_bank_rad, p.min_pitch_rad, p.max_pitch_rad, p.max_roll_rate_rad_s, p.min_load_factor,
                         p.max_load_factor, p.max_tilt_rad, p.max_acceleration_ms2, p.max_deceleration_ms2, p.max_climb_ms, p.max_descent_ms,
                         p.altitude_gain_per_s, p.heading_gain, p.heading_reference_tas_ms, p.bank_rate_rad_s, p.velocity_bandwidth_rad_s,
                         p.least_cas_ms, p.load_speed_ratio, p.load_kg);
}

/* control_revision(id) -> int */
PyObject* world_control_revision(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    uint32_t id, revision = 0;
    if (!check_args(n, 1, 1, "control_revision") || !as_u32(args[0], &id)) return NULL;
    if (fsim_vehicle_control_revision(((WorldObject*)o)->world, id, &revision) != FSIM_OK) return fail();
    return PyLong_FromUnsignedLong(revision);
}

/* set_control_mode(id, mode) */
PyObject* world_set_control_mode(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    int mode;
    if (!check_args(n, 2, 2, "set_control_mode") || !as_u32(args[0], &id) || !as_int(args[1], &mode) || !WORLD_IDLE(self)) return NULL;
    if (fsim_vehicle_set_control_mode(self->world, id, mode) != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

/* control_mode(id) -> mode */
PyObject* world_control_mode(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    uint32_t id;
    int32_t mode = 0;
    if (!check_args(n, 1, 1, "control_mode") || !as_u32(args[0], &id)) return NULL;
    if (fsim_vehicle_control_mode(((WorldObject*)o)->world, id, &mode) != FSIM_OK) return fail();
    return PyLong_FromLong(mode);
}

/* request_control(id, capability, controller=0) -> reason (0: granted) */
PyObject* world_request_control(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id, controller = 0;
    int32_t reason = 0;
    if (!check_args(n, 2, 3, "request_control") || !as_u32(args[0], &id) || (n > 2 && !as_u32(args[2], &controller)) || !WORLD_IDLE(self))
        return NULL;
    const char* cap = as_str(args[1], "capability id");
    if (!cap) return NULL;
    if (fsim_vehicle_request_control_by(self->world, id, cap, controller, &reason) != FSIM_OK) return fail();
    return PyLong_FromLong(reason);
}

/* release_control(id, capability, controller=0) -> reason (0: released; not_granted: another controller's) */
PyObject* world_release_control(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id, controller = 0;
    int32_t reason = 0;
    if (!check_args(n, 2, 3, "release_control") || !as_u32(args[0], &id) || (n > 2 && !as_u32(args[2], &controller)) || !WORLD_IDLE(self))
        return NULL;
    const char* cap = as_str(args[1], "capability id");
    if (!cap) return NULL;
    if (fsim_vehicle_release_control_by(self->world, id, cap, controller, &reason) != FSIM_OK) return fail();
    return PyLong_FromLong(reason);
}

/* revoke_control(id, capability, reason) */
PyObject* world_revoke_control(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    int reason;
    if (!check_args(n, 3, 3, "revoke_control") || !as_u32(args[0], &id) || !as_int(args[2], &reason) || !WORLD_IDLE(self)) return NULL;
    const char* cap = as_str(args[1], "capability id");
    if (!cap) return NULL;
    if (fsim_vehicle_revoke_control(self->world, id, cap, reason) != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

/* set_allowed(id, capability, allowed) */
PyObject* world_set_allowed(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    int allowed;
    if (!check_args(n, 3, 3, "set_allowed") || !as_u32(args[0], &id) || !as_int(args[2], &allowed) || !WORLD_IDLE(self)) return NULL;
    const char* cap = as_str(args[1], "capability id");
    if (!cap) return NULL;
    if (fsim_vehicle_set_allowed(self->world, id, cap, allowed) != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

/* control_status(id, capability) -> (allowed, granted, holder) */
PyObject* world_control_status(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    uint32_t id, holder = 0;
    int32_t allowed = 0, granted = 0;
    if (!check_args(n, 2, 2, "control_status") || !as_u32(args[0], &id)) return NULL;
    const char* cap = as_str(args[1], "capability id");
    if (!cap) return NULL;
    if (fsim_vehicle_control_status(((WorldObject*)o)->world, id, cap, &allowed, &granted) != FSIM_OK ||
        fsim_vehicle_control_holder(((WorldObject*)o)->world, id, cap, NULL, &holder) != FSIM_OK)
        return fail();
    return Py_BuildValue("(OOI)", allowed ? Py_True : Py_False, granted ? Py_True : Py_False, holder);
}

/* set_capability_precedence(id, capability, precedence) */
PyObject* world_set_capability_precedence(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id, precedence;
    if (!check_args(n, 3, 3, "set_capability_precedence") || !as_u32(args[0], &id) || !as_u32(args[2], &precedence) || !WORLD_IDLE(self)) return NULL;
    const char* cap = as_str(args[1], "capability id");
    if (!cap) return NULL;
    if (fsim_vehicle_set_capability_precedence(self->world, id, cap, precedence) != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

/* capability_precedence(id, capability) -> precedence */
PyObject* world_capability_precedence(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    uint32_t id, precedence = 0;
    if (!check_args(n, 2, 2, "capability_precedence") || !as_u32(args[0], &id)) return NULL;
    const char* cap = as_str(args[1], "capability id");
    if (!cap) return NULL;
    if (fsim_vehicle_capability_precedence(((WorldObject*)o)->world, id, cap, &precedence) != FSIM_OK) return fail();
    return PyLong_FromUnsignedLong(precedence);
}

/* set_availability(id, capability, availability, reason) */
PyObject* world_set_availability(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    int availability, reason;
    if (!check_args(n, 4, 4, "set_availability") || !as_u32(args[0], &id) || !as_int(args[2], &availability) || !as_int(args[3], &reason) ||
        !WORLD_IDLE(self))
        return NULL;
    const char* cap = as_str(args[1], "capability id");
    if (!cap) return NULL;
    if (fsim_vehicle_set_availability(self->world, id, cap, availability, reason) != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

/* profile_value(id, path) -> float (NaN if unknown) */
PyObject* world_profile_value(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    double value = 0.0;
    if (!check_args(n, 2, 2, "profile_value") || !as_u32(args[0], &id)) return NULL;
    const char* path = as_str(args[1], "profile path");
    if (!path) return NULL;
    if (fsim_vehicle_profile_value(self->world, id, path, &value) != FSIM_OK) return fail();
    return PyFloat_FromDouble(value);
}

/* profile_section(id, section) -> (version, provenance) */
PyObject* world_profile_section(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id, version = 0;
    int32_t provenance = 0;
    if (!check_args(n, 2, 2, "profile_section") || !as_u32(args[0], &id)) return NULL;
    const char* section = as_str(args[1], "section name");
    if (!section) return NULL;
    if (fsim_vehicle_profile_section(self->world, id, section, &version, &provenance) != FSIM_OK) {
        PyErr_Format(PyExc_KeyError, "no vehicle %u or no profile section %s", (unsigned)id, section);
        return NULL;
    }
    return Py_BuildValue("(Ii)", version, provenance);
}

/* set_vehicle_default(id, mode) -> reason (0: set) */
PyObject* world_set_vehicle_default(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    int mode;
    int32_t reason = 0;
    if (!check_args(n, 2, 2, "set_vehicle_default") || !as_u32(args[0], &id) || !as_int(args[1], &mode) || !WORLD_IDLE(self)) return NULL;
    if (fsim_vehicle_set_default(self->world, id, mode, &reason) != FSIM_OK && reason == 0) return fail();
    return PyLong_FromLong(reason);
}

/* vehicle_default(id) -> mode */
PyObject* world_vehicle_default(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    uint32_t id;
    int32_t mode = 0;
    if (!check_args(n, 1, 1, "vehicle_default") || !as_u32(args[0], &id)) return NULL;
    if (fsim_vehicle_get_default(((WorldObject*)o)->world, id, &mode) != FSIM_OK) return fail();
    return PyLong_FromLong(mode);
}

/* set_protection(id, mode) */
PyObject* world_set_protection(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    int mode;
    if (!check_args(n, 2, 2, "set_protection") || !as_u32(args[0], &id) || !as_int(args[1], &mode) || !WORLD_IDLE(self)) return NULL;
    if (fsim_vehicle_set_protection(self->world, id, mode) != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

/* protection(id) -> mode */
PyObject* world_protection(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    uint32_t id;
    int32_t mode = 0;
    if (!check_args(n, 1, 1, "protection") || !as_u32(args[0], &id)) return NULL;
    if (fsim_vehicle_get_protection(((WorldObject*)o)->world, id, &mode) != FSIM_OK) return fail();
    return PyLong_FromLong(mode);
}

/* envelope(id) -> (mode, [(limited_updates, exceeded_updates, exceeded_s, worst_excess)] per limit) */
PyObject* world_envelope(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    fsim_envelope_status s;
    if (!check_args(n, 1, 1, "envelope") || !as_u32(args[0], &id) || !WORLD_IDLE(self)) return NULL;
    if (fsim_vehicle_envelope(self->world, id, &s) != FSIM_OK) return fail();
    PyObject* rows = PyList_New(FSIM_LIMIT_COUNT);
    for (int l = 0; rows && l < FSIM_LIMIT_COUNT; ++l) {
        PyObject* t = Py_BuildValue("(IIdd)", s.limited_updates[l], s.exceeded_updates[l], s.exceeded_s[l], s.worst_excess[l]);
        if (!t || PyList_SetItem(rows, l, t) < 0) { /* SetItem takes t, even when it fails */
            Py_CLEAR(rows);
            break;
        }
    }
    return rows ? Py_BuildValue("(iN)", s.mode, rows) : NULL;
}

PyObject* world_active_level(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    uint32_t id;
    if (!check_args(n, 1, 1, "active_level") || !as_u32(args[0], &id)) return NULL;
    return PyLong_FromLong(fsim_vehicle_active_level(((WorldObject*)o)->world, id));
}

PyObject* world_behavior_finished(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    uint32_t id;
    if (!check_args(n, 1, 1, "behavior_finished") || !as_u32(args[0], &id)) return NULL;
    return PyBool_FromLong(fsim_vehicle_behavior_finished(((WorldObject*)o)->world, id) == 1);
}

PyObject* world_use_controller(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    int level;
    if (!check_args(n, 3, 3, "use_controller") || !as_u32(args[0], &id) || !as_int(args[1], &level) || !WORLD_IDLE(self)) return NULL;
    const char* cid = as_str(args[2], "controller id");
    if (!cid) return NULL;
    if (fsim_vehicle_use_controller(self->world, id, level, cid) != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

PyObject* world_set_controller_parameter(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    int level;
    if (!check_args(n, 4, 4, "set_controller_parameter") || !as_u32(args[0], &id) || !as_int(args[1], &level) || !WORLD_IDLE(self))
        return NULL;
    const char* name = as_str(args[2], "parameter name");
    const double value = name ? PyFloat_AsDouble(args[3]) : 0.0;
    if (!name || (value == -1.0 && PyErr_Occurred())) return NULL;
    if (fsim_vehicle_set_controller_parameter(self->world, id, level, name, value) != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

PyObject* world_controller_parameter(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    int level;
    if (!check_args(n, 3, 3, "controller_parameter") || !as_u32(args[0], &id) || !as_int(args[1], &level) || !WORLD_IDLE(self))
        return NULL;
    const char* name = as_str(args[2], "parameter name");
    double v = 0.0;
    if (!name) return NULL;
    if (fsim_vehicle_controller_parameter(self->world, id, level, name, &v) != FSIM_OK) return fail();
    return PyFloat_FromDouble(v);
}

/* last_findings() -> [(reason, index, constraint, from, to, associated, description)]: every reason the last command
 * cannot be flown as asked (ABI 1.8), the answer's the first */
PyObject* world_last_findings(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    (void)args;
    if (!check_args(n, 0, 0, "last_findings")) return NULL;
    fsim_command_detail d;
    fsim_command_detail_init(&d);
    fsim_last_command_detail(self->world, &d);
    PyObject* list = PyList_New(0);
    for (uint32_t i = 0; list && i < d.finding_count; ++i) {
        fsim_command_finding f;
        fsim_command_finding_init(&f);
        if (fsim_last_command_finding(self->world, i, &f) != FSIM_OK) break; /* (past those kept) */
        PyObject* t = Py_BuildValue("(iiiddKs)", f.reason, f.index, f.constraint, f.from, f.to, (unsigned long long)f.associated,
                                    f.description ? f.description : "");
        if (!t || PyList_Append(list, t) < 0) {
            Py_XDECREF(t);
            Py_DECREF(list);
            return NULL;
        }
        Py_DECREF(t);
    }
    return list;
}

/* last_endurance() -> (energy, remaining, required, remaining_s, required_s) or None: what the last command's flight
 * needs against what the vehicle has above its reserve, where it needs more (ABI 1.16) */
PyObject* world_last_endurance(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    (void)args;
    if (!check_args(n, 0, 0, "last_endurance")) return NULL;
    fsim_command_endurance e;
    fsim_command_endurance_init(&e);
    if (fsim_last_command_endurance(self->world, &e) != FSIM_OK || e.energy == 0) Py_RETURN_NONE;
    return Py_BuildValue("(idddd)", e.energy, e.remaining, e.required, e.remaining_s, e.required_s);
}

/* last_terrain() -> (index, latitude_rad, longitude_rad, altitude_msl_m, ground_m, time_s) or None: where the last
 * command's path first goes below the terrain (ABI 1.17) */
PyObject* world_last_terrain(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    (void)args;
    if (!check_args(n, 0, 0, "last_terrain")) return NULL;
    fsim_command_terrain t;
    fsim_command_terrain_init(&t);
    if (fsim_last_command_terrain(self->world, &t) != FSIM_OK || !t.hit) Py_RETURN_NONE;
    return Py_BuildValue("(iddddd)", t.index, t.latitude_rad, t.longitude_rad, t.altitude_msl_m, t.ground_m, t.time_s);
}

/* last_adjustments() -> [(index, field, constraint, requested, adjusted)]: every value the last command is flown
 * with other than asked (ABI 1.8) */
PyObject* world_last_adjustments(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    (void)args;
    if (!check_args(n, 0, 0, "last_adjustments")) return NULL;
    fsim_command_detail d;
    fsim_command_detail_init(&d);
    fsim_last_command_detail(self->world, &d);
    PyObject* list = PyList_New(0);
    for (uint32_t i = 0; list && i < d.adjustment_count; ++i) {
        fsim_command_adjustment a;
        fsim_command_adjustment_init(&a);
        if (fsim_last_command_adjustment(self->world, i, &a) != FSIM_OK) break;
        PyObject* t = Py_BuildValue("(iiidd)", a.index, a.field, a.constraint, a.requested, a.adjusted);
        if (!t || PyList_Append(list, t) < 0) {
            Py_XDECREF(t);
            Py_DECREF(list);
            return NULL;
        }
        Py_DECREF(t);
    }
    return list;
}

/* One batch item's storage: its values, behaviour, waypoints, segments and options. */
typedef struct {
    double row[FSIM_PY_VALUES];
    fsim_py_params params;
    fsim_position_command* points;
    fsim_waypoint* waypoints;
    fsim_bezier_segment* segments;
    fsim_nurbs_segment* nurbs;
    fsim_route_loiter* loiters;
    fsim_route_state* states;
    fsim_route_path* paths;
    fsim_route_branch* branches;
    fsim_route_terminator* terminators;
    fsim_behavior_command behavior;
    fsim_command_options options;
} BatchItem;

static void batch_free(BatchItem* items, Py_ssize_t count) {
    for (Py_ssize_t i = 0; i < count; ++i) {
        fsim_py_params_free(&items[i].params);
        PyMem_Free(items[i].points);
        PyMem_Free(items[i].waypoints);
        PyMem_Free(items[i].segments);
        PyMem_Free(items[i].nurbs);
        PyMem_Free(items[i].loiters);
        PyMem_Free(items[i].states);
        PyMem_Free(items[i].paths);
        PyMem_Free(items[i].branches);
        PyMem_Free(items[i].terminators);
    }
    PyMem_Free(items);
}

/* One batch item (kind, code, values, behavior, waypoints, segments, options[, loiters[, states[, paths[, branches[,
 * terminators]]]]]) into `it` and `b`: kind fsim_batch_kind, behavior (id, target, params, points) or None, options (source,
 * axes, range, min_version, envelope) or None, a route's loiters (ABI 1.28: read_loiters's rows), states (ABI 1.31:
 * read_states's), paths (ABI 1.33: read_paths's), branches (ABI 1.34: read_branches's) and terminators (ABI 1.35:
 * read_terminators's) or None. 0 with a Python error if malformed. */
static int read_batch_item(PyObject* item, BatchItem* it, fsim_batch_command* b) {
    b->struct_size = sizeof *b;
    PyObject* part[12] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};
    const Py_ssize_t parts = item && PySequence_Check(item) ? PySequence_Size(item) : -1;
    int ok = parts >= 7 && parts <= 12;
    if (item && !ok && !PyErr_Occurred())
        PyErr_SetString(PyExc_ValueError, "each item must be (kind, code, values, behavior, waypoints, segments, options[, loiters[, states[, paths[, "
                                          "branches[, terminators]]]]])");
    for (Py_ssize_t k = 0; ok && k < parts; ++k) ok = (part[k] = PySequence_GetItem(item, k)) != NULL;
    if (ok) ok = as_int(part[0], &b->kind) && as_int(part[1], &b->code);
    if (ok) {
        const Py_ssize_t values = read_values(part[2], it->row, "a batch item");
        ok = values >= 0;
        b->fields = it->row, b->count = (uint32_t)(values > 0 ? values : 0);
    }
    if (ok && part[3] != Py_None) { /* a behaviour: (id, target, params, points) */
        PyObject* bp[4] = {NULL, NULL, NULL, NULL};
        ok = PySequence_Check(part[3]) && PySequence_Size(part[3]) == 4;
        if (!ok && !PyErr_Occurred()) PyErr_SetString(PyExc_ValueError, "a behaviour must be (id, target, params, points)");
        for (Py_ssize_t k = 0; ok && k < 4; ++k) ok = (bp[k] = PySequence_GetItem(part[3], k)) != NULL;
        if (ok) ok = (it->behavior.id = as_str(bp[0], "behaviour id")) != NULL && as_u32(bp[1], &it->behavior.target);
        if (ok) ok = fsim_py_params_read(bp[2], &it->params) >= 0;
        if (ok && bp[3] != Py_None) ok = read_points(bp[3], &it->points, &it->behavior.point_count) >= 0;
        it->behavior.param_names = it->params.names, it->behavior.param_values = it->params.values, it->behavior.param_count = it->params.count;
        it->behavior.points = it->points;
        b->behavior = &it->behavior;
        for (Py_ssize_t k = 0; k < 4; ++k) Py_XDECREF(bp[k]); /* (the behaviour's id stays alive in the caller's item) */
    }
    if (ok && part[4] != Py_None) {
        const Py_ssize_t np = read_waypoints(part[4], &it->waypoints);
        ok = np >= 0;
        b->waypoints = it->waypoints, b->waypoint_count = (uint32_t)(np > 0 ? np : 0);
    }
    if (ok && part[5] != Py_None && b->kind == FSIM_BATCH_NURBS) { /* (ABI 1.24: read_nurbs's rows) */
        const Py_ssize_t ns = read_nurbs(part[5], &it->nurbs);
        ok = ns >= 0;
        b->nurbs = it->nurbs, b->segment_count = (uint32_t)(ns > 0 ? ns : 0);
    } else if (ok && part[5] != Py_None) {
        const Py_ssize_t ns = read_segments(part[5], &it->segments);
        ok = ns >= 0;
        b->segments = it->segments, b->segment_count = (uint32_t)(ns > 0 ? ns : 0);
    }
    if (ok) { /* options: (source, axes, range, min_version, envelope) */
        fsim_command_options_init(&it->options);
        if (part[6] != Py_None) {
            PyObject* op[5] = {NULL, NULL, NULL, NULL, NULL};
            const Py_ssize_t size = PySequence_Check(part[6]) ? PySequence_Size(part[6]) : -1;
            ok = size >= 0 && size <= 5;
            if (!ok && !PyErr_Occurred()) PyErr_SetString(PyExc_ValueError, "options must be (source, axes, range, min_version, envelope)");
            for (Py_ssize_t k = 0; ok && k < size; ++k) ok = (op[k] = PySequence_GetItem(part[6], k)) != NULL;
            if (ok) ok = read_options(op, size, 0, &it->options);
            for (Py_ssize_t k = 0; k < 5; ++k) Py_XDECREF(op[k]);
        }
        b->options = &it->options;
    }
    if (ok && part[7] && part[7] != Py_None) {
        const Py_ssize_t nl = read_loiters(part[7], &it->loiters);
        ok = nl >= 0;
        b->loiters = it->loiters, b->loiter_count = (uint32_t)(nl > 0 ? nl : 0);
    }
    if (ok && part[8] && part[8] != Py_None) {
        const Py_ssize_t ns = read_states(part[8], &it->states);
        ok = ns >= 0;
        b->states = it->states, b->state_count = (uint32_t)(ns > 0 ? ns : 0);
    }
    if (ok && part[9] && part[9] != Py_None) {
        const Py_ssize_t nr = read_paths(part[9], &it->paths);
        ok = nr >= 0;
        b->paths = it->paths, b->path_count = (uint32_t)(nr > 0 ? nr : 0);
    }
    if (ok && part[10] && part[10] != Py_None) {
        const Py_ssize_t nb = read_branches(part[10], &it->branches);
        ok = nb >= 0;
        b->branches = it->branches, b->branch_count = (uint32_t)(nb > 0 ? nb : 0);
    }
    if (ok && part[11] && part[11] != Py_None) {
        const Py_ssize_t nt = read_terminators(part[11], &it->terminators);
        ok = nt >= 0;
        b->terminators = it->terminators, b->terminator_count = (uint32_t)(nt > 0 ? nt : 0);
    }
    for (Py_ssize_t k = 0; k < 12; ++k) Py_XDECREF(part[k]);
    return ok;
}

/* submit_batch(id, items) -> [result]: several NEWs at once (ABI 1.8), each item as read_batch_item reads it, each
 * answered on its own */
PyObject* world_submit_batch(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    if (!check_args(n, 2, 2, "submit_batch") || !as_u32(args[0], &id) || !WORLD_IDLE(self)) return NULL;
    const Py_ssize_t count = PySequence_Size(args[1]);
    if (count < 0) return NULL;
    BatchItem* items = (BatchItem*)PyMem_Calloc((size_t)(count ? count : 1), sizeof(BatchItem));
    fsim_batch_command* batch = (fsim_batch_command*)PyMem_Calloc((size_t)(count ? count : 1), sizeof(fsim_batch_command));
    fsim_command_result* results = (fsim_command_result*)PyMem_Calloc((size_t)(count ? count : 1), sizeof(fsim_command_result));
    fsim_command_detail* details = (fsim_command_detail*)PyMem_Calloc((size_t)(count ? count : 1), sizeof(fsim_command_detail));
    for (Py_ssize_t i = 0; details && i < (count ? count : 1); ++i) fsim_command_detail_init(&details[i]);
    int ok = items && batch && results && details;
    if (!ok) PyErr_NoMemory();
    for (Py_ssize_t i = 0; ok && i < count; ++i) {
        PyObject* item = PySequence_GetItem(args[1], i);
        ok = read_batch_item(item, &items[i], &batch[i]);
        Py_XDECREF(item); /* (alive in the caller's list) */
    }
    PyObject* out = NULL;
    if (ok) {
        if (fsim_vehicle_submit_batch(self->world, id, batch, (uint32_t)count, results, details) != FSIM_OK) {
            fail();
        } else {
            out = PyList_New(count);
            for (Py_ssize_t i = 0; out && i < count; ++i) {
                PyObject* t = result_tuple_with(&results[i], &details[i]);
                if (!t || PyList_SetItem(out, i, t) < 0) Py_CLEAR(out);
            }
        }
    }
    if (items) batch_free(items, count);
    PyMem_Free(batch);
    PyMem_Free(results);
    PyMem_Free(details);
    return out;
}

/* (task_id, state, reason, suggested, activity, run, runs, percent, start_time, end_time, command_id) */
static PyObject* task_tuple(const fsim_task_status* t) {
    return Py_BuildValue("(KiiOKIIdddK)", (unsigned long long)t->task_id, t->state, t->reason, t->suggested ? Py_True : Py_False,
                         (unsigned long long)t->activity, t->run, t->runs, t->percent, t->start_time, t->end_time, (unsigned long long)t->command_id);
}

/* store_task(id, task_id, item, attempts, interval_s) -> reason: a task kept (ABI 1.11), its command a batch item */
PyObject* world_store_task(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id, attempts = 1;
    uint64_t task;
    if (!check_args(n, 5, 5, "store_task") || !as_u32(args[0], &id) || !as_u64(args[1], &task) || !as_u32(args[3], &attempts) || !WORLD_IDLE(self))
        return NULL;
    const double interval = PyFloat_AsDouble(args[4]);
    if (PyErr_Occurred()) return NULL;
    BatchItem* it = (BatchItem*)PyMem_Calloc(1, sizeof(BatchItem));
    fsim_batch_command b;
    memset(&b, 0, sizeof b);
    if (!it) return PyErr_NoMemory();
    PyObject* out = NULL;
    int32_t reason = 0;
    if (read_batch_item(args[2], it, &b)) {
        if (fsim_vehicle_store_task(self->world, id, task, &b, attempts, interval, &reason) != FSIM_OK) fail();
        else out = PyLong_FromLong(reason);
    }
    batch_free(it, 1);
    return out;
}

/* command_task(id, task_id, source, axes, range, min_version, envelope) -> result */
PyObject* world_command_task(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    uint64_t task;
    fsim_command_options opt;
    fsim_command_result r;
    if (!check_args(n, 2, 7, "command_task") || !as_u32(args[0], &id) || !as_u64(args[1], &task) || !read_options(args, n, 2, &opt) ||
        !WORLD_IDLE(self))
        return NULL;
    if (fsim_vehicle_command_task(self->world, id, task, &opt, &r) != FSIM_OK) return fail();
    return result_tuple(self->world, &r);
}

/* cancel_task(id, task_id, source=0) -> result */
PyObject* world_cancel_task(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    uint64_t task;
    int source = 0;
    uint32_t controller = 0;
    fsim_command_result r;
    if (!check_args(n, 2, 4, "cancel_task") || !as_u32(args[0], &id) || !as_u64(args[1], &task) || (n > 2 && !as_int(args[2], &source)) ||
        (n > 3 && !as_u32(args[3], &controller)) || !WORLD_IDLE(self))
        return NULL;
    if (fsim_vehicle_cancel_task(self->world, id, task, source, controller, &r) != FSIM_OK) return fail();
    return result_tuple(self->world, &r);
}

/* remove_task(id, task_id) -> reason */
PyObject* world_remove_task(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    uint64_t task;
    int32_t reason = 0;
    if (!check_args(n, 2, 2, "remove_task") || !as_u32(args[0], &id) || !as_u64(args[1], &task) || !WORLD_IDLE(self)) return NULL;
    if (fsim_vehicle_remove_task(self->world, id, task, &reason) != FSIM_OK) return fail();
    return PyLong_FromLong(reason);
}

/* task_status(id, task_id) -> status tuple, or None for one not kept */
PyObject* world_task_status(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    uint64_t task;
    fsim_task_status t;
    if (!check_args(n, 2, 2, "task_status") || !as_u32(args[0], &id) || !as_u64(args[1], &task)) return NULL;
    fsim_task_status_init(&t);
    if (fsim_vehicle_task_status(self->world, id, task, &t) != FSIM_OK) Py_RETURN_NONE;
    return task_tuple(&t);
}

/* tasks(id) -> [status tuple] */
PyObject* world_tasks(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    if (!check_args(n, 1, 1, "tasks") || !as_u32(args[0], &id)) return NULL;
    const uint32_t count = fsim_vehicle_task_count(self->world, id);
    PyObject* out = PyList_New(0);
    for (uint32_t i = 0; out && i < count; ++i) {
        fsim_task_status t;
        fsim_task_status_init(&t);
        if (fsim_vehicle_task_at(self->world, id, i, &t) != FSIM_OK) continue;
        PyObject* tt = task_tuple(&t);
        if (!tt || PyList_Append(out, tt) < 0) Py_CLEAR(out);
        Py_XDECREF(tt);
    }
    return out;
}

/* --- Route plans (ABI 1.36; docs/flight-autonomy.md, 4.39) --- */

/* (completed, plan_id, command, state, reason, check): `check` the validation's or the NEW's result tuple (a preparation
 * for activation's, an activation's), else None */
static PyObject* plan_result_tuple(const fsim_world* world, const fsim_plan_command_result* r) {
    PyObject* check = r->command == FSIM_PLAN_PREPARE_FOR_ACTIVATION || r->command == FSIM_PLAN_ACTIVATE ? result_tuple(world, &r->check)
                                                                                                          : Py_NewRef(Py_None);
    if (!check) return NULL;
    return Py_BuildValue("(iKiiiN)", r->completed, (unsigned long long)r->plan_id, r->command, r->state, r->reason, check);
}

/* (plan_id, state, version, revision, execution, reason, for_planning_use_only, activity, percent, start_time, end_time,
 * command_id) */
static PyObject* plan_status_tuple(const fsim_plan_status* s) {
    return Py_BuildValue("(KiIIiiOKdddKO)", (unsigned long long)s->plan_id, s->state, s->version, s->revision, s->execution, s->reason,
                         s->for_planning_use_only ? Py_True : Py_False, (unsigned long long)s->activity, s->percent, s->start_time, s->end_time,
                         (unsigned long long)s->command_id, s->fa_owned ? Py_True : Py_False);
}

/* A text: a str, or None (NULL); its UTF-8, the object's while it lives (the platform refuses what is not printable ASCII). */
static int text_of(PyObject* o, const char** out) {
    *out = NULL;
    if (o == Py_None) return 1;
    return (*out = as_str(o, "a text")) != NULL;
}

/* publish_plan(id, plan_id, version, for_planning_use_only, detailed, remarks_name, remarks, item, points, paths) -> reason:
 * `item` a route's batch item; `points` [(point, source, locked, modified, remarks_name, remarks, fix_key, fix_system)];
 * `paths` [(path, a state's row of 30 numbers (its point not used), endurance_s, fuel_kg, gross_weight_kg, transition_plan,
 * airfield, runway)]. load_plan takes the same: FA's own (ABI 1.37). */
/* A validation's inputs: (wind_north_ms, wind_east_ms, gust_ms, origin_latitude_rad, origin_longitude_rad, origin_altitude_m,
 * modify_to_validate, parts), or None - as the vehicle is now (ABI 1.38). */
static int read_validation(PyObject* t, fsim_plan_validation* v) {
    fsim_plan_validation_init(v);
    if (t == Py_None) return 1;
    if (!PyTuple_Check(t) || PyTuple_Size(t) != 8) {
        PyErr_SetString(PyExc_ValueError, "a validation must be (wind_north_ms, wind_east_ms, gust_ms, origin_latitude_rad, "
                                          "origin_longitude_rad, origin_altitude_m, modify_to_validate, parts)");
        return 0;
    }
    double* f[6] = {&v->wind_north_ms, &v->wind_east_ms, &v->gust_ms, &v->origin_latitude_rad, &v->origin_longitude_rad, &v->origin_altitude_m};
    for (Py_ssize_t k = 0; k < 6; ++k) {
        *f[k] = PyFloat_AsDouble(PyTuple_GetItem(t, k));
        if (PyErr_Occurred()) return 0;
    }
    v->modify_to_validate = PyObject_IsTrue(PyTuple_GetItem(t, 6)) == 1;
    return as_u32(PyTuple_GetItem(t, 7), &v->parts);
}

/* A validation's answer: (valid, the check's result tuple) */
static PyObject* validation_tuple(const fsim_world* world, const fsim_plan_validation_result* r) {
    PyObject* check = result_tuple(world, &r->check);
    return check ? Py_BuildValue("(ON)", r->valid ? Py_True : Py_False, check) : NULL;
}

/* mode 0 publish_plan, 1 load_plan, 2 validate_plan (its 11th argument the validation) */
static PyObject* plan_call(PyObject* o, PyObject* const* args, Py_ssize_t n, int load) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id, version;
    uint64_t plan_id;
    fsim_route_plan plan;
    fsim_plan_validation validation;
    const Py_ssize_t count = load == 2 ? 11 : 10;
    if (!check_args(n, count, count, load == 2 ? "validate_plan" : load ? "load_plan" : "publish_plan") || !as_u32(args[0], &id) ||
        !as_u64(args[1], &plan_id) || !as_u32(args[2], &version) || !WORLD_IDLE(self) || (load == 2 && !read_validation(args[10], &validation)))
        return NULL;
    fsim_route_plan_init(&plan);
    plan.plan_id = plan_id, plan.version = version;
    plan.for_planning_use_only = PyObject_IsTrue(args[3]) == 1, plan.detailed = PyObject_IsTrue(args[4]) == 1;
    if (!text_of(args[5], &plan.remarks_name) || !text_of(args[6], &plan.remarks)) return NULL;
    /* (the rows held by these tuples, their texts by the rows: they live until the call is done) */
    PyObject* points = PySequence_Tuple(args[8]);
    PyObject* paths = points ? PySequence_Tuple(args[9]) : NULL;
    const Py_ssize_t np = paths ? PyTuple_Size(points) : 0, nq = paths ? PyTuple_Size(paths) : 0;
    BatchItem* it = paths ? (BatchItem*)PyMem_Calloc(1, sizeof(BatchItem)) : NULL;
    fsim_point_metadata* pm = it ? (fsim_point_metadata*)PyMem_Calloc((size_t)(np ? np : 1), sizeof(fsim_point_metadata)) : NULL;
    fsim_path_metadata* qm = pm ? (fsim_path_metadata*)PyMem_Calloc((size_t)(nq ? nq : 1), sizeof(fsim_path_metadata)) : NULL;
    PyObject* out = NULL;
    int ok = qm != NULL;
    if (!ok && !PyErr_Occurred()) PyErr_NoMemory();
    for (Py_ssize_t i = 0; ok && i < np; ++i) {
        PyObject* row = PyTuple_GetItem(points, i);
        fsim_point_metadata* m = &pm[i];
        fsim_point_metadata_init(m);
        ok = row && PyTuple_Check(row) && PyTuple_Size(row) == 8;
        if (!ok && !PyErr_Occurred())
            PyErr_SetString(PyExc_ValueError, "each point's metadata must be (point, source, locked, modified, remarks_name, remarks, fix_key, fix_system)");
        if (ok) ok = as_u32(PyTuple_GetItem(row, 0), &m->point) && as_int(PyTuple_GetItem(row, 1), &m->source);
        if (ok) m->locked = PyObject_IsTrue(PyTuple_GetItem(row, 2)) == 1, m->modified = PyObject_IsTrue(PyTuple_GetItem(row, 3)) == 1;
        if (ok) ok = text_of(PyTuple_GetItem(row, 4), &m->remarks_name) && text_of(PyTuple_GetItem(row, 5), &m->remarks) &&
                     text_of(PyTuple_GetItem(row, 6), &m->fix_key) && text_of(PyTuple_GetItem(row, 7), &m->fix_system);
    }
    for (Py_ssize_t i = 0; ok && i < nq; ++i) {
        PyObject* row = PyTuple_GetItem(paths, i);
        fsim_path_metadata* m = &qm[i];
        fsim_path_metadata_init(m);
        ok = row && PyTuple_Check(row) && PyTuple_Size(row) == 8;
        if (!ok && !PyErr_Occurred())
            PyErr_SetString(PyExc_ValueError, "each path's metadata must be (path, initial state row, endurance_s, fuel_kg, gross_weight_kg, "
                                              "transition_plan, airfield, runway)");
        if (ok) ok = as_u32(PyTuple_GetItem(row, 0), &m->path) && as_u64(PyTuple_GetItem(row, 5), &m->transition_plan) &&
                     as_u64(PyTuple_GetItem(row, 6), &m->airfield) && as_u64(PyTuple_GetItem(row, 7), &m->runway);
        if (ok) {
            PyObject* one = PyTuple_Pack(1, PyTuple_GetItem(row, 1));
            fsim_route_state* initial = NULL;
            ok = one && read_states(one, &initial) == 1;
            if (ok) m->initial = initial[0];
            PyMem_Free(initial);
            Py_XDECREF(one);
        }
        if (ok) {
            m->endurance_s = PyFloat_AsDouble(PyTuple_GetItem(row, 2));
            m->fuel_kg = PyFloat_AsDouble(PyTuple_GetItem(row, 3));
            m->gross_weight_kg = PyFloat_AsDouble(PyTuple_GetItem(row, 4));
            ok = !PyErr_Occurred();
        }
    }
    if (ok) ok = read_batch_item(args[7], it, &plan.route);
    if (ok) {
        int32_t reason = 0;
        plan.points = np ? pm : NULL, plan.point_count = (uint32_t)np;
        plan.paths = nq ? qm : NULL, plan.path_count = (uint32_t)nq;
        if (load == 2) {
            fsim_plan_validation_result r;
            fsim_plan_validation_result_init(&r);
            if (fsim_vehicle_validate_plan(self->world, id, &plan, &validation, &r) != FSIM_OK) fail();
            else out = validation_tuple(self->world, &r);
        } else {
            const int status = load ? fsim_vehicle_load_plan(self->world, id, &plan, &reason) : fsim_vehicle_publish_plan(self->world, id, &plan, &reason);
            if (status != FSIM_OK) fail();
            else out = PyLong_FromLong(reason);
        }
    }
    if (it) batch_free(it, 1);
    PyMem_Free(pm);
    PyMem_Free(qm);
    Py_XDECREF(points);
    Py_XDECREF(paths);
    return out;
}

PyObject* world_publish_plan(PyObject* o, PyObject* const* args, Py_ssize_t n) { return plan_call(o, args, n, 0); }

/* validate_plan(...publish_plan's, validation) -> (valid, result): a route plan validated without flying it (ABI 1.38) */
PyObject* world_validate_plan(PyObject* o, PyObject* const* args, Py_ssize_t n) { return plan_call(o, args, n, 2); }

/* validate_stored_plan(id, plan_id, validation) -> (valid, result): a kept plan's */
PyObject* world_validate_stored_plan(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    uint64_t plan_id;
    fsim_plan_validation validation;
    fsim_plan_validation_result r;
    if (!check_args(n, 3, 3, "validate_stored_plan") || !as_u32(args[0], &id) || !as_u64(args[1], &plan_id) || !WORLD_IDLE(self) ||
        !read_validation(args[2], &validation))
        return NULL;
    fsim_plan_validation_result_init(&r);
    if (fsim_vehicle_validate_stored_plan(self->world, id, plan_id, &validation, &r) != FSIM_OK) return fail();
    return validation_tuple(self->world, &r);
}

/* load_plan(...) -> reason: FA's own plan, the platform's (ABI 1.37), its arguments publish_plan's */
PyObject* world_load_plan(PyObject* o, PyObject* const* args, Py_ssize_t n) { return plan_call(o, args, n, 1); }

/* A runway point: 4 numbers (latitude_rad, longitude_rad, altitude_m, altitude_reference). */
static int read_runway_point(PyObject* seq, Py_ssize_t at, fsim_runway_point* q) {
    double* f[4] = {&q->latitude_rad, &q->longitude_rad, &q->altitude_m, &q->altitude_reference};
    for (Py_ssize_t k = 0; k < 4; ++k) {
        PyObject* v = PyTuple_GetItem(seq, at + k);
        if (!v) return 0;
        *f[k] = PyFloat_AsDouble(v);
        if (PyErr_Occurred()) return 0;
    }
    return 1;
}

/* load_airfield(id, airfield_id, icao, qnh_pa, runways) -> reason (ABI 1.37): `runways` [(runway_id, direction_rad,
 * available_length_m, then the takeoff's start, threshold and limit and the landing's, each 4 numbers: 27 in all)] */
PyObject* world_load_airfield(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    uint64_t airfield_id;
    fsim_airfield field;
    if (!check_args(n, 5, 5, "load_airfield") || !as_u32(args[0], &id) || !as_u64(args[1], &airfield_id) || !WORLD_IDLE(self)) return NULL;
    fsim_airfield_init(&field);
    field.airfield_id = airfield_id;
    if (!text_of(args[2], &field.icao)) return NULL;
    field.qnh_pa = PyFloat_AsDouble(args[3]);
    if (PyErr_Occurred()) return NULL;
    PyObject* rows = PySequence_Tuple(args[4]);
    if (!rows) return NULL;
    const Py_ssize_t count = PyTuple_Size(rows);
    fsim_runway* runways = (fsim_runway*)PyMem_Calloc((size_t)(count ? count : 1), sizeof(fsim_runway));
    int ok = runways != NULL;
    if (!ok) PyErr_NoMemory();
    for (Py_ssize_t i = 0; ok && i < count; ++i) {
        PyObject* row = PyTuple_GetItem(rows, i);
        fsim_runway* r = &runways[i];
        fsim_runway_init(r);
        ok = row && PyTuple_Check(row) && PyTuple_Size(row) == 27;
        if (!ok && !PyErr_Occurred()) PyErr_SetString(PyExc_ValueError, "each runway must be 27 numbers: id, direction, length, then 6 points of 4");
        if (ok) ok = as_u64(PyTuple_GetItem(row, 0), &r->runway_id);
        if (ok) {
            r->direction_rad = PyFloat_AsDouble(PyTuple_GetItem(row, 1));
            r->available_length_m = PyFloat_AsDouble(PyTuple_GetItem(row, 2));
            ok = !PyErr_Occurred();
        }
        fsim_runway_point* points[6] = {&r->takeoff.start, &r->takeoff.threshold, &r->takeoff.limit, &r->landing.start, &r->landing.threshold, &r->landing.limit};
        for (Py_ssize_t k = 0; ok && k < 6; ++k) ok = read_runway_point(row, 3 + 4 * k, points[k]);
    }
    PyObject* out = NULL;
    if (ok) {
        int32_t reason = 0;
        field.runways = count ? runways : NULL, field.runway_count = (uint32_t)count;
        if (fsim_vehicle_load_airfield(self->world, id, &field, &reason) != FSIM_OK) fail();
        else out = PyLong_FromLong(reason);
    }
    PyMem_Free(runways);
    Py_DECREF(rows);
    return out;
}

/* An airfield read back: (airfield_id, icao, qnh_pa, revision, [runway rows as load_airfield takes them]) */
static PyObject* airfield_tuple(const fsim_airfield* a) {
    PyObject* runways = PyList_New(0);
    for (uint32_t i = 0; runways && i < a->runway_count; ++i) {
        const fsim_runway* r = &a->runways[i];
        const fsim_runway_point* q[6] = {&r->takeoff.start, &r->takeoff.threshold, &r->takeoff.limit, &r->landing.start, &r->landing.threshold, &r->landing.limit};
        PyObject* row = PyTuple_New(27);
        int ok = row != NULL;
        PyObject* rid = ok ? PyLong_FromUnsignedLongLong((unsigned long long)r->runway_id) : NULL;
        ok = ok && rid && PyTuple_SetItem(row, 0, rid) == 0;
        double values[26];
        values[0] = r->direction_rad, values[1] = r->available_length_m;
        for (int k = 0; k < 6; ++k)
            values[2 + 4 * k] = q[k]->latitude_rad, values[3 + 4 * k] = q[k]->longitude_rad, values[4 + 4 * k] = q[k]->altitude_m,
                      values[5 + 4 * k] = q[k]->altitude_reference;
        for (Py_ssize_t k = 0; ok && k < 26; ++k) {
            PyObject* v = PyFloat_FromDouble(values[k]);
            ok = v && PyTuple_SetItem(row, k + 1, v) == 0; /* (SetItem takes the item, even when it fails) */
        }
        if (!ok || PyList_Append(runways, row) < 0) Py_CLEAR(runways);
        Py_XDECREF(row);
    }
    if (!runways) return NULL;
    return Py_BuildValue("(KsdIN)", (unsigned long long)a->airfield_id, a->icao ? a->icao : "", a->qnh_pa, a->revision, runways);
}

/* airfields(id) -> [airfield tuple]; get_airfield(id, airfield_id) -> airfield tuple or None (ABI 1.37) */
PyObject* world_airfields(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    if (!check_args(n, 1, 1, "airfields") || !as_u32(args[0], &id)) return NULL;
    const uint32_t count = fsim_vehicle_airfield_count(self->world, id);
    PyObject* out = PyList_New(0);
    for (uint32_t i = 0; out && i < count; ++i) {
        fsim_airfield a;
        fsim_airfield_init(&a);
        if (fsim_vehicle_get_airfield_at(self->world, id, i, &a) != FSIM_OK) continue;
        PyObject* t = airfield_tuple(&a);
        if (!t || PyList_Append(out, t) < 0) Py_CLEAR(out);
        Py_XDECREF(t);
    }
    return out;
}

PyObject* world_get_airfield(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    uint64_t airfield_id;
    fsim_airfield a;
    if (!check_args(n, 2, 2, "get_airfield") || !as_u32(args[0], &id) || !as_u64(args[1], &airfield_id)) return NULL;
    fsim_airfield_init(&a);
    if (fsim_vehicle_get_airfield(self->world, id, airfield_id, &a) != FSIM_OK) Py_RETURN_NONE;
    return airfield_tuple(&a);
}

/* plan_command(id, plan_id, command, source, axes, range, min_version, envelope) -> plan result tuple */
PyObject* world_plan_command(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    uint64_t plan_id;
    int command = 0;
    fsim_command_options opt;
    fsim_plan_command_result r;
    if (!check_args(n, 3, 8, "plan_command") || !as_u32(args[0], &id) || !as_u64(args[1], &plan_id) || !as_int(args[2], &command) ||
        !read_options(args, n, 3, &opt) || !WORLD_IDLE(self))
        return NULL;
    fsim_plan_command_result_init(&r);
    if (fsim_vehicle_plan_command(self->world, id, plan_id, command, &opt, &r) != FSIM_OK) return fail();
    return plan_result_tuple(self->world, &r);
}

/* abort_plan(id, plan_id, reason) -> plan result tuple: FA's own deactivation (0: restricted) */
PyObject* world_abort_plan(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    uint64_t plan_id;
    int reason = 0;
    fsim_plan_command_result r;
    if (!check_args(n, 2, 3, "abort_plan") || !as_u32(args[0], &id) || !as_u64(args[1], &plan_id) || (n > 2 && !as_int(args[2], &reason)) ||
        !WORLD_IDLE(self))
        return NULL;
    fsim_plan_command_result_init(&r);
    if (fsim_vehicle_abort_plan(self->world, id, plan_id, reason, &r) != FSIM_OK) return fail();
    return plan_result_tuple(self->world, &r);
}

/* remove_plan(id, plan_id) -> reason */
PyObject* world_remove_plan(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    uint64_t plan_id;
    int32_t reason = 0;
    if (!check_args(n, 2, 2, "remove_plan") || !as_u32(args[0], &id) || !as_u64(args[1], &plan_id) || !WORLD_IDLE(self)) return NULL;
    if (fsim_vehicle_remove_plan(self->world, id, plan_id, &reason) != FSIM_OK) return fail();
    return PyLong_FromLong(reason);
}

/* plan_status(id, plan_id) -> status tuple, or None for one not kept */
PyObject* world_plan_status(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    uint64_t plan_id;
    fsim_plan_status s;
    if (!check_args(n, 2, 2, "plan_status") || !as_u32(args[0], &id) || !as_u64(args[1], &plan_id)) return NULL;
    fsim_plan_status_init(&s);
    if (fsim_vehicle_plan_status(self->world, id, plan_id, &s) != FSIM_OK) Py_RETURN_NONE;
    return plan_status_tuple(&s);
}

/* A route intercept's segment (ABI 1.44): (path_id, point, point_id, capture_time_s, capture_distance_m, loiter, orbits,
 * loiter_end_s, heading_rad, along_ms, across_ms), or None where it has none */
static PyObject* segment_tuple(int has, const fsim_segment_status* g) {
    if (!has) Py_RETURN_NONE;
    return Py_BuildValue("(KiKddOIdddd)", (unsigned long long)g->path_id, g->point, (unsigned long long)g->point_id, g->capture_time_s,
                         g->capture_distance_m, g->loiter ? Py_True : Py_False, g->orbits, g->loiter_end_s, g->heading_rad, g->along_ms, g->across_ms);
}

/* intercept_status(activity) -> (plan_id, execution, joined, laid, join_latitude_rad, join_longitude_rad, previous, current, next),
 * or None for an activity that is no route intercept's (ABI 1.44) */
PyObject* world_intercept_status(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint64_t activity;
    fsim_intercept_status s;
    if (!check_args(n, 1, 1, "intercept_status") || !as_u64(args[0], &activity)) return NULL;
    fsim_intercept_status_init(&s);
    if (fsim_activity_intercept_status(self->world, (fsim_activity_id)activity, &s) != FSIM_OK) Py_RETURN_NONE;
    PyObject* previous = segment_tuple(s.has_previous, &s.previous);
    PyObject* current = previous ? segment_tuple(s.has_current, &s.current) : NULL;
    PyObject* next = current ? segment_tuple(s.has_next, &s.next) : NULL;
    if (!next) {
        Py_XDECREF(previous);
        Py_XDECREF(current);
        return NULL;
    }
    return Py_BuildValue("(KiiiddNNN)", (unsigned long long)s.plan_id, s.execution, s.joined, s.laid, s.join_latitude_rad, s.join_longitude_rad,
                         previous, current, next);
}

/* plans(id) -> [status tuple] */
PyObject* world_plans(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    if (!check_args(n, 1, 1, "plans") || !as_u32(args[0], &id)) return NULL;
    const uint32_t count = fsim_vehicle_plan_count(self->world, id);
    PyObject* out = PyList_New(0);
    for (uint32_t i = 0; out && i < count; ++i) {
        fsim_plan_status s;
        fsim_plan_status_init(&s);
        if (fsim_vehicle_plan_at(self->world, id, i, &s) != FSIM_OK) continue;
        PyObject* st = plan_status_tuple(&s);
        if (!st || PyList_Append(out, st) < 0) Py_CLEAR(out);
        Py_XDECREF(st);
    }
    return out;
}

/* get_plan(id, plan_id) -> (plan_id, version, for_planning_use_only, detailed, remarks_name, remarks, its route as
 * activity_setpoint's tuple, [point metadata rows], [path metadata rows]) as publish_plan takes them; None for one not
 * uploaded */
PyObject* world_get_plan(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    uint64_t plan_id;
    fsim_route_plan p;
    if (!check_args(n, 2, 2, "get_plan") || !as_u32(args[0], &id) || !as_u64(args[1], &plan_id)) return NULL;
    fsim_route_plan_init(&p);
    if (fsim_vehicle_get_plan(self->world, id, plan_id, &p) != FSIM_OK) Py_RETURN_NONE;
    PyObject* route = batch_tuple(&p.route);
    PyObject* points = route ? PyList_New(0) : NULL;
    for (uint32_t i = 0; points && i < p.point_count; ++i) {
        const fsim_point_metadata* m = &p.points[i];
        PyObject* row = Py_BuildValue("(IiOOssss)", m->point, m->source, m->locked ? Py_True : Py_False, m->modified ? Py_True : Py_False,
                                      m->remarks_name, m->remarks, m->fix_key, m->fix_system);
        if (!row || PyList_Append(points, row) < 0) Py_CLEAR(points);
        Py_XDECREF(row);
    }
    PyObject* paths = points ? PyList_New(0) : NULL;
    for (uint32_t i = 0; paths && i < p.path_count; ++i) {
        const fsim_path_metadata* m = &p.paths[i];
        PyObject* initial = state_row(&m->initial);
        PyObject* row = initial ? Py_BuildValue("(INdddKKK)", m->path, initial, m->endurance_s, m->fuel_kg, m->gross_weight_kg,
                                                (unsigned long long)m->transition_plan, (unsigned long long)m->airfield,
                                                (unsigned long long)m->runway)
                                : NULL;
        if (!row || PyList_Append(paths, row) < 0) Py_CLEAR(paths);
        Py_XDECREF(row);
    }
    if (!paths) {
        Py_XDECREF(route), Py_XDECREF(points);
        return NULL;
    }
    return Py_BuildValue("(KIOOssNNN)", (unsigned long long)p.plan_id, p.version, p.for_planning_use_only ? Py_True : Py_False,
                         p.detailed ? Py_True : Py_False, p.remarks_name, p.remarks, route, points, paths);
}

PyObject* mod_reason_name(PyObject* m, PyObject* const* args, Py_ssize_t n) {
    int code;
    (void)m;
    if (!check_args(n, 1, 1, "reason_name") || !as_int(args[0], &code)) return NULL;
    return PyUnicode_FromString(fsim_reason_name(code));
}

PyObject* mod_activity_state_name(PyObject* m, PyObject* const* args, Py_ssize_t n) {
    int code;
    (void)m;
    if (!check_args(n, 1, 1, "activity_state_name") || !as_int(args[0], &code)) return NULL;
    return PyUnicode_FromString(fsim_activity_state_name(code));
}

PyObject* mod_limit_name(PyObject* m, PyObject* const* args, Py_ssize_t n) {
    int code;
    (void)m;
    if (!check_args(n, 1, 1, "limit_name") || !as_int(args[0], &code)) return NULL;
    return PyUnicode_FromString(fsim_limit_name(code));
}

PyObject* mod_constraint_name(PyObject* m, PyObject* const* args, Py_ssize_t n) {
    int code;
    (void)m;
    if (!check_args(n, 1, 1, "constraint_name") || !as_int(args[0], &code)) return NULL;
    return PyUnicode_FromString(fsim_constraint_name(code));
}

PyObject* mod_support_features(PyObject* m, PyObject* const* args, Py_ssize_t n) {
    (void)m;
    (void)args;
    if (!check_args(n, 0, 0, "support_features")) return NULL;
    const uint32_t count = fsim_support_feature_count();
    PyObject* t = PyTuple_New(count);
    for (uint32_t i = 0; t && i < count; ++i) {
        PyObject* s = PyUnicode_FromString(fsim_support_feature(i));
        if (!s) {
            Py_CLEAR(t);
            break;
        }
        PyTuple_SetItem(t, i, s); /* steals */
    }
    return t;
}

#define NAME_FUNCTION(fn, call, what)                                                  \
    PyObject* fn(PyObject* m, PyObject* const* args, Py_ssize_t n) {             \
        int code;                                                                      \
        (void)m;                                                                       \
        if (!check_args(n, 1, 1, what) || !as_int(args[0], &code)) return NULL;      \
        return PyUnicode_FromString(call(code));                                       \
    }
NAME_FUNCTION(mod_support_name, fsim_support_name, "support_name")
NAME_FUNCTION(mod_rule_name, fsim_rule_name, "rule_name")
NAME_FUNCTION(mod_rule_description, fsim_rule_description, "rule_description")
NAME_FUNCTION(mod_availability_name, fsim_availability_name, "availability_name")
NAME_FUNCTION(mod_reason_description, fsim_reason_description, "reason_description")
#undef NAME_FUNCTION

PyObject* mod_flight_mode_name(PyObject* m, PyObject* const* args, Py_ssize_t n) {
    int code;
    (void)m;
    if (!check_args(n, 1, 1, "flight_mode_name") || !as_int(args[0], &code)) return NULL;
    return PyUnicode_FromString(fsim_flight_mode_name(code));
}

PyObject* mod_command_field_count(PyObject* m, PyObject* const* args, Py_ssize_t n) {
    int level;
    (void)m;
    if (!check_args(n, 1, 1, "command_field_count") || !as_int(args[0], &level)) return NULL;
    return PyLong_FromUnsignedLong(fsim_command_field_count(level));
}

PyObject* mod_command_field_count_full(PyObject* m, PyObject* const* args, Py_ssize_t n) {
    int level;
    (void)m;
    if (!check_args(n, 1, 1, "command_field_count_full") || !as_int(args[0], &level)) return NULL;
    return PyLong_FromUnsignedLong(fsim_command_field_count_full(level));
}
