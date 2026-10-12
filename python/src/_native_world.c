/* Native World operations over the C ABI. Registration stays in _native.c. */
#define FSIM_PY_NO_BUFFER_TYPE
#include "_native_world.h"

#define SF(f, k) FSIM_PY_FIELD(fsim_vehicle_spec, f, k)
static const fsim_py_field spec_fields[] = {
    SF(name, FSIM_PY_STR), SF(type, FSIM_PY_STR), SF(latitude_deg, FSIM_PY_F64), SF(longitude_deg, FSIM_PY_F64),
    SF(altitude_msl_m, FSIM_PY_F64), SF(heading_deg, FSIM_PY_F64), SF(pitch_deg, FSIM_PY_F64), SF(roll_deg, FSIM_PY_F64),
    SF(airspeed_ms, FSIM_PY_F64), SF(on_ground, FSIM_PY_I32), SF(model, FSIM_PY_STR), SF(control_divider, FSIM_PY_U32),
    SF(fuel_fraction, FSIM_PY_F64)};
#undef SF
PyObject* world_step(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t steps = 1;
    if (!check_args(n, 0, 1, "step") || (n == 1 && !as_u32(args[0], &steps)) || !WORLD_IDLE(self)) return NULL;
    int rc;
    *self->busy = 1;
    Py_BEGIN_ALLOW_THREADS
    rc = fsim_world_step(self->world, steps);
    Py_END_ALLOW_THREADS
    *self->busy = 0;
    if (rc != FSIM_OK) return fail();
    Py_RETURN_NONE;
}
/* agent_audit() -> [(vehicle, controller, name, [(call, calls, refused), ...]), ...] (the referee's; AB-2d) */
PyObject* world_agent_audit(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    fsim_world* w = ((WorldObject*)o)->world;
    uint32_t count, i, k;
    PyObject* out;
    (void)args;
    if (!check_args(n, 0, 0, "agent_audit")) return NULL;
    count = fsim_world_agent_count(w);
    out = PyList_New(0);
    if (!out) return NULL;
    for (i = 0; i < count; ++i) {
        fsim_agent_audit a;
        fsim_audit_call* calls;
        PyObject *list, *row;
        fsim_agent_audit_init(&a);
        if (fsim_world_agent_audit(w, i, &a, NULL, 0) != FSIM_OK) {
            Py_DECREF(out);
            return fail();
        }
        calls = (fsim_audit_call*)PyMem_Malloc(sizeof(fsim_audit_call) * (a.call_count ? a.call_count : 1u));
        if (!calls) {
            Py_DECREF(out);
            return PyErr_NoMemory();
        }
        if (fsim_world_agent_audit(w, i, &a, calls, a.call_count) != FSIM_OK) {
            PyMem_Free(calls);
            Py_DECREF(out);
            return fail();
        }
        list = PyList_New(0);
        for (k = 0; list && k < a.call_count; ++k) {
            PyObject* t = Py_BuildValue("(sKK)", calls[k].call, (unsigned long long)calls[k].calls, (unsigned long long)calls[k].refused);
            if (!t || PyList_Append(list, t) < 0) {
                Py_XDECREF(t);
                Py_CLEAR(list);
                break;
            }
            Py_DECREF(t);
        }
        PyMem_Free(calls);
        if (!list) {
            Py_DECREF(out);
            return NULL;
        }
        row = Py_BuildValue("(IIsN)", a.vehicle, a.controller, a.name, list);
        if (!row || PyList_Append(out, row) < 0) {
            Py_XDECREF(row);
            Py_DECREF(out);
            return NULL;
        }
        Py_DECREF(row);
    }
    return out;
}

PyObject* world_agent_identity(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    (void)args;
    if (!check_args(n, 0, 0, "agent_identity")) return NULL;
    uint32_t vehicle = 0, controller = 0;
    const char* name = "";
    if (!fsim_agent_identity(((WorldObject*)o)->world, &vehicle, &controller, &name)) Py_RETURN_NONE;
    return Py_BuildValue("(IIs)", vehicle, controller, name);
}

PyObject* world_info(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    (void)args;
    if (!check_args(n, 0, 0, "info")) return NULL;
    return Py_BuildValue("{s:d,s:d,s:K,s:O,s:O,s:I}", "time", fsim_world_time(self->world), "step_seconds",
                         fsim_world_step_seconds(self->world), "vehicle_steps",
                         (unsigned long long)fsim_world_vehicle_steps(self->world), "published",
                         fsim_world_published(self->world) ? Py_True : Py_False, "recording",
                         fsim_world_recording(self->world) ? Py_True : Py_False, "vehicle_count",
                         fsim_world_vehicle_count(self->world));
}

PyObject* world_time(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    (void)args;
    if (!check_args(n, 0, 0, "time")) return NULL;
    return PyFloat_FromDouble(fsim_world_time(((WorldObject*)o)->world));
}
PyObject* world_create_vehicle(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    if (!check_args(n, 1, 1, "create_vehicle") || !WORLD_IDLE(self)) return NULL;
    fsim_vehicle_spec spec;
    fsim_vehicle_spec_init(&spec);
    if (fsim_py_set_fields(args[0], &spec, spec_fields, COUNT(spec_fields), "vehicle") < 0) return NULL;
    uint32_t id = 0;
    int rc;
    *self->busy = 1;
    Py_BEGIN_ALLOW_THREADS
    rc = fsim_world_create_vehicle(self->world, &spec, &id);
    Py_END_ALLOW_THREADS
    *self->busy = 0;
    if (rc != FSIM_OK) return fail();
    return PyLong_FromUnsignedLong(id);
}

PyObject* world_remove_vehicle(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    if (!check_args(n, 1, 1, "remove_vehicle") || !as_u32(args[0], &id) || !WORLD_IDLE(self)) return NULL;
    if (fsim_world_remove_vehicle(self->world, id) != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

/* focus_viewer(id): the viewers follow the vehicle, once (ABI 1.47). */
PyObject* world_focus_viewer(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    if (!check_args(n, 1, 1, "focus_viewer") || !as_u32(args[0], &id)) return NULL;
    if (fsim_world_focus_viewer(self->world, id) != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

/* reset_vehicle(id, spec=None): spec's initial-state fields, or the vehicle's own. */
PyObject* world_reset_vehicle(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    if (!check_args(n, 1, 2, "reset_vehicle") || !as_u32(args[0], &id) || !WORLD_IDLE(self)) return NULL;
    fsim_vehicle_spec spec;
    const fsim_vehicle_spec* p = NULL;
    if (n == 2 && args[1] != Py_None) {
        fsim_vehicle_spec_init(&spec);
        if (fsim_py_set_fields(args[1], &spec, spec_fields, COUNT(spec_fields), "vehicle") < 0) return NULL;
        p = &spec;
    }
    int rc;
    *self->busy = 1;
    Py_BEGIN_ALLOW_THREADS
    rc = fsim_world_reset_vehicle(self->world, id, p);
    Py_END_ALLOW_THREADS
    *self->busy = 0;
    if (rc != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

PyObject* world_find_vehicle(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    if (!check_args(n, 1, 1, "find_vehicle")) return NULL;
    const char* name = as_str(args[0], "name");
    if (!name) return NULL;
    return PyLong_FromUnsignedLong(fsim_world_find_vehicle(((WorldObject*)o)->world, name));
}

PyObject* world_vehicle_ids(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    (void)args;
    if (!check_args(n, 0, 0, "vehicle_ids")) return NULL;
    const uint32_t count = fsim_world_vehicle_ids(self->world, NULL, 0);
    PyObject* bytes = PyBytes_FromStringAndSize(NULL, (Py_ssize_t)count * 4);
    if (!bytes) return NULL;
    fsim_world_vehicle_ids(self->world, (uint32_t*)PyBytes_AsString(bytes), count);
    return bytes;
}

PyObject* world_vehicle_info(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    if (!check_args(n, 1, 1, "vehicle_info") || !as_u32(args[0], &id)) return NULL;
    return Py_BuildValue("(ss)", fsim_vehicle_name(self->world, id), fsim_vehicle_type(self->world, id));
}
/* gather_states(ids uint32[n], out bytes[n * sizeof(state)], sensed) */
PyObject* world_gather_states(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    int sensed = 0;
    if (!check_args(n, 2, 3, "gather_states") || (n == 3 && !as_int(args[2], &sensed)) || !WORLD_IDLE(self)) return NULL;
    Py_buffer ids, out;
    if (!get_array(args[0], &ids, 0, "IL", 4, "gather_states ids")) return NULL;
    if (PyObject_GetBuffer(args[1], &out, PyBUF_C_CONTIGUOUS | PyBUF_WRITABLE) < 0) {
        PyBuffer_Release(&ids);
        PyErr_Clear();
        PyErr_SetString(PyExc_TypeError, "gather_states: out must be a writable C-contiguous array");
        return NULL;
    }
    const Py_ssize_t count = ids.len / 4;
    if (out.len != count * (Py_ssize_t)sizeof(fsim_vehicle_state)) {
        PyErr_Format(PyExc_ValueError, "gather_states: out holds %zd bytes, %zd vehicles need %zd", out.len, count,
                     count * (Py_ssize_t)sizeof(fsim_vehicle_state));
        PyBuffer_Release(&ids);
        PyBuffer_Release(&out);
        return NULL;
    }
    const int rc = fsim_world_gather_states(self->world, (const uint32_t*)ids.buf, (uint32_t)count, sensed, (fsim_vehicle_state*)out.buf);
    PyBuffer_Release(&ids);
    PyBuffer_Release(&out);
    if (rc != FSIM_OK) return fail();
    Py_RETURN_NONE;
}
PyObject* world_get_property(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    if (!check_args(n, 2, 2, "get_property") || !as_u32(args[0], &id) || !WORLD_IDLE(self)) return NULL;
    const char* path = as_str(args[1], "property path");
    double v = 0.0;
    if (!path) return NULL;
    if (fsim_vehicle_get_property(self->world, id, path, &v) != FSIM_OK) return fail();
    return PyFloat_FromDouble(v);
}

PyObject* world_set_property(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    if (!check_args(n, 3, 3, "set_property") || !as_u32(args[0], &id) || !WORLD_IDLE(self)) return NULL;
    const char* path = as_str(args[1], "property path");
    const double v = path ? PyFloat_AsDouble(args[2]) : 0.0;
    if (!path || (v == -1.0 && PyErr_Occurred())) return NULL;
    if (fsim_vehicle_set_property(self->world, id, path, v) != FSIM_OK) return fail();
    Py_RETURN_NONE;
}
PyObject* world_comm_create_node(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t address;
    if (!check_args(n, 1, 1, "comm_create_node") || !as_u32(args[0], &address) || !WORLD_IDLE(self)) return NULL;
    if (fsim_comm_create_node(self->world, address) != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

/* comm_send(from, to, channel, format, data: bytes-like) */
PyObject* world_comm_send(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t from, to, channel, format;
    if (!check_args(n, 5, 5, "comm_send") || !as_u32(args[0], &from) || !as_u32(args[1], &to) || !as_u32(args[2], &channel) ||
        !as_u32(args[3], &format) || !WORLD_IDLE(self))
        return NULL;
    Py_buffer data;
    if (PyObject_GetBuffer(args[4], &data, PyBUF_C_CONTIGUOUS) < 0) return NULL;
    const int rc = fsim_comm_send(self->world, from, to, channel, format, data.buf, (size_t)data.len);
    PyBuffer_Release(&data);
    if (rc != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

/* comm_inbox(node) -> [(from, to, channel, format, time_sent, time_delivered, bytes)] */
PyObject* world_comm_inbox(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t node;
    if (!check_args(n, 1, 1, "comm_inbox") || !as_u32(args[0], &node)) return NULL;
    const uint32_t count = fsim_comm_inbox_count(self->world, node);
    PyObject* list = PyList_New((Py_ssize_t)count);
    if (!list) return NULL;
    for (uint32_t i = 0; i < count; ++i) {
        fsim_message m;
        memset(&m, 0, sizeof m);
        if (fsim_comm_inbox_get(self->world, node, i, &m) != FSIM_OK) {
            Py_DECREF(list);
            return fail();
        }
        PyObject* t = Py_BuildValue("(IIIIddy#)", m.from, m.to, m.channel, m.format, m.time_sent, m.time_delivered,
                                    (const char*)m.bytes, (Py_ssize_t)m.length);
        if (!t) {
            Py_DECREF(list);
            return NULL;
        }
        PyList_SetItem(list, (Py_ssize_t)i, t);
    }
    return list;
}

PyObject* world_comm_set_medium(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    if (!check_args(n, 1, 2, "comm_set_medium") || !WORLD_IDLE(self)) return NULL;
    const char* medium = as_str(args[0], "medium id");
    fsim_py_params p;
    if (!medium || fsim_py_params_read(n == 2 ? args[1] : NULL, &p) < 0) {
        if (medium) fsim_py_params_free(&p);
        return NULL;
    }
    const int rc = fsim_comm_set_medium(self->world, medium, p.names, p.values, p.count);
    fsim_py_params_free(&p);
    if (rc != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

PyObject* world_comm_attach_protocol(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t node;
    if (!check_args(n, 2, 3, "comm_attach_protocol") || !as_u32(args[0], &node) || !WORLD_IDLE(self)) return NULL;
    const char* protocol = as_str(args[1], "protocol id");
    fsim_py_params p;
    if (!protocol || fsim_py_params_read(n == 3 ? args[2] : NULL, &p) < 0) {
        if (protocol) fsim_py_params_free(&p);
        return NULL;
    }
    const int rc = fsim_comm_attach_protocol(self->world, node, protocol, p.names, p.values, p.count);
    fsim_py_params_free(&p);
    if (rc != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

PyObject* world_comm_attach_udp_bridge(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t node, local, remote;
    if (!check_args(n, 4, 4, "comm_attach_udp_bridge") || !as_u32(args[0], &node) || !as_u32(args[1], &local) ||
        !as_u32(args[3], &remote) || !WORLD_IDLE(self))
        return NULL;
    const char* host = as_str(args[2], "remote host");
    if (!host) return NULL;
    if (fsim_comm_attach_udp_bridge(self->world, node, (uint16_t)local, host, (uint16_t)remote) != FSIM_OK) return fail();
    Py_RETURN_NONE;
}
PyObject* world_capsule(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    (void)args;
    if (!check_args(n, 0, 0, "_capsule")) return NULL;
    return PyCapsule_New(&((WorldObject*)o)->ref, FSIM_PY_WORLD_CAPSULE, NULL);
}

PyObject* world_borrowed_flag(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    (void)args;
    if (!check_args(n, 0, 0, "borrowed")) return NULL;
    return PyBool_FromLong(((WorldObject*)o)->owner != NULL);
}
