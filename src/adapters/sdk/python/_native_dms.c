/* fsim._native's DMS/C2 adapter over the C ABI (ADR-31, AB-4/AB-5).
 * World/module registration and shared error ownership remain in _native.c. */
#define FSIM_PY_NO_BUFFER_TYPE
#include "_native_world.h"

/* --- Systems, nets and the DMS (ABI 1.61; ADR-31, AB-4) ------------------------------------------------------------ */

/* create_c2_node(name, latitude_rad, longitude_rad, altitude_msl_m) -> node id */
PyObject* world_create_c2_node(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    if (!check_args(n, 4, 4, "create_c2_node") || !WORLD_IDLE(self)) return NULL;
    const char* name = as_str(args[0], "name");
    if (!name) return NULL;
    fsim_c2_node_spec s;
    fsim_c2_node_spec_init(&s);
    s.name = name;
    s.latitude_rad = PyFloat_AsDouble(args[1]);
    s.longitude_rad = PyFloat_AsDouble(args[2]);
    s.altitude_msl_m = PyFloat_AsDouble(args[3]);
    if (PyErr_Occurred()) return NULL;
    uint32_t id = 0;
    if (fsim_world_create_c2_node(self->world, &s, &id) != FSIM_OK) return fail();
    return PyLong_FromUnsignedLong(id);
}

PyObject* world_remove_c2_node(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    if (!check_args(n, 1, 1, "remove_c2_node") || !as_u32(args[0], &id) || !WORLD_IDLE(self)) return NULL;
    return PyBool_FromLong(fsim_world_remove_c2_node(self->world, id) == FSIM_OK);
}

/* create_net(name, {spec field: number}) -> net id */
PyObject* world_create_net(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    if (!check_args(n, 1, 2, "create_net") || !WORLD_IDLE(self)) return NULL;
    const char* name = as_str(args[0], "name");
    fsim_py_params p;
    if (!name || fsim_py_params_read(n == 2 ? args[1] : NULL, &p) < 0) {
        if (name) fsim_py_params_free(&p);
        return NULL;
    }
    fsim_net_spec s;
    fsim_net_spec_init(&s);
    s.name = name;
    for (uint32_t i = 0; i < p.count; ++i) {
        const char* k = p.names[i];
        const double v = p.values[i];
        if (!strcmp(k, "range_m")) s.range_m = v;
        else if (!strcmp(k, "frequency_hz")) s.frequency_hz = v;
        else if (!strcmp(k, "line_of_sight")) s.line_of_sight = v != 0.0;
        else if (!strcmp(k, "latency_s")) s.latency_s = v;
        else if (!strcmp(k, "jitter_s")) s.jitter_s = v;
        else if (!strcmp(k, "loss_probability")) s.loss_probability = v;
        else if (!strcmp(k, "data_rate_bps")) s.data_rate_bps = v;
        else if (!strcmp(k, "max_queue_s")) s.max_queue_s = v;
        else if (!strcmp(k, "reliable_timeout_s")) s.reliable_timeout_s = v;
        else if (!strcmp(k, "retry_s")) s.retry_s = v;
        else if (!strcmp(k, "max_payload_bytes")) s.max_payload_bytes = (uint32_t)v;
        else if (!strcmp(k, "header_bytes")) s.header_bytes = (uint32_t)v;
        else {
            PyErr_Format(PyExc_ValueError, "create_net: no net field '%s'", k);
            fsim_py_params_free(&p);
            return NULL;
        }
    }
    uint32_t id = 0;
    const int rc = fsim_world_create_net(self->world, &s, &id);
    fsim_py_params_free(&p);
    if (rc != FSIM_OK) return fail();
    return PyLong_FromUnsignedLong(id);
}

/* join_net(entity, net), leave_net(entity, net) -> bool */
PyObject* world_join_net(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity, net;
    if (!check_args(n, 2, 2, "join_net") || !as_u32(args[0], &entity) || !as_u32(args[1], &net) || !WORLD_IDLE(self)) return NULL;
    return PyBool_FromLong(fsim_world_join_net(self->world, entity, net) == FSIM_OK);
}
PyObject* world_leave_net(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity, net;
    if (!check_args(n, 2, 2, "leave_net") || !as_u32(args[0], &entity) || !as_u32(args[1], &net) || !WORLD_IDLE(self)) return NULL;
    return PyBool_FromLong(fsim_world_leave_net(self->world, entity, net) == FSIM_OK);
}

/* assign_system(entity, system, tail_number) -> SystemID (0: refused) */
PyObject* world_assign_system(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity, system, out = 0;
    if (!check_args(n, 3, 3, "assign_system") || !as_u32(args[0], &entity) || !as_u32(args[1], &system) || !WORLD_IDLE(self)) return NULL;
    const char* tail = as_str(args[2], "tail number");
    if (!tail) return NULL;
    fsim_world_assign_system(self->world, entity, system, tail, &out);
    PyErr_Clear();
    return PyLong_FromUnsignedLong(out);
}

PyObject* world_dms_system(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity;
    if (!check_args(n, 1, 1, "dms_system") || !as_u32(args[0], &entity)) return NULL;
    return PyLong_FromUnsignedLong(fsim_dms_system(self->world, entity));
}

/* dms_nets(entity) -> [(net id, name)] */
PyObject* world_dms_nets(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity;
    if (!check_args(n, 1, 1, "dms_nets") || !as_u32(args[0], &entity)) return NULL;
    const uint32_t count = fsim_dms_net_count(self->world, entity);
    PyObject* list = PyList_New((Py_ssize_t)count);
    if (!list) return NULL;
    for (uint32_t i = 0; i < count; ++i) {
        uint32_t id = 0;
        fsim_net_spec s;
        fsim_net_spec_init(&s);
        if (fsim_dms_net(self->world, entity, i, &id, &s) != FSIM_OK) {
            Py_DECREF(list);
            return fail();
        }
        PyList_SetItem(list, (Py_ssize_t)i, Py_BuildValue("(Is)", id, s.name ? s.name : ""));
    }
    return list;
}

/* dms_send(entity, topic, data, destinations: sequence of SystemIDs) -> message number */
PyObject* world_dms_send(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity, topic;
    if (!check_args(n, 4, 4, "dms_send") || !as_u32(args[0], &entity) || !as_u32(args[1], &topic) || !WORLD_IDLE(self)) return NULL;
    const Py_ssize_t count = PySequence_Size(args[3]);
    if (count < 0) return NULL;
    uint32_t* dest = count ? (uint32_t*)PyMem_Malloc(sizeof(uint32_t) * (size_t)count) : NULL;
    for (Py_ssize_t i = 0; i < count; ++i) {
        PyObject* item = PySequence_GetItem(args[3], i);
        const int ok = item && as_u32(item, &dest[i]);
        Py_XDECREF(item);
        if (!ok) {
            PyMem_Free(dest);
            return NULL;
        }
    }
    Py_buffer data;
    if (PyObject_GetBuffer(args[2], &data, PyBUF_C_CONTIGUOUS) < 0) {
        PyMem_Free(dest);
        return NULL;
    }
    uint64_t message = 0;
    int32_t reason = 0;
    const int rc = fsim_dms_send(self->world, entity, topic, data.buf, (size_t)data.len, dest, (uint32_t)count, &message, &reason);
    PyBuffer_Release(&data);
    PyMem_Free(dest);
    if (rc != FSIM_OK) return fail();
    return PyLong_FromUnsignedLongLong(message);
}

/* dms_inbox_take(entity, count) */
PyObject* world_dms_inbox_take(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity, count;
    if (!check_args(n, 2, 2, "dms_inbox_take") || !as_u32(args[0], &entity) || !as_u32(args[1], &count)) return NULL;
    if (fsim_dms_inbox_take(self->world, entity, count) != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

/* dms_subscribe(entity, topic, on), dms_subscribe_all(entity, on) */
PyObject* world_dms_subscribe(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity, topic;
    if (!check_args(n, 3, 3, "dms_subscribe") || !as_u32(args[0], &entity) || !as_u32(args[1], &topic)) return NULL;
    return PyBool_FromLong(fsim_dms_subscribe(self->world, entity, topic, PyObject_IsTrue(args[2])) == FSIM_OK);
}
PyObject* world_dms_subscribe_all(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity;
    if (!check_args(n, 2, 2, "dms_subscribe_all") || !as_u32(args[0], &entity)) return NULL;
    return PyBool_FromLong(fsim_dms_subscribe_all(self->world, entity, PyObject_IsTrue(args[1])) == FSIM_OK);
}

/* dms_subscriptions(entity) -> ([topics], all) */
PyObject* world_dms_subscriptions(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity;
    if (!check_args(n, 1, 1, "dms_subscriptions") || !as_u32(args[0], &entity)) return NULL;
    int32_t all = 0;
    const uint32_t count = fsim_dms_subscriptions(self->world, entity, NULL, 0, &all);
    uint32_t* topics = count ? (uint32_t*)PyMem_Malloc(sizeof(uint32_t) * count) : NULL;
    fsim_dms_subscriptions(self->world, entity, topics, count, &all);
    PyObject* list = PyList_New((Py_ssize_t)count);
    for (uint32_t i = 0; list && i < count; ++i) PyList_SetItem(list, (Py_ssize_t)i, PyLong_FromUnsignedLong(topics[i]));
    PyMem_Free(topics);
    if (!list) return NULL;
    return Py_BuildValue("(NO)", list, all ? Py_True : Py_False);
}

/* dms_inbox(entity) -> [(message, source, topic, net, time_sent, time_delivered, bytes)] */
PyObject* world_dms_inbox(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity;
    if (!check_args(n, 1, 1, "dms_inbox") || !as_u32(args[0], &entity)) return NULL;
    const uint32_t count = fsim_dms_inbox_count(self->world, entity);
    PyObject* list = PyList_New((Py_ssize_t)count);
    if (!list) return NULL;
    for (uint32_t i = 0; i < count; ++i) {
        fsim_dms_received r;
        memset(&r, 0, sizeof r);
        if (fsim_dms_inbox_get(self->world, entity, i, &r) != FSIM_OK) {
            Py_DECREF(list);
            return fail();
        }
        PyObject* t = Py_BuildValue("(KIIIddy#)", (unsigned long long)r.message, r.source, r.topic, r.net, r.time_sent,
                                    r.time_delivered, (const char*)r.bytes, (Py_ssize_t)r.length);
        if (!t) {
            Py_DECREF(list);
            return NULL;
        }
        PyList_SetItem(list, (Py_ssize_t)i, t);
    }
    return list;
}

/* dms_transmission(entity, message) -> (message, topic, time_sent, published, queue_dropped, reached, [(system, delivery)])
 * or None */
PyObject* world_dms_transmission(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity;
    uint64_t message;
    if (!check_args(n, 2, 2, "dms_transmission") || !as_u32(args[0], &entity) || !as_u64(args[1], &message)) return NULL;
    fsim_dms_transmission t;
    memset(&t, 0, sizeof t);
    if (fsim_dms_transmission_status(self->world, entity, message, &t) != FSIM_OK) {
        PyErr_Clear();
        Py_RETURN_NONE;
    }
    PyObject* dests = PyList_New((Py_ssize_t)t.destination_count);
    if (!dests) return NULL;
    for (uint32_t i = 0; i < t.destination_count; ++i) {
        uint32_t system = 0;
        int32_t delivery = 0;
        fsim_dms_transmission_destination(self->world, entity, message, i, &system, &delivery);
        PyList_SetItem(dests, (Py_ssize_t)i, Py_BuildValue("(Ii)", system, (int)delivery));
    }
    return Py_BuildValue("(KIdOOIN)", (unsigned long long)t.message, t.topic, t.time_sent, t.published ? Py_True : Py_False,
                         t.queue_dropped ? Py_True : Py_False, t.reached, dests);
}

PyObject* world_system_entity(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t system;
    if (!check_args(n, 1, 1, "system_entity") || !as_u32(args[0], &system)) return NULL;
    return PyLong_FromUnsignedLong(fsim_world_system_entity(self->world, system));
}
PyObject* world_entity_system(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity;
    if (!check_args(n, 1, 1, "entity_system") || !as_u32(args[0], &entity)) return NULL;
    return PyLong_FromUnsignedLong(fsim_world_entity_system(self->world, entity));
}
PyObject* world_net_count(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    (void)args;
    if (!check_args(n, 0, 0, "net_count")) return NULL;
    return PyLong_FromUnsignedLong(fsim_world_net_count(self->world));
}
/* net_stats(net) -> (transmissions, delivered, duplicates, out_of_range, masked, lost, queue_dropped, failed) */
PyObject* world_net_stats(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t net;
    if (!check_args(n, 1, 1, "net_stats") || !as_u32(args[0], &net)) return NULL;
    fsim_net_stats s;
    if (fsim_world_net_stats(self->world, net, &s) != FSIM_OK) return fail();
    return Py_BuildValue("(KKKKKKKKK)", (unsigned long long)s.transmissions, (unsigned long long)s.delivered,
                         (unsigned long long)s.duplicates, (unsigned long long)s.out_of_range, (unsigned long long)s.masked,
                         (unsigned long long)s.lost, (unsigned long long)s.queue_dropped, (unsigned long long)s.failed,
                         (unsigned long long)s.overflowed);
}

/* --- tasks, overrides, release consent (ABI 1.64; AB-5b) ------------------------------------------------------------ */

PyObject* world_set_strike_policy(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity, policy;
    if (!check_args(n, 2, 2, "set_strike_policy") || !as_u32(args[0], &entity) || !as_u32(args[1], &policy) || !WORLD_IDLE(self)) return NULL;
    if (fsim_world_set_strike_policy(self->world, entity, (int)policy) != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

PyObject* world_strike_policy(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity;
    if (!check_args(n, 1, 1, "strike_policy") || !as_u32(args[0], &entity)) return NULL;
    return PyLong_FromLong(fsim_world_strike_policy(self->world, entity));
}

static PyObject* c2_task_tuple(const fsim_c2_task* t) {
    char text[65];
    memcpy(text, t->text, 64);
    text[64] = 0;
    return Py_BuildValue("(IdidKIIddddddIIKs)", t->from, t->received_s, (int)t->state, t->percent, (unsigned long long)t->task_id, t->type,
                         t->priority, t->latitude_rad, t->longitude_rad, t->altitude_m, t->radius_m, t->start_s, t->end_s, t->target_source,
                         t->target_payload, (unsigned long long)t->target_track, text);
}

/* agent_c2_tasks(entity) -> [(from, received_s, state, percent, task_id, type, priority, lat, lon, alt, radius, start, end,
 * target_source, target_payload, target_track, text)] */
PyObject* world_agent_c2_tasks(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity;
    if (!check_args(n, 1, 1, "agent_c2_tasks") || !as_u32(args[0], &entity)) return NULL;
    const uint32_t count = fsim_agent_c2_task_count(self->world, entity);
    PyObject* list = PyList_New((Py_ssize_t)count);
    for (uint32_t i = 0; list && i < count; ++i) {
        fsim_c2_task t;
        fsim_c2_task_init(&t);
        if (fsim_agent_c2_task_get(self->world, entity, i, &t) != FSIM_OK) {
            Py_DECREF(list);
            return fail();
        }
        PyList_SetItem(list, (Py_ssize_t)i, c2_task_tuple(&t));
    }
    return list;
}

/* agent_answer_task(entity, from, task_id, state, code, percent) -> bool */
PyObject* world_agent_answer_task(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity, from, state, code;
    uint64_t task;
    if (!check_args(n, 6, 6, "agent_answer_task") || !as_u32(args[0], &entity) || !as_u32(args[1], &from) || !as_u64(args[2], &task) ||
        !as_u32(args[3], &state) || !as_u32(args[4], &code) || !WORLD_IDLE(self))
        return NULL;
    const double percent = PyFloat_AsDouble(args[5]);
    if (PyErr_Occurred()) return NULL;
    const int rc = fsim_agent_answer_task(self->world, entity, from, task, (int)state, code, percent);
    if (rc != FSIM_OK) PyErr_Clear();
    return PyBool_FromLong(rc == FSIM_OK);
}

/* agent_overrides(entity) -> [(from, kind, value, expiry_s)] */
PyObject* world_agent_overrides(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity;
    if (!check_args(n, 1, 1, "agent_overrides") || !as_u32(args[0], &entity)) return NULL;
    const uint32_t count = fsim_agent_override_count(self->world, entity);
    PyObject* list = PyList_New((Py_ssize_t)count);
    for (uint32_t i = 0; list && i < count; ++i) {
        uint32_t from = 0;
        int32_t kind = 0;
        double value = 0, expiry = 0;
        fsim_agent_override_get(self->world, entity, i, &from, &kind, &value, &expiry);
        PyList_SetItem(list, (Py_ssize_t)i, Py_BuildValue("(Iidd)", from, (int)kind, value, expiry));
    }
    return list;
}

/* c2_send_task(entity, ma, type, priority, lat, lon, alt, radius, start, end, target_source, target_payload, target_track, text) */
PyObject* world_c2_send_task(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity, ma, type, priority, source, payload;
    uint64_t track;
    double f[6];
    if (!check_args(n, 14, 14, "c2_send_task") || !as_u32(args[0], &entity) || !as_u32(args[1], &ma) || !as_u32(args[2], &type) ||
        !as_u32(args[3], &priority) || !as_doubles(args, 4, 6, f) || !as_u32(args[10], &source) || !as_u32(args[11], &payload) ||
        !as_u64(args[12], &track) || !WORLD_IDLE(self))
        return NULL;
    const char* text = as_str(args[13], "text");
    if (!text) return NULL;
    fsim_c2_task t;
    fsim_c2_task_init(&t);
    t.type = type, t.priority = priority;
    t.latitude_rad = f[0], t.longitude_rad = f[1], t.altitude_m = f[2], t.radius_m = f[3], t.start_s = f[4], t.end_s = f[5];
    t.target_source = source, t.target_payload = payload, t.target_track = track;
    strncpy(t.text, text, sizeof t.text - 1);
    uint64_t id = 0;
    int32_t reason = 0;
    if (fsim_c2_send_task(self->world, entity, ma, &t, &id, &reason) != FSIM_OK) return fail();
    return PyLong_FromUnsignedLongLong(id);
}

/* c2_set_override(entity, ma, kind, cancel, value, expiry_s) -> command id */
PyObject* world_c2_set_override(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity, ma, kind;
    double f[2];
    if (!check_args(n, 6, 6, "c2_set_override") || !as_u32(args[0], &entity) || !as_u32(args[1], &ma) || !as_u32(args[2], &kind) ||
        !as_doubles(args, 4, 2, f) || !WORLD_IDLE(self))
        return NULL;
    uint64_t id = 0;
    int32_t reason = 0;
    if (fsim_c2_set_override(self->world, entity, ma, (int)kind, PyObject_IsTrue(args[3]), f[0], f[1], &id, &reason) != FSIM_OK) return fail();
    return PyLong_FromUnsignedLongLong(id);
}

/* c2_consents(entity) -> [(from, received_s, answered, request_id, strike, store, quantity, target_source, target_payload,
 * target_track, lat, lon, alt, time_s)] */
PyObject* world_c2_consents(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity;
    if (!check_args(n, 1, 1, "c2_consents") || !as_u32(args[0], &entity)) return NULL;
    const uint32_t count = fsim_c2_consent_count(self->world, entity);
    PyObject* list = PyList_New((Py_ssize_t)count);
    for (uint32_t i = 0; list && i < count; ++i) {
        fsim_consent_request c;
        fsim_consent_request_init(&c);
        if (fsim_c2_consent_get(self->world, entity, i, &c) != FSIM_OK) {
            Py_DECREF(list);
            return fail();
        }
        char store[25];
        memcpy(store, c.store, 24);
        store[24] = 0;
        PyList_SetItem(list, (Py_ssize_t)i, Py_BuildValue("(IdOKKsIIIKdddd)", c.from, c.received_s, c.answered ? Py_True : Py_False,
                                                          (unsigned long long)c.request_id, (unsigned long long)c.strike, store, c.quantity,
                                                          c.target_source, c.target_payload, (unsigned long long)c.target_track,
                                                          c.latitude_rad, c.longitude_rad, c.altitude_m, c.time_s));
    }
    return list;
}

/* c2_answer_consent(entity, ma, request_id, granted) -> bool */
PyObject* world_c2_answer_consent(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity, ma;
    uint64_t request;
    if (!check_args(n, 4, 4, "c2_answer_consent") || !as_u32(args[0], &entity) || !as_u32(args[1], &ma) || !as_u64(args[2], &request) ||
        !WORLD_IDLE(self))
        return NULL;
    int32_t reason = 0;
    const int rc = fsim_c2_answer_consent(self->world, entity, ma, request, PyObject_IsTrue(args[3]), &reason);
    if (rc != FSIM_OK) PyErr_Clear();
    return PyBool_FromLong(rc == FSIM_OK);
}

/* --- C2 authority (ABI 1.63; AB-5a) ------------------------------------------------------------------------------- */

/* set_rbac(entity, [(id, name, permissions, approves_strikes)]) */
PyObject* world_set_rbac(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity;
    if (!check_args(n, 2, 2, "set_rbac") || !as_u32(args[0], &entity) || !WORLD_IDLE(self)) return NULL;
    const Py_ssize_t count = PySequence_Size(args[1]);
    if (count < 0) return NULL;
    fsim_rbac_role* roles = count ? (fsim_rbac_role*)PyMem_Calloc((size_t)count, sizeof(fsim_rbac_role)) : NULL;
    PyObject** keep = count ? (PyObject**)PyMem_Calloc((size_t)count, sizeof(PyObject*)) : NULL;
    int ok = 1;
    for (Py_ssize_t i = 0; ok && i < count; ++i) {
        PyObject* t = PySequence_GetItem(args[1], i);
        keep[i] = t;
        PyObject *a0 = t ? PySequence_GetItem(t, 0) : NULL, *a1 = t ? PySequence_GetItem(t, 1) : NULL,
                 *a2 = t ? PySequence_GetItem(t, 2) : NULL, *a3 = t ? PySequence_GetItem(t, 3) : NULL;
        const char* name = a1 ? as_str(a1, "role name") : NULL;
        ok = t && a0 && a2 && a3 && name && as_u32(a0, &roles[i].id) && as_u32(a2, &roles[i].permissions);
        if (ok) roles[i].name = name, roles[i].approves_strikes = PyObject_IsTrue(a3);
        Py_XDECREF(a0);
        Py_XDECREF(a2);
        Py_XDECREF(a3);
        Py_XDECREF(a1); /* (the name lives in the tuple, kept) */
    }
    const int rc = ok ? fsim_world_set_rbac(self->world, entity, roles, (uint32_t)count) : FSIM_INVALID_ARGUMENT;
    for (Py_ssize_t i = 0; i < count; ++i) Py_XDECREF(keep[i]);
    PyMem_Free(keep);
    PyMem_Free(roles);
    if (!ok) return NULL;
    if (rc != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

/* rbac(entity) -> [(id, name, permissions, approves_strikes)] */
PyObject* world_rbac(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity;
    if (!check_args(n, 1, 1, "rbac") || !as_u32(args[0], &entity)) return NULL;
    const uint32_t count = fsim_world_rbac_role_count(self->world, entity);
    PyObject* list = PyList_New((Py_ssize_t)count);
    for (uint32_t i = 0; list && i < count; ++i) {
        fsim_rbac_role r;
        if (fsim_world_rbac_role(self->world, entity, i, &r) != FSIM_OK) {
            Py_DECREF(list);
            return fail();
        }
        PyList_SetItem(list, (Py_ssize_t)i, Py_BuildValue("(IsIO)", r.id, r.name, r.permissions, r.approves_strikes ? Py_True : Py_False));
    }
    return list;
}

/* agent_authority(entity) -> (designated, modes, by, designated_s, [(system, role_id, role)], [(activity, system, command_id)]) */
PyObject* world_agent_authority(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity;
    if (!check_args(n, 1, 1, "agent_authority") || !as_u32(args[0], &entity)) return NULL;
    fsim_c2_authority a;
    fsim_c2_authority_init(&a);
    if (fsim_agent_authority(self->world, entity, &a) != FSIM_OK) return fail();
    PyObject* ops = PyList_New((Py_ssize_t)a.operator_count);
    for (uint32_t i = 0; ops && i < a.operator_count; ++i) {
        uint32_t system = 0, role = 0;
        fsim_agent_authority_operator(self->world, entity, i, &system, &role);
        PyList_SetItem(ops, (Py_ssize_t)i, Py_BuildValue("(IIs)", system, role, fsim_agent_role_name(self->world, entity, role)));
    }
    PyObject* cmds = PyList_New((Py_ssize_t)a.command_count);
    for (uint32_t i = 0; cmds && i < a.command_count; ++i) {
        uint64_t activity = 0, id = 0;
        uint32_t system = 0;
        fsim_agent_authority_command(self->world, entity, i, &activity, &system, &id);
        PyList_SetItem(cmds, (Py_ssize_t)i, Py_BuildValue("(KIK)", (unsigned long long)activity, system, (unsigned long long)id));
    }
    if (!ops || !cmds) {
        Py_XDECREF(ops);
        Py_XDECREF(cmds);
        return NULL;
    }
    return Py_BuildValue("(OIIdNN)", a.designated ? Py_True : Py_False, a.designated_modes, a.designated_by, a.designated_s, ops, cmds);
}

static int u32_list(PyObject* seq, uint32_t** out, Py_ssize_t* count) {
    *count = PySequence_Size(seq);
    *out = NULL;
    if (*count < 0) return 0;
    *out = *count ? (uint32_t*)PyMem_Malloc(sizeof(uint32_t) * (size_t)*count) : NULL;
    for (Py_ssize_t i = 0; i < *count; ++i) {
        PyObject* item = PySequence_GetItem(seq, i);
        const int ok = item && as_u32(item, &(*out)[i]);
        Py_XDECREF(item);
        if (!ok) {
            PyMem_Free(*out);
            *out = NULL;
            return 0;
        }
    }
    return 1;
}

/* --- lost communication and teams (ABI 1.65; AB-5c) ---------------------------------------------------------------- */

/* agent_comms(entity) -> (lost_comms_set, threshold_s, plan, lost, lost_since_s, alerts, package_id, leader, [members]) */
PyObject* world_agent_comms(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity;
    if (!check_args(n, 1, 1, "agent_comms") || !as_u32(args[0], &entity)) return NULL;
    fsim_c2_authority a;
    fsim_c2_authority_init(&a);
    if (fsim_agent_authority(self->world, entity, &a) != FSIM_OK) return fail();
    PyObject* members = PyList_New((Py_ssize_t)a.member_count);
    for (uint32_t i = 0; members && i < a.member_count; ++i) {
        uint32_t m = 0;
        fsim_agent_package_member(self->world, entity, i, &m);
        PyList_SetItem(members, (Py_ssize_t)i, PyLong_FromUnsignedLong(m));
    }
    if (!members) return NULL;
    return Py_BuildValue("(OdKOdIIIN)", a.lost_comms_set ? Py_True : Py_False, a.lost_comms_threshold_s, (unsigned long long)a.lost_comms_plan,
                         a.lost ? Py_True : Py_False, a.lost_since_s, a.alerts, a.package_id, a.package_leader, members);
}

/* c2_set_lost_comms(entity, ma, threshold_s, plan_id) -> command id */
PyObject* world_c2_set_lost_comms(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity, ma;
    uint64_t plan;
    if (!check_args(n, 4, 4, "c2_set_lost_comms") || !as_u32(args[0], &entity) || !as_u32(args[1], &ma) || !as_u64(args[3], &plan) ||
        !WORLD_IDLE(self))
        return NULL;
    const double threshold = PyFloat_AsDouble(args[2]);
    if (PyErr_Occurred()) return NULL;
    uint64_t id = 0;
    int32_t reason = 0;
    if (fsim_c2_set_lost_comms(self->world, entity, ma, threshold, plan, &id, &reason) != FSIM_OK) return fail();
    return PyLong_FromUnsignedLongLong(id);
}

/* c2_package(entity, package_id, leader, systems, dissolve) -> command id */
PyObject* world_c2_package(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity, package, leader;
    if (!check_args(n, 5, 5, "c2_package") || !as_u32(args[0], &entity) || !as_u32(args[1], &package) || !as_u32(args[2], &leader) ||
        !WORLD_IDLE(self))
        return NULL;
    uint32_t* list = NULL;
    Py_ssize_t count = 0;
    if (!u32_list(args[3], &list, &count)) return NULL;
    uint64_t id = 0;
    int32_t reason = 0;
    const int rc = PyObject_IsTrue(args[4]) ? fsim_c2_dissolve_package(self->world, entity, package, list, (uint32_t)count, &id, &reason)
                                            : fsim_c2_form_package(self->world, entity, package, leader, list, (uint32_t)count, &id, &reason);
    PyMem_Free(list);
    if (rc != FSIM_OK) return fail();
    return PyLong_FromUnsignedLongLong(id);
}

/* c2_declare_role(entity, role_id, callsign, destinations) */
PyObject* world_c2_declare_role(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity, role;
    if (!check_args(n, 4, 4, "c2_declare_role") || !as_u32(args[0], &entity) || !as_u32(args[1], &role) || !WORLD_IDLE(self)) return NULL;
    const char* callsign = as_str(args[2], "callsign");
    uint32_t* dest = NULL;
    Py_ssize_t count = 0;
    if (!callsign || !u32_list(args[3], &dest, &count)) return NULL;
    int32_t reason = 0;
    const int rc = fsim_c2_declare_role(self->world, entity, role, callsign, dest, (uint32_t)count, &reason);
    PyMem_Free(dest);
    if (rc != FSIM_OK) return fail();
    Py_RETURN_TRUE;
}

/* c2_designate_control(entity, ma, modes) -> command id */
PyObject* world_c2_designate_control(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity, ma, modes;
    if (!check_args(n, 3, 3, "c2_designate_control") || !as_u32(args[0], &entity) || !as_u32(args[1], &ma) || !as_u32(args[2], &modes) ||
        !WORLD_IDLE(self))
        return NULL;
    uint64_t id = 0;
    int32_t reason = 0;
    if (fsim_c2_designate_control(self->world, entity, ma, modes, &id, &reason) != FSIM_OK) return fail();
    return PyLong_FromUnsignedLongLong(id);
}

/* c2_command_hsa(entity, ma, heading_rad, course_rad, speed, speed_reference, altitude_m, altitude_reference) -> command id */
PyObject* world_c2_command_hsa(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity, ma;
    double f[6];
    if (!check_args(n, 8, 8, "c2_command_hsa") || !as_u32(args[0], &entity) || !as_u32(args[1], &ma) || !as_doubles(args, 2, 6, f) ||
        !WORLD_IDLE(self))
        return NULL;
    uint64_t id = 0;
    int32_t reason = 0;
    if (fsim_c2_command_hsa(self->world, entity, ma, f, &id, &reason) != FSIM_OK) return fail();
    return PyLong_FromUnsignedLongLong(id);
}

/* c2_command_route(entity, ma, [(lat_rad, lon_rad, alt_m, speed)]) -> command id */
PyObject* world_c2_command_route(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity, ma;
    if (!check_args(n, 3, 3, "c2_command_route") || !as_u32(args[0], &entity) || !as_u32(args[1], &ma) || !WORLD_IDLE(self)) return NULL;
    const Py_ssize_t count = PySequence_Size(args[2]);
    if (count < 0) return NULL;
    if (count == 0 || count > 16) {
        PyErr_SetString(PyExc_ValueError, "c2_command_route: 1 to 16 points");
        return NULL;
    }
    double pts[64];
    for (Py_ssize_t i = 0; i < count; ++i) {
        PyObject* t = PySequence_GetItem(args[2], i);
        for (Py_ssize_t k = 0; t && k < 4; ++k) {
            PyObject* v = PySequence_GetItem(t, k);
            pts[i * 4 + k] = v ? PyFloat_AsDouble(v) : 0.0;
            Py_XDECREF(v);
        }
        Py_XDECREF(t);
        if (PyErr_Occurred()) return NULL;
    }
    uint64_t id = 0;
    int32_t reason = 0;
    if (fsim_c2_command_route(self->world, entity, ma, pts, (uint32_t)count, &id, &reason) != FSIM_OK) return fail();
    return PyLong_FromUnsignedLongLong(id);
}

/* c2_cancel_command(entity, ma, command id) */
PyObject* world_c2_cancel_command(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity, ma;
    uint64_t id;
    if (!check_args(n, 3, 3, "c2_cancel_command") || !as_u32(args[0], &entity) || !as_u32(args[1], &ma) || !as_u64(args[2], &id) ||
        !WORLD_IDLE(self))
        return NULL;
    int32_t reason = 0;
    if (fsim_c2_cancel_command(self->world, entity, ma, id, &reason) != FSIM_OK) return fail();
    Py_RETURN_TRUE;
}

/* c2_command_statuses(entity) -> [(from, received_s, command_id, topic, state, reason, activity, time_s)] */
PyObject* world_c2_command_statuses(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity;
    if (!check_args(n, 1, 1, "c2_command_statuses") || !as_u32(args[0], &entity)) return NULL;
    const uint32_t count = fsim_c2_command_status_count(self->world, entity);
    PyObject* list = PyList_New((Py_ssize_t)count);
    for (uint32_t i = 0; list && i < count; ++i) {
        fsim_c2_status s;
        fsim_c2_status_init(&s);
        if (fsim_c2_command_status_get(self->world, entity, i, &s) != FSIM_OK) {
            Py_DECREF(list);
            return fail();
        }
        PyList_SetItem(list, (Py_ssize_t)i, Py_BuildValue("(IdKIIIKdd)", s.from, s.received_s, (unsigned long long)s.command_id, s.topic, s.state,
                                                          s.reason, (unsigned long long)s.activity, s.time_s, s.percent));
    }
    return list;
}
/* --- A-GRA's native messages and the picture (ABI 1.62; AB-4b) ------------------------------------------------------ */

/* set_dissemination(entity, position_period_s, status_period_s, tracks_period_s, peer_timeout_s, track_timeout_s) -> bool */
PyObject* world_set_dissemination(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity;
    double v[5];
    if (!check_args(n, 6, 6, "set_dissemination") || !as_u32(args[0], &entity) || !as_doubles(args, 1, 5, v) || !WORLD_IDLE(self))
        return NULL;
    fsim_dissemination_plan p;
    fsim_dissemination_plan_init(&p);
    p.position_period_s = v[0], p.status_period_s = v[1], p.tracks_period_s = v[2], p.peer_timeout_s = v[3], p.track_timeout_s = v[4];
    if (fsim_world_set_dissemination(self->world, entity, &p) != FSIM_OK) return fail();
    Py_RETURN_TRUE;
}

/* dms_publish(entity, topic, payload, destinations) -> (first message number, how many) */
PyObject* world_dms_publish(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity, topic, payload;
    if (!check_args(n, 4, 4, "dms_publish") || !as_u32(args[0], &entity) || !as_u32(args[1], &topic) || !as_u32(args[2], &payload) ||
        !WORLD_IDLE(self))
        return NULL;
    const Py_ssize_t count = PySequence_Size(args[3]);
    if (count < 0) return NULL;
    uint32_t* dest = count ? (uint32_t*)PyMem_Malloc(sizeof(uint32_t) * (size_t)count) : NULL;
    for (Py_ssize_t i = 0; i < count; ++i) {
        PyObject* item = PySequence_GetItem(args[3], i);
        const int ok = item && as_u32(item, &dest[i]);
        Py_XDECREF(item);
        if (!ok) {
            PyMem_Free(dest);
            return NULL;
        }
    }
    uint64_t first = 0;
    uint32_t sent = 0;
    int32_t reason = 0;
    const int rc = fsim_dms_publish(self->world, entity, topic, payload, dest, (uint32_t)count, &first, &sent, &reason);
    PyMem_Free(dest);
    if (rc != FSIM_OK) return fail();
    return Py_BuildValue("(KI)", (unsigned long long)first, sent);
}

/* dms_peers(entity) -> [(system, net, received_s, tail_number, has_position, time_s, lat, lon, alt, (vn, ve, vd), heading,
 * tas, cas, mach)] */
PyObject* world_dms_peers(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity;
    if (!check_args(n, 1, 1, "dms_peers") || !as_u32(args[0], &entity)) return NULL;
    const uint32_t count = fsim_dms_peer_count(self->world, entity);
    PyObject* list = PyList_New((Py_ssize_t)count);
    if (!list) return NULL;
    for (uint32_t i = 0; i < count; ++i) {
        fsim_peer_report p;
        fsim_peer_report_init(&p);
        if (fsim_dms_peer_get(self->world, entity, i, &p) != FSIM_OK) {
            Py_DECREF(list);
            return fail();
        }
        PyObject* t = Py_BuildValue("(IIdsOdddd(ddd)dddd)", p.system, p.net, p.received_s, p.tail_number, p.has_position ? Py_True : Py_False,
                                    p.time_s, p.latitude_rad, p.longitude_rad, p.altitude_msl_m, p.velocity_ned_ms[0], p.velocity_ned_ms[1],
                                    p.velocity_ned_ms[2], p.heading_rad, p.airspeed_true_ms, p.airspeed_calibrated_ms, p.mach);
        if (!t) {
            Py_DECREF(list);
            return NULL;
        }
        PyList_SetItem(list, (Py_ssize_t)i, t);
    }
    return list;
}

/* dms_link_tracks(entity) -> [(source, net, received_s, track, payload, updates, time_s, lat, lon, height, (vn, ve, vd),
 * position_sigma_m, velocity_sigma_ms)] */
PyObject* world_dms_link_tracks(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity;
    if (!check_args(n, 1, 1, "dms_link_tracks") || !as_u32(args[0], &entity)) return NULL;
    const uint32_t count = fsim_dms_link_track_count(self->world, entity);
    PyObject* list = PyList_New((Py_ssize_t)count);
    if (!list) return NULL;
    for (uint32_t i = 0; i < count; ++i) {
        fsim_link_track l;
        fsim_link_track_init(&l);
        if (fsim_dms_link_track_get(self->world, entity, i, &l) != FSIM_OK) {
            Py_DECREF(list);
            return fail();
        }
        PyObject* t = Py_BuildValue("(IIdKIIdddd(ddd)dd)", l.source, l.net, l.received_s, (unsigned long long)l.track, l.payload, l.updates,
                                    l.time_s, l.latitude_rad, l.longitude_rad, l.height_m, l.velocity_ned_ms[0], l.velocity_ned_ms[1],
                                    l.velocity_ned_ms[2], l.position_sigma_m, l.velocity_sigma_ms);
        if (!t) {
            Py_DECREF(list);
            return NULL;
        }
        PyList_SetItem(list, (Py_ssize_t)i, t);
    }
    return list;
}

/* designate_link(id, source, payload, track) -> target (0: not held) */
PyObject* world_designate_link(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id, source, payload, target = 0;
    uint64_t track;
    if (!check_args(n, 4, 4, "designate_link") || !as_u32(args[0], &id) || !as_u32(args[1], &source) || !as_u32(args[2], &payload) ||
        !as_u64(args[3], &track) || !WORLD_IDLE(self))
        return NULL;
    if (fsim_vehicle_designate_link(self->world, id, source, payload, track, &target) != FSIM_OK) return fail();
    return PyLong_FromUnsignedLong(target);
}

PyObject* world_designation_source(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id, target;
    if (!check_args(n, 2, 2, "designation_source") || !as_u32(args[0], &id) || !as_u32(args[1], &target)) return NULL;
    return PyLong_FromUnsignedLong(fsim_vehicle_designation_source(self->world, id, target));
}

PyObject* world_link_track_truth(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t source, payload;
    uint64_t track;
    if (!check_args(n, 3, 3, "link_track_truth") || !as_u32(args[0], &source) || !as_u32(args[1], &payload) || !as_u64(args[2], &track))
        return NULL;
    return PyLong_FromUnsignedLong(fsim_world_link_track_truth(self->world, source, payload, track));
}

/* --- MA services (ABI 1.72; docs/sdk/mission-training.md, section 9) ---------------------------------------------- */

/* bind_ma_service_agent(agent's handle, service_id, capabilities: sequence of str, owns_dms_ingress, owns_dms_egress): the
 * service is the agent whose native handle is given - its platform and controller read from that handle, which the
 * library holds to this very world. Python binds by the agent alone, never by its numbers: two worlds mint the same
 * vehicle ids and controllers. Anything that is not an agent's handle of this world is refused as an fsim.Error. */
PyObject* world_bind_ma_service_agent(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    fsim_ma_service_binding b;
    fsim_ma_service_binding_init(&b);
    if (!check_args(n, 5, 5, "bind_ma_service_agent") || !as_u64(args[1], &b.service_id) || !WORLD_IDLE(self)) return NULL;
    if (!PyObject_TypeCheck(args[0], Py_TYPE(o)) || !((WorldObject*)args[0])->world) {
        char why[128];
        PyOS_snprintf(why, sizeof why, "bind_ma_service: MA service %llu: not an fsim.Agent of this World", (unsigned long long)b.service_id);
        fsim_set_last_error(why);
        return fail();
    }
    const fsim_world* agent = ((WorldObject*)args[0])->world;
    PyObject* names = PySequence_Tuple(args[2]); /* (holds each name while its text is borrowed) */
    if (!names) return NULL;
    const Py_ssize_t count = PyTuple_Size(names);
    const char** list = count ? (const char**)PyMem_Malloc(sizeof(const char*) * (size_t)count) : NULL;
    int ok = count == 0 || list != NULL;
    for (Py_ssize_t i = 0; ok && i < count; ++i) ok = (list[i] = as_str(PyTuple_GetItem(names, i), "a command capability")) != NULL;
    const int ingress = ok ? PyObject_IsTrue(args[3]) : -1, egress = ok ? PyObject_IsTrue(args[4]) : -1;
    int rc = FSIM_INVALID_ARGUMENT;
    if (ok && ingress >= 0 && egress >= 0) {
        b.command_capability_count = (uint32_t)count;
        b.command_capabilities = list;
        b.owns_dms_ingress = ingress, b.owns_dms_egress = egress;
        rc = fsim_world_bind_ma_service_agent(self->world, agent, &b);
    } else if (!PyErr_Occurred() && !list && count) {
        PyErr_NoMemory();
    }
    PyMem_Free((void*)list);
    Py_DECREF(names);
    if (PyErr_Occurred()) return NULL;
    if (rc != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

/* ma_services() -> [(service_id, platform, controller, (capability, ...), owns_dms_ingress, owns_dms_egress)], by service id */
PyObject* world_ma_services(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    (void)args;
    if (!check_args(n, 0, 0, "ma_services") || !WORLD_IDLE(self)) return NULL;
    const uint32_t count = fsim_world_ma_service_count(self->world);
    PyObject* list = PyList_New(0);
    for (uint32_t i = 0; list && i < count; ++i) {
        fsim_ma_service_binding b;
        fsim_ma_service_binding_init(&b);
        if (fsim_world_ma_service(self->world, i, &b) != FSIM_OK) continue;
        PyObject* names = PyTuple_New((Py_ssize_t)b.command_capability_count);
        for (uint32_t k = 0; names && k < b.command_capability_count; ++k) {
            PyObject* name = PyUnicode_FromString(b.command_capabilities[k]);
            if (!name || PyTuple_SetItem(names, (Py_ssize_t)k, name) < 0) Py_CLEAR(names);
        }
        PyObject* item = names ? Py_BuildValue("(KIINOO)", (unsigned long long)b.service_id, (unsigned int)b.platform, (unsigned int)b.controller,
                                               names, b.owns_dms_ingress ? Py_True : Py_False, b.owns_dms_egress ? Py_True : Py_False)
                               : NULL;
        if (!item || PyList_Append(list, item) < 0) Py_CLEAR(list);
        Py_XDECREF(item);
    }
    return list;
}

/* ma_capability_owner(platform, capability) -> the owning controller (0: none) */
PyObject* world_ma_capability_owner(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t platform;
    if (!check_args(n, 2, 2, "ma_capability_owner") || !as_u32(args[0], &platform) || !WORLD_IDLE(self)) return NULL;
    const char* capability = as_str(args[1], "capability");
    if (!capability) return NULL;
    return PyLong_FromUnsignedLong(fsim_world_ma_capability_owner(self->world, platform, capability));
}
