/* Mission Systems bindings for fsim._native. */
#define FSIM_PY_NO_BUFFER_TYPE
#include "_native_world.h"

#include <math.h>

/* designate(id, payload, track) -> target id (0: no such track now); undesignate(id, target); designations(id);
 * designation_truth(target) (ABI 1.58; ADR-31, AB-2b) */
PyObject* world_designate(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    uint32_t id, payload, target = 0;
    unsigned long long track;
    if (!check_args(n, 3, 3, "designate") || !as_u32(args[0], &id) || !as_u32(args[1], &payload)) return NULL;
    track = PyLong_AsUnsignedLongLong(args[2]);
    if (PyErr_Occurred()) return NULL;
    if (fsim_vehicle_designate(((WorldObject*)o)->world, id, payload, (uint64_t)track, &target) != FSIM_OK) return fail();
    return PyLong_FromUnsignedLong(target);
}

PyObject* world_undesignate(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    uint32_t id, target;
    if (!check_args(n, 2, 2, "undesignate") || !as_u32(args[0], &id) || !as_u32(args[1], &target)) return NULL;
    if (fsim_vehicle_undesignate(((WorldObject*)o)->world, id, target) != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

PyObject* world_designations(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    uint32_t id, count = 0, i;
    fsim_designation d[64];
    PyObject* list;
    if (!check_args(n, 1, 1, "designations") || !as_u32(args[0], &id)) return NULL;
    if (fsim_vehicle_designations(((WorldObject*)o)->world, id, d, 64, &count) != FSIM_OK) return fail();
    list = PyList_New(0);
    if (!list) return NULL;
    for (i = 0; i < count && i < 64; ++i) {
        PyObject* t = Py_BuildValue("(IIKi)", d[i].target, d[i].payload, (unsigned long long)d[i].track, (int)d[i].valid);
        if (!t || PyList_Append(list, t) < 0) {
            Py_XDECREF(t);
            Py_DECREF(list);
            return NULL;
        }
        Py_DECREF(t);
    }
    return list;
}

PyObject* world_designation_truth(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    uint32_t target;
    if (!check_args(n, 1, 1, "designation_truth") || !as_u32(args[0], &target)) return NULL;
    return PyLong_FromUnsignedLong(fsim_world_designation_truth(((WorldObject*)o)->world, target));
}

/* --- Mission Systems (ADR-30) --------------------------------------------------- */

static PyObject* ms_options_from(PyObject* const* args, Py_ssize_t at, Py_ssize_t n, fsim_ms_options* o) {
    /* (..., source, controller, command_id) from args[at:] */
    int32_t source = 0;
    uint32_t controller = 0;
    unsigned long long command = 0;
    fsim_ms_options_init(o);
    if (n > at && !as_int(args[at], &source)) return NULL;
    if (n > at + 1 && !as_u32(args[at + 1], &controller)) return NULL;
    if (n > at + 2) {
        command = PyLong_AsUnsignedLongLong(args[at + 2]);
        if (PyErr_Occurred()) return NULL;
    }
    o->source = source, o->controller = controller, o->command_id = command;
    Py_RETURN_NONE;
}

static PyObject* ms_result_tuple(const fsim_ms_result* r) {
    return Py_BuildValue("(iiKK)", r->status, r->reason, (unsigned long long)r->activity, (unsigned long long)r->command_id);
}

static PyObject* ms_activity_tuple(const fsim_ms_activity_info* a) {
    return Py_BuildValue("(KIIiIKiiddd)", (unsigned long long)a->id, a->capability, a->payload, a->source, a->controller,
                         (unsigned long long)a->command_id, a->state, a->reason, a->start_time, a->end_time, a->percent);
}

/* payloads(id) -> [(index, name, card, designation, summary, family, state, count, source)] */
PyObject* world_payloads(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    if (!check_args(n, 1, 1, "payloads") || !as_u32(args[0], &id)) return NULL;
    const uint32_t count = fsim_vehicle_payload_count(self->world, id);
    PyObject* list = PyList_New(0);
    for (uint32_t i = 0; list && i < count; ++i) {
        fsim_payload_info p;
        fsim_payload_info_init(&p);
        if (fsim_vehicle_payload(self->world, id, i, &p) != FSIM_OK) {
            Py_DECREF(list);
            return fail();
        }
        PyObject* t = Py_BuildValue("(Isssss" "iiIs)", p.index, p.name, p.card, p.designation, p.summary, "", p.family, p.state, p.count,
                                    p.source);
        if (!t || PyList_Append(list, t) < 0) {
            Py_XDECREF(t);
            Py_CLEAR(list);
            break;
        }
        Py_DECREF(t);
    }
    return list;
}

/* ms_capabilities(id) -> [(index, payload, type, id, support, stage)] */
PyObject* world_ms_capabilities(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    if (!check_args(n, 1, 1, "ms_capabilities") || !as_u32(args[0], &id)) return NULL;
    const uint32_t count = fsim_vehicle_ms_capability_count(self->world, id);
    PyObject* list = PyList_New(0);
    for (uint32_t i = 0; list && i < count; ++i) {
        fsim_ms_capability_info c;
        fsim_ms_capability_info_init(&c);
        if (fsim_vehicle_ms_capability(self->world, id, i, &c) != FSIM_OK) {
            Py_DECREF(list);
            return fail();
        }
        PyObject* t = Py_BuildValue("(IIisiI)", c.index, c.payload, c.type, c.id, c.support, c.stage);
        if (!t || PyList_Append(list, t) < 0) {
            Py_XDECREF(t);
            Py_CLEAR(list);
            break;
        }
        Py_DECREF(t);
    }
    return list;
}

/* ms_capability_status(id, capability) -> (availability, reason, next_available_s) */
PyObject* world_ms_capability_status(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    int32_t availability = 0, reason = 0;
    double next = 0.0;
    if (!check_args(n, 2, 2, "ms_capability_status") || !as_u32(args[0], &id)) return NULL;
    const char* cap = as_str(args[1], "capability id");
    if (!cap) return NULL;
    if (fsim_vehicle_ms_capability_status(self->world, id, cap, &availability, &reason, &next) != FSIM_OK) return fail();
    return Py_BuildValue("(iid)", availability, reason, next);
}

/* payload_state(id, payload, state, source=0, controller=0, command_id=0) -> (status, reason, activity, command_id) */
PyObject* world_payload_state(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id, payload;
    int32_t state;
    fsim_ms_options opts;
    fsim_ms_result r;
    if (!check_args(n, 3, 6, "payload_state") || !as_u32(args[0], &id) || !as_u32(args[1], &payload) || !as_int(args[2], &state)) return NULL;
    if (!ms_options_from(args, 3, n, &opts)) return NULL;
    Py_DECREF(Py_None);
    fsim_ms_result_init(&r);
    if (fsim_vehicle_payload_state(self->world, id, payload, state, &opts, &r) != FSIM_OK) return fail();
    return ms_result_tuple(&r);
}

/* payload_bit(id, payload, source=0, controller=0, command_id=0) -> (status, reason, activity, command_id) */
PyObject* world_payload_bit(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id, payload;
    fsim_ms_options opts;
    fsim_ms_result r;
    if (!check_args(n, 2, 5, "payload_bit") || !as_u32(args[0], &id) || !as_u32(args[1], &payload)) return NULL;
    if (!ms_options_from(args, 2, n, &opts)) return NULL;
    Py_DECREF(Py_None);
    fsim_ms_result_init(&r);
    if (fsim_vehicle_payload_bit(self->world, id, payload, &opts, &r) != FSIM_OK) return fail();
    return ms_result_tuple(&r);
}

/* ms_cancel(activity, source=0, controller=0) -> (status, reason, activity, command_id) */
PyObject* world_ms_cancel(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    fsim_ms_options opts;
    fsim_ms_result r;
    if (!check_args(n, 1, 3, "ms_cancel")) return NULL;
    const unsigned long long activity = PyLong_AsUnsignedLongLong(args[0]);
    if (PyErr_Occurred()) return NULL;
    if (!ms_options_from(args, 1, n, &opts)) return NULL;
    Py_DECREF(Py_None);
    fsim_ms_result_init(&r);
    if (fsim_ms_activity_cancel(self->world, (fsim_activity_id)activity, &opts, &r) != FSIM_OK) return fail();
    return ms_result_tuple(&r);
}

/* ms_activity(activity) -> tuple, or None */
PyObject* world_ms_activity(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    fsim_ms_activity_info a;
    if (!check_args(n, 1, 1, "ms_activity")) return NULL;
    const unsigned long long activity = PyLong_AsUnsignedLongLong(args[0]);
    if (PyErr_Occurred()) return NULL;
    fsim_ms_activity_info_init(&a);
    if (fsim_ms_activity_get(self->world, (fsim_activity_id)activity, &a) != FSIM_OK) Py_RETURN_NONE;
    return ms_activity_tuple(&a);
}

/* ms_activities(id) -> [tuple] */
PyObject* world_ms_activities(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    if (!check_args(n, 1, 1, "ms_activities") || !as_u32(args[0], &id)) return NULL;
    const uint32_t count = fsim_vehicle_ms_activity_count(self->world, id);
    PyObject* list = PyList_New(0);
    for (uint32_t i = 0; list && i < count; ++i) {
        fsim_ms_activity_info a;
        fsim_ms_activity_info_init(&a);
        if (fsim_vehicle_ms_activity(self->world, id, i, &a) != FSIM_OK) break;
        PyObject* t = ms_activity_tuple(&a);
        if (!t || PyList_Append(list, t) < 0) {
            Py_XDECREF(t);
            Py_CLEAR(list);
            break;
        }
        Py_DECREF(t);
    }
    return list;
}

/* payload_fail(id, payload, failed) */
PyObject* world_payload_fail(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id, payload;
    int failed;
    if (!check_args(n, 3, 3, "payload_fail") || !as_u32(args[0], &id) || !as_u32(args[1], &payload) || !as_int(args[2], &failed)) return NULL;
    if (fsim_vehicle_payload_fail(self->world, id, payload, failed) != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

/* Weapons in flight and the radio path (ABI 1.50; ADR-30 MS-2) */

/* launch_weapon(launcher, type, name, ox, oy, oz, ex, ey, ez) -> id */
PyObject* world_launch_weapon(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t launcher, id = 0;
    double v[6];
    if (!check_args(n, 9, 9, "launch_weapon") || !as_u32(args[0], &launcher)) return NULL;
    const char* type = as_str(args[1], "type");
    const char* name = as_str(args[2], "name");
    if (!type || !name || !as_doubles(args, 3, 6, v)) return NULL;
    fsim_weapon_launch l;
    fsim_weapon_launch_init(&l);
    l.type = type, l.name = name;
    for (int i = 0; i < 3; ++i) l.offset_body_m[i] = v[i], l.ejection_body_ms[i] = v[3 + i];
    if (fsim_world_launch_weapon(self->world, launcher, &l, &id) != FSIM_OK) return fail();
    return PyLong_FromUnsignedLong(id);
}

/* weapons() -> [(id, launcher, name, type, launch_time_s, end_time_s, end, end_lat, end_lon, end_height_m, end_speed_ms)] */
PyObject* world_weapons(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    (void)args;
    if (!check_args(n, 0, 0, "weapons")) return NULL;
    const uint32_t count = fsim_world_weapon_count(self->world);
    PyObject* list = PyList_New(0);
    for (uint32_t i = 0; list && i < count; ++i) {
        fsim_weapon_info w;
        fsim_weapon_info_init(&w);
        if (fsim_world_weapon(self->world, i, &w) != FSIM_OK) {
            Py_DECREF(list);
            return fail();
        }
        PyObject* t = Py_BuildValue("(IIssddiddddsiiKIisIdiiidd)", w.id, w.launcher, w.name, w.type, w.launch_time_s, w.end_time_s, w.end,
                                    w.end_latitude_rad, w.end_longitude_rad, w.end_height_m, w.end_speed_ms, w.store ? w.store : "",
                                    w.station, w.jettisoned, (unsigned long long)w.activity, w.target, w.phase, w.seeker,
                                    w.holding, w.miss_m, w.burning, w.tracking, w.supported,w.time_to_go_s,w.time_to_active_s);
        if (!t || PyList_Append(list, t) < 0) {
            Py_XDECREF(t);
            Py_CLEAR(list);
            break;
        }
        Py_DECREF(t);
    }
    return list;
}

PyObject* world_dispense(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self=(WorldObject*)o; uint32_t id; int flare;
    if (!check_args(n,2,2,"dispense") || !as_u32(args[0],&id)) return NULL;
    flare=PyObject_IsTrue(args[1]); if(flare<0) return NULL;
    if(fsim_world_dispense(self->world,id,flare)!=FSIM_OK) return fail();
    Py_RETURN_NONE;
}
PyObject* world_countermeasures(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self=(WorldObject*)o; uint32_t id,chaff,flare;
    if(!check_args(n,1,1,"countermeasures") || !as_u32(args[0],&id)) return NULL;
    if(fsim_world_countermeasures(self->world,id,&chaff,&flare)!=FSIM_OK) return fail();
    return Py_BuildValue("(II)",chaff,flare);
}
PyObject* world_launch_zone(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self=(WorldObject*)o; uint32_t id,target; const char* store;
    if(!check_args(n,3,3,"launch_zone") || !as_u32(args[0],&id) || !as_u32(args[2],&target)) return NULL;
    store=PyUnicode_AsUTF8AndSize(args[1],NULL); if(!store) return NULL;
    fsim_launch_zone zone; fsim_launch_zone_init(&zone);
    if(fsim_world_launch_zone(self->world,id,store,target,&zone)!=FSIM_OK) return fail();
    return Py_BuildValue("(ddd)",zone.maximum_m,zone.no_escape_m,zone.minimum_m);
}
PyObject* world_seeker_target(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self=(WorldObject*)o; uint32_t id,target; const char* store;
    if(!check_args(n,2,2,"seeker_target") || !as_u32(args[0],&id)) return NULL;
    store=PyUnicode_AsUTF8AndSize(args[1],NULL); if(!store) return NULL;
    if(fsim_world_seeker_target(self->world,id,store,&target)!=FSIM_OK) return fail();
    return PyLong_FromUnsignedLong(target);
}
PyObject* world_engagement_events(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self=(WorldObject*)o; unsigned long long since;
    if(!check_args(n,1,1,"engagement_events")) return NULL;
    since=PyLong_AsUnsignedLongLong(args[0]); if(PyErr_Occurred()) return NULL;
    const uint32_t count=fsim_world_engagement_count(self->world,since);
    PyObject* list=PyList_New(0);
    for(uint32_t i=0;list && i<count;++i) {
        fsim_engagement_event event; fsim_engagement_event_init(&event);
        if(fsim_world_engagement(self->world,i,&event)!=FSIM_OK) {Py_DECREF(list);return fail();}
        PyObject* item=Py_BuildValue("(KdssIII(ddd)(ddd)i)",(unsigned long long)event.serial,event.time_s,event.kind,event.reason,
            event.entity,event.launcher,event.target,event.position_ecef[0],event.position_ecef[1],event.position_ecef[2],
            event.velocity_ecef[0],event.velocity_ecef[1],event.velocity_ecef[2],event.destroyed);
        if(!item || PyList_Append(list,item)<0) {Py_XDECREF(item);Py_CLEAR(list);break;} Py_DECREF(item);
    }
    return list;
}
PyObject* world_decoys(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self=(WorldObject*)o; (void)args;
    if(!check_args(n,0,0,"decoys")) return NULL;
    const uint32_t count=fsim_world_decoy_count(self->world);
    PyObject* list=PyList_New(0);
    for(uint32_t i=0;list && i<count;++i) {
        fsim_decoy_state d;fsim_decoy_state_init(&d);
        if(fsim_world_decoy(self->world,i,&d)!=FSIM_OK) {Py_DECREF(list);return fail();}
        PyObject* item=Py_BuildValue("(IIid(ddd))",d.id,d.owner,d.flare,d.age_s,d.position_ecef[0],d.position_ecef[1],d.position_ecef[2]);
        if(!item || PyList_Append(list,item)<0) {Py_XDECREF(item);Py_CLEAR(list);break;} Py_DECREF(item);
    }
    return list;
}

/* stores(id) -> [(index, payload, station, station_name, store, mnemonic, class, carriage, powerable, jettisonable, flyable,
 * status, mass_kg, weapon)] (MS-7) */
PyObject* world_stores(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    if (!check_args(n, 1, 1, "stores") || !as_u32(args[0], &id)) return NULL;
    const uint32_t count = fsim_vehicle_store_count(self->world, id);
    PyObject* list = PyList_New(0);
    for (uint32_t i = 0; list && i < count; ++i) {
        fsim_store_info s;
        fsim_store_info_init(&s);
        if (fsim_vehicle_store(self->world, id, i, &s) != FSIM_OK) {
            Py_DECREF(list);
            return fail();
        }
        PyObject* t = Py_BuildValue("(IIisssssiiiidI)", s.index, s.payload, s.station, s.station_name, s.store, s.mnemonic, s.store_class,
                                    s.carriage, s.powerable, s.jettisonable, s.flyable, s.status, s.mass_kg, s.weapon);
        if (!t || PyList_Append(list, t) < 0) {
            Py_XDECREF(t);
            Py_CLEAR(list);
            break;
        }
        Py_DECREF(t);
    }
    return list;
}

/* load_store(id, payload, station, store, count) -> reason */
PyObject* world_load_store(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id, payload;
    int station, count;
    int32_t reason = 0;
    if (!check_args(n, 5, 5, "load_store") || !as_u32(args[0], &id) || !as_u32(args[1], &payload) || !as_int(args[2], &station) ||
        !as_int(args[4], &count))
        return NULL;
    const char* store = as_str(args[3], "store");
    if (!store) return NULL;
    if (fsim_vehicle_load_store(self->world, id, payload, station, store, count, &reason) != FSIM_OK) return fail();
    return PyLong_FromLong(reason);
}

/* load_loadout(id, name) -> reason */
PyObject* world_load_loadout(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    int32_t reason = 0;
    if (!check_args(n, 2, 2, "load_loadout") || !as_u32(args[0], &id)) return NULL;
    const char* name = as_str(args[1], "loadout");
    if (!name) return NULL;
    if (fsim_vehicle_load_loadout(self->world, id, name, &reason) != FSIM_OK) return fail();
    return PyLong_FromLong(reason);
}

/* (a sequence of store indices, or None) into `out` (at most `max`); -1 on error */
static int store_indices(PyObject* seq, uint32_t* out, uint32_t max) {
    if (seq == Py_None) return 0;
    const Py_ssize_t len = PySequence_Size(seq);
    if (len < 0) return -1;
    if (len > (Py_ssize_t)max) {
        PyErr_SetString(PyExc_ValueError, "too many stores");
        return -1;
    }
    for (Py_ssize_t i = 0; i < len; ++i) {
        PyObject* item = PySequence_GetItem(seq, i);
        const int ok = item && as_u32(item, &out[i]);
        Py_XDECREF(item);
        if (!ok) return -1;
    }
    return (int)len;
}

/* strike(id, payload, store, quantity, stores, target, consent, ripple_s, source=0, controller=0, command_id=0) -> result */
PyObject* world_strike(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id, payload, target, indices[256];
    int quantity, consent;
    double ripple;
    fsim_strike_command c;
    fsim_ms_options opts;
    fsim_ms_result r;
    if (!check_args(n, 8, 11, "strike") || !as_u32(args[0], &id) || !as_u32(args[1], &payload) || !as_int(args[3], &quantity) ||
        !as_u32(args[5], &target) || !as_int(args[6], &consent) || !as_doubles(args, 7, 1, &ripple))
        return NULL;
    const char* store = as_str(args[2], "store");
    if (!store) return NULL;
    const int count = store_indices(args[4], indices, 256);
    if (count < 0) return NULL;
    if (!ms_options_from(args, 8, n, &opts)) return NULL;
    Py_DECREF(Py_None);
    fsim_strike_command_init(&c);
    c.payload = payload, c.store = store, c.quantity = quantity, c.target = target, c.consent = consent, c.ripple_s = ripple;
    c.stores = count > 0 ? indices : NULL, c.store_count = (uint32_t)count;
    fsim_ms_result_init(&r);
    if (fsim_vehicle_strike(self->world, id, &c, &opts, &r) != FSIM_OK) return fail();
    return ms_result_tuple(&r);
}

/* jettison(id, payload, stores, emergency, source=0, controller=0, command_id=0) -> result */
PyObject* world_jettison(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id, payload, indices[256];
    int emergency;
    fsim_jettison_command c;
    fsim_ms_options opts;
    fsim_ms_result r;
    if (!check_args(n, 4, 7, "jettison") || !as_u32(args[0], &id) || !as_u32(args[1], &payload) || !as_int(args[3], &emergency)) return NULL;
    const int count = store_indices(args[2], indices, 256);
    if (count < 0) return NULL;
    if (!ms_options_from(args, 4, n, &opts)) return NULL;
    Py_DECREF(Py_None);
    fsim_jettison_command_init(&c);
    c.payload = payload, c.emergency = emergency;
    c.stores = count > 0 ? indices : NULL, c.store_count = (uint32_t)count;
    fsim_ms_result_init(&r);
    if (fsim_vehicle_jettison(self->world, id, &c, &opts, &r) != FSIM_OK) return fail();
    return ms_result_tuple(&r);
}

/* strike_consent(activity, granted) -> reason */
PyObject* world_strike_consent(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint64_t activity;
    int granted;
    int32_t reason = 0;
    if (!check_args(n, 2, 2, "strike_consent") || !as_u64(args[0], &activity) || !as_int(args[1], &granted)) return NULL;
    if (fsim_strike_consent(self->world, activity, granted, &reason) != FSIM_OK) return fail();
    return PyLong_FromLong(reason);
}

/* strike_status(activity) -> (phase, jettison, target, (stores...), (weapons...)) or None */
PyObject* world_strike_status(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint64_t activity;
    uint32_t stores[256], weapons[256];
    fsim_strike_status s;
    if (!check_args(n, 1, 1, "strike_status") || !as_u64(args[0], &activity)) return NULL;
    fsim_strike_status_init(&s);
    if (fsim_strike_status_get(self->world, activity, &s, stores, 256, weapons, 256) != FSIM_OK) Py_RETURN_NONE;
    PyObject* st = PyTuple_New((Py_ssize_t)(s.store_count < 256 ? s.store_count : 256));
    PyObject* wt = PyTuple_New((Py_ssize_t)(s.weapon_count < 256 ? s.weapon_count : 256));
    if (!st || !wt) {
        Py_XDECREF(st);
        Py_XDECREF(wt);
        return NULL;
    }
    for (uint32_t i = 0; i < s.store_count && i < 256; ++i) PyTuple_SetItem(st, (Py_ssize_t)i, PyLong_FromUnsignedLong(stores[i]));
    for (uint32_t i = 0; i < s.weapon_count && i < 256; ++i) PyTuple_SetItem(wt, (Py_ssize_t)i, PyLong_FromUnsignedLong(weapons[i]));
    return Py_BuildValue("(iiINN)", s.phase, s.jettison, s.target, st, wt);
}

/* mass_kg(id) -> float */
PyObject* world_mass_kg(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    if (!check_args(n, 1, 1, "mass_kg") || !as_u32(args[0], &id)) return NULL;
    return PyFloat_FromDouble(fsim_vehicle_mass_kg(self->world, id));
}

/* vehicle_role(id) -> (role, launcher) */
PyObject* world_vehicle_role(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id, launcher = 0;
    int32_t role = 0;
    if (!check_args(n, 1, 1, "vehicle_role") || !as_u32(args[0], &id)) return NULL;
    if (fsim_vehicle_role(self->world, id, &role, &launcher) != FSIM_OK) return fail();
    return Py_BuildValue("(iI)", role, launcher);
}

/* radio_path(from, to, frequency_hz, polarization, time_percent, rain_rate_mmh, rain_height_m, itm)
 *   -> (code, via_itm, mode, masked, ground_range, slant_range, el0, el1, az0, az1, clearance, free_space, basic, gas, terrain, rain, total) */
PyObject* world_radio_path(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t from, to;
    int pol, itm;
    double f, time, rain, rainh;
    if (!check_args(n, 8, 8, "radio_path") || !as_u32(args[0], &from) || !as_u32(args[1], &to) || !as_doubles(args, 2, 1, &f) ||
        !as_int(args[3], &pol) || !as_doubles(args, 4, 1, &time) || !as_doubles(args, 5, 1, &rain) || !as_doubles(args, 6, 1, &rainh) ||
        !as_int(args[7], &itm))
        return NULL;
    fsim_radio_path_options opt;
    fsim_radio_path_options_init(&opt);
    opt.frequency_hz = f, opt.polarization = pol, opt.time_percent = time, opt.rain_rate_mmh = rain, opt.rain_height_m = rainh, opt.itm = itm;
    fsim_radio_path p;
    fsim_radio_path_init(&p);
    if (fsim_world_radio_path(self->world, from, to, &opt, &p) != FSIM_OK) return fail();
    return Py_BuildValue("(iiiiddddddddddddd)", p.code, p.via_itm, p.mode, p.terrain_masked, p.ground_range_m, p.slant_range_m,
                         p.elevation_rad[0], p.elevation_rad[1], p.azimuth_rad[0], p.azimuth_rad[1], p.clearance_m, p.free_space_db,
                         p.basic_db, p.gas_db, p.terrain_db, p.rain_db, p.total_db);
}

/* radar_cross_section(observer, target, frequency_hz, tx_rad, rx_rad) -> (modelled, sigma_m2, fluctuation, az_rad, el_rad) */
PyObject* world_radar_cross_section(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t observer, target;
    double v[3];
    if (!check_args(n, 5, 5, "radar_cross_section") || !as_u32(args[0], &observer) || !as_u32(args[1], &target) || !as_doubles(args, 2, 3, v))
        return NULL;
    fsim_rcs_query q;
    fsim_rcs_query_init(&q);
    q.frequency_hz = v[0], q.transmit_polarization_rad = v[1], q.receive_polarization_rad = v[2];
    fsim_radar_cross_section r;
    fsim_radar_cross_section_init(&r);
    if (fsim_world_radar_cross_section(self->world, observer, target, &q, &r) != FSIM_OK) return fail();
    return Py_BuildValue("(idddd)", r.modelled, r.sigma_m2, r.fluctuation, r.aspect_azimuth_rad, r.aspect_elevation_rad);
}

PyObject* mod_role_name(PyObject* m, PyObject* const* args, Py_ssize_t n) {
    int code;
    (void)m;
    if (!check_args(n, 1, 1, "role_name") || !as_int(args[0], &code)) return NULL;
    return PyUnicode_FromString(fsim_role_name(code));
}

/* amti_search(id, payload, mode, az_centre, az_half_width, el_centre, bars, range_scale, signal_level, source=0, controller=0,
 * command_id=0, task=0, track=0, el_half_width=NaN, illuminate=0) */
PyObject* world_amti_search(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id, payload;
    int32_t mode, bars, signal, task = 0, illuminate = 0;
    uint64_t track = 0;
    double v[3], scale, tall = Py_NAN;
    fsim_amti_command c;
    fsim_ms_options opts;
    fsim_ms_result r;
    if (!check_args(n, 9, 16, "amti_search") || !as_u32(args[0], &id) || !as_u32(args[1], &payload) || !as_int(args[2], &mode) ||
        !as_doubles(args, 3, 3, v) || !as_int(args[6], &bars) || !as_doubles(args, 7, 1, &scale) || !as_int(args[8], &signal))
        return NULL;
    if (!ms_options_from(args, 9, n < 12 ? n : 12, &opts)) return NULL;
    Py_DECREF(Py_None);
    if ((n > 12 && !as_int(args[12], &task)) || (n > 13 && !as_u64(args[13], &track)) || (n > 14 && !as_doubles(args, 14, 1, &tall)) ||
        (n > 15 && !as_int(args[15], &illuminate)))
        return NULL;
    fsim_amti_command_init(&c);
    c.payload = payload, c.mode = mode, c.bars = bars;
    c.azimuth_centre_rad = v[0], c.azimuth_half_width_rad = v[1], c.elevation_centre_rad = v[2], c.range_scale_m = scale;
    c.signal_level = signal;
    c.task = task, c.track = track, c.elevation_half_width_rad = tall, c.illuminate = illuminate;
    fsim_ms_result_init(&r);
    if (fsim_vehicle_amti_search(self->world, id, &c, &opts, &r) != FSIM_OK) return fail();
    return ms_result_tuple(&r);
}

/* radar_status(id, payload) -> (searching, activity, mode, bars, rotodome, electronic, beam_az, beam_el, frame_period, az_centre,
 * az_half_width, el_centre, range_scale, az_limit, el_limit, instrumented_range, frequency, bw_az, bw_el, dwells, detections) or None */
PyObject* world_radar_status(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id, payload;
    fsim_radar_status s;
    if (!check_args(n, 2, 2, "radar_status") || !as_u32(args[0], &id) || !as_u32(args[1], &payload)) return NULL;
    fsim_radar_status_init(&s);
    if (fsim_vehicle_radar_status(self->world, id, payload, &s) != FSIM_OK) Py_RETURN_NONE;
    return Py_BuildValue("(iKiiii" "ddddddddddddd" "KKKIIi" "iKiid)", s.searching, (unsigned long long)s.activity, s.mode, s.bars, s.rotodome, s.electronic,
                         s.beam_azimuth_rad, s.beam_elevation_rad, s.frame_period_s, s.azimuth_centre_rad, s.azimuth_half_width_rad,
                         s.elevation_centre_rad, s.range_scale_m, s.azimuth_limit_rad, s.elevation_limit_rad, s.instrumented_range_m,
                         s.frequency_hz, s.beamwidth_azimuth_rad, s.beamwidth_elevation_rad, (unsigned long long)s.dwells,
                         (unsigned long long)s.detections, (unsigned long long)s.false_alarms, s.tracks, s.tentative_tracks, s.signal_level,
                         s.task, (unsigned long long)s.designated_track, s.acquiring, s.illuminating, s.track_memory_s);
}

/* amti_detections(id, payload, since) -> [(serial, payload, time, truth, range, az, el, rate, snr, lat, lon, h)] */
PyObject* world_amti_detections(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id, payload, count = 0;
    if (!check_args(n, 3, 3, "amti_detections") || !as_u32(args[0], &id) || !as_u32(args[1], &payload)) return NULL;
    const unsigned long long since = PyLong_AsUnsignedLongLong(args[2]);
    if (PyErr_Occurred()) return NULL;
    if (fsim_vehicle_amti_detections(self->world, id, payload, since, NULL, 0, &count) != FSIM_OK) return fail();
    fsim_amti_detection* d = count ? (fsim_amti_detection*)PyMem_Malloc(sizeof(fsim_amti_detection) * count) : NULL;
    if (count && !d) return PyErr_NoMemory();
    if (count) {
        fsim_amti_detection_init(&d[0]);
        uint32_t got = 0;
        if (fsim_vehicle_amti_detections(self->world, id, payload, since, d, count, &got) != FSIM_OK) {
            PyMem_Free(d);
            return fail();
        }
        if (got < count) count = got;
    }
    PyObject* list = PyList_New(0);
    for (uint32_t i = 0; list && i < count; ++i) {
        PyObject* t = Py_BuildValue("(KIdIdddddddd)", (unsigned long long)d[i].serial, d[i].payload, d[i].time_s, d[i].truth, d[i].range_m,
                                    d[i].azimuth_rad, d[i].elevation_rad, d[i].range_rate_ms, d[i].snr_db, d[i].latitude_rad, d[i].longitude_rad,
                                    d[i].height_m);
        if (!t || PyList_Append(list, t) < 0) {
            Py_XDECREF(t);
            Py_CLEAR(list);
            break;
        }
        Py_DECREF(t);
    }
    PyMem_Free(d);
    return list;
}

/* amti_tracks(id, payload) -> [(id, payload, time, last_update, updates, truth, lat, lon, h, vn, ve, vd, sigma_p, sigma_v, range, az, el, rate)] */
PyObject* world_amti_tracks(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id, payload, count = 0;
    if (!check_args(n, 2, 2, "amti_tracks") || !as_u32(args[0], &id) || !as_u32(args[1], &payload)) return NULL;
    if (fsim_vehicle_amti_tracks(self->world, id, payload, NULL, 0, &count) != FSIM_OK) return fail();
    fsim_amti_track* t = count ? (fsim_amti_track*)PyMem_Malloc(sizeof(fsim_amti_track) * count) : NULL;
    if (count && !t) return PyErr_NoMemory();
    if (count) {
        fsim_amti_track_init(&t[0]);
        uint32_t got = 0;
        if (fsim_vehicle_amti_tracks(self->world, id, payload, t, count, &got) != FSIM_OK) {
            PyMem_Free(t);
            return fail();
        }
        if (got < count) count = got;
    }
    PyObject* list = PyList_New(0);
    for (uint32_t i = 0; list && i < count; ++i) {
        PyObject* x = Py_BuildValue("(KIddII" "dddddddd" "dddd)", (unsigned long long)t[i].id, t[i].payload, t[i].time_s, t[i].last_update_s, t[i].updates,
                                    t[i].truth, t[i].latitude_rad, t[i].longitude_rad, t[i].height_m, t[i].velocity_ned_ms[0], t[i].velocity_ned_ms[1],
                                    t[i].velocity_ned_ms[2], t[i].position_sigma_m, t[i].velocity_sigma_ms, t[i].range_m, t[i].azimuth_rad,
                                    t[i].elevation_rad, t[i].range_rate_ms);
        if (!x || PyList_Append(list, x) < 0) {
            Py_XDECREF(x);
            Py_CLEAR(list);
            break;
        }
        Py_DECREF(x);
    }
    PyMem_Free(t);
    return list;
}

/* contacts(id, payload) -> [(id, payload, kind, mode, warning, truth, first, time, az, el, sigma, range, rate, lat, lon, h, strength, frequency, size)] */
PyObject* world_contacts(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id, payload, count = 0;
    if (!check_args(n, 2, 2, "contacts") || !as_u32(args[0], &id) || !as_u32(args[1], &payload)) return NULL;
    if (fsim_vehicle_contacts(self->world, id, payload, NULL, 0, &count) != FSIM_OK) return fail();
    fsim_contact* t = count ? (fsim_contact*)PyMem_Malloc(sizeof(fsim_contact) * count) : NULL;
    if (count && !t) return PyErr_NoMemory();
    if (count) {
        fsim_contact_init(&t[0]);
        uint32_t got = 0;
        if (fsim_vehicle_contacts(self->world, id, payload, t, count, &got) != FSIM_OK) {
            PyMem_Free(t);
            return fail();
        }
        if (got < count) count = got;
    }
    PyObject* list = PyList_New(0);
    for (uint32_t i = 0; list && i < count; ++i) {
        PyObject* x = Py_BuildValue("(KIiiiI" "ddddddd" "dddddd)", (unsigned long long)t[i].id, t[i].payload, (int)t[i].kind, (int)t[i].mode,
                                    (int)t[i].warning, t[i].truth, t[i].first_time_s, t[i].time_s, t[i].azimuth_rad, t[i].elevation_rad,
                                    t[i].bearing_sigma_rad, t[i].range_m, t[i].range_rate_ms, t[i].latitude_rad, t[i].longitude_rad, t[i].height_m,
                                    t[i].strength_db, t[i].frequency_hz, t[i].size_m);
        if (!x || PyList_Append(list, x) < 0) {
            Py_XDECREF(x);
            Py_CLEAR(list);
            break;
        }
        Py_DECREF(x);
    }
    PyMem_Free(t);
    return list;
}

PyObject* mod_amti_mode_name(PyObject* m, PyObject* const* args, Py_ssize_t n) {
    int code;
    (void)m;
    if (!check_args(n, 1, 1, "amti_mode_name") || !as_int(args[0], &code)) return NULL;
    return PyUnicode_FromString(fsim_amti_mode_name(code));
}

PyObject* mod_amti_task_name(PyObject* m, PyObject* const* args, Py_ssize_t n) {
    int code;
    (void)m;
    if (!check_args(n, 1, 1, "amti_task_name") || !as_int(args[0], &code)) return NULL;
    return PyUnicode_FromString(fsim_amti_task_name(code));
}

/* amti_acm(pattern) -> (mode, az_centre, az_half_width, el_centre, bars, range_scale, el_half_width): a fighter radar's
 * air-combat acquisition volume (fsim_acm_pattern), a cued task's */
PyObject* mod_amti_acm(PyObject* m, PyObject* const* args, Py_ssize_t n) {
    int pattern;
    fsim_amti_command c;
    (void)m;
    if (!check_args(n, 1, 1, "amti_acm") || !as_int(args[0], &pattern)) return NULL;
    fsim_amti_command_init(&c);
    if (fsim_amti_command_acm(&c, pattern) != FSIM_OK) return fail();
    return Py_BuildValue("(idddidd)", c.mode, c.azimuth_centre_rad, c.azimuth_half_width_rad, c.elevation_centre_rad, c.bars, c.range_scale_m,
                         c.elevation_half_width_rad);
}

PyObject* mod_store_status_name(PyObject* m, PyObject* const* args, Py_ssize_t n) {
    int code;
    (void)m;
    if (!check_args(n, 1, 1, "store_status_name") || !as_int(args[0], &code)) return NULL;
    return PyUnicode_FromString(fsim_store_status_name(code));
}

PyObject* mod_strike_phase_name(PyObject* m, PyObject* const* args, Py_ssize_t n) {
    int code;
    (void)m;
    if (!check_args(n, 1, 1, "strike_phase_name") || !as_int(args[0], &code)) return NULL;
    return PyUnicode_FromString(fsim_strike_phase_name(code));
}

PyObject* mod_weapon_end_name(PyObject* m, PyObject* const* args, Py_ssize_t n) {
    int code;
    (void)m;
    if (!check_args(n, 1, 1, "weapon_end_name") || !as_int(args[0], &code)) return NULL;
    return PyUnicode_FromString(fsim_weapon_end_name(code));
}

PyObject* mod_ms_reason_name(PyObject* m, PyObject* const* args, Py_ssize_t n) {
    int code;
    (void)m;
    if (!check_args(n, 1, 1, "ms_reason_name") || !as_int(args[0], &code)) return NULL;
    return PyUnicode_FromString(fsim_ms_reason_name(code));
}

PyObject* mod_ms_reason_description(PyObject* m, PyObject* const* args, Py_ssize_t n) {
    int code;
    (void)m;
    if (!check_args(n, 1, 1, "ms_reason_description") || !as_int(args[0], &code)) return NULL;
    return PyUnicode_FromString(fsim_ms_reason_description(code));
}

PyObject* mod_payload_family_name(PyObject* m, PyObject* const* args, Py_ssize_t n) {
    int code;
    (void)m;
    if (!check_args(n, 1, 1, "payload_family_name") || !as_int(args[0], &code)) return NULL;
    return PyUnicode_FromString(fsim_payload_family_name(code));
}

PyObject* mod_payload_state_name(PyObject* m, PyObject* const* args, Py_ssize_t n) {
    int code;
    (void)m;
    if (!check_args(n, 1, 1, "payload_state_name") || !as_int(args[0], &code)) return NULL;
    return PyUnicode_FromString(fsim_payload_state_name(code));
}

PyObject* mod_ms_capability_type_id(PyObject* m, PyObject* const* args, Py_ssize_t n) {
    int code;
    (void)m;
    if (!check_args(n, 1, 1, "ms_capability_type_id") || !as_int(args[0], &code)) return NULL;
    return PyUnicode_FromString(fsim_ms_capability_type_id(code));
}

/* --- Legal reports with their provenance (ABI 1.73; docs/sdk/mission-training.md, section 10) ----------------------- */

static PyObject* equipment_indices(const uint32_t* indices, uint32_t count) {
    PyObject* tuple = PyTuple_New((Py_ssize_t)count);
    if (!tuple) return NULL;
    for (uint32_t i = 0; i < count; ++i) {
        PyObject* value = PyLong_FromUnsignedLong(indices[i]);
        if (!value) {
            Py_DECREF(tuple);
            return NULL;
        }
        PyTuple_SetItem(tuple, (Py_ssize_t)i, value);
    }
    return tuple;
}

/* ABI 1.75. Safe strike reports retain activity identity and use no world weapon or target truth getter. */
PyObject* world_agent_strike_reports(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity, count = 0;
    int32_t availability;
    if (!check_args(n, 1, 1, "agent_strike_reports") || !as_u32(args[0], &entity)) return NULL;
    if (fsim_agent_report(self->world, entity, FSIM_REPORT_STRIKES, Py_NAN, &availability, NULL, 0, &count) != FSIM_OK)
        return fail();
    PyObject* list = PyList_New(0);
    for (uint32_t i = 0; list && i < count; ++i) {
        uint64_t activity = 0;
        fsim_strike_status status;
        fsim_strike_status_init(&status);
        if (fsim_agent_strike_report_get(self->world, entity, i, &activity, &status, NULL, 0, NULL, 0) != FSIM_OK) {
            Py_DECREF(list);
            return fail();
        }
        const uint32_t store_count = status.store_count, weapon_count = status.weapon_count;
        uint32_t* stores = store_count ? (uint32_t*)PyMem_Malloc((size_t)store_count * sizeof(uint32_t)) : NULL;
        uint32_t* weapons = weapon_count ? (uint32_t*)PyMem_Malloc((size_t)weapon_count * sizeof(uint32_t)) : NULL;
        if ((store_count && !stores) || (weapon_count && !weapons)) {
            PyMem_Free(stores);
            PyMem_Free(weapons);
            Py_DECREF(list);
            return PyErr_NoMemory();
        }
        if (fsim_agent_strike_report_get(self->world, entity, i, &activity, &status, stores, store_count,
                                          weapons, weapon_count) != FSIM_OK) {
            PyMem_Free(stores);
            PyMem_Free(weapons);
            Py_DECREF(list);
            return fail();
        }
        if (status.store_count != store_count || status.weapon_count != weapon_count) {
            PyMem_Free(stores);
            PyMem_Free(weapons);
            Py_DECREF(list);
            PyErr_SetString(PyExc_RuntimeError, "strike report changed while reading; serialize World.step and report reads");
            return NULL;
        }
        PyObject* store_values = equipment_indices(stores, store_count);
        PyObject* weapon_values = equipment_indices(weapons, weapon_count);
        PyMem_Free(stores);
        PyMem_Free(weapons);
        if (!store_values || !weapon_values) {
            Py_XDECREF(store_values);
            Py_XDECREF(weapon_values);
            Py_DECREF(list);
            return NULL;
        }
        PyObject* row = Py_BuildValue("(KiiINN)", (unsigned long long)activity, status.phase, status.jettison,
                                      status.target, store_values, weapon_values);
        if (!row || PyList_Append(list, row) < 0) Py_CLEAR(list);
        Py_XDECREF(row);
    }
    return list;
}

/* ABI 1.75. Delivered requests through the ownership-checked report getter, never the legacy unrestricted inbox. */
PyObject* world_agent_consent_reports(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity, count = 0;
    int32_t availability;
    if (!check_args(n, 1, 1, "agent_consent_reports") || !as_u32(args[0], &entity)) return NULL;
    if (fsim_agent_report(self->world, entity, FSIM_REPORT_CONSENTS, Py_NAN, &availability, NULL, 0, &count) != FSIM_OK)
        return fail();
    if (availability == FSIM_REPORT_NOT_OWNER) {
        fsim_consent_request refused;
        fsim_consent_request_init(&refused);
        if (fsim_agent_consent_report_get(self->world, entity, 0, &refused) != FSIM_OK) return fail();
    }
    PyObject* list = PyList_New(0);
    for (uint32_t i = 0; list && i < count; ++i) {
        fsim_consent_request request;
        fsim_consent_request_init(&request);
        if (fsim_agent_consent_report_get(self->world, entity, i, &request) != FSIM_OK) {
            Py_DECREF(list);
            return fail();
        }
        char store[25];
        memcpy(store, request.store, 24);
        store[24] = 0;
        PyObject* row = Py_BuildValue("(IdOKKsIIIKdddd)", request.from, request.received_s,
                                      request.answered ? Py_True : Py_False, (unsigned long long)request.request_id,
                                      (unsigned long long)request.strike, store, request.quantity, request.target_source,
                                      request.target_payload, (unsigned long long)request.target_track, request.latitude_rad,
                                      request.longitude_rad, request.altitude_m, request.time_s);
        if (!row || PyList_Append(list, row) < 0) Py_CLEAR(list);
        Py_XDECREF(row);
    }
    return list;
}

/* agent_report(entity, kind, stale_after_s) -> (availability name,
 *     [(source name, validity name, system, net, payload, sample_time_s, receive_time_s, age_s)]) */
PyObject* world_agent_report(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity, kind, count = 0;
    if (!check_args(n, 3, 3, "agent_report") || !as_u32(args[0], &entity) || !as_u32(args[1], &kind)) return NULL;
    const double stale = PyFloat_AsDouble(args[2]);
    if (PyErr_Occurred()) return NULL;
    int32_t availability = FSIM_REPORT_UNAVAILABLE;
    if (fsim_agent_report(self->world, entity, (int)kind, stale, &availability, NULL, 0, &count) != FSIM_OK) return fail();
    fsim_report_meta* m = count ? (fsim_report_meta*)PyMem_Malloc(sizeof(fsim_report_meta) * count) : NULL;
    if (count && !m) return PyErr_NoMemory();
    if (count) {
        fsim_report_meta_init(&m[0]);
        uint32_t got = 0;
        if (fsim_agent_report(self->world, entity, (int)kind, stale, &availability, m, count, &got) != FSIM_OK) {
            PyMem_Free(m);
            return fail();
        }
        if (got < count) count = got;
    }
    PyObject* list = PyList_New(0);
    for (uint32_t i = 0; list && i < count; ++i) {
        PyObject* x = Py_BuildValue("(ssIIIddd)", fsim_report_source_name(m[i].source), fsim_report_validity_name(m[i].validity),
                                    (unsigned int)m[i].system, (unsigned int)m[i].net, (unsigned int)m[i].payload, m[i].sample_time_s,
                                    m[i].receive_time_s, m[i].age_s);
        if (!x || PyList_Append(list, x) < 0) Py_CLEAR(list);
        Py_XDECREF(x);
    }
    PyMem_Free(m);
    if (!list) return NULL;
    return Py_BuildValue("(sN)", fsim_report_availability_name(availability), list);
}

/* agent_operator_callsign(entity, index) -> the callsign an operator declared ("" for none) */
PyObject* world_agent_operator_callsign(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t entity, index;
    if (!check_args(n, 2, 2, "agent_operator_callsign") || !as_u32(args[0], &entity) || !as_u32(args[1], &index)) return NULL;
    return PyUnicode_FromString(fsim_agent_operator_callsign(self->world, entity, index));
}
