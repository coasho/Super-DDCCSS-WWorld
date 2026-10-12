/* Native Environment, effect, terrain and magnetic operations. */
#define FSIM_PY_NO_BUFFER_TYPE
#include "_native_world.h"

#include <math.h>

#define EF(f) FSIM_PY_FIELD(fsim_environment, f, FSIM_PY_F64)
static const fsim_py_field environment_fields[] = {
    EF(epoch_utc_seconds), EF(time_factor), EF(temperature_sl_k), EF(pressure_sl_pa), EF(humidity),
    EF(wind_direction_deg), EF(wind_speed_ms), EF(wind_gust_ms), EF(turbulence), EF(visibility_m),
    EF(cloud_base_m), EF(cloud_cover), EF(precipitation)};
#undef EF
/* magnetic_year() -> the date the World Magnetic Model is read at for this world now (ABI 1.20) */
PyObject* world_magnetic_year(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    (void)args;
    if (!check_args(n, 0, 0, "magnetic_year")) return NULL;
    return PyFloat_FromDouble(fsim_world_magnetic_year(self->world));
}
PyObject* world_get_environment(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    (void)args;
    if (!check_args(n, 0, 0, "get_environment")) return NULL;
    fsim_environment e;
    fsim_environment_init(&e);
    if (fsim_world_get_environment(((WorldObject*)o)->world, &e) != FSIM_OK) return fail();
    return fsim_py_get_fields(&e, environment_fields, COUNT(environment_fields));
}

/* set_environment(changes): the current environment with `changes` applied. */
PyObject* world_set_environment(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    if (!check_args(n, 1, 1, "set_environment") || !WORLD_IDLE(self)) return NULL;
    fsim_environment e;
    fsim_environment_init(&e);
    if (fsim_world_get_environment(self->world, &e) != FSIM_OK) return fail();
    if (fsim_py_set_fields(args[0], &e, environment_fields, COUNT(environment_fields), "environment") < 0) return NULL;
    if (fsim_world_set_environment(self->world, &e) != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

PyObject* world_add_effect(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    if (!check_args(n, 2, 3, "add_effect") || !as_u32(args[0], &id) || !WORLD_IDLE(self)) return NULL;
    const char* effect = as_str(args[1], "effect id");
    fsim_py_params p;
    if (!effect || fsim_py_params_read(n == 3 ? args[2] : NULL, &p) < 0) {
        if (effect) fsim_py_params_free(&p);
        return NULL;
    }
    const int rc = fsim_vehicle_add_effect(self->world, id, effect, p.names, p.values, p.count);
    fsim_py_params_free(&p);
    if (rc != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

PyObject* world_clear_effects(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    uint32_t id;
    if (!check_args(n, 1, 1, "clear_effects") || !as_u32(args[0], &id) || !WORLD_IDLE(self)) return NULL;
    if (fsim_vehicle_clear_effects(self->world, id) != FSIM_OK) return fail();
    Py_RETURN_NONE;
}
/* terrain(latitudes_rad, longitudes_rad) -> [height_m or None]: the ground's height above the WGS-84 ellipsoid at each
 * place - the physics' own - None where it has no data (ABI 1.17) */
PyObject* world_terrain(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    WorldObject* self = (WorldObject*)o;
    if (!check_args(n, 2, 2, "terrain")) return NULL;
    const Py_ssize_t count = PySequence_Size(args[0]);
    if (count < 0) return NULL;
    if (PySequence_Size(args[1]) != count || count > (Py_ssize_t)UINT32_MAX) {
        if (!PyErr_Occurred()) PyErr_SetString(PyExc_ValueError, "terrain: as many longitudes as latitudes");
        return NULL;
    }
    double* v = (double*)PyMem_Malloc(sizeof(double) * 3 * (size_t)(count ? count : 1));
    if (!v) return PyErr_NoMemory();
    for (Py_ssize_t i = 0; i < count && !PyErr_Occurred(); ++i)
        for (int k = 0; k < 2; ++k) {
            PyObject* item = PySequence_GetItem(args[k], i);
            v[k * count + i] = item ? PyFloat_AsDouble(item) : 0.0;
            Py_XDECREF(item);
        }
    PyObject* list = NULL;
    if (!PyErr_Occurred()) {
        if (fsim_world_terrain(self->world, (uint32_t)count, v, v + count, v + 2 * count) < 0) {
            PyErr_SetString(PyExc_RuntimeError, "terrain: the query failed");
        } else {
            list = PyList_New(count);
            for (Py_ssize_t i = 0; list && i < count; ++i) {
                const double h = v[2 * count + i];
                PyObject* x = NULL;
                if (isnan(h)) {
                    Py_INCREF(Py_None);
                    x = Py_None;
                } else {
                    x = PyFloat_FromDouble(h);
                }
                if (!x || PyList_SetItem(list, i, x) < 0) { /* (it takes x) */
                    Py_DECREF(list);
                    list = NULL;
                }
            }
        }
    }
    PyMem_Free(v);
    return list;
}
/* magnetic_field(latitude_rad, longitude_rad, height_m, decimal_year) -> (north_nt, east_nt, down_nt, horizontal_nt,
 * total_nt, declination_rad, inclination_rad): the World Magnetic Model 2025 (ABI 1.20) */
PyObject* mod_magnetic_field(PyObject* m, PyObject* const* args, Py_ssize_t n) {
    (void)m;
    if (!check_args(n, 4, 4, "magnetic_field")) return NULL;
    double v[4];
    for (int k = 0; k < 4; ++k) v[k] = PyFloat_AsDouble(args[k]);
    if (PyErr_Occurred()) return NULL;
    fsim_magnetic_field f;
    fsim_magnetic_field_init(&f);
    if (fsim_magnetic_field_at(v[0], v[1], v[2], v[3], &f) != FSIM_OK) {
        PyErr_SetString(PyExc_ValueError, "magnetic_field: a place or date not finite, or a latitude off the Earth");
        return NULL;
    }
    return Py_BuildValue("(ddddddd)", f.north_nt, f.east_nt, f.down_nt, f.horizontal_nt, f.total_nt, f.declination_rad, f.inclination_rad);
}

/* decimal_year(unix_seconds) -> the year and the part of it gone (ABI 1.20) */
PyObject* mod_decimal_year(PyObject* m, PyObject* const* args, Py_ssize_t n) {
    (void)m;
    if (!check_args(n, 1, 1, "decimal_year")) return NULL;
    const double t = PyFloat_AsDouble(args[0]);
    if (PyErr_Occurred()) return NULL;
    return PyFloat_FromDouble(fsim_decimal_year(t));
}
