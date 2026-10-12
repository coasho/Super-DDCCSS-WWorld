/* Native Scenario lifecycle and queries. Registration stays in _native.c. */
#define FSIM_PY_NO_BUFFER_TYPE
#include "_native_world.h"
#include "_native_world_options.h"

/* Scenario(path) or Scenario(None, json, source_name) */
PyObject* scenario_new(PyTypeObject* type, PyObject* args, PyObject* kwargs) {
    PyObject *path = Py_None, *json = Py_None;
    const char* source = "inline";
    if (kwargs && PyDict_Size(kwargs)) {
        PyErr_SetString(PyExc_TypeError, "Scenario(path) or Scenario(None, json, source)");
        return NULL;
    }
    if (!PyArg_ParseTuple(args, "|OOs:Scenario", &path, &json, &source)) return NULL;
    fsim_scenario* sc = NULL;
    int rc;
    if (path != Py_None) {
        const char* p = as_str(path, "path");
        if (!p) return NULL;
        rc = fsim_scenario_load(p, &sc);
    } else {
        const char* text = as_str(json, "json");
        if (!text) return NULL;
        rc = fsim_scenario_parse(text, source, &sc);
    }
    if (rc != FSIM_OK) return fail();
    allocfunc alloc = (allocfunc)PyType_GetSlot(type, Py_tp_alloc);
    ScenarioObject* self = (ScenarioObject*)alloc(type, 0);
    if (!self) {
        fsim_scenario_destroy(sc);
        return NULL;
    }
    self->scenario = sc;
    return (PyObject*)self;
}

void scenario_dealloc(PyObject* o) {
    PyTypeObject* tp = Py_TYPE(o);
    fsim_scenario_destroy(((ScenarioObject*)o)->scenario);
    freefunc f = (freefunc)PyType_GetSlot(tp, Py_tp_free);
    f(o);
    Py_DECREF(tp);
}

PyObject* scenario_world_options(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    (void)args;
    if (!check_args(n, 0, 0, "world_options")) return NULL;
    fsim_world_options wo;
    fsim_world_options_init(&wo);
    if (fsim_scenario_world_options(((ScenarioObject*)o)->scenario, &wo) != FSIM_OK) return fail();
    return fsim_py_get_fields(&wo, world_fields, COUNT(world_fields));
}

PyObject* scenario_vehicle_count(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    (void)args;
    if (!check_args(n, 0, 0, "vehicle_count")) return NULL;
    return PyLong_FromUnsignedLong(fsim_scenario_vehicle_count(((ScenarioObject*)o)->scenario));
}
