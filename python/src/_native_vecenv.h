/* Private VecEnv object layout and callbacks. */
#ifndef FSIM_NATIVE_VECENV_H
#define FSIM_NATIVE_VECENV_H

#include "_native_world.h"

typedef struct {
    PyObject_HEAD
    fsim_vecenv* env;
    int busy;
    Py_ssize_t action_count; /* M*K*A floats per step */
    float* scratch;          /* float64 actions narrowed here */
    fsim_py_ref ref;
} VecEnvObject;

PyObject* vecenv_new(PyTypeObject* type, PyObject* args, PyObject* kwargs);
void vecenv_dealloc(PyObject* o);
PyObject* vecenv_step(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* vecenv_reset(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* vecenv_set_action_ranges(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* vecenv_names(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* vecenv_vehicle_steps(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* vecenv_set_autoreset(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* vecenv_autoreset(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* vecenv_vehicle_ids(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* vecenv_capsule(PyObject* o, PyObject* const* args, Py_ssize_t n);

#endif /* FSIM_NATIVE_VECENV_H */
