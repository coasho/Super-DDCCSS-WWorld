/* Private Recording object layout and callbacks. */
#ifndef FSIM_NATIVE_RECORDING_H
#define FSIM_NATIVE_RECORDING_H

#include "_native_world.h"

typedef struct {
    PyObject_HEAD
    fsim_recording* recording;
} RecordingObject;

PyObject* recording_new(PyTypeObject* type, PyObject* args, PyObject* kwargs);
void recording_dealloc(PyObject* o);
PyObject* recording_info(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* recording_frame_time(PyObject* o, PyObject* const* args, Py_ssize_t n);
PyObject* recording_events(PyObject* o, PyObject* const* args, Py_ssize_t n);

#endif /* FSIM_NATIVE_RECORDING_H */
