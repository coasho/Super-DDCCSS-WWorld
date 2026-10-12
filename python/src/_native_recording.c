/* Native Recording operations over the C ABI. Registration stays in _native.c. */
#define FSIM_PY_NO_BUFFER_TYPE
#include "_native_recording.h"

PyObject* recording_new(PyTypeObject* type, PyObject* args, PyObject* kwargs) {
    const char* path;
    if (kwargs && PyDict_Size(kwargs)) {
        PyErr_SetString(PyExc_TypeError, "Recording(path)");
        return NULL;
    }
    if (!PyArg_ParseTuple(args, "s:Recording", &path)) return NULL;
    fsim_recording* r = NULL;
    int rc;
    Py_BEGIN_ALLOW_THREADS
    rc = fsim_recording_load(path, &r);
    Py_END_ALLOW_THREADS
    if (rc != FSIM_OK) return fail();
    allocfunc alloc = (allocfunc)PyType_GetSlot(type, Py_tp_alloc);
    RecordingObject* self = (RecordingObject*)alloc(type, 0);
    if (!self) {
        fsim_recording_destroy(r);
        return NULL;
    }
    self->recording = r;
    return (PyObject*)self;
}

void recording_dealloc(PyObject* o) {
    PyTypeObject* tp = Py_TYPE(o);
    fsim_recording_destroy(((RecordingObject*)o)->recording);
    freefunc f = (freefunc)PyType_GetSlot(tp, Py_tp_free);
    f(o);
    Py_DECREF(tp);
}

PyObject* recording_info(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    const fsim_recording* r = ((RecordingObject*)o)->recording;
    (void)args;
    if (!check_args(n, 0, 0, "info")) return NULL;
    return Py_BuildValue("{s:I,s:s,s:d,s:i}", "frame_count", fsim_recording_frame_count(r), "world_name",
                         fsim_recording_world_name(r), "dt", fsim_recording_dt(r), "frame_skip", (int)fsim_recording_frame_skip(r));
}

PyObject* recording_frame_time(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    uint32_t frame;
    if (!check_args(n, 1, 1, "frame_time") || !as_u32(args[0], &frame)) return NULL;
    return PyFloat_FromDouble(fsim_recording_frame_time(((RecordingObject*)o)->recording, frame));
}

PyObject* recording_events(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    uint32_t frame, count = 0;
    if (!check_args(n, 1, 1, "events") || !as_u32(args[0], &frame)) return NULL;
    const fsim_recorded_event* e = fsim_recording_events(((RecordingObject*)o)->recording, frame, &count);
    PyObject* list = PyList_New((Py_ssize_t)count);
    if (!list) return NULL;
    for (uint32_t i = 0; i < count; ++i) {
        PyObject* d = Py_BuildValue(
            "{s:I,s:I,s:K,s:O,s:i,s:s,s:s,s:s,s:d,s:d,s:d,s:d}", "slot", e[i].slot, "id", e[i].id, "generation",
            (unsigned long long)e[i].generation, "alive", e[i].alive ? Py_True : Py_False, "control_level", (int)e[i].control_level,
            "name", e[i].name ? e[i].name : "", "type", e[i].type ? e[i].type : "", "model", e[i].model ? e[i].model : "",
            "initial_latitude_deg", e[i].initial_latitude_deg, "initial_longitude_deg", e[i].initial_longitude_deg,
            "initial_altitude_msl_m", e[i].initial_altitude_msl_m, "initial_heading_deg", e[i].initial_heading_deg);
        if (!d) {
            Py_DECREF(list);
            return NULL;
        }
        PyList_SetItem(list, (Py_ssize_t)i, d);
    }
    return list;
}
