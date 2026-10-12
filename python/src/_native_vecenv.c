/* Native VecEnv operations over the C ABI. Registration stays in _native.c. */
#define FSIM_PY_NO_BUFFER_TYPE
#include "_native_vecenv.h"

#define OF(f, k) FSIM_PY_FIELD(fsim_options, f, k)
static const fsim_py_field vecenv_fields[] = {
    OF(num_envs, FSIM_PY_U32), OF(vehicles_per_env, FSIM_PY_U32), OF(workers, FSIM_PY_U32), OF(seed, FSIM_PY_U64),
    OF(aircraft, FSIM_PY_STR), OF(jsbsim_root, FSIM_PY_STR), OF(task, FSIM_PY_STR), OF(observation, FSIM_PY_STR),
    OF(action, FSIM_PY_STR), OF(dt, FSIM_PY_F64), OF(frame_skip, FSIM_PY_I32), OF(max_episode_steps, FSIM_PY_U32),
    OF(latitude_deg, FSIM_PY_F64), OF(longitude_deg, FSIM_PY_F64), OF(altitude_m, FSIM_PY_F64), OF(heading_deg, FSIM_PY_F64),
    OF(airspeed_ms, FSIM_PY_F64), OF(latitude_jitter_deg, FSIM_PY_F64), OF(longitude_jitter_deg, FSIM_PY_F64),
    OF(altitude_jitter_m, FSIM_PY_F64), OF(heading_jitter_deg, FSIM_PY_F64), OF(airspeed_jitter_ms, FSIM_PY_F64),
    OF(target_altitude_delta_m, FSIM_PY_F64), OF(target_heading_delta_deg, FSIM_PY_F64), OF(world_name, FSIM_PY_STR),
    OF(publish, FSIM_PY_I32), OF(terrain, FSIM_PY_I32), OF(scenario_path, FSIM_PY_STR)};
#undef OF

PyObject* vecenv_new(PyTypeObject* type, PyObject* args, PyObject* kwargs) {
    PyObject* options = NULL;
    if (kwargs && PyDict_Size(kwargs)) {
        PyErr_SetString(PyExc_TypeError, "VecEnv(options): options is a dict");
        return NULL;
    }
    if (!PyArg_ParseTuple(args, "|O:VecEnv", &options)) return NULL;
    fsim_options o;
    fsim_options_init(&o);
    if (fsim_py_set_fields(options, &o, vecenv_fields, COUNT(vecenv_fields), "VecEnv") < 0) return NULL;

    allocfunc alloc = (allocfunc)PyType_GetSlot(type, Py_tp_alloc);
    VecEnvObject* self = (VecEnvObject*)alloc(type, 0);
    if (!self) return NULL;
    int rc;
    fsim_vecenv* env = NULL;
    Py_BEGIN_ALLOW_THREADS
    rc = fsim_vecenv_create(&o, &env);
    Py_END_ALLOW_THREADS
    if (rc != FSIM_OK) {
        Py_DECREF(self);
        return fail();
    }
    self->env = env;
    fsim_buffers b;
    memset(&b, 0, sizeof b);
    b.struct_size = sizeof b;
    fsim_vecenv_buffers(env, &b);
    self->action_count = (Py_ssize_t)b.num_envs * b.vehicles_per_env * b.action_size;
    self->ref.handle = env;
    self->ref.busy = &self->busy;
    return (PyObject*)self;
}

void vecenv_dealloc(PyObject* o) {
    VecEnvObject* self = (VecEnvObject*)o;
    PyTypeObject* tp = Py_TYPE(o);
    if (self->env) fsim_vecenv_destroy(self->env);
    PyMem_Free(self->scratch);
    freefunc f = (freefunc)PyType_GetSlot(tp, Py_tp_free);
    f(o);
    Py_DECREF(tp);
}

/* step(actions): M*K*A float32 (read in place) or float64 (narrowed). */
PyObject* vecenv_step(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    VecEnvObject* self = (VecEnvObject*)o;
    if (!check_args(n, 1, 1, "step") || !fsim_py_idle(&self->busy, "this VecEnv")) return NULL;
    Py_buffer b;
    if (PyObject_GetBuffer(args[0], &b, PyBUF_C_CONTIGUOUS | PyBUF_FORMAT) < 0) {
        /* numpy says ValueError for a strided array, others BufferError or
         * TypeError: all mean "convert it first", which fsim.VecEnv does. */
        PyErr_Clear();
        PyErr_SetString(PyExc_TypeError, "step: actions must be a C-contiguous float32 or float64 array");
        return NULL;
    }
    const char* f = b.format ? b.format : "B";
    if (*f == '<' || *f == '=' || *f == '@') ++f;
    const float* actions = NULL;
    if (f[0] == 'f' && f[1] == '\0' && b.itemsize == 4) {
        actions = (const float*)b.buf;
    } else if (f[0] == 'd' && f[1] == '\0' && b.itemsize == 8) {
        if (!self->scratch) self->scratch = (float*)PyMem_Malloc(sizeof(float) * (size_t)(self->action_count ? self->action_count : 1));
        if (!self->scratch) {
            PyBuffer_Release(&b);
            return PyErr_NoMemory();
        }
        const Py_ssize_t count = b.len / 8 < self->action_count ? b.len / 8 : self->action_count;
        for (Py_ssize_t i = 0; i < count; ++i) self->scratch[i] = (float)((const double*)b.buf)[i];
        actions = self->scratch;
    } else {
        PyBuffer_Release(&b);
        PyErr_Format(PyExc_TypeError, "step: actions must be float32 or float64, got '%s'", b.format ? b.format : "B");
        return NULL;
    }
    if (b.len / b.itemsize != self->action_count) {
        PyErr_Format(PyExc_ValueError, "step: expected %zd action values (vehicles x action size), got %zd", self->action_count,
                     b.len / b.itemsize);
        PyBuffer_Release(&b);
        return NULL;
    }
    int rc;
    self->busy = 1;
    Py_BEGIN_ALLOW_THREADS
    rc = fsim_vecenv_step(self->env, actions, (size_t)self->action_count);
    Py_END_ALLOW_THREADS
    self->busy = 0;
    PyBuffer_Release(&b);
    if (rc != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

PyObject* vecenv_reset(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    VecEnvObject* self = (VecEnvObject*)o;
    if (!check_args(n, 0, 1, "reset") || !fsim_py_idle(&self->busy, "this VecEnv")) return NULL;
    unsigned long long seed = 0;
    if (n == 1 && args[0] != Py_None) {
        seed = PyLong_AsUnsignedLongLong(args[0]);
        if (seed == (unsigned long long)-1 && PyErr_Occurred()) return NULL;
    }
    int rc;
    self->busy = 1;
    Py_BEGIN_ALLOW_THREADS
    rc = fsim_vecenv_reset(self->env, (uint64_t)seed);
    Py_END_ALLOW_THREADS
    self->busy = 0;
    if (rc != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

static PyObject* names_list(const char* (*get)(const fsim_vecenv*, uint32_t), const fsim_vecenv* env, uint32_t count) {
    PyObject* list = PyList_New((Py_ssize_t)count);
    if (!list) return NULL;
    for (uint32_t i = 0; i < count; ++i) {
        PyObject* s = PyUnicode_FromString(get(env, i));
        if (!s) {
            Py_DECREF(list);
            return NULL;
        }
        PyList_SetItem(list, (Py_ssize_t)i, s);
    }
    return list;
}

PyObject* vecenv_set_action_ranges(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    VecEnvObject* self = (VecEnvObject*)o;
    if (!check_args(n, 1, 1, "set_action_ranges")) return NULL;
    const char* mode = as_str(args[0], "action ranges");
    if (!mode) return NULL;
    if (fsim_vecenv_set_action_ranges(self->env, mode) != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

PyObject* vecenv_names(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    VecEnvObject* self = (VecEnvObject*)o;
    (void)args;
    if (!check_args(n, 0, 0, "names")) return NULL;
    fsim_buffers b;
    memset(&b, 0, sizeof b);
    b.struct_size = sizeof b;
    fsim_vecenv_buffers(self->env, &b);
    PyObject* obs = names_list(fsim_vecenv_observation_name, self->env, b.observation_size);
    PyObject* act = obs ? names_list(fsim_vecenv_action_name, self->env, b.action_size) : NULL;
    if (!act) {
        Py_XDECREF(obs);
        return NULL;
    }
    return Py_BuildValue("(NN)", obs, act);
}

PyObject* vecenv_vehicle_steps(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    (void)args;
    if (!check_args(n, 0, 0, "vehicle_steps")) return NULL;
    return PyLong_FromUnsignedLongLong(fsim_vecenv_vehicle_steps(((VecEnvObject*)o)->env));
}

PyObject* vecenv_set_autoreset(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    VecEnvObject* self = (VecEnvObject*)o;
    int mode;
    if (!check_args(n, 1, 1, "set_autoreset") || !as_int(args[0], &mode) || !fsim_py_idle(&self->busy, "this VecEnv")) return NULL;
    if (fsim_vecenv_set_autoreset(self->env, mode) != FSIM_OK) return fail();
    Py_RETURN_NONE;
}

PyObject* vecenv_autoreset(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    (void)args;
    if (!check_args(n, 0, 0, "autoreset")) return NULL;
    return PyLong_FromLong(fsim_vecenv_autoreset(((VecEnvObject*)o)->env));
}

PyObject* vecenv_vehicle_ids(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    VecEnvObject* self = (VecEnvObject*)o;
    (void)args;
    if (!check_args(n, 0, 0, "vehicle_ids")) return NULL;
    const uint32_t count = fsim_vecenv_vehicle_ids(self->env, NULL, 0);
    PyObject* bytes = PyBytes_FromStringAndSize(NULL, (Py_ssize_t)count * 4);
    if (!bytes) return NULL;
    fsim_vecenv_vehicle_ids(self->env, (uint32_t*)PyBytes_AsString(bytes), count);
    return bytes; /* uint32 in batch order; numpy.frombuffer(..., np.uint32) */
}

PyObject* vecenv_capsule(PyObject* o, PyObject* const* args, Py_ssize_t n) {
    (void)args;
    if (!check_args(n, 0, 0, "_capsule")) return NULL;
    return PyCapsule_New(&((VecEnvObject*)o)->ref, FSIM_PY_VECENV_CAPSULE, NULL);
}
