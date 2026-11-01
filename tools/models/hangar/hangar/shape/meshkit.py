"""hangar's mesher (tools/models/hangar/native): signed-distance scenes to closed,
simplified triangle meshes, loaded with ctypes from the platform's build
(the staged Python directory, on the path hangar runs with)."""
import ctypes
import json
import os
import sys
import threading

import numpy as np

_LIB = None
_PROBE_TOTALS = dict(queries=0,requested_points=0,native_points=0,cache_hits=0)
_PROBE_TOTALS_LOCK = threading.Lock()


def probe_cache_stats():
    """Process-local diagnostic counts; no geometry result is kept globally."""
    with _PROBE_TOTALS_LOCK:return dict(_PROBE_TOTALS)


class _Mesh(ctypes.Structure):
    _fields_ = [("vertex_count", ctypes.c_int), ("triangle_count", ctypes.c_int),
                ("positions", ctypes.POINTER(ctypes.c_float)), ("normals", ctypes.POINTER(ctypes.c_float)),
                ("indices", ctypes.POINTER(ctypes.c_uint32)), ("materials", ctypes.POINTER(ctypes.c_uint16)),
                ("boundary_edges", ctypes.c_int), ("nonmanifold_edges", ctypes.c_int),
                ("misoriented_edges", ctypes.c_int), ("components", ctypes.c_int),
                ("degenerate_triangles", ctypes.c_int), ("raw_triangles", ctypes.c_int),
                ("fragments_removed", ctypes.c_int),
                ("volume", ctypes.c_double), ("area", ctypes.c_double), ("seconds", ctypes.c_double),
                ("seconds_polygonize", ctypes.c_double), ("seconds_decimate", ctypes.c_double),
                ("error", ctypes.c_char * 512)]


def library():
    """The loaded DLL, or None when the platform was built without it."""
    global _LIB
    if _LIB is not None:
        return _LIB or None
    names = ("hangar_meshkit.dll", "hangar_meshkit.so", "libhangar_meshkit.so")
    for d in [os.environ.get("HANGAR_MESHKIT_DIR", "")] + sys.path:
        for n in names:
            path = os.path.join(d or ".", n)
            if d and os.path.isfile(path):
                lib = ctypes.CDLL(path)
                lib.mk_build.argtypes = [ctypes.c_char_p, ctypes.POINTER(_Mesh)]
                lib.mk_build.restype = ctypes.c_int
                lib.mk_release.argtypes = [ctypes.POINTER(_Mesh)]
                lib.mk_eval.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_double), ctypes.c_int,
                                        ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_int),
                                        ctypes.c_char_p, ctypes.c_int]
                lib.mk_eval.restype = ctypes.c_int
                lib.mk_scene_new.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int]
                lib.mk_scene_new.restype = ctypes.c_void_p
                lib.mk_scene_eval.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_double), ctypes.c_int,
                                              ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_int)]
                lib.mk_scene_eval.restype = None
                lib.mk_scene_free.argtypes = [ctypes.c_void_p]
                lib.mk_scene_free.restype = None
                _LIB = lib
                return lib
    _LIB = False
    return None


def _dump(scene):
    from ..resources import threads
    cap=threads()
    if cap:
        scene=dict(scene,threads=threads(scene.get('threads',0)))
    return json.dumps(scene, separators=(",", ":"), default=_plain).encode()


def _plain(o):
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    raise TypeError("cannot serialise %r" % type(o))


class MeshError(RuntimeError):
    pass


def build(scene):
    """Meshes a scene (see native/meshkit.h). Returns a dict: positions (n,3),
    normals (n,3), triangles (m,3), materials (m,) and the stats the checks
    read."""
    lib = library()
    if lib is None:
        raise MeshError("hangar_meshkit is not built (cmake --build --preset ucrt64-release)")
    m = _Mesh()
    if lib.mk_build(_dump(scene), ctypes.byref(m)) != 0:
        raise MeshError(m.error.decode(errors="replace"))
    try:
        nv, nt = m.vertex_count, m.triangle_count
        out = {"positions": np.ctypeslib.as_array(m.positions, (nv, 3)).astype(np.float64),
               "normals": np.ctypeslib.as_array(m.normals, (nv, 3)).copy(),
               "triangles": np.ctypeslib.as_array(m.indices, (nt, 3)).astype(np.int64),
               "materials": np.ctypeslib.as_array(m.materials, (nt,)).astype(np.int64)}
        for k in ("boundary_edges", "nonmanifold_edges", "misoriented_edges", "components", "degenerate_triangles",
                  "raw_triangles", "fragments_removed", "volume", "area", "seconds", "seconds_polygonize",
                  "seconds_decimate"):
            out[k] = getattr(m, k)
        return out
    finally:
        lib.mk_release(ctypes.byref(m))


class Probe:
    """A scene built once to be evaluated many times (probe(points) -> the
    distances and materials); free it with close(), or use it in a with."""

    def __init__(self, scene, *, cache_capacity=65536):
        if type(cache_capacity) is not int or not 0<=cache_capacity<=65536:
            raise ValueError('Probe cache capacity must be an integer in [0,65536]')
        self._h = None
        self._cache = {}
        self._cache_capacity = cache_capacity
        self._cache_lock = threading.RLock()
        self._stats = dict(queries=0,requested_points=0,native_points=0,cache_hits=0)
        self._lib = library()
        if self._lib is None:
            raise MeshError("hangar_meshkit is not built")
        err = ctypes.create_string_buffer(512)
        self._h = self._lib.mk_scene_new(_dump(scene), err, 512)
        if not self._h:
            raise MeshError(err.value.decode(errors="replace"))

    def __call__(self, points):
        p = np.ascontiguousarray(np.asarray(points, np.float64).reshape(-1, 3))
        if self._cache_capacity and len(p)<=4096:p=p.copy()
        # Keep this handle alive through native evaluation. Independent scenes
        # and native batches may run in parallel; calls on one Probe serialize.
        with self._cache_lock:
            if not self._h:raise MeshError('Probe is closed')
            return self._evaluate(p)

    def _evaluate(self,p):
        d = np.empty(len(p))
        mat = np.empty(len(p), np.int32)
        # Scene handles are immutable. Small fitting queries repeatedly contain
        # identical corners/face centres. Reuse only the exact 24 input bytes:
        # no rounding, proximity matching, source cache or altered fit sample.
        # Large scans bypass the cache so memory and lookup work stay bounded.
        keys=[row.tobytes() for row in p] if self._cache_capacity and len(p)<=4096 else None
        if keys is None:
            with self._cache_lock:
                self._stats['queries']+=1;self._stats['requested_points']+=len(p);self._stats['native_points']+=len(p)
            self._lib.mk_scene_eval(self._h,p.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),len(p),
                d.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),mat.ctypes.data_as(ctypes.POINTER(ctypes.c_int)))
            return d,mat
        missing=[];positions={}
        with self._cache_lock:
            self._stats['queries']+=1;self._stats['requested_points']+=len(p)
            for i,key in enumerate(keys):
                value=self._cache.get(key)
                if value is not None:d[i],mat[i]=value
                else:
                    if key not in positions:positions[key]=[];missing.append(i)
                    positions[key].append(i)
            self._stats['cache_hits']+=len(p)-len(missing)
            self._stats['native_points']+=len(missing)
        if missing:
            query=np.ascontiguousarray(p[missing]);dist=np.empty(len(query));material=np.empty(len(query),np.int32)
            self._lib.mk_scene_eval(self._h,query.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),len(query),
                dist.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),material.ctypes.data_as(ctypes.POINTER(ctypes.c_int)))
            with self._cache_lock:
                for k,i in enumerate(missing):
                    value=(dist[k],material[k])
                    key=keys[i]
                    for index in positions[key]:d[index],mat[index]=value
                    if key not in self._cache and len(self._cache)>=self._cache_capacity:
                        self._cache.pop(next(iter(self._cache)))  # bounded FIFO; eviction changes cost only
                    self._cache[key]=value
        return d, mat

    def close(self):
        if not hasattr(self,'_cache_lock'):return
        with self._cache_lock:
            if self._h:
                self._lib.mk_scene_free(self._h)
                self._h = None
                with _PROBE_TOTALS_LOCK:
                    for key,value in self._stats.items():_PROBE_TOTALS[key]+=value
                self._cache.clear()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def __del__(self):
        self.close()


def evaluate(scene, points):
    """Signed distances and materials of a scene at points (n, 3)."""
    lib = library()
    if lib is None:
        raise MeshError("hangar_meshkit is not built")
    p = np.ascontiguousarray(np.asarray(points, np.float64).reshape(-1, 3))
    n = len(p)
    d = np.empty(n)
    mat = np.empty(n, np.int32)
    err = ctypes.create_string_buffer(512)
    rc = lib.mk_eval(_dump(scene), p.ctypes.data_as(ctypes.POINTER(ctypes.c_double)), n,
                     d.ctypes.data_as(ctypes.POINTER(ctypes.c_double)), mat.ctypes.data_as(ctypes.POINTER(ctypes.c_int)),
                     err, 512)
    if rc != 0:
        raise MeshError(err.value.decode(errors="replace"))
    return d, mat
