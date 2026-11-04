"""M03 liquid occupancy, independent of flight mass and vulnerability hit probes.

Every emitted cell is a disjoint, spatial box certified inside the delivered
neutral structural GLB by Manifold Boolean operations. Capacity fixes volume at
the declared physical density. Neither a fitted probe nor a density adjustment
may supply missing volume. See docs/research/sim-m03/liquid-occupancy-method.md.
"""
import hashlib
import heapq
import importlib.metadata
import json
import math
import multiprocessing as mp
import numbers
import os
from pathlib import Path
import struct
import tempfile
import time

import numpy as np

from .files import atomic_write_text

SCHEMA = 'fsim.liquid-occupancy.v1'
FRAME = 'body_origin_forward_right_down_m'
FLUID_POLICY_V1 = dict(law='quadratic_unresolved_tumbling_v1', drag_coefficient=1.0, basis='assumed',
                       screening_uncertainty_fraction=1.0,
                       note='Unresolved tumbling pressure-drag engineering policy; not FAA/OEM calibration or air Cd.')
DEFAULT_LIMITS = dict(max_cells=65536, max_screened_cells=1048576,
                      max_assignment_edges=2097152, max_depth=60,
                      min_extent_m=1e-7, max_seconds=300.0,
                      max_glb_bytes=268435456, max_vertices=2097152,
                      max_triangles=2097152)
# model3d._to_gltf(design, origin) = [-dy, +dz, -dx].
# Native body from that same origin is [-dx, +dy, -dz].
_GLTF_BODY = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])


class LiquidError(RuntimeError):
    """A required geometric, material, source or finite-budget check failed."""


class _OwnerBudget:
    """One shared deadline and actual cumulative work ceiling for an invocation."""
    def __init__(self,context,limits,deadline,lanes):
        self.limits=_limits(limits);self.deadline=deadline;self.lanes=lanes
        self._values=context.RawArray('Q',3);self._lock=context.Lock();self.cancel=context.Event()
        self.receipts=[]

    def check(self):
        if time.monotonic()>self.deadline:raise LiquidError('Liquid global time deadline exceeded')
        if self.cancel.is_set():raise LiquidError('Liquid owner execution cancelled')

    def _charge(self,index,count,limit,name):
        if type(count) is not int or count<0:raise LiquidError('Invalid shared '+name+' charge')
        with self._lock:
            if self._values[index]+count>self.limits[limit]:raise LiquidError('Liquid global '+name+' budget exceeded')
            self.check();self._values[index]+=count

    def screen(self):self._charge(0,1,'max_screened_cells','screen')
    def reserve_edges(self,count):self._charge(1,count,'max_assignment_edges','edge')
    def emit_cell(self):self._charge(2,1,'max_cells','cell')
    def counts(self):
        with self._lock:return dict(zip(('screened','edges','cells'),list(self._values)))


class _OwnerExecution:
    def __init__(self,budget,lane):
        self.budget=budget;self.cache_metrics=None
        self.cache_limits={key:int(budget.limits[key])//budget.lanes+(lane<int(budget.limits[key])%budget.lanes)
            for key in ('max_vertices','max_triangles','max_screened_cells')}
    def check(self):self.budget.check()
    def screen(self):self.budget.screen()
    def reserve_edges(self,count):self.budget.reserve_edges(count)
    def emit_cell(self):self.budget.emit_cell()


class _OwnerProcess(mp.context.SpawnProcess):
    @staticmethod
    def _Popen(process_obj):
        from .resources import leased_spawn_popen
        return leased_spawn_popen(process_obj,process_obj._lease_cpus)


def _owner_process_entry(worker,recipe,budget,lane,cpu,path,ordinal):
    from . import resources
    execution=_OwnerExecution(budget,lane)
    started=time.monotonic()
    try:
        with resources.cpu_lease(1,cpus=(cpu,)):
            execution.check();result=worker(recipe,execution);execution.check()
            message=dict(ok=True,ordinal=ordinal,result=result,cpus=list(resources.cpu_affinity()),
                         cache=execution.cache_metrics,elapsed_s=time.monotonic()-started)
    except BaseException as error:
        message=dict(ok=False,ordinal=ordinal,error=type(error).__name__+': '+str(error))
    temporary=Path(path+'.tmp')
    with temporary.open('x',encoding='utf-8') as stream:
        json.dump(message,stream,sort_keys=True,allow_nan=False,separators=(',',':'))
    os.replace(temporary,path)


def _process_owner_jobs(jobs,worker,budget,cpus):
    """Owned spawn lifetime; outputs are private and admitted in ordinal order."""
    from . import resources
    cpus=tuple(cpus)
    if set(resources.cpu_affinity())!=set(cpus) or not 1<=len(cpus)<=2:
        raise LiquidError('Owner coordinator lacks its verified whole-process CPU lease')
    try:
        if not isinstance(jobs,list) or not 1<=len(jobs)<=128:raise ValueError('Invalid owner jobs')
        encoded=json.dumps(jobs,allow_nan=False,separators=(',',':'))
        if len(encoded.encode())>32768:raise ValueError('Oversized owner recipe')
    except (ValueError,TypeError,OverflowError) as error:raise LiquidError('Invalid plain owner job recipe') from error
    processes={};next_job=0;results=[None]*len(jobs);failure=None
    root='D:/ENV/tmp' if os.name=='nt' else None
    with tempfile.TemporaryDirectory(prefix='hangar-liquid-owners-',dir=root) as folder:
        directory=Path(folder)
        def start(lane):
            nonlocal next_job
            index=next_job;next_job+=1;path=directory/(str(index)+'.json')
            process=_OwnerProcess(target=_owner_process_entry,args=(worker,jobs[index],budget,lane,cpus[lane],str(path),index))
            process._lease_cpus=(cpus[lane],)
            resources.start_owned_process(process,budget.deadline);processes[lane]=(process,index,path)
        try:
            budget.check()
            for lane in range(min(len(cpus),len(jobs))):start(lane)
            while processes:
                budget.check()
                for lane,(process,index,path) in list(processes.items()):
                    if process.exitcode is None:continue
                    process.join(timeout=0)
                    if process.exitcode or not path.is_file():raise LiquidError('Owner worker exited without a complete result')
                    if path.stat().st_size>budget.limits['max_glb_bytes']:raise LiquidError('Oversized owner result')
                    message=json.loads(path.read_text(encoding='utf-8'))
                    if message.get('ordinal')!=index or type(message.get('ok')) is not bool:
                        raise LiquidError('Malformed owner completion')
                    if not message['ok']:raise LiquidError(message.get('error','Owner worker refused'))
                    if message.get('cpus')!=[cpus[lane]]:raise LiquidError('Owner worker escaped its single-CPU lease')
                    budget.receipts.append(dict(ordinal=index,lane=lane,cpus=message['cpus'],cache=message.get('cache'),elapsed_s=message.get('elapsed_s')))
                    results[index]=message['result']
                    resources.close_spawn_jobs(process._popen);process.close();del processes[lane]
                    if next_job<len(jobs):start(lane)
                if processes:time.sleep(.005)
            budget.check()
        except BaseException as error:
            failure=error;budget.cancel.set()
        finally:
            cleanup_failure=None
            for process,_,_ in processes.values():
                try:
                    if process.is_alive():process.terminate()
                    process.join(timeout=5)
                    if process.is_alive():raise RuntimeError('Owner worker did not join; CPU lease cannot be released')
                    resources.close_spawn_jobs(process._popen);process.close()
                except BaseException as error:cleanup_failure=error
            if cleanup_failure is not None:raise cleanup_failure
        if failure is not None:
            if isinstance(failure,LiquidError):raise failure
            raise LiquidError('Owner process execution refused: '+str(failure)) from failure
    return results


def _mf():
    try:
        import manifold3d
    except ImportError as refused:
        raise LiquidError('Liquid occupancy requires the declared manifold3d dependency') from refused
    return manifold3d


def _limits(overrides):
    result = dict(DEFAULT_LIMITS)
    if overrides:
        if set(overrides)-set(result):
            raise LiquidError('Unknown liquid occupancy resource limit')
        result.update(overrides)
    for key, value in result.items():
        if isinstance(value, bool) or not isinstance(value, numbers.Real) or not math.isfinite(value) or value <= 0:
            raise LiquidError('Invalid liquid occupancy limit: '+key)
        if key.startswith('max_') and key != 'max_seconds' and int(value) != value:
            raise LiquidError('Integer liquid occupancy limit required: '+key)
    return result


def _valid(solid, context, *, nonempty=False):
    if solid.status() != _mf().Error.NoError:
        raise LiquidError(context+': manifold status '+str(solid.status()))
    volume = solid.volume()
    if not math.isfinite(volume) or volume < 0:
        raise LiquidError(context+': nonfinite or negative solid volume')
    if nonempty and (solid.is_empty() or volume <= 0):
        raise LiquidError(context+': closed positive-volume solid required')
    return solid


def _vec(value, context):
    a = np.asarray(value, dtype=float)
    if a.shape != (3,) or not np.isfinite(a).all():
        raise LiquidError(context+': finite three-vector required')
    return a


def _body(point, origin):
    return (np.asarray(point, float)-origin)*[-1., 1., -1.]


def _interval(lo, hi):
    if not math.isfinite(lo) or not math.isfinite(hi):
        raise LiquidError('Owner interval arithmetic exceeded finite numerical bounds')
    return math.nextafter(lo, -math.inf), math.nextafter(hi, math.inf)


def _add(a, b): return _interval(a[0]+b[0], a[1]+b[1])
def _sub(a, b): return _interval(a[0]-b[1], a[1]-b[0])
def _mul(a, b):
    values = [x*y for x in a for y in b]
    return _interval(min(values), max(values))
def _div(a, b):
    if b[0] <= 0 <= b[1]: raise LiquidError('Owner interval division crosses zero')
    values = [x/y for x in a for y in b]
    return _interval(min(values), max(values))
def _abs(a): return (0. if a[0] <= 0 <= a[1] else min(abs(a[0]), abs(a[1])), max(abs(a[0]), abs(a[1])))
def _point(x): return float(x), float(x)
def _ge(a, b, strict=False):
    if (a[0] > b[1]) if strict else (a[0] >= b[1]): return 1
    if (a[1] <= b[0]) if strict else (a[1] < b[0]): return -1
    return 0
def _dot(a, b):
    result = (0., 0.)
    for x, coefficient in zip(a, b): result = _add(result, _mul(x, _point(coefficient)))
    return result


class _OwnerClassifier:
    """Inclusion enclosure of the frozen native mass/breakup point partition.

    -1/0/+1 mean false/unknown/true. Every elementary floating arithmetic
    operation rounds its lower/upper result outward by nextafter. Constants
    (including normalized segment directions) are computed identically to the
    existing classifier; no sampled corner/centre becomes a box certificate.
    """
    def __init__(self, aircraft, origin):
        from .shape import breakup
        self.origin = _vec(origin, 'owner origin'); self.breakup = breakup
        wings = [s for s in aircraft.surfaces if s.kind == 'wing']
        self.unsectioned = not wings
        if self.unsectioned: return
        tails = [s for s in aircraft.surfaces if s.kind in ('htail', 'fin', 'vtail')]
        self.x_nose = min(s.sections[0].le[0] for s in wings)-breakup.NOSE_AHEAD
        self.x_tail = min(s.sections[0].le[0] for s in tails)-breakup.TAIL_AHEAD if tails else None
        root = wings[0].sections[0]
        self.y_root = breakup._fuselage_half_width(aircraft, root.le[0]+.5*root.chord)+.05
        self.surfaces = []
        for surface in ([s for s in aircraft.surfaces if s.kind == 'canard']+wings+tails):
            segments = []
            for a, b in zip(surface.sections[:-1], surface.sections[1:]):
                la, lb = np.asarray(a.le, float), np.asarray(b.le, float)
                span = np.array([0., lb[1]-la[1], lb[2]-la[2]]); length = np.linalg.norm(span)
                if length < 1e-9: continue
                direction = span/length; normal = np.cross([1., 0., 0.], direction)
                segments.append((la, lb, float(a.chord), float(b.chord), float(length), direction,
                                 normal, float((lb-la)@normal)))
            self.surfaces.append((surface.kind, surface.mirror, segments))

    def _planform(self, q, mirror, segments):
        q = list(q)
        if mirror: q[1] = _abs(q[1])
        predicates = []
        for la, lb, chord_a, chord_b, length, direction, normal, normal_run in segments:
            delta = [_sub(x, _point(c)) for x, c in zip(q, la)]
            t = _div(_dot(delta, direction), _point(length))
            tc = (max(0., min(1., t[0])), max(0., min(1., t[1])))
            leading = _add(_point(la[0]), _mul(tc, _point(lb[0]-la[0])))
            chord = _add(_point(chord_a), _mul(tc, _point(chord_b-chord_a)))
            plane = _abs(_sub(_dot(delta, normal), _mul(tc, _point(normal_run))))
            predicates.append(min(_ge(t, _point(-.02)), _ge(_point(1.02), t),
                                  _ge(q[0], _sub(leading, _point(self.breakup.CHORD_MARGIN))),
                                  _ge(_add(_add(leading, chord), _point(self.breakup.CHORD_MARGIN)), q[0]),
                                  _ge(_add(_point(self.breakup.PLANE), _mul(_point(.1), chord)), plane)))
        return max(predicates, default=-1)

    def possible(self, low, high):
        if self.unsectioned: return frozenset({'airframe'})
        low = _vec(low, 'owner box low'); high = _vec(high, 'owner box high')
        if np.any(low > high): raise LiquidError('Reversed owner certificate box')
        body = list(zip(low, high)); o = self.origin
        q = [_sub(_point(o[0]), body[0]), _add(_point(o[1]), body[1]), _sub(_point(o[2]), body[2])]
        owners = {'centre'}
        def assign(condition, name):
            nonlocal owners
            if condition == 1: owners = {name}
            elif condition == 0: owners.add(name)
        assign(_ge(_point(self.x_nose), q[0], strict=True), 'nose')
        if self.x_tail is not None: assign(_ge(q[0], _point(self.x_tail), strict=True), 'tail')
        for kind, mirror, segments in self.surfaces:
            inside = self._planform(q, mirror, segments)
            if kind == 'canard': assign(inside, 'nose')
            elif kind == 'wing':
                outboard = _ge(_abs(q[1]), _point(self.y_root), strict=True)
                assign(min(inside, outboard, _ge(_point(0.), q[1], strict=True)), 'left wing')
                assign(min(inside, outboard, _ge(q[1], _point(0.), strict=True)), 'right wing')
            else: assign(inside, 'tail')
        return frozenset(owners)

    def candidate_boxes(self, owner, low, high):
        """Necessary analytic bounds only; never a substitute for certification.

        Each segment's t/chord/plane bands form an enclosing parallelepiped.
        Solve its frozen Y/Z projection matrix with outward interval arithmetic;
        priority overrides can remove points but cannot expand these owner unions.
        """
        low, high = _vec(low, 'candidate low'), _vec(high, 'candidate high')
        if self.unsectioned: return [(low, high)]
        o = self.origin
        envelope = [_sub(_point(o[0]), (low[0], high[0])),
                    _add(_point(o[1]), (low[1], high[1])),
                    _sub(_point(o[2]), (low[2], high[2]))]
        boxes = []
        def add(box):
            box = [(max(a, e[0]), min(b, e[1])) for (a, b), e in zip(box, envelope)]
            if all(a <= b for a, b in box): boxes.append(box)
        if owner == 'centre':
            box = list(envelope); box[0] = (max(box[0][0], self.x_nose),
                min(box[0][1], self.x_tail) if self.x_tail is not None else box[0][1]); add(box)
        elif owner == 'nose':
            box = list(envelope); box[0] = (box[0][0], min(box[0][1], self.x_nose)); add(box)
        elif owner == 'tail' and self.x_tail is not None:
            box = list(envelope); box[0] = (max(box[0][0], self.x_tail), box[0][1]); add(box)
        wanted = ('wing',) if owner in ('left wing', 'right wing') else ('canard',) if owner == 'nose' else ('htail', 'fin', 'vtail') if owner == 'tail' else ()
        for kind, mirror, segments in self.surfaces:
            if kind not in wanted: continue
            for la, lb, ca, cb, length, direction, normal, normal_run in segments:
                tc = (0., 1.)
                leading = _add(_point(la[0]), _mul(tc, _point(lb[0]-la[0])))
                chord = _add(_point(ca), _mul(tc, _point(cb-ca)))
                xb = (_sub(leading, _point(self.breakup.CHORD_MARGIN))[0],
                      _add(_add(leading, chord), _point(self.breakup.CHORD_MARGIN))[1])
                along = _mul((-.02, 1.02), _point(length))
                margin = _add(_point(self.breakup.PLANE), _mul(_point(.1), chord))[1]
                across = _add((-margin, margin), _mul(tc, _point(normal_run)))
                sy, sz, ny, nz = map(_point, (direction[1], direction[2], normal[1], normal[2]))
                determinant = _sub(_mul(sy, nz), _mul(sz, ny))
                yb = _add(_point(la[1]), _div(_sub(_mul(nz, along), _mul(sz, across)), determinant))
                zb = _add(_point(la[2]), _div(_sub(_mul(sy, across), _mul(ny, along)), determinant))
                if mirror:
                    yb = (max(0., yb[0]), yb[1])
                    candidates = [yb, (-yb[1], -yb[0])]
                else: candidates = [yb]
                for y in candidates:
                    if owner == 'left wing': y = (y[0], min(y[1], -self.y_root))
                    if owner == 'right wing': y = (max(y[0], self.y_root), y[1])
                    add([xb, y, zb])
        if not boxes: raise LiquidError('insufficient possible native-owner candidate volume')
        result = []
        for bounds in boxes:
            body = [_sub(_point(o[0]), bounds[0]), _sub(bounds[1], _point(o[1])), _sub(_point(o[2]), bounds[2])]
            result.append((np.maximum(low, [b[0] for b in body]), np.minimum(high, [b[1] for b in body])))
        return result

    def candidate_bounds(self, owner, low, high):
        boxes = self.candidate_boxes(owner, low, high)
        return np.min([a for a, _ in boxes], axis=0), np.max([b for _, b in boxes], axis=0)

    def candidate_region(self, owner, low, high):
        boxes = [_box(a, b) for a, b in self.candidate_boxes(owner, low, high) if np.all(b > a)]
        if not boxes: raise LiquidError('insufficient positive native-owner candidate region')
        return _valid(_mf().Manifold.batch_boolean(boxes, _mf().OpType.Add), 'Necessary classifier box union', nonempty=True)


def owner_candidates(aircraft, low_body, high_body, origin):
    return _OwnerClassifier(aircraft, origin).possible(low_body, high_body)


def tank_records(aircraft, origin):
    """Native tank enumeration and units; helicopters retain aggregate tank 0."""
    origin = _vec(origin, 'origin')
    spec = aircraft.spec
    avgas = bool(aircraft.engines) and all(e.type == 'piston' for e in aircraft.engines)
    density = 720.0 if avgas else 800.0
    result = []
    for index, item in enumerate(spec.get('mass', {}).get('tank', [])):
        capacity = float(item.get('capacity', 0.))
        if not math.isfinite(capacity) or capacity < 0:
            raise LiquidError('Invalid authored tank capacity')
        if capacity == 0:
            continue
        result.append(dict(index=index, capacity_kg=capacity, density_kg_m3=density,
                           anchor_body_m=_body(_vec(item.get('position', [0, 0, 0]), 'tank anchor'), origin).tolist()))
    if spec.get('aircraft', {}).get('kind') == 'helicopter':
        if result:
            raise LiquidError('Ambiguous rotor aggregate and fixed-wing tank declarations')
        capacity = float(spec.get('fuel', {}).get('capacity_lb', 0.))*.45359237
        if not math.isfinite(capacity) or capacity < 0:
            raise LiquidError('Invalid authored rotor fuel capacity')
        if capacity > 0:
            anchor = _vec(spec['mass']['cg_in'], 'rotor fuel point')*.0254
            result.append(dict(index=0, capacity_kg=capacity, density_kg_m3=800.,
                               anchor_body_m=_body(anchor, origin).tolist()))
    return result


def _transform(node):
    if 'matrix' in node:
        if any(k in node for k in ('translation', 'rotation', 'scale')):
            raise LiquidError('GLB node has both matrix and TRS')
        a = np.asarray(node['matrix'], float)
        if a.shape != (16,):
            raise LiquidError('Invalid GLB node matrix')
        matrix = a.reshape(4, 4, order='F')
    else:
        translation = _vec(node.get('translation', [0, 0, 0]), 'GLB translation')
        scale = _vec(node.get('scale', [1, 1, 1]), 'GLB scale')
        q = np.asarray(node.get('rotation', [0, 0, 0, 1]), float)
        if q.shape != (4,) or not np.isfinite(q).all() or abs(np.dot(q, q)-1) > 1e-5:
            raise LiquidError('GLB rotation is not a unit quaternion')
        q /= np.linalg.norm(q)
        x, y, z, w = q
        rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                             [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                             [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
        matrix = np.eye(4); matrix[:3, :3] = rotation@np.diag(scale)
        matrix[:3, 3] = translation
    if not np.isfinite(matrix).all() or not np.array_equal(matrix[3], [0, 0, 0, 1]) or np.linalg.det(matrix[:3, :3]) == 0:
        raise LiquidError('GLB transform is not finite nonsingular affine')
    return matrix


def read_glb(path, *, limits=None, with_sections=False, owner_names_only=False):
    """Import own neutral structural meshes with every ancestor transform.

    Material-split vertices weld at exact float coordinates. No hole filling,
    tolerance expansion or smooth-geometry change repairs an invalid source.
    Real sectioned models require the original unsplit exterior accessors in
    extras. Inferred breakup caps are placement bounds, not a material envelope.
    """
    limits = _limits(limits); mf = _mf(); path = Path(path)
    if path.stat().st_size > limits['max_glb_bytes']:
        raise LiquidError('GLB byte budget exceeded')
    raw = path.read_bytes()
    if len(raw) < 20 or struct.unpack_from('<III', raw) != (0x46546c67, 2, len(raw)):
        raise LiquidError('Invalid GLB header or length')
    chunks = {}; cursor = 12
    while cursor < len(raw):
        if cursor+8 > len(raw): raise LiquidError('Truncated GLB chunk')
        size, kind = struct.unpack_from('<II', raw, cursor); cursor += 8
        if size % 4 or cursor+size > len(raw) or kind in chunks:
            raise LiquidError('Invalid GLB chunk boundary')
        chunks[kind] = raw[cursor:cursor+size]; cursor += size
    try:
        doc = json.loads(chunks[0x4e4f534a]); binary = chunks[0x004e4942]
        declared_bytes = doc['buffers'][0]['byteLength']
        if (doc['asset']['version'] != '2.0' or len(doc['buffers']) != 1 or 'uri' in doc['buffers'][0] or
                type(declared_bytes) is not int or declared_bytes < 0 or not 0 <= len(binary)-declared_bytes <= 3):
            raise LiquidError('GLB must have one embedded bounded buffer')
        counts = dict(vertices=0, triangles=0, structural_nodes=0)

        def indexed(sequence, index, context):
            if type(index) is not int or not 0 <= index < len(sequence):
                raise LiquidError('Invalid GLB '+context+' index')
            return sequence[index]

        def accessor(index, expected):
            item = indexed(doc['accessors'], index, 'accessor')
            if 'sparse' in item or item.get('normalized', False):
                raise LiquidError('Sparse or normalized structural accessor unsupported')
            if item['type'] != expected: raise LiquidError('Invalid GLB accessor shape')
            kinds = {5121: '<u1', 5123: '<u2', 5125: '<u4', 5126: '<f4'}
            dtype = np.dtype(kinds[item['componentType']]); width = 3 if expected == 'VEC3' else 1
            if expected == 'VEC3' and item['componentType'] != 5126:
                raise LiquidError('Structural positions require float32')
            if expected == 'SCALAR' and item['componentType'] == 5126:
                raise LiquidError('Structural indices require unsigned integers')
            n = item['count']
            if type(n) is not int or n <= 0 or n > limits['max_vertices']*3:
                raise LiquidError('GLB accessor count budget exceeded')
            view = indexed(doc['bufferViews'], item['bufferView'], 'buffer view')
            offset = item.get('byteOffset', 0); base = view.get('byteOffset', 0)
            stride = view.get('byteStride', dtype.itemsize*width)
            end = offset+(n-1)*stride+dtype.itemsize*width
            if (view.get('buffer', 0) != 0 or min(base, offset) < 0 or stride < dtype.itemsize*width or
                    stride % dtype.itemsize or (base+offset) % dtype.itemsize or
                    end > view['byteLength'] or base+view['byteLength'] > declared_bytes):
                raise LiquidError('GLB accessor exceeds embedded buffer view')
            return np.ndarray((n, width), dtype=dtype, buffer=binary, offset=base+offset,
                              strides=(stride, dtype.itemsize)).copy()

        solids = []; sections = {}; visited = set(); world_matrices = {}

        def visit(index, parent):
            if type(index) is not int or index < 0 or index >= len(doc['nodes']) or index in visited:
                raise LiquidError('GLB graph contains invalid, cyclic or shared node')
            visited.add(index); node = doc['nodes'][index]; matrix = parent@_transform(node)
            world_matrices[index] = matrix
            name = node.get('name', '')
            if 'mesh' in node and (name == 'airframe' or name.startswith('fsim:section:')):
                positions = []; faces = []; start = 0
                for primitive in indexed(doc['meshes'], node['mesh'], 'mesh')['primitives']:
                    if primitive.get('mode', 4) != 4: raise LiquidError('Structural GLB is not triangles')
                    p = accessor(primitive['attributes']['POSITION'], 'VEC3')
                    t = accessor(primitive['indices'], 'SCALAR').reshape(-1)
                    if len(t) % 3 or np.max(t) >= len(p): raise LiquidError('Invalid GLB triangle indices')
                    p = (p@matrix[:3, :3].T+matrix[:3, 3])@_GLTF_BODY.T
                    t = t.astype(np.int64).reshape(-1, 3)
                    if np.linalg.det(matrix[:3, :3]) < 0: t = t[:, [0, 2, 1]]
                    positions.append(p); faces.append(t+start); start += len(p)
                    counts['vertices'] += len(p); counts['triangles'] += len(t)
                    if counts['vertices'] > limits['max_vertices'] or counts['triangles'] > limits['max_triangles']:
                        raise LiquidError('Structural GLB mesh budget exceeded')
                p = np.concatenate(positions); t = np.concatenate(faces)
                if not np.isfinite(p).all(): raise LiquidError('Nonfinite GLB structural vertex')
                unique, inverse = np.unique(p, axis=0, return_inverse=True)
                section = name[len('fsim:section:'):].replace('-', ' ') if name.startswith('fsim:section:') else 'airframe'
                if section in sections: raise LiquidError('Duplicate structural section '+section)
                if owner_names_only and name.startswith('fsim:section:'):
                    # Rendered caps are not physical bulkheads or Boolean input.
                    sections[section] = None
                else:
                    mesh = mf.Mesh64(np.ascontiguousarray(unique), np.ascontiguousarray(inverse[t], dtype=np.uint64))
                    solid = _valid(mf.Manifold(mesh), 'GLB closed structural mesh '+name, nonempty=True)
                    sections[section] = solid; solids.append(solid)
                counts['structural_nodes'] += 1
            for child in node.get('children', []): visit(child, matrix)

        for root in indexed(doc['scenes'], doc.get('scene', 0), 'scene')['nodes']: visit(root, np.eye(4))
        if not sections: raise LiquidError('GLB has no neutral structural mesh')
        extra = doc.get('extras', {}).get('fsim_liquid_boundary_v1')
        authority_receipt = {}
        if extra is not None:
            if (extra['schema'] != 'fsim.liquid-boundary.v1' or extra['frame'] != 'gltf_model_origin_m' or
                    type(extra['node']) is not int or extra['node'] not in world_matrices):
                raise LiquidError('Invalid original unsplit authoritative boundary identity/frame/node')
            raw_p = accessor(extra['positions'], 'VEC3'); raw_t = accessor(extra['indices'], 'SCALAR')
            if len(raw_t) % 3 or np.max(raw_t) >= len(raw_p):
                raise LiquidError('Invalid authoritative boundary triangle indices')
            counts['vertices'] += len(raw_p); counts['triangles'] += len(raw_t)//3
            if counts['vertices'] > limits['max_vertices'] or counts['triangles'] > limits['max_triangles']:
                raise LiquidError('Authoritative GLB mesh budget exceeded')
            matrix = world_matrices[extra['node']]
            p = (raw_p@matrix[:3, :3].T+matrix[:3, 3])@_GLTF_BODY.T
            if not np.isfinite(p).all(): raise LiquidError('Nonfinite authoritative boundary point')
            t = raw_t.astype(np.int64).reshape(-1, 3)
            if np.linalg.det(matrix[:3, :3]) < 0: t = t[:, [0, 2, 1]]
            unique, inverse = np.unique(p, axis=0, return_inverse=True)
            hull = _valid(mf.Manifold(mf.Mesh64(np.ascontiguousarray(unique),
                          np.ascontiguousarray(inverse[t], dtype=np.uint64))),
                          'Original unsplit authoritative exterior', nonempty=True)
            authority = 'original_unsplit_airframe_accessors_v1'
            authority_receipt = dict(positions=extra['positions'], indices=extra['indices'], node=extra['node'],
                                     positions_sha256=hashlib.sha256(raw_p.tobytes()).hexdigest(),
                                     indices_sha256=hashlib.sha256(raw_t.tobytes()).hexdigest(),
                                     vertices=len(raw_p), triangles=len(t), status=str(hull.status()))
        elif set(sections) == {'airframe'}:
            hull = sections['airframe']; authority = 'legacy_unsectioned_airframe'
        else:
            raise LiquidError('Sectioned GLB requires original unsplit authoritative boundary extras')
        receipt = dict(counts, volume_m3=hull.volume(), boundary='delivered_original_neutral_polygonal_exterior',
                       authority=authority, authoritative_accessors=authority_receipt)
        return (hull, receipt, sections) if with_sections else (hull, receipt)
    except (KeyError, IndexError, ValueError, TypeError, struct.error) as refused:
        raise LiquidError('Invalid GLB structural data: '+str(refused)) from refused


def _box(low, high):
    # Explicit endpoints avoid lo+(hi-lo) rounding a shared face past hi.
    vertices = np.array([[low[0], low[1], low[2]], [high[0], low[1], low[2]],
                         [high[0], high[1], low[2]], [low[0], high[1], low[2]],
                         [low[0], low[1], high[2]], [high[0], low[1], high[2]],
                         [high[0], high[1], high[2]], [low[0], high[1], high[2]]])
    triangles = np.array([[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7],
                          [0, 1, 5], [0, 5, 4], [1, 2, 6], [1, 6, 5],
                          [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]], dtype=np.uint64)
    return _valid(_mf().Manifold(_mf().Mesh64(vertices, triangles)), 'Cell box', nonempty=True)


def _native_exclusion_payload(native_damage, origin):
    payload = dict(origin_design_m=_vec(origin, 'native exclusion origin').tolist(),
                   native_damage_present=native_damage is not None, components=[])
    if native_damage is None: return payload
    try:
        if np.linalg.norm(_vec(native_damage['origin_design_m'], 'native damage origin')-origin) > 1e-9:
            raise LiquidError('Native solid exclusion origin differs from structural frame')
        if not isinstance(native_damage['components'], list): raise LiquidError('Native component list required')
        for i, component in enumerate(native_damage['components']):
            if component['kind'] == 'fuel': continue
            if component['kind'] not in ('engine', 'crew', 'control') or component['shape'] not in ('box', 'cylinder'):
                raise LiquidError('Unsupported native nonfuel solid kind/shape')
            _vec(component['centre'], 'native solid centre')
            if np.any(_vec(component['half'], 'native solid half extent') <= 0):
                raise LiquidError('Native nonfuel solid must have positive extents')
            payload['components'].append([i, component])
    except (KeyError, TypeError, ValueError) as refused:
        raise LiquidError('Malformed native solid exclusion: '+str(refused)) from refused
    return payload


def _clear_of_exclusion_boxes(low, high, rows):
    """Exact positive-interior test; face/edge contact occupies no fluid volume."""
    if not rows: return True
    lows = np.array([r['low_m'] for r in rows]); highs = np.array([r['high_m'] for r in rows])
    return not np.any(np.all(np.minimum(highs, high) > np.maximum(lows, low), axis=1))


def _exclusions(aircraft, origin, *, native_damage=None):
    """Conservative unfitted component AABBs; rotor-only estimates are explicit."""
    from . import vulnerability
    records = []; boxes = []

    def add(name, low, high, basis, native_component=None):
        low, high = np.minimum(low, high).copy(), np.maximum(low, high).copy()
        if np.any(high <= low): raise LiquidError('Degenerate occupied-component exclusion')
        boxes.append(_box(low, high))
        record = dict(name=name, low_m=low.tolist(), high_m=high.tolist(), basis=basis)
        if native_component is not None: record['native_component'] = native_component
        records.append(record)

    if aircraft.spec.get('aircraft', {}).get('kind') == 'helicopter':
        for body in aircraft.bodies:
            if any(word in body.name.lower() for word in ('cowling', 'nacelle', 'deck')):
                for side in ((1, -1) if body.mirror else (1,)):
                    yc = side*np.asarray(body.y); half = np.asarray(body.w)*.5
                    lo = [min(body.x), min(yc-half), min(body.bottom)]
                    hi = [max(body.x), max(yc+half), max(body.top)]
                    add(body.name, _body(lo, origin), _body(hi, origin),
                        'estimated conservative engine/transmission authored-body bounds')
            if body.name.lower() == 'fuselage':
                # M03 placement estimate, not an OEM cabin/tank or native crew declaration.
                span = float(body.x[-1]-body.x[0]); x0 = float(body.x[0]+.08*span); x1 = float(body.x[0]+.38*span)
                inside = (body.x >= x0)&(body.x <= x1)
                xs = np.concatenate(([x0, x1], body.x[inside]))
                widths = np.interp(xs, body.x, body.w); ys = np.interp(xs, body.x, body.y)
                bottoms = np.interp(xs, body.x, body.bottom); tops = np.interp(xs, body.x, body.top)
                lo = [x0, min(ys-.45*widths), min(bottoms+.35*(tops-bottoms))]
                hi = [x1, max(ys+.45*widths), max(tops)]
                add('estimated occupied cockpit', _body(lo, origin), _body(hi, origin),
                    'estimated forward .08-.38 fuselage span, upper .65 height, .90 width; not calibrated')
    else:
        for component in vulnerability._engines(aircraft)+vulnerability._crew(aircraft):
            centre = _body(component['centre'], origin); half = np.asarray(component['half'], float)
            add(component['name'], centre-half, centre+half,
                'existing unfitted vulnerability '+component['kind']+' conservative bounding box')
    # These coordinates already are native BODY coordinates. Do not reapply
    # design origin or crop/shrink a represented solid to make fuel fit.
    for index, component in _native_exclusion_payload(native_damage, origin)['components']:
        centre = _vec(component['centre'], 'native solid centre'); half = _vec(component['half'], 'native solid half')
        add(component['name'], centre-half, centre+half,
            'actual native '+component['kind']+'/'+component['shape']+' enclosing AABB; geometry estimate', index)
    return (_valid(_mf().Manifold.batch_boolean(boxes, _mf().OpType.Add), 'Exclusion union')
            if boxes else None), records


def _tolerance(volume, terms):
    # Float64 summation and representable slab-coordinate error, fixed before outcomes.
    return 512*np.finfo(float).eps*max(1., abs(volume))*max(1, terms)


def pack(design, hull, tanks, *, excluded=None, limits=None, authority=None, occupied=None,
         owner_certificate=None, candidate_bounds=None, necessary_region=None, exclusion_boxes=None, occupied_boxes=None,
         _execution=None):
    """Produce capacity-conserving disjoint cells; raise on incomplete geometry.

    Box partitions prove nonoverlap by construction. Pool boxes retain a strict
    Boolean containment proof from themselves or an enclosing parent. Every
    emitted clipped slab additionally receives its own fresh containment check.
    Nearest-anchor assignment is an explicitly estimated internal distribution.
    """
    limits = _limits(limits); mf = _mf(); deadline = time.monotonic()+limits['max_seconds']
    _valid(hull, 'Structural hull', nonempty=True)
    tanks = [dict(t, anchor_body_m=_vec(t['anchor_body_m'], 'Tank anchor').tolist()) for t in tanks]
    seen = set()
    for t in tanks:
        if type(t['index']) is not int or t['index'] < 0 or t['index'] in seen:
            raise LiquidError('Invalid or duplicate native tank index')
        seen.add(t['index'])
        if any(isinstance(t[k], bool) or not isinstance(t[k], numbers.Real) or
               not math.isfinite(t[k]) or t[k] <= 0 for k in ('capacity_kg', 'density_kg_m3')):
            raise LiquidError('Finite positive capacity and physical density required')
    tanks.sort(key=lambda t: t['index'])
    if not tanks: raise LiquidError('pack requires liquid tanks; no-liquid admission uses produce')
    volumes = np.array([t['capacity_kg']/t['density_kg_m3'] for t in tanks])
    if not np.isfinite(volumes).all() or np.any(volumes <= 0): raise LiquidError('Invalid physical liquid volume')
    required = math.fsum(volumes)
    available = _valid(hull ^ authority, 'Owner/original exterior clip') if authority is not None else hull
    if excluded is not None: available = _valid(available-excluded, 'Available interior excluding equipment')
    if occupied is not None: available = _valid(available-occupied, 'Available interior excluding prior actual liquid cells')
    if available.volume()+_tolerance(required, 1) < required:
        raise LiquidError('insufficient available interior volume: %.12g < %.12g m3' % (available.volume(), required))
    bounds = np.array(available.bounding_box()); low = bounds[:3]; high = bounds[3:]
    anchors = np.array([t['anchor_body_m'] for t in tanks])
    target_volume = required/max(64, 16*len(tanks)); max_edge = max(high-low)/8
    if necessary_region is not None:
        # A union of valid analytic necessary boxes, never inferred cap solids.
        # Save the original grid criterion above; final whole-hull containment
        # and singleton owner certificates remain independently mandatory.
        available = _valid(available ^ necessary_region(low, high), 'Original hull/necessary classifier region clip')
        if available.volume()+_tolerance(required, 1) < required:
            raise LiquidError('insufficient physical volume in necessary native-owner region')
        bounds = np.array(available.bounding_box()); low, high = bounds[:3], bounds[3:]
    if candidate_bounds is not None:
        low, high = candidate_bounds(low, high)
        if np.any(low >= high): raise LiquidError('insufficient possible native-owner bounding volume')
    pending = []; sequence = 0
    # A queued child is an exact subset of its midpoint parent. Reuse that
    # parent's already-computed partial overlap as a smaller immutable Boolean
    # domain; cache pressure always falls back to ORIGINAL available. Counts
    # include the currently processed record until it releases its reference.
    _cache_records = {}; _cache_sequence = 0
    _cache_metrics = dict(admitted=0, reused=0, unknown_reused=0, fallback=0,
                          reclaimed=0, vertices=0, triangles=0, records=0,
                          peak_vertices=0, peak_triangles=0, peak_records=0)
    if _execution is not None:_execution.cache_metrics=_cache_metrics
    cache_limits=limits if _execution is None else _execution.cache_limits

    def cache_admit(solid):
        nonlocal _cache_sequence
        vertices, triangles = solid.num_vert(), solid.num_tri()
        if (_cache_metrics['vertices']+vertices > cache_limits['max_vertices'] or
            _cache_metrics['triangles']+triangles > cache_limits['max_triangles'] or
            _cache_metrics['records']+1 > cache_limits['max_screened_cells']):
            _cache_metrics['fallback'] += 1
            return None
        key = _cache_sequence; _cache_sequence += 1
        _cache_records[key] = [solid, vertices, triangles, 0]
        _cache_metrics['admitted'] += 1
        for name, count in (('vertices', vertices), ('triangles', triangles), ('records', 1)):
            _cache_metrics[name] += count
            _cache_metrics['peak_'+name] = max(_cache_metrics['peak_'+name], _cache_metrics[name])
        return key

    def cache_retain(key):
        if key is not None: _cache_records[key][3] += 1

    def cache_release(key):
        if key is None: return
        record = _cache_records[key]; record[3] -= 1
        if record[3] == 0:
            del _cache_records[key]
            _cache_metrics['vertices'] -= record[1]; _cache_metrics['triangles'] -= record[2]
            _cache_metrics['records'] -= 1; _cache_metrics['reclaimed'] += 1

    def push(lo, hi, depth, contained=False, domain=None):
        nonlocal sequence
        distance = float(np.min(np.sum(np.maximum(np.maximum(lo-anchors, anchors-hi), 0.)**2, axis=1)))
        ownership = owner_certificate(lo, hi) if owner_certificate is not None else 1
        # Breadth first is mandatory: certified but hull-straddling cells must
        # not refine forever ahead of a coarser indeterminate owner region.
        # Within one depth, prefer proven owners and cheap impossible discards.
        cache_retain(domain)
        heapq.heappush(pending, (depth, int(ownership == 0), distance, sequence,
                               tuple(lo), tuple(hi), ownership, contained, domain)); sequence += 1

    push(low, high, 0); pool = []; accepted_volume = 0.; screened = 0
    initial_upper_volume = math.prod(high-low); upper_volume = initial_upper_volume
    if initial_upper_volume+_tolerance(initial_upper_volume, 1) < required:
        raise LiquidError('insufficient native-owner candidate volume upper bound')
    def discard_volume(volume):
        nonlocal upper_volume
        upper_volume = math.fsum((upper_volume, -volume))
        # This is only an impossibility bound, never a capacity admission.
        if upper_volume+_tolerance(initial_upper_volume, screened) < required:
            raise LiquidError('insufficient native-owner certified volume upper bound')
    def split(lo, hi, depth, contained=False, domain=None):
        axis = int(np.argmax(hi-lo)); midpoint = float((lo[axis]+hi[axis])*.5)
        if depth >= limits['max_depth'] or hi[axis]-lo[axis] <= limits['min_extent_m'] or midpoint in (lo[axis], hi[axis]):
            raise LiquidError('Liquid spatial refinement budget exhausted before capacity closure')
        left = hi.copy(); left[axis] = midpoint; right = lo.copy(); right[axis] = midpoint
        push(lo, left, depth+1, contained, domain); push(right, hi, depth+1, contained, domain)
    def exhausted(reason, remaining=None):
        clock = ('clock=not-sampled-count-short-circuit' if remaining is None else
                 'remaining_clock_s=%.17g' % remaining)
        return LiquidError('Liquid geometry screening budget exceeded: reason=%s tanks=%s '
            'screened=%d pending=%d pool=%d accepted_m3=%.17g required_m3=%.17g depth=%d %s' %
            (reason, ','.join(str(t['index']) for t in tanks), screened, len(pending), len(pool),
             accepted_volume, required, pending[0][0], clock))
    while pending and accepted_volume < required:
        if screened >= limits['max_screened_cells']:
            raise exhausted('count')
        now = time.monotonic()
        if now > deadline:
            raise exhausted('time', deadline-now)
        if _execution is not None:_execution.screen()
        depth, _, _, _, lo, hi, ownership, contained, domain = heapq.heappop(pending); lo = np.array(lo); hi = np.array(hi)
        volume = math.prod(hi-lo)
        if ownership == -1:
            cache_release(domain); screened += 1; discard_volume(volume); continue
        if ownership == 0:
            # It cannot be emitted yet. Refining its exact partition enclosure
            # needs no expensive physical hull Boolean; its whole box remains
            # in the conservative unexplored-volume upper bound.
            if domain is not None: _cache_metrics['unknown_reused'] += 1
            screened += 1; split(lo, hi, depth, contained, domain); cache_release(domain); continue
        screened += 1
        child_domain = None
        if not contained:
            box = _box(lo, hi)
            if domain is not None: _cache_metrics['reused'] += 1
            candidate_domain = available if domain is None else _cache_records[domain][0]
            contained = _valid(box-candidate_domain, 'Cell/interior difference').is_empty()
            if not contained:
                overlap = _valid(box ^ candidate_domain, 'Cell/interior intersection')
                if overlap.is_empty():
                    candidate_domain = None; overlap = None
                    cache_release(domain); discard_volume(volume); continue
                child_domain = cache_admit(overlap)
                overlap = None
            candidate_domain = None
        if contained and ownership == 1 and volume <= target_volume*(1+1e-12) and max(hi-lo) <= max_edge*(1+1e-12):
            pool.append([lo, hi]); accepted_volume = math.fsum([accepted_volume, volume])
            if len(pool) > limits['max_cells']: raise LiquidError('Liquid cell budget exceeded')
        else:
            split(lo, hi, depth, contained, child_domain)
        cache_release(domain)
    # Successful capacity closure can leave unvisited queued siblings. Their
    # shared records are no longer needed; no cache survives this invocation.
    for item in pending: cache_release(item[-1])
    pending.clear()
    if accepted_volume+_tolerance(required, len(pool)) < required:
        raise LiquidError('insufficient certified cell volume')
    if len(pool)*len(tanks) > limits['max_assignment_edges']:
        raise LiquidError('Liquid nearest-anchor assignment budget exceeded')
    if _execution is not None:_execution.reserve_edges(len(pool)*len(tanks))
    centres = np.array([(lo+hi)*.5 for lo, hi in pool])
    orders = [np.argsort(np.sum((centres-anchor)**2, axis=1), kind='stable') for anchor in anchors]
    positions = [0]*len(tanks); remaining = volumes.copy(); assignment = []; allocated_volumes = [[] for _ in tanks]

    def candidate(k):
        while positions[k] < len(pool):
            i = int(orders[k][positions[k]]); positions[k] += 1
            lo, hi = pool[i]
            if np.all(hi > lo):
                heapq.heappush(assignment, (float(np.sum((centres[i]-anchors[k])**2)), k, i)); return
        if remaining[k] > _tolerance(volumes[k], len(pool)):
            raise LiquidError('insufficient unowned cells for native tank')

    for k in range(len(tanks)): candidate(k)
    cells = []
    while assignment:
        if time.monotonic() > deadline: raise LiquidError('Liquid assignment time budget exceeded')
        if _execution is not None:_execution.check()
        _, k, i = heapq.heappop(assignment); lo, hi = pool[i]
        if remaining[k] <= 0: continue
        if np.any(hi <= lo): candidate(k); continue
        lo = lo.copy(); hi = hi.copy(); free = math.prod(hi-lo)
        take = min(float(remaining[k]), free); allocated_low = lo.copy(); allocated_high = hi.copy()
        if take < free:
            axis = int(np.argmax(hi-lo)); cross = math.prod(np.delete(hi-lo, axis)); width = take/cross
            if anchors[k, axis] <= centres[i, axis]:
                allocated_high[axis] = lo[axis]+width; pool[i][0] = allocated_high.copy()
                pool[i][0][np.arange(3) != axis] = lo[np.arange(3) != axis]
            else:
                allocated_low[axis] = hi[axis]-width; pool[i][1] = allocated_low.copy()
                pool[i][1][np.arange(3) != axis] = hi[np.arange(3) != axis]
        else: pool[i][0] = hi.copy()
        slab = _box(allocated_low, allocated_high)
        if owner_certificate is not None and owner_certificate(allocated_low, allocated_high) != 1:
            raise LiquidError('Emitted liquid slab has uncertified native owner membership')
        if not _valid(slab-hull, 'Emitted slab/hull difference').is_empty():
            raise LiquidError('Emitted liquid slab escapes structural boundary')
        if authority is not None and not _valid(slab-authority, 'Emitted slab/original exterior difference').is_empty():
            raise LiquidError('Emitted liquid slab escapes original authoritative exterior')
        if occupied is not None:
            intersection = _valid(slab ^ occupied, 'Emitted slab/prior liquid intersection')
            # Manifold can retain an exactly coplanar zero-volume contact as
            # nonempty geometry. For the per-TK box path, original endpoints
            # give a stronger exact positive-interior test, without tolerance.
            disjoint = (_clear_of_exclusion_boxes(allocated_low, allocated_high, occupied_boxes)
                        if occupied_boxes is not None else intersection.is_empty())
            if not disjoint: raise LiquidError('Emitted liquid slab overlaps prior actual liquid cells')
        if exclusion_boxes is not None:
            if not _clear_of_exclusion_boxes(allocated_low, allocated_high, exclusion_boxes):
                raise LiquidError('Emitted liquid slab has positive interior overlap with occupied equipment box')
        elif excluded is not None and not _valid(slab ^ excluded, 'Emitted slab/exclusion intersection').is_empty():
            raise LiquidError('Emitted liquid slab intersects occupied equipment')
        actual = math.prod(allocated_high-allocated_low)
        if _execution is not None:_execution.emit_cell()
        cells.append(dict(id=len(cells), tank=tanks[k]['index'], section=tanks[k].get('section', 'airframe'), low_m=allocated_low.tolist(),
                          high_m=allocated_high.tolist(), volume_m3=actual))
        if len(cells) > limits['max_cells']: raise LiquidError('Emitted liquid cell budget exceeded')
        allocated_volumes[k].append(actual)
        residual = volumes[k]-math.fsum(allocated_volumes[k])
        # Use the already-declared final ledger's binary64 roundoff budget;
        # never construct an unrepresentable zero-width residual slab.
        remaining[k] = 0. if abs(residual) <= _tolerance(volumes[k], len(allocated_volumes[k])) else max(0., residual)
        if remaining[k] > 0: candidate(k)
    checks = []
    for t, expected in zip(tanks, volumes):
        owned = [c for c in cells if c['tank'] == t['index']]
        actual = math.fsum(c['volume_m3'] for c in owned); tolerance = _tolerance(expected, len(owned))
        if not owned or abs(actual-expected) > tolerance:
            raise LiquidError('Liquid tank capacity ledger failed for native tank '+str(t['index']))
        checks.append(dict(tank=t['index'], required_m3=float(expected), actual_m3=actual,
                           error_m3=actual-expected, tolerance_m3=tolerance))
    return dict(schema=SCHEMA, design=design, frame=FRAME, sources=[], tanks=tanks, cells=cells,
                fluid_policy_v1=dict(FLUID_POLICY_V1),
                ledger=dict(capacity_conserved=True, containment_certified=True, exclusions_certified=True,
                            disjoint_certified=True, disjoint_basis='recursive box partition and consumed clipped slabs',
                            screened_cells=screened, certified_pool_cells=len(pool),
                            assignment_edges=len(pool)*len(tanks),
                            available_interior_m3=available.volume(), tanks=checks),
                uncertainty=dict(distribution='estimated capacity-constrained nearest-anchor occupancy; not OEM tank layout',
                                 boundary='delivered neutral polygonal GLB, not exact smooth CAD',
                                 maximum_cell_diagonal_m=max(float(np.linalg.norm(np.array(c['high_m'])-c['low_m'])) for c in cells),
                                 dynamics='full-capacity geometry; runtime contents/free surface and slosh are separate'),
                limits=limits)


def _source(role, path):
    path = Path(path)
    return dict(role=role, path=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def _ownership(tanks, sections, damage_path, design, origin):
    if set(sections) == {'airframe'}:
        return [dict(t, section='airframe') for t in tanks]
    if 'airframe' in sections:
        raise LiquidError('Ambiguous whole-airframe and breakup structural meshes')
    if not damage_path.is_file():
        raise LiquidError('Sectioned liquid geometry requires existing damage tank ownership')
    doc = json.loads(damage_path.read_text(encoding='utf-8'))
    if doc.get('aircraft') != design or not np.array_equal(_vec(doc.get('origin_design_m'), 'damage origin'), origin):
        raise LiquidError('Damage ownership design or origin mismatch')
    owners = {}
    for section in doc['sections']:
        for tank in section.get('tanks', []):
            index = tank['index']
            if type(index) is not int or index in owners:
                raise LiquidError('Duplicate/invalid native tank section ownership')
            owners[index] = (section['name'], float(tank['capacity_kg']))
    if set(owners) != {t['index'] for t in tanks}:
        raise LiquidError('Damage ownership does not exactly cover authored native tank IDs')
    result = []
    for tank in tanks:
        section, capacity = owners[tank['index']]
        if section not in sections:
            raise LiquidError('Tank owner has no closed structural GLB section: '+section)
        # Existing damage serialization rounds this capacity to five decimals.
        if not math.isfinite(capacity) or abs(capacity-tank['capacity_kg']) > 5.0001e-6:
            raise LiquidError('Damage ownership capacity differs from authored tank')
        result.append(dict(tank, section=section))
    return result


def _execution_binding(path):
    path=Path(path)
    if any(p.casefold()=='.codex' for p in path.parts):raise LiquidError('Excluded owner execution input')
    if path.is_symlink() or (hasattr(path,'is_junction') and path.is_junction()):
        raise LiquidError('Indirect owner execution input')
    return dict(path=str(path.absolute()),sha256=hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None)


def _verify_execution_bindings(bindings):
    for row in bindings:
        if _execution_binding(row['path'])!=row:raise LiquidError('Stale owner execution source/input: '+row['path'])


def _ordinary_owner_worker(recipe,execution):
    """Reconstruct the ORIGINAL GLB/classifier, never pickle a live Manifold."""
    from .geometry import Aircraft
    execution.check();_verify_execution_bindings(recipe['bindings'])
    aircraft=Aircraft.load(recipe['design_path']);origin=np.array(recipe['origin'],float)
    if aircraft.name!=recipe['design']:raise LiquidError('Owner recipe design mismatch')
    effective=hashlib.sha256(json.dumps(aircraft.spec,sort_keys=True,allow_nan=False,separators=(',',':')).encode()).hexdigest()
    if effective!=recipe['effective_spec']:raise LiquidError('Owner loaded design differs from captured effective specification')
    hull,receipt,sections=read_glb(recipe['glb'],limits=execution.budget.limits,with_sections=True,owner_names_only=True)
    if receipt!=recipe['boundary_receipt']:raise LiquidError('Owner original-boundary reconstruction mismatch')
    damage=Path(recipe['glb']).with_suffix('.damage.json')
    tanks=_ownership(tank_records(aircraft,origin),sections,damage,aircraft.name,origin)
    owned=[t for t in tanks if t['section']==recipe['owner']]
    if owned!=recipe['tanks']:raise LiquidError('Owner reconstructed native tanks mismatch')
    native=json.loads(damage.read_text(encoding='utf-8'))
    excluded,exclusions=_exclusions(aircraft,origin,native_damage=native)
    if exclusions!=recipe['exclusions']:raise LiquidError('Owner reconstructed material exclusions mismatch')
    classifier=_OwnerClassifier(aircraft,origin);trace=[]
    def certificate(lo,hi):
        possible=classifier.possible(lo,hi);owner=recipe['owner']
        result=1 if possible==frozenset({owner}) else 0 if owner in possible else -1
        if recipe.get('trace',False):trace.append(([float(x).hex() for x in lo],[float(x).hex() for x in hi],result))
        return result
    execution.check();audits={}
    global _valid
    original_valid=_valid
    if recipe.get('trace',False):
        def audited(solid,context,**options):
            if context.startswith('Emitted '):audits[context]=audits.get(context,0)+1
            return original_valid(solid,context,**options)
        _valid=audited
    try:
        result=pack(aircraft.name,hull,owned,excluded=excluded,limits=execution.budget.limits,
            owner_certificate=certificate,candidate_bounds=lambda lo,hi:classifier.candidate_bounds(recipe['owner'],lo,hi),
            necessary_region=lambda lo,hi:classifier.candidate_region(recipe['owner'],lo,hi),
            exclusion_boxes=exclusions,_execution=execution)
    finally:_valid=original_valid
    _verify_execution_bindings(recipe['bindings']);execution.check()
    return dict(pack=result,owner_trace=trace if recipe.get('trace',False) else None,audit_counts=audits)


def _pack_owned(design, hull, sections, tanks, excluded, limits, *, aircraft=None, origin=None, exclusion_boxes=None,
                vessel_policy=None,_recipe=None):
    limits = _limits(limits); deadline = time.monotonic()+limits['max_seconds']
    if vessel_policy is not None:
        from . import liquid_vessels
        if _recipe is not None: _recipe['_deadline'] = deadline
        return liquid_vessels._pack_shared(design, hull, tanks, excluded, limits, vessel_policy,
                                          exclusion_boxes=exclusion_boxes, deadline=deadline)
    owners = sorted({t['section'] for t in tanks}); occupied = None
    classifier = vessel_policy.classifier if vessel_policy is not None else _OwnerClassifier(aircraft, origin) if aircraft is not None else None
    combined = None; cells = []; checks = []; ledgers = []; screened = 0; edges = 0
    groups = [(owner, [t for t in tanks if t['section'] == owner]) for owner in owners]
    if vessel_policy is not None:
        by_index = {t['index']: t for t in tanks}
        groups = [(by_index[i]['section'], [by_index[i]]) for i in vessel_policy.assignment_order(list(by_index))]
    parallel=None;_budget=None
    from .resources import current_cpu_lease
    cpus=current_cpu_lease()
    if _recipe is not None and classifier is not None and vessel_policy is None and len(groups)>1 and cpus is not None and len(cpus)>=2:
        _recipe['_deadline']=deadline
        jobs=[dict(_recipe,owner=owner,tanks=owned) for owner,owned in groups]
        _budget=_OwnerBudget(mp.get_context('spawn'),limits,deadline,2)
        parallel=_process_owner_jobs(jobs,_ordinary_owner_worker,_budget,cpus[:2])
    for ordinal,(owner, owned_tanks) in enumerate(groups):
        remaining = dict(limits, max_cells=limits['max_cells']-len(cells),
                         max_screened_cells=limits['max_screened_cells']-screened,
                         max_assignment_edges=limits['max_assignment_edges']-edges,
                         max_seconds=deadline-time.monotonic())
        if min(remaining['max_cells'], remaining['max_screened_cells'],
               remaining['max_assignment_edges'], remaining['max_seconds']) <= 0:
            raise LiquidError('Combined model liquid geometry budget exceeded')
        try:
            def certificate(lo, hi):
                if vessel_policy is not None: return vessel_policy.certificate(owned_tanks[0]['index'], lo, hi)
                possible = classifier.possible(lo, hi)
                return 1 if possible == frozenset({owner}) else 0 if owner in possible else -1
            def region(lo, hi):
                if vessel_policy is not None: return vessel_policy.region(owned_tanks[0]['index'], lo, hi)
                return classifier.candidate_region(owner, lo, hi)
            result = parallel[ordinal]['pack'] if parallel is not None else pack(design, hull if classifier is not None else sections[owner],
                          owned_tanks, excluded=excluded, limits=remaining,
                          authority=None if classifier is not None else hull, occupied=occupied,
                          owner_certificate=certificate if classifier is not None else None,
                          candidate_bounds=(lambda lo, hi: classifier.candidate_bounds(owner, lo, hi)) if classifier is not None and vessel_policy is None else None,
                          necessary_region=region if classifier is not None else None,
                          exclusion_boxes=exclusion_boxes, occupied_boxes=cells if vessel_policy is not None else None)
            if parallel is not None:
                ledger=result['ledger']
                if (result['tanks']!=owned_tanks or len(result['cells'])>remaining['max_cells'] or
                    ledger['certified_pool_cells']>remaining['max_cells'] or
                    ledger['screened_cells']>remaining['max_screened_cells'] or
                    ledger['assignment_edges']>remaining['max_assignment_edges']):
                    raise LiquidError('Canonical owner/global resource ceiling exceeded')
                _budget.check()
        except LiquidError as refused:
            raise LiquidError('Tank owner section '+owner+': '+str(refused)) from refused
        if combined is None: combined = result
        for cell in result['cells']:
            if vessel_policy is not None:
                cell['eligibility_v1'] = vessel_policy.eligibility(cell['tank'], np.array(cell['low_m']), np.array(cell['high_m']))
            cell['id'] = len(cells); cells.append(cell)
        if classifier is None or vessel_policy is not None:
            # Abstract synthetic owner solids may overlap. Real singleton native
            # partition certificates already prove different owners disjoint.
            new_cells = [_box(np.array(c['low_m']), np.array(c['high_m'])) for c in result['cells']]
            occupied = _valid(_mf().Manifold.batch_boolean(([occupied] if occupied is not None else [])+new_cells,
                                                           _mf().OpType.Add), 'Actual allocated liquid union', nonempty=True)
        checks.extend(result['ledger']['tanks']); screened += result['ledger']['screened_cells']
        edges += result['ledger']['assignment_edges']
        if vessel_policy is None:
            ledgers.append(dict(section=owner, **result['ledger']))
        else:
            row = next((r for r in ledgers if r['section'] == owner), None)
            if row is None:
                row = dict(section=owner, capacity_conserved=True, containment_certified=True, exclusions_certified=True,
                           disjoint_certified=True, tanks=[], screened_cells=0, assignment_edges=0,
                           tank_packing_interiors_m3=[], assignment_order=[])
                ledgers.append(row)
            row['tanks'].extend(result['ledger']['tanks'])
            row['screened_cells'] += result['ledger']['screened_cells']; row['assignment_edges'] += result['ledger']['assignment_edges']
            row['assignment_order'].append(owned_tanks[0]['index'])
            row['tank_packing_interiors_m3'].append(dict(tank=owned_tanks[0]['index'],
                                                       available_m3=result['ledger']['available_interior_m3']))
    combined['tanks'] = tanks; combined['cells'] = cells; combined['limits'] = limits
    # Independent exact endpoint audit, separate from ownership certificates and
    # the recursive construction proof. Shared faces are legal, interiors aren't.
    lows = np.array([c['low_m'] for c in cells]); highs = np.array([c['high_m'] for c in cells])
    axis = int(np.argmax((highs.max(axis=0)-lows.min(axis=0))/(highs-lows).sum(axis=0)))
    order = np.argsort(lows[:, axis], kind='stable'); active = []; comparisons = 0
    for i in order:
        if _budget is not None:_budget.check()
        active = [j for j in active if highs[j, axis] > lows[i, axis]]
        if active:
            comparisons += len(active)
            if np.any(np.all(np.minimum(highs[active], highs[i]) > np.maximum(lows[active], lows[i]), axis=1)):
                raise LiquidError('Independent global liquid AABB audit found overlapping interiors')
        active.append(i)
    if _budget is not None:
        actual=_budget.counts()
        if actual!=dict(screened=screened,edges=edges,cells=len(cells)):
            raise LiquidError('Shared actual-work and canonical owner ledgers disagree')
        _budget.check()
    combined['ledger'] = dict(capacity_conserved=True, containment_certified=True, exclusions_certified=True,
                              disjoint_certified=True, single_native_tank_owner=True,
                              authoritative_boundary_certified=True,
                              native_owner_interval_certified=classifier is not None and vessel_policy is None,
                              global_aabb_nonoverlap_verified=True, global_aabb_pair_candidates=comparisons,
                              disjoint_basis=('per-tank declared vessel eligibility, prior actual cell subtraction, consumed slabs and independent AABB audit'
                                  if vessel_policy is not None else 'singleton native point-partition intervals, consumed box partitions and independent AABB audit'
                                  if classifier is not None else 'synthetic owner bounds minus prior actual allocated cells, independently AABB audited'),
                              screened_cells=screened, assignment_edges=edges,
                              tanks=sorted(checks, key=lambda x: x['tank']), sections=ledgers)
    combined['uncertainty']['maximum_cell_diagonal_m'] = max(float(np.linalg.norm(np.array(c['high_m'])-c['low_m'])) for c in cells)
    if vessel_policy is not None:
        combined['ledger']['per_tank_vessel_eligibility_certified'] = True
        combined['inferred_vessels_v1'] = vessel_policy.receipt(cells)
        combined['uncertainty']['inferred_vessel_attachment'] = (
            'Theoretical inference: sealed inboard root bladder remains attached to the whole native wing tank module; '
            'not a structural fracture, seal-survival or OEM tank-layout validation')
        combined['uncertainty']['native_lump_discrepancy'] = (
            'Actual spatial cell centroids/bounds differ from unchanged native lumped tank points and inertia; '
            'full and body -Z half-fill offsets are reported, not corrected or calibrated')
    return combined


def produce(aircraft, origin, *, glb, limits=None):
    """Normal model-stage producer, including explicit no-liquid applicability."""
    origin = _vec(origin, 'origin'); tanks = tank_records(aircraft, origin)
    from . import liquid_vessels
    declaration = liquid_vessels.read_declaration(aircraft.path, aircraft.name)
    if declaration is not None and not tanks: raise LiquidError('Inferred vessel declaration has no native liquid tanks')
    here = Path(__file__).parent
    source_paths = [('producer', Path(__file__)), ('geometry_body', here/'geometry/body.py'),
                    ('geometry_aircraft_engine', here/'geometry/aircraft.py'),
                    ('native_engine_order', here/'jsbsim.py'),
                    ('normal_model_stage', here/('rotorcraft/__init__.py' if
                     aircraft.spec.get('aircraft', {}).get('kind') in ('helicopter', 'multirotor') else 'pipeline.py'))]
    sources = [_source('design', aircraft.path)]+[_source(role, path) for role, path in source_paths]
    sources.append(dict(role='effective_design_spec', path='canonical-design-spec',
                        sha256=hashlib.sha256(json.dumps(aircraft.spec, sort_keys=True, allow_nan=False,
                                                       separators=(',', ':')).encode()).hexdigest()))
    if not tanks:
        return dict(schema=SCHEMA, design=aircraft.name, frame=FRAME, sources=sources, tanks=[], cells=[],
                    fluid_policy_v1=dict(FLUID_POLICY_V1),
                    no_liquid_reason='No positive liquid capacity in the authored native tank contract',
                    ledger=dict(capacity_conserved=True, containment_certified=True, exclusions_certified=True,
                                disjoint_certified=True, tanks=[]),
                    method=dict(version=1, origin_design_m=origin.tolist()),
                    uncertainty=dict(distribution='liquid-inapplicable'), limits=_limits(limits))
    from . import resources
    execution_sources=[('execution_resources',here/'resources.py'),
                       ('spawn_runtime',Path(mp.__file__).parent/'popen_spawn_win32.py')]
    sources += [_source(role,path) for role,path in execution_sources]
    with resources.cpu_lease():
        hull, receipt, sections = read_glb(glb, limits=limits, with_sections=True, owner_names_only=True)
        tanks = _ownership(tanks, sections, Path(glb).with_suffix('.damage.json'), aircraft.name, origin)
        damage_path = Path(glb).with_suffix('.damage.json')
        native_damage = json.loads(damage_path.read_text(encoding='utf-8')) if damage_path.is_file() else None
        excluded, exclusions = _exclusions(aircraft, origin, native_damage=native_damage)
        vessel_policy = None
        if declaration is not None:
            vessel_policy, vessel_sources = liquid_vessels.create(aircraft, origin, tanks, declaration)
            sources += vessel_sources
        from . import vulnerability
        from .shape import breakup
        binding_paths=[aircraft.path,glb,damage_path,Path(aircraft.path).with_name('calibration.toml'),
                       Path(aircraft.path).with_suffix('.liquid-geometry-v1.json')]
        binding_paths += [path for _,path in source_paths+execution_sources]
        binding_paths += [Path(vulnerability.__file__),Path(breakup.__file__),here/'geometry/surface.py',here/'geometry/airfoil.py']
        recipe=dict(design=aircraft.name,design_path=str(Path(aircraft.path).absolute()),glb=str(Path(glb).absolute()),
            origin=origin.tolist(),boundary_receipt=receipt,exclusions=exclusions,
            effective_spec=next(row['sha256'] for row in sources if row['role']=='effective_design_spec'),
            bindings=[_execution_binding(path) for path in binding_paths])
        for role,path in [('design',Path(aircraft.path))]+source_paths+execution_sources:
            expected=next(row['sha256'] for row in sources if row['role']==role)
            if _execution_binding(path)['sha256']!=expected:raise LiquidError('Mixed source epoch before owner dispatch')
        result = _pack_owned(aircraft.name, hull, sections, tanks, excluded, limits, aircraft=aircraft, origin=origin,
                             exclusion_boxes=exclusions, vessel_policy=vessel_policy,_recipe=recipe)
    from . import vulnerability
    from .shape import breakup
    sources += [_source('glb', glb), _source('exclusion_method', vulnerability.__file__),
                _source('native_partition_method', breakup.__file__),
                _source('manifold_binary', _mf().__file__)]
    result['sources'] = sources; result['boundary_receipt'] = receipt
    result['sources'].append(dict(role='native_tank_ownership', path='canonical-ownership',
                                  sha256=hashlib.sha256(json.dumps([(t['index'], t['section']) for t in tanks],
                                                                  separators=(',', ':')).encode()).hexdigest()))
    result['sources'].append(dict(role='native_material_exclusion', path='canonical-native-material-exclusion',
        sha256=hashlib.sha256(json.dumps(_native_exclusion_payload(native_damage, origin), sort_keys=True,
                                        separators=(',', ':'), allow_nan=False).encode()).hexdigest()))
    result['exclusions'] = exclusions
    result['uncertainty']['density'] = 'estimated constant reference density: jet 800 or piston avgas 720 kg/m3; no thermal correction'
    result['method'] = dict(version=1, manifold_version=importlib.metadata.version('manifold3d'),
                          numpy_version=np.__version__, origin_design_m=origin.tolist())
    if recipe.get('_deadline') is not None and time.monotonic()>recipe['_deadline']:
        raise LiquidError('Liquid global time deadline exceeded before complete result admission')
    return result


def write(aircraft, path, origin, *, glb, limits=None):
    result = produce(aircraft, origin, glb=glb, limits=limits)
    atomic_write_text(path, json.dumps(result, sort_keys=True, indent=2, allow_nan=False)+'\n')
    # Annotate only a real existing damage document. Unsectioned rotor products
    # remain standalone; an empty proxy must not imply a native damage recipient.
    damage_path = Path(glb).with_suffix('.damage.json')
    if damage_path.is_file():
        damage = json.loads(damage_path.read_text(encoding='utf-8'))
        if damage.get('aircraft') != aircraft.name: raise LiquidError('Damage declaration design mismatch')
        if result['tanks']:
            damage['liquid_geometry_v1'] = dict(file=Path(path).name, schema=SCHEMA, required=True)
        else: damage.pop('liquid_geometry_v1', None)
        atomic_write_text(damage_path, json.dumps(damage, indent=1, allow_nan=False)+'\n')
    return result
