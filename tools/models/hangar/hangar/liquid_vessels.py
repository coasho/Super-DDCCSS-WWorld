"""Explicit inferred inner vessels; never alter flight mass or native ownership.

The only v1 declaration is RQ4B's two inboard wing tanks. The extra eligible
solid is the original, uninflated source wing between its centreline and the
unchanged structural root cut. The original whole hull and all exclusions
remain separate mandatory clips in liquid.py. This is not OEM tank CAD.
"""
import hashlib
import heapq
import json
import math
import numbers
import sys
from collections import Counter
from fractions import Fraction
from pathlib import Path
import time

import numpy as np

from . import liquid

SCHEMA = 'fsim.inferred-liquid-vessels.v1'
ORDINARY = 'native_owner_interval_v1'
ROOT = 'inferred_original_wing_root_v1'
PACKING = 'nested_eligible_shared_pool_v1'


def _mesh_arrays(mesh):
    """Read actual MeshGL64 positions/indices; never weld by coordinates."""
    try:
        positions = np.asarray(mesh.vert_properties, dtype=float)
        triangles = np.asarray(mesh.tri_verts)
        if (positions.ndim != 2 or positions.shape[1] < 3 or not np.isfinite(positions[:, :3]).all() or
                triangles.ndim != 2 or triangles.shape[1] != 3 or not len(triangles) or triangles.dtype.kind not in 'iu' or
                np.any(triangles < 0) or np.any(triangles >= len(positions))): return None
        return positions[:, :3], triangles
    except (AttributeError, TypeError, ValueError): return None


def _represented_box(mesh, lo, hi):
    """Sufficient exact eight-corner/twelve-triangle represented-box proof.

    The caller first validates positive materialized S=Box intersect domain.
    Property seams use ONLY the supplied MeshGL64 merge topology. Two outward
    triangles per face, reverse-paired edges and unique corner roots exclude
    holes, duplicate shells and multiplicity. Unrecognized meshes return false.
    """
    data = _mesh_arrays(mesh)
    if data is None: return False
    p, triangles = data
    if (len(triangles) != 12 or np.shape(lo) != (3,) or np.shape(hi) != (3,) or
            not np.isfinite(lo).all() or not np.isfinite(hi).all() or np.any(hi <= lo) or
            not np.all((p == lo) | (p == hi)) or len(set(triangles.reshape(-1).tolist())) != len(p)):
        return False
    bits = (p == hi).astype(np.int64); codes = bits[:, 0]+2*bits[:, 1]+4*bits[:, 2]
    if set(codes.tolist()) != set(range(8)): return False
    parents = list(range(len(p)))
    def find(i):
        while parents[i] != i: parents[i] = parents[parents[i]]; i = parents[i]
        return i
    try:
        source = np.asarray(mesh.merge_from_vert); target = np.asarray(mesh.merge_to_vert)
        if source.ndim != 1 or target.ndim != 1 or len(source) != len(target): return False
        if len(source) and (source.dtype.kind not in 'iu' or target.dtype.kind not in 'iu' or
                           np.any(source < 0) or np.any(target < 0) or
                           np.any(source >= len(p)) or np.any(target >= len(p))): return False
        for a, b in zip(source.tolist(), target.tolist()):
            if not np.array_equal(p[a], p[b]): return False
            parents[find(a)] = find(b)
    except (AttributeError, TypeError, ValueError, IndexError): return False
    roots = [find(i) for i in range(len(p))]
    if any(len({roots[i] for i, code in enumerate(codes) if code == k}) != 1 for k in range(8)): return False
    faces = Counter(); edges = Counter()
    for triangle in triangles:
        a, b, c = map(int, triangle)
        normal = np.cross(bits[b]-bits[a], bits[c]-bits[a])
        axes = np.flatnonzero(normal)
        if len(axes) != 1: return False
        axis = int(axes[0]); side = int(bits[a, axis])
        if (not np.all(bits[triangle, axis] == side) or abs(int(normal[axis])) != 1 or
                (int(normal[axis]) > 0) != bool(side)): return False
        faces[(axis, side)] += 1
        for x, y in ((a, b), (b, c), (c, a)):
            if roots[x] == roots[y]: return False
            edges[(roots[x], roots[y])] += 1
    return (len(faces) == 6 and all(v == 2 for v in faces.values()) and
            all(v == 1 and edges[(b, a)] == 1 for (a, b), v in edges.items()))


def _strict_partial_box(candidate, lo, hi, check=None):
    """Sufficient NEGATIVE-containment proof for an already admitted solid.

    A strict outside box corner and a plane bounding EVERY candidate vertex
    prove a positive-volume part of Box lies outside candidate's convex hull.
    No positive containment is admitted here. 64 vertices/128 faces/4 planes
    bound optional arithmetic; unavailable/unproven inputs retain the original
    strict Boolean. Float scores choose work only; every final sign is exact.
    No persistent cache, moved plane, epsilon or model/budget change.
    """
    if check is not None: check()
    if candidate.num_vert() > 64 or candidate.num_tri() > 128: return False
    data = _mesh_arrays(candidate.to_mesh64())
    if data is None: return False
    p, triangles = data
    if (len(p) > 64 or len(triangles) > 128 or np.shape(lo) != (3,) or np.shape(hi) != (3,) or
            not np.isfinite(lo).all() or not np.isfinite(hi).all() or np.any(hi <= lo)): return False
    # Min/max select existing represented coordinates exactly, with no
    # arithmetic tolerance. A corner beyond this axis hull proves NOT-contained.
    minimum, maximum = np.min(p, axis=0), np.max(p, axis=0)
    if np.any(maximum <= minimum): return False
    if np.any(lo < minimum) or np.any(hi > maximum): return True
    # One bounded batched ranking replaces per-face/per-sign NumPy dispatch.
    # These floats select optional work ONLY; all final plane signs below
    # remain exact. A missed/poor candidate retains the original strict proof.
    if check is not None: check()
    with np.errstate(over='ignore', invalid='ignore', under='ignore', divide='ignore'):
        vertices = p[triangles]
        origins = np.repeat(vertices[:, 0], 2, axis=0)
        normals = np.repeat(np.cross(vertices[:, 1]-vertices[:, 0], vertices[:, 2]-vertices[:, 0]), 2, axis=0)
        normals[1::2] *= -1
        scales = np.max(np.abs(normals), axis=1)
        corners = np.where(normals >= 0, hi, lo)
        box_values = np.einsum('fi,fi->f', normals, corners-origins)
        candidate_values = np.einsum('fvi,fi->fv', p[None, :, :]-origins[:, None, :], normals)
        scores = (box_values-np.maximum(0., np.max(candidate_values, axis=1)))/scales
        valid = np.isfinite(normals).all(axis=1) & np.isfinite(scales) & (scales > 0) & np.isfinite(scores) & (scores > 0)
        choices = [(-float(scores[i]), int(i//2), 1 if i % 2 == 0 else -1) for i in np.flatnonzero(valid)]
    if check is not None: check()
    if not choices: return False
    choices.sort()
    exact = [tuple(Fraction.from_float(float(x)) for x in row) for row in p]
    lower, upper = [tuple(Fraction.from_float(float(x)) for x in row) for row in (lo, hi)]
    for _, index, sign in choices[:4]:
        if check is not None: check()
        a, b, c = (exact[int(i)] for i in triangles[index])
        u, v = tuple(y-x for x, y in zip(a, b)), tuple(y-x for x, y in zip(a, c))
        n = tuple(sign*x for x in (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0]))
        if not any(n): continue
        corner = tuple(upper[i] if n[i] >= 0 else lower[i] for i in range(3))
        if sum(n[i]*(corner[i]-a[i]) for i in range(3)) <= 0: continue
        supported = True
        for count, point in enumerate(exact):
            if check is not None and count % 16 == 0: check()
            if sum(n[i]*(point[i]-a[i]) for i in range(3)) > 0:
                supported = False; break
        if supported: return True
    return False


def _cell_contained(box, candidate, overlap, lo, hi, check=None):
    """Original represented-box/strict Boolean classification seam."""
    contained = False; mesh = None
    if overlap.volume() > 0 and overlap.num_tri() == 12:
        mesh = overlap.to_mesh64(); contained = _represented_box(mesh, lo, hi)
    if not contained and _strict_partial_box(candidate, lo, hi, check): return False, mesh
    # A validated materialized intersection can expose a supporting plane
    # absent from the bounded candidate shortlist. Partiality is relative to
    # this represented intersection, never positive containment or capacity.
    if not contained and _strict_partial_box(overlap, lo, hi, check): return False, mesh
    if not contained:
        contained = liquid._valid(box-candidate, 'Cell/interior difference').is_empty()
    return contained, mesh


def _inscribed_vertical_band(mesh, lo, hi, check=None):
    """Same exact facet inequalities on a common binary64 dyadic grid.

    Integer operations avoid intermediate Fraction normalization. Only final
    selected bounds/sensitivities become Fractions; no geometric sign, endpoint
    rounding, proposal/fallback rule or deadline callback position changes.
    """
    data = _mesh_arrays(mesh)
    if data is None or np.any(hi <= lo): return None
    positions, triangles = data
    ratios = [tuple(float(x).as_integer_ratio() for x in row) for row in positions]
    ratios += [tuple(float(x).as_integer_ratio() for x in row) for row in (lo, hi)]
    exponent = max(denominator.bit_length()-1 for row in ratios for _, denominator in row)
    grid = 1 << exponent
    scaled = [tuple(numerator << (exponent-(denominator.bit_length()-1))
                    for numerator, denominator in row) for row in ratios]
    p, low, high = scaled[:-2], scaled[-2], scaled[-1]
    del ratios, scaled
    lower, upper = (low[2], 1), (high[2], 1)
    zero = ((0, 1), (0, 1)); lower_s = upper_s = vertical_s = zero
    conflict = False
    def larger(a, b): return a[0]*b[1] > b[0]*a[1]
    def equal(a, b): return a[0]*b[1] == b[0]*a[1]
    def maxima(a, b): return tuple(x if larger(x, y) else y for x, y in zip(a, b))
    for triangle in triangles:
        if check is not None: check()
        a, b, c = (p[int(i)] for i in triangle)
        u, v = tuple(y-x for x, y in zip(a, b)), tuple(y-x for x, y in zip(a, c))
        n = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
        if not any(n): return None
        bound = sum(x*y for x, y in zip(n, a))
        maximum = sum(n[i]*(high[i] if n[i] >= 0 else low[i]) for i in (0, 1))
        if n[2] == 0:
            if maximum > bound:
                conflict = True
                changes = tuple(abs(n[i])*(high[i]-low[i]) for i in (0, 1))
                scale = sum(changes)
                if not scale: return None
                vertical_s = maxima(vertical_s, tuple((change, scale) for change in changes))
            continue
        numerator, denominator = bound-maximum, n[2]
        if denominator < 0: numerator, denominator = -numerator, -denominator
        value = (numerator, denominator)
        sensitivity = tuple((abs(n[i])*(high[i]-low[i]), abs(n[2])*grid) for i in (0, 1))
        if n[2] < 0:
            if larger(value, lower): lower, lower_s = value, sensitivity
            elif equal(value, lower): lower_s = maxima(lower_s, sensitivity)
        else:
            if larger(upper, value): upper, upper_s = value, sensitivity
            elif equal(value, upper): upper_s = maxima(upper_s, sensitivity)
    lower = Fraction(lower[0], lower[1]*grid)
    upper = Fraction(upper[0], upper[1]*grid)
    try:
        a, b = float(lower), float(upper)
        if not math.isfinite(a) or not math.isfinite(b): return None
        if Fraction.from_float(a) < lower: a = math.nextafter(a, math.inf)
        if Fraction.from_float(b) > upper: b = math.nextafter(b, -math.inf)
    except (OverflowError, ValueError): return None
    chosen = vertical_s if conflict else maxima(lower_s, upper_s)
    return dict(lower=a, upper=b, vertical_conflict=conflict,
                sensitivity=tuple(Fraction(numerator, denominator) for numerator, denominator in chosen))


def _column_residual_axis(proposal, lo, hi, max_edge):
    widths = hi-lo
    if max(widths) > max_edge*(1+1e-12) or proposal is None: return int(np.argmax(widths))
    sensitivity = proposal['sensitivity']
    return int(sensitivity[1] > sensitivity[0]) if any(sensitivity) else int(np.argmax(widths))


def _column_partition(lo, hi, lower, upper):
    """Exact represented central/lower/upper boxes partition parent interiors."""
    if not (math.isfinite(lower) and math.isfinite(upper) and lo[2] <= lower < upper <= hi[2]): return []
    a, b = lo.copy(), hi.copy(); a[2], b[2] = lower, upper
    result = [(a, b, 'core')]
    if lower > lo[2]:
        b = hi.copy(); b[2] = lower; result.append((lo.copy(), b, 'lower'))
    if upper < hi[2]:
        a = lo.copy(); a[2] = upper; result.append((a, hi.copy(), 'upper'))
    return result



def _column_core_box(low, high):
    """Materialize the optional proposal with the original exact box endpoints.

    This intentionally matches liquid._box's eight vertices/twelve triangles.
    It exposes the object before strict validation only at this proposal seam;
    ordinary cells and all original Boolean proofs still use liquid._box.
    """
    vertices = np.array([[low[0], low[1], low[2]], [high[0], low[1], low[2]],
                         [high[0], high[1], low[2]], [low[0], high[1], low[2]],
                         [low[0], low[1], high[2]], [high[0], low[1], high[2]],
                         [high[0], high[1], high[2]], [low[0], high[1], high[2]]])
    triangles = np.array([[0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7],
                          [0, 1, 5], [0, 5, 4], [1, 2, 6], [1, 6, 5],
                          [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]], dtype=np.uint64)
    return liquid._mf().Manifold(liquid._mf().Mesh64(vertices, triangles))


def _column_core(low, high, report=None):
    """Observe an optional proposal; hard geometry refusals remain unchanged."""
    solid = _column_core_box(low, high)
    status, volume, empty = solid.status(), solid.volume(), solid.is_empty()
    unusable = empty or volume == 0
    hard = status != liquid._mf().Error.NoError or not math.isfinite(volume) or volume < 0
    if report is not None and (hard or unusable):
        report(dict(core_low_m=low.tolist(), core_high_m=high.tolist(),
                    core_low_hex=[float(v).hex() for v in low],
                    core_high_hex=[float(v).hex() for v in high],
                    core_extent_m=(high-low).tolist(),
                    kernel_status=str(status), kernel_is_empty=bool(empty),
                    kernel_volume_m3=volume if math.isfinite(volume) else repr(volume),
                    kernel_vertices=solid.num_vert(), kernel_triangles=solid.num_tri(),
                    mesh_input_tolerance=0, hard_refusal=bool(hard)))
    if unusable and not hard:
        # No capacity or material is credited. The unchanged parent overlap
        # follows the ordinary bounded split below; it is not discarded.
        return None
    return liquid._valid(solid, 'Cell box', nonempty=True)


def _nested_requirements(tanks):
    """Only the declared two nested pairs, never a general tank reassignment."""
    by_index = {t['index']: t for t in tanks}
    if len(tanks) != 4 or set(by_index) != {0, 1, 2, 3}:
        raise liquid.LiquidError('Inferred shared pool requires the four unchanged RQ native tanks')
    pairs = []
    for flexible, restricted, section in ((0, 2, 'left wing'), (1, 3, 'right wing')):
        if any(by_index[i]['section'] != section for i in (flexible, restricted)):
            raise liquid.LiquidError('Inferred shared pool native mechanical owner mismatch')
        demand = [by_index[i]['capacity_kg']/by_index[i]['density_kg_m3'] for i in (restricted, flexible)]
        pairs.append((restricted, flexible, demand[0], math.fsum(demand)))
    return pairs


def _audit_disjoint(cells, check):
    """Independent strict positive-interior audit; shared faces are legal."""
    lows = np.array([c['low_m'] for c in cells]); highs = np.array([c['high_m'] for c in cells])
    axis = int(np.argmax((highs.max(axis=0)-lows.min(axis=0))/(highs-lows).sum(axis=0)))
    order = np.argsort(lows[:, axis], kind='stable'); active = []; comparisons = 0
    for i in order:
        check()
        active = [j for j in active if highs[j, axis] > lows[i, axis]]
        comparisons += len(active)
        if active and np.any(np.all(np.minimum(highs[active], highs[i]) > np.maximum(lows[active], lows[i]), axis=1)):
            raise liquid.LiquidError('Independent global liquid AABB audit found overlapping interiors')
        active.append(i)
    return comparisons


def _prior_clear(lo, hi, bounds):
    """Exact original AABB predicate over immutable earlier-tank endpoints."""
    lows, highs = bounds
    return not np.any(np.all(np.minimum(highs, hi) > np.maximum(lows, lo), axis=1))


def _prior_endpoints(cells):
    """Validate original material boxes once before immutable native-tank use."""
    try:
        lows = np.array([liquid._vec(c['low_m'], 'prior material low') for c in cells]).reshape((-1, 3))
        highs = np.array([liquid._vec(c['high_m'], 'prior material high') for c in cells]).reshape((-1, 3))
        if np.any(highs <= lows): raise liquid.LiquidError('Nonpositive prior material box')
    except (KeyError, TypeError, ValueError) as refused:
        raise liquid.LiquidError('Malformed prior material box') from refused
    for endpoints in (lows, highs): endpoints.setflags(write=False)
    return lows, highs


def _contract_overlap(overlap, lo, hi):
    """Losslessly enclose represented overlap vertices, or keep the old box.

    BoundingBox bounds ALL vertices of the already materialized Manifold.
    Triangle interiors lie within those extrema. An outlying/unsupported box
    cannot justify discarding any part of the parent and therefore falls back.
    """
    try: bounds = np.asarray(overlap.bounding_box(), dtype=float)
    except (TypeError, ValueError): return lo, hi
    if (bounds.shape != (6,) or not np.isfinite(bounds).all() or np.any(bounds[3:] <= bounds[:3]) or
            np.any(bounds[:3] < lo) or np.any(bounds[3:] > hi)):
        return lo, hi
    # The API returns the actual float64 vertex extrema, not a centre/extent
    # reconstruction. Extra outward padding is unnecessary and can cross an
    # EXACT equipment/cavity plane by one ULP. Never move a represented face.
    return bounds[:3].copy(), bounds[3:].copy()


def _pack_shared(design, hull, tanks, excluded, limits, policy, *, exclusion_boxes=None, deadline=None):
    """RQ-only serial shared pool with definite per-tank material eligibility.

    Search the original unpunctured material once. For each nested pair,
    ordinary certified volume must meet restricted demand, AND all certified
    eligible volume must meet the pair's total demand. Exact consumed slabs
    then realize those fractional-volume constraints restricted-first.
    Neither geometric available.volume nor an unresolved mask admits capacity.
    """
    limits = liquid._limits(limits); mf = liquid._mf()
    deadline = time.monotonic()+limits['max_seconds'] if deadline is None else deadline
    def check():
        if time.monotonic() > deadline:
            raise liquid.LiquidError('Liquid global time deadline exceeded')
    check(); liquid._valid(hull, 'Structural hull', nonempty=True)
    tanks = [dict(t, anchor_body_m=liquid._vec(t['anchor_body_m'], 'Tank anchor').tolist()) for t in tanks]
    for t in tanks:
        if type(t['index']) is not int or t['index'] < 0:
            raise liquid.LiquidError('Invalid native tank index')
        for key in ('capacity_kg', 'density_kg_m3'):
            if isinstance(t[key], bool) or not isinstance(t[key], numbers.Real) or not math.isfinite(t[key]) or t[key] <= 0:
                raise liquid.LiquidError('Finite positive capacity and physical density required')
    tanks.sort(key=lambda t: t['index']); pairs = _nested_requirements(tanks)
    if design != 'rq4b' or policy.selected != {0, 1} or policy.assignment_order([0, 1, 2, 3]) != [2, 3, 0, 1]:
        raise liquid.LiquidError('Unsupported inferred shared-pool policy')
    if {t['index']: t for t in tanks} != policy.tanks:
        raise liquid.LiquidError('Shared pool differs from captured native tank policy')
    volumes = {t['index']: t['capacity_kg']/t['density_kg_m3'] for t in tanks}
    if any(not math.isfinite(v) or v <= 0 for v in volumes.values()):
        raise liquid.LiquidError('Invalid physical liquid volume')
    required = math.fsum(volumes.values()); targets = {i: v/64 for i, v in volumes.items()}
    available = liquid._valid(hull-excluded, 'Available interior excluding equipment') if excluded is not None else hull
    bounds = np.array(available.bounding_box()); low, high = bounds[:3], bounds[3:]
    max_edge = max(high-low)/8
    if available.volume()+liquid._tolerance(required, 1) < required:
        raise liquid.LiquidError('insufficient available interior volume')
    # This is only a necessary search clip. Final eligibility and original-hull
    # checks remain fresh for every actual emitted slab.
    regions = [policy.region(i, low, high) for i in range(4)]
    region = liquid._valid(mf.Manifold.batch_boolean(regions, mf.OpType.Add), 'Shared necessary vessel region')
    available = liquid._valid(available ^ region, 'Original hull/necessary vessel region clip')
    if available.volume()+liquid._tolerance(required, 1) < required:
        raise liquid.LiquidError('insufficient physical volume in necessary vessel region')
    bounds = np.array(available.bounding_box()); low, high = bounds[:3], bounds[3:]
    anchors = np.array([t['anchor_body_m'] for t in tanks])
    pending = []; pool = []; sequence = 0; screened = 0
    # Same bounded immutable partial-parent reuse as ordinary pack. Children
    # are exact midpoint subsets; cache pressure falls back to ORIGINAL available.
    records = {}; cache_sequence = 0; cached_vertices = 0; cached_triangles = 0
    def admit(solid):
        nonlocal cache_sequence, cached_vertices, cached_triangles
        vertices, triangles = solid.num_vert(), solid.num_tri()
        if (cached_vertices+vertices > limits['max_vertices'] or cached_triangles+triangles > limits['max_triangles'] or
                len(records)+1 > limits['max_screened_cells']): return None
        key = cache_sequence; cache_sequence += 1
        records[key] = [solid, vertices, triangles, 0]
        cached_vertices += vertices; cached_triangles += triangles
        return key
    def retain(key):
        if key is not None: records[key][3] += 1
    def release(key):
        nonlocal cached_vertices, cached_triangles
        if key is None: return
        row = records[key]; row[3] -= 1
        if row[3] == 0:
            cached_vertices -= row[1]; cached_triangles -= row[2]; del records[key]
    def push(lo, hi, depth, contained=False, domain=None, definite=None):
        nonlocal sequence
        # Only all-definite parent facts propagate to exact subsets. Unknown
        # predicates are reevaluated using ONE original classifier query.
        values = policy._certificates(lo, hi) if definite is None else definite
        if len(values) != 4 or any(type(v) is not int or v not in (-1, 0, 1) for v in values):
            raise liquid.LiquidError('Invalid inferred vessel eligibility certificate')
        unknown = 0 in values
        mask = frozenset(i for i, value in enumerate(values) if value == 1)
        # A true flexible predicate does not turn unknown restricted ownership
        # into a flexible-only cell. All four masks must be definite first.
        distance = float(np.min(np.sum(np.maximum(np.maximum(lo-anchors, anchors-hi), 0.)**2, axis=1)))
        retain(domain)
        heapq.heappush(pending, (depth, int(unknown), distance, sequence, tuple(lo), tuple(hi), mask, unknown, contained, domain))
        sequence += 1
    def split(lo, hi, depth, contained=False, domain=None, definite=None, axis=None):
        axis = int(np.argmax(hi-lo)) if axis is None else axis
        midpoint = float((lo[axis]+hi[axis])*.5)
        if depth >= limits['max_depth'] or hi[axis]-lo[axis] <= limits['min_extent_m'] or midpoint in (lo[axis], hi[axis]):
            raise liquid.LiquidError('Liquid spatial refinement budget exhausted before capacity closure')
        left = hi.copy(); left[axis] = midpoint; right = lo.copy(); right[axis] = midpoint
        push(lo, left, depth+1, contained, domain, definite); push(right, hi, depth+1, contained, domain, definite)
    certified = {i: 0. for i in range(4)}
    def feasible():
        return all(certified[r] >= dr and certified[f] >= total for r, f, dr, total in pairs)
    core_reported = False
    def report_core(context, info):
        nonlocal core_reported
        if core_reported: return
        core_reported = True
        # One bounded raw diagnostic per invocation. Do not filter background
        # output through a buffered pipeline; root retains the complete log.
        print('HANGAR_LIQUID_OPTIONAL_CORE '+json.dumps(dict(context, **info),
              allow_nan=False, sort_keys=True), file=sys.stderr, flush=True)
    push(low, high, 0)
    while pending and not feasible():
        if screened >= limits['max_screened_cells']:
            raise liquid.LiquidError('Liquid geometry screening budget exceeded')
        check(); depth, _, _, _, lo, hi, mask, unknown, contained, domain = heapq.heappop(pending)
        lo, hi = np.array(lo), np.array(hi); screened += 1
        if unknown:
            box = None; candidate = None; overlap = None
            try:
                box = liquid._box(lo, hi)
                candidate = available if domain is None else records[domain][0]
                overlap = liquid._valid(box ^ candidate, 'Unknown cell/interior intersection')
                if not overlap.is_empty():
                    child_domain = admit(overlap)
                    lo, hi = _contract_overlap(overlap, lo, hi)
                    split(lo, hi, depth, False, child_domain)
            finally:
                overlap = None; candidate = None; box = None
                release(domain)
            continue
        if not mask:
            release(domain); continue
        # The only supported masks are the two nested side families. This
        # prevents an accidental shared root/cross-side eligibility expansion.
        if mask not in (frozenset({0, 2}), frozenset({1, 3}), frozenset({0}), frozenset({1})):
            raise liquid.LiquidError('Non-nested inferred vessel eligibility mask')
        definite = tuple(1 if i in mask else -1 for i in range(4))
        child_domain = None; overlap = None; candidate = None; box = None; mesh = None; proposal = None; core = None; proof = None
        try:
            if not contained:
                box = liquid._box(lo, hi); candidate = available if domain is None else records[domain][0]
                overlap = liquid._valid(box ^ candidate, 'Cell/interior intersection')
                if overlap.is_empty(): continue
                contained, mesh = _cell_contained(box, candidate, overlap, lo, hi, check)
            if not contained:
                parent_lo, parent_hi = lo.copy(), hi.copy()
                lo, hi = _contract_overlap(overlap, lo, hi)
                # Coarse over-edge cells first obey the unchanged edge rule;
                # local proposals then inspect ALL materialized oriented facets.
                if max(hi-lo) <= max_edge*(1+1e-12):
                    if mesh is None: mesh = overlap.to_mesh64()
                    proposal = _inscribed_vertical_band(mesh, lo, hi, check)
                if proposal is not None and not proposal['vertical_conflict']:
                    parts = _column_partition(lo, hi, proposal['lower'], proposal['upper'])
                    if parts and all(kind == 'core' or b[2]-a[2] > limits['min_extent_m'] for a, b, kind in parts):
                        core_lo, core_hi, _ = parts[0]
                        context = dict(design=design, depth=depth, screened_cells=screened,
                                       parent_low_m=parent_lo.tolist(), parent_high_m=parent_hi.tolist(),
                                       contracted_low_m=lo.tolist(), contracted_high_m=hi.tolist(),
                                       proposed_lower_m=proposal['lower'], proposed_upper_m=proposal['upper'],
                                       lower_hex=float(proposal['lower']).hex(), upper_hex=float(proposal['upper']).hex(),
                                       min_extent_m=limits['min_extent_m'], max_edge_m=max_edge,
                                       overlap_volume_m3=overlap.volume())
                        core = _column_core(core_lo, core_hi, lambda info: report_core(context, info))
                        core_contained = False
                        if core is not None:
                            proof = liquid._valid(core ^ overlap, 'Column core/interior intersection')
                            core_contained = (not proof.is_empty() and proof.volume() > 0 and proof.num_tri() == 12 and
                                              _represented_box(proof.to_mesh64(), core_lo, core_hi))
                            if not core_contained:
                                core_contained = liquid._valid(core-overlap, 'Column core/interior difference').is_empty()
                        proof = None; core = None
                        if core_contained:
                            if len(parts) == 1:
                                lo, hi, contained = core_lo, core_hi, True
                            else:
                                if depth >= limits['max_depth']:
                                    raise liquid.LiquidError('Liquid spatial refinement budget exhausted before capacity closure')
                                child_domain = admit(overlap)
                                for a, b, kind in parts:
                                    push(a, b, depth+1, kind == 'core', None if kind == 'core' else child_domain, definite)
                                continue
            volume = math.prod(hi-lo); target = min(targets[i] for i in mask)
            if contained and volume <= target*(1+1e-12) and max(hi-lo) <= max_edge*(1+1e-12):
                pool.append([lo, hi, mask])
                for i in mask: certified[i] = math.fsum((certified[i], volume))
                if len(pool) > limits['max_cells']: raise liquid.LiquidError('Liquid cell budget exceeded')
            else:
                child_domain = admit(overlap) if overlap is not None else None
                split(lo, hi, depth, contained, child_domain, definite,
                      None if contained else _column_residual_axis(proposal, lo, hi, max_edge))
        finally:
            overlap = None; candidate = None; box = None; mesh = None; core = None; proof = None
            release(domain)
    for item in pending: release(item[-1])
    pending.clear()
    if records or cached_vertices or cached_triangles:
        raise liquid.LiquidError('Shared pool retained unfinished geometric cache references')
    if not feasible(): raise liquid.LiquidError('insufficient nested certified cell volume')
    edges = len(pool)*len(tanks)
    if edges > limits['max_assignment_edges']: raise liquid.LiquidError('Liquid nearest-anchor assignment budget exceeded')
    # Record capacity feasibility before mutating the disjoint pool by exact
    # slabs. These are certified cell volumes, never available.volume estimates.
    feasibility = [dict(restricted_tank=r, flexible_tank=f, ordinary_certified_m3=certified[r],
                        total_certified_m3=certified[f], restricted_required_m3=dr, total_required_m3=total)
                   for r, f, dr, total in pairs]
    centres = np.array([(lo+hi)*.5 for lo, hi, _ in pool]); cells = []; checks = []
    for index in (2, 3, 0, 1):
        check(); tank = tanks[index]; anchor = anchors[index]
        prior_bounds = None
        if cells:
            # Snapshot ALL and ONLY earlier native tanks once. Same-tank
            # disjointness follows from consumed slabs and the final independent
            # audit. Endpoint arithmetic is identical to the original predicate.
            prior_bounds = _prior_endpoints(cells)
        order = np.argsort(np.sum((centres-anchor)**2, axis=1), kind='stable')
        allocated = []; remaining = volumes[index]
        for item in order:
            if remaining <= 0: break
            check(); lo, hi, mask = pool[item]
            if index not in mask or np.any(hi <= lo): continue
            lo, hi = lo.copy(), hi.copy(); free = math.prod(hi-lo); take = min(remaining, free)
            slab_low, slab_high = lo.copy(), hi.copy()
            if take < free:
                axis = int(np.argmax(hi-lo)); width = take/math.prod(np.delete(hi-lo, axis))
                if anchor[axis] <= centres[item, axis]:
                    slab_high[axis] = lo[axis]+width; pool[item][0] = slab_high.copy()
                    pool[item][0][np.arange(3) != axis] = lo[np.arange(3) != axis]
                else:
                    slab_low[axis] = hi[axis]-width; pool[item][1] = slab_low.copy()
                    pool[item][1][np.arange(3) != axis] = hi[np.arange(3) != axis]
            else: pool[item][0] = hi.copy()
            if np.any(slab_high <= slab_low): raise liquid.LiquidError('Unrepresentable positive liquid slab')
            slab = liquid._box(slab_low, slab_high)
            if policy.certificate(index, slab_low, slab_high) != 1:
                raise liquid.LiquidError('Emitted liquid slab has uncertified native vessel membership')
            eligibility = policy.eligibility(index, slab_low, slab_high)
            if not liquid._valid(slab-hull, 'Emitted slab/hull difference').is_empty():
                raise liquid.LiquidError('Emitted liquid slab escapes structural boundary')
            if prior_bounds is not None:
                # Every prior material is an actual validated AABB. Exact
                # positive-interior broadphase is complete, including one-ULP
                # penetration. Zero-volume face/edge contact requires no CSG;
                # a general contact intersection can retain degenerate negative
                # signed volume. No such result is computed or accepted here.
                if not _prior_clear(slab_low, slab_high, prior_bounds):
                    raise liquid.LiquidError('Emitted liquid slab overlaps prior actual liquid cells')
            if exclusion_boxes is not None:
                if not liquid._clear_of_exclusion_boxes(slab_low, slab_high, exclusion_boxes):
                    raise liquid.LiquidError('Emitted liquid slab has positive interior overlap with occupied equipment box')
            elif excluded is not None and not liquid._valid(slab ^ excluded, 'Emitted slab/exclusion intersection').is_empty():
                raise liquid.LiquidError('Emitted liquid slab intersects occupied equipment')
            actual = math.prod(slab_high-slab_low)
            cells.append(dict(id=len(cells), tank=index, section=tank['section'], low_m=slab_low.tolist(),
                              high_m=slab_high.tolist(), volume_m3=actual, eligibility_v1=eligibility))
            if len(cells) > limits['max_cells']: raise liquid.LiquidError('Emitted liquid cell budget exceeded')
            allocated.append(actual); residual = volumes[index]-math.fsum(allocated)
            remaining = 0. if abs(residual) <= liquid._tolerance(volumes[index], len(allocated)) else max(0., residual)
        actual = math.fsum(allocated); tolerance = liquid._tolerance(volumes[index], len(allocated))
        if not allocated or abs(actual-volumes[index]) > tolerance:
            raise liquid.LiquidError('Liquid tank capacity ledger failed for native tank '+str(index))
        checks.append(dict(tank=index, required_m3=volumes[index], actual_m3=actual,
                           error_m3=actual-volumes[index], tolerance_m3=tolerance))
    comparisons = _audit_disjoint(cells, check); check()
    receipt = policy.receipt(cells)
    receipt['packing_v1'] = dict(method=PACKING, assignment_order=[2, 3, 0, 1],
        target_volume_m3_by_tank=[dict(tank=i, target_m3=targets[i]) for i in range(4)],
        definite_eligibility_masks=True, nested_certified_feasibility=feasibility)
    ledger = dict(capacity_conserved=True, containment_certified=True, exclusions_certified=True,
        disjoint_certified=True, single_native_tank_owner=True, authoritative_boundary_certified=True,
        native_owner_interval_certified=False, per_tank_vessel_eligibility_certified=True,
        global_aabb_nonoverlap_verified=True, global_aabb_pair_candidates=comparisons,
        disjoint_basis='definite nested vessel eligibility, consumed shared box pool, exact prior AABB positive-interior proof (zero-contact CSG omitted), independent AABB audit',
        screened_cells=screened, certified_pool_cells=len(pool), assignment_edges=edges,
        tanks=sorted(checks, key=lambda x: x['tank']), sections=[])
    for section in ('left wing', 'right wing'):
        ledger['sections'].append(dict(section=section, capacity_conserved=True, containment_certified=True,
            exclusions_certified=True, disjoint_certified=True,
            tanks=[row for row in ledger['tanks'] if tanks[row['tank']]['section'] == section],
            assignment_order=[i for i in (2, 3, 0, 1) if tanks[i]['section'] == section]))
    check()
    return dict(schema=liquid.SCHEMA, design=design, frame=liquid.FRAME, sources=[], tanks=tanks, cells=cells,
        fluid_policy_v1=dict(liquid.FLUID_POLICY_V1), ledger=ledger, inferred_vessels_v1=receipt, limits=limits,
        uncertainty=dict(distribution='estimated nested eligible shared-pool nearest-anchor occupancy; not OEM tank layout',
            boundary='delivered neutral polygonal GLB, not exact smooth CAD',
            maximum_cell_diagonal_m=max(float(np.linalg.norm(np.array(c['high_m'])-c['low_m'])) for c in cells),
            dynamics='full-capacity geometry; runtime contents/free surface and slosh are separate',
            inferred_vessel_attachment='Theoretical inference: sealed inboard root bladder remains attached to the whole native wing tank module; not a structural fracture, seal-survival or OEM tank-layout validation',
            native_lump_discrepancy='Actual spatial cell centroids/bounds differ from unchanged native lumped tank points and inertia; full and body -Z half-fill offsets are reported, not corrected or calibrated'))


def validate_declaration(doc, design):
    expected = dict(schema=SCHEMA, design='rq4b', basis='theoretical_inference',
                    geometry='original_source_wing_root_strip_v1',
                    attachment='sealed_whole_native_tank_wing_module_v1',
                    tanks=[dict(index=0, section='left wing', surface='wing'),
                           dict(index=1, section='right wing', surface='wing')])
    if (design != 'rq4b' or not isinstance(doc, dict) or doc != expected or
            any(type(t.get('index')) is not int for t in doc.get('tanks', []))):
        raise liquid.LiquidError('Unsupported or malformed inferred liquid vessel declaration')
    return doc


def read_declaration(design_path, design):
    path = Path(design_path).with_suffix('.liquid-geometry-v1.json')
    if path.is_symlink(): raise liquid.LiquidError('Inferred vessel declaration must be an ordinary source file')
    if not path.exists(): return None
    if not path.is_file() or path.stat().st_size > 16384:
        raise liquid.LiquidError('Inferred vessel declaration exceeds bounded source contract')
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result: raise liquid.LiquidError('Duplicate inferred vessel declaration key')
            result[key] = value
        return result
    def invalid(_): raise liquid.LiquidError('Nonfinite inferred vessel declaration token')
    try:
        doc = json.loads(path.read_text(encoding='utf-8'), object_pairs_hook=pairs, parse_constant=invalid)
    except (ValueError, RecursionError) as refused:
        raise liquid.LiquidError('Invalid inferred vessel declaration JSON') from refused
    return validate_declaration(doc, design), path


def spatial_summary(cells, anchor, fraction):
    """Exact AABB first moments at full or reference-up hydrostatic contents.

    Reference up is body -Z, so liquid occupies z >= surface. This diagnostic
    does not replace the runtime's actual orientation/free-surface solution.
    """
    anchor = liquid._vec(anchor, 'native lump anchor')
    if (type(fraction) not in (int, float) or not math.isfinite(fraction) or not 0 < fraction <= 1 or not cells):
        raise liquid.LiquidError('Positive finite spatial-summary contents required')
    lows = np.array([liquid._vec(c['low_m'], 'cell low') for c in cells])
    highs = np.array([liquid._vec(c['high_m'], 'cell high') for c in cells])
    if np.any(highs <= lows): raise liquid.LiquidError('Nonpositive spatial-summary box')
    areas = (highs[:, 0]-lows[:, 0])*(highs[:, 1]-lows[:, 1])
    total = math.fsum((areas*(highs[:, 2]-lows[:, 2])).tolist()); wanted = total*fraction
    lower, upper = float(lows[:, 2].min()), float(highs[:, 2].max())
    surface = lower
    if fraction < 1:
        for _ in range(96):
            middle = (lower+upper)*.5
            volume = math.fsum((areas*np.maximum(0., highs[:, 2]-np.maximum(lows[:, 2], middle))).tolist())
            if volume >= wanted: lower = middle
            else: upper = middle
        surface = (lower+upper)*.5
    wetlow = lows.copy(); wetlow[:, 2] = np.maximum(wetlow[:, 2], surface)
    wet = highs[:, 2] > wetlow[:, 2]; wetlow, wethigh = wetlow[wet], highs[wet]
    weights = np.prod(wethigh-wetlow, axis=1); volume = math.fsum(weights.tolist())
    if not math.isfinite(volume) or volume <= 0: raise liquid.LiquidError('Unresolved reference hydrostatic volume')
    centres = (wetlow+wethigh)*.5
    centroid = np.array([math.fsum((weights*centres[:, a]).tolist())/volume for a in range(3)])
    offset = centroid-anchor
    return dict(volume_m3=volume, centroid_body_m=centroid.tolist(), offset_from_native_lump_m=offset.tolist(),
                offset_norm_m=float(np.linalg.norm(offset)), low_m=wetlow.min(axis=0).tolist(),
                high_m=wethigh.max(axis=0).tolist(), surface_body_z_m=surface,
                reference_up_body=[0., 0., -1.], contents_fraction=fraction)


class Policy:
    def __init__(self, declaration, classifier, root_solid, tanks, *, source_receipt=None):
        self.declaration = validate_declaration(declaration, 'rq4b')
        self.classifier = classifier
        if (not math.isfinite(classifier.y_root) or classifier.y_root <= 0 or
                not np.isfinite(classifier.origin).all()):
            raise liquid.LiquidError('Invalid original structural root cut or origin')
        self.root = liquid._valid(root_solid, 'Original source wing root strip', nonempty=True)
        self.bounds = np.array(self.root.bounding_box())
        self.tanks = {t['index']: t for t in tanks}; self.selected = {t['index'] for t in declaration['tanks']}
        for row in declaration['tanks']:
            if row['index'] not in self.tanks or self.tanks[row['index']]['section'] != row['section']:
                raise liquid.LiquidError('Inferred vessel native tank ownership mismatch')
        self.source_receipt = source_receipt or {}

    def assignment_order(self, indices):
        # Restricted owners must close before a flexible vessel consumes their
        # common ordinary region. No outboard tank may borrow root volume.
        return sorted(indices, key=lambda i: (i in self.selected, i))

    def certificate(self, index, lo, hi):
        return self._certificate(index, lo, hi, self.classifier.possible(lo, hi))

    def _certificates(self, lo, hi):
        possible = self.classifier.possible(lo, hi)
        return tuple(self._certificate(i, lo, hi, possible) for i in range(4))

    def _certificate(self, index, lo, hi, possible):
        owner = self.tanks[index]['section']
        ordinary = 1 if possible == frozenset({owner}) else 0 if owner in possible else -1
        if ordinary == 1 or index not in self.selected: return ordinary
        # Side/strip limits are exact, with no chord/thickness enlargement.
        origin_y = self.classifier.origin[1]; cut = self.classifier.y_root
        lower, upper = (-cut-origin_y, -origin_y) if index == 0 else (-origin_y, cut-origin_y)
        if (hi[1] <= lower or lo[1] >= upper or
                np.any(hi <= self.bounds[:3]) or np.any(lo >= self.bounds[3:])): return ordinary
        if lo[1] < lower or hi[1] > upper: return max(ordinary, 0)
        box = liquid._box(lo, hi)
        if liquid._valid(box-self.root, 'Inferred root cell/source wing difference').is_empty(): return 1
        if liquid._valid(box ^ self.root, 'Inferred root cell/source wing intersection').is_empty(): return ordinary
        return max(ordinary, 0)

    def eligibility(self, index, lo, hi):
        if self.classifier.possible(lo, hi) == frozenset({self.tanks[index]['section']}): return ORDINARY
        if index in self.selected and self.certificate(index, lo, hi) == 1: return ROOT
        raise liquid.LiquidError('Emitted slab lacks per-tank vessel eligibility')

    def region(self, index, lo, hi):
        ordinary = self.classifier.candidate_region(self.tanks[index]['section'], lo, hi)
        if index not in self.selected: return ordinary
        lower = np.maximum(lo, self.bounds[:3]); upper = np.minimum(hi, self.bounds[3:])
        origin_y = self.classifier.origin[1]; cut = self.classifier.y_root
        lower[1] = max(lower[1], -cut-origin_y if index == 0 else -origin_y)
        upper[1] = min(upper[1], -origin_y if index == 0 else cut-origin_y)
        if np.any(upper <= lower): return ordinary
        root = liquid._valid(self.root ^ liquid._box(lower, upper), 'Per-tank root half-strip')
        return liquid._valid(ordinary+root, 'Ordinary/root vessel candidate union')

    def receipt(self, cells):
        rows = []
        for index in sorted(self.selected):
            owned = [c for c in cells if c['tank'] == index]
            root = math.fsum(c['volume_m3'] for c in owned if c['eligibility_v1'] == ROOT)
            ordinary = math.fsum(c['volume_m3'] for c in owned if c['eligibility_v1'] == ORDINARY)
            rows.append(dict(index=index, section=self.tanks[index]['section'], ordinary_m3=ordinary, root_m3=root,
                             full=spatial_summary(owned, self.tanks[index]['anchor_body_m'], 1.),
                             half_reference_up=spatial_summary(owned, self.tanks[index]['anchor_body_m'], .5)))
        return dict(declaration=self.declaration, source_wing=self.source_receipt,
                    structural_root_cut_design_y_m=self.classifier.y_root,
                    attachment='sealed_whole_native_tank_wing_module_v1',
                    centroid_scope='Actual emitted cells; half contents at body -Z reference up only; native point masses unchanged',
                    tanks=rows)


def create(aircraft, origin, tanks, declaration):
    """Mesh the ORIGINAL uninflated source wing, clipped to its root strip.

    Same existing wing primitive/foil tables/mesher resolution, without body's
    smoothing, root extension, skin inflation, or a capacity-derived parameter.
    Every emitted slab additionally remains inside the original delivered hull.
    """
    from .shape import airframe as shape, meshkit
    from .geometry import airfoil, surface
    doc, path = declaration; validate_declaration(doc, aircraft.name)
    classifier = liquid._OwnerClassifier(aircraft, origin)
    wings = [s for s in aircraft.surfaces if s.name == 'wing' and s.kind == 'wing']
    if len(wings) != 1 or not wings[0].mirror or wings[0].sections[0].le[1] != 0.:
        raise liquid.LiquidError('Inferred vessel requires the original single mirrored centreline wing')
    wing_spec = wings[0].spec
    if (wing_spec.get('airfoil') != 'lrn1015.dat' or
            any(r.get('airfoil', 'lrn1015.dat') != 'lrn1015.dat' for r in wing_spec['sections'])):
        raise liquid.LiquidError('Inferred vessel requires the declared original LRN coordinate source')
    foil_path = Path(aircraft.path).parent/'lrn1015.dat'
    if foil_path.is_symlink() or not foil_path.is_file():
        raise liquid.LiquidError('Original inferred-vessel airfoil coordinates are absent or indirect')
    foil_sha = hashlib.sha256(foil_path.read_bytes()).hexdigest(); current_foil = airfoil.read_dat(foil_path)
    for section in wings[0].sections:
        if any(not np.array_equal(getattr(section.airfoil, key), getattr(current_foil, key)) for key in ('_x', '_yc', '_yt')):
            raise liquid.LiquidError('Loaded inferred-vessel airfoil differs from actual coordinate bytes')
    foil = shape.Foils(); wing = shape.union(shape.segments(wings[0], foil, inflate=0.))
    low, high = map(lambda v: np.asarray(v, float), aircraft.extent())
    low[1], high[1] = -classifier.y_root, classifier.y_root
    box = dict(prim='box', material=shape.SKIN, centre=(low+high)*.5, half=(high-low)*.5,
               axes=np.eye(3), mirror=False)
    h = shape.cell_size(aircraft)
    scene = dict(cell=h, error=.25*h, safety=3., sharp_deg=48.,
                 max_triangles=int(getattr(aircraft, 'triangles', None) or 75000), foils=foil.entries,
                 root=dict(op='intersect', k=0., children=[wing, box]))
    mesh = meshkit.build(scene)
    if any(mesh[k] for k in ('boundary_edges', 'nonmanifold_edges', 'misoriented_edges', 'degenerate_triangles')):
        raise liquid.LiquidError('Original source wing root meshing is not closed and oriented')
    vertices = (np.asarray(mesh['positions'], float)-origin)*[-1., 1., -1.]
    unique, inverse = np.unique(vertices, axis=0, return_inverse=True)
    triangles = np.ascontiguousarray(inverse[mesh['triangles']], dtype=np.uint64)
    solid = liquid._valid(liquid._mf().Manifold(liquid._mf().Mesh64(np.ascontiguousarray(unique), triangles)),
                          'Original uninflated source wing root mesh', nonempty=True)
    receipt = dict(method='original_uninflated_wing_primitive_root_clip_v1', surface='wing',
                   cell_m=h, error_m=.25*h, vertices=len(unique), triangles=len(triangles),
                   vertices_sha256=hashlib.sha256(unique.astype('<f8').tobytes()).hexdigest(),
                   triangles_sha256=hashlib.sha256(triangles.astype('<u8').tobytes()).hexdigest(),
                   volume_m3=solid.volume(), skin_inflation_m=0., root_extension_m=0.)
    policy = Policy(doc, classifier, solid, tanks, source_receipt=receipt)
    sources = [liquid._source('inferred_vessel_declaration', path), liquid._source('inferred_vessel_method', __file__),
               liquid._source('inferred_vessel_geometry', shape.__file__), liquid._source('inferred_vessel_mesher', meshkit.__file__),
               liquid._source('inferred_vessel_section_method', surface.__file__),
               liquid._source('inferred_vessel_airfoil_method', airfoil.__file__),
               liquid._source('inferred_vessel_airfoil_coordinates', foil_path),
               liquid._source('inferred_vessel_mesher_binary', meshkit.library()._name)]
    if next(r['sha256'] for r in sources if r['role'] == 'inferred_vessel_airfoil_coordinates') != foil_sha:
        raise liquid.LiquidError('Original inferred-vessel airfoil changed during source-wing construction')
    return policy, sources
