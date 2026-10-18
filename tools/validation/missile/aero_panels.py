"""Inspect the solid-panel prefix of the pinned VSPGEOM v3 format.

No vertex welding, face repair or geometry substitution is performed. Wake,
UV and trailing metadata are retained by the solver and are outside this
prefix inspector; lifting-wake checks remain a separate solver-log gate.
"""
from collections import Counter, defaultdict
import hashlib
import math
from pathlib import Path

from aero_mesh import cross, subtract


def _simple_polygon(points, vector):
    """Reject zero edges and crossings in the dominant normal projection.

    Native curved-surface NGons need not be perfectly planar. This is a local
    polygon check, not a proof against intersections between different panels.
    """
    axis = max(range(3), key=lambda i: abs(vector[i]))
    planar = [tuple(v for i, v in enumerate(p) if i != axis) for p in points]

    def orientation(a, b, c):
        return math.fsum(((b[0]-a[0])*(c[1]-a[1]), -(b[1]-a[1])*(c[0]-a[0])))

    def contains(a, b, p):
        return all(min(a[i], b[i]) <= p[i] <= max(a[i], b[i]) for i in (0, 1))

    size = len(points)
    for i, a in enumerate(planar):
        b = planar[(i+1) % size]
        if points[i] == points[(i+1) % size] or a == b:
            raise ValueError("zero-length VSPGEOM polygon edge")
        for j in range(i+1, size):
            if j == i+1 or (i == 0 and j == size-1):
                continue
            c, d = planar[j], planar[(j+1) % size]
            ab_c, ab_d, cd_a, cd_b = orientation(a, b, c), orientation(a, b, d), orientation(c, d, a), orientation(c, d, b)
            proper = (ab_c != 0 and ab_d != 0 and cd_a != 0 and cd_b != 0
                      and (ab_c > 0) != (ab_d > 0) and (cd_a > 0) != (cd_b > 0))
            touching = ((ab_c == 0 and contains(a, b, c)) or (ab_d == 0 and contains(a, b, d))
                        or (cd_a == 0 and contains(c, d, a)) or (cd_b == 0 and contains(c, d, b)))
            if proper or touching:
                raise ValueError("self-crossing VSPGEOM polygon")


def read_topology(path):
    raw = Path(path).read_bytes()
    lines = iter(raw.decode("ascii").splitlines())
    try:
        if next(lines).strip() != "# vspgeom v3" or next(lines).strip() != "1":
            raise ValueError("expected one VSPGEOM v3 mesh")
        counts = [int(v) for v in next(lines).split()]
        if len(counts) != 3 or counts[0] < 4 or counts[1] < 1 or counts[2] < 0:
            raise ValueError("invalid VSPGEOM mesh counts")
        count, panel_count, wakes = counts
        points = []
        for _ in range(count):
            point = tuple(float(v) for v in next(lines).split())
            if len(point) != 3 or not all(math.isfinite(v) for v in point):
                raise ValueError("invalid VSPGEOM coordinate")
            points.append(point)
        if int(next(lines).strip()) != panel_count:
            raise ValueError("VSPGEOM polygon counts disagree")
        faces = []
        for _ in range(panel_count):
            row = [int(v) for v in next(lines).split()]
            face = tuple(v-1 for v in row[1:])
            if (not row or row[0] < 3 or len(face) != row[0] or len(set(face)) != len(face)
                    or any(v < 0 or v >= count for v in face)):
                raise ValueError("invalid VSPGEOM polygon connectivity")
            faces.append(face)
    except (StopIteration, UnicodeError, OverflowError) as error:
        raise ValueError("incomplete or unsupported VSPGEOM solid prefix") from error

    edges, vectors, volumes, areas = defaultdict(list), [], [], []
    for index, face in enumerate(faces):
        for a, b in zip(face, face[1:]+face[:1]):
            edges[tuple(sorted((a, b)))].append((index, a, b))
        a = points[face[0]]
        triangles = [cross(subtract(points[face[i]], a), subtract(points[face[i+1]], a))
                     for i in range(1, len(face)-1)]
        vector = tuple(math.fsum(v[axis]/2 for v in triangles) for axis in range(3))
        if not all(math.isfinite(v) for v in vector) or math.hypot(*vector) == 0:
            raise ValueError("degenerate VSPGEOM polygon")
        _simple_polygon([points[v] for v in face], vector)
        vectors.append(vector)
        areas.append(math.fsum(math.hypot(*v)/2 for v in triangles))
        volumes.append(math.fsum(sum(a[k]*v[k] for k in range(3))/6 for v in triangles))
    neighbours = [set() for _ in faces]
    for owners in edges.values():
        for owner in owners:
            neighbours[owner[0]].update(other[0] for other in owners if other[0] != owner[0])
    components, remaining = 0, set(range(len(faces)))
    while remaining:
        components += 1
        pending = [remaining.pop()]
        while pending:
            new = neighbours[pending.pop()] & remaining
            remaining.difference_update(new)
            pending.extend(new)
    uses = Counter(len(v) for v in edges.values())
    same = sum(len(v) == 2 and v[0][1:] == v[1][1:] for v in edges.values())
    volume = math.fsum(volumes)
    vector = [math.fsum(v[axis] for v in vectors) for axis in range(3)]
    area = math.fsum(areas)
    if not all(math.isfinite(v) for v in (volume, area, *vector)):
        raise ValueError("nonfinite VSPGEOM surface integral")
    # An edge-paired shell can still be inward, disconnected or inconsistently
    # directed. Area closure is a scale-relative roundoff check, not a hole fix.
    closed = (set(uses) == {2} and same == 0 and components == 1 and volume > 0
              and math.hypot(*vector) <= area * 1e-10)
    return {"format": "VSPGEOM v3 solid prefix", "sha256": hashlib.sha256(raw).hexdigest(),
            "nodes": count, "polygons": panel_count, "wake_lists": wakes,
            "edge_use_counts": {str(k): v for k, v in sorted(uses.items())},
            "boundary_edges": uses.get(1, 0), "same_direction_double_edges": same,
            "components": components, "signed_volume_m3": volume, "surface_area_m2": area,
            "area_vector_norm_m2": math.hypot(*vector), "closed_oriented": closed}
