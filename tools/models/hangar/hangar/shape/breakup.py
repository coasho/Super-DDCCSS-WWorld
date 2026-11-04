"""The sections an airframe breaks into (ADR-30, MS-7e1, for MS-7g's kills and MS-7h's breakup): the left and right
wings, the tail, the nose and the centre - by one rule for every fixed-wing design, from its own geometry:

- a wing: what lies in a `wing` surface's planform (its chord band at its span station, near its plane) outboard of
  the fuselage at the wing's root - its side's;
- the tail: what lies in a tail surface's planform (`htail`, `fin`, `vtail`), and the fuselage aft of the tail's
  roots (some 0.3 m ahead of them);
- the nose: the fuselage ahead of the wing's root (0.6 m ahead of its leading edge), a canard's surfaces with it;
- the centre: everything else - the fuselage between, its intakes, nacelles and pods, the strakes.

The airframe's mesh is split triangle by triangle (by each one's centre), so the sections together are the airframe
as it was, joined without a crack; each is closed over its break by triangulating its complete boundary contours -
in the dark of a duct, the torn inside drawn as nothing more. Each section is a node (fsim:section:<name>) at its
break (where a tumbling piece turns about), with what is mounted on it below it: its control surfaces, gear, pylons,
nozzles. A design without a wing (a rotorcraft) has none."""
import numpy as np

SECTIONS = ("nose", "centre", "left wing", "right wing", "tail")
PLANE = 0.15         # m from a surface's plane that is still in it (its thickness and fillets), plus a tenth of the chord
CHORD_MARGIN = 0.05  # m ahead of and behind a surface's chord band
TAIL_AHEAD = 0.3     # m: the tail's break, ahead of its surfaces' roots
NOSE_AHEAD = 0.6     # m: the nose's break, ahead of the wing root's leading edge


TIP_REACH = 0.5  # m: a mounted part past a surface's tip still its (a wingtip rail and its missile)


def _in_planform(surface, p, below=False):
    """Which of the points p (n, 3; design frame) lie in the surface's planform: within its chord band at their span
    station along its sections, near its plane - its mirror image's too when it is mirrored. below: or anywhere under
    or over it (what hangs from a wing: its pylons)."""
    q = p.copy()
    if surface.mirror:
        q[:, 1] = np.abs(q[:, 1])
    inside = np.zeros(len(p), bool)
    last = len(surface.sections) - 2
    for k, (a, b) in enumerate(zip(surface.sections[:-1], surface.sections[1:])):
        la, lb = np.asarray(a.le, float), np.asarray(b.le, float)
        span = np.array([0.0, lb[1] - la[1], lb[2] - la[2]])
        length = np.linalg.norm(span)
        if length < 1e-9:
            continue
        s = span / length
        n = np.cross([1.0, 0.0, 0.0], s)
        t = ((q - la) @ s) / length
        # (what hangs from it reaches past its tip too: a wingtip's launch rail, TIP_REACH beyond)
        ok = (t >= -0.02) & (t <= (1.0 + TIP_REACH / length if below and k == last else 1.02))
        tc = np.clip(t, 0.0, 1.0)
        le_x = la[0] + tc * (lb[0] - la[0])
        chord = a.chord + tc * (b.chord - a.chord)
        plane = np.abs((q - la) @ n - tc * ((lb - la) @ n))
        ok &= (q[:, 0] >= le_x - CHORD_MARGIN) & (q[:, 0] <= le_x + chord + CHORD_MARGIN)
        if not below:
            ok &= plane <= PLANE + 0.1 * chord
        inside |= ok
    return inside


def _fuselage_half_width(aircraft, x):
    w = [0.5 * float(np.interp(x, b.x, b.w)) for b in aircraft.bodies if b.kind == "fuselage" and b.x[0] <= x <= b.x[-1]]
    return max(w) if w else 0.0


def classify(aircraft, points, mounted=False):
    """Each point's section (an index into SECTIONS), or None for a design without a wing. mounted: the points are
    where parts are mounted (a control surface's hinge, a pylon's middle) - a wing's own anywhere under or over it."""
    wings = [s for s in aircraft.surfaces if s.kind == "wing"]
    if not wings:
        return None
    tails = [s for s in aircraft.surfaces if s.kind in ("htail", "fin", "vtail")]
    canards = [s for s in aircraft.surfaces if s.kind == "canard"]
    root = wings[0].sections[0]
    y_root = _fuselage_half_width(aircraft, root.le[0] + 0.5 * root.chord) + 0.05
    x_nose = min(s.sections[0].le[0] for s in wings) - NOSE_AHEAD
    x_tail = min(s.sections[0].le[0] for s in tails) - TAIL_AHEAD if tails else np.inf
    out = np.full(len(points), SECTIONS.index("centre"))
    out[points[:, 0] < x_nose] = SECTIONS.index("nose")
    out[points[:, 0] > x_tail] = SECTIONS.index("tail")
    for s in canards:
        out[_in_planform(s, points)] = SECTIONS.index("nose")
    for s in wings:
        w = _in_planform(s, points, below=mounted) & (np.abs(points[:, 1]) > y_root)
        out[w & (points[:, 1] < 0)] = SECTIONS.index("left wing")
        out[w & (points[:, 1] > 0)] = SECTIONS.index("right wing")
    for s in tails:
        out[_in_planform(s, points)] = SECTIONS.index("tail")
    return out


def caps(positions, triangles):
    """Close every oriented boundary halfedge, including touching fans/rings.

    Traverse through incident faces, not a global visited-vertex set: several
    distinct boundary loops may legitimately touch at one welded vertex. Cap
    triangulation preserves original endpoints. Nested projected contours
    are triangulated together so a duct's inner ring remains a hole. These are
    inferred breakup surfaces, not changes to the actual exterior skin.
    """
    from collections import Counter
    import manifold3d as manifold
    # (on the welded mesh: a painted mesh's vertices are split along its texture's seams - edges there are no break)
    _, canon = np.unique(positions, axis=0, return_inverse=True)
    canon = canon.reshape(-1)
    welded = canon[triangles]
    first = {}
    for i, c in enumerate(canon):
        first.setdefault(int(c), i)
    positions = positions[[first[k] for k in range(len(first))]]
    triangles = welded
    edges = Counter()
    following = {}
    for a, b, c in triangles:
        for u, v in ((a, b), (b, c), (c, a)):
            edges[(u, v)] += 1
        following[(a, b)] = (b, c)
        following[(b, c)] = (c, a)
        following[(c, a)] = (a, b)
    if any(n != 1 for n in edges.values()):
        raise ValueError('Breakup cap input has a repeated oriented halfedge')
    boundary = [(u, v) for (u, v) in edges if (v, u) not in edges]
    if not boundary:
        return np.zeros((0, 3)), np.zeros((0, 3), int), None
    boundary_set = set(boundary)

    def successor(edge):
        # Rotate through this vertex's actual incident face fan until its other
        # boundary is reached; another fan sharing the vertex remains separate.
        candidate = following[edge]
        for _ in range(len(edges)):
            reverse = (candidate[1], candidate[0])
            if reverse not in edges:
                return candidate
            candidate = following[reverse]
        raise ValueError('Breakup boundary fan has no boundary successor')

    loops = []
    used = set()
    for start in boundary:
        if start in used:
            continue
        loop, edge = [], start
        while edge not in used:
            if edge not in boundary_set:
                raise ValueError('Breakup cap traversal left its boundary')
            used.add(edge); loop.append(edge[0]); edge = successor(edge)
        if edge != start or len(loop) < 3:
            raise ValueError('Breakup cap boundary does not form a complete cycle')
        # Opposite winding closes the skin's directed boundary.
        loops.append(positions[loop[::-1]])
    if used != boundary_set:
        raise ValueError('Breakup cap missed a boundary halfedge')

    records = []
    for points in loops:
        area_normal = np.cross(points, np.roll(points, -1, axis=0)).sum(axis=0)
        axis = int(np.argmax(np.abs(area_normal)))
        axes = [(axis+1) % 3, (axis+2) % 3]
        contour = points[:, axes]
        area = .5*area_normal[axis]
        if area == 0.:
            raise ValueError('Breakup boundary has zero projected area')
        region = manifold.CrossSection([contour], manifold.FillRule.NonZero)
        records.append(dict(points=points, axis=axis, contour=contour, area=area, region=region,
                            low=float(points[:, axis].min()), high=float(points[:, axis].max())))

    # A projected containment may be a duct only when its cap-normal extents
    # overlap and its winding opposes the enclosing contour. Parallel openings
    # at distinct stations cannot be bridged just because their projections nest.
    parents = [None]*len(records)
    for i, inner in enumerate(records):
        candidates = []
        for j, outer in enumerate(records):
            if (inner['axis'] != outer['axis'] or inner['area']*outer['area'] >= 0 or
                    abs(inner['area']) >= abs(outer['area']) or
                    max(inner['low'], outer['low']) > min(inner['high'], outer['high'])):
                continue
            if (inner['region']-outer['region']).is_empty():
                candidates.append(j)
        if candidates: parents[i] = min(candidates, key=lambda j: (abs(records[j]['area']), j))

    groups = {}
    for i in range(len(records)):
        root = i
        while parents[root] is not None: root = parents[root]
        groups.setdefault(root, []).append(i)
    vertices = []; faces = []; face_groups = []; group_axes = []
    for root, members in groups.items():
        sign = 1 if records[root]['area'] > 0 else -1
        points = [records[i]['points'] if sign > 0 else records[i]['points'][::-1] for i in members]
        axis = records[root]['axis']; axes = [(axis+1) % 3, (axis+2) % 3]
        contours = [np.ascontiguousarray(p[:, axes]) for p in points]
        triangles = np.asarray(manifold.triangulate(contours, allow_convex=False), dtype=np.int64)
        if sign < 0: triangles = triangles[:, [0, 2, 1]]
        base = len(vertices); vertices.extend(np.concatenate(points).tolist())
        faces.extend((triangles+base).tolist())
        face_groups.extend([len(group_axes)]*len(triangles)); group_axes.append(axes)
    vertices = np.array(vertices); faces = np.array(faces, dtype=np.int64)

    # A valid polygon diagonal can already be an actual skin edge, or another
    # cap diagonal when distinct contour fans share two welded vertices. Reusing it would
    # create four incident faces after coordinate welding. Preserve the mature
    # triangulator's projected domain, but replace such an INTERNAL diagonal
    # using two interior Steiner points and an edge flip. Their tangential
    # positions lie in the middle half of the diagonal, making the flip's
    # quadrilateral convex. No skin point/triangle or cap boundary is changed.
    ids = {tuple(p): i for i, p in enumerate(positions)}
    cap_faces = np.array([ids[tuple(p)] for p in vertices])[faces].tolist()
    # A repeated contour point can produce an exactly zero-area bridge in the
    # 2-D triangulator. Its coincident endpoints add a canceling edge pair, not
    # a material surface; do not count that bridge as a third cap face.
    cap_faces = [face if len(set(face)) == 3 else None for face in cap_faces]
    cap_points = positions.tolist()
    forbidden = {tuple(sorted(edge)) for edge in edges}
    while True:
        incidence = {}
        for i, face in enumerate(cap_faces):
            if face is None: continue
            a, b, c = face
            for u, v in ((a, b), (b, c), (c, a)):
                key = tuple(sorted((u, v)))
                incidence.setdefault(key, []).append((i, u, v))
        replacement = None
        for key, entries in incidence.items():
            if key not in forbidden and len(entries) <= 2:
                continue
            for i, u, v in entries:
                pair = next(((j, x, y) for j, x, y in entries
                             if j != i and x == v and y == u and face_groups[j] == face_groups[i]), None)
                if pair is not None:
                    replacement = (i, pair[0], u, v); break
            if replacement is not None: break
        if replacement is None: break
        i, j, u, v = replacement
        a = next(k for k in cap_faces[i] if k not in (u, v))
        b = next(k for k in cap_faces[j] if k not in (u, v))
        pu, pv, pa, pb = np.asarray([cap_points[k] for k in (u, v, a, b)])
        axes = group_axes[face_groups[i]]; d = (pv-pu)[axes]; mid = .5*(pu+pv)
        denominator = max(abs(float(np.dot((pa-mid)[axes], d))), abs(float(np.dot((pb-mid)[axes], d))))
        weight = min(1./3., float(np.dot(d, d))/(4*denominator)) if denominator else 1./3.
        if weight <= 0: raise ValueError('Breakup diagonal has zero projected extent')
        c1, c2 = len(cap_points), len(cap_points)+1
        cap_points += [((1-weight)*mid+weight*pa).tolist(), ((1-weight)*mid+weight*pb).tolist()]
        cap_faces[i] = cap_faces[j] = None
        cap_faces += [[v, a, c1], [a, u, c1], [u, b, c2], [b, v, c2], [u, c2, c1], [v, c1, c2]]
        face_groups += [face_groups[i]]*6
    mapped = np.array([face for face in cap_faces if face is not None], dtype=np.int64)

    # Verify the triangulator consumed every boundary edge once in reverse,
    # with paired opposite interior edges; do not accept a visually filled hole.
    cap_edges = Counter((u, v) for a, b, c in mapped for u, v in ((a, b), (b, c), (c, a)))
    for u, v in boundary:
        if cap_edges[(v, u)] != 1 or cap_edges[(u, v)] != 0:
            raise ValueError('Breakup triangulation did not preserve a boundary edge')
    for (u, v), n in cap_edges.items():
        if (v, u) not in boundary_set and (n != 1 or cap_edges[(v, u)] != 1):
            raise ValueError('Breakup triangulation has an unpaired interior edge')
        if (u, v) in edges and (v, u) in edges:
            raise ValueError('Breakup triangulation reuses an internal skin edge')
    used_vertices = np.unique(mapped)
    remap = np.full(len(cap_points), -1); remap[used_vertices] = np.arange(len(used_vertices))
    vertices = np.asarray(cap_points)[used_vertices]; faces = remap[mapped]
    centre = np.concatenate(loops).mean(axis=0)
    return vertices, faces, centre
