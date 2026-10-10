"""The sections an airframe breaks into (ADR-30, MS-7e1, for MS-7g's kills and MS-7h's breakup): the left and right
wings, the tail, the nose and the centre - by one rule for every fixed-wing design, from its own geometry:

- a wing: what lies in a `wing` surface's planform (its chord band at its span station, near its plane) outboard of
  the fuselage at the wing's root - its side's;
- the tail: what lies in a tail surface's planform (`htail`, `fin`, `vtail`), and the fuselage aft of the tail's
  roots (some 0.3 m ahead of them);
- the nose: the fuselage ahead of the wing's root (0.6 m ahead of its leading edge), a canard's surfaces with it;
- the centre: everything else - the fuselage between, its intakes, nacelles and pods, the strakes.

The airframe's mesh is split triangle by triangle (by each one's centre), so the sections together are the airframe
as it was, joined without a crack; each is closed over its break by a cap - a fan from each of its torn edges' loops -
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
    """A section's caps over its torn edges: each loop of its boundary edges (those of one triangle only) fanned from
    its centre, wound to close the section. (vertices, triangles, the loops' centre - the break - or None without
    a boundary)."""
    from collections import defaultdict
    # (on the welded mesh: a painted mesh's vertices are split along its texture's seams - edges there are no break)
    _, canon = np.unique(positions, axis=0, return_inverse=True)
    canon = canon.reshape(-1)
    welded = canon[triangles]
    first = {}
    for i, c in enumerate(canon):
        first.setdefault(int(c), i)
    positions = positions[[first[k] for k in range(len(first))]]
    triangles = welded
    edges = defaultdict(int)
    for a, b, c in triangles:
        for u, v in ((a, b), (b, c), (c, a)):
            edges[(u, v)] += 1
    boundary = [(u, v) for (u, v) in edges if (v, u) not in edges]
    if not boundary:
        return np.zeros((0, 3)), np.zeros((0, 3), int), None
    nxt = {}
    for u, v in boundary:
        nxt.setdefault(u, []).append(v)
    verts, tris, centres = [], [], []
    used = set()
    for start in [u for u, _ in boundary]:
        if start in used:
            continue
        loop, u = [], start
        while u not in used and u in nxt:
            used.add(u)
            loop.append(u)
            u = nxt[u][0]
        if len(loop) < 3:
            continue
        pts = positions[loop]
        c = pts.mean(axis=0)
        base = len(verts)
        verts.extend(pts.tolist())
        verts.append(c.tolist())
        k = len(loop)
        for i in range(k):  # (the boundary runs u -> v in the section's own triangles: the cap runs v -> u)
            tris.append((base + (i + 1) % k, base + i, base + k))
        centres.append((c, k))
    if not tris:
        return np.zeros((0, 3)), np.zeros((0, 3), int), None
    w = np.array([k for _, k in centres], float)
    centre = (np.array([c for c, _ in centres]) * w[:, None]).sum(axis=0) / w.sum()
    return np.array(verts), np.array(tris), centre
