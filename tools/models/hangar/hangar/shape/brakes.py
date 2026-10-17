"""Drag devices drawn (docs/hangar.md, Drag devices): an airbrake's plates and a spoiler's panels as thin shells lying on
the skin where the design puts them, each hinged at its forward edge and opened by the speedbrake's position (the
viewer's fsim:speedbrake:<radians> node), over a dark patch - the recess it opens out of. A set of control surfaces
deflected as a speedbrake is drawn by those surfaces' own nodes (model3d.speedbrake_term).

An airbrake's plates are placed by its `place` (docs/hangar.md):

  dorsal    one plate on top of the skin at each of `stations` (y; default the centreline), opening up
  ventral   the same underneath, opening down
  split     a pair at each station, one on top opening up and one underneath opening down (a split tail, a wing's
            upper and lower plates, a deceleron)
  sides     a plate on either side of the fuselage at each of `stations` (z; default its position's), opening outwards

Without a place: dorsal above the centre of its position's height, ventral below, sides otherwise. The plates share
its area; each is `length` long (m, along x; default square), hinged at `hinge_x` (m) or where its position - their
centroid opened - puts the forward edge. Spoilers: the upper skin over their span and chord, in panels about PANEL_SPAN wide.
"""
import numpy as np

from .airframe import CONTROL, DARK, GAP

THICK = 0.015   # a plate's thickness (m)
LIFT = 0.006    # its underside off the skin, so the skin never shows through it shut
PATCH = 0.003   # the dark recess patch off the skin, under the plate
PANEL_SPAN = 2.0  # a spoiler panel's width (m) at most
PLACES = ("dorsal", "ventral", "split", "sides")
REACH = 8.0     # m: rays start this far out along their direction


def _hits(probe, starts, n, reach, step=0.03):
    """Where rays from starts (N, 3) running along -n first enter the solid: (N, 3), nan where one misses or starts
    inside."""
    ts = np.arange(0.0, reach + step, step)
    P = starts[:, None, :] - ts[None, :, None] * n
    d, _ = probe(P.reshape(-1, 3))
    d = d.reshape(len(starts), len(ts))
    out = np.full((len(starts), 3), np.nan)
    enter = (d[:, :-1] >= 0.0) & (d[:, 1:] < 0.0)
    for k in np.flatnonzero(enter.any(axis=1) & (d[:, 0] >= 0.0)):
        i = int(np.argmax(enter[k]))
        t = ts[i] + step * d[k, i] / (d[k, i] - d[k, i + 1])
        out[k] = starts[k] - t * n
    # refined: the distance field near the skin is a distance, so one step along -n by it lands on the skin
    ok = ~np.isnan(out[:, 0])
    if ok.any():
        for _ in range(3):
            dd, _ = probe(out[ok])
            out[ok] = out[ok] - dd[:, None] * n
    return out


def _shell(G, n, label):
    """A plate over the grid of skin points G (nu, nv, 3) facing out along n: a closed shell LIFT off the skin and THICK
    thick (its faces split where they turn), and the dark patch under it."""
    du = np.gradient(G, axis=0)
    dv = np.gradient(G, axis=1)
    N = np.cross(du, dv)
    N /= np.linalg.norm(N, axis=-1, keepdims=True)
    if np.sum(N * n) < 0.0:
        N = -N
    nu, nv = G.shape[:2]
    idx = np.arange(nu * nv).reshape(nu, nv)
    quads = np.stack([idx[:-1, :-1], idx[1:, :-1], idx[1:, 1:], idx[:-1, 1:]], axis=-1).reshape(-1, 4)
    tris = np.concatenate([quads[:, [0, 1, 2]], quads[:, [0, 2, 3]]])
    # the top's triangles face out
    a, b, c = (G.reshape(-1, 3)[tris[:, k]] for k in range(3))
    if np.sum(np.cross(b - a, c - a) * N.reshape(-1, 3)[tris[:, 0]]) < 0.0:
        tris = tris[:, [0, 2, 1]]
    top = (G + N * (LIFT + THICK)).reshape(-1, 3)
    bot = (G + N * LIFT).reshape(-1, 3)
    pos, nrm, tri = [top, bot], [N.reshape(-1, 3), -N.reshape(-1, 3)], [tris, tris[:, [0, 2, 1]] + nu * nv]
    count = 2 * nu * nv
    # the walls round its edge
    ring = list(idx[:, 0]) + list(idx[-1, 1:]) + list(idx[::-1, -1][1:]) + list(idx[0, ::-1][1:-1])
    centre = top.mean(axis=0)
    for i, j in zip(ring, ring[1:] + ring[:1]):
        q = np.array([top[i], top[j], bot[j], bot[i]])
        f = np.cross(q[1] - q[0], q[3] - q[0])
        out = 0.5 * (q[0] + q[1]) - centre
        if np.linalg.norm(f) < 1e-12:
            continue
        f /= np.linalg.norm(f)
        t = np.array([[0, 1, 2], [0, 2, 3]]) if np.dot(f, out) >= 0.0 else np.array([[0, 2, 1], [0, 3, 2]])
        if np.dot(f, out) < 0.0:
            f = -f
        pos.append(q)
        nrm.append(np.repeat(f[None], 4, axis=0))
        tri.append(t + count)
        count += 4
    tri = np.concatenate(tri).astype(np.int64)
    plate = {"positions": np.concatenate(pos), "normals": np.concatenate(nrm), "triangles": tri,
             "materials": np.full(len(tri), CONTROL, np.int32)}
    patch = {"positions": (G + N * PATCH).reshape(-1, 3), "normals": N.reshape(-1, 3), "triangles": tris.astype(np.int64),
             "materials": np.full(len(tris), DARK, np.int32)}
    return plate, patch


def _hinge_axis(p0, p1, centre, n):
    """The hinge line p0..p1 as a unit axis turning the plate's free edge (round centre) out along n for a positive
    (right-hand) turn."""
    d = (p1 - p0) / np.linalg.norm(p1 - p0)
    return d if np.dot(np.cross(d, centre - 0.5 * (p0 + p1)), n) > 0.0 else -d


def _plate(probe, x0, length, lat, width, lat_axis, n, label, nu=12, nv=10):
    """An airbrake plate: the skin over x0..x0+length by lat +- width/2 along lat_axis (1 y, 2 z), seen from out along
    n - or None where the skin there is not one smooth piece (it misses, or jumps to something else)."""
    us = np.linspace(x0, x0 + length, nu)
    vs = np.linspace(lat - 0.5 * width, lat + 0.5 * width, nv)
    U, V = np.meshgrid(us, vs, indexing="ij")
    starts = np.zeros((nu * nv, 3))
    starts[:, 0] = U.ravel()
    starts[:, lat_axis] = V.ravel()
    k = int(np.flatnonzero(np.abs(n) > 0.5)[0])
    starts[:, k] = n[k] * REACH
    G = _hits(probe, starts, n, 2.0 * REACH)
    if np.isnan(G).any():
        return None
    G = G.reshape(nu, nv, 3)
    h = G[..., k] * n[k]
    # one piece of skin: within a band round its middle, no step between neighbours
    if np.abs(h - np.median(h)).max() > 0.35 * width + 0.05:
        return None
    if max(np.abs(np.diff(h, axis=0)).max(), np.abs(np.diff(h, axis=1)).max()) > 0.12 + 0.6 * max(length / nu, width / nv):
        return None
    plate, patch = _shell(G, n, label)
    p0, p1 = G[0, 0], G[0, -1]
    return {"plate": plate, "patch": patch, "hinge": 0.5 * (p0 + p1), "axis": _hinge_axis(p0, p1, G.reshape(-1, 3).mean(axis=0), n),
            "label": label}


def _airbrake(device, probe, aircraft):
    """An airbrake's plates (dicts _plate makes)."""
    spec = device.spec
    P = device.position
    place = spec.get("place") or ("dorsal" if P[2] > 0.3 else "ventral" if P[2] < -0.3 else "sides")
    if place not in PLACES:
        raise ValueError("drag_device %r: place must be one of %s" % (device.name, ", ".join(PLACES)))
    up, down = np.array([0.0, 0.0, 1.0]), np.array([0.0, 0.0, -1.0])
    if place == "sides":
        stations = [float(z) for z in spec.get("stations", [P[2]])]
        where = [(z, 2, s * np.array([0.0, 1.0, 0.0])) for z in stations for s in (1.0, -1.0)]
    else:
        stations = [float(y) for y in spec.get("stations", [0.0])]
        dirs = {"dorsal": [up], "ventral": [down], "split": [up, down]}[place]
        where = [(y, 1, n) for y in stations for n in dirs]
    a = device.area / len(where)
    length = float(spec.get("length", np.sqrt(a)))
    out = []
    for i, (lat, axis, n) in enumerate(where):
        L, W = length, a / length
        # their centroid opened: the forward edge half its length ahead, turned by the opening
        for _ in range(16):
            x0 = float(spec["hinge_x"]) if "hinge_x" in spec else P[0] - 0.5 * L * np.cos(np.radians(device.max_deg))
            p = _plate(probe, x0, L, lat, W, axis, n, "%s %d" % (device.name, i + 1))
            if p is not None:
                break
            W *= 0.88  # narrower, and longer to keep its area (no more than three times as long as wide)
            L = min(a / W, 3.0 * W)
        if p is None:
            raise ValueError("drag_device %r: no smooth skin for plate %d at x %.2f, %s %.2f, facing %s - move its position "
                             "or set its place, stations or length" % (device.name, i + 1, P[0], "yz"[axis - 1], lat, n.tolist()))
        p["area"] = L * W
        out.append(p)
    return out


def _spoilers(device, probe):
    """A spoiler's panels: the wing's upper skin over its span and chord, both halves."""
    s = device.surface
    span_m = (device.eta1 - device.eta0) * s.half_arc
    count = max(1, int(np.ceil(span_m / PANEL_SPAN - 1e-9)))
    edges = np.linspace(device.eta0, device.eta1, count + 1)
    gap = 0.5 * GAP / s.half_arc
    out = []
    up = np.array([0.0, 0.0, 1.0])
    for side in ((1.0, -1.0) if s.mirror else (1.0,)):
        for i in range(count):
            etas = np.linspace(edges[i] + gap, edges[i + 1] - gap, 10)
            fracs = np.linspace(device.hinge, device.hinge + device.chord_fraction, 8)
            base = np.zeros((len(fracs), len(etas), 3))
            for j, e in enumerate(etas):
                le, chord, _, _ = s.station(e)
                c, _ = s.frame(e)
                base[:, j] = le[None] + c[None] * (fracs[:, None] * chord)
            base[..., 1] *= side
            starts = base.reshape(-1, 3).copy()
            starts[:, 2] += 3.0
            G = _hits(probe, starts, up, 6.0)
            if np.isnan(G).any():
                raise ValueError("drag_device %r: panel %d misses the wing's upper skin" % (device.name, i + 1))
            G = G.reshape(len(fracs), len(etas), 3)
            label = "%s %s %d" % (device.name, "right" if side > 0 else "left", i + 1)
            plate, patch = _shell(G, up, label)
            p0, p1 = G[0, 0], G[0, -1]
            out.append({"plate": plate, "patch": patch, "hinge": 0.5 * (p0 + p1),
                        "axis": _hinge_axis(p0, p1, G.reshape(-1, 3).mean(axis=0), up), "label": label})
    return out


def panels(aircraft, skin):
    """Every drag device's plates and panels: [{"device", "plate" (closed shell, CONTROL), "patch" (DARK), "hinge",
    "axis", "rad" (opened, radians), "label"}]; skin: the airframe's scene before its controls are cut (airframe())."""
    from ..drag import drag_devices
    from . import meshkit
    devices = [d for d in drag_devices(aircraft) if d.kind in ("airbrake", "spoiler")]
    if not devices:
        return []
    out = []
    with meshkit.Probe(skin) as probe:
        for d in devices:
            got = _airbrake(d, probe, aircraft) if d.kind == "airbrake" else _spoilers(d, probe)
            for p in got:
                p.update(device=d.name, rad=float(np.radians(d.max_deg)))
            out += got
    return out


#: which way each channel's surfaces move as a speedbrake: their trailing edges up, down, or out (twin rudders apart)
SPEEDBRAKE_MOTION = {"aileron": "up", "elevator": "up", "flap": "down", "rudder": "out"}


def speedbrake_term(aircraft, ctrl, axis, p0, te):
    """"+speedbrake:<radians>" for a control piece a 'surfaces' drag device deflects (its channel's degrees at the full
    speedbrake, the trailing edge moving as SPEEDBRAKE_MOTION has it), or "" - axis and p0 its node's hinge, te its
    trailing edge."""
    from ..drag import drag_devices
    deg = 0.0
    for d in drag_devices(aircraft):
        if d.kind == "surfaces" and ctrl.channel in d.channels:
            deg = d.channels[ctrl.channel]
    if not deg:
        return ""
    motion = SPEEDBRAKE_MOTION.get(ctrl.channel)
    if motion == "out":
        if abs(te[1]) < 0.1:  # a single fin's rudder: nothing to move against
            return ""
        want = np.array([0.0, np.sign(te[1]), 0.0])
    elif motion in ("up", "down"):
        want = np.array([0.0, 0.0, 1.0 if motion == "up" else -1.0])
    else:
        return ""
    sign = 1.0 if np.dot(np.cross(axis, np.asarray(te, float) - np.asarray(p0, float)), want) > 0.0 else -1.0
    return "+speedbrake:%.6g" % (sign * np.radians(deg))
