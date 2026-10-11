"""The stores' shapes, one for each class (docs/hangar.md, Stores): what no card gives.

A store's published figures are its card's (assets/stores/<id>.json); its shape is its class's, entered here - the Mk 82's
profile and fins, a tank's ogive, cylinder and cone, a pod's two half-spheres - in the design frame (x aft from the nose,
y right, z up). storedesigns.py writes each store's design from them (aircraft/<name>/<name>.toml), and the pipeline
builds that as it builds an aircraft: its model is model3d's, its build-up reads its design. What no source gives is
marked as estimated.

A cargo store's picture alone is drawn here (write_cargo: its pallet, its load and its straps - a box the mesher has
nothing to add to), by its model stage.
"""
import math
import os

import numpy as np

from . import model3d, pipeline

INCH = 0.0254

STORES = {
    # Mk 82 low-drag general-purpose bomb with the MAU-93 conical fin: length 2.21 m, diameter 273 mm (Wikipedia,
    # "Mark 82 bomb", 2026-09-29). Its CG 31 in aft of the nose: JSBSim's mk82.xml, the flight model it is drawn for.
    # The ogive nose's length, the boattail and the fins' size are estimated from photographs.
    "mk82": {
        "length": 2.21, "diameter": 0.273, "cg": 31 * INCH,
        "profile": [(0.00, 0.00), (0.10, 0.25), (0.25, 0.55), (0.45, 0.82), (0.65, 0.97), (0.80, 1.00), (1.70, 1.00), (1.85, 0.93),
                    (2.00, 0.72), (2.21, 0.55)],  # (station m, radius / max radius) - estimated
        "fins": {"count": 4, "roll_deg": 45.0, "root_le": 1.78, "root_chord": 0.43, "tip_chord": 0.30, "span": 0.13,
                 "sweep_deg": 20.0, "thickness": 0.006},  # estimated
        "colour": (0.30, 0.33, 0.24), "band": (0.85, 0.75, 0.15), "band_at": (0.55, 0.62),
    },
    # CJ-10A land-attack cruise missile as an H-6K carries it: length 6.4 m without its boost motor, diameter 0.75 m (the
    # Hong Niao family's, CSIS Missile Threat: its card). A body on its pylon and, released, an inert one - its wings
    # folded under it, its turbofan and its guidance not modelled. Its ogive nose, its boattail, its cruciform tail fins
    # and its CG at 45 % of its length are estimated (a cruise missile of the Tomahawk's layout).
    "cj10a": {
        "length": 6.4, "diameter": 0.75, "cg": 0.45 * 6.4,
        "profile": [(0.00, 0.00), (0.15, 0.30), (0.40, 0.62), (0.75, 0.88), (1.10, 1.00), (5.40, 1.00), (5.90, 0.90),
                    (6.40, 0.62)],  # (station m, radius / max radius) - estimated
        "fins": {"count": 4, "roll_deg": 45.0, "root_le": 5.45, "root_chord": 0.75, "tip_chord": 0.40, "span": 0.45,
                 "sweep_deg": 30.0, "thickness": 0.012},  # estimated
        "colour": (0.62, 0.64, 0.66), "band": None, "band_at": None,
    },
}


# An external fuel tank's shape from its card (its length and diameter): an ogive nose over its first quarter, the
# cylinder to 0.65 of its length, a cone to a small tail - the same for every tank (assumed from photographs), its CG
# at its middle; fins when its card says so.
TANK_PROFILE = [(0.00, 0.08), (0.04, 0.45), (0.10, 0.75), (0.17, 0.93), (0.25, 1.00), (0.65, 1.00), (0.80, 0.85),
                (0.92, 0.55), (1.00, 0.20)]
TANK_FINS = {"count": 3, "roll_deg": 90.0, "root_le": 0.80, "root_chord": 0.16, "tip_chord": 0.08, "span": 0.35,
             "sweep_deg": 40.0, "thickness": 0.01}  # (of its length, its span of its diameter: a finned tank's - assumed)


# A pod's shape from its card (its length and diameter - a boxy pod's the circle of its section's area): a half-sphere
# nose, the cylinder, a half-sphere tail - the same for every pod (assumed: targeting pods end in a turret and a blunt
# tail, jamming pods in two radomes). Its rounded tail is too steep for the air to follow at the end of a long body's
# thick boundary layer (an afterbody steeper than some 15 - 20 deg separates at its shoulder: Hoerner ch. 3, 20): a wake
# POD_WAKE of its diameter across, a blunt base's to its drag (an estimate; a sphere's 120 deg would make it 0.85).
POD_WAKE = 1.0


def pod_profile(L, D):
    R = 0.5 * D
    arc = [math.radians(a) for a in (0.0, 22.5, 45.0, 67.5, 90.0)]
    nose = [(R * (1.0 - math.cos(a)), math.sin(a)) for a in arc]
    return nose + [(L - x, r) for x, r in reversed(nose)]


def tank_cards():
    """Each tank's and pod's card (assets/stores), by the name it is drawn by (its id without hyphens)."""
    import json
    out = {}
    d = os.path.join(pipeline.repo_root(), "assets", "stores")
    for f in sorted(os.listdir(d)):
        if f.endswith(".json"):
            with open(os.path.join(d, f), encoding="utf-8") as h:
                c = json.load(h)
            if c.get("class") in ("tank", "pod"):
                out[c["id"].replace("-", "")] = c
    return out


def tank_spec(card):
    """A tank's or a pod's shape from its card."""
    L, D = card["parameters"]["length_m"]["value"], card["parameters"]["diameter_m"]["value"]
    if card.get("class") == "pod":
        return {"length": L, "diameter": D, "cg": 0.5 * L, "profile": pod_profile(L, D), "base": POD_WAKE, "colour": (0.42, 0.44, 0.46)}
    s = {"length": L, "diameter": D, "cg": 0.5 * L, "profile": [(x * L, r) for x, r in TANK_PROFILE],
         "colour": (0.62, 0.64, 0.65)}
    if card.get("fins"):
        f = dict(TANK_FINS)
        for k in ("root_le", "root_chord", "tip_chord"):
            f[k] *= L
        f["span"] *= D
        s["fins"] = f
    return s


def cargo_cards():
    """Each cargo store's card (assets/stores, class "cargo"), by the name it is drawn by (its id without hyphens)."""
    import json
    out = {}
    d = os.path.join(pipeline.repo_root(), "assets", "stores")
    for f in sorted(os.listdir(d)):
        if f.endswith(".json"):
            with open(os.path.join(d, f), encoding="utf-8") as h:
                c = json.load(h)
            if c.get("class") == "cargo":
                out[c["id"].replace("-", "")] = c
    return out


def _box(x0, x1, y0, y1, z0, z1):
    """A box's vertices and triangles (the design's frame: x aft, y right, z up), each face its own four vertices so
    that its edges stay sharp."""
    c = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0), (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    faces = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (2, 3, 7, 6), (1, 2, 6, 5), (3, 0, 4, 7)]  # outward, counter-clockwise
    v, t = [], []
    for f in faces:
        n = len(v)
        v += [c[i] for i in f]
        t += [(n, n + 1, n + 2), (n, n + 2, n + 3)]
    return np.array(v, float), np.array(t)


def write_cargo(name, card, path):
    """A cargo store for the viewer: its pallet or skid board, its load a block under it, and the straps of its net -
    a box of its card's size, as its flight model is, its origin at its middle."""
    p = card["parameters"]
    L, W, H = p["length_m"]["value"], p["width_m"]["value"], p["height_m"]["value"]
    B = model3d._Builder()
    base = B.material("pallet", (0.68, 0.69, 0.70), metallic=0.6, rough=0.45)   # aluminium skin, or a plywood skid board
    load = B.material("load", (0.36, 0.38, 0.26), metallic=0.0, rough=0.9)      # olive canvas
    strap = B.material("net", (0.16, 0.17, 0.13), metallic=0.0, rough=0.85)
    origin = np.array([0.5 * L, 0.0, 0.0])
    slab, inset, proud, band = 0.057, 0.04, 0.012, 0.05  # m: the pallet's thickness, the load's margin on it, the straps
    z0, z1 = -0.5 * H, 0.5 * H
    parts = []

    def add(box, material):
        v, t = box
        parts.append((model3d._to_gltf(v, origin), t, material))

    add(_box(0.0, L, -0.5 * W, 0.5 * W, z0, z0 + slab), base)
    add(_box(inset, L - inset, -0.5 * W + inset, 0.5 * W - inset, z0 + slab, z1 - proud), load)
    for f in (0.3, 0.7):  # two straps round it each way, standing proud of the load
        x = f * L
        add(_box(x - band / 2, x + band / 2, -0.5 * W + inset - proud, 0.5 * W - inset + proud, z0 + slab, z1), strap)
        y = (f - 0.5) * W
        add(_box(inset - proud, L - inset + proud, y - band / 2, y + band / 2, z0 + slab, z1), strap)
    mesh = B.mesh(name, parts)
    node = B.node(name + "-body", mesh=mesh)
    model3d.finish(B, [node], name, path)
    return path
