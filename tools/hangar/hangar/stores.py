"""Stores' models for the viewer (ADR-30: weapons in flight are entities the viewer draws).

    python -m hangar store <name>     writes assets/models/<name>.glb (the viewer's model for jsbsim:<name>)
    python -m hangar store            every one: the STORES below and each tank's and pod's card (assets/stores)

A store is a body of revolution - its radius along its length - with a ring of
fins, built in the design frame (x aft from the nose, y right, z up) and
written the way model3d writes an aircraft: origin at the CG, forward -y and up
+z after the viewer's import. Each store's numbers cite their source; what no
source gives is marked as estimated, and only shapes the picture.
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
}


# An external fuel tank's shape from its card (its length and diameter): an ogive nose over its first quarter, the
# cylinder to 0.65 of its length, a cone to a small tail - the same for every tank (assumed from photographs: the
# shape only shapes the picture; its flight model is storemodels.py's), its CG at its middle; no fins (its card's).
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


def _fin(spec, radius, angle):
    """One fin's plate: its root on the body at `radius`, out along the direction at `angle` (rad, from +y towards +z)."""
    f = spec
    out = np.array([0.0, math.cos(angle), math.sin(angle)])
    side = np.cross([1.0, 0.0, 0.0], out)
    le = f["root_le"]
    sweep = math.tan(math.radians(f["sweep_deg"])) * f["span"]
    root = [(le, radius), (le + f["root_chord"], radius)]
    tip = [(le + sweep + f["tip_chord"], radius + f["span"]), (le + sweep, radius + f["span"])]
    outline = root + tip
    v = []
    for s in (-0.5, 0.5):
        for x, r in outline:
            v.append(np.array([x, 0.0, 0.0]) + r * out + s * f["thickness"] * side)
    v = np.array(v)
    t = [(0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6),  # the two faces
         (0, 4, 5), (0, 5, 1), (1, 5, 6), (1, 6, 2), (2, 6, 7), (2, 7, 3), (3, 7, 4), (3, 4, 0)]
    return v, np.array(t)


def write_store(name, path=None):
    s = STORES[name] if name in STORES else tank_spec(tank_cards()[name])
    path = path or os.path.join(pipeline.repo_root(), "assets", "models", name + ".glb")
    B = model3d._Builder()
    body = B.material("body", s["colour"], metallic=0.2, rough=0.55)
    band = B.material("band", s["band"], metallic=0.1, rough=0.6) if "band" in s else None
    origin = np.array([s["cg"], 0.0, 0.0])
    st = np.array([p[0] for p in s["profile"]])
    r = np.array([p[1] for p in s["profile"]]) * s["diameter"] / 2.0
    # the body, with its nose band (a fused bomb's yellow ring) as its own part
    lo, hi = s.get("band_at", (None, None))
    parts = []
    if lo is not None:
        cuts = sorted(set(st.tolist() + [lo, hi]))
        rr = np.interp(cuts, st, r)
        v, t = model3d._revolve_mesh(np.array(cuts), rr, n=40)
        n = 40
        ring = [k for k, x in enumerate(cuts[:-1]) if lo <= x < hi]
        mask = np.zeros(len(t), bool)
        for k in ring:
            mask[k * 2 * n:(k + 1) * 2 * n] = True
        g = model3d._to_gltf(v, origin)
        parts += [(g, t[~mask], body), (g, t[mask], band)]
    else:
        v, t = model3d._revolve_mesh(st, r, n=40)
        parts.append((model3d._to_gltf(v, origin), t, body))
    f = s.get("fins") or {"count": 0, "root_le": 0.0, "root_chord": 0.0}
    root_r = float(np.interp(f["root_le"] + f["root_chord"] / 2, st, r))
    for k in range(f["count"]):
        v, t = _fin(f, root_r * 0.98, math.radians(f["roll_deg"]) + 2 * math.pi * k / f["count"])
        parts.append((model3d._to_gltf(v, origin), t, body))
    mesh = B.mesh(name, parts)
    node = B.node(name + "-body", mesh=mesh)
    model3d.finish(B, [node], name, path)
    return path
