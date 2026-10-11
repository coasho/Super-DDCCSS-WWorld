"""A store through the pipeline (docs/hangar.md, Stores): a missile, a bomb, a tank, a pod or a cargo store is a design
like an aircraft - aircraft/<name>/<name>.toml, its card (assets/stores) the published figures it holds - built by the
same command and the same stages,

    python -m hangar <name> [stage ...] [--force]     one store's (default: all of DEFAULT)
    python -m hangar stores [stage ...] [--force] [--jobs N] [--record FILE]
                                                      every store's, several at once

--force makes each output again whatever is cached: its model meshed anew, its flight model's files removed and
written. Each run notes what its model and its flight model took to make and what they weigh (out/model.json,
out/build.json); --record writes every store's as a table (docs/hangar-stores-build.md), with --jobs 1 one store at
a time, so that no store's time is another's.

each stage a store's own where an aircraft's would say nothing of it:

  geometry    the aircraft's (its summary and views); its length and width against its card
  aero        its build-up (storeaero.py: slender body, Barrowman, Jorgensen; storedrag.py: Hoerner) from the design -
              and its card's carriage figures (its drag area, its Munk volume, its fins' side force) held to it; a
              cargo store's faces
  mass        the aircraft's mass model; its mass against its card's
  propulsion  none - or a missile's motor: its design's phases as its flight model burns them (storebodies.py)
  build       its flight model, aircraft/stores/<model>/<model>.xml, written: a tank's, a pod's and a cargo store's
              from its card (storemodels.py), a missile's and a bomb's from its design, a missile's with its motor's
              files (storebodies.py); its mass, its inertia and its lift held to the design
  model       the aircraft's (model3d.py), aircraft/<name>/<name>.glb; a cargo store's box (stores.py)
  verify      the platform loads it: what it weighs and where its CG is, against the design
  fly         carried and released: on a station of an aircraft that takes it, jettisoned, followed to the ground -
              the station's mass, the released vehicle's, its fall an unpowered body's; a missile's motor burnt (made on
              its own, the platform lights it): its propellant, its burn's time, the speed it gains
  report      the aircraft's page

calibrate, autopilot, performance and ground are not a store's: it has no published performance to fit, no loops and
no wheels.
"""
import json
import math
import os
import re
import subprocess
import sys
import time
import tomllib

from . import pipeline
from .pipeline import check, info

DEFAULT = ("geometry", "aero", "mass", "propulsion", "build", "model", "verify", "fly", "report")
NOT_A_STORES = {"calibrate": "it has no published performance to fit", "autopilot": "it has no control loops",
                "performance": "it flies no tables", "ground": "it has no wheels: its fall ends the fly stage"}
TITLES = {"geometry": "Geometry", "aero": "Aerodynamics: its build-up, and its card's figures", "mass": "Mass", "propulsion": "Propulsion",
          "build": "Its flight model", "verify": "The platform loads it", "fly": "Carried and released"}
LB, IN, G = 0.45359237, 0.0254, 9.80665


def is_store(path):
    with open(path, "rb") as f:
        return tomllib.load(f).get("aircraft", {}).get("category") in ("missile", "store")


def designs():
    """Every store's design (its toml), by name."""
    root = os.path.join(pipeline.repo_root(), "aircraft")
    out = {}
    for name in sorted(os.listdir(root)):
        path = os.path.join(root, name, name + ".toml")
        if os.path.isfile(path) and is_store(path):
            out[name] = path
    return out


def card_of(d):
    cid = d.aircraft.spec.get("aircraft", {}).get("card")
    if not cid:
        raise ValueError("%s: a store's design names its card ([aircraft] card)" % d.name)
    with open(os.path.join(pipeline.repo_root(), "assets", "stores", cid + ".json"), encoding="utf-8") as f:
        return json.load(f)


def _par(card, name, fallback=None):
    p = card.get("parameters", {}).get(name)
    return p["value"] if p else fallback


def _box(d):
    return d.aircraft.spec.get("store", {}).get("shape") == "box"


def flight_model(card):
    """Its flight model's file (aircraft/stores/<model>/<model>.xml), and whether hangar writes it - every store's."""
    name = card.get("model", "")[7:]
    return os.path.join(pipeline.repo_root(), "aircraft", "stores", name, name + ".xml"), True


def inertia(d):
    """(Ixx, Iyy, Izz) kg m2 of the store as it is launched, about its CG: the mass model's - but a missile's radii of
    gyration are over its length (missiles.py: a slender rod's), not Roskam's over half of it."""
    from .mass import MassModel
    e = MassModel(d.aircraft).empty()
    if d.aircraft.category == "missile":
        from . import storebodies
        a = d.aircraft
        length = float(max(b.x[-1] for b in a.bodies) - min(b.x[0] for b in a.bodies))
        ixx, iyy = storebodies._inertia(a.spec, e["mass"], length)
        return ixx, iyy, iyy
    return e["ixx"], e["iyy"], e["izz"]


def glb_points(path):
    """Every vertex of a .glb as it is drawn (its nodes' transforms applied; glTF axes: x left, y up, z forward - its
    manifest's), m from its origin. A motor's flame is no part of it."""
    import struct
    import numpy as np
    with open(path, "rb") as f:
        data = f.read()
    n = struct.unpack_from("<I", data, 12)[0]
    j = json.loads(data[20:20 + n])
    binary = data[20 + n + 8:]
    nodes = j.get("nodes", [])

    def local(node):
        if "matrix" in node:
            return np.array(node["matrix"], float).reshape(4, 4).T
        m = np.eye(4)
        x, y, z, w = node.get("rotation", [0.0, 0.0, 0.0, 1.0])
        m[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
        m[:3, :3] = m[:3, :3] * np.array(node.get("scale", [1.0, 1.0, 1.0]), float)
        m[:3, 3] = node.get("translation", [0.0, 0.0, 0.0])
        return m

    def positions(index):
        acc = j["accessors"][index]
        view = j["bufferViews"][acc["bufferView"]]
        start = view.get("byteOffset", 0) + acc.get("byteOffset", 0)
        stride = view.get("byteStride", 0) or 12
        raw = np.frombuffer(binary, np.uint8, stride * (acc["count"] - 1) + 12, start)
        idx = (np.arange(acc["count"])[:, None] * stride + np.arange(12)[None, :]).ravel()
        return raw[idx].view("<f4").reshape(-1, 3).astype(float)

    out = []

    def walk(i, parent):
        node = nodes[i]
        m = parent @ local(node)
        if "mesh" in node and "fsim:afterburner" not in node.get("name", ""):
            for prim in j["meshes"][node["mesh"]]["primitives"]:
                v = positions(prim["attributes"]["POSITION"])
                out.append(v @ m[:3, :3].T + m[:3, 3])
        for c in node.get("children", []):
            walk(c, m)

    for root in j["scenes"][j.get("scene", 0)]["nodes"]:
        walk(root, np.eye(4))
    return np.vstack(out)


def picture_checks(d, card, glb):
    """Its model against its flight model: the viewer hangs the model's origin where the simulation has the store (its
    CG as it is launched: storebodies.py's AERORP and VRP, a tank's and a cargo store's middle), its nose along its
    body's x - so the picture is the store's only if its origin is its CG and its size its card's."""
    import numpy as np
    v = glb_points(glb)
    lo, hi = v.min(axis=0), v.max(axis=0)
    a = d.aircraft
    nose = float(min(b.x[0] for b in a.bodies))
    tail = float(max(b.x[-1] for b in a.bodies))
    cg = float(a.spec["mass"]["empty_cg"][0])
    length = tail - nose
    tol = max(0.004, 0.003 * length)
    # (its fins' trailing edges may reach behind its body's end: its design's)
    aft = max([tail] + [float(sec.le[0] + sec.chord) for srf in a.surfaces for sec in srf.sections])
    checks = [check("its model's nose ahead of its origin: its CG's", float(hi[2]), cg - nose - tol, cg - nose + tol, "m",
                    note="the flight model's CG is %.4g m from its nose" % (cg - nose), fmt="%.4g"),
              check("its model's tail behind its origin", float(-lo[2]), tail - cg - tol, aft - cg + tol, "m",
                    note="%.4g m long: its card's %.4g m%s" % (float(hi[2] - lo[2]), _par(card, "length_m"),
                                                              "; its fins end %.3f m behind its body" % (aft - tail) if aft > tail + 1e-6 else ""), fmt="%.4g")]
    if _box(d):
        w, h = _par(card, "width_m"), _par(card, "height_m")
        checks += [check("its model's width", float(hi[0] - lo[0]), w - tol, w + tol, "m", note="its card: %.4g m" % w, fmt="%.4g"),
                   check("its model's height", float(hi[1] - lo[1]), h - tol, h + tol, "m", note="its card: %.4g m" % h, fmt="%.4g"),
                   check("its model about its origin", float(max(abs(hi[0] + lo[0]), abs(hi[1] + lo[1]))), None, 2.0 * tol, "m",
                         note="its middle: the flight model's CG")]
    else:
        body = next(b for b in a.bodies if getattr(b, "kind", "") == "fuselage")
        dia = float(max(body.w))
        # across it: twice the furthest it reaches from its axis - its body's diameter, or its fins' tips (a missile's
        # span as its card gives it, tip to tip across opposite fins)
        reach = pipeline.missile_span(a) if a.surfaces else dia
        across = 2.0 * float(np.hypot(v[:, 0], v[:, 1]).max())
        want = max(reach, dia)
        span = _par(card, "span_m")
        # (a fin's thin tip is meshed a few millimetres short - two or three of the mesher's cells; a rolleron's wheel
        # stands out past the tip it is on)
        wheels = max([2.0 * float(srf.get("rolleron", {}).get("radius", 0.0)) for srf in a.spec.get("surface", [])] or [0.0])
        slack = max(tol, 0.03 * want) if a.surfaces else max(0.004, 0.01 * want)
        checks.append(check("its model across, from its axis", across, want - slack, want + slack + wheels, "m",
                            note=("its design's fins tip to tip: %.4g m%s%s" % (reach, "; its card's span %.4g m" % span if span else "",
                                                                                "; its rollerons past their tips" if wheels else ""))
                            if a.surfaces else "its card's diameter: %.4g m" % _par(card, "diameter_m"), fmt="%.4g"))
        checks.append(check("its model about its axis", float(max(abs(hi[0] + lo[0]), abs(hi[1] + lo[1]))), None, max(0.01, 0.03 * across), "m",
                            note="its axis through its origin"))
    return checks


def glb_triangles(path):
    """The triangles of a .glb, every mesh's."""
    import struct
    with open(path, "rb") as f:
        data = f.read()
    n = struct.unpack_from("<I", data, 12)[0]
    j = json.loads(data[20:20 + n])
    return sum(j["accessors"][p["indices"]]["count"] // 3 for m in j.get("meshes", []) for p in m["primitives"] if "indices" in p)


def _xml_mass(text):
    """(its empty weight, what its tanks hold, its CG's x from the nose m, [ixx, iyy, izz] kg m2) as its file has them."""
    num = lambda pattern: float(re.search(pattern, text).group(1))
    empty = num(r"<emptywt[^>]*>\s*([-\d.eE+]+)") * LB
    held = sum(float(v) for v in re.findall(r"<contents[^>]*>\s*([-\d.eE+]+)", text)) * LB
    cg = re.search(r'<location name="CG" unit="IN">\s*<x>\s*([-\d.eE+]+)', text)
    k = 1.35581795
    inertia = [num(r"<%s unit=\"SLUG\*FT2\">\s*([-\d.eE+]+)" % a) * k for a in ("ixx", "iyy", "izz")]
    return empty, held, float(cg.group(1)) * IN if cg else None, inertia


# --- the stages -----------------------------------------------------------------------------------------------------
def geometry(d):
    g = dict(d.geometry())
    card = card_of(d)
    a = d.aircraft
    body = next(b for b in a.bodies if getattr(b, "kind", "") == "fuselage")
    length = float(max(b.x[-1] for b in a.bodies) - min(b.x[0] for b in a.bodies))  # (nose to tail: a seeker's dome is a body of its own)
    across = float(max(body.w))
    checks = list(g.get("checks", []))
    L = _par(card, "length_m")
    D = _par(card, "width_m") if _box(d) else _par(card, "diameter_m")
    checks.append(check("length against its card", length, 0.995 * L, 1.005 * L, "m", note="the card: %.4g m" % L))
    checks.append(check("%s against its card" % ("width" if _box(d) else "diameter"), across, 0.995 * D, 1.005 * D, "m", note="the card: %.4g m" % D))
    g["checks"] = checks
    for k in ("stage", "hangar", "time"):
        g.pop(k, None)
    return d.save("geometry", g)


def aero(d):
    card = card_of(d)
    cg = float(d.aircraft.spec["mass"]["empty_cg"][0])
    if _box(d):
        L, W, H = _par(card, "length_m"), _par(card, "width_m"), _par(card, "height_m")
        cd = _par(card, "drag_cd", 1.1)
        data = {"kind": "box", "cd": cd, "front_m2": W * H, "side_m2": L * H, "top_m2": L * W}
        m = _par(card, "mass_kg")
        vt = [math.sqrt(2.0 * m * G / (1.225 * cd * s)) for s in (W * H, L * W)]
        checks = [info("its faces' drag coefficient", cd, note="its card's drag_cd: a bluff body's on each face, broadside to the flow"),
                  info("drag area end on, side on, flat", "%.2f, %.2f, %.2f" % (cd * W * H, cd * L * H, cd * L * W), "m2"),
                  info("its fall's speed at sea level, end on and flat", "%.0f, %.0f" % (vt[0], vt[1]), "m/s", note="where its drag is its weight")]
        data["terminal_ms"] = vt
        return d.save("aero", dict(data, checks=checks))
    from . import storeaero, storedrag
    geo = storeaero.geometry(card)
    if geo is None or geo.get("source") != "its design":
        raise ValueError("%s: its build-up does not read its design (its card %s names another)" % (d.name, card["id"]))
    b = storeaero.build_up(geo)
    s = storeaero.summary(geo)
    cds, parts = storedrag.build_up(card)
    checks = []
    # the card's carriage figures are this design's: what its aircraft feels while it carries it
    for key, value, unit in (("drag_area_m2", cds, "m2"), ("munk_volume_m3", parts["munk"], "m3"), ("side_slope_m2", parts["slope"], "m2"),
                             ("side_arm_m", parts["arm"], "m")):
        held = _par(card, key)
        if held is None:
            checks.append(check("its card's %s" % key, None, level="warn", note="the card has none: python -m hangar storedrag"))
            continue
        tol = max(2e-5, 0.002 * abs(value))
        checks.append(check("its card's %s is its design's" % key, held, value - tol, value + tol, unit, note="built up from the design: %.5g" % value,
                            fmt="%.5g"))
    i = storeaero.MACH.index(0.6)
    margin = (s["normal_cp_m"] - s["cg_m"]) / geo["diameter"]
    checks += [info("drag area in carriage, below Mach 0.8", cds, "m2", note="body %.5f, base %.5f, fins %.5f (Hoerner)" % (parts["body"], parts["base"], parts["fins"])),
               info("normal-force slope, Mach 0.6", b["cna"][i], "/rad", note="on its frontal area %.4f m2" % b["S"]),
               info("centre of pressure behind its CG, Mach 0.6", margin if abs(margin) < 50.0 else "far ahead", "calibres",
                    note="it weathercocks" if margin > 0.0 else "ahead of its CG: free, it tumbles (Munk's couple, no fins to hold it)")]
    data = {"kind": "body", "S": b["S"], "d": b["d"], "ref": b["ref"], "mach": b["mach"], "cna": b["cna"], "cma": b["cma"], "cmq": b["cmq"],
            "cyb": b["cyb"], "cnb": b["cnb"], "cnr": b["cnr"], "clp": b["clp"], "cross_n": b["cross_n"], "cross_m": b["cross_m"],
            "drag_area_m2": cds, "drag": {k: parts[k] for k in ("body", "base", "fins", "frontal")}, "munk_m3": parts["munk"],
            "side_slope_m2": parts["slope"], "side_arm_m": parts["arm"], "cp_m": s["normal_cp_m"], "cg_m": cg}
    return d.save("aero", dict(data, checks=checks))


def mass(d):
    from .mass import MassModel
    card = card_of(d)
    mm = MassModel(d.aircraft)
    e = mm.empty()
    m, cg = mm.loaded()
    g = mm.gyration()
    want = _par(card, "mass_kg")
    checks = [check("mass against its card", e["mass"], want - 0.01, want + 0.01, "kg",
                    note="the card: %.6g kg%s" % (want, " (a tank's: dry; its %.0f kg of fuel is its station's)" % _par(card, "fuel_kg")
                                                   if _par(card, "fuel_kg") else "")),
              info("its CG from its nose", float(e["cg"][0]), "m"),
              info("moments of inertia Ixx, Iyy, Izz", "%.4g, %.4g, %.4g" % inertia(d), "kg m2",
                   note="[mass] gyration%s" % (": a missile's, over its length - its motor full" if d.aircraft.category == "missile" else ""))]
    breakdown = [{"name": n, "mass_kg": mk, "kind": k} for n, mk, k in mm.breakdown()]
    return d.save("mass", {"empty": e, "loaded_mass_kg": m, "loaded_cg_m": cg, "gyration": g, "static_margin": None, "breakdown": breakdown,
                           "checks": checks})


def propulsion(d):
    motor = d.aircraft.spec.get("motor")
    if not motor:
        return d.save("propulsion", {"checks": [info("propulsion", "none", note="an unpowered store")]})
    from . import storebodies
    a = d.aircraft
    body = next(b for b in a.bodies if getattr(b, "kind", "") == "fuselage")
    m = storebodies.motor_of(a.spec, float(a.spec["mass"]["empty"]), float(a.spec["mass"]["empty_cg"][0]), 0.5 * float(max(body.w)))
    phases = motor.get("phase", [])
    impulse = sum(1000.0 * p["thrust_kn"] * p["duration_s"] for p in phases)
    checks = [info("its motor's burn", "%.1f s, %.0f kN s" % (m["burn_s"], impulse / 1000.0),
                   note="its design's phases (assumed: none is published): %s kN" % ", ".join("%g" % p["thrust_kn"] for p in phases)),
              info("its propellant", m["propellant_kg"], "kg", note="what that impulse takes at %.0f s (its smoke's specific impulse: assumed)" % m["isp"]),
              info("its share of the launch mass", 100.0 * m["propellant_kg"] / float(a.spec["mass"]["empty"]), "%"),
              info("its grain", "%.2f m long, %.3f m across, a bore of %.3f m" % (m["length"], 2.0 * m["radius"], m["bore"]), "",
                   note="a cylinder in the body ahead of its nozzle, at %.0f kg/m3 (assumed)" % storebodies.GRAIN_DENSITY),
              check("its grain fits its body", m["x"] - 0.5 * m["length"], float(body.x[0]) + 0.3 * float(body.x[-1] - body.x[0]), None, "m", level="warn",
                    note="its forward end, from the nose: behind the first three tenths (its seeker, its warhead)")]
    return d.save("propulsion", {"motor": {k: v for k, v in m.items() if k != "rows"}, "checks": checks})


def build(d, force=False):
    from . import storemodels
    card = card_of(d)
    path, _ = flight_model(card)
    folder = os.path.dirname(path)
    t0 = time.perf_counter()
    out = storemodels.files(card)
    if out is None:
        raise ValueError("%s: its card or its design lacks what its flight model is written from (its mass, its size)" % d.name)
    paths = {rel: os.path.join(folder, *rel.split("/")) for rel in out}
    if force:
        for p in paths.values():
            if os.path.isfile(p):
                os.remove(p)
    changed = []
    for rel, text in out.items():
        os.makedirs(os.path.dirname(paths[rel]), exist_ok=True)
        if pipeline.write_text(paths[rel], text):
            changed.append(rel)
    seconds = time.perf_counter() - t0
    sizes = {rel: os.path.getsize(p) for rel, p in paths.items()}
    main = os.path.basename(path)
    checks = [info("flight model", pipeline.shown(path), note=("written" if main in changed else "as it was") + ": %.1f kB in %.0f ms" % (
        sizes[main] / 1000.0, 1000.0 * seconds))]
    for rel in sorted(out):
        if rel != main:
            checks.append(info("its motor's file", rel, note=("written" if rel in changed else "as it was") + ": %.1f kB" % (sizes[rel] / 1000.0)))
    text = out[main]
    empty, held, cg, got_inertia = _xml_mass(text)
    want = _par(card, "mass_kg")
    checks.append(check("its mass is its card's", empty + held, want - 0.02, want + 0.02, "kg",
                        note="its structure %.2f kg%s" % (empty, " and %.2f kg of propellant" % held if held else "")))
    want_inertia = inertia(d)
    if held > 0.0:
        # its file has its structure burnt out, and JSBSim adds its grain: the two together, about its launch CG
        from . import storebodies
        a = d.aircraft
        body = next(b for b in a.bodies if getattr(b, "kind", "") == "fuselage")
        launch = float(a.spec["mass"]["empty_cg"][0])
        m = storebodies.motor_of(a.spec, want, launch, 0.5 * float(max(body.w)))
        R, rb, lg, xg, mp = m["radius"], 0.5 * m["bore"], m["length"], m["x"], m["propellant_kg"]
        got_inertia = [got_inertia[0] + 0.5 * mp * (R * R + rb * rb)] + [
            i + mp * (3.0 * (R * R + rb * rb) + lg * lg) / 12.0 + mp * (xg - launch) ** 2 + empty * (cg - launch) ** 2 for i in got_inertia[1:]]
        checks.append(check("its CG at launch is its design's", (empty * cg + mp * xg) / (empty + mp), launch - 0.002, launch + 0.002, "m",
                            note="its structure's %.3f m and its grain's %.3f m from the nose" % (cg, xg)))
    for axis, got, want_i in zip(("Ixx", "Iyy", "Izz"), got_inertia, want_inertia):
        checks.append(check("its %s is its design's" % axis, got, 0.98 * want_i, 1.02 * want_i, "kg m2",
                            note="the design: %.4g%s" % (want_i, " (its motor full)" if held > 0.0 else "")))
    aero_data = d.load("aero")
    if aero_data and aero_data.get("kind") == "body" and card.get("class") in ("aam", "bomb"):
        # its lift and its moment, as its file tables them over Mach, against the aero stage's build-up
        for label, key in (("lift slope", "cna"), ("moment slope about its launch CG", "cma")):
            axis = "NORMAL" if key == "cna" else "PITCH"
            block = text[text.index('<axis name="%s">' % axis):]
            rows = re.findall(r"^\s*([\d.]+)\s+(-?[\d.]+)\s*$", block[:block.index("</tableData>")], re.M)
            table = {float(m): float(v) for m, v in rows}
            worst = max(abs(table[m] - v) / max(abs(v), 1e-9) for m, v in zip(aero_data["mach"], aero_data[key]) if m in table)
            checks.append(check("its file's %s is its design's" % label, 100.0 * worst, None, 0.5, "%",
                                note="the largest difference over Mach from the aero stage's build-up"))
    return d.save("build", {"path": path, "written": True, "changed": changed, "seconds": seconds, "bytes": sizes, "checks": checks})


def model(d, force=False):
    glb = os.path.join(d.dir, d.name + ".glb")
    if not _box(d):
        out = d.model(force=force)
        out["triangles"] = glb_triangles(glb)
        out["bytes"] = os.path.getsize(glb)
        out["checks"] = list(out.get("checks", [])) + picture_checks(d, card_of(d), glb)
        return d.save("model", {k: v for k, v in out.items() if k not in ("stage", "hangar", "time")})
    from . import stores
    card = card_of(d)
    t0 = time.perf_counter()
    if force and os.path.isfile(glb):
        os.remove(glb)
    stores.write_cargo(d.name, card, glb)
    seconds = time.perf_counter() - t0
    return d.save("model", {"glb": glb, "seconds": seconds, "triangles": glb_triangles(glb), "bytes": os.path.getsize(glb),
                            "checks": [info("3D model", pipeline.shown(glb), note="its box, its pallet and its straps (stores.py)"),
                                       info("3D model: triangles", glb_triangles(glb), note="%.1f kB, written in %.0f ms" % (
                                           os.path.getsize(glb) / 1000.0, 1000.0 * seconds))] + picture_checks(d, card, glb)})


def _world(name):
    import fsim
    return fsim, fsim.World(name, publish=False, workers=1, pin_workers=False, seed=3, terrain=False)


def verify(d):
    card = card_of(d)
    fsim, w = _world("hangar-store-verify")
    checks = []
    try:
        v = w.create_vehicle(d.name, card["model"], latitude_deg=37.6, longitude_deg=-122.4, altitude_msl_m=3000.0, airspeed_ms=150.0)
        # (as it is made, before a step: a missile made on its own - not released from a station - has its motor lit)
        want = _par(card, "mass_kg")
        checks.append(check("the platform loads it: its mass", v.mass_kg, want - 0.02, want + 0.02, "kg", note="its card: %.6g kg" % want))
        cg = float(d.aircraft.spec["mass"]["empty_cg"][0])
        body = next(b for b in d.aircraft.bodies if getattr(b, "kind", "") == "fuselage")
        got = v.get_property("inertia/cg-x-in") * IN
        tol = 0.02 * float(body.x[-1] - body.x[0])
        checks.append(check("its CG from its nose", got, cg - tol, cg + tol, "m", level="warn", note="the design: %.4g m" % cg))
    except Exception as ex:  # noqa: BLE001 - what the platform said
        checks.append(check("the platform loads it", None, note=str(ex)))
        checks[-1]["status"] = "fail"
    finally:
        w.close()
    return d.save("verify", {"checks": checks})


def carrier_for(card):
    """(aircraft, station number, in a loadout of its) that carries the store: one whose loadouts hang it, else any
    whose station takes its class and its weight. None: nothing carries it."""
    root = pipeline.repo_root()
    need = _par(card, "mass_kg") + (_par(card, "fuel_kg") or 0.0)
    found = []
    for name in sorted(os.listdir(os.path.join(root, "aircraft"))):
        fit = os.path.join(root, "aircraft", name, "payloads.json")
        if not os.path.isfile(fit) or not os.path.isfile(os.path.join(root, "aircraft", name, name + ".xml")):
            continue
        with open(fit, encoding="utf-8") as f:
            fit = json.load(f)
        stations = {}
        for p in fit.get("payloads", []):
            path = os.path.join(root, "assets", "payloads", p.get("card", "") + ".json")
            if os.path.isfile(path):
                with open(path, encoding="utf-8") as f:
                    c = json.load(f)
                if c.get("family") == "store_station":
                    stations.update({s["number"]: s for s in c.get("stations", [])})
        hung = [it["station"] for lo in fit.get("loadouts", []) for it in lo.get("stations", []) if it.get("store") == card["id"]]
        for n in hung:
            if n in stations:
                found.append((0, name, n))
                break
        else:
            for n, s in sorted(stations.items()):
                if card["class"] in s.get("takes", []) and need <= s.get("max_kg", 0.0):
                    found.append((1, name, n))
                    break
    if not found:
        return None
    rank, name, n = min(found)
    return name, n, rank == 0


def fly(d):
    card = card_of(d)
    who = carrier_for(card)
    if who is None:
        return d.save("fly", {"checks": [check("an aircraft that carries it", None, level="warn",
                                               note="no station of any fit takes a %s of %.0f kg: it is not flown" % (card["class"], _par(card, "mass_kg")))]})
    carrier, station, in_loadout = who
    fsim, w = _world("hangar-store-fly")
    checks, track = [], []
    try:
        probe = w.create_vehicle("probe", "jsbsim:" + carrier, latitude_deg=37.6, longitude_deg=-122.4, altitude_msl_m=1500.0, airspeed_ms=150.0)
        w.step()
        speed = max(probe.performance.cruise_tas_ms, 1.6 * probe.performance.min_cas_ms)
        probe.remove()
        w.step()
        v = w.create_vehicle("carrier", "jsbsim:" + carrier, latitude_deg=37.6, longitude_deg=-122.4, altitude_msl_m=1500.0, airspeed_ms=speed,
                             heading_deg=90.0)
        w.step()
        clean = v.mass_kg
        v.load_store(station, card["id"])
        w.step(2)
        s = next(x for x in v.stores if x.station == station)
        want = _par(card, "mass_kg") + (_par(card, "fuel_kg") or 0.0)
        tank = bool(_par(card, "fuel_kg"))
        checks.append(info("carried by", "%s, station %d" % (carrier, station), note="one of its loadouts hangs it there" if in_loadout
                           else "no loadout hangs it: a station that takes its class and its weight"))
        checks.append(check("on its station: its card's mass", s.mass_kg, want - (2.0 if tank else 0.01), want + 0.01, "kg",
                            note="its card: %.6g kg%s" % (want, " full (less what the aircraft has drawn)" if tank else "")))
        checks.append(check("its aircraft heavier by it", v.mass_kg - clean, s.mass_kg - 1.0, s.mass_kg + 1.0, "kg"))
        v.submit_hsa(heading_rad=0.5 * math.pi, altitude_m=1500.0, speed=speed, speed_reference="true_airspeed")
        carried = s.mass_kg
        loaded = v.mass_kg
        v.jettison(emergency=True)
        body = None
        for _ in range(int(6.0 / w.step_seconds)):
            w.step()
            s = next(x for x in v.stores if x.station == station)
            if s.weapon:
                body = w.vehicle(s.weapon)
                break
        if body is None:
            checks.append(check("released", None, note="it did not leave its station in 6 s (%s)" % s.status))
            checks[-1]["status"] = "fail"
            return d.save("fly", {"carrier": carrier, "station": station, "checks": checks})
        st = body.state
        v0, h0 = st.airspeed_true_ms, st.altitude_msl_m
        checks.append(check("released: a vehicle of its own, as heavy", body.mass_kg, carried - 0.1, carried + 0.1, "kg"))
        checks.append(check("its aircraft lighter by it", loaded - v.mass_kg, carried - 1.5, carried + 1.5, "kg"))
        n = int(round(0.1 / w.step_seconds))
        lightest, heaviest, fastest, bound, landed, lost = body.mass_kg, body.mass_kg, 0.0, 0.0, None, None
        for i in range(900):  # followed for 90 s, or to the ground and 6 s on
            w.step(n)
            st = body.state
            t = 0.1 * (i + 1)
            speed_now = math.sqrt(sum(c * c for c in st.velocity_ned_ms))
            track.append((t, st.altitude_msl_m, speed_now))
            if st.diverged or not math.isfinite(speed_now):
                lost = t
                break
            lightest, heaviest = min(lightest, body.mass_kg), max(heaviest, body.mass_kg)
            if landed is None:
                # no faster than a body falling from where it left with nothing pushing it (its drag only slows it)
                allowed = math.sqrt(v0 * v0 + 2.0 * G * max(h0 - st.altitude_msl_m, 0.0)) + 1.0
                if speed_now > allowed and speed_now - allowed > bound:
                    bound = speed_now - allowed
                fastest = max(fastest, speed_now)
                if st.on_ground or st.crashed or st.altitude_agl_m < 0.5:
                    landed = t
            elif t - landed > 6.0:
                break
        checks.append(check("its flight model holds", 0.0 if lost is None else 1.0, None, 0.0, "", note="followed %.0f s" % track[-1][0] if lost is None
                            else "diverged %.1f s after its release" % lost, fmt="%.0f"))
        checks.append(check("its mass as it falls", heaviest - lightest, None, 0.01, "kg", note="inert: nothing burnt, nothing lost"))
        checks.append(check("no faster than a falling body", bound, None, 0.0, "m/s", note="over what its release's speed and the height fallen allow; its fastest %.0f m/s" % fastest))
        if landed is not None:
            rest = track[-1][2]
            checks.append(info("reached the ground", landed, "s", note="from 1,500 m"))
            checks.append(check("comes to rest", rest, None, 15.0, "m/s", level="warn", note="6 s after it struck"))
        else:
            checks.append(check("reached the ground", None, level="warn", note="still %.0f m up after 90 s" % track[-1][1]))
    except Exception as ex:  # noqa: BLE001 - what the platform said
        checks.append(check("carried and released", None, note="%s: %s" % (type(ex).__name__, ex)))
        checks[-1]["status"] = "fail"
    finally:
        w.close()
    burn = _burn(d, card) if d.aircraft.spec.get("motor") else {}
    checks += burn.pop("checks", [])
    return d.save("fly", dict(burn, carrier=carrier, station=station, track=track[::5], checks=checks))


def _burn(d, card):
    """A missile's motor, burnt: made on its own (not released from a station) the platform lights it - an unguided
    body that flies off on its fins. Its flight model's motor is its design's if it burns its propellant in its burn's
    time, its mass ends at its structure's, it gains the speed its impulse gives it, and it holds together."""
    from . import storebodies
    a = d.aircraft
    body = next(b for b in a.bodies if getattr(b, "kind", "") == "fuselage")
    launch = float(a.spec["mass"]["empty"])
    m = storebodies.motor_of(a.spec, launch, float(a.spec["mass"]["empty_cg"][0]), 0.5 * float(max(body.w)))
    fsim, w = _world("hangar-store-burn")
    checks, track = [], []
    try:
        v0 = 250.0
        v = w.create_vehicle(d.name, card["model"], latitude_deg=37.6, longitude_deg=-122.4, altitude_msl_m=8000.0, airspeed_ms=v0, heading_deg=270.0)
        n = max(1, int(round(0.05 / w.step_seconds)))
        burnt_at, fastest, lost, t = None, v0, None, 0.0
        end = launch - m["propellant_kg"]
        while t < m["burn_s"] + 3.0:
            w.step(n)
            t += n * w.step_seconds
            st = v.state
            speed = math.sqrt(sum(c * c for c in st.velocity_ned_ms))
            if st.diverged or not math.isfinite(speed):
                lost = t
                break
            fastest = max(fastest, speed)
            track.append((t, v.mass_kg, speed))
            if burnt_at is None and v.mass_kg <= end + 0.01 * m["propellant_kg"]:
                burnt_at = t
        final = track[-1][1] if track else float("nan")
        impulse = m["isp"] * G * m["propellant_kg"]
        ideal = m["isp"] * G * math.log(launch / end)  # (Tsiolkovsky: what its impulse gives it with no drag and no weight)
        checks.append(check("its motor lit: its flight model holds", 0.0 if lost is None else 1.0, None, 0.0, "", fmt="%.0f",
                            note="made on its own at 8,000 m and %.0f m/s, followed %.0f s" % (v0, t) if lost is None else "diverged after %.1f s" % lost))
        checks.append(check("its propellant burnt", launch - final, 0.98 * m["propellant_kg"], 1.02 * m["propellant_kg"], "kg",
                            note="its design's %.2f kg (%.0f kN s at %.0f s)" % (m["propellant_kg"], impulse / 1000.0, m["isp"])))
        checks.append(check("burnt out at", burnt_at, 0.9 * m["burn_s"] - 0.2, 1.1 * m["burn_s"] + 0.3, "s", note="its design's phases: %.1f s" % m["burn_s"]))
        checks.append(check("the speed it gained", fastest - v0, 0.45 * ideal, 1.02 * ideal, "m/s",
                            note="its impulse on its mass would give %.0f m/s with no drag: its fastest %.0f m/s (Mach %.1f at that height)" % (
                                ideal, fastest, fastest / 308.0)))
    except Exception as ex:  # noqa: BLE001 - what the platform said
        checks.append(check("its motor lit", None, note="%s: %s" % (type(ex).__name__, ex)))
        checks[-1]["status"] = "fail"
    finally:
        w.close()
    return {"burn": track[::4], "checks": checks}


def report(d):
    from .report import html
    card = card_of(d)
    return {"path": html.write(d, titles=TITLES, description="%s - a store (%s): its card assets/stores/%s.json" % (
        card.get("designation", d.name), card.get("class"), card["id"]))}


STAGES = {"geometry": geometry, "aero": aero, "mass": mass, "propulsion": propulsion, "build": build, "model": model, "verify": verify, "fly": fly,
          "report": report}


def run(d, stage, force=False):
    if stage in NOT_A_STORES:
        return {"checks": [info(stage, "not a store's", note=NOT_A_STORES[stage])]}
    if stage in ("build", "model"):
        return STAGES[stage](d, force=force)
    return STAGES[stage](d)


def record(names):
    """What each store's model and flight model took to make and what they weigh, from its last run (out/model.json,
    out/build.json): rows of {name, class, designation, triangles, model_s, glb_bytes, xml_s, xml_bytes, motor_bytes}."""
    rows = []
    root = pipeline.repo_root()
    for name in names:
        folder = os.path.join(root, "aircraft", name)
        with open(os.path.join(folder, name + ".toml"), "rb") as f:
            spec = tomllib.load(f)
        with open(os.path.join(root, "assets", "stores", spec["aircraft"]["card"] + ".json"), encoding="utf-8") as f:
            card = json.load(f)
        row = {"name": name, "class": card.get("class", ""), "designation": card.get("designation", name)}
        for stage in ("model", "build"):
            p = os.path.join(folder, "out", stage + ".json")
            data = {}
            if os.path.isfile(p):
                with open(p, encoding="utf-8") as f:
                    data = json.load(f)
            if stage == "model":
                glb = os.path.join(folder, name + ".glb")
                row.update(triangles=glb_triangles(glb) if os.path.isfile(glb) else None, model_s=data.get("seconds"),
                           glb_bytes=os.path.getsize(glb) if os.path.isfile(glb) else None, model_at=data.get("time"))
            else:
                sizes = data.get("bytes", {})
                main = name + ".xml"
                row.update(xml_s=data.get("seconds"), xml_bytes=sizes.get(main), motor_bytes=sum(v for k, v in sizes.items() if k != main),
                           xml_at=data.get("time"))
        rows.append(row)
    return rows


def record_text(rows, stages, forced, jobs):
    """The record as a page (docs/hangar-stores-build.md)."""
    import platform
    from . import __version__
    done = sorted(r["model_at"] for r in rows if r.get("model_at"))
    lines = ["# Stores: what the pipeline built", "",
             "Every store on the cards in `assets/stores`, through hangar's stages ([hangar.md](hangar.md#stores)) - written by",
             "`python -m hangar stores%s%s --record docs/hangar-stores-build.md`. Each row is one store's last run: its model for" % (
                 " --force" if forced else "", " --jobs %d" % jobs),
             "the viewer (`aircraft/<name>/<name>.glb`) and its JSBSim flight model (`aircraft/stores/<name>/<name>.xml`; a",
             "missile's motor and nozzle files beside it, `Engines/`).", "",
             "- **Run:** %s to %s, hangar %s, %s, %d logical processors, %s." % (
                 done[0] if done else "-", done[-1] if done else "-", __version__, platform.platform(), os.cpu_count() or 0,
                 "one store at a time" if jobs == 1 else "%d stores at a time (a time here is longer than the store's alone)" % jobs),
             "- **Made again:** %s." % ("every output - each model meshed anew, each flight model's files removed and written (`--force`)" if forced
                                        else "what had changed; a time is the last making's"),
             "- **Model time:** the mesher's, the paint's and the file's writing (a cargo store's box is drawn, not meshed).",
             "- **XML time:** the build-up from the card and the design, the text and the file's writing - the engine files with it.",
             "", "| store | class | triangles | model time s | model size kB | XML time ms | XML size kB | motor files kB |",
             "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |"]

    def num(v, f):
        return "-" if v is None else f % v

    for r in rows:
        lines.append("| %s (`%s`) | %s | %s | %s | %s | %s | %s | %s |" % (
            r["designation"], r["name"], r["class"], num(r["triangles"], "%d"), num(r["model_s"], "%.3f"),
            num(None if r["glb_bytes"] is None else r["glb_bytes"] / 1000.0, "%.1f"),
            num(None if r["xml_s"] is None else 1000.0 * r["xml_s"], "%.1f"),
            num(None if r["xml_bytes"] is None else r["xml_bytes"] / 1000.0, "%.1f"),
            num(r["motor_bytes"] / 1000.0, "%.1f") if r.get("motor_bytes") else "-"))
    have = [r for r in rows if r["triangles"] is not None and r["model_s"] is not None and r["xml_s"] is not None]
    lines += ["| **%d stores** | | **%d** | **%.1f** | **%.1f** | **%.1f** | **%.1f** | **%.1f** |" % (
        len(have), sum(r["triangles"] for r in have), sum(r["model_s"] for r in have), sum(r["glb_bytes"] for r in have) / 1000.0,
        1000.0 * sum(r["xml_s"] for r in have), sum(r["xml_bytes"] or 0 for r in have) / 1000.0, sum(r.get("motor_bytes") or 0 for r in have) / 1000.0),
        "", "By class:", "", "| class | stores | triangles, each | model time s, each | model size kB, each | XML time ms, each | XML size kB, each |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: |"]
    for cls in sorted({r["class"] for r in have}):
        rs = [r for r in have if r["class"] == cls]
        rng = lambda key, scale, f: (f + " - " + f) % (min(r[key] for r in rs) * scale, max(r[key] for r in rs) * scale)  # noqa: E731
        lines.append("| %s | %d | %s | %s | %s | %s | %s |" % (cls, len(rs), rng("triangles", 1, "%d"), rng("model_s", 1.0, "%.3f"),
                                                             rng("glb_bytes", 0.001, "%.1f"), rng("xml_s", 1000.0, "%.1f"), rng("xml_bytes", 0.001, "%.1f")))
    lines += ["", "Stages run: %s." % ", ".join(stages), ""]
    return "\n".join(lines)


def main(argv):
    """Every store through `argv`'s stages (default: all), several at once; a line a store."""
    forced = "--force" in argv
    jobs = max(1, (os.cpu_count() or 2) // 2)
    target, stages, skip = None, [], False
    for i, a in enumerate(argv):
        if skip:
            skip = False
        elif a == "--jobs":
            jobs, skip = max(1, int(argv[i + 1])), True
        elif a == "--record":
            target, skip = argv[i + 1], True
        elif not a.startswith("-"):
            stages.append(a)
    stages = stages or list(DEFAULT)
    names = list(designs())
    running, results = [], {}
    env = dict(os.environ)

    def reap(block):
        for item in list(running):
            name, p = item
            if block:
                p.wait()
            if p.poll() is not None:
                running.remove(item)
                results[name] = p.returncode
                block = False

    for name in names:
        while len(running) >= jobs:
            reap(True)
        running.append((name, subprocess.Popen([sys.executable, "-m", "hangar", name] + stages + (["--force"] if forced else []), env=env, stdout=subprocess.DEVNULL,
                                               stderr=subprocess.DEVNULL, cwd=pipeline.repo_root())))
    while running:
        reap(True)
    failed = 0
    print("%-20s %-6s %5s %5s  %s" % ("store", "class", "fail", "warn", "what"))
    for name in names:
        out = os.path.join(pipeline.repo_root(), "aircraft", name, "out")
        fails, warns, what = 0, 0, []
        for st in DEFAULT:
            p = os.path.join(out, st + ".json")
            if not os.path.isfile(p):
                continue
            with open(p, encoding="utf-8") as f:
                for c in json.load(f).get("checks", []):
                    if c["status"] in ("fail", "warn"):
                        fails += c["status"] == "fail"
                        warns += c["status"] == "warn"
                        what.append("%s: %s" % (st, c["name"]))
        with open(os.path.join(pipeline.repo_root(), "aircraft", name, name + ".toml"), "rb") as f:
            spec = tomllib.load(f)
        with open(os.path.join(pipeline.repo_root(), "assets", "stores", spec["aircraft"]["card"] + ".json"), encoding="utf-8") as f:
            cls = json.load(f).get("class", "")
        crashed = results.get(name, 0) not in (0, 1)
        failed += bool(fails) or crashed
        print("%-20s %-6s %5d %5d  %s" % (name, cls, fails, warns, ("the run ended %d; " % results[name] if crashed else "") + "; ".join(what[:4])))
    print("%d stores, %d with a failed check" % (len(names), failed))
    rows = record(names)
    print()
    print("%-20s %-6s %9s %9s %9s %9s %9s" % ("store", "class", "triangles", "model s", "glb kB", "xml ms", "xml kB"))
    for r in rows:
        print("%-20s %-6s %9s %9s %9s %9s %9s" % (
            r["name"], r["class"], r["triangles"], "-" if r["model_s"] is None else "%.2f" % r["model_s"],
            "-" if r["glb_bytes"] is None else "%.1f" % (r["glb_bytes"] / 1000.0), "-" if r["xml_s"] is None else "%.1f" % (1000.0 * r["xml_s"]),
            "-" if r["xml_bytes"] is None else "%.1f" % (r["xml_bytes"] / 1000.0)))
    if target:
        path = target if os.path.isabs(target) else os.path.join(pipeline.repo_root(), target)
        pipeline.write_text(path, record_text(rows, stages, forced, jobs))
        with open(os.path.splitext(path)[0] + ".json", "w", encoding="utf-8", newline="\n") as f:
            json.dump({"stages": stages, "forced": forced, "jobs": jobs, "stores": rows}, f, indent=1)
            f.write("\n")
        print("recorded: %s" % pipeline.shown(path))
    return 1 if failed else 0
