"""An airframe's structure as the world's damage chain takes it (ADR-30, MS-7g's damage part, G2): what a warhead's
fragments and blast are laid across, and what a hit section's loads are held against - one rule for every design,
from its own geometry, mass statement and aerodynamics, the same method for every nation's aircraft:

- the breakup sections (shape/breakup.py's: the nose, the centre, each wing, the tail): each one's mass, centre of
  mass and inertia from the mass model's items (each item's points classified by breakup's rule; the fuel and the
  stores apart, as they change in flight: a section's tanks are JSBSim's, its stations the payloads'), the break it
  turns about (its root) and its share of the airframe's aerodynamic force - from hangar's own strip model, each
  strip's normal force as alpha rises 4 degrees (a section's share of dN/d alpha; the bodies' lift the centre's) and
  its side force as sideslip does, with the centres of pressure those shares act at;
- each section's strength: the bending at its root it was built to carry - the design's ultimate load factor (1.5
  times its limit: LOAD_LIMITS's, or the flight control's n_max and n_min, the envelope its structure is designed
  to) at its design mass, flown symmetrically (its share of n W at its centre of pressure, less its own structure's
  inertia relief, n g m at its centre of mass, its tanks empty: the case a wing's root is sized by) - so the intact
  aircraft fails at its ultimate load and not before, as a structure sized to it
  does (Raymer, *Aircraft Design*, 14.2; FAR/CS 25.303 - 25.305, MIL-A-8861's 1.5 factor of safety); the tail's
  and the nose's from the same rule, their own shares and masses, the tail's yaw from its fins stalling (CL 1.0) at
  the corner speed (estimates: a tail is sized by balancing, gust, rudder and sideslip loads the rule does not fly);
- the skin the fragments and the blast meet: each surface's planform in panels (no wider than PANEL_SPAN along the
  span, front and back halves of the chord) as thick as its section, each body as boxes PANEL_LENGTH long round its
  sections - each on its section, its share of the section's skin by area, the skin's mass SKIN_SHARE of its
  section's structure (an estimate: a fighter's skins and stringers some third of its structure);
- the vulnerable components (vulnerability.py), with what the world needs of each: an engine's JSBSim index, a tank's
  JSBSim index and the fuel its box holds, a control's channel.

Written beside the model (<name>.damage.json), in the model's body axes from its origin (x forward, y right, z down;
m) as the manifest's component lines are. Not the model's shape: outside its key, the model stage writes it whether
it re-meshes or not, and an edit here re-meshes nothing."""
import json
import math

import numpy as np

G0 = 9.80665
PANEL_SPAN = 1.5      # m: the widest skin panel along a surface's span
PANEL_LENGTH = 1.0    # m: a body's skin boxes along its length
SKIN_SHARE = 0.35     # of a section's structure: its skin (an estimate)
ALPHA_STEP = math.radians(4.0)

# Design limit load factors (positive, negative) where the design's flight control gives none (a fighter's envelope
# limits are its structure's: F-16C, F-15C, F-22A, Typhoon, Gripen +9/-3 and the rest as their designs give them), each
# with its basis - the airworthiness rule for its class where no figure for the type is published:
LOAD_LIMITS = {
    "b52h": (2.0, -1.0, "estimate: no published figure (a heavy bomber's, below FAR 25.337's 2.5 transport minimum)"),
    "h6k": (2.5, -1.0, "estimate: no published figure (FAR 25.337's minimum for its weight)"),
    "c130j": (3.0, -1.0, "C-130 manoeuvre load criteria, flaps up (DTIC AD0255752, the C-130A's)"),
    "ec130h": (3.0, -1.0, "C-130 manoeuvre load criteria, flaps up (DTIC AD0255752, the C-130A's)"),
    "c17a": (2.5, -1.0, "FAR 25.337 at its weight"),
    "kc135r": (2.5, -1.0, "FAR 25.337 at its weight"),
    "rc135w": (2.5, -1.0, "FAR 25.337 at its weight"),
    "e3g": (2.5, -1.0, "FAR 25.337 at its weight"),
    "kc46a": (2.5, -1.0, "FAR 25.337 at its weight"),
    "e7a": (2.5, -1.0, "FAR 25.337 at its weight"),
    "u2s": (2.5, -1.0, "estimate: no published figure (FAR 25.337's minimum)"),
    "rq4b": (2.5, -1.0, "estimate: no published figure (FAR 25.337's minimum)"),
    "c172": (3.8, -1.52, "the Cessna 172's normal category (FAR 23.337: 3.8, -0.4 of it)"),
    "skua": (3.8, -1.52, "estimate: FAR 23.337's normal category"),
    "a10c": (7.33, -3.0, "+7.33 clean (milavia, GlobalSecurity); -3 an estimate"),
    "su25": (6.5, -2.6, "+6.5 (milavia); -0.4 of it an estimate (FAR 23.337's proportion)"),
}
BETA_STEP = math.radians(4.0)


def _body_axes(p, origin):
    """Design frame (x aft, y right, z up) to body axes from the origin (x forward, y right, z down)."""
    q = np.asarray(p, float) - origin
    return np.array([-q[..., 0], q[..., 1], -q[..., 2]]).T if q.ndim > 1 else np.array([-q[0], q[1], -q[2]])


def _strip_forces(model, alpha, beta):
    """Each strip's force (design frame, per unit dynamic pressure's 0.5 rho V^2 = 0.5) and the whole aircraft's
    coefficients at alpha, beta: the strips' as the model sums them (each flow's share of each strip), caught from its
    own _strips without a change to the model (its code keys the aero stage's tables)."""
    caught = []
    own = model._strips

    def spy(*args, **kwargs):
        r = own(*args, **kwargs)
        caught.append(r[0])
        return r
    model._strips = spy
    try:
        c = model.evaluate(alpha, beta, detail=True)
    finally:
        del model._strips
    # (at the small angles the shares are taken at, the flow is one - the vortex regime's, or the attached one's -
    # every strip wholly in it: aero/model.py's VORTEX_FLOW gives way from 45 degrees)
    if len(c["detail"]["flows"]) != 1:
        raise ValueError("damage shares: %d flows at alpha %.1f deg, beta %.1f deg - expected one"
                         % (len(c["detail"]["flows"]), math.degrees(alpha), math.degrees(beta)))
    return caught[-1], c


def _sections_of(aircraft, points, mounted=False):
    from .shape import breakup
    where = breakup.classify(aircraft, np.asarray(points, float).reshape(-1, 3), mounted=mounted)
    return None if where is None else [breakup.SECTIONS[int(k)] for k in where]


def _shares(aircraft, model, origin):
    """Each section's share of the normal force's rise with alpha and of the side force's with sideslip, and the
    centres (body axes) they act at."""
    from .shape import breakup
    L = model.lat
    c4 = L.c4
    where = _sections_of(aircraft, c4)
    F0, c0 = _strip_forces(model, 0.0, 0.0)
    Fa, ca = _strip_forces(model, ALPHA_STEP, 0.0)
    Fb, cb = _strip_forces(model, 0.0, BETA_STEP)
    to_body = np.array([-1.0, 1.0, -1.0])
    dN = -((Fa - F0) * to_body)[:, 2]           # the normal force (up) each strip adds
    dY = ((Fb - F0) * to_body)[:, 1]            # the side force (right) each strip adds
    qS = 0.5 * model.S
    N_total = -(ca["CZ"] - c0["CZ"]) * qS        # the whole aircraft's, the bodies' with it
    Y_total = (cb["CY"] - c0["CY"]) * qS
    pos = _body_axes(c4, origin)
    out = {}
    for name in breakup.SECTIONS:
        k = np.array([w == name for w in where])
        n_s, y_s = float(dN[k].sum()), float(dY[k].sum())
        out[name] = {
            "normal_share": n_s / N_total if abs(N_total) > 1e-12 else 0.0,
            "side_share": y_s / Y_total if abs(Y_total) > 1e-12 else 0.0,
            "normal_cp": (dN[k] @ pos[k] / n_s).tolist() if abs(n_s) > 1e-12 else None,
            "side_cp": (dY[k] @ pos[k] / y_s).tolist() if abs(y_s) > 1e-12 else None,
        }
    # the bodies' lift (not strips) is the centre's: what the strips leave of the whole
    rest_n = 1.0 - sum(v["normal_share"] for v in out.values())
    rest_y = 1.0 - sum(v["side_share"] for v in out.values())
    out["centre"]["normal_share"] += rest_n
    out["centre"]["side_share"] += rest_y
    return out


def _masses(aircraft, mass_model, origin):
    """Each section's structure, systems, engines and gear (the fuel and the stores apart): mass, centre, inertia
    about its centre (body axes; kg, m, kg m^2) - each mass item's points classified by breakup's rule."""
    from .shape import breakup
    acc = {n: [] for n in breakup.SECTIONS}
    for name, m, pts, w, kind in mass_model.items:
        where = _sections_of(aircraft, pts, mounted=kind in ("gear", "engine"))
        for p, wi, s in zip(pts, w, where):
            acc[s].append((m * wi, _body_axes(p, origin)))
    out = {}
    for s, items in acc.items():
        if not items:
            out[s] = {"mass_kg": 0.0, "cg": [0.0, 0.0, 0.0], "inertia": [0.0] * 6}
            continue
        dm = np.array([d for d, _ in items])
        r = np.array([p for _, p in items])
        m = float(dm.sum())
        cg = (dm @ r) / m
        q = r - cg
        J = np.einsum("n,ni,nj->ij", dm, q, q)
        out[s] = {"mass_kg": m, "cg": cg.tolist(),
                  # ixx, iyy, izz, ixy, ixz, iyz (the products as plain sums: JSBSim's sign convention is the reader's)
                  "inertia": [float(J[1, 1] + J[2, 2]), float(J[0, 0] + J[2, 2]), float(J[0, 0] + J[1, 1]),
                              float(J[0, 1]), float(J[0, 2]), float(J[1, 2])]}
    return out


def _roots(aircraft, origin):
    """Each section's break - the point it turns about - and the axis its root's bending is taken about (body axes):
    a wing's at its root's mid-chord where it leaves the fuselage, bending about the body's x axis; the tail's and the
    nose's at their breaks on the fuselage's axis, bending about y (and the tail about z too)."""
    from .shape import breakup
    wings = [s for s in aircraft.surfaces if s.kind == "wing"]
    tails = [s for s in aircraft.surfaces if s.kind in ("htail", "fin", "vtail")]
    root = wings[0].sections[0]
    y_root = breakup._fuselage_half_width(aircraft, root.le[0] + 0.5 * root.chord) + 0.05
    x_mid = root.le[0] + 0.5 * root.chord
    x_nose = min(s.sections[0].le[0] for s in wings) - breakup.NOSE_AHEAD
    x_tail = min(s.sections[0].le[0] for s in tails) - breakup.TAIL_AHEAD if tails else None
    fus = [b for b in aircraft.bodies if b.kind == "fuselage"]

    def axis_z(x):
        if not fus:
            return 0.0
        _, top, bot, _, _ = fus[0].section(x)
        return float(0.5 * (top + bot))
    out = {
        "left wing": {"at": _body_axes([x_mid, -y_root, root.le[2]], origin).tolist(), "axes": [[1.0, 0.0, 0.0]]},
        "right wing": {"at": _body_axes([x_mid, y_root, root.le[2]], origin).tolist(), "axes": [[1.0, 0.0, 0.0]]},
        "nose": {"at": _body_axes([x_nose, 0.0, axis_z(x_nose)], origin).tolist(), "axes": [[0.0, 1.0, 0.0]]},
        "centre": None,
    }
    out["tail"] = None if x_tail is None else {"at": _body_axes([x_tail, 0.0, axis_z(x_tail)], origin).tolist(),
                                               "axes": [[0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]}
    return out


def _panels(aircraft, origin):
    """The skin: each surface's planform in panels (front and back halves of the chord, at most PANEL_SPAN along the
    span; mirrored where it is), each body as boxes PANEL_LENGTH long round its sections. Each a dict of its section,
    kind (surface | body), its corners (a surface's four, body axes) and thickness, or a body box's centre and half
    extents, and its area (m^2: a surface panel's both faces, a box's sides)."""
    out = []
    for s in aircraft.surfaces:
        for a, b in zip(s.sections[:-1], s.sections[1:]):
            la, lb = np.asarray(a.le, float), np.asarray(b.le, float)
            length = math.hypot(lb[1] - la[1], lb[2] - la[2])
            n = max(1, int(math.ceil(length / PANEL_SPAN)))
            for k in range(n):
                t0, t1 = k / n, (k + 1) / n
                e0, e1 = la + t0 * (lb - la), la + t1 * (lb - la)
                c0, c1 = a.chord + t0 * (b.chord - a.chord), a.chord + t1 * (b.chord - a.chord)
                tc = 0.5 * (a.airfoil.thickness_ratio + b.airfoil.thickness_ratio)
                for f0, f1 in ((0.0, 0.5), (0.5, 1.0)):
                    quad = np.array([e0 + [f0 * c0, 0, 0], e0 + [f1 * c0, 0, 0], e1 + [f1 * c1, 0, 0], e1 + [f0 * c1, 0, 0]])
                    for side in ((1.0, -1.0) if s.mirror else (1.0,)):
                        q = quad * np.array([1.0, side, 1.0])
                        area = 0.5 * np.linalg.norm(np.cross(q[2] - q[0], q[3] - q[1]))
                        out.append({"surface": s.name, "points": q, "thickness": tc * 0.5 * (c0 + c1), "area": 2.0 * area,
                                    "kind": "surface"})
    for body in aircraft.bodies:
        n = max(1, int(math.ceil((body.x[-1] - body.x[0]) / PANEL_LENGTH)))
        for k in range(n):
            x0 = body.x[0] + (body.x[-1] - body.x[0]) * k / n
            x1 = body.x[0] + (body.x[-1] - body.x[0]) * (k + 1) / n
            xm = 0.5 * (x0 + x1)
            w, top, bot, yc, _ = body.section(xm)
            w, top, bot, yc = float(w), float(top), float(bot), float(yc)
            if w <= 1e-3 or top - bot <= 1e-3:
                continue
            for sign in body.copies():
                c = np.array([xm, sign * yc, 0.5 * (top + bot)])
                half = np.array([0.5 * (x1 - x0), 0.5 * w, 0.5 * (top - bot)])
                # (the box's sides, an ellipse's girth: Ramanujan's)
                a_, b_ = half[1], half[2]
                girth = math.pi * (3 * (a_ + b_) - math.sqrt((3 * a_ + b_) * (a_ + 3 * b_)))
                out.append({"body": body.name, "centre": c, "half": half, "area": girth * (x1 - x0), "kind": "body"})
    centres = [p["points"].mean(axis=0) if p["kind"] == "surface" else p["centre"] for p in out]
    where = _sections_of(aircraft, centres) if out else []
    # (a pod, a nacelle, a launcher is on what it is mounted on, all of it: each copy's boxes on its middle's section)
    bodies = {b.name: b for b in aircraft.bodies}
    for i, p in enumerate(out):
        b = bodies.get(p.get("body"))
        if b is None or b.kind == "fuselage":
            continue
        _, top, bot, yc, _ = b.section(0.5 * (b.x[0] + b.x[-1]))
        mid = np.array([0.5 * (b.x[0] + b.x[-1]), np.sign(p["centre"][1] or 1.0) * abs(float(yc)) if b.mirror else float(yc),
                        0.5 * float(top + bot)])
        where[i] = _sections_of(aircraft, [mid], mounted=True)[0]
    panels = []
    for p, s in zip(out, where or []):
        if p["kind"] == "surface":
            panels.append({"section": s, "kind": "surface", "surface": p["surface"],
                           "corners": [_body_axes(q, origin).tolist() for q in p["points"]],
                           "thickness_m": float(p["thickness"]), "area_m2": float(p["area"])})
        else:
            panels.append({"section": s, "kind": "body", "body": p["body"], "centre": _body_axes(p["centre"], origin).tolist(),
                           "half": [float(v) for v in p["half"]], "area_m2": float(p["area"])})
    return panels


def _strength(roots, shares, masses, design_kg, n):
    """Each section's ultimate root bending (N m, about each axis; n the ultimate load factor): a wing's lift less
    the relief of its structure alone (its tanks empty - the case its root is sized by); the nose's and the tail's the
    larger of their lift's and their weight's (two terms that nearly cancel in a symmetric pull, so neither is left
    unsized by the other)."""
    lift = _bending(roots, shares, masses, {}, design_kg, n, parts=("lift",))
    weight = _bending(roots, shares, masses, {}, design_kg, n, parts=("weight",))
    both = _bending(roots, shares, masses, {}, design_kg, n)
    out = {}
    for s in roots:
        if roots[s] is None:
            out[s] = None
        elif s in ("left wing", "right wing"):
            out[s] = [abs(v) for v in both[s]]
        else:
            out[s] = [max(abs(a), abs(b)) for a, b in zip(lift[s], weight[s])]
    return out


def _bending(roots, shares, masses, tanks, design_kg, n, parts=("lift", "weight")):
    """Each section's root bending (N m, about each of its axes) flown symmetrically at load factor n with the
    aircraft at design_kg: its share of the lift n W g at its centre of pressure, less n g times its mass (with its
    tanks' fuel) at its centre of mass - the world's rule (src/damage) at that state."""
    out = {}
    for s, r in roots.items():
        if r is None:
            out[s] = None
            continue
        at = np.asarray(r["at"], float)
        sh, ms = shares[s], masses[s]
        lift = np.array([0.0, 0.0, -sh["normal_share"] * n * design_kg * G0])   # up: -z
        m = ms["mass_kg"] + sum(t["capacity_kg"] for t in tanks.get(s, []))
        cg = np.asarray(ms["cg"], float)
        if tanks.get(s):
            fuel = sum(t["capacity_kg"] for t in tanks[s])
            cg = (ms["mass_kg"] * cg + sum(t["capacity_kg"] * np.asarray(t["centre"], float) for t in tanks[s])) / max(m, 1e-9)
        inertia = np.array([0.0, 0.0, n * m * G0])                          # its weight times n: down, +z
        M = np.zeros(3)
        if sh["normal_cp"] is not None and "lift" in parts:
            M += np.cross(np.asarray(sh["normal_cp"], float) - at, lift)
        if "weight" in parts:
            M += np.cross(cg - at, inertia)
        out[s] = [float(M @ np.asarray(ax, float)) for ax in r["axes"]]
    return out


def damage_data(aircraft, model=None, mass_model=None, origin=None):
    """The aircraft's damage data (a dict, as written) - None for a design without a wing (no breakup sections)."""
    from . import vulnerability
    from .aero.model import AeroModel
    from .mass import MassModel
    from .shape import breakup
    if not any(s.kind == "wing" for s in aircraft.surfaces):
        return None
    mass_model = mass_model or MassModel(aircraft)
    if origin is None:
        origin = mass_model.empty()["cg"]
    origin = np.asarray(origin, float)
    model = model or AeroModel(aircraft)
    fc = aircraft.spec.get("flight_control", {})
    if aircraft.name in LOAD_LIMITS:
        n_max, n_min, limits_basis = LOAD_LIMITS[aircraft.name]
    elif "n_max" in fc:
        n_max = float(fc["n_max"])
        n_min = float(fc.get("n_min", -0.4 * n_max))
        limits_basis = "the design's flight control envelope (its structure's limits)" + ("" if "n_min" in fc else
                                                                                           "; n_min -0.4 of n_max, an estimate")
    else:
        n_max, n_min, limits_basis = 3.8, -1.52, "estimate: FAR 23.337's normal category (no figure for the type)"
    design_kg = float(mass_model.mtow)

    comps = vulnerability.components(aircraft)
    components = []
    for c in comps:
        d = {"kind": c["kind"], "shape": c["shape"], "section": c["section"], "name": c["name"],
             "centre": _body_axes(c["centre"], origin).tolist(), "half": [float(h) for h in c["half"]]}
        for key in ("engine", "tank", "channel"):
            if key in c:
                d[key] = c[key]
        if c["kind"] == "fuel":
            d["fuel_kg"] = float(c.get("volume", 0.0)) * vulnerability.FUEL_DENSITY[vulnerability._fuel_kind(aircraft)]
        components.append(d)
    # each section's tanks: JSBSim's tank index, its capacity, where its fuel is (its boxes' middle, by volume)
    tank_specs = aircraft.spec.get("mass", {}).get("tank", [])
    tanks = {}
    for i, t in enumerate(tank_specs):
        boxes = [c for c in components if c["kind"] == "fuel" and c.get("tank") == i]
        if not boxes or float(t.get("capacity", 0.0)) <= 0.0:
            continue
        kg = np.array([b["fuel_kg"] for b in boxes])
        centre = (kg @ np.array([b["centre"] for b in boxes])) / max(kg.sum(), 1e-9)
        # (a tank's boxes on more than one section: the one holding most of it)
        sec = max({b["section"] for b in boxes}, key=lambda s: sum(b["fuel_kg"] for b in boxes if b["section"] == s))
        tanks.setdefault(sec, []).append({"index": i, "name": str(t.get("name", "tank")), "capacity_kg": float(t["capacity"]),
                                          "centre": centre.tolist()})
    shares = _shares(aircraft, model, origin)
    masses = _masses(aircraft, mass_model, origin)
    roots = _roots(aircraft, origin)
    for name in roots:  # (a section with nothing in it - a design with no nose ahead of its wing - has no break)
        if masses[name]["mass_kg"] <= 0.0 and abs(shares[name]["normal_share"]) < 1e-9:
            roots[name] = None
    up = _strength(roots, shares, masses, design_kg, 1.5 * n_max)
    down = _strength(roots, shares, masses, design_kg, 1.5 * n_min)
    panels = _panels(aircraft, origin)
    skin = {s: sum(p["area_m2"] for p in panels if p["section"] == s) for s in breakup.SECTIONS}
    # the store stations on each (what is carried there parts with it), by breakup's rule for what is mounted
    from . import carriage
    on = {s: [] for s in breakup.SECTIONS}
    st = carriage.stations(aircraft)
    if st:
        where = _sections_of(aircraft, [s["at"] for s in st], mounted=True)
        for s, w in zip(st, where or []):
            on[w].append({"number": int(s["number"]), "at": _body_axes(s["at"], origin).tolist()})
    sections = []
    for s in breakup.SECTIONS:
        entry = {"name": s, **masses[s], "tanks": tanks.get(s, []), "stations": on[s], **shares[s], "root": roots[s],
                 "skin_area_m2": skin[s], "skin_kg": SKIN_SHARE * masses[s]["mass_kg"]}
        if roots[s] is not None:
            # the bending it was built to carry, each way about each axis: the larger magnitude of the two pulls
            entry["strength_nm"] = [max(abs(a), abs(b)) for a, b in zip(up[s], down[s])]
            if s == "tail" and shares[s]["side_cp"] is not None:
                # and the tail's yaw (no symmetric pull loads it): its fins' side force at their own stall (CL 1.0)
                # at the corner speed, where the aircraft pulls n_max at CL 1.0 - n_max W S_fins / S - times 1.5 (an
                # estimate: a fin is sized by rudder and sideslip manoeuvres the rule does not fly)
                fins = sum(f.area for f in aircraft.surfaces if f.kind in ("fin", "vtail"))
                side = np.array([0.0, 1.5 * n_max * design_kg * G0 * fins / max(model.S, 1e-9), 0.0])
                arm = np.asarray(shares[s]["side_cp"], float) - np.asarray(roots[s]["at"], float)
                entry["strength_nm"][1] = max(entry["strength_nm"][1], abs(float(np.cross(arm, side)[2])))
        sections.append(entry)
    return {
        "aircraft": aircraft.name,
        "frame": "body axes from the model's origin (the empty CG): x forward, y right, z down; m, kg",
        "origin_design_m": origin.tolist(),
        "design_mass_kg": design_kg,
        "n_limit": [n_min, n_max],
        "n_ultimate": [1.5 * n_min, 1.5 * n_max],
        "basis": {"limits": limits_basis, "strength": "the design's ultimate load (1.5 x the flight control's n limits) at its design mass, flown "
                              "symmetrically; tails and nose by the same rule (estimate)",
                  "skin_share": SKIN_SHARE, "shares": "hangar's strip model: dN/dalpha and dY/dbeta per section"},
        "sections": sections,
        "panels": panels,
        "components": components,
    }


def write(aircraft, path, origin, model=None):
    """<name>.damage.json beside the model (None written for a design without a wing: the file removed)."""
    import os
    data = damage_data(aircraft, model=model, origin=origin)
    if data is None:
        if os.path.isfile(path):
            os.remove(path)
        return None

    def rounded(x):
        if isinstance(x, float):
            return round(x, 5)
        if isinstance(x, dict):
            return {k: rounded(v) for k, v in x.items()}
        if isinstance(x, (list, tuple)):
            return [rounded(v) for v in x]
        return x
    text = json.dumps(rounded(data), indent=1) + "\n"
    if os.path.isfile(path):
        with open(path, encoding="utf-8") as f:
            if f.read() == text:
                return data  # (as it was: the file left alone)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    return data
