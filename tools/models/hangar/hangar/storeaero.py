"""A store's aerodynamics by one component build-up for every store (ADR-30, MS-7e1 and MS-7g) - a missile from its
design (aircraft/<name>/<name>.toml), a bomb from stores.py's shape, a tank from its card's (stores.py's tank
profile): what its own flight model flies on (storemodels.py) and what the aircraft carrying it feels (storedrag.py's
card figures, the world's forces at its station). Estimates, by published methods:

- the body's lift by slender-body theory (Munk; Barrowman 1967, "The Practical Calculation of the Aerodynamic
  Characteristics of Slender Finned Vehicles"): each change of its section adds 2 dA / S per radian at its place - a
  nose's 2, a boattail's loss - so a body closed at both ends carries a pure unstable couple (an unfinned tank
  tumbles);
- its crossflow at incidence (Jorgensen, NASA TN D-6996, 1973): eta Cdc sin^2(alpha) on its planform, eta 0.7 for
  its length, Cdc 1.2; its fins' plates 1.2 on theirs;
- each fin's lift (Barrowman's, with Diederich's compressibility as OpenRocket uses it):
  2 pi AR (Af / S) / (2 + sqrt(4 + (beta AR / cos sweep)^2)) below Mach 0.9; above Mach 1.3 linear theory's
  4 / beta (1 - 1 / (2 beta AR)), a slender wing's pi AR / 2 where that is less; a line between. On its body by
  K = 1 + r / (s + r). A pair canted from the horizontal gives the pitch plane cos^2 of its cant, the yaw plane
  sin^2 (an X of four: a planar pair's in each). A lattice fin twice a plate's (an estimate);
- a fin's centre of pressure a quarter of its mean chord back to Mach 0.8, half by Mach 2;
- damping: each lifting part's slope times its arm squared (Barrowman), the body's crossflow turning with it
  (quadratic in its rate), the fins' roll damping.
Not followed: a canard's downwash on the fins behind it, the body's vortices at incidence, a motor's plume.
"""
import math
import os
import tomllib

from . import pipeline, stores

ETA_CDC = 0.7 * 1.2   # Jorgensen: a finite body's crossflow drag
PLATE = 1.2           # a fin broadside
MACH = [0.0, 0.6, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.6, 2.0, 2.5, 3.0, 4.0, 5.0]
THICKNESS = {"biconvex4": 0.04, "plate": 0.03}


def design_of(card_id, *, with_path=False):
    """The design whose card this is (its toml), or None."""
    root = os.path.join(pipeline.repo_root(), "aircraft")
    # (a store's design is filed under its card's id without its hyphens - looked at first; any other is searched for)
    first = card_id.replace("-", "")
    for name in [first] + sorted(n for n in os.listdir(root) if n != first and n.casefold() != ".codex"):
        path = os.path.join(root, name, name + ".toml")
        if os.path.isfile(path):
            with open(path, "rb") as f:
                d = tomllib.load(f)
            if d.get("aircraft", {}).get("card") == card_id:
                return (d, path) if with_path else d
    return (None, None) if with_path else None


def geometry(card):
    """Its shape: {x, r (m, from its nose), fins, cg, length, diameter, from}; None without one."""
    d, path = design_of(card["id"], with_path=True)
    if d and d.get("store", {}).get("shape") == "box":
        return None  # (a cargo store: no body of revolution)
    if d:
        body = next(b for b in d["body"] if b.get("kind") == "fuselage")
        st = body["stations"]
        x = [s["x"] - st[0]["x"] for s in st]
        r = [0.25 * (s["w"] + s["top"] - s["bottom"]) for s in st]
        fins = []
        for srf in d.get("surface", []):
            secs = srf["sections"]
            a, b = secs[0], secs[-1]
            dy, dz = b["le"][1] - a["le"][1], b["le"][2] - a["le"][2]
            s = math.hypot(dy, dz)
            if s <= 0.0:
                continue
            fins.append(dict(x=a["le"][0] - st[0]["x"], cr=a["chord"], ct=b["chord"], xt=b["le"][0] - a["le"][0], s=s,
                             r=math.hypot(a["le"][1], a["le"][2]), cant2=dz * dz / (s * s), n=2 if srf.get("mirror") else 1,
                             tc=THICKNESS.get(srf.get("airfoil", "plate"), 0.03), lattice="lattice" in srf))
        return dict(x=x, r=r, fins=fins, cg=d["mass"]["empty_cg"][0] - st[0]["x"], length=x[-1], diameter=2.0 * max(r),
                    design=d, design_path=path, source="its design")
    name = (card.get("model", "")[7:] or card["id"]).replace("-", "")
    if name in stores.STORES:
        s, src = stores.STORES[name], "stores.py"
    elif card.get("class") == "tank":
        s, src = stores.tank_spec(card), "its card, stores.py's tank profile"
    else:
        return None
    L, D = s["length"], s["diameter"]
    x = [p[0] for p in s["profile"]]
    r = [p[1] * D / 2.0 for p in s["profile"]]
    fins = []
    f = s.get("fins")
    if f and f["count"]:
        rr = float(_interp(f["root_le"] + f["root_chord"] / 2.0, x, r))
        xt = math.tan(math.radians(f["sweep_deg"])) * f["span"]
        for k in range(f["count"]):
            a = math.radians(f["roll_deg"]) + 2.0 * math.pi * k / f["count"]
            fins.append(dict(x=f["root_le"], cr=f["root_chord"], ct=f["tip_chord"], xt=xt, s=f["span"], r=rr,
                             cant2=math.sin(a) ** 2, n=1, tc=f["thickness"] / (0.5 * (f["root_chord"] + f["tip_chord"])), lattice=False))
    return dict(x=x, r=r, fins=fins, cg=s["cg"], length=L, diameter=D, design=None, source=src)


def _interp(v, xs, ys):
    if v <= xs[0]:
        return ys[0]
    for i in range(1, len(xs)):
        if v <= xs[i]:
            t = (v - xs[i - 1]) / (xs[i] - xs[i - 1]) if xs[i] > xs[i - 1] else 0.0
            return ys[i - 1] + t * (ys[i] - ys[i - 1])
    return ys[-1]


def _fin_slope(f, S, mach):
    """One fin's normal-force slope on S (per radian), alone."""
    af = 0.5 * f["s"] * (f["cr"] + f["ct"])
    ar = 2.0 * f["s"] ** 2 / af
    sweep = math.atan2(f["xt"] + 0.5 * f["ct"] - 0.5 * f["cr"], f["s"])

    def sub(m):
        beta = math.sqrt(max(1.0 - m * m, 0.0))
        return 2.0 * math.pi * ar * (af / S) / (2.0 + math.sqrt(4.0 + (beta * ar / math.cos(sweep)) ** 2))

    def sup(m):
        beta = math.sqrt(m * m - 1.0)
        slender = 0.5 * math.pi * ar
        linear = 4.0 / beta * (1.0 - 1.0 / (2.0 * beta * ar)) if beta * ar > 1.0 else slender
        return min(slender, linear) * af / S

    if mach <= 0.9:
        k = sub(mach)
    elif mach >= 1.3:
        k = sup(mach)
    else:
        t = (mach - 0.9) / 0.4
        k = (1.0 - t) * sub(0.9) + t * sup(1.3)
    return k * (2.0 if f["lattice"] else 1.0)


def _fin_cp(f, mach):
    """Its centre of pressure (m from the nose) and its spanwise place (m from the axis)."""
    cr, ct = f["cr"], f["ct"]
    mac = 2.0 / 3.0 * (cr * cr + cr * ct + ct * ct) / (cr + ct)
    lead = f["x"] + f["xt"] * (cr + 2.0 * ct) / (3.0 * (cr + ct))
    frac = 0.25 if mach <= 0.8 else 0.5 if mach >= 2.0 else 0.25 + 0.25 * (mach - 0.8) / 1.2
    y = f["r"] + f["s"] * (cr + 2.0 * ct) / (3.0 * (cr + ct))
    return lead + frac * mac, y


def build_up(geo, cg=None):
    """Its coefficients on its frontal area S and its diameter d, moments about `cg` (m from the nose; its own given
    none): per Mach in MACH the pitch and yaw planes' normal-force slopes and their moments' (per radian), the
    damping; and the crossflow's, the same at every Mach."""
    x, r = geo["x"], geo["r"]
    d = geo["diameter"]
    S = math.pi * d * d / 4.0
    ref = geo["cg"] if cg is None else cg
    # the body: slender-body lift where its section changes
    cn_b = cm_b = cq_b = 0.0
    for i in range(1, len(x)):
        da = math.pi * (r[i] ** 2 - r[i - 1] ** 2)
        xm = 0.5 * (x[i] + x[i - 1])
        cn_b += 2.0 * da / S
        cm_b += -2.0 * da / S * (xm - ref) / d
    cq_b = -2.0 * math.pi * r[-1] ** 2 / S * ((x[-1] - ref) / d) ** 2  # (slender body: its base's)
    # its crossflow: the planform and its place; the turning body's (the integral of 2 r |x - ref|^3)
    plan = xplan = turn = 0.0
    for i in range(1, len(x)):
        w = (r[i] + r[i - 1]) * (x[i] - x[i - 1])
        xm = 0.5 * (x[i] + x[i - 1])
        plan += w
        xplan += w * xm
        turn += w * abs(xm - ref) ** 3
    xplan = xplan / plan if plan > 0.0 else ref
    out = dict(S=S, d=d, ref=ref, mach=list(MACH), cna=[], cma=[], cyb=[], cnb=[], cmq=[], cnr=[], clp=[], cn_body=cn_b, plan=plan)
    for m in MACH:
        cna, cma, cmq, cyb, cnb, cnr, clp = cn_b, cm_b, cq_b, cn_b, cm_b, cq_b, 0.0
        for f in geo["fins"]:
            one = _fin_slope(f, S, m) * (1.0 + f["r"] / (f["s"] + f["r"])) * f["n"]
            xf, yf = _fin_cp(f, m)
            arm = (xf - ref) / d
            p, y = one * (1.0 - f["cant2"]), one * f["cant2"]
            cna += p
            cma += -p * arm
            cmq += -2.0 * p * arm * arm
            cyb += y
            cnb += -y * arm
            cnr += -2.0 * y * arm * arm
            clp += -2.0 * one * (yf / d) ** 2
        out["cna"].append(cna), out["cma"].append(cma), out["cmq"].append(cmq)
        out["cyb"].append(cyb), out["cnb"].append(cnb), out["cnr"].append(cnr), out["clp"].append(min(clp, -0.01))
    # crossflow: the body's and the fins' plates', each plane's
    cross_p, cross_pm, cross_y, cross_ym = ETA_CDC * plan / S, -ETA_CDC * plan / S * (xplan - ref) / d, 0.0, 0.0
    cross_y, cross_ym = cross_p, cross_pm
    for f in geo["fins"]:
        af = 0.5 * f["s"] * (f["cr"] + f["ct"]) * f["n"]
        xc = f["x"] + 0.5 * f["xt"] + 0.25 * (f["cr"] + f["ct"])
        cross_p += PLATE * af * (1.0 - f["cant2"]) / S
        cross_pm += -PLATE * af * (1.0 - f["cant2"]) / S * (xc - ref) / d
        cross_y += PLATE * af * f["cant2"] / S
        cross_ym += -PLATE * af * f["cant2"] / S * (xc - ref) / d
    out.update(cross_n=cross_p, cross_m=cross_pm, cross_y=cross_y, cross_ym=cross_ym, turn=0.5 * ETA_CDC * turn)
    return out


def coefficients(geo, card_id):
    """Use the explicitly selected offline derivative table, or the existing build-up."""
    options = (geo.get("design") or {}).get("store", {})
    if "aero_table" not in options:
        return build_up(geo)
    from . import storetables
    return storetables.load(geo, card_id, options["aero_table"])


def summary(geo, selected=None):
    """What a card holds of it (below Mach 0.8, about its own CG): its normal-force slope's area and its centre's
    place - the pitch plane's and the yaw plane's."""
    b = build_up(geo) if selected is None else selected
    S, d = b["S"], b["d"]

    def at(key):
        if not b["mach"][0] <= 0.6 <= b["mach"][-1]:
            return None
        if 0.6 in b["mach"]:
            return b[key][b["mach"].index(0.6)]
        return _interp(0.6, b["mach"], b[key])

    cn, cm, cy, cnb = (at(key) for key in ("cna", "cma", "cyb", "cnb"))
    scale = lambda value, factor: None if value is None else value * factor

    def place(cn, cm):
        if cn is None or cm is None:
            return None
        return (b["ref"] - cm / cn * d) if abs(cn) > 1e-6 else b["ref"]

    return dict(normal_area_m2=scale(cn, S), normal_cp_m=place(cn, cm), normal_moment_m3=scale(cm, S * d),
                side_area_m2=scale(cy, S), side_cp_m=place(cy, cnb), side_moment_m3=scale(cnb, S * d),
                cross_area_m2=b["cross_n"] * S, cg_m=b["ref"])
