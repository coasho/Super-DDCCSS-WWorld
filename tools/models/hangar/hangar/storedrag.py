"""Each store's own aerodynamics, as the aircraft carrying it feels them (ADR-30, MS-7g) - isolated, on its card:

- `drag_area_m2`: its drag coefficient times its reference area (CD S, m^2) below Mach 0.8 at a reference Reynolds
  number;
- `munk_volume_m3`: its body's unstable moment in a crossflow, per unit of dynamic pressure and of sin(2 x angle) (m^3);
- `side_slope_m2`, `side_arm_m`: its fins' and base's force across it per radian of crossflow (m^2), and how far
  behind its CG that force acts (m).

The world adds its carriage's interference, its pylon's and launcher's drag and the transonic rise
(ms::Host::carriageDrag); a store flying free takes the same figures (storemodels.py).

    python -m hangar storedrag        writes every store card's figures (and prints their build-up)

One build-up for every store, from the geometry hangar already holds for it - a missile's design
(aircraft/<name>/<name>.toml, its card named there), stores.py's shape for a bomb, a tank's or a pod's card's shape
(stores.py's tank and pod profiles) - in the classical way:
- its body's drag: a streamlined body of revolution's, on its frontal area, Cf (3 L/D + 4.5 (D/L)^0.5 + 21 (D/L)^2)
  (Hoerner, Fluid-Dynamic Drag, 1965, eq. 6-26: friction with its forebody's pressure drag);
- its base's: what its tail leaves blunt (its profile's last radius; a pod's rounded tail's wake, stores.POD_WAKE),
  0.029 / sqrt(CD_body) on the base's area (Hoerner eq. 3-36, a base behind a turbulent body);
- its fins' drag: friction on both faces of each panel, its form factor 1 + 2 t/c + 60 (t/c)^4 (a lattice fin's walls
  twice a plain fin's planform: an estimate);
- the friction a turbulent flat plate's, 0.455 / (log10 Re)^2.58 (Schlichting), at REYNOLDS_PER_M - 220 m/s at
  5,000 m; the world's air changes it by (Re / Re_ref)^-0.2 at most a few tens of percent: not followed (said so);
- its body's unstable moment: Munk's (NACA Report 184, 1924) - a body of revolution across a flow is turned broadside
  by M = q (k2 - k1) V sin(2 alpha), V its volume, k1 and k2 Lamb's apparent-mass factors of the spheroid of its
  fineness (LAMB below); a couple, no force;
- its base's force across it: slender-body theory's 2 x the base's area per radian, at the base;
- its fins' force across it: one pair's (a cruciform's four give the same in any plane), the exposed pair's lift
  slope by Helmbold's 2 pi A / (2 + sqrt(4 + A^2)), carried over onto the body by (1 + d / b)^2 (Pitts, Nielsen &
  Kaattari, NACA Report 1307), at the quarter chord of its mean chord. A lattice fin as a plain one of its outline (an
  estimate).
"""
import json
import math
import os
import tomllib

from . import pipeline, stores

REYNOLDS_PER_M = 1.0e7       # 220 m/s at 5,000 m (rho 0.736 kg/m^3, mu 1.63e-5 Pa s): a fighter's cruise
THICKNESS = {"biconvex4": 0.04, "plate": 0.03}  # its fins' t/c by the design's airfoil (a plate's: an estimate)
# Lamb's apparent-mass factors of a prolate spheroid moving along (k1) and across (k2) its axis, by its fineness
# (Lamb, Hydrodynamics; the table in Munk's NACA Report 184)
LAMB = [(1.0, 0.500, 0.500), (1.5, 0.305, 0.621), (2.0, 0.209, 0.702), (2.51, 0.156, 0.763), (2.99, 0.122, 0.803),
        (3.99, 0.082, 0.860), (4.99, 0.059, 0.895), (6.01, 0.045, 0.918), (6.97, 0.036, 0.933), (8.01, 0.029, 0.945),
        (9.02, 0.024, 0.954), (9.97, 0.021, 0.960)]


def cf(reynolds):
    return 0.455 / math.log10(reynolds) ** 2.58


def lamb(fineness):
    """k2 - k1 of the spheroid of that fineness (1 beyond the table's: a needle)."""
    f = max(fineness, 1.0)
    for (f0, a0, b0), (f1, a1, b1) in zip(LAMB, LAMB[1:]):
        if f <= f1:
            t = (f - f0) / (f1 - f0)
            return (b0 + t * (b1 - b0)) - (a0 + t * (a1 - a0))
    # beyond fineness 10: towards 1 as the spheroid's exact factors do (k2 - k1 = 1 - 3.9 / f^2 fits the table's end)
    return 1.0 - (1.0 - (LAMB[-1][2] - LAMB[-1][1])) * (LAMB[-1][0] / f) ** 2


def _body(L, D, base_d):
    """A body of revolution's CD S (m^2): friction and forebody, its base."""
    frontal = math.pi * D * D / 4.0
    cd = cf(REYNOLDS_PER_M * L) * (3.0 * L / D + 4.5 * (D / L) ** 0.5 + 21.0 * (D / L) ** 2)
    base = 0.029 / math.sqrt(cd) * math.pi * base_d * base_d / 4.0 if base_d > 0.0 else 0.0
    return cd * frontal, base, frontal


def _fins(panels):
    """Fins' CD S (m^2): each (planform m^2, mean chord m, t/c, lattice)."""
    s = 0.0
    for area, chord, tc, lattice in panels:
        ff = 1.0 + 2.0 * tc + 60.0 * tc ** 4
        s += cf(REYNOLDS_PER_M * chord) * ff * 2.0 * area * (2.0 if lattice else 1.0)
    return s


def _volume(profile):
    """A body of revolution's volume (m^3): its profile's (x, radius), frustum by frustum."""
    v = 0.0
    for (x0, r0), (x1, r1) in zip(profile, profile[1:]):
        v += math.pi * (x1 - x0) * (r0 * r0 + r0 * r1 + r1 * r1) / 3.0
    return v


def _side(sets, profile, cg, r_base):
    """The force across it per radian (m^2) and where it acts behind the CG (m): each fin set's pair - (exposed span of
    one panel m, root chord m, tip chord m, the root's leading edge x m, its sweep back at the tip m, body radius
    there m) - and the base."""
    slope = moment = 0.0
    for span, root, tip, x_le, sweep, radius in sets:
        if span <= 0.0:
            continue
        area = span * (root + tip)                    # the exposed pair's
        aspect = (2.0 * span) ** 2 / area
        cl = 2.0 * math.pi * aspect / (2.0 + math.sqrt(4.0 + aspect * aspect))
        b = 2.0 * (span + radius)
        k = cl * area * (1.0 + 2.0 * radius / b) ** 2
        taper = tip / root if root > 0.0 else 1.0
        mac = 2.0 / 3.0 * root * (1.0 + taper + taper * taper) / (1.0 + taper)
        y_mac = span / 3.0 * (1.0 + 2.0 * taper) / (1.0 + taper)
        x = x_le + sweep * y_mac / span + 0.25 * mac
        slope += k
        moment += k * (x - cg)
    base = 2.0 * math.pi * r_base * r_base
    slope += base
    moment += base * (profile[-1][0] - cg)
    return slope, (moment / slope if slope > 0.0 else 0.0)


def _design(card_id):
    """The design whose card this is (its toml), or None."""
    from . import storeaero
    return storeaero.design_of(card_id)


def _from_design(d):
    body = next(b for b in d["body"] if b.get("kind") == "fuselage")
    st = body["stations"]
    profile = [(s["x"] - st[0]["x"], 0.25 * (s["w"] + s["top"] - s["bottom"])) for s in st]
    L, D = profile[-1][0], 2.0 * max(r for _, r in profile)
    panels, sets, seen = [], [], set()
    for srf in d.get("surface", []):
        secs = srf["sections"]
        n = 2 if srf.get("mirror") else 1
        tc = THICKNESS.get(srf.get("airfoil", "plate"), 0.03)
        for a, b in zip(secs, secs[1:]):
            span = math.hypot(b["le"][1] - a["le"][1], b["le"][2] - a["le"][2])
            chord = 0.5 * (a["chord"] + b["chord"])
            panels.append((n * span * chord, chord, tc, "lattice" in srf))
        # one pair of each ring of fins (an upper and a lower surface, each mirrored, are one cruciform ring)
        a, b = secs[0], secs[-1]
        ring = round(a["le"][0], 3)
        if ring in seen:
            continue
        seen.add(ring)
        span = math.hypot(b["le"][1] - a["le"][1], b["le"][2] - a["le"][2])
        radius = math.hypot(a["le"][1], a["le"][2])
        sets.append((span, a["chord"], b["chord"], a["le"][0] - st[0]["x"], b["le"][0] - a["le"][0], radius))
    cg = d.get("mass", {}).get("empty_cg", [0.5 * L])[0] - st[0]["x"]
    # its base: where its body ends - or its wake's, where its design says its rounded tail's flow leaves it ([store] wake)
    wake = d.get("store", {}).get("wake")
    return L, D, profile, panels, sets, cg, 0.5 * wake if wake else profile[-1][1]


def _from_shape(s):
    L, D = s["length"], s["diameter"]
    profile = [(x, r * D / 2.0) for x, r in s["profile"]]
    panels, sets = [], []
    f = s.get("fins")
    if f:
        chord = 0.5 * (f["root_chord"] + f["tip_chord"])
        panels.append((f["count"] * f["span"] * chord, chord, f["thickness"] / chord, False))
        radius = 0.5 * D * _radius_at(s["profile"], f["root_le"] + 0.5 * f["root_chord"])
        sets.append((f["span"], f["root_chord"], f["tip_chord"], f["root_le"], math.tan(math.radians(f["sweep_deg"])) * f["span"], radius))
    return L, D, profile, panels, sets, s.get("cg", 0.5 * L), s["base"] * D / 2.0 if "base" in s else profile[-1][1]


def _radius_at(profile, x):
    for (x0, r0), (x1, r1) in zip(profile, profile[1:]):
        if x <= x1:
            return r0 + (r1 - r0) * (x - x0) / (x1 - x0) if x1 > x0 else r1
    return profile[-1][1]


def build_up(card):
    """(CD S m^2, its parts) of the card's store, or None without its geometry."""
    d = _design(card["id"])
    if d and d.get("store", {}).get("shape") == "box":
        return None  # (a cargo store: a bluff box, its faces' drag its card's drag_cd - no body of revolution's build-up)
    if d:
        geo, src = _from_design(d), "its design"
    else:
        name = card.get("model", "")[7:] or card["id"].replace("-", "")
        if name in stores.STORES:
            geo, src = _from_shape(stores.STORES[name]), "stores.py"
        elif card.get("class") in ("tank", "pod"):
            geo, src = _from_shape(stores.tank_spec(card)), "its card, stores.py's %s profile" % card["class"]
        else:
            return None
    L, D, profile, panels, sets, cg, r_base = geo
    body, base, frontal = _body(L, D, 2.0 * r_base)
    fins = _fins(panels)
    volume = _volume(profile)
    k = lamb(L / D)
    slope, arm = _side(sets, profile, cg, r_base)
    return body + base + fins, {"body": body, "base": base, "fins": fins, "frontal": frontal, "L": L, "D": D, "from": src,
                                "volume": volume, "lamb": k, "munk": k * volume, "slope": slope, "arm": arm}


def write(cards_dir):
    out = []
    for name in sorted(os.listdir(cards_dir)):
        if not name.endswith(".json"):
            continue
        path = os.path.join(cards_dir, name)
        with open(path, encoding="utf-8") as f:
            card = json.load(f)
        r = build_up(card)
        if r is None:
            out.append((card["id"], None, None))
            continue
        cds, p = r
        P = card.setdefault("parameters", {})
        P["drag_area_m2"] = {
            "value": round(cds, 5), "unit": "m^2", "uncertainty": round(0.3 * cds, 5), "basis": "assumed",
            "note": ("the isolated store's CD S below Mach 0.8 at a Reynolds number of 1e7 per metre, built up from %s "
                     "(hangar storedrag.py: Hoerner's body of revolution %.5f, its base %.5f, its fins' friction %.5f m^2): "
                     "an estimate by one method for every store" % (p["from"], p["body"], p["base"], p["fins"])),
        }
        P["munk_volume_m3"] = {
            "value": round(p["munk"], 5), "unit": "m^3", "uncertainty": round(0.3 * p["munk"], 5), "basis": "assumed",
            "note": ("its body's unstable moment across a flow, q x this x sin(2 x angle) (Munk, NACA Report 184): its volume "
                     "%.4f m^3 from %s, times Lamb's k2 - k1 = %.3f for its fineness %.1f (hangar storedrag.py): an estimate "
                     "by one method for every store" % (p["volume"], p["from"], p["lamb"], p["L"] / p["D"])),
        }
        P["side_slope_m2"] = {
            "value": round(p["slope"], 5), "unit": "m^2", "uncertainty": round(0.3 * p["slope"], 5), "basis": "assumed",
            "note": ("its fins' and base's force across it per radian of crossflow, over the dynamic pressure (hangar "
                     "storedrag.py: one fin pair's Helmbold slope carried onto the body, slender-body theory's base): an "
                     "estimate by one method for every store"),
        }
        P["side_arm_m"] = {
            "value": round(p["arm"], 4), "unit": "m", "uncertainty": round(0.1 * p["L"], 4), "basis": "assumed",
            "note": "where that force acts, behind the store's CG (hangar storedrag.py: its fins' quarter chords, its base)",
        }
        pipeline.write_text(path, json.dumps(card, indent=2, ensure_ascii=False) + "\n")
        out.append((card["id"], cds, p))
    return out


def main():
    rows = write(os.path.join(pipeline.repo_root(), "assets", "stores"))
    print("%-20s %9s %9s %8s %8s %8s %9s %9s %7s  %s" % ("store", "CD S m2", "CD front", "body", "base", "fins", "munk m3", "side m2", "arm m", "from"))
    for cid, cds, p in rows:
        if cds is None:
            print("%-20s (no geometry)" % cid)
        else:
            print("%-20s %9.5f %9.4f %8.5f %8.5f %8.5f %9.4f %9.4f %7.3f  %s" % (cid, cds, cds / p["frontal"], p["body"], p["base"], p["fins"],
                                                                         p["munk"], p["slope"], p["arm"], p["from"]))
