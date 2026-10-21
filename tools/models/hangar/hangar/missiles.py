"""The air-to-air missiles as hangar designs (ADR-30, MS-7e1), written by one rule for every one of them.

    python -m hangar missiles [<name> ...]    writes aircraft/<name>/<name>.toml (+ paint.toml) for each

Each missile's published figures come from its store card (assets/stores/<card>.json): length, diameter, span (tip to
tip across opposite fins) and mass; the design holds them. What no card gives - the nose's shape and length, each
surface's place, chords and sweep, the motor's nozzle and its burn's phases, the CG - is read off photographs and
estimated, the same way for every nation's missiles (docs/hangar.md, Missiles): those numbers are marked assumed in the
file written, with what they were estimated from. The written file is the design's record; this table is how it was
made, so a better source is entered here and the file written again.

A row: the card; the nose (`radome`: a tangent ogive `nose` calibres long; `dome`: an infrared seeker's glass dome on a
cone); an optional step in the body (`step`: where a wider motor section starts, and its diameter); the surface groups
front to back - each `canard`, `wing`, `strake` or `tail`, its root's leading edge `x` (m from the nose), root and tip
chords, and its control: `moving` (all-moving about its spindle), `aileron` (a trailing-edge aileron), `fixed`, or
`lattice` (a grid fin: drawn as a plate until hangar draws lattices); the one group whose tips set the card's span has
`span = true`, the others an exposed semi-span `s` (m); the motor's smoke and phases (s, kN).
"""
import json
import math
import os

from . import pipeline
from .files import open_write

R45 = math.sqrt(0.5)
# An AIM-9's rollerons: wheels some 8 cm across (assumed from photographs). A free-rolling tail's turns: two a kilometre
# (assumed - it is turned by its fins' small cant and the body's roll, not driven; no rate is published)
ROLLERON_RADIUS = 0.04
FREE_ROLL_PER_M = 0.002

US = {"bands": True, "colour": "#c4c8cc"}  # MIL-STD-1168's code: a yellow band (live warhead), a brown one (live motor)
NATO = {"bands": True, "colour": "#c4c8cc"}  # STANAG 2953 follows the same code (assumed for every NATO maker's round)
OTHER = {"bands": False, "colour": "#c4c8cc"}  # no published colour code found: no bands, the same light grey

# name: (card, maker's colours, nose, nose calibres, body extras, surfaces, motor)
MISSILES = {
    "aim120c": dict(card="aim-120c", paint=US, nose="radome", cal=2.6,
                    surfaces=[dict(role="wing", x=1.52, root=0.42, tip=0.12, s=None, span=True, control="fixed"),
                              dict(role="tail", x=3.30, root=0.30, tip=0.13, s=None, span=True, control="moving")],
                    motor=dict(smoke="reduced", phases=[(2.5, 22.0), (5.5, 7.0)], note="boost-sustain (the card: reduced-smoke HTPB)")),
    "aim120d": dict(card="aim-120d", paint=US, nose="radome", cal=2.6,
                    surfaces=[dict(role="wing", x=1.52, root=0.42, tip=0.12, span=True, control="fixed"),
                              dict(role="tail", x=3.30, root=0.30, tip=0.13, span=True, control="moving")],
                    motor=dict(smoke="reduced", phases=[(2.5, 22.0), (6.5, 7.0)], note="the C-7's airframe; its longer burn assumed")),
    "aim7m": dict(card="aim-7m", paint=US, nose="radome", cal=3.0,
                  surfaces=[dict(role="wing", x=1.30, root=0.62, tip=0.10, span=True, control="moving", pivot=0.5),
                            dict(role="tail", x=3.05, root=0.50, tip=0.18, s=0.30, control="fixed")],
                  motor=dict(smoke="aluminised", phases=[(3.0, 25.0), (9.0, 3.5)], note="the Mk 58's boost and sustain; aluminised (its dense trail in photographs)")),
    "aim9m": dict(card="aim-9m", paint=US, nose="dome", dome=0.36,
                  surfaces=[dict(role="canard", x=0.38, root=0.27, tip=0.03, s=0.13, control="moving", pivot=0.5),
                            dict(role="tail", x=2.38, root=0.40, tip=0.12, span=True, control="fixed", rollerons=True)],
                  motor=dict(smoke="reduced", phases=[(5.0, 11.0)], note="the Mk 36 Mod 11, reduced smoke (the card)")),
    "aim9x": dict(card="aim-9x", paint=US, nose="dome", dome=0.36,
                  surfaces=[dict(role="canard", x=0.42, root=0.12, tip=0.06, s=0.036, control="fixed"),
                            dict(role="tail", x=2.70, root=0.22, tip=0.10, span=True, control="moving")],
                  motor=dict(smoke="reduced", phases=[(5.0, 11.0)], note="the AIM-9M's Mk 36 (the card)")),
    "asraam": dict(card="asraam", paint=NATO, nose="dome", dome=0.40,
                   surfaces=[dict(role="tail", x=2.55, root=0.30, tip=0.15, span=True, control="moving")],
                   motor=dict(smoke="reduced", phases=[(2.0, 25.0), (4.0, 6.0)], note="dual burn (the card); low smoke assumed")),
    "irist": dict(card="iris-t", paint=NATO, nose="dome", dome=0.36,
                  surfaces=[dict(role="strake", x=0.70, root=1.20, tip=1.10, s=0.025, control="fixed"),
                            dict(role="tail", x=2.62, root=0.28, tip=0.15, span=True, control="moving")],
                  motor=dict(smoke="reduced", phases=[(5.0, 12.0)], note="HTPB (the card); its nozzle's thrust vanes not drawn")),
    "magic2": dict(card="magic-2", paint=NATO, nose="dome", dome=0.36,
                   surfaces=[dict(role="canard", x=0.30, root=0.12, tip=0.04, s=0.07, control="fixed"),
                             dict(role="canard", x=0.45, root=0.20, tip=0.05, s=0.13, control="moving", pivot=0.5),
                             dict(role="tail", x=2.30, root=0.35, tip=0.14, span=True, control="fixed", free_rolling=True)],
                   motor=dict(smoke="reduced", phases=[(2.0, 25.0)], note="single stage (the card); its smoke assumed")),
    "meteor": dict(card="meteor", paint=NATO, nose="radome", cal=2.8, span_m=0.40,
                   surfaces=[dict(role="tail", x=3.25, root=0.30, tip=0.18, span=True, control="moving")],
                   intakes=dict(x=2.05, end=3.10, w=0.085, h=0.070),
                   motor=dict(smoke="reduced", phases=[(2.0, 25.0), (25.0, 3.0)], exit=0.80,
                              note="the integrated booster, then the ducted rocket throttled (the card); its smoke assumed")),
    "micaem": dict(card="mica-em", paint=NATO, nose="radome", cal=2.6,
                   surfaces=[dict(role="strake", x=0.95, root=1.40, tip=1.30, s=0.06, control="fixed"),
                             dict(role="tail", x=2.75, root=0.28, tip=0.14, span=True, control="moving")],
                   motor=dict(smoke="reduced", phases=[(4.0, 15.0)], note="solid (the card); its smoke assumed")),
    "micair": dict(card="mica-ir", paint=NATO, nose="dome", dome=0.40,
                   surfaces=[dict(role="strake", x=0.95, root=1.40, tip=1.30, s=0.06, control="fixed"),
                             dict(role="tail", x=2.75, root=0.28, tip=0.14, span=True, control="moving")],
                   motor=dict(smoke="reduced", phases=[(4.0, 15.0)], note="the MICA EM's airframe and motor")),
    "pl10": dict(card="pl-10", paint=OTHER, nose="dome", dome=0.40, span_m=0.40,
                 surfaces=[dict(role="strake", x=0.80, root=1.50, tip=1.40, s=0.04, control="fixed"),
                           dict(role="tail", x=2.66, root=0.26, tip=0.12, span=True, control="moving")],
                 motor=dict(smoke="reduced", phases=[(5.0, 12.0)], note="solid (the card); its smoke assumed")),
    "pl12": dict(card="pl-12", paint=OTHER, nose="radome", cal=2.6,
                 surfaces=[dict(role="wing", x=1.70, root=0.45, tip=0.12, span=True, control="fixed"),
                           dict(role="tail", x=3.50, root=0.34, tip=0.15, span=True, control="moving")],
                 motor=dict(smoke="reduced", phases=[(3.0, 25.0), (6.0, 6.0)], note="dual thrust (the card); its smoke assumed")),
    "pl15": dict(card="pl-15", paint=OTHER, nose="radome", cal=2.8, span_m=0.45,
                 surfaces=[dict(role="tail", x=3.65, root=0.30, tip=0.14, span=True, control="moving")],
                 motor=dict(smoke="reduced", phases=[(4.0, 25.0), (6.0, 0.0), (4.0, 12.0)], note="dual pulse (the card): a coast between"),),
    "pl8": dict(card="pl-8", paint=OTHER, nose="dome", dome=0.36,
                surfaces=[dict(role="canard", x=0.30, root=0.12, tip=0.04, s=0.06, control="fixed"),
                          dict(role="canard", x=0.45, root=0.30, tip=0.06, s=0.13, control="moving", pivot=0.5),
                          dict(role="tail", x=2.20, root=0.62, tip=0.20, span=True, control="aileron")],
                motor=dict(smoke="reduced", phases=[(3.0, 20.0)], note="solid (the card); its smoke assumed")),
    "r27r": dict(card="r-27r", paint=OTHER, nose="radome", cal=2.4,
                 surfaces=[dict(role="canard", x=0.70, root=0.20, tip=0.08, s=0.085, control="fixed"),
                           dict(role="wing", x=1.70, root=0.25, tip=0.55, tip_x=1.55, span=True, control="moving", pivot=0.5),
                           dict(role="tail", x=3.55, root=0.48, tip=0.20, s=0.235, control="aileron")],
                 motor=dict(smoke="aluminised", phases=[(6.0, 18.0)], note="single mode (the card); aluminised assumed from its trail in photographs")),
    "r27t": dict(card="r-27t", paint=OTHER, nose="dome", dome=0.45,
                 surfaces=[dict(role="canard", x=0.42, root=0.20, tip=0.08, s=0.085, control="fixed"),
                           dict(role="wing", x=1.42, root=0.25, tip=0.55, tip_x=1.27, span=True, control="moving", pivot=0.5),
                           dict(role="tail", x=3.27, root=0.48, tip=0.20, s=0.235, control="aileron")],
                 motor=dict(smoke="aluminised", phases=[(6.0, 18.0)], note="the R-27R's motor")),
    "r27er": dict(card="r-27er", paint=OTHER, nose="radome", cal=2.4, step=dict(x=1.95, d=0.26), front_d=0.23,
                  surfaces=[dict(role="canard", x=0.70, root=0.20, tip=0.08, s=0.085, control="fixed"),
                            dict(role="wing", x=1.70, root=0.25, tip=0.55, tip_x=1.55, span=True, control="moving", pivot=0.5),
                            dict(role="tail", x=4.25, root=0.48, tip=0.20, s=0.22, control="aileron")],
                  motor=dict(smoke="aluminised", phases=[(3.0, 35.0), (7.0, 9.0)], note="dual mode, 260 mm (the card)")),
    "r27et": dict(card="r-27et", paint=OTHER, nose="dome", dome=0.45, step=dict(x=1.67, d=0.26), front_d=0.23,
                  surfaces=[dict(role="canard", x=0.42, root=0.20, tip=0.08, s=0.085, control="fixed"),
                            dict(role="wing", x=1.42, root=0.25, tip=0.55, tip_x=1.27, span=True, control="moving", pivot=0.5),
                            dict(role="tail", x=3.96, root=0.48, tip=0.20, s=0.22, control="aileron")],
                  motor=dict(smoke="aluminised", phases=[(3.0, 35.0), (7.0, 9.0)], note="the R-27ER's motor")),
    "r37m": dict(card="r-37m", paint=OTHER, nose="radome", cal=2.2,
                 surfaces=[dict(role="strake", x=1.20, root=2.00, tip=1.90, s=0.04, control="fixed"),
                           dict(role="tail", x=3.55, root=0.42, tip=0.20, span=True, control="moving")],
                 motor=dict(smoke="reduced", phases=[(6.0, 40.0), (10.0, 0.0), (6.0, 15.0)], note="dual pulse (the card); its smoke assumed")),
    "r60m": dict(card="r-60m", paint=OTHER, nose="dome", dome=0.40,
                 surfaces=[dict(role="canard", x=0.20, root=0.08, tip=0.03, s=0.03, control="fixed"),
                           dict(role="canard", x=0.33, root=0.15, tip=0.04, s=0.065, control="moving", pivot=0.5),
                           dict(role="tail", x=1.75, root=0.30, tip=0.10, span=True, control="fixed")],
                 motor=dict(smoke="reduced", phases=[(3.0, 8.0)], note="solid (the card); its smoke assumed")),
    "r73": dict(card="r-73", paint=OTHER, nose="dome", dome=0.40,
                surfaces=[dict(role="canard", x=0.25, root=0.10, tip=0.04, s=0.04, control="fixed"),
                          dict(role="canard", x=0.40, root=0.18, tip=0.05, s=0.1075, control="moving", pivot=0.5),
                          dict(role="tail", x=2.40, root=0.45, tip=0.15, span=True, control="aileron")],
                motor=dict(smoke="aluminised", phases=[(4.0, 14.0)], note="solid (the card); aluminised assumed from its trail in photographs")),
    "r77m": dict(card="r-77m", paint=OTHER, nose="radome", cal=2.6, span_m=0.60,
                 surfaces=[dict(role="strake", x=1.50, root=1.20, tip=1.10, s=0.05, control="fixed"),
                           dict(role="tail", x=3.50, root=0.12, tip=0.12, span=True, control="lattice", width=0.15, cells=3)],
                 cell=0.0015,
                 motor=dict(smoke="reduced", phases=[(4.0, 25.0), (6.0, 0.0), (4.0, 12.0)], note="dual pulse (the card); its smoke assumed")),
    # (its wings' root 0.95 m: a 1.10 m root swept 77 deg met the body in an edge the mesher left open)
    "super530d": dict(card="super-530d", paint=NATO, nose="radome", cal=2.4,
                      surfaces=[dict(role="wing", x=1.70, root=0.95, tip=0.35, span=True, control="fixed"),
                                dict(role="tail", x=3.22, root=0.40, tip=0.20, s=0.17, control="moving")],
                      motor=dict(smoke="aluminised", phases=[(2.0, 40.0), (4.0, 8.0)], note="boost-sustain assumed; aluminised assumed from its trail in photographs")),
}


def _card(card_id):
    with open(os.path.join(pipeline.repo_root(), "assets", "stores", card_id + ".json"), encoding="utf-8") as f:
        return json.load(f)


def _f(v):
    return ("%.4f" % v).rstrip("0").rstrip(".") if abs(v) > 1e-12 else "0.0"


def _station(x, r, n=2.0):
    return "  { x = %s, w = %s, top = %s, bottom = %s, n = %s }," % (_f(x), _f(2 * r), _f(r), _f(-r), _f(n))


def _body(row, L, D):
    """The body's stations: the nose (an ogive radome, or a cone under an infrared dome), the cylinder (and a wider
    motor section where the row has a step), the boattail over the nozzle."""
    R = 0.5 * (row.get("front_d") or D)
    rows = []
    if row["nose"] == "radome":
        ln = row["cal"] * 2.0 * R
        rho = (R * R + ln * ln) / (2.0 * R)  # a tangent ogive
        for f in (0.0, 0.03, 0.08, 0.16, 0.28, 0.42, 0.6, 0.8, 1.0):
            x = f * ln
            rows.append((x, max(0.0, math.sqrt(max(rho * rho - (ln - x) ** 2, 0.0)) + R - rho)))
    else:
        rd = row["dome"] * R
        lc = 2.2 * R  # the seeker's cone behind the dome
        rows += [(0.45 * rd, 0.80 * rd), (rd, 0.97 * rd)]
        for f in (0.0, 0.5, 1.0):
            x = rd + f * lc
            rows.append((x, rd + (R - rd) * (f if f < 1.0 else 1.0) ** 0.8))
    step = row.get("step")
    tail_r = 0.5 * D
    if step:
        rows += [(step["x"] - 0.12, R), (step["x"], 0.5 * step["d"])]
    end = L - 0.6 * D
    rows += [(end, tail_r), (L, 0.80 * tail_r)]
    out, last = [], -1.0
    for x, r in rows:
        if x > last + 1e-4:
            out.append((x, r))
            last = x
    return out


def _radius(body, x):
    xs = [p[0] for p in body]
    rs = [p[1] for p in body]
    if x <= xs[0]:
        return rs[0]
    for i in range(1, len(xs)):
        if x <= xs[i]:
            return rs[i - 1] + (rs[i] - rs[i - 1]) * (x - xs[i - 1]) / (xs[i] - xs[i - 1])
    return rs[-1]


def design(name):
    row = MISSILES[name]
    card = _card(row["card"])
    par = card["parameters"]
    L, D, M = par["length_m"]["value"], par["diameter_m"]["value"], par["mass_kg"]["value"]
    span = par["span_m"]["value"] if "span_m" in par else row["span_m"]
    span_src = "the card" if "span_m" in par else "assumed from photographs (the card has none)"
    body = _body(row, L, D)
    lines = ["# %s, built by hangar from public data (ADR-30, MS-7e1) - written by `python -m hangar missiles %s`" % (card["designation"], name),
             "# (tools/hangar/hangar/missiles.py), one rule for every missile: edit the table there and write it again.",
             "#",
             "# Frame: metres, x aft from the nose's tip, y right, z up from the body's axis. The card (assets/stores/%s.json)" % row["card"],
             "# holds the published figures: length %s m, diameter %s m, mass %s kg; span %s m (%s), tip to tip across" % (_f(L), _f(D), _f(M), _f(span), span_src),
             "# opposite fins. Its surfaces fly in an X, 45 deg from the vertical, as it hangs on its launcher (assumed for all).",
             "# Assumed from photographs, the same way for every missile: the nose's shape and length, each surface's place,",
             "# chords and sweep, the motor's nozzle and burn, the CG. They only shape the picture until the aero stage runs.",
             "",
             "[aircraft]",
             'name = "%s"' % name,
             'description = "%s (hangar build)"' % card["designation"],
             'category = "missile"',
             'card = "%s"             # its store card: the published figures this design holds' % row["card"],
             "",
             "[analysis]",
             "speed = 900.0                # m/s: the Reynolds numbers (Mach 3 at altitude) - assumed",
             ""]
    if row.get("cell"):
        lines += ["[model]", "cell_m = %s               # the mesher's cell: its lattice fins' walls a few of them thick" % _f(row["cell"]), ""]
    if row["nose"] == "dome":
        R = 0.5 * (row.get("front_d") or D)
        rd = row["dome"] * R
        lines += ["# The infrared seeker's glass dome (%.0f %% of the body's radius - assumed), on its cone" % (100 * row["dome"]),
                  "[[body]]", 'name = "canopy seeker dome"', 'kind = "pod"', "stations = [",
                  _station(0.0, 0.0), _station(0.25 * rd, 0.66 * rd), _station(0.6 * rd, 0.92 * rd), _station(rd, rd), _station(1.4 * rd, rd),
                  "]", ""]
    lines += ["# The body: %s, the cylinder%s, a boattail over the nozzle (assumed)" % (
        "an ogive radome %.1f calibres long" % row["cal"] if row["nose"] == "radome" else "the seeker's cone",
        ", a wider motor section from x %.2f m (the card's diameter is the motor's)" % row["step"]["x"] if row.get("step") else ""),
              "[[body]]", 'name = "body"', 'kind = "fuselage"', "stations = ["]
    lines += [_station(x, r) for x, r in body] + ["]", ""]
    if row.get("intakes"):
        it = row["intakes"]
        r = _radius(body, it["x"])
        yc, zc = (r + 0.2 * it["w"]) * R45, -(r + 0.2 * it["w"]) * R45
        lines += ["# The ducted rocket's two air intakes, low on either side (their place and size assumed from photographs)",
                  "[[intake]]", 'name = "intake"', "mirror = true", "lip = 0.006", "duct = 0.4", "stations = ["]
        for x, f in ((it["x"], 1.0), (it["x"] + 0.25, 1.0), (it["end"] - 0.25, 0.9), (it["end"], 0.25)):
            lines.append("  { x = %s, y = %s, w = %s, top = %s, bottom = %s, n = 2.4 }," % (
                _f(x), _f(yc), _f(f * it["w"]), _f(zc + 0.5 * f * it["h"]), _f(zc - 0.5 * f * it["h"])))
        lines += ["]", ""]
    for k, g in enumerate(row["surfaces"]):
        r = _radius(body, g["x"] + 0.5 * g["root"])
        s = (0.5 * span - r) if g.get("span") else g["s"]
        tip_le = g.get("tip_x", g["x"] + g["root"] - g["tip"])  # the trailing edge straight across, unless given
        label = {"canard": "canards", "wing": "wings", "strake": "strakes", "tail": "tail fins"}[g["role"]]
        what = {"moving": "all-moving about their spindles", "aileron": "fixed, with ailerons along their trailing edges",
                "fixed": "fixed", "lattice": "lattice (grid) fins, all-moving, %s m wide, %d cells across" % (
                    _f(g.get("width", 0.0)), g.get("cells", 3))}[g["control"]]
        extra = []
        if g.get("rollerons"):
            extra.append("their tips' rollerons, %s m across, spun by the airstream" % _f(2 * ROLLERON_RADIUS))
        if g.get("free_rolling"):
            extra.append("free to roll on their bearing ring, %s turns per km flown" % _f(1000 * FREE_ROLL_PER_M))
        lines.append("# The %s: %s%s; root chord %s m, tip %s m, %s m out from the body%s (assumed from photographs)" % (
            label, what, "; " + ", ".join(extra) if extra else "", _f(g["root"]), _f(g["tip"]), _f(s),
            ", their tips across the card's span" if g.get("span") else ""))
        for side, sz in (("upper", 1.0), ("lower", -1.0)):
            r0 = 0.97 * r
            lines += ["[[surface]]", 'name = "%s %s"' % (label[:-1] if label.endswith("s") else label, side), 'kind = "fin"', "mirror = true",
                      'airfoil = "%s"' % ("plate" if g["role"] == "strake" or g["control"] == "lattice" else "biconvex4"), "sections = [",
                      "  { le = [%s, %s, %s], chord = %s }," % (_f(g["x"]), _f(r0 * R45), _f(sz * r0 * R45), _f(g["root"])),
                      "  { le = [%s, %s, %s], chord = %s }," % (_f(tip_le), _f((r + s) * R45), _f(sz * (r + s) * R45), _f(g["tip"])),
                      "]"]
            if g["control"] == "lattice":
                lines.append("lattice = { width = %s, cells = %d }" % (_f(g["width"]), g.get("cells", 3)))
            if g.get("rollerons"):
                lines.append("rolleron = { radius = %s }" % _f(ROLLERON_RADIUS))
            if g.get("free_rolling"):
                lines.append("free_rolling = { turns_per_m = %s }" % _f(FREE_ROLL_PER_M))
            if g["control"] in ("moving", "lattice"):
                canard = g["role"] == "canard"
                mix = "{ rudder = -1.0 }" if canard else "{ rudder = 1.0, aileron = 1.0 }"
                lines += ["controls = [",
                          '  { name = "%s %s", channel = "elevator"%s, mix = %s, span = [0.0, 1.0], chord_fraction = 1.0, pivot = %s, limits = [-20, 20] },' % (
                              label[:-1], side, ", gain = -1.0" if canard else "", mix, _f(g.get("pivot", 0.45))),
                          "]"]
            elif g["control"] == "aileron":
                lines += ["controls = [",
                          '  { name = "aileron %s", channel = "aileron", span = [0.05, 0.95], chord_fraction = 0.3, limits = [-20, 20] },' % side,
                          "]"]
            lines.append("")
    m = row["motor"]
    re = 0.5 * m.get("exit", 0.65) * 0.80 * D
    from . import components
    legacy_motor = {"exit": [float(_f(L)), 0.0, 0.0], "exit_diameter": float(_f(2*re)),
        "throat_diameter": float(_f(.9*re)), "cone_length": float(_f(1.1*re*2)), "smoke": m["smoke"],
        "phase": [{"duration_s": float(_f(d)), "thrust_kn": float(_f(t))} for d,t in m["phases"]]}
    selected_motor, _ = components.motor(card, {"motor": legacy_motor})
    chosen = selected_motor["motor"]
    def configured_number(value):
        rounded = _f(value)
        return rounded if float(rounded)==value else "%.17g" % value
    lines += ["# The rocket motor: %s. Its nozzle in the boattail, its exit %.0f %% of the base and its burn's phases assumed" % (m["note"], 100 * m.get("exit", 0.65)),
              "# (none published): only what a replay shows the plume with until a flight model burns it.",
              "[motor]", "exit = [%s]" % ", ".join(configured_number(v) for v in chosen["exit"]),
              "exit_diameter = %s" % configured_number(chosen["exit_diameter"]),
              "throat_diameter = %s" % configured_number(chosen["throat_diameter"]),
              "cone_length = %s" % configured_number(chosen["cone_length"]), 'smoke = "%s"' % chosen["smoke"]]
    for phase in chosen["phase"]:
        lines += ["[[motor.phase]]", "duration_s = %s" % configured_number(phase["duration_s"]),
                  "thrust_kn = %s" % configured_number(phase["thrust_kn"])]
    lines += ["",
              "[mass]",
              "empty = %s                 # kg, the card, motor full" % _f(M),
              "empty_cg = [%s, 0.0, 0.0]  # assumed: half way along" % _f(0.5 * L),
              "gyration = [%s, 0.289, 0.289]  # radii of gyration over length - assumed (a slender rod's: L/sqrt(12))" % _f(0.35 * D / L),
              "",
              "[dimensions]                  # the card: the span tip to tip across opposite fins",
              "length = %s" % _f(L)]
    if "span_m" in par:
        lines.append("span = %s" % _f(span))
    return "\n".join(lines) + "\n", _paint(row, card, body, L)


def _paint(row, card, body, L):
    p = row["paint"]
    lines = ["# %s's paint, by one rule for every missile (tools/hangar/hangar/missiles.py): light grey overall; %s" % (
        card["designation"], "a radar seeker's radome a lighter grey" if row["nose"] == "radome" else "the seeker's dome glass"),
             "# %s" % ("a yellow band on the warhead (a live round) and a brown one on the motor - the maker's colour code"
                       " (MIL-STD-1168, STANAG 2953); the bands' places assumed" if p["bands"] else
                       "no bands: no published colour code for its maker's munitions was found"),
             "[paint]", 'scheme = "single"', 'colours = ["%s"]' % p["colour"]]
    if row["nose"] == "radome":
        lines += ['radome = "#d8dadc"', "radome_x = %s" % _f(row["cal"] * (row.get("front_d") or card["parameters"]["diameter_m"]["value"]))]
    if p["bands"]:
        x1 = 0.34 * L
        x2 = 0.63 * L
        lines += ["bands = [", '  { x = [%s, %s], colour = "#c8a020" },' % (_f(x1), _f(x1 + 0.05)),
                  '  { x = [%s, %s], colour = "#6b4a2b" },' % (_f(x2), _f(x2 + 0.05)), "]"]
    lines += ["panel_lines = 0.0", "wear = 0.03", "gloss = 0.3"]
    return "\n".join(lines) + "\n"


def write(name):
    toml, paint = design(name)
    d = os.path.join(pipeline.repo_root(), "aircraft", name)
    os.makedirs(d, exist_ok=True)
    for fn, text in ((name + ".toml", toml), ("paint.toml", paint)):
        with open_write(os.path.join(d, fn), "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
    return os.path.join(d, name + ".toml")


def main(argv):
    for name in argv or sorted(MISSILES):
        if name not in MISSILES:
            raise SystemExit("%s: not a missile in hangar/missiles.py (%s)" % (name, ", ".join(sorted(MISSILES))))
        print(write(name))
    return 0
