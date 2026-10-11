"""The stores that are not missiles as hangar designs, written by one rule for each class - so that every payload is built
by the pipeline its aircraft is (docs/hangar.md, Stores):

    python -m hangar storedesigns [<name> ...]    writes aircraft/<name>/<name>.toml (+ paint.toml) for each

A store's published figures are its card's (assets/stores/<id>.json): its length, its diameter (a cargo store's width
and height) and its mass; the design holds them. What no card gives is the class's shape, the same for every store of
it (stores.py: a tank's ogive, cylinder and cone; a pod's two half-spheres; the Mk 82's estimated profile and fins; a
cargo store's box) - assumed, and said so in the file written. The written file is the design's record and what the
stages read; this module is how it was made, so a better shape is entered in stores.py and the file written again.

A store's design differs from an aircraft's in three things: its category is "store" (a missile's is "missile": it
has its own writer, missiles.py); its reference is its frontal area and its diameter, as its flight model's; and
`[store]` says what the build-up cannot read from its body - a pod's wake (its rounded tail's separated flow), a cargo
store's box.
"""
import json
import math
import os

from . import pipeline, stores

#: the classes this module writes, and the inertia each is given (assumed): a tank and a pod a thin shell, a bomb a
#: solid body, cargo a uniform block
CLASSES = ("tank", "pod", "bomb", "cargo")

#: the triangles each class's model is given at most ([model] triangles): a smooth body of revolution of a few metres is
#: round to a millimetre in these, where an aircraft's 75,000 would be drawn for nothing on every station that carries
#: one; a bomb's more for its fins' edges
TRIANGLES = {"tank": 6000, "pod": 6000, "bomb": 10000}


def _f(v):
    t = ("%.6f" % v).rstrip("0").rstrip(".")
    return "0" if t in ("-0", "") else t


def _hex(rgb):
    return "#%02x%02x%02x" % tuple(int(round(255 * c)) for c in rgb)


def cards():
    """Each store card this module designs, by its model's name (its id without hyphens)."""
    out = {}
    d = os.path.join(pipeline.repo_root(), "assets", "stores")
    for f in sorted(os.listdir(d)):
        if f.endswith(".json"):
            with open(os.path.join(d, f), encoding="utf-8") as h:
                c = json.load(h)
            if c.get("class") in CLASSES:
                out[c["id"].replace("-", "")] = c
    return out


def _spec(name, card):
    """Its shape as stores.py has it: {length, diameter, cg, profile [(x m, r m)], fins, wake, colour, band}."""
    if card["class"] == "bomb":
        s = dict(stores.STORES[name])
        s["profile"] = [(x, r * s["diameter"] / 2.0) for x, r in s["profile"]]
        return s
    s = dict(stores.tank_spec(card))
    s["profile"] = [(x, r * s["diameter"] / 2.0) for x, r in s["profile"]]
    if "base" in s:
        s["wake"] = s["base"] * s["diameter"]
    return s


def _fins(f, profile):
    """Its fins as surfaces: (lines, the span tip to tip across the mirrored ones). Each fin `count` round the body from
    `roll_deg` (from +y towards +z); one on the plane of symmetry a surface of its own, the others in mirrored pairs."""
    xs, rs = [p[0] for p in profile], [p[1] for p in profile]
    mid = f["root_le"] + 0.5 * f["root_chord"]
    root_r = 0.98 * float(_interp(mid, xs, rs))
    tip_x = f["root_le"] + math.tan(math.radians(f["sweep_deg"])) * f["span"]
    lines, span, k = [], 0.0, 0
    for i in range(f["count"]):
        a = math.radians(f["roll_deg"]) + 2.0 * math.pi * i / f["count"]
        cy, cz = math.cos(a), math.sin(a)
        if cy < -1e-9:
            continue  # (the mirror image of one on the right)
        mirror = cy > 1e-9
        k += 1
        tip_r = root_r / 0.98 + f["span"]
        lines += ["[[surface]]", 'name = "fin %d"' % k, 'kind = "fin"', "mirror = %s" % ("true" if mirror else "false"), 'airfoil = "plate"',
                  "sections = [",
                  "  { le = [%s, %s, %s], chord = %s }," % (_f(f["root_le"]), _f(root_r * cy), _f(root_r * cz), _f(f["root_chord"])),
                  "  { le = [%s, %s, %s], chord = %s }," % (_f(tip_x), _f(tip_r * cy), _f(tip_r * cz), _f(f["tip_chord"])),
                  "]", ""]
        if mirror:
            span = max(span, 2.0 * tip_r * cy)
    return lines, span


def _interp(v, xs, ys):
    if v <= xs[0]:
        return ys[0]
    for i in range(1, len(xs)):
        if v <= xs[i]:
            t = (v - xs[i - 1]) / (xs[i] - xs[i - 1]) if xs[i] > xs[i - 1] else 0.0
            return ys[i - 1] + t * (ys[i] - ys[i - 1])
    return ys[-1]


def _gyration(ixx, iyy, izz, m, span, length):
    """hangar's radii of gyration (Roskam's: over half the span, half the length, a quarter of their sum)."""
    return [math.sqrt(ixx / m) / (0.5 * span), math.sqrt(iyy / m) / (0.5 * length), math.sqrt(izz / m) / (0.25 * (span + length))]


def design(name):
    """(the design's text, its paint's text) of the store `name`."""
    card = cards()[name]
    par = card["parameters"]
    M, L = par["mass_kg"]["value"], par["length_m"]["value"]
    head = ["# %s, a store built by hangar from its card (docs/hangar.md, Stores) - written by `python -m hangar storedesigns %s`" % (card["designation"], name),
            "# (tools/hangar/hangar/storedesigns.py), one rule for every %s: edit stores.py's shape and write it again." % card["class"],
            "#",
            "# Frame: metres, x aft from its nose, y right, z up from its axis. The card (assets/stores/%s.json) holds its" % card["id"]]
    if card["class"] == "cargo":
        W, H = par["width_m"]["value"], par["height_m"]["value"]
        head += ["# figures: %s m long, %s m wide, %s m high, %s kg as loaded. A box of that size, its mass throughout (assumed)." % (_f(L), _f(W), _f(H), _f(M)),
                 ""]
        area, ref = W * H, W
        inertia = (M * (W * W + H * H) / 12.0, M * (L * L + H * H) / 12.0, M * (L * L + W * W) / 12.0)
        cg = 0.5 * L
        e = 0.002  # (its end faces: closed over 2 mm, as the mesher wants a body's ends)
        rows = [(0.0, 0.0, 0.0), (e, W, H), (L - e, W, H), (L, 0.0, 0.0)]
        body = ["# Its box: a rectangle the whole way (a superellipse of a high power: its edges sharp)",
                "[[body]]", 'name = "load"', 'kind = "fuselage"', "stations = ["]
        body += ["  { x = %s, w = %s, top = %s, bottom = %s, n = 16 }," % (_f(x), _f(w), _f(0.5 * h), _f(-0.5 * h)) for x, w, h in rows]
        body += ["]", ""]
        fins, span = [], ref
        store = ["[store]", 'shape = "box"            # a bluff body: its faces\' drag (its card\'s drag_cd), not a body of revolution\'s',
                 "width = %s" % _f(W), "height = %s" % _f(H), ""]
        colour, band = (0.36, 0.38, 0.26), None
    else:
        s = _spec(name, card)
        D = s["diameter"]
        head += ["# figures: %s m long, %s m across, %s kg%s. Its shape is its class's, the same for every %s (assumed from" % (
            _f(L), _f(D), _f(M), " empty (its fuel is what its station holds)" if card["class"] == "tank" else "", card["class"]),
                 "# photographs: stores.py) - it shapes the picture, the build-up's drag and where its fins act.", ""]
        r = 0.5 * D
        area, ref, cg = math.pi * r * r, D, s["cg"]
        if card["class"] == "bomb":
            inertia = (0.5 * M * r * r, M * (0.25 * r * r + L * L / 12.0), M * (0.25 * r * r + L * L / 12.0))
            note = "a solid cylinder's (assumed)"
        else:
            inertia = (M * r * r, M * (0.5 * r * r + L * L / 12.0), M * (0.5 * r * r + L * L / 12.0))
            note = "a thin shell's, as its flight model's (assumed)"
        body = ["# Its body: a body of revolution, its radius along it its class's profile",
                "[[body]]", 'name = "body"', 'kind = "fuselage"', "stations = ["]
        body += ["  { x = %s, w = %s, top = %s, bottom = %s, n = 2 }," % (_f(x), _f(2.0 * rr), _f(rr), _f(-rr)) for x, rr in s["profile"]]
        body += ["]", ""]
        fins, span = [], ref
        f = s.get("fins")
        if f and f["count"]:
            fins, fin_span = _fins(f, s["profile"])
            span = max(fin_span, ref) if fin_span > 0.0 else ref
            fins = ["# Its fins: %d round its tail (assumed from photographs)" % f["count"]] + fins
        store = []
        if "wake" in s:
            store = ["[store]",
                     "wake = %s                # m: its rounded tail's separated flow, a blunt base's of this diameter to its drag (stores.py)" % _f(s["wake"]),
                     ""]
        colour, band = s["colour"], (s.get("band"), s.get("band_at"))
    g = _gyration(*inertia, M, span, L)
    lines = head + [
        "[aircraft]", 'name = "%s"' % name, 'description = "%s (hangar build)"' % card["designation"], 'category = "store"',
        'card = "%s"             # its store card: the published figures this design holds' % card["id"], "",
        "[analysis]", "speed = 250.0                # m/s: the Reynolds numbers of its carriage - assumed", "",
        "[reference]                  # a store's: its frontal area and its diameter (its width), as its flight model's",
        "area = %s" % _f(area), "span = %s" % _f(ref), "chord = %s" % _f(ref), "aero_point = [%s, 0.0, 0.0]" % _f(cg), ""]
    if card["class"] in TRIANGLES:
        lines += ["[model]", "triangles = %d             # its model's at most: a store's, drawn on every station that carries one" % TRIANGLES[card["class"]], ""]
    lines += store + body + fins
    lines += ["[mass]",
              "empty = %s                 # kg, the card%s" % (_f(M), ": its structure, dry" if card["class"] == "tank" else ""),
              "empty_cg = [%s, 0.0, 0.0]  # %s" % (_f(cg), "JSBSim's mk82.xml, the flight model it is drawn for" if card["class"] == "bomb" else "assumed: half way along"),
              "gyration = [%s, %s, %s]  # %s" % (_f(g[0]), _f(g[1]), _f(g[2]),
                                                 "a uniform block's (assumed)" if card["class"] == "cargo" else note),
              "", "[dimensions]                  # the card", "length = %s" % _f(L)]
    if card["class"] == "cargo":
        lines.append("span = %s" % _f(par["width_m"]["value"]))
    paint = ["# %s's paint, by one rule for every %s (tools/hangar/hangar/storedesigns.py): one colour overall%s" % (
        card["designation"], card["class"], "; a yellow band behind its nose (a live bomb: MIL-STD-1168), its place assumed" if band and band[0] else ""),
             "[paint]", 'scheme = "single"', 'colours = ["%s"]' % _hex(colour)]
    if band and band[0]:
        paint += ["bands = [", '  { x = [%s, %s], colour = "%s" },' % (_f(band[1][0]), _f(band[1][1]), _hex(band[0])), "]"]
    paint += ["panel_lines = 0.0", "wear = 0.03", "gloss = 0.3"]
    return "\n".join(lines) + "\n", "\n".join(paint) + "\n"


def write(name):
    toml, paint = design(name)
    d = os.path.join(pipeline.repo_root(), "aircraft", name)
    os.makedirs(d, exist_ok=True)
    for fn, text in ((name + ".toml", toml), ("paint.toml", paint)):
        pipeline.write_text(os.path.join(d, fn), text)
    return os.path.join(d, name + ".toml")


def main(argv):
    known = cards()
    for name in argv or sorted(known):
        if name not in known:
            raise SystemExit("%s: not a tank, pod, bomb or cargo card in assets/stores (%s)" % (name, ", ".join(sorted(known))))
        print(write(name))
    return 0
