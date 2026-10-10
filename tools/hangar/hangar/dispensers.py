"""Each aircraft's chaff and flare dispensers' places (ADR-30, MS-7e1: MS-9a's chaff, MS-7h's drawn bundles): where on
the airframe its cartridges leave and the way they go, from what is public about each type - its photographs
releasing flares (Wikimedia Commons), encyclopedias and the defence press, and where those say nothing, other games'
public documentation and forums (the owner's call, 2026-10-09) - each group's sources and their kind in TYPES. A
combat type none of them places has one rule, an estimate said so: a pair under the aft fuselage, firing down and
20 deg outboard, where most photographed fighters' are. A transport, tanker or reconnaissance type nothing places
has none.

A group is anchored to the design's own geometry - the fuselage at a station set by a feature (its main gear, its wing
root's trailing edge, its end, or 40 % of the way from that trailing edge to the tail's roots) on its belly, sides or
top; a named body (a gear pod, a nacelle) along its length; a lifting surface's skin at a span and chord fraction; the
outermost store stations' aft ends (a launch rail's) - one each side, and each set on the airframe's skin by coming
in from outside along the way it fires until the mesher's solid begins. The cartridges themselves - their count and
kinds - are MS-9a's payload card's; the counts here are the sources', for it.

The manifest's `dispenser` lines: `dispenser <section> <x y z> <direction x y z> <name>`, body axes from the origin."""
import math

import numpy as np

COMBAT = ("fighter", "attack", "bomber")
AFT = 0.4                  # of the way from the wing root's trailing edge to the tail's roots (the rule's station)
CANT_DEG = 20.0            # outboard of straight down (or up)
SIDE_DOWN_DEG = 30.0       # below the horizontal, a side group's

# the way a group fires: (y outboard, z up) before the side's sign
FIRES = {"down": (math.sin(math.radians(CANT_DEG)), -math.cos(math.radians(CANT_DEG))),
         "up": (math.sin(math.radians(CANT_DEG)), math.cos(math.radians(CANT_DEG))),
         "out": (math.cos(math.radians(SIDE_DOWN_DEG)), -math.sin(math.radians(SIDE_DOWN_DEG))),
         "steep up": (0.5, math.sqrt(0.75)), "straight down": (0.0, -1.0)}


def _g(name, where, fires, sources, basis="sourced", count=None):
    return dict(name=name, where=where, fires=fires, sources=sources, basis=basis, count=count)


RULE = _g("aft belly", ("fuselage", ("between", AFT), "belly", 0.3), "down", [], "rule")

# Each type's groups; each source "kind: what it says" - kinds photo (Wikimedia Commons file), encyclopedia, press,
# official, game, forum (2026-10-09)
TYPES = {
    "f15c": [_g("aft belly", ("fuselage", ("gear", 1.0), "belly", 0.3), "down",
                ["photo: F-15C Eagle, releasing a flare.jpg - one lit under the fuselage between the engines, aft of "
                 "the wing", "photo: F-15C Flares After Take Off.jpg"])],
    "f16c": [_g("aft belly", ("fuselage", ("between", AFT), "belly", 0.3), "down",
                ["photo: F-16 releases four flares.jpg, F-16 releases a flare.jpg - from under the aft fuselage",
                 "forum: f-16.net - four on the aft fuselage (Block 30 on)"], count="4 x 30 chaff / 15 flares")],
    "fa18c": [_g("trunks' undersides", ("fuselage", ("gear", -1.0), "belly", 0.55), "down",
                 ["press: The War Zone - on the underside of the engine trunks forward of the main gear",
                  "photo: Finnish F-18 releases flares.JPG - below the fuselage"])],
    "ea18g": [_g("trunks' undersides", ("fuselage", ("gear", -1.0), "belly", 0.55), "down",
                 ["press: The War Zone (the Hornet's) - the engine trunks' undersides forward of the main gear",
                  "photo: U S Navy EA-18 Growlers conduct combat patrols over CENTCOM AOR (8818991).jpg - flares below"])],
    "gripen": [_g("aft belly", ("fuselage", ("between", AFT), "belly", 0.3), "down",
                  ["photo: Gripen dropping flares.jpg - from under the aft fuselage"])],
    "mirage2000": [_g("aft belly", ("fuselage", ("between", AFT), "belly", 0.3), "down",
                      ["photo: French Live Fire, Djibouti 170114-Z-BT533-1089.jpg - one leaving under its aft fuselage"])],
    "f22a": [_g("aft fuselage sides", ("fuselage", ("between", AFT), "sides", 0.0), "out",
                ["photo: F-22 put out Flare.jpg - fanning out from its aft fuselage's side",
                 "photo: 433d WS F-22A Flares - Aviation Nation 2019.jpg"])],
    "c130j": [_g("aft fuselage sides", ("fuselage", ("between", AFT), "sides", 0.0), "out",
                 ["photo: C-130J emitting flares at Danish Air Show 2014-06-22.jpg - from both sides of the aft fuselage"])],
    "c17a": [_g("aft fuselage sides", ("fuselage", ("between", AFT), "sides", 0.0), "out",
                ["photo: C-17A Globemaster III expends countermeasure flares (26217307222).jpg - beside the gear "
                 "fairings, aft of the wing", "photo: C-17 Globemaster IIIs deploy flares.jpg"])],
    "su27s": [_g("tail boom", ("fuselage", ("end", -0.6), "top", 0.15), "steep up",
                 ["photo: Su-27 Flanker shoots off false heat targets.jpg, Sukhoi Su-27 flares display.jpg - up and "
                  "out from its tail"])],
    "mig29a": [_g("aft top", ("fuselage", ("between", AFT), "top", 0.3), "up",
                  ["photo: MiG-29 mamci.jpg - above and behind it, from its upper aft fuselage"])],
    "a10c": [_g("wingtips' undersides", ("surface", "wing", 0.96, 0.45), "straight down",
                ["press: IPMS USA review - covered chaff dispensers under the wing tips",
                 "forum: Quora - chaff in the wingtips, 4 a side"], count="4 a side x 30"),
             _g("gear pods' aft ends", ("body", "gear pod", 0.93), "straight down",
                ["encyclopedia: aerospaceweb A-10 gallery - a flare from a dispenser on the aft end of the landing "
                 "gear pod", "forum: Quora - flares by the main gear pods, 4 a side"], count="4 a side x 30")],
    "su25": [_g("tail cone's top", ("fuselage", ("end", -1.2), "top", 0.35), "up",
                ["encyclopedia: Wikipedia Su-25 - ASO-2V along the tailcone's upper surface aft of the fin",
                 "press: Armada International - either side of the vertical stabiliser",
                 "game: War Thunder wiki - upward-firing, above the engines and on the tail"], count="4 x 32"),
             _g("nacelles' tops", ("body", "nacelle", 0.9), "up",
                ["press: warhistory.org - four 32-round dispensers on top of the engine nacelles (after Afghanistan)",
                 "game: War Thunder wiki - above the engines"], count="4 x 32")],
    "b52h": [_g("stabilizers' undersides", ("surface", "htail", 0.45, 0.35), "straight down",
                ["encyclopedia: FAS - 12 AN/ALE-20 flare dispensers",
                 "forum: FineScale (a former B-52 ECM technician) - under the horizontal stabilizers, 6 a side"],
                count="6 a side (ALE-20 flares; the ALE-24 chaff's in the wings, unplaced)")],
    "typhoon": [_g("inner flaperons' fairings", ("surface", "wing", 0.2, 0.85), "down",
                   ["encyclopedia: Wikipedia Praetorian DASS - flares in the actuator housings of the inner "
                    "flaperons, 16 rounds each", "press: JED - in the actuator fairings beneath the wings"],
                   count="2 x 16 (55 mm)"),
                _g("outboard rails' aft ends", ("station",), "down",
                   ["encyclopedia: Wikipedia Praetorian DASS - BOL-510 at the end of the launch rails near the wing tips",
                    "press: JED - integrated in the weapon pylons"], count="2 x 160 (chaff, BOL-510)")],
    "rafale": [_g("flares on top at the wing roots", ("fuselage", ("te", -0.5), "top", 0.7), "steep up",
                  ["forum: secretprojects (from Fox Three, World Airpower Journal, Air & Cosmos) - four "
                   "vertical-firing flare dispensers on top of the fuselage near the wing trailing edges",
                   "forum: a Rafale blog - just forward of the engine exhausts, in the wing roots"], count="4 x 16 (40 mm)"),
               _g("chaff on the rear sides", ("fuselage", ("te", 0.4), "sides", 0.0), "out",
                  ["forum: secretprojects - two chaff dispensers on the rear fuselage sides behind the wings"],
                  count="2 x 112")],
    "su57": [_g("tail sting underside", ("fuselage", ("end", -1.0), "belly", 0.2), "straight down",
                ["encyclopedia: Wikipedia Su-57 (Butowski 2021) - in the tail boom between the engines",
                 "forum: War Thunder forum (after Butowski's diagrams) - 3 facing downward, 2 sideways"], count="3"),
             _g("tail sting sides", ("fuselage", ("end", -1.0), "sides", 0.0), "out",
                ["forum: War Thunder forum (after Butowski) - 2 facing sideways"], count="2")],
    "j10a": [_g("rear fuselage by the inboard flaps", ("fuselage", ("te", -0.8), "belly", 0.8), "down",
                ["press: Air International (Nov 2014) - open chaff-and-flare boxes on the rear fuselage",
                 "forum: War Thunder forum - at the base of the delta wings near the flaps (its direction unstated: "
                 "down assumed)"], count="72 (game: War Thunder)")],
    "f35a": [_g("aft of the weapon bays", ("fuselage", ("gear", 1.8), "belly", 0.35), "straight down",
                ["press: The War Zone - dual flip-open doors on the lower rear fuselage immediately aft of the "
                 "weapons bays (down: its photographs)"])],
}
# (the J-20A and the H-6K: nothing public places them - the rule)


def groups(aircraft):
    """(its dispenser groups, carried): the type's from TYPES, else the rule's for a combat type."""
    if aircraft.name in TYPES:
        return [dict(g) for g in TYPES[aircraft.name]], True
    return [dict(RULE)], aircraft.category in COMBAT


def _fuselage(aircraft, x):
    """(width, top, bottom, the centre's y) of the widest fuselage body on the centreline at x (not a mirrored boom
    out at a tail's root), or None."""
    best = None
    for b in aircraft.bodies:
        if getattr(b, "kind", "") not in ("fuselage", "boom") or not hasattr(b, "top") or not (b.x[0] <= x <= b.x[-1]):
            continue
        w = float(np.interp(x, b.x, b.w))
        if b.mirror or abs(float(np.interp(x, b.x, b.y))) > 0.25 * w:
            continue
        if best is None or w > best[0]:
            best = (w, float(np.interp(x, b.x, b.top)), float(np.interp(x, b.x, b.bottom)), float(np.interp(x, b.x, b.y)))
    return best


def _station_x(aircraft, spec):
    """The fuselage station a group is at: ("between", f) of the way from the wing root's trailing edge to the tail's
    roots (within the centre fuselage: a delta's root can run to its end); ("gear", dx) from the main gear; ("te", dx)
    from the wing root's trailing edge; ("end", dx) from the centre fuselage's end."""
    wings = [s for s in aircraft.surfaces if s.kind == "wing"]
    tails = [s for s in aircraft.surfaces if s.kind in ("htail", "fin", "vtail")]
    centre = [b.x[-1] for b in aircraft.bodies if getattr(b, "kind", "") in ("fuselage", "boom") and not b.mirror]
    if not wings or not centre:
        return None
    te = max(s.sections[0].le[0] + s.sections[0].chord for s in wings)
    end = max(centre)
    kind, v = spec
    if kind == "gear":
        mains = [g.position[0] for g in aircraft.gear if abs(g.position[1]) > 0.3]
        return (max(mains) if mains else te) + v
    if kind == "te":
        return te + v
    if kind == "end":
        return end + v
    ends = [b.x[-1] for b in aircraft.bodies if getattr(b, "kind", "") in ("fuselage", "boom")]
    tail = min(s.sections[0].le[0] for s in tails) if tails else max(ends) - 0.15 * (max(ends) - te)
    if tail <= te:  # (a tail ahead of the wing's trailing edge: a delta's fins) - between it and the body's end
        tail = max(ends)
    le = min(s.sections[0].le[0] for s in wings)
    return min(te + v * (tail - te), end - 0.15 * (end - le))


def _direction(fires, side):
    y, z = FIRES[fires]
    d = np.array([0.0, side * y, z])
    return d / np.linalg.norm(d)


def _anchors(aircraft, g, x_shift=0.0, lift=0.0):
    """(name, a point in the part's middle, the direction it fires in) for each side of the group (design frame: x
    aft, y right, z up), moved x_shift (m) and lift (of the part's depth) by the search."""
    kind = g["where"][0]
    sides = (("left", -1.0), ("right", 1.0))
    out = []
    if kind == "fuselage":
        _, xs, face, off = g["where"]
        x = _station_x(aircraft, xs)
        f = None if x is None else _fuselage(aircraft, x + x_shift)
        if f is None:
            return []
        w, top, bottom, yc = f
        mid = 0.5 * (top + bottom) + lift * (top - bottom)
        for side, s in sides:
            y = yc + s * (0.1 if face == "sides" else off) * 0.5 * w
            out.append(("%s %s" % (side, g["name"]), np.array([x + x_shift, y, mid]), _direction(g["fires"], s)))
    elif kind == "body":
        _, name, frac = g["where"]
        b = next((b for b in aircraft.bodies if b.name == name and hasattr(b, "top")), None)
        if b is None:
            return []
        x = b.x[0] + frac * (b.x[-1] - b.x[0]) + x_shift
        y0 = float(np.interp(x, b.x, b.y))
        top, bottom = float(np.interp(x, b.x, b.top)), float(np.interp(x, b.x, b.bottom))
        for side, s in sides:
            y = s * abs(y0) if b.mirror else y0
            out.append(("%s %s" % (side, g["name"]), np.array([x, y, 0.5 * (top + bottom) + lift * (top - bottom)]),
                        _direction(g["fires"], s)))
    elif kind == "surface":
        _, skind, eta, cf = g["where"]
        surf = next((s for s in aircraft.surfaces if s.kind == skind and s.mirror), None)
        if surf is None:
            return []
        le, chord, _, _ = surf.station(eta)
        for side, s in sides:
            out.append(("%s %s" % (side, g["name"]), np.array([le[0] + cf * chord + x_shift, s * le[1], le[2]]),
                        _direction(g["fires"], s)))
    elif kind == "station":
        from . import carriage
        st = carriage.stations(aircraft)
        if not st:
            return []
        # (the rails are drawn, not in the airframe's solid: its aft end set on the wing's underside above it, where
        # the rail hangs)
        wing = next((s for s in aircraft.surfaces if s.kind == "wing" and s.mirror), None)
        far = max(abs(s["at"][1]) for s in st)
        for s in sorted(st, key=lambda s: s["at"][1]):
            if abs(abs(s["at"][1]) - far) < 1e-3:
                side = -1.0 if s["at"][1] < 0 else 1.0
                x = s["at"][0] + 0.5 * float(s.get("length") or 0.0) + x_shift
                z = s["at"][2]
                if wing is not None:
                    root, tip = wing.sections[0].le, wing.sections[-1].le
                    eta = min(max((abs(s["at"][1]) - root[1]) / max(tip[1] - root[1], 1e-6), 0.0), 1.0)
                    z = wing.station(eta)[0][2]
                out.append(("%s %s" % ("left" if side < 0 else "right", g["name"]),
                            np.array([x, s["at"][1], z]), _direction(g["fires"], side)))
    return out


def _on_skin(probe, inside, direction, reach=8.0, step=0.01):
    """Where the skin is that a cartridge fired along direction leaves: come from outside - reach metres out from the
    part's middle along direction - back in until the solid begins (refined by bisection), so a hollow duct or bay
    inside does not count; None when nothing is met."""
    start = inside + reach * direction
    if probe(start[None, :])[0][0] <= 0.0:
        return None
    ts = np.arange(0.0, reach + 1.0, step)  # (on past the middle a little: a drooped tip's skin can lie beyond it)
    d = probe(start[None, :] - ts[:, None] * direction[None, :])[0]
    hit = np.flatnonzero(d <= 0.0)
    if not len(hit):
        return None
    lo, hi = ts[hit[0]] - step, ts[hit[0]]
    for _ in range(20):
        mid = 0.5 * (lo + hi)
        if probe((start - mid * direction)[None, :])[0][0] <= 0.0:
            hi = mid
        else:
            lo = mid
    return start - lo * direction  # (just outside: on the skin)


def _surface_skin(aircraft, g, inside, direction):
    """A surface group's place on its section's skin: half the section's thickness from its chord plane, the way the
    group fires."""
    _, skind, eta, cf = g["where"]
    surf = next((s for s in aircraft.surfaces if s.kind == skind and s.mirror), None)
    if surf is None:
        return None
    _, chord, _, foil = surf.station(eta)
    return inside + direction * 0.5 * foil.thickness_ratio * chord


def dispensers(aircraft):
    """The dispensers: dicts of name, at (design frame, on the skin), direction (unit, the way they fire), group, basis
    ('sourced' or 'rule'), sources, count and section (the breakup section they are on). Empty for a type that carries
    none, or without hangar's mesher built."""
    from .shape import airframe as sh
    from .shape import breakup
    from .shape import meshkit
    gs, carried = groups(aircraft)
    if not carried or not aircraft.surfaces or meshkit.library() is None:
        return []
    out = []
    with meshkit.Probe(sh.airframe(aircraft)) as probe:
        for g in gs:
            first = _anchors(aircraft, g)
            for k in range(len(first)):
                # its place first, then forward of it (an engine bay or a nozzle's cavity is hollow in the solid, and
                # a flat aft body can let the ray by), at the part's middle and a quarter of its depth either side
                tries = [(0.0, 0.0)] + [(-dx, lift) for dx in np.arange(0.0, 3.01, 0.5) for lift in (0.0, -0.25, 0.25)]
                for dx, lift in tries:
                    c = _anchors(aircraft, g, dx, lift)
                    if len(c) != len(first):
                        continue
                    name, inside, direction = c[k]
                    at = _on_skin(probe, inside, direction)
                    if at is None and g["where"][0] == "surface" and dx == 0.0:
                        # (an all-moving surface - a stabilizer - is cut from the solid as its own part: its skin from
                        # its section, half its thickness off the chord plane, the way it fires)
                        at = _surface_skin(aircraft, g, inside, direction)
                    if at is not None:
                        out.append(dict(name=name, at=at, direction=direction, group=g["name"], basis=g["basis"],
                                        sources=g["sources"], count=g.get("count")))
                        break
    if out:
        where = breakup.classify(aircraft, np.array([d["at"] for d in out]), mounted=True)
        for i, d in enumerate(out):
            d["section"] = "airframe" if where is None else breakup.SECTIONS[int(where[i])]
    return out


def dispenser_lines(aircraft, origin):
    """The manifest's lines: `dispenser <section> <x> <y> <z> <dx> <dy> <dz> <name>` (body axes from the origin: x
    forward, y right, z down; the direction a unit vector)."""
    ds = dispensers(aircraft)
    if not ds:
        return []
    sourced = any(d["basis"] == "sourced" for d in ds)
    basis = "placed from public sources (hangar/dispensers.py TYPES)" if sourced else \
        "the rule's estimate (nothing public places them)"
    lines = ["# chaff and flare dispensers (MS-9a, MS-7h), %s: <section> <place x y z> <direction x y z> (body axes "
             "from the origin, m) <name>" % basis]
    o = np.asarray(origin, float)
    for d in ds:
        q = np.asarray(d["at"], float) - o
        v = [-q[0], q[1], -q[2], -d["direction"][0], d["direction"][1], -d["direction"][2]]
        lines.append("dispenser %s %s %s" % (d["section"].replace(" ", "-"),
                                             " ".join("%.4f" % (x if abs(x) >= 5e-5 else 0.0) for x in v), d["name"]))
    return lines
