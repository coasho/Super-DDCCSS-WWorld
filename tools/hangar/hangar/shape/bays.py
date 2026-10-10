"""Weapon bays (ADR-30, MS-7e1): each bay's stations (the store-station card's, grouped by name: hangar/carriage.py) cut
into the airframe's underside as a dark cavity, and its doors - the skin the cut takes away, a shell of it over the
opening - each a part of its own hinged along its outer edge, turning down as the store management opens the bay
(fsim:bay:<bay>:<deg>; the viewer turns it by StoreInfo::bayDoors).

The cavity: a box round its stations' stores - their length fore and aft, their fins' span across and up (as they hang
in an X) - with MARGIN to spare, from below the skin up to its ceiling just over the stores. A bay wider than
SPLIT_WIDTH has two doors, split down its middle, each hinged on its outer edge (the F-22A's, the F-35A's main bays); a
narrower one a single door hinged on its outboard edge (a side bay). Each opens DOOR_DEG. All of it estimated from the
stations' places, the same way for every aircraft: a bay's real shape - its doors' sawtooth edges, the F-22's side bays'
doors hinged above - is not drawn.
"""
import numpy as np

MARGIN = 0.10       # m round the stores (assumed)
CEILING = 0.05      # m over them
SPLIT_WIDTH = 1.2   # m: wider, two doors (assumed)
DOOR_DEG = 100.0    # how far a door opens (assumed: past the vertical, as the F-22's and F-35's main bays' do)
DOOR_GAP = 0.012    # m between a door and the opening's edge (as a control surface's cut)
DOOR_SKIN = 0.035   # m: a door's thickness, the skin's shell it is cut as


def bays(aircraft):
    """Each bay: its index (1..), name, its stations' numbers, the cavity's box (design frame: x0, x1, y0, y1, z0, z1)
    and its doors (each: the box of the skin it is cut from, y0, y1; which edge it hinges on: -1 its low y, +1 its high y)."""
    from ..carriage import stations
    groups = {}
    for s in stations(aircraft):
        if s["bay"]:
            groups.setdefault(s["bay_index"], []).append(s)
    out = []
    for k in sorted(groups):
        sts = groups[k]
        half = [max(0.5 * s["diameter"], 0.5 * s["span"]) for s in sts]  # (the fins of an X reach 0.71 of it up)
        up = [max(0.5 * s["diameter"], 0.354 * s["span"]) for s in sts]
        x0 = min(s["at"][0] - 0.5 * s["length"] for s in sts) - MARGIN
        x1 = max(s["at"][0] + 0.5 * s["length"] for s in sts) + MARGIN
        y0 = min(s["at"][1] - h for s, h in zip(sts, half)) - MARGIN
        y1 = max(s["at"][1] + h for s, h in zip(sts, half)) + MARGIN
        z1 = max(s["at"][2] + u for s, u in zip(sts, up)) + CEILING
        z0 = min(s["at"][2] - u for s, u in zip(sts, up)) - 1.0  # (well below the skin: the opening)
        if y1 - y0 > SPLIT_WIDTH:
            yc = 0.5 * (y0 + y1)
            doors = [(y0, yc, -1), (yc, y1, +1)]
        else:
            outboard = +1 if abs(y1) > abs(y0) else -1
            doors = [(y0, y1, outboard)]
        out.append(dict(index=k, name=sts[0]["bay_name"], stations=[s["number"] for s in sts],
                        box=(x0, x1, y0, y1, z0, z1), doors=doors))
    return out


def _box(x0, x1, y0, y1, z0, z1, material):
    return {"prim": "box", "material": material, "centre": [0.5 * (x0 + x1), 0.5 * (y0 + y1), 0.5 * (z0 + z1)],
            "half": [0.5 * (x1 - x0), 0.5 * (y1 - y0), 0.5 * (z1 - z0)], "axes": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]}


def cavities(aircraft, material):
    """The bays' boxes, to cut from the airframe (their cut surfaces dark)."""
    return [_box(*b["box"], material) for b in bays(aircraft)]


def door_scenes(aircraft, skin, material):
    """Each door as a scene of its own for the mesher: (bay, the edge it hinges on, its scene) - the skin's shell (the
    solid less itself drawn in by DOOR_SKIN) within its part of the opening, a DOOR_GAP short of each edge."""
    out = []
    for b in bays(aircraft):
        x0, x1, _, _, z0, z1 = b["box"]
        for ya, yb, edge in b["doors"]:
            region = _box(x0 + DOOR_GAP, x1 - DOOR_GAP, ya + DOOR_GAP, yb - DOOR_GAP, z0, z1, material)
            shell = {"op": "subtract", "k": 0.0, "a": {"op": "intersect", "k": 0.0, "children": [skin["root"], region]},
                     "b": {"op": "offset", "r": -DOOR_SKIN, "child": skin["root"]}}
            out.append((b["index"], edge, {"foils": skin["foils"], "root": shell}))
    return out


def cover_scenes(aircraft, skin, material):
    """Each bay's cover, for the radar cross section only (fsim:cover:<bay>; the viewer does not draw it): the skin's
    shell over its whole opening, no gap - the bay shut as it is in flight, its doors closed flush against their seals.
    The doors' DOOR_GAP is the viewer's, so they can turn: seen by the radar's rays it was a slot into the bay, a cavity
    that returned (diffusely) more than a stealth airframe's whole front, where a 12 mm slot is below its cutoff (half a
    wavelength) at every band but Ku and a real door's seams are sealed. (bay, scene)"""
    out = []
    for b in bays(aircraft):
        x0, x1, y0, y1, z0, z1 = b["box"]
        region = _box(x0, x1, y0, y1, z0, z1, material)
        shell = {"op": "subtract", "k": 0.0, "a": {"op": "intersect", "k": 0.0, "children": [skin["root"], region]},
                 "b": {"op": "offset", "r": -DOOR_SKIN, "child": skin["root"]}}
        out.append((b["index"], {"foils": skin["foils"], "root": shell}))
    return out
