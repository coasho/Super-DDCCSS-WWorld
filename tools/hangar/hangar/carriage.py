"""The carriage on an aircraft's store stations (ADR-30, MS-7e1): each station's launcher or rack and the pylon that holds
it under the skin, drawn as a node of its own (fsim:station:<n>) the viewer shows once a store has been on that
station; and the stations' places in the model's manifest (`station <n> <x> <y> <z> <bay>`), where the viewer
hangs each store.

The stations are the aircraft's store-station card's (assets/payloads, named by its payloads.json): a store's CG from
the aircraft's empty CG, body axes. What holds a store there, by the card's carriage and the store's class:
- a missile on a rail: a launcher rail along the top of its body, three quarters of its length (an LAU-129 for an
  AIM-120's 3.66 m: 2.7 m), 0.12 m wide, 0.10 m deep;
- anything else (an ejector, a bomb on a rail-or-ejector station): a rack 0.9 m long, 0.25 m wide, 0.14 m deep;
- above either, a pylon from its top up to the skin - a streamlined blade, 0.12 m thick, its chord from the store's
  length - where the skin is further than the launcher's own depth above it; nearer, a rack fills the gap up into the
  skin, and a rail is the airframe's own launcher's (a wingtip's: there nothing is added).
A station in a weapon bay (its card's name says so) has no pylon: its store hangs in the bay (shape/bays.py). A cargo
hold's pallet position (a station that takes cargo) has nothing at all - no rack, and no bay cut into the skin: the
hold is the fuselage as it is drawn, and its load rides inside it. The
store a station is sized for is the largest any of the aircraft's loadouts put there (none: an AIM-120's size on a
rail, a Mk 82's on an ejector). All of it estimated, the same way for every aircraft; drawn only - no drag, no mass
(the stores' masses are the stations' point masses), not in the RCS tables of the clean aircraft.
"""
import json
import os

import numpy as np

from . import pipeline

RAIL = dict(length_share=0.75, width=0.12, depth=0.10)
RACK = dict(length=0.9, width=0.25, depth=0.14)
PYLON_THICK = 0.12
EMBED = 0.06  # m: how far a pylon reaches into the skin above it (its top hidden inside)
DEFAULT = {"rail": (3.66, 0.1778, 0.483), "ejector": (2.21, 0.273, 0.40)}  # m: an AIM-120C's, a Mk 82's (their cards; its fins assumed)


def _store_card(card_id):
    path = os.path.join(pipeline.repo_root(), "assets", "stores", card_id + ".json")
    if not os.path.isfile(path):
        return None
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def _size(card):
    """A store's length, its diameter and its fins' span (m), from its card: a cargo store's box as the cylinder round it."""
    par = card["parameters"]
    if "diameter_m" not in par:
        across = max(par["width_m"]["value"], par["height_m"]["value"])
        return par["length_m"]["value"], across, across
    d = par["diameter_m"]["value"]
    return par["length_m"]["value"], d, par["span_m"]["value"] if "span_m" in par else 2.7 * d


def stations(aircraft):
    """The aircraft's stations, each: number, name, place (design frame, m), carriage, whether in a bay, what it is
    sized for (`rail` or `ejector`, the store's length and diameter). [] without a store-station card."""
    fit = os.path.join(aircraft.dir, "payloads.json")
    if not os.path.isfile(fit):
        return []
    with open(fit, encoding="utf-8") as f:
        fit = json.load(f)
    card = None
    for p in fit.get("payloads", []):
        path = os.path.join(pipeline.repo_root(), "assets", "payloads", p.get("card", "") + ".json")
        if os.path.isfile(path):
            with open(path, encoding="utf-8") as f:
                c = json.load(f)
            if c.get("family") == "store_station" and "stations" in c:
                card = c
    if card is None:
        return []
    # the stores the loadouts put on each station
    on = {}
    for lo in fit.get("loadouts", []):
        for s in lo.get("stations", []):
            store = _store_card(s.get("store", ""))
            if store is not None:
                on.setdefault(s["station"], []).append(store)
    from .mass import MassModel
    cg = np.asarray(MassModel(aircraft).empty()["cg"], float)
    out = []
    for s in card["stations"]:
        b = np.asarray(s["position_m"], float)
        stores = on.get(s["number"], [])
        missiles = [x for x in stores if x.get("class") == "aam"]
        kind = "rail" if "rail" in s.get("carriage", "") and (missiles or not stores) else "ejector"
        if s.get("carriage") == "rail":
            kind = "rail"
        sized = missiles if kind == "rail" else stores
        if sized:
            biggest = max(sized, key=lambda x: _size(x)[0] * _size(x)[1])
            size = _size(biggest)
        else:
            size = DEFAULT[kind]
        name = s.get("name", "")
        # a cargo hold's pallet position (it takes cargo; named "cargo bay <n>", so that the store management opens the
        # hold's ramp for it and gives its load no drag): inside the airframe as it is drawn - no cavity is cut for it,
        # no door, and no rack or pylon hung
        hold = "cargo" in s.get("takes", [])
        bay = name[:name.find("bay") + 3] if "bay" in name and not hold else ""  # its weapon bay (src/ms/Cards.cpp's rule): "main bay"
        out.append(dict(number=s["number"], name=name, at=cg + np.array([-b[0], b[1], -b[2]]), body=b,
                        carriage=s.get("carriage", ""), bay=bool(bay), bay_name=bay, kind=kind, hold=hold,
                        length=size[0], diameter=size[1], span=size[2]))
    # the bays numbered 1.. in the order of their first stations: the manifest's and the doors' (fsim:bay:<n>) numbers
    order = []
    for s in out:
        if s["bay_name"] and s["bay_name"] not in order:
            order.append(s["bay_name"])
    for s in out:
        s["bay_index"] = order.index(s["bay_name"]) + 1 if s["bay_name"] else 0
    return out


def _skin_above(positions, triangles, x, y, z0):
    """How high the skin is straight above (x, y) from z0 (design frame, m): the lowest surface over it, None if none
    within 3 m - a vertical ray against the airframe's triangles."""
    v = positions[triangles]  # (n, 3, 3)
    a, b, c = v[:, 0, :2], v[:, 1, :2], v[:, 2, :2]
    p = np.array([x, y])
    d = (b[:, 1] - c[:, 1]) * (a[:, 0] - c[:, 0]) + (c[:, 0] - b[:, 0]) * (a[:, 1] - c[:, 1])
    ok = np.abs(d) > 1e-12
    d = np.where(ok, d, 1.0)
    w0 = ((b[:, 1] - c[:, 1]) * (p[0] - c[:, 0]) + (c[:, 0] - b[:, 0]) * (p[1] - c[:, 1])) / d
    w1 = ((c[:, 1] - a[:, 1]) * (p[0] - c[:, 0]) + (a[:, 0] - c[:, 0]) * (p[1] - c[:, 1])) / d
    w2 = 1.0 - w0 - w1
    inside = ok & (w0 >= 0) & (w1 >= 0) & (w2 >= 0)
    z = w0 * v[:, 0, 2] + w1 * v[:, 1, 2] + w2 * v[:, 2, 2]
    z = z[inside & (z > z0) & (z < z0 + 3.0)]
    return float(z.min()) if z.size else None


def _blade(x0, x1, y, z0, z1, thick, n=12):
    """A pylon's blade: a lenticular section from x0 (its leading edge, forward: smaller x) to x1, swept by nothing,
    from z0 up to z1 (design frame); a closed prism."""
    u = np.linspace(0.0, 1.0, n)
    xs = x0 + (x1 - x0) * u
    half = 0.5 * thick * np.sqrt(np.clip(1.0 - (2.0 * u - 1.0) ** 2, 0.0, 1.0)) ** 0.8
    outline = [(xs[k], y + half[k]) for k in range(n)] + [(xs[k], y - half[k]) for k in range(n - 2, 0, -1)]
    m = len(outline)
    v = np.array([[px, py, z0] for px, py in outline] + [[px, py, z1] for px, py in outline])
    t = []
    for k in range(m):
        j = (k + 1) % m
        t += [(k, j, m + j), (k, m + j, m + k)]
    for k in range(1, m - 1):  # the caps, as fans
        t += [(0, k + 1, k), (m, m + k, m + k + 1)]
    return v, np.array(t)


def parts(aircraft, positions, triangles):
    """Each station's carriage: (number, [(vertices design frame, triangles), ...]) - its launcher or rack, and its
    pylon where the skin is further above than the launcher's depth."""
    from .model3d import _box
    out = []
    for s in stations(aircraft):
        if s["bay"] or s["hold"]:
            continue
        P, r, L = s["at"], 0.5 * s["diameter"], s["length"]
        top = P[2] + r
        if s["kind"] == "rail":
            length, width, depth = RAIL["length_share"] * L, RAIL["width"], RAIL["depth"]
        else:
            length, width, depth = RACK["length"], RACK["width"], RACK["depth"]
        skin = _skin_above(positions, triangles, P[0], P[1], top)
        if skin is None:
            continue  # nothing to hang from
        if skin - top < depth + 0.02:
            if s["kind"] == "rail":
                continue  # held by the airframe's own launcher (a wingtip's)
            # a rack in a tight gap (a fuselage station close under the belly): it fills it, up into the skin
            bx, bt = _box([P[0] - 0.5 * length, P[1], 0.5 * (top + skin + EMBED)], [P[0] + 0.5 * length, P[1], 0.5 * (top + skin + EMBED)],
                          width, skin + EMBED - top)
            out.append((s["number"], [(bx, bt)]))
            continue
        pieces = []
        # the launcher (or rack): its bottom on the store's top, centred on the store's CG
        bx, bt = _box([P[0] - 0.5 * length, P[1], top + 0.5 * depth], [P[0] + 0.5 * length, P[1], top + 0.5 * depth], width, depth)
        pieces.append((bx, bt))
        # the pylon: up from the launcher to the skin, into it; its chord the skin's lowest over it
        chord = float(np.clip(0.45 * L + 0.4, 0.8, 2.4))
        x0, x1 = P[0] - 0.55 * chord, P[0] + 0.45 * chord
        hits = [_skin_above(positions, triangles, x, P[1], top) for x in np.linspace(x0, x1, 5)]
        hits = [h for h in hits if h is not None]
        z1 = (max(hits) if hits else skin) + EMBED
        pv, pt = _blade(x0, x1, P[1], top + depth - 0.01, z1, PYLON_THICK)
        pieces.append((pv, pt))
        out.append((s["number"], pieces))
    return out


def station_lines(aircraft, origin):
    """The manifest's lines for the stations: `station <n> <x> <y> <z> <bay 0|1>`, a store's CG in body axes from the
    model's origin (x forward, y right, z down; m)."""
    out = []
    for s in stations(aircraft):
        if not out:
            out.append("# store stations: <number> <x> <y> <z> (a store's CG, body axes from the origin, m) <its bay: 0 none>")
        q = s["at"] - np.asarray(origin, float)
        out.append("station %d %.4f %.4f %.4f %d" % (s["number"], -q[0], q[1], -q[2], s["bay_index"]))
    return out
