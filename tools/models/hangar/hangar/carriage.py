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
import hashlib
import math
from functools import lru_cache
from pathlib import Path
import threading
import copy

import numpy as np

from . import pipeline

RAIL = dict(length_share=0.75, width=0.12, depth=0.10)
RACK = dict(length=0.9, width=0.25, depth=0.14)
PYLON_THICK = 0.12
EMBED = 0.06  # m: how far a pylon reaches into the skin above it (its top hidden inside)
DEFAULT = {"rail": (3.66, 0.1778, 0.483), "ejector": (2.21, 0.273, 0.40)}  # m: an AIM-120C's, a Mk 82's (their cards; its fins assumed)
_CONTROL_DESIGN_REPORTS = {}
CONTROL_DESIGN_MODEL_SET = 'fsim.store-yaw-control-idealizations.v1'
CONTROL_DESIGN_SOURCES = {
    'attached': 'https://ntrs.nasa.gov/api/citations/19930091249/downloads/19930091249.pdf',
    'nominal_separated': 'https://ntrs.nasa.gov/api/citations/19770023140/downloads/19770023140.pdf',
}


def _store_card(card_id):
    path = os.path.join(pipeline.repo_root(), "assets", "stores", card_id + ".json")
    if not os.path.isfile(path):
        return None
    with open(path, encoding="utf-8") as f:
        return json.load(f)


@lru_cache(maxsize=64)
def _store_model(path, source_hash, calibration_hash):
    # Source tables are ready after the fleet aero stage. Reading them here
    # avoids depending on whether a concurrent store publisher has run yet.
    from . import airframe
    from .verify import TableModel
    d=airframe.design(path)
    return d.aircraft,TableModel(d.calibrated(d.tables()))


def _yaw_gradient(store, position, mach, alpha_deg):
    """Neutral store yaw derivative about the carrier CG, in m^3/rad.

    Use the same wind/body transform and reference arm as StoreFlightBinding.
    The source model supplies both force and moment; no scalar store formula.
    """
    from .verify import body_forces
    if not store.get('model','').startswith('jsbsim:'):
        raise ValueError('Carried store requires an authored VLM model')
    name=store['model'].removeprefix('jsbsim:')
    if not name or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_' for c in name):
        raise ValueError('Carried store requires an authored VLM model')
    path=Path(pipeline.repo_root())/'aircraft'/name/(name+'.toml')
    calibration=path.parent/'calibration.toml'
    digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest() if p.is_file() else ''
    a,model=_store_model(str(path),digest(path),digest(calibration))
    origin=np.asarray(a.spec['mass']['empty_cg'],float)
    arm=np.asarray(position,float)+(a.aero_point-origin)*np.array([-1.,1.,-1.])
    h=1.e-4
    values=[]
    for beta in (-h,h):
        coefficient=model.evaluate(alpha_deg,math.degrees(beta),mach=mach)
        force,moment=body_forces(coefficient,math.radians(alpha_deg),beta,1.,a.S,a.b,a.c)
        values.append(moment[2]+np.cross(arm,force)[2])
    return float((values[1]-values[0])/(2*h))


def _attached_recovery_design_model(body):
    """Control-design alternative; nominal aerodynamic integration is unchanged.

    NACA TR184 pp15-17 distinguishes attached potential flow and real rear
    recovery. NASA CR145251 pp18-19 supplies the nominal separation model.
    Neither case is a measured or continuous real-load bound.
    """
    model=copy.copy(body)
    model.parts=[dict(p,potential=np.ones_like(p['potential'])) for p in body.parts]
    return model


@lru_cache(maxsize=32)
def _recovery_models(path, source_hash, calibration_hash, mach):
    from .aero.model import AeroModel
    aircraft,_=_store_model(path,source_hash,calibration_hash)
    nominal=AeroModel(aircraft,mach=mach)
    return nominal,_attached_recovery_design_model(nominal.bodies),threading.RLock()


@lru_cache(maxsize=256)
def _recovery_gradient(path, source_hash, calibration_hash, mach, alpha_deg):
    from .aero.model import wind_axes,to_body
    flow,attached,lock=_recovery_models(path,source_hash,calibration_hash,mach)
    alpha=math.radians(alpha_deg);h=1.e-4;differences=[]
    with lock:  # The lattice's wake selection and per-condition flow are mutable.
        for beta in (-h,h):
            result=flow.evaluate(alpha,beta,detail=True)
            induced=result.get('detail',{}).get('body_induced')
            xw,_,_=wind_axes(alpha,beta)
            velocity=np.array([xw[0],-xw[1],xw[2]])
            force,moment=flow.bodies.forces(velocity,np.zeros(3),flow.ref,induced)
            other_force,other_moment=attached.forces(velocity,np.zeros(3),flow.ref,induced)
            # Common forces use rho=1, |V|=1: q=1/2. Normalize by that
            # dynamic pressure, as for the nominal shared-table prediction.
            differences.append(np.r_[to_body(other_force-force),to_body(other_moment-moment)]/.5)
    return tuple((differences[1]-differences[0])/(2*h))


def _yaw_design_cases(store, position, mach, alpha_deg):
    """Two declared control-design idealizations, not measured load bounds.

    Keep the nominal shared-table prediction. Its alternative changes only
    the body's potential-pressure recovery using the common force integrator;
    fin loads, finite endpoints and the carrier-CG moment transfer remain.
    NACA TR184 pp15-17 motivates this design uncertainty. NASA CR145251
    pp18-19 describes the nominal separated-pressure idealization. Neither
    establishes that all real shapes/conditions lie between these two cases.
    """
    nominal_gradient=_yaw_gradient(store,position,mach,alpha_deg)
    from .aero.mach import MACH_SUB
    path=Path(pipeline.repo_root())/'aircraft'/store['model'].removeprefix('jsbsim:')
    source=path/(path.name+'.toml');calibration=path/'calibration.toml'
    digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest() if p.is_file() else ''
    source_hash,calibration_hash=digest(source),digest(calibration)
    aircraft,_=_store_model(str(source),source_hash,calibration_hash)
    origin=np.asarray(aircraft.spec['mass']['empty_cg'],float)
    arm=np.asarray(position,float)+(aircraft.aero_point-origin)*np.array([-1.,1.,-1.])
    # This pressure-recovery idealization is subsonic. Use the same upper
    # PG boundary as the common Mach build; above it hold its correction as
    # an explicitly estimated continuation. The nominal table keeps its full
    # scheduled Mach response and unchanged fin loads.
    recovery_mach=min(float(mach),float(MACH_SUB[-1]))
    delta=np.asarray(_recovery_gradient(str(source),source_hash,calibration_hash,recovery_mach,float(alpha_deg)))
    alternative=nominal_gradient+float(delta[5]+np.cross(arm,delta[:3])[2])
    if not all(math.isfinite(v) for v in (nominal_gradient,alternative)):
        raise ValueError('Nonfinite store control-design idealization')
    root=Path(pipeline.repo_root())
    versions={str(source.relative_to(root)):source_hash}
    if calibration.is_file():versions[str(calibration.relative_to(root))]=calibration_hash
    return dict(derivatives=dict(nominal_shared=nominal_gradient,attached_recovery_design=alternative),
        recovery_mach=recovery_mach,recovery_mach_hold=float(mach)>recovery_mach,
        recovery_mach_domain=[0.,float(MACH_SUB[-1])],source_sha256=versions,
        frozen_contributions='Nominal shared-table fin loads; identical geometry/endpoints/CG transfer; PG induced velocity at recovery_mach held in the recovery comparison')


@lru_cache(maxsize=64)
def _empty_cg(aircraft):
    from .mass import MassModel
    return np.asarray(MassModel(aircraft).empty()['cg'],float)


def directional_load(aircraft, mach=.3, alpha_deg=0., cg=None):
    """Control-sizing penalty across loadouts and two declared flow ideals.

    This is not the nominal native force or a certified real-load envelope.
    See directional_load_cases for both predictions and their provenance.
    """
    return directional_load_cases(aircraft,mach,alpha_deg,cg)['selected_station_sum_m3_per_rad']


def directional_load_cases(aircraft, mach=.3, alpha_deg=0., cg=None):
    """What the stores an aircraft's loadouts hang on it take from its weathercock stability, at the most: its yawing
    moment per unit of dynamic pressure and of sideslip (m^3; negative: the nose turned away from the wind), the sum
    over its stations of the worst store its loadouts put on each. Differentiate the common neutral aircraft
    tables at the scheduled Mach/alpha, including the store's moment and its force arm to the carrier CG.
    Internal bays/holds are shielded, as in the native binding. A store that steadies it is not counted: it may not be
    carried. 0 without stations or loadouts. Divided by the wing's area and span, the change of Cn_beta."""
    report=dict(method=CONTROL_DESIGN_MODEL_SET,basis='estimated',sources=CONTROL_DESIGN_SOURCES,
        conditions=dict(mach=float(mach),alpha_deg=float(alpha_deg),carrier_cg_m=None if cg is None else list(map(float,cg))),
        assumptions='Control sizing against nominal shared tables and attached-pressure recovery; not a continuous separation or measured real-load bound',
        rows=[],station_selections=[],selected_station_sum_m3_per_rad=0.)
    fit = os.path.join(aircraft.dir, "payloads.json")
    if not os.path.isfile(fit):
        return report
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
        return report
    places = {s["number"]: s for s in card["stations"]}
    shift=np.zeros(3)
    if cg is not None:
        shift=(np.asarray(cg,float)-_empty_cg(aircraft))*np.array([-1.,1.,-1.])
    worst = {};winning={}
    derivatives={}
    for lo in fit.get("loadouts", []):
        for e in lo.get("stations", []):
            st, store = places.get(e.get("station")), _store_card(e.get("store", ""))
            if st is None or store is None:
                continue
            name = st.get("name", "")
            if "bay" in name:
                continue
            key=(st['number'],store['id'])
            if key not in derivatives:
                evidence=_yaw_design_cases(store,np.asarray(st['position_m'],float)-shift,mach,alpha_deg)
                cases=evidence.pop('derivatives')
                selected=min(cases,key=cases.get)
                derivatives[key]=cases[selected]
                report['rows'].append(dict(station=st['number'],store=store['id'],derivatives_m3_per_rad=cases,selected_case=selected,**evidence))
            moment=derivatives[key]
            count=int(e.get('count',1));penalty=moment*count;station=st['number']
            if station not in worst or penalty<worst[station]:
                worst[station]=min(0.,penalty)
                winning[station]=dict(station=station,store=store['id'],count=count,
                    per_unit_m3_per_rad=moment,selected_penalty_m3_per_rad=worst[station])
    report['selected_station_sum_m3_per_rad']=float(sum(worst.values()))
    report['station_selections']=[winning[k] for k in sorted(winning)]
    _CONTROL_DESIGN_REPORTS[(aircraft.name,float(mach),float(alpha_deg),tuple(map(float,cg)) if cg is not None else None)]=report
    return report


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
        bay = name[:name.find("bay") + 3] if "bay" in name and not hold else ""  # its weapon bay (src/simulation/mission_systems/Cards.cpp's rule): "main bay"
        out.append(dict(number=s["number"], name=name, at=cg + np.array([-b[0], b[1], -b[2]]), body=b,
                        carriage=s.get("carriage", ""), bay=bool(bay), bay_name=bay, kind=kind, hold=hold,
                        length=size[0], diameter=size[1], span=size[2], rack=s.get("rack", "")))
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


def _rack(group, positions, triangles):
    """A multiple rack's beam and its one pylon (design frame): the beam along its places, between its upper stores (its
    under side on its lowest place's rack, its top above its highest stores'), and a pylon from the beam up into the
    skin over its middle. [] where no skin is above it."""
    from .model3d import _box
    at = np.array([s["at"] for s in group])
    r = max(0.5 * s["diameter"] for s in group)
    half = 0.5 * max(s["length"] for s in group)
    x0, x1 = at[:, 0].min() - 0.8 * half, at[:, 0].max() + 0.8 * half
    y = float(np.mean(at[:, 1]))
    bottom = at[:, 2].min() + r + RACK["depth"] - 0.01          # on the lowest places' racks
    top = at[:, 2].max() + r + RACK["depth"] + 0.08             # over the highest places' racks
    width = max(0.22, float(np.ptp(at[:, 1])) - 2.0 * r + 0.10)  # between the shoulders' stores
    pieces = [_box([x0, y, 0.5 * (bottom + top)], [x1, y, 0.5 * (bottom + top)], width, top - bottom)]
    chord = float(np.clip(0.6 * (x1 - x0), 1.2, 3.5))
    xm = 0.5 * (x0 + x1)
    p0, p1 = xm - 0.55 * chord, xm + 0.45 * chord
    hits = [_skin_above(positions, triangles, x, y, top) for x in np.linspace(p0, p1, 5)]
    hits = [h for h in hits if h is not None]
    if hits:
        pieces.append(_blade(p0, p1, y, top - 0.01, max(hits) + EMBED, 1.5 * PYLON_THICK))
    return pieces


def parts(aircraft, positions, triangles):
    """Each station's carriage: (number, [(vertices design frame, triangles), ...]) - its launcher or rack, and its
    pylon where the skin is further above than the launcher's depth."""
    from .model3d import _box
    out = []
    every = stations(aircraft)
    racks = {}
    for s in every:
        if s["rack"] and not (s["bay"] or s["hold"]):
            racks.setdefault(s["rack"], []).append(s)
    beams = {name: _rack(group, positions, triangles) for name, group in racks.items()}
    for s in every:
        if s["bay"] or s["hold"]:
            continue
        if s["rack"]:
            # a place on a multiple rack: its own ejector rack on its store, and the beam and the pylon it shares with
            # the rack's other places - drawn with each of them, so that whichever is loaded shows them
            P, r = s["at"], 0.5 * s["diameter"]
            bx, bt = _box([P[0] - 0.5 * RACK["length"], P[1], P[2] + r + 0.5 * RACK["depth"]],
                          [P[0] + 0.5 * RACK["length"], P[1], P[2] + r + 0.5 * RACK["depth"]], RACK["width"], RACK["depth"])
            out.append((s["number"], [(bx, bt)] + beams[s["rack"]]))
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
