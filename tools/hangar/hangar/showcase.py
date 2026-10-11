"""A missile design shown in the viewer before anything flies it (ADR-30, MS-7e): a scripted recording.

    python -m hangar showcase <missile> [--carrier f16c] [--station N] [--launch-s 6] [--duration 30]

writes aircraft/<missile>/out/showcase.json (the script) and showcase.fsrec (the recording, by flightsim-showcase), and
prints the viewer's command that plays it: the carrier flying level with the missile on a station its loadouts put it
on, then the launch - off the rail, or ejected - its motor's thrust through the design's [[motor.phase]]s, its fins
deflected by a scripted manoeuvre. The motion is scripted, not flown: the states are those a flight model writes, so
the viewer draws the model, its moving parts and its effects from them as it will from a flight.
"""
import argparse
import json
import math
import os
import subprocess

from . import pipeline
from .geometry.aircraft import Aircraft
from .files import open_write

CRUISE_MS = 230.0  # the carrier's speed: some Mach 0.72 at 6 km (assumed: a typical launch)


def _station(carrier, missile_card, number=None):
    """The carrier's station for the missile - the one asked for, or the first its loadouts put the missile on, else
    its first rail - and how it releases there: (number, place in body axes from the carrier's CG, "rail"|"eject",
    whether it is in a weapon bay)."""
    root = pipeline.repo_root()
    with open(os.path.join(root, "aircraft", carrier, "payloads.json"), encoding="utf-8") as f:
        fit = json.load(f)
    stations = {}
    for p in fit.get("payloads", []):
        path = os.path.join(root, "assets", "payloads", p.get("card", "") + ".json")
        if os.path.isfile(path):
            with open(path, encoding="utf-8") as f:
                card = json.load(f)
            if card.get("family") == "store_station":
                stations.update({s["number"]: s for s in card["stations"]})
    if not stations:
        raise SystemExit("%s: no store stations (its payloads.json names no store-station card)" % carrier)
    if number is None:
        on = [s["station"] for lo in fit.get("loadouts", []) for s in lo.get("stations", []) if s.get("store") == missile_card]
        rails = [n for n, s in sorted(stations.items()) if "rail" in s.get("carriage", "")]
        number = (sorted(on, key=lambda n: -n) or rails or sorted(stations))[0]
    s = stations[number]
    return number, [float(v) for v in s["position_m"]], "rail" if "rail" in s.get("carriage", "") else "eject", "bay" in s.get("name", "")


# A blast-fragmentation warhead's explosive as a share of its mass (an estimate, until each card gives its charge:
# air-to-air missiles' warheads carry some 35 - 45 % of their mass as explosive, the rest case and fragments)
EXPLOSIVE_SHARE = 0.4


def target_spec(target, heading_deg=270.0, speed_ms=220.0, miss_m=(0.0, 5.0, 0.0), seed=1, separate=None):
    """The showcase's target: a design flown by its own flight model through the first detonation (miss_m from it, NED),
    what the burst breaks on it the world's damage chain's (World::detonate: MS-7g's damage part)."""
    Aircraft.load(os.path.join(pipeline.repo_root(), "aircraft", target, target + ".toml"))  # (a design hangar knows)
    return {"type": "jsbsim:" + target, "name": target.upper(), "speed_ms": float(speed_ms), "heading_deg": float(heading_deg),
            "miss_m": [float(v) for v in miss_m], "seed": int(seed),
            # (a section's root failed outright just after the burst: a scenario's injected failure, World::separateSection)
            **({"separate": [{"section": separate, "after_s": 0.05}]} if separate else {})}


def script(name, carrier="f16c", station=None, launch_s=6.0, duration=30.0, altitude=6000.0, humidity=None, dive_s=None,
           dive_deg=-30.0, detonate_s=None, target=None, miss_m=(0.0, 5.0, 0.0), seed=1, separate=None):
    root = pipeline.repo_root()
    path = os.path.join(root, "aircraft", name, name + ".toml")
    a = Aircraft.load(path)
    if a.motor is None:
        raise SystemExit("%s: no [motor] - nothing to launch" % name)
    card_id = a.spec.get("aircraft", {}).get("card", name)
    with open(os.path.join(root, "assets", "stores", card_id + ".json"), encoding="utf-8") as f:
        card = json.load(f)
    par = card["parameters"]
    number, place, launch, bay = _station(carrier, card_id, station)
    extra = {} if humidity is None else {"environment": {"humidity": float(humidity)}}
    # a dive to the ground: the missile ends where it strikes the map set's terrain, as World ends a weapon
    dive = {} if dive_s is None else {"dive_s": float(dive_s), "dive_deg": float(dive_deg)}
    if target is not None:
        if detonate_s is None:
            raise SystemExit("--target needs --detonate-s: the target is flown through the detonation")
        extra["target"] = target_spec(target, miss_m=miss_m, seed=seed, separate=separate)
    if detonate_s is not None:
        warhead = float(par.get("warhead_kg", {}).get("value", 0.0))
        dive.update({"detonate_s": float(detonate_s), "warhead_kg": warhead,
                     "explosive_kg": float(par.get("explosive_kg", {}).get("value", EXPLOSIVE_SHARE * warhead))})
    if dive_s is not None:
        extra["ground"] = True
    return {
        **extra,
        "name": "%s showcase" % name,
        "duration_s": duration, "rate_hz": 60,
        "start": {"lat_deg": 36.25, "lon_deg": -115.03, "alt_m": float(altitude), "heading_deg": 90.0},
        "carrier": {"type": "jsbsim:" + carrier, "name": carrier.upper(), "speed_ms": CRUISE_MS, "thrust_n": 45000.0, "n2": 92.0},
        "missiles": [{
            "type": "jsbsim:" + name, "card": card_id, "name": card.get("designation", name), "station": place, "station_number": number,
            "launch_s": launch_s, "launch": launch, "bay": bay,
            # an ejector's push and the motor's delay after it (assumed: an AIM-120's from a bay or a fuselage
            # station, some 7 m/s and a quarter second, until each launcher's card says)
            "eject_ms": 7.0 if launch == "eject" else 0.0, "ignition_delay_s": 0.25 if launch == "eject" else 0.0,
            "mass_kg": par["mass_kg"]["value"], "diameter_m": par["diameter_m"]["value"], "cd": 0.35,
            "phases": [[d, 1000.0 * t] for d, t in a.motor.phases],
            "fin_deg": max((max(abs(c.min_deg), abs(c.max_deg)) for s in a.surfaces for c in s.controls), default=15.0),
            **dive,
        }],
    }


def main(argv):
    p = argparse.ArgumentParser(prog="hangar showcase")
    p.add_argument("name")
    p.add_argument("--carrier", default="f16c")
    p.add_argument("--station", type=int)
    p.add_argument("--launch-s", type=float, default=6.0)
    p.add_argument("--duration", type=float, default=30.0)
    p.add_argument("--altitude", type=float, default=6000.0, help="m: the carrier's (10,500 for cold air: an ice contrail)")
    p.add_argument("--humidity", type=float, help="0..1 over water: the day's air (none: the viewer's typical day)")
    p.add_argument("--dive-s", type=float, help="s after its launch: it dives (--dive-deg) and ends where it strikes the ground")
    p.add_argument("--dive-deg", type=float, default=-30.0)
    p.add_argument("--detonate-s", type=float, help="s after its launch: its warhead goes off there (MS-7h's detonation)")
    p.add_argument("--target", help="a design flown by its own flight model through the detonation: what the burst breaks "
                                    "on it the world's damage chain's (MS-7g)")
    p.add_argument("--miss-m", type=float, nargs=3, default=(0.0, 5.0, 0.0), metavar=("N", "E", "D"),
                   help="where the burst is from the target (NED, m; default 5 m east of it)")
    p.add_argument("--seed", type=int, default=1, help="the world's seed: the damage's chances")
    p.add_argument("--separate", help="a section of the target failed outright just after the burst (a scenario's injected "
                                      "failure: 'left wing', 'tail' ...)")
    a = p.parse_args(argv)
    s = script(a.name, a.carrier, a.station, a.launch_s, a.duration, a.altitude, a.humidity, a.dive_s, a.dive_deg,
               a.detonate_s, a.target, tuple(a.miss_m), a.seed, a.separate)
    out = os.path.join(pipeline.repo_root(), "aircraft", a.name, "out")
    os.makedirs(out, exist_ok=True)
    js, rec = os.path.join(out, "showcase.json"), os.path.join(out, "showcase.fsrec")
    with open_write(js, "w", encoding="utf-8", newline="\n") as f:
        json.dump(s, f, indent=2)
    exe = os.path.join(pipeline.repo_root(), "build", "ucrt64-release", "bin", "flightsim-showcase.exe")
    if not os.path.isfile(exe):
        raise SystemExit("%s is not built (cmake --build --preset ucrt64-release --target flightsim-showcase)" % exe)
    subprocess.run([exe, js, rec], check=True)
    m = s["missiles"][0]
    print("%s on %s's station %d (%s launch at %.0f s)" % (m["name"], a.carrier, m["station_number"], m["launch"], m["launch_s"]))
    print("play it: build\\ucrt64-release\\bin\\flightsim-viewer.exe --replay %s --fullscreen --camera chase" % rec)
    return 0
