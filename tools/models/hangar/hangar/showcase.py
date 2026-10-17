"""Record a native SDK missile encounter for the viewer.

    python -m hangar showcase <missile> [--carrier f16c] [--station N] [--launch-s 12] [--duration 40] [--live]

Writes aircraft/<missile>/out/showcase.json, showcase.fsrec and showcase.fsrec.engagement.json.
The SDK owns carriage, release consent, missile motion, seekers, fuzes and damage. A requested launch time is
the earliest strike submission; the sidecar reports actual release and terminal events. Unsupported stores fail
without a substitute trajectory. This preview uses the existing native game models and is not real-world calibration.
Use --live to publish and pace the same SDK World in real time; attach a fullscreen viewer in another terminal.
"""
import argparse
import json
import math
import os
import subprocess

from . import pipeline
from .geometry.aircraft import Aircraft
from .files import open_write

CRUISE_MS = 250.0  # basis: assumed; the stable native engagement acceptance setup.
SCHEMA = "native-engagement-v1"


def _station(carrier, missile_card, number=None):
    """Choose a real SDK station from the carrier's payload cards and loadouts."""
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
        with open(os.path.join(root, "assets", "stores", missile_card + ".json"), encoding="utf-8") as f:
            store = json.load(f)
        parameters = store.get("parameters", {})
        mass = sum(float(parameters.get(key, {}).get("value", 0.0)) for key in ("mass_kg", "fuel_kg"))
        # Choose a legal input using the same card constraints Host::load checks; the SDK remains the final authority.
        compatible = {n: s for n, s in stations.items() if store.get("class") in s.get("takes", []) and
                      int(s.get("capacity", 1)) >= 1 and mass <= float(s.get("max_kg", 0.0))}
        on = [s["station"] for lo in fit.get("loadouts", []) for s in lo.get("stations", [])
              if s.get("store") == missile_card and s["station"] in compatible]
        rails = [n for n, s in sorted(compatible.items()) if "rail" in s.get("carriage", "")]
        choices = sorted(on, key=lambda n: -n) or rails or sorted(compatible)
        if not choices:
            raise SystemExit("%s: no station can take %s under its native card constraints" % (carrier, missile_card))
        number = choices[0]
    if number not in stations:
        raise SystemExit("%s: station %d does not exist" % (carrier, number))
    return number


def target_spec(target, heading_deg=270.0, speed_ms=CRUISE_MS, range_m=5000.0, altitude_offset_m=0.0):
    """An encounter opponent placed ahead of the settled launcher and flown by its own JSBSim model."""
    Aircraft.load(os.path.join(pipeline.repo_root(), "aircraft", target, target + ".toml"))
    return {"type": "jsbsim:" + target, "name": target.upper() + "-TARGET", "speed_ms": float(speed_ms),
            "heading_deg": float(heading_deg), "range_m": float(range_m), "altitude_offset_m": float(altitude_offset_m)}


def script(name, carrier="f16c", station=None, launch_s=12.0, duration=40.0, altitude=6000.0, humidity=None,
           target="f16c", target_range=5000.0, target_altitude_offset=0.0, seed=1):
    root = pipeline.repo_root()
    a = Aircraft.load(os.path.join(root, "aircraft", name, name + ".toml"))
    if a.motor is None:
        raise SystemExit("%s: no [motor] - nothing to launch" % name)
    card_id = a.spec.get("aircraft", {}).get("card", name)
    with open(os.path.join(root, "assets", "stores", card_id + ".json"), encoding="utf-8") as f:
        card = json.load(f)
    if card.get("class") != "aam" or card.get("model") != "jsbsim:" + name or not card.get("seeker"):
        raise SystemExit("%s: no native guided missile card/model" % name)
    values = (launch_s, duration, altitude, target_range, target_altitude_offset)
    if not all(math.isfinite(float(v)) for v in values) or launch_s < 0.0 or duration <= launch_s or target_range <= 0.0:
        raise SystemExit("the native encounter needs finite times and geometry, positive range, and duration after launch")
    if humidity is not None and (not math.isfinite(float(humidity)) or not 0.0 <= humidity <= 1.0):
        raise SystemExit("humidity must be finite and within 0..1")
    number = _station(carrier, card_id, station)
    return {
        "schema": SCHEMA, "name": "%s native encounter" % name, "duration_s": float(duration), "rate_hz": 30,
        "seed": int(seed),
        "start": {"lat_deg": 36.25, "lon_deg": -115.03, "alt_m": float(altitude), "heading_deg": 90.0},
        "carrier": {"type": "jsbsim:" + carrier, "name": carrier.upper(), "speed_ms": CRUISE_MS},
        "target": target_spec(target, range_m=target_range, altitude_offset_m=target_altitude_offset),
        "missiles": [{"card": card_id, "name": card.get("designation", name),
                      "station_number": number, "launch_s": float(launch_s)}],
        **({"environment": {"humidity": float(humidity)}} if humidity is not None else {}),
    }


def main(argv):
    p = argparse.ArgumentParser(prog="hangar showcase", description=__doc__)
    p.add_argument("name")
    p.add_argument("--carrier", default="f16c")
    p.add_argument("--station", type=int)
    p.add_argument("--launch-s", type=float, default=12.0, help="earliest native strike submission; release waits for SDK consent")
    p.add_argument("--duration", type=float, default=40.0)
    p.add_argument("--altitude", type=float, default=6000.0, help="m: the carrier's (10,500 for cold air: an ice contrail)")
    p.add_argument("--humidity", type=float, help="0..1 over water: the world's humidity")
    p.add_argument("--target", default="f16c", help="opponent flown by its native JSBSim model")
    p.add_argument("--target-range", type=float, default=5000.0, help="m ahead of the launcher after its native settle")
    p.add_argument("--target-altitude-offset", type=float, default=0.0, help="m above the settled launcher's altitude")
    p.add_argument("--seed", type=int, default=1, help="the native world's deterministic seed")
    p.add_argument("--live", action="store_true", help="publish and pace the same native SDK World in real time; attach a viewer separately")
    a = p.parse_args(argv)
    s = script(a.name, a.carrier, a.station, a.launch_s, a.duration, a.altitude, a.humidity,
               a.target, a.target_range, a.target_altitude_offset, a.seed)
    preset = os.environ.get("FSIM_PRESET", "ucrt64-headless")
    bin_dir = os.environ.get("FSIM_BIN") or os.path.join(pipeline.repo_root(), "build", preset, "bin")
    exe = os.path.join(bin_dir, "flightsim-showcase.exe")
    if not os.path.isfile(exe):
        raise SystemExit("selected native showcase is not built: %s (fsim --preset %s build --target flightsim-showcase)" % (exe, preset))
    out = os.path.join(pipeline.repo_root(), "aircraft", a.name, "out")
    os.makedirs(out, exist_ok=True)
    js, rec = os.path.join(out, "showcase.json"), os.path.join(out, "showcase.fsrec")
    with open_write(js, "w", encoding="utf-8", newline="\n") as f:
        json.dump(s, f, indent=2)
    sidecar = rec + ".engagement.json"
    if os.path.isfile(sidecar):
        os.remove(sidecar)  # A failed generation cannot inherit an earlier native success.
    command = [exe, js, rec]
    if a.live:
        print('live viewer (start in another terminal): fsim --preset ucrt64-release viewer --world "%s" --fullscreen --camera chase' %
              s["name"], flush=True)
        command.append("--live")
    subprocess.run(command, check=True)
    with open(sidecar, encoding="utf-8") as f:
        events = json.load(f)
    if events.get("schema") != SCHEMA or events.get("source") != "native-sdk" or not events.get("weapons"):
        raise SystemExit("showcase returned no native SDK release provenance")
    m, weapon = s["missiles"][0], events["weapons"][0]
    print("%s on %s's station %d: native release at %.3f s, outcome %s" %
          (m["name"], a.carrier, m["station_number"], weapon["launch_time_s"], weapon["terminal_kind"]))
    print('play it: fsim --preset ucrt64-release viewer --replay "%s" --fullscreen --camera chase --follow %d' %
          (rec, weapon["weapon_slot"]))
    return 0
