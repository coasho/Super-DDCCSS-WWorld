"""The viewer's frame rate on fixed scenes, at 3840 x 2160 fullscreen, against a target (docs/sdk/viewer.md).

    python tools/validation/viewer_perf/viewer_perf.py [--build build/ucrt64-release] [--scene fleet strike hit ...]
                                            [--target 160] [--out <dir>] [--label <name>] [-- <viewer arguments>]

Each scene is a world the simulation flies (or a recording of one), drawn by flightsim-viewer --stats; the viewer's
per-second statistics are read and held to the target:

    fleet    all 35 aircraft with their loads, line abreast, turning together (live; no fire)
    strike   an F-16C flown into the sea at 60 deg and 233 m/s: its fireball and the fire that burns on (live; NVIDIA
             Flow solving from the strike)
    hit      an SDK AIM-120C radar engagement against a native F-16C: release, guidance and fuze events recorded
             by flightsim-showcase from hit.json; damage and any fire or breakup come from the SDK outcome
    reheat   an F-16C in full reheat through the transonic, close: its flame, shock train and vapour cone (live)

The picture is held as well as the rate (an optimisation that changes what is drawn is not one):

    viewer_perf.py --shots <label>            each scene's recording replayed on its own clock, frames saved at fixed
                                              times of it (<out>/shots/<label>/<scene>_NNN.png, 3840 x 2160, no panels)
    viewer_perf.py --compare <label> <label>  the two sets frame by frame: the mean and the 99.9th percentile of the
                                              pixels' difference (of 255), and the share of pixels more than 8 apart

The scenes' recordings are cached (<out>/shots/<scene>.fsrec) and replayed for every label. The hit cache requires
native SDK provenance matching its input and the current tool build; an old mechanical fixture is regenerated.
The hit captures include native release, weapon flight and approach before its burst, followed by the target's
aftermath. Their actual clocks and camera slots are recorded in hit.frames.json beside the frames.
What moves from run to run in one viewer - the clouds' and the vapour's temporal noise, the fire
solver's own timing - is the floor: take two sets from the same build to see it.

A scene passes when, after its first seconds (the viewer loading its tiles and models: --warmup), every one-second
window's mean rate is at least 99 % of the target (158.4 fps for 160) and no frame took longer than twice its interval. Exit code
1 when a scene fails. The machine must be idle: anything else on its CPU or GPU is in these numbers.
"""
import argparse
import json
import math
import os
import re
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
HERE = os.path.dirname(os.path.abspath(__file__))
WIDTH, HEIGHT = 3840, 2160
SUN = ["--sun-utc", "20"]  # midday over the scenes (37.7 N, 123.2 W; 36.3 N, 115 W), whatever the clock says

# scene: (world's name, how it is made, viewer arguments, seconds of statistics)
SCENES = {
    "fleet": ("vi-fleet", "live", ["--follow", "9", "--camera", "chase", "--chase-distance", "900"], 40.0),
    "strike": ("vi-strike", "live", ["--follow", "0", "--camera", "chase", "--chase-distance", "160", "--chase-elevation", "10",
                                     "--chase-azimuth", "70"], 26.0),
    "hit": ("", "replay", ["--camera", "chase", "--chase-distance", "70", "--chase-elevation", "35", "--chase-azimuth", "0"], 34.0),
    "reheat": ("vi-reheat", "live", ["--follow", "0", "--camera", "chase", "--chase-distance", "35", "--chase-elevation", "8",
                                     "--chase-azimuth", "50"], 26.0),
}


# the frames --shots saves of each scene: the replay's times (s), and the cameras (more than one: the same times each)
SHOTS = {
    "fleet": ([6.0, 9.0, 12.0], [["--follow", "9", "--camera", "chase", "--chase-distance", "900"],
                                 ["--follow", "12", "--camera", "chase", "--chase-distance", "45", "--chase-elevation", "12", "--chase-azimuth", "140"]]),
    "strike": ([9.15, 9.4, 9.8, 10.6, 12.0, 15.0, 20.0], [SCENES["strike"][2]]),
    "reheat": ([16.0, 19.0, 22.0], [SCENES["reheat"][2]]),
}
RECORD_S = {"fleet": 14.0, "strike": 22.0, "reheat": 24.0}


def native_metadata(rec, scene_path):
    """Read native provenance for this input, refusing legacy or unrelated recordings."""
    try:
        with open(scene_path, "rb") as f:
            raw = f.read()
        scene = json.loads(raw)
        digest = 0xcbf29ce484222325
        for byte in raw:
            digest = ((digest ^ byte) * 0x100000001b3) & 0xffffffffffffffff
        with open(rec + ".engagement.json", encoding="utf-8") as f:
            data = json.load(f)
        if (scene.get("schema") != "native-engagement-v1" or data.get("schema") != "native-engagement-v1" or
                data.get("source") != "native-sdk" or data.get("scene_hash") != "%016x" % digest or
                not os.path.isfile(rec) or os.path.getsize(rec) == 0 or not data.get("weapons")):
            raise ValueError("missing or mismatched native provenance")
        return data
    except (OSError, ValueError, TypeError, AttributeError) as error:
        raise RuntimeError("no verified native SDK recording: %s (%s)" % (rec, error)) from error


def hit_weapon(data):
    """Select a weapon whose native guidance, fuze and damage events establish an actual hit."""
    target = data.get("target_id")
    if not isinstance(target, int) or target <= 0 or not isinstance(data.get("target_slot"), int) or data["target_slot"] < 0:
        raise RuntimeError("native hit recording has no target identity/slot")
    for weapon in data.get("weapons", []):
        end, launch = weapon.get("end_time_s"), weapon.get("launch_time_s")
        if (weapon.get("terminal_kind") != "detonated" or weapon.get("holding_id") != target or not weapon.get("acquired") or
                not isinstance(end, (int, float)) or not math.isfinite(end) or not isinstance(launch, (int, float)) or
                not math.isfinite(launch) or end <= launch):
            continue
        events = [e for e in data.get("engagements", []) if e.get("entity") == weapon.get("id") and e.get("target") == target]
        if not any(e.get("kind") == "detonation" and e.get("time_s") == end for e in events):
            continue
        if not any(e.get("kind") == "damage" for e in events):
            continue
        duration = data.get("duration_s")
        if not isinstance(duration, (int, float)) or not math.isfinite(duration) or duration <= end:
            raise RuntimeError("native hit recording ends before its visual aftermath")
        times = [end + dt for dt in (0.05, 0.3, 0.8, 1.5, 3.0, 6.0, 10.0) if end + dt < duration]
        if not times:
            raise RuntimeError("native hit recording contains no aftermath frame")
        return weapon
    raise RuntimeError("native SDK encounter did not produce a guided hit; no substitute burst is permitted")


def hit_reference(data):
    """Follow the actual opponent and capture relative to an authoritative native hit."""
    weapon = hit_weapon(data)
    end = weapon["end_time_s"]
    times = [end + dt for dt in (0.05, 0.3, 0.8, 1.5, 3.0, 6.0, 10.0) if end + dt < data["duration_s"]]
    return times, [["--follow", str(data["target_slot"])] + SCENES["hit"][2]]


def hit_capture_plan(data):
    """Show the released native weapon before its burst, then show SDK damage on the target."""
    weapon = hit_weapon(data)
    launch, end = weapon["launch_time_s"], weapon["end_time_s"]
    carrier_slot, weapon_slot = data.get("carrier_slot"), weapon.get("weapon_slot")
    if not isinstance(carrier_slot, int) or carrier_slot < 0 or not isinstance(weapon_slot, int) or weapon_slot < 0:
        raise RuntimeError("native hit recording has no carrier/weapon camera slots")
    if end - 0.25 <= launch + 1.0:
        raise RuntimeError("native hit weapon interval is too short for the requested flight captures")
    carrier = ["--follow", str(carrier_slot), "--camera", "chase", "--chase-distance", "35", "--chase-elevation", "12", "--chase-azimuth", "140"]
    missile = ["--follow", str(weapon_slot), "--camera", "chase", "--chase-distance", "30", "--chase-elevation", "0", "--chase-azimuth", "270"]
    plan = [{"phase": "release", "time_s": launch + 0.1, "camera": carrier},
            {"phase": "flight", "time_s": launch + 1.0, "camera": missile},
            {"phase": "approach", "time_s": end - 0.25, "camera": missile}]
    times, cameras = hit_reference(data)
    plan.extend({"phase": "aftermath", "time_s": t, "camera": cameras[0]} for t in times)
    return plan


def make_native_recording(scene_path, rec, bin_dir, reuse=False):
    """Regenerate stale recordings and validate native outcome before a viewer can consume them."""
    exe = os.path.join(bin_dir, "flightsim-showcase.exe")
    if reuse:
        try:
            data = native_metadata(rec, scene_path)
            inputs = [scene_path, exe, os.path.join(bin_dir, "libfsim.dll")]
            if all(os.path.isfile(path) and os.path.getmtime(path) <= os.path.getmtime(rec) for path in inputs):
                hit_reference(data)
                return data
        except RuntimeError:
            pass
    try:
        if os.path.isfile(rec + ".engagement.json"):
            os.remove(rec + ".engagement.json")  # Invalidate prior success before an overwrite attempt.
        made = subprocess.run([exe, scene_path, rec], cwd=bin_dir, capture_output=True, text=True)
    except OSError as error:
        raise RuntimeError("native SDK showcase cannot run: %s" % error) from error
    with open(rec + ".scene.out", "w", encoding="utf-8") as f:
        f.write(made.stdout + made.stderr)
    if made.returncode != 0:
        raise RuntimeError("native SDK showcase failed: " + made.stdout[-1000:] + made.stderr[-1000:])
    data = native_metadata(rec, scene_path)
    hit_reference(data)
    return data


# --- the scenes' worlds (each run as a process of its own: `viewer_perf.py --publish <scene> <ready file> <seconds>
# [recording]`; with a recording it is not published and not paced, only written)
PACED = True


def paced(w, start, t, steps):
    w.step(steps)
    t += steps * w.step_seconds
    ahead = start + t - time.perf_counter()
    if PACED and ahead > 0.0:
        time.sleep(ahead)
    return t


def publish(scene, ready, seconds, record=None):
    import glob

    import fsim
    global PACED
    PACED = record is None
    name = SCENES[scene][0]
    lat, lon, west = 37.70, -123.20, 1.5 * math.pi
    if record:
        w = fsim.World(name + "-rec", publish=False, workers=4 if scene == "fleet" else 2, pin_workers=False, seed=7, terrain=False,
                       record_path=record, record_interval_s=1.0 / 60.0)
    else:
        w = fsim.World(name, publish=True, workers=4 if scene == "fleet" else 2, pin_workers=False, seed=7, terrain=False)
        assert w.published
    if scene == "fleet":
        kinds = sorted(os.path.basename(os.path.dirname(p)) for p in glob.glob(os.path.join(ROOT, "aircraft", "*", "*.xml"))
                       if os.path.basename(p)[:-4] == os.path.basename(os.path.dirname(p)))
        rotors = {"uh60", "uh1h", "iris", "cf2"}
        wings = [k for k in kinds if k not in rotors and os.path.isfile(os.path.join(ROOT, "aircraft", k, k + ".toml"))]
        fleet = []
        for i, kind in enumerate(wings + sorted(rotors & set(kinds))):
            rotor = kind in rotors
            v = w.create_vehicle(kind, "jsbsim:" + kind, latitude_deg=lat + (i - 17) * 70.0 / 111000.0, longitude_deg=lon + (0.01 if rotor else 0.0),
                                 altitude_msl_m=300.0 if rotor else 3000.0, airspeed_ms=0.0 if rotor else 130.0, heading_deg=270.0)
            w.step()
            pj = os.path.join(ROOT, "aircraft", kind, "payloads.json")
            for lo in (json.load(open(pj, encoding="utf-8")).get("loadouts", []) if os.path.isfile(pj) else []):
                try:
                    v.load_loadout(lo["name"])
                except Exception:  # noqa: BLE001 (a loadout it cannot carry as it stands: the next)
                    pass
            w.step(2)
            if rotor:
                a = v.submit_hsa(heading_rad=west, altitude_m=300.0, speed=0.0, speed_reference="ground_speed")
            else:
                a = v.submit_hsa(heading_rad=west, altitude_m=3000.0, speed=130.0, speed_reference="true_airspeed")
            fleet.append((rotor, a))
        steps = max(1, int(round(0.05 / w.step_seconds)))
        open(ready, "w").write("ready")
        start, t, turned, heading = time.perf_counter(), 0.0, 0.0, west
        while t < seconds:
            t = paced(w, start, t, steps)
            if t - turned > 20.0:  # the wings turn together, 30 deg at a time, and back
                turned, heading = t, west + (math.radians(30.0) if heading == west else 0.0)
                for rotor, a in fleet:
                    if not rotor:
                        a.update(heading_rad=heading)
    elif scene == "strike":
        angle, speed, lead = 60.0, 233.0, 9.0
        w.create_vehicle("viper", "jsbsim:f16c", latitude_deg=lat, longitude_deg=lon, altitude_msl_m=20.0 + lead * speed * math.sin(math.radians(angle)),
                         heading_deg=270.0, pitch_deg=-angle, airspeed_ms=speed)
        steps = max(1, int(round(0.02 / w.step_seconds)))
        open(ready, "w").write("ready")
        start, t = time.perf_counter(), 0.0
        while t < seconds:
            t = paced(w, start, t, steps)
    elif scene == "reheat":
        v = w.create_vehicle("viper", "jsbsim:f16c", latitude_deg=lat, longitude_deg=lon, altitude_msl_m=4000.0, heading_deg=270.0, airspeed_ms=260.0)
        w.step()
        v.submit_hsa(heading_rad=west, altitude_m=4000.0, speed=520.0, speed_reference="true_airspeed")  # (more than it can: full reheat)
        steps = max(1, int(round(0.02 / w.step_seconds)))
        open(ready, "w").write("ready")
        start, t = time.perf_counter(), 0.0
        while t < seconds:
            t = paced(w, start, t, steps)


# --- the measurement
STATS = re.compile(r"stats t=\s*([\d.]+)\s+([\d.]+) fps\s+frame avg\s+([\d.]+) max\s+([\d.]+) ms \| app ([\d.]+) advance\s+[\d.]+ events [\d.]+ "
                   r"update ([\d.]+) record ([\d.]+) present\s+[\d.]+ sleep\s+[\d.]+ ms \| cpu\s+([\d.]+)%")


def read(path, warmup):
    """The viewer's statistics after `warmup` s: its windows (t, fps, mean ms, worst ms, app ms, record ms), the
    frames' percentiles, the GPU's passes and Flow's cost, each averaged over the windows that gave them."""
    windows, passes, flow, frames, keep = [], {}, {}, {"p99": [], "over": 0, "n": 0}, False
    for line in open(path, encoding="utf-8", errors="replace"):
        m = STATS.match(line)
        if m:
            keep = float(m.group(1)) >= warmup
            if keep:
                windows.append(dict(t=float(m.group(1)), fps=float(m.group(2)), avg=float(m.group(3)), worst=float(m.group(4)),
                                    app=float(m.group(5)), record=float(m.group(7)), cpu=float(m.group(8))))
        elif keep and line.startswith("frames |"):
            m = re.search(r"p99 ([\d.]+) ms \| (\d+) of (\d+) over", line)
            frames["p99"].append(float(m.group(1)))
            frames["over"] += int(m.group(2))
            frames["n"] += int(m.group(3))
        elif keep and line.startswith("gpu |"):
            for label, ms in re.findall(r"\| ([^|]+?) ([\d.]+)(?= \||\s*ms)", line):
                passes.setdefault(label.strip(), []).append(float(ms))
        elif keep and line.startswith("flow |"):
            m = re.search(r"solver gpu avg ([\d.]+) max ([\d.]+) ms \| frame thread avg ([\d.]+) max ([\d.]+) ms \| solver thread avg ([\d.]+) "
                          r"max ([\d.]+) ms \(packing ([\d.]+)\), (\d+) waiting \| leaves (\d+)", line)
            if m and int(m.group(9)) > 0:
                for k, v in zip(("gpu", "gpu max", "frame thread", "frame thread max", "solver thread", "solver thread max", "packing", "waiting max",
                                 "leaves max"), m.groups()):
                    flow.setdefault(k, []).append(float(v))
            elif not m:  # (a viewer from before the solver had a thread of its own)
                m = re.search(r"solver gpu avg ([\d.]+) max ([\d.]+) ms \| cpu avg ([\d.]+) max ([\d.]+) ms \(packing ([\d.]+)\) \| leaves (\d+)", line)
                if m and int(m.group(6)) > 0:
                    for k, v in zip(("gpu", "gpu max", "frame thread", "frame thread max", "packing", "leaves max"), m.groups()):
                        flow.setdefault(k, []).append(float(v))
    return windows, frames, passes, flow


def run(scene, args, out):
    world, kind, view, seconds = SCENES[scene]
    bin_dir = os.path.join(args.build, "bin")
    env = dict(os.environ, PYTHONPATH=os.path.join(args.build, "python"), FSIM_BIN=bin_dir)
    stem = os.path.join(out, "%s_%s" % (args.label, scene))
    publisher = None
    if kind == "live":
        ready = stem + ".ready"
        if os.path.exists(ready):
            os.remove(ready)
        publisher = subprocess.Popen([sys.executable, os.path.abspath(__file__), "--publish", scene, ready, str(seconds + 20.0)], env=env,
                                     stdout=open(stem + ".scene.out", "w"), stderr=subprocess.STDOUT, creationflags=0x08000000)
        waited = 0.0
        while not os.path.exists(ready) and publisher.poll() is None and waited < 120.0:
            time.sleep(0.1)
            waited += 0.1
        if not os.path.exists(ready):
            publisher.kill()
            raise RuntimeError("the %s scene did not start: %s" % (scene, open(stem + ".scene.out").read()[-2000:]))
        source = ["--world", world]
    else:
        rec = stem + ".fsrec"
        data = make_native_recording(os.path.join(HERE, scene + ".json"), rec, bin_dir)
        _, cameras = hit_reference(data)
        view = cameras[0]
        source = ["--replay", rec]
    command = [os.path.join(bin_dir, args.exe)] + source + view + [
        "--offline", "--width", str(WIDTH), "--height", str(HEIGHT), "--fullscreen", "--stats", str(seconds), *SUN,
        "--stats-budget", "%.4f" % (2000.0 / args.target)] + args.viewer
    try:
        with open(stem + ".out", "w") as o, open(stem + ".err", "w") as e:
            measured = subprocess.run(command, cwd=bin_dir, stdout=o, stderr=e, timeout=seconds + 120.0)
        if measured.returncode != 0:
            raise RuntimeError("viewer process failed (%d): see %s.err" % (measured.returncode, stem))
    finally:
        if publisher is not None:
            publisher.kill()
            publisher.wait()
    log = open(stem + ".err", encoding="utf-8", errors="replace").read() + open(stem + ".out", encoding="utf-8", errors="replace").read()
    if "fire and smoke solved" not in log:
        raise RuntimeError("the viewer ran without NVIDIA Flow: see " + stem + ".err")
    for sign in ("did not build", "failed to parse", "none drawn"):  # (a pass that is not drawn costs nothing: no measurement)
        if sign in log:
            raise RuntimeError("the viewer did not draw everything (\"%s\"): see %s.err" % (sign, stem))
    if re.search(r"^stats t=\s*%d\.\d .*\| vehicles 0$" % int(args.warmup + 1), log, re.M):
        raise RuntimeError("the viewer drew no vehicle of the %s scene (it did not find the world in time): see %s.err" % (scene, stem))
    return read(stem + ".out", args.warmup)


def recording(scene, args):
    """The scene's recording for --shots, made the first time it is asked for."""
    folder = os.path.join(args.out, "shots")
    os.makedirs(folder, exist_ok=True)
    rec = os.path.join(folder, scene + ".fsrec")
    if os.path.isfile(rec) and SCENES[scene][1] != "replay":
        return rec
    bin_dir = os.path.join(args.build, "bin")
    if SCENES[scene][1] == "replay":
        make_native_recording(os.path.join(HERE, scene + ".json"), rec, bin_dir, reuse=True)
        return rec
    else:
        env = dict(os.environ, PYTHONPATH=os.path.join(args.build, "python"), FSIM_BIN=bin_dir)
        made = subprocess.run([sys.executable, os.path.abspath(__file__), "--publish", scene, rec + ".ready", str(RECORD_S[scene]), rec], env=env,
                              capture_output=True, text=True)
    if made.returncode != 0 or not os.path.isfile(rec):
        raise RuntimeError("no recording of %s: %s %s" % (scene, made.stdout[-1500:], made.stderr[-1500:]))
    return rec


def shots(args):
    bin_dir = os.path.join(args.build, "bin")
    folder = os.path.join(args.out, "shots", args.shots)
    os.makedirs(folder, exist_ok=True)
    for scene in args.scene:
        rec = recording(scene, args)
        if scene == "hit":
            data = native_metadata(rec, os.path.join(HERE, scene + ".json"))
            plans = [(scene, hit_capture_plan(data))]
            manifest_path = os.path.join(folder, "hit.frames.json")
            if os.path.isfile(manifest_path):
                os.remove(manifest_path)
        else:
            times, cameras = SHOTS[scene]
            plans = [(scene if c == 0 else "%s%d" % (scene, c + 1),
                      [{"phase": scene, "time_s": t, "camera": camera} for t in times]) for c, camera in enumerate(cameras)]
        frames = []
        for name, plan in plans:
            stem = os.path.join(folder, name)
            for k, frame in enumerate(plan):  # (a viewer a frame: the reference clocks are not evenly apart)
                t, camera = frame["time_s"], frame["camera"]
                png = "%s_%03d.png" % (stem, k)
                command = [os.path.join(bin_dir, args.exe), "--replay", rec] + camera + [
                    "--offline", "--no-gui", "--width", str(WIDTH), "--height", str(HEIGHT), "--fullscreen", *SUN, "--screenshot", png,
                    "--screenshot-clock", "replay", "--screenshot-after", "%.3f" % t] + args.viewer
                if os.path.isfile(png):
                    os.remove(png)
                with open(stem + ".log", "a") as log:
                    captured = subprocess.run(command, cwd=bin_dir, stdout=log, stderr=subprocess.STDOUT, timeout=180.0)
                if captured.returncode != 0 or not os.path.isfile(png):
                    raise RuntimeError("viewer screenshot failed (%d): see %s.log" % (captured.returncode, stem))
                frames.append({"file": png, **frame})
                print("saved %s" % png, flush=True)
        if scene == "hit":
            with open(manifest_path, "w", encoding="utf-8") as manifest:
                json.dump({"schema": data["schema"], "source": data["source"], "scene_hash": data["scene_hash"],
                           "recording": rec, "frames": frames}, manifest, indent=2)
    return 0


def compare(a, b, args):
    import glob

    import numpy
    from PIL import Image
    fa, fb = os.path.join(args.out, "shots", a), os.path.join(args.out, "shots", b)
    print("%-16s %9s %9s %9s %10s" % ("frame", "mean", "p99.9", "max", "> 8 of 255"))
    worst = 0.0
    for pa in sorted(glob.glob(os.path.join(fa, "*.png"))):
        pb = os.path.join(fb, os.path.basename(pa))
        if not os.path.isfile(pb):
            print("%-16s missing in %s" % (os.path.basename(pa), b))
            continue
        x = numpy.asarray(Image.open(pa).convert("RGB"), dtype=numpy.int16)
        y = numpy.asarray(Image.open(pb).convert("RGB"), dtype=numpy.int16)
        if x.shape != y.shape:
            print("%-16s sizes differ" % os.path.basename(pa))
            continue
        d = numpy.abs(x - y).max(axis=2)
        share = float((d > 8).mean())
        worst = max(worst, share)
        print("%-16s %9.3f %9.1f %9d %9.3f%%" % (os.path.basename(pa)[:-4], float(d.mean()), float(numpy.percentile(d, 99.9)), int(d.max()), 100.0 * share))
        if args.diff:
            Image.fromarray(numpy.clip(d * 8, 0, 255).astype(numpy.uint8)).save(os.path.join(fb, os.path.basename(pa)[:-4] + ".diff_" + a + ".jpg"), quality=85)
    print("the most pixels more than 8 apart in a frame: %.3f%%" % (100.0 * worst))
    return 0


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "--publish":
        publish(sys.argv[2], sys.argv[3], float(sys.argv[4]), sys.argv[5] if len(sys.argv) > 5 else None)
        return 0
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--build", default=os.path.join(ROOT, "build", "ucrt64-release"))
    ap.add_argument("--scene", nargs="+", default=list(SCENES), choices=list(SCENES))
    ap.add_argument("--target", type=float, default=160.0, help="frames a second (the display's refresh)")
    ap.add_argument("--warmup", type=float, default=5.0, help="seconds of each run not held to the target")
    ap.add_argument("--out", default=os.path.join(ROOT, "build", "viewer-perf"))
    ap.add_argument("--label", default="run")
    ap.add_argument("--exe", default="flightsim-viewer.exe", help="the viewer in the build's bin (another: one kept from before a change)")
    ap.add_argument("--shots", metavar="LABEL", help="save each scene's reference frames under this label instead of measuring")
    ap.add_argument("--compare", nargs=2, metavar="LABEL", help="compare two sets of reference frames")
    ap.add_argument("--diff", action="store_true", help="with --compare: save each frame's difference (x 8) beside the second set")
    ap.add_argument("viewer", nargs="*", help="more arguments for the viewer (after --)")
    args = ap.parse_args()
    args.build = os.path.abspath(args.build)
    os.makedirs(args.out, exist_ok=True)
    if args.compare:
        return compare(args.compare[0], args.compare[1], args)
    if args.shots:
        return shots(args)
    failed, report = False, {}
    print("%-8s %5s %7s %7s %7s %8s %8s %6s %7s %7s %6s  %s" % ("scene", "secs", "fps", "lowest", "p99 ms", "worst ms", "over 2x", "app", "record", "gpu ms", "cpu%", ""))
    for scene in args.scene:
        windows, frames, passes, flow = run(scene, args, args.out)
        if not windows:
            print("%-8s no statistics" % scene)
            failed = True
            continue
        mean = lambda v: sum(v) / len(v) if v else float("nan")  # noqa: E731
        fps, lowest, worst = mean([w["fps"] for w in windows]), min(w["fps"] for w in windows), max(w["worst"] for w in windows)
        gpu = {k: mean(v) for k, v in passes.items()}
        ok = lowest >= 0.99 * args.target and frames["over"] == 0
        failed |= not ok
        print("%-8s %5d %7.1f %7.1f %7.2f %8.2f %8d %6.2f %7.2f %7.2f %6.0f  %s" % (
            scene, len(windows), fps, lowest, max(frames["p99"]) if frames["p99"] else float("nan"), worst, frames["over"],
            mean([w["app"] for w in windows]), mean([w["record"] for w in windows]), gpu.get("total", float("nan")),
            mean([w["cpu"] for w in windows]), "ok" if ok else "BELOW %g" % args.target))
        print("         gpu: " + ", ".join("%s %.2f" % (k, v) for k, v in gpu.items() if k != "total"))
        if flow:
            print("         flow: " + ", ".join("%s %.2f" % (k, (max if "max" in k else mean)(v)) for k, v in flow.items()) + " ms")
        report[scene] = dict(fps=fps, lowest=lowest, worst_ms=worst, over=frames["over"], frames=frames["n"], gpu=gpu,
                             flow={k: (max if "max" in k else mean)(v) for k, v in flow.items()}, windows=windows, ok=ok)
    json.dump(dict(target=args.target, width=WIDTH, height=HEIGHT, scenes=report), open(os.path.join(args.out, args.label + ".json"), "w"), indent=1)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
