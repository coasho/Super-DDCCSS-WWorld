"""Run one complete fullscreen native strike with exactly two event-relative captures.

The preflight records the unchanged SDK scene to establish precise event anchors.
The live viewer still requires its own matching native detonation and fragment Hit.
Both processes continue to the scene's full duration after the two screenshots.
"""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import time

from PIL import Image

ROOT = Path(__file__).resolve().parents[3]


def read_evidence(recording):
    return json.loads(Path(str(recording) + ".engagement.json").read_text(encoding="utf-8"))


def read_target_outcome(log_path):
    lines = log_path.read_text(encoding="utf-8", errors="replace").splitlines()
    reports = [json.loads(line.removeprefix("target outcome: ")) for line in lines if line.startswith("target outcome: ")]
    if len(reports) != 1 or reports[0].get("schema") != "strike-target-outcome-v1":
        raise RuntimeError("native readback omitted the target's post-hit flight outcome: " + str(log_path))
    return reports[0]


def check_capacity_rejection(viewer, evidence_path, scene, output, environment):
    """Reject an unavailable camera slot before creating a fullscreen window."""
    evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
    capacity = max(evidence["carrier_slot"], evidence["target_slot"], evidence["weapons"][0]["weapon_slot"])
    log_path = output / "capacity-rejection.log"
    with log_path.open("w", encoding="utf-8") as log:
        result = subprocess.run([str(viewer), "--world", scene["name"], "--fullscreen", "--offline",
                                 "--capacity", str(capacity), "--stats", "0.1", "--strike-capture",
                                 str(evidence_path), "--screenshot", str(output / "invalid-capacity.png")],
                                env=environment, stdout=log, stderr=subprocess.STDOUT, timeout=30)
    text = log_path.read_text(encoding="utf-8", errors="replace")
    if result.returncode != 2 or "strike subjects exceed viewer capacity" not in text or "capture ready" in text:
        raise RuntimeError("insufficient strike capacity was not rejected before viewer creation; see " + str(log_path))


def run(args):
    build = args.build.resolve()
    output = args.out.resolve()
    output.mkdir(parents=True, exist_ok=True)
    (output / "result.json").unlink(missing_ok=True)
    scene = json.loads(args.scene.read_text(encoding="utf-8"))
    scene["name"] = "native strike acceptance"
    scene_path = output / "scene.json"
    scene_path.write_text(json.dumps(scene, indent=2) + "\n", encoding="utf-8")
    native = build / "bin" / "flightsim-showcase.exe"
    viewer = build / "bin" / "flightsim-viewer.exe"
    verifier = build / "tests" / "fsim_strike_capture_test.exe"
    for executable in (native, viewer, verifier):
        if not executable.is_file():
            raise RuntimeError("missing native executable: " + str(executable))
    environment = os.environ.copy()
    environment["PATH"] = str(native.parent) + os.pathsep + environment.get("PATH", "")
    for key in ("TMP", "TEMP", "TORCH_EXTENSIONS_DIR"):
        environment[key] = "D:/ENV/tmp"
    preflight = output / "preflight.fsrec"
    with (output / "preflight.log").open("w", encoding="utf-8") as log:
        subprocess.run([str(native), str(scene_path), str(preflight)], env=environment,
                       stdout=log, stderr=subprocess.STDOUT, timeout=120, check=True)
    evidence = read_evidence(preflight)
    with (output / "preflight-readback.log").open("w", encoding="utf-8") as log:
        subprocess.run([str(verifier), str(preflight) + ".engagement.json", str(preflight)],
                       env=environment, stdout=log, stderr=subprocess.STDOUT, timeout=30, check=True)
    preflight_outcome = read_target_outcome(output / "preflight-readback.log")
    check_capacity_rejection(viewer, Path(str(preflight) + ".engagement.json"), scene, output, environment)
    prefix = output / "strike.png"
    captures = [output / "strike_launch_plus_1s.png", output / "strike_hit_plus_0.3s.png"]
    for capture in captures:
        capture.unlink(missing_ok=True)
    viewer_log = output / "viewer.log"
    command = [str(viewer), "--world", scene["name"], "--fullscreen", "--offline", "--no-gui",
               "--camera", "chase", "--chase-distance", "20", "--chase-azimuth", "240",
               "--chase-elevation", "10", "--sun-utc", "20", "--strike-capture",
               str(preflight) + ".engagement.json", "--screenshot", str(prefix)]
    live = output / "live.fsrec"
    source = None
    started = time.monotonic()
    with viewer_log.open("w", encoding="utf-8") as v_log, (output / "live.log").open("w", encoding="utf-8") as s_log:
        window = subprocess.Popen(command, env=environment, stdout=v_log, stderr=subprocess.STDOUT)
        try:
            # Read the native readiness marker; no screenshot or extra window is used for readiness.
            while "capture ready" not in viewer_log.read_text(encoding="utf-8", errors="replace"):
                if window.poll() is not None:
                    raise RuntimeError("viewer exited before readiness; see " + str(viewer_log))
                if time.monotonic() - started > 60:
                    raise RuntimeError("viewer readiness timed out; see " + str(viewer_log))
                time.sleep(0.05)
            native_started = time.monotonic()
            source = subprocess.Popen([str(native), str(scene_path), str(live), "--live"],
                                      env=environment, stdout=s_log, stderr=subprocess.STDOUT)
            timeout = float(scene["duration_s"]) + 30
            source_code = source.wait(timeout=timeout)
            native_elapsed = time.monotonic() - native_started
            viewer_code = window.wait(timeout=15)
            if source_code != 0 or viewer_code != 0:
                raise RuntimeError("native/viewer failed: %d/%d; inspect live.log and viewer.log" % (source_code, viewer_code))
        finally:
            # Only stop processes created by this harness if validation failed or was interrupted.
            for process in (source, window):
                if process is not None and process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
    actual = read_evidence(live)
    if actual != evidence:
        raise RuntimeError("live SDK result differs from the same-scene preflight")
    with (output / "live-readback.log").open("w", encoding="utf-8") as log:
        subprocess.run([str(verifier), str(live) + ".engagement.json", str(live)],
                       env=environment, stdout=log, stderr=subprocess.STDOUT, timeout=30, check=True)
    target_outcome = read_target_outcome(output / "live-readback.log")
    if target_outcome != preflight_outcome:
        raise RuntimeError("live target flight outcome differs from the same-scene preflight")
    text = viewer_log.read_text(encoding="utf-8", errors="replace")
    camera = [(float(t), int(s), int(e)) for t, s, e in
              re.findall(r"strike camera time=([\d.]+) slot=(\d+) entity=(\d+)", text)]
    shots = [(int(i), float(t), int(s)) for i, t, s in
             re.findall(r"strike capture index=(\d+) time=([\d.]+) slot=(\d+)", text)]
    weapon = actual["weapons"][0]
    crew_losses = [(float(t), int(s)) for t, s in
                   re.findall(r"strike crew lost time=([\d.]+) slot=(\d+)", text)]
    if len(crew_losses) != 1 or abs(crew_losses[0][0] - weapon["end_time_s"]) > 1e-7 or crew_losses[0][1] != actual["target_slot"]:
        raise RuntimeError("live viewer did not observe the expected native target crew loss")
    expected_slots = [actual["carrier_slot"], weapon["weapon_slot"], actual["target_slot"]]
    if [item[1] for item in camera] != expected_slots or len(shots) != 2 or [item[0] for item in shots] != [0, 1]:
        raise RuntimeError("camera order or exact two-capture contract failed")
    step = actual["native_dt_s"] * actual["native_frame_skip"]
    requests = [weapon["launch_time_s"] + 1.0, weapon["end_time_s"] + 0.3]
    for (index, observed, slot), requested, expected in zip(shots, requests, expected_slots[1:]):
        if not -1e-8 <= observed - requested <= step + 1e-8 or slot != expected:
            raise RuntimeError("capture does not match its event-relative native time and camera")
    if "strike complete" not in text or native_elapsed < actual["duration_s"] - 0.1:
        raise RuntimeError("the complete live duration was not observed")
    dimensions = []
    for capture in captures:
        with Image.open(capture) as image:
            image.load()
            dimensions.append(list(image.size))
            if image.size != (3840, 2160):
                raise RuntimeError("fullscreen capture must be 3840 x 2160: " + str(capture))
    result = dict(native_exit=source_code, viewer_exit=viewer_code, native_wall_seconds=native_elapsed,
                  viewer_wall_seconds=time.monotonic() - started, simulation_seconds=actual["duration_s"],
                  native_frames=actual["recorded_frames"], scene_hash=actual["scene_hash"],
                  launch_s=weapon["launch_time_s"], hit_s=weapon["end_time_s"],
                  target_outcome=target_outcome,
                  camera=camera, requested_capture_s=requests, captures=shots, dimensions=dimensions,
                  native_result_matches_preflight=True, viewer_command=command)
    (output / "result.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", type=Path, default=ROOT / "build" / "ucrt64-release")
    parser.add_argument("--scene", type=Path, default=Path(__file__).with_name("hit.json"))
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    run(args)


if __name__ == "__main__":
    main()
