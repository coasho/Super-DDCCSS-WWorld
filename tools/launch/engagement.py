"""Open an unrecorded native engagement until the user closes the Viewer."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]


def run(build, scene):
    build, scene = build.resolve(), scene.resolve()
    definition = json.loads(scene.read_text(encoding="utf-8"))
    viewer = build / "bin/flightsim-viewer.exe"
    native = build / "bin/flightsim-showcase.exe"
    for executable in (viewer, native):
        if not executable.is_file():
            raise RuntimeError(f"Missing {executable}. Build ucrt64-release first.")
    environment = os.environ.copy()
    environment["PATH"] = str(build / "bin") + os.pathsep + environment.get("PATH", "")
    window = source = None
    try:
        window = subprocess.Popen(
            [str(viewer), "--world", definition["name"], "--fullscreen", "--offline", "--camera", "chase"],
            env=environment,
        )
        source = subprocess.Popen([str(native), str(scene), "--interactive"], env=environment)
        while window.poll() is None:
            if source.poll() is not None:
                raise RuntimeError(f"Native engagement exited unexpectedly with code {source.returncode}.")
            time.sleep(0.1)
        return window.returncode
    finally:
        # Closing the Viewer ends only this launcher's native World and children.
        for process in (source, window):
            if process is not None and process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", type=Path, default=ROOT / "build/ucrt64-release")
    parser.add_argument("--scene", type=Path, default=ROOT / "tools/validation/viewer_perf/engagement.json")
    args = parser.parse_args()
    try:
        return run(args.build, args.scene)
    except KeyboardInterrupt:
        return 130
    except (OSError, ValueError, RuntimeError) as error:
        print(f"Engagement failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
