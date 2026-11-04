"""Black-box regression for the unrecorded, unbounded native engagement mode."""
import argparse
import os
from pathlib import Path
import subprocess
import tempfile


def verify(build, scene, output):
    build, scene, output = build.resolve(), scene.resolve(), output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    environment = os.environ.copy()
    environment["PATH"] = str(build / "bin") + os.pathsep + environment.get("PATH", "")
    log_path = output / "interactive.log"
    with tempfile.TemporaryDirectory(prefix="engagement-interactive-", dir=environment.get("TEMP")) as temporary:
        directory = Path(temporary)
        sentinel = directory / "--interactive"
        sentinel.write_bytes(b"not a recording output")
        with log_path.open("w", encoding="utf-8") as log:
            native = subprocess.Popen(
                [str(build / "bin" / "flightsim-showcase.exe"), str(scene), "--interactive"],
                cwd=directory, env=environment, stdin=subprocess.DEVNULL,
                stdout=log, stderr=subprocess.STDOUT,
            )
            try:
                try:
                    code = native.wait(timeout=43)
                except subprocess.TimeoutExpired:
                    pass
                else:
                    raise AssertionError(f"interactive mode exited with {code} instead of continuing past 40 s; see {log_path}")
            finally:
                if native.poll() is None:
                    native.terminate()
                    try:
                        native.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        native.kill()
                        native.wait(timeout=5)
        assert sentinel.read_bytes() == b"not a recording output", "interactive flag was treated as an output file"
        assert set(directory.iterdir()) == {sentinel}, "interactive mode created output artifacts"
    text = log_path.read_text(encoding="utf-8", errors="replace")
    assert "interactive native engagement: recording=off" in text, "native recording was not disabled"
    assert "live focus missile:" in text, "native release did not occur"
    assert "live focus target:" in text, "matching native fragment Hit did not occur"
    print("PASS: native release and matching Hit, alive beyond 40 s, no recording or capture outputs", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", type=Path, required=True)
    parser.add_argument("--scene", type=Path, default=Path(__file__).parents[1] / "validation/viewer_perf/engagement.json")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    verify(args.build, args.scene, args.out)
