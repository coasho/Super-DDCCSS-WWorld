"""Run one VSPAERO case (reference_body, released N8/80 inputs) with a hash-bound executable into a fresh folder.

Inputs are copied byte-for-byte from native/numerics/r2-<case>/ and checked against the released hashes (geometry
065d9321..., vkey 5b874cac..., config alpha0 36ad325e... / alpha2 958d89e4...). The command is the released one,
<exe> -omp 2 reference_body, with TMP/TEMP/TORCH_EXTENSIONS_DIR = D:/ENV/tmp, OMP_NUM_THREADS = 2 and the pinned
UCRT64 runtime directory first on PATH. Raw stdout/stderr go straight to run.log. Only the output-only switches
REFERENCE_BODY_WITNESS_* and SIMM02_WAKE_RESIDUAL_PROBE may be added. A timeout (120 s), a nonzero exit, a missing
output or a changed input or executable makes the run fail explicitly (native-run.json records why).

Usage (from the project root):
    fsim.cmd --preset ucrt64-headless python tools/validation/missile/run_numerics_vspaero.py --exe <exe> --exe-sha256 <hex>
        --case alpha2 --run-dir <fresh dir> [--env KEY=VALUE ...]
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

FIXTURES = Path(__file__).with_name("native") / "numerics"
UCRT64 = "D:/ENV/DevLanguages/Cpp/msys2/ucrt64/bin"
INPUTS = {"reference_body.vspgeom": "065d932167a3e0cfdd28443fbf94f7f698cb51ecec71135f08036f0f81f46ef9",
          "reference_body.vkey": "5b874cac4ff8701edc29f54a6eb234558fee5d49e6345ed5eaaf4060d85176da"}
CONFIGS = {"alpha0": "36ad325e6a41f6460c69d169e5632e3ebabd09d92a054c08901ae4c98d634dea",
           "alpha2": "958d89e4349757eafa6fcf77b197f0c7aa4f5d9f547449ada3d8bb996dbea5c0"}
OUTPUTS = ("reference_body.history", "reference_body.polar", "reference_body.adb", "reference_body.adb.cases")
ALLOWED_ENV = ("REFERENCE_BODY_WITNESS_LOOP", "REFERENCE_BODY_WITNESS_FILE", "REFERENCE_BODY_WITNESS_ITERATIONS",
               "REFERENCE_BODY_WITNESS_CAP", "SIMM02_WAKE_RESIDUAL_PROBE")
TIMEOUT_S = 120


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def kill_tree(process):
    taskkill = Path(os.environ.get("SystemRoot", r"C:\Windows")) / "System32" / "taskkill.exe"
    subprocess.run([str(taskkill), "/T", "/F", "/PID", str(process.pid)], stdin=subprocess.DEVNULL,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=60, check=False)
    if process.poll() is None:
        process.kill()
    return process.wait()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--exe", required=True)
    parser.add_argument("--exe-sha256", required=True)
    parser.add_argument("--case", required=True, choices=sorted(CONFIGS))
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--env", action="append", default=[])
    parser.add_argument("--diagnostic-config", default=None,
                        help="DIAGNOSTIC CONTROL ONLY: a config that differs from the released one by declared lines; the "
                             "run is labelled diagnostic and can never serve as acceptance evidence")
    parser.add_argument("--diagnostic-change", action="append", default=[],
                        help="KEY = VALUE line that the diagnostic config must carry instead of the released line")
    args = parser.parse_args(argv)
    exe, run_dir = Path(args.exe).resolve(), Path(args.run_dir).resolve()
    extra = {}
    for item in args.env:
        key, _, value = item.partition("=")
        if key not in ALLOWED_ENV or not value:
            raise SystemExit("only output-only switches %s may be set (got %r)" % (ALLOWED_ENV, item))
        extra[key] = value
    if sha256(exe) != args.exe_sha256:
        raise SystemExit("executable identity mismatch: %s" % exe)
    run_dir.mkdir(parents=True, exist_ok=False)
    case = FIXTURES / ("r2-" + args.case)
    expected = dict(INPUTS, **{"reference_body.vspaero": CONFIGS[args.case]})
    for name, digest in expected.items():
        if sha256(case / name) != digest:
            raise SystemExit("released input identity mismatch: %s" % (case / name))
        shutil.copyfile(case / name, run_dir / name)
        if sha256(run_dir / name) != digest:
            raise SystemExit("staged input differs: %s" % name)
    diagnostic = None
    if args.diagnostic_config:
        released = (case / "reference_body.vspaero").read_text(encoding="ascii").splitlines()
        changed = Path(args.diagnostic_config).read_text(encoding="ascii").splitlines()
        wanted = {line.split("=")[0].strip(): line.strip() for line in args.diagnostic_change}
        differences = [(a, b) for a, b in zip(released, changed) if a != b]
        if len(released) != len(changed) or not wanted or len(differences) != len(wanted) or any(
                b.split("=")[0].strip() not in wanted or b.strip() != wanted[b.split("=")[0].strip()] for _, b in differences):
            raise SystemExit("the diagnostic config must differ from the released config exactly by the declared lines")
        shutil.copyfile(args.diagnostic_config, run_dir / "reference_body.vspaero")
        expected["reference_body.vspaero"] = sha256(run_dir / "reference_body.vspaero")
        diagnostic = {"role": "DIAGNOSTIC CONTROL ONLY - not acceptance evidence, not a fix",
                      "released_config_sha256": CONFIGS[args.case], "changed_lines": differences,
                      "config_sha256": expected["reference_body.vspaero"]}
    env = dict(os.environ, TMP="D:/ENV/tmp", TEMP="D:/ENV/tmp", TORCH_EXTENSIONS_DIR="D:/ENV/tmp", OMP_NUM_THREADS="2",
               PATH=str(Path(UCRT64)) + os.pathsep + os.environ.get("PATH", ""))  # native separators for DLL search
    for key in ALLOWED_ENV:
        env.pop(key, None)
    env.update(extra)
    command = [str(exe), "-omp", "2", "reference_body"]
    record = {"command": command, "cwd": str(run_dir), "case": args.case, "executable_sha256": args.exe_sha256,
              "env": {key: env.get(key) for key in ("TMP", "TEMP", "TORCH_EXTENSIONS_DIR", "OMP_NUM_THREADS") + ALLOWED_ENV},
              "path_prefix": UCRT64, "timeout_s": TIMEOUT_S, "inputs": expected, "diagnostic": diagnostic,
              "started_utc": datetime.now(timezone.utc).isoformat()}
    started = time.monotonic()
    with open(run_dir / "run.log", "xb") as log:
        process = subprocess.Popen(command, cwd=str(run_dir), stdout=log, stderr=subprocess.STDOUT,
                                   stdin=subprocess.DEVNULL, env=env)
        try:
            exit_code, timed_out = process.wait(timeout=TIMEOUT_S), False
        except subprocess.TimeoutExpired:
            exit_code, timed_out = kill_tree(process), True
    record.update(exit_code=exit_code, timed_out=timed_out, seconds=time.monotonic() - started,
                  finished_utc=datetime.now(timezone.utc).isoformat(), log_sha256=sha256(run_dir / "run.log"))
    problems = []
    if timed_out:
        problems.append("timeout")
    if exit_code != 0:
        problems.append("exit code %r" % exit_code)
    record["outputs"] = {name: sha256(run_dir / name) for name in OUTPUTS if (run_dir / name).is_file()}
    problems += ["missing output %s" % name for name in OUTPUTS if name not in record["outputs"]]
    after = {name: sha256(run_dir / name) for name in expected}
    if after != expected:
        problems.append("an input changed during the run")
    if sha256(exe) != args.exe_sha256:
        problems.append("the executable changed during the run")
    record["input_hashes_after"] = after
    record["problems"] = problems
    record["status"] = "failed" if problems else "completed"
    with open(run_dir / "native-run.json", "x", encoding="utf-8") as stream:
        json.dump(record, stream, indent=1)
        stream.write("\n")
    print(json.dumps({key: record[key] for key in ("case", "status", "exit_code", "seconds", "problems")}))
    return 0 if not problems else 1


if __name__ == "__main__":
    sys.exit(main())
