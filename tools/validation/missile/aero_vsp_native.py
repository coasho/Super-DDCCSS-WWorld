"""Explicit, optional public-library OpenVSP 3.55.0 geometry regression.

Only this CLI compiles/runs the native fixture. Importing this module and the
ordinary stdlib tests requires neither a compiler nor an installed solver.
The caller supplies every external build input; the reviewed patch is never
applied automatically. A passing attachment regression is not aero acceptance.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

EXPECTED_SOURCE_SHA256 = "0dd0e712b0136dab67a5425a5c086207b4c26727b3334dd68caa53dade87f36f"
SOURCE_COMMIT = "5240ef210f41c5d6aeed0ba7687f6fab0ca0cafb"
_HERE = Path(__file__).resolve().parent
_NATIVE = _HERE / "native"
_GUARDS = ("surfaces_unchanged", "other_fine_nodes_unchanged", "roots_attached",
           "repeated_update_idempotent", "all_coordinates_finite", "passed",
           "deflect_wake_matches_fine", "undeflect_wake_matches_fine",
           "deflect_fine_nodes_unchanged", "undeflect_fine_nodes_unchanged")
_CASES = (("level0-identity", 0, False), ("level1-identity", 1, False),
          ("level1-unused-nonidentity", 1, True), ("level2-unused-nonidentity", 2, True))


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _file(path, label):
    value = Path(path).resolve(strict=True)
    if not value.is_file():
        raise ValueError(label + " must be an explicit file")
    return value


def _timeout(value, label):
    if isinstance(value, bool):
        raise ValueError(label + " must be finite and in (0, 300] seconds")
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(label + " must be finite and in (0, 300] seconds") from error
    if not math.isfinite(number) or not 0 < number <= 300:
        raise ValueError(label + " must be finite and in (0, 300] seconds")
    return number


def _terminate_tree(process, log):
    """Stop compiler children as well as the parent on a bounded timeout."""
    if os.name == "nt":
        try:
            subprocess.run(["taskkill", "/PID", str(process.pid), "/T", "/F"],
                           stdout=log, stderr=subprocess.STDOUT, timeout=5, check=False)
        except (OSError, subprocess.TimeoutExpired):
            pass
    else:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass
    if process.poll() is None:
        process.kill()


def _execute(command, cwd, log, timeout, env):
    """Retain raw merged output; no shell, buffered filter or polling task."""
    started = time.monotonic()
    timed_out = False
    options = {"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP | subprocess.CREATE_NO_WINDOW} if os.name == "nt" else {"start_new_session": True}
    with Path(log).open("wb") as stream:
        process = subprocess.Popen(list(map(str, command)), cwd=cwd, env=env,
                                   stdout=stream, stderr=subprocess.STDOUT, **options)
        try:
            process.communicate(timeout=timeout)
        except subprocess.TimeoutExpired:
            timed_out = True
            _terminate_tree(process, stream)
            process.communicate(timeout=5)
    return {"command": list(map(str, command)), "exit_code": process.returncode,
            "timed_out": timed_out, "seconds": time.monotonic()-started, "log": str(log)}


def _strict_report(value):
    if not isinstance(value, dict) or value.get("all_passed") is not True or value.get("model_accepted") is not False:
        raise ValueError("native report must declare all_passed=true and model_accepted=false")
    cases = value.get("cases")
    if not isinstance(cases, list) or len(cases) != len(_CASES):
        raise ValueError("native report must contain all four required cases, including level2")
    for case, (name, level, unused) in zip(cases, _CASES):
        if not isinstance(case, dict) or case.get("name") != name:
            raise ValueError("native case order/name does not match the pinned fixture")
        integers = {"level": level, "input_vertices": 5 if unused else 4,
                    "reported_surface_count": 4, "actual_surface_nodes": 4,
                    "zero_control_surfaces": 0, "checked_wake_nodes_per_path": 8}
        if any(type(case.get(key)) is not int or case[key] != expected for key, expected in integers.items()):
            raise ValueError(name + " has an invalid level or surface-node contract")
        if type(case.get("available_levels")) is not int or case["available_levels"] < max(2, level):
            raise ValueError(name + " did not produce the required real native multigrid levels")
        if case.get("unused_vertex") is not unused or case.get("nonidentity") is not unused:
            raise ValueError(name + " has the wrong unused-vertex/nonidentity control")
        for guard in _GUARDS:
            if case.get(guard) is not True:
                raise ValueError(name + " failed native guard " + guard)
        expected = [{"te": 2, "wake": 5, "fine_te": 3 if unused else 2, "fine_wake": 6 if unused else 5},
                    {"te": 3, "wake": 9, "fine_te": 4 if unused else 3, "fine_wake": 10 if unused else 9}]
        attachments = case.get("attachments")
        if not isinstance(attachments, list) or len(attachments) != 2:
            raise ValueError(name + " must retain both TE/wake attachment maps")
        if any(not isinstance(item, dict) or set(item) != set(want)
               or any(type(item[key]) is not int or item[key] != want[key] for key in want)
               for item, want in zip(attachments, expected)):
            raise ValueError(name + " does not match the pinned native attachment maps")
    return value


def _read_report(path):
    def reject(value):
        raise ValueError("native report contains nonfinite JSON token " + value)
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError("native report contains duplicate JSON key " + key)
            result[key] = value
        return result
    def finite_float(value):
        number = float(value)
        if not math.isfinite(number):
            raise ValueError("native report contains an overflowing JSON number")
        return number
    return json.loads(path.read_text(encoding="utf-8"), parse_constant=reject,
                      parse_float=finite_float, object_pairs_hook=unique)


def run_validation(compiler, solver_include, library, output, *, geom_object=None,
                   compile_timeout=120., run_timeout=30.):
    """Build one explicit external-library fixture and preserve its evidence.

    Invalid existing output/timeout arguments raise before creating evidence.
    All later failures produce a failed manifest; no partial result is success.
    """
    compile_timeout = _timeout(compile_timeout, "compile timeout")
    run_timeout = _timeout(run_timeout, "run timeout")
    output = Path(output).absolute()
    if output.exists():
        raise ValueError("output directory must be new; existing evidence is never overwritten")
    output.mkdir(parents=True, exist_ok=False)
    manifest = {"schema": "fsim.vspaero.native-attachment.v1", "status": "failed", "passed": False,
                "model_accepted": False, "scope": "Public-library geometry attachment indexing only; no flow solve",
                "source_commit": SOURCE_COMMIT, "expected_original_source_sha256": EXPECTED_SOURCE_SHA256,
                "started_utc": datetime.now(timezone.utc).isoformat(), "output": str(output),
                "compile_timeout_s": compile_timeout, "run_timeout_s": run_timeout,
                "geometry_object": None, "inputs": [], "inputs_unchanged": False, "steps": []}
    inputs = []
    try:
        compiler = _file(compiler, "compiler")
        library = _file(library, "library")
        solver_include = Path(solver_include).resolve(strict=True)
        if not solver_include.is_dir():
            raise ValueError("Solver include path must be an explicit directory")
        source = _file(solver_include / "VSP_Geom.C", "original VSP_Geom.C")
        header = _file(solver_include / "VSP_Geom.H", "public VSP_Geom.H")
        manifest["original_source"] = {"path": str(source), "sha256": _sha256(source)}
        fixture = _file(_NATIVE / "vsp_wake_attachment.cpp", "maintained fixture")
        patch = _file(_NATIVE / "openvsp-3.55.0-wake-root.patch", "reviewed patch")
        license_path = _file(_NATIVE / "LICENSE.OpenVSP", "upstream license")
        if geom_object is not None:
            geom_object = _file(geom_object, "explicit geometry override object")
            manifest["geometry_object"] = str(geom_object)
        # Public includes in this pinned Solver layout are direct siblings.
        # No recursive discovery or inferred alternate solver/library is used.
        headers = sorted({p.resolve() for pattern in ("*.H", "*.h", "*.hpp", "*.inl")
                          for p in solver_include.glob(pattern) if p.is_file()})
        files = [Path(__file__).resolve(), fixture, patch, license_path, compiler, library, source, header, *headers]
        if geom_object is not None:
            files.append(geom_object)
        inputs = [{"path": str(path), "sha256": _sha256(path), "bytes": path.stat().st_size}
                  for path in dict.fromkeys(files)]
        manifest.update(inputs=inputs, compiler=str(compiler), solver_include=str(solver_include), library=str(library))
        if manifest["original_source"]["sha256"] != EXPECTED_SOURCE_SHA256:
            raise ValueError("original VSP_Geom.C does not match pinned unmodified OpenVSP 3.55.0")

        def unchanged():
            current = [{"path": item["path"], "sha256": _sha256(item["path"]), "bytes": Path(item["path"]).stat().st_size}
                       for item in inputs]
            manifest["inputs_after"] = current
            manifest["inputs_unchanged"] = current == inputs
            if not manifest["inputs_unchanged"]:
                raise ValueError("a declared build input changed during validation")

        env = os.environ.copy()
        env.update(TMP="D:/ENV/tmp", TEMP="D:/ENV/tmp", TORCH_EXTENSIONS_DIR="D:/ENV/tmp",
                   OMP_NUM_THREADS="2", OMP_THREAD_LIMIT="2")
        env["PATH"] = str(compiler.parent) + os.pathsep + env.get("PATH", "")
        manifest["environment"] = {key: env[key] for key in ("TMP", "TEMP", "TORCH_EXTENSIONS_DIR", "OMP_NUM_THREADS", "OMP_THREAD_LIMIT")}

        def execute(command, name, timeout):
            log = output / (name + ".log")
            step = {"command": list(map(str, command)), "cwd": str(output), "timeout_s": timeout,
                    "log": str(log), "exit_code": None, "timed_out": False}
            manifest["steps"].append(step)
            try:
                step.update(_execute(command, output, log, timeout, env))
            except (OSError, subprocess.SubprocessError) as error:
                step["error"] = type(error).__name__ + ": " + str(error)
                step["timed_out"] = isinstance(error, subprocess.TimeoutExpired)
                raise ValueError(name + " execution failed: " + str(error)) from error
            finally:
                if log.is_file():
                    step["log_sha256"] = _sha256(log)
            if step["timed_out"] or step["exit_code"] != 0:
                raise ValueError(name + (" timed out" if step["timed_out"] else " failed with exit " + str(step["exit_code"])))
            return step

        execute([str(compiler), "--version"], "compiler-version", min(compile_timeout, 10.))
        executable = output / ("vsp_wake_attachment.exe" if os.name == "nt" else "vsp_wake_attachment")
        command = [str(compiler), "-std=c++17", "-O3", "-DNDEBUG", "-DVSPAERO_OPENMP", "-fopenmp"]
        if os.name == "nt":
            command.append("-DWIN32")
        command.extend(["-iquote", str(solver_include), str(fixture)])
        if geom_object is not None:
            command.append(str(geom_object))
        command.append(str(library))
        if os.name == "nt":
            command.append("-lmingwthrd")
        command.extend(["-o", str(executable)])
        execute(command, "compile", compile_timeout)
        unchanged()
        executable = _file(executable, "compiled fixture executable")
        manifest["executable"] = {"path": str(executable), "sha256_before_run": _sha256(executable)}
        execute([str(executable), str(output / "cases")], "native", run_timeout)
        manifest["executable"]["sha256_after_run"] = _sha256(executable)
        unchanged()
        if manifest["executable"]["sha256_after_run"] != manifest["executable"]["sha256_before_run"]:
            raise ValueError("compiled fixture executable changed during native execution")
        report_path = output / "cases/report.json"
        _strict_report(_read_report(report_path))
        manifest.update(status="passed", passed=True)
    except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError) as error:
        manifest["error"] = type(error).__name__ + ": " + str(error)
    finally:
        # Failure evidence is retained too: a nonzero native exit, malformed
        # report or launch failure must never erase its attempted inputs/logs.
        if "executable" in manifest:
            executable_record = manifest["executable"]
            try:
                executable_record["sha256_after_run"] = _sha256(executable_record["path"])
                if executable_record["sha256_after_run"] != executable_record["sha256_before_run"]:
                    manifest.update(status="failed", passed=False)
                    manifest.setdefault("error", "compiled fixture executable changed during native execution")
            except OSError as error:
                executable_record["recheck_error"] = str(error)
                manifest.update(status="failed", passed=False)
                manifest.setdefault("error", "compiled fixture executable is unavailable after execution")
        report_path = output / "cases/report.json"
        if report_path.is_file():
            item = {"path": str(report_path), "sha256": _sha256(report_path)}
            try:
                item["contents"] = _read_report(report_path)
            except (OSError, ValueError, TypeError) as error:
                item["parse_error"] = type(error).__name__ + ": " + str(error)
                manifest.update(status="failed", passed=False)
            else:
                try:
                    _strict_report(item["contents"])
                except (ValueError, TypeError) as error:
                    item["validation_error"] = str(error)
                    manifest.update(status="failed", passed=False)
                    manifest.setdefault("error", str(error))
            manifest["report"] = item
        if inputs:
            try:
                after = [{"path": item["path"], "sha256": _sha256(item["path"]), "bytes": Path(item["path"]).stat().st_size}
                         for item in inputs]
                manifest["inputs_after"] = after
                manifest["inputs_unchanged"] = after == inputs
            except OSError as error:
                manifest["inputs_unchanged"] = False
                manifest["input_recheck_error"] = str(error)
            if not manifest["inputs_unchanged"]:
                manifest.update(status="failed", passed=False)
                manifest.setdefault("error", "a declared build input changed during validation")
        manifest["finished_utc"] = datetime.now(timezone.utc).isoformat()
        (output / "manifest.json").write_text(json.dumps(manifest, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", required=True, type=Path, help="explicit GNU-compatible C++ compiler")
    parser.add_argument("--solver-include", required=True, type=Path, help="pinned original 3.55.0 Solver directory")
    parser.add_argument("--library", required=True, type=Path, help="explicit compatible native libvspaero_lib.a")
    parser.add_argument("--geometry-object", type=Path, help="explicit separately compiled corrected VSP_Geom object, linked before library")
    parser.add_argument("--output", required=True, type=Path, help="new evidence directory")
    parser.add_argument("--compile-timeout", type=float, default=120.)
    parser.add_argument("--run-timeout", type=float, default=30.)
    args = parser.parse_args(argv)
    try:
        result = run_validation(args.compiler, args.solver_include, args.library, args.output,
                                geom_object=args.geometry_object, compile_timeout=args.compile_timeout,
                                run_timeout=args.run_timeout)
    except (OSError, ValueError) as error:
        parser.exit(2, str(error) + "\n")
    print(json.dumps({"status": result["status"], "passed": result["passed"], "model_accepted": False,
                      "manifest": str(args.output.absolute() / "manifest.json")}))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
