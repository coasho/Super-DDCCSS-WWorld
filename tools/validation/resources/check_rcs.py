"""Check RCS/model byte identity using the existing native provenance contract.

This is resource consistency, not electromagnetic or equipment calibration.
Native --verify may retain an older bake while recording a matching new model;
its original noise-relative criteria must have passed. No threshold is fitted.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct


def _allowed(path):
    path = Path(path)
    if any(part.casefold() == ".codex" for part in path.parts):
        raise ValueError("excluded resource path")
    return path


def _number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
        raise ValueError("RCS verification metric must be finite and nonnegative")
    return value


def _model_hash(path):
    # Exact legacy native algorithm, tools/models/rcs/main.cpp:fnv. Its offset
    # is part of the existing file contract, not the standard FNV-1a offset.
    value = 1469598103934665603
    sha = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(65536), b""):
            sha.update(block)
            for byte in block:
                value = ((value ^ byte) * 1099511628211) & 0xffffffffffffffff
    return "%016x" % value, sha.hexdigest()


def check_binding(table, model):
    table, model = _allowed(table), _allowed(model)
    raw = table.read_bytes()
    if len(raw) < 12 or raw[:8] != b"FSRCS\x00\x01\x00":
        raise ValueError("unsupported RCS header: " + str(table))
    size = struct.unpack_from("<I", raw, 8)[0]
    if size > (1 << 24) or len(raw) < 12 + size:
        raise ValueError("invalid or truncated RCS header: " + str(table))
    try:
        header = json.loads(raw[12:12 + size].decode("utf-8"))
        provenance = header["provenance"]
        if header["aircraft"] != model.stem or provenance["model"] != model.name:
            raise ValueError("RCS model name differs: " + str(table))
        actual, model_sha = _model_hash(model)
        binding = "bake"
        if actual != provenance["model_fnv1a"]:
            verified = provenance["verified"]
            if verified["model"] != model.name or actual != verified["model_fnv1a"]:
                raise ValueError("RCS bake and verification both refer to other model bytes: " + str(table))
            if verified["bands_hz"] != header["bands_hz"] or _number(verified["bins"]) <= 0:
                raise ValueError("incomplete or mismatched native RCS verification")
            tolerance = _number(verified["tolerance_db"])
            # Unchanged native gate: main.cpp, differences(table, old) versus
            # differences(table, jittered), for max, p99 and p95 independently.
            for metric in ("max_db", "p99_db", "p95_db"):
                if _number(verified[metric]) > _number(verified["noise_" + metric]) + tolerance:
                    raise ValueError("stored RCS verification fails the native noise criterion")
            binding = "verification"
    except (KeyError, TypeError, UnicodeError) as error:
        raise ValueError("missing or malformed RCS binding metadata: " + str(table)) from error
    return {"passed": True, "binding": binding, "model": str(model), "table": str(table),
            "native_model_hash": actual, "model_sha256": model_sha,
            "table_sha256": hashlib.sha256(raw).hexdigest()}


def audit_root(root):
    root = _allowed(root)
    rows = []
    # Only immediate design folders. Excluded names are discarded before any
    # inspection of their contents; no repository-wide traversal is performed.
    for folder in sorted(root.iterdir()):
        if folder.name.casefold() == ".codex" or not folder.is_dir():
            continue
        table = folder / "rcs.bin"
        if table.is_file():
            rows.append(check_binding(table, folder / (folder.name + ".glb")))
    if not rows:
        raise ValueError("empty RCS asset inventory")
    return {"passed": True, "scope": "resource binding only; physical accuracy not assessed", "files": rows}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--aircraft-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    target = _allowed(args.output)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text('{"passed":false,"status":"running"}\n', encoding="utf-8")
    try:
        result = audit_root(args.aircraft_root)
    except (ValueError, OSError) as error:
        result = {"passed": False, "reason": str(error)}
    target.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({key: value for key, value in result.items() if key != "files"} |
                     {"checked": len(result.get("files", []))}))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
