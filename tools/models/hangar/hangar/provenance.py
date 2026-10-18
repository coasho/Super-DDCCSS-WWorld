"""Content-bound evidence for explicitly listed files; not physical-model acceptance.

Run ``python -m hangar.provenance capture|verify --root ROOT --manifest FILE
--input RELATIVE_PATH ... --artifact RELATIVE_PATH ... [--card RELATIVE_PATH]``.
Verification requires the caller's expected file sets. This does not discover a
dependency closure, archive remote references or assess their scientific quality.
"""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import sys


SCHEMA = "fsim.file-provenance.v1"
SOURCE_KINDS = frozenset(("government_publication", "manufacturer_publication", "operator_publication",
                          "research_publication", "secondary_reference", "game_reference", "unclassified"))


def _root(folder):
    path = Path(folder)
    if any(p.casefold() == ".codex" for p in path.parts):
        raise ValueError("excluded root")
    path = path.resolve(strict=True)
    if any(p.casefold() == ".codex" for p in path.parts) or not path.is_dir():
        raise ValueError("root must be a permitted directory")
    return path


def _manifest_path(filename):
    path = Path(filename)
    if any(p.casefold() == ".codex" for p in path.parts):
        raise ValueError("excluded manifest path")
    path = path.resolve()
    if any(p.casefold() == ".codex" for p in path.parts):
        raise ValueError("excluded manifest path")
    return path


def _paths(paths):
    result, seen = [], set()
    for value in paths:
        if not isinstance(value, str):
            raise ValueError("paths must be relative POSIX strings")
        path = PurePosixPath(value)
        if (not value or path.is_absolute() or "\\" in value or ":" in value or
                any(p in ("", ".", "..") or p.casefold() == ".codex" for p in value.split("/")) or
                path.as_posix() != value):
            raise ValueError("noncanonical or excluded path: %s" % value)
        if value.casefold() in seen:
            raise ValueError("duplicate or case-ambiguous path: %s" % value)
        seen.add(value.casefold())
        result.append(value)
    return sorted(result)


def _file(root, relative):
    try:
        path = root.joinpath(*PurePosixPath(relative).parts).resolve(strict=True)
    except OSError as error:
        raise ValueError("missing or unreadable file: %s" % relative) from error
    if not path.is_relative_to(root) or any(p.casefold() == ".codex" for p in path.parts):
        raise ValueError("path leaves the permitted root: %s" % relative)
    if not path.is_file():
        raise ValueError("not a file: %s" % relative)
    return path


def _record(root, relative):
    path = _file(root, relative)
    before = path.stat()
    digest, count = hashlib.sha256(), 0
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
            count += len(block)
    after = path.stat()
    if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns) or count != after.st_size:
        raise ValueError("file changed during capture: %s" % relative)
    return {"path": relative, "bytes": count, "sha256": digest.hexdigest()}


def _invalid_constant(value):
    raise ValueError("nonfinite JSON constant: %s" % value)


def _load(path):
    return json.loads(path.read_text(encoding="utf-8"), parse_constant=_invalid_constant)


def _card(root, relative):
    data = _load(_file(root, relative))
    if not isinstance(data, dict) or data.get("id") != PurePosixPath(relative).stem:
        raise ValueError("card id must match its filename")
    if not isinstance(data.get("model"), str) or not data["model"].startswith("jsbsim:") or not data["model"][7:]:
        raise ValueError("card must identify its model")
    parameters, sources = data.get("parameters"), data.get("sources")
    if not isinstance(parameters, dict) or not isinstance(sources, dict):
        raise ValueError("card parameters and sources must be objects")
    source_kinds = {}
    for name, source in sorted(sources.items()):
        kind = source.get("evidence_kind", "unclassified") if isinstance(source, dict) else None
        if not isinstance(kind, str) or kind not in SOURCE_KINDS:
            raise ValueError("unrecognized source category for %s" % name)
        source_kinds[name] = kind
    summary = {}
    for name, parameter in sorted(parameters.items()):
        if not isinstance(parameter, dict) or parameter.get("basis") not in ("published", "derived", "assumed"):
            raise ValueError("parameter %s has no recognized basis" % name)
        source = parameter.get("source")
        if source is not None and (not isinstance(source, str) or not isinstance(sources.get(source), dict)):
            raise ValueError("unresolved citation for parameter %s" % name)
        if parameter["basis"] == "published" and source is None:
            raise ValueError("published parameter %s needs a citation" % name)
        summary[name] = {"basis": parameter["basis"], "source": source,
                         "source_kind": source_kinds.get(source)}
    return {"path": relative, "id": data["id"], "model": data["model"], "parameters": summary,
            "source_kinds": source_kinds}


def _digest(payload):
    canonical = json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False)
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def capture(root, inputs, artifacts, *, card=None):
    """Capture only the named files and declared card labels, without admitting their physics."""
    root = _root(root)
    inputs, artifacts = _paths(inputs), _paths(artifacts)
    if not inputs or not artifacts:
        raise ValueError("expected input and artifact sets cannot be empty")
    _paths(inputs + artifacts)
    if card is not None:
        card = _paths([card])[0]
        if card not in inputs:
            raise ValueError("card must be in the expected input set")
    payload = {"schema": SCHEMA, "source_assessment": "not_assessed",
               "inputs": [_record(root, p) for p in inputs],
               "artifacts": [_record(root, p) for p in artifacts], "card": _card(root, card) if card else None}
    if card and _record(root, card) != next(r for r in payload["inputs"] if r["path"] == card):
        raise ValueError("card changed during capture")
    return dict(payload, snapshot_sha256=_digest(payload))


def verify(root, manifest, inputs, artifacts, *, card=None):
    """Read-only verification against independent expected sets and current file bytes."""
    keys = {"schema", "source_assessment", "inputs", "artifacts", "card", "snapshot_sha256"}
    if not isinstance(manifest, dict) or set(manifest) != keys or manifest["schema"] != SCHEMA:
        raise ValueError("unsupported or malformed provenance manifest")
    for group, expected in (("inputs", inputs), ("artifacts", artifacts)):
        records = manifest[group]
        if not isinstance(records, list):
            raise ValueError("malformed %s records" % group)
        for record in records:
            if (not isinstance(record, dict) or set(record) != {"path", "bytes", "sha256"} or
                    type(record["bytes"]) is not int or record["bytes"] < 0 or
                    not isinstance(record["sha256"], str) or not re.fullmatch(r"[0-9a-f]{64}", record["sha256"])):
                raise ValueError("malformed %s record" % group)
        actual = _paths([r["path"] for r in records])
        if actual != _paths(expected):
            raise ValueError("expected %s set differs from manifest" % group)
    if manifest["source_assessment"] != "not_assessed":
        raise ValueError("this manifest cannot claim source admission")
    payload = {k: v for k, v in manifest.items() if k != "snapshot_sha256"}
    if manifest["snapshot_sha256"] != _digest(payload):
        raise ValueError("manifest snapshot digest differs")
    fresh = capture(root, inputs, artifacts, card=card)
    if fresh != manifest:
        raise ValueError("file contents or card metadata differ from the captured snapshot")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("operation", choices=("capture", "verify"))
    parser.add_argument("--root", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--input", action="append", default=[], dest="inputs", required=True)
    parser.add_argument("--artifact", action="append", default=[], dest="artifacts")
    parser.add_argument("--card")
    args = parser.parse_args(argv)
    try:
        path = _manifest_path(args.manifest)
        if args.operation == "capture":
            root = _root(args.root)
            listed = _paths(args.inputs + args.artifacts)
            if any(path.resolve() == _file(root, name) for name in listed):
                raise ValueError("manifest must not overwrite an input or artifact")
            manifest = capture(root, args.inputs, args.artifacts, card=args.card)
            path.write_text(json.dumps(manifest, sort_keys=True, indent=2) + "\n", encoding="utf-8")
        else:
            verify(args.root, _load(path), args.inputs, args.artifacts, card=args.card)
        print("provenance %s passed; file integrity only, not model acceptance" % args.operation)
        return 0
    except (OSError, ValueError) as error:
        print("provenance: %s" % error, file=sys.stderr)
        return 3


if __name__ == "__main__":
    sys.exit(main())
