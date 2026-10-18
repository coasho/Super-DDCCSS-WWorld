"""Strict ingestion of declared offline derivative tables; no model-accuracy claim.

Tables are used as supplied. This module does not fit, interpolate, extrapolate,
or fill missing data, and producer metadata describes the input's declaration.
"""
import hashlib
import json
import math
from pathlib import Path, PurePosixPath


SCHEMA = "fsim.store-aero-derivatives.v1"
_GEOMETRY_KEYS = ("x", "r", "fins", "cg", "length", "diameter")
_COEFFICIENTS = frozenset(("ca", "cna", "cma", "cyb", "cnb", "cmq", "cnr", "clp"))
_CROSSFLOW = {"normal": "cross_n", "pitch": "cross_m", "side": "cross_y",
              "yaw": "cross_ym", "turn_m5": "turn"}


def geometry_digest(geo):
    """Hash only the geometry fields which define the table's declared identity."""
    try:
        payload = {key: geo[key] for key in _GEOMETRY_KEYS}
        canonical = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False)
    except (KeyError, TypeError, ValueError, OverflowError) as error:
        raise ValueError("geometry cannot be represented by the table contract") from error
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def _excluded(path):
    return any(part.casefold() == ".codex" for part in path.parts)


def _input_path(geo, filename):
    if not isinstance(filename, str):
        raise ValueError("table path must be a canonical relative POSIX string")
    relative = PurePosixPath(filename)
    if (not filename or relative.is_absolute() or "\\" in filename or ":" in filename or
            any(part in ("", ".", "..") or part.casefold() == ".codex" for part in filename.split("/")) or
            relative.as_posix() != filename):
        raise ValueError("table path must be canonical, relative and outside excluded directories")
    try:
        design = Path(geo["design_path"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("geometry must identify its design path") from error
    if _excluded(design):
        raise ValueError("excluded design path")
    try:
        directory = design.parent.resolve(strict=True)
        if _excluded(directory) or not directory.is_dir():
            raise ValueError("design directory is excluded or invalid")
        path = directory.joinpath(*relative.parts).resolve(strict=True)
        if _excluded(path) or not path.is_relative_to(directory) or not path.is_file():
            raise ValueError("table must be a file within its permitted design directory")
    except (OSError, RuntimeError) as error:
        raise ValueError("table path is missing or unreadable") from error
    return path


def _unique_object(pairs):
    out = {}
    for key, value in pairs:
        if key in out:
            raise ValueError("duplicate JSON key: %s" % key)
        out[key] = value
    return out


def _nonfinite_constant(value):
    raise ValueError("nonfinite JSON constant: %s" % value)


def _object(value, keys, label):
    if not isinstance(value, dict) or set(value) != set(keys):
        raise ValueError("%s must contain exactly: %s" % (label, ", ".join(sorted(keys))))
    return value


def _number(value, label):
    try:
        valid = isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
    except (TypeError, OverflowError):
        valid = False
    if not valid:
        raise ValueError("%s must be a finite real number" % label)
    return value


def load(geo, card_id, filename):
    """Read and validate one offline table bound to a card and the exact geometry."""
    path = _input_path(geo, filename)
    try:
        raw = path.read_bytes()
        data = json.loads(raw.decode("utf-8"), object_pairs_hook=_unique_object, parse_constant=_nonfinite_constant)
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise ValueError("table is missing, unreadable or invalid UTF-8 JSON") from error
    _object(data, ("schema", "card", "geometry_sha256", "reference", "producer", "mach", "coefficients", "crossflow"),
            "table")
    if data["schema"] != SCHEMA:
        raise ValueError("unsupported table schema")
    if not isinstance(card_id, str) or not card_id.strip() or data["card"] != card_id:
        raise ValueError("table card does not match the requested card")
    if data["geometry_sha256"] != geometry_digest(geo):
        raise ValueError("table geometry digest does not match the design")

    reference = _object(data["reference"], ("area_m2", "diameter_m", "moment_x_m", "frame"), "reference")
    if reference["frame"] != "body-from-first-station":
        raise ValueError("unsupported table reference frame")
    try:
        diameter = _number(geo["diameter"], "geometry diameter")
        length = _number(geo["length"], "geometry length")
        moment = _number(geo["cg"], "geometry moment reference")
        area = _number(math.pi * diameter * diameter / 4.0, "geometry reference area")
    except (KeyError, OverflowError) as error:
        raise ValueError("invalid geometry reference quantities") from error
    if diameter <= 0.0 or length <= 0.0 or area <= 0.0:
        raise ValueError("geometry reference area, diameter and length must be positive")
    for key, expected in (("area_m2", area), ("diameter_m", diameter), ("moment_x_m", moment)):
        value = _number(reference[key], "reference " + key)
        if key != "moment_x_m" and value <= 0.0:
            raise ValueError("table reference %s must be positive" % key)
        if not math.isclose(value, expected, rel_tol=1e-9, abs_tol=1e-12 if key == "moment_x_m" else 0.0):
            raise ValueError("table reference %s does not match the geometry" % key)

    producer = _object(data["producer"], ("name", "version", "case"), "producer")
    if any(not isinstance(value, str) or not value.strip() for value in producer.values()):
        raise ValueError("producer metadata must contain nonempty strings")

    mach = data["mach"]
    if not isinstance(mach, list) or len(mach) < 2:
        raise ValueError("mach must contain at least two coordinates")
    for index, value in enumerate(mach):
        _number(value, "mach coordinate")
        if value < 0 or (index and value <= mach[index - 1]):
            raise ValueError("mach coordinates must be nonnegative and strictly increasing")
    coefficients = _object(data["coefficients"], _COEFFICIENTS, "coefficients")
    for key, values in coefficients.items():
        if not isinstance(values, list) or len(values) != len(mach):
            raise ValueError("coefficient %s must match the mach grid length" % key)
        for value in values:
            _number(value, "coefficient " + key)
    crossflow = _object(data["crossflow"], _CROSSFLOW, "crossflow")
    for key, value in crossflow.items():
        _number(value, "crossflow " + key)

    out = {"S": reference["area_m2"], "d": reference["diameter_m"], "ref": reference["moment_x_m"],
           "mach": mach, **coefficients, **{_CROSSFLOW[key]: value for key, value in crossflow.items()}}
    out["input"] = {"path": filename, "sha256": hashlib.sha256(raw).hexdigest(), "producer": producer, "schema": SCHEMA}
    return out
