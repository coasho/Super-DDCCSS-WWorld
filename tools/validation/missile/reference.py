"""Numerical safeguards for offline missile reference comparisons.

These helpers describe explicit game inputs. They do not admit equipment data
or participate in the running simulator.
"""
import math
import hashlib
from pathlib import Path, PurePosixPath
import xml.etree.ElementTree as ET


CHANNELS = ("mass_kg", "speed_ms", "height_m")
PLANET_RADIUS_M = 6378137.0
PLANET_MU_M3_S2 = 398600441800000.0


def radial_gravity(height):
    """The declared nonrotating spherical reference planet's central field."""
    radius = _number(PLANET_RADIUS_M + _number(height, "altitude"), "radius", positive=True)
    return PLANET_MU_M3_S2 / radius**2


def verify_environment(actual, requested, *, spherical=False):
    """Reject ignored native overrides before any trajectory can earn a pass."""
    wanted = dict(requested)
    if spherical:
        wanted["gravity_ms2"] = radial_gravity(8000)
    for key, value in wanted.items():
        observed = _number(actual.get(key), "native " + key)
        if not math.isclose(observed, value, rel_tol=1e-10, abs_tol=1e-12):
            raise ValueError("native reference environment differs: " + key)


def axial_drag(text):
    """Read the supported emitted CA product, including every constant factor."""
    try:
        nodes = ET.fromstring(text).findall(".//axis[@name='AXIAL']/function[@name='aero/coefficient/CA']")
        if len(nodes) != 1:
            raise ValueError("one axial CA function is required")
        product = nodes[0].find("product")
        if product is None or any(c.tag not in ("property", "value", "table") for c in product):
            raise ValueError("unsupported axial drag expression")
        if sorted(c.text.strip() for c in product.findall("property")) != ["aero/qbar-psf", "metrics/Sw-sqft"]:
            raise ValueError("unsupported drag reference properties")
        factor = 1.0
        for value in product.findall("value"):
            factor *= _number(float(value.text), "axial drag factor", positive=True)
        tables = product.findall("table")
        if len(tables) != 1 or tables[0].findtext("independentVar", "").strip() != "velocities/mach":
            raise ValueError("one Mach drag table is required")
        rows = []
        for line in tables[0].findtext("tableData", "").strip().splitlines():
            parts = line.split()
            if len(parts) != 2:
                raise ValueError("drag rows require Mach and coefficient")
            mach, coefficient = [_number(float(v), "drag table") for v in parts]
            coefficient = _number(coefficient * factor, "axial drag coefficient")
            if mach < 0 or coefficient < 0 or (rows and mach <= rows[-1][0]):
                raise ValueError("invalid Mach drag grid")
            rows.append([mach, coefficient])
        if len(rows) < 2:
            raise ValueError("complete drag table is required")
        return rows
    except (ET.ParseError, AttributeError, TypeError, OverflowError) as error:
        raise ValueError("unsupported generated axial drag") from error


def verify_generated(generated, folder):
    """Require the complete generated model to match its current source text."""
    if not isinstance(generated, dict) or not generated:
        raise ValueError("a nonempty generated file set is required")
    for name, text in generated.items():
        if (not isinstance(name, str) or not name or "\\" in name or ":" in name or
                PurePosixPath(name).is_absolute() or
                any(p in ("", ".", "..") or p.casefold() == ".codex" for p in name.split("/")) or
                not isinstance(text, str)):
            raise ValueError("invalid generated path or source text")
    root = Path(folder)
    if any(p.casefold() == ".codex" for p in root.parts):
        raise ValueError("excluded model directory")
    records = []
    for name in sorted(generated):
        path = root / name
        # Refuse a redirected directory before resolving or reading the file.
        if any(p.is_symlink() or getattr(p, "is_junction", lambda: False)() for p in (path, *path.parents)):
            raise ValueError("redirected model path")
        try:
            actual = path.read_bytes()
            decoded = actual.decode("utf-8").replace("\r\n", "\n")
        except (OSError, UnicodeError) as error:
            raise ValueError("missing or unreadable generated model: " + name) from error
        if decoded != generated[name].replace("\r\n", "\n"):
            raise ValueError("stale generated model; run the Hangar aero/build stages: " + name)
        records.append({"path": str(path.resolve()), "sha256": hashlib.sha256(actual).hexdigest(), "bytes": len(actual)})
    return records


def _number(value, label, *, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(label + " must be a finite number")
    try:
        value = float(value)
    except OverflowError as error:
        raise ValueError(label + " is not representable") from error
    if not math.isfinite(value) or (positive and value <= 0):
        raise ValueError(label + " must be finite" + (" and positive" if positive else ""))
    return value


def phase_curve(phases, *, buildup_s=0.05, spacing_s=0.001):
    """Convert constant impulse phases to a time curve with sine ignition.

    Phase lengths specify impulse at the phase's vacuum thrust. During ignition
    the sine ramp burns less propellant, so later boundaries move by its lost
    effective time. Each phase boundary has a narrow, explicitly sampled step.
    This bounded reference supports a first phase at least as long as ignition;
    zero-thrust coast phases must be supplied as their simulated positive thrust.
    """
    buildup = _number(buildup_s, "ignition buildup", positive=True)
    spacing = _number(spacing_s, "curve spacing", positive=True)
    if not isinstance(phases, list) or not phases:
        raise ValueError("nonempty phases are required")
    elapsed, impulse, boundaries, forces = 0.0, 0.0, [], []
    for phase in phases:
        if not isinstance(phase, dict):
            raise ValueError("each phase must be an object")
        duration = _number(phase.get("duration_s"), "phase duration", positive=True)
        force = _number(phase.get("thrust_kn"), "phase thrust", positive=True) * 1000
        impulse = _number(impulse + _number(force * duration, "phase impulse", positive=True), "total impulse", positive=True)
        elapsed = _number(elapsed + duration, "total phase duration", positive=True)
        boundaries.append(elapsed)
        forces.append(force)
    if phases[0]["duration_s"] < buildup:
        raise ValueError("reference requires the first phase to cover ignition")
    deficit = buildup * (1 - 2 / math.pi)
    end = _number(elapsed + deficit, "burnout time", positive=True)
    if end / spacing > 100000 or end / spacing < 1:
        raise ValueError("curve spacing must produce between 1 and 100000 intervals")
    epsilon = min(1e-8, min(p["duration_s"] for p in phases) * 1e-6)
    times = {0.0, buildup, end}
    times.update(i * spacing for i in range(1, math.ceil(end / spacing)))
    for boundary in boundaries:
        times.update((boundary + deficit - epsilon, boundary + deficit))
    out = []
    for t in sorted(times):
        effective = t - deficit if t >= buildup else 2 * buildup / math.pi * (1 - math.cos(math.pi * t / (2 * buildup)))
        index = next((i for i, boundary in enumerate(boundaries) if effective < boundary - 1e-12), len(forces))
        thrust = forces[index] if index < len(forces) and t < end else 0.0
        if t < buildup:
            thrust *= math.sin(math.pi * t / (2 * buildup))
        out.append([t, thrust])
    return out


def compare(native, expected, limits):
    """Compare every aligned sample [time, mass, axial speed, altitude]."""
    if not isinstance(limits, dict) or set(limits) != set(CHANNELS):
        raise ValueError("all three residual limits are required")
    limits = {key: _number(limits[key], key + " limit", positive=True) for key in CHANNELS}
    if not isinstance(native, list) or not isinstance(expected, list) or len(native) != len(expected) or len(native) < 2:
        raise ValueError("complete nonempty histories with at least two samples are required")
    maxima = dict.fromkeys(CHANNELS, 0.0)
    previous = -math.inf
    for actual, wanted in zip(native, expected):
        if not isinstance(actual, (list, tuple)) or not isinstance(wanted, (list, tuple)) or len(actual) != 4 or len(wanted) != 4:
            raise ValueError("each sample needs time, mass, axial speed and altitude")
        actual = [_number(v, "native sample") for v in actual]
        wanted = [_number(v, "reference sample") for v in wanted]
        if actual[0] < 0 or actual[1] <= 0 or wanted[1] <= 0:
            raise ValueError("flight samples require nonnegative time and positive mass")
        if actual[0] <= previous or abs(actual[0] - wanted[0]) > 1e-9:
            raise ValueError("samples must have increasing aligned times")
        previous = actual[0]
        for i, key in enumerate(CHANNELS, 1):
            residual = _number(abs(actual[i] - wanted[i]), key + " residual")
            maxima[key] = max(maxima[key], residual)
    return {"passed": all(maxima[key] <= limits[key] for key in CHANNELS),
            "samples": len(native), "max_abs": maxima, "limits": limits}
