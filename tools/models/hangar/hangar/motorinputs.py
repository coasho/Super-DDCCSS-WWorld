"""Validate the existing inline motor declaration before any consumer uses it.

This checks input shape and numeric validity only. It does not change the
declaration, add physical inputs, or assess a model's accuracy.
"""
import math
from collections.abc import Mapping, Sequence
from numbers import Real


SMOKE = ("minimum", "reduced", "aluminised")


def _number(value, field):
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError("motor: %s must be a finite real number" % field)
    try:
        number = float(value)
    except (OverflowError, TypeError, ValueError) as error:
        raise ValueError("motor: %s must be a finite real number" % field) from error
    if not math.isfinite(number):
        raise ValueError("motor: %s must be a finite real number" % field)
    return number


def validate_motor(spec):
    """Validate a motor mapping without mutation; return None on success.

    An absent or empty phase list remains valid for a visual-only declaration.
    Other optional fields retain their existing defaults. Additional descriptive
    fields are left untouched.
    """
    if not isinstance(spec, Mapping):
        raise ValueError("motor: declaration must be a mapping")
    if "exit" not in spec or "exit_diameter" not in spec:
        raise ValueError("motor: 'exit' and 'exit_diameter' are required")
    position = spec["exit"]
    if isinstance(position, (str, bytes)) or not isinstance(position, Sequence) or len(position) != 3:
        raise ValueError("motor: exit must contain exactly three finite real numbers")
    for index, value in enumerate(position):
        _number(value, "exit[%d]" % index)
    diameter = _number(spec["exit_diameter"], "exit_diameter")
    if diameter <= 0.0:
        raise ValueError("motor: exit_diameter must be positive")
    throat = _number(spec.get("throat_diameter", 0.4 * diameter), "throat_diameter")
    cone = _number(spec.get("cone_length", 1.2 * diameter), "cone_length")
    if not 0.0 < throat < diameter or cone <= 0.0:
        raise ValueError("motor: the throat is narrower than the exit, and the cone has a length")
    smoke = spec.get("smoke", "reduced")
    if not isinstance(smoke, str) or smoke not in SMOKE:
        raise ValueError("motor: smoke must be one of %s" % ", ".join(SMOKE))
    phases = spec.get("phase", [])
    if not isinstance(phases, list):
        raise ValueError("motor: phase must be a list of mappings")
    burn_s = impulse_ns = 0.0
    for index, phase in enumerate(phases):
        if not isinstance(phase, Mapping) or "duration_s" not in phase or "thrust_kn" not in phase:
            raise ValueError("motor: phase[%d] requires duration_s and thrust_kn" % index)
        duration = _number(phase["duration_s"], "phase[%d].duration_s" % index)
        thrust = _number(phase["thrust_kn"], "phase[%d].thrust_kn" % index)
        if duration <= 0.0 or thrust < 0.0:
            raise ValueError("motor: each phase lasts some time at a thrust of zero or more")
        newtons = _number(1000.0 * thrust, "phase[%d] thrust in N" % index)
        impulse = _number(newtons * duration, "phase[%d] impulse in N s" % index)
        burn_s = _number(burn_s + duration, "total burn duration in s")
        impulse_ns = _number(impulse_ns + impulse, "total impulse in N s")
