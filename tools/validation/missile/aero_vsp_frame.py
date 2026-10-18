"""Rigid frame utilities for the representative OpenVSP panel experiment.

The pinned native trim classifier's (1, 1e-6, 1e-6) ray is tangent to the
authored diagonal fin tip caps. A complete 0.123-degree roll avoids that
coordinate degeneracy without changing physical geometry. Wind, origins,
the moment reference and all load vectors must share that same proper roll.
This helper does not establish mesh closure, flow convergence or calibration.

Pinned OpenVSP_3.54.0 source conventions:
* TMesh.h::DeterIntExt defines the classification ray.
* VSP_Geom.C uses wind=(cos(alpha)*cos(beta), -sin(beta), sin(alpha)*cos(beta)).
* VSP_Solver.C::IntegrateForcesAndMoments normalizes moments by
  q*Sref*[Bref, Cref, Bref]. Undo that scaling before rotating moments.
https://github.com/OpenVSP/OpenVSP/tree/OpenVSP_3.54.0/src
"""
from collections.abc import Mapping
import math


FRAME_ROLL_DEG = .123
_LOAD_KEYS = ("CFix", "CFiy", "CFiz", "CMix", "CMiy", "CMiz")


def _finite(value, label):
    try:
        result = float(value)
    except (ValueError, TypeError, OverflowError) as error:
        raise ValueError(label + " must be finite") from error
    if isinstance(value, bool) or not math.isfinite(result):
        raise ValueError(label + " must be finite")
    return result


def _vector(values, label):
    if not isinstance(values, (list, tuple)) or len(values) != 3:
        raise ValueError(label + " must have three finite coordinates")
    return [_finite(value, label) for value in values]


def rotate_vector(values, roll_deg=FRAME_ROLL_DEG):
    """Rotate a position or vector about the common x axis and origin.

    Use this on the CG as well as geometry. Passing a negative roll applies
    the inverse. A proper rotation also applies to axial moment vectors.
    """
    x, y, z = _vector(values, "vector")
    angle = math.radians(_finite(roll_deg, "roll"))
    c, s = math.cos(angle), math.sin(angle)
    return [_finite(value, "rotated vector") for value in (x, c*y-s*z, s*y+c*z)]


def paired_conditions(roll_deg=FRAME_ROLL_DEG):
    """Return exactly the physical 0/2-degree cases and paired native winds.

    Each dictionary describes one solver execution. Its native alpha and beta
    are a pair, not independent sweep axes. A Cartesian sweep would introduce
    different physical conditions and must not be used by the caller.
    """
    roll = _finite(roll_deg, "roll")
    cases = []
    for alpha in (0.0, 2.0):
        angle = math.radians(alpha)
        wind = [math.cos(angle), 0.0, math.sin(angle)]
        native = rotate_vector(wind, roll)
        native_alpha = math.degrees(math.atan2(native[2], native[0]))
        native_beta = math.degrees(math.asin(max(-1.0, min(1.0, -native[1]))))
        cases.append({"roll_deg": roll,
                      "physical": {"mach": .6, "alpha_deg": alpha, "beta_deg": 0.0, "wind_xyz": wind},
                      "native": {"mach": .6, "alpha_deg": native_alpha, "beta_deg": native_beta,
                                 "wind_xyz": native}})
    return cases


def roll_component_frames(components, roll_deg=FRAME_ROLL_DEG):
    """Prevalidate a complete top-level component batch, then return copies.

    This seam supports the representative mesher's independent native bodies
    and fins, each with only an existing x-axis Euler rotation. Nested frames
    or nonzero y/z Euler rotations require composition outside this helper;
    they fail explicitly instead of accepting an incorrect additive roll.
    No supplied dictionary or coordinate list is mutated, even on failure.
    """
    roll = _finite(roll_deg, "roll")
    if not isinstance(components, (list, tuple)) or not components:
        raise ValueError("component frames must be a nonempty complete batch")
    validated, identities = [], set()
    for component in components:
        if not isinstance(component, Mapping):
            raise ValueError("component frame must be a mapping")
        try:
            identity, parent = component["id"], component["parent_id"]
            origin = _vector(component["origin_xyz"], "component origin")
            rotation = _vector(component["rotation_xyz"], "component rotation")
        except KeyError as error:
            raise ValueError("incomplete component frame") from error
        if not isinstance(identity, str) or not identity.strip() or identity in identities:
            raise ValueError("component identities must be unique nonempty strings")
        if parent != "" or rotation[1:] != [0.0, 0.0]:
            raise ValueError("only top-level component frames with x-axis rotation are supported")
        identities.add(identity)
        validated.append((dict(component), origin, rotation))
    result = []
    for component, origin, rotation in validated:
        component["origin_xyz"] = rotate_vector(origin, roll)
        component["rotation_xyz"] = [_finite(rotation[0]+roll, "component rotation"), 0.0, 0.0]
        result.append(component)
    return result


def recover_loads(native_coefficients, roll_deg=FRAME_ROLL_DEG, *, bref, cref):
    """Return all six original-frame inviscid coefficients and raw inputs.

    The native and physical moment references must be the same physical CG,
    transformed with rotate_vector. No origin shift is inferred. Forces share
    q*Sref, whereas moments first need their [Bref,Cref,Bref] length factors.
    """
    roll = _finite(roll_deg, "roll")
    span, chord = _finite(bref, "Bref"), _finite(cref, "Cref")
    if span <= 0 or chord <= 0:
        raise ValueError("moment reference lengths must be positive")
    if not isinstance(native_coefficients, Mapping) or set(native_coefficients) != set(_LOAD_KEYS):
        raise ValueError("exactly the six native inviscid body coefficients are required")
    native = {key: _finite(native_coefficients[key], key) for key in _LOAD_KEYS}
    force = rotate_vector([native[key] for key in _LOAD_KEYS[:3]], -roll)
    scales = [span, chord, span]
    moment = rotate_vector([native[key]*length for key, length in zip(_LOAD_KEYS[3:], scales)], -roll)
    values = force + [_finite(value/length, "recovered moment") for value, length in zip(moment, scales)]
    return {"roll_deg": roll, "bref": span, "cref": chord, "native": native,
            "original": dict(zip(_LOAD_KEYS, values))}
