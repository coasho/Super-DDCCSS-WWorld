"""Validate the pinned VSPAERO 7.2.2 steady nonlinear history.

OpenVSP 3.54.0 pins commit db6c6fe8f4eca27d7517f4ae0b2078abdd3fd927:
https://github.com/OpenVSP/OpenVSP/blob/db6c6fe8f4eca27d7517f4ae0b2078abdd3fd927/src/vsp_aero/Solver/VSP_Solver.C

VSP_SOLVER::Solve, line 3225, requires MaxResidual <=
0.001 * NonLinearConvergenceFactor. OutputStatusFile, lines 35490-35491,
writes log10(L2Residual) and log10(MaxResidual) to the final three history
columns. The campaign fixes that factor to 1, giving the default -3 threshold.
Passing inner GMRES targets does not establish this outer convergence.

This helper accepts only the two declared steady conditions. It checks the
history against the current polar before judging convergence. Its result does
not establish physically credible loads or equipment calibration.
"""
import math
from pathlib import Path
import re


_COLUMNS = ("Iter Mach AoA Beta CLo CLi CLtot CDo CDi CDtot CSo CSi CStot L/D E "
            "CMox CMoy CMoz CMix CMiy CMiz CMxtot CMytot CMztot CFox CFoy CFoz "
            "CFix CFiy CFiz CFxtot CFytot CFztot CLwtot CDwtot CSwtot CLiw CDiw "
            "CSiw CFwxtot CFwytot CFwztot CFiwx CFiwy CFiwz LoDw Ew T/QS "
            "L2Residual MaxResidual Wall_Time").split()
_METADATA = {"Mach_": "no_unit", "AoA_": "deg", "Beta_": "deg"}
_CONDITION_ABS = 1e-10
_LOAD_REL, _LOAD_ABS = 1e-9, 1e-10


def _finite(value, label):
    try:
        number = float(value)
    except (ValueError, TypeError, OverflowError) as error:
        raise ValueError(label + " must be finite") from error
    if isinstance(value, bool) or not math.isfinite(number):
        raise ValueError(label + " must be finite")
    return number


def _condition(mach, alpha, beta):
    mach, alpha, beta = (_finite(value, label) for value, label in
                         ((mach, "Mach"), (alpha, "alpha"), (beta, "beta")))
    if (not math.isclose(mach, .6, rel_tol=0, abs_tol=_CONDITION_ABS)
            or alpha not in (0.0, 2.0) or abs(beta) > _CONDITION_ABS):
        raise ValueError("history/polar conditions must be Mach 0.6, alpha 0/2 and beta 0")
    return mach, alpha, beta


def _loads_match(left, right):
    return math.isclose(left, right, rel_tol=_LOAD_REL, abs_tol=_LOAD_ABS)


def _polar_loads(rows):
    if not isinstance(rows, (list, tuple)) or len(rows) != 2:
        raise ValueError("polar requires exactly the two frozen cases")
    expected = {}
    try:
        for row in rows:
            _, alpha, _ = _condition(row["mach"], row["alpha_deg"], row.get("beta_deg", 0))
            if alpha in expected:
                raise ValueError("duplicate polar condition")
            coefficients = row["coefficients"]
            ca, cn, cm = (_finite(coefficients[key], key) for key in ("ca", "cn", "cm"))
            cd = _finite(row["wind_cd"], "polar CDi")
            angle = math.radians(alpha)
            if not _loads_match(cd, ca * math.cos(angle) + cn * math.sin(angle)):
                raise ValueError("polar body and wind coefficients disagree")
            expected[alpha] = {"CDi": cd, "CLi": cn * math.cos(angle) - ca * math.sin(angle), "CMiy": cm}
    except (KeyError, TypeError, AttributeError) as error:
        raise ValueError("polar is missing finite inviscid loads or conditions") from error
    if set(expected) != {0.0, 2.0}:
        raise ValueError("polar requires exactly alpha 0 and 2")
    return expected


def parse_history(path, polar_rows, threshold=-3.0):
    """Return final per-case residuals; invalid inputs raise ValueError.

    A structurally valid history above the threshold returns a false gate,
    distinguishing numerical nonconvergence from a stale/malformed history.
    Existing parse_vspaero rows omit beta; the history itself must prove beta 0.
    """
    threshold = _finite(threshold, "nonlinear residual threshold")
    expected = _polar_loads(polar_rows)
    try:
        lines = Path(path).read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeError, TypeError) as error:
        raise ValueError("missing or unreadable VSPAERO history") from error
    cases, metadata, current = [], {}, None

    def finish():
        if current is None:
            return
        if not current["header"] or not current["rows"]:
            raise ValueError("VSPAERO history case has no complete numeric table")
        cases.append(current)

    for line in lines:
        text = line.strip()
        if not text:
            continue
        if set(text) == {"*"}:
            finish()
            current, metadata = None, {}
            continue
        case = re.fullmatch(r"Solver Case:\s*(\d+)", text)
        if text.startswith("Solver Case:") and not case:
            raise ValueError("malformed VSPAERO history case identifier")
        if case:
            if current is not None or int(case[1]) != len(cases) + 1 or len(cases) >= 2:
                raise ValueError("duplicate, missing or extra VSPAERO history case")
            if set(metadata) != set(_METADATA):
                raise ValueError("history case is missing unique condition metadata")
            condition = _condition(metadata["Mach_"], metadata["AoA_"], metadata["Beta_"])
            current = {"case": int(case[1]), "condition": condition, "header": False, "rows": []}
            metadata = {}
            continue
        fields = text.split()
        if fields[0] in _METADATA:
            key = fields[0]
            if current is not None or key in metadata or len(fields) != 3 or fields[2] != _METADATA[key]:
                raise ValueError("invalid or duplicate history condition metadata")
            metadata[key] = _finite(fields[1], key)
            continue
        if current is None:
            # Case banners, reference quantities and explanatory text precede
            # each Solver Case. An orphan numeric table is never a valid case.
            if fields[0] in ("Iter", "Time") or re.match(r"^[+-]?(?:\d|\.\d)", text):
                raise ValueError("history table is missing its Solver Case header")
            continue
        if fields[0] in ("Iter", "Time"):
            normalized = text.replace("L2 Residual", "L2Residual").replace("Max Residual", "MaxResidual").split()
            if current["header"] or normalized != _COLUMNS:
                raise ValueError("unsupported, duplicate or incomplete steady 7.2.2 history header")
            current["header"] = True
            continue
        if not current["header"]:
            if text.startswith(("Surface Integration Forces", "Surf-Surf")):
                continue
            raise ValueError("history case has no supported steady table header")
        if len(fields) != len(_COLUMNS):
            raise ValueError("truncated or unsupported VSPAERO history row")
        row = {key: _finite(value, "history " + key) for key, value in zip(_COLUMNS, fields)}
        iteration = row["Iter"]
        if (iteration <= 0 or not iteration.is_integer()
                or (current["rows"] and iteration <= current["rows"][-1]["Iter"])):
            raise ValueError("history iterations must be positive increasing integers")
        condition = _condition(row["Mach"], row["AoA"], row["Beta"])
        if any(not math.isclose(a, b, rel_tol=0, abs_tol=_CONDITION_ABS)
               for a, b in zip(condition, current["condition"])):
            raise ValueError("history numeric conditions disagree with case metadata")
        current["rows"].append(row)
    finish()
    if metadata or len(cases) != 2 or {case["condition"][1] for case in cases} != {0.0, 2.0}:
        raise ValueError("history requires exactly the two complete frozen cases")

    records = []
    for case in sorted(cases, key=lambda item: item["condition"][1]):
        last = case["rows"][-1]
        mach, alpha, beta = case["condition"]
        if any(not _loads_match(last[key], value) for key, value in expected[alpha].items()):
            raise ValueError("final history inviscid loads do not match the current polar")
        records.append({"case": case["case"], "mach": mach, "alpha_deg": alpha, "beta_deg": beta,
                        "iterations": len(case["rows"]), "final_iteration": int(last["Iter"]),
                        "l2_residual_log10": last["L2Residual"], "max_residual_log10": last["MaxResidual"],
                        "inviscid_loads": {key: last[key] for key in ("CLi", "CDi", "CMiy")},
                        "passed": last["MaxResidual"] <= threshold})
    return {"nonlinear_converged": all(case["passed"] for case in records),
            "threshold_log10": threshold, "cases": records}
