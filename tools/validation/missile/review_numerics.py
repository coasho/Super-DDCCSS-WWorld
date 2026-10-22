"""Reassess the imported numerical handoff without running external solvers.

The checked-in histories and geometry are research fixtures, never generated
flight-model inputs. A successful review preserves the documented coarse-grid
failure. It does not declare that failure acceptable for a production model.
"""
import argparse
import hashlib
import json
from pathlib import Path

import aero
from aero_vsp_frame import FRAME_ROLL_DEG, paired_conditions

FIXTURES = Path(__file__).with_name("native") / "numerics"
ROOT = Path(__file__).resolve().parents[3]
# Released case references, not fitted values; bound by the native config hashes.
REFERENCE = {"moment_x_m": 1.8300000000000001, "length_m": 0.17780000000000001,
             "area_m2": 0.024828666475777392}


def verify_imports():
    manifest = json.loads((FIXTURES / "PROVENANCE.json").read_text())
    checked = 0
    for row in manifest["files"]:
        if "target" not in row:
            continue
        relative = Path(row["target"])
        if relative.is_absolute() or ".." in relative.parts or any(p.lower() == ".codex" for p in relative.parts):
            raise ValueError("unsafe imported target")
        path = ROOT / relative
        if hashlib.sha256(path.read_bytes()).hexdigest() != row["target_sha256"]:
            raise ValueError("imported target hash mismatch: " + str(relative))
        checked += 1
    if not checked:
        raise ValueError("empty numerical handoff")
    return checked


def assess(vsp_cases=None):
    count = verify_imports()
    su2 = {}
    for grid in ("coarse", "local"):
        rows = [aero.parse_su2(FIXTURES / ("su2-%s-alpha%d.csv" % (grid, angle)), angle, residual_limit=-9)
                for angle in (0, 2)]
        su2[grid] = {"cases": rows, "physical_gates": aero.physical_gates(rows), "model_accepted": False}
    folders = vsp_cases or [FIXTURES / ("r2-alpha%d" % index) for index in (0, 2)]
    rows = [aero.assess_single_vsp_case(folder, pair, REFERENCE, basename="reference_body",
                                      log_name="run.log" if vsp_cases else "solver.txt")
            for folder, pair in zip(folders, paired_conditions(FRAME_ROLL_DEG))]
    gates = aero.physical_gates(rows)
    expected = (not su2["coarse"]["physical_gates"]["nonnegative_wind_drag"] and
                all(value for key, value in su2["coarse"]["physical_gates"].items() if key != "nonnegative_wind_drag") and
                all(su2["local"]["physical_gates"].values()) and all(gates.values()))
    return {"review_passed": expected, "model_accepted": False, "verified_imports": count,
            "su2": su2, "vspaero_r2": {"cases": rows, "physical_gates": gates},
            "remaining": ["coarse zero-angle negative drag", "spatial convergence and independent load accuracy",
                          "dynamic, crossflow and viscous channels", "equipment and variant calibration"]}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--vsp-cases", type=Path, nargs=2, metavar=("ALPHA0", "ALPHA2"),
                        help="fresh native replay directories, in alpha 0/2 order")
    args = parser.parse_args()
    result = assess(args.vsp_cases)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=1) + "\n")
    print(json.dumps({key: result[key] for key in ("review_passed", "model_accepted", "verified_imports", "remaining")}))
    return 0 if result["review_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
