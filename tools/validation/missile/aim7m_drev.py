"""Compare the generated AIM-7M model with DREV-TM-1999-050.

The DREV report is a public, unclassified 1:7-scale wind-tunnel dataset. This
tool measures the SDK model at the same Mach numbers and alpha range, computes
the same linear slopes, and reports the error. It does not fit or publish
parameters; it is a calibration/validation reference.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from hangar.sdk_measurements import Measurements


ROOT = Path(__file__).resolve().parents[3]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def load_dataset(path: Path) -> dict:
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("schema") != "fsim.missile-wind-tunnel.v1":
        raise ValueError("unsupported dataset schema")
    if data.get("model") != "aim7m":
        raise ValueError("dataset is not for aim7m")
    source = data.get("source", {})
    local_pdf = ROOT / source.get("local_pdf", "")
    if local_pdf.is_file() and source.get("pdf_sha256"):
        if sha256(local_pdf) != source["pdf_sha256"]:
            raise ValueError("local DREV PDF hash does not match the dataset pin")
    return data


def selected_tables(data: dict, roll: float, wing: float) -> list[dict]:
    rows = []
    for row in data["tables"]:
        if row["roll_deg"] != roll or row["wing_deflection_deg"] != wing:
            continue
        if row.get("cn_alpha_per_rad") is None or row.get("cm_alpha_per_rad") is None:
            continue
        rows.append(row)
    if not rows:
        raise ValueError("no admitted table rows for the requested roll/wing")
    return rows


def measure(rows: list[dict], scratch: Path) -> list[dict]:
    cases = []
    for index, row in enumerate(rows):
        for alpha in (-5.0, 0.0, 5.0):
            cases.append({
                "id": f"drev-{index}-a{alpha:+.0f}",
                "metric": "CZ",
                "row": index,
                "alpha": alpha,
                "conditions": {
                    "alpha_deg": alpha,
                    "beta_deg": 0.0,
                    "mach": row["mach"],
                    "altitude_m": 1000.0,
                },
            })
    measurement = Measurements("aim7m", scratch)
    measured = measurement.static({}, cases)
    grouped: dict[int, list[dict]] = {}
    for case, result in zip(cases, measured):
        grouped.setdefault(case["row"], []).append({"alpha": case["alpha"], **result})
    return [grouped[index] for index in range(len(rows))]


def slope(points: list[dict], key: str) -> float:
    alpha = np.array([math.radians(point["alpha"]) for point in points], dtype=float)
    values = np.array([point["values"][key] for point in points], dtype=float)
    return float(np.polyfit(alpha, values, 1)[0])


def relative_error(actual: float, reference: float) -> float:
    if reference == 0:
        return float("nan")
    return 100.0 * (actual - reference) / reference


def evaluate(data: dict, roll: float, wing: float, scratch: Path) -> dict:
    rows = selected_tables(data, roll, wing)
    groups = measure(rows, scratch)
    points = []
    for row, group in zip(rows, groups):
        # DREV CN is the normal force in the body-normal sense; the SDK CZ is
        # the opposite body-z force at positive alpha.
        cn = [-point["values"]["CZ"] for point in group]
        cm = [point["values"]["Cm"] for point in group]
        cn_alpha = slope([{"alpha": point["alpha"], "values": {"CN": value}}
                          for point, value in zip(group, cn)], "CN")
        cm_alpha = slope([{"alpha": point["alpha"], "values": {"CM": value}}
                          for point, value in zip(group, cm)], "CM")
        xcp = -cm_alpha / cn_alpha if cn_alpha else float("nan")
        points.append({
            "mach": row["mach"],
            "roll_deg": row["roll_deg"],
            "wing_deflection_deg": row["wing_deflection_deg"],
            "locator": row["locator"],
            "reference": {
                "cn_alpha_per_rad": row["cn_alpha_per_rad"],
                "cm_alpha_per_rad": row["cm_alpha_per_rad"],
                "xcp_cal": row["xcp_cal"],
            },
            "sdk": {
                "cn_alpha_per_rad": cn_alpha,
                "cm_alpha_per_rad": cm_alpha,
                "xcp_cal": xcp,
            },
            "error_percent": {
                "cn_alpha": relative_error(cn_alpha, row["cn_alpha_per_rad"]),
                "cm_alpha": relative_error(cm_alpha, row["cm_alpha_per_rad"]),
                "xcp": relative_error(xcp, row["xcp_cal"]),
            },
            "samples": [
                {
                    "alpha_deg": point["alpha"],
                    "CZ": point["values"]["CZ"],
                    "Cm": point["values"]["Cm"],
                    "fixture_sha256": point["actual"]["measurement_fixture_xml_sha256"],
                }
                for point in group
            ],
        })

    def nrmse(key: str) -> float:
        values = np.array([point["error_percent"][key] for point in points], dtype=float)
        return float(np.sqrt(np.mean(values * values)))

    return {
        "schema": "fsim.aim7m-drev-validation.v1",
        "model": "aim7m",
        "source": data["source"],
        "conditions": {
            "roll_deg": roll,
            "wing_deflection_deg": wing,
            "alpha_deg": [-5.0, 0.0, 5.0],
            "altitude_m": 1000.0,
            "note": "SDK static fixture at 1000 m; DREV tunnel Reynolds is not matched. Slopes use the same -5/+5 deg interval.",
        },
        "points": points,
        "summary": {
            "points": len(points),
            "cn_alpha_nrmse_percent": nrmse("cn_alpha"),
            "cm_alpha_nrmse_percent": nrmse("cm_alpha"),
            "xcp_nrmse_percent": nrmse("xcp"),
        },
        "limits": data.get("limits", []),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path,
                        default=Path(__file__).with_name("data") / "aim7m_drev_tm1999_050.json")
    parser.add_argument("--out", type=Path,
                        default=ROOT / "build/architecture-reviews/sim-m02-sources/aim7m-drev-validation.json")
    parser.add_argument("--scratch", type=Path,
                        default=ROOT / "build/architecture-reviews/sim-m02-sources/aim7m-drev-scratch")
    parser.add_argument("--roll", type=float, default=0.0)
    parser.add_argument("--wing", type=float, default=0.0)
    args = parser.parse_args()

    data = load_dataset(args.data)
    if args.roll != 0.0:
        raise SystemExit("only 0 deg roll is currently measured by the static fixture")
    result = evaluate(data, args.roll, args.wing, args.scratch)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps(result["summary"], indent=2))
    for point in result["points"]:
        print(f"Mach {point['mach']:.1f}: "
              f"CNa {point['reference']['cn_alpha_per_rad']:.3f} -> {point['sdk']['cn_alpha_per_rad']:.3f} "
              f"({point['error_percent']['cn_alpha']:+.2f}%), "
              f"CMa {point['reference']['cm_alpha_per_rad']:.3f} -> {point['sdk']['cm_alpha_per_rad']:.3f} "
              f"({point['error_percent']['cm_alpha']:+.2f}%), "
              f"XCP {point['reference']['xcp_cal']:.4f} -> {point['sdk']['xcp_cal']:.4f} "
              f"({point['error_percent']['xcp']:+.2f}%)")
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
