"""Check whether DREV Appendix B control-fin geometry closes the AIM-7M gap.

This is a sensitivity check, not a production correction. It copies the
authored AIM-7M design to a scratch path, replaces only the forward control
fin with the Figure B-4 dimensions converted at 1:7, and compares the shared
VLM coefficients. The tail-fin drawing is not used because its dimension
chain is less clear.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from hangar import airframe

ROOT = Path(__file__).resolve().parents[3]


def replace_control_fin(text: str) -> str:
    # Figure B-4: root chord 2.616 in, tip 0.360 in, semispan 2.305 in.
    # 1:7 conversion: 0.465 m, 0.064 m, 0.410 m. Tip radial station is the
    # body radius plus the exposed semispan; y=z at the 45 deg X orientation.
    text = text.replace("chord = 0.62", "chord = 0.465")
    text = text.replace("chord = 0.1", "chord = 0.064")
    text = text.replace("le = [1.82, 0.3536, 0.3536]", "le = [1.701, 0.3595, 0.3595]")
    text = text.replace("le = [1.82, 0.3536, -0.3536]", "le = [1.701, 0.3595, -0.3595]")
    return text


def coefficients(path: Path) -> dict:
    values = airframe.coefficients(str(path))
    return {
        "mach": values["mach"],
        "cna": values["cna"],
        "cma": values["cma"],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--out",
        type=Path,
        default=ROOT / "build/architecture-reviews/sim-m02-sources/aim7m-geometry-sensitivity.json",
    )
    parser.add_argument(
        "--scratch",
        type=Path,
        default=ROOT / "build/architecture-reviews/sim-m02-sources/aim7m-geometry-scratch",
    )
    args = parser.parse_args()

    source = ROOT / "aircraft/aim7m/aim7m.toml"
    baseline = coefficients(source)
    args.scratch.mkdir(parents=True, exist_ok=True)
    candidate = args.scratch / "aim7m-drev-control-only.toml"
    candidate.write_text(replace_control_fin(source.read_text(encoding="utf-8")), encoding="utf-8")
    variant = coefficients(candidate)

    result = {
        "schema": "fsim.aim7m-geometry-sensitivity.v1",
        "source": {
            "report": "DREV-TM-1999-050",
            "locator": "Figure B-4, model control fin, dimensions in inches",
            "local_page": "build/architecture-reviews/sim-m02-sources/aim7m-page47.jpg",
            "scale": "1:7",
            "figure_values_in": {
                "root_chord": 2.616,
                "tip_chord": 0.360,
                "exposed_semispan": 2.305,
            },
            "converted_m": {
                "root_chord": 0.465,
                "tip_chord": 0.064,
                "exposed_semispan": 0.410,
            },
        },
        "baseline": baseline,
        "drev_control_fin": variant,
        "note": "Shared VLM coefficients, not the SDK static fixture. The variant only tests Figure B-4 control-fin geometry; the Figure B-5 tail-fin dimensions are not used. A negative result means geometry alone does not close the DREV gap.",
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print("Mach  baseline_cna  variant_cna  baseline_cma  variant_cma")
    for i, mach in enumerate(baseline["mach"]):
        if mach not in (2.0, 3.0):
            continue
        print(f"{mach:4.1f} {baseline['cna'][i]:13.6f} {variant['cna'][i]:12.6f} "
              f"{baseline['cma'][i]:13.6f} {variant['cma'][i]:12.6f}")
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
