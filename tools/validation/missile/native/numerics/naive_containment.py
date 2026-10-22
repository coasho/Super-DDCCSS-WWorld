"""NEGATIVE CONTROL ONLY: the coordinate-matching check, used to show that test_wake_containment.py fails it (RED).

Like the accepted aero_vsp_adb point check, it verifies only that each internal binary64 wake point rounds to its
packed binary32 point. It never asks where the wake lies relative to the body, so every consistent wake passes.
It is never used for evidence.
"""
from pathlib import Path
import struct
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import aero_wake_containment as real  # noqa: E402  (only its ADB reader, which checks the binary64/binary32 match)

geometry_reader = real.geometry_reader


def assess(run):
    adb = real.read_adb(Path(run) / "reference_body.adb")
    trails = [{"trail": record["index"] + 1, "strength": 1.0, "concave": record["count"] == 1, "active": record["count"] != 1}
              for record in adb["records"]]
    return {"passed": True, "failures": [], "disabled_not_outside": [], "trail_details": trails}
