# check.py: the viewer's rocket plumes against launch photographs (photos.json, measured by measure.py).
#
#   python tools/plume_check/check.py [--altitude 6000] [--at 7.0]
#
# For an aluminised motor (AIM-7M) and two reduced-smoke ones (AIM-120C, AIM-9M): the showcase's launch recorded
# (fsim hangar showcase), the missile drawn side-on from 30 m a second into its boost, fullscreen and without the
# interface, once with its effects and once without; its plume measured as the photographs were, and held to them:
#   aluminised - saturated from the nozzle (within 0.1 L), to where the photographs' are (1.6 - 2.3 L, 0.2 L either
#                way), as wide at 1 L (3.2 - 4.1 diameters, 0.5 either way), no dark gap behind the nozzle (gap >= 0.5);
#   reduced    - bright near the nozzle (contrast >= 0.25 at 0.1 L), faded at 1 L to the photograph's light against its sky
#                (1.23 times it, 0.12 either way), no dark gap (gap >= 0.2).
# Needs the viewer built (build/ucrt64-release) and a display: it is a check run by hand, not a ctest. Writes its frames
# to build/plume_check.
import argparse
import json
import pathlib
import subprocess
import sys

import numpy as np
from PIL import Image

here = pathlib.Path(__file__).resolve().parent
root = here.parents[1]
sys.path.insert(0, str(here))
from measure import line, measure  # noqa: E402

VIEWER = root / "build" / "ucrt64-release" / "bin" / "flightsim-viewer.exe"
MISSILES = [("aim7m", "aluminised", 0.203, 3.66), ("aim120c", "reduced", 0.178, 3.66), ("aim9m", "reduced", 0.127, 2.87)]


def render(rec, out, at, effects):
    args = [str(VIEWER), "--replay", str(rec), "--fullscreen", "--no-gui", "--camera", "chase", "--follow", "1",
            "--chase-distance", "30", "--chase-azimuth", "270", "--chase-elevation", "0", "--sun-utc", "20",
            "--screenshot", str(out), "--screenshot-after", str(at)]
    if not effects:
        args.append("--no-effects")
    subprocess.run(args, check=True, timeout=180, capture_output=True)


def ends(background):
    """The missile's nose and nozzle along the screen's middle row (it flies right to left, centred): the grey body's
    first and last pixels there, in the frame without effects."""
    a = np.asarray(Image.open(background).convert("RGB")).astype(int)
    y = a.shape[0] // 2
    row = a[y]
    grey = (row[:, 2] - row[:, 0] < 12) & (row[:, 0] < 215)  # (not the sky, the one thing there blue: its body and its bands)
    # (the run through the screen's middle, where the chase camera holds it - gaps of a few pixels bridged - not
    # whatever else is grey along the row: its carrier behind it)
    mid = a.shape[1] // 2
    lo = hi = mid
    while lo > 0 and grey[max(lo - 4, 0):lo].any():
        lo -= 1
    while hi < a.shape[1] - 1 and grey[hi + 1:hi + 5].any():
        hi += 1
    return (float(hi), float(y)), (float(lo), float(y))


def same_view(lit, bare, nose):
    """Whether two frames show the same view: the sky ahead of the missile's nose alike (mean difference of a patch)."""
    a = np.asarray(Image.open(lit).convert("RGB")).astype(float)
    b = np.asarray(Image.open(bare).convert("RGB")).astype(float)
    x, y = int(nose[0]), int(nose[1])
    patch = (slice(max(y - 60, 0), y + 60), slice(max(x - 260, 0), max(x - 20, 1)))
    return float(np.abs(a[patch] - b[patch]).mean()) < 6.0


def main(argv):
    p = argparse.ArgumentParser(prog="plume_check")
    p.add_argument("--altitude", type=float, default=6000.0)
    p.add_argument("--at", type=float, default=7.0, help="s into the replay (the launch is at 6 s)")
    a = p.parse_args(argv)
    photos = json.loads((here / "photos.json").read_text(encoding="utf-8"))["photos"]
    out = root / "build" / "plume_check"
    out.mkdir(parents=True, exist_ok=True)
    print("photographs:")
    for ph in photos:
        print("  " + line(ph["missile"] + " (" + ph["smoke"] + ")", ph["measured"]))
    al = [ph["measured"] for ph in photos if ph["smoke"] == "aluminised"]
    lo_to, hi_to = min(m["sat_to"] for m in al) - 0.2, max(m["sat_to"] for m in al) + 0.2
    lo_w, hi_w = min(m["width"][2] for m in al) - 0.5, max(m["width"][2] for m in al) + 0.5
    rs = [ph["measured"] for ph in photos if ph["smoke"] == "reduced"]
    ok = True
    print("the viewer's, at %.0f m, %.1f s:" % (a.altitude, a.at))
    for name, smoke, diameter, length in MISSILES:
        subprocess.run(["cmd", "/c", str(root / "fsim.cmd"), "hangar", "showcase", name, "--altitude", str(a.altitude),
                        "--duration", "12"], check=True, capture_output=True, cwd=root)
        rec = root / "aircraft" / name / "out" / "showcase.fsrec"
        lit, bare = out / (name + ".png"), out / (name + "_bare.png")
        render(rec, lit, a.at, True)
        nozzle, nose = ends(lit)  # (its flame is no body: white)
        # the frame without effects the same view: the sky ahead of its nose as the lit frame's (the chase camera can
        # settle otherwise in another run - then it is drawn again)
        for _ in range(3):
            render(rec, bare, a.at, False)
            if same_view(lit, bare, nose):
                break
        else:
            raise SystemExit("%s: the frame without effects is not the same view" % name)
        m = measure(str(lit), nozzle, nose, diameter, length, str(bare))
        if smoke == "aluminised":
            fails = [why for why, bad in (
                ("not saturated from the nozzle", m["sat_from"] is None or m["sat_from"] > 0.1),
                ("saturated to %s L, photographs %.2f - %.2f" % (m["sat_to"], lo_to + 0.2, hi_to - 0.2),
                 m["sat_to"] is None or not lo_to <= m["sat_to"] <= hi_to),
                ("%.1f d wide at 1 L, photographs %.1f - %.1f" % (m["width"][2], lo_w + 0.5, hi_w - 0.5), not lo_w <= m["width"][2] <= hi_w),
                ("a dark gap behind the nozzle", m["gap"] < 0.5)) if bad]
        else:
            # (its light against the sky beside it, as the photograph's: a ratio, which the cameras' exposures leave)
            c, faded = m["contrast"], rs[0]["contrast"][3] + 0.12
            fails = [why for why, bad in (
                ("dim at the nozzle (%.2f at 0.1 L)" % c[0], c[0] < 0.25),
                ("%.2f times the sky at 1 L, the photograph %.2f" % (1 + c[3], 1 + rs[0]["contrast"][3]), c[3] > faded),
                ("a dark gap behind the nozzle", m["gap"] < 0.2)) if bad]
        ok &= not fails
        print("  " + line("%s (%s)" % (name, smoke), m) + ("  ok" if not fails else "  FAILS: " + "; ".join(fails)))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
