# measure.py: a rocket plume's numbers in an image - the same for a launch photograph and for a render.
#
#   python tools/validation/plume_check/measure.py image nozzle_x nozzle_y nose_x nose_y diameter_m length_m [background_image]
#
# Along the missile's axis aft of its nozzle (in missile lengths L), across +-0.3 L of it: the plume's peak luminance
# (linear Y) and its contrast against the sky beside it - for a render, against its own frame drawn without effects,
# pixel by pixel. Reported:
#   sat   where it is saturated (Y >= 0.93): from where, unbroken to where (L), and how wide (body diameters) at 0.1,
#         0.5 and 1 L
#   gap   the least contrast between 0.05 and 0.5 L over the most between 0.05 and 1 L (1: none; 0: dark behind the
#         nozzle)
#   c     its contrast at 0.1, 0.3, 0.5, 1 and 2 L
#   edge  the colour (r/g, b/g of linear values) of its unsaturated edge, 0.2 - 0.8 L aft
# A photograph's exposure is its camera's: compare where it is saturated and how its light falls off, not its contrast
# one for one (a render's sky, drawn brighter, leaves a plume less of it before it saturates).
import sys

import numpy as np
from PIL import Image

SAT = 0.93


def _linear(path):
    a = np.asarray(Image.open(path).convert("RGB")).astype(float) / 255.0
    return np.where(a <= 0.04045, a / 12.92, ((a + 0.055) / 1.055) ** 2.4)


def measure(path, nozzle, nose, diameter, length, background=None):
    lin = _linear(path)
    Y = lin @ np.array([0.2126, 0.7152, 0.0722])
    YB = _linear(background) @ np.array([0.2126, 0.7152, 0.0722]) if background else None
    H, W = Y.shape
    (nx, ny), (tx, ty) = nozzle, nose
    aft = np.array([nx - tx, ny - ty], float)
    Lpx = float(np.hypot(*aft))
    aft /= Lpx
    across = np.array([-aft[1], aft[0]])
    dpx = Lpx * diameter / length
    s = np.linspace(-0.6, 0.6, 241) * Lpx

    def at(img, xs, ys):
        return img[np.clip(np.round(ys).astype(int), 0, H - 1), np.clip(np.round(xs).astype(int), 0, W - 1)]

    rows = []
    for t in np.arange(0.0, 3.001, 0.02):
        cx, cy = nx + aft[0] * t * Lpx, ny + aft[1] * t * Lpx
        xs, ys = cx + across[0] * s, cy + across[1] * s
        if not ((xs >= 0) & (xs < W) & (ys >= 0) & (ys < H)).all():
            break
        y = at(Y, xs, ys)
        near = np.abs(s) < 0.15 * Lpx
        b = at(YB, xs, ys) if YB is not None else np.full_like(y, np.median(y[np.abs(s) > 0.3 * Lpx]))
        rel = (y - b) / np.maximum(b, 1e-4)
        k = int(np.argmax(np.where(near, rel, -1e9)))
        sat = (y >= SAT) & (np.abs(s) < 0.3 * Lpx)
        edge = (rel > 0.1 * max(rel[k], 1e-3)) & (y < 0.9) & (np.abs(s) < 0.3 * Lpx)
        rgb = at(lin, xs[edge], ys[edge]) if edge.any() else np.zeros((0, 3))
        rows.append((round(float(t), 2), float(rel[k]), float(sat.sum()) * (s[1] - s[0]) / dpx, rgb))

    ts = np.array([r[0] for r in rows])
    cons = np.array([r[1] for r in rows])
    satw = np.array([r[2] for r in rows])
    satd = satw > 0.3
    start = end = None
    if satd.any():
        i = int(np.argmax(satd))
        start = float(ts[i])
        while i + 1 < len(ts) and satd[i + 1]:
            i += 1
        end = float(ts[i])
    inner = (ts >= 0.05) & (ts <= 0.5)
    pick = lambda arr, x: float(arr[np.argmin(np.abs(ts - x))]) if len(ts) and ts[-1] >= x - 0.01 else float("nan")
    e = [r[3] for r in rows if 0.2 <= r[0] <= 0.8 and len(r[3])]
    e = np.concatenate(e) if e else np.full((1, 3), 1e-4)
    g = max(e[:, 1].mean(), 1e-4)
    return {
        "sat_from": start, "sat_to": end,
        "width": [pick(satw, 0.1), pick(satw, 0.5), pick(satw, 1.0)],
        "gap": float(cons[inner].min() / max(cons[(ts >= 0.05) & (ts <= 1.0)].max(), 1e-3)),  # (aft of the nozzle itself)
        "contrast": [pick(cons, x) for x in (0.1, 0.3, 0.5, 1.0, 2.0)],
        "edge": [float(e[:, 0].mean() / g), float(e[:, 2].mean() / g)],
    }


def line(name, m):
    f = lambda v: "-" if v is None else "%.2f" % v
    return ("%-24s sat %s..%s L, width %.1f/%.1f/%.1f d at 0.1/0.5/1 L | gap %.2f | c %s | edge r/g %.2f b/g %.2f"
            % (name[:24], f(m["sat_from"]), f(m["sat_to"]), *m["width"], m["gap"],
               " ".join("%.2f" % c for c in m["contrast"]), *m["edge"]))


if __name__ == "__main__":
    a = sys.argv[1:]
    m = measure(a[0], (float(a[1]), float(a[2])), (float(a[3]), float(a[4])), float(a[5]), float(a[6]), a[7] if len(a) > 7 else None)
    print(line(a[0].replace("\\", "/").split("/")[-1], m))
