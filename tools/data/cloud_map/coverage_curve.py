# How much of the sky takram's clouds cover for each value of their `coverage`: the fraction of columns, over one
# local-weather tile, whose optical depth through layers r and g (their default heights) exceeds 1 - takram's own
# sampleWeather / sampleMedia (clouds.glsl), shape and detail at mip 0, no turbulence or evolution. Its output is the
# curve world::Clouds::coverageForCover and assets/clouds/shaders/fsim/coverage.glsl turn a cloud cover with.
# Needs numpy and Pillow.
import pathlib

import numpy as np
from PIL import Image

A = str(pathlib.Path(__file__).resolve().parents[3] / "assets" / "clouds") + "/"
weather = np.asarray(Image.open(A + "local_weather.png").convert("RGBA"), dtype=np.float32)[::-1] / 255.0  # flipY
shape = np.fromfile(A + "shape.bin", dtype=np.uint8).reshape(128, 128, 128).astype(np.float32) / 255.0  # [z][y][x]
detail = np.fromfile(A + "shape_detail.bin", dtype=np.uint8).reshape(32, 32, 32).astype(np.float32) / 255.0

TILE = 100000.0  # m per weather repeat (a cube face is ~10,000 km, repeated 100 times)
N = 256
xs = (np.arange(N) + 0.5) / N * TILE
X, Y = np.meshgrid(xs, xs, indexing="xy")
layers = [  # channel, altitude, height, densityScale, shapeAmount, detailAmount, bias, filterWidth
    (0, 750.0, 650.0, 0.2, 1.0, 1.0, 0.35, 0.6),
    (1, 1000.0, 1200.0, 0.2, 1.0, 1.0, 0.35, 0.6),
]
heights = np.arange(750.0, 2200.0, 20.0) + 10.0


def rc(x, a, b):
    return np.clip((x - a) / (b - a), 0.0, 1.0)


wu = (X / TILE * 512).astype(int) % 512
wv = (Y / TILE * 512).astype(int) % 512


def fraction(cov):
    tau = np.zeros_like(X)
    for h in heights:
        sx = (X * 0.0003 * 128).astype(int) % 128
        sy = (Y * 0.0003 * 128).astype(int) % 128
        sz = int(h * 0.0003 * 128) % 128
        s = shape[sz, sy, sx]
        dx = (X * 0.006 * 32).astype(int) % 32
        dy = (Y * 0.006 * 32).astype(int) % 32
        dz = int(h * 0.006 * 32) % 32
        d = detail[dz, dy, dx]
        total = np.zeros_like(X)
        for ch, alt, hgt, ds, sa, da, bias, fw in layers:
            hf = np.clip((h - alt) / hgt, 0.0, 1.0)
            inside = (h > alt) & (h < alt + hgt)
            if not inside:
                continue
            lw = weather[wv, wu, ch]
            xb = np.clip(hf ** bias * 2.0 - 1.0, -1.0, 1.0)
            hs = 1.0 - xb * xb
            f = 1.0 - cov * hs
            dens = rc(lw * (1 - fw) + fw, f, f + fw)
            dens = rc(dens, (1.0 - s) * sa, 1.0)
            m = (d ** 6) * (1 - rc(hf, 0.2, 0.4)) + (1 - d) * rc(hf, 0.2, 0.4)
            m = m * da
            dens = rc(dens * 2.0, m * 0.5, 1.0)
            dens = np.clip(dens * ds * (0.75 * hf + 0.25), 0, 1)
            total += dens
        tau += total * 20.0
    return float(np.mean(tau > 1.0))


for c in [0.15 + 0.025 * i for i in range(19)]:
    print(f"coverage {c:.3f}: sky covered {fraction(c):.3f}")
