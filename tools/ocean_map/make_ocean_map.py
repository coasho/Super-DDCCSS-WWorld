"""make_ocean_map.py <GEBCO_YYYY.nc> <GEBCO_YYYY_TID.nc> <out.png> [arc-minutes, 2]: the viewer's sea map (world::Clouds draws the sea)
from GEBCO's global grid - its elevations and its type identifiers (TID 0: land; anything else: a depth, measured or
modelled, so sea, inland seas and the lakes it has bathymetry for).

The map is equirectangular, `arc-minutes` a texel (2: 10800 x 5400), its rows from the north (89.97 deg) to the south, its
columns from 180 W; a two-channel 8-bit PNG (grey + alpha):
  - grey: the depth of the water in the texel (the mean over its GEBCO cells that are water; 0 m where none are), as
    255 * ln(1 + depth / 1 m) / ln(1 + 11000): 1 m 19, 10 m 65, 100 m 126, 1000 m 189, 11000 m 255 - fine where the
    water's colour and its waves change (its first hundred metres), coarse in the abyss where neither does;
  - alpha: the share of the texel's GEBCO cells that are water (TID != 0), 0 .. 255.

GEBCO Compilation Group (2026) GEBCO 2026 Grid; the GEBCO Grid is in the public domain.
"""
import math
import sys
import zlib
import struct

import numpy as np

from netcdf_classic import NetCDF

MAX_DEPTH = 11000.0


def png_grey_alpha(path, image):
    """A 16-bit-free, two-channel PNG (colour type 4, 8 bits), rows as given."""
    h, w, _ = image.shape
    raw = b"".join(b"\x00" + image[y].tobytes() for y in range(h))
    def chunk(kind, data):
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF)
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 4, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(raw, 9)))
        f.write(chunk(b"IEND", b""))


def open_grid(path, name):
    """A GEBCO grid variable, whichever netCDF it comes in: classic (memory-mapped here) or netCDF-4 (HDF5, by h5py)."""
    with open(path, "rb") as f:
        magic = f.read(4)
    if magic[:3] == b"CDF":
        return NetCDF(path).variable(name)
    import h5py
    return h5py.File(path, "r")[name]


def main():
    elevation_path, tid_path, out = sys.argv[1], sys.argv[2], sys.argv[3]
    minutes = float(sys.argv[4]) if len(sys.argv) > 4 else 2.0
    elevation = open_grid(elevation_path, "elevation")  # (lat, lon), south first
    tid = open_grid(tid_path, "tid")
    rows, cols = elevation.shape
    assert tid.shape == elevation.shape, (tid.shape, elevation.shape)
    cells = int(round(minutes * 60.0 / 15.0))  # GEBCO's cells (15 arc-seconds) per texel each way
    h, w = rows // cells, cols // cells
    out_image = np.zeros((h, w, 2), np.uint8)
    scale = 255.0 / math.log1p(MAX_DEPTH)
    for r in range(h):
        y0 = rows - (r + 1) * cells  # (the map's rows from the north; GEBCO's from the south)
        e = np.asarray(elevation[y0:y0 + cells, :], dtype=np.float32).reshape(cells, w, cells)
        t = np.asarray(tid[y0:y0 + cells, :]).reshape(cells, w, cells)
        water = t != 0
        n = water.sum(axis=(0, 2))
        depth = np.where(water, np.maximum(-e, 0.0), 0.0).sum(axis=(0, 2)) / np.maximum(n, 1)
        out_image[r, :, 0] = np.clip(np.round(np.log1p(np.minimum(depth, MAX_DEPTH)) * scale), 0, 255).astype(np.uint8)
        out_image[r, :, 1] = np.clip(np.round(n * 255.0 / (cells * cells)), 0, 255).astype(np.uint8)
        if r % 200 == 0:
            print("row %d of %d" % (r, h), flush=True)
    png_grey_alpha(out, out_image)
    print("%s: %d x %d, %.1f arc-minutes a texel; water in %.1f %% of the texels" % (out, w, h, minutes, 100.0 * (out_image[..., 1] > 127).mean()))


if __name__ == "__main__":
    main()
