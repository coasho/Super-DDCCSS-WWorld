"""sea_mask_tiles.py <land-polygons.shp> <terrarium dir> <backup dir> [--bbox lon0,lat0,lon1,lat1] [--workers n] [--dry-run]

Burns the coastline into the map set's Terrarium elevation tiles, so the viewer knows the sea from the land exactly
(world::setSeaFloorDepth draws the sea's floor below sea level and the sea over it; the land at the shore hides it).

The tiles as tools/tile_prefetch stores them hold whole metres with the sea at 0 - and so does land at or below sea
level (the Dutch polders, salt flats and marshes, a beach rounded down): every such texel looked like sea, and the
sea flooded it. OpenStreetMap's land polygons (osmdata.openstreetmap.de, land-polygons-split-4326; (c) OpenStreetMap
contributors, ODbL) say which texels are land. In each tile:

  - land at or below 0 m is raised to 1/256 m (Terrarium's third byte, which the stored tiles leave at 0): land, and
    no higher than it was to the metre;
  - sea above 0 m but below 5 m is lowered to 0 (the coastline is the polygons', not the elevation model's: its
    beaches and breakwaters smeared a few texels out to sea); higher than that is left - a cliff the two disagree on
    by a texel is not cut down;
  - everything else is left as it is.

Tiles are rewritten only where something changes, in the stored form (RGB, zlib at its best), the original copied
to the backup directory first (once: running again leaves the first original there).
"""
import argparse
import math
import os
import shutil
import sys
import time
from multiprocessing import Pool

import numpy as np
from PIL import Image, ImageDraw

from shapefile_polygons import Polygons, ring_area

MAX_LAT = 85.0511287798

_polygons = None
_args = None


def tile_bounds(z, x, y):
    """lon0, lat_south, lon1, lat_north of a Web Mercator tile"""
    n = 2 ** z
    lon0, lon1 = x / n * 360.0 - 180.0, (x + 1) / n * 360.0 - 180.0
    lat = lambda v: math.degrees(math.atan(math.sinh(math.pi * (1.0 - 2.0 * v))))
    return lon0, lat((y + 1) / n), lon1, lat(y / n)


def land_mask(z, x, y, size):
    """1 where the tile's texels (centres) are on land, by the polygons"""
    lon0, lat_s, lon1, lat_n = tile_bounds(z, x, y)
    margin = (lon1 - lon0) * 0.01
    b = _polygons.boxes
    hit = np.nonzero((b[:, 0] <= lon1 + margin) & (b[:, 2] >= lon0 - margin) & (b[:, 1] <= lat_n + margin) & (b[:, 3] >= lat_s - margin))[0]
    image = Image.new("L", (size, size), 0)
    if len(hit) == 0:
        return np.zeros((size, size), dtype=bool)
    draw = ImageDraw.Draw(image)
    scale = (2 ** z) * size
    for i in hit:
        for ring in _polygons.rings(int(i)):
            lon = ring[:, 0]
            lat = np.clip(ring[:, 1], -MAX_LAT, MAX_LAT)
            u = ((lon + 180.0) / 360.0) * scale - x * size
            phi = np.radians(lat)
            v = (1.0 - np.log(np.tan(phi) + 1.0 / np.cos(phi)) / math.pi) / 2.0 * scale - y * size
            # to a quarter of a texel, the repeats dropped (a level-0 tile would draw every vertex of every coast)
            q = np.round(np.stack([u, v], axis=1) * 4.0) / 4.0 - 0.5
            keep = np.ones(len(q), dtype=bool)
            keep[1:] = np.any(q[1:] != q[:-1], axis=1)
            q = q[keep]
            if len(q) < 3:
                continue
            outer = ring_area(ring) < 0.0  # (shapefiles: outer rings clockwise)
            draw.polygon(q.ravel().tolist(), fill=1 if outer else 0)
    return np.asarray(image, dtype=np.uint8) > 0


def init(shp, args):
    global _polygons, _args
    _polygons = Polygons(shp)
    _args = args


def process(path):
    rel = os.path.relpath(path, _args.tiles)
    parts = rel.replace("\\", "/").split("/")
    z, x, y = int(parts[0]), int(parts[1]), int(os.path.splitext(parts[2])[0])
    im = Image.open(path)
    mode = im.mode
    rgb = np.array(im.convert("RGB"), dtype=np.uint8)
    size = rgb.shape[0]
    if rgb.shape[1] != size:
        return rel, 0, 0, "not square"
    h = rgb[..., 0].astype(np.float64) * 256.0 + rgb[..., 1] + rgb[..., 2] / 256.0 - 32768.0
    land = land_mask(z, x, y, size)
    raise_land = land & (h <= 0.0)
    lower_sea = (~land) & (h > 0.0) & (h < 5.0)
    changed = int(raise_land.sum() + lower_sea.sum())
    if changed and not _args.dry_run:
        out = rgb.copy()
        out[raise_land] = (128, 0, 1)
        out[lower_sea] = (128, 0, 0)
        backup = os.path.join(_args.backup, rel)
        if not os.path.exists(backup):
            os.makedirs(os.path.dirname(backup), exist_ok=True)
            shutil.copy2(path, backup)
        tmp = path + ".tmp"
        Image.fromarray(out, "RGB").save(tmp, format="PNG", compress_level=9)
        os.replace(tmp, path)
    return rel, int(raise_land.sum()), int(lower_sea.sum()), mode


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("shp")
    ap.add_argument("tiles")
    ap.add_argument("backup")
    ap.add_argument("--bbox", default=None)
    ap.add_argument("--workers", type=int, default=12)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    box = [float(v) for v in args.bbox.split(",")] if args.bbox else None
    paths = []
    for zdir in sorted(os.listdir(args.tiles), key=lambda s: int(s) if s.isdigit() else -1):
        if not zdir.isdigit():
            continue
        z = int(zdir)
        for xdir in os.listdir(os.path.join(args.tiles, zdir)):
            for f in os.listdir(os.path.join(args.tiles, zdir, xdir)):
                if not f.endswith(".png"):
                    continue
                if box:
                    lon0, lat_s, lon1, lat_n = tile_bounds(z, int(xdir), int(f[:-4]))
                    if lon1 < box[0] or lon0 > box[2] or lat_n < box[1] or lat_s > box[3]:
                        continue
                paths.append(os.path.join(args.tiles, zdir, xdir, f))
    print(f"{len(paths)} tiles", flush=True)
    t0 = time.time()
    raised = lowered = touched = 0
    with Pool(args.workers, initializer=init, initargs=(args.shp, args)) as pool:
        for k, (rel, r, l, mode) in enumerate(pool.imap_unordered(process, paths, chunksize=8)):
            raised += r
            lowered += l
            touched += 1 if r + l else 0
            if (k + 1) % 2000 == 0:
                print(f"  {k + 1}/{len(paths)}  {time.time() - t0:.0f} s  tiles changed {touched}", flush=True)
    print(f"done in {time.time() - t0:.0f} s: {touched} tiles changed; land raised off the sea {raised} texels, sea lowered to it {lowered}")


if __name__ == "__main__":
    sys.exit(main())
