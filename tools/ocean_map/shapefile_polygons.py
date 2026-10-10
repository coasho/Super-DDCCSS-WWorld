"""A reader for ESRI shapefile polygons (shape type 5), enough for OpenStreetMap's land polygons: each record's
bounding box and its rings as (n, 2) float64 arrays of lon, lat. Memory-mapped, so the whole file is read only as far
as its records are asked for.

The format (ESRI Shapefile Technical Description, 1998): a 100-byte header, then records - a big-endian record
header (number, length in 16-bit words), and the content, little-endian: shape type, bounding box (4 doubles), number
of parts, number of points, the parts' first point indices, the points (x, y doubles).
"""
import struct

import numpy as np


class Polygons:
    def __init__(self, path):
        self.data = np.memmap(path, dtype=np.uint8, mode="r")
        code, = struct.unpack(">i", self.data[0:4].tobytes())
        if code != 9994:
            raise ValueError(f"{path}: not a shapefile")
        length = struct.unpack(">i", self.data[24:28].tobytes())[0] * 2
        shape_type, = struct.unpack("<i", self.data[32:36].tobytes())
        if shape_type != 5:
            raise ValueError(f"{path}: shape type {shape_type}, not polygons")
        # the records' offsets and bounding boxes, in one pass
        offsets, boxes = [], []
        at = 100
        while at + 8 <= length:
            content = struct.unpack(">i", self.data[at + 4:at + 8].tobytes())[0] * 2
            body = at + 8
            kind, = struct.unpack("<i", self.data[body:body + 4].tobytes())
            if kind == 5:
                offsets.append(body)
                boxes.append(struct.unpack("<4d", self.data[body + 4:body + 36].tobytes()))
            at = body + content
        self.offsets = np.array(offsets, dtype=np.int64)
        self.boxes = np.array(boxes, dtype=np.float64).reshape(-1, 4)  # xmin, ymin, xmax, ymax

    def __len__(self):
        return len(self.offsets)

    def rings(self, i):
        body = int(self.offsets[i])
        parts_n, points_n = struct.unpack("<2i", self.data[body + 36:body + 44].tobytes())
        parts = np.frombuffer(self.data[body + 44:body + 44 + 4 * parts_n].tobytes(), dtype="<i4")
        start = body + 44 + 4 * parts_n
        points = np.frombuffer(self.data[start:start + 16 * points_n].tobytes(), dtype="<f8").reshape(-1, 2)
        ends = list(parts[1:]) + [points_n]
        return [points[a:b] for a, b in zip(parts, ends)]


def ring_area(ring):
    """Signed area (shoelace): negative for the clockwise rings shapefiles use for outer rings."""
    x, y = ring[:, 0], ring[:, 1]
    return 0.5 * float(np.dot(x[:-1], y[1:]) - np.dot(x[1:], y[:-1]))
