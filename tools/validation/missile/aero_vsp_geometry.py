"""Strict saved-geometry readers derived from Claude's panel-closure delivery.

See native/numerics/PROVENANCE.json; no repair, welding or filtering.
"""
import hashlib
import math
from pathlib import Path


def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest()


def _ints(text, count=None, what="integers"):
    try:
        values = [int(token) for token in text.split()]
    except ValueError as error:
        raise ValueError("malformed %s: %r" % (what, text)) from error
    if count is not None and len(values) != count:
        raise ValueError("expected %d %s, found %r" % (count, what, text))
    return values


def _floats(tokens, what):
    try:
        values = [float(token) for token in tokens]
    except ValueError as error:
        raise ValueError("malformed %s: %r" % (what, tokens)) from error
    if not all(math.isfinite(value) for value in values):
        raise ValueError("nonfinite %s: %r" % (what, tokens))
    return values


def read_vspgeom(path):
    """Parse a one-level VSPGEOM v3 file in the order the 3.55.0 VSPAERO reader consumes it."""
    raw = Path(path).read_bytes()
    lines = raw.decode("ascii").splitlines()
    position = 0

    def line():
        nonlocal position
        if position >= len(lines):
            raise ValueError("truncated VSPGEOM after line %d" % position)
        position += 1
        return lines[position - 1]
    if line().strip() != "# vspgeom v3":
        raise ValueError("not a VSPGEOM v3 file")
    if _ints(line(), 1, "refinement levels") != [1]:
        raise ValueError("expected exactly one refinement level")
    node_count, loop_count, wake_lists = _ints(line(), 3, "header counts")
    if node_count < 4 or loop_count < 1 or wake_lists < 0:
        raise ValueError("invalid VSPGEOM header counts")
    points = [tuple(_floats(line().split(), "node coordinates")) for _ in range(node_count)]
    if any(len(point) != 3 for point in points):
        raise ValueError("a node record does not hold three coordinates")
    if _ints(line(), 1, "loop count") != [loop_count]:
        raise ValueError("loop counts disagree")
    loops = []
    for number in range(loop_count):
        row = _ints(line(), None, "loop connectivity")
        loop = tuple(value - 1 for value in row[1:])
        if not row or row[0] < 3 or len(loop) != row[0] or len(set(loop)) != len(loop) or \
                any(not 0 <= value < node_count for value in loop):
            raise ValueError("invalid connectivity of loop %d" % (number + 1))
        loops.append(loop)
    surfaces, subsurfaces = [], []
    for number, loop in enumerate(loops):
        tokens = line().split()
        if len(tokens) != 2 + 2 * len(loop):
            raise ValueError("loop %d surface/UV record has %d tokens" % (number + 1, len(tokens)))
        surfaces.append(_ints(tokens[0], 1, "surface id")[0])
        subsurfaces.append(_ints(tokens[1], 1, "sub-surface id")[0])
        _floats(tokens[2:], "loop UV")
    parents = [tuple(_ints(line(), 2, "parent record")) for _ in range(loop_count)]
    kutta_count = _ints(line(), 1, "Kutta list count")[0]
    kutta = []
    for number in range(kutta_count):
        row = _ints(line(), None, "Kutta list")
        if len(row) < 2 or len(row) != 2 + abs(row[0]):
            raise ValueError("Kutta list %d is malformed: %r" % (number + 1, row))
        nodes = [value - 1 for value in row[2:]]
        if any(not 0 <= value < node_count for value in nodes):
            raise ValueError("Kutta list %d references a missing node" % (number + 1))
        kutta.append({"count": abs(row[0]), "body_wake": row[0] < 0, "part": row[1], "nodes": nodes})
    alternates = []
    for number, loop in enumerate(loops):
        row = _ints(line(), None, "alternate triangulation")
        if len(row) < 2 or row[0] != number + 1 or row[1] < 1 or len(row) != 2 + 3 * row[1]:
            raise ValueError("alternate triangulation record %d is malformed" % (number + 1))
        triangles = [tuple(value - 1 for value in row[2 + 3 * k: 5 + 3 * k]) for k in range(row[1])]
        if any(not 0 <= value < node_count for triangle in triangles for value in triangle):
            raise ValueError("alternate triangle of loop %d references a missing node" % (number + 1))
        alternates.append(triangles)
    for number, triangles in enumerate(alternates):
        tokens = line().split()
        if len(tokens) != 3 + 6 * len(triangles) or _ints(tokens[0], 1, "UV index") != [number + 1] or \
                _ints(tokens[1], 1, "UV surface")[0] != surfaces[number]:
            raise ValueError("trailing triangle UV record %d is malformed" % (number + 1))
        _floats(tokens[3:], "triangle UV")
    if position != len(lines):
        raise ValueError("unexpected data after the trailing triangle UV records (line %d)" % (position + 1))
    return {"sha256": sha256_bytes(raw), "bytes": len(raw), "points": points, "loops": loops, "surfaces": surfaces,
            "subsurfaces": subsurfaces, "parents": parents, "header_wake_lists": wake_lists, "kutta": kutta,
            "alternates": alternates}


def read_vkey(path):
    """Parse a VSPGEOM v3 tag key file; line 2 (a provenance path) is recorded, not interpreted."""
    raw = Path(path).read_bytes()
    lines = raw.decode("ascii").splitlines()
    if len(lines) < 5 or lines[0].strip() != "# VSPGEOM v3 Tag Key File":
        raise ValueError("not a VSPGEOM v3 tag key file")
    count = _ints(lines[2], 1, "part count")[0]
    if lines[3].strip() != "" or not lines[4].startswith("# part#,geom#,surf#,gname,gid,thick,plate,copy#,geomcopy#"):
        raise ValueError("unexpected vkey part header")
    parts = []
    for text in lines[5:5 + count]:
        fields = text.split(",")
        if len(fields) != 9:
            raise ValueError("malformed vkey part row %r" % text)
        parts.append({"part": int(fields[0]), "geom": int(fields[1]), "surf": int(fields[2]), "gname": fields[3],
                      "gid": fields[4], "thick": int(fields[5]), "plate": int(fields[6]), "copy": int(fields[7]),
                      "geomcopy": int(fields[8])})
    rest = lines[5 + count:]
    if len(rest) < 4 or rest[0].strip() != "" or rest[2].strip() != "" or not rest[3].startswith("# tag#,part#"):
        raise ValueError("unexpected vkey tag header")
    tag_count = _ints(rest[1], 1, "tag count")[0]
    tags = [tuple(int(value) for value in text.split(",")) for text in rest[4:4 + tag_count]]
    if len(tags) != tag_count or rest[4 + tag_count:] and any(text.strip() for text in rest[4 + tag_count:]):
        raise ValueError("vkey tag rows are incomplete or followed by data")
    return {"sha256": sha256_bytes(raw), "provenance_line": lines[1], "parts": parts, "tags": tags}
