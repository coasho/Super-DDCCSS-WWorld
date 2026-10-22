"""Wake/body containment of one VSPAERO 7.2.2 steady case (reference_body completion revision).

Matching exported coordinates (aero_vsp_adb.assess_adb) shows that the ADB wake equals the solver's wake. It does not
show that the wake stays outside the body. This check does, from the run folder's own files.

Inputs: reference_body.adb (pinned 7.2.2 single steady case layout), reference_body.vspgeom and reference_body.vkey.

* **Body.** The closed thick body is the set of alternate triangles of every thick part of the VSPGEOM (vkey thick
  flag). These are the exact coordinates the solver parsed.
* **Trails.** The ADB emits one wake record per Kutta node: its root node, and either the full internal trail in
  binary64 or, for a concave trail, the root only. Every trail point is also packed in binary32 on the solve level.
  Each internal binary64 point must round to its packed binary32 point, or the input is refused.
* **Circulation (source contract of convergence/strength-summary.json).**
  - Strip value: the signed sum of the binary64 solve-level circulation Gamma0 of the thin-sheet loops that own the
    TE edge between two consecutive Kutta-chain nodes (+1 when the edge runs in the loop's node order, −1 against).
  - Trail strength: the left strip value minus the right one (zero beyond the chain ends).
  - **Active**: the trail is not concave (a concave trail induces no velocity in the native edge code) and its
    strength is nonzero. Every other trail is disabled or dummy data. It is reported, never judged.
* **Coordinate precision.** A point's uncertainty δ is its binary32 rounding radius (half the binary32 spacing of each
  coordinate, combined) plus the largest binary32 rounding radius of a body vertex. The point is then:
  - **inside** when its generalized winding number exceeds 1/2 and its exact distance to the body exceeds δ;
  - **outside** when the winding number is at most 1/2 and the distance exceeds δ;
  - **uncertain** otherwise.
* **Segments.** Each segment of an active trail is tested at 65 evenly spaced points and at the midpoints between
  its consecutive intersections with the body triangles, using the same three classes.
* **Pass condition.** No point or segment sample of an active trail is inside or uncertain. Disabled trails are
  classified and listed only.

Usage:
    fsim.cmd --preset ucrt64-headless python tools/validation/missile/aero_wake_containment.py <run-folder> --out <json>
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import sys

import numpy as np

sys.dont_write_bytecode = True
import aero_vsp_geometry as geometry_reader  # noqa: E402

SEGMENT_SAMPLES = 65


class Reader:
    def __init__(self, raw):
        if not raw or len(raw) > 64 * 1024 * 1024:
            raise ValueError("empty or oversized ADB")
        self.raw, self.offset = raw, 0

    def take(self, fmt, count=1):
        if not 0 <= count <= 1_000_000:
            raise ValueError("invalid ADB record count")
        size = struct.calcsize("<" + fmt)
        if self.offset + size * count > len(self.raw):
            raise ValueError("truncated ADB while reading %s" % fmt)
        rows = [struct.unpack_from("<" + fmt, self.raw, self.offset + size * k) for k in range(count)]
        if any(isinstance(value, float) and not math.isfinite(value) for row in rows for value in row):
            raise ValueError("nonfinite ADB value")
        self.offset += size * count
        return rows

    def one(self, fmt):
        return self.take(fmt)[0]


def read_adb(path):
    """The parts of the pinned 7.2.2 single steady case layout needed here (the accepted aero_vsp_adb order)."""
    raw = Path(path).read_bytes()
    reader = Reader(raw)
    magic, model, symmetry, unsteady, loops, nodes, triangles, edges = reader.one("8i")
    if (magic, model, symmetry, unsteady) != (-123789453, 2, 0, 0):
        raise ValueError("unsupported ADB format/model/symmetry/time profile")
    reader.one("6f")
    surfaces = reader.one("i")[0]
    reader.take("i100si", surfaces)
    reader.take("6if", triangles)
    reader.take("3f", nodes)
    if reader.one("i")[0] != 0 or reader.one("i")[0] != 0:
        raise ValueError("rotor/nozzle ADB profiles are unsupported")
    levels = reader.one("i")[0]
    solve_xyz = None
    for level in range(1, levels + 1):
        nnode, nedge = reader.one("2i")
        xyz = reader.take("3f", nnode)
        reader.take("4i", nedge)
        if level == 1:
            solve_xyz = xyz
    reader.take("i", reader.one("i")[0])
    kutta = [row[0] for row in reader.take("i", reader.one("i")[0])]
    if reader.one("i")[0] != 0:
        raise ValueError("control-surface ADB profiles are unsupported")
    reader.one("5f")
    gamma = [row[0] for row in reader.take("2d", loops)]
    reader.take("d", 3 * edges)
    reader.take("d", 3 * loops)
    reader.take("f", 3 * triangles)
    records = []
    for index in range(reader.one("i")[0]):
        node, _span, count = reader.one("idi")
        records.append({"index": index, "node": node, "count": count, "points": [list(row) for row in reader.take("3d", count)]})
    if reader.offset != len(raw):
        raise ValueError("extra ADB bytes")
    if [record["node"] for record in records] != kutta:
        raise ValueError("wake records do not follow the native Kutta node order")
    counts = {record["count"] for record in records}
    wake_nodes = max(counts)
    if not counts <= {1, wake_nodes}:
        raise ValueError("wake records export neither the root only nor the full trail")
    n_surface = len(solve_xyz) - len(records) * wake_nodes
    for record in records:
        start = n_surface + record["index"] * wake_nodes
        record["packed"] = [list(solve_xyz[start + j]) for j in range(wake_nodes)]
        for j, point in enumerate(record["points"]):
            if [struct.unpack("<f", struct.pack("<f", value))[0] for value in point] != record["packed"][j]:
                raise ValueError("internal binary64 wake point %d of record %d does not round to its packed binary32 point"
                                 % (j, record["index"]))
    return {"sha256": hashlib.sha256(raw).hexdigest(), "loops": loops, "gamma": gamma, "records": records,
            "wake_nodes": wake_nodes, "surface_nodes": n_surface}


def f32_radius(point):
    """Half the binary32 spacing of each coordinate, combined (the rounding radius of a binary32 point)."""
    total = 0.0
    for value in point:
        magnitude = abs(float(np.float32(value)))
        spacing = float(np.spacing(np.float32(magnitude))) if magnitude > 0 else float(np.finfo(np.float32).smallest_subnormal)
        total += (0.5 * spacing) ** 2
    return math.sqrt(total)


def body(geometry, vkey):
    parts = {part["part"]: part for part in vkey["parts"]}
    tags = {tag[0]: tag[1] for tag in vkey["tags"]}
    thick = {surface for surface in set(geometry["surfaces"]) if parts[tags[surface]]["thick"] == 1}
    triangles = [tri for tris, surface in zip(geometry["alternates"], geometry["surfaces"]) if surface in thick for tri in tris]
    if not triangles:
        raise ValueError("the VSPGEOM has no thick body")
    points = np.asarray(geometry["points"], dtype=float)
    return points, np.asarray(triangles, dtype=np.int64)


def winding(points, triangles, query):
    a, b, c = (points[triangles[:, k]] - query for k in range(3))
    la, lb, lc = (np.linalg.norm(item, axis=1) for item in (a, b, c))
    numerator = np.einsum("ij,ij->i", a, np.cross(b, c))
    denominator = la * lb * lc + np.einsum("ij,ij->i", a, b) * lc + np.einsum("ij,ij->i", b, c) * la + \
        np.einsum("ij,ij->i", c, a) * lb
    return float(np.sum(2.0 * np.arctan2(numerator, denominator)) / (4.0 * math.pi))


def distance(points, triangles, query):
    """Exact Euclidean distance to a triangle set (closest point on triangle, Ericson)."""
    a, b, c = (points[triangles[:, k]] for k in range(3))
    ab, ac, ap = b - a, c - a, query - a
    d1, d2 = np.einsum("ij,ij->i", ab, ap), np.einsum("ij,ij->i", ac, ap)
    bp, cp = query - b, query - c
    d3, d4 = np.einsum("ij,ij->i", ab, bp), np.einsum("ij,ij->i", ac, bp)
    d5, d6 = np.einsum("ij,ij->i", ab, cp), np.einsum("ij,ij->i", ac, cp)
    vc, vb, va = d1 * d4 - d3 * d2, d5 * d2 - d1 * d6, d3 * d6 - d5 * d4
    closest = np.empty_like(a)
    with np.errstate(divide="ignore", invalid="ignore"):
        denominator = 1.0 / (va + vb + vc)
        closest[:] = a + ab * (vb * denominator)[:, None] + ac * (vc * denominator)[:, None]
        mask = (vc <= 0) & (d1 >= 0) & (d3 <= 0)
        closest[mask] = (a + ab * (d1 / (d1 - d3))[:, None])[mask]
        mask = (vb <= 0) & (d2 >= 0) & (d6 <= 0)
        closest[mask] = (a + ac * (d2 / (d2 - d6))[:, None])[mask]
        mask = (va <= 0) & ((d4 - d3) >= 0) & ((d5 - d6) >= 0)
        closest[mask] = (b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)))[:, None])[mask]
        for mask, vertex in (((d1 <= 0) & (d2 <= 0), a), ((d3 >= 0) & (d4 <= d3), b), ((d6 >= 0) & (d5 <= d6), c)):
            closest[mask] = vertex[mask]
    return float(np.min(np.linalg.norm(closest - query, axis=1)))


def classify(points, triangles, query, body_radius):
    query = np.asarray(query, dtype=float)
    w = winding(points, triangles, query)
    d = distance(points, triangles, query)
    delta = f32_radius(query) + body_radius
    state = "uncertain" if d <= delta else "inside" if w > 0.5 else "outside"
    return {"xyz": query.tolist(), "winding": w, "distance_m": d, "delta_m": delta, "state": state}


def signed_clearance(item):
    """Distance outside the body (positive), inside it (negative), zero when uncertain."""
    return {"outside": item["distance_m"], "inside": -item["distance_m"], "uncertain": 0.0}[item["state"]]


def crossings(points, triangles, p, q):
    """Parameters t in (0, 1) where segment p->q meets a body triangle (Moller-Trumbore, binary64)."""
    a, b, c = (points[triangles[:, k]] for k in range(3))
    d = q - p
    e1, e2 = b - a, c - a
    h = np.cross(np.broadcast_to(d, e2.shape), e2)
    det = np.einsum("ij,ij->i", e1, h)
    with np.errstate(divide="ignore", invalid="ignore"):
        inv = 1.0 / det
        s = p - a
        u = inv * np.einsum("ij,ij->i", s, h)
        qv = np.cross(s, e1)
        v = inv * (qv @ d)
        t = inv * np.einsum("ij,ij->i", e2, qv)
        hit = (det != 0) & (u >= 0) & (v >= 0) & (u + v <= 1) & (t > 0) & (t < 1)
    return sorted(set(float(value) for value in t[hit]))


def segment_samples(points, triangles, p, q):
    p, q = np.asarray(p, dtype=float), np.asarray(q, dtype=float)
    ts = [k / (SEGMENT_SAMPLES - 1) for k in range(SEGMENT_SAMPLES)]
    cuts = crossings(points, triangles, p, q)
    edges = [0.0] + cuts + [1.0]
    ts += [(lo + hi) / 2 for lo, hi in zip(edges, edges[1:]) if hi > lo]
    return sorted(set(ts)), cuts


def strengths(geometry, adb):
    """Strip values and trail strengths per the strength-summary source contract."""
    loops = geometry["loops"]
    if adb["surface_nodes"] != len(geometry["points"]):
        raise ValueError("ADB solve-level surface nodes differ from the VSPGEOM nodes")
    position = {}
    for chain_index, chain in enumerate(geometry["kutta"]):
        for k, node in enumerate(chain["nodes"]):
            if node in position:
                raise ValueError("node %d appears in two Kutta chains" % (node + 1))
            position[node] = (chain_index, k)
    strips = {}
    for chain_index, chain in enumerate(geometry["kutta"]):
        nodes = chain["nodes"]
        for k in range(len(nodes) - 1):
            a, b = nodes[k], nodes[k + 1]
            owners = []
            for index, loop in enumerate(loops):
                n = len(loop)
                for j in range(n):
                    if loop[j] == a and loop[(j + 1) % n] == b:
                        owners.append((index, 1))
                    elif loop[j] == b and loop[(j + 1) % n] == a:
                        owners.append((index, -1))
            if not owners:
                raise ValueError("TE edge %d-%d has no owning loop" % (a + 1, b + 1))
            strips[(chain_index, k)] = {"te_nodes": [a + 1, b + 1], "owners": [{"loop": index + 1, "direction": sign} for index, sign in owners],
                                        "value": math.fsum(sign * adb["gamma"][index] for index, sign in owners)}
    trails = []
    for record in adb["records"]:
        node = record["node"] - 1
        if node not in position:
            raise ValueError("wake record root node %d is not on a Kutta chain" % record["node"])
        if [float(value) for value in record["points"][0]] != [float(value) for value in geometry["points"][node]]:
            raise ValueError("wake record %d root differs from its VSPGEOM Kutta node" % record["index"])
        chain_index, k = position[node]
        left = strips[(chain_index, k - 1)]["value"] if k > 0 else 0.0
        right = strips[(chain_index, k)]["value"] if (chain_index, k) in strips else 0.0
        trails.append({"trail": record["index"] + 1, "te_node": record["node"], "chain": chain_index + 1,
                       "chain_position": k + 1, "concave": record["count"] == 1, "strip_left": left, "strip_right": right,
                       "strength": left - right})
    return strips, trails


def assess(run, basename="reference_body"):
    run = Path(run)
    geometry = geometry_reader.read_vspgeom(run / (basename + ".vspgeom"))
    vkey = geometry_reader.read_vkey(run / (basename + ".vkey"))
    adb = read_adb(run / (basename + ".adb"))
    points, triangles = body(geometry, vkey)
    body_radius = max(f32_radius(point) for point in points[np.unique(triangles)])
    strips, trails = strengths(geometry, adb)
    failures, disabled_inside = [], []
    for trail, record in zip(trails, adb["records"]):
        trail["active"] = (not trail["concave"]) and trail["strength"] != 0.0
        source = record["points"] if not trail["concave"] else record["packed"]
        trail["points_source"] = "internal binary64" if not trail["concave"] else "packed binary32 (concave: root only in binary64)"
        trail["points"] = [dict(classify(points, triangles, point, body_radius), node=j + 1) for j, point in enumerate(source)]
        trail["segments"] = []
        for j in range(len(source) - 1):
            ts, cuts = segment_samples(points, triangles, source[j], source[j + 1])
            p, q = np.asarray(source[j], dtype=float), np.asarray(source[j + 1], dtype=float)
            states = [classify(points, triangles, p + t * (q - p), body_radius) for t in ts]
            worst = min(states, key=signed_clearance)
            trail["segments"].append({"from_node": j + 1, "to_node": j + 2, "body_crossings": len(cuts), "samples": len(ts),
                                      "inside_samples": sum(item["state"] == "inside" for item in states),
                                      "uncertain_samples": sum(item["state"] == "uncertain" for item in states),
                                      "worst_sample": worst})
        bad_points = [item for item in trail["points"] if item["state"] != "outside"]
        bad_segments = [item for item in trail["segments"] if item["inside_samples"] or item["uncertain_samples"]]
        if trail["active"] and (bad_points or bad_segments):
            failures.append({"trail": trail["trail"], "te_node": trail["te_node"], "strength": trail["strength"],
                             "points": bad_points, "segments": bad_segments})
        if not trail["active"] and (bad_points or bad_segments):
            disabled_inside.append({"trail": trail["trail"], "te_node": trail["te_node"], "concave": trail["concave"],
                                    "strength": trail["strength"], "points_not_outside": len(bad_points),
                                    "segments_not_outside": len(bad_segments)})
    active = [trail for trail in trails if trail["active"]]
    result = {"run": str(run), "adb_sha256": adb["sha256"], "vspgeom_sha256": geometry["sha256"], "vkey_sha256": vkey["sha256"],
              "body_triangles": int(len(triangles)), "body_binary32_radius_max_m": body_radius,
              "trails": len(trails), "active_trails": len(active), "disabled_trails": len(trails) - len(active),
              "active_points_inside": sum(item["state"] == "inside" for trail in active for item in trail["points"]),
              "active_points_uncertain": sum(item["state"] == "uncertain" for trail in active for item in trail["points"]),
              "active_segments_not_outside": sum(1 for trail in active for item in trail["segments"]
                                                 if item["inside_samples"] or item["uncertain_samples"]),
              "failures": failures, "disabled_not_outside": disabled_inside,
              "strips": [dict(value, chain=key[0] + 1, strip=key[1] + 1) for key, value in sorted(strips.items())],
              "trail_details": trails}
    result["passed"] = not failures
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("run")
    parser.add_argument("--out", required=True)
    args = parser.parse_args(argv)
    result = assess(args.run)
    with open(args.out, "x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=1)
        stream.write("\n")
    print(json.dumps({key: result[key] for key in ("passed", "trails", "active_trails", "disabled_trails", "active_points_inside",
                                                   "active_points_uncertain", "active_segments_not_outside")}
                     | {"failing_trails": [item["trail"] for item in result["failures"]],
                        "disabled_not_outside": [item["trail"] for item in result["disabled_not_outside"]]}))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
