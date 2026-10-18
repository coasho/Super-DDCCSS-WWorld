"""Inspect one pinned VSPAERO 7.2.2 steady binary case without running a tool.

Layout source: OpenVSP 3.55.0 commit 5240ef210f41c5d6aeed0ba7687f6fab0ca0cafb,
PGMulti::WriteVSPGeom, VSP_SOLVER::WriteOutAerothermalDatabase{Header,Geometry,
Solution}, and VORTEX_TRAIL::WriteToFile. The shared ADB version-3 magic does
not establish solver provenance; the caller must verify its actual process.
"""
from collections import Counter
import hashlib
import math
from pathlib import Path
import re
import struct

from aero_vsp_case import _intended

_MAX_BYTES = 64 * 1024 * 1024
_MAX_RECORDS = 1_000_000
_INTEGER = re.compile(r"[-+]?\d+\Z")


def _finite(value, label):
    try:
        result = float(value)
    except (ValueError, TypeError, OverflowError) as error:
        raise ValueError(label + " must be finite") from error
    if isinstance(value, bool) or not math.isfinite(result):
        raise ValueError(label + " must be finite")
    return result


def _f32(value):
    try:
        result = struct.unpack("<f", struct.pack("<f", value))[0]
    except (struct.error, OverflowError) as error:
        raise ValueError("value is outside finite binary32 range") from error
    if not math.isfinite(result):
        raise ValueError("value is outside finite binary32 range")
    return result


def _xyz32(point):
    return tuple(_f32(value) for value in point)


def _read(path):
    try:
        path = Path(path)
        if path.stat().st_size > _MAX_BYTES:
            raise ValueError("native input exceeds the explicit 64 MiB limit")
        raw = path.read_bytes()
        if not raw or len(raw) > _MAX_BYTES:
            raise ValueError("empty or oversized native input")
        return raw, {"path": str(path.resolve()), "sha256": hashlib.sha256(raw).hexdigest()}
    except (OSError, TypeError) as error:
        raise ValueError("missing or unreadable native input") from error


class _Binary:
    def __init__(self, raw):
        self.raw, self.offset = raw, 0

    def preflight(self, count, size, label):
        if count < 0 or count > 8 * _MAX_RECORDS or count * size > len(self.raw) - self.offset:
            raise ValueError("invalid or truncated ADB table: " + label)

    def records(self, fmt, count, label):
        item = struct.Struct("<" + fmt)
        self.preflight(count, item.size, label)
        end = self.offset + count * item.size
        values = list(item.iter_unpack(memoryview(self.raw)[self.offset:end]))
        self.offset = end
        if any(isinstance(value, float) and not math.isfinite(value) for row in values for value in row):
            raise ValueError("nonfinite ADB field: " + label)
        return values

    def one(self, fmt, label):
        values = self.records(fmt, 1, label)[0]
        return values[0] if len(values) == 1 else values

    def count(self, label, *, positive=False):
        result = self.one("i", label)
        if result < int(positive) or result > _MAX_RECORDS:
            raise ValueError("invalid bounded ADB count: " + label)
        return result

    def floats(self, kind, count, label):
        item = struct.Struct("<" + kind)
        self.preflight(count, item.size, label)
        end = self.offset + count * item.size
        if any(not math.isfinite(row[0]) for row in item.iter_unpack(memoryview(self.raw)[self.offset:end])):
            raise ValueError("nonfinite ADB payload: " + label)
        self.offset = end


class _Text:
    def __init__(self, raw):
        try:
            self.lines = [line.split() for line in raw.decode("utf-8").splitlines() if line.strip()]
        except UnicodeError as error:
            raise ValueError("source geometry must be UTF-8 text") from error
        self.index = 0

    def row(self):
        if self.index >= len(self.lines):
            raise ValueError("truncated source VSPGEOM")
        result = self.lines[self.index]
        self.index += 1
        return result

    @staticmethod
    def integers(fields):
        if any(not _INTEGER.fullmatch(value) for value in fields):
            raise ValueError("invalid source geometry integer")
        return list(map(int, fields))


def _source(raw):
    text = _Text(raw)
    if text.row() != ["#", "vspgeom", "v3"] or text.row() != ["1"]:
        raise ValueError("only one complete source VSPGEOM v3 mesh is supported")
    sizes = text.integers(text.row())
    if len(sizes) != 3 or any(value < 1 or value > _MAX_RECORDS for value in sizes):
        raise ValueError("invalid source geometry counts")
    nn, nf, nw = sizes
    if nn + 5 * nf + nw > len(text.lines) - text.index:
        raise ValueError("source table counts exceed available lines")
    points = []
    for _ in range(nn):
        row = text.row()
        if len(row) != 3:
            raise ValueError("source point must have three coordinates")
        points.append(tuple(_finite(value, "source coordinate") for value in row))
    if text.row() != [str(nf)]:
        raise ValueError("source face count does not match the prefix")
    faces, used, edges = [], set(), set()
    for _ in range(nf):
        row = text.integers(text.row())
        if (len(row) < 4 or row[0] != len(row) - 1 or len(set(row[1:])) != row[0]
                or min(row[1:]) < 1 or max(row[1:]) > nn):
            raise ValueError("invalid source polygon connectivity")
        face = row[1:]
        faces.append(face); used.update(face)
        edges.update(tuple(sorted((a, b))) for a, b in zip(face, face[1:] + face[:1]))
    parts, tags = [], []
    for face in faces:
        row = text.row()
        if len(row) != 2 + 2 * len(face):
            raise ValueError("invalid source polygon UV count")
        part, tag = text.integers(row[:2])
        if part < 1 or tag < 1:
            raise ValueError("invalid source part/tag")
        for value in row[2:]:
            _finite(value, "source UV")
        parts.append(part); tags.append(tag)
    part_ids = set(parts)
    if part_ids != set(range(1, len(part_ids) + 1)):
        raise ValueError("unsupported noncontiguous source surface IDs")
    for index in range(1, nf + 1):
        if text.integers(text.row()) != [index, index]:
            raise ValueError("single-mesh source parents must refer to themselves")
    if text.row() != [str(nw)]:
        raise ValueError("source wake count does not match the prefix")
    wake_nodes, wake_edges, wake_pairs = [], 0, []
    for _ in range(nw):
        row = text.integers(text.row())
        if len(row) < 2 or not 2 <= row[0] <= nn or row[1] not in part_ids:
            raise ValueError("only positive wing wake chains are supported")
        size, _part = row[:2]
        ids = row[2:]
        while len(ids) < size:
            ids.extend(text.integers(text.row()))
        if (len(ids) != size or len(set(ids)) != size or not set(ids) <= used
                or any(tuple(sorted(pair)) not in edges for pair in zip(ids, ids[1:]))):
            raise ValueError("invalid source TE chain")
        wake_nodes.extend(ids); wake_edges += size - 1
        wake_pairs.extend(zip(ids, ids[1:]))
    if len(set(wake_nodes)) != len(wake_nodes):
        raise ValueError("overlapping source wake-node chains are outside this profile")
    alternatives = []
    for index, face in enumerate(faces, 1):
        row = text.integers(text.row())
        if len(row) < 5 or row[0] != index or row[1] < 1 or len(row) != 2 + 3 * row[1]:
            raise ValueError("invalid source alternate triangulation")
        tris = [row[j:j+3] for j in range(2, len(row), 3)]
        if any(len(set(tri)) != 3 or not set(tri) <= set(face) for tri in tris):
            raise ValueError("source alternate triangle leaves its parent polygon")
        alternatives.append(tris)
    for index, tris in enumerate(alternatives, 1):
        row = text.row()
        if len(row) != 3 + 6 * len(tris) or text.integers(row[:3]) != [index, parts[index-1], tags[index-1]]:
            raise ValueError("invalid source alternate UV metadata")
        for value in row[3:]:
            _finite(value, "source alternate UV")
    if text.index != len(text.lines):
        raise ValueError("extra source geometry sections are unsupported")
    return {"points": points, "used": used, "parts": part_ids, "faces": nf,
            "fine_triangles": sum(map(len, alternatives)), "triangles": [tri for group in alternatives for tri in group],
            "wake_nodes": wake_nodes, "wake_edges": wake_edges, "wake_pairs": wake_pairs}


def _triangle(points):
    """Ignore the starting vertex while preserving native winding."""
    points = tuple(points)
    return min(points[index:] + points[:index] for index in range(3))


def assess_adb(adb_path, geometry_path, pair, *, reference_area, bref, cref,
               native_cg, wake_nodes, solver_version="7.2.2"):
    """Check actual geometry; this never establishes model admission.

    Structural/reference/unsupported-profile failures raise ValueError. Validly
    decoded detached or changed geometry returns passed=False. Coordinates are
    compared exactly after the writer's binary32 rounding, with multiplicity.
    Concave trails export only their root; unexported points are not inspected.
    """
    if solver_version != "7.2.2":
        raise ValueError("only the explicitly pinned 7.2.2 writer layout is supported")
    intended = _intended(pair)["native"]
    area, span, chord = (_finite(value, label) for value, label in
                         ((reference_area, "Sref"), (bref, "Bref"), (cref, "Cref")))
    if min(area, span, chord) <= 0 or not isinstance(native_cg, (list, tuple)) or len(native_cg) != 3:
        raise ValueError("positive references and a three-coordinate native CG are required")
    cg = [_finite(value, "native CG") for value in native_cg]
    if isinstance(wake_nodes, bool) or not isinstance(wake_nodes, int) or not 3 <= wake_nodes <= _MAX_RECORDS:
        raise ValueError("wake_nodes must be an integer of at least three")
    raw, adb_record = _read(adb_path)
    source_raw, source_record = _read(geometry_path)
    source = _source(source_raw)
    nk_expected = len(source["wake_nodes"])
    ns_expected = len(source["parts"])
    n_surface = len(source["used"])
    near_loops = source["wake_edges"] * (wake_nodes - 1)
    reader = _Binary(raw)
    magic, model, symmetry, unsteady, loops, nodes, triangles, edges = reader.one("8i", "header")
    if (magic, model, symmetry, unsteady) != (-123789453, 2, 0, 0):
        raise ValueError("unsupported ADB format/model/symmetry/time profile")
    if any(value < 1 or value > _MAX_RECORDS for value in (loops, nodes, triangles, edges)):
        raise ValueError("invalid bounded ADB header counts")
    if (loops != source["faces"] + near_loops or nodes != len(source["points"]) + nk_expected * wake_nodes
            or triangles != source["fine_triangles"] + 2 * near_loops):
        raise ValueError("ADB counts differ from the declared level-1 source/wake packing")
    refs = reader.one("6f", "references")
    # Native writer order is Sref, Cref, Bref (not API argument order), then CG.
    if refs != tuple(_f32(value) for value in (area, chord, span, *cg)):
        raise ValueError("ADB reference area/lengths/CG differ from the intended case")
    ns = reader.count("surface names", positive=True)
    if ns != ns_expected:
        raise ValueError("ADB surface count differs from the source")
    reader.preflight(ns, 108, "surface metadata")
    surface_ids = set()
    for _ in range(ns):
        number, name, component = reader.one("i100si", "surface metadata")
        if number in surface_ids or number not in source["parts"] or not 1 <= component <= ns:
            raise ValueError("invalid or duplicate ADB surface/component ID")
        try:
            name.split(b"\0", 1)[0].decode("utf-8")
        except UnicodeError as error:
            raise ValueError("invalid ADB component name") from error
        surface_ids.add(number)
    reader.preflight(1, triangles * 28 + nodes * 12 + 12, "fine geometry")
    fine_tris = reader.records("6if", triangles, "fine triangles")
    for row in fine_tris:
        if (min(row[:3]) < 1 or max(row[:3]) > nodes or row[4] not in surface_ids | {0}
                or not 0 <= row[3] <= ns or row[5] < 0 or row[6] < 0
                or (row[4] > 0 and (row[3] == 0 or row[6] <= 0 or len(set(row[:3])) != 3))):
            raise ValueError("invalid physical/wake triangle IDs, time or area")
    fine_xyz = reader.records("3f", nodes, "fine coordinates")
    for row in fine_tris:
        if row[4] > 0:
            a, b, c = (fine_xyz[node-1] for node in row[:3])
            u, v = ([point[axis]-a[axis] for axis in range(3)] for point in (b, c))
            cross = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
            if not any(value != 0 for value in cross):
                raise ValueError("exported physical triangle coordinates have zero area")
    if reader.count("rotors") != 0 or reader.count("nozzles") != 0:
        raise ValueError("rotor/nozzle ADB profiles are unsupported")
    levels = reader.count("grid levels", positive=True)
    if levels > 32:
        raise ValueError("unsupported grid hierarchy size")
    reader.preflight(levels, 8, "grid hierarchy headers")
    solve_xyz, solve_edges = None, None
    for level in range(1, levels + 1):
        nnode, nedge = reader.one("2i", "coarse counts")
        if min(nnode, nedge) < 1 or max(nnode, nedge) > _MAX_RECORDS:
            raise ValueError("invalid bounded coarse-grid counts")
        reader.preflight(1, 12*nnode + 16*nedge, "coarse geometry")
        xyz = reader.records("3f", nnode, "coarse coordinates")
        links = reader.records("4i", nedge, "coarse edges")
        if any(abs(row[0]) > ns or row[1] < 0 or min(row[2:]) < 1 or max(row[2:]) > nnode for row in links):
            raise ValueError("invalid coarse-edge surface/time/node IDs")
        if level == 1:
            solve_xyz, solve_edges = xyz, links
    if len(solve_edges) != edges or len(solve_xyz) != n_surface + nk_expected * wake_nodes:
        raise ValueError("unsupported solve level or nonmatching source node packing")
    nte = reader.count("Kutta edges", positive=True)
    if nte != source["wake_edges"]:
        raise ValueError("native Kutta edge count differs from the authored TE chains")
    kte = [row[0] for row in reader.records("i", nte, "Kutta edges")]
    if len(set(kte)) != nte or min(kte) < 1 or max(kte) > edges:
        raise ValueError("invalid or duplicate Kutta edge IDs")
    nk = reader.count("Kutta nodes", positive=True)
    if nk != nk_expected:
        raise ValueError("native Kutta node count differs from the authored TE chains")
    kutta = [row[0] for row in reader.records("i", nk, "Kutta nodes")]
    if len(set(kutta)) != nk or min(kutta) < 1 or max(kutta) > n_surface:
        raise ValueError("invalid or duplicate physical Kutta node IDs")
    if reader.count("controls") != 0:
        raise ValueError("control-surface ADB profiles are unsupported")
    solution_offset = reader.offset
    reader.preflight(1, 20 + 40*loops + 24*edges + 12*triangles + 4, "complete solution arrays")
    condition = reader.one("5f", "condition and Cp range")
    # Solver.C sets lower/upper pressure limits before the writer emits them.
    if condition[3] > condition[4]:
        raise ValueError("ADB pressure limits are reversed")
    expected_condition = (intended["mach"], math.radians(intended["alpha_deg"]), math.radians(intended["beta_deg"]))
    if condition[:3] != tuple(_f32(value) for value in expected_condition):
        raise ValueError("ADB Mach or radian native angles differ from the paired condition")
    for kind, count, label in (("d", 2*loops, "coarse gamma/unsteady pressure"),
                               ("d", 3*edges, "edge forces"), ("d", 3*loops, "loop velocities"),
                               ("f", 3*triangles, "fine pressure/gamma")):
        reader.floats(kind, count, label)
    nw = reader.count("wake records", positive=True)
    if nw != nk:
        raise ValueError("missing or extra native wake records")
    reader.preflight(nw, 40, "minimum wake records")
    root_checks, point_checks, exported_counts = [], [], []
    for index in range(nw):
        node, _span, count = reader.one("idi", "wake association/span/count")
        if node != kutta[index] or count not in (1, wake_nodes):
            raise ValueError("invalid native wake order, association or exported count")
        xyz = reader.records("3d", count, "internal wake coordinates")
        root_checks.append(_xyz32(xyz[0]) == solve_xyz[node-1])
        start = n_surface + index * wake_nodes
        point_checks.append(all(_xyz32(point) == solve_xyz[start+j] for j, point in enumerate(xyz)))
        exported_counts.append(count)
    if reader.offset != len(raw):
        raise ValueError("extra ADB bytes or multiple cases are unsupported")
    expected_points = Counter(_xyz32(source["points"][node-1]) for node in source["used"])
    actual_nodes = {node for row in fine_tris if row[4] > 0 for node in row[:3]}
    actual_points = Counter(fine_xyz[node-1] for node in actual_nodes)
    source_te = Counter(_xyz32(source["points"][node-1]) for node in source["wake_nodes"])
    source_te_edges = Counter(tuple(sorted(_xyz32(source["points"][node-1]) for node in pair))
                              for pair in source["wake_pairs"])
    actual_te_edges = Counter(tuple(sorted(solve_xyz[node-1] for node in solve_edges[edge-1][2:]))
                              for edge in kte)
    expected_triangles = Counter(_triangle(_xyz32(source["points"][node-1]) for node in tri)
                                 for tri in source["triangles"])
    actual_triangles = Counter(_triangle(fine_xyz[node-1] for node in row[:3])
                               for row in fine_tris if row[4] > 0)
    checks = {"physical_surface_points": actual_points == expected_points,
              "physical_surface_triangles": actual_triangles == expected_triangles,
              "solve_surface_points": Counter(solve_xyz[:n_surface]) == expected_points,
              "complete_authored_te_points": Counter(solve_xyz[node-1] for node in kutta) == source_te,
              "complete_authored_te_edges": actual_te_edges == source_te_edges,
              "every_internal_root_attached": all(root_checks),
              "every_exported_internal_point_matches_mesh": all(point_checks)}
    return {"passed": all(checks.values()), "checks": checks, "model_accepted": False,
            "layout": "pinned 7.2.2 single steady unsymmetrized solve-level-1 case, positive wing wakes, no optional rotor/nozzle/control records",
            "raw_files": {"adb": adb_record, "geometry": source_record},
            "bytes": len(raw), "decoded_bytes": reader.offset, "solution_offset": solution_offset,
            "physical_source_nodes": n_surface, "physical_actual_nodes": len(actual_nodes),
            "source_components": ns_expected, "native_kutta_nodes": nk, "native_wake_records": nw,
            "concave_single_node_records": exported_counts.count(1),
            "exported_internal_points": sum(exported_counts), "exported_points_per_trail": exported_counts,
            "root_checks": root_checks, "all_internal_point_checks": point_checks,
            "binary_rounding": "exact equality after one IEEE754 binary32 conversion of source coordinates/internal doubles; physical coordinate multiplicity retained",
            "unobserved": "Concave trails export only their root; unexported internal points and internal normals are not verified by this format. Panel closure, self-intersections and wake-solid contacts require separate checks.",
            "references_float32": {"Sref": refs[0], "Cref": refs[1], "Bref": refs[2], "native_cg": list(refs[3:])},
            "conditions_float32": {"mach": condition[0], "alpha_rad": condition[1], "beta_rad": condition[2],
                                   "cp_min": condition[3], "cp_max": condition[4]}}
