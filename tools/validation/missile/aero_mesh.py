"""Mesh symmetric authored missile geometry for an offline static experiment.

Gmsh/NumPy/Hangar are optional worker dependencies. Pure topology, reflection
and export guards require only the standard library. A valid mesh is not proof
of spatial convergence, credible aerodynamic loads or equipment calibration.
"""
import hashlib
import math
from pathlib import Path
import sys
import tomllib


def finite(value, label):
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(label + " must be finite") from error
    if isinstance(value, bool) or not math.isfinite(result):
        raise ValueError(label + " must be finite")
    return result


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def subtract(a, b):
    return tuple(x-y for x, y in zip(a, b))


def det(a, b, c, d):
    return finite(sum(x*y for x, y in zip(cross(subtract(b, a), subtract(c, a)), subtract(d, a))),
                  "tetrahedron determinant")


def _points(points):
    result = []
    for point in points:
        if len(point) != 3:
            raise ValueError("mesh coordinates require three components")
        result.append(tuple(finite(v, "mesh coordinate") for v in point))
    if len(result) < 4:
        raise ValueError("mesh requires at least four points")
    return result


def _indices(values, count, size):
    values = tuple(values)
    if (len(values) != size or len(set(values)) != size or
            any(isinstance(v, bool) or not isinstance(v, int) or v < 0 or v >= count for v in values)):
        raise ValueError("invalid element connectivity")
    return values


def compact_volume_nodes(points, cells, markers):
    """Discard meshed CAD entities outside fluid, never a physical boundary."""
    points = _points(points)
    cells = [_indices(cell, len(points), 4) for cell in cells]
    used = sorted({v for cell in cells for v in cell})
    indices = {global_id: local for local, global_id in enumerate(used)}
    compact = {}
    for name, faces in markers.items():
        compact[name] = []
        for face in faces:
            face = _indices(face, len(points), 3)
            if any(v not in indices for v in face):
                raise ValueError("physical boundary contains a nonfluid CAD node")
            compact[name].append(tuple(indices[v] for v in face))
    return ([points[v] for v in used], [tuple(indices[v] for v in cell) for cell in cells], compact,
            len(points)-len(used))


def validate_mesh(points, cells, markers):
    """Require positive tetrahedra and exact exterior coverage; orient into fluid.

    SU2's right-hand boundary normal must point towards the tetrahedron's
    opposite vertex. Every boundary triangle has exactly one adjacent cell.
    """
    points = _points(points)
    if len(set(points)) != len(points):
        raise ValueError("mesh contains duplicate coordinates")
    cells = [_indices(cell, len(points), 4) for cell in cells]
    if not cells or len({tuple(sorted(cell)) for cell in cells}) != len(cells):
        raise ValueError("missing or duplicate volume tetrahedra")
    owners, volumes = {}, []
    for index, cell in enumerate(cells):
        volume = det(*(points[v] for v in cell)) / 6
        if volume <= 0:
            raise ValueError("tetrahedron must have positive signed volume")
        volumes.append(volume)
        for opposite in range(4):
            face = tuple(sorted(cell[:opposite] + cell[opposite+1:]))
            owners.setdefault(face, []).append((index, cell[opposite]))
    if any(len(adjacent) > 2 for adjacent in owners.values()):
        raise ValueError("nonmanifold volume face")
    neighbours = [[] for _ in cells]
    for face, adjacent in owners.items():
        if len(adjacent) == 2:
            left_side = det(*(points[v] for v in face), points[adjacent[0][1]])
            right_side = det(*(points[v] for v in face), points[adjacent[1][1]])
            # Compare signs directly: a product can overflow or underflow even
            # when both determinants and the valid cell volumes are finite.
            if left_side == 0 or right_side == 0 or (left_side > 0) == (right_side > 0):
                raise ValueError("shared-face tetrahedra must lie on opposite sides")
            left, right = adjacent[0][0], adjacent[1][0]
            neighbours[left].append(right)
            neighbours[right].append(left)
    seen_cells, pending = {0}, [0]
    while pending:
        for neighbour in neighbours[pending.pop()]:
            if neighbour not in seen_cells:
                seen_cells.add(neighbour)
                pending.append(neighbour)
    if len(seen_cells) != len(cells):
        raise ValueError("volume contains disconnected cell components")
    exterior = {face for face, adjacent in owners.items() if len(adjacent) == 1}
    seen, oriented, flips = set(), {}, 0
    for name, faces in markers.items():
        if not name or "\n" in name or "\r" in name:
            raise ValueError("invalid boundary marker name")
        oriented[name] = []
        for face in faces:
            face = _indices(face, len(points), 3)
            key = tuple(sorted(face))
            if key in seen or key not in exterior:
                raise ValueError("boundary face duplicated, missing from volume or internal")
            seen.add(key)
            signed = det(*(points[v] for v in face), points[owners[key][0][1]])
            if signed == 0:
                raise ValueError("degenerate boundary triangle")
            if signed < 0:
                face = (face[1], face[0], face[2])
                flips += 1
            oriented[name].append(face)
    if seen != exterior:
        raise ValueError("boundary markers do not cover every exterior face")
    return oriented, {"volume_m3": finite(math.fsum(volumes), "volume sum"),
                      "minimum_signed_tetrahedron_volume_m3": min(volumes),
                      "exterior_faces": len(exterior), "reoriented_boundary_triangles": flips}


def mirror_quarter(points, cells, markers, tolerance=1e-12):
    """Reflect a positive-y/positive-z mesh into a full unconstrained flow mesh.

    Only seam roundoff within the recorded tolerance is snapped to zero; no
    general rounding or near-node merge changes the original coordinates.
    """
    tolerance = finite(tolerance, "axis tolerance")
    if tolerance <= 0 or set(markers) != {"missile", "farfield", "seam_y", "seam_z"}:
        raise ValueError("quarter mesh requires four explicit physical/seam markers")
    points = _points(points)
    if any(point[axis] < -tolerance for point in points for axis in (1, 2)):
        raise ValueError("quarter mesh contains a point outside positive y/z")
    snap = max((abs(point[axis]) for point in points for axis in (1, 2) if abs(point[axis]) <= tolerance), default=0)
    points = [(x, 0. if abs(y) <= tolerance else y, 0. if abs(z) <= tolerance else z) for x, y, z in points]
    markers, _ = validate_mesh(points, cells, markers)
    for name, axis in (("seam_y", 1), ("seam_z", 2)):
        if not markers[name] or any(points[v][axis] != 0 for face in markers[name] for v in face):
            raise ValueError(name + " does not lie on its reflection plane")
    full_points, full_cells, full_markers, unique = [], [], {"missile": [], "farfield": []}, {}
    for sy, sz in ((1, 1), (-1, 1), (1, -1), (-1, -1)):
        indices = []
        for x, y, z in points:
            point = (x, sy*y, sz*z)
            if point not in unique:
                unique[point] = len(full_points)
                full_points.append(point)
            indices.append(unique[point])
        for cell in cells:
            cell = [indices[v] for v in cell]
            if sy*sz < 0:
                cell[0], cell[1] = cell[1], cell[0]
            full_cells.append(tuple(cell))
        for name in full_markers:
            for face in markers[name]:
                face = [indices[v] for v in face]
                if sy*sz < 0:
                    face[0], face[1] = face[1], face[0]
                full_markers[name].append(tuple(face))
    full_markers, report = validate_mesh(full_points, full_cells, full_markers)
    report.update(reflection_count=4, maximum_axis_snap_m=snap, axis_tolerance_m=tolerance)
    return full_points, full_cells, full_markers, report


def export_su2(path, points, cells, markers):
    points = _points(points)
    markers, report = validate_mesh(points, cells, markers)
    with Path(path).open("w", encoding="ascii") as stream:
        stream.write("NDIME= 3\nNELEM= %d\n" % len(cells))
        for cell in cells:
            stream.write("10 %d %d %d %d\n" % tuple(cell))
        stream.write("NPOIN= %d\n" % len(points))
        for index, point in enumerate(points):
            stream.write("%.17g %.17g %.17g %d\n" % (*point, index))
        stream.write("NMARK= %d\n" % len(markers))
        for name, faces in markers.items():
            stream.write("MARKER_TAG= %s\nMARKER_ELEMS= %d\n" % (name, len(faces)))
            for face in faces:
                stream.write("5 %d %d %d\n" % face)
    return report


def orient_closed_surface(points, faces):
    """Repair shared-edge directions and return a closed outward solid shell."""
    points = _points(points)
    faces = [_indices(face, len(points), 3) for face in faces]
    edges = {}
    for index, face in enumerate(faces):
        for a, b in zip(face, face[1:] + face[:1]):
            edges.setdefault(tuple(sorted((a, b))), []).append((index, a < b))
    if not edges or any(len(shared) != 2 for shared in edges.values()):
        raise ValueError("surface is open or nonmanifold")
    neighbours = [[] for _ in faces]
    for shared in edges.values():
        (left, direction), (right, other) = shared
        neighbours[left].append((right, direction == other))
        neighbours[right].append((left, direction == other))
    flips, queue = {0: False}, [0]
    while queue:
        left = queue.pop()
        for right, opposite in neighbours[left]:
            expected = flips[left] ^ opposite
            if right in flips and flips[right] != expected:
                raise ValueError("surface is not orientable")
            if right not in flips:
                flips[right] = expected
                queue.append(right)
    if len(flips) != len(faces):
        raise ValueError("surface contains disconnected shells")
    faces = [face[::-1] if flips[index] else face for index, face in enumerate(faces)]
    vectors, volumes = [], []
    for face in faces:
        a, b, c = (points[index] for index in face)
        normal = tuple(v/2 for v in cross(subtract(b, a), subtract(c, a)))
        if finite(math.hypot(*normal), "surface area") <= 0:
            raise ValueError("degenerate surface triangle")
        vectors.append(normal)
        volumes.append(sum(x*y for x, y in zip(a, cross(b, c))) / 6)
    volume = finite(math.fsum(volumes), "enclosed volume")
    if volume == 0:
        raise ValueError("surface has no enclosed volume")
    vector = [finite(math.fsum(v[axis] for v in vectors), "surface area vector") for axis in range(3)]
    return ([face[::-1] for face in faces] if volume < 0 else faces,
            {"volume_m3": abs(volume), "area_vector_norm_m2": math.hypot(*vector),
             "reoriented_triangles": sum(flips.values())})


def export_cart3d(path, points, faces):
    faces, report = orient_closed_surface(points, faces)
    used = sorted({v for face in faces for v in face})
    indices = {index: local+1 for local, index in enumerate(used)}
    with Path(path).open("w", encoding="ascii") as stream:
        stream.write("%d %d\n" % (len(used), len(faces)))
        for index in used:
            stream.write("%.17g %.17g %.17g\n" % tuple(points[index]))
        for face in faces:
            stream.write("%d %d %d\n" % tuple(indices[v] for v in face))
    return report


def surface_metrics(points, faces):
    """Triangle integrals; projected area counts all panels, not a silhouette."""
    points = _points(points)
    faces = [_indices(face, len(points), 3) for face in faces]
    if not faces:
        raise ValueError("surface metrics require boundary triangles")
    vectors = []
    for face in faces:
        a, b, c = (points[v] for v in face)
        vector = tuple(finite(v/2, "triangle area vector") for v in cross(subtract(b, a), subtract(c, a)))
        if math.hypot(*vector) == 0:
            raise ValueError("surface metrics found a degenerate triangle")
        vectors.append(vector)
    used = {v for face in faces for v in face}
    return {"surface_area_m2": finite(math.fsum(math.hypot(*v) for v in vectors), "surface area sum"),
            "half_absolute_projected_area_m2": [math.fsum(abs(v[axis]) for v in vectors)/2 for axis in range(3)],
            "bounds_m": {"minimum": [min(points[v][axis] for v in used) for axis in range(3)],
                         "maximum": [max(points[v][axis] for v in used) for axis in range(3)]}}


def _symmetric_geometry(aircraft, sections):
    """Reject designs outside this representative mirrored-missile campaign."""
    import numpy as np
    body = next(b for b in aircraft.bodies if b.kind == "fuselage")
    width, top, bottom, yc, _ = body.section(sections)
    if np.max(np.abs(yc)) > 1e-10 or np.max(np.abs(top+bottom)) > 1e-10:
        raise ValueError("quarter meshing requires a centered y/z symmetric body")
    cloud = []
    for surface in aircraft.surfaces:
        if len(surface.sections) != 2:
            raise ValueError("representative fin mesh requires exactly two authored sections")
        for eta in (0., .25, .5, .75, 1.):
            le, chord, _, foil = surface.station(eta)
            along, normal = surface.frame(eta)
            xx = .5 * (1-np.cos(np.linspace(0, math.pi, 25)))
            for height in (foil.upper(xx), foil.lower(xx)):
                positions = le + np.outer(xx*chord, along) + np.outer(height*chord, normal)
                cloud.extend(positions.tolist())
                if surface.mirror:
                    cloud.extend((positions * np.asarray([1, -1, 1])).tolist())
    keys = {tuple(round(float(v), 9) for v in point) for point in cloud}
    if any(tuple(round(float(v) * (-1 if index == axis else 1), 9) for index, v in enumerate(point)) not in keys
           for point in cloud for axis in (1, 2)):
        raise ValueError("quarter meshing requires reflected y/z symmetric fin geometry")
    return width, top, bottom


def mesh_controls(size, length):
    """Uniformly scale every size source while keeping CAD/domain fixed.

    Gmsh applies MeshSizeFactor to all element sizes, including curvature and
    farfield sources. Scaling only the near-wall field cannot define a global
    refinement family. The base definition corresponds to size=0.04 metres.
    """
    size, length = finite(size, "mesh size"), finite(length, "body length")
    if size <= 0 or length <= 0:
        raise ValueError("mesh size and body length must be positive")
    return {"options": {"General.NumThreads": 1, "Mesh.MaxNumThreads1D": 1, "Mesh.MaxNumThreads2D": 1,
                        "Mesh.MaxNumThreads3D": 1, "Mesh.RandomSeed": 1, "Mesh.Reproducible": 1,
                        "Mesh.Algorithm3D": 10, "Mesh.MeshSizeFromPoints": 0, "Mesh.MeshSizeExtendFromBoundary": 0,
                        "Mesh.MeshSizeMin": .04*.04, "Mesh.MeshSizeMax": length, "Mesh.MeshSizeFromCurvature": 24,
                        "Mesh.MeshSizeFactor": finite(size/.04, "global mesh-size factor")},
            "distance_field": {"SizeMin": .04, "SizeMax": length, "DistMin": .04, "DistMax": length}}


def mesh(design, output, gmsh_path, size):
    """Build the original sampled body and foil geometry with a genuine nose tip.

    The body is circularized and linear between the same 44 reference samples;
    fin sections interpolate 25 original foil points on each side. A positive
    quarter HXT/Netgen tetrahedral mesh is reflected into the full domain. The
    solver receives no symmetry boundary conditions. Approximation changes and
    axis snapping are explicit in the returned evidence. Size controls a global
    factor on all size sources; the original CAD definition stays unchanged.
    """
    size = finite(size, "mesh size")
    if size <= 0:
        raise ValueError("mesh size must be positive")
    sys.path.insert(0, str(gmsh_path))
    import gmsh
    import numpy as np
    root = Path(__file__).resolve().parents[3]
    sys.path.insert(0, str(root / "tools/models/hangar"))
    from hangar.geometry.aircraft import Aircraft
    design, output = Path(design), Path(output)
    aircraft = Aircraft.load(str(design))
    spec = tomllib.loads(design.read_text(encoding="utf-8"))
    if len(aircraft.bodies) != 1 or aircraft.bodies[0].kind != "fuselage":
        raise ValueError("representative mesh requires one authored fuselage body")
    body = aircraft.bodies[0]
    sections = np.unique(np.concatenate((body.x, np.linspace(body.x[0], body.x[-1], 35))))
    width, top, bottom = _symmetric_geometry(aircraft, sections)
    radii = (width+top-bottom)/4
    length = finite(body.x[-1]-body.x[0], "body length")
    if abs(body.x[0]) > 1e-12 or length <= 0 or np.any(radii < 0):
        raise ValueError("body requires a nonnegative radius profile beginning at x=0")
    controls = mesh_controls(size, length)
    diameter = finite(max(body.w), "body diameter")
    reference = {"area_m2": math.pi*diameter*diameter/4, "length_m": diameter,
                 "moment_x_m": finite(spec["mass"]["empty_cg"][0], "moment reference")}
    output.mkdir(parents=True, exist_ok=True)
    gmsh.initialize()
    try:
        for key, value in controls["options"].items():
            gmsh.option.setNumber(key, value)
        gmsh.model.add("authored-game-missile-point-nose")
        occ = gmsh.model.occ
        solids = []
        for x0, x1, r0, r1 in zip(sections, sections[1:], radii, radii[1:]):
            if abs(r0-r1) < 1e-12:
                tag = occ.addCylinder(float(x0), 0, 0, float(x1-x0), 0, 0, float(r0))
            else:
                tag = occ.addCone(float(x0), 0, 0, float(x1-x0), 0, 0, float(r0), float(r1))
            solids.append((3, tag))
        solids, _ = occ.fuse([solids[0]], solids[1:])
        fin_count, fin_geometry = 0, []
        for surface in aircraft.surfaces:
            for mirrored in ((False, True) if surface.mirror else (False,)):
                wires, stations = [], []
                for eta in (0., 1.):
                    le, chord, _, foil = surface.station(eta)
                    along, normal = surface.frame(eta)
                    xx = .5*(1-np.cos(np.linspace(0, math.pi, 25)))
                    upper = le + np.outer(xx*chord, along) + np.outer(foil.upper(xx)*chord, normal)
                    lower = le + np.outer(xx*chord, along) + np.outer(foil.lower(xx)*chord, normal)
                    if mirrored:
                        upper[:, 1] *= -1
                        lower[:, 1] *= -1
                    station_le = [float(v) for v in le]
                    if mirrored:
                        station_le[1] *= -1
                    stations.append({"leading_edge_m": station_le, "chord_m": float(chord)})
                    if np.max(np.abs(upper[[0, -1]]-lower[[0, -1]])) > 1e-10:
                        raise ValueError("fin mesh requires a closed leading/trailing-edge foil")
                    upper_ids = [occ.addPoint(*map(float, pos)) for pos in upper]
                    lower_ids = [upper_ids[0]] + [occ.addPoint(*map(float, pos)) for pos in lower[1:-1]] + [upper_ids[-1]]
                    wires.append(occ.addWire([occ.addSpline(upper_ids), occ.addSpline(lower_ids[::-1])]))
                solids += occ.addThruSections(wires, makeSolid=True, makeRuled=True)
                fin_count += 1
                fin_geometry.append({"name": surface.name, "mirrored_y": mirrored, "stations": stations,
                                     "authored_planform_area_m2": float(surface.area/(2 if surface.mirror else 1))})
        solids, _ = occ.fuse([solids[0]], solids[1:])
        if len(solids) != 1 or solids[0][0] != 3:
            raise ValueError("authored body and fins must form one connected solid")
        box = occ.addBox(-3*length, 0, 0, 8*length, 3*length, 3*length)
        fluid, _ = occ.cut([(3, box)], solids)
        occ.synchronize()
        groups = {"missile": [], "farfield": [], "seam_y": [], "seam_z": []}
        for dim, signed in gmsh.model.getBoundary(fluid, oriented=True):
            tag = abs(signed)
            bounds = gmsh.model.getBoundingBox(dim, tag)
            if abs(bounds[1]) < 1e-6 and abs(bounds[4]) < 1e-6:
                name = "seam_y"
            elif abs(bounds[2]) < 1e-6 and abs(bounds[5]) < 1e-6:
                name = "seam_z"
            else:
                name = "farfield" if max(abs(v) for v in bounds[1:3]+bounds[4:6]) > length else "missile"
            groups[name].append(tag)
        for name, tags in groups.items():
            if not tags:
                raise ValueError("missing CAD boundary group " + name)
            gmsh.model.addPhysicalGroup(2, tags, name=name)
        distance = gmsh.model.mesh.field.add("Distance")
        gmsh.model.mesh.field.setNumbers(distance, "SurfacesList", groups["missile"])
        threshold = gmsh.model.mesh.field.add("Threshold")
        for key, value in dict(controls["distance_field"], InField=distance).items():
            gmsh.model.mesh.field.setNumber(threshold, key, value)
        gmsh.model.mesh.field.setAsBackgroundMesh(threshold)
        gmsh.model.mesh.generate(3)
        gmsh.model.mesh.optimize("Netgen")
        tags, coords, _ = gmsh.model.mesh.getNodes()
        points = [tuple(map(float, point)) for point in np.asarray(coords).reshape(-1, 3)]
        index = {int(tag): local for local, tag in enumerate(tags)}
        cells, qualities, markers = [], [], {}
        for dim, tag in fluid:
            kinds, ids, connectivity = gmsh.model.mesh.getElements(3, tag)
            for kind, element_ids, nodes in zip(kinds, ids, connectivity):
                if kind != 4:
                    raise ValueError("volume must contain first-order tetrahedra")
                cells.extend(tuple(index[int(v)] for v in cell) for cell in np.asarray(nodes).reshape(-1, 4))
                qualities.extend(float(v) for v in gmsh.model.mesh.getElementQualities(element_ids, "minSICN"))
        for name, tags in groups.items():
            markers[name] = []
            for tag in tags:
                kinds, _, connectivity = gmsh.model.mesh.getElements(2, tag)
                for kind, nodes in zip(kinds, connectivity):
                    if kind != 2:
                        raise ValueError("boundary must contain first-order triangles")
                    markers[name].extend(tuple(index[int(v)] for v in face) for face in np.asarray(nodes).reshape(-1, 3))
        if not qualities or any(not math.isfinite(v) or v <= 0 for v in qualities):
            raise ValueError("volume contains invalid tetrahedron quality")
        points, cells, markers, excluded = compact_volume_nodes(points, cells, markers)
        points, cells, markers, reflection = mirror_quarter(points, cells, markers)
        volume_report = export_su2(output / "mesh.su2", points, cells, markers)
        topology = export_cart3d(output / "missile.tri", points, markers["missile"])
        radius_difference = float(max(np.max(np.abs(radii-width/2)), np.max(np.abs(radii-(top-bottom)/2))))
        return {"gmsh_version": gmsh.__version__, "reference": reference, "surface_triangles": len(markers["missile"]),
                "topology": topology, "volume_topology": volume_report, "volume_nodes": len(points), "tetrahedra": len(cells),
                "physical_wall": surface_metrics(points, markers["missile"]), "fin_geometry": fin_geometry,
                "authored_total_fin_planform_area_m2": sum(fin["authored_planform_area_m2"] for fin in fin_geometry),
                "minimum_tetrahedron_SICN": min(qualities), "tetrahedron_SICN_percentiles": np.percentile(qualities, [1, 5, 50]).tolist(),
                "body_section_count": len(sections), "fin_count": fin_count, "axisymmetry_radius_difference_m": radius_difference,
                "nose_radius_m": float(radii[0]), "nose_representation": "native point/conical patch; no truncated cap",
                "fin_foil_points_per_side": 25, "fin_foil_interpolation": "cubic spline through authored cosine-spaced coordinates",
                "reflection": reflection, "flow_domain": "full reflected mesh; no symmetry boundary condition",
                "mesh_controls": controls,
                "excluded_nonfluid_CAD_mesh_nodes": excluded,
                "mesh_sensitivity": "Not established by mesh validity or a single-grid run",
                "surface_sha256": hashlib.sha256((output / "missile.tri").read_bytes()).hexdigest(),
                "volume_mesh_sha256": hashlib.sha256((output / "mesh.su2").read_bytes()).hexdigest(),
                "design_sha256": hashlib.sha256(design.read_bytes()).hexdigest()}
    finally:
        gmsh.finalize()
