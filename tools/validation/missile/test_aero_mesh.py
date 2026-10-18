"""Pure connectivity, reflection and export regressions; no mesher required."""
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

_SPEC = importlib.util.spec_from_file_location("aero_mesh", Path(__file__).with_name("aero_mesh.py"))
mesh = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(mesh)

POINTS = [(0., 0., 0.), (1., 0., 0.), (0., 1., 0.), (0., 0., 1.)]
TETS = [(0, 1, 2, 3)]
BOUNDARY = {"wall": [(0, 1, 2), (0, 1, 3), (0, 2, 3), (1, 2, 3)]}
QUARTER = {"seam_z": [(0, 1, 2)], "seam_y": [(0, 1, 3)],
           "farfield": [(0, 2, 3)], "missile": [(1, 2, 3)]}


class MeshContracts(unittest.TestCase):
    def test_global_mesh_family_scales_curvature_and_farfield_with_fixed_cad(self):
        coarse = mesh.mesh_controls(.04, 3.66)
        fine = mesh.mesh_controls(.02, 3.66)
        self.assertEqual(coarse["options"]["Mesh.MeshSizeFactor"], 1.)
        self.assertEqual(fine["options"]["Mesh.MeshSizeFactor"], .5)
        for key, value in coarse["options"].items():
            if key != "Mesh.MeshSizeFactor":
                self.assertEqual(value, fine["options"][key])
        self.assertEqual(coarse["distance_field"], fine["distance_field"])
        self.assertEqual(coarse["options"]["General.NumThreads"], 1)
        self.assertEqual(coarse["options"]["Mesh.RandomSeed"], 1)
        self.assertEqual(coarse["options"]["Mesh.Reproducible"], 1)
        for key in ("Mesh.MeshSizeMin", "Mesh.MeshSizeMax"):
            self.assertEqual(fine["options"][key]*fine["options"]["Mesh.MeshSizeFactor"], coarse["options"][key]/2)
        for invalid in (0, -.01, math.nan, math.inf, True):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                mesh.mesh_controls(invalid, 3.66)

    def test_boundary_orientation_is_repaired_using_adjacent_tetrahedron(self):
        faces, report = mesh.validate_mesh(POINTS, TETS, BOUNDARY)
        self.assertAlmostEqual(report["volume_m3"], 1 / 6)
        self.assertEqual(report["exterior_faces"], 4)
        for face in faces["wall"]:
            opposite = next(index for index in range(4) if index not in face)
            self.assertGreater(mesh.det(POINTS[face[0]], POINTS[face[1]], POINTS[face[2]], POINTS[opposite]), 0)

    def test_invalid_volume_cells_and_coordinates_are_rejected(self):
        for points, cells in ((POINTS, [(0, 2, 1, 3)]), (POINTS, [(0, 1, 2, 2)]),
                              (POINTS, [(0, 1, 2, 4)]), (POINTS, [(False, 1, 2, 3)]),
                              (POINTS, TETS * 2), (POINTS[:3] + [(0, 0, 0)], TETS),
                              (POINTS[:3] + [(0, 0, math.nan)], TETS)):
            with self.subTest(points=points, cells=cells), self.assertRaises(ValueError):
                mesh.validate_mesh(points, cells, BOUNDARY)

    def test_boundary_must_cover_exterior_exactly_once(self):
        for markers in ({"wall": BOUNDARY["wall"][:-1]},
                        {"wall": BOUNDARY["wall"] + [BOUNDARY["wall"][0]]},
                        {"wall": [(0, 1, 4)] + BOUNDARY["wall"][1:]}):
            with self.subTest(markers=markers), self.assertRaises(ValueError):
                mesh.validate_mesh(POINTS, TETS, markers)

    def test_disconnected_volume_and_duplicate_coordinates_are_rejected(self):
        points = POINTS + [(x+3, y, z) for x, y, z in POINTS]
        markers = {"wall": BOUNDARY["wall"] + [tuple(v+4 for v in face) for face in BOUNDARY["wall"]]}
        with self.assertRaises(ValueError):
            mesh.validate_mesh(points, TETS + [(4, 5, 6, 7)], markers)
        with self.assertRaises(ValueError):
            mesh.validate_mesh(POINTS + [POINTS[0]], TETS, BOUNDARY)

    def test_shared_face_rejects_positive_tetrahedra_folded_onto_same_side(self):
        points = POINTS + [(.1, .1, .5)]
        cells = [(0, 1, 2, 3), (0, 1, 2, 4)]
        boundary = {"wall": [(0, 1, 3), (0, 2, 3), (1, 2, 3),
                             (0, 1, 4), (0, 2, 4), (1, 2, 4)]}
        self.assertTrue(all(mesh.det(*(points[v] for v in cell)) > 0 for cell in cells))
        with self.assertRaisesRegex(ValueError, "opposite sides"):
            mesh.validate_mesh(points, cells, boundary)

    def test_shared_face_sign_check_handles_finite_extreme_determinants(self):
        boundary = {"wall": [(0, 1, 3), (0, 2, 3), (1, 2, 3),
                             (0, 1, 4), (0, 2, 4), (1, 2, 4)]}
        for scale in (1e-100, 1e100):
            with self.subTest(scale=scale):
                points = [(0, 0, 0), (scale, 0, 0), (0, scale, 0),
                          (0, 0, scale), (0, 0, -scale)]
                _, report = mesh.validate_mesh(points, [(0, 1, 2, 3), (0, 2, 1, 4)], boundary)
                self.assertTrue(math.isfinite(report["volume_m3"]))
                self.assertGreater(report["volume_m3"], 0)

    def test_auxiliary_cad_nodes_are_excluded_but_boundary_nodes_must_belong_to_fluid(self):
        points, cells, markers, count = mesh.compact_volume_nodes(POINTS + [(0, -2, -2)], TETS, QUARTER)
        self.assertEqual((points, cells, count), (POINTS, TETS, 1))
        self.assertEqual(markers, QUARTER)
        invalid = dict(QUARTER, missile=[(1, 2, 4)])
        with self.assertRaises(ValueError):
            mesh.compact_volume_nodes(POINTS + [(0, -2, -2)], TETS, invalid)

    def test_quarter_reflection_builds_full_positive_mesh_with_shared_seams(self):
        points, cells, markers, report = mesh.mirror_quarter(POINTS, TETS, QUARTER)
        self.assertEqual((len(points), len(cells)), (6, 4))
        self.assertEqual(set(markers), {"missile", "farfield"})
        _, checked = mesh.validate_mesh(points, cells, markers)
        self.assertAlmostEqual(checked["volume_m3"], 2 / 3)
        self.assertEqual(report["reflection_count"], 4)
        for x, y, z in points:
            self.assertIn((x, -y, z), points)
            self.assertIn((x, y, -z), points)

    def test_quarter_reflection_rejects_wrong_half_space_or_seam(self):
        bad = {key: list(value) for key, value in QUARTER.items()}
        bad["seam_y"], bad["missile"] = bad["missile"], bad["seam_y"]
        for points, markers in ((POINTS[:2] + [(0, -1, 0), POINTS[3]], QUARTER), (POINTS, bad),
                                (POINTS, {key: value for key, value in QUARTER.items() if key != "seam_z"})):
            with self.subTest(points=points, markers=markers), self.assertRaises(ValueError):
                mesh.mirror_quarter(points, TETS, markers)

    def test_axis_roundoff_is_bounded_and_does_not_create_duplicate_nodes(self):
        noisy = [(x, 1e-14 if y == 0 else y, -1e-14 if z == 0 else z) for x, y, z in POINTS]
        points, cells, markers, report = mesh.mirror_quarter(noisy, TETS, QUARTER)
        self.assertEqual(len(points), 6)
        self.assertLessEqual(report["maximum_axis_snap_m"], 1e-12)
        self.assertTrue(all(mesh.det(*(points[v] for v in tet)) > 0 for tet in cells))

    def test_su2_export_checks_before_writing_and_uses_inward_domain_normals(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "mesh.su2"
            report = mesh.export_su2(path, POINTS, TETS, BOUNDARY)
            text = path.read_text()
            self.assertIn("NELEM= 1", text)
            self.assertIn("NPOIN= 4", text)
            self.assertEqual(report["exterior_faces"], 4)
            invalid = Path(folder) / "invalid.su2"
            with self.assertRaises(ValueError):
                mesh.export_su2(invalid, POINTS, [(0, 2, 1, 3)], BOUNDARY)
            self.assertFalse(invalid.exists())

    def test_cart3d_export_requires_closed_orientable_surface(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "surface.tri"
            report = mesh.export_cart3d(path, POINTS, BOUNDARY["wall"])
            self.assertAlmostEqual(report["volume_m3"], 1 / 6)
            self.assertLess(report["area_vector_norm_m2"], 1e-15)
            self.assertEqual(path.read_text().splitlines()[0], "4 4")
            with self.assertRaises(ValueError):
                mesh.export_cart3d(path, POINTS, BOUNDARY["wall"][:-1])

    def test_wall_metrics_report_actual_area_projection_and_bounds(self):
        report = mesh.surface_metrics(POINTS, BOUNDARY["wall"])
        self.assertAlmostEqual(report["surface_area_m2"], 1.5 + math.sqrt(3)/2)
        self.assertEqual(report["half_absolute_projected_area_m2"], [.5, .5, .5])
        self.assertEqual(report["bounds_m"], {"minimum": [0., 0., 0.], "maximum": [1., 1., 1.]})

    def test_import_has_no_optional_runtime_dependency(self):
        code = ("import runpy,sys;runpy.run_path(sys.argv[1]);"
                "assert 'gmsh' not in sys.modules;assert 'numpy' not in sys.modules")
        result = subprocess.run([sys.executable, "-B", "-c", code, str(Path(__file__).with_name("aero_mesh.py"))],
                                capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
