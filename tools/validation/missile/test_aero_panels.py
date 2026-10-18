"""Indexed thick-panel topology regressions; no optional solver dependency."""
from pathlib import Path
import tempfile
import unittest

import aero_panels


def document(points=None, faces=None):
    points = points or [(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)]
    faces = faces or [(0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3)]
    return ("# vspgeom v3\n1\n%d %d 0\n" % (len(points), len(faces)) +
            "".join("%g %g %g\n" % tuple(p) for p in points) + "%d\n" % len(faces) +
            "".join(str(len(face)) + " " + " ".join(str(v+1) for v in face) + "\n" for face in faces))


class PanelTopology(unittest.TestCase):
    def read(self, text):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "case.vspgeom"
            path.write_text(text, encoding="ascii")
            return aero_panels.read_topology(path)

    def test_closed_outward_tetrahedron(self):
        row = self.read(document())
        self.assertTrue(row["closed_oriented"])
        self.assertEqual(row["components"], 1)
        self.assertEqual(row["edge_use_counts"], {"2": 6})
        self.assertAlmostEqual(row["signed_volume_m3"], 1/6)
        self.assertLess(row["area_vector_norm_m2"], 1e-15)

    def test_missing_panel_is_reported_as_open(self):
        row = self.read(document(faces=[(0, 2, 1), (0, 1, 3), (0, 3, 2)]))
        self.assertFalse(row["closed_oriented"])
        self.assertEqual(row["boundary_edges"], 3)

    def test_reversed_face_and_inward_shell_are_rejected_without_repair(self):
        for faces in ([(0, 1, 2), (0, 1, 3), (0, 3, 2), (1, 2, 3)],
                      [(0, 1, 2), (0, 3, 1), (0, 2, 3), (1, 3, 2)]):
            row = self.read(document(faces=faces))
            self.assertFalse(row["closed_oriented"])

    def test_disconnected_closed_shells_do_not_pass_assembly(self):
        points = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1),
                  (3, 0, 0), (4, 0, 0), (3, 1, 0), (3, 0, 1)]
        faces = [(0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3)]
        row = self.read(document(points, faces + [tuple(v+4 for v in face) for face in faces]))
        self.assertEqual(row["components"], 2)
        self.assertFalse(row["closed_oriented"])

    def test_edge_paired_but_self_crossing_polygon_fails(self):
        points = [(0, 0, 0), (1, 1, 0), (0, 1, 0), (2, 0, 0), (.5, .5, 1)]
        faces = [(0, 1, 2, 3), (1, 0, 4), (2, 1, 4), (3, 2, 4), (0, 3, 4)]
        with self.assertRaisesRegex(ValueError, "self-crossing"):
            self.read(document(points, faces))

    def test_bad_prefix_and_connectivity_fail(self):
        valid = document()
        for text in (valid.replace("v3", "v2"), valid.replace("\n1\n", "\n2\n", 1),
                     valid.replace("4 4 0", "4 5 0"), valid.replace("3 1 3 2", "3 1 3 5"),
                     valid.replace("3 1 3 2", "3 1 1 2"), valid.replace("1 0 0", "nan 0 0"),
                     "\n".join(valid.splitlines()[:-1])):
            with self.subTest(text=text), self.assertRaises(ValueError):
                self.read(text)


if __name__ == "__main__":
    unittest.main()
