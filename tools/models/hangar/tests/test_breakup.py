"""The breakup sections (hangar/shape/breakup.py): which section a point of an airframe is in, by its geometry; the caps
that close a section over its break, found on the welded mesh (a painted mesh's seams are no break)."""
import os
import types
import unittest

import numpy as np

from hangar import pipeline
from hangar.geometry.aircraft import Aircraft
from hangar.shape import breakup


def _design(name):
    return Aircraft.load(os.path.join(pipeline.repo_root(), "aircraft", name, name + ".toml"))


def _cube():
    v = np.array([[x, y, z] for x in (0.0, 1.0) for y in (0.0, 1.0) for z in (0.0, 1.0)], float)
    q = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    t = [(a, b, c) for a, b, c, d in q] + [(a, c, d) for a, b, c, d in q]
    return v, np.array(t)


class Breakup(unittest.TestCase):
    def test_sections_of_a_fighter(self):
        a = _design("f16c")
        name = lambda p: breakup.SECTIONS[int(breakup.classify(a, np.array([p], float))[0])]
        self.assertEqual(name([1.0, 0.0, -0.3]), "nose")         # the radome
        self.assertEqual(name([7.5, 0.0, 0.0]), "centre")        # over the wing's root
        self.assertEqual(name([8.5, 3.5, 0.0]), "right wing")
        self.assertEqual(name([8.5, -3.5, 0.0]), "left wing")
        self.assertEqual(name([12.5, 0.0, 2.0]), "tail")         # the fin
        self.assertEqual(name([12.0, 2.0, -0.2]), "tail")        # a stabilator, outboard as a wing's tip is
        # what hangs from a wing goes with it (mounted): a pylon 0.5 m under it
        pylon = np.array([[8.8, 3.05, -0.5]])
        self.assertEqual(breakup.SECTIONS[int(breakup.classify(a, pylon)[0])], "centre")
        self.assertEqual(breakup.SECTIONS[int(breakup.classify(a, pylon, mounted=True)[0])], "right wing")

    def test_no_wing_no_sections(self):
        self.assertIsNone(breakup.classify(types.SimpleNamespace(surfaces=[], bodies=[]), np.zeros((1, 3))))

    def test_caps_close_a_break(self):
        v, t = _cube()
        top = [k for k, tri in enumerate(t) if all(v[i][2] == 1.0 for i in tri)]
        open_box = np.delete(t, top, axis=0)  # the box without its lid: one torn loop round its top
        cv, ct, centre = breakup.caps(v, open_box)
        self.assertEqual(len(ct), 4)  # a fan over the four edges
        np.testing.assert_allclose(centre, [0.5, 0.5, 1.0])
        # the cap closes it: every edge of box and cap shared by two triangles, wound opposite ways
        allv = np.concatenate([v, cv])
        allt = np.concatenate([open_box, ct + len(v)])
        _, canon = np.unique(allv, axis=0, return_inverse=True)
        edges = {}
        for a, b, c in canon.reshape(-1)[allt]:
            for e in ((a, b), (b, c), (c, a)):
                edges[e] = edges.get(e, 0) + 1
        self.assertTrue(all(edges.get((b, a)) == 1 for (a, b) in edges))
        # the whole box: no break, no cap - nor where its vertices are split along a seam (a painted mesh's)
        self.assertIsNone(breakup.caps(v, t)[2])
        split_v = np.concatenate([v, v])
        split_t = t.copy()
        split_t[:6] += 8  # half its faces on a copy of its vertices
        self.assertIsNone(breakup.caps(split_v, split_t)[2])


if __name__ == "__main__":
    unittest.main()
