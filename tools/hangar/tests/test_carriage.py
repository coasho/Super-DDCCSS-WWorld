"""The carriage on the stations (hangar/carriage.py): the stations read from the store-station card, sized for what the
loadouts put there; a pylon reaching up to the skin, a rack filling a tight gap, nothing under a wingtip's own launcher
or in a bay; the stations' lines in the manifest."""
import os
import unittest

import numpy as np

from hangar import carriage, pipeline
from hangar.geometry.aircraft import Aircraft


def _design(name):
    return Aircraft.load(os.path.join(pipeline.repo_root(), "aircraft", name, name + ".toml"))


def _plate(z, half=50.0):
    """A horizontal square of skin at height z (two triangles)."""
    p = np.array([[-half, -half, z], [half, -half, z], [half, half, z], [-half, half, z]], float)
    return p, np.array([[0, 1, 2], [0, 2, 3]])


class Carriage(unittest.TestCase):
    def test_stations_from_the_card(self):
        st = {s["number"]: s for s in carriage.stations(_design("f16c"))}
        self.assertEqual(sorted(st), list(range(1, 10)))
        # the loadouts put AIM-120Cs on the wingtips and stations 3 and 7, AIM-9Xs on 2 and 8: rails sized for them
        self.assertEqual(st[9]["kind"], "rail")
        self.assertAlmostEqual(st[9]["length"], 3.66, places=2)
        self.assertAlmostEqual(st[2]["length"], 3.018, places=3)
        self.assertEqual(st[5]["kind"], "ejector")  # the centreline's rack (a Mk 82's, the loadouts')
        self.assertFalse(any(s["bay"] for s in st.values()))
        # body axes from the CG into the design frame: x aft, z up
        self.assertAlmostEqual(st[9]["at"][1], 4.75)
        bays = [s for s in carriage.stations(_design("f22a")) if s["bay"]]
        self.assertEqual(len(bays), 8)  # the F-22A's main and side bays

    def test_skin_above(self):
        p, t = _plate(2.0)
        self.assertAlmostEqual(carriage._skin_above(p, t, 0.0, 0.0, 1.0), 2.0)
        self.assertIsNone(carriage._skin_above(p, t, 0.0, 0.0, 2.5))  # above it
        self.assertIsNone(carriage._skin_above(p, t, 80.0, 0.0, 1.0))  # beside it

    def test_parts_reach_the_skin(self):
        a = _design("f16c")
        st = {s["number"]: s for s in carriage.stations(a)}
        # the wing's underside 0.6 m above station 3's missile's top: a rail and a pylon up into it
        top = st[3]["at"][2] + 0.5 * st[3]["diameter"]
        p, t = _plate(top + 0.6)
        pieces = dict(carriage.parts(a, p, t))
        self.assertEqual(len(pieces[3]), 2)
        pylon = pieces[3][1][0]
        self.assertAlmostEqual(pylon[:, 2].max(), top + 0.6 + carriage.EMBED, places=6)
        self.assertAlmostEqual(pieces[3][0][0][:, 2].min(), top, places=6)  # the rail on the missile's top
        # close above: a wingtip's rail is the airframe's own (nothing), a rack fills the gap
        p, t = _plate(top + 0.05)
        pieces = dict(carriage.parts(a, p, t))
        self.assertNotIn(3, pieces)  # station 3 sized for an AIM-120C: a rail
        top5 = st[5]["at"][2] + 0.5 * st[5]["diameter"]
        p, t = _plate(top5 + 0.10)
        pieces = dict(carriage.parts(a, p, t))
        self.assertEqual(len(pieces[5]), 1)
        self.assertAlmostEqual(pieces[5][0][0][:, 2].max(), top5 + 0.10 + carriage.EMBED, places=6)

    def test_station_lines(self):
        a = _design("f16c")
        from hangar.mass import MassModel
        lines = carriage.station_lines(a, MassModel(a).empty()["cg"])
        rows = [ln.split() for ln in lines if ln.startswith("station ")]
        self.assertEqual(len(rows), 9)
        nine = next(r for r in rows if r[1] == "9")
        self.assertEqual([float(v) for v in nine[2:5]], [-1.758, 4.75, -0.0])  # the card's place, body axes
        self.assertEqual(nine[5], "0")


class Bays(unittest.TestCase):
    def test_bays_from_the_stations(self):
        from hangar.shape import bays
        b = {x["name"]: x for x in bays.bays(_design("f22a"))}
        self.assertEqual(sorted(b), ["left side bay", "main bay", "right side bay"])
        main = b["main bay"]
        self.assertEqual(main["stations"], [4, 5, 6, 7, 8, 9])
        # wide: two doors down its middle, each hinged on its outer edge; a side bay's one, hinged outboard
        self.assertEqual([d[2] for d in main["doors"]], [-1, +1])
        self.assertEqual(len(b["left side bay"]["doors"]), 1)
        self.assertEqual(b["left side bay"]["doors"][0][2], -1)
        self.assertEqual(b["right side bay"]["doors"][0][2], +1)
        # the cavity holds its stores: their length and fins, the margin round them
        x0, x1, y0, y1, z0, z1 = main["box"]
        a = _design("f22a")
        for s in carriage.stations(a):
            if s["number"] in main["stations"]:
                self.assertLess(x0, s["at"][0] - 0.5 * s["length"])
                self.assertGreater(x1, s["at"][0] + 0.5 * s["length"])
                self.assertLess(y0, s["at"][1] - 0.5 * s["span"])
                self.assertGreater(y1, s["at"][1] + 0.5 * s["span"])
                self.assertGreater(z1, s["at"][2])
        # the bays' numbers are the manifest's
        indices = {s["number"]: s["bay_index"] for s in carriage.stations(a)}
        self.assertEqual(indices[5], main["index"])
        self.assertEqual(indices[1], 0)
        self.assertEqual(bays.bays(_design("f16c")), [])  # no bays

    def test_doors_cut_from_the_skin(self):
        from hangar.shape import bays
        skin = {"foils": [], "root": {"prim": "box"}}
        scenes = bays.door_scenes(_design("f22a"), skin, 0)
        self.assertEqual(len(scenes), 4)  # the main bay's two, a side bay's one each
        for bay, edge, scene in scenes:
            self.assertIn(edge, (-1, +1))
            self.assertEqual(scene["root"]["op"], "subtract")  # the skin less its inside: a shell
            self.assertIs(scene["root"]["a"]["children"][0], skin["root"])

    def test_covers_over_the_whole_openings(self):
        # each bay shut, for the radar cross section: the skin's shell over its whole opening, no door gap
        from hangar.shape import bays
        a = _design("f22a")
        skin = {"foils": [], "root": {"prim": "box"}}
        covers = bays.cover_scenes(a, skin, 0)
        bs = {b["index"]: b for b in bays.bays(a)}
        self.assertEqual(sorted(k for k, _ in covers), sorted(bs))  # one a bay
        for k, scene in covers:
            self.assertEqual(scene["root"]["op"], "subtract")
            region = scene["root"]["a"]["children"][1]
            x0, x1, y0, y1, z0, z1 = bs[k]["box"]
            self.assertAlmostEqual(region["centre"][0] - region["half"][0], x0)  # the opening's own edges
            self.assertAlmostEqual(region["centre"][1] + region["half"][1], y1)
        self.assertEqual(bays.cover_scenes(_design("f16c"), skin, 0), [])


if __name__ == "__main__":
    unittest.main()
