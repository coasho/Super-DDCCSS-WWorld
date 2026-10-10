"""The air-to-air missiles as hangar designs (hangar/missiles.py, ADR-30 MS-7e1): each design is what the table writes,
holds its card's published figures, and carries its motor to the viewer; the showcase's script launches it from a
station its carrier's loadouts put it on."""
import json
import os
import unittest

import numpy as np

from hangar import missiles, model3d, pipeline, showcase
from hangar.geometry.aircraft import Aircraft, Motor
from hangar.pipeline import missile_span


def _design(name):
    return os.path.join(pipeline.repo_root(), "aircraft", name, name + ".toml")


class Missiles(unittest.TestCase):
    def test_written_as_committed(self):
        # the table is how each design was made: the committed files are what it writes
        for name in missiles.MISSILES:
            toml, paint = missiles.design(name)
            with open(_design(name), encoding="utf-8") as f:
                self.assertEqual(f.read(), toml, "%s is not what `python -m hangar missiles %s` writes" % (_design(name), name))
            with open(os.path.join(os.path.dirname(_design(name)), "paint.toml"), encoding="utf-8") as f:
                self.assertEqual(f.read(), paint, name)

    def test_card_figures_held(self):
        for name, row in missiles.MISSILES.items():
            a = Aircraft.load(_design(name))
            self.assertEqual(a.category, "missile")
            with open(os.path.join(pipeline.repo_root(), "assets", "stores", row["card"] + ".json"), encoding="utf-8") as f:
                par = json.load(f)["parameters"]
            body = next(b for b in a.bodies if b.name == "body")
            nose = min(float(b.x[0]) for b in a.bodies)  # (an infrared seeker's dome ahead of the body)
            self.assertAlmostEqual(float(body.x[-1]) - nose, par["length_m"]["value"], places=3, msg=name)
            self.assertAlmostEqual(float(2.0 * max(body.top)), par["diameter_m"]["value"], places=3, msg=name)
            if "span_m" in par:  # tip to tip across opposite fins
                self.assertAlmostEqual(missile_span(a), par["span_m"]["value"], delta=0.005 * par["span_m"]["value"], msg=name)
            self.assertEqual(a.spec["mass"]["empty"], par["mass_kg"]["value"], name)
            # every missile moves something: its fins, canards or wings, or its tail's ailerons
            self.assertTrue(any(s.controls for s in a.surfaces), name)

    def test_motor_to_the_viewer(self):
        a = Aircraft.load(_design("aim120c"))
        self.assertEqual(a.motor.smoke, "reduced")
        lines = model3d.engine_lines(a)
        rocket = [ln for ln in lines if ln.startswith("rocket ")]
        self.assertEqual(len(rocket), 1)
        _, index, smoke, exit_d, thrust = rocket[0].split()
        self.assertEqual((index, smoke), ("0", "reduced"))
        self.assertAlmostEqual(float(exit_d), a.motor.exit_diameter, places=4)
        self.assertAlmostEqual(float(thrust), max(t for _, t in a.motor.phases), places=3)
        self.assertFalse(any(ln.startswith("engine ") for ln in lines))  # no jet's line for it
        for bad in ({"exit": [1, 0, 0]}, {"exit": [1, 0, 0], "exit_diameter": 0.1, "smoke": "sooty"},
                    {"exit": [1, 0, 0], "exit_diameter": 0.1, "throat_diameter": 0.2}):
            with self.assertRaises(ValueError):
                Motor(bad)

    def test_showcase_script(self):
        s = showcase.script("aim120c")
        m = s["missiles"][0]
        # the F-16C's loadouts put the AIM-120C on its wingtip rails (stations 1 and 9): a rail launch
        self.assertIn(m["station_number"], (1, 9))
        self.assertEqual(m["launch"], "rail")
        self.assertEqual(m["type"], "jsbsim:aim120c")
        a = Aircraft.load(_design("aim120c"))
        self.assertEqual(m["phases"], [[d, 1000.0 * t] for d, t in a.motor.phases])
        self.assertEqual(m["fin_deg"], 20.0)
        # no dive, no ground: it flies its manoeuvre to the recording's end
        self.assertNotIn("dive_s", m)
        self.assertNotIn("ground", s)

    def test_showcase_detonation(self):
        # its warhead goes off detonate_s after its launch: the card's warhead mass and its charge (the card's
        # explosive_kg, MS-7g's G1)
        s = showcase.script("aim120c", detonate_s=6.0)
        m = s["missiles"][0]
        self.assertEqual(m["detonate_s"], 6.0)
        self.assertEqual(m["warhead_kg"], 20.0)
        self.assertAlmostEqual(m["explosive_kg"], 7.17)

    def test_showcase_dive_to_the_ground(self):
        # a dive: it ends where it strikes the map set's terrain, as World ends a weapon
        s = showcase.script("aim9m", "fa18c", dive_s=4.0, dive_deg=-25.0)
        m = s["missiles"][0]
        self.assertEqual((m["dive_s"], m["dive_deg"]), (4.0, -25.0))
        self.assertIs(s["ground"], True)


class Lattice(unittest.TestCase):
    def test_grid_fins(self):
        from hangar.shape import lattice
        a = Aircraft.load(_design("r77m"))
        fins = [s for s in a.surfaces if s.lattice]
        self.assertEqual(len(fins), 2)  # the upper and lower pairs of its X, each mirrored
        self.assertEqual(a.cell, 0.0015)  # (its [model] cell: walls a few cells thick)
        scene = lattice.lattice(fins[0], 0)
        frame, grid = scene["children"]
        self.assertEqual(frame["op"], "subtract")  # a box less its inside
        outer, hole = frame["a"], frame["b"]
        self.assertAlmostEqual(outer["half"][2], 0.5 * fins[0].lattice["width"])
        self.assertAlmostEqual(outer["half"][2] - hole["half"][2], lattice.FRAME)
        walls = grid["children"][0]["children"]
        self.assertGreaterEqual(len(walls), 2 * 3)  # two families, at least a cell's worth each way
        # the walls at 45 deg across the frame's face, along the flow as deep as the fin's chord
        s_axis, u_axis = np.array(outer["axes"][1]), np.array(outer["axes"][2])
        along = np.array(walls[0]["axes"][1])
        self.assertAlmostEqual(abs(float(along @ s_axis)), np.sqrt(0.5), places=6)
        self.assertAlmostEqual(abs(float(along @ u_axis)), np.sqrt(0.5), places=6)
        self.assertAlmostEqual(walls[0]["half"][0], outer["half"][0])
        self.assertTrue(all(w["mirror"] for w in walls))  # the defined half's, mirrored as the surface is
        self.assertFalse(lattice.lattice(fins[0], 0, mirror=False)["children"][0]["a"]["mirror"])


if __name__ == "__main__":
    unittest.main()
