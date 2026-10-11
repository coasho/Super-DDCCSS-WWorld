"""Each airframe's damage data for the world's damage chain (hangar/damage.py, ADR-30, MS-7g's damage part G2): its
sections' shares of the aerodynamic force whole, its sections' strength its own rule's at its ultimate load, the
components' JSBSim indices its JSBSim file's, and each committed file what its design writes now."""
import json
import os
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np

from hangar import damage, pipeline
from hangar.geometry.aircraft import Aircraft
from hangar.mass import MassModel


def _committed():
    root = os.path.join(pipeline.repo_root(), "aircraft")
    out = []
    for name in sorted(os.listdir(root)):
        path = os.path.join(root, name, name + ".damage.json")
        if os.path.isfile(path):
            with open(path, encoding="utf-8") as f:
                out.append((name, json.load(f)))
    return out


class Data(unittest.TestCase):
    def test_every_fixed_wing_design_has_one(self):
        names = {n for n, _ in _committed()}
        root = os.path.join(pipeline.repo_root(), "aircraft")
        for name in sorted(os.listdir(root)):
            toml = os.path.join(root, name, name + ".toml")
            if not os.path.isfile(toml) or not os.path.isfile(os.path.join(root, name, name + ".glb.manifest")):
                continue
            try:
                a = Aircraft.load(toml)
            except (ValueError, AttributeError):  # (a rotorcraft's design: no lifting surfaces)
                continue
            if a.category not in ("missile", "store") and any(s.kind == "wing" for s in a.surfaces):
                self.assertIn(name, names, "%s: no %s.damage.json (fsim hangar %s --stage model)" % (name, name, name))

    def test_shares_whole_and_strengths_positive(self):
        for name, d in _committed():
            with self.subTest(name):
                s = {x["name"]: x for x in d["sections"]}
                self.assertAlmostEqual(sum(x["normal_share"] for x in d["sections"]), 1.0, places=4)  # (written to 5 places)
                self.assertAlmostEqual(sum(x["side_share"] for x in d["sections"]), 1.0, places=4)  # (written to 5 places)
                # the wings carry the most of it, alike each side
                self.assertGreater(s["left wing"]["normal_share"], 0.15)
                self.assertAlmostEqual(s["left wing"]["normal_share"], s["right wing"]["normal_share"], places=4)
                self.assertAlmostEqual(s["left wing"]["strength_nm"][0], s["right wing"]["strength_nm"][0], delta=1e-3 *
                                       s["left wing"]["strength_nm"][0])
                for x in d["sections"]:
                    if x["root"] is not None:
                        self.assertTrue(all(v > 0.0 for v in x["strength_nm"]), x["name"])
                self.assertLess(d["n_limit"][0], 0.0)
                self.assertAlmostEqual(d["n_ultimate"][1], 1.5 * d["n_limit"][1])

    def test_indices_are_the_jsbsim_files(self):
        for name, d in _committed():
            with self.subTest(name):
                xml = os.path.join(pipeline.repo_root(), "aircraft", name, name + ".xml")
                root = ET.parse(xml).getroot()
                engines = len(root.findall("./propulsion/engine"))
                tanks = len(root.findall("./propulsion/tank"))
                for c in d["components"]:
                    if c["kind"] == "engine":
                        self.assertTrue(0 <= c["engine"] < engines, c["name"])
                    if c["kind"] == "fuel":
                        self.assertTrue(0 <= c["tank"] < tanks, c["name"])
                    if c["kind"] == "control":
                        self.assertIn("channel", c)

    def test_a_wing_fails_at_its_ultimate_load(self):
        # the rule the world flies (src/damage): at the design mass, its tanks empty, a symmetric pull of 1.5 n_max
        # bends the wing's root by just its strength
        a = Aircraft.load(os.path.join(pipeline.repo_root(), "aircraft", "f16c", "f16c.toml"))
        d = dict(_committed())["f16c"]
        s = {x["name"]: x for x in d["sections"]}
        roots = {"left wing": s["left wing"]["root"]}
        shares = {"left wing": s["left wing"]}
        masses = {"left wing": s["left wing"]}
        M = damage._bending(roots, shares, masses, {}, d["design_mass_kg"], d["n_ultimate"][1])["left wing"][0]
        self.assertAlmostEqual(abs(M), s["left wing"]["strength_nm"][0], delta=1e-3 * abs(M))
        self.assertEqual(a.name, "f16c")


class Current(unittest.TestCase):
    def test_committed_files_are_what_the_designs_write(self):
        # (two small designs: each committed file what damage.py writes from its design now)
        for name in ("c172", "skua"):
            with self.subTest(name):
                a = Aircraft.load(os.path.join(pipeline.repo_root(), "aircraft", name, name + ".toml"))
                out = os.path.join(os.environ.get("FSIM_TEST_OUTPUT") or tempfile.mkdtemp(), name + ".damage.json")
                os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
                damage.write(a, out, MassModel(a).empty()["cg"])
                with open(out, encoding="utf-8") as f, \
                        open(os.path.join(pipeline.repo_root(), "aircraft", name, name + ".damage.json"), encoding="utf-8") as g:
                    self.assertEqual(f.read(), g.read(), "%s.damage.json is stale: fsim hangar %s --stage model" % (name, name))
                self.assertTrue(np.isfinite(MassModel(a).empty()["mass"]))


if __name__ == "__main__":
    unittest.main()


class Pieces(unittest.TestCase):
    def test_each_parting_section_has_its_flight_model(self):
        # each section that can part (pieces.PARTS, with a root and a mass) a JSBSim definition beside its aircraft,
        # what its damage data writes now (pieces.py)
        from hangar import pieces
        for name, d in _committed():
            with self.subTest(name):
                for s in d["sections"]:
                    if s["name"] not in pieces.PARTS:
                        continue
                    text = pieces.piece_xml(d, s["name"])
                    path = os.path.join(pipeline.repo_root(), "aircraft", name, "pieces", pieces.piece_name(name, s["name"]),
                                        pieces.piece_name(name, s["name"]) + ".xml")
                    if text is None:
                        self.assertFalse(os.path.isfile(path), path)
                        continue
                    self.assertTrue(os.path.isfile(path), "%s: no %s (fsim hangar %s --stage model)" % (name, path, name))
                    with open(path, encoding="utf-8") as f:
                        self.assertEqual(f.read(), text, "%s is stale" % path)
                    ET.fromstring(text.encode("utf-8"))  # (well formed)
