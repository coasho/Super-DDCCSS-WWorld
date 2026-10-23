"""Each airframe's damage data for the world's damage chain (hangar/damage.py, ADR-30, MS-7g's damage part G2): its
sections' shares of the aerodynamic force whole, its sections' strength its own rule's at its ultimate load, the
components' JSBSim indices its JSBSim file's, and each committed file what its design writes now."""
import json
import os
import tempfile
import unittest
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np

from hangar import damage, pipeline
from hangar.geometry.aircraft import Aircraft
from hangar.geometry.body import Body
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
        from hangar.aero.model import AeroModel
        for name in ("c172", "skua"):
            with self.subTest(name):
                a = Aircraft.load(os.path.join(pipeline.repo_root(), "aircraft", name, name + ".toml"))
                out = os.path.join(os.environ.get("FSIM_TEST_OUTPUT") or tempfile.mkdtemp(), name + ".damage.json")
                os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
                # The model stage passes the aerodynamic model for an aircraft with a wing
                # (pipeline _components): the side centres of pressure come from it.
                model = AeroModel(a) if any(s.kind == "wing" for s in a.surfaces) else None
                damage.write(a, out, MassModel(a).empty()["cg"], model=model)
                with open(out, encoding="utf-8") as f, \
                        open(os.path.join(pipeline.repo_root(), "aircraft", name, name + ".damage.json"), encoding="utf-8") as g:
                    self.assertEqual(f.read(), g.read(), "%s.damage.json is stale: fsim hangar %s --stage model" % (name, name))
                self.assertTrue(np.isfinite(MassModel(a).empty()["mass"]))


class ContactGeometry(unittest.TestCase):
    @staticmethod
    def cone(mirror=False, translated=False):
        body = Body({"name": "analytic cone", "mirror": mirror,
                     "origin": [3, 4, 2] if translated else [0, 0, 0],
                     "stations": [{"x": 0, "w": 0, "top": 0, "bottom": 0},
                                  {"x": 2, "w": 4, "top": 2, "bottom": -2}]})
        aircraft = SimpleNamespace(bodies=[body])
        origin = np.array([1.25, .7, -.3]) if translated else np.zeros(3)
        panels = []
        for k in range(2):
            for sign in body.copies():
                centre = np.array([body.x[0] + k + .5, sign * body.y[0], .5 * (body.top[0] + body.bottom[0])])
                panels.append({"kind": "body", "body": body.name, "section": "left wing" if sign < 0 else "right wing",
                               "centre": damage._body_axes(centre, origin).tolist(), "half": [.5, k + .5, k + .5]})
        return aircraft, origin, panels

    def assert_closed(self, mesh):
        vertices = np.asarray(mesh["vertices"])
        self.assertEqual(len(vertices), len(np.unique(vertices, axis=0)))
        edges = {}
        for triangle in mesh["triangles"]:
            self.assertTrue(all(isinstance(i, int) and 0 <= i < len(vertices) for i in triangle))
            a, b, c = vertices[triangle]
            self.assertGreater(np.linalg.norm(np.cross(b - a, c - a)), 0)
            for a, b in zip(triangle, np.roll(triangle, -1)):
                key = tuple(sorted((int(a), int(b))))
                n, winding = edges.get(key, (0, 0))
                edges[key] = n + 1, winding + (1 if a < b else -1)
        self.assertTrue(edges)
        self.assertTrue(all(n == 2 and winding == 0 for n, winding in edges.values()))
        self.assertEqual(len(mesh["triangle_neighbors"]), len(mesh["triangles"]))

    def test_tapered_contact_preserves_the_authored_tip_and_closed_cut(self):
        aircraft, origin, panels = self.cone()
        before = json.loads(json.dumps(panels))
        contact = damage.contact_geometry(aircraft, origin, panels)
        self.assertEqual(panels, before)
        self.assertEqual(contact["version"], 1)
        self.assertEqual([m["index"] for m in contact["meshes"]], [0, 1])
        for mesh in contact["meshes"]:
            self.assert_closed(mesh)
            other = 1 - mesh["index"]
            cuts = [(triangle, neighbor) for triangle, neighbor in zip(mesh["triangles"], mesh["triangle_neighbors"])
                    if neighbor is not None]
            self.assertTrue(cuts)
            for triangle, neighbor in cuts:
                self.assertEqual(neighbor, {"kind": "panel", "index": other})
                np.testing.assert_array_equal(np.asarray(mesh["vertices"])[triangle, 0], -np.ones(3))
        first = np.asarray(contact["meshes"][0]["vertices"])
        tip = first[first[:, 0] == 0]
        self.assertGreater(len(tip), 0)
        np.testing.assert_array_equal(tip[:, 1:], np.zeros_like(tip[:, 1:]))
        self.assertEqual(float(first[:, 0].min()), -1)
        # Every vertex lies in the analytic cone r<=-x; a midpoint box's
        # fabricated wide front corners would violate this independent bound.
        self.assertTrue(np.all(np.hypot(first[:, 1], first[:, 2]) <= -first[:, 0] + 2e-5))
        self.assertGreater(float(np.abs(first[:, 1]).max()), panels[0]["half"][1])

    def test_mirrored_translated_volumes_keep_their_own_panel_identity(self):
        aircraft, origin, panels = self.cone(mirror=True, translated=True)
        contact = damage.contact_geometry(aircraft, origin, panels)
        self.assertEqual([m["index"] for m in contact["meshes"]], [0, 1, 2, 3])
        for mesh in contact["meshes"]:
            self.assert_closed(mesh)
            vertices = np.asarray(mesh["vertices"])
            source = panels[mesh["index"]]
            # Both source bodies remain separated by a real gap. Mirroring is
            # about design y=0, before the translated model-origin transform.
            if source["section"] == "left wing":
                self.assertLess(float(vertices[:, 1].max()), -origin[1])
            else:
                self.assertGreater(float(vertices[:, 1].min()), -origin[1])
            self.assertAlmostEqual(float(vertices[:, 0].mean()), source["centre"][0], delta=.6)
            for neighbor in mesh["triangle_neighbors"]:
                if neighbor is not None:
                    self.assertEqual(neighbor["kind"], "panel")
                    self.assertEqual(neighbor["index"] % 2, mesh["index"] % 2)
                    self.assertEqual(panels[neighbor["index"]]["section"], source["section"])

    def test_contact_generation_uses_no_aero_or_mass_recomputation(self):
        aircraft, origin, panels = self.cone()
        # A minimal geometry-only object has no model/mass/strength interface.
        first = damage.contact_geometry(aircraft, origin, panels)
        self.assertEqual(first, damage.contact_geometry(aircraft, origin, panels))

    def test_authored_blunt_end_caps_remain_physical_skin(self):
        body = Body({"name": "closed cylinder", "stations": [
            {"x": 0, "w": 2, "top": 1, "bottom": -1},
            {"x": 2, "w": 2, "top": 1, "bottom": -1}]})
        aircraft = SimpleNamespace(bodies=[body])
        panels = [{"kind": "body", "body": body.name, "section": "centre"},
                  {"kind": "body", "body": body.name, "section": "tail"}]
        geometry = damage.contact_geometry(aircraft, np.zeros(3), panels)
        for mesh in geometry["meshes"]:
            self.assert_closed(mesh)
            vertices = np.asarray(mesh["vertices"])
            physical_end = 0 if mesh["index"] == 0 else -2
            ends = [(triangle, neighbor) for triangle, neighbor in zip(mesh["triangles"], mesh["triangle_neighbors"])
                    if np.all(vertices[triangle, 0] == physical_end)]
            self.assertTrue(ends)
            self.assertTrue(all(neighbor is None for _, neighbor in ends))


if __name__ == "__main__":
    unittest.main()


class Pieces(unittest.TestCase):
    def test_native_mass_properties_preserve_source_precision(self):
        from hangar import pieces
        section = {
            "name": "left wing", "root": {"at": [0, 0, 0]},
            "mass_kg": 1266.87257, "cg": [-1.01215, -2.08348, .30797],
            "inertia": [2608.58879, 2296.76227, 3189.99767, 768.86901, 298.21195, 409.00528],
            "tanks": [{"centre": [-.8123456789, -2.234567891, .456789123]}],
            "stations": [{"number": 1, "at": [-2.008, -4.544, .354]},
                         {"number": 2, "at": [-1.034, -2.982, .318]}],
            "skin_area_m2": 4,
        }
        origin = [9.262, 0, -.35]
        data = {"aircraft": "synthetic", "origin_design_m": origin, "sections": [section],
                "panels": [{"section": "left wing", "kind": "surface", "area_m2": 4,
                            "corners": [[-2, -3, 0], [0, -3, 0], [0, -1, 0], [-2, -1, 0]]}]}
        mass = ET.fromstring(pieces.piece_xml(data, "left wing")).find("mass_balance")
        self.assertAlmostEqual(float(mass.find("emptywt").text) * .45359237,
                               section["mass_kg"], delta=1e-12)
        # These are structural plain products, with X/Z reversed from body
        # coordinates. Conversion precision must retain every tensor entry.
        expected_inertia = section["inertia"][:3] + [-section["inertia"][3],
                                                    section["inertia"][4], -section["inertia"][5]]
        for name, expected in zip(("ixx", "iyy", "izz", "ixy", "ixz", "iyz"), expected_inertia):
            self.assertAlmostEqual(float(mass.find(name).text) * 1.3558179483314, expected, delta=1e-11)
        expected_locations = [(mass.find("location"), section["cg"]),
                              (mass.find("pointmass/location"), section["tanks"][0]["centre"])]
        for point, station in zip(mass.findall("pointmass")[1:], section["stations"]):
            expected_locations.append((point.find("location"), station["at"]))
        for location, body_point in expected_locations:
            expected = [origin[0]-body_point[0], origin[1]+body_point[1], origin[2]-body_point[2]]
            for axis, value in zip(("x", "y", "z"), expected):
                self.assertAlmostEqual(float(location.find(axis).text) * .0254, value, delta=1e-13)

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
