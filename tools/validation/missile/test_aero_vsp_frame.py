"""Rigid-frame covariance regressions; no optional solver dependencies."""
import importlib.util
import math
from pathlib import Path
import unittest


_SPEC = importlib.util.spec_from_file_location("missile_aero_vsp_frame", Path(__file__).with_name("aero_vsp_frame.py"))
frame = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(frame)


def component(identity="body", origin=(1, 2, 3), rotation=(17, 0, 0), parent=""):
    return {"id": identity, "parent_id": parent, "origin_xyz": list(origin), "rotation_xyz": list(rotation)}


class RigidVspFrame(unittest.TestCase):
    def assertVectorAlmostEqual(self, actual, expected, places=13):
        self.assertEqual(len(actual), len(expected))
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, places=places)

    def test_two_physical_conditions_stay_paired_with_native_wind(self):
        cases = frame.paired_conditions(.123)
        self.assertEqual(len(cases), 2)
        self.assertEqual([case["physical"]["alpha_deg"] for case in cases], [0, 2])
        self.assertEqual(cases[0]["native"]["alpha_deg"], 0)
        self.assertEqual(cases[0]["native"]["beta_deg"], 0)
        self.assertGreater(cases[1]["native"]["beta_deg"], 0)
        for case in cases:
            physical, native = case["physical"], case["native"]
            self.assertEqual((physical["mach"], physical["beta_deg"], native["mach"]), (.6, 0, .6))
            alpha, beta = math.radians(native["alpha_deg"]), math.radians(native["beta_deg"])
            actual_native_wind = [math.cos(alpha)*math.cos(beta), -math.sin(beta), math.sin(alpha)*math.cos(beta)]
            self.assertVectorAlmostEqual(native["wind_xyz"], actual_native_wind)
            self.assertVectorAlmostEqual(frame.rotate_vector(physical["wind_xyz"], .123), actual_native_wind)

    def test_quarter_turn_exposes_native_sideslip_sign(self):
        case = frame.paired_conditions(90)[1]
        self.assertAlmostEqual(case["native"]["alpha_deg"], 0, places=13)
        self.assertAlmostEqual(case["native"]["beta_deg"], 2, places=13)
        self.assertVectorAlmostEqual(case["native"]["wind_xyz"], [math.cos(math.radians(2)), -math.sin(math.radians(2)), 0])

    def test_every_component_origin_and_orientation_rotates_without_mutation(self):
        original = [component(), component("fin", (4, -2, 1), (-135, 0, 0))]
        transformed = frame.roll_component_frames(original, 90)
        self.assertVectorAlmostEqual(transformed[0]["origin_xyz"], [1, -3, 2])
        self.assertVectorAlmostEqual(transformed[1]["origin_xyz"], [4, -1, -2])
        self.assertEqual(transformed[0]["rotation_xyz"], [107, 0, 0])
        self.assertEqual(transformed[1]["rotation_xyz"], [-45, 0, 0])
        self.assertEqual(original, [component(), component("fin", (4, -2, 1), (-135, 0, 0))])
        self.assertAlmostEqual(math.dist(original[0]["origin_xyz"], original[1]["origin_xyz"]),
                               math.dist(transformed[0]["origin_xyz"], transformed[1]["origin_xyz"]), places=13)

    def test_cg_and_mesh_point_rotate_about_the_same_origin(self):
        cg, point = [1.83, .1, -.2], [2.5, -.7, .3]
        rotated_cg, rotated_point = frame.rotate_vector(cg, .123), frame.rotate_vector(point, .123)
        difference = [p-c for p, c in zip(rotated_point, rotated_cg)]
        self.assertVectorAlmostEqual(difference, frame.rotate_vector([p-c for p, c in zip(point, cg)], .123))

    def test_unsupported_later_component_rejects_entire_batch(self):
        first, bad = component(), component("child", parent="body")
        before = first["origin_xyz"][:], first["rotation_xyz"][:]
        with self.assertRaises(ValueError):
            frame.roll_component_frames([first, bad], .123)
        self.assertEqual((first["origin_xyz"], first["rotation_xyz"]), before)
        for rotation in ((0, 1, 0), (0, 0, -1)):
            with self.assertRaises(ValueError):
                frame.roll_component_frames([component(rotation=rotation)], .123)

    def test_component_identity_and_complete_numeric_frames_are_required(self):
        for components in ([], [component(), component()], [component(identity="")],
                           [component(origin=(1, 2))], [component(rotation=(math.nan, 0, 0))]):
            with self.subTest(components=components), self.assertRaises(ValueError):
                frame.roll_component_frames(components, .123)

    def test_all_six_loads_recover_with_unequal_moment_reference_lengths(self):
        native = {"CFix": 1, "CFiy": -3, "CFiz": 2, "CMix": 4, "CMiy": -4, "CMiz": 7.5}
        result = frame.recover_loads(native, 90, bref=2, cref=3)
        self.assertEqual(result["native"], native)
        self.assertVectorAlmostEqual([result["original"][key] for key in native], [1, 2, 3, 4, 5, 6])
        self.assertEqual(native, {"CFix": 1, "CFiy": -3, "CFiz": 2, "CMix": 4, "CMiy": -4, "CMiz": 7.5})

    def test_force_wind_projection_is_invariant_for_each_paired_condition(self):
        original_force = [.19, -.31, .47]
        native_force = frame.rotate_vector(original_force, .123)
        raw = dict(zip(("CFix", "CFiy", "CFiz", "CMix", "CMiy", "CMiz"), native_force + [.2, -.1, .4]))
        recovered = frame.recover_loads(raw, .123, bref=.1778, cref=.1778)
        self.assertVectorAlmostEqual([recovered["original"][key] for key in ("CFix", "CFiy", "CFiz")], original_force)
        for case in frame.paired_conditions(.123):
            drag_original = sum(f*w for f, w in zip(original_force, case["physical"]["wind_xyz"]))
            drag_native = sum(f*w for f, w in zip(native_force, case["native"]["wind_xyz"]))
            self.assertAlmostEqual(drag_original, drag_native, places=14)

    def test_malformed_loads_and_references_fail_explicitly(self):
        good = dict.fromkeys(("CFix", "CFiy", "CFiz", "CMix", "CMiy", "CMiz"), 1)
        for raw in ({}, {**good, "extra": 0}, {**good, "CMiz": math.inf}, {**good, "CFiy": True}):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                frame.recover_loads(raw, .123, bref=1, cref=1)
        for bref, cref in ((0, 1), (1, -1), (1, math.nan)):
            with self.subTest(bref=bref, cref=cref), self.assertRaises(ValueError):
                frame.recover_loads(good, .123, bref=bref, cref=cref)

    def test_nonfinite_roll_or_vector_is_never_silently_accepted(self):
        for roll in (math.nan, math.inf, True):
            with self.subTest(roll=roll), self.assertRaises(ValueError):
                frame.paired_conditions(roll)
        for vector in ([1, 2], [1, 2, 3, 4], [1, 2, math.inf], [True, 0, 0], "123"):
            with self.subTest(vector=vector), self.assertRaises(ValueError):
                frame.rotate_vector(vector, .123)


if __name__ == "__main__":
    unittest.main()
