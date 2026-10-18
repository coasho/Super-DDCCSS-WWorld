"""Offline reference safeguards; no SDK, solver or network is needed."""
import copy
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock
import contextlib
import io
import json

sys.path.insert(0, str(Path(__file__).resolve().parent))
import reference
import crosscheck


class ReferenceTest(unittest.TestCase):
    def test_ignored_reference_environment_overrides_are_rejected(self):
        desired = {"pressure_pa": 35000.0, "density_kg_m3": 0.5, "temperature_k": 236.15}
        actual = dict(desired, gravity_ms2=reference.radial_gravity(8000))
        self.assertIsNone(reference.verify_environment(actual, desired, spherical=True))
        for key in actual:
            wrong = dict(actual, **{key: actual[key] + 0.01})
            with self.subTest(key=key), self.assertRaises(ValueError):
                reference.verify_environment(wrong, desired, spherical=True)

    def test_radial_reference_gravity_matches_the_declared_planet(self):
        self.assertAlmostEqual(reference.radial_gravity(0), 398600441800000 / 6378137**2)
        self.assertLess(reference.radial_gravity(8000), reference.radial_gravity(0))
        for height in (float("nan"), True, -6378137):
            with self.subTest(height=height), self.assertRaises(ValueError):
                reference.radial_gravity(height)

    def test_campaign_requires_unique_nonempty_models_before_loading_a_runtime(self):
        with mock.patch.object(crosscheck, "runtime", side_effect=AssertionError("must validate first")):
            for models in ([], ["aim120c", "aim120c"]):
                with self.subTest(models=models), self.assertRaises(ValueError):
                    crosscheck.campaign(models, Path("unused"))

    def test_failed_or_interrupted_campaign_cannot_leave_old_passing_evidence(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / "flight.json"
            args = ["--model", "aim120c", "--native-bin", folder, "--output", str(output)]
            for failure in (RuntimeError("native failure"), ZeroDivisionError("bad reference")):
                output.write_text('{"passed":true}', encoding="utf-8")
                with mock.patch.object(crosscheck, "campaign", side_effect=failure), contextlib.redirect_stderr(io.StringIO()):
                    self.assertEqual(crosscheck.main(args), 1)
                self.assertFalse(json.loads(output.read_text(encoding="utf-8"))["passed"])
            output.write_text('{"passed":true}', encoding="utf-8")
            with mock.patch.object(crosscheck, "campaign", side_effect=KeyboardInterrupt), self.assertRaises(KeyboardInterrupt):
                crosscheck.main(args)
            self.assertFalse(json.loads(output.read_text(encoding="utf-8"))["passed"])

    def test_axial_drag_includes_the_emitted_base_coefficient(self):
        xml = '''<fdm_config><aerodynamics><axis name="AXIAL"><function name="aero/coefficient/CA"><product>
          <property>aero/qbar-psf</property><property>metrics/Sw-sqft</property><value>0.3</value>
          <table><independentVar>velocities/mach</independentVar><tableData>0 1\n1 2</tableData></table>
          </product></function></axis></aerodynamics></fdm_config>'''
        self.assertEqual(reference.axial_drag(xml), [[0.0, 0.3], [1.0, 0.6]])
        self.assertEqual(reference.axial_drag(xml.replace('<value>0.3</value>', '')), [[0.0, 1.0], [1.0, 2.0]])
        for bad in (xml.replace('0.3', 'NaN'), xml.replace('0 1', '0 -1'),
                    xml.replace('1 2', '0 2'), xml.replace('velocities/mach', 'unknown'),
                    xml.replace('<value>0.3</value>', '<function/>')):
            with self.subTest(xml=bad), self.assertRaises(ValueError):
                reference.axial_drag(bad)

    def test_generated_inputs_must_match_every_file_before_reference(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / "Engines").mkdir()
            (root / "body.xml").write_text("body\n", encoding="utf-8")
            (root / "Engines/motor.xml").write_text("motor\n", encoding="utf-8")
            generated = {"body.xml": "body\n", "Engines/motor.xml": "motor\n"}
            records = reference.verify_generated(generated, root)
            self.assertEqual(len(records), 2)
            self.assertTrue(all(len(item["sha256"]) == 64 for item in records))
            (root / "Engines/motor.xml").write_text("stale\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "stale"):
                reference.verify_generated(generated, root)
            (root / "Engines/motor.xml").unlink()
            with self.assertRaises(ValueError):
                reference.verify_generated(generated, root)

    def test_generated_paths_are_validated_before_access(self):
        for path in ("../outside.xml", "X:/outside.xml", "/outside.xml", "a/../x.xml", "a\\x.xml"):
            with self.subTest(path=path), self.assertRaises(ValueError):
                reference.verify_generated({path: "value"}, Path("unused"))

    def test_phase_curve_conserves_impulse_and_accounts_for_ignition(self):
        phases = [{"duration_s": 0.5, "thrust_kn": 0.02},
                  {"duration_s": 0.25, "thrust_kn": 0.01}]
        curve = reference.phase_curve(phases, buildup_s=0.05, spacing_s=0.0005)
        impulse = sum((b[0] - a[0]) * (a[1] + b[1]) / 2 for a, b in zip(curve, curve[1:]))
        self.assertAlmostEqual(impulse, 12.5, delta=0.0001)
        self.assertAlmostEqual(curve[-1][0], 0.75 + 0.05 * (1 - 2 / math.pi), delta=1e-7)
        self.assertEqual(curve[0], [0.0, 0.0])
        self.assertEqual(curve[-1][1], 0.0)
        self.assertTrue(all(b[0] > a[0] for a, b in zip(curve, curve[1:])))

    def test_curve_rejects_unsupported_or_unsafe_inputs(self):
        good = [{"duration_s": 1.0, "thrust_kn": 0.02}]
        for phases in ([], [{}], [{"duration_s": True, "thrust_kn": 0.02}],
                       [{"duration_s": 0.01, "thrust_kn": 0.02}],
                       [{"duration_s": 1, "thrust_kn": 0}],
                       [{"duration_s": 1e308, "thrust_kn": 1e308}],
                       [{"duration_s": 1, "thrust_kn": 9e304}] * 2,
                       [{"duration_s": 1, "thrust_kn": float("nan")}],
                       [{"duration_s": 1, "thrust_kn": -1}]):
            with self.subTest(phases=phases), self.assertRaises(ValueError):
                reference.phase_curve(phases)
        for spacing in (0, -1, True, float("inf"), 1e-20):
            with self.subTest(spacing=spacing), self.assertRaises(ValueError):
                reference.phase_curve(good, spacing_s=spacing)

    def test_comparison_requires_full_finite_aligned_nonempty_histories(self):
        good = [[0.0, 3.0, 5.0, 10.0], [0.1, 2.0, 6.0, 11.0]]
        limits = {"mass_kg": 0.01, "speed_ms": 0.1, "height_m": 0.2}
        self.assertTrue(reference.compare(good, good, limits)["passed"])
        for bad in ([], good[:1], [[0, 3, 5, 10], [0, 2, 6, 11]],
                    [[0, 3, 5, 10], [0.11, 2, 6, 11]],
                    [[0, 3, 5, 10], [0.1, True, 6, 11]],
                    [[0, 3, 5, 10], [0.1, float("nan"), 6, 11]],
                    [[0, 3, 5, 10], [0.1, 2, 6]],
                    [[0, 3, 5, 10], [0.1, -1, 6, 11]],
                    [[0, 3, 5, 10], [float("inf"), 2, 6, 11]]):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                reference.compare(good, bad, limits)
        for limits_bad in ({}, {**limits, "speed_ms": float("nan")}, {**limits, "mass_kg": -1}):
            with self.subTest(limits=limits_bad), self.assertRaises(ValueError):
                reference.compare(good, good, limits_bad)

    def test_comparison_fails_a_single_bad_sample_and_preserves_inputs(self):
        native = [[0, 3, 5, 10], [0.1, 2, 6, 11], [0.2, 1, 7, 12]]
        expected = copy.deepcopy(native)
        expected[1][2] = 6.5
        before = copy.deepcopy((native, expected))
        out = reference.compare(native, expected, {"mass_kg": 0.01, "speed_ms": 0.1, "height_m": 0.2})
        self.assertFalse(out["passed"])
        self.assertEqual(out["max_abs"]["speed_ms"], 0.5)
        self.assertEqual(out["samples"], 3)
        self.assertEqual((native, expected), before)


if __name__ == "__main__":
    unittest.main()
