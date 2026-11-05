"""Captured-data integration math tests; a complete actual24 PRE population is required.

Set FSIM_DIRECTIONAL_CONTROL_CAPTURE_ROOT to a deliberate capture epoch. Without
it, use the normal UCRT64 Release full-chain output, never a historical archive.
Missing population is an error, not a skipped or synthetic comparison.
"""
from __future__ import annotations

import copy
import json
import os
from pathlib import Path
import unittest
from unittest import mock

import numpy as np
from scipy.signal import cont2discrete, lfilter, place_poles

import directional_control_theory as theory


ROOT = Path(__file__).absolute().parents[3]


def packets():
    ledger = json.loads(theory.guarded(ROOT / theory.MATRIX_PATH).read_text(encoding="utf-8"))
    capture = theory.capture_root_from_environment(ROOT)
    for cell in ledger["cells"]:
        if cell["component"] != "autopilot":
            continue
        observed, _ = theory.load_first_pre(ROOT, capture, cell["model"])
        for packet in observed:
            yield cell["model"], packet


class DirectionalMathematics(unittest.TestCase):
    def test_trim_matches_independent_three_equation_newton_euler_solve(self):
        # Solve force, moment/Euler torque, and incidence kinematics together.
        # This reference never evaluates the production eliminated determinant.
        count = 0
        for model, packet in packets():
            for name, sign, force_axis, moment_axis in (("pitch", 1, 2, 1), ("yaw", -1, 1, 2)):
                with self.subTest(model=model, sequence=packet["sequence"], axis=name):
                    i = packet["input"]
                    a = i[name]
                    residual = theory.residuals(i, name)
                    mass, speed = i["mass_kg"], i["speed_m_s"]
                    demand, gravity = i["demand_body_m_s2"], i["gravity_body_m_s2"]
                    force_x = i["measured"]["force_body_n"][0]
                    matrix = np.array([
                        [a["force_angle_n_rad"] - force_x + mass * demand[0], 0, a["force_control_n_rad"]],
                        [a["moment_angle_nm_rad"], a["moment_rate_nm_s_rad"], a["moment_control_nm_rad"]],
                        [-sign * (demand[0] + gravity[0]) / speed, 1, 0],
                    ])
                    rhs = [mass * demand[force_axis] - residual["force"],
                           -residual["moment"] + residual["gyro"],
                           -sign * (demand[force_axis] + gravity[force_axis]) / speed]
                    reference = np.linalg.solve(matrix, rhs)
                    actual = theory.directional_trim(i, name)
                    np.testing.assert_allclose([actual["angle_rad"], actual["path_rate_rad_s"], actual["surface_rad"]],
                                               reference, rtol=2e-11, atol=2e-11)
                    count += 1
        self.assertGreaterEqual(count, 48)

    def test_state_matrix_matches_finite_difference_of_body_velocity_equations(self):
        # Derive incidence rate directly from body Newton equations, including
        # body rotation and gravity, then differentiate; do not reuse A00.
        for model, packet in packets():
            i = packet["input"]
            for name, sign, faxis, maxis in (("pitch", 1, 2, 1), ("yaw", -1, 1, 2)):
                with self.subTest(model=model, axis=name):
                    a = i[name]
                    mass, speed, inertia = i["mass_kg"], i["speed_m_s"], i["inertia_kg_m2"][maxis]
                    fx = i["measured"]["force_body_n"][0]
                    gravity = i["gravity_body_m_s2"]
                    tau = i["actuator_lag_s"]

                    def physical(state):
                        angle, rate, surface = state
                        u, transverse = speed * np.cos(angle), speed * np.sin(angle)
                        force = a["force_angle_n_rad"] * angle + a["force_control_n_rad"] * surface
                        udot = fx / mass + gravity[0] - sign * rate * transverse
                        tdot = force / mass + gravity[faxis] + sign * rate * u
                        return np.array([(u * tdot - transverse * udot) / speed**2,
                                         (a["moment_angle_nm_rad"] * angle + a["moment_rate_nm_s_rad"] * rate +
                                          a["moment_control_nm_rad"] * surface) / inertia,
                                         -surface / tau])

                    h = 1e-6
                    numerical = np.column_stack([(physical(np.eye(3)[j] * h) - physical(-np.eye(3)[j] * h)) / (2*h)
                                                 for j in range(3)])
                    np.testing.assert_allclose(theory.directional_state_matrix(i, name), numerical, rtol=1e-7, atol=1e-8)

    def test_actual_captured_commands_match_current_selected_law(self):
        mechanisms = set()
        compared_models = set()
        for model, packet in packets():
            mechanisms.add(packet["mechanism"])
            if packet["mechanism"] == "fsim.control.operating_point_bounded.v1":
                continue  # A bounded nonlinear search is not a cascade port.
            with self.subTest(model=model, sequence=packet["sequence"]):
                compared_models.add(model)
                actual = theory.commands(packet)
                reference = packet["commands"]
                self.assertEqual(actual["available"], reference["available"])
                for axis in ("aileron", "elevator", "rudder"):
                    self.assertAlmostEqual(actual[axis], reference[axis], delta=2e-9)
        self.assertEqual(len(mechanisms), 3)
        self.assertEqual(len(compared_models), 23)

    def test_operating_point_virtual_lag_matches_scipy_bilinear_discretization(self):
        model, packet = next((m, p) for m, p in packets() if m == "r-77m")
        point = packet["operating_point"]
        self.assertEqual(point["channel_input_rad"], [0, 0, 0])
        self.assertEqual(point["channel_lag_rad"], [0, 0, 0])
        dt = point["dt_s"] / point["actuator_substeps"]
        num, den, _ = cont2discrete(([1.], [packet["input"]["actuator_lag_s"], 1.]), dt, method="bilinear")
        command = [packet["commands"][axis] for axis in ("aileron", "elevator", "rudder")]
        actual = theory.operating_prediction(packet, command)
        for axis in range(3):
            normalized = command[axis]
            extent = (-packet["stops"]["lower_rad"][axis] if normalized < 0 else packet["stops"]["upper_rad"][axis])
            reference = lfilter(num[0], den, [normalized*extent]*point["actuator_substeps"])[-1]
            self.assertAlmostEqual(actual["channel_lag_rad"][axis], reference, delta=1e-12)

    def test_actual_operating_point_command_improves_finite_moment_objective(self):
        for model, packet in packets():
            if model != "r-77m":
                continue
            with self.subTest(sequence=packet["sequence"]):
                result = theory.operating_reference(packet)
                self.assertLessEqual(result["captured_objective_s_4"], result["best_neutral_or_corner_objective_s_4"]+1e-9)
                self.assertEqual(len(result["predicted_fin_angles_rad"]), len(packet["operating_point"]["fins"]))

    def test_actuator_aware_gains_match_scipy_independent_pole_placement(self):
        for model, packet in packets():
            if model != "pl-10":
                continue
            for axis in ("pitch", "yaw"):
                with self.subTest(sequence=packet["sequence"], axis=axis):
                    A = np.asarray(theory.directional_state_matrix(packet["input"], axis))
                    B = np.array([[0.], [0.], [1/packet["input"]["actuator_lag_s"]]])
                    rate, incidence = theory.response_times(packet)
                    desired = [-1/packet["input"]["actuator_lag_s"], -1/rate, -1/incidence]
                    reference = place_poles(A, B, desired).gain_matrix[0]
                    np.testing.assert_allclose(theory.feedback_gains(packet, axis), reference, rtol=2e-8, atol=2e-9)

    def test_powered_zero_aero_has_directional_authority_and_true_singularity_refuses(self):
        _, packet = next(packets())
        p = copy.deepcopy(packet)
        i = p["input"]
        i["demand_body_m_s2"] = [0., 0., -10.]
        i["gravity_body_m_s2"] = [0., 0., 0.]
        i["rates_rad_s"] = [0., 0., 0.]
        i["measured"]["force_body_n"] = [2000., 0., 0.]
        i["measured"]["moment_body_nm"] = [0., 0., 0.]
        i["measured"]["control_rad"] = [0., 0., 0.]
        i["alpha_rad"] = i["beta_rad"] = 0.
        i["pitch"] = dict(force_angle_n_rad=0., force_control_n_rad=-1000.,
                          moment_angle_nm_rad=0., moment_control_nm_rad=3000., moment_rate_nm_s_rad=0.)
        result = theory.directional_trim(i, "pitch")
        self.assertEqual(result["determinant_n_rad"], -2000.)
        self.assertTrue(np.isfinite(result["angle_rad"]))
        i["measured"]["force_body_n"][0] = 0.
        with self.assertRaises(ValueError):
            theory.directional_trim(i, "pitch")

    def test_only_autopilot_object_spans_can_be_replaced(self):
        sample = '{"models":[{"retain":1}],"cells":[ {"model":"a","component":"guidance","x":1.0},\n {"model":"a","component":"autopilot","x":2} ]}'
        changed = theory.replace_autopilot_cells(sample, {"a": {"model":"a", "component":"autopilot", "x":3}})
        self.assertIn('{"model":"a","component":"guidance","x":1.0}', changed)
        self.assertEqual(json.loads(changed)["models"], [{"retain": 1}])
        self.assertEqual(json.loads(changed)["cells"][1]["x"], 3)

    def test_missing_or_false_axial_contract_cannot_be_used_as_current_law_input(self):
        _, packet = next(packets())
        theory.validate_packet(packet)
        missing = copy.deepcopy(packet)
        del missing["axial_force_contract"]
        with self.assertRaises(ValueError):
            theory.validate_packet(missing)
        changed = copy.deepcopy(packet)
        changed["input"]["measured"]["force_body_n"][0] += 1
        with self.assertRaises(ValueError):
            theory.validate_packet(changed)

    def test_capture_selection_has_normal_output_fallback_and_explicit_override(self):
        with mock.patch.dict(os.environ, {}, clear=True):
            self.assertEqual(theory.capture_root_from_environment(ROOT),
                             ROOT / "build/ucrt64-release/validation/aam_full_chain")
        selected = ROOT / "deliberately-selected-capture"
        with mock.patch.dict(os.environ, {"FSIM_DIRECTIONAL_CONTROL_CAPTURE_ROOT": str(selected)}):
            self.assertEqual(theory.capture_root_from_environment(ROOT), selected)

    def test_excluded_paths_are_rejected_before_any_read_or_parent_access(self):
        excluded = ROOT / "unopened" / ".CoDeX" / "unopened"
        with mock.patch.object(Path, "open", side_effect=AssertionError("filesystem read reached")):
            with self.assertRaises(ValueError):
                theory.sha(excluded)
            with self.assertRaises(ValueError):
                theory.generate(excluded, ROOT, "must not read")
            with self.assertRaises(ValueError):
                theory.load_first_pre(ROOT, excluded, "aim-120d")

    def test_joint_effectiveness_scaling_reports_surface_and_command_response(self):
        _, packet = next(packets())
        result = theory.numerical_sensitivity(packet)
        joint = next(row for row in result if row["parameter"] == "control_force_and_moment_effectiveness")
        self.assertIn("expected_invariance", joint)
        for label in ("minus10percent", "baseline", "plus10percent"):
            self.assertIn("trim_surface_rad", joint[label]["pitch"])
            self.assertIn("normalized_commands", joint[label])
            self.assertIn("output_scope", joint[label])


if __name__ == "__main__":
    unittest.main()
