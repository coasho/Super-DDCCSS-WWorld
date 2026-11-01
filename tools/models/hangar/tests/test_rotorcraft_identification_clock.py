"""Native filter clock and public observation clock are separate contracts."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from hangar.rotorcraft.fly import FDM_DT, Session, identify, step_input


class IdentificationClock(unittest.TestCase):
    def test_native_filter_matches_closed_form_at_public_samples(self):
        t = np.arange(1, 31) / 30.0
        tau, size = 0.047, 0.03
        for substeps in (1, 2, 3, 5, 12):
            with self.subTest(substeps=substeps):
                dt = FDM_DT / substeps
                n = np.rint((t - dt) / dt).astype(int)
                b = dt / (2 * tau + dt)
                a = (2 * tau - dt) / (2 * tau + dt)
                expected = size * (1 - (1 - b) * a ** (n - 1))
                np.testing.assert_allclose(
                    step_input(t, size, tau, dt, native_tustin=True, native_dt=dt),
                    expected, rtol=1e-12, atol=1e-15)

    def test_heave_identification_uses_native_clock_with_outer_samples(self):
        t = np.arange(1, 31) / 30.0
        dt, tau, size = FDM_DT / 3, 0.047, 0.03
        n = np.rint((t - dt) / dt).astype(int)
        b = dt / (2 * tau + dt)
        a = (2 * tau - dt) / (2 * tau + dt)
        u = size * (1 - (1 - b) * a ** (n - 1))
        velocity = np.sin(t * 2.0)
        acceleration = -0.7 * velocity + 23.0 * u
        power, damping, error = identify(t, acceleration, size, tau, 1 / 30,
                                        velocity, delay=dt, native_tustin=True, native_dt=dt)
        self.assertAlmostEqual(power, 23.0, places=10)
        self.assertAlmostEqual(damping, 0.7, places=10)
        self.assertLess(error, 1e-12)

    def test_invalid_native_clock_is_rejected(self):
        for dt in (0.0, -0.1, float('nan'), float('inf')):
            with self.subTest(dt=dt), self.assertRaises(ValueError):
                step_input(np.array([0.1]), 0.03, 0.047, 0.0,
                           native_tustin=True, native_dt=dt)

    def test_steady_tilted_hover_has_zero_observed_cg_heave_acceleration(self):
        class HoldStub:
            def __init__(self, *args, **kwargs):
                self.i = {}
                self.cmd = dict(ail=0.0, ele=0.0, rud=0.0, col=0.5)

            def __call__(self, *args):
                return self.cmd

        def velocity_property(name):
            if name not in ('velocities/u-fps', 'velocities/v-fps'):
                raise AssertionError('Heave must use CG NED velocity, not pilot-axis specific force')
            return 0.0

        for phi, theta in ((0.0, 0.0), (0.2, 0.3)):
            with self.subTest(phi=phi, theta=theta):
                state = SimpleNamespace(euler_rad=(phi, theta, 0.0), altitude_msl_m=100.0,
                                        velocity_ned_ms=(0.0, 0.0, 0.0), angular_rate_body_rad_s=(0.0, 0.0, 0.0))
                vehicle = SimpleNamespace(state=state, get_property=velocity_property,
                                          command_actuator=lambda **kwargs: None)
                session = Session.__new__(Session)
                session.r = SimpleNamespace(kind='helicopter')
                session.dt = 1 / 30
                session.spawn = lambda speed: vehicle
                session.world = SimpleNamespace(step=lambda: None)
                trim = dict(aileron=0.0, elevator=0.0, rudder=0.0, throttle=0.5)
                with patch('hangar.rotorcraft.fly.Hold', HoldStub):
                    _, acceleration, velocity = session.step_response(trim, 'heave', 0.03, 0.5)
                np.testing.assert_array_equal(acceleration, np.zeros(15))
                np.testing.assert_array_equal(velocity, np.zeros(15))


if __name__ == '__main__':
    unittest.main()
