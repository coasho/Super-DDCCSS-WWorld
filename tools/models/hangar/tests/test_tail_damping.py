"""Published derivative transfer, native inflow slope and passive force port."""
import copy
import math
from pathlib import Path
import tomllib
import unittest

import numpy as np

from hangar.rotorcraft import tail_damping


class TailDamping(unittest.TestCase):
    def setUp(self):
        path = Path(__file__).resolve().parents[4] / 'aircraft/uh1h/uh1h.toml'
        with path.open('rb') as stream:
            self.spec = tomllib.load(stream)
        radius = self.spec['rotor']['tail']['radius_ft']
        self.point = dict(rho=.002377, rpm=740. / radius * 60 / (2 * math.pi),
                          mu=0., **{'lambda': -.05}, ct=2 * .05**2, native_dt_s=1/360)

    def test_native_dc_slope_matches_implicit_steady_inflow(self):
        data = tail_damping.derive(self.spec, self.point)
        rotor = self.spec['rotor']['tail']
        radius, chord = rotor['radius_ft'], rotor['chord_ft']
        sigma = rotor['blades'] * chord / (math.pi * radius)
        a = rotor['lift_slope'] * sigma / 4
        c0 = self.point['ct'] - a * self.point['lambda']
        def thrust(w):
            lo, hi = .001, .5
            for _ in range(70):
                nu = .5 * (lo + hi)
                lam = w/740. - nu
                ct = c0 + a * lam
                residual = nu - ct / (2 * abs(lam) + 1e-15)
                if residual > 0: hi = nu
                else: lo = nu
            return self.point['rho'] * math.pi * radius**2 * 740.**2 * (c0 + a*(w/740. - .5*(lo+hi)))
        derivative = (thrust(.0001) - thrust(-.0001)) / .0002
        self.assertAlmostEqual(data['native_dc_lb_per_fps'], derivative, delta=1e-7)

    def test_yaw_target_uses_the_current_physical_port(self):
        data = tail_damping.derive(self.spec, self.point)
        yaw_port = data['physical_arm_ft'][0] * data['normal'][1]
        self.assertAlmostEqual(data['target_normal_dc_lb_per_fps'] * yaw_port**2,
                               data['target_yaw_damping_lb_ft_s'])
        self.assertNotEqual(abs(data['physical_arm_ft'][0]), self.spec['rotor']['tail']['damping_arm_ft'][0])

    def test_passive_supplement_cannot_match_a_smaller_target(self):
        self.spec['rotor']['tail']['damping_lb_per_fps'] = .001
        with self.assertRaises(ValueError): tail_damping.derive(self.spec, self.point)

    def test_instantaneous_excess_is_passive(self):
        data = tail_damping.derive(self.spec, self.point)
        speed = np.linspace(-100., 100., 4097)
        self.assertTrue(np.all(-data['excess_dc_lb_per_fps']*speed**2 <= 0))

    def test_force_and_hub_moment_share_the_same_power_port(self):
        data = tail_damping.derive(self.spec, self.point)
        arm = np.array(data['physical_arm_ft']);normal = np.array(data['normal'])
        velocity = np.array([12., 3., -2.]);omega = np.array([.2, -.4, .1])
        q = np.dot(normal, velocity + np.cross(omega, arm))
        coefficient = data['excess_dc_lb_per_fps']
        force = -coefficient * q * normal;moment = np.cross(arm, force)
        power = np.dot(force, velocity) + np.dot(moment, omega)
        self.assertAlmostEqual(power, -coefficient*q**2)


if __name__ == '__main__':
    unittest.main()
