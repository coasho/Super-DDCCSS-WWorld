"""Body-force integration must reproduce constant density and its first moment."""
import unittest
import numpy as np
from hangar.aero.bodies import BodyAero


class BodyQuadrature(unittest.TestCase):
    def test_constant_skin_friction_and_crossflow_wrench(self):
        for x in (np.linspace(0., 2., 61), np.array([0., .02, .23, .9, 2.])):
            with self.subTest(stations=len(x)):
                body = BodyAero.__new__(BodyAero)
                n = len(x)
                body.inlets = []
                body.parts = [dict(x=x, pts=np.column_stack([x, np.zeros(n), np.zeros(n)]),
                    inside=np.zeros(n, dtype=bool), s_v=np.ones(n), s_h=np.ones(n),
                    potential=np.zeros(n), w=np.full(n, .3), h=np.full(n, .2),
                    eta=.7, cdc=np.full(n, 1.2), friction=.04)]
                rho, speed_x, speed_z = 1.1, 2., 3.
                force, moment = body.forces(np.array([speed_x, 0., speed_z]), np.zeros(3), np.zeros(3), rho=rho)
                expected_x = .5 * rho * speed_x**2 * .04
                expected_z = .5 * rho * speed_z**2 * .7 * 1.2 * .3 * 2.
                np.testing.assert_allclose(force, [expected_x, 0., expected_z], rtol=1e-13, atol=1e-14)
                np.testing.assert_allclose(moment, [0., -expected_z, 0.], rtol=1e-13, atol=1e-14)


if __name__ == '__main__':
    unittest.main()
