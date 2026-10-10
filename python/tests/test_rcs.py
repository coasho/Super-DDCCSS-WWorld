"""Radar cross sections through Python (ADR-30, MS-3; docs/sdk/mission-systems.md): an F-16C as another F-16C's radar
sees it, head on and from the side, in either polarisation."""
import math
import unittest

import fsim


class RcsTest(unittest.TestCase):
    def test_head_on_and_side(self):
        w = fsim.World("py-rcs", publish=False, workers=1, pin_workers=False, seed=3, terrain=False)
        target = w.create_vehicle("target", "jsbsim:f16c", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=5000.0,
                                  airspeed_ms=200.0, heading_deg=90.0)
        ahead = w.create_vehicle("ahead", "jsbsim:f16c", latitude_deg=40.0, longitude_deg=0.2, altitude_msl_m=5000.0,
                                 airspeed_ms=200.0, heading_deg=270.0)
        beside = w.create_vehicle("beside", "jsbsim:f16c", latitude_deg=40.15, longitude_deg=0.0, altitude_msl_m=5000.0,
                                  airspeed_ms=200.0, heading_deg=90.0)
        head = w.radar_cross_section(ahead, target)
        self.assertIsInstance(head, fsim.RadarCrossSection)
        self.assertTrue(head.modelled)
        self.assertLess(abs(head.aspect_azimuth_rad), math.radians(2))
        self.assertGreater(head.sigma_m2, 0.3)
        self.assertLess(head.sigma_m2, 30.0)
        self.assertGreaterEqual(head.fluctuation, 1.0)
        side = w.radar_cross_section(beside, target, frequency_hz=9.5e9, polarization="horizontal")
        self.assertAlmostEqual(side.aspect_azimuth_rad, -math.pi / 2, delta=math.radians(3))  # (north of it: its left)
        self.assertGreater(side.sigma_m2, 20 * head.sigma_m2)  # (broadside: its flat sides and fin)
        crossed = w.radar_cross_section(ahead, target, polarization="vertical", receive="horizontal")
        self.assertLess(crossed.sigma_m2, head.sigma_m2)
        with self.assertRaises(fsim.Error):
            w.radar_cross_section(ahead, 999)


if __name__ == "__main__":
    unittest.main()
