"""Weapons in flight and the radio path through Python (ADR-30, MS-2; docs/sdk/mission-systems.md): a Mk 82 released
by an F-16C falls as an entity of the world, its flight capabilities restricted, and ends where it strikes; the radio
path between two entities."""
import math
import unittest

import fsim


def make_world():
    return fsim.World("py-weapons", publish=False, workers=1, pin_workers=False, seed=3, terrain=False)


class WeaponsTest(unittest.TestCase):
    def test_release_fall_and_impact(self):
        w = make_world()
        jet = w.create_vehicle("viper", "jsbsim:f16c", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=2000.0,
                               airspeed_ms=180.0, heading_deg=90.0)
        self.assertEqual(jet.role, "aircraft")
        bomb = w.launch_weapon(jet, "jsbsim:mk82", offset_m=(0.5, 2.5, 0.8), ejection_ms=(0.0, 0.0, 3.0))
        self.assertEqual(bomb.role, "weapon")
        self.assertEqual(bomb.launcher, jet.id)
        self.assertEqual(bomb.type, "jsbsim:mk82")
        self.assertEqual(bomb.capability_status("fsim.flight.attitude"), (fsim.Availability.UNAVAILABLE, "restricted"))
        [live] = w.weapons
        self.assertEqual(live.end, "in_flight")
        self.assertTrue(math.isnan(live.end_time_s))
        w.step(30)
        path = w.radio_path(jet, bomb)
        self.assertIsInstance(path, fsim.RadioPath)
        self.assertGreater(path.slant_range_m, 3.0)  # (a second after the release: the push and a fall of 5 m)
        self.assertLess(path.elevation_rad[0], 0.0)
        self.assertEqual(path.mode, 1)
        for _ in range(3000):
            if bomb.id not in w.vehicle_ids():
                break
            w.step()
        [r] = w.weapons
        self.assertEqual(r.end, "impact")
        self.assertEqual(r.id, bomb.id)
        self.assertGreater(r.end_time_s - r.launch_time_s, 19.0)  # (2000 m: 20.2 s in a vacuum)
        self.assertLess(r.end_time_s - r.launch_time_s, 26.0)
        self.assertLess(abs(r.end_height_m), 5.0)
        self.assertGreater(r.end_speed_ms, 180.0)

    def test_radio_path_options(self):
        w = make_world()
        a = w.create_vehicle("a", "jsbsim:f16c", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=9000.0, airspeed_ms=200.0)
        b = w.create_vehicle("b", "jsbsim:f16c", latitude_deg=40.0, longitude_deg=0.5, altitude_msl_m=9000.0, airspeed_ms=200.0)
        clear = w.radio_path(a, b, frequency_hz=9.4e9)
        self.assertAlmostEqual(clear.basic_db, clear.free_space_db + clear.gas_db, delta=1.0)
        self.assertEqual(clear.rain_db, 0.0)
        wet = w.radio_path(a, b, frequency_hz=9.4e9, rain_mm_h=25.0, rain_height_m=10000.0)
        self.assertGreater(wet.rain_db, 10.0)
        self.assertAlmostEqual(wet.total_db, clear.total_db + wet.rain_db, delta=1e-6)
        with self.assertRaises(fsim.Error):
            w.launch_weapon(999)


if __name__ == "__main__":
    unittest.main()
