"""Stores and weapon employment through Python (ADR-30 MS-7a, docs/sdk/mission-systems.md): an F-16C starts with its
stations empty, takes its loadout, refuses a load its station cannot take, strikes with release consent asked for and
given, releases its Mk 82s as weapons that remember the store they were, and jettisons the rest in an emergency."""
import math
import unittest

import fsim


def make_world():
    return fsim.World("py-stores", publish=False, workers=1, pin_workers=False, seed=5, terrain=False)


class StoresTest(unittest.TestCase):
    def test_loadout_strike_consent_and_jettison(self):
        w = make_world()
        v = w.create_vehicle("viper", "jsbsim:f16c", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=5000.0, airspeed_ms=220.0,
                             heading_deg=90.0)
        self.assertEqual(v.stores, [])
        empty = v.mass_kg
        v.load_loadout("mk82 x4")
        stores = v.stores
        self.assertEqual([s.station for s in stores], [3, 4, 6, 7])
        self.assertTrue(all(s.store == "mk82" and s.carriage == "ejector" and s.status == "loaded" and s.flyable for s in stores))
        with self.assertRaises(fsim.MissionRejected) as bad:
            v.load_store(1, "mk82")  # (a wingtip rail takes missiles)
        self.assertEqual(bad.exception.reason, "station_cannot_take")
        w.step(60)
        self.assertAlmostEqual(v.mass_kg - empty, 4 * 226.8, delta=10.0)  # (less the fuel burnt)

        # strike: refused with the master arm off, then asking for consent
        with self.assertRaises(fsim.MissionRejected) as off:
            v.strike("mk82", 2)
        self.assertEqual(off.exception.reason, "payload_off")
        v.set_payload_state("stations", "operate")
        w.step()
        activity = v.strike("mk82", 2, ripple_s=0.5)
        w.step(120)
        self.assertEqual(v.strike_status(activity).phase, "awaiting_consent")
        self.assertEqual(w.weapons, [])
        v.strike_consent(activity)
        with self.assertRaises(fsim.MissionRejected) as again:
            v.strike_consent(activity)
        self.assertEqual(again.exception.reason, "not_awaiting_consent")
        w.step(120)
        weapons = w.weapons
        self.assertEqual(len(weapons), 2)
        self.assertTrue(all(x.store == "mk82" and x.activity == activity and not x.jettisoned for x in weapons))
        self.assertEqual([x.station for x in weapons], [3, 4])
        self.assertAlmostEqual(weapons[1].launch_time_s - weapons[0].launch_time_s, 0.5, delta=0.05)
        status = v.strike_status(activity)
        self.assertEqual(status.phase, "done")
        self.assertEqual(sorted(status.weapons), sorted(x.id for x in weapons))
        self.assertEqual([s.status for s in v.stores], ["released", "released", "loaded", "loaded"])

        # the rest jettisoned in an emergency, the stations off
        v.set_payload_state("stations", "off")
        v.jettison(emergency=True)
        w.step(30)
        self.assertEqual([s.status for s in v.stores], ["released", "released", "jettisoned", "jettisoned"])
        self.assertEqual(sum(1 for x in w.weapons if x.jettisoned), 2)
        self.assertTrue(math.isfinite(v.mass_kg))


if __name__ == "__main__":
    unittest.main()
