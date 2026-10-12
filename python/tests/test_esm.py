"""The warning receiver through Python (ADR-30, MS-9a; docs/sdk/mission-systems.md): what an F-16C's receiver holds of
another's radar - nothing while it is off, a search once it listens, a warning once the other locks it and lights it."""
import math
import unittest

import fsim

EAST_40_KM_DEG = math.degrees(40e3 / (6371000.0 * math.cos(math.radians(40.0))))


def make_world():
    return fsim.World("py-esm", publish=False, workers=1, pin_workers=False, seed=3, terrain=False)


class EsmTest(unittest.TestCase):
    def test_contacts(self):
        w = make_world()
        own = w.create_vehicle("viper", "jsbsim:f16c", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=5000.0, heading_deg=90.0,
                               airspeed_ms=200.0)
        other = w.create_vehicle("bandit", "jsbsim:f16c", latitude_deg=40.0, longitude_deg=EAST_40_KM_DEG, altitude_msl_m=5000.0,
                                 heading_deg=270.0, airspeed_ms=200.0)
        caps = {c.id: c for c in own.mission_capabilities}
        self.assertEqual(caps["fsim.ms.esm:rwr"].support, "supported")
        other.set_payload_state("radar", "operate")
        w.step()
        other.amti_search("radar", azimuth_half_width_rad=math.radians(30.0))
        w.step(60)
        self.assertEqual(own.contacts("rwr"), [])        # (its receiver off)
        own.set_payload_state("rwr", "operate")
        w.step(30)
        [c] = own.contacts("rwr")
        self.assertIsInstance(c, fsim.Contact)
        self.assertIs(c.kind, fsim.ContactKind.EMITTER)
        self.assertIs(c.mode, fsim.EmitterMode.SEARCH)
        self.assertIs(c.warning, False)
        self.assertEqual(c.truth, other.id)
        self.assertEqual(c.payload, own.payload("rwr").index)
        self.assertTrue(math.isnan(c.range_m))           # (a receiver measures none)
        self.assertLess(abs(c.azimuth_rad), math.radians(40.0))
        self.assertAlmostEqual(c.bearing_sigma_rad, math.radians(10.0))
        self.assertAlmostEqual(c.frequency_hz, other.radar_status("radar").frequency_hz)
        self.assertTrue(-50.0 < c.strength_db < -30.0)
        self.assertEqual(own.contacts("radar"), [])      # (a radar holds no such contacts)
        with self.assertRaises(fsim.MissionRejected) as unknown:
            own.contacts("nose")
        self.assertEqual(unknown.exception.reason, "unknown_payload")
        # the other takes it in single-target track and lights it: a warning
        tracks = []
        for _ in range(90):
            w.step(30)
            tracks = other.amti_tracks("radar")
            if tracks:
                break
        self.assertTrue(tracks)
        other.amti_track(tracks[0])
        w.step(60)
        [c] = own.contacts("rwr")
        self.assertIs(c.mode, fsim.EmitterMode.LOCK)
        self.assertIs(c.warning, True)
        other.amti_track(tracks[0], illuminate=True)
        w.step(60)
        [c] = own.contacts("rwr")
        self.assertIs(c.mode, fsim.EmitterMode.ILLUMINATION)
        self.assertIs(c.warning, True)
        # the other's radar off: forgotten
        other.set_payload_state("radar", "standby")
        w.step(120)
        self.assertEqual(own.contacts("rwr"), [])


if __name__ == "__main__":
    unittest.main()
