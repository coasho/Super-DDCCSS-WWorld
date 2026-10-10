"""An air search radar through Python (ADR-30 MS-4, docs/sdk/mission-systems.md): an F-16C's radar refused while off,
searching once operating, detecting another F-16C head on where it is, read only for what is new, and stopped."""
import math
import unittest

import fsim


def make_world():
    return fsim.World("py-radar", publish=False, workers=1, pin_workers=False, seed=3, terrain=False)


class RadarTest(unittest.TestCase):
    def test_search_and_detections(self):
        w = make_world()
        v = w.create_vehicle("viper", "jsbsim:f16c", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=5000.0, airspeed_ms=200.0,
                             heading_deg=90.0)
        bandit = w.create_vehicle("bandit", "jsbsim:f16c", latitude_deg=40.0, longitude_deg=30e3 / (6371e3 * math.cos(math.radians(40.0))) * 180 / math.pi,
                                  altitude_msl_m=5000.0, airspeed_ms=200.0, heading_deg=270.0)
        with self.assertRaises(fsim.MissionRejected) as off:
            v.amti_search()
        self.assertEqual(off.exception.reason, "payload_off")
        self.assertIsNone(v.radar_status("rwr"))
        v.set_payload_state("radar", "operate")
        w.step()
        with self.assertRaises(fsim.MissionRejected) as bad:
            v.amti_search(mode="surveillance")  # (a rotodome's)
        self.assertEqual(bad.exception.reason, "invalid_parameter")
        activity = v.amti_search(azimuth_half_width_rad=math.radians(30.0))
        w.step(300)
        status = v.radar_status()
        self.assertTrue(status.searching)
        self.assertEqual(status.activity, activity)
        self.assertEqual(status.mode, fsim.AmtiMode.RANGE_WHILE_SEARCH)
        self.assertFalse(status.rotodome)
        self.assertGreater(status.dwells, 0)
        found = v.amti_detections()
        self.assertGreater(len(found), 0)
        self.assertEqual(status.detections, len(found))
        real = [d for d in found if d.truth != 0]  # (the rest false alarms, at the card's rate)
        self.assertGreater(len(real), 0)
        self.assertEqual(status.false_alarms, len(found) - len(real))
        for d in real:
            self.assertEqual(d.truth, bandit.id)
            self.assertLess(abs(d.azimuth_rad), math.radians(3.0))
            self.assertLess(d.range_rate_ms, -300.0)
            self.assertTrue(15e3 < d.range_m < 31e3)
        # its track on the bandit (MS-5)
        tracks = [t for t in v.amti_tracks() if t.truth == bandit.id]
        self.assertEqual(len(tracks), 1)
        self.assertGreaterEqual(tracks[0].updates, 3)
        self.assertLess(tracks[0].range_rate_ms, -300.0)
        self.assertEqual(status.tracks, len(v.amti_tracks()))
        self.assertEqual([d.serial for d in found], list(range(len(found))))
        self.assertEqual(v.amti_detections(since=found[-1].serial + 1), [])
        self.assertEqual(v.mission_activity(activity).state, fsim.ActivityState.ACTIVE)
        self.assertFalse(status.signal_level)
        # the same search at the signal level (MS-6): its own detections of the bandit; not in velocity search
        activity = v.amti_search(azimuth_half_width_rad=math.radians(30.0), signal_level=True)
        before = v.radar_status().detections
        w.step(300)
        status = v.radar_status()
        self.assertTrue(status.signal_level)
        seen = [d for d in v.amti_detections(since=before) if d.truth == bandit.id]
        self.assertGreater(len(seen), 0)
        self.assertTrue(all(15e3 < d.range_m < 31e3 and d.range_rate_ms < -300.0 for d in seen))
        with self.assertRaises(fsim.MissionRejected) as vs:
            v.amti_search(mode="velocity_search", signal_level=True)
        self.assertEqual(vs.exception.reason, "invalid_parameter")
        v.cancel_mission(activity)
        self.assertFalse(v.radar_status().searching)


if __name__ == "__main__":
    unittest.main()
