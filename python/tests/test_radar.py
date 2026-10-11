"""An air search radar through Python (ADR-30 MS-4, docs/sdk/mission-systems.md): an F-16C's radar refused while off,
searching once operating, detecting another F-16C head on where it is, read only for what is new, and stopped."""
import math
import unittest

import fsim


def make_world():
    return fsim.World("py-radar", publish=False, workers=1, pin_workers=False, seed=3, terrain=False)


class RadarTest(unittest.TestCase):
    def test_engagement_modes(self):
        # MS-4b: a track followed alone and lit, lost when its target is gone; an air-combat acquisition
        w = make_world()
        v = w.create_vehicle("viper", "jsbsim:f16c", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=5000.0, airspeed_ms=200.0,
                             heading_deg=90.0)
        east = lambda km: km * 1e3 / (6371e3 * math.cos(math.radians(40.0))) * 180 / math.pi  # noqa: E731
        bandit = w.create_vehicle("bandit", "jsbsim:f16c", latitude_deg=40.0, longitude_deg=east(40.0), altitude_msl_m=5000.0,
                                  airspeed_ms=200.0, heading_deg=270.0)
        v.set_payload_state("radar", "operate")
        w.step()
        with self.assertRaises(fsim.MissionRejected) as none:
            v.amti_track(42)
        self.assertEqual(none.exception.reason, "unknown_track")
        v.amti_search(azimuth_half_width_rad=math.radians(30.0))
        for _ in range(4000):
            w.step()
            tracks = [t for t in v.amti_tracks() if t.truth == bandit.id]
            if tracks:
                break
        self.assertEqual(len(tracks), 1)
        status = v.radar_status()
        self.assertEqual(status.task, fsim.AmtiTask.VOLUME)
        self.assertEqual(status.designated_track, 0)
        with self.assertRaises(fsim.MissionRejected) as scan:
            v.amti_track(tracks[0], mode="track_while_scan", illuminate=True)  # (a scan lights nothing)
        self.assertEqual(scan.exception.reason, "invalid_parameter")
        # the scan kept on it, then every dwell on it and the target lit
        v.amti_track(tracks[0], mode="track_while_scan", azimuth_half_width_rad=math.radians(10.0), bars=2)
        w.step(120)
        status = v.radar_status()
        self.assertEqual((status.task, status.mode, status.designated_track),
                         (fsim.AmtiTask.TRACK, fsim.AmtiMode.TRACK_WHILE_SCAN, tracks[0].id))
        self.assertFalse(status.illuminating)
        activity = v.amti_track(tracks[0].id, illuminate=True)
        w.step(120)
        status = v.radar_status()
        self.assertEqual((status.task, status.mode, status.designated_track),
                         (fsim.AmtiTask.TRACK, fsim.AmtiMode.SINGLE_TARGET_TRACK, tracks[0].id))
        self.assertTrue(status.illuminating)
        self.assertEqual(status.track_memory_s, 3.0)
        held = [t for t in v.amti_tracks() if t.id == tracks[0].id]
        self.assertEqual(len(held), 1)
        self.assertLess(held[0].position_sigma_m, tracks[0].position_sigma_m)  # (a dwell at a time on it: a better track)
        # the target gone: the track kept for the memory time, then the activity failed - track_lost - the radar stopped
        bandit.remove()
        for _ in range(2000):
            w.step()
            if not v.radar_status().searching:
                break
        status = v.radar_status()
        self.assertFalse(status.searching)
        self.assertFalse(status.illuminating)
        ended = v.mission_activity(activity)
        self.assertEqual(ended.state, fsim.ActivityState.FAILED)
        self.assertEqual(ended.reason, "track_lost")
        # an air-combat acquisition: nothing ahead, it searches; an aircraft 8 km ahead, it takes it
        v.amti_acquire(pattern="boresight")
        w.step(60)
        status = v.radar_status()
        self.assertEqual((status.task, status.designated_track), (fsim.AmtiTask.CUED, 0))
        self.assertAlmostEqual(status.range_scale_m, 18520.0)
        st = v.state  # (8 km along its nose, wherever that points by now)
        pitch, yaw = float(st.euler_rad[1]), float(st.euler_rad[2])
        north, along = 8.0 * math.cos(pitch) * math.cos(yaw), 8.0 * math.cos(pitch) * math.sin(yaw)
        ahead = w.create_vehicle("ahead", "jsbsim:f16c", latitude_deg=math.degrees(st.latitude_rad) + north * 1e3 / 6371e3 * 180 / math.pi,
                                 longitude_deg=math.degrees(st.longitude_rad) + east(along),
                                 altitude_msl_m=st.altitude_msl_m + 8e3 * math.sin(pitch), airspeed_ms=200.0,
                                 heading_deg=math.degrees(yaw))
        for _ in range(2000):
            w.step()
            if v.radar_status().task == fsim.AmtiTask.TRACK:
                break
        status = v.radar_status()
        self.assertEqual((status.task, status.mode), (fsim.AmtiTask.TRACK, fsim.AmtiMode.SINGLE_TARGET_TRACK))
        self.assertEqual([t.truth for t in v.amti_tracks() if t.id == status.designated_track], [ahead.id])
        tall = v.amti_acquire(pattern=fsim.AcmPattern.VERTICAL_SCAN)
        self.assertGreater(v.radar_status().bars, 8)  # (60 deg of elevation: more bars than a search's eight)
        v.cancel_mission(tall)

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
