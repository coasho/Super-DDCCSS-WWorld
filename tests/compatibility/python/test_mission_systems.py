"""Mission Systems through Python (ADR-30, docs/sdk/mission-systems.md): the payloads an F-16C carries from its fit,
their capabilities discovered honestly, a payload powered, tested and failed, and its grants through the control calls."""
import unittest

import fsim


def make_world():
    return fsim.World("py-mission-systems", publish=False, workers=1, pin_workers=False, seed=3, terrain=False)


def viper(w, name="viper"):
    return w.create_vehicle(name, "jsbsim:f16c", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=3000.0, airspeed_ms=160.0,
                            heading_deg=90.0)


class MissionSystemsTest(unittest.TestCase):
    def test_payloads_and_discovery(self):
        w = make_world()
        v = viper(w)
        names = [p.name for p in v.payloads]
        self.assertEqual(names, ["radar", "rwr", "cmds", "gun", "stations", "ews"])
        self.assertEqual(v.payload("ews").designation, "AN/ALQ-213")
        radar = v.payload("radar")
        self.assertEqual(radar.designation, "AN/APG-68(V)9")
        self.assertEqual(radar.family, "air_radar")
        self.assertEqual(radar.state, fsim.PayloadState.OFF)
        self.assertTrue(radar.source)
        self.assertEqual(v.payload("stations").count, 9)
        caps = {c.id: c for c in v.mission_capabilities}
        self.assertEqual(caps["fsim.ms.status:radar"].support, "supported")
        self.assertEqual(caps["fsim.ms.amti:radar"].support, "supported")  # (its card describes the radar: MS-4)
        self.assertEqual(caps["fsim.ms.esm:rwr"].support, "supported")  # (its warning receiver: MS-9a)
        self.assertEqual(caps["fsim.ms.ea:cmds"].support, "not_implemented")
        self.assertEqual(caps["fsim.ms.ea:cmds"].stage, 9)
        availability, reason, _ = v.mission_capability_status("fsim.ms.amti:radar")
        self.assertEqual(availability, fsim.Availability.TEMPORARILY_UNAVAILABLE)
        self.assertEqual(reason, "payload_off")
        availability, reason, _ = v.mission_capability_status("fsim.ms.ea:cmds")
        self.assertEqual(availability, fsim.Availability.UNAVAILABLE)
        self.assertEqual(reason, "not_implemented")
        self.assertEqual(v.mission_capability_status("fsim.ms.sar:radar")[1], "not_supported")

    def test_state_bit_and_failure(self):
        w = make_world()
        v = viper(w)
        a = v.set_payload_state("radar", "operate", command_id=42)
        info = v.mission_activity(a)
        self.assertEqual(info.command_id, 42)
        self.assertEqual(v.payload("radar").state, fsim.PayloadState.OPERATE)  # (its card gives no warm-up yet: at once)
        self.assertEqual(info.state, fsim.ActivityState.COMPLETED)
        with self.assertRaises(fsim.MissionRejected) as again:
            v.set_payload_state("radar", fsim.PayloadState.OPERATE)
        self.assertEqual(again.exception.reason, "already_in_state")
        b = v.payload_bit("radar")
        self.assertEqual(v.mission_activity(b).reason, "goal_reached")
        v.fail_payload("radar")
        self.assertEqual(v.payload("radar").state, fsim.PayloadState.FAILED)
        with self.assertRaises(fsim.MissionRejected) as failed:
            v.set_payload_state("radar", "standby")
        self.assertEqual(failed.exception.reason, "payload_failed")
        v.fail_payload("radar", failed=False)
        self.assertEqual(v.payload("radar").state, fsim.PayloadState.OFF)
        self.assertGreaterEqual(len(v.mission_activities), 2)
        with self.assertRaises(fsim.MissionRejected) as unknown:
            v.set_payload_state("nose", "standby")
        self.assertEqual(unknown.exception.reason, "unknown_payload")

    def test_grants(self):
        w = make_world()
        v = viper(w)
        v.set_control_mode("granted")
        with self.assertRaises(fsim.MissionRejected) as refused:
            v.set_payload_state("rwr", "operate")
        self.assertEqual(refused.exception.reason, "not_granted")
        v.request_control("fsim.ms.status:rwr", controller=5)
        v.set_payload_state("rwr", "operate", controller=5)
        self.assertEqual(v.payload("rwr").state, fsim.PayloadState.OPERATE)
        # (a capability carried but not modelled yet: the authority calls say why, as for a flight capability the vehicle lacks)
        with self.assertRaises(fsim.Error) as later:
            v.request_control("fsim.ms.ea:cmds", controller=5)
        self.assertIn("not_implemented", str(later.exception))


if __name__ == "__main__":
    unittest.main()
