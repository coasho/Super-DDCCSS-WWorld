"""The limits that follow the load, and an activity's abnormal states (docs/vehicle-interface.md, 7.3): a wing's least
airspeed rises with what it carries, and its profile is given at that weight; commanded slower than it can hold level, it
keeps its height on the speed floor and says so; a turn at a low airspeed is flown within the bank that speed leaves; an
aircraft pulled from the altitude it commands says that it deviates."""
import math
import types
import unittest

import fsim
from fsim import agra


def make_world():
    return fsim.World("py-load", publish=False, workers=1, pin_workers=False, seed=5, terrain=False)


def seconds(w, s):
    w.step(int(round(s / w.step_seconds)))


def loaded_f16c(w, altitude_m=3000.0):
    v = w.create_vehicle("viper", "jsbsim:f16c", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=altitude_m, airspeed_ms=165.0, heading_deg=90.0)
    w.step()
    spawned = types.SimpleNamespace(mass_kg=v.mass_kg, min_cas_ms=v.performance.min_cas_ms, revision=v.control_revision)
    v.load_loadout("air superiority")
    v.load_loadout("external tanks")
    w.step(2)
    return v, spawned


class LoadLimits(unittest.TestCase):
    def test_the_flags(self):
        self.assertEqual((int(fsim.ActivityFlag.LOW_ENERGY), int(fsim.ActivityFlag.DEVIATING), int(fsim.ActivityFlag.UPSET)), (64, 128, 256))
        active = lambda flags: types.SimpleNamespace(state=fsim.ActivityState.ACTIVE, constraints=int(flags))  # noqa: E731
        self.assertEqual(agra.activity_state(active(fsim.ActivityFlag.LOW_ENERGY)), "ACTIVE_PARTIALLY_CONSTRAINED")
        self.assertEqual(agra.activity_state(active(fsim.ActivityFlag.DEVIATING)), "ACTIVE_FULLY_CONSTRAINED")
        self.assertEqual(agra.activity_state(active(fsim.ActivityFlag.UPSET)), "ACTIVE_FULLY_CONSTRAINED")

    def test_the_least_airspeed_follows_the_load(self):
        w = make_world()
        v, spawned = loaded_f16c(w)
        ratio = math.sqrt(v.mass_kg / spawned.mass_kg)
        self.assertGreater(ratio, 1.15)
        self.assertAlmostEqual(v.performance.min_cas_ms, spawned.min_cas_ms * ratio, delta=0.003 * spawned.min_cas_ms)
        self.assertGreater(v.control_revision, spawned.revision)
        # the least a command may ask keeps its margin over that: one below is refused under REJECT, one at it flown
        least = v.performance.least_cas_ms
        self.assertAlmostEqual(least, 1.1 * v.performance.min_cas_ms, places=9)
        for below in (spawned.min_cas_ms, v.performance.min_cas_ms, least - 0.5):
            with self.assertRaises(fsim.Rejected) as refused:
                v.submit_hsa(speed=below, speed_reference="calibrated_airspeed", range=fsim.RangePolicy.REJECT)
            self.assertEqual(refused.exception.reason, "performance_limit")
            self.assertEqual(refused.exception.constraint, "min_airspeed")
        v.submit_hsa(speed=least + 0.01, speed_reference="calibrated_airspeed", range=fsim.RangePolicy.REJECT)
        # its profile at the weight it has: none was given beyond its tables' heaviest row
        profile = v.performance_profile("hsa_csa")
        self.assertAlmostEqual(profile.weight_kg, v.mass_kg, delta=5.0)
        self.assertTrue(profile.min_airspeed)
        self.assertTrue(profile.max_airspeed)
        # unloaded: as it spawned
        for station in sorted({s.station for s in v.stores}):
            v.load_store(station, None)
        w.step(2)
        self.assertAlmostEqual(v.performance.min_cas_ms, spawned.min_cas_ms, delta=1e-9)

    def test_slower_than_it_can_hold_level_it_keeps_its_height_and_says_so(self):
        # a Mirage 2000 with its tanks and missiles: it cannot hold its height at the least airspeed it may be commanded
        w = make_world()
        v = w.create_vehicle("mirage", "jsbsim:mirage2000", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=3000.0, airspeed_ms=165.0,
                             heading_deg=90.0)
        w.step()
        v.load_loadout("air defence")
        v.load_loadout("external tanks")
        w.step(2)
        asked = v.performance.least_cas_ms
        a = v.submit_hsa(heading_rad=math.pi / 2, altitude_m=3000.0, speed=asked, speed_reference="calibrated_airspeed")
        lowest, seen = 3000.0, 0
        for _ in range(300):
            seconds(w, 1.0)
            lowest = min(lowest, v.state.altitude_msl_m)
            seen |= int(a.info.constraints)
        s = v.state
        self.assertFalse(s.crashed)
        self.assertGreater(lowest, 2940.0)
        self.assertLess(abs(s.altitude_msl_m - 3000.0), 60.0)
        self.assertEqual(a.info.state, fsim.ActivityState.ACTIVE)
        if s.airspeed_calibrated_ms > asked + 2.0:  # (faster than asked: on its speed floor, and said)
            self.assertTrue(a.info.constraints & fsim.ActivityFlag.LOW_ENERGY)
            self.assertIn(agra.activity_state(a.info), ("ACTIVE_PARTIALLY_CONSTRAINED", "ACTIVE_FULLY_CONSTRAINED"))
        else:
            self.assertFalse(a.info.constraints & fsim.ActivityFlag.LOW_ENERGY)
        self.assertFalse(seen & fsim.ActivityFlag.UPSET)

    def test_a_slow_turn_is_flown_within_the_bank_its_speed_leaves(self):
        w = make_world()
        v, _ = loaded_f16c(w)
        least = v.performance.min_cas_ms
        a = v.submit_hsa(heading_rad=math.pi / 2, altitude_m=3000.0, speed=1.3 * least, speed_reference="calibrated_airspeed")
        seconds(w, 150.0)
        self.assertLess(abs(v.state.airspeed_calibrated_ms - 1.3 * least), 4.0)
        # (its profile and its performance say the bank that speed leaves: 39.7 deg at 1.3 times its least)
        self.assertAlmostEqual(math.degrees(v.performance_profile("hsa_csa").max_orientation[0].roll_rad), 39.7, delta=3.0)
        self.assertAlmostEqual(math.degrees(v.performance.turn_bank_rad(1.3 * least)), 39.7, delta=0.1)
        a.update(heading_rad=math.pi / 2 + math.radians(60.0))
        bank, lowest, over = 0.0, 3000.0, 0
        for _ in range(600):
            seconds(w, 0.1)
            bank = max(bank, abs(v.state.euler_rad[0]))
            over += abs(v.state.euler_rad[0]) > math.radians(45.0)
            lowest = min(lowest, v.state.altitude_msl_m)
        # (loaded, its roll overshoots the bank it is held to, for a second or two; it would stall there at 53.7 deg)
        self.assertLess(math.degrees(bank), 53.0)
        self.assertLess(over, 30)
        self.assertGreater(lowest, 2940.0)
        self.assertLess(abs(math.degrees(v.state.euler_rad[2]) - 150.0), 3.0)
        self.assertFalse(a.info.constraints_seen & fsim.ActivityFlag.UPSET)

    def test_its_engine_out_it_says_that_it_deviates_and_is_low_on_energy(self):
        # its tanks run dry, slow: its hsa sinks from the altitude it commands, its speed floor asking for a speed it cannot make
        w = make_world()
        v = w.create_vehicle("skyhawk", "jsbsim:c172", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=2000.0, airspeed_ms=45.0, heading_deg=90.0)
        w.step()
        asked = v.performance.least_cas_ms
        a = v.submit_hsa(heading_rad=math.pi / 2, altitude_m=2000.0, speed=asked, speed_reference="calibrated_airspeed")
        seconds(w, 60.0)
        self.assertFalse(a.info.constraints_seen & (fsim.ActivityFlag.DEVIATING | fsim.ActivityFlag.UPSET))
        for tank in range(2):
            v.set_property("propulsion/tank[%d]/contents-lbs" % tank, 0.0)
        seen = 0
        for _ in range(120):
            seconds(w, 1.0)
            seen |= int(a.info.constraints)
        self.assertLess(v.state.altitude_msl_m, 1950.0)
        self.assertFalse(v.state.crashed)
        self.assertTrue(seen & fsim.ActivityFlag.DEVIATING)
        self.assertTrue(seen & fsim.ActivityFlag.LOW_ENERGY)  # (asked for more speed than its command: it has no thrust to make it)
        self.assertEqual(a.info.state, fsim.ActivityState.ACTIVE)
        self.assertEqual(agra.activity_state(a.info), "ACTIVE_FULLY_CONSTRAINED")


if __name__ == "__main__":
    unittest.main()
