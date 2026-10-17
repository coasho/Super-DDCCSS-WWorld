"""Public weapon telemetry, including inert releases and unknown values."""
import math
import unittest

import fsim


class WeaponTelemetryTest(unittest.TestCase):
    def test_phase_names_and_inert_defaults(self):
        self.assertTrue(hasattr(fsim, "WeaponPhase"), "SDK must expose WeaponPhase")
        with fsim.World("weapon-telemetry", publish=False, workers=1, terrain=False) as world:
            carrier = world.create_vehicle("carrier", "jsbsim:f16c", altitude_msl_m=5000, airspeed_ms=200)
            weapon = world.launch_weapon(carrier, "jsbsim:mk82")
            record = next(r for r in world.weapons if r.id == weapon.id)
            self.assertEqual(record.phase, fsim.WeaponPhase.UNGUIDED)
            self.assertEqual(record.seeker, "")
            self.assertEqual(record.holding, 0)
            self.assertTrue(math.isnan(record.miss_m))
            self.assertFalse(record.tracking)
            self.assertFalse(record.supported)
            self.assertEqual(record.end, "in_flight")

    def test_phase_enum_values_are_stable(self):
        self.assertTrue(hasattr(fsim, "WeaponPhase"), "SDK must expose WeaponPhase")
        self.assertEqual([int(p) for p in fsim.WeaponPhase], [0, 1, 2, 3, 4])


if __name__ == "__main__":
    unittest.main()
