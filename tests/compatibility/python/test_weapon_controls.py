"""Actual native fin telemetry is a Range query, never an actor observation."""
import unittest
import fsim


class WeaponControlsTest(unittest.TestCase):
    def test_unknown_and_agent_queries_refuse(self):
        with fsim.World("fin-permissions", publish=False, terrain=False, workers=1) as world:
            carrier = world.create_vehicle("carrier", "jsbsim:f16c", altitude_msl_m=6000, airspeed_ms=250)
            self.assertIsNone(world.weapon_controls(carrier))
            self.assertIsNone(world.weapon_controls(0))
            agent = world.create_agent(carrier, "actor")
            with self.assertRaises(fsim.Error):
                agent._world.weapon_controls(carrier.id)


if __name__ == "__main__":
    unittest.main()
