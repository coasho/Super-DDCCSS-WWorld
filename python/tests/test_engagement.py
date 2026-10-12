"""Native engagement contracts; no client-side missile simulation."""
import unittest
import fsim


class EngagementTest(unittest.TestCase):
    def test_dispenser_inventory_and_event_cursor(self):
        with fsim.World(publish=False, workers=1, dt=0.05, frame_skip=1) as world:
            carrier = world.create_vehicle("carrier", "jsbsim:f16c", altitude_msl_m=6000, airspeed_ms=250)
            cmds = next(p for p in carrier.payloads if p.family == "countermeasures")
            carrier.set_payload_state(cmds.name, "operate")
            world.step(30)
            before = world.countermeasures(carrier)
            self.assertGreater(before.chaff, 0)
            world.dispense(carrier, "chaff")
            self.assertEqual(world.countermeasures(carrier).chaff, before.chaff - 1)
            events = world.engagement_events()
            self.assertEqual(events[-1].kind, "chaff")
            self.assertEqual(world.engagement_events(), events)
            self.assertEqual(world.engagement_events(since=events[-1].serial), [])
            self.assertEqual(len(world.decoys()), 1)
            with self.assertRaises(fsim.Error):
                world.dispense(carrier, "flare")
            self.assertEqual(world.countermeasures(carrier).flare, before.flare)
            world.step(121)
            self.assertEqual(world.decoys(), [])

    def test_unknown_target_has_no_launch_zone(self):
        with fsim.World(publish=False, workers=1) as world:
            carrier = world.create_vehicle("carrier", "jsbsim:f16c")
            with self.assertRaises(fsim.Error):
                world.launch_zone(carrier, "aim-120c", 999999)

    def test_released_decoy_outlives_its_carrier(self):
        with fsim.World(publish=False, workers=1, dt=0.05, frame_skip=1) as world:
            carrier = world.create_vehicle("carrier", "jsbsim:f16c", altitude_msl_m=6000, airspeed_ms=250)
            cmds = next(p for p in carrier.payloads if p.family == "countermeasures")
            carrier.set_payload_state(cmds.name, "operate")
            world.step(30)
            world.dispense(carrier, "flare")
            released = world.decoys()[0].id
            carrier.remove()
            world.step()
            self.assertEqual(world.decoys()[0].id, released)
            world.step(81)
            self.assertEqual(world.decoys(), [])

    def test_agent_handle_cannot_read_engagement_truth(self):
        with fsim.World(publish=False, workers=1) as world:
            carrier = world.create_vehicle("carrier", "jsbsim:f16c")
            agent = world.create_agent(carrier, "own-agent")
            # Count-style C refusals return empty snapshots; status-style queries raise an error.
            self.assertEqual(agent._world.weapons, [])
            self.assertEqual(agent._world.engagement_events(0), [])
            self.assertEqual(agent._world.decoys(), [])
            with self.assertRaises(fsim.Error):
                agent._world.seeker_target(carrier.id, "aim-9x")


if __name__ == "__main__":
    unittest.main()
