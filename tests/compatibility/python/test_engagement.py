"""Native engagement contracts; no client-side missile simulation."""
import unittest
import fsim


class EngagementTest(unittest.TestCase):
    def test_launch_rejects_foreign_vehicle_without_side_effects(self):
        with fsim.World("launch-owner", publish=False, workers=1, pin_workers=False) as world, \
                fsim.World("launch-foreign", publish=False, workers=1, pin_workers=False) as other:
            own = world.create_vehicle("carrier", "jsbsim:f16c", altitude_msl_m=6000, airspeed_ms=250)
            foreign = other.create_vehicle("carrier", "jsbsim:f16c", altitude_msl_m=6000, airspeed_ms=250)
            self.assertEqual(own.id, foreign.id)
            own.load_loadout("mk82 x4")
            foreign.load_loadout("mk82 x4")
            before = [(w.vehicle_ids().tolist(), w.time, w.vehicle_steps, v.stores, v.mass_kg,
                       tuple(v.state.position_ecef), tuple(v.state.velocity_ned_ms))
                      for w, v in ((world, own), (other, foreign))]
            with self.assertRaises(fsim.Error):
                world.launch_weapon(foreign)
            for (w, v), expected in zip(((world, own), (other, foreign)), before):
                self.assertEqual((w.vehicle_ids().tolist(), w.time, w.vehicle_steps, v.stores, v.mass_kg,
                                  tuple(v.state.position_ecef), tuple(v.state.velocity_ned_ms)), expected)
                self.assertEqual(w.weapons, [])
                self.assertEqual(w.engagement_events(), [])
            released = world.launch_weapon(own)
            self.assertEqual(released.id, own.id + 1)
            self.assertEqual(released.role, "weapon")
            self.assertEqual(released.launcher, own.id)
            self.assertEqual(other.weapons, [])

    def test_launch_accepts_handle_alias_and_numeric_id(self):
        with fsim.World("launch-alias", publish=False, workers=1, pin_workers=False) as world:
            own = world.create_vehicle("carrier", "jsbsim:f16c", altitude_msl_m=6000, airspeed_ms=250)
            alias = fsim.World(_handle=world._h)
            first = alias.launch_weapon(own)
            second = world.launch_weapon(alias.vehicle(own.id))
            third = world.launch_weapon(own.id)
            self.assertEqual([first.id, second.id, third.id], [own.id + 1, own.id + 2, own.id + 3])
            for released in (first, second, third):
                self.assertEqual(released.role, "weapon")
                self.assertEqual(released.launcher, own.id)
            own.remove()
            before = world.vehicle_ids().tolist()
            with self.assertRaises(fsim.Error):
                world.launch_weapon(own)
            self.assertEqual(world.vehicle_ids().tolist(), before)
            self.assertEqual(len(world.weapons), 3)

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
