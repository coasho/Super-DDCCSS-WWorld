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
            with self.assertRaises(fsim.Error):
                world.launch_zone_v2(carrier, "aim-120c", 999999)
            with self.assertRaises(fsim.Error):
                world.launch_zone_wez(carrier, "aim-120c", 999999)

    def test_flyout_launch_zone_v2_and_wez_for_aam(self):
        with fsim.World(publish=False, workers=1, terrain=False) as world:
            carrier = world.create_vehicle("carrier", "jsbsim:f16c", latitude_deg=40.0,
                                           altitude_msl_m=10000.0, heading_deg=0.0, airspeed_ms=300.0)
            target = world.create_vehicle("target", "jsbsim:f16c", latitude_deg=40.2,
                                          altitude_msl_m=10000.0, heading_deg=180.0, airspeed_ms=265.0)
            carrier.load_store(1, "aim-120c")
            z2 = world.launch_zone_v2(carrier, "aim-120c", target)
            self.assertIsInstance(z2, fsim.LaunchZoneV2)
            self.assertTrue(z2.valid)
            self.assertTrue(z2.flyout_derived)
            self.assertEqual(z2.basis, "simulated")
            self.assertEqual(len(z2.table_sha256), 64)
            self.assertGreater(z2.maximum_m, 40000.0)
            self.assertGreater(z2.no_escape_m, 10000.0)
            self.assertLessEqual(z2.no_escape_m, z2.maximum_m)
            self.assertGreater(z2.maximum_uncertainty_m, 0.0)
            z1 = world.launch_zone(carrier, "aim-120c", target)
            self.assertAlmostEqual(z1.maximum_m, z2.maximum_m, places=3)
            self.assertAlmostEqual(z1.no_escape_m, z2.no_escape_m, places=3)
            wez = world.launch_zone_wez(carrier, "aim-120c", target)
            self.assertIsInstance(wez, fsim.LaunchZoneWezV1)
            self.assertTrue(wez.valid)
            self.assertTrue(wez.flyout_derived)
            self.assertEqual(wez.table_sha256, z2.table_sha256)
            self.assertGreaterEqual(len(wez.aspect_contour), 5)
            self.assertGreater(wez.aspect_contour[0].maximum_m, wez.aspect_contour[-1].maximum_m)

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

    def test_direct_agent_strike_status_redacts_range_target(self):
        with fsim.World("py-strike-status-visibility", publish=False, terrain=False, workers=1) as world:
            carrier = world.create_vehicle("carrier", "jsbsim:f16c", latitude_deg=40.0,
                                           altitude_msl_m=6000.0, airspeed_ms=230.0)
            other = world.create_vehicle("other", "jsbsim:f16c", latitude_deg=40.05,
                                         altitude_msl_m=6000.0, airspeed_ms=230.0)
            carrier.load_loadout("mk82 x4")
            agent = world.create_agent(carrier, "carrier")
            observer = world.create_agent(other, "observer")
            carrier.set_payload_state("stations", "operate")
            world.step()
            activity = carrier.strike("mk82", target=other)
            world.step()
            native = carrier.strike_status(activity)
            self.assertEqual(native.target, other.id)
            visible = agent.strike_status(activity)
            self.assertEqual(visible.target, 0)
            self.assertEqual(visible, native._replace(target=0))
            self.assertEqual(agent.strike_status(activity), visible)
            self.assertEqual(agent.report_strikes().items[0].value.status, visible)
            self.assertEqual(carrier.strike_status(activity), native)
            self.assertIsNone(observer.strike_status(activity))

    def test_sim_m05_electronic_attack_and_programmed_salvo(self):
        with fsim.World("py-sim-m05", publish=False, terrain=False, workers=1, dt=1.0 / 120.0, frame_skip=4) as world:
            f16 = world.create_vehicle("f16", "jsbsim:f16c", latitude_deg=35.0, longitude_deg=-117.0,
                                       altitude_msl_m=6000.0, heading_deg=90.0, airspeed_ms=220.0)
            growler = world.create_vehicle("ea18g", "jsbsim:ea18g", latitude_deg=35.0, longitude_deg=-116.80,
                                           altitude_msl_m=6000.0, heading_deg=270.0, airspeed_ms=220.0)
            f16.set_payload_state("rwr", "operate")
            f16.set_payload_state("cmds", "operate")
            growler.set_payload_state("jammers", "operate")
            world.step(5)
            f16_agent = world.create_agent(f16, "f16-agent")
            growler_agent = world.create_agent(growler, "growler-agent")

            jam_act = growler_agent.jam("jammers", frequency_hz=9.86e9, bandwidth_hz=40e6)
            self.assertGreater(jam_act, 0)
            rep = {v.name: v.value for v in growler_agent.payload_report("jammers")}
            self.assertEqual(rep.get("active"), 1.0)
            self.assertAlmostEqual(rep.get("frequency"), 9.86e9, delta=1e6)

            world.step(30)
            contacts = f16_agent.contacts("rwr")
            self.assertTrue(any(c.mode == fsim.EmitterMode.JAMMER and c.truth == 0 for c in contacts))

            before = f16_agent.countermeasures()
            salvo_act = f16_agent.dispense_salvo("cmds", chaff=1, flares=1, salvos=1, interval_s=0.5)
            self.assertGreater(salvo_act, 0)
            world.step(60)
            after = f16_agent.countermeasures()
            self.assertEqual(before.chaff - after.chaff, 1)
            self.assertEqual(before.flare - after.flare, 1)


if __name__ == "__main__":
    unittest.main()
