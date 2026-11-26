"""Own-platform equipment parity and delivered-consent reports through public Python Agents."""
import math
import os
import unittest

import fsim


def setup(name):
    world = fsim.World(name, publish=False, terrain=False, workers=1, pin_workers=False, seed=7)
    aircraft = world.create_vehicle("blue", "jsbsim:f16c", latitude_deg=40.0, longitude_deg=0.0,
                                    altitude_msl_m=6000.0, airspeed_ms=230.0)
    aircraft.load_loadout("mk82 x4")
    return world, aircraft, world.create_agent(aircraft, "blue")


class MAEquipmentTest(unittest.TestCase):
    def test_native_lost_store_status_has_a_cross_language_name(self):
        self.assertEqual(fsim._native.store_status_name(7), "lost")
        self.assertEqual(fsim._native.store_status_name(8), "")

    def test_equipment_reports_use_the_existing_isolated_agent_channel(self):
        import fsim.isolation

        world, aircraft, agent = setup("py-ma-equipment-isolated")
        dispenser = next(payload for payload in agent.payloads if payload.family == "countermeasures")
        agent.set_payload_state(dispenser.index, "operate")
        agent.set_payload_state("stations", "operate")
        world.step()
        activity = agent.strike("mk82", 1)
        world.step()
        direct = (agent.report_stores(), agent.report_strikes(),
                  agent.report_countermeasures(), agent.report_consents())
        self.assertEqual(direct[1].items[0].value.activity, activity)
        with fsim.isolation.AgentProcess(world, agent, "equipment_policy:EquipmentProbe",
                                         path=[os.path.dirname(__file__)]) as process:
            self.assertEqual(process.call("read"), direct)
            after = process.call("dispense")
            self.assertEqual(after, agent.report_countermeasures())
            self.assertEqual(after.value.chaff, direct[2].value.chaff - 1)
            self.assertEqual(after.value.flare, direct[2].value.flare)

    def test_inventory_and_cartridge_release_use_native_state(self):
        world, aircraft, agent = setup("py-ma-equipment-inventory")
        stores = agent.report_stores()
        self.assertEqual(stores.availability, "available")
        self.assertEqual([item.value for item in stores.items], agent.stores)
        self.assertEqual(len(stores.items), 4)
        self.assertEqual(agent.report_stores(), stores)
        for item in stores.items:
            self.assertEqual((item.meta.source, item.meta.validity), ("own_platform", "valid"))
            self.assertEqual(item.meta.receive_time_s, world.time)
        before = agent.countermeasures()
        self.assertEqual(before, world.countermeasures(aircraft))
        inventory = agent.report_countermeasures()
        self.assertEqual((inventory.availability, inventory.value), ("available", before))
        self.assertEqual(inventory.meta.source, "own_platform")
        self.assertGreater(before.chaff, 0)
        with self.assertRaises(fsim.Error):
            agent.dispense("chaff")
        self.assertEqual(agent.countermeasures(), before)
        dispenser = next(payload for payload in agent.payloads if payload.family == "countermeasures")
        agent.set_payload_state(dispenser.index, "operate")
        world.step()
        agent.dispense("chaff")
        after = agent.countermeasures()
        self.assertEqual((after.chaff, after.flare), (before.chaff - 1, before.flare))
        self.assertEqual(agent.report_countermeasures().value, after)
        with self.assertRaises(ValueError):
            agent.dispense("burst")
        self.assertEqual(agent.countermeasures(), after)
        self.assertEqual(agent._world._h.weapons(), [])  # The legacy count getter refuses with no values.
        self.assertFalse(hasattr(agent, "engagement_events"))
        aircraft.remove()
        self.assertEqual(agent.report_stores(), fsim.ReportCollection("unavailable", []))
        self.assertEqual(agent.report_strikes(), fsim.ReportCollection("unavailable", []))
        self.assertEqual(agent.report_countermeasures(), fsim.Report("unavailable", None, None))

    def test_reports_preserve_activity_identity_and_delivered_consent_scope(self):
        world, aircraft, agent = setup("py-ma-equipment-consent")
        crc = world.create_c2_node("crc", 39.7, 0.0, 200.0)
        net = world.create_net("l16")
        self.assertTrue(world.join_net(aircraft, net) and world.join_net(crc, net))
        owner, observer = world.create_agent(crc, "ingress"), world.create_agent(crc, "observer")
        world.bind_ma_service(owner, 1, (), owns_dms_ingress=True, owns_dms_egress=True)
        world.bind_ma_service(observer, 2, (), owns_dms_ingress=False, owns_dms_egress=False)
        owner.declare_role(fsim.ROLE_QB, "CRC")
        world.step(30)
        agent.set_payload_state("stations", "operate")
        world.step()
        activity = agent.strike("mk82", 2)
        self.assertEqual(owner.report_consents(), fsim.ReportCollection("available", []))
        world.step(360)
        report = agent.report_strikes()
        self.assertEqual(report.availability, "available")
        self.assertEqual(len(report.items), 1)
        item = report.items[0]
        self.assertEqual(item.value.activity, activity)
        self.assertEqual(item.value.status, agent.strike_status(activity))
        self.assertEqual(item.value.status.phase, "awaiting_consent")
        self.assertEqual(len(item.value.status.stores), 2)
        self.assertEqual(item.meta.source, "own_platform")
        self.assertEqual(agent.report_strikes(), report)
        self.assertEqual(agent.report_strikes(stale_after_s=-1).items[0].meta.validity, "stale")
        [consent] = owner.report_consents().items
        self.assertEqual(consent.value, owner.consent_requests()[0])
        self.assertEqual((consent.value.source, consent.value.strike, consent.value.quantity), (agent.system, activity, 2))
        self.assertEqual((consent.meta.source, consent.meta.system), ("dms", agent.system))
        self.assertEqual((consent.meta.sample_time_s, consent.meta.receive_time_s),
                         (consent.value.time_s, consent.value.received_s))
        self.assertEqual(owner.report_consents(), owner.report_consents())
        self.assertEqual(observer.report_consents(), fsim.ReportCollection("not_owner", []))
        with self.assertRaises(fsim.Error):
            observer._world._h.agent_consent_reports(crc)
        self.assertEqual(len(observer.consent_requests()), 1)
        self.assertTrue(owner.answer_consent(agent.system, consent.value.request_id, True))
        world.step(360)
        [released] = agent.report_strikes().items
        self.assertEqual(released.value.activity, activity)
        self.assertEqual(released.value.status, agent.strike_status(activity))
        self.assertEqual(released.value.status.phase, "done")
        self.assertEqual(len(released.value.status.weapons), 2)
        self.assertEqual(released.value.status.weapons, [weapon.id for weapon in world.weapons if weapon.activity == activity])
        self.assertTrue(owner.report_consents().items[0].value.answered)
        self.assertEqual(owner.report_stores(), fsim.ReportCollection("not_applicable", []))
        self.assertEqual(owner.report_strikes(), fsim.ReportCollection("not_applicable", []))
        self.assertEqual(owner.report_countermeasures(), fsim.Report("not_applicable", None, None))

    def test_launch_zone_refuses_true_entity_and_foreign_scope(self):
        world, aircraft, agent = setup("py-ma-equipment-zone")
        other = world.create_vehicle("other", "jsbsim:f16c", latitude_deg=40.1, altitude_msl_m=6000.0, airspeed_ms=230.0)
        with self.assertRaises(fsim.Error):
            agent.launch_zone("mk82", other.id)
        with self.assertRaises(fsim.Error):
            agent.launch_zone_v2("mk82", other.id)
        with self.assertRaises(fsim.Error):
            agent.launch_zone_wez("mk82", other.id)
        with self.assertRaises(fsim.Error):
            agent._world._h.countermeasures(other.id)
        with self.assertRaises(fsim.Error):
            agent._world._h.agent_report(other.id, 8, math.nan)

    def test_range_created_strike_cannot_expose_a_true_target(self):
        world, aircraft, agent = setup("py-ma-equipment-range-target")
        other = world.create_vehicle("other", "jsbsim:f16c", latitude_deg=40.1, altitude_msl_m=6000.0, airspeed_ms=230.0)
        aircraft.set_payload_state("stations", "operate")
        world.step()
        activity = aircraft.strike("mk82", target=other, consent=True)
        self.assertEqual(aircraft.strike_status(activity).target, other.id)
        [reported] = agent.report_strikes().items
        self.assertEqual(reported.value.activity, activity)
        self.assertEqual(reported.value.status.target, 0)

    def test_optical_ir_search_track_and_laser_ranging(self):
        world = fsim.World("py-ma-optical-ir", publish=False, terrain=False, workers=1, pin_workers=False, seed=31)
        f35 = world.create_vehicle("f35a", "jsbsim:f35a", latitude_deg=35.0, longitude_deg=-117.0,
                                   altitude_msl_m=7000.0, heading_deg=90.0, airspeed_ms=230.0)
        target = world.create_vehicle("target", "jsbsim:f16c", latitude_deg=35.0,
                                      longitude_deg=-117.0 + 12000.0 / (6371000.0 * math.cos(math.radians(35.0))) * 180.0 / math.pi,
                                      altitude_msl_m=7000.0, heading_deg=90.0, airspeed_ms=230.0)
        agent = world.create_agent(f35, "f35-agent")
        agent.set_payload_state("eots", "operate")
        for _ in range(300):
            if next(p for p in agent.payloads if p.name == "eots").state == "operate":
                break
            world.step(1)
        agent.search_optical("eots", azimuth_half_width_rad=math.radians(30.0), elevation_half_width_rad=math.radians(20.0))
        world.step(30)
        contacts = agent.contacts("eots")
        self.assertGreaterEqual(len(contacts), 1)
        self.assertEqual(contacts[0].truth, 0)
        self.assertTrue(math.isnan(contacts[0].range_m))
        agent.track_optical("eots", contacts[0].id)
        world.step(30)
        tracked = agent.contacts("eots")
        self.assertGreaterEqual(len(tracked), 1)
        self.assertTrue(math.isfinite(tracked[0].range_m))
        self.assertAlmostEqual(tracked[0].range_m, 12000.0, delta=600.0)
        rep = {kv.name: kv.value for kv in agent.payload_report("eots")}
        self.assertEqual(rep["laser_capable"], 1.0)
        self.assertEqual(rep["laser_ranged"], 1.0)
        self.assertEqual(target.id > 0, True)

    def test_ms11_weather_sar_and_pnt_through_python_agent(self):
        world = fsim.World("py-ma-ms11", publish=False, terrain=False, workers=1, pin_workers=False, seed=41)
        u2 = world.create_vehicle("u2s", "jsbsim:u2s", latitude_deg=35.0, longitude_deg=-117.0,
                                  altitude_msl_m=18000.0, heading_deg=90.0, airspeed_ms=200.0)
        c130 = world.create_vehicle("c130j", "jsbsim:c130j", latitude_deg=35.0, longitude_deg=-117.0,
                                    altitude_msl_m=3000.0, heading_deg=90.0, airspeed_ms=140.0)
        u2_agent = world.create_agent(u2, "u2-agent")
        c130_agent = world.create_agent(c130, "c130-agent")
        u2_agent.set_payload_state("sar", "operate")
        c130_agent.set_payload_state("radar", "operate")
        for _ in range(300):
            if (next(p for p in u2_agent.payloads if p.name == "sar").state == "operate" and
                    next(p for p in c130_agent.payloads if p.name == "radar").state == "operate"):
                break
            world.step(1)
        world.set_environment(cloud_base_m=2000.0, cloud_cover=0.9, precipitation=0.9)
        c130_agent.search_weather("radar", range_scale_m=120000.0)
        lat_south = math.radians(35.0 - 45000.0 / 6371000.0 * 180.0 / math.pi)
        sar_act = u2_agent.image_sar("sar", lat_south, math.radians(-117.0), resolution_m=1.0)
        world.step(200)
        wx_contacts = c130_agent.contacts("radar")
        self.assertGreaterEqual(len(wx_contacts), 1)
        self.assertEqual(wx_contacts[0].kind, fsim.ContactKind.WEATHER)
        sar_imgs = u2_agent.contacts("sar")
        self.assertEqual(len(sar_imgs), 1)
        self.assertEqual(sar_imgs[0].kind, fsim.ContactKind.IMAGE)
        self.assertAlmostEqual(sar_imgs[0].size_m, 1.0, places=3)
        self.assertEqual(u2_agent.mission_activity(sar_act).state, fsim.ActivityState.COMPLETED)

    def test_sim_m10_gun_firing_and_b33_mass_conservation_through_python_agent(self):
        world = fsim.World("py-ma-gun", publish=False, terrain=False, workers=1, pin_workers=False, seed=51)
        f16 = world.create_vehicle("f16c", "jsbsim:f16c", latitude_deg=35.0, longitude_deg=-117.0,
                                   altitude_msl_m=5000.0, heading_deg=90.0, airspeed_ms=220.0)
        twin = world.create_vehicle("twin", "jsbsim:f16c", latitude_deg=35.05, longitude_deg=-117.0,
                                    altitude_msl_m=5000.0, heading_deg=90.0, airspeed_ms=220.0)
        agent = world.create_agent(f16, "f16-agent")
        agent.set_payload_state("gun", "operate")
        for _ in range(100):
            if next(p for p in agent.payloads if p.name == "gun").state == "operate":
                break
            world.step(1)
        world.step(5)
        burst = agent.fire_gun("gun", rounds=100)
        world.step(80)
        self.assertEqual(agent.mission_activity(burst).state, fsim.ActivityState.COMPLETED)
        rep = {kv.name: kv.value for kv in agent.payload_report("gun")}
        self.assertEqual(rep["rounds_fired"], 100.0)
        self.assertEqual(rep["rounds_remaining"], 400.0)
        self.assertAlmostEqual(twin.mass_kg - f16.mass_kg, 15.5, delta=0.6)


if __name__ == "__main__":
    unittest.main()
