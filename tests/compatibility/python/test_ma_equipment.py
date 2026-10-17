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


if __name__ == "__main__":
    unittest.main()
