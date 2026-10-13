"""Explicit MA-service binding (docs/sdk/mission-training.md, section 9) through the Python SDK: configured ownership
whatever order the agents were made in, refused overlaps and foreign controllers, the legacy path as it was."""
import unittest

import fsim

HSA, ROUTE = "fsim.guidance.hsa", "fsim.guidance.route"


def value(x):
    return x() if callable(x) else x


def world(name):
    w = fsim.World(name, publish=False, workers=1, pin_workers=False, terrain=False)
    a = w.create_vehicle("viper", "jsbsim:f16c", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=6000.0, airspeed_ms=230.0)
    crc = w.create_c2_node("crc", 39.6, 0.0, 200.0)
    net = w.create_net("l16")
    for e in (a, crc):
        assert w.join_net(e, net)
    return w, a, crc


def amti(vehicle):
    radar = next(p.index for p in value(vehicle.payloads) if p.family == "air_radar")
    return next(c.id for c in value(vehicle.mission_capabilities) if c.payload == radar and c.type == "fsim.ms.amti")


def designate_everything(w, boss, ma_system):
    boss.declare_role(fsim.ROLE_AVC if hasattr(fsim, "ROLE_AVC") else 3, "AVC-1")
    w.step(10)
    command = boss.designate_control(ma_system, 0xFFFFFFFF)
    for _ in range(60):
        if any(s.command_id == command and s.topic == fsim.TOPIC_CONTROL_DESIGNATION for s in boss.command_statuses()):
            return
        w.step()
    raise AssertionError("the designation was not answered")


class MAServiceTest(unittest.TestCase):
    def configured(self, flight_first):
        w, a, crc = world("py-ma-binding-%d" % flight_first)
        if flight_first:
            flight, sensors = w.create_agent(a, "flight"), w.create_agent(a, "sensors")
        else:
            sensors, flight = w.create_agent(a, "sensors"), w.create_agent(a, "flight")
        boss = w.create_agent(crc, "crc")
        radar = amti(a)
        for service, agent, capabilities, ingress, egress in sorted(
                ((1, flight, (HSA, ROUTE), False, True), (2, sensors, (radar,), True, False)), reverse=not flight_first,
                key=lambda b: b[0]):
            w.bind_ma_service(agent, service, capabilities, owns_dms_ingress=ingress, owns_dms_egress=egress)
        self.assertEqual(value(a.control_mode), fsim.ControlMode.GRANTED)
        services = w.ma_services()
        self.assertEqual([s.service_id for s in services], [1, 2])
        self.assertIsInstance(services[0], fsim.MAService)
        by_controller = {s.controller: s.service_id for s in services}
        designate_everything(w, boss, flight.system)
        out = {"owner": {c: by_controller.get(w.ma_capability_owner(a, c), 0) for c in (HSA, ROUTE, radar)},
               "holder": {c: by_controller.get(a.control_status(c).holder, 0) if a.control_status(c).granted else 0 for c in (HSA, ROUTE)},
               "ingress": next(s.service_id for s in services if s.owns_dms_ingress),
               "egress": next(s.service_id for s in services if s.owns_dms_egress),
               "capabilities": {s.service_id: tuple(sorted(s.command_capabilities)) for s in services}}
        flight.submit_hsa(heading_rad=1.0, speed=230.0, altitude_m=6000.0)
        with self.assertRaises(fsim.Error):
            sensors.submit_hsa(heading_rad=2.0, speed=230.0, altitude_m=6000.0)
        with self.assertRaises(fsim.Rejected) as refused:
            sensors.request_control(HSA)
        self.assertIn("not_allowed", str(refused.exception))
        with self.assertRaises(fsim.Rejected):
            flight.request_control(radar)
        return out

    def test_ownership_is_the_bindings_whatever_order_the_agents_were_made_in(self):
        a, b = self.configured(True), self.configured(False)
        self.assertEqual(a, b)
        self.assertEqual(a["owner"][HSA], 1)
        self.assertEqual(a["holder"], {HSA: 1, ROUTE: 1})
        self.assertEqual((a["ingress"], a["egress"]), (2, 1))

    def test_refusals_change_nothing(self):
        w, a, crc = world("py-ma-binding-refusals")
        b = w.create_vehicle("wing", "jsbsim:f16c", latitude_deg=40.02, longitude_deg=0.0, altitude_msl_m=6000.0, airspeed_ms=230.0)
        one, two, other, boss = w.create_agent(a, "one"), w.create_agent(a, "two"), w.create_agent(b, "other"), w.create_agent(crc, "crc")

        class Forged:  # (an identity the World did not mint for that platform)
            def __init__(self, vehicle_id, controller):
                self.vehicle_id, self.controller = vehicle_id, controller

        for agent, service, capabilities in ((Forged(a.id, 0x40000099), 7, (HSA,)), (Forged(a.id, 0), 7, (HSA,)), (Forged(a.id, 7), 7, (HSA,)),
                                             (Forged(a.id, other.controller), 7, (HSA,)), (Forged(b.id, one.controller), 7, (HSA,)),
                                             (one, 0, (HSA,)), (one, 7, ("fsim.guidance.no_such",)), (one, 7, (HSA, HSA)),
                                             (one, 7, ("*", HSA)), (boss, 7, (HSA,)), (boss, 7, ("*",))):
            with self.assertRaises(fsim.Error, msg=(service, capabilities)):
                w.bind_ma_service(agent, service, capabilities)
        self.assertEqual(w.ma_services(), [])
        self.assertEqual(value(a.control_mode), fsim.ControlMode.OPEN)
        w.bind_ma_service(one, 7, (HSA,), owns_dms_ingress=True, owns_dms_egress=False)
        for agent, service, capabilities, ingress, egress in ((two, 7, (ROUTE,), False, False), (one, 8, (ROUTE,), False, False),
                                                             (two, 8, (HSA,), False, False), (two, 8, ("*",), False, False),
                                                             (two, 8, (ROUTE,), True, False)):
            with self.assertRaises(fsim.Error) as refused:
                w.bind_ma_service(agent, service, capabilities, owns_dms_ingress=ingress, owns_dms_egress=egress)
            self.assertIn("MA service", str(refused.exception))
        self.assertEqual(len(w.ma_services()), 1)
        w.bind_ma_service(two, 8, (ROUTE,), owns_dms_ingress=False, owns_dms_egress=True)
        w.bind_ma_service(boss, 30, (), owns_dms_ingress=True, owns_dms_egress=True)
        self.assertEqual([(s.service_id, s.platform) for s in w.ma_services()], [(7, a.id), (8, a.id), (30, crc)])
        self.assertEqual(w.ma_capability_owner(a, HSA), one.controller)
        self.assertEqual(w.ma_capability_owner(a, amti(a)), 0)
        # the range's alone: an agent's handle reads no binding and makes none
        self.assertEqual(one._world.ma_services(), [])
        self.assertEqual(one._world.ma_capability_owner(a.id, HSA), 0)
        with self.assertRaises(fsim.Error):
            one._world.bind_ma_service(two, 99, ())
        self.assertEqual(len(w.ma_services()), 3)

    def test_dms_sides(self):
        w, a, crc = world("py-ma-binding-dms")
        talker, listener = w.create_agent(a, "talker"), w.create_agent(a, "listener")
        boss = w.create_agent(crc, "crc")
        w.bind_ma_service(talker, 1, ("*",), owns_dms_ingress=False, owns_dms_egress=True)
        w.bind_ma_service(listener, 2, (), owns_dms_ingress=True, owns_dms_egress=False)
        data = bytes([0xFF, 0xFE, 0x00, 0x80])
        self.assertGreater(talker.send(fsim.TOPIC_FREE_FORM + 1, data, [boss.system]), 0)
        with self.assertRaises(fsim.Error):
            listener.send(fsim.TOPIC_FREE_FORM + 1, data, [boss.system])
        with self.assertRaises(fsim.Error):
            listener.publish_position()
        boss.send(fsim.TOPIC_FREE_FORM + 2, b"\x07", [talker.system])
        w.step(90)
        self.assertEqual(talker.inbox(), [])
        self.assertEqual(talker.receive(), [])
        got = listener.receive()
        self.assertTrue(any(m.topic == fsim.TOPIC_FREE_FORM + 2 and m.data == b"\x07" for m in got))
        self.assertEqual(listener.receive(), [])
        self.assertEqual(len(talker.peers()), len(listener.peers()))  # (the platform's picture is every service's alike)
        self.assertEqual(talker.system, listener.system)

    def test_a_platform_with_no_binding_is_as_before(self):
        w, a, crc = world("py-ma-binding-legacy")
        first, last = w.create_agent(a, "first"), w.create_agent(a, "last")
        boss = w.create_agent(crc, "crc")
        self.assertEqual(w.ma_services(), [])
        self.assertEqual(value(a.control_mode), fsim.ControlMode.OPEN)
        first.submit_hsa(heading_rad=1.0, speed=230.0, altitude_m=6000.0)
        last.submit_hsa(heading_rad=2.0, speed=230.0, altitude_m=6000.0)
        self.assertGreater(first.send(fsim.TOPIC_FREE_FORM, b"x"), 0)
        self.assertGreater(last.send(fsim.TOPIC_FREE_FORM, b"y"), 0)
        designate_everything(w, boss, first.system)
        self.assertEqual(a.control_status(HSA).holder, last.controller)  # (the agent made last, as ever)

    def test_a_grant_another_agent_held_follows_the_binding(self):
        w, a, crc = world("py-ma-binding-existing-grant")
        legacy, owner = w.create_agent(a, "legacy"), w.create_agent(a, "owner")
        legacy.request_control(HSA)
        flown = legacy.submit_hsa(heading_rad=1.0, speed=230.0, altitude_m=6000.0)
        self.assertEqual(a.control_status(HSA).holder, legacy.controller)
        w.bind_ma_service(owner, 1, (HSA,))
        self.assertEqual(w.ma_capability_owner(a, HSA), owner.controller)
        self.assertEqual(a.control_status(HSA).holder, owner.controller)  # (the grant is the configured owner's)
        self.assertEqual(value(a.control_mode), fsim.ControlMode.GRANTED)
        self.assertFalse(value(flown.live))
        with self.assertRaises(fsim.Rejected) as refused:
            legacy.submit_hsa(heading_rad=2.0, speed=230.0, altitude_m=6000.0)
        self.assertIn("not_granted", str(refused.exception))
        with self.assertRaises(fsim.Rejected) as refused:
            legacy.request_control(HSA)
        self.assertIn("not_allowed", str(refused.exception))
        owner.submit_hsa(heading_rad=1.0, speed=230.0, altitude_m=6000.0)
        # neither the default policy nor an open mode gets around it
        with self.assertRaises(fsim.Error):
            a.request_control(HSA)
        with self.assertRaises(fsim.Error):
            a.set_control_mode(fsim.ControlMode.OPEN)
        self.assertEqual(value(a.control_mode), fsim.ControlMode.GRANTED)
        self.assertEqual(a.control_status(HSA).holder, owner.controller)

    def test_an_agent_of_another_world_is_refused_and_nothing_changes(self):
        (first, a1, _), (second, a2, _) = world("py-ma-binding-world-one"), world("py-ma-binding-world-two")
        one, two = first.create_agent(a1, "one"), second.create_agent(a2, "two")
        # the two worlds minted the same numbers: only the agent's own world tells them apart
        self.assertEqual((a1.id, one.controller), (a2.id, two.controller))
        with self.assertRaises(fsim.Error) as refused:
            first.bind_ma_service(two, 1, (HSA,))
        self.assertIn("MA service 1", str(refused.exception))
        self.assertEqual(first.ma_services(), [])
        self.assertEqual(second.ma_services(), [])
        self.assertEqual(value(a1.control_mode), fsim.ControlMode.OPEN)
        self.assertEqual(first.ma_capability_owner(a1, HSA), 0)
        self.assertFalse(a1.control_status(HSA).granted)
        one.submit_hsa(heading_rad=1.0, speed=230.0, altitude_m=6000.0)  # (still open: nothing was bound)
        # neither the range's own handle nor something that only looks like an agent is one
        with self.assertRaises(fsim.Error):
            first.bind_ma_service(first, 1, (HSA,))
        self.assertEqual(first.ma_services(), [])
        # its own agent binds, each world its own
        first.bind_ma_service(one, 1, (HSA,))
        second.bind_ma_service(two, 1, (HSA,))
        self.assertEqual([(s.service_id, s.platform, s.controller) for s in first.ma_services()], [(1, a1.id, one.controller)])
        self.assertEqual([(s.service_id, s.platform, s.controller) for s in second.ma_services()], [(1, a2.id, two.controller)])
        self.assertEqual(a1.control_status(HSA).holder, one.controller)


if __name__ == "__main__":
    unittest.main()
