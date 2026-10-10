"""Agents and the referee (ADR-31, AB-2a): fsim.Agent is a mission autonomy's view of its own aircraft, as itself -
and the objects behind it answer nothing else, the library refusing an agent's handle every call that is not its own."""
import math
import unittest

import fsim


def world(name):
    w = fsim.World(name, publish=False, workers=1, pin_workers=False, terrain=False)
    a = w.create_vehicle("a", "jsbsim:f16c", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=6000.0, airspeed_ms=230.0)
    b = w.create_vehicle("b", "jsbsim:f16c", latitude_deg=40.2, longitude_deg=0.0, altitude_msl_m=6000.0, airspeed_ms=230.0,
                         heading_deg=180.0)
    return w, a, b


class AgentTest(unittest.TestCase):
    def test_identity_and_commands(self):
        w, a, b = world("py-agent-identity")
        blue = w.create_agent(a, "blue-1")
        self.assertEqual((blue.vehicle_id, blue.agent_name), (a.id, "blue-1"))
        self.assertGreater(blue.controller, 0x40000000)
        self.assertEqual(blue.name, "a")
        ship = blue.ownship()
        self.assertIsInstance(ship, fsim.OwnShip)
        self.assertGreater(ship.altitude_msl_m, 5000.0)
        self.assertAlmostEqual(ship.latitude_rad, a.state.latitude_rad, places=6)
        self.assertTrue(0.0 <= ship.throttle <= 1.0)
        act = blue.submit_hsa(heading_rad=1.0, speed=230.0, altitude_m=6200.0)
        self.assertIsInstance(act, fsim.AgentActivity)
        self.assertEqual(act.controller, blue.controller)
        self.assertFalse(hasattr(act, "world") or hasattr(act, "vehicle"))
        w.step(5)
        self.assertEqual(w.activity(act.id).controller, blue.controller)
        with self.assertRaises(fsim.Error):
            blue.submit_hsa(heading_rad=1.0, source=fsim.Source.OVERRIDE)
        with self.assertRaises(fsim.Error):
            blue.submit_hsa(heading_rad=1.0, controller=7)
        act.cancel()
        self.assertEqual(w.referee().agents(), [(a.id, blue.controller, "blue-1")])

    def test_nothing_else_answers(self):
        w, a, b = world("py-agent-scope")
        blue = w.create_agent(a, "blue-1")
        for attr in ("world", "state", "sensed", "load_store", "set_allowed", "strike_consent", "command_attitude", "reset"):
            self.assertFalse(hasattr(blue, attr), attr)
        # even the objects behind it: the library refuses an agent's handle the range's calls
        with self.assertRaises(fsim.Error):
            blue._world.create_vehicle("c", "jsbsim:f16c")
        with self.assertRaises(fsim.Error):
            blue._world.step()
        with self.assertRaises(fsim.Error):
            fsim.world.Vehicle(blue._world, b.id).submit_hsa(heading_rad=2.0)
        theirs = b.submit_hsa(heading_rad=3.0)
        self.assertIsNone(blue.activity(theirs.id))

    def test_no_entity_targets(self):
        w, a, b = world("py-agent-entities")
        blue = w.create_agent(a, "blue-1")
        with self.assertRaises(fsim.Error):
            blue.submit_behavior("pursuit", target=b.id)

    def test_designation(self):
        w, a, b = world("py-agent-designate")
        blue = w.create_agent(a, "blue-1")
        self.assertEqual(blue.designate("radar", 12345), 0)  # (no such track)
        self.assertEqual(blue.designations(), [])

    def test_mission_data(self):
        w, a, b = world("py-agent-mission-data")
        blue, red = w.create_agent(a, "blue-1"), w.create_agent(b, "red-1")
        zone = fsim.OpZone(1, "ellipse", latitude_rad=0.7, longitude_rad=0.001, semi_major_m=1000.0, semi_minor_m=1000.0)
        w.set_op_zone(zone)
        blue.set_op_zone(zone._replace(id=2, semi_major_m=500.0, semi_minor_m=500.0))
        self.assertEqual([z.id for z in w.op_zones()], [1])
        self.assertEqual([z.id for z in blue.op_zones()], [1, 2])
        self.assertEqual([z.id for z in red.op_zones()], [1])
        self.assertIsNone(red.op_zone(2))
        self.assertEqual(blue.op_zone(2).semi_major_m, 500.0)

    def test_seal_and_audit(self):
        w, a, b = world("py-agent-seal")
        blue = w.create_agent(a, "blue-1")
        blue.submit_hsa(heading_rad=1.0)
        with self.assertRaises(fsim.Error):
            blue.submit_hsa(heading_rad=1.0, source=fsim.Source.OVERRIDE)
        r = w.seal()
        with self.assertRaises(fsim.Error):
            w.step()
        r.step(2)
        blue.ownship()  # (an agent's handle goes on)
        (vehicle, controller, name, calls), = r.referee().audit()
        counts = {call: (made, refused) for call, made, refused in calls}
        self.assertEqual((vehicle, name), (a.id, "blue-1"))
        self.assertEqual(counts["fsim_vehicle_submit_mode"], (2, 1))
        self.assertEqual(counts["fsim_vehicle_ownship"][0], 1)

    def test_referee(self):
        w, a, b = world("py-agent-referee")
        ref = w.referee()
        self.assertEqual(sorted(ref.vehicles()), sorted([a.id, b.id]))
        self.assertGreater(ref.state(b.id).altitude_msl_m, 5000.0)

    def test_links(self):
        """AB-4a: systems by SystemID, a C2 node and its agent, sends stamped by the DMS, each its own inbox."""
        w, a, b = world("py-agent-links")
        node = w.create_c2_node("c2", 39.8, 0.0, 100.0)
        net = w.create_net("l16", latency_s=0.05)
        self.assertEqual(w.assign_system(a, 0x3001, "AF-7"), 0x3001)
        for e in (a, b, node):
            self.assertTrue(w.join_net(e, net))
        blue, wing, boss = w.create_agent(a, "blue-1"), w.create_agent(b, "blue-2"), w.create_agent(node, "c2")
        self.assertEqual(blue.system, 0x3001)
        self.assertEqual(blue.nets(), [(net, "l16")])
        self.assertNotIn(wing.system, (0, b.id))
        with self.assertRaises(fsim.Error):
            blue.send(fsim.TOPIC_POSITION_REPORT, b"x")  # (a native type: the MA core's)
        self.assertTrue(boss.subscribe_all(False))
        self.assertTrue(boss.subscribe(1001))
        self.assertEqual(boss.subscriptions(), ([1001], False))
        published = blue.send(1001, b"tally")
        ignored = blue.send(1002, b"chatter")
        sent = wing.send(1003, b"to you", destinations=[blue.system])
        got = {"wing": [], "boss": [], "blue": []}
        for _ in range(15):
            w.step()
            for who, agent in (("wing", wing), ("boss", boss), ("blue", blue)):
                got[who] += agent.receive()
        self.assertEqual(sorted(m.topic for m in got["wing"]), [1001, 1002])
        self.assertEqual([m.topic for m in got["boss"]], [1001])
        self.assertEqual([(m.topic, m.source, m.data) for m in got["blue"]], [(1003, wing.system, b"to you")])
        self.assertTrue(all(m.source == 0x3001 for m in got["wing"]))
        self.assertEqual(blue.transmission(published).reached, 2)
        self.assertEqual(blue.transmission(ignored).reached, 1)
        t = wing.transmission(sent)
        self.assertEqual(t.destinations, [(0x3001, fsim.DELIVERY_DELIVERED)])
        ref = w.referee()
        self.assertEqual(ref.system_entity(0x3001), a.id)
        self.assertEqual(ref.entity_system(node), boss.system)
        self.assertEqual(ref.nets(), [net])
        self.assertEqual(ref.net_stats(net).delivered, 4)
        for call in ("create_net", "join_net", "net_stats"):
            self.assertFalse(hasattr(blue, call))

    def test_link_picture(self):
        """AB-4b: the lead's plan shares its position, heartbeat and radar tracks; the wingman, its radar off, designates the
        bandit's received track and pursues it."""
        w = fsim.World("py-agent-picture", publish=False, workers=1, pin_workers=False, terrain=False)
        lead_v = w.create_vehicle("lead", "jsbsim:f16c", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=6000.0,
                                  airspeed_ms=230.0, heading_deg=90.0)
        bandit = w.create_vehicle("bandit", "jsbsim:f16c", latitude_deg=40.027, longitude_deg=0.646, altitude_msl_m=7000.0,
                                  airspeed_ms=230.0, heading_deg=270.0)
        wing_v = w.create_vehicle("wing", "jsbsim:f16c", latitude_deg=39.91, longitude_deg=0.0, altitude_msl_m=6000.0,
                                  airspeed_ms=230.0, heading_deg=90.0)
        net = w.create_net("l16")
        w.assign_system(lead_v, 0x4001, "VIPER11")
        for v in (lead_v, wing_v):
            w.join_net(v, net)
        self.assertTrue(w.set_dissemination(lead_v, position_period_s=1.0, status_period_s=5.0, tracks_period_s=2.0))
        lead, wing = w.create_agent(lead_v, "blue-1"), w.create_agent(wing_v, "blue-2")
        lead.set_payload_state("radar", "operate")
        w.step()
        lead.amti_search("radar", elevation_centre_rad=math.radians(-1.0), bars=4)
        w.step(int(round(45.0 / w.step_seconds)))
        (peer,) = wing.peers()
        self.assertEqual((peer.system, peer.tail_number, peer.has_position), (0x4001, "VIPER11", True))
        ref = w.referee()
        tracks = [t for t in wing.link_tracks() if ref.link_track_truth(t.source, t.payload, t.track) == bandit.id]
        self.assertEqual(len(tracks), 1)
        t = tracks[0]
        target = wing.designate_link(t.source, t.payload, t.track)
        self.assertGreater(target, 0x80000000)
        self.assertEqual(wing.designation_source(target), 0x4001)
        self.assertEqual(ref.designation_truth(target), bandit.id)
        act = wing.submit_behavior("pursuit", target=target)
        self.assertTrue(act.live)
        self.assertGreater(len(lead.publish_tracks(destinations=[wing.system])), 0)  # (on demand too: each track its message)
        self.assertEqual(wing.publish_tracks(), [])  # (its radar is off: nothing to send)

    def test_c2_authority(self):
        """AB-5a: a QB's direct command flies as C2's, held against the MA's agent; an Observer is refused; designations."""
        w, a, b = world("py-agent-c2")
        crc = w.create_c2_node("crc", 39.7, 0.0, 200.0)
        watch = w.create_c2_node("watch", 39.6, 0.0, 200.0)
        net = w.create_net("l16")
        for e in (a, crc, watch):
            w.join_net(e, net)
        blue, boss, obs = w.create_agent(a, "blue-1"), w.create_agent(crc, "crc"), w.create_agent(watch, "watch")
        self.assertEqual([r.name for r in w.rbac(a)], ["Admin", "QB", "AVC", "LRE", "Observer"])
        boss.declare_role(fsim.ROLE_QB, "CRC LEAD")
        obs.declare_role(fsim.ROLE_OBSERVER, "WATCH")
        w.step(10)
        roles = {s: (rid, name) for s, rid, name in blue.authority().operators}
        self.assertEqual(roles[boss.system], (fsim.ROLE_QB, "QB"))
        self.assertEqual(roles[obs.system][1], "Observer")

        def status(c2, command, state=None):
            for _ in range(60):
                for s in c2.command_statuses():
                    if s.command_id == command and s.topic == fsim.TOPIC_FLIGHT_COMMAND and (state is None or s.state == state):
                        return s
                w.step()
            return None

        refused = status(obs, obs.command_hsa(blue.system, heading_rad=math.pi, speed=230.0, altitude_m=6500.0))
        self.assertEqual(refused.state, fsim.C2_UNAUTHORIZED)
        order = boss.command_hsa(blue.system, heading_rad=math.pi, speed=230.0, altitude_m=6500.0)
        accepted = status(boss, order)
        self.assertEqual(accepted.state, fsim.C2_ACCEPTED)
        self.assertEqual(blue.authority().commands, [(accepted.activity, boss.system, order)])
        with self.assertRaises(fsim.Rejected) as held:
            blue.submit_hsa(heading_rad=0.0, speed=230.0, altitude_m=6000.0)
        self.assertEqual(held.exception.reason, "authority_held")
        boss.cancel_command(blue.system, order)
        self.assertIsNotNone(status(boss, order, fsim.C2_CANCELED))
        blue.submit_hsa(heading_rad=0.0, speed=230.0, altitude_m=6000.0)  # (its own again)
        for call in ("set_rbac", "rbac"):
            self.assertFalse(hasattr(blue, call))

    def test_c2_tasks_overrides_consent(self):
        """AB-5b: a task the agent decides on; an altitude override on its HSA; a strike's release consent from the QB."""
        w, a, b = world("py-agent-c2-consent")
        a.load_loadout("mk82 x4")
        crc = w.create_c2_node("crc", 39.7, 0.0, 200.0)
        net = w.create_net("l16")
        for e in (a, crc):
            w.join_net(e, net)
        blue, boss = w.create_agent(a, "blue-1"), w.create_agent(crc, "crc")
        boss.declare_role(fsim.ROLE_QB, "CRC")
        w.step(10)
        tid = boss.send_task(blue.system, fsim.TASK_PATROL, latitude_rad=0.71, longitude_rad=0.0, altitude_m=7000.0, text="CAP")
        w.step(15)
        (task,) = blue.c2_tasks()
        self.assertEqual((task.source, task.task_id, task.state, task.text), (boss.system, tid, fsim.TASK_RECEIVED, "CAP"))
        self.assertTrue(blue.accept_task(boss.system, tid))
        w.step(15)
        self.assertIn((tid, fsim.C2_ACCEPTED), [(s.command_id, s.state) for s in boss.command_statuses() if s.topic == fsim.TOPIC_TASK])

        boss.set_override(blue.system, fsim.OVERRIDE_ALTITUDE, 7200.0)
        w.step(15)
        (o,) = blue.overrides()
        self.assertEqual((o.kind, o.value), (fsim.OVERRIDE_ALTITUDE, 7200.0))
        blue.submit_hsa(heading_rad=0.0, speed=230.0, altitude_m=6000.0)  # (flies 7,200 m: the override)

        blue.set_payload_state("stations", "operate")
        w.step()
        strike = blue.strike("mk82", 1)
        w.step(90)
        self.assertEqual(blue.strike_status(strike).phase, "awaiting_consent")
        (req,) = boss.consent_requests()
        self.assertEqual((req.source, req.strike, req.store, req.quantity), (blue.system, strike, "mk82", 1))
        self.assertTrue(boss.answer_consent(blue.system, req.request_id, True))
        w.step(90)
        self.assertNotEqual(blue.strike_status(strike).phase, "awaiting_consent")
        self.assertTrue(w.referee().weapons())

    def test_c2_lost_comms_and_team(self):
        """AB-5c: C2's lost-communication contingency raised when its heartbeats stop; a team's leader commands its member."""
        w, a, b = world("py-agent-c2-team")
        crc = w.create_c2_node("crc", 39.7, 0.0, 200.0)
        net = w.create_net("l16")
        for e in (a, b, crc):
            w.join_net(e, net)
        w.set_dissemination(crc, status_period_s=2.0)
        lead, wing, boss = w.create_agent(a, "blue-1"), w.create_agent(b, "blue-2"), w.create_agent(crc, "crc")
        boss.declare_role(fsim.ROLE_AVC, "AVC")
        w.step(10)
        boss.set_lost_comms(wing.system, 10.0)
        boss.form_package(3, lead.system, [lead.system, wing.system])
        w.step(15)
        c = wing.comms()
        self.assertEqual((c.lost_comms_set, c.threshold_s, c.lost, c.package_id, c.leader), (True, 10.0, False, 3, lead.system))
        order = lead.command_hsa(wing.system, heading_rad=math.pi, speed=230.0, altitude_m=6000.0)
        w.step(30)  # (its 640 bytes on a third of the net's rate, and the answer back)
        self.assertIn((order, fsim.C2_ACCEPTED), [(s.command_id, s.state) for s in lead.command_statuses()])
        self.assertEqual([cmd[1] for cmd in wing.authority().commands], [lead.system])
        w.leave_net(crc, net)
        w.step(int(round(12.0 / w.step_seconds)))
        c = wing.comms()
        self.assertTrue(c.lost)
        self.assertEqual(c.alerts, 1)


if __name__ == "__main__":
    unittest.main()
