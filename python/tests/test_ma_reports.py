"""Legal reports with their provenance (docs/sdk/mission-training.md, section 10) through the Python SDK: what an agent's
own reads give, each with where it came from, when, and how old it is - an unknown None, a stale one marked, an
undelivered one absent, and nothing the referee alone knows in any of it."""
import math
import unittest

import fsim

EAST_KM_DEG = math.degrees(1e3 / (6371000.0 * math.cos(math.radians(40.0))))
NORTH_KM_DEG = 1.0 / 111.2


def make_world(name):
    return fsim.World(name, publish=False, workers=1, pin_workers=False, seed=3, terrain=False)


def aircraft(w, name, north_km, east_km, altitude_m, heading_deg, airspeed_ms=230.0):
    return w.create_vehicle(name, "jsbsim:f16c", latitude_deg=40.0 + north_km * NORTH_KM_DEG, longitude_deg=east_km * EAST_KM_DEG,
                            altitude_msl_m=altitude_m, heading_deg=heading_deg, airspeed_ms=airspeed_ms)


def steps(w, seconds):
    return int(round(seconds / w.step_seconds))


class TwoShip:
    """A lead searching with its radar and sharing its tracks, a wingman with its radar off, a bandit ahead of them."""

    def __init__(self, name):
        self.w = w = make_world(name)
        self.lead = aircraft(w, "lead", 0.0, 0.0, 6000.0, 90.0)
        self.bandit = aircraft(w, "bandit", 3.0, 55.0, 7000.0, 270.0)
        self.wing = aircraft(w, "wing", -10.0, 0.0, 6000.0, 90.0)
        self.lead_ma, self.wing_ma = w.create_agent(self.lead, "blue-1"), w.create_agent(self.wing, "blue-2")
        self.net = w.create_net("l16")
        assert w.assign_system(self.lead, 0x4001, "VIPER11") == 0x4001
        assert w.join_net(self.lead, self.net) and w.join_net(self.wing, self.net)
        assert w.set_dissemination(self.lead, position_period_s=1.0, tracks_period_s=2.0)
        self.lead_ma.set_payload_state("radar", "operate")
        w.step()
        self.lead_ma.amti_search("radar", elevation_centre_rad=math.radians(-1.0), bars=4)
        w.step(steps(w, 45.0))


class MAReportsTest(unittest.TestCase):
    def test_own_ship_and_own_tracks(self):
        s = TwoShip("py-ma-reports-own")
        now = s.w.time
        nav = s.lead_ma.report_navigation()
        self.assertIsInstance(nav, fsim.Report)
        self.assertEqual(nav.availability, "available")
        self.assertIsInstance(nav.meta, fsim.ReportMeta)
        own, endurance = nav.value
        self.assertEqual((nav.meta.source, nav.meta.validity), ("own_navigation", "valid"))
        self.assertEqual((nav.meta.system, nav.meta.net, nav.meta.payload), (None, None, None))
        self.assertEqual(own, s.lead_ma.ownship())
        self.assertEqual(endurance.fuel_kg, s.lead_ma.navigation_report().fuel_kg)
        self.assertEqual(nav.meta.sample_time_s, own.time_s)
        self.assertEqual(nav.meta.receive_time_s, now)
        self.assertEqual(nav.meta.age_s, now - own.time_s)

        radar = s.lead.payload("radar").index
        tracks = s.lead_ma.amti_tracks("radar")
        reported = s.lead_ma.report_tracks()
        self.assertIsInstance(reported, fsim.ReportCollection)
        self.assertEqual(reported.availability, "available")
        self.assertTrue(tracks)
        self.assertEqual([i.value for i in reported.items], tracks)
        for item in reported.items:
            self.assertIsInstance(item, fsim.ReportItem)
            self.assertEqual(item.value.truth, 0)  # (the entity behind it is the referee's)
            self.assertEqual((item.meta.source, item.meta.validity, item.meta.payload), ("own_sensor", "valid", radar))
            self.assertEqual(item.meta.sample_time_s, item.value.last_update_s)
            self.assertEqual(item.meta.receive_time_s, now)
            self.assertEqual(item.meta.age_s, now - item.value.last_update_s)
        self.assertIn(s.bandit.id, [t.truth for t in s.lead.amti_tracks("radar")])  # (the range's read does carry it)
        self.assertEqual(s.lead_ma.report_tracks(), reported)  # (reading takes nothing)
        # stale is the caller's threshold on a held report's age, and only that
        self.assertTrue(all(i.meta.validity == "valid" for i in s.lead_ma.report_tracks(stale_after_s=1e9).items))
        self.assertTrue(all(i.meta.validity == "stale" for i in s.lead_ma.report_tracks(stale_after_s=-1.0).items))
        self.assertEqual(s.lead_ma.report_navigation(stale_after_s=-1.0).meta.validity, "stale")
        # the wingman's radar is off: an available, empty collection - not the lead's, not truth
        self.assertEqual(s.wing_ma.report_tracks(), fsim.ReportCollection("available", []))

    def test_invalid_navigation_stays_as_reported(self):
        w = make_world("py-ma-reports-invalid")
        a = aircraft(w, "viper", 0.0, 0.0, 6000.0, 90.0)
        blue = w.create_agent(a, "blue-1")
        w.step(10)
        self.assertEqual(blue.report_navigation().meta.validity, "valid")
        a.add_effect("gnss_degradation", loss_probability_per_s=1e9, outage_s=1e6)
        w.step(steps(w, 2.0))
        nav = blue.report_navigation()
        self.assertEqual(nav.availability, "available")
        self.assertFalse(nav.value[0].gnss_valid)
        self.assertEqual(nav.meta.validity, "invalid")
        self.assertEqual(nav.value[0], blue.ownship())  # (as its platform reports it: not truth, not blanked)
        self.assertEqual(nav.meta.sample_time_s, nav.value[0].time_s)

    def test_a_link_track_is_there_once_delivered_and_as_its_source_said_it(self):
        s = TwoShip("py-ma-reports-link")
        w = s.w

        def newest():
            return max((t.received_s for t in s.wing_ma.link_tracks()), default=-1.0)

        before = newest()
        self.assertGreater(before, 0.0)
        for _ in range(400):  # (to just after a delivery: the next is the tracks period away)
            if newest() != before:
                break
            w.step()
        self.assertGreater(newest(), before)
        t1 = w.time
        held = s.wing_ma.link_tracks()
        first = s.wing_ma.report_link_tracks()
        self.assertEqual(first.availability, "available")
        self.assertTrue(held)
        self.assertEqual([i.value for i in first.items], held)
        for item in first.items:
            m, v = item
            self.assertEqual((m.source, m.validity, m.system, m.net, m.payload), ("link", "valid", 0x4001, s.net, v.payload))
            self.assertEqual((m.sample_time_s, m.receive_time_s), (v.time_s, v.received_s))
            self.assertEqual(m.age_s, t1 - v.time_s)
            self.assertGreaterEqual(m.receive_time_s, m.sample_time_s)
            self.assertFalse(hasattr(v, "truth"))  # (a link track carries no entity at all)
        # truth moves on; nothing is delivered meanwhile: what the wingman holds does not move - only its age
        truth_then = s.bandit.state.longitude_rad
        w.step(steps(w, 1.0))
        t2 = w.time
        self.assertNotEqual(s.bandit.state.longitude_rad, truth_then)
        second = s.wing_ma.report_link_tracks()
        self.assertEqual([i.value for i in second.items], [i.value for i in first.items])
        for a, b in zip(first.items, second.items):
            self.assertEqual((b.meta.sample_time_s, b.meta.receive_time_s), (a.meta.sample_time_s, a.meta.receive_time_s))
            self.assertAlmostEqual(b.meta.age_s, a.meta.age_s + (t2 - t1), places=9)
        # the range makes, moves and removes an aircraft no sensor holds: no actor report changes
        lead_before = s.lead_ma.report_tracks()
        ghost = aircraft(w, "ghost", -600.0, 0.0, 9000.0, 0.0)
        ghost.reset(latitude_deg=30.0, longitude_deg=10.0, altitude_msl_m=3000.0, airspeed_ms=200.0)
        ghost.remove()
        self.assertEqual(s.lead_ma.report_tracks(), lead_before)
        self.assertEqual(s.wing_ma.report_link_tracks(), second)
        # the next delivery: now it changes, and says when it was sampled and when it arrived
        number, said = held[0].track, held[0].time_s
        updated = None
        for _ in range(400):
            updated = next((t for t in s.wing_ma.link_tracks() if t.track == number), None)
            if updated is None or updated.time_s != said:
                break
            w.step()
        self.assertIsNotNone(updated)
        self.assertGreater(updated.time_s, said)
        self.assertGreater(updated.received_s, t1)
        self.assertTrue(all(i.meta.validity == "stale" for i in s.wing_ma.report_link_tracks(stale_after_s=0.0).items))
        self.assertTrue(all(i.meta.validity == "valid" for i in s.wing_ma.report_link_tracks().items))
        # peers: the lead, by its position reports
        [peer] = s.wing_ma.report_peers().items
        self.assertTrue(peer.value.has_position)
        self.assertEqual((peer.meta.source, peer.meta.system, peer.meta.net), ("link", 0x4001, s.net))
        self.assertEqual((peer.meta.sample_time_s, peer.meta.receive_time_s), (peer.value.time_s, peer.value.received_s))
        self.assertEqual(s.lead_ma.report_peers(), fsim.ReportCollection("available", []))  # (the wingman publishes nothing)
        # the link lost: after its picture's timeout the track is gone - dropped, not kept and marked
        self.assertTrue(w.leave_net(s.lead, s.net))
        w.step(steps(w, 13.0))
        self.assertEqual(s.wing_ma.report_link_tracks(), fsim.ReportCollection("available", []))

    def test_tasks_authority_and_what_a_c2_node_has(self):
        w = make_world("py-ma-reports-c2")
        a = aircraft(w, "viper", 0.0, 0.0, 6000.0, 0.0)
        crc = w.create_c2_node("crc", 40.0 - 40.0 * NORTH_KM_DEG, 0.0, 200.0)
        net = w.create_net("l16")
        self.assertTrue(w.join_net(a, net) and w.join_net(crc, net))
        self.assertTrue(w.set_dissemination(crc, status_period_s=2.0))
        blue, boss = w.create_agent(a, "blue-1"), w.create_agent(crc, "crc")
        boss.declare_role(2, "CRC LEAD")
        w.step(steps(w, 3.0))

        authority = blue.report_authority()
        self.assertEqual(authority.availability, "available")
        self.assertEqual((authority.meta.source, authority.meta.validity), ("ma_core", "valid"))
        self.assertEqual(authority.meta.sample_time_s, w.time)
        self.assertEqual(authority.value, blue.authority())
        self.assertEqual([o[0] for o in authority.value.operators], [boss.system])
        self.assertEqual(blue.operator_callsigns(), ["CRC LEAD"])

        # a task is not there until the link delivers it
        sent = w.time
        task = boss.send_task(blue.system, fsim.TASK_PATROL, latitude_rad=math.radians(40.3), longitude_rad=0.0)
        self.assertGreater(task, 0)
        self.assertEqual(blue.report_tasks(), fsim.ReportCollection("available", []))
        w.step(steps(w, 2.0))
        [held] = blue.report_tasks().items
        self.assertEqual(repr(held.value), repr(blue.c2_tasks()[0]))  # (its unknown fields NaN: compared as written)
        self.assertEqual((held.meta.source, held.meta.system, held.meta.validity), ("link", boss.system, "valid"))
        self.assertEqual(held.meta.receive_time_s, held.value.received_s)
        self.assertGreater(held.meta.receive_time_s, sent)
        self.assertIsNone(held.meta.sample_time_s)  # (an order states no time of its own)
        self.assertIsNone(held.meta.age_s)
        self.assertEqual(blue.report_tasks(stale_after_s=0.0).items[0].meta.validity, "valid")  # (no age: never stale)
        self.assertEqual((held.value.task_id, held.value.state), (task, fsim.TASK_RECEIVED))
        self.assertTrue(math.isnan(held.value.altitude_m))  # (unknown stays unknown: not zero)
        self.assertEqual(len(blue.c2_tasks()), 1)  # (reading took nothing)

        # a peer heard by its heartbeat alone: held, its sample time unknown
        [peer] = blue.report_peers().items
        self.assertFalse(peer.value.has_position)
        self.assertIsNone(peer.meta.sample_time_s)
        self.assertIsNone(peer.meta.age_s)
        self.assertEqual(peer.meta.receive_time_s, peer.value.received_s)

        # the C2 node: no navigation, no sensors, no tasks, no authority over it - not applicable, none of it a zero
        self.assertEqual(boss.report_navigation(), fsim.Report("not_applicable", None, None))
        self.assertEqual(boss.report_authority(), fsim.Report("not_applicable", None, None))
        for report in (boss.report_tracks, boss.report_contacts, boss.report_tasks):
            self.assertEqual(report(), fsim.ReportCollection("not_applicable", []))
        self.assertEqual(boss.report_link_tracks().availability, "available")
        self.assertEqual(boss.report_peers().availability, "available")
        self.assertEqual(boss.report_peer_heartbeats().availability, "available")

        # a platform gone: unavailable, and nothing listed
        a.remove()
        self.assertEqual(blue.report_navigation(), fsim.Report("unavailable", None, None))
        self.assertEqual(blue.report_authority(), fsim.Report("unavailable", None, None))
        for report in (blue.report_tracks, blue.report_contacts, blue.report_link_tracks, blue.report_peers, blue.report_peer_heartbeats,
                       blue.report_tasks):
            self.assertEqual(report(), fsim.ReportCollection("unavailable", []))

    def test_a_passive_contact_as_its_receiver_measured_it(self):
        w = make_world("py-ma-reports-contacts")
        own = aircraft(w, "viper", 0.0, 0.0, 5000.0, 90.0, 200.0)
        other = aircraft(w, "bandit", 0.0, 40.0, 5000.0, 270.0, 200.0)
        blue = w.create_agent(own, "blue-1")
        rwr = own.payload("rwr").index
        other.set_payload_state("radar", "operate")
        w.step()
        other.amti_search("radar", azimuth_half_width_rad=math.radians(30.0))
        w.step(60)
        self.assertEqual(blue.contacts("rwr"), [])  # (its receiver off)
        self.assertEqual(blue.report_contacts(), fsim.ReportCollection("available", []))
        blue.set_payload_state("rwr", "operate")
        w.step(30)
        [mine] = blue.contacts("rwr")
        self.assertEqual(mine.truth, 0)  # (the agent's read: the entity behind it is the referee's)
        self.assertEqual(own.contacts("rwr")[0].truth, other.id)  # (the range's read of the same receiver has it)
        self.assertIs(mine.kind, fsim.ContactKind.EMITTER)
        self.assertTrue(math.isnan(mine.range_m))  # (a receiver measures none: unknown, not zero)
        [item] = blue.report_contacts().items
        self.assertEqual(item.value.id, mine.id)
        self.assertEqual(item.value.truth, 0)
        self.assertTrue(math.isnan(item.value.range_m))
        self.assertEqual((item.meta.source, item.meta.validity, item.meta.payload), ("own_sensor", "valid", rwr))
        self.assertEqual(item.meta.sample_time_s, mine.time_s)
        self.assertEqual(item.meta.receive_time_s, w.time)
        self.assertEqual(item.meta.age_s, w.time - mine.time_s)
        # another platform's agent reads none of it, by any report
        red = w.create_agent(other, "red-1")
        with self.assertRaises(fsim.Error):
            red._world._h.agent_report(own.id, 2, math.nan)
        with self.assertRaises(fsim.Error):
            red._world._h.agent_report(own.id, 0, math.nan)

    def test_a_peers_position_keeps_the_delivery_it_came_by(self):
        w = make_world("py-ma-reports-peer-provenance")
        sender, receiver = aircraft(w, "sender", 0.0, 0.0, 6000.0, 0.0), aircraft(w, "receiver", 2.0, 0.0, 6000.0, 0.0)
        tx, rx = w.create_agent(sender, "sender"), w.create_agent(receiver, "receiver")
        net_a = w.create_net("position-net", latency_s=0.05, jitter_s=0.0)
        net_b = w.create_net("heartbeat-net", latency_s=0.05, jitter_s=0.0)
        self.assertTrue(w.join_net(receiver, net_a) and w.join_net(receiver, net_b) and w.join_net(sender, net_a))
        self.assertTrue(tx.publish_position([rx.system]))
        w.step(30)
        [before] = rx.report_peers().items
        self.assertTrue(before.value.has_position)
        self.assertEqual((before.meta.source, before.meta.system, before.meta.net), ("link", tx.system, net_a))
        self.assertEqual((before.meta.sample_time_s, before.meta.receive_time_s), (before.value.time_s, before.value.received_s))
        [silent] = rx.report_peer_heartbeats().items  # (no heartbeat heard: of that, nothing is held)
        self.assertEqual(silent.meta, fsim.ReportMeta("link", "unknown", tx.system, None, None, None, None, None))
        self.assertEqual(silent.value, before.value)
        # its heartbeat comes over the other net: the position it holds is the same report, delivered as it was
        self.assertTrue(w.leave_net(sender, net_a) and w.join_net(sender, net_b))
        self.assertTrue(tx.publish_status([rx.system]))
        w.step(30)
        [after] = rx.report_peers().items
        self.assertEqual(after.value[4:], before.value[4:])  # (the position: unchanged)
        self.assertEqual(after.value.net, net_b)  # (the raw peer: last heard by, as it always was)
        self.assertGreater(after.value.received_s, before.value.received_s)
        self.assertEqual(rx.peers()[0], after.value)
        self.assertEqual((after.meta.net, after.meta.system), (net_a, tx.system))
        self.assertEqual((after.meta.sample_time_s, after.meta.receive_time_s), (before.meta.sample_time_s, before.meta.receive_time_s))
        self.assertEqual(after.meta.age_s, w.time - before.meta.sample_time_s)
        [beat] = rx.report_peer_heartbeats().items  # (the heartbeat's own: the other net, its delivery, the time it states)
        self.assertEqual((beat.meta.source, beat.meta.validity, beat.meta.system, beat.meta.net), ("link", "valid", tx.system, net_b))
        self.assertEqual(beat.meta.receive_time_s, after.value.received_s)
        self.assertGreater(beat.meta.sample_time_s, after.meta.sample_time_s)
        self.assertLess(beat.meta.sample_time_s, beat.meta.receive_time_s)
        self.assertEqual(beat.meta.age_s, w.time - beat.meta.sample_time_s)
        between = (after.meta.age_s + beat.meta.age_s) / 2.0  # (each stale by its own age: the position is the older)
        self.assertEqual(rx.report_peers(stale_after_s=between).items[0].meta.validity, "stale")
        self.assertEqual(rx.report_peer_heartbeats(stale_after_s=between).items[0].meta.validity, "valid")
        for _ in range(3):  # (read again: the same, and nothing taken)
            self.assertEqual(rx.report_peers().items, [after])
            self.assertEqual(rx.report_peer_heartbeats().items, [beat])
        # a new position over the net it is on now: the position's provenance moves with it, the heartbeat's stays
        self.assertTrue(tx.publish_position([rx.system]))
        w.step(30)
        [moved] = rx.report_peers().items
        self.assertEqual(moved.meta.net, net_b)
        self.assertEqual((moved.meta.sample_time_s, moved.meta.receive_time_s), (moved.value.time_s, moved.value.received_s))
        self.assertGreater(moved.meta.receive_time_s, beat.meta.receive_time_s)
        still = rx.report_peer_heartbeats().items[0].meta
        self.assertEqual((still.net, still.sample_time_s, still.receive_time_s), (beat.meta.net, beat.meta.sample_time_s, beat.meta.receive_time_s))


if __name__ == "__main__":
    unittest.main()
