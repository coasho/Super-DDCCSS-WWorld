"""The composition example (examples/training/ma_composition; docs/sdk/ma-composition.md; M6 noncombat) on the SDK under
test: one snapshot shared by every component of a tick, the service's messages ingested once, conflicts resolved the
same way every time, only selected requests submitted, activity updates and cancels by their originator alone, SDK
refusals back to their originator, components at their own periods, and a new generation that keeps nothing."""
import json
import os
import sys
import unittest

import fsim
from fsim import mission

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
sys.path.insert(0, os.path.join(ROOT, "examples", "training"))
from ma_composition import components, contracts, scenario  # noqa: E402
from ma_composition.contracts import Proposal  # noqa: E402
from ma_composition.coordinator import Coordinator  # noqa: E402

TICKS = 10


class CountingEndpoint:
    """The lead's endpoint, counting what the system calls on it."""

    def __init__(self, endpoint):
        self.endpoint, self.reads, self.submits = endpoint, 0, 0

    def observation_json(self):
        self.reads += 1
        return self.endpoint.observation_json()

    def submit_json(self, text):
        self.submits += 1
        return self.endpoint.submit_json(text)


def run(local=7, ticks=TICKS, observe=None):
    """One generation in a fresh episode, as scenario.run_episode, with `observe(system, endpoint)` after each tick."""
    with mission.MissionEpisode(scenario.config_text()) as episode:
        system = scenario.make_system()
        seen = {}
        for c in system.components:  # (each component records the snapshot object it was handed)
            def propose(snapshot, c=c, original=c.propose):
                seen.setdefault(int(snapshot["tick"]), []).append((c.component_id, snapshot))
                return original(snapshot)
            c.propose = propose
        episode.reset(local)
        lead = CountingEndpoint(episode.endpoint(scenario.LEAD_FLIGHT))
        c2 = scenario.C2Script(episode.endpoint(scenario.CRC))
        key = json.loads(lead.endpoint.observation_json())["episode"]
        system.begin_episode(contracts.context(scenario.BASE_SEED, key, scenario.LEAD_FLIGHT, [c.component_id for c in system.components]))
        snapshots = []
        for _ in range(ticks):
            c2.tick()
            snapshots.append(system.tick(lead))
            if observe:
                observe(system, lead)
            episode.step()
        final = json.loads(lead.endpoint.observation_json())
        return {"system": system, "seen": seen, "lead": lead, "snapshots": snapshots, "trace": system.trace(), "final": final}


class CompositionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.a = run()
        cls.b = run()

    def test_seed_streams_are_the_contracts(self):
        with open(os.path.join(ROOT, "tests", "training", "fixtures", "mission_training", "contract", "manifest.json"), "rb") as f:
            vectors = json.loads(f.read().decode("utf-8"))["seed_streams"]
        self.assertEqual(contracts._next(0), 16294208416658607535)
        for v in vectors:
            b, e, l, s, c = (int(v[k]) for k in ("base_seed", "env_id", "local_episode_index", "service_id", "component_id"))
            self.assertEqual(contracts.service_seed(b, e, l, s), int(v["service_seed"]), v)
            self.assertEqual(contracts.component_seed(b, e, l, s, c), int(v["component_seed"]), v)

    def test_one_snapshot_object_per_tick_read_once_and_read_only(self):
        a = self.a
        self.assertEqual(a["lead"].reads, TICKS)    # (one read per tick)
        self.assertEqual(a["lead"].submits, TICKS)  # (one submission per tick)
        for tick, handed in a["seen"].items():
            frozen = a["snapshots"][tick]
            self.assertTrue(all(s is frozen for _, s in handed), "tick %d: components were handed different snapshots" % tick)
        with self.assertRaises(TypeError):
            frozen["tick"] = "99"
        with self.assertRaises((TypeError, AttributeError)):  # (arrays are tuples)
            frozen["reports"]["tasks"]["items"].append({})

    def test_messages_are_ingested_once(self):
        a = self.a
        delivered = [(m["source_system_id"], m["message_id"]) for s in a["snapshots"] for m in s["messages"]["items"]]
        self.assertGreater(len(delivered), 0)
        self.assertEqual(len(delivered), len(set(delivered)))  # (each message in exactly one snapshot)
        self.assertEqual(a["system"].blackboard["duplicates"], 0)
        self.assertEqual(len(a["system"].blackboard["messages"]), len(delivered))

    def test_conflicts_resolve_by_owner_priority_then_component_id(self):
        flight = ("flight",)
        p = lambda kind, priority, resources=flight, operation="new", parameters=None: Proposal(kind, operation, parameters or {}, resources, priority)
        c = Coordinator(owners={"dms:1001": 2})
        decided = c.arbitrate([(1, 0, p("flight", 1)), (4, 0, p("flight", 2)), (3, 0, p("flight", 2)),
                               (1, 1, p("dms", 9, ("dms:1001",))), (2, 0, p("dms", 1, ("dms:1001",)))], {}, {})
        outcome = {(d[0], d[1]): d[4] for d in decided}
        self.assertEqual(outcome[(3, 0)], "selected")      # (equal priority: the lower component id)
        self.assertEqual(outcome[(4, 0)], "taken_by:3")
        self.assertEqual(outcome[(1, 0)], "taken_by:3")
        self.assertEqual(outcome[(2, 0)], "selected")      # (the configured owner beats a higher priority)
        self.assertEqual(outcome[(1, 1)], "owned_by:2")
        held = c.arbitrate([(1, 0, p("flight", 9))], {"flight": 4}, {})
        self.assertEqual(held[0][4], "held_by:4")
        addressing = c.arbitrate([(2, 0, p("flight", 9, operation="cancel", parameters={"activity_id": "7"})),
                                  (4, 0, p("flight", 1, operation="update", parameters={"activity_id": "7"}))], {"flight": 4}, {"7": 4})
        self.assertEqual({d[0]: d[4] for d in addressing}, {2: "not_activity_owner:4", 4: "selected"})

    def test_the_same_inputs_give_the_same_trace(self):
        self.assertEqual(self.a["trace"], self.b["trace"])

    def test_only_selected_requests_are_submitted_and_answered(self):
        trace = self.a["trace"]["records"]
        selected = {str(d[4]) for d in trace["decisions"] if d[4] is not None}
        lost = [d for d in trace["decisions"] if d[4] is None]
        self.assertTrue(lost, "the run must contain conflicts")
        submitted = {r[0] for s in trace["submissions"] for r in s[3]}
        self.assertEqual(submitted, selected)
        answered = {r["request_id"] for r in self.a["final"]["receipts"]}
        self.assertEqual(answered, selected)  # (the SDK received nothing the coordinator did not select)
        for s in trace["submissions"]:
            self.assertTrue(s[1])
            self.assertTrue(all(status == "queued" for _, status, _ in s[3]))

    def test_activities_are_updated_and_canceled_by_their_originator_alone(self):
        trace = self.a["trace"]["records"]
        reasons = {(d[0], d[1], d[2]): d[3] for d in trace["decisions"]}
        self.assertEqual(reasons[(6, 2, 1)], "not_activity_owner:%d" % components.RoutePlanner.component_id)
        planner = components.RoutePlanner.component_id
        updates = [r for r in trace["receipts"] if r[2] == planner and r[3] == "accepted"]
        self.assertGreaterEqual(len(updates), 2)  # (its own route, re-planned twice)
        route = self.a["system"].components[3].activity
        live = {a["activity_id"]: a["state"] for a in self.a["final"]["reports"]["activities"]["items"]}
        self.assertEqual(live.get(route), "active")  # (the cancel another component proposed never reached the SDK)

    def test_refusals_reach_their_originator_and_are_not_retried(self):
        trace = self.a["trace"]["records"]
        refused = [r for r in trace["receipts"] if r[6] or r[3] == "rejected"]
        self.assertEqual([(r[2], r[6]) for r in refused], [(components.CoordinationPolicy.component_id, "capability_not_bound")])
        state = self.a["trace"]["state"]["components"]
        self.assertEqual(state["2"]["radar"], "capability_not_bound")
        self.assertEqual([s["refusals"] for k, s in state.items() if k != "2"], [[], [], []])
        refused_tick = refused[0][0]
        later = [p for p in trace["proposals"] if p[1] == 2 and p[3] == "sensor" and p[0] >= refused_tick]
        self.assertEqual(later, [])  # (no retry around the SDK's authority)

    def test_components_run_at_their_own_periods(self):
        ticks = {}
        for p in self.a["trace"]["records"]["proposals"]:
            ticks.setdefault(p[1], set()).add(p[0])
        for c in self.a["system"].components:
            self.assertTrue(all(t % c.period_ticks == 0 for t in ticks.get(c.component_id, ())), c.component_id)
        self.assertEqual(ticks[components.RoutePlanner.component_id], {0, 4, 8})

    def test_a_c2_task_is_accepted_and_reported_through_the_bt(self):
        tasks = self.a["final"]["reports"]["tasks"]["items"]
        self.assertEqual([t["state"] for t in tasks], ["completed"])
        tree = self.a["trace"]["state"]["components"]["3"]["tree"]
        self.assertEqual(tree["runs"]["propose-accept"], 1)
        self.assertEqual(tree["runs"]["propose-report"], 2)

    def test_a_new_generation_keeps_nothing(self):
        with mission.MissionEpisode(scenario.config_text()) as episode:
            system = scenario.make_system()
            first = scenario.run_episode(episode, system, 7, TICKS)
            pristine = scenario.make_system()
            stale = episode.endpoint(scenario.LEAD_FLIGHT)
            old_context = system.context
            episode.reset(7)
            current = episode.endpoint(scenario.LEAD_FLIGHT)
            key = json.loads(current.observation_json())["episode"]
            system.begin_episode(old_context)
            with self.assertRaises(RuntimeError):  # (the new generation's snapshot before its begin_episode: nothing sent)
                system.tick(current)
            self.assertEqual(json.loads(current.receipts_json())["submission"], None)
            context = contracts.context(scenario.BASE_SEED, key, scenario.LEAD_FLIGHT, [c.component_id for c in system.components])
            system.begin_episode(context)
            pristine.begin_episode(context)
            self.assertEqual(system.state(), pristine.state())  # (blackboard, BT, planner, memory, RNG, queues, mappings)
            self.assertEqual(system.trace()["records"], pristine.trace()["records"])
            with self.assertRaises(fsim.Error):  # (an earlier generation's endpoint: nothing it computes is sent)
                system.tick(stale)
        self.assertEqual(first["records"], self.a["trace"]["records"])  # (the reset run as a fresh episode's)
        self.assertEqual(first["state"], self.a["trace"]["state"])

    def test_the_system_is_told_its_permitted_feedback_alone(self):
        system = scenario.make_system()
        with self.assertRaises(ValueError):
            system.end_episode({"episode_done": True, "reason": "truncated", "truth": {}})
        system.end_episode({"episode_done": False, "reason": None})
        self.assertEqual(system.outcome, {"episode_done": False, "reason": None})


if __name__ == "__main__":
    unittest.main()
