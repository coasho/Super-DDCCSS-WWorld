"""Batches and the training adapter through the Python SDK (docs/sdk/mission-training.md, section 18; M7 core): slots of
different topology, autoreset by mode, final documents by principal, the training adapter's lifecycle hooks around
every generation, and three streams kept apart - what actors see, the lifecycle signals a learner needs, and the
referee's documents."""
import json
import os
import sys
import threading
import time
import unittest

import fsim
from fsim import mission, training

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
CONTRACT = os.path.join(ROOT, "tests", "fixtures", "mission_training", "contract")
sys.path.insert(0, os.path.join(ROOT, "tools", "ma_contract"))
sys.path.insert(0, os.path.join(ROOT, "examples", "python"))
import check  # noqa: E402

WING, CRC, LEAD = 9007199254740995, 4, 1


def config(env_id, max_ticks=600):
    with open(os.path.join(CONTRACT, "config.two-ship-c2.json"), "rb") as f:
        document = json.loads(f.read().decode("utf-8"))
    document["env_id"] = str(env_id)
    document["task"]["criteria"][0]["max_ticks"] = max_ticks
    return json.dumps(document)


def wing_and_c2(env_id):
    document = json.loads(config(env_id))
    document["scenario"]["vehicles"] = document["scenario"]["vehicles"][1:]
    document["systems"] = [s for s in document["systems"] if s["platform"] != "lead"]
    document["nets"][0]["members"] = ["wing", "crc"]
    document["dissemination"] = [d for d in document["dissemination"] if d["platform"] != "lead"]
    document["services"] = [s for s in document["services"] if s["platform"] != "lead"]
    document["task"]["criteria"] = [document["task"]["criteria"][0]]
    return json.dumps(document)


class Waiting:
    """A stand-in MA system: waits every tick, and counts its lifecycle."""

    def __init__(self, slot, service_id, log):
        self.slot, self.service_id, self.log = slot, service_id, log
        self.ticks = 0

    def begin_episode(self, context):
        self.ticks = 0
        self.log.append(("begin", self.slot, self.service_id, context["episode_key"]["episode_generation"]))

    def tick(self, endpoint):
        snapshot = json.loads(endpoint.observation_json())
        self.ticks += 1
        endpoint.submit_json(json.dumps({"schema": "ma.actions.v1", "episode": snapshot["episode"], "service_id": snapshot["service"]["service_id"],
                                         "tick": snapshot["tick"], "requests": []}))

    def end_episode(self, feedback):
        self.log.append(("end", self.slot, self.service_id, feedback["reason"]))


class MissionBatchTest(unittest.TestCase):
    def test_slots_status_and_errors(self):
        with mission.MissionBatch([config(201), wing_and_c2(202)], autoreset="next_step", workers=2, first_local_episode_index=5) as batch:
            self.assertEqual(len(batch), 2)
            with self.assertRaises(fsim.Error):
                batch.step()  # (not reset)
            batch.reset()
            status = batch.status(0)
            self.assertEqual((status.generation, status.local_episode_index, status.tick, status.reset), (1, 5, 0, True))
            with self.assertRaises(fsim.Error):
                batch.endpoint(1, LEAD)  # (slot 1 has no lead)
            with self.assertRaises(fsim.Error):
                batch.status(2)
            self.assertEqual(json.loads(batch.endpoint(1, CRC).observation_json())["service"]["platform_kind"], "c2_node")
            batch.step()
            self.assertEqual(batch.status(1).tick, 1)
            self.assertTrue(batch.remove_platform(0, "wing"))
            self.assertFalse(batch.remove_platform(0, "wing"))
            self.assertEqual(batch.final_observation_json(0, WING), "")
        with self.assertRaises(fsim.Error):
            mission.MissionBatch([config(203), config(203)])  # (an env_id twice)
        with self.assertRaises(ValueError):
            mission.MissionBatch([config(204)], autoreset="sometimes")

    def test_same_step_keeps_final_documents_apart(self):
        with mission.MissionBatch([config(211, max_ticks=2)], autoreset="same_step") as batch:
            batch.reset()
            batch.step()
            batch.step()
            status = batch.status(0)
            self.assertTrue(status.ended and status.reset)
            final = json.loads(batch.final_observation_json(0, WING))
            self.assertEqual((final["tick"], final["episode"]["episode_generation"]), ("2", "1"))
            self.assertEqual(json.loads(batch.final_outcome_json(0))["reason"], "truncated")
            self.assertEqual(json.loads(batch.endpoint(0, WING).observation_json())["episode"]["episode_generation"], "2")

    def test_costs_are_a_referee_document(self):
        with open(os.path.join(ROOT, "docs", "sdk", "ma-training.schema.json"), "rb") as f:
            schema = check.Schema(check.parse(f.read()))
        episode = mission.MissionEpisode(config(221))
        episode.reset(0)
        wing = episode.endpoint(WING)
        key = json.loads(wing.observation_json())["episode"]
        wing.submit_json(json.dumps({"schema": "ma.actions.v1", "episode": key, "service_id": str(WING), "tick": "0", "requests": [
            {"request_id": "1", "kind": "dms", "operation": "send",
             "parameters": {"topic": "1001", "destination_system_ids": ["1392508931"], "byte_length": 2, "data_b64": "AAE="}}]}))
        episode.step()
        costs = check.parse(episode.referee().costs_json().encode("utf-8"))
        self.assertEqual(schema.validate(costs), [])
        totals = {s["service_id"]: s["total"] for s in costs["services"]}
        self.assertEqual((totals[str(WING)]["dms_messages"], totals[str(WING)]["dms_bytes"]), (1, 2))
        self.assertNotIn("dms_bytes", wing.observation_json())
        self.assertTrue(episode.remove_platform("wing"))
        episode.close()


class MissionBatchConcurrencyTest(unittest.TestCase):
    """reset, reset_slot and step release the GIL: a call from another thread meanwhile fails busy, whole."""

    def test_a_call_meeting_another_fails_busy_and_changes_nothing(self):
        # One-tick episodes under same_step: each whole step or reset begins exactly one generation, and between whole
        # calls the slot is always a fresh generation (tick 0, not done).
        with mission.MissionBatch([config(261, 1)], autoreset="same_step") as batch:
            batch.reset()
            done, errors = threading.Event(), []

            def worker():
                while True:
                    try:
                        batch.step()
                        break
                    except fsim.Error as e:
                        if "busy" not in str(e):
                            errors.append(repr(e))
                            break
                done.set()

            thread = threading.Thread(target=worker)
            thread.start()
            whole, busy, torn = 1, 0, []
            deadline = time.monotonic() + 120
            reads = [lambda: batch.final_outcome_json(0), lambda: batch.final_truth_json(0), lambda: batch.referee(0),
                     lambda: batch.endpoint(0, WING), lambda: batch.final_observation_json(0, WING)]
            writes = [batch.step, lambda: batch.reset_slot(0), batch.reset]
            while not done.is_set() and time.monotonic() < deadline:
                try:
                    st = batch.status(0)
                    if st.tick != 0 or st.episode_done or not st.reset:
                        torn.append(st)
                except fsim.Error as e:
                    self.assertIn("busy", str(e))
                    busy += 1
                for call, counts in [(r, 0) for r in reads] + ([(w, 1) for w in writes] if whole < 3 else []):
                    try:
                        call()
                    except fsim.Error as e:
                        self.assertIn("busy", str(e))
                        busy += 1
                    else:
                        whole += counts  # (a whole reset or step: one generation)
            thread.join(timeout=120)
            self.assertFalse(thread.is_alive())
            self.assertEqual(errors, [])
            whole += 1  # (the worker's step)
            self.assertGreater(busy, 0)
            self.assertEqual(torn, [])
            st = batch.status(0)
            self.assertEqual((st.generation, st.local_episode_index, st.tick, st.episode_done), (whole, whole - 1, 0, False))

    def test_a_batch_never_crosses_to_an_actor(self):
        from fsim import isolation
        with mission.MissionBatch([config(262)]) as batch, mission.MissionEpisode(config(263)) as episode:
            episode.reset(0)
            with self.assertRaises(TypeError):
                isolation.MissionEndpointProcess(episode, episode.endpoint(WING), "isolated_mission_actor:Actor", batch, path=[HERE])
            with self.assertRaises(TypeError):
                isolation.MissionEndpointProcess(episode, episode.endpoint(WING), "isolated_mission_actor:Actor", {"k": [batch]}, path=[HERE])


class TrainingAdapterTest(unittest.TestCase):
    def test_seed_streams_are_the_contracts(self):
        with open(os.path.join(CONTRACT, "manifest.json"), "rb") as f:
            vectors = json.loads(f.read().decode("utf-8"))["seed_streams"]
        for v in vectors:
            b, e, l, s, c = (int(v[k]) for k in ("base_seed", "env_id", "local_episode_index", "service_id", "component_id"))
            self.assertEqual(training.world_seed(b, e, l), int(v["world_seed"]))
            self.assertEqual(training.service_seed(b, e, l, s), int(v["service_seed"]))
            self.assertEqual(training.component_seed(b, e, l, s, c), int(v["component_seed"]))

    def test_hooks_run_around_every_generation_and_the_streams_stay_apart(self):
        log = []
        batch = mission.MissionBatch([config(231, max_ticks=2), wing_and_c2(232)], autoreset="next_step")
        adapter = training.TrainingAdapter(batch)
        for slot, services in ((0, (LEAD, CRC, WING)), (1, (CRC, WING))):
            for service in services:
                adapter.register(slot, service, lambda slot, service: Waiting(slot, service, log))
        with self.assertRaises(ValueError):
            adapter.register(1, LEAD, lambda slot, service: Waiting(slot, service, log))  # (slot 1 has no such service)
        adapter.reset()
        self.assertEqual(sorted(e for e in log if e[0] == "begin"), sorted(("begin", s, v, "1") for s, vs in ((0, (LEAD, CRC, WING)), (1, (CRC, WING))) for v in vs))
        for _ in range(3):
            adapter.step()
        ends = [e for e in log if e[0] == "end"]
        self.assertEqual(sorted(ends), sorted(("end", 0, v, "truncated") for v in (LEAD, CRC, WING)))
        self.assertEqual(sorted(e for e in log if e[0] == "begin" and e[3] == "2"), sorted(("begin", 0, v, "2") for v in (LEAD, CRC, WING)))
        actor, lifecycle, critic = adapter.actor_stream(), adapter.lifecycle_stream(), adapter.critic_stream()
        self.assertTrue(actor and lifecycle and critic)
        for record in actor:
            for field in ("observation", "final_observation"):
                if record.get(field):
                    self.assertEqual(json.loads(record[field])["schema"], "ma.observation.v1")
            self.assertTrue(set(record) <= {"slot", "service_id", "tick", "observation", "final_observation", "receipts"}, set(record))
        for record in lifecycle:
            self.assertTrue(set(record) <= {"slot", "service_id", "tick", "generation", "episode_done", "reason", "active", "terminated",
                                            "truncated", "final_observation", "bootstrap"}, set(record))
        for record in critic:
            self.assertEqual({json.loads(record[k])["schema"] for k in ("outcome", "truth", "costs")}, {"ma.outcome.v1", "ma.truth.v1", "ma.costs.v1"})
        truncated = [r for r in lifecycle if r["slot"] == 0 and r["truncated"]]
        self.assertEqual(sorted(r["service_id"] for r in truncated), sorted(str(v) for v in (LEAD, CRC, WING)))
        self.assertTrue(all(r["bootstrap"] for r in truncated))
        finals = [r for r in actor if r["slot"] == 0 and r.get("final_observation")]
        self.assertEqual(len(finals), 3)
        adapter.close()



class TrainingAdapterRoleTest(unittest.TestCase):
    def test_a_lost_role_stops_acting_and_leaves_one_final_snapshot(self):
        log, systems = [], {}

        def factory(slot, service):
            systems[service] = Waiting(slot, service, log)
            return systems[service]
        with mission.MissionBatch([config(271, 8)], autoreset="same_step") as batch:
            adapter = training.TrainingAdapter(batch)
            adapter.register(0, WING, factory)
            adapter.register(0, LEAD, factory)
            adapter.reset()
            adapter.step()
            batch.remove_platform(0, "wing")
            adapter.step()  # (the step its role ends in: it acted in it, before the removal took effect)
            ticks = systems[WING].ticks
            for _ in range(3):
                adapter.step()
            self.assertEqual(systems[WING].ticks, ticks)  # (an ended role acts no more)
            self.assertEqual(systems[LEAD].ticks, 5)       # (its teammate goes on)
            wing_finals = [r for r in adapter.actor_stream() if r["service_id"] == str(WING) and "final_observation" in r]
            self.assertEqual(len(wing_finals), 1)
            final = json.loads(wing_finals[0]["final_observation"])
            self.assertFalse(final["service"]["alive"])
            self.assertEqual(final["tick"], str(wing_finals[0]["tick"]))
            records = [r for r in adapter.lifecycle_stream() if r["service_id"] == str(WING)]
            self.assertTrue(records[1]["terminated"] and records[1]["final_observation"] and not records[1]["active"])
            self.assertFalse(records[1]["episode_done"])
            # the generation ends (tick limit 8): the wing's end_episode comes then, once; no second final snapshot
            for _ in range(3):
                adapter.step()
            wing_finals = [r for r in adapter.actor_stream() if r["service_id"] == str(WING) and "final_observation" in r]
            self.assertEqual(len(wing_finals), 1)
            lead_finals = [r for r in adapter.actor_stream() if r["service_id"] == str(LEAD) and "final_observation" in r]
            self.assertEqual(len(lead_finals), 1)
            self.assertEqual(sorted((e[2], e[3]) for e in log if e[0] == "end"), sorted([(WING, "truncated"), (LEAD, "truncated")]))
            # the new generation (same_step): the wing acts again
            self.assertEqual(sorted(e[2] for e in log if e[0] == "begin" and e[3] == "2"), sorted([WING, LEAD]))
            ticks = systems[WING].ticks
            adapter.step()
            self.assertEqual(systems[WING].ticks, ticks + 1)

    def test_one_begin_and_one_end_per_generation_through_every_reset(self):
        log = []
        with mission.MissionBatch([config(272, 2)], autoreset="disabled") as batch:
            adapter = training.TrainingAdapter(batch)
            adapter.register(0, WING, lambda slot, service: Waiting(slot, service, log))
            adapter.reset()
            for _ in range(4):
                adapter.step()  # (ends at tick 2; then holds)
            adapter.reset_slot(0)  # (the range's reset of a disabled slot: its systems begin again)
            adapter.step()
            adapter.reset()        # (cuts generation 2 short: its end_episode first)
            self.assertEqual([(e[0], e[3]) for e in log],
                             [("begin", "1"), ("end", "truncated"), ("begin", "2"), ("end", None), ("begin", "3")])

    def test_the_context_is_the_composition_examples(self):
        from ma_composition import contracts
        contexts = []

        class Keeps(Waiting):
            def begin_episode(self, context):
                contexts.append(context)
                super().begin_episode(context)
        with mission.MissionBatch([config(273)], first_local_episode_index=6) as batch:
            adapter = training.TrainingAdapter(batch)
            adapter.register(0, LEAD, lambda slot, service: Keeps(slot, service, []))
            adapter.reset()
            self.assertEqual(batch.base_seed(0), int(json.loads(config(273))["base_seed"]))
        context, base_seed = contexts[0], int(json.loads(config(273))["base_seed"])
        expected = contracts.context(base_seed, context.episode_key, LEAD, [1, 2, 7])
        self.assertEqual(context["slot"], 0)
        self.assertEqual(context.service_id, expected.service_id)
        self.assertEqual(context.service_seed, expected.service_seed)
        for c in (1, 2, 7):
            self.assertEqual(tuple(context.component(c)), tuple(expected.component(c)))
        self.assertEqual(context.episode_key["local_episode_index"], "6")


class Actor(threading.Thread):
    """An actor's thread on one slot: an endpoint and a referee opened earlier - reopened when their generation ended -
    read and submitted to until stopped, so that its calls meet the range's steps."""

    def __init__(self, batch, service_id=WING):
        super().__init__()
        self.batch, self.service_id = batch, service_id
        self.stopping, self.calls, self.busy, self.errors = threading.Event(), 0, 0, []
        self.first = (batch.endpoint(0, service_id), batch.referee(0))  # (opened before the steps: preopened handles)

    def run(self):
        endpoint, referee = self.first
        request = 0
        while not self.stopping.is_set():
            try:
                if endpoint is None:
                    endpoint, referee = self.batch.endpoint(0, self.service_id), self.batch.referee(0)
                for _ in range(50):  # (native calls back to back: they meet the step wherever it is)
                    text = endpoint.observation_json()
                    self.calls += 1
                    referee.outcome_json()
                snapshot = json.loads(text)
                request += 1
                endpoint.submit_json(json.dumps({"schema": "ma.actions.v1", "episode": snapshot["episode"], "service_id": str(self.service_id),
                                                 "tick": snapshot["tick"], "requests": [{"request_id": str(request), "kind": "flight", "operation": "new",
                                                                                         "parameters": {"mode": "hsa", "heading_rad": 0.2}}]}))
            except fsim.Error as e:
                if "busy" in str(e):
                    self.busy += 1
                elif "earlier generation" in str(e):
                    endpoint = referee = None
                else:
                    self.errors.append(str(e))
                    return

    def stop(self):
        self.stopping.set()
        self.join(timeout=60)


def unbusy(f):
    while True:
        try:
            return f()
        except fsim.Error as e:
            if "busy" not in str(e):
                raise


class PreopenedHandlesTest(unittest.TestCase):
    """Calls of endpoints and referees opened before a step meet that step's transition - its native step, outcome,
    final documents and autoreset - and the step runs whole or fails busy with nothing changed (M7 core, issue 1)."""

    def setUp(self):
        fsim.set_log_level("off")  # (each busy refusal is also logged: tens of thousands of them)

    def tearDown(self):
        fsim.set_log_level(os.environ.get("FSIM_LOG_LEVEL", "warning"))

    def run_steps(self, autoreset, env_id, whole_steps):
        with mission.MissionBatch([config(env_id, 1)], autoreset=autoreset) as batch:
            batch.reset()
            actor = Actor(batch)
            actor.start()
            whole, refused = 0, 0
            try:
                deadline = time.monotonic() + 180
                while whole < whole_steps and time.monotonic() < deadline:
                    before = unbusy(lambda: batch.status(0))
                    final_before = unbusy(lambda: batch.final_outcome_json(0))
                    try:
                        batch.step()
                    except fsim.Error as e:
                        self.assertIn("busy", str(e))
                        refused += 1
                        self.assertEqual(unbusy(lambda: batch.status(0)), before)  # (refused whole)
                        self.assertEqual(unbusy(lambda: batch.final_outcome_json(0)), final_before)
                        continue
                    whole += 1
                    after = unbusy(lambda: batch.status(0))
                    if autoreset == "next_step" and before.episode_done:  # a reset alone: nothing of the ended generation
                        self.assertEqual((after.generation, after.tick, after.ended, after.reset, after.episode_done),
                                         (before.generation + 1, 0, False, True, False))
                        self.assertEqual(json.loads(unbusy(lambda: batch.referee(0).truth_json()))["time_s"], 0.0)
                        continue
                    outcome = json.loads(unbusy(lambda: batch.final_outcome_json(0)))
                    observation = json.loads(unbusy(lambda: batch.final_observation_json(0, WING)))
                    self.assertTrue(outcome["episode_done"])
                    self.assertEqual((outcome["tick"], outcome["episode"]["episode_generation"]), ("1", str(before.generation)))
                    self.assertEqual((observation["tick"], observation["episode"]["episode_generation"]), ("1", str(before.generation)))
                    self.assertTrue(unbusy(lambda: batch.final_truth_json(0)) and unbusy(lambda: batch.final_costs_json(0)))
                    if autoreset == "same_step":
                        self.assertEqual((after.generation, after.tick, after.ended, after.reset, after.episode_done),
                                         (before.generation + 1, 0, True, True, False))
                    else:
                        self.assertEqual((after.generation, after.tick, after.ended, after.reset, after.episode_done),
                                         (before.generation, 1, True, False, True))
            finally:
                actor.stop()
            self.assertEqual(actor.errors, [])
            self.assertEqual(whole, whole_steps)
            self.assertGreater(actor.calls, 0)
            self.assertGreater(actor.busy, 0)  # (the actor's calls met the steps)
            last = batch.status(0)
            batch.step()
            batch.step()  # (uncontended: nothing stuck)
            self.assertEqual(batch.status(0).generation, last.generation + (2 if autoreset == "same_step" else 1))

    def test_same_step_under_preopened_calls(self):
        self.run_steps("same_step", 281, 200)

    def test_next_step_under_preopened_calls(self):
        self.run_steps("next_step", 282, 200)

    def test_the_adapter_under_preopened_calls(self):
        # The adapter's own steps meet an actor's preopened calls: a busy batch step is raised with nothing recorded,
        # and the next step() runs it without ticking the systems twice.
        log, ticks, steps = [], {}, 61

        class Counting(Waiting):
            def tick(self, endpoint):
                ticks[self.service_id] = ticks.get(self.service_id, 0) + 1
                super().tick(endpoint)
        with mission.MissionBatch([config(283, 2)], autoreset="same_step") as batch:
            adapter = training.TrainingAdapter(batch)
            for service in (WING, LEAD):
                adapter.register(0, service, lambda slot, service: Counting(slot, service, log))
            adapter.reset()
            actor = Actor(batch, CRC)
            actor.start()
            whole, retries = 0, 0
            try:
                deadline = time.monotonic() + 180
                while whole < steps and time.monotonic() < deadline:
                    try:
                        adapter.step()
                    except fsim.Error as e:
                        self.assertIn("busy", str(e))
                        retries += 1
                        continue
                    whole += 1
            finally:
                actor.stop()
            self.assertEqual(actor.errors, [])
            self.assertEqual(whole, steps)
            self.assertGreater(actor.busy, 0)
            self.assertEqual(ticks, {WING: steps, LEAD: steps})  # (once per decision tick, retries or not)
            self.assertEqual(len(adapter.critic_stream()), steps)
            lifecycle = adapter.lifecycle_stream()
            self.assertEqual(sorted(r["service_id"] for r in lifecycle), sorted([str(WING)] * steps + [str(LEAD)] * steps))
            ended = whole // 2  # (two-tick episodes, SameStep)
            for service in (WING, LEAD):
                self.assertEqual(sum(1 for e in log if e[0] == "end" and e[2] == service), ended)
                self.assertEqual(sum(1 for e in log if e[0] == "begin" and e[2] == service), ended + 1)
                finals = [r for r in adapter.actor_stream() if r["service_id"] == str(service) and "final_observation" in r]
                self.assertEqual([json.loads(r["final_observation"])["episode"]["episode_generation"] for r in finals],
                                 [str(g) for g in range(1, ended + 1)])
            self.assertEqual(batch.status(0).generation, ended + 1)


class CompositionUnderTrainingTest(unittest.TestCase):
    """The accepted M6 example's MA system, unchanged, through the training adapter and both autoresets: each
    generation the same as the example's own direct run of the same local episode."""

    def test_the_example_runs_unchanged_under_the_adapter(self):
        from ma_composition import batch as example
        direct = example.reference(4, 3)
        self.assertEqual(len(direct), 2)
        self.assertNotEqual(direct[0]["records"]["snapshots"], direct[1]["records"]["snapshots"])  # (two local episodes)
        for autoreset in ("same_step", "next_step"):
            with self.subTest(autoreset=autoreset):
                self.assertEqual(example.under_training(4, 3, autoreset), direct)


if __name__ == "__main__":
    unittest.main()
