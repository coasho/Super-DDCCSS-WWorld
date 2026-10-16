"""Calls from other threads meeting mission-batch transitions, through the Python SDK (M7 core; docs/sdk/mission-training.md,
section 18): batch calls that meet fail busy whole and change nothing, and endpoints and referees opened earlier meet
each step's transition - its native step, outcome, final documents and autoreset - without leaving a slot stuck, also
under the training adapter. Hundreds of steps each: CTest test "python_sdk test_mission_concurrency", label stress,
apart from the rest of test_mission_training, whose configurations and stand-in system it uses."""
import json
import os
import threading
import time
import unittest

import fsim
from fsim import mission, training

from test_mission_training import CRC, HERE, LEAD, WING, Waiting, config


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
                deadline = time.monotonic() + 600  # (a hang guard: loaded, it steps slower)
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
                deadline = time.monotonic() + 600
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


if __name__ == "__main__":
    unittest.main()
