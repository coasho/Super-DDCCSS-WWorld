"""Batches and the training adapter through the Python SDK (docs/sdk/mission-training.md, section 18; M7 core): slots of
different topology, autoreset by mode, final documents by principal, the training adapter's lifecycle hooks around
every generation, and three streams kept apart - what actors see, the lifecycle signals a learner needs, and the
referee's documents."""
import json
import os
import sys
import unittest

import fsim
from fsim import mission, training

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
CONTRACT = os.path.join(ROOT, "tests", "fixtures", "mission_training", "contract")
sys.path.insert(0, os.path.join(ROOT, "tools", "ma_contract"))
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


if __name__ == "__main__":
    unittest.main()
