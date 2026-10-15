"""Mission episodes through the Python SDK (docs/sdk/mission-training.md; M5 core): every document the episode writes held
to the frozen schema and the repository checker's rules, reads that consume nothing, the contract's errors as
fsim.Error, a busy episode that never waits, and the actor's process boundary - its endpoint's three calls and nothing
else, no diagnostic episode, no referee document across."""
import json
import os
import subprocess
import sys
import threading
import time
import unittest

import fsim
from fsim import isolation, mission

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
CONTRACT = os.path.join(ROOT, "tests", "fixtures", "mission_training", "contract")
sys.path.insert(0, os.path.join(ROOT, "tools", "ma_contract"))
import check  # noqa: E402  (the repository's contract checker)

WING, CRC, LEAD_FLIGHT, LEAD_SENSORS = 9007199254740995, 4, 1, 2


def fixture(name):
    with open(os.path.join(CONTRACT, name), "rb") as f:
        return f.read().decode("utf-8")


def config(edit=None):
    document = json.loads(fixture("config.two-ship-c2.json"))
    if edit:
        edit(document)
    return json.dumps(document)


def answering(observation, requests):
    """An ma.actions.v1 document answering a snapshot's episode key, service and tick."""
    o = json.loads(observation)
    return json.dumps({"schema": "ma.actions.v1", "episode": o["episode"], "service_id": o["service"]["service_id"], "tick": o["tick"],
                       "requests": requests})


class MissionDocumentsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with open(os.path.join(ROOT, "docs", "sdk", "ma-contract.schema.json"), "rb") as f:
            cls.schema = check.Schema(check.parse(f.read()))

    def conforms(self, text, name):
        document = check.parse(text.encode("utf-8"))
        errors = self.schema.validate(document) + check.rules(document)[0]
        self.assertEqual(errors, [], "%s does not conform:\n%s" % (name, "\n".join(errors[:10])))
        return document

    def test_every_document_conforms_through_every_generic_operation(self):
        with mission.MissionEpisode(config()) as episode:
            self.assertIsNone(episode.reset(41))
            endpoints = {s: episode.endpoint(s) for s in (LEAD_FLIGHT, LEAD_SENSORS, CRC, WING)}
            referee = episode.referee()
            # every generic operation the frozen fixtures name, as the services they were written for
            for service, name in ((WING, "actions.wing-ma.json"), (CRC, "actions.crc-c2.json")):
                requests = json.loads(fixture(name))["requests"]
                ack = endpoints[service].submit_json(answering(endpoints[service].observation_json(), requests))
                document = self.conforms(ack, "%s acknowledgment" % name)
                self.assertTrue(document["submission"]["accepted"])
            for tick in range(1, 5):
                episode.step()
                for service, endpoint in endpoints.items():
                    observation = self.conforms(endpoint.observation_json(), "observation %d of %d" % (tick, service))
                    self.assertEqual(observation["tick"], str(tick))
                    self.conforms(endpoint.receipts_json(), "receipts %d of %d" % (tick, service))
                self.conforms(referee.outcome_json(), "outcome %d" % tick)
                self.conforms(referee.truth_json(), "truth %d" % tick)
            snapshot = json.loads(endpoints[WING].observation_json())
            self.assertEqual(snapshot["service"]["service_id"], "9007199254740995")  # (above 2**53, exact)
            states = {r["request_id"]: r["state"] for r in snapshot["receipts"]}
            self.assertTrue(states)
            self.assertNotIn("queued", states.values())

    def test_reads_consume_nothing_and_errors_are_fsim_errors(self):
        episode = mission.MissionEpisode(config())
        with self.assertRaises(fsim.Error):
            episode.endpoint(WING)  # (not reset)
        episode.reset(3)
        with self.assertRaises(fsim.Error):
            episode.endpoint(5)  # (not configured)
        wing = episode.endpoint(WING)
        first = wing.observation_json()
        self.assertIsInstance(first, str)
        self.assertEqual([wing.observation_json() for _ in range(5)], [first] * 5)
        with self.assertRaises(fsim.Error):
            wing.submit_json("{")
        with self.assertRaises(TypeError):
            wing.submit_json(42)
        ack = wing.submit_json(answering(first, []).encode("utf-8"))
        self.assertEqual(wing.receipts_json(), ack)
        episode.reset(3)
        with self.assertRaises(fsim.Error):
            wing.observation_json()  # (stale)
        fresh = episode.endpoint(WING)
        stale = json.loads(fresh.submit_json(answering(first, [])))
        self.assertEqual(stale["submission"]["reason"], "stale_generation")
        episode.close()
        episode.close()
        with self.assertRaises(fsim.Error):
            fresh.observation_json()  # (closed with its episode)
        fresh.close()
        fresh.close()

    def test_a_busy_episode_answers_at_once_and_never_waits(self):
        episode = mission.MissionEpisode(config())
        episode.reset(9)
        wing = episode.endpoint(WING)
        done, seen = threading.Event(), {"ok": 0, "busy": 0, "other": []}

        def stepping():
            for _ in range(6):
                episode.step()
            done.set()

        thread = threading.Thread(target=stepping)
        thread.start()
        while not done.is_set():
            try:
                wing.observation_json()
                seen["ok"] += 1
            except fsim.Error as e:
                if "busy" in str(e):
                    seen["busy"] += 1
                else:
                    seen["other"].append(str(e))
            time.sleep(0.005)
        thread.join()
        self.assertEqual(seen["other"], [])
        self.assertGreater(seen["ok"] + seen["busy"], 0)
        self.assertEqual(json.loads(wing.observation_json())["tick"], "6")


class MissionIsolationTest(unittest.TestCase):
    def test_an_actor_in_its_own_process_holds_its_endpoint_alone(self):
        episode = mission.MissionEpisode(config())
        episode.reset(5)
        wing = episode.endpoint(WING)
        with isolation.MissionEndpointProcess(episode, wing, "isolated_mission_actor:Actor", "wing", path=[HERE]) as actor:
            self.assertEqual(actor.call("observe"), wing.observation_json())
            ack = json.loads(actor.call("wait"))
            self.assertTrue(ack["submission"]["accepted"])
            self.assertEqual(actor.call("receipts"), wing.receipts_json())
            self.assertEqual(set(actor.call("reach").values()), {"refused"})
            self.assertEqual(set(actor.call("forge").values()), {"AttributeError", "TypeError"})
            episode.step()
            self.assertEqual(actor.call("observe"), wing.observation_json())
            # a new generation: the process's endpoint is stale until the range re-binds it explicitly
            episode.reset(5)
            with self.assertRaises(fsim.Error):
                actor.call("observe")
            actor.rebind(episode.endpoint(WING))
            self.assertEqual(json.loads(actor.call("observe"))["episode"]["episode_generation"], "2")
            with self.assertRaises(ValueError):
                actor.rebind(episode.endpoint(CRC))
        self.assertGreater(actor.calls, 0)

    def test_the_process_endpoint_is_the_episodes_own_current_one(self):
        """Ownership is the episode's real handle, not an equal service id or episode key: another episode's endpoint,
        a diagnostic episode's, a stale one or a closed one is refused at construction and at every rebind."""
        clean = mission.MissionEpisode(config())
        twin = mission.MissionEpisode(config())  # (the same configuration: equal keys, equal service ids)
        diagnostic = mission.MissionEpisode(config(lambda c: c["execution"].update(diagnostic=True)))
        for e in (clean, twin, diagnostic):
            e.reset(5)
        stale = clean.endpoint(WING)
        clean.reset(5)
        closed = clean.endpoint(WING)
        closed.close()
        for foreign in (twin.endpoint(WING), diagnostic.endpoint(WING), stale, closed):
            with self.assertRaises(ValueError, msg=repr(foreign)):
                isolation.MissionEndpointProcess(clean, foreign, "isolated_mission_actor:Actor", "x", path=[HERE])
        with self.assertRaises(RuntimeError):
            isolation.MissionEndpointProcess(diagnostic, diagnostic.endpoint(WING), "isolated_mission_actor:Actor", "x", path=[HERE])
        with isolation.MissionEndpointProcess(clean, clean.endpoint(WING), "isolated_mission_actor:Actor", "x", path=[HERE]) as actor:
            for foreign in (twin.endpoint(WING), diagnostic.endpoint(WING), stale):
                with self.assertRaises(ValueError, msg=repr(foreign)):
                    actor.rebind(foreign)
            self.assertEqual(json.loads(actor.call("observe"))["episode"], json.loads(clean.endpoint(WING).observation_json())["episode"])
            clean.reset(6)
            actor.rebind(clean.endpoint(WING))  # (the same episode's new generation: supported)
            self.assertEqual(json.loads(actor.call("observe"))["episode"]["local_episode_index"], "6")
        for e in (clean, twin, diagnostic):
            e.close()

    def test_referee_documents_never_cross_parsed_or_nested(self):
        episode = mission.MissionEpisode(config())
        episode.reset(2)
        wing, referee = episode.endpoint(WING), episode.referee()
        truth, outcome = json.loads(referee.truth_json()), json.loads(referee.outcome_json())
        carried = [truth, outcome, [truth], (1, {"nested": [outcome]}), {"k": (truth,)}, json.dumps([{"inner": truth}]),
                   json.dumps({"a": outcome}).encode("utf-8"), bytearray(json.dumps([[outcome]]).encode("utf-8"))]
        for argument in carried:
            with self.assertRaises(TypeError, msg=repr(argument)[:80]):
                isolation.MissionEndpointProcess(episode, wing, "isolated_mission_actor:Actor", argument, path=[HERE])
        with isolation.MissionEndpointProcess(episode, wing, "isolated_mission_actor:Actor", {"ordinary": [1, "two"]}, path=[HERE]) as actor:
            self.assertEqual(actor.call("echo", {"ordinary": [1, "two"]}), {"ordinary": [1, "two"]})
            for argument in carried:
                with self.assertRaises(TypeError, msg=repr(argument)[:80]):
                    actor.call("echo", argument)
                with self.assertRaises(TypeError, msg=repr(argument)[:80]):
                    actor.call("echo", value={"deep": [argument]})
        episode.close()


class MissionLifetimeTest(unittest.TestCase):
    """Cases that could end a process run in one of their own, bounded: a native failure fails the case, not the runner."""

    def probe(self, case):
        env = dict(os.environ)
        env["PYTHONFAULTHANDLER"] = "1"
        try:
            run = subprocess.run([sys.executable, "-B", os.path.join(HERE, "mission_lifetime_probe.py"), case], env=env, cwd=HERE,
                                 capture_output=True, text=True, timeout=240)
        except subprocess.TimeoutExpired:
            self.fail("%s: did not finish in 240 s" % case)
        lines = [line for line in run.stdout.splitlines() if line.startswith("{")]
        self.assertEqual(run.returncode, 0, "%s: exit %s\n%s\n%s" % (case, run.returncode, run.stdout[-2000:], run.stderr[-3000:]))
        verdict = json.loads(lines[-1])
        self.assertTrue(verdict["ok"], verdict)
        return verdict

    def test_close_during_a_reset_never_waits_and_never_faults(self):
        verdict = self.probe("close_reset")
        self.assertEqual(verdict["first"], "returned")

    def test_close_during_a_step_never_waits_and_never_faults(self):
        self.probe("close_step")

    def test_a_reset_refused_as_busy_changes_nothing(self):
        verdict = self.probe("overlap_reset")
        self.assertEqual(verdict["episode"]["local_episode_index"], "11")

    def test_strict_isolation_refuses_what_could_carry_truth(self):
        diagnostic = mission.MissionEpisode(config(lambda c: c["execution"].update(diagnostic=True)))
        diagnostic.reset(1)
        self.assertTrue(diagnostic.diagnostic)
        with self.assertRaises(RuntimeError):
            isolation.MissionEndpointProcess(diagnostic, diagnostic.endpoint(WING), "isolated_mission_actor:Actor", "x", path=[HERE])
        episode = mission.MissionEpisode(config())
        episode.reset(1)
        wing, referee = episode.endpoint(WING), episode.referee()
        for argument in (episode, referee, episode.endpoint(CRC), referee.outcome_json(), referee.truth_json()):
            with self.assertRaises(TypeError):
                isolation.MissionEndpointProcess(episode, wing, "isolated_mission_actor:Actor", argument, path=[HERE])
        self.assertEqual(set(isolation.mission_endpoint_calls()), {"observation_json", "submit_json", "receipts_json"})
        self.assertEqual({n for n in dir(isolation.RemoteMissionEndpoint) if not n.startswith("_")},
                         {"observation_json", "submit_json", "receipts_json"})


if __name__ == "__main__":
    unittest.main()
