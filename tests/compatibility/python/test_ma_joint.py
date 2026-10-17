"""Joint acceptance through the Python SDK (M8; docs/sdk/ma-training-validation.md): the recorded joint chains replayed
in-process and with the wing's actor strictly isolated in a process of its own - the same trace, byte for byte - and
the isolated actor's proxy: its endpoint's calls alone, no range or referee handle across, and no answer from an
earlier generation until the range re-binds it."""
import json
import os
import sys
import unittest

from fsim import isolation, mission

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))
JOINT = os.path.join(ROOT, "tests", "training", "fixtures", "mission_training", "joint")
sys.path.insert(0, os.path.join(ROOT, "tools", "validation", "ma_contract"))
import joint_chain  # noqa: E402

WING = joint_chain.WING


def replay_isolated(config, ticks, lines, service=WING):
    """The chain with one service's actor in its own process: it reads its snapshot and submits through its proxy."""
    episode = mission.MissionEpisode(config.replace(joint_chain.TOKEN, ""))
    episode.reset(0)
    services = sorted(int(s["service_id"]) for s in json.loads(config.replace(joint_chain.TOKEN, ""))["services"])
    trace, k = [], 0
    with isolation.MissionEndpointProcess(episode, episode.endpoint(service), "isolated_mission_actor:Actor", "joint", path=[HERE]) as actor:
        for tick in range(ticks + 1):
            snapshots = {}
            for s in services:
                snapshots[s] = actor.call("observe") if s == service else episode.endpoint(s).observation_json()
                trace.append("obs %d %d %s" % (tick, s, snapshots[s]))
            while k < len(lines) and lines[k][0] == tick:
                _, s, requests = lines[k]
                key = json.dumps(json.loads(snapshots[s])["episode"], separators=(",", ":"))
                document = '{"schema":"ma.actions.v1","episode":%s,"service_id":"%d","tick":"%d","requests":%s}' % (key, s, tick, requests)
                ack = actor.call("submit", document) if s == service else episode.endpoint(s).submit_json(document)
                trace.append("ack %d %d %s" % (tick, s, ack))
                k += 1
            if tick < ticks:
                episode.step()
    episode.close()
    return trace


class JointChainTest(unittest.TestCase):
    def test_an_isolated_actor_sees_and_does_what_an_in_process_one_does(self):
        for name in ("chain.joint.txt", "chain.lossy.txt"):
            with self.subTest(chain=name):
                config, ticks, lines = joint_chain.read_chain(os.path.join(JOINT, name))
                self.assertTrue(any(s == WING for _, s, _ in lines))  # (the isolated service acts in the chain)
                in_process = joint_chain.replay(mission, config, ticks, lines)
                self.assertEqual(replay_isolated(config, ticks, lines), in_process)

    def test_the_isolated_proxy_holds_its_endpoint_alone_and_its_generation_only(self):
        config, ticks, lines = joint_chain.read_chain(os.path.join(JOINT, "chain.joint.txt"))
        with mission.MissionEpisode(config.replace(joint_chain.TOKEN, "")) as episode:
            episode.reset(0)
            referee = episode.referee()
            with isolation.MissionEndpointProcess(episode, episode.endpoint(WING), "isolated_mission_actor:Actor", "joint", path=[HERE]) as actor:
                self.assertEqual(set(actor.call("reach").values()), {"refused"})  # (no range member on its proxy)
                for handle in (referee, episode, json.loads(referee.truth_json())):
                    with self.assertRaises(TypeError):
                        actor.call("echo", handle)
                before = actor.call("observe")
                episode.reset(0)  # (the next generation: the process's endpoint is an earlier one's)
                with self.assertRaises(Exception):
                    actor.call("observe")
                actor.rebind(episode.endpoint(WING))
                after = json.loads(actor.call("observe"))
                self.assertEqual(after["episode"]["episode_generation"], "2")
                self.assertEqual(json.loads(before)["episode"]["episode_generation"], "1")


if __name__ == "__main__":
    unittest.main()
