"""ADR-31, AB-6: an MA in a process of its own - tier 3. Its only channel is its agent's calls; the isolated run is the
in-process run, to the bit."""
import math
import os
import sys
import tomllib
import unittest

import fsim
import fsim.isolation

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(os.path.dirname(HERE)))


def world(name, publish=False):
    w = fsim.World(name, publish=publish, workers=1, pin_workers=False, terrain=False)
    a = w.create_vehicle("a", "jsbsim:f16c", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=6000.0, airspeed_ms=230.0)
    return w, a


class IsolationTest(unittest.TestCase):
    def test_the_isolated_run_is_the_in_process_run(self):
        sys.path.insert(0, os.path.join(REPO, "examples", "simulation", "python"))
        try:
            import links
        finally:
            sys.path.pop(0)
        here = links.run(isolated=False, seconds=60, quiet=True)
        apart = links.run(isolated=True, seconds=60, quiet=True)
        self.assertEqual(here["states"], apart["states"])  # (every vehicle's true state, to the bit)
        self.assertEqual(here["net"], apart["net"])
        self.assertEqual(here["audit"], apart["audit"])  # (the same calls, the same number of times)
        self.assertEqual(here["target"], apart["target"])

    def test_its_only_channel(self):
        w, a = world("py-isolation-reach")
        agent = w.create_agent(a, "blue-1")
        with fsim.isolation.AgentProcess(w, agent, "isolated_ma:Prober", "probe", path=[HERE]) as ma:
            self.assertEqual(ma.call("identity"), ("probe", a.id, "blue-1", agent.controller))
            reach = ma.call("reach")
            self.assertEqual({k for k, v in reach.items() if v != "refused"}, set())
            aid, live, level = ma.call("fly", 90.0)
            self.assertTrue(live)
            w.step(5)
            sp = ma.call("update", 120.0)
            self.assertEqual(sp.method, "submit_hsa")
            self.assertAlmostEqual(sp.kwargs["heading_rad"], math.radians(120.0))
            self.assertEqual(ma.call("own_ship"), agent.ownship())  # (a named tuple, exactly)
            kind, text = ma.call("refused")
            try:  # (as the same call raises in the range's process)
                agent.submit_hsa(heading_rad=0.0, speed=230.0, altitude_m=6000.0, source=fsim.Source.OVERRIDE)
                here = None
            except fsim.Error as e:
                here = (type(e).__name__, str(e))
            self.assertEqual((kind, text), here)
            with self.assertRaises(fsim.isolation.AgentError) as raised:
                ma.call("fail")
            self.assertIn("the MA's own mistake", str(raised.exception))
            data = bytes(range(256))
            self.assertEqual(ma.call("bytes_round_trip", data)[:2], (data, list(data)))
            # what the range would answer a forged message: no call but an agent's, no activity it does not hold
            for forged in ({"op": "agent", "m": "_world"}, {"op": "agent", "m": "__class__"}, {"op": "agent", "m": "step"},
                           {"op": "activity", "id": aid + 12345, "m": "cancel"}):
                self.assertEqual(ma._serve(forged)["op"], "error", forged)
            self.assertEqual(ma._serve({"op": "activity", "id": aid, "m": "live"})["op"], "result")
            self.assertGreater(ma.calls, 0)

    def test_nothing_is_rebuilt_but_fsim_s_own(self):
        with self.assertRaises(TypeError):
            fsim.isolation._decode({"$n": "os.system", "v": ["x"]}, lambda i: i)
        with self.assertRaises(TypeError):
            fsim.isolation._decode({"$e": "subprocess.Popen", "v": 0}, lambda i: i)
        with self.assertRaises(TypeError):
            fsim.isolation._decode({"$np": "|O", "s": [1], "b": ""}, lambda i: i)
        own = fsim.OwnShip(*([0.0] * 5 + [(0.0, 0.0, 0.0)] * 4 + [0.0] * 7 + [True, True, True, 0.0]))
        self.assertEqual(fsim.isolation._decode(fsim.isolation._encode(own), lambda i: i), own)

    def test_side_channels_refused(self):
        w, a = world("py-isolation-published", publish=True)
        agent = w.create_agent(a, "blue-1")
        if w.published:  # (another process may hold the name: then unpublished, and nothing to refuse)
            with self.assertRaises(RuntimeError):
                fsim.isolation.AgentProcess(w, agent, "isolated_ma:Prober", "probe", path=[HERE])
        w2, _ = world("py-isolation-not-an-agent")
        with self.assertRaises(TypeError):
            fsim.isolation.AgentProcess(w2, w2, "isolated_ma:Prober", path=[HERE])  # (an agent alone crosses)

    def test_its_calls_are_the_agents(self):
        """What an isolated MA may call is exactly what docs/sdk/boundary.toml gives an agent of fsim.Agent and
        fsim.AgentActivity."""
        rules = tomllib.load(open(os.path.join(REPO, "docs", "sdk", "boundary.toml"), "rb"))["rule"]
        agent = {n for r in rules if r["principal"] == "agent" for n in r.get("names", [])}
        calls = {"fsim.agent.Agent." + n for n in fsim.isolation.agent_calls()}
        calls |= {"fsim.agent.AgentActivity." + n for n in fsim.isolation.activity_calls()}
        self.assertEqual(calls - agent, set())
        self.assertEqual({n for n in agent if n.startswith(("fsim.agent.Agent.", "fsim.agent.AgentActivity."))} - calls, set())


if __name__ == "__main__":
    unittest.main()
