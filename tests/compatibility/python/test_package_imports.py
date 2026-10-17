"""The package's import structure (ARCH-07; docs/architecture/design/directory-layout.md, section 4).

The core modules import without the optional learning and GPU packages, the optional modules fail without them, and
the public names whose code moved into the private implementation packages keep their public module as owner: their
qualified names and pickling are what they were."""
import importlib
import os
import pickle
import subprocess
import sys
import textwrap
import unittest

import fsim
import fsim.training
import fsim.world

CORE = ["fsim", "fsim.world", "fsim.agent", "fsim.vecenv", "fsim.mission", "fsim.training", "fsim.isolation", "fsim.agra",
        "fsim.catalog", "fsim.recording"]
OPTIONAL = {"fsim.gym": "gymnasium", "fsim.sb3": "gymnasium", "fsim.vision": "fsim._vision"}
BLOCKED = ["gymnasium", "stable_baselines3", "torch", "fsim._vision"]

# the records World and Agent share (fsim._simulation.values), owned by fsim.world
SHARED_RECORDS = ["ActiveOverride", "C2Authority", "C2Task", "CommandStatus", "CommsState", "ConsentRequest", "DmsReceived",
                  "DmsTransmission", "LinkTrack", "NetStats", "PeerReport", "Report", "ReportCollection", "ReportItem",
                  "ReportMeta", "StrikeStatus"]


class WithoutOptionalPackages(unittest.TestCase):
    def run_blocked(self, body):
        """`body` in a fresh interpreter where importing any BLOCKED package raises ImportError."""
        script = textwrap.dedent('''
            import importlib.abc, json, sys
            BLOCKED = %r
            class Block(importlib.abc.MetaPathFinder):
                def find_spec(self, name, path=None, target=None):
                    if any(name == b or name.startswith(b + ".") for b in BLOCKED):
                        raise ImportError("blocked for the test: " + name)
                    return None
            sys.meta_path.insert(0, Block())
        ''' % BLOCKED) + textwrap.dedent(body)
        run = subprocess.run([sys.executable, "-B", "-c", script], capture_output=True, text=True, env=dict(os.environ),
                             timeout=300)
        self.assertEqual(run.returncode, 0, run.stderr)
        return run.stdout

    def test_the_core_imports_without_learning_or_gpu_packages(self):
        out = self.run_blocked('''
            import importlib
            for name in %r:
                importlib.import_module(name)
            print(json.dumps(sorted(m for m in sys.modules if any(m == b or m.startswith(b + ".") for b in BLOCKED))))
        ''' % CORE)
        self.assertEqual(out.strip(), "[]")

    def test_an_optional_module_fails_without_its_package(self):
        out = self.run_blocked('''
            import importlib
            failed = {}
            for name in %r:
                try:
                    importlib.import_module(name)
                except ImportError as e:
                    failed[name] = str(e)
            print(json.dumps(sorted(failed)))
        ''' % sorted(OPTIONAL))
        self.assertEqual(out.strip(), str(sorted(OPTIONAL)).replace("'", '"'))


class PublicOwners(unittest.TestCase):
    def test_the_training_names_keep_their_module(self):
        for name in fsim.training.__all__:
            with self.subTest(name=name):
                obj = getattr(fsim.training, name)
                self.assertEqual(obj.__module__, "fsim.training")
                self.assertEqual(obj.__qualname__, name)
                self.assertIs(pickle.loads(pickle.dumps(obj)), obj)
        context = fsim.training.ComponentContext({"env_id": 1}, "7", 42)
        self.assertEqual(pickle.loads(pickle.dumps(context)), context)
        episode = fsim.training.EpisodeContext(0, 7, {"env_id": 1, "local_episode_index": 2}, 3)
        again = pickle.loads(pickle.dumps(episode))
        self.assertEqual((again, again.service_seed, type(again)), (episode, episode.service_seed, fsim.training.EpisodeContext))

    def test_the_shared_records_keep_fsim_world_as_their_owner(self):
        for name in SHARED_RECORDS:
            with self.subTest(name=name):
                cls = getattr(fsim.world, name)
                self.assertEqual((cls.__module__, cls.__qualname__), ("fsim.world", name))
                record = cls(*range(len(cls._fields)))
                again = pickle.loads(pickle.dumps(record))
                self.assertIs(type(again), cls)
                self.assertEqual(again, record)
                self.assertIs(getattr(fsim.agent, name), cls)  # (the agent wrapper builds the same records)

    def test_the_isolation_names_keep_fsim_isolation_as_their_owner(self):
        import fsim.isolation
        for name in fsim.isolation.__all__ + ["RemoteAgent"]:
            with self.subTest(name=name):
                obj = getattr(fsim.isolation, name)
                self.assertEqual((obj.__module__, obj.__qualname__), ("fsim.isolation", name))
                self.assertIs(pickle.loads(pickle.dumps(obj)), obj)
        error = fsim.isolation.AgentError("the MA's own mistake")
        again = pickle.loads(pickle.dumps(error))
        self.assertEqual((type(again), again.args), (fsim.isolation.AgentError, error.args))
        # the codec carries the public value classes fsim.isolation binds, and imports no public module itself
        codec = importlib.import_module("fsim._adapters.codec")
        self.assertIs(codec._BatchCommand, fsim.world.BatchCommand)
        self.assertIs(codec._AgentActivity, fsim.agent.AgentActivity)
        command = fsim.world.BatchCommand("submit_hsa", heading_rad=1.0)
        again = codec._decode(codec._encode(command), lambda i: i)
        self.assertEqual((type(again), again.method, again.kwargs), (fsim.world.BatchCommand, "submit_hsa", {"heading_rad": 1.0}))

    def test_the_training_errors_are_the_package_error(self):
        adapter = importlib.import_module("fsim._training.adapter")
        self.assertIs(adapter._native.Error, fsim.Error)

    def test_the_cameras_keep_fsim_vision_as_their_owner(self):
        try:
            import fsim.vision
        except ImportError as e:
            self.skipTest("fsim.vision is not built here: %s" % e)
        for name in ("Camera", "Vision", "VisionBatch"):
            with self.subTest(name=name):
                cls = getattr(fsim.vision, name)
                self.assertEqual((cls.__module__, cls.__qualname__), ("fsim.vision", name))
                self.assertIs(pickle.loads(pickle.dumps(cls)), cls)
        cameras = importlib.import_module("fsim._visualization.cameras")
        self.assertNotIn("VecEnv", vars(cameras))  # (the camera core composes no VecEnv: fsim.vision's VisionBatch does)


if __name__ == "__main__":
    unittest.main()
