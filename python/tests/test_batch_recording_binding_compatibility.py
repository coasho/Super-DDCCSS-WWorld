"""A5 batch/recording binding contracts before native adapter extraction."""
import array
import gc
import os
import struct
import tempfile
import types
import unittest

import numpy as np

import fsim
from fsim import _native as native
from fsim.recording import Recording
from fsim.vecenv import VecEnv


class BatchRecordingBindingCompatibility(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory(prefix="a5-binding-")
        cls.path = os.path.join(cls.directory.name, "record.fsrec")
        world = fsim.World("batch-recording-binding", publish=False, workers=1,
                           pin_workers=False, record_path=cls.path)
        world.create_vehicle("recorded", altitude_msl_m=1200.0, airspeed_ms=60.0)
        world.step(3)
        world.close()
        del world
        gc.collect()

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def make(self):
        env = fsim.VecEnv(1, publish=False, workers=1, seed=7)
        self.addCleanup(env.close)
        return env

    def test_native_type_and_descriptor_contracts(self):
        self.assertIs(fsim.VecEnv, VecEnv)
        self.assertIs(fsim.Recording, Recording)
        env = self.make()
        rec = native.Recording(self.path)
        cases = (
            (native.VecEnv, env._h, 8, ("step", "reset", "buffers", "names", "set_action_ranges",
             "vehicle_steps", "set_autoreset", "autoreset", "vehicle_ids", "world", "_capsule")),
            (native.Recording, rec, 3, ("info", "frame_time", "samples", "events")),
        )
        for owner, instance, words, methods in cases:
            self.assertEqual((owner.__module__, owner.__qualname__), ("fsim._native", owner.__name__))
            self.assertEqual(owner.__basicsize__, words * struct.calcsize("P"))
            self.assertEqual(owner.__itemsize__, 0)
            mask = (1 << 4) | (1 << 9) | (1 << 10) | (1 << 14)
            self.assertEqual(owner.__flags__ & mask, 1 << 9)
            with self.assertRaisesRegex(TypeError, "not an acceptable base type"):
                type("Derived", (owner,), {})
            for name in methods:
                with self.subTest(owner=owner.__name__, method=name):
                    descriptor = vars(owner)[name]
                    self.assertIsInstance(descriptor, types.MethodDescriptorType)
                    self.assertIs(descriptor.__objclass__, owner)
                    self.assertEqual(descriptor.__qualname__, owner.__name__ + "." + name)
                    self.assertIs(getattr(instance, name).__self__, instance)
                    with self.assertRaisesRegex(TypeError, "takes no keyword arguments"):
                        getattr(instance, name)(unused=0)

    def test_borrowed_world_and_all_result_buffers_keep_owner_alive(self):
        env = self.make()
        world = env.world
        self.assertIs(env.world, world)
        self.assertTrue(world._h.borrowed())
        self.assertIsNot(env._h.world(), env._h.world())
        ids = env.vehicle_ids
        env.close()
        del env
        gc.collect()
        self.assertEqual(world.vehicle(int(ids[0])).name, "env0/0")
        world.close()

        env = self.make()
        keys = ("observations", "rewards", "terminated", "truncated", "final_observations", "episode_steps")
        views = [np.frombuffer(env._h.buffers()[key], dtype=np.uint8) for key in keys]
        pointers = [view.ctypes.data for view in views]
        for view in views:
            self.assertFalse(view.flags.writeable)
            with self.assertRaises(ValueError):
                view[0] = 0
        env.reset(7)
        env.step(np.zeros(env.action_size, np.float32))
        self.assertEqual(pointers, [np.frombuffer(env._h.buffers()[key], dtype=np.uint8).ctypes.data
                                    for key in keys])
        expected = [view.copy() for view in views]
        env.close()
        del env
        gc.collect()
        for view, before in zip(views, expected):
            np.testing.assert_array_equal(view, before)

    def test_float_narrowing_conversion_count_and_export_release(self):
        reference, env = self.make(), self.make()
        a64 = np.array([0.123456789, -0.123456789, 0.0, 0.700000001], np.float64)
        a32 = a64.astype(np.float32)
        forms = [a32, a64, a64.tolist(), np.column_stack((a32, a32)).reshape(-1)[::2]]
        for form in forms:
            reference.reset(19)
            env.reset(19)
            reference.step(a32)
            env.step(form)
            np.testing.assert_array_equal(env.observations, reference.observations)
        for code, count, error in (("f", 4, None), ("d", 4, None),
                                   ("f", 3, ValueError), ("d", 5, ValueError), ("i", 4, TypeError)):
            with self.subTest(code=code, count=count):
                exporter = array.array(code, [0] * count)
                if error is None:
                    env._h.step(exporter)
                else:
                    with self.assertRaises(error):
                        env._h.step(exporter)
                exporter.append(0)  # An unreleased Py_buffer would refuse this resize.
        steps = env.vehicle_steps
        with self.assertRaisesRegex(ValueError, "expected 4 action values.*got 3"):
            env._h.step(np.zeros(3, np.float64))
        self.assertEqual(env.vehicle_steps, steps)
        env._h.step(np.zeros(4, np.float32))
        self.assertGreater(env.vehicle_steps, steps)

    def test_autoreset_validation_preserves_current_mode(self):
        env = self.make()
        env.autoreset = "same_step"
        with self.assertRaises(ValueError):
            env.autoreset = "sometimes"
        with self.assertRaisesRegex(fsim.Error, "FSIM_AUTORESET_NEXT_STEP"):
            env._h.set_autoreset(99)
        with self.assertRaises(TypeError):
            env._h.set_autoreset("same_step")
        self.assertEqual(env.autoreset, "same_step")
        env.autoreset = "next_step"
        self.assertEqual(env.autoreset, "next_step")

    def test_recording_metadata_empty_frames_and_sample_owner(self):
        rec = fsim.Recording(self.path)
        self.assertEqual((rec.world_name, rec.frame_count, rec.frame_skip),
                         ("batch-recording-binding", 3, 4))
        self.assertEqual(rec.dt, 1.0 / 120.0)
        self.assertEqual(set(rec._h.info()), {"world_name", "frame_count", "dt", "frame_skip"})
        self.assertEqual(rec._h.samples(99), b"")
        self.assertEqual(rec.samples(99).size, 0)
        self.assertEqual(rec.events(99), [])
        self.assertEqual(rec.frame_time(99), 0.0)
        event = rec.events(0)[0]
        self.assertEqual(event["name"], "recorded")
        self.assertIs(event["alive"], True)
        for key in ("name", "type", "model"):
            self.assertIsInstance(event[key], str)
        samples = rec.samples(1)
        self.assertEqual(samples.dtype, fsim.recorded_sample_dtype)
        self.assertFalse(samples.flags.writeable)
        self.assertEqual(samples.ctypes.data, rec.samples(1).ctypes.data)
        with self.assertRaises(ValueError):
            samples["slot"][0] = 7
        expected = samples.copy()
        del rec
        gc.collect()
        np.testing.assert_array_equal(samples, expected)
        self.assertEqual(event["name"], "recorded")

    def test_recording_constructor_and_frame_conversion_guards(self):
        with self.assertRaisesRegex(TypeError, "Recording\\(path\\)"):
            native.Recording(path=self.path)
        with self.assertRaises(TypeError):
            native.Recording(None)
        with self.assertRaisesRegex(fsim.Error, "fsim_recording_load"):
            native.Recording(self.path + ".missing")
        rec = native.Recording(self.path)
        for name in ("samples", "events", "frame_time"):
            for frame, error in ((-1, OverflowError), (1 << 32, OverflowError), (0.5, TypeError)):
                with self.subTest(method=name, frame=frame):
                    with self.assertRaises(error):
                        getattr(rec, name)(frame)


if __name__ == "__main__":
    unittest.main()
