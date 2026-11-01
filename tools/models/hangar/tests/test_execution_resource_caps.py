"""Fleet-only execution limits; no model, SDK or native solver is executed."""
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from hangar import performance
from hangar.shape import meshkit


class StopBeforeSdkExecution(Exception):
    pass


class ExecutionResourceCaps(unittest.TestCase):
    def test_phase_reservation_is_local_nested_bounded_and_restored(self):
        from hangar import resources
        observed=[]
        with patch.dict(os.environ,{'FSIM_HANGAR_THREADS':'1'}):
            with resources.thread_reservation(2):
                self.assertEqual(resources.threads(),2)
                with resources.thread_reservation(4):self.assertEqual(resources.threads(),2)
                with resources.thread_reservation(1):self.assertEqual(resources.threads(),1)
                worker=threading.Thread(target=lambda:observed.append(resources.threads()))
                worker.start();worker.join(2);self.assertFalse(worker.is_alive())
                self.assertEqual(resources.threads(),2)
            self.assertEqual(resources.threads(),1)
            self.assertEqual(observed,[1])
            with self.assertRaises(RuntimeError),resources.thread_reservation(2):raise RuntimeError('scope')
            self.assertEqual(resources.threads(),1)

    def test_phase_reservation_does_not_hide_invalid_environment_inputs(self):
        from hangar import resources
        with patch.dict(os.environ,{'FSIM_HANGAR_THREADS':'invalid'}),resources.thread_reservation(2):
            with self.assertRaises(ValueError):resources.threads()
        for cap in (0,129,True,1.5):
            with self.subTest(cap=cap),self.assertRaises(ValueError),resources.thread_reservation(cap):pass

    def test_build_evaluate_and_persistent_probe_all_forward_the_context_cap(self):
        captured=[]
        class Library:
            def mk_build(self, raw, output):
                captured.append(json.loads(raw))
                return 1
            def mk_eval(self, raw, *args):
                captured.append(json.loads(raw))
                return 0
            def mk_scene_new(self, raw, *args):
                captured.append(json.loads(raw))
                return 1
            def mk_scene_free(self, pointer):
                pass
        scene=dict(cell=.02)
        with patch.object(meshkit,'library',return_value=Library()), patch.dict(os.environ,{'FSIM_HANGAR_THREADS':'2'}):
            with self.assertRaises(meshkit.MeshError):
                meshkit.build(scene)
            meshkit.evaluate(scene,[[0.,0.,0.]])
            with meshkit.Probe(scene):
                pass
        self.assertEqual([value['threads'] for value in captured],[2,2,2])
        self.assertEqual(scene,dict(cell=.02))

    def test_scene_caps_cover_every_serialized_native_entry_without_mutating_scene(self):
        scene = dict(cell=.02, children=[], threads=0)
        with patch.dict(os.environ, {'FSIM_HANGAR_THREADS': '2'}):
            self.assertEqual(json.loads(meshkit._dump(scene))['threads'], 2)
            self.assertEqual(json.loads(meshkit._dump(dict(scene, threads=1)))['threads'], 1)
        self.assertEqual(scene['threads'], 0)

    def test_outside_fleet_context_native_defaults_are_unchanged(self):
        with patch.dict(os.environ, {}, clear=True):
            self.assertEqual(json.loads(meshkit._dump(dict(cell=.02))), dict(cell=.02))

    def test_invalid_fleet_thread_declarations_fail_before_native_serialization(self):
        for value in ('0', '-1', '1.5', 'two', '129'):
            with self.subTest(value=value), patch.dict(os.environ, {'FSIM_HANGAR_THREADS': value}):
                with self.assertRaises(ValueError):
                    meshkit._dump(dict(cell=.02))

    def test_performance_default_is_bounded_only_in_fleet_context(self):
        calls = []
        def world(*args, **kwargs):
            calls.append(kwargs)
            raise StopBeforeSdkExecution()
        fake = SimpleNamespace(World=world)
        with patch.dict(sys.modules, {'fsim': fake}):
            with patch.dict(os.environ, {}, clear=True):
                with self.assertRaises(StopBeforeSdkExecution):
                    performance.fly('fixture', 1000., 30., 15.)
            with patch.dict(os.environ, {'FSIM_HANGAR_THREADS': '2'}):
                with self.assertRaises(StopBeforeSdkExecution):
                    performance.fly('fixture', 1000., 30., 15.)
                with self.assertRaises(StopBeforeSdkExecution):
                    performance.climbs('fixture', [], workers=8)
        self.assertEqual([entry['workers'] for entry in calls], [0, 2, 2])




if __name__ == '__main__':
    unittest.main()
