"""The fleet uses the ordinary design CLI and propagates child failures."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from hangar import fleet_build
from tests.paths import TEMP_ROOT


class PublicFleetBuild(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(dir=TEMP_ROOT)
        self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name).resolve()
        self.paths={}
        for name in ('plane','store','rotor'):
            path=self.root/'aircraft'/name/(name+'.toml')
            path.parent.mkdir(parents=True);path.write_text('[aircraft]\nname="'+name+'"\n',encoding='utf-8')
            self.paths[name]=str(path)

    def run_fleet(self, child, **options):
        args=dict(jobs=8,memory_limit_mib=2048,out=self.root/'report',runtime_threads=2,
                  runtime_memory_mib=1024,stages=['aero','build'])
        args.update(options)
        with patch.object(fleet_build.pipeline,'repo_root',return_value=str(self.root)), \
             patch.object(fleet_build.pipeline,'find_design',side_effect=self.paths.__getitem__), \
             patch.object(fleet_build.subprocess,'run',side_effect=child):
            return fleet_build.build(list(self.paths),**args)

    def test_normal_entrypoint_resources_and_complete_roster(self):
        calls=[]
        def child(command,**kwargs):
            calls.append((command,kwargs));return SimpleNamespace(returncode=0)
        result=self.run_fleet(child)
        self.assertEqual(result['status'],'passed');self.assertEqual(result['workers'],2)
        self.assertEqual(set(result['models']),set(self.paths))
        for command,kw in calls:
            self.assertEqual(command[1:5],['-u','-B','-m','hangar'])
            self.assertEqual(command[-2:],['aero','build'])
            self.assertEqual(kw['env']['OPENBLAS_NUM_THREADS'],'2')
            self.assertEqual(kw['env']['FSIM_HANGAR_THREADS'],'2')
        saved=json.loads((self.root/'report/fleet-build.json').read_text(encoding='utf-8'))
        self.assertEqual(saved['status'],'passed')

    def test_nonzero_child_and_exception_cannot_be_replaced_by_cached_success(self):
        def child(command,**kw):
            if Path(command[5]).stem=='store':return SimpleNamespace(returncode=7)
            if Path(command[5]).stem=='rotor':raise OSError('synthetic launch error')
            return SimpleNamespace(returncode=0)
        result=self.run_fleet(child)
        self.assertEqual(result['status'],'failed')
        self.assertEqual(result['models']['store']['exit_code'],7)
        self.assertIn('launch error',result['models']['rotor']['reason'])
        self.assertEqual(result['models']['plane']['status'],'passed')

    def test_invalid_roster_and_resource_limits_fail_before_launch(self):
        with self.assertRaises(ValueError):self.run_fleet(lambda *a,**k:self.fail('launched'),memory_limit_mib=100)
        with self.assertRaises(ValueError):fleet_build.build(['same','same'],jobs=1,memory_limit_mib=2048,out=self.root/'out')


if __name__=='__main__':unittest.main()
