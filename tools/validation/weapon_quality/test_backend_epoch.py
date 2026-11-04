"""Final admission reconstructs its declared method without a prepared shell."""
import copy
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

import backend_epoch as epoch


class BackendEnvironment(unittest.TestCase):
    def setUp(self):
        self.temporary=tempfile.TemporaryDirectory(dir='D:/ENV/tmp')
        self.addCleanup(self.temporary.cleanup)
        self.root=Path(self.temporary.name)/'source';self.root.mkdir()
        package=self.root/'tools/models/hangar/hangar';package.mkdir(parents=True)
        (package/'__init__.py').write_text('',encoding='utf-8')
        self.module=package/'method_backends.py'
        self.module.write_text("import os\ndef snapshot():\n return dict(schema='fsim.hangar.method-backends.v1',observed={k:os.environ.get(k) for k in ('PYTHONPATH','HANGAR_MESHKIT_DIR','MPLCONFIGDIR','XDG_CACHE_HOME','MPLBACKEND','PYTHONNOUSERSITE','PYTHONUTF8','TEMP')})\n",encoding='utf-8')
        self.dependencies=Path(self.temporary.name)/'dependencies';self.dependencies.mkdir()
        (self.root/'build/ucrt64-release/python').mkdir(parents=True)
        self.record=epoch.freeze(self.root,self.dependencies)

    def test_sanitized_parent_can_run_exact_current_source_without_inherited_paths(self):
        foreign=Path(self.temporary.name)/'foreign';(foreign/'hangar').mkdir(parents=True)
        (foreign/'hangar/__init__.py').write_text("raise RuntimeError('Foreign source imported')\n",encoding='utf-8')
        with patch.dict(os.environ,{'PYTHONPATH':str(foreign),'HANGAR_MESHKIT_DIR':str(foreign)},clear=True):
            result=epoch.capture(self.record,self.root)
        observed=result['observed'];expected=epoch.environment(self.record,self.root,parent={})
        for key,value in observed.items():self.assertEqual(value,expected[key])
        self.assertNotIn(str(foreign),observed['PYTHONPATH'])
        self.assertEqual(observed['HANGAR_MESHKIT_DIR'],self.record['meshkit_directory'])

    def test_generation_environment_keeps_caller_tools_after_declared_priorities(self):
        result=epoch.environment(self.record,self.root,parent={'PATH':'caller-git','FSIM_MAPS_DIR':'offline-map'})
        self.assertTrue(result['PATH'].endswith('caller-git'))
        self.assertEqual(result['FSIM_MAPS_DIR'],'offline-map')
        self.assertEqual(result['MPLBACKEND'],'Agg')
        self.assertEqual(result['PYTHONNOUSERSITE'],'1')

    def test_wrong_source_missing_fields_and_redirected_methods_refuse(self):
        for field,value in [('schema','unknown'),('source_root',str(self.dependencies)),
            ('meshkit_directory',str(self.dependencies)),('python_test_path','C:/foreign-packages'),
            ('mplconfigdir','C:/foreign-cache')]:
            record=copy.deepcopy(self.record);record[field]=value
            with self.subTest(field=field),self.assertRaises(ValueError):epoch.environment(record,self.root,parent={})
        record=copy.deepcopy(self.record);record.pop('python_executable')
        with self.assertRaises(ValueError):epoch.capture(record,self.root)
        record=copy.deepcopy(self.record);record['arbitrary']='not-an-environment-channel'
        with self.assertRaises(ValueError):epoch.environment(record,self.root,parent={})

    def test_failed_wrong_schema_and_timed_out_children_cannot_admit(self):
        for source in ("raise RuntimeError('actual refusal')\n", "def snapshot(): return {'schema':'other'}\n"):
            self.module.write_text(source,encoding='utf-8')
            with self.assertRaises(ValueError):epoch.capture(self.record,self.root)
        self.module.write_text("import time\ndef snapshot():\n time.sleep(2)\n return {}\n",encoding='utf-8')
        with self.assertRaises(subprocess.TimeoutExpired):epoch.capture(self.record,self.root,timeout=.1)


if __name__=='__main__':unittest.main(verbosity=2)
