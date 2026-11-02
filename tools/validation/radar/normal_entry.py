"""One actual radar/link fusion case through the normal selected SDK launcher."""
import argparse
import importlib.util
import json
from pathlib import Path
import sys
import unittest

import fsim
from provenance import loaded_modules

ROOT=Path(__file__).resolve().parents[3]


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise ValueError('Preserve prior normal-entry receipts')
    spec=importlib.util.spec_from_file_location('normal_fusion',ROOT/'tests/compatibility/python/test_fusion.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    result=unittest.TextTestRunner().run(unittest.defaultTestLoader.loadTestsFromModule(module))
    record=dict(schema='fsim.sim-m01.normal-entry.v2',python=sys.executable,package=fsim.__file__,
        native=fsim._native.__file__,loaded_modules=loaded_modules(ROOT/'build/ucrt64-release'),
        tests_run=result.testsRun,passed=result.wasSuccessful() and result.testsRun>0,
        scope='Normal SDK launcher; actual sensor/link fusion, not Viewer presentation or equipment calibration')
    args.output.write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(record));raise SystemExit(not record['passed'])
