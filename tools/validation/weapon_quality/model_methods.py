"""Run the mandatory M03 model methods with configured authoring dependencies."""
import argparse
import json
import os
from pathlib import Path
import sys
import unittest

MODULES = (
    'test_breakup', 'test_breakup_caps', 'test_glb_publication', 'test_producer_publication',
    'test_liquid', 'test_liquid_vessels', 'test_liquid_vessel_inputs', 'test_liquid_vessel_packing',
    'test_liquid_pack_equivalence', 'test_liquid_owner_processes', 'test_piece_passivity',
    'test_piece_publication', 'test_runtime_piece_epoch',
    'test_generation_input_epoch', 'test_method_backends',
)


def directory(value):
    path = Path(value)
    if any(part.casefold() == '.codex' for part in path.parts):
        raise ValueError('Excluded model-method input')
    path = path.resolve()
    if any(part.casefold() == '.codex' for part in path.parts) or not path.is_dir():
        raise ValueError('Existing ordinary model-method directory required')
    return path


def method_suite(names, loader=None):
    if not names:
        raise ValueError('Empty required model-method module list')
    loader = loader or unittest.TestLoader()
    suite = unittest.TestSuite()
    for name in names:
        previous_errors = len(loader.errors)
        selected = loader.loadTestsFromName(name)
        if len(loader.errors) != previous_errors:
            raise ValueError('Required model-method module failed to load: '+name+'\n'+loader.errors[-1])
        if not selected.countTestCases():
            raise ValueError('Empty required model-method module: '+name)
        suite.addTest(selected)
    return suite


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', type=directory, required=True)
    parser.add_argument('--build-dir', type=directory, required=True)
    parser.add_argument('--dependency-path', type=directory, required=True)
    args = parser.parse_args()
    source = args.source_root/'tools/models/hangar'
    tests = directory(source/'tests')
    build_python = directory(args.build_dir/'python')
    os.environ.update(TMP='D:/ENV/tmp', TEMP='D:/ENV/tmp', TORCH_EXTENSIONS_DIR='D:/ENV/tmp',
        PYTHONDONTWRITEBYTECODE='1', MPLCONFIGDIR='D:/ENV/cache/fsim-m03/matplotlib',
        XDG_CACHE_HOME='D:/ENV/cache/fsim-m03', MPLBACKEND='Agg',
        FSIM_BIN=str(args.build_dir/'bin'))
    configured_paths = [str(tests), str(source), str(build_python), str(args.dependency_path)]
    sys.path[:0] = configured_paths
    os.environ['PYTHONPATH'] = os.pathsep.join(configured_paths)
    suite = method_suite(MODULES)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    passed = (result.wasSuccessful() and result.testsRun > 0 and not result.skipped
              and not result.expectedFailures and not result.unexpectedSuccesses)
    print(json.dumps(dict(schema='fsim.sim-m03.model-methods.v1', passed=passed,
        modules=list(MODULES), tests=result.testsRun, failures=len(result.failures),
        errors=len(result.errors), skipped=len(result.skipped))), flush=True)
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
