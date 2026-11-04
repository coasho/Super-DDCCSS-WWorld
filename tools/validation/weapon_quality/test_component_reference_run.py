"""Synthetic negative controls; never treated as native component evidence."""
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import component_reference_run as capture


class CaptureTests(unittest.TestCase):
    def test_ambient_powered_output_cannot_redirect_owned_capture(self):
        output=Path('D:/ENV/tmp/owned-reference-run')
        with patch.dict(os.environ, FSIM_COMPONENT_PROPULSION_JSON='D:/ENV/tmp/foreign.json'):
            _, supplied, captures=capture.capture_environment('kinematics', output, output/'epoch.json')
            native_environment=os.environ.copy();native_environment.update(supplied)
            self.assertEqual(native_environment['FSIM_COMPONENT_PROPULSION_JSON'], str(captures[1]))
            self.assertEqual(captures[1].parent, output)

    def fixture(self):
        root = Path(tempfile.mkdtemp(prefix='m03-reference-capture-', dir='D:/ENV/tmp'))
        exe, epoch, native = root/'fixture.bin', root/'epoch.json', root/'native.json'
        exe.write_bytes(b'Synthetic executable identity only; never run')
        epoch.write_text(json.dumps(dict(sources=[dict(path='assets/stores/model'+str(i)+'.json', sha256='0'*64)
                                               for i in range(24)])), encoding='utf-8')
        value = dict(schema='fsim.sim-m03.component-effects-native.v1', runtime_modules=[],
                     input_epoch_sha256=capture.sha(epoch), executable=capture.binding(exe),
                     models=[dict(model='model'+str(i)) for i in range(24)])
        native.write_text(json.dumps(value), encoding='utf-8')
        return root, exe, epoch, native, value

    def test_capture_side_epoch_and_actual_executable_identity_are_required(self):
        _, exe, epoch, native, value = self.fixture()
        prepared = capture.staged_runtime(exe, 'effects')
        self.assertEqual(len(capture.verify_capture(native, epoch, exe, prepared)['models']), 24)
        for mutation in ('epoch', 'executable_hash', 'executable_path', 'missing', 'duplicate', 'foreign'):
            current = json.loads(json.dumps(value))
            if mutation == 'epoch': current['input_epoch_sha256'] = '0'*64
            elif mutation == 'executable_hash': current['executable']['sha256'] = '0'*64
            elif mutation == 'executable_path': current['executable']['path'] = str(exe.with_name('different.bin'))
            elif mutation == 'missing': current['models'].pop()
            elif mutation == 'duplicate': current['models'][-1] = current['models'][0]
            else: current['models'][-1]['model'] = 'foreign'
            native.write_text(json.dumps(current), encoding='utf-8')
            with self.subTest(mutation=mutation), self.assertRaises((ValueError, FileNotFoundError)):
                capture.verify_capture(native, epoch, exe, prepared)

    def test_actual_loaded_module_must_match_prepared_path_and_bytes(self):
        root, exe, _, _, native = self.fixture()
        module = root/'libJSBSim.dll';module.write_bytes(b'synthetic staged JSBSim, never loaded')
        prepared = capture.staged_runtime(exe, 'kinematics')
        native['runtime_modules'] = [capture.binding(module)]
        self.assertEqual(set(capture.observed_modules(native, prepared, exe, True)), {module.resolve()})
        for records in ([], native['runtime_modules']*2, [dict(path=str(root/'missing.dll'), sha256='0'*64)]):
            changed = dict(native, runtime_modules=records)
            with self.subTest(records=records), self.assertRaises((ValueError, FileNotFoundError)):
                capture.observed_modules(changed, prepared, exe, True)
        checksum = capture.sha(exe)
        module.write_bytes(b'changed dynamic producer, same executable')
        self.assertEqual(capture.sha(exe), checksum)
        with self.assertRaises(ValueError):capture.observed_modules(native, prepared, exe, True)

    def test_unused_staged_library_is_not_invented_as_loaded_code(self):
        root, exe, _, _, native = self.fixture()
        unused = root/'libfsim.dll';unused.write_bytes(b'synthetic unused staged DLL')
        prepared = capture.staged_runtime(exe, 'effects')
        run = dict(staged_runtime=prepared, runtime=[capture.binding(exe)])
        capture.verify_execution_runtime(run, [native], exe, 'effects')
        run['runtime'].append(capture.binding(unused))
        with self.assertRaises(ValueError):capture.verify_execution_runtime(run, [native], exe, 'effects')

    def test_powered_capture_requires_jsbsim_and_merged_real_module_inventory(self):
        root, exe, _, _, base = self.fixture()
        with self.assertRaises(ValueError):capture.staged_runtime(exe, 'kinematics')
        module = root/'libJSBSim.dll';module.write_bytes(b'synthetic powered DLL')
        prepared = capture.staged_runtime(exe, 'kinematics')
        base = dict(base, schema='fsim.sim-m03.component-native.v1')
        powered = dict(base, schema='fsim.sim-m03.propulsion-reference-native.v1',
                       runtime_modules=[capture.binding(module)])
        run = dict(staged_runtime=prepared, runtime=[capture.binding(exe), capture.binding(module)])
        capture.verify_execution_runtime(run, [base, powered], exe, 'kinematics')
        with self.assertRaises(ValueError):capture.verify_execution_runtime(run, [base, dict(powered, runtime_modules=[])], exe, 'kinematics')
        # Refreshing prepared bytes cannot reauthorize an older observed DLL.
        module.write_bytes(b'new staged DLL bytes')
        run['staged_runtime'] = capture.staged_runtime(exe, 'kinematics')
        with self.assertRaises(ValueError):capture.verify_execution_runtime(run, [base, powered], exe, 'kinematics')

    def test_real_positive_catch_counters_and_exact_case_are_required(self):
        root, *_ = self.fixture()
        report = root/'catch.xml'
        name = next(iter(capture.CASES['effects']))
        def write(**changes):
            fields = dict(tests='818', failures='0', errors='0', skipped='0', status='run', name=name)
            fields.update(changes)
            report.write_text('<testsuites><testsuite tests="{tests}" failures="{failures}" errors="{errors}" skipped="{skipped}">'
                '<testcase name="{name}" status="{status}"/></testsuite></testsuites>'.format(**fields), encoding='utf-8')
        write();self.assertEqual(capture.catch_counts(report, 'effects')['assertions'], 818)
        for mutation in (dict(tests='0'), dict(failures='1'), dict(errors='1'), dict(skipped='1'),
                         dict(status='notrun'), dict(name=name+' unrelated')):
            write(**mutation)
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                capture.catch_counts(report, 'effects')
        report.write_text('<testcase name="'+name+'"/>', encoding='utf-8')
        with self.assertRaises(ValueError): capture.catch_counts(report, 'effects')

    def test_excluded_path_is_rejected_before_resolution_or_io(self):
        with patch.object(Path, 'resolve', side_effect=AssertionError('Resolution must not occur')):
            with self.assertRaises(ValueError): capture.path('D:/fixture/.CoDeX/private.json')

    def test_foreign_runtime_root_is_refused_before_hashing(self):
        with patch.object(capture, 'sha', side_effect=AssertionError('Foreign bytes must not be read')):
            with self.assertRaisesRegex(ValueError, 'Foreign runtime evidence root'):
                capture.bindings([dict(path='C:/foreign-runtime/libJSBSim.dll', sha256='0'*64)])


if __name__ == '__main__':
    unittest.main(verbosity=2)
