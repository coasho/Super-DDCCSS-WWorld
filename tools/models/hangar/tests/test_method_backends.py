"""Synthetic backend byte/origin admission; no native imports or solvers."""
import copy
import json
import os
from pathlib import Path
import stat
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock

from hangar import method_backends as method


class MethodBackendTests(unittest.TestCase):
    def setUp(self):
        self.real_provider = method._backend_roots
        temporary = Path('D:/ENV/tmp') if os.name == 'nt' else None
        if temporary is not None:
            temporary.mkdir(parents=True, exist_ok=True)
        self.temporary = tempfile.TemporaryDirectory(prefix='fsim-method-backends-', dir=temporary)
        self.root = Path(self.temporary.name)
        self.package = self.root / 'numpy'
        self.package.mkdir()
        (self.package / '__init__.py').write_text('version = 1\n', encoding='utf-8')
        (self.package / 'core.pyd').write_bytes(b'native-core')
        (self.package / 'data.json').write_text('{"asset":1}\n', encoding='utf-8')
        self.libs = self.root / 'numpy.libs'
        self.libs.mkdir()
        (self.libs / 'actual-blas.dll').write_bytes(b'actual-blas')
        self.metadata = self.root / 'numpy-1.dist-info'
        self.metadata.mkdir()
        (self.metadata / 'METADATA').write_text('Name: numpy\nVersion: 1\n', encoding='utf-8')
        self.mesh = self.root / 'hangar_meshkit.dll'
        self.mesh.write_bytes(b'meshkit')
        self.python = self.root / 'python.exe'
        self.python.write_bytes(b'python')
        self.runtime = self.root / 'python313.dll'
        self.runtime.write_bytes(b'python-runtime')
        self.inputs = dict(origins={'numpy': str(self.package / '__init__.py'), 'meshkit': str(self.mesh), 'python': str(self.python)},
            versions={'numpy': {'value': '1', 'basis': 'imported-module'}}, platform_basis={'system': 'synthetic'},
            native_names=['python313.dll'], required_python_runtime='python313.dll', roots=[
                dict(role='package:numpy', path=str(self.package), kind='tree', required=True),
                dict(role='adjacent:numpy.libs', path=str(self.libs), kind='tree', required=False),
                dict(role='metadata:numpy', path=str(self.metadata), kind='tree', required=True),
                dict(role='native:meshkit', path=str(self.mesh), kind='file', required=True),
                dict(role='runtime:python-executable', path=str(self.python), kind='file', required=True),
                dict(role='adjacent:optional', path=str(self.root / 'optional.libs'), kind='tree', required=False)])
        self.native = [dict(name='python313.dll', state='loaded-file', path=str(self.runtime)),
                       dict(name='kernel32.dll', state='os-system', path='C:/Windows/System32/kernel32.dll')]
        self.provider = mock.patch.object(method, '_backend_roots', side_effect=lambda: copy.deepcopy(self.inputs))
        self.dependencies = mock.patch.object(method, '_native_dependencies', side_effect=lambda names: copy.deepcopy(self.native))
        self.provider.start()
        self.dependencies.start()
        self.addCleanup(self.dependencies.stop)
        self.addCleanup(self.provider.stop)
        self.addCleanup(self.temporary.cleanup)

    def test_deterministic_complete_selected_payload_and_no_timing(self):
        first = method.snapshot()
        self.assertEqual(first, method.snapshot())
        self.assertTrue(method.verify(first))
        self.assertEqual(first['file_count'], 8)
        self.assertEqual(len(first['sha256']), 64)
        self.assertNotIn('seconds', json.dumps(first))
        self.assertNotIn('time', first)
        files = {file['relative'] for root in first['roots'] for file in root['files']}
        self.assertTrue({'__init__.py', 'core.pyd', 'data.json', 'actual-blas.dll', 'METADATA'}.issubset(files))
        self.assertNotIn('file', first['native_dependencies'][1])

    def test_each_changed_actual_package_asset_metadata_and_native_byte_invalidates(self):
        for path in (self.package / '__init__.py', self.package / 'core.pyd', self.package / 'data.json',
                     self.libs / 'actual-blas.dll', self.metadata / 'METADATA', self.mesh, self.python, self.runtime):
            with self.subTest(path=path.name):
                before = method.snapshot()
                original = path.read_bytes()
                path.write_bytes(original + b' changed')
                self.assertFalse(method.verify(before))
                path.write_bytes(original)
                self.assertTrue(method.verify(before))

    def test_added_removed_and_optional_roots_are_part_of_current_byte_set(self):
        before = method.snapshot()
        extra = self.package / 'new-kernel.pyd'
        extra.write_bytes(b'new native file')
        self.assertFalse(method.verify(before))
        extra.unlink()
        self.assertTrue(method.verify(before))
        retained = (self.package / 'data.json').read_bytes()
        (self.package / 'data.json').unlink()
        self.assertFalse(method.verify(before))
        (self.package / 'data.json').write_bytes(retained)
        self.assertTrue(method.verify(before))
        optional = self.root / 'optional.libs'
        optional.mkdir()
        (optional / 'added.dll').write_bytes(b'new dependency')
        self.assertFalse(method.verify(before))

    def test_origin_redirect_rejects_even_identical_payload(self):
        before = method.snapshot()
        redirected = self.root / 'redirected.py'
        redirected.write_bytes((self.package / '__init__.py').read_bytes())
        self.inputs['origins']['numpy'] = str(redirected)
        with self.assertRaises(method.BackendClosureError):
            method.snapshot()
        self.assertFalse(method.verify(before))
        self.inputs['origins']['numpy'] = str(self.package / '__init__.py')
        other = self.root / 'other-numpy'
        other.mkdir()
        for source in self.package.iterdir():
            (other / source.name).write_bytes(source.read_bytes())
        self.inputs['origins']['numpy'] = str(other / '__init__.py')
        next(row for row in self.inputs['roots'] if row['role'] == 'package:numpy')['path'] = str(other)
        self.assertFalse(method.verify(before))

    def test_excluded_names_are_pruned_before_any_metadata_without_real_excluded_dirs(self):
        before = method.snapshot()

        class UntouchableEntry:
            def __init__(self, name):
                self.name = name

            def __getattr__(self, name):
                raise AssertionError('Excluded entry metadata must never be read: ' + name)

        scan = os.scandir

        class Entries:
            def __init__(self, values):
                self.values = values

            def __enter__(self):
                return iter(self.values)

            def __exit__(self, *args):
                return False

        def pruned_scan(path):
            if Path(path) == self.package:
                with scan(path) as stream:
                    actual = list(stream)
                # Virtual names only: this test never creates, opens, stats or
                # traverses a real excluded directory.
                return Entries(actual + [UntouchableEntry('.CoDeX'), UntouchableEntry('.git'),
                                         UntouchableEntry('__pycache__')])
            return scan(path)

        with mock.patch.object(method.os, 'scandir', side_effect=pruned_scan):
            self.assertEqual(before, method.snapshot())
        (self.package / 'discard.pyc').write_bytes(b'ephemeral')
        self.assertEqual(before, method.snapshot())
        with mock.patch.object(method.os.path, 'abspath', side_effect=AssertionError('Excluded input must not resolve')):
            with self.assertRaises(method.BackendClosureError):
                method._path(self.root / '.CoDeX' / 'never-read')

        class UntouchableMetadata:
            _path = self.root / '.CoDeX' / 'never-read.dist-info'

            @property
            def metadata(self):
                raise AssertionError('Excluded distribution metadata must never be read')

        with mock.patch.object(method.importlib.metadata, 'distributions', return_value=[UntouchableMetadata()]), \
             mock.patch.object(method.os.path, 'abspath', side_effect=AssertionError('Excluded metadata must not resolve')):
            self.assertEqual(method._distribution_metadata('numpy', self.root), (None, []))

    def test_reparse_and_symbolic_payloads_refuse_before_read(self):
        before = method.snapshot()
        lstat = os.lstat
        selected = self.package / 'core.pyd'

        def junction(path):
            value = lstat(path)
            if Path(path) == selected:
                return SimpleNamespace(st_mode=value.st_mode, st_file_attributes=0x400)
            return value

        with mock.patch.object(method.os, 'lstat', side_effect=junction):
            self.assertFalse(method.verify(before))
            with self.assertRaises(method.BackendClosureError):
                method.snapshot()

        def symlink(path):
            value = lstat(path)
            return SimpleNamespace(st_mode=stat.S_IFLNK, st_file_attributes=0) if Path(path) == selected else value

        with mock.patch.object(method.os, 'lstat', side_effect=symlink):
            with self.assertRaises(method.BackendClosureError):
                method.snapshot()

    def test_required_absence_and_budget_exhaustion_refuse_instead_of_truncating(self):
        before = method.snapshot()
        for name, value in (('MAX_FILES', 1), ('MAX_ENTRIES', 1), ('MAX_TOTAL_BYTES', 1), ('MAX_FILE_BYTES', 1)):
            with self.subTest(bound=name), mock.patch.object(method, name, value):
                with self.assertRaises(method.BackendClosureError):
                    method.snapshot()
                self.assertFalse(method.verify(before))
        self.mesh.unlink()
        with self.assertRaises(method.BackendClosureError):
            method.snapshot()

    def test_loaded_runtime_redirect_and_required_unavailability_invalidate(self):
        before = method.snapshot()
        other = self.root / 'other-python313.dll'
        other.write_bytes(self.runtime.read_bytes())
        self.native[0]['path'] = str(other)
        self.assertFalse(method.verify(before))
        self.native[0] = dict(name='python313.dll', state='not-loaded')
        self.assertFalse(method.verify(before))
        with self.assertRaises(method.BackendClosureError):
            method.snapshot()

    def test_receipt_cannot_request_imports_or_roots(self):
        record = method.snapshot()
        record['roots'].append(dict(role='untrusted', path='untrusted-package', kind='tree', present=True, files=[]))
        self.assertFalse(method.verify(record))
        self.assertFalse(method.verify(None))

    def test_foreign_distribution_metadata_cannot_rebind_actual_origin(self):
        distribution = SimpleNamespace(metadata={'Name': 'numpy'}, locate_file=lambda relative: self.root / 'foreign', files=[], version='1')
        with mock.patch.object(method.importlib.metadata, 'distributions', return_value=[distribution]) as query:
            with self.assertRaises(method.BackendClosureError):
                method._distribution_metadata('numpy', self.root)
            query.assert_called_once_with(path=[str(self.root)])
        with mock.patch.object(method.importlib.metadata, 'distributions', return_value=[]):
            self.assertEqual(method._distribution_metadata('numpy', self.root), (None, []))
        # Missing RECORD cannot hide a present real metadata directory. Its
        # actual root is inventoried without trusting a payload file list.
        matched = SimpleNamespace(locate_file=lambda relative: self.root / relative,
                                  metadata={'Name': 'NumPy'}, files=None, version='1', _path=self.metadata)
        with mock.patch.object(method.importlib.metadata, 'distributions', return_value=[matched]):
            self.assertEqual(method._distribution_metadata('numpy', self.root), ('1', [self.metadata]))

    def test_missing_overlay_metadata_ignores_unrelated_global_distribution(self):
        # A global NumPy installation must neither replace the imported overlay
        # origin nor manufacture a false redirect refusal. No live query runs.
        global_record = SimpleNamespace(metadata={'Name': 'numpy'},
                                        locate_file=lambda relative: self.root / 'unrelated-global',
                                        files=[], version='999')
        with mock.patch.object(method.importlib.metadata, 'distribution', return_value=global_record) as global_query, \
             mock.patch.object(method.importlib.metadata, 'distributions', return_value=[]) as local_query:
            self.assertEqual(method._distribution_metadata('numpy', self.root), (None, []))
            global_query.assert_not_called()
            local_query.assert_called_once_with(path=[str(self.root)])

    def test_unconfigured_plot_cache_refuses_before_any_live_import(self):
        with mock.patch.object(method.os, 'name', 'nt'), mock.patch.dict(os.environ), \
             mock.patch.object(method.importlib, 'import_module', side_effect=AssertionError('No live backend import allowed')):
            os.environ.pop('MPLCONFIGDIR', None)
            os.environ.pop('XDG_CACHE_HOME', None)
            with self.assertRaises(method.BackendClosureError):
                self.real_provider()


if __name__ == '__main__':
    unittest.main()
