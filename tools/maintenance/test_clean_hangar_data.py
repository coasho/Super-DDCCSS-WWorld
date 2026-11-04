"""Deletion boundaries and raw-evidence retention for Hangar maintenance."""
import json
import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from types import SimpleNamespace


SCRIPT = Path(__file__).with_name('clean_hangar_data.py')


class CleanupTests(unittest.TestCase):
    def test_cleanup_preserves_active_raw_sources_and_transitive_seed(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / 'tools/models/hangar/hangar').mkdir(parents=True)
            base = root / '.local/campaigns/hangar'
            active, old = base / ('a' * 32), base / ('b' * 32)
            source = old / 'cfd/model/training/case-000001/physical-time'
            source.mkdir(parents=True)
            (active / 'cfd').mkdir(parents=True)
            files = [active / 'cfd/current.dat', source / 'restart-out_00001.dat',
                     old / 'cfd/seed.dat', old / 'cfd/junk.dat']
            for path in files:
                path.write_bytes(b'conservative-state')
            duplicate = old / 'cfd/fleet-result.json'
            duplicate.write_text('{"status":"failed","duplicated_leaf_rows":[]}')
            plan = old / 'cfd/plan.json'
            plan.write_text(json.dumps({'inputs_sha256': {str(files[2]): 'pin'}}))
            cold = old / 'cfd/independent/cold.dat'
            cold.parent.mkdir()
            cold.write_bytes(b'independent-cold-source')
            product = root / 'build/hangar/native-responses/qualified.json'
            product.parent.mkdir(parents=True)
            product.write_text(json.dumps({'schema': 'fsim.hangar-native-response-product.v1',
                                           'publication': {'inputs_sha256': {str(cold): 'pin'}}}))
            legacy = root / 'aircraft/model/aero/obsolete.json'
            legacy.parent.mkdir(parents=True)
            legacy.write_text(json.dumps({'schema': 'fsim.store-static-grid.v2',
                                         'old_inputs': {str(files[3]): 'retired-pin'}}))
            selected_raw = old / 'cfd/selected-publication.dat'
            selected_raw.write_bytes(b'full-model-independent-source')
            selected = root / 'aircraft/selected'
            (selected / 'aero').mkdir(parents=True)
            (selected / 'selected.toml').write_text('[cfd]\nmodel="aero/model.json"\n')
            (selected / 'aero/model.json').write_text(json.dumps({
                'parts': [{'artifact': {'path': 'aero/family.json'}}]}))
            (selected / 'aero/family.json').write_text(json.dumps({
                'states': [{'artifact': {'path': 'aero/grid.json'},
                            'publication': {'inputs_sha256': {str(selected_raw): 'pin'}}}]}))
            (selected / 'aero/grid.json').write_text('{}')
            cache = root / '.local/cache/native-execution-data/key'
            cache.mkdir(parents=True)
            (cache / 'data.json').write_text(json.dumps({
                'schema': 'fsim.native-execution-data.v1',
                'identity': {'kind': 'trajectory'},
                'source_plan': {'path': str(plan)},
                'source_row': {'passes': [{'folder': str(source)}]}}))
            authored = root / 'aircraft/model.toml'
            authored.parent.mkdir(exist_ok=True)
            authored.write_text('source = true')
            command = [sys.executable, '-B', str(SCRIPT), '--root', str(root),
                       '--active', active.name, '--report', str(root / 'cleanup.json')]
            subprocess.run(command, check=True, capture_output=True)
            self.assertTrue(all(p.is_file() for p in files))
            subprocess.run(command + ['--apply'], check=True, capture_output=True)
            self.assertTrue(all(p.is_file() for p in files[:3]))
            self.assertFalse(files[3].exists())
            self.assertFalse(duplicate.exists())
            self.assertTrue(authored.is_file())
            self.assertTrue(cold.is_file())
            self.assertTrue(selected_raw.is_file())
            report = json.loads((root / 'cleanup.json').read_text())
            self.assertEqual(report['deleted_files'], 2)
            self.assertTrue(report['retained_hashes_verified'])

    def test_published_point_keeps_retained_numeric_inputs_without_preserving_retired_fields(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp).resolve()
            (root / 'tools/models/hangar/hangar').mkdir(parents=True)
            case = root / '.local/campaigns/hangar' / ('b' * 32) / 'cfd/closed/case-000000'
            case.mkdir(parents=True)
            retained = case / 'restart-out_00301.dat'
            retired = case / 'restart-out_00002.dat'
            history = case / 'solver.log'
            for path in (retained, retired, history):
                path.write_bytes(b'actual numeric data')
            product = root / '.local/cache/native-point-products/key/point.json'
            product.parent.mkdir(parents=True)
            product.write_text(json.dumps({
                'schema': 'fsim.published-native-pressure-point.v1',
                'retained_inputs_sha256': {str(retained): 'pin', str(history): 'pin'},
                'retired_field_sha256': {str(retired): 'historical-only'}}))
            result = subprocess.run([sys.executable, '-B', str(SCRIPT), '--root', str(root),
                                     '--report', str(root / 'cleanup.json'), '--apply'],
                                    capture_output=True, check=True)
            self.assertTrue(retained.is_file())
            self.assertTrue(history.is_file())
            self.assertTrue(product.is_file())
            self.assertFalse(retired.exists())
            report = json.loads((root / 'cleanup.json').read_text())
            self.assertEqual(report['deleted_files'], 1)
            self.assertTrue(report['retained_hashes_verified'])

    def test_rejects_parent_traversal_active_root(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / 'tools/models/hangar/hangar').mkdir(parents=True)
            result = subprocess.run([sys.executable, '-B', str(SCRIPT), '--root', str(root),
                                     '--active', '../outside', '--report', str(root / 'r.json'),
                                     '--apply'], capture_output=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertFalse((root / 'r.json').exists())

    def test_normal_runtime_commands_clean_before_solving_but_not_listing(self):
        path = SCRIPT.parents[1] / 'bootstrap/project.py'
        spec = importlib.util.spec_from_file_location('project_storage_test', path)
        project = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(project)
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            manifest = {'tools': {'python': {'path': sys.executable}, 'cmake': {'path': 'cmake'}}}
            args = SimpleNamespace(mode='hangar', preset='ucrt64-release', arguments=['sample', '--force'])
            commands = project.run_commands(root, manifest, args)
            self.assertIn('clean_hangar_data.py', commands[0][2])
            self.assertEqual(commands[-1][-2:], ['sample', '--force'])
            args.arguments = ['list']
            commands = project.run_commands(root, manifest, args)
            self.assertFalse(any('clean_hangar_data.py' in item for command in commands for item in command))


if __name__ == '__main__':
    unittest.main()
