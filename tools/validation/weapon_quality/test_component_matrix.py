"""Independent refusal controls for component-evidence admission, not physics."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock
from types import SimpleNamespace

import component_matrix as matrix


class ComponentMatrixTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(dir='D:/ENV/tmp', prefix='m03-component-admission-')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root/'source.cpp').write_text('synthetic admission input; not production physics', encoding='utf-8')
        (self.root/'raw.log').write_text('Synthetic evidence-format fixture', encoding='utf-8')
        (self.root/'image.png').write_bytes(b'synthetic image binding fixture')
        self.source = matrix.binding(self.root/'source.cpp', self.root)
        self.cells = []
        for i in range(24):
            for component in matrix.COMPONENTS:
                model = 'fixture-'+str(i)
                self.cells.append(dict(model=model, native_model='jsbsim:'+model, component=component,
                    implementation_id='equipment.'+model+'.'+component,
                    mechanism_id='fixture.mechanism.'+component, card_sha256='a'*64,
                    parameter_sha256=matrix.digest([model, component]), sources=[self.source]))
        self.artifacts = {}
        for gate in matrix.GATES:
            rows = []
            for cell in self.cells:
                row = {key: cell[key] for key in ('model', 'component', 'implementation_id', 'mechanism_id', 'card_sha256', 'parameter_sha256')}
                row.update(passed=True,
                    theory=dict(equations='Independent fixture identity y=x', limits='Synthetic admission controls only',
                        delta='No production numerical change', primary_sources=[dict(title='Fixture source',
                            url='https://example.invalid/fixture', locator='equation1', scope='Synthetic format only')]),
                    method=dict(case='Independent scalar identity', tests=1, assertions=1,
                        residuals=[dict(name='identity', actual=1., expected=1., tolerance=0.)]),
                    shared_equivalence=dict(id='fixture-'+cell['component'], mechanism_id=cell['mechanism_id'],
                        models=['fixture-'+str(i) for i in range(24)], scope='Same synthetic equation', limits='No equipment equivalence'),
                    native=dict(case='Synthetic native-format control', tests=1, assertions=1,
                        native_model=cell['native_model'], per_model_execution=True,
                        coupled_components=list(matrix.COMPONENTS), presets=['ucrt64-headless', 'ucrt64-release']),
                    public_installed=dict(case='Synthetic installed-format control', tests=1, assertions=1,
                        native_model=cell['native_model'], per_model_execution=True,
                        languages=['cpp', 'c99', 'python'], loaded_identity_verified=True),
                    visual=dict(case='Synthetic visual-format control', tests=1, assertions=1,
                        native_model=cell['native_model'], reviewed=True, fullscreen=True, rate=1,
                        enabled_effects=['earth', 'terrain', 'clouds', 'sunlight', 'shadows', 'exhaust', 'contrails', 'native_effects'],
                        image=matrix.binding(self.root/'image.png', self.root)))
                rows.append(row)
            doc = dict(schema=matrix.SCHEMAS[gate], passed=True, models=rows,
                       sources=[self.source], raw=[matrix.binding(self.root/'raw.log', self.root)])
            self.artifacts[gate] = doc
            self.write(gate)
        self.manifest = dict(schema='fsim.sim-m03.component-evidence.v1', cells=[
            dict(model=c['model'], component=c['component'], evidence={
                gate: dict(artifact=matrix.binding(self.root/(gate+'.json'), self.root), row=i)
                for gate in matrix.GATES}) for i, c in enumerate(self.cells)],
            calibration=[dict(model='fixture-'+str(i), status='uncalibrated', reason='No equipment experiment') for i in range(24)])

    def write(self, gate):
        path = self.root/(gate+'.json')
        path.write_text(json.dumps(self.artifacts[gate], allow_nan=False), encoding='utf-8')
        if hasattr(self, 'manifest'):
            binding = matrix.binding(path, self.root)
            for row in self.manifest['cells']:
                row['evidence'][gate]['artifact'] = binding

    def reject_mutation(self, gate, change):
        original = copy.deepcopy(self.artifacts[gate])
        change(self.artifacts[gate]['models'][0])
        self.write(gate)
        try:
            with self.assertRaises((ValueError, KeyError, TypeError)):
                matrix.admit_cells(self.cells, self.manifest, self.root)
        finally:
            self.artifacts[gate] = original
            self.write(gate)

    def test_all216_cells_require_their_own_matching_identity_and_five_gates(self):
        admitted = matrix.admit_cells(self.cells, self.manifest, self.root)
        self.assertEqual(len(admitted), 216)
        self.assertTrue(all(r['passed'] and len(r['gates']) == 5 for r in admitted))
        ledger = matrix.calibration_ledger(self.cells, self.manifest['calibration'], self.root)
        self.assertEqual(len(ledger), 24)
        self.assertTrue(all(r['status'] == 'uncalibrated' for r in ledger))

    def test_omission_duplicate_foreign_or_extra_component_rows_refuse(self):
        for change in (lambda rows: rows.pop(), lambda rows: rows.append(rows[0]),
                       lambda rows: rows[0].update(model='unknown'),
                       lambda rows: rows[0].update(component='link')):
            with self.subTest(change=change):
                doc = copy.deepcopy(self.manifest);change(doc['cells'])
                with self.assertRaises(ValueError):
                    matrix.admit_cells(self.cells, doc, self.root)

    def test_missing_unknown_or_false_gate_cannot_complete(self):
        for change in (lambda gates: gates.pop('native'), lambda gates: gates.update(invented=gates['native']),
                       lambda gates: gates.update(native=False)):
            doc = copy.deepcopy(self.manifest);change(doc['cells'][0]['evidence'])
            with self.assertRaises((ValueError, TypeError)):
                matrix.admit_cells(self.cells, doc, self.root)

    def test_wrong_model_component_selector_mechanism_card_and_parameter_epoch_refuse(self):
        for key in ('model', 'component', 'implementation_id', 'mechanism_id', 'card_sha256', 'parameter_sha256'):
            with self.subTest(key=key):
                self.reject_mutation('native', lambda row: row.update({key: 'foreign'}))

    def test_artifact_schema_false_pass_and_wrong_row_refuse(self):
        original = self.artifacts['native']['schema']
        self.artifacts['native']['schema'] = 'unrelated.aggregate-pass.v1';self.write('native')
        with self.assertRaises(ValueError):matrix.admit_cells(self.cells, self.manifest, self.root)
        self.artifacts['native']['schema'] = original;self.artifacts['native']['passed'] = False;self.write('native')
        with self.assertRaises(ValueError):matrix.admit_cells(self.cells, self.manifest, self.root)
        self.artifacts['native']['passed'] = True;self.write('native')
        self.manifest['cells'][0]['evidence']['native']['row'] = 1
        with self.assertRaises(ValueError):matrix.admit_cells(self.cells, self.manifest, self.root)

    def test_boolean_row_selector_and_duplicate_proof_identity_refuse(self):
        self.manifest['cells'][0]['evidence']['native']['row'] = False
        with self.assertRaises(ValueError):matrix.admit_cells(self.cells, self.manifest, self.root)
        self.manifest['cells'][0]['evidence']['native']['row'] = 0
        self.artifacts['native']['models'].append(copy.deepcopy(self.artifacts['native']['models'][0]));self.write('native')
        with self.assertRaises(ValueError):matrix.admit_cells(self.cells, self.manifest, self.root)

    def test_changed_source_raw_or_evidence_bytes_refuse_even_old_pass_remains(self):
        for name in ('source.cpp', 'raw.log', 'native.json'):
            with self.subTest(name=name):
                path = self.root/name;before = path.read_bytes();path.write_bytes(before+b'changed')
                try:
                    with self.assertRaises(ValueError):matrix.admit_cells(self.cells, self.manifest, self.root)
                finally:path.write_bytes(before)

    def test_missing_or_duplicate_sources_and_raw_outputs_refuse(self):
        for key, replacement in (('sources', []), ('sources', [self.source, self.source]), ('raw', [])):
            original = self.artifacts['method'][key];self.artifacts['method'][key] = replacement;self.write('method')
            try:
                with self.assertRaises(ValueError):matrix.admit_cells(self.cells, self.manifest, self.root)
            finally:self.artifacts['method'][key] = original;self.write('method')

    def test_execution_zero_skip_expected_failure_and_real_failure_refuse(self):
        for gate in ('method', 'native', 'public_installed', 'visual'):
            for key, value in (('tests', 0), ('assertions', 0), ('tests', True), ('skipped', 1),
                               ('expected_failures', 1), ('failures', 1), ('errors', 1)):
                with self.subTest(gate=gate, key=key):
                    self.reject_mutation(gate, lambda row: row[gate].update({key: value}))

    def test_method_residuals_cannot_be_omitted_failed_or_tolerance_negative(self):
        self.reject_mutation('method', lambda row: row['method'].update(residuals=[]))
        self.reject_mutation('method', lambda row: row['method']['residuals'][0].update(actual=2))
        self.reject_mutation('method', lambda row: row['method']['residuals'][0].update(tolerance=-1))

    def test_shared_method_is_explicit_and_cannot_replace_other_mechanisms_or_native_execution(self):
        for change in (lambda row: row.pop('shared_equivalence'),
                       lambda row: row['shared_equivalence'].update(mechanism_id='foreign'),
                       lambda row: row['shared_equivalence'].update(models=['unknown']),
                       lambda row: row['shared_equivalence'].update(models=['fixture-0', 'fixture-0']),
                       lambda row: row['shared_equivalence'].update(limits='')):
            self.reject_mutation('method', change)
        self.reject_mutation('native', lambda row: row['native'].update(per_model_execution=False))
        self.reject_mutation('native', lambda row: row['native'].update(coupled_components=['seeker']))
        self.reject_mutation('native', lambda row: row['native'].update(coupled_components=list(matrix.COMPONENTS)+['seeker']))
        self.reject_mutation('native', lambda row: row['native'].update(presets=['ucrt64-release']))

    def test_theory_needs_exact_primary_source_scope_and_equations(self):
        for change in (lambda row: row['theory'].update(equations=''),
                       lambda row: row['theory'].update(delta=''),
                       lambda row: row['theory'].update(primary_sources=[]),
                       lambda row: row['theory']['primary_sources'][0].update(locator='')):
            self.reject_mutation('theory', change)

    def test_public_and_visual_provenance_cannot_be_option_only_or_incomplete(self):
        self.reject_mutation('public_installed', lambda row: row['public_installed'].update(languages=['python']))
        self.reject_mutation('public_installed', lambda row: row['public_installed'].update(loaded_identity_verified=False))
        for change in (lambda row: row['visual'].update(reviewed=False), lambda row: row['visual'].update(fullscreen=False),
                       lambda row: row['visual'].update(rate=2), lambda row: row['visual'].update(rate=True),
                       lambda row: row['visual'].update(enabled_effects=['earth'])):
            self.reject_mutation('visual', change)

    def test_partial_progress_publishes_only_verified_item_without_final_flag(self):
        first = self.manifest['cells'][0]
        partial = dict(schema=self.manifest['schema'], cells=[dict(model=first['model'], component=first['component'],
                      evidence={'method': first['evidence']['method']})])
        result = matrix.progress_cells(self.cells, partial, self.root)
        self.assertEqual(sum(r['gates']['method']['passed'] for r in result), 1)
        self.assertEqual(sum(r['passed'] for r in result), 0)
        self.assertFalse(result[0]['gates']['native']['passed'])

    def test_calibration_requires_all24_records_and_independent_equipment_holdout(self):
        missing = self.manifest['calibration'][:-1]
        with self.assertRaises(ValueError):matrix.calibration_ledger(self.cells, missing, self.root)
        fake = copy.deepcopy(self.manifest['calibration']);fake[0] = dict(model='fixture-0', status='calibrated',
            evidence=matrix.binding(self.root/'method.json', self.root))
        with self.assertRaises(ValueError):matrix.calibration_ledger(self.cells, fake, self.root)

    def test_exact_parameter_payload_preserves_basis_units_and_uncertainty(self):
        card = dict(parameters={'gain': dict(value=3.5, unit='1', basis='assumed', uncertainty=1., source='generic-law')},
                    component_models=dict(components={'guidance': dict(implementation_id='equipment.test.guidance',
                        basis='assumed', parameter_refs={'navigation_gain': 'gain'})}))
        payload = matrix.parameter_payload(card, 'guidance')
        self.assertEqual(payload['parameters']['navigation_gain'], card['parameters']['gain'])
        before = matrix.parameter_hash(card, 'guidance');card['parameters']['gain']['basis'] = 'sourced'
        self.assertNotEqual(before, matrix.parameter_hash(card, 'guidance'))
        card['parameters']['gain']['value'] = float('nan')
        with self.assertRaises(ValueError):matrix.parameter_hash(card, 'guidance')

    def test_clean_export_reads_only_the_explicit_git_baseline_source(self):
        git_root = Path('D:/Projects/Cpp/Super-DDCCSS-WWorld')
        baseline = 'e7ea9abe9dd9a148a42b1f6174353115bc9c8f5f'
        with mock.patch.object(matrix.subprocess, 'run', return_value=SimpleNamespace(returncode=0, stdout=b'original')) as run:
            actual = matrix.head_bytes('assets/stores/aim-120c.json', self.root, baseline, git_root)
        self.assertEqual(actual, b'original')
        self.assertEqual(run.call_args.kwargs['cwd'], git_root)
        self.assertEqual(run.call_args.args[0], ['git', 'show', baseline+':assets/stores/aim-120c.json'])

    def test_native_scalar_values_are_retained_and_large_tables_reference_actual_bytes(self):
        text = '\n'.join('1 2 3 4 5' for _ in range(300))
        path = self.root/'native.xml'
        path.write_text('<fdm_config><aerodynamics><gain unit="1">3.5</gain><tableData>'+text+
                        '</tableData></aerodynamics></fdm_config>', encoding='utf-8')
        values = matrix.xml_values(path, 'aerodynamics')[0]['children']
        self.assertEqual(values[0]['text'], '3.5')
        self.assertIsNone(values[1]['text'])
        self.assertEqual(values[1]['external_text']['sha256'], matrix.hashlib.sha256(text.encode('utf-8')).hexdigest())
        self.assertEqual(values[1]['external_text']['lines'], 300)


if __name__ == '__main__':
    unittest.main()
