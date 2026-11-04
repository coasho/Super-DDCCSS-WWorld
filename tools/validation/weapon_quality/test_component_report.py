"""Synthetic final-report refusal controls; never native physics evidence."""
import json
import unittest
from unittest import mock

import component_matrix as matrix
import test_component_matrix as matrix_controls
import report


class ComponentReportTests(unittest.TestCase):
    def setUp(self):
        self.fixture=matrix_controls.ComponentMatrixTest('runTest')
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.path=self.fixture.root/'manifest.json'

    def join(self):
        self.path.write_text(json.dumps(self.fixture.manifest),encoding='utf-8')
        with mock.patch.object(report,'derive_cells',return_value=self.fixture.cells):
            return report.join_component_closure({},self.path,self.fixture.root)

    def test_final_report_joins_all_five_current_gates_and_separate_calibration(self):
        value=self.join()
        self.assertEqual(value['cells'],216)
        self.assertEqual(value['passed_gates'],dict.fromkeys(matrix.GATES,216))
        self.assertEqual(value['calibration']['required_records'],24)
        self.assertEqual(value['calibration']['admitted'],0)
        self.assertTrue(value['calibration']['separate_from_engineering'])

    def test_aggregate_pass_cannot_replace_an_omitted_component_native_gate(self):
        self.fixture.manifest['passed']=True
        self.fixture.manifest['engineering_acceptance']=True
        self.fixture.manifest['cells'][0]['evidence'].pop('native')
        with self.assertRaises((ValueError,KeyError)):
            self.join()

    def test_final_report_rechecks_real_source_and_proof_bytes(self):
        self.join()
        (self.fixture.root/'source.cpp').write_text('Changed after old pass',encoding='utf-8')
        with self.assertRaises(ValueError):
            self.join()

    def test_final_report_cannot_omit_calibration_identity_ledger(self):
        self.fixture.manifest['calibration'].pop()
        with self.assertRaises(ValueError):
            self.join()

    def test_public_component_gates_require_the_exact_current_full_installed_campaign(self):
        root=self.fixture.root;folder=root/'component-public';folder.mkdir()
        proof=dict(models=[{k:c[k] for k in ('model','component')} for c in self.fixture.cells])
        for name,value in (('components.json',proof),('run.json',dict(synthetic=True))):
            (folder/name).write_text(json.dumps(value),encoding='utf-8')
        campaign=dict(component_public={key:report.binding(folder/name) for key,name in
            (('proof','components.json'),('run','run.json'))},commands=[dict(label='installed-component-chain',exit_code=0,command=['synthetic'])])
        rows=[dict(model=c['model'],component=c['component'],gates=dict(public_installed=dict(
            artifact=matrix.binding(folder/'components.json',root),row=i))) for i,c in enumerate(self.fixture.cells)]
        components=dict(records=rows)
        with mock.patch.object(report,'admit_component_public',return_value=proof):
            self.assertEqual(report.join_component_public({},root,root,campaign,components)['executions'],72)
            campaign['commands'][0]['command'].extend(['--only-model','fixture-0'])
            with self.assertRaises(ValueError):report.join_component_public({},root,root,campaign,components)
            campaign['commands'][0]['command']=['synthetic']
            rows[0]['gates']['public_installed']['artifact']['path']='old-public-proof.json'
            with self.assertRaises(ValueError):report.join_component_public({},root,root,campaign,components)


if __name__=='__main__':
    unittest.main(verbosity=2)
