"""Input-bound simulated products must not imply measured calibration."""
import copy,json,tempfile,unittest
from unittest.mock import patch
from pathlib import Path
from types import SimpleNamespace

from hangar import pipeline

ROOT=Path(__file__).resolve().parents[4]

class AeroEvidence(unittest.TestCase):
    def test_all_aam_aerodynamic_gates_require_exact_model_measurements_not_solver_convergence(self):
        from hangar import missiles,reference_campaign as C
        catalog=C.load_catalog()
        policy=catalog.get('aam_aerodynamic_gate_policy',{})
        self.assertFalse(policy.get('solver_convergence_is_acceptance_criterion',True))
        self.assertEqual(policy.get('generated_estimates_basis'),'simulated')
        self.assertIn('SU2',policy.get('prohibited_execution_methods',[]))
        self.assertIn('VSPAERO',policy.get('prohibited_alternative_acceptance_methods',[]))
        self.assertEqual(len(missiles.MISSILES),24)
        for name in missiles.MISSILES:
            with self.subTest(model=name):
                audit=catalog['model_audits'][name]
                if catalog['models'][name]['cases']:continue
                gate=audit.get('aerodynamic_gate',{})
                self.assertEqual(gate.get('status'),'blocked')
                self.assertTrue(gate.get('unknown_reason'))
                self.assertFalse(gate.get('exact_model_data_available',True))
                self.assertFalse(gate.get('solver_convergence_required',True))
                self.assertEqual(gate.get('estimated_table_basis'),'simulated')

    def test_missing_specific_model_data_stays_blocked_despite_historical_override(self):
        from hangar import aero_evidence as E
        row=dict(observations=[],evidence_audit={'status':'blocked',
            'parameter_audit':{'configuration_status':'configuration-reference-fit'}})
        self.assertEqual(E.closure_status(row),'blocked')

    def test_new_card_dimensions_cannot_be_rebound_to_old_source_geometry(self):
        from hangar import aero_evidence as E
        d=pipeline.Design(str(ROOT/'aircraft/asraam/asraam.toml'),log=lambda message:None)
        card=json.loads((ROOT/'assets/stores/asraam.json').read_text())
        card['parameters']['length_m']['value']+=.01
        with self.assertRaisesRegex(ValueError,'geometry'):
            E.validate_card_geometry(d,card)

    def test_delivery_calibrated_requires_retained_parameter_change_and_current_holdout_gain(self):
        from hangar import aero_evidence as E
        proof=dict(reference_inputs_sha256='source',initial_parameters={'x':1.},parameters={'x':2.},
            baseline_validation={'CD':{'nrmse_percent':20.}},target_metrics=['CD'])
        row=dict(parameters={'x':2.},reference_inputs_sha256='source',observations=[{}],
            validation={'CD':{'nrmse_percent':10.}},evidence_audit=dict(
                parameter_audit={'configuration_status':'sdk-reference-fit'},retained_calibration_change=proof))
        self.assertEqual(E.closure_status(row),'calibrated')
        row['validation']['CD']['nrmse_percent']=20.
        self.assertEqual(E.closure_status(row),'historical')
        row['validation']['CD']['nrmse_percent']=10.;row['parameters']['x']=1.
        self.assertEqual(E.closure_status(row),'historical')

    def test_delivery_does_not_convert_sdk_execution_or_old_fit_into_improvement(self):
        from hangar import aero_evidence as E
        self.assertEqual(E.closure_status(dict(observations=[],evidence_audit={
            'parameter_audit':{'configuration_status':'unfitted'}})),'blocked')
        self.assertEqual(E.closure_status(dict(observations=[{}],evidence_audit={
            'parameter_audit':{'configuration_status':'sdk-reference-fit'}})),'historical')

    def test_reference_fit_does_not_promote_an_inherited_performance_parameter(self):
        from hangar import aero_evidence as E
        row=E.parameter_audit(dict(provenance=dict(status='sdk-reference-fit',
            fitted_parameter_names=['extra_drag_area_m2']),
            calibration=dict(extra_drag_area_m2=.1,throttle_ratio=1.2)))
        self.assertEqual(row['parameters']['extra_drag_area_m2']['basis'],'sdk_reference_fit')
        self.assertEqual(row['parameters']['throttle_ratio']['basis'],'historical_override')
        self.assertEqual(row['unverified_parameters'],['throttle_ratio'])

    def test_campaign_loader_requires_complete_audit_coverage(self):
        from hangar import reference_campaign as C
        catalog=C.load_catalog()
        del catalog['model_audits']['asraam']
        from tests.paths import TEMP_ROOT
        with tempfile.TemporaryDirectory(dir=TEMP_ROOT) as folder:
            path=Path(folder)/'catalog.json'
            path.write_text(json.dumps(catalog),encoding='utf-8')
            with self.assertRaisesRegex(ValueError,'audit'):
                C.load_catalog(path)

    def test_readable_report_exposes_unverified_parameters_and_searches(self):
        from hangar import reference_campaign as C,reference_report as R
        c=C.load_catalog();a=c['model_audits']['aim120c']
        html=R.render(dict(models=[dict(model='aim120c',status='unknown',evidence_audit=a)],sources={}))
        self.assertIn('historical_override',html)
        self.assertIn('0.0017621684384979',html)
        self.assertIn('Search date',html)

    def test_store_verification_rejects_stale_binding_before_loading_sdk(self):
        from hangar import storestages
        stale = {'model':'jsbsim:asraam','sources':{'hangar-vlm':{}}}
        with patch.object(storestages,'card_of',return_value=stale), \
             patch('hangar.aero_evidence.verify_binding',side_effect=ValueError('Stale VLM binding')), \
             patch.object(storestages,'_world') as world:
            with self.assertRaisesRegex(ValueError,'Stale VLM binding'):
                storestages.verify(SimpleNamespace(name='asraam'))
            world.assert_not_called()

    def test_generic_reference_removes_axial_base_force_without_claiming_equipment_data(self):
        from hangar import generic_validation as G
        import math
        a=math.radians(6.)
        # Adding a force along body x changes wind-axis lift and drag, but
        # cannot change body-normal force.
        self.assertAlmostEqual(G.normal_force(.3,.04,6.),G.normal_force(.3-.02*math.sin(a),.04+.02*math.cos(a),6.))
        self.assertEqual(len(G.REFERENCE),9)

    def test_every_aam_product_binds_current_geometry_and_card(self):
        from hangar import aero_evidence as E,missiles
        for name,row in missiles.MISSILES.items():
            with self.subTest(model=name):
                d=pipeline.Design(str(ROOT/'aircraft'/name/(name+'.toml')),log=lambda s:None)
                card=json.loads((ROOT/'assets/stores'/(row['card']+'.json')).read_text(encoding='utf-8'))
                E.verify_binding(d,card)
                # Derived in the frozen SDK vocabulary means derived from
                # published values. A simulated coefficient is not one.
                self.assertEqual(card['parameters']['drag_area_m2']['basis'],'assumed')
                self.assertGreater(card['parameters']['drag_area_m2']['uncertainty'],0.)
                self.assertEqual(card['sources']['hangar-vlm']['simulation']['basis'],'simulated')

    def test_all_88_models_have_individual_evidence_audits(self):
        from hangar import aero_evidence as E
        catalog=json.loads((ROOT/'tools/models/hangar/hangar/public_data.json').read_text())
        self.assertEqual(set(catalog['models']),set(catalog.get('model_audits',{})))
        for name,audit in catalog['model_audits'].items():
            with self.subTest(model=name):E.validate_audit(audit)

    def test_strut_change_invalidates_the_actual_aerodynamic_cache_key(self):
        d=pipeline.Design(str(ROOT/'aircraft/uh1h/uh1h.toml'),log=lambda s:None)
        before=d.table_inputs_hash()
        d.aircraft.spec['strut'][0]['chord']*=1.1
        self.assertNotEqual(before,d.table_inputs_hash())

    def test_current_card_requires_input_bound_simulation_provenance(self):
        from hangar import aero_evidence as E
        card=json.loads((ROOT/'assets/stores/asraam.json').read_text())
        record=E.card_inputs(card)
        changed=copy.deepcopy(card);changed['parameters']['length_m']['value']+=.01
        self.assertNotEqual(record,E.card_inputs(changed))
        # Output references and descriptive computed drag must not create a
        # circular hash dependency on the product they identify.
        changed=copy.deepcopy(card);changed['sources']['hangar-vlm']['url']='changed'
        changed['parameters']['drag_area_m2']['value']+=.01
        self.assertEqual(record,E.card_inputs(changed))

    def test_old_geometry_cannot_pass_a_current_binding_check(self):
        from hangar import aero_evidence as E
        current=dict(length_m=3.,diameter_m=.2,mass_kg=100.)
        binding=dict(basis='simulated',card_inputs=current,table_sha256='a'*64,
                     design_sha256='b'*64,table_inputs_hash='old')
        E.validate_binding(binding,card_inputs=current,design_sha256='b'*64,table_inputs_hash='old',table_sha256='a'*64)
        with self.assertRaisesRegex(ValueError,'card geometry'):
            E.validate_binding(binding,card_inputs=dict(current,length_m=3.1),design_sha256='b'*64,table_inputs_hash='old',table_sha256='a'*64)
        with self.assertRaisesRegex(ValueError,'design'):
            E.validate_binding(binding,card_inputs=current,design_sha256='c'*64,table_inputs_hash='old',table_sha256='a'*64)

    def test_parameter_history_never_becomes_a_calibration_success(self):
        from hangar import aero_evidence as E
        for status in ('configuration-reference-fit','existing-performance-fit'):
            row=E.parameter_audit(dict(provenance=dict(status=status),calibration=dict(extra_drag_area_m2=.1)))
            self.assertEqual(row['verification_status'],'unverified')
            self.assertEqual(row['parameters']['extra_drag_area_m2']['basis'],'historical_override')
            self.assertIsNone(row['parameters']['extra_drag_area_m2']['matched_error_percent'])

    def test_each_unknown_requires_a_specific_source_review_and_nonzero_uncertainty(self):
        from hangar import aero_evidence as E
        with self.assertRaisesRegex(ValueError,'source review'):
            E.validate_audit(dict(status='unknown',source_reviews=[],unknown_reason='Specific reason',estimation={'assumed_relative_half_width':.5}))
        with self.assertRaisesRegex(ValueError,'uncertainty'):
            E.validate_audit(dict(status='unknown',source_reviews=[{'url':'https://example.org','locator':'Product page',
                'conditions':'Not stated','decision':'No coefficient table'}],unknown_reason='Specific reason',
                estimation={'assumed_relative_half_width':0.}))

if __name__=='__main__':unittest.main()
