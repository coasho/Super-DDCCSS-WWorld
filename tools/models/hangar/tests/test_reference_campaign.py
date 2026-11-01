"""Reference eligibility, leakage prevention and SDK-only fitting contracts."""
import copy
import unittest
import json
from pathlib import Path
import tempfile
from unittest.mock import patch

from hangar import reference_campaign as campaign


class ReferenceCampaignTests(unittest.TestCase):
    def test_worse_frozen_holdout_candidate_never_reaches_production(self):
        from tests.paths import TEMP_ROOT
        import tomllib
        path=campaign.roster()['mk82'].parent/'calibration.toml'
        before=path.read_bytes();initial=tomllib.loads(before.decode())['calibration']
        bad=dict(initial,extra_drag_area_m2=.03)
        with tempfile.TemporaryDirectory(dir=TEMP_ROOT) as folder, \
             patch.object(campaign,'fit_parameters',return_value={'parameters':bad,'history':[]}), \
             patch.object(campaign,'persist_parameters',side_effect=AssertionError('Rejected candidate must not publish')), \
             patch('hangar.pipeline.run',side_effect=AssertionError('Rejected candidate must not regenerate products')):
            row=campaign.execute_model('mk82',campaign.load_catalog(),Path(folder),apply=True)
        self.assertTrue(row['candidate_rejected'])
        self.assertFalse(row['configuration_updated'])
        self.assertEqual(row['parameters'],initial)
        self.assertEqual(before,path.read_bytes())

    def test_no_fit_retest_does_not_claim_an_older_gain_as_new(self):
        from tests.paths import TEMP_ROOT
        source=campaign.roster()['mk82']
        report=source.parent/'out/fly.json';before=report.read_bytes()
        with tempfile.TemporaryDirectory(dir=TEMP_ROOT) as folder:
            row=campaign.execute_model('mk82',campaign.load_catalog(),Path(folder),apply=False)
            self.assertTrue((Path(row['runtime_check_directory'])/'verify.json').is_file())
            self.assertTrue((Path(row['runtime_check_directory'])/'fly.json').is_file())
            self.assertEqual(report.read_bytes(),before,'No-fit SDK checks must not overwrite canonical reports')
        self.assertFalse(row['configuration_updated'])
        self.assertIn('No new fidelity gain',row['round_outcome'])

    def test_status_csv_separates_current_round_from_retained_gain(self):
        import csv
        from tests.paths import TEMP_ROOT
        row=dict(model='sample',status='calibrated',kind='store',initial_parameters={'x':2.},parameters={'x':2.},
            baseline_validation={'CD':{'nrmse_percent':10.}},validation={'CD':{'nrmse_percent':10.}},
            evidence_audit={'retained_calibration_change':{'baseline_validation':{'CD':{'nrmse_percent':20.}},
                'parameter_diff':{'x':{'before':1.,'after':2.}}}})
        with tempfile.TemporaryDirectory(dir=TEMP_ROOT) as folder:
            path=Path(folder)/'report.json';campaign.publish(dict(models=[row]),path)
            with path.with_name('report-status.csv').open(newline='') as stream:actual=next(csv.DictReader(stream))
        self.assertEqual(json.loads(actual['baseline_nrmse_percent']),{'CD':10.})
        self.assertEqual(json.loads(actual['parameter_diff']),{})
        self.assertEqual(json.loads(actual['retained_baseline_nrmse_percent']),{'CD':20.})

    def test_unchanged_source_fit_retests_without_rewriting_or_reflying(self):
        from tests.paths import TEMP_ROOT
        path=campaign.roster()['mk82'].parent/'calibration.toml'
        before=path.read_bytes()
        with tempfile.TemporaryDirectory(dir=TEMP_ROOT) as folder, \
             patch.object(campaign,'persist_parameters',side_effect=AssertionError('Unchanged fit must not rewrite configuration')), \
             patch('hangar.pipeline.run',side_effect=AssertionError('Unchanged fit must not regenerate flight profiles')):
            row=campaign.execute_model('mk82',campaign.load_catalog(),Path(folder),apply=True)
        self.assertFalse(row['configuration_updated'])
        self.assertTrue(row['calibration_executed'])
        self.assertEqual(len(row['observations']),12)
        self.assertEqual(row['validation']['CD']['count'],6)
        self.assertEqual(before,path.read_bytes())

    def test_coupled_parameters_are_refitted_when_one_hits_its_bound(self):
        # The unconstrained solution has negative y. Clipping it leaves x
        # wrong, even though an admissible descent direction still exists.
        result=campaign.fit_parameters(lambda p,c:[p['x']+p['y'],10.*p['x']+p['y']],
            [dict(value=-1.),dict(value=1.)],{'x':0.,'y':0.},{'x':[0.,1.],'y':[0.,1.]})
        self.assertAlmostEqual(result['parameters']['x'],9./101.,places=7)
        self.assertAlmostEqual(result['parameters']['y'],0.,places=7)

    def test_normal_configuration_accepts_sdk_fitted_drag_shape(self):
        from hangar import calibration
        row=calibration.configure(campaign.roster()['iris'])
        self.assertEqual(row['status'],'sdk-reference-fit')
        self.assertGreater(row['overrides']['base_drag_scale'],0.)

    def test_drag_shape_fit_is_bound_to_the_shared_writer(self):
        import numpy as np
        from types import SimpleNamespace
        from hangar.pipeline import Design
        base=np.array([.02,.04,.02])
        tables={'base':{'CD':base,'CL':np.array([-.1,0.,.1])},'mach':{'CD':np.array([.01])}}
        design=SimpleNamespace(aircraft=SimpleNamespace(S=2.,calibration={'base_drag_scale':2.,'extra_drag_area_m2':.01}))
        out=Design.calibrated(design,tables)
        np.testing.assert_allclose(out['base']['CD'],2.*base+.005)
        np.testing.assert_array_equal(tables['base']['CD'],base)
        self.assertIs(out['base']['CL'],tables['base']['CL'])
        self.assertIs(out['mach'],tables['mach'])

    def test_drag_scale_reference_binding_requires_positive_bounds(self):
        r=self.record();r['parameters']={'base_drag_scale':[.25,4.]}
        campaign.validate_record(r,{'wind':{'url':'https://example.org/report'}})
        r['parameters']['base_drag_scale']=[-1.,4.]
        with self.assertRaisesRegex(ValueError,'positive'):
            campaign.validate_record(r,{'wind':{'url':'https://example.org/report'}})

    def test_pressure_altitude_condition_uses_sdk_atmosphere(self):
        from hangar.sdk_measurements import reference_altitude,FT
        from types import SimpleNamespace
        class Vehicle:
            altitude=0.
            def set_property(self,key,value):
                if key=='ic/h-sl-ft':self.altitude=value*FT
            def get_property(self,key):
                self.asserted_property=key
                return self.altitude*.94/FT
        v=Vehicle();environment=[]
        world=SimpleNamespace(step=lambda n:None,set_environment=lambda **kw:environment.append(kw))
        c=dict(altitude_m=1800.,pressure_altitude_m=1800.,temperature_c=20.)
        geometric=reference_altitude(SimpleNamespace(world=world),v,c)
        self.assertAlmostEqual(v.get_property('atmosphere/pressure-altitude')*FT,1800.,places=2)
        self.assertGreater(geometric,c['altitude_m'])
        self.assertAlmostEqual(environment[-1]['temperature_sl_k']-.0065*geometric,293.15)

    def record(self):
        return dict(searches=['manufacturer model wind tunnel data'], unknown_reason='',
                    parameters={'extra_drag_area_m2':[-1.,1.]},
                    cases=[dict(id='train', split='calibration', metric='CD', value=.2,
                                source='wind', locator='table 1', campaign='tunnel-a',
                                conditions=dict(alpha_deg=0.,beta_deg=0.,mach=.5,altitude_m=1000.),
                                configuration='Clean authored geometry; approximation declared'),
                           dict(id='test', split='validation', metric='CD', value=.3,
                                source='wind', locator='table 2', campaign='tunnel-b',
                                conditions=dict(alpha_deg=0.,beta_deg=0.,mach=1.1,altitude_m=1000.),
                                configuration='Clean authored geometry; approximation declared')])

    def test_missing_source_or_conditions_rejected(self):
        r=self.record();r['cases'][0]['source']='absent'
        with self.assertRaises(ValueError):campaign.validate_record(r,{'wind':{'url':'https://example.org/report'}})
        r=self.record();del r['cases'][0]['conditions']['mach']
        with self.assertRaises(ValueError):campaign.validate_record(r,{'wind':{'url':'https://example.org/report'}})

    def test_same_observation_cannot_be_used_for_training_and_validation(self):
        r=self.record();r['cases'][1]['conditions']=copy.deepcopy(r['cases'][0]['conditions'])
        r['cases'][1]['campaign']=r['cases'][0]['campaign'];r['cases'][1]['locator']='different page label'
        with self.assertRaises(ValueError):campaign.validate_record(r,{'wind':{'url':'https://example.org/report'}})

    def test_nonfinite_or_negative_reference_rejected(self):
        for value in (float('nan'),float('inf')):
            r=self.record();r['cases'][0]['value']=value
            with self.assertRaises(ValueError):campaign.validate_record(r,{'wind':{'url':'https://example.org/report'}})

    def test_unsupported_parameter_cannot_silently_produce_a_fit(self):
        r=self.record();r['parameters']={'unbound_engine_scale':[.8,1.2]}
        with self.assertRaisesRegex(ValueError,'Unsupported SDK calibration binding'):
            campaign.validate_record(r,{'wind':{'url':'https://example.org/report'}})

    def test_holdout_values_are_never_used_by_optimizer(self):
        train=[dict(id='train',value=2.)];calls=[]
        def measure(parameters,cases):
            calls.append([c['id'] for c in cases]);return [parameters['x']+1 for c in cases]
        fit=campaign.fit_parameters(measure,train,{'x':0.},{'x':[-3,3]},iterations=5)
        self.assertAlmostEqual(fit['parameters']['x'],1.,places=5)
        self.assertTrue(all(ids==['train'] for ids in calls))

    def test_unknown_is_null_not_zero_error(self):
        self.assertIsNone(campaign.errors([],[])['nrmse_percent'])

    def test_optimizer_scales_tiny_physical_coefficients(self):
        result=campaign.fit_parameters(lambda p,c:[p['k']*1.e9],
            [dict(id='train',value=.2)],{'k':3.e-10},{'k':[1.e-10,5.e-10]})
        self.assertAlmostEqual(result['parameters']['k']/1.e-10,2.,places=6)

    def test_zero_reference_has_absolute_error_without_infinite_percent(self):
        r=campaign.errors([1.],[0.]);self.assertIsNone(r['nrmse_percent'])
        self.assertEqual(r['max_absolute_error'],1.)

    def test_training_and_validation_statistics_are_separate(self):
        self.assertEqual(campaign.errors([2.],[1.])['nrmse_percent'],100.)
        self.assertEqual(campaign.errors([1.],[1.])['nrmse_percent'],0.)

    def test_reference_reading_uncertainty_is_not_hidden_by_tiny_fit_residual(self):
        r=campaign.errors([.02],[.02],[.001])
        self.assertEqual(r['nrmse_percent'],0.)
        self.assertGreater(r['reference_reading_uncertainty_nrmse_interval_percent'][1],5.)

    def test_every_authored_model_has_a_reference_or_explicit_unknown(self):
        catalog=campaign.load_catalog()
        self.assertEqual(set(catalog['models']),set(campaign.roster()))
        for model,record in catalog['models'].items():
            self.assertTrue(record['cases'] or record['unknown_reason'],model)

    def test_mach_holdout_changes_invalidate_its_model_reference_fingerprint(self):
        catalog=campaign.load_catalog();changed=copy.deepcopy(catalog)
        changed['models']['mk82']['cases'][-1]['value']+=.001
        self.assertNotEqual(campaign.reference_hash('mk82',catalog),campaign.reference_hash('mk82',changed))
        self.assertEqual(campaign.reference_hash('c172',catalog),campaign.reference_hash('c172',changed))

    def test_card_and_carriage_products_publish_into_the_same_loaded_sdk(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);source=root/'aircraft/sample/sample.toml'
            source.parent.mkdir(parents=True)
            source.write_text('[aircraft]\nname="sample"\ncategory="store"\ncard="sample-card"\n')
            products=root/'aircraft/stores/sample';(products/'aero').mkdir(parents=True)
            (products/'sample.xml').write_text('<new-aircraft/>')
            current='vlm-carriage-'+'a'*64+'.json';old='vlm-carriage-'+'b'*64+'.json'
            (products/'aero'/current).write_text('{"new":true}')
            card=dict(id='sample-card',sources={'hangar-vlm':dict(url='aero/'+current)})
            (root/'assets/stores').mkdir(parents=True)
            (root/'assets/stores/sample-card.json').write_text(json.dumps(card))
            library=root/'build/sdk/python/fsim/bin/libfsim.dll'
            data=library.parent.parent/'share/flightsim';(data/'stores').mkdir(parents=True)
            destination=data/'aircraft/stores/sample';(destination/'aero').mkdir(parents=True)
            (destination/'aero'/old).write_text('{}');(destination/'aero/retained.json').write_text('{}')
            (data/'stores/sample-card.json').write_text('{"old":true}')
            with patch.object(campaign.pipeline,'repo_root',return_value=str(root)),patch('hangar.sdk_measurements.native_library',return_value={'path':str(library)}):
                campaign.stage_runtime_products(source)
            published=json.loads((data/'stores/sample-card.json').read_text())
            self.assertEqual(published,card)
            self.assertTrue((destination/published['sources']['hangar-vlm']['url']).is_file())
            self.assertEqual((destination/'sample.xml').read_text(),'<new-aircraft/>')
            self.assertFalse((destination/'aero'/old).exists())
            self.assertTrue((destination/'aero/retained.json').exists())

    def test_publication_never_writes_into_an_external_sdk_installation(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder)/'project';root.mkdir()
            library=Path(folder)/'external/bin/libfsim.dll'
            with patch.object(campaign.pipeline,'repo_root',return_value=str(root)),patch('hangar.sdk_measurements.native_library',return_value={'path':str(library)}):
                with self.assertRaisesRegex(ValueError,'built project SDK'):
                    campaign.stage_runtime_products(root/'aircraft/sample/sample.toml')

    def test_configuration_inventory_preserves_unchanged_source_bound_receipts(self):
        from hangar import calibration
        with tempfile.TemporaryDirectory() as folder:
            source=Path(folder)/'sample.toml';source.write_text('[aircraft]\nname="sample"\n')
            path=source.with_name('calibration.toml')
            original='[calibration]\nextra_drag_area_m2=0.1\n\n[provenance]\nschema="fsim.hangar-calibration.v1"\nmethod="shared-aircraft-vlm"\nstatus="existing-performance-fit"\nobserved_source_sha256="'+campaign.sha(source)+'"\n'
            path.write_text(original)
            self.assertEqual(calibration.configure(source)['status'],'existing-performance-fit')
            self.assertEqual(path.read_text(),original)


if __name__=='__main__':unittest.main()
