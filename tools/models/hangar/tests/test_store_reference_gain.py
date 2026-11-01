"""The frozen MK82 change must improve the original external holdouts."""
import hashlib,json,tempfile,unittest
from pathlib import Path
from hangar import pipeline,reference_campaign as C
from hangar.sdk_measurements import Measurements
from tests.paths import TEMP_ROOT
ROOT=Path(__file__).resolve().parents[4]
class StoreReferenceGain(unittest.TestCase):
    def test_fixed_simulation_inputs_are_distinct_from_fitted_and_historical_parameters(self):
        from hangar.aero_evidence import parameter_audit
        row=parameter_audit(dict(provenance=dict(status='sdk-reference-fit',
            fitted_parameter_names=['extra_drag_area_m2'],
            simulated_method_input_names=['body_friction_temperature_k']),calibration=dict(
            extra_drag_area_m2=.1,body_friction_temperature_k=281.65,throttle_ratio=1.2)))
        self.assertEqual(row['parameters']['body_friction_temperature_k']['basis'],'simulated_method_input')
        self.assertEqual(row['parameters']['extra_drag_area_m2']['basis'],'sdk_reference_fit')
        self.assertEqual(row['unverified_parameters'],['throttle_ratio'])

    def test_a22_original_cross_scale_holdouts_improve_without_using_their_values_in_fit(self):
        import fsim
        fsim.set_log_level('off');cat=C.load_catalog();rec=cat['models']['cdsa22']
        held=[c for c in rec['cases'] if c['split']=='validation']
        self.assertEqual([c['value'] for c in held],[.802,.804])
        with tempfile.TemporaryDirectory(dir=TEMP_ROOT) as folder:
            r=Measurements('cdsa22',Path(folder))
            before=r.measure({'base_drag_scale':1.},held)
            after=r.measure(r.initial,held)
        observed=[c['value'] for c in held]
        self.assertLessEqual(C.errors(after,observed)['nrmse_percent'],.9*C.errors(before,observed)['nrmse_percent'])

    def test_tunnel_measurements_preserve_source_dynamic_pressure_and_density_after_reset(self):
        import fsim
        fsim.set_log_level('off')
        case=C.load_catalog()['models']['cdsa22']['cases'][0]
        with tempfile.TemporaryDirectory(dir=TEMP_ROOT) as folder:
            r=Measurements('cdsa22',Path(folder));r.measure({},[case]);actual=r.last[0]
        c=case['conditions']
        self.assertLess(abs(actual['density_kg_m3']/c['density_kg_m3']-1.),1.e-4)
        self.assertLess(abs(actual['qbar_pa']/c['qbar_pa']-1.),1.e-4)

    def test_a22_reference_geometry_and_original_scale_split_are_explicit(self):
        card=json.loads((ROOT/'assets/stores/cds-a22.json').read_text())
        for key,value in [('length_m',52*.0254),('width_m',43*.0254),('height_m',60*.0254)]:
            self.assertAlmostEqual(card['parameters'][key]['value'],value)
            self.assertEqual(card['parameters'][key]['source'],'asd-tr-61-326')
        cases=C.load_catalog()['models']['cdsa22']['cases']
        self.assertEqual([(c['split'],c['value']) for c in cases],
                         [('calibration',.950),('validation',.802),('validation',.804)])
        self.assertEqual([c['conditions']['geometry_scale'] for c in cases],[1/6,1/8,1/4])
        self.assertTrue(all(c['conditions']['alpha_deg']==90. for c in cases))

    def test_body_only_wind_tunnel_fixture_scales_geometry_and_reference_metrics(self):
        import tomllib
        with tempfile.TemporaryDirectory(dir=TEMP_ROOT) as folder:
            r=Measurements('cdsa22',Path(folder));s=1./6
            _,xml=r.fixture({},geometry_scale=s,reference_reynolds=366000.,
                            reference_length_m=1.714*s)
            metrics=xml.find('metrics')
            self.assertAlmostEqual(float(metrics.findtext('wingarea'))*.3048**2,r.design.aircraft.S*s*s)
            self.assertAlmostEqual(float(metrics.findtext('chord'))*.3048,r.design.aircraft.c*s)
            metadata=json.loads((Path(folder)/'scaled-fixture.json').read_text())
            self.assertAlmostEqual(metadata['solver_reynolds'],366000.)
            self.assertAlmostEqual(metadata['geometry_scale'],s)
            self.assertEqual(metadata['source_sha256'],C.sha(r.source))

    def test_retention_and_kpi_are_separate_under_the_authorized_policy(self):
        gain=C.fidelity_gain(7.436524643295269,7.243470188039856)
        self.assertTrue(gain['retained']);self.assertFalse(gain['kpi_met'])
        self.assertFalse(C.fidelity_gain(10.,9.81)['retained'])
        self.assertTrue(C.fidelity_gain(10.,9.8)['retained'])
        self.assertTrue(C.fidelity_gain(10.,9.)['kpi_met'])
    def test_body_reynolds_component_reproduces_frozen_store_holdout_gain(self):
        import fsim
        fsim.set_log_level('off');cases=C.load_catalog()['models']['mk82']['cases']
        held=[c for c in cases if c['split']=='validation']
        parameters=dict(extra_drag_area_m2=.0030627708865117666,korn_kappa=.9299999999999999,
            wave_drag_efficiency=1.2,wave_drag_rise_exponent=2.0882491001430443,
            body_friction_reynolds_coupling=1.,body_friction_temperature_k=281.65102236356387,
            body_friction_density_kg_m3=1.1116683389856714)
        with tempfile.TemporaryDirectory(dir=TEMP_ROOT) as folder:
            r=Measurements('mk82',Path(folder));values=r.measure(parameters,held)
        self.assertLessEqual(C.errors(values,[c['value'] for c in held])['nrmse_percent'],.98*7.436524643295269)

    def test_body_reynolds_component_requires_a_positive_reference_atmosphere(self):
        d=pipeline.Design(str(ROOT/'aircraft/mk82/mk82.toml'),log=lambda message:None)
        d.aircraft.calibration.update(body_friction_reynolds_coupling=1.,body_friction_temperature_k=-1.,
                                      body_friction_density_kg_m3=1.)
        with self.assertRaisesRegex(ValueError,'reference atmosphere'):d.tables()

    def test_retest_rejects_a_changed_reference_partition_before_loading_sdk(self):
        from hangar.store_reference_gain import measure_gain
        evidence=C.load_catalog()['model_audits']['mk82']['frozen_store_calibration']
        with tempfile.TemporaryDirectory(dir=TEMP_ROOT) as folder:
            changed=dict(evidence);changed['original_cases_sha256']='changed'
            path=Path(folder)/'evidence.json';path.write_text(json.dumps(changed))
            with self.assertRaisesRegex(ValueError,'reference partition'):
                measure_gain('mk82',path,Path(folder)/'out.json')

    def test_default_curvature_is_exactly_the_existing_method(self):
        import numpy as np
        d=pipeline.Design(str(ROOT/'aircraft/mk82/mk82.toml'),log=lambda message:None)
        d.aircraft.calibration.pop('wave_drag_rise_exponent',None)
        old=d.tables()['mach']['dCD0'].copy()
        d.aircraft.calibration['wave_drag_rise_exponent']=1.
        np.testing.assert_array_equal(d.tables()['mach']['dCD0'],old)

    def test_mk82_frozen_curvature_reduces_original_holdout_error_ten_percent(self):
        import fsim
        fsim.set_log_level('off');cases=C.load_catalog()['models']['mk82']['cases']
        self.assertEqual(hashlib.sha256(json.dumps(cases,sort_keys=True,separators=(',',':')).encode()).hexdigest(),
                         '27fcb6472a02873cfae75220eb2ee6f65790fd728de686241696cf051601c2e4')
        held=[c for c in cases if c['split']=='validation']
        parameters=dict(extra_drag_area_m2=.003156072225662091,korn_kappa=.9333311439835957,
                        wave_drag_efficiency=1.2,wave_drag_rise_exponent=2.1602699799154053)
        with tempfile.TemporaryDirectory(dir=TEMP_ROOT) as folder:
            r=Measurements('mk82',Path(folder));values=r.measure(parameters,held)
        self.assertEqual(len(held),6)
        self.assertLessEqual(C.errors(values,[c['value'] for c in held])['nrmse_percent'],.9*10.278073298737759)
    def test_curvature_preserves_supersonic_anchor_and_lift_moment(self):
        import numpy as np
        d=pipeline.Design(str(ROOT/'aircraft/mk82/mk82.toml'),log=lambda message:None)
        d.aircraft.calibration=dict(d.aircraft.calibration,wave_drag_rise_exponent=1.)
        original=d.tables();reference={key:value.copy() for key,value in original['mach'].items() if isinstance(value,np.ndarray)}
        lift=original['base']['CL'].copy();moment=original['base']['Cm'].copy()
        d.aircraft.calibration['wave_drag_rise_exponent']=2.
        changed=d.tables()
        np.testing.assert_array_equal(changed['base']['CL'],lift);np.testing.assert_array_equal(changed['base']['Cm'],moment)
        grid=reference['mach'];sup=grid>=1.2;sub=(grid>.9)&(grid<1.05)
        np.testing.assert_array_equal(changed['mach']['dCD0'][sup],reference['dCD0'][sup])
        self.assertTrue(np.any(changed['mach']['dCD0'][sub]<reference['dCD0'][sub]))
if __name__=='__main__':unittest.main()
