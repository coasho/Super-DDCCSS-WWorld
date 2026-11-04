"""Independent numeric controls; native coupling is a separate required input."""
import copy
import math
import json
import tempfile
from pathlib import Path
import unittest
from unittest import mock

import numpy as np

import component_references_kinematics as ref


class ComponentKinematicsReferenceTests(unittest.TestCase):
    def test_selected_r77_reference_uses_the_finite_input_signal_stage(self):
        card=next(c for _,c,_ in ref.roster() if c['id']=='r-77m')
        control=ref.resolved(card,'autopilot')|ref.resolved(card,'actuator')
        legacy=ref.operating_benchmark_reference(control,'instantaneous_virtual_angle')
        actual=ref.operating_benchmark_reference(control,'finite_fcs_input_endpoint')
        nominal=np.asarray(ref.cascade_port(control,matrix_reference=True))
        h=.01/(2*control['lag_s']+.01)
        independent=np.array([-3.,-190.,-70.])+np.array([1000.,-10000.,-10000.])*.3*h*nominal
        np.testing.assert_allclose(actual['target'],independent,atol=1e-10,rtol=0.)
        recovered=(actual['target']-np.array([-3.,-190.,-70.]))/np.array([1000.,-10000.,-10000.])/(.3*h)
        np.testing.assert_allclose(recovered,nominal,atol=1e-12,rtol=0.)
        self.assertTrue(actual['input_equivalence_domain'])
        self.assertLess(actual['residual'],1e-8)
        self.assertGreater(np.linalg.norm(legacy['target']-actual['target']),1.)
        with self.assertRaisesRegex(ValueError,'Unknown'):
            ref.operating_benchmark_reference(control,'unlabelled-stage')

    def test_selected_entry_identity_rejects_legacy_alias_and_wrong_benchmark(self):
        def record(code):
            entry,stage=ref.CONTROL_ENTRIES[code]
            return dict(selected_control_entry=entry,selected_control_target_stage=stage,
                operating_legacy_entry='operatingPointCommandsV1',operating_legacy_target_stage='instantaneous_virtual_angle',
                control_benchmark_scope='Synthetic SI method plant; not model flight',
                control_benchmark_mass_kg=100.,control_benchmark_speed_m_s=500.,
                selected_operating_point={} if code==3 else None)
        for code in (1,2,3):self.assertEqual(ref.selected_control_identity(record(code),code),ref.CONTROL_ENTRIES[code])
        for key,value in [('selected_control_entry','operatingPointCommandsV1'),
                          ('selected_control_target_stage','instantaneous_virtual_angle'),
                          ('selected_operating_point',None),('control_benchmark_mass_kg',190.),
                          ('control_benchmark_scope','Actual R77M native flight')]:
            changed=record(3);changed[key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):ref.selected_control_identity(changed,3)
        wrong=record(1);wrong['selected_operating_point']={}
        with self.assertRaises(ValueError):ref.selected_control_identity(wrong,1)

    def test_capped_cascade_reference_preserves_simultaneous_force_and_moment_paths(self):
        card=next(c for _,c,_ in ref.roster() if c['id']=='r-77m')
        control=ref.resolved(card,'autopilot')|ref.resolved(card,'actuator')
        # The unchanged30m/s2/12m/s2 bench clips its15deg incidence circle.
        # The two independent reference routes must retain the same feasible
        # force+moment path instead of preserving rejected demand curvature.
        direct=ref.cascade_port(control)
        matrix=ref.cascade_port(control,matrix_reference=True)
        np.testing.assert_allclose(direct,matrix,atol=1e-12,rtol=0.)

    def test_native_feed_forward_reference_preserves_priority_and_unequal_share(self):
        tanks=[dict(contents=.001,unusable=0.,priority=1,selected=True),
               dict(contents=.1,unusable=0.,priority=1,selected=True),
               dict(contents=9.,unusable=0.,priority=2,selected=True)]
        batch=ref.finite_feed_reference(.008,tanks,[0,1,2])
        self.assertEqual(batch['priority'],1)
        self.assertEqual(batch['group'],[0,1])
        self.assertAlmostEqual(batch['supplied'],.005,places=16)
        self.assertEqual(batch['post'],[0.,.096,9.])
        self.assertEqual(tanks[0]['contents'],.001)

    def test_native_feed_forward_reference_empty_unusable_deselected_and_invalid(self):
        tanks=[dict(contents=.1,unusable=.1,priority=1,selected=True),
               dict(contents=1.,unusable=0.,priority=2,selected=False),
               dict(contents=.202,unusable=.2,priority=3,selected=True)]
        batch=ref.finite_feed_reference(.008,tanks,[0,1,2])
        self.assertEqual(batch['group'],[2])
        self.assertEqual(batch['post'],[.1,1.,.2])
        self.assertAlmostEqual(batch['supplied'],.002,places=16)
        self.assertEqual(ref.finite_feed_reference(.008,tanks,[])['supplied'],0.)
        for demand,feeds in ((math.nan,[0]),(-1.,[0]),(.008,[0,0]),(.008,[3])):
            with self.subTest(demand=demand,feeds=feeds),self.assertRaises(ValueError):
                ref.finite_feed_reference(demand,tanks,feeds)

    def test_excluded_reference_paths_refuse_before_any_io(self):
        forbidden=Path('D:/ENV/tmp')/'.CoDeX'/'never-opened.json'
        with mock.patch.object(Path,'read_bytes',side_effect=AssertionError('Unexpected I/O')):
            with self.assertRaisesRegex(ValueError,'Excluded'):
                ref.digest(forbidden)
        with mock.patch.object(Path,'read_text',side_effect=AssertionError('Unexpected I/O')):
            with self.assertRaisesRegex(ValueError,'Excluded'):
                ref.roster(forbidden)
        with mock.patch.object(Path,'resolve',side_effect=AssertionError('Unexpected metadata I/O')):
            with self.assertRaisesRegex(ValueError,'Excluded'):
                ref.validate_capture_run([forbidden],forbidden,forbidden)
    def test_native_reference_refuses_absent_population(self):
        with self.assertRaises(ValueError):
            ref.native_comparisons({'families': {}},
                {'schema':'fsim.sim-m03.component-native.v1','models':[]})

    def test_powered_reference_refuses_absent_population(self):
        with self.assertRaises(ValueError):
            ref.powered_comparisons({'families':{}},
                {'schema':'fsim.sim-m03.propulsion-reference-native.v1','models':[]})

    def test_zero_powered_engine_is_not_a_reference_pass(self):
        actual=[]
        for _,card,_ in ref.roster():
            actual.append(dict(model=card['id'],native_dt_s=.001,native_samples=1,initial_fuel_kg=1.,
                final_fuel_kg=1.,flow_integral_kg=0.,samples=[[1.,.001,1.,0.,0.,0.,245.,1000.]],
                columns=['native_sample','native_time_s','fuel_kg','engine0_flow_kg_s','engine0_vacuum_thrust_n',
                         'engine0_delivered_thrust_n','engine0_isp_s','engine0_pressure_psf']))
        result={'families':{'propulsion':[dict(model=c['id'],method=dict(tests=0,assertions=0,residuals=[])) for _,c,_ in ref.roster()]}}
        with self.assertRaisesRegex(ValueError,'never produced force'):
            ref.powered_comparisons(result,dict(schema='fsim.sim-m03.propulsion-reference-native.v1',models=actual))

    def test_actuator_aware_reference_is_a_distinct_finite_feedback(self):
        row=next(c for _,c,_ in ref.roster() if c['id']=='pl-10')
        control=dict(ref.resolved(row,'autopilot'),**ref.resolved(row,'actuator'))
        output=ref.actuator_aware_reference(control)
        self.assertEqual(len(output),3)
        self.assertTrue(np.all(np.isfinite(output)))
        self.assertTrue(np.all(np.abs(output)<=1))
        self.assertGreater(np.linalg.norm(np.array(output)-ref.cascade_port(control)),1e-4)

    def test_three_autopilot_and_three_propulsion_cohorts_are_actual(self):
        rows=ref.roster()
        for component in ('autopilot','propulsion'):
            self.assertEqual(len({e['components'][component]['mechanism_id'] for e,_,_ in rows}),3)
        self.assertEqual([c['id'] for e,c,_ in rows if e['components']['autopilot']['mechanism_id'].endswith('operating_point_bounded.v1')],['r-77m'])

    def test_exact_roster_is_independently_enumerated(self):
        rows = ref.roster()
        self.assertEqual(len(rows), 24)
        self.assertEqual(len({card['id'] for _, card, _ in rows}), 24)
        for _, card, _ in rows:
            for family in ref.FAMILIES:
                self.assertEqual(len(ref.parameter_hash(card, family)), 64)

    def test_parameter_hash_binds_values_provenance_and_descriptor(self):
        _, card, _ = ref.roster()[0]
        initial = ref.parameter_hash(card, 'guidance')
        key = card['component_models']['components']['guidance']['parameter_refs']['navigation_gain']
        for role, value in (('value', card['parameters'][key]['value']+.1), ('basis', 'derived'), ('note', 'changed source')):
            changed = copy.deepcopy(card); changed['parameters'][key][role] = value
            self.assertNotEqual(ref.parameter_hash(changed, 'guidance'), initial)

    def test_head_on_has_only_declared_gravity_compensation(self):
        command = ref.measured_pn_port([1, 0, 0], [0, 0, 0], [500, 0, 0], [0, 0, 1], 3., 500., 30.)
        np.testing.assert_array_equal(command, [0, 0, ref.G0])

    def test_pn_reference_is_rotation_covariant(self):
        q = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
        d, rate, v, up = map(np.array, ([1., 0., 0.], [0., .02, -.01], [500., 30., -10.], [0., 0., 1.]))
        a = ref.measured_pn_port(d, rate, v, up, 3.5, 550., 30.)
        b = ref.measured_pn_port(q@d, q@rate, q@v, q@up, 3.5, 550., 30.)
        np.testing.assert_allclose(b, q@a, atol=1e-12, rtol=0.)

    def test_pn_limit_and_bearing_scale_are_executed(self):
        args = ([1, 0, 0], [0, 10, 5], [500, 0, 0], [0, 0, 1], 4., 500.)
        limited = ref.measured_pn_port(*args, 20.)
        self.assertAlmostEqual(np.linalg.norm(limited), 20*ref.G0, places=11)
        base = ref.measured_pn_port([1, 0, 0], [0, .01, 0], [500, 0, 0], [0, 0, 1], 4., 200., 40.)
        faster = ref.measured_pn_port([1, 0, 0], [0, .01, 0], [500, 0, 0], [0, 0, 1], 4., 400., 40.)
        self.assertAlmostEqual(faster[1], 2*base[1], places=12)

    def test_cascade_independent_trim_and_parameter_sensitivity(self):
        _, card, _ = ref.roster()[0]
        control = ref.resolved(card, 'autopilot') | ref.resolved(card, 'actuator')
        a = ref.cascade_port(control); b = ref.cascade_port(control, matrix_reference=True)
        np.testing.assert_allclose(a, b, atol=1e-12, rtol=0.)
        other = dict(control, rate_time_min_s=2*control['rate_time_min_s'])
        self.assertGreater(np.linalg.norm(np.array(a)-ref.cascade_port(other)), 1e-6)

    def test_actuator_rate_stops_and_continuous_limit(self):
        values = ref.lag_rate_stops_port([.5]*200, .04, .3, .01, -.1, .2)
        self.assertLessEqual(max(values), .2)
        self.assertGreaterEqual(min(values), -.1)
        self.assertLessEqual(max(abs(b-a) for a, b in zip([0.]+values, values)), .003+1e-15)
        # Initialized constant input through Tustin: compare the exact pole
        # recurrence with the independently sampled continuous homogeneous mode.
        errors = []
        for dt in (.004, .002):
            r = (2*.04-dt)/(2*.04+dt)
            errors.append(abs(r**round(.2/dt)-math.exp(-.2/.04)))
        self.assertLess(errors[1], .26*errors[0])

    def test_bounded_least_squares_can_release_saturated_axes(self):
        B = np.array([[1., .25, -.1], [.1, 1., .4], [.3, -.2, .8]])
        target = np.array([2., -1.5, .8]); cost, command = ref.bounded_least_squares_reference(B, target)
        self.assertTrue(np.all(np.abs(command) <= 1))
        clipped = np.clip(np.linalg.solve(B, target), -1, 1)
        self.assertLess(cost, np.sum((B@clipped-target)**2))

    def test_six_executed_method_families_project_every_actual_card(self):
        output = ref.execute()
        self.assertFalse(output['native_coupling_passed'])
        self.assertEqual(set(output['families']), set(ref.FAMILIES))
        for family, rows in output['families'].items():
            self.assertEqual(len(rows), 24)
            self.assertEqual(len({r['model'] for r in rows}), 24)
            for row in rows:
                self.assertEqual(row['component'], family)
                self.assertGreater(row['method']['tests'], 0)
                self.assertTrue(all(check['passed'] for check in row['method']['residuals']))


class CaptureProvenanceTests(unittest.TestCase):
    """Synthetic file-link controls only; these are not native execution evidence."""
    def make_fixture(self,folder):
        folder=Path(folder);exe=folder/'synthetic.exe';exe.write_bytes(b'explicit synthetic metadata fixture')
        epoch=folder/'epoch.json';source={'schema':'synthetic-source-epoch','sources':[{'path':'fixture','sha256':'a'*64}]}
        epoch.write_text(json.dumps(source))
        natives=[folder/'kinematics.json',folder/'kinematics.json.propulsion.json']
        module=folder/'libJSBSim.dll';module.write_bytes(b'synthetic staged JSBSim; no native execution')
        module_binding=dict(path=str(module),sha256=ref.digest(module))
        for index,native in enumerate(natives):
            native.write_text(json.dumps(dict(schema='fsim.sim-m03.propulsion-reference-native.v1' if index else 'fsim.sim-m03.component-native.v1',
                runtime_modules=[module_binding],input_epoch_sha256=ref.digest(epoch),executable=dict(path=str(exe),sha256=ref.digest(exe)))))
        report=folder/'catch.xml'
        report.write_text('<testsuites><testsuite tests="100" failures="0" errors="0" skipped="0"><properties>'
            '<property name="filters" value="[component-reference-native]"/></properties>'
            '<testcase name="All actual AAM card configurations execute numeric guidance and control method references" status="run"/>'
            '<testcase name="Every actual AAM engine executes an independent powered propulsion reference bench" status="run"/>'
            '</testsuite></testsuites>')
        binding=lambda p:dict(path=str(p),sha256=ref.digest(p))
        receipt=folder/'run.json';document=dict(schema='fsim.sim-m03.component-reference-native-run.v1',passed=True,executions=[
            dict(command=[str(exe),'[component-reference-native]','--reporter','junit','--out',str(report)],
                 environment=dict(FSIM_COMPONENT_REFERENCE_EPOCH=str(epoch),FSIM_COMPONENT_REFERENCE_JSON=str(natives[0]),
                                  FSIM_COMPONENT_PROPULSION_JSON=str(natives[1])),
                 exit_code=0,staged_runtime=[binding(exe),binding(module)],runtime=[binding(exe),binding(module)],raw=[binding(p) for p in [*natives,epoch,report]])])
        receipt.write_text(json.dumps(document))
        return natives,epoch,receipt,source,report,exe,document

    def run_fixture(self,fixture,source=None):
        natives,epoch,receipt,current,*_=fixture
        with mock.patch.object(ref,'source_epoch',return_value=source or current):
            return ref.validate_capture_run(natives,epoch,receipt)

    def test_synthetic_current_links_positive(self):
        with tempfile.TemporaryDirectory(dir='D:/ENV/tmp',prefix='m03-reference-link-') as folder:
            fixture=self.make_fixture(folder)
            self.assertEqual(self.run_fixture(fixture)['assertions'],100)

    def test_self_consistent_old_capture_and_epoch_refuse_current_source(self):
        with tempfile.TemporaryDirectory(dir='D:/ENV/tmp',prefix='m03-reference-old-') as folder:
            fixture=self.make_fixture(folder);changed=copy.deepcopy(fixture[3]);changed['sources'][0]['sha256']='b'*64
            with self.assertRaisesRegex(ValueError,'stale'):
                self.run_fixture(fixture,changed)

    def test_old_capture_with_fresh_epoch_and_updated_receipt_refuses(self):
        with tempfile.TemporaryDirectory(dir='D:/ENV/tmp',prefix='m03-reference-rebind-') as folder:
            fixture=self.make_fixture(folder);natives,epoch,receipt,source,_,_,document=fixture
            source['sources'][0]['sha256']='c'*64;epoch.write_text(json.dumps(source))
            for item in document['executions'][0]['raw']:
                if item['path']==str(epoch):item['sha256']=ref.digest(epoch)
            receipt.write_text(json.dumps(document))
            with self.assertRaisesRegex(ValueError,'Capture-side source epoch'):
                self.run_fixture(fixture)

    def test_foreign_capture_executable_refuses_even_when_raw_hash_updated(self):
        with tempfile.TemporaryDirectory(dir='D:/ENV/tmp',prefix='m03-reference-foreign-') as folder:
            fixture=self.make_fixture(folder);natives,epoch,receipt,source,_,_,document=fixture
            other=Path(folder)/'foreign.exe';other.write_bytes(b'foreign synthetic fixture')
            value=json.loads(natives[0].read_text());value['executable']=dict(path=str(other),sha256=ref.digest(other));natives[0].write_text(json.dumps(value))
            for item in document['executions'][0]['raw']:
                if item['path']==str(natives[0]):item['sha256']=ref.digest(natives[0])
            receipt.write_text(json.dumps(document))
            with self.assertRaisesRegex(ValueError,'Capture-side executable'):
                self.run_fixture(fixture)

    def test_capture_dll_must_be_bound_by_actual_runtime(self):
        with tempfile.TemporaryDirectory(dir='D:/ENV/tmp',prefix='m03-reference-module-') as folder:
            fixture=self.make_fixture(folder)
            fixture[6]['executions'][0]['runtime']=fixture[6]['executions'][0]['runtime'][:1]
            fixture[2].write_text(json.dumps(fixture[6]))
            with self.assertRaises(ValueError):self.run_fixture(fixture)

    def test_control_family_admission_still_requires_both_runtime_headers(self):
        with tempfile.TemporaryDirectory(dir='D:/ENV/tmp',prefix='m03-reference-both-headers-') as folder:
            fixture=self.make_fixture(folder)
            natives,epoch,receipt,source,_,_,document=fixture
            with mock.patch.object(ref,'source_epoch',return_value=source):
                self.assertTrue(ref.validate_capture_run(natives[:1],epoch,receipt)['passed'])
                document['executions'][0]['raw']=[item for item in document['executions'][0]['raw']
                                                 if item['path']!=str(natives[1])]
                receipt.write_text(json.dumps(document))
                with self.assertRaisesRegex(ValueError,'Both actual native capture headers'):
                    ref.validate_capture_run(natives[:1],epoch,receipt)

    def test_current_dll_cannot_reauthorize_old_capture_module_hash(self):
        with tempfile.TemporaryDirectory(dir='D:/ENV/tmp',prefix='m03-reference-old-dll-') as folder:
            fixture=self.make_fixture(folder)
            module=Path(folder)/'libJSBSim.dll';module.write_bytes(b'new DLL, unchanged executable')
            execution=fixture[6]['executions'][0]
            for key in ('runtime','staged_runtime'):
                for item in execution[key]:
                    if item['path']==str(module):item['sha256']=ref.digest(module)
            fixture[2].write_text(json.dumps(fixture[6]))
            with self.assertRaisesRegex(ValueError,'Runtime bytes changed'):
                self.run_fixture(fixture)

    def test_runtime_binary_mutation_refuses(self):
        with tempfile.TemporaryDirectory(dir='D:/ENV/tmp',prefix='m03-reference-runtime-') as folder:
            fixture=self.make_fixture(folder);fixture[5].write_bytes(b'changed synthetic binary')
            with self.assertRaisesRegex(ValueError,'bytes changed'):
                self.run_fixture(fixture)

    def test_redirected_powered_capture_environment_refuses(self):
        with tempfile.TemporaryDirectory(dir='D:/ENV/tmp',prefix='m03-reference-redirect-') as folder:
            fixture=self.make_fixture(folder)
            fixture[6]['executions'][0]['environment']['FSIM_COMPONENT_PROPULSION_JSON']=str(Path(folder)/'wrong-output.json')
            fixture[2].write_text(json.dumps(fixture[6]))
            with self.assertRaisesRegex(ValueError,'powered native output environment'):
                self.run_fixture(fixture)

    def test_bare_misnamed_zero_assertion_reports_never_admit(self):
        for mode in ('bare','misnamed','zero'):
            with self.subTest(mode=mode),tempfile.TemporaryDirectory(dir='D:/ENV/tmp',prefix='m03-reference-report-') as folder:
                fixture=self.make_fixture(folder);natives,epoch,receipt,source,report,_,document=fixture
                text=report.read_text()
                if mode=='bare':text='<testsuites><testsuite><testcase name="All actual AAM card configurations execute numeric guidance and control method references"/></testsuite></testsuites>'
                elif mode=='misnamed':text=text.replace('Every actual AAM engine','An unrelated synthetic engine')
                else:text=text.replace('tests="100"','tests="0"')
                report.write_text(text)
                for item in document['executions'][0]['raw']:
                    if item['path']==str(report):item['sha256']=ref.digest(report)
                receipt.write_text(json.dumps(document))
                with self.assertRaises(ValueError):self.run_fixture(fixture)


if __name__ == '__main__':
    unittest.main(verbosity=2)
