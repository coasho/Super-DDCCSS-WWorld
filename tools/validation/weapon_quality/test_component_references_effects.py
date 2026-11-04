"""Independent controls for primary-equation comparisons and failed admission."""
import copy
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import component_references_effects as ref


class ReferenceMethods(unittest.TestCase):
    def test_planck_series_against_independent_wavelength_quadrature(self):
        # Composite midpoint in wavelength space is independent of the
        # reference's exact exponential-series antiderivative and native's
        # dimensionless Simpson quadrature. 32768 intervals bound the smooth
        # mid-IR integral well inside 1e-7 relative in these declared cases.
        for t, low, high in ((300., 3., 5.), (1000., 3., 5.), (2000., 8., 12.)):
            count = 32768
            step = (high-low)*1e-6/count
            values = []
            for i in range(count):
                wavelength = low*1e-6+(i+.5)*step
                values.append(2*6.62607015e-34*299792458.**2/wavelength**5 /
                    math.expm1(6.62607015e-34*299792458./(wavelength*1.380649e-23*t)))
            expected = math.fsum(values)*step
            self.assertLess(abs(ref.planck_radiance(t, 1., low, high)/expected-1), 1e-7)

    def test_planck_band_additivity_and_grey_linearity(self):
        a = ref.planck_radiance(600., .8, 2., 4.)
        b = ref.planck_radiance(600., .8, 4., 8.)
        whole = ref.planck_radiance(600., .8, 2., 8.)
        self.assertAlmostEqual((a+b)/whole, 1., places=14)
        self.assertAlmostEqual(whole/ref.planck_radiance(600., 1., 2., 8.), .8, places=14)

    def test_planck_refuses_invalid_domain(self):
        for args in ((0, 1, 3, 5), (300, -1, 3, 5), (300, 1, 5, 3)):
            with self.assertRaises(ValueError):ref.planck_radiance(*args)

    def test_radar_range_dwell_and_separate_transmitter_scalings(self):
        m = dict(frequency_hz=1e10, power_w=100, transmitter_gain=100,
            receiver_gain=100, transmitter_range_m=1000, receiver_range_m=1000,
            rcs_m2=1, system_loss_db=0, receiver_excess_loss_db=0,
            transmitter_excess_loss_db=0, dwell_s=.01, temperature_k=290, noise_figure_db=0)
        base = ref.radar_energy(m)
        self.assertAlmostEqual(ref.radar_energy(dict(m, receiver_range_m=2000))/base, .25)
        self.assertAlmostEqual(ref.radar_energy(dict(m, transmitter_range_m=2000, receiver_range_m=2000))/base, 1/16)
        self.assertAlmostEqual(ref.radar_energy(dict(m, dwell_s=.02))/base, 2.)
        self.assertAlmostEqual(ref.radar_energy(dict(m, transmitter_excess_loss_db=10))/base, .1)

    def test_exact_projection_non_unit_stationary_endpoint_and_interior(self):
        for a, b, expected in (((3, 4, 0), (-3, 4, 0), (.5, 4.)),
                               ((3, 4, 0), (3, 4, 0), (0., 5.)),
                               ((6, 8, 0), (3, 4, 0), (1., 5.))):
            self.assertEqual(ref.segment_reference(a, b), expected)
        a = math.nextafter(1., 2.)
        u, miss = ref.segment_reference((a, .125, 0), (-a, .125, 0))
        self.assertEqual(u, .5);self.assertEqual(miss, .125)

    def test_gurney_energy_not_mass_proportional_speed(self):
        self.assertEqual(ref.cylinder_reference(1000., .5, 1.), 1000.)
        self.assertAlmostEqual(ref.cylinder_reference(1000., 1.5, 1.), 1000/math.sqrt(2))
        self.assertEqual(ref.cylinder_reference(1000., 50., 100.), 1000.)
        with self.assertRaises(ValueError):ref.cylinder_reference(1000., 1., 0.)

    def test_detector_dark_only_limit_and_solid_angle_scaling(self):
        m = dict(column=4., row=4., width=8., height=8., pixel_pitch_rad=.001,
            aperture_m2=.01, transmission=.8, integration_s=.01, wavelength_m=4e-6,
            quantum_efficiency=.5, response_s=.01, dark_electrons_s=100., radiance_w_m2_sr=0.)
        self.assertEqual(ref.detector_charge(m), 1.)
        m['radiance_w_m2_sr'] = 1.
        self.assertGreater(ref.detector_charge(m), 1.)
        self.assertAlmostEqual((ref.detector_charge(dict(m, aperture_m2=.02))-1)/(ref.detector_charge(m)-1), 2.)


class AdmissionControls(unittest.TestCase):
    def test_nonfinite_negative_tolerance_and_numeric_mismatch_refused(self):
        for values in ((math.nan, 1, 1), (1, 1, -1), (1, 2, 0), (math.inf, 1, 1)):
            with self.assertRaises(ValueError):ref.residual('control', *values)

    def test_reference_tolerance_is_not_allowed_to_fit_observation(self):
        m = dict(law='planck_band_radiance', temperature_k=600., emissivity=.8,
                 low_um=3., high_um=5., actual=0.)
        with self.assertRaises(ValueError):ref.compare_seeker([m])

    def test_empty_or_foreign_or_duplicate_population_refused(self):
        for native in ({}, dict(schema='fsim.sim-m03.component-effects-native.v1', complete=True, models=[]),
            dict(schema='fsim.sim-m03.component-effects-native.v1', complete=True, models=[dict(model='x')]*24)):
            with self.assertRaises((ValueError, KeyError)):ref.compare_capture(native, [])

    def test_native_failure_skip_wrong_case_or_zero_tests_refused(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'report.xml'
            name = 'Actual selected AAM effects methods expose primary-reference observations'
            for body in ('', '<testcase name="other"/>',
                f'<testcase name="{name}"><failure/></testcase>',
                f'<testcase name="{name}"><skipped/></testcase>', f'<testcase name="{name}"/>'):
                path.write_text('<testsuite>'+body+'</testsuite>')
                with self.assertRaises(ValueError):ref.passing_native_report(path)
            shape = ('<testsuite name="fsim_component_effect_reference_native_test.exe" tests="818" '
                     'errors="0" failures="0" skipped="0"><testcase name="'+name+'" status="run"/></testsuite>')
            for changed in (shape.replace('tests="818"', 'tests="0"'), shape.replace(name, name+' foreign'),
                            shape.replace('errors="0"', ''), shape.replace('status="run"', 'status="notrun"')):
                path.write_text(changed)
                with self.assertRaises(ValueError):ref.passing_native_report(path)
            path.write_text(shape)
            self.assertEqual(ref.passing_native_report(path), 1)

    def test_json_duplicate_and_nonfinite_refused(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'bad.json'
            for text in ('{"x":1,"x":2}', '{"x":NaN}'):
                path.write_text(text)
                with self.assertRaises(ValueError):ref.read(path)

    def test_fuze_distinguishes_clamped_minimum_from_actual_turn(self):
        row = dict(fuze=[dict(x0_m=100, x1_m=-100, y_m=5, fraction=.5, miss_m=5,
                            radius_m=10, scale=.5, triggered=True),
                        dict(x0_m=100, x1_m=-100, y_m=15, fraction=.5, miss_m=15,
                            radius_m=10, scale=1.5, triggered=False)],
                   fuze_endpoint=dict(model_version=0, radius_m=10, miss_m=5, fraction=1, triggered=True))
        self.assertEqual(len(ref.compare_fuze(row)), 8)
        row['fuze_endpoint'].update(model_version=1, triggered=False)
        self.assertEqual(len(ref.compare_fuze(row)), 8)
        row['fuze_endpoint']['triggered'] = True
        with self.assertRaises(ValueError):ref.compare_fuze(row)

    def test_missing_detector_or_rf_branch_refused(self):
        for data in ([], [dict(law='foreign', actual=1)]):
            with self.assertRaises(ValueError):ref.compare_seeker(data)

    def test_actual_roster_and_resolved_parameter_identity(self):
        cells = ref.inventory()
        self.assertEqual(len(cells), 72)
        self.assertEqual({c['component'] for c in cells}, set(ref.FAMILIES))
        self.assertEqual({f: len({c['mechanism_id'] for c in cells if c['component'] == f}) for f in ref.FAMILIES},
                         dict(seeker=4, fuze=2, warhead=3))
        self.assertTrue(all(c['parameter_sha256'] and c['parameters'] for c in cells))

    def test_stale_epoch_refuses_before_native_or_report_is_opened(self):
        with tempfile.TemporaryDirectory() as directory:
            epoch = Path(directory)/'epoch.json'
            epoch.write_text('{"schema":"old","sources":[]}')
            with patch.object(ref, 'source_epoch', return_value=dict(schema='current', sources=[])):
                with self.assertRaisesRegex(ValueError, 'stale or incomplete'):
                    ref.generate(Path(directory)/'missing-native', Path(directory)/'missing-report', epoch, directory,
                                 run_receipt=Path(directory)/'unused-run.json')

    def test_fixed_reference_inputs_cannot_be_replaced_by_foreign_band_or_geometry(self):
        cell = next(c for c in ref.inventory() if c['model'] == 'r-60m' and c['component'] == 'seeker')
        with self.assertRaises(ValueError):ref.validate_inputs(cell, dict(seeker=[]))
        cell = next(c for c in ref.inventory() if c['model'] == 'aim-120c' and c['component'] == 'fuze')
        p = {k:v['value'] for k, v in cell['parameters'].items()}
        m = dict(radius_m=p['radius_m'], arming_s=p['arming_s'], life_s=p['life_s'],
            delay_s=p.get('delay_s', 0), x0_m=100, x1_m=-100, y_m=.5*p['radius_m'],
            scale=.5, age_s=p['arming_s']+.25, duration_s=1)
        row = dict(fuze=[m], fuze_endpoint=dict(model_version=p['model_version'], radius_m=p['radius_m']))
        ref.validate_inputs(cell, row)
        row['fuze'][0]['radius_m'] *= 2
        with self.assertRaises(ValueError):ref.validate_inputs(cell, row)

    def test_emitted_population_mass_energy_topology_are_actual_predicates(self):
        speed = 1000.
        e = dict(count=8, mass_kg=.5, energy_j=.5*.5*speed**2, segments=0, expanding=0,
            minimum_speed_ms=speed, maximum_speed_ms=speed, minimum_radius_m=.04,
            maximum_radius_m=.05, maximum_axial_m=.1, outer_radius_m=.05,
            inner_radius_m=.03, length_m=.2)
        w = dict(gurney_ms=1000., case_kg=.5, charge_kg=1., actual=speed,
            source_speed_ms=speed, rod=False, model_version=2, fragment_count=8, emission=e)
        self.assertGreater(len(ref.compare_warhead(w)), 8)
        for key, changed in (('mass_kg', .49), ('energy_j', e['energy_j']*1.01),
                              ('count', 7), ('maximum_radius_m', .051), ('segments', 8)):
            bad = copy.deepcopy(w);bad['emission'][key] = changed
            with self.subTest(key=key), self.assertRaises(ValueError):ref.compare_warhead(bad)

    def test_card_projection_rejects_self_consistent_wrong_native_source(self):
        cells = ref.inventory()
        for model in ('magic-2', 'r-60m', 'r-27er', 'aim-120c'):
            cell = next(c for c in cells if c['model'] == model and c['component'] == 'warhead')
            projection = ref.project_warhead(cell)
            def reconcile(w):
                base = ref.cylinder_reference(w['gurney_ms'], w['case_kg'], w['charge_kg'])
                w['actual'] = base
                speed = min(base, 1400.) if w['rod'] and w['model_version'] == 0 else base
                w['source_speed_ms'] = speed
                e = w['emission'];n = w['fragment_count'];connected = e['expanding'] > 0
                e.update(count=n, mass_kg=w['case_kg'], segments=n if w['rod'] else 0,
                         minimum_speed_ms=speed, maximum_speed_ms=speed,
                         energy_j=.5*w['case_kg']*speed**2*((2+math.cos(2*math.pi/n))/3 if connected else 1))
                if connected:
                    material = n*e['length_m']*math.sin(math.pi/n)/math.pi
                    e.update(expanding=n, expansion_per_s=speed/e['outer_radius_m'],
                        breakup_s=(n*e['length_m']/(2*math.pi)-e['outer_radius_m'])/speed,
                        minimum_material_length_m=material, maximum_material_length_m=material)
                return w
            w = dict(projection)
            outer, inner, length = (projection[k] for k in ('outer_radius_m', 'inner_radius_m', 'length_m'))
            connected = cell['mechanism_id'] == 'native.warhead.connected_ring.v1'
            w['emission'] = dict(outer_radius_m=outer, inner_radius_m=inner, length_m=length,
                minimum_radius_m=outer if connected else (outer+inner)/2, maximum_radius_m=outer,
                maximum_axial_m=0 if connected else length/2, expanding=projection['fragment_count'] if connected else 0,
                momentum_norm_ns=0, ring_gap_m=0)
            w = reconcile(w)
            ref.compare_warhead(w);ref.validate_inputs(cell, dict(warhead=w))
            for field, value in (('case_kg', w['case_kg']*.9), ('fragment_count', w['fragment_count']+1),
                ('model_version', 0 if w['model_version'] else 2), ('fragment_source', 99), ('rod', not w['rod'])):
                bad = copy.deepcopy(w);bad[field] = value;bad = reconcile(bad)
                # The original conservation-only comparator accepts this
                # self-consistent bad binding. Card-only projection must not.
                ref.compare_warhead(bad)
                with self.subTest(model=model, field=field), self.assertRaises(ValueError):
                    ref.validate_inputs(cell, dict(warhead=bad))
            bad = copy.deepcopy(w);bad['emission']['outer_radius_m'] *= 1.01;bad = reconcile(bad)
            ref.compare_warhead(bad)
            with self.assertRaises(ValueError):ref.validate_inputs(cell, dict(warhead=bad))

    def test_forbidden_paths_are_refused_before_any_filesystem_or_xml_operation(self):
        bad = Path('D:/ENV/tmp/never-created/.CoDeX/evidence.json')
        actions = [lambda:ref.read(bad), lambda:ref.sha(bad), lambda:ref.bind(bad),
            lambda:ref.passing_native_report(bad), lambda:ref.inventory(bad), lambda:ref.source_epoch(bad),
            lambda:ref.bound_file(dict(path=str(bad), sha256='0'*64)),
            lambda:ref.generate(bad, Path('report'), Path('epoch'), Path('out'), run_receipt=Path('run')),
            lambda:ref.validate_run(bad, Path('native'), Path('report'), Path('epoch'), {})]
        with (patch.object(Path, 'read_text', side_effect=AssertionError('Unexpected read')),
             patch.object(Path, 'open', side_effect=AssertionError('Unexpected open')),
             patch.object(Path, 'resolve', side_effect=AssertionError('Unexpected resolution')),
             patch.object(ref.ET, 'parse', side_effect=AssertionError('Unexpected XML I/O'))):
            for action in actions:
                with self.assertRaisesRegex(ValueError, 'Excluded'):action()

    def test_actual_run_links_refuse_stale_capture_epoch_binary_and_reports(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            epoch, exe, capture, report, receipt = [root/n for n in ('epoch.json', 'producer.exe', 'native.json', 'junit.xml', 'run.json')]
            epoch.write_text('{"epoch":1}');exe.write_bytes(b'synthetic producer bytes; no native execution claim')
            report.write_text('<testsuite/>')
            native = dict(schema='fsim.sim-m03.component-effects-native.v1', runtime_modules=[],
                          input_epoch_sha256=ref.sha(epoch), executable=ref.bind(exe, root))
            capture.write_text(json.dumps(native))
            execution = dict(command=[str(exe), '[component-effects-reference]', '--reporter', 'junit', '--out', str(report)],
                environment=dict(FSIM_COMPONENT_EFFECT_REFERENCE_EPOCH=str(epoch), FSIM_COMPONENT_EFFECT_REFERENCE_DIR=str(root)),
                exit_code=0, raw=[ref.bind(p, root) for p in (capture, report, epoch)],
                staged_runtime=[ref.bind(exe, root)], runtime=[ref.bind(exe, root)])
            good = dict(schema='fsim.sim-m03.component-reference-native-run.v1', passed=True, executions=[execution])
            def verify(value=good, observed=native):
                receipt.write_text(json.dumps(value))
                return ref.validate_run(receipt, capture, report, epoch, observed, root)
            verify()
            variants=[]
            bad=copy.deepcopy(good);bad['executions'][0]['raw'].pop();variants.append(bad)
            bad=copy.deepcopy(good);bad['executions'][0]['runtime']=[];variants.append(bad)
            bad=copy.deepcopy(good);bad['executions'][0]['exit_code']=1;variants.append(bad)
            bad=copy.deepcopy(good);bad['executions'][0]['command'][1]='[foreign]';variants.append(bad)
            bad=copy.deepcopy(good);bad['executions'].append(copy.deepcopy(execution));variants.append(bad)
            for bad in variants:
                with self.assertRaises(ValueError):verify(bad)
            observed=copy.deepcopy(native);observed['input_epoch_sha256']='0'*64
            with self.assertRaises(ValueError):verify(observed=observed)
            # A fresh epoch cannot reauthorize an old otherwise identical
            # numeric capture, even after refreshing the receipt's epoch hash.
            epoch.write_text('{"epoch":2}')
            refreshed=copy.deepcopy(good);refreshed['executions'][0]['raw'][-1]=ref.bind(epoch, root)
            with self.assertRaises(ValueError):verify(refreshed)
            epoch.write_text('{"epoch":1}')
            exe.write_bytes(b'different producer binary')
            with self.assertRaises(ValueError):verify()

    def test_observed_dll_cannot_be_omitted_from_actual_runtime_receipt(self):
        with tempfile.TemporaryDirectory(dir='D:/ENV/tmp', prefix='m03-effects-module-') as directory:
            root = Path(directory)
            epoch, exe, dll, native, report, receipt = [root/name for name in
                ('epoch.json', 'producer.exe', 'libJSBSim.dll', 'native.json', 'report.xml', 'run.json')]
            epoch.write_text('{}');exe.write_bytes(b'synthetic executable');dll.write_bytes(b'synthetic loaded module');report.write_text('<testsuite/>')
            capture = dict(schema='fsim.sim-m03.component-effects-native.v1', input_epoch_sha256=ref.sha(epoch),
                           executable=ref.bind(exe, root), runtime_modules=[ref.bind(dll, root)])
            native.write_text(json.dumps(capture))
            execution = dict(command=[str(exe), '[component-effects-reference]', '--reporter', 'junit', '--out', str(report)],
                environment=dict(FSIM_COMPONENT_EFFECT_REFERENCE_EPOCH=str(epoch), FSIM_COMPONENT_EFFECT_REFERENCE_DIR=str(root)),
                exit_code=0, raw=[ref.bind(p, root) for p in (native, report, epoch)],
                staged_runtime=[ref.bind(exe, root), ref.bind(dll, root)], runtime=[ref.bind(exe, root)])
            receipt.write_text(json.dumps(dict(schema='fsim.sim-m03.component-reference-native-run.v1', passed=True, executions=[execution])))
            with self.assertRaises(ValueError):ref.validate_run(receipt, native, report, epoch, capture, root)


if __name__ == '__main__':unittest.main()
