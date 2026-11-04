"""Admission negative controls only; no simulation, build, or equipment claims."""
import copy
import csv
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import countermeasures as cm


def summary(model='aim-9m', family='flare', channels=1):
    return [model,'test.seeker.v3','4',family,'15','14','10','11.5',str(channels),'1',str(channels==2 and 1 or 0),'120','120','40','expired','30']


def proof(channel=0,radio=False,semi=False):
    values=[channel,'radio' if radio else 'thermal',12,2,.5,2000,1,0,
        0 if radio else 100,0 if radio else 20,0,0,2 if radio else 0,
        40 if radio else 0,4e-199 if radio else 0,int(semi),int(semi),int(semi),
        'retained_first_interval' if radio else 'preclip_charge']
    return dict(zip(cm.PROBE_FIELDS,map(str,values)))


class CountermeasureAdmission(unittest.TestCase):
    def reject(self,s=None,p=None,model=None):
        with self.assertRaises(ValueError):cm.validate_model(s or summary(),p if p is not None else [proof()],model or dict(card='aim-9m',implementation='test.seeker.v3',family='flare',channels=1,semi=False))

    def test_no_cartridge_cannot_pass(self):
        row=summary();row[5]=row[4];self.reject(s=row)

    def test_missing_receiver_contribution_cannot_pass(self):
        self.reject(p=[])
        row=proof();row['positive_expected_electrons']='0';self.reject(p=[row])

    def test_wrong_model_or_provider_cannot_pass(self):
        for field in (0,1):
            row=summary();row[field]='other';self.reject(s=row)

    def test_full_kernel_must_match_actual_detector(self):
        row=proof();row['full_replay_max_error']='1e-15';self.reject(p=[row])

    def test_dual_band_requires_both_distinct_channels(self):
        model=dict(card='mica-ir',implementation='test.seeker.v3',family='flare',channels=2,semi=False)
        self.reject(s=summary('mica-ir',channels=2),p=[proof()],model=model)
        self.reject(s=summary('mica-ir',channels=2),p=[proof(),proof()],model=model)

    def test_wrong_epochs_and_geometric_ambiguity_cannot_pass(self):
        for field,value in (('launch_age_s','1.4'),('decoy_age_s','0.09'),('time_s','9'),('matched','2'),('range_m','nan')):
            row=proof();row[field]=value;self.reject(p=[row])
        row=summary();row[7]='11.49';self.reject(s=row)

    def test_rf_requires_positive_kernel_energy_and_retained_cell_change(self):
        model=dict(card='rf',implementation='test.seeker.v3',family='chaff',channels=1,semi=False)
        for field,value in (('retained_cell_max_delta','0'),('native_energy_snr','0'),('residual_energy_snr','0'),('residual_energy_snr','40')):
            row=proof(radio=True);row[field]=value;self.reject(s=summary('rf','chaff'),p=[row],model=model)

    def test_semi_active_requires_actual_waveform_and_clock(self):
        model=dict(card='sarh',implementation='test.seeker.v3',family='chaff',channels=1,semi=True)
        for field in ('semi_active','illumination_waveform','illumination_clock'):
            row=proof(radio=True,semi=True);row[field]='0';self.reject(s=summary('sarh','chaff'),p=[row],model=model)

    def test_artifact_mutation_cannot_pass(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'receipt.csv';path.write_bytes(b'original')
            binding=dict(path=str(path),sha256=cm.sha(path));path.write_bytes(b'changed')
            with self.assertRaises(ValueError):cm.binding(binding)

    def test_duplicate_or_missing_population_cannot_pass(self):
        for rows in ([dict(card='a'),dict(card='a')],[dict(card='a')]):
            with self.assertRaises(ValueError):cm.exact({'a','b'},rows,'card')

    def test_source_epoch_mutation_cannot_pass(self):
        with self.assertRaises(ValueError):cm.require_epoch(dict(schema='fsim.sim-m03.countermeasure-source-epoch.v1',coverage_sha256='0'*64), '1'*64)

    def test_positive_thermal_and_both_dual_channels_preserve_saturation_and_miss(self):
        row=proof();row['saturated_pixels']='8'
        self.assertEqual(cm.validate_model(summary(),[row],dict(card='aim-9m',implementation='test.seeker.v3',family='flare',channels=1,semi=False))['terminal'],'expired')
        model=dict(card='mica-ir',implementation='test.seeker.v3',family='flare',channels=2,semi=False)
        self.assertEqual(len(cm.validate_model(summary('mica-ir',channels=2),[proof(),proof(1)],model)['channels']),2)

    def test_positive_rf_preserves_native_terminal(self):
        model=dict(card='rf',implementation='test.seeker.v3',family='chaff',channels=1,semi=False)
        self.assertTrue(cm.validate_model(summary('rf','chaff'),[proof(radio=True)],model)['passed'])

    def test_frozen_forty_second_end_preserves_native_in_flight(self):
        row=summary();row[14]='in_flight';row[15]=''
        self.assertEqual(cm.validate_model(row,[proof()],dict(card='aim-9m',implementation='test.seeker.v3',family='flare',channels=1,semi=False))['terminal'],'in_flight')

    def test_actual_ctest_report_requires_real_assertions_and_no_skip(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'ctest.xml'
            for tail in ('<skipped/>','<failure/>','<system-out>No tests ran</system-out>'):
                path.write_text('<testsuites><testsuite><testcase name="sensor_native.'+cm.CASE+'">'+tail+'</testcase></testsuite></testsuites>',encoding='utf-8')
                with self.assertRaises(ValueError):cm.actual_test_report(path)
            path.write_text('<testsuites><testsuite><testcase name="sensor_native.'+cm.CASE+'"><system-out>All tests passed (123 assertions in 1 test case)</system-out></testcase></testsuite></testsuites>',encoding='utf-8')
            self.assertEqual(cm.actual_test_report(path),'ctest-junit')

    def test_complete_join_requires_exact_both_presets_and_unchanged_raw_bytes(self):
        # Synthetic admission fixture only: no native execution is implied.
        import json
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            def artifact(name,value):
                path=root/name;path.parent.mkdir(parents=True,exist_ok=True)
                path.write_text(value,encoding='utf-8',newline='');return dict(path=name,sha256=cm.sha(path))
            sources=[artifact(name,'source fixture') for name in sorted(cm.SOURCE_PATHS)]
            stores=[];definitions=[]
            for kind,count,receiver in (('active_radar',8,'rf_signal.v1'),('semi_active_radar',4,'rf_signal.v1'),
                ('infrared',7,'thermal_quadrant.v1'),('imaging_infrared',5,'thermal_image.v1')):
                for index in range(count):
                    model='mica-ir' if kind=='imaging_infrared' and index==0 else kind+str(index)
                    dual=model=='mica-ir';policy=dict(implementation_id='test.seeker.v3',bindings=dict(receiver_model='thermal_dual_image.v1' if dual else receiver))
                    native=dict(seeker=kind,component_models=dict(components=dict(seeker=policy)))
                    source=artifact('assets/stores/'+model+'.json',json.dumps(native));sources.append(source)
                    stores.append(dict(card=model,**{'class':'aam'},card_path=source['path'],card_sha256=source['sha256'],components=dict(seeker=dict(implementation='test.seeker.v3'))))
                    definitions.append((model,'infrared' in kind,dual,kind=='semi_active_radar'))
            epoch=dict(schema='fsim.sim-m03.countermeasure-source-epoch.v1',main_head='e'*40,coverage_sha256='a'*64,sources=sources,presets=[])
            for preset in sorted(cm.PRESETS):
                base='build/'+preset+'/';traces=[];rows=[]
                for model,thermal,dual,semi in definitions:
                    stream=io.StringIO(newline='');writer=csv.DictWriter(stream,fieldnames=cm.PROBE_FIELDS);writer.writeheader()
                    for channel in range(2 if dual else 1):writer.writerow(proof(channel,not thermal,semi))
                    traces.append(dict(card=model,**artifact(base+model+'-countermeasure-skip4.csv',stream.getvalue())))
                    rows.append(summary(model,'flare' if thermal else 'chaff',2 if dual else 1))
                stream=io.StringIO(newline='');csv.writer(stream).writerows(rows)
                epoch['presets'].append(dict(preset=preset,binary=artifact(base+'tests/fsim_weapon_sensor_native_test.exe','binary fixture'),
                    runtime=[artifact(base+'fsim_session.dll','sdk fixture'),artifact(base+'libJSBSim.dll','jsbsim fixture')],
                    test_report=artifact(base+'ctest.xml','<testsuites><testsuite><testcase name="sensor_native.'+cm.CASE+'"><system-out>All tests passed (123 assertions in 1 test case)</system-out></testcase></testsuite></testsuites>'),
                    summary=artifact(base+'countermeasure-summary-skip4.csv',stream.getvalue()),traces=traces))
            with patch.object(cm.subprocess,'check_output',side_effect=lambda args,**kwargs:'main\n' if '--show-current' in args else 'e'*40+'\n'):
                joined=cm.admit(dict(stores=stores),'a'*64,epoch,root)
                self.assertEqual(len(joined['models']),48)
                # An ordinary exact-source export has no .git directory. Only
                # ancestry comes from main; every source/artifact remains bound
                # to the actual export used by native execution.
                checkout=root/'main-checkout'
                with patch.object(cm.subprocess,'check_output',side_effect=lambda args,**kwargs:'main\n' if '--show-current' in args else 'e'*40+'\n') as git:
                    exported=cm.admit(dict(stores=stores),'a'*64,epoch,root,git_source_root=checkout)
                    self.assertEqual(len(exported['models']),48)
                    self.assertTrue(all(call.kwargs['cwd']==checkout for call in git.call_args_list))
                missing=copy.deepcopy(epoch);missing['presets'].pop()
                with self.assertRaises(ValueError):cm.admit(dict(stores=stores),'a'*64,missing,root)
                duplicate=copy.deepcopy(epoch);duplicate['presets'][1]=duplicate['presets'][0]
                with self.assertRaises(ValueError):cm.admit(dict(stores=stores),'a'*64,duplicate,root)
                (root/epoch['presets'][0]['traces'][0]['path']).write_text('mutated',encoding='utf-8')
                with self.assertRaises(ValueError):cm.admit(dict(stores=stores),'a'*64,epoch,root)


if __name__=='__main__':unittest.main()
