"""Negative controls for actual per-model native chain admission."""
import copy
import csv
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import full_chain as chain


class SceneSourceClosureTests(unittest.TestCase):
    def setUp(self):
        self.temporary=tempfile.TemporaryDirectory(prefix='m03-scene-sources-',dir='D:/ENV/tmp')
        self.addCleanup(self.temporary.cleanup);self.root=Path(self.temporary.name)
        self.coverage=dict(stores=[dict(card='missile',model='missile',card_path='assets/stores/missile.json',**{'class':'aam'})])
        grid='{"schema":"fsim.airframe-aerodynamics.v1","method":"vlm"}'
        self.grid='aero/vlm-carriage-'+hashlib.sha256(grid.encode()).hexdigest()+'.json'
        self.put('aircraft/stores/missile/'+self.grid,grid)
        self.put('assets/stores/missile.json',json.dumps(dict(id='missile',model='jsbsim:missile',
            parameters=dict(airframe_aerodynamics_version=dict(value=1,unit='1',source='typed-carriage')),
            sources={'typed-carriage':dict(url=self.grid)})))
        self.put('aircraft/missile/missile.toml','[aircraft]\ncategory="missile"\n')
        self.put('aircraft/stores/missile/missile.xml','<fdm_config><propulsion><engine file="motor"><thruster file="nozzle"/></engine></propulsion><system file="control"/></fdm_config>')
        self.put('aircraft/stores/missile/Engines/motor.xml','<rocket_engine><isp>245</isp></rocket_engine>')
        self.put('aircraft/stores/missile/Engines/nozzle.xml','<nozzle><area>0</area></nozzle>')
        self.put('aircraft/stores/missile/Systems/control.xml','<system/>')
        self.put('aircraft/carrier/carrier.toml','[aircraft]\ncategory="aircraft"\n')
        self.put('aircraft/carrier/carrier.xml','<fdm_config/>')
        self.put('aircraft/carrier/payloads.json','{"aircraft":"carrier","payloads":[{"card":"radar"}],"loadouts":[]}')
        self.put('assets/payloads/radar.json','{"id":"radar","family":"air_radar"}')
        self.put('aircraft/carrier/carrier.damage.json','{"sections":[{"name":"left wing"}],"liquid_geometry_v1":{"file":"carrier.liquid-v1.json","required":true}}')
        self.put('aircraft/carrier/carrier.liquid-v1.json','{"schema":"synthetic physical-input binding only"}')
        self.put('tools/models/hangar/hangar/storebodies.py','from . import local_motor\n')
        self.put('tools/models/hangar/hangar/local_motor.py','VALUE=1\n')
        self.put('cmake/JsbsimPatches.cmake','# native patch input\n')
        self.patch=mock.patch.object(chain,'GENERATION_SOURCE_PATHS',('tools/models/hangar/hangar/storebodies.py','cmake/JsbsimPatches.cmake'),create=True)
        self.patch.start();self.addCleanup(self.patch.stop)

    def put(self,name,value):
        path=self.root/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_text(value,encoding='utf-8');return path

    def epoch(self):
        return {name:chain.digest(self.root/name) for name in chain.scene_input_paths(self.coverage,self.root,airframes=('carrier',))}

    def test_external_engine_thruster_system_and_local_generator_are_bound(self):
        before=self.epoch()
        for name in ('aircraft/stores/missile/Engines/motor.xml','aircraft/stores/missile/Engines/nozzle.xml',
                     'aircraft/stores/missile/Systems/control.xml','tools/models/hangar/hangar/local_motor.py','cmake/JsbsimPatches.cmake'):
            self.assertIn(name,before)
            path=self.root/name;old=path.read_text();path.write_text(old+'\n')
            self.assertNotEqual(before,self.epoch());path.write_text(old)

    def test_actual_carrier_target_fit_payload_and_material_are_bound(self):
        before=self.epoch()
        for name in ('aircraft/carrier/carrier.xml','aircraft/carrier/carrier.toml','aircraft/carrier/payloads.json',
                     'assets/payloads/radar.json','aircraft/carrier/carrier.damage.json','aircraft/carrier/carrier.liquid-v1.json'):
            self.assertIn(name,before)
            path=self.root/name;old=path.read_text();path.write_text(old+'\n')
            self.assertNotEqual(before,self.epoch());path.write_text(old)

    def test_new_optional_runtime_input_and_piece_are_epoch_changes(self):
        before=self.epoch();self.put('aircraft/carrier/rcs.bin','new actual model input')
        self.assertNotEqual(before,self.epoch());before=self.epoch()
        self.put('aircraft/carrier/pieces/carrier-left-wing/carrier-left-wing.xml','<fdm_config/>')
        self.assertNotEqual(before,self.epoch())

    def test_missing_resource_parent_escape_and_excluded_reference_refuse(self):
        path=self.root/'aircraft/stores/missile/missile.xml'
        for name in ('missing','../../outside','.CoDeX/never-opened'):
            path.write_text('<fdm_config><propulsion><engine file="'+name+'"/></propulsion></fdm_config>')
            with self.subTest(name=name),self.assertRaises(ValueError):self.epoch()

    def test_excluded_root_refuses_before_metadata(self):
        with mock.patch.object(Path,'resolve',side_effect=AssertionError('Forbidden metadata')):
            with self.assertRaisesRegex(ValueError,'Excluded'):
                chain.scene_input_paths(self.coverage,Path('D:/ENV/tmp/.CoDeX/never-opened'))

    def test_excluded_xml_reference_is_pruned_before_reference_metadata(self):
        self.put('aircraft/stores/missile/missile.xml','<fdm_config><propulsion><engine file=".CoDeX/never-opened"/></propulsion></fdm_config>')
        original=Path.lstat
        def guarded(path,*args,**kwargs):
            if any(part.casefold()=='.codex' for part in path.parts):raise AssertionError('Excluded metadata attempted')
            return original(path,*args,**kwargs)
        with mock.patch.object(Path,'lstat',guarded),self.assertRaisesRegex(ValueError,'Excluded'):
            self.epoch()

    def test_native_engine_and_system_search_priority_and_vendor_fallback(self):
        self.put('aircraft/stores/missile/motor.xml','<rocket_engine><wrong/></rocket_engine>')
        self.put('third_party/jsbsim/engine/motor.xml','<rocket_engine><fallback/></rocket_engine>')
        paths=self.epoch()
        self.assertIn('aircraft/stores/missile/Engines/motor.xml',paths)
        self.assertNotIn('aircraft/stores/missile/motor.xml',paths)
        self.assertNotIn('third_party/jsbsim/engine/motor.xml',paths)
        (self.root/'aircraft/stores/missile/Engines/motor.xml').unlink()
        self.assertIn('third_party/jsbsim/engine/motor.xml',self.epoch())
        self.put('aircraft/stores/missile/control.xml','<system/>')
        paths=self.epoch()
        self.assertIn('aircraft/stores/missile/control.xml',paths)
        self.assertNotIn('aircraft/stores/missile/Systems/control.xml',paths)

    def test_recursive_xml_reference_cycle_is_finite_and_each_file_bound_once(self):
        self.put('aircraft/stores/missile/Systems/control.xml','<system><system file="other"/></system>')
        self.put('aircraft/stores/missile/Systems/other.xml','<system><system file="control"/></system>')
        paths=chain.scene_input_paths(self.coverage,self.root,airframes=('carrier',))
        self.assertIn('aircraft/stores/missile/Systems/other.xml',paths)
        self.assertEqual(len(paths),len(set(paths)))

    def test_actual_typed_carriage_table_is_bound_and_content_address_verified(self):
        path='aircraft/stores/missile/'+self.grid
        before=self.epoch();self.assertIn(path,before)
        (self.root/path).write_text('{"schema":"changed numeric product"}')
        with self.assertRaises(ValueError):self.epoch()

    def test_missing_carriage_marker_owner_file_and_foreign_url_refuse(self):
        path=self.root/'assets/stores/missile.json';original=path.read_text()
        for mutation in ('marker','owner','wrongroot','escape','excluded','missing','wrongmodel','version','unit'):
            card=json.loads(original)
            if mutation=='marker':card['parameters'].pop('airframe_aerodynamics_version')
            elif mutation=='owner':card['parameters']['airframe_aerodynamics_version']['source']='another-source'
            elif mutation=='wrongroot':card['sources']['typed-carriage']['url']='other/'+self.grid
            elif mutation=='escape':card['sources']['typed-carriage']['url']='../'+self.grid
            elif mutation=='excluded':card['sources']['typed-carriage']['url']='.CoDeX/private.json'
            elif mutation=='missing':card['sources']['typed-carriage']['url']='aero/vlm-carriage-'+'0'*64+'.json'
            elif mutation=='wrongmodel':card['model']='jsbsim:foreign'
            elif mutation=='version':card['parameters']['airframe_aerodynamics_version']['value']=True
            else:card['parameters']['airframe_aerodynamics_version']['unit']='kg'
            path.write_text(json.dumps(card))
            with self.subTest(mutation=mutation),self.assertRaises(ValueError):self.epoch()
        path.write_text(original)

    def test_store_optional_runtime_inputs_and_calibration_appearance_change_epoch(self):
        before=self.epoch()
        for relative,text in (
            ('aircraft/stores/missile/payloads.json','{"aircraft":"missile","payloads":[]}'),
            ('aircraft/stores/missile/missile.damage.json','{"sections":[]}'),
            ('aircraft/stores/missile/missile.liquid-v1.json','{"synthetic":true}'),
            ('aircraft/missile/calibration.toml','[calibration]\n'),
            ('aircraft/carrier/calibration.toml','[calibration]\n')):
            path=self.put(relative,text)
            self.assertNotEqual(before,self.epoch());path.unlink()
        self.assertEqual(before,self.epoch())

    def test_store_declared_liquid_and_piece_runtime_inputs_are_not_omitted(self):
        self.put('aircraft/stores/missile/missile.damage.json',json.dumps(dict(sections=[dict(name='wing')],
            liquid_geometry_v1=dict(file='missile.liquid-v1.json',required=True))))
        with self.assertRaises(ValueError):self.epoch()
        self.put('aircraft/stores/missile/missile.liquid-v1.json','{}')
        piece='aircraft/stores/missile/pieces/missile-wing/missile-wing'
        self.put(piece+'.xml','<fdm_config/>');before=self.epoch()
        for suffix,text in (('.damage.json','{"sections":[]}'),('.liquid-v1.json','{}')):
            path=self.put(piece+suffix,text)
            self.assertNotEqual(before,self.epoch());path.unlink()

    def test_optional_store_links_cannot_escape_or_read_excluded_paths(self):
        self.put('aircraft/stores/missile/missile.damage.json',json.dumps(dict(sections=[],
            liquid_geometry_v1=dict(file='.CoDeX/private.json',required=True))))
        original=Path.lstat
        def guarded(path,*args,**kwargs):
            if any(part.casefold()=='.codex' for part in path.parts):raise AssertionError('Excluded metadata attempted')
            return original(path,*args,**kwargs)
        with mock.patch.object(Path,'lstat',guarded),self.assertRaises(ValueError):self.epoch()


class FullChainTests(unittest.TestCase):
    def fixture(self):
        model = 'model'
        cells = [dict(model=model, component=c, implementation_id='equipment.model.'+c,
                      mechanism_id='native.'+c) for c in chain.COMPONENTS]
        mechanisms = [dict(card=model, component=c['component'], implementation=c['implementation_id'],
                           mechanism=c['mechanism_id']) for c in cells]
        first = dict(card=model, weapon='3', generation='1', time='20', age='0.1', phase='3',
            tracking='1', burning='1', thermal_status='1', rf_status='2', guidance_status='1',
            guidance_flags='8', control_status='1', controller_time='19.995', fin_count='4',
            propulsion_flags='1', fuel_kg='5', thrust_n='500', fuze_flags='9', burst_time='0',
            fragments='0', damage_records='0', component_energy_j='0', alpha='.01', beta='.02',
            speed='400', fx='-5', fy='1', fz='2', l='0', m='1', n='0')
        last = dict(first, time='21', age='1.1', controller_time='20.995', fuze_flags='27', burst_time='20.9',
                    fragments='100', damage_records='1', component_energy_j='2')
        summary = dict(card=model, weapon='3', generation='1', released='1', damage='1',
            terminal='detonated', terminal_time='20.9', samples='2', burst_time='20.9',
            fragments='100', damage_records='1', component_energy_j='2')
        summary.update({c:'1' for c in chain.COMPONENTS})
        return model, [summary], [first, last], mechanisms, cells

    def test_each_actual_component_stage_and_native_lifetime_are_required(self):
        valid = self.fixture()
        self.assertEqual(chain.validate_case(*valid)['samples'], 2)
        for component in chain.COMPONENTS:
            current = copy.deepcopy(valid)
            current[1][0][component] = '0'
            with self.subTest(component=component), self.assertRaises(ValueError):
                chain.validate_case(*current)
        for field in ('card', 'weapon', 'generation'):
            current = copy.deepcopy(valid)
            current[2][1][field] = 'different' if field == 'card' else '4'
            with self.subTest(field=field), self.assertRaises(ValueError):
                chain.validate_case(*current)

    def test_actual_measurements_cannot_be_replaced_by_summary_flags(self):
        variants = (
            dict(tracking='0'), dict(guidance_status='0'), dict(guidance_flags='0'),
            dict(control_status='0'), dict(fin_count='0'), dict(propulsion_flags='0'),
            dict(thrust_n='0'), dict(fuze_flags='0'), dict(fragments='0'),
            dict(component_energy_j='0'))
        for patch in variants:
            current = self.fixture()
            for row in current[2]:
                row.update(patch)
            with self.subTest(patch=patch), self.assertRaises(ValueError):
                chain.validate_case(*current)

    def test_empty_duplicate_reordered_nonfinite_and_missing_outcomes_refuse(self):
        for mutation in ('empty', 'count', 'reordered', 'nonfinite', 'terminal', 'damage', 'work', 'burst'):
            current = list(self.fixture())
            if mutation == 'empty': current[2] = []
            elif mutation == 'count': current[1][0]['samples'] = '3'
            elif mutation == 'reordered': current[2].reverse()
            elif mutation == 'nonfinite': current[2][0]['fx'] = 'nan'
            elif mutation == 'terminal': current[1][0]['terminal'] = 'in_flight'
            elif mutation == 'damage': current[1][0]['damage'] = '0'
            elif mutation == 'work': current[1][0]['component_energy_j'] = '0'
            else: current[1][0]['burst_time'] = '0'
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                chain.validate_case(*current)

    def test_exact_selected_mechanisms_cannot_be_omitted_duplicated_or_renamed(self):
        for mutation in ('omit', 'duplicate', 'alias', 'foreign'):
            current = self.fixture()
            if mutation == 'omit': current[3].pop()
            elif mutation == 'duplicate': current[3][-1] = current[3][0].copy()
            elif mutation == 'alias': current[3][0]['mechanism'] = 'new equipment alias'
            else: current[3][0]['implementation'] = 'equipment.other.seeker'
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                chain.validate_case(*current)

    def test_future_prelaunch_stale_and_inconsistent_controller_epochs_refuse(self):
        for patch in (dict(controller_time='1e99'), dict(controller_time='0'),
                      dict(controller_time='19.8'), dict(age='9')):
            current = self.fixture()
            current[2][1].update(patch)
            with self.subTest(patch=patch), self.assertRaises(ValueError):
                chain.validate_case(*current)

    def test_actual_nonzero_nested_ctest_case_must_pass(self):
        base = Path(tempfile.mkdtemp(prefix='m03-chain-admission-', dir='D:/ENV/tmp'))
        path = base/'junit.xml'
        def write(body):
            path.write_text('<testsuite><testcase name="aam_full_chain.model" time="1" status="run">'+body+
                            '</testcase></testsuite>', encoding='utf-8')
        write('<system-out>All tests passed (28 assertions in 1 test case)</system-out>')
        self.assertEqual(chain.executed_case(path, 'model')['assertions'], 28)
        for body in ('<skipped/>', '<failure/>', '<error/>', '<system-out>0 tests</system-out>',
                     '<system-out>All tests passed (0 assertions in 1 test case)</system-out>'):
            write(body)
            with self.subTest(body=body), self.assertRaises(ValueError):
                chain.executed_case(path, 'model')
        with self.assertRaises(ValueError):
            chain.executed_case(path, 'foreign')


class FullChainProvenanceTests(unittest.TestCase):
    """Synthetic file controls, never substituted for actual native captures."""
    def setUp(self):
        self.temporary=tempfile.TemporaryDirectory(prefix='m03-chain-links-',dir='D:/ENV/tmp')
        self.addCleanup(self.temporary.cleanup)
        self.root=Path(self.temporary.name)
        source=self.root/'source.cpp';source.write_text('synthetic source epoch')
        self.epoch=dict(schema='fsim.sim-m03.full-chain-source-epoch.v1',sources=[chain.binding(source,self.root)])
        self.epoch_path=self.root/'epoch.json';self.epoch_path.write_text(json.dumps(self.epoch))
        self.model='model';_,summary,trace,mechanisms,cells=FullChainTests().fixture()
        self.cells=[dict(c,native_model='jsbsim:model',card_sha256='a'*64,parameter_sha256='b'*64) for c in cells]
        self.patch=mock.patch.object(chain,'derive_cells',return_value=self.cells);self.patch.start();self.addCleanup(self.patch.stop)
        self.patch_epoch=mock.patch.object(chain,'freeze_sources',return_value=self.epoch);self.patch_epoch.start();self.addCleanup(self.patch_epoch.stop)
        self.runs={}
        for preset in chain.PRESETS:
            binary=self.root/'build'/preset/'tests';binary.mkdir(parents=True)
            for name in ('fsim_aam_full_chain_native_test.exe','libfsim.dll','libJSBSim.dll'):
                (binary/name).write_bytes(('synthetic '+preset+name).encode())
            directory=self.root/'build'/preset/'validation/aam_full_chain'/self.model;directory.mkdir(parents=True)
            records=dict(summary=summary,trace=trace,components=mechanisms,
                fins=[dict(card=self.model,weapon='3',generation='1',time=time,fin_id=str(fin))
                      for time in ('20','21') for fin in range(4)],
                kinematics=[dict(card=self.model,weapon='3',generation='1',time='20',age='.1')],
                events=[dict(card=self.model,weapon='3',generation='1',time='20.9',serial='1')])
            for name,values in records.items():
                with (directory/(name+'.csv')).open('w',newline='') as stream:
                    writer=csv.DictWriter(stream,fieldnames=list(values[0]));writer.writeheader();writer.writerows(values)
            junit=self.root/(preset+'.xml');junit.write_text('<testsuite><testcase name="aam_full_chain.model" time="1" status="run">'
                '<system-out>All tests passed (28 assertions in 1 test case)</system-out></testcase></testsuite>')
            run=chain.prepare_preset(self.epoch_path,preset,self.root)
            run.update(junit=str(junit));self.runs[preset]=run
            capture=dict(schema='fsim.sim-m03.aam-chain-capture.v1',complete=True,admission_eligible=True,
                input_epoch_sha256=chain.digest(self.epoch_path),model=self.model,native_model='jsbsim:model',
                weapon='3',generation='1',terminal='detonated',terminal_time_s=20.9,launch_time_s=19.9,samples=2,
                executable=chain.binding(binary/'fsim_aam_full_chain_native_test.exe',self.root),
                runtime_modules=[chain.binding(binary/name,self.root) for name in ('libJSBSim.dll','libfsim.dll')],
                files=[dict(path=name+'.csv',sha256=chain.digest(directory/(name+'.csv')),
                            bytes=(directory/(name+'.csv')).stat().st_size) for name in records])
            (directory/'capture.json').write_text(json.dumps(capture))

    def admit(self):
        return chain.admit({},self.epoch,self.runs,self.root)

    def change_capture(self,fn,preset=None):
        preset=preset or chain.PRESETS[0]
        path=Path(self.runs[preset]['directory'])/self.model/'capture.json'
        value=json.loads(path.read_text());fn(value);path.write_text(json.dumps(value))

    def test_current_capture_matches_all_original_files_and_actual_runtime(self):
        self.assertEqual(len(self.admit()['models']),9)

    def test_static_session_omits_unused_sdk_dll_but_requires_actual_jsbsim(self):
        for preset in chain.PRESETS:
            self.change_capture(lambda value:value.update(runtime_modules=[item for item in value['runtime_modules']
                if Path(item['path']).name=='libJSBSim.dll']),preset)
        self.assertEqual(len(self.admit()['models']),9)
        self.change_capture(lambda value:value.update(runtime_modules=[item for item in
            self.runs[chain.PRESETS[0]]['runtime'] if Path(item['path']).name=='libfsim.dll']))
        with self.assertRaisesRegex(ValueError,'did not observe native JSBSim'):self.admit()

    def test_old_trace_cannot_be_paired_with_new_epoch_or_runtime_receipt(self):
        self.epoch['sources'][0]['sha256']='c'*64;self.epoch_path.write_text(json.dumps(self.epoch))
        for run in self.runs.values():run['source_epoch']['sha256']=chain.digest(self.epoch_path)
        with self.assertRaises(ValueError):self.admit()

    def test_changed_loaded_dll_refuses_even_with_refreshed_prepared_binding(self):
        run=self.runs[chain.PRESETS[0]]
        dll=self.root/'build'/chain.PRESETS[0]/'tests/libJSBSim.dll';dll.write_bytes(b'changed producer, same EXE')
        for item in run['runtime']:
            if Path(item['path']).name=='libJSBSim.dll':item['sha256']=chain.digest(dll)
        with self.assertRaises(ValueError):self.admit()

    def test_omitted_and_changed_csv_files_or_hashes_refuse(self):
        for name in ('summary','trace','components','fins','kinematics','events'):
            with self.subTest(file=name):
                run=self.runs[chain.PRESETS[0]];path=Path(run['directory'])/self.model/(name+'.csv')
                before=path.read_bytes();path.write_bytes(before+b'changed bytes\n')
                try:
                    with self.assertRaises(ValueError):self.admit()
                finally:path.write_bytes(before)
        self.change_capture(lambda value:value['files'].pop())
        with self.assertRaises(ValueError):self.admit()

    def test_unidentifiable_diagnostic_capture_and_wrong_terminal_refuse(self):
        for key,value in (('admission_eligible',False),('model','foreign'),('weapon','4'),
                          ('terminal','expired'),('terminal_time_s',22),('samples',3)):
            path=Path(self.runs[chain.PRESETS[0]]['directory'])/self.model/'capture.json';before=path.read_bytes()
            try:
                self.change_capture(lambda record:record.update({key:value}))
                with self.subTest(key=key),self.assertRaises(ValueError):self.admit()
            finally:path.write_bytes(before)

    def test_omitted_row_and_omitted_file_refuse(self):
        run=self.runs[chain.PRESETS[0]];folder=Path(run['directory'])/self.model
        path=folder/'trace.csv';lines=path.read_text().splitlines();path.write_text('\n'.join(lines[:-1])+'\n')
        self.change_capture(lambda value:[item.update(sha256=chain.digest(path),bytes=path.stat().st_size)
                                         for item in value['files'] if item['path']=='trace.csv'])
        with self.assertRaises(ValueError):self.admit()
        (folder/'fins.csv').unlink()
        with self.assertRaises((ValueError,FileNotFoundError)):self.admit()

    def test_missing_auxiliary_fin_row_refuses_even_with_refreshed_file_hash(self):
        path=Path(self.runs[chain.PRESETS[0]]['directory'])/self.model/'fins.csv'
        lines=path.read_text().splitlines();path.write_text('\n'.join(lines[:-1])+'\n')
        self.change_capture(lambda value:[item.update(sha256=chain.digest(path),bytes=path.stat().st_size)
                                         for item in value['files'] if item['path']=='fins.csv'])
        with self.assertRaisesRegex(ValueError,'actual fin observations'):self.admit()

    def test_foreign_auxiliary_model_cannot_be_hidden_by_refreshed_file_hash(self):
        path=Path(self.runs[chain.PRESETS[0]]['directory'])/self.model/'events.csv'
        path.write_text(path.read_text().replace('model,','foreign,'))
        self.change_capture(lambda value:[item.update(sha256=chain.digest(path),bytes=path.stat().st_size)
                                         for item in value['files'] if item['path']=='events.csv'])
        with self.assertRaisesRegex(ValueError,'auxiliary native model/lifetime'):self.admit()

    def test_foreign_and_excluded_paths_refuse_before_io(self):
        with mock.patch.object(Path,'resolve',side_effect=AssertionError('No excluded resolution')):
            with self.assertRaises(ValueError):chain.checked_path('D:/fixture/.CoDeX/capture.json')
        self.change_capture(lambda value:value['files'][0].update(path='../foreign.csv'))
        with self.assertRaises(ValueError):self.admit()

    def test_duplicate_capture_metadata_refuses(self):
        path=Path(self.runs[chain.PRESETS[0]]['directory'])/self.model/'capture.json'
        path.write_text(path.read_text().replace('"complete": true','"complete": true, "complete": true'))
        with self.assertRaisesRegex(ValueError,'Duplicate native JSON field'):self.admit()

    def test_duplicate_csv_header_refuses_even_with_refreshed_file_hash(self):
        path=Path(self.runs[chain.PRESETS[0]]['directory'])/self.model/'trace.csv'
        lines=path.read_text().splitlines()
        path.write_text('\n'.join(line+(','+'time' if i==0 else ',20') for i,line in enumerate(lines))+'\n')
        self.change_capture(lambda value:[item.update(sha256=chain.digest(path),bytes=path.stat().st_size)
                                         for item in value['files'] if item['path']=='trace.csv'])
        with self.assertRaisesRegex(ValueError,'CSV header'):self.admit()

    def test_exact_one_case_and_positive_finite_ctest_status_required(self):
        path=Path(self.runs[chain.PRESETS[0]]['junit']);before=path.read_text()
        for text in (before.replace('1 test case','2 test cases'),before.replace('time="1"','time="nan"'),
                     before.replace('status="run"','status="notrun"'),before.replace(' status="run"','')):
            path.write_text(text)
            with self.subTest(text=text),self.assertRaises(ValueError):self.admit()
        path.write_text(before)


class FullChainReportJoinTests(unittest.TestCase):
    """Test report joining separately from native collector numeric controls."""
    def setUp(self):
        import report
        self.report=report
        self.temporary=tempfile.TemporaryDirectory(prefix='m03-chain-report-',dir='D:/ENV/tmp')
        self.addCleanup(self.temporary.cleanup);self.root=Path(self.temporary.name)
        self.epoch=self.root/'native-full-chain-source-epoch.json';self.epoch.write_text('{}')
        self.proof_path=self.root/'native-full-chain.json';self.runs_path=self.root/'native-full-chain-runs.json'
        self.proof=dict(models=[dict(model='model'+str(m),component=c) for m in range(24) for c in chain.COMPONENTS])
        self.proof_path.write_text(json.dumps(self.proof))
        self.campaign=dict(native_full_chain=dict(source_epoch=chain.binding(self.epoch,self.root),
            runs={},proof=chain.binding(self.proof_path,self.root)),full_regression={},commands=[])
        self.runs={}
        for preset in chain.PRESETS:
            folder=self.root/preset;folder.mkdir();junit=folder/'junit.xml';junit.write_text('synthetic actual-JUnit binding only')
            regression=folder/'regression.json';regression.write_text(json.dumps(dict(junit_sha256=chain.digest(junit))))
            self.runs[preset]=dict(junit=str(junit))
            self.campaign['full_regression'][preset]=dict(path=str(regression),sha256=chain.digest(regression))
            self.campaign['commands'].append(dict(label=preset+'-full',exit_code=0,
                environment=dict(FSIM_AAM_CHAIN_EPOCH=str(self.epoch))))
        self.runs_path.write_text(json.dumps(self.runs))
        # Real campaign records contain absolute paths, while component proof
        # references may use source-relative paths.
        for key,path in (('source_epoch',self.epoch),('runs',self.runs_path),('proof',self.proof_path)):
            self.campaign['native_full_chain'][key]=dict(path=str(path),sha256=chain.digest(path))
        self.components=dict(records=[dict(model=row['model'],component=row['component'],gates=dict(native=dict(
            artifact=chain.binding(self.proof_path,self.root),row=index))) for index,row in enumerate(self.proof['models'])])

    def join(self):
        with mock.patch.object(self.report,'admit_full_chain',return_value=self.proof):
            return self.report.join_native_chain({},self.root,self.root,self.campaign,self.components)

    def test_report_requires_every_native_cell_to_reference_current_campaign(self):
        self.assertEqual(self.join()['cells'],216)
        for field,value in (('row',1),('artifact',dict(path='foreign.json',sha256='a'*64))):
            record=self.components['records'][0]['gates']['native'];before=copy.deepcopy(record)
            try:
                record[field]=value
                with self.subTest(field=field),self.assertRaises(ValueError):self.join()
            finally:self.components['records'][0]['gates']['native']=before

    def test_missing_native_row_cannot_be_hidden_by_component_pass(self):
        self.components['records'].pop()
        with self.assertRaises(ValueError):self.join()

    def test_epoch_must_have_been_supplied_to_actual_both_full_regressions(self):
        for command in self.campaign['commands']:
            previous=command['environment'];command['environment']={'FSIM_AAM_CHAIN_EPOCH':str(self.root/'other-epoch.json')}
            try:
                with self.assertRaises(ValueError):self.join()
            finally:command['environment']=previous

    def test_native_capture_cannot_use_foreign_campaign_regression_output(self):
        self.runs[chain.PRESETS[0]]['junit']=str(self.root/'foreign.xml')
        self.runs_path.write_text(json.dumps(self.runs))
        self.campaign['native_full_chain']['runs']['sha256']=chain.digest(self.runs_path)
        with self.assertRaises(ValueError):self.join()

    def test_changed_full_junit_or_changed_native_receipt_refuses(self):
        junit=self.root/chain.PRESETS[0]/'junit.xml';before=junit.read_bytes();junit.write_bytes(b'new foreign output')
        with self.assertRaises(ValueError):self.join()
        junit.write_bytes(before)
        self.proof_path.write_text(json.dumps(dict(models=[])))
        self.campaign['native_full_chain']['proof']['sha256']=chain.digest(self.proof_path)
        with self.assertRaises(ValueError):self.join()


if __name__ == '__main__':
    unittest.main(verbosity=2)
