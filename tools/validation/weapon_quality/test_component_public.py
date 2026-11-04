"""Synthetic admission refusal tests; these never count as installed execution."""
import copy
import csv
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import component_public as public
from component_matrix import COMPONENTS


class PublicAdmissionTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(prefix='m03-public-admission-',dir='D:/ENV/tmp')
        self.addCleanup(self.temp.cleanup);self.root=Path(self.temp.name)
        self.prefix=self.root/'prefix';(self.prefix/'bin').mkdir(parents=True)
        self.exe=self.root/'fixture.exe';self.exe.write_bytes(b'Synthetic identity only; never executed')
        modules=[self.exe]
        for name in ('libfsim.dll','libJSBSim.dll'):
            p=self.prefix/'bin'/name;p.write_bytes(b'Synthetic module identity '+name.encode());modules.append(p)
        self.prepared={p.name.casefold():public.binding(p) for p in modules}
        self.capture=dict(schema='fsim.sim-m03.public-aam-capture.v1',language='cpp',model='fixture',native_model='jsbsim:fixture',
            input_epoch_sha256='a'*64,weapon=3,target=2,checks=100,observations=1200,control_samples=1200,
            tracking_samples=1200,burning_samples=1200,damage_events=1,terminal='detonated',launch_s=10.,stop_s=50.,
            modules=[dict(path=str(p),sha256=public.sha(p)) for p in modules])
        self.trace=[dict(model='fixture',weapon=3,time=10+i/30,age=i/30,tracking=1,burning=1,controller=1,fin_count=4,
                         terminal=3 if i==1200 else 0) for i in range(1,1201)]
        self.events=[dict(model='fixture',weapon=3,serial=1,time=15.,kind='damage',source=3,target=2)]
        self.write()

    def write(self):
        for name,rows in (('trace.csv',self.trace),('events.csv',self.events)):
            with (self.root/name).open('w',encoding='utf-8',newline='') as stream:
                writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
        self.capture['files']=[dict(path=name,sha256=public.sha(self.root/name)) for name in ('trace.csv','events.csv')]
        public.save(self.root/'capture.json',self.capture)

    def validate(self):
        return public.validate_case(self.capture,self.root,'cpp','fixture','jsbsim:fixture','a'*64,self.prefix,self.prepared)

    def test_actual_paths_fixed_cadence_model_and_event_identity_are_joined(self):
        value=self.validate();self.assertEqual(value['observations'],1200);self.assertEqual(value['damage_events'],1)

    def test_foreign_model_language_epoch_terminal_or_zero_execution_refuse(self):
        original=copy.deepcopy(self.capture)
        for change in (dict(model='other'),dict(native_model='jsbsim:other'),dict(language='c99'),
                       dict(input_epoch_sha256='b'*64),dict(terminal='in_flight'),dict(checks=0),dict(observations=0),
                       dict(control_samples=True),dict(tracking_samples=0),dict(burning_samples=0),dict(damage_events=0)):
            self.capture={**original,**change}
            with self.subTest(change=change),self.assertRaises(ValueError):self.validate()

    def test_changed_loaded_module_or_duplicate_module_refuses(self):
        self.capture['modules'][1]=self.capture['modules'][0]
        with self.assertRaises(ValueError):self.validate()
        self.capture['modules'][1]=dict(path=str(self.prefix/'bin/libfsim.dll'),sha256=public.sha(self.prefix/'bin/libfsim.dll'))
        (self.prefix/'bin/libJSBSim.dll').write_bytes(b'Changed after preparation')
        with self.assertRaises(ValueError):self.validate()

    def test_old_capture_cannot_pair_with_newly_prepared_producer_or_changed_raw(self):
        module=self.prefix/'bin/libfsim.dll';module.write_bytes(b'New product at the same path')
        self.prepared[module.name.casefold()]=public.binding(module)
        with self.assertRaises(ValueError):self.validate()
        self.capture['modules'][1]['sha256']=public.sha(module)
        (self.root/'trace.csv').write_text('Replaced after native checked close',encoding='utf-8')
        with self.assertRaises(ValueError):self.validate()

    def test_explicit_facade_import_closure_includes_agent_and_shared_value_owners(self):
        paths={p.relative_to(public.ROOT).as_posix() for p in public.facade_paths()}
        for name in ('python/fsim/agent.py','python/fsim/_simulation/values.py','python/fsim/_state.py',
                     'python/fsim/_convert.py','python/fsim/fusion.py'):
            self.assertIn(name,paths)

    def test_prepared_binaries_facades_and_models_must_still_match_original_installer(self):
        record=dict(schema='fsim.sim-m03.installed-consumers.v1',passed=True,prefix=str(self.prefix),provenance=[],component_client_sources=[])
        def pair(source,product,body):
            source.parent.mkdir(parents=True,exist_ok=True);product.parent.mkdir(parents=True,exist_ok=True)
            source.write_bytes(body);product.write_bytes(body)
            return dict(source=str(source),installed=str(product),sha256=public.sha(source))
        for name in ('libfsim.dll','libJSBSim.dll'):
            record['provenance'].append(pair(self.root/'build/bin'/name,self.prefix/'bin'/name,b'Synthetic original '+name.encode()))
        facade=self.root/'python/fsim/agent.py'
        record['provenance'].append(pair(facade,self.prefix/'fsim/agent.py',b'Synthetic facade source'))
        for name in ('aam_public.cpp','aam_public.c','aam_public_capture.h','CMakeLists.txt'):
            record['component_client_sources'].append(pair(self.root/'tools/validation/weapon_quality'/name,
                self.root/'copied-consumer'/name,b'Synthetic consumer source'))
        pyd=self.prefix/'fsim/_native.pyd';pyd.write_bytes(b'Synthetic original extension')
        record['python_loaded_modules']=dict(native_python=str(pyd),native_python_sha256=public.sha(pyd))
        model=self.prefix/'share/flightsim/aircraft/stores/fixture/fixture.xml';model.parent.mkdir(parents=True)
        model.write_text('Synthetic installed model bytes',encoding='utf-8')
        producer=self.root/'producer.json';public.save(producer,dict(schema='fsim.sim-m02.production-install.v1',prefix=str(self.prefix),
            product_count=1,native_dll_sha256=public.sha(self.prefix/'bin/libfsim.dll'),products={'stores/fixture/fixture.xml':public.sha(model)}))
        record['producer']=public.binding(producer,self.root)
        with mock.patch.object(public,'facade_paths',return_value=[facade]):
            public.verify_installed(record,self.prefix,self.root)
            for product in (self.prefix/'bin/libfsim.dll',pyd,model):
                original=product.read_bytes();product.write_bytes(b'Replaced before component preparation')
                with self.subTest(product=product),self.assertRaises(ValueError):public.verify_installed(record,self.prefix,self.root)
                product.write_bytes(original)
            record['provenance'].pop()
            with self.assertRaises(ValueError):public.verify_installed(record,self.prefix,self.root)

    def test_equal_module_bytes_outside_installed_prefix_still_refuse(self):
        outside=self.root/'libfsim.dll';outside.write_bytes((self.prefix/'bin/libfsim.dll').read_bytes())
        self.capture['modules'][1]['path']=str(outside)
        with self.assertRaises(ValueError):self.validate()

    def test_summary_flags_cannot_replace_actual_target_damage_or_trace(self):
        self.events[0]['target']=99;self.write()
        with self.assertRaises(ValueError):self.validate()
        self.events[0]['target']=2;self.trace[-1]['terminal']=0;self.write()
        with self.assertRaises(ValueError):self.validate()
        self.trace[-1]['terminal']=3;self.trace[50]['tracking']=0;self.write()
        with self.assertRaises(ValueError):self.validate()

    def test_missing_reordered_or_nonfinite_sample_clock_refuses(self):
        original=copy.deepcopy(self.trace)
        for change in ('omit','reorder','future','nonfinite'):
            self.trace=copy.deepcopy(original)
            if change=='omit':self.trace.pop(12)
            elif change=='reorder':self.trace[12],self.trace[13]=self.trace[13],self.trace[12]
            elif change=='future':self.trace[12]['age']=1e99
            else:self.trace[12]['time']='nan'
            self.write()
            with self.subTest(change=change),self.assertRaises(ValueError):self.validate()

    def test_duplicate_nonfinite_json_and_excluded_paths_refuse_before_io(self):
        for text in ('{"passed":true,"passed":false}','{"value":NaN}'):
            path=self.root/'invalid.json';path.write_text(text,encoding='utf-8')
            with self.assertRaises(ValueError):public.read(path)
        with mock.patch.object(Path,'resolve',side_effect=AssertionError('Forbidden resolution')):
            with self.assertRaises(ValueError):public.checked_path('D:/ENV/tmp/.CoDeX/private.json')

    def batch_fixture(self):
        """Format-only72-row fixture. No executable or DLL is ever executed."""
        output=self.root/'run';output.mkdir();(self.prefix/'fsim').mkdir()
        pyd=self.prefix/'fsim/_native.pyd';pyd.write_bytes(b'Synthetic extension identity; never imported')
        clients={language:public.binding(self.exe,self.root) for language in public.LANGUAGES}
        installed=dict(prefix=str(self.prefix),component_clients=clients)
        installed_path=self.root/'installed.json';public.save(installed_path,installed)
        epoch=dict(schema='synthetic-format-epoch',sources=[]);epoch_path=output/'source-epoch.json';public.save(epoch_path,epoch)
        cells=[];executions=[];observations=[]
        (self.root/'assets/stores').mkdir(parents=True);(self.prefix/'share/flightsim/stores').mkdir(parents=True)
        for i in range(24):
            model='fixture-'+str(i);card=dict(id=model,model='jsbsim:'+model,seeker='active_radar')
            public.save(self.root/'assets/stores'/(model+'.json'),card)
            public.save(self.prefix/'share/flightsim/stores'/(model+'.json'),card)
            for component in COMPONENTS:
                cells.append(dict(model=model,component=component,native_model=card['model'],implementation_id='synthetic.'+component,
                    mechanism_id='synthetic.'+component,card_sha256='a'*64,parameter_sha256='b'*64))
            selection=dict(carrier='synthetic-carrier',station=1)
            for language in public.LANGUAGES:
                directory=output/language/model;directory.mkdir(parents=True)
                prepared={name.casefold():public.binding(self.prefix/'bin'/name,self.root) for name in ('libfsim.dll','libJSBSim.dll')}
                prepared[self.exe.name.casefold()]=public.binding(self.exe,self.root)
                if language=='python':prepared[pyd.name.casefold()]=public.binding(pyd,self.root)
                capture=copy.deepcopy(self.capture);capture.update(model=model,native_model=card['model'],language=language,
                    input_epoch_sha256=public.sha(epoch_path))
                capture['modules']=[dict(path=str(public.bound(v,self.root)),sha256=v['sha256']) for v in prepared.values()]
                rows=[]
                for row in self.trace:
                    item={**row,'model':model}
                    if language=='python':item['terminal']='detonated' if row['terminal']==3 else 'in_flight'
                    rows.append(item)
                for name,data in (('trace.csv',rows),('events.csv',[{**self.events[0],'model':model}])):
                    with (directory/name).open('w',encoding='utf-8',newline='') as stream:
                        writer=csv.DictWriter(stream,fieldnames=list(data[0]));writer.writeheader();writer.writerows(data)
                capture['files']=[dict(path=name,sha256=public.sha(directory/name)) for name in ('trace.csv','events.csv')]
                public.save(directory/'capture.json',capture);(directory/'run.log').write_text('Synthetic run format only',encoding='utf-8')
                if language=='python':public.save(directory/'world-health.json',dict(rows=[
                    [r['time'],1,'synthetic',False,*([0.]*13)] for r in self.trace],invalid_entities=[]))
                with mock.patch.object(public,'ROOT',self.root):
                    command=public.invocation(language,model,card,selection,self.exe,self.prefix,directory,public.sha(epoch_path))
                executions.append(dict(model=model,language=language,exit_code=0,elapsed_s=1.,command=command,
                    environment=public.sdk_environment(self.prefix),prepared=list(prepared.values()),raw=[public.binding(directory/name,self.root)
                        for name in ('capture.json','trace.csv','events.csv','run.log')+ (('world-health.json',) if language=='python' else ())]))
                observations.append(public.validate_case(capture,directory,language,model,card['model'],public.sha(epoch_path),self.prefix,prepared,self.root))
        record=dict(schema='fsim.sim-m03.component-public-run.v1',passed=True,failures=[],installed=public.binding(installed_path,self.root),
            source_epoch=public.binding(epoch_path,self.root),executions=executions,models=observations)
        return output,installed_path,epoch,cells,record

    def test_readonly_admission_reconstructs72_executions_and_rejects_aggregate_omission(self):
        output,installed,epoch,cells,record=self.batch_fixture();run=output/'run.json'
        def join():
            public.save(run,record)
            with mock.patch.object(public,'ROOT',self.root),mock.patch.object(public,'verify_installed'),\
                 mock.patch.object(public,'freeze_sources',return_value=epoch),mock.patch.object(public,'derive_cells',return_value=cells),\
                 mock.patch.object(public,'carrier_for',return_value=dict(carrier='synthetic-carrier',station=1)):
                return public.admit({},run,installed,self.root)
        proof=join();self.assertEqual(len(proof['models']),216)
        self.assertTrue(all(r['public_installed']['tests']==3 for r in proof['models']))
        removed=record['executions'].pop()
        with self.assertRaises(ValueError):join()
        record['executions'].append(removed);record['executions'][0]['command'][-1]='foreign-output'
        with self.assertRaises(ValueError):join()


if __name__=='__main__':unittest.main(verbosity=2)
