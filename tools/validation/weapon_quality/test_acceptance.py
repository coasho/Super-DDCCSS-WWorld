"""Independent negative controls for complete M03 admission, not physical tests."""
import copy
import os
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import patch

from acceptance import admit_models, reconcile_inventory, reconcile_source_review, verify_artifact, reconcile_population, reconcile_world_health, reconcile_model_products, canonical_product_key, canonical_product_map
import cadence
from model_methods import method_suite


def product_fixture(category='fixed-wing',directory='plane'):
    hashes={directory+'/plane.xml':'a'*64,directory+'/Engines/engine.xml':'c'*64,
            'plane/plane.glb':'d'*64,'plane/plane.glb.manifest':'e'*64}
    visuals=['plane/plane.glb','plane/plane.glb.manifest']
    audit=dict(xml_sha256='a'*64)
    if category=='fixed-wing':
        hashes[directory+'/plane.damage.json']='b'*64;audit['damage_sha256']='b'*64
    if category!='store':
        hashes['plane/plane.liquid-v1.json']='f'*64;visuals.append('plane/plane.liquid-v1.json')
    row=dict(name='plane',category=category,product_directory=directory,products=dict(hashes),
        normal_model=dict(regenerated=True,products=visuals,
            checks=[dict(name='current normal model',status='pass')]))
    return row,audit,hashes


class CompleteAdmission(unittest.TestCase):
    def test_native_shard_collection_refuses_missing_duplicate_or_wrong_models(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            for name in ('a','b'):
                folder=root/name;folder.mkdir()
                (folder/'summary-skip1.csv').write_text(name+',provider,1,1,2,2,2,in_flight,\n',encoding='utf-8')
            rows,raw,inputs=cadence.read_summary_shards(root,{'a','b'},1)
            self.assertEqual([r[0] for r in rows],['a','b']);self.assertEqual(len(inputs),2)
            self.assertEqual(raw.count(b'\n'),2)
            for invalid in (b'a,provider,1,1,2,2,2,in_flight,\n',b'b,provider,1,1,2,2,2,in_flight,\nb,provider,1,1,2,2,2,in_flight,\n'):
                (root/'b/summary-skip1.csv').write_bytes(invalid)
                with self.assertRaises(ValueError):cadence.read_summary_shards(root,{'a','b'},1)
            (root/'b/summary-skip1.csv').unlink()
            with self.assertRaises(FileNotFoundError):cadence.read_summary_shards(root,{'a','b'},1)

    def setUp(self):
        self.population={
            'aam':{'components':['seeker','guidance','fuze','warhead'],
                   'gates':['methods','native','public','installed','visual']},
            'bomb':{'components':['fuze','warhead'],
                    'gates':['methods','native','public','installed','visual']},
            'tank':{'components':[], 'gates':['applicability']}}
        self.rows=[dict(model=name,status='engineering-complete-uncalibrated',
                        components=list(row['components']),
                        gates={gate:dict(passed=True,evidence=['receipt']) for gate in row['gates']},
                        equipment_calibrated=False,unclosed_gates=[])
                   for name,row in self.population.items()]

    def test_every_declared_model_and_function_is_required(self):
        self.assertEqual(admit_models(self.population,self.rows),3)
        with self.assertRaises(ValueError):admit_models(self.population,self.rows[:-1])
        with self.assertRaises(ValueError):admit_models(self.population,self.rows+[self.rows[0]])
        for key in ('components','gates'):
            bad=copy.deepcopy(self.rows)
            if key=='components':bad[0][key].remove('seeker')
            else:bad[0][key].pop('native')
            with self.assertRaises(ValueError):admit_models(self.population,bad)

    def test_uncalibrated_does_not_waive_an_unclosed_function(self):
        for change in ({'unclosed_gates':['servo']},{'status':'blocked'},
                       {'gates':{'methods':dict(passed=True,evidence=['receipt'])}}):
            bad=copy.deepcopy(self.rows);bad[0].update(change)
            with self.assertRaises(ValueError):admit_models(self.population,bad)
        bad=copy.deepcopy(self.rows);bad[0]['gates']['native']['passed']=False
        with self.assertRaises(ValueError):admit_models(self.population,bad)
        bad=copy.deepcopy(self.rows);bad[0]['gates']['native']['evidence']=[]
        with self.assertRaises(ValueError):admit_models(self.population,bad)

    def test_calibration_cannot_be_claimed_without_equipment_evidence(self):
        bad=copy.deepcopy(self.rows);bad[0].update(status='engineering-complete',equipment_calibrated=True)
        with self.assertRaises(ValueError):admit_models(self.population,bad)

    def test_test_inventory_requires_nonzero_exact_execution_and_no_new_skip(self):
        self.assertEqual(reconcile_inventory(['one','two'],['one','two'],[],[]),2)
        for expected,executed,failed,skipped in (([],[],[],[]),(['one'],[],[],[]),
                (['one'],['one','one'],[],[]),(['one'],['other'],[],[]),
                (['one'],['one'],['one'],[]),(['one'],['one'],[],['one'])):
            with self.assertRaises(ValueError):reconcile_inventory(expected,executed,failed,skipped)

    def test_evidence_requires_actual_bytes_and_excluded_paths_are_rejected(self):
        import hashlib
        with tempfile.TemporaryDirectory(dir='D:/ENV/tmp' if os.name=='nt' else None) as folder:
            path=Path(folder)/'result.json';path.write_bytes(b'{"passed":true}\n')
            expected=hashlib.sha256(path.read_bytes()).hexdigest()
            self.assertEqual(verify_artifact(path,expected),expected)
            path.write_bytes(b'{"passed":false}\n')
            with self.assertRaises(ValueError):verify_artifact(path,expected)
            with self.assertRaises(ValueError):verify_artifact(Path(folder)/'.codex'/'never-read',expected)

    def test_review_requires_every_changed_file_with_exact_hashes(self):
        manifest=[dict(path='native.cpp',sha256='a'*64,deleted=False),dict(path='api.h',sha256='b'*64,deleted=False)]
        review=[dict(path=r['path'],sha256=r['sha256'],passed=True) for r in manifest]
        self.assertEqual(reconcile_source_review(manifest,review),2)
        for rows in ([],review[:-1],review+[review[0]],[dict(path='native.cpp',sha256='0'*64,passed=True),review[1]]):
            with self.assertRaises(ValueError):reconcile_source_review(manifest,rows)

    def test_aggregate_review_cannot_override_failed_missing_or_untyped_file_results(self):
        for name in ('native.cpp','aircraft/model/model.xml'):
            manifest=[dict(path=name,sha256='a'*64,deleted=False)]
            aggregate=dict(passed=True,files=[dict(path=name,sha256='a'*64,passed=True)])
            self.assertEqual(reconcile_source_review(manifest,aggregate['files']),1)
            for value in (False,None,1,0,'true',[],{}):
                bad=copy.deepcopy(aggregate);bad['files'][0]['passed']=value
                with self.subTest(path=name,passed=value),self.assertRaises(ValueError):
                    reconcile_source_review(manifest,bad['files'])
            missing=copy.deepcopy(aggregate);missing['files'][0].pop('passed')
            with self.subTest(path=name,passed='missing'),self.assertRaises(ValueError):
                reconcile_source_review(manifest,missing['files'])

    def test_population_cannot_hide_a_failed_duplicate_before_building_a_map(self):
        rows=[dict(card='one',passed=True),dict(card='two',passed=True)]
        self.assertEqual(set(reconcile_population({'one','two'},rows,('card',))),{'one','two'})
        for values in (rows[:-1],rows+[dict(card='one',passed=False)],
                       [dict(card='one',passed=False)]+rows,[dict(card='other'),rows[1]]):
            with self.assertRaises(ValueError):reconcile_population({'one','two'},values,('card',))
        expected={(name,scene) for name in ('one','two') for scene in ('head-on','tail-turn')}
        cases=[dict(card=name,scenario=scene) for name,scene in sorted(expected)]
        self.assertEqual(len(reconcile_population(expected,cases,('card','scenario'))),4)
        with self.assertRaises(ValueError):reconcile_population(expected,cases[:-1]+[cases[0]],('card','scenario'))

    def test_cadence_alignment_uses_only_prior_reference_decisions_and_retains_misses(self):
        baseline=[dict(age=0.,point=(1.,0.,0.),demand=(1.,0.,0.)),
                  dict(age=1.,point=(0.,1.,0.),demand=(3.,0.,0.))]
        coarse=[dict(age=.5,point=(0.,1.,0.),demand=(2.,0.,0.)),baseline[1]]
        value=cadence.compare_decisions(baseline,coarse,
            dict(terminal='in_flight',miss_m=None),dict(terminal='impact',miss_m=4.))
        self.assertEqual(value['decision_pairs'],2)
        self.assertAlmostEqual(value['maximum_alignment_age_s'],.5)
        self.assertAlmostEqual(value['maximum_pointing_difference_rad'],__import__('math').pi/2)
        self.assertAlmostEqual(value['maximum_demand_difference_m_s2'],1.)
        self.assertFalse(value['terminal_equal']);self.assertIsNone(value['miss_difference_m'])
        self.assertEqual(value['current_miss_m'],4.)
        with self.assertRaises(ValueError):cadence.compare_decisions(baseline,[],dict(terminal='x',miss_m=0),dict(terminal='x',miss_m=0))

    def test_terminal_weapon_cannot_hide_a_failed_or_unobserved_aftermath(self):
        def row(t):return [t,1,'target',False,200.,0.,0.,0.,200.,0.,0.,0.,0.,0.,0.,0.,1000.]
        raw=dict(world_health=[row(t) for t in (0.,1.,2.)],invalid_entities=[])
        run=dict(frame_skip=120,launch_time_s=0.,duration_s=2.,complete_world_finite=True,observed_world_states=3)
        self.assertEqual(reconcile_world_health(run,raw),3)
        for mutation in ('diverged','nonfinite','invalid','duplicate','missing-middle','missing-end','mass'):
            bad=copy.deepcopy(raw);record=copy.deepcopy(run)
            if mutation=='diverged':bad['world_health'][-1][3]=True
            if mutation=='nonfinite':bad['world_health'][-1][4]=float('nan')
            if mutation=='invalid':bad['invalid_entities']=[1]
            if mutation=='duplicate':bad['world_health'].append(row(2.))
            if mutation=='missing-middle':bad['world_health'].pop(1)
            if mutation=='missing-end':bad['world_health'].pop()
            if mutation=='mass':bad['world_health'][-1][-1]=0.
            record['observed_world_states']=len(bad['world_health'])
            with self.assertRaises(ValueError,msg=mutation):reconcile_world_health(record,bad)

    def test_all_model_products_must_join_the_audited_installed_epoch(self):
        row,audit,hashes=product_fixture()
        self.assertEqual(reconcile_model_products([row],{'plane':audit},hashes),6)
        for name in hashes:
            bad=dict(hashes);bad[name]='0'*64
            with self.assertRaises(ValueError):reconcile_model_products([row],{'plane':audit},bad)
            bad=dict(hashes);bad.pop(name)
            with self.assertRaises(ValueError):reconcile_model_products([row],{'plane':audit},bad)
        bad=dict(hashes,extra='e'*64)
        with self.assertRaises(ValueError):reconcile_model_products([row],{'plane':audit},bad)
        wrong=dict(audit,xml_sha256='f'*64)
        with self.assertRaises(ValueError):reconcile_model_products([row],{'plane':wrong},hashes)

    def test_native_windows_product_receipts_join_without_mutating_raw_keys(self):
        row,audit,hashes=product_fixture(directory='stores/plane')
        hashes={name.replace('/','\\'):digest for name,digest in hashes.items()}
        row['product_directory']='stores\\plane';row['products']=dict(hashes)
        row['normal_model']['products']=[name.replace('/','\\') for name in row['normal_model']['products']]
        original=copy.deepcopy(row)
        for installed in (hashes,{name.replace('\\','/'):digest for name,digest in hashes.items()}):
            self.assertEqual(reconcile_model_products([row],{'plane':audit},installed),6)
        self.assertEqual(row,original)

    def test_required_normal_visual_roster_is_derived_for_each_model_category(self):
        for category in ('store','fixed-wing','rotorcraft'):
            with self.subTest(category=category):
                row,audit,hashes=product_fixture(category,'stores/plane' if category=='store' else 'plane')
                # Informational and warning checks retain their established
                # normal-model meaning; none is relabeled as a failed check.
                row['normal_model']['checks'].extend([
                    dict(name='current information',status='info'),dict(name='current warning',status='warn')])
                self.assertEqual(reconcile_model_products([row],{'plane':audit},hashes),len(hashes))

    def test_matching_generation_and_install_omissions_do_not_waive_required_visuals(self):
        for category in ('store','fixed-wing','rotorcraft'):
            row,audit,hashes=product_fixture(category)
            for name in row['normal_model']['products']:
                for omit_declaration in (False,True):
                    with self.subTest(category=category,product=name,omit_declaration=omit_declaration):
                        bad=copy.deepcopy(row);installed=dict(hashes)
                        del bad['products'][name];del installed[name]
                        if omit_declaration:bad['normal_model']['products'].remove(name)
                        with self.assertRaises(ValueError):reconcile_model_products([bad],{'plane':audit},installed)

    def test_normal_visual_identity_list_must_be_exact_safe_and_unique(self):
        row,audit,hashes=product_fixture();visuals=row['normal_model']['products']
        for malformed in (None,'plane/plane.glb',{},[],visuals[:-1],visuals+['other/extra.glb'],
                visuals+[visuals[0]],visuals+[visuals[0].replace('/','\\')],
                visuals+['Plane/plane.glb'],[None,*visuals[1:]],['../plane/plane.glb',*visuals[1:]]):
            with self.subTest(products=malformed):
                bad=copy.deepcopy(row);bad['normal_model']['products']=malformed
                with self.assertRaises(ValueError):reconcile_model_products([bad],{'plane':audit},hashes)

    def test_normal_generation_record_and_current_checks_are_required(self):
        row,audit,hashes=product_fixture()
        for record in (None,[],{},dict(regenerated=True,products=row['normal_model']['products']),
                dict(regenerated=True,products=row['normal_model']['products'],checks=[])):
            with self.subTest(record=record):
                bad=copy.deepcopy(row);bad['normal_model']=record
                with self.assertRaises(ValueError):reconcile_model_products([bad],{'plane':audit},hashes)
        missing=copy.deepcopy(row);del missing['normal_model']
        with self.assertRaises(ValueError):reconcile_model_products([missing],{'plane':audit},hashes)
        for field,value in (('regenerated',False),('regenerated',1),('checks',None),('checks','pass'),
                ('checks',[None]),('checks',[{}]),('checks',[dict(name='current',status='fail')]),
                ('checks',[dict(name='current',status='unknown')])):
            with self.subTest(field=field,value=value):
                bad=copy.deepcopy(row);bad['normal_model'][field]=value
                with self.assertRaises(ValueError):reconcile_model_products([bad],{'plane':audit},hashes)

    def test_normal_product_roster_rejects_unsafe_names_and_unknown_categories(self):
        row,audit,hashes=product_fixture()
        for field,value in (('name','../plane'),('name','plane/nested'),('name','plane\\nested'),
                ('name','NUL'),('category','unknown'),('category',None)):
            with self.subTest(field=field,value=value):
                bad=copy.deepcopy(row);bad[field]=value
                with self.assertRaises(ValueError):reconcile_model_products([bad],{'plane':audit},hashes)

    def test_product_key_readers_refuse_unsafe_names_and_normalized_collisions(self):
        self.assertEqual(canonical_product_key('stores\\r77m/Engines\\motor.xml'),'stores/r77m/Engines/motor.xml')
        for name in (None,1,'','/absolute','\\rooted','C:relative','C:\\absolute','\\\\server\\share\\file',
                '../escape','a\\..\\escape','a/./b','a//b','a/.codex/file','a\\.CoDeX\\file','a/.git/file',
                'a/file:stream','a/NUL.xml','a/COM\u00b9.xml','a/name.','a/name ','a/line\nname'):
            with self.subTest(path=name),self.assertRaises(ValueError):canonical_product_key(name)
        for products in ({'a\\b':'a'*64,'a/b':'a'*64},{'A/b':'a'*64,'a/B':'a'*64}):
            with self.subTest(products=products),self.assertRaisesRegex(ValueError,'collide'):
                canonical_product_map(products)
        for products in ({},None,{'safe/file':'bad'},{'safe/file':True},{'a\\..\\b':'a'*64}):
            with self.subTest(products=products),self.assertRaises(ValueError):canonical_product_map(products)

    def test_cadence_records_cached_sample_age_and_observed_exposure_gaps(self):
        def row(decision,now,sample,delivery):
            return dict(instance='0' if decision else '1',decision=str(decision),time_s=str(now),launch_age_s=str(now),tracking='1',
                measurement_s=str(sample),processing_ready_s=str(sample+.1),delivered_s=str(delivery),angular_variance_rad2='.01',
                point_x='1',point_y='0',point_z='0',demand_x='1',demand_y='0',demand_z='0')
        rows=[row(0,.2,0.,.2),row(1,.2,0.,.2),row(1,.7,0.,.2),row(0,.7,.5,.7)]
        metrics,decisions=cadence.analyze_trace(rows,.1,.1,.11)
        self.assertAlmostEqual(metrics['maximum_sample_age_s'],.7)
        self.assertAlmostEqual(metrics['maximum_measurement_interval_s'],.5)
        self.assertEqual(len(decisions),2)
        bad=copy.deepcopy(rows);bad[0]['delivered_s']='.5'
        with self.assertRaises(ValueError):cadence.analyze_trace(bad,.1,.1,.11)


class RequiredMethodModules(unittest.TestCase):
    def fixtures(self):
        full=types.ModuleType('_m03_required_full_fixture')
        empty=types.ModuleType('_m03_required_empty_fixture')
        class Observation(unittest.TestCase):
            def test_observation(self):
                self.assertTrue(True)
        full.Observation=Observation
        return {full.__name__:full,empty.__name__:empty}

    def test_an_empty_module_cannot_hide_inside_a_nonempty_population(self):
        modules=self.fixtures()
        with patch.dict(sys.modules,modules):
            with self.assertRaisesRegex(ValueError,'Empty required model-method module'):
                method_suite(tuple(modules))

    def test_an_empty_required_module_list_is_refused(self):
        with self.assertRaises(ValueError):method_suite(())

    def test_a_nonempty_required_module_keeps_its_actual_case_count(self):
        modules=self.fixtures();name='_m03_required_full_fixture'
        with patch.dict(sys.modules,modules):
            self.assertEqual(method_suite((name,)).countTestCases(),1)


if __name__=='__main__':unittest.main()
