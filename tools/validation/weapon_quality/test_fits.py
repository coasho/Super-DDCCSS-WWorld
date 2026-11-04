"""Pure source and evidence mutation tests; never import or execute the SDK."""
import copy
from pathlib import Path
import tempfile
import unittest

import fits


class AuthoredFits(unittest.TestCase):
    def setUp(self):
        directory=Path('D:/ENV/tmp');directory.mkdir(parents=True,exist_ok=True)
        self.temp=tempfile.TemporaryDirectory(dir=directory);self.addCleanup(self.temp.cleanup)
        self.prefix=Path(self.temp.name).resolve()
        self.binding=dict(design='carrier',loadout='dual rail',payload='stations',station=11,card='missile',
            count=2,model='jsbsim:missile',store_class='aam',jettisonable=True,guided=True,armed=True,
            components={'guidance':dict(availability='available',reason=None)})
        self.population=dict(airframes=[dict(design='carrier',category='fixed-wing',fit_path='aircraft/carrier/payloads.json',
            fit_sha256='1'*64,authored_loadouts=1)],loadouts=[dict(design='carrier',loadout='dual rail',source='source',note=None,
                stations=[dict(number=11,card='missile',count=2,payload='stations')])],bindings=[self.binding],
            sources={'assets/stores/missile.json':'2'*64},counts={'physical_stores':2},inherited_rotor_scope={})
        def product(name):
            path=self.prefix/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_text(name,encoding='utf-8')
            return dict(path=str(path),sha256=fits.sha(path))
        dll=product('bin/libfsim.dll');jsbsim=product('bin/libJSBSim.dll')
        self.python=dict(facade=product('fsim/__init__.py'),native=product('fsim/_native.pyd'),
            world=product('fsim/world.py'),state=product('fsim/_state.py'))
        model=product('share/flightsim/aircraft/carrier/carrier.xml')
        self.producer=dict(schema='fsim.sim-m02.production-install.v1',product_count=1,prefix=str(self.prefix),native_dll_sha256=dll['sha256'],products={'carrier/carrier.xml':model['sha256']})
        self.modules=[dict(name='libfsim.dll',**dll),dict(name='libJSBSim.dll',**jsbsim)]
        self.runtime_inputs={name:dict(source=source,**product(name)) for name,source in (
            ('share/flightsim/stores/missile.json','assets/stores/missile.json'),
            ('share/flightsim/payloads/stations.json','assets/payloads/stations.json'),
            ('share/flightsim/aircraft/carrier/payloads.json','aircraft/carrier/payloads.json'))}
        self.population['sources']={item['source']:item['sha256'] for item in self.runtime_inputs.values()}
        for name,source in (('facade','python/fsim/__init__.py'),('world','python/fsim/world.py'),('state','python/fsim/_state.py')):
            self.population['sources'][source]=self.python[name]['sha256']
        self.runtime_inputs=fits.derive_runtime_inputs(self.population,self.prefix,self.python)
        self.population['mass_preflight']=dict(schema='fsim.sim-m03.authored-fit-mass-preflight.v1',fuel_fraction=.5,
            rows=[dict(design='carrier',loadout='dual rail',xml_sha256=model['sha256'],loaded_mass_kg=84.,limit_availability='available',mtow_kg=100.,margin_kg=16.,passed=True)])
        self.epoch=fits.runtime_epoch(self.population,self.producer,self.modules,self.python,self.runtime_inputs)
        self.receipt=dict(schema=fits.SCHEMA,passed=True,prefix=str(self.prefix),population=self.population,policy=fits.POLICY,
            source_sha256=fits.digest(self.population['sources']),producer_sha256=fits.digest(self.producer),products=self.producer['products'],
            loaded_modules=self.modules,python=self.python,epoch_sha256=self.epoch,loadouts=[],armed=[],runtime_inputs=self.runtime_inputs,
            mass_preflight=self.population['mass_preflight'])
        for mode in ('jettison','armed'):
            scene,raw=self.make_scene(mode);path=self.prefix/(mode+'.json');fits.write(path,raw)
            scene.update(raw_path=str(path),raw_sha256=fits.sha(path))
            self.receipt['loadouts' if mode=='jettison' else 'armed'].append(scene)

    def make_scene(self,mode):
        inventory=[dict(index=i,payload_name='stations',station=11,store='missile',flyable=True,jettisonable=True,status='loaded',weapon='0') for i in range(2)]
        step=fits.POLICY['dt_s']*fits.POLICY['frame_skip']
        times=[11*step+i*step for i in range(fits.POLICY['aftermath_steps']+1)]
        releases=[dict(index=i,payload_name='stations',station=11,store='missile',entity=str(i+2),launcher='1',activity='4294967297',
            expected_activity='4294967297',target='0',type='jsbsim:missile',jettisoned=mode=='jettison',launch_time_s=(2+i*8)*step,slot=i+1,generation='1') for i in range(2)]
        fins=[dict(id=4,native_index=0,valid=True,failed_stuck=False,failed_zero=False,failed_hardover=False,
            actual_rad=.01,demand_rad=.02,minimum_rad=-.3,maximum_rad=.3,rate_rad_s=4.)]
        controls=[dict(entity=r['entity'],slot=r['slot'],generation='1',available=True,controller_available=mode=='armed',
            sample_time_s=t,controller_time_s=t-fits.POLICY['dt_s'],commands=[0.,.1,0.],fins=copy.deepcopy(fins)) for t in times for r in releases]
        states=[dict(entity=str(entity),time_s=t,diverged=False,values=[1.]*14+[100.,1500.]) for t in times for entity in (1,2,3)]
        after=[dict(s,status='jettisoned' if mode=='jettison' else 'released',weapon=str(s['index']+2)) for s in inventory]
        pre_times=[i*step for i in range(12)]
        pre_states=[dict(entity='1',time_s=t,diverged=False,values=[1.]*14+[100.,1500.]) for t in pre_times]
        pre_states.extend(dict(entity=r['entity'],time_s=t,diverged=False,values=[1.]*14+[100.,1500.])
            for t in pre_times for r in releases if t>=r['launch_time_s'])
        raw=dict(design='carrier',loadout='dual rail',mode=mode,policy=fits.POLICY,scope=fits.POLICY['scope'],epoch_sha256=self.epoch,
            initial_conditions=dict(fits.POLICY[mode],fuel_fraction=.5),
            loadout_bindings=[self.binding],inventory_before=inventory,inventory_after=after,releases=releases,controls=controls,
            states=states,times=times,launcher='1',activities={'stations':'4294967297'},station_wait_steps=1,release_wait_steps=10,
            pre_release_times=pre_times,pre_release_states=pre_states,
            outcomes=[dict(entity=r['entity'],end='in_flight',**{k:r[k] for k in ('jettisoned','activity','store','station')}) for r in releases])
        scene=dict(design='carrier',loadout='dual rail',passed=True,loadout_bindings=[self.binding])
        if mode=='armed':scene.update(payload='stations',station=11,card='missile',binding=self.binding)
        return scene,raw

    def test_exact_source_denominator_and_dual_rail(self):
        population=fits.derive_population()
        self.assertEqual(population['counts'],dict(airframes=35,airframes_with_loadouts=21,loadouts=46,station_bindings=214,
            physical_stores=215,aam_station_bindings=89,armed_station_bindings=119,armed_physical_stores=120))
        dual=[b for b in population['bindings'] if b['count']>1]
        self.assertEqual([(b['design'],b['loadout'],b['station'],b['card'],b['count']) for b in dual],[('a10c','self-defence',11,'aim-9m',2)])
        self.assertEqual(population['inherited_rotor_scope']['deferred_scenarios'],28)
        self.assertFalse(population['inherited_rotor_scope']['dynamic_acceptance_added'])

    def test_complete_receipt_admits_two_actual_physical_stores(self):
        self.assertEqual(fits.validate_receipt(self.receipt,self.population,self.producer),dict(loadouts=1,armed_station_bindings=1,physical_jettisons=2,armed_physical_stores=2))

    def test_missing_or_duplicate_loadout_and_armed_rows_fail(self):
        for field in ('loadouts','armed'):
            for rows in ([],self.receipt[field]*2):
                bad=copy.deepcopy(self.receipt);bad[field]=rows
                with self.subTest(field=field,count=len(rows)),self.assertRaises(ValueError):fits.validate_receipt(bad,self.population,self.producer)

    def test_wrong_source_product_or_epoch_fails(self):
        mutations=[lambda r:r.update(source_sha256='0'*64),lambda r:r.update(producer_sha256='0'*64),
            lambda r:r['products'].update({'carrier/carrier.xml':'0'*64}),lambda r:r.update(epoch_sha256='0'*64),
            lambda r:r['loaded_modules'][0].update(sha256='0'*64),lambda r:r['python']['native'].update(path='D:/ENV/wrong/_native.pyd'),
            lambda r:r['population']['bindings'][0].update(count=1)]
        for mutate in mutations:
            bad=copy.deepcopy(self.receipt);mutate(bad)
            with self.subTest(mutation=mutations.index(mutate)),self.assertRaises(ValueError):fits.validate_receipt(bad,self.population,self.producer)

    def test_installed_product_and_raw_bytes_are_verified(self):
        for relative in ('share/flightsim/aircraft/carrier/carrier.xml','bin/libJSBSim.dll','fsim/_native.pyd','armed.json'):
            path=self.prefix/relative;original=path.read_bytes();path.write_bytes(original+b'changed')
            with self.subTest(relative=relative),self.assertRaises(ValueError):fits.validate_receipt(self.receipt,self.population,self.producer)
            path.write_bytes(original)

    def test_inventory_count_and_station_cannot_be_replaced(self):
        for mutate in (lambda r:r['inventory_before'].pop(),lambda r:r['inventory_before'][0].update(station=1),
                       lambda r:r['inventory_before'][0].update(status='released'),lambda r:r['inventory_before'][0].update(index=1)):
            scene,raw=self.make_scene('armed');mutate(raw)
            with self.assertRaises(ValueError):fits.validate_scene(scene,raw,[self.binding],'armed',self.epoch)

    def test_armed_release_and_controller_are_mandatory(self):
        mutations=[lambda r:r['releases'][0].update(jettisoned=True),lambda r:r['releases'].pop(),
            lambda r:r['releases'].append(copy.deepcopy(r['releases'][0])),
            lambda r:[c.update(controller_available=False) for c in r['controls']],
            lambda r:[c.update(controller_time_s=None) for c in r['controls']],
            lambda r:r['controls'].pop(),lambda r:r['controls'][0].update(generation='2'),
            lambda r:r['controls'][0].update(entity='99'),lambda r:r['controls'][0].update(commands=[float('nan'),0.,0.]),
            lambda r:r['controls'][0]['fins'][0].update(valid=False),lambda r:r['controls'][0]['fins'][0].update(actual_rad=1.)]
        for mutate in mutations:
            scene,raw=self.make_scene('armed');mutate(raw)
            with self.subTest(mutation=mutations.index(mutate)),self.assertRaises(ValueError):fits.validate_scene(scene,raw,[self.binding],'armed',self.epoch)

    def test_correlations_and_consumed_store_are_exact(self):
        mutations=[lambda r:r['releases'][0].update(activity='1'),lambda r:r['releases'][0].update(entity='0'),
            lambda r:r['releases'][0].update(generation='01'),lambda r:r['releases'][0].update(slot=-1),
            lambda r:r['releases'][0].update(target='8'),lambda r:r['releases'][0].update(store='other'),
            lambda r:r['releases'][0].update(type='jsbsim:other'),lambda r:r['inventory_after'][0].update(weapon='3'),
            lambda r:r['inventory_after'][0].update(store='other'),lambda r:r['inventory_after'][0].update(status='loaded')]
        for mutate in mutations:
            scene,raw=self.make_scene('armed');mutate(raw)
            with self.subTest(mutation=mutations.index(mutate)),self.assertRaises(ValueError):fits.validate_scene(scene,raw,[self.binding],'armed',self.epoch)

    def test_another_loaded_station_cannot_be_consumed(self):
        scene,raw=self.make_scene('armed')
        other=dict(self.binding,station=9,card='other',count=1)
        raw['loadout_bindings'].append(other)
        scene['loadout_bindings']=raw['loadout_bindings']
        extra=dict(index=2,payload_name='stations',station=9,store='other',flyable=True,jettisonable=True,status='loaded',weapon='0')
        raw['inventory_before'].append(extra)
        raw['inventory_after'].append(copy.deepcopy(extra))
        self.assertEqual(fits.validate_scene(scene,raw,[self.binding],'armed',self.epoch),2)
        raw['inventory_after'][-1].update(status='released',weapon='4')
        with self.assertRaises(ValueError):fits.validate_scene(scene,raw,[self.binding],'armed',self.epoch)

    def test_complete_bounded_finite_native_integration_is_required(self):
        mutations=[lambda r:r['times'].pop(),lambda r:r['states'].pop(),lambda r:r['states'][0].update(diverged=True),
            lambda r:r['states'][0]['values'].__setitem__(14,0),lambda r:r['states'][0]['values'].__setitem__(0,float('inf')),
            lambda r:r.update(release_wait_steps=0),lambda r:r.update(release_wait_steps=fits.POLICY['release_steps']+1),
            lambda r:r['states'].append(copy.deepcopy(r['states'][0])),lambda r:r.update(epoch_sha256='0'*64)]
        for mutate in mutations:
            scene,raw=self.make_scene('jettison');mutate(raw)
            with self.subTest(mutation=mutations.index(mutate)),self.assertRaises(ValueError):fits.validate_scene(scene,raw,[self.binding],'jettison',self.epoch)

    def test_coverage_requires_authored_count_field(self):
        airframe=self.population['airframes'][0]
        coverage=dict(inventory_passed=True,fits=[dict(airframe,declared_loadouts=[dict(name='dual rail',stations=[dict(number=11,card='missile',count=2)])])],
            stores=[dict(card='missile',card_sha256=self.population['sources']['assets/stores/missile.json'])])
        fits.compare_coverage(self.population,coverage)
        coverage['fits'][0]['declared_loadouts'][0]['stations'][0]['count']=1
        with self.assertRaises(ValueError):fits.compare_coverage(self.population,coverage)

    def test_global_fuel_policy_is_explicit_and_bound(self):
        self.assertEqual(fits.POLICY.get('fuel_fraction'),.5)
        for fuel in (1.,None):
            scene,raw=self.make_scene('armed');raw['initial_conditions']=dict(fits.POLICY['armed'])
            if fuel is not None:raw['initial_conditions']['fuel_fraction']=fuel
            with self.subTest(fuel=fuel),self.assertRaises(ValueError):fits.validate_scene(scene,raw,[self.binding],'armed',self.epoch)

    def test_source_mass_preflight_preserves_missing_limits(self):
        population=fits.derive_population();mass=fits.derive_mass_preflight(population)
        self.assertEqual(len(mass['rows']),46)
        known=[r for r in mass['rows'] if r['limit_availability']=='available']
        unknown=[r for r in mass['rows'] if r['limit_availability']=='unavailable']
        self.assertEqual(len(known),44);self.assertTrue(all(r['passed'] and r['margin_kg']>0 for r in known))
        self.assertEqual({r['design'] for r in unknown},{'j10a'})
        self.assertEqual(len(unknown),2);self.assertTrue(all(r['passed'] is None and r['mtow_kg'] is None and r['reason'] for r in unknown))
        a10=next(r for r in known if (r['design'],r['loadout'])==('a10c','external tanks'))
        self.assertAlmostEqual(a10['margin_kg'],2264.6328,places=4)

    def test_mass_preflight_uses_capacity_and_complete_store_fuel(self):
        root=self.prefix/'mass-source';(root/'aircraft/carrier').mkdir(parents=True);(root/'assets/stores').mkdir(parents=True)
        (root/'aircraft/carrier/carrier.toml').write_text('[mass]\nmax_takeoff=100\n',encoding='utf-8')
        (root/'aircraft/carrier/carrier.xml').write_text('<fdm_config><mass_balance><emptywt unit="KG">9</emptywt><pointmass><weight unit="KG">1</weight></pointmass></mass_balance><propulsion><tank type="FUEL"><capacity unit="KG">100</capacity><contents unit="KG">1</contents></tank></propulsion></fdm_config>',encoding='utf-8')
        fits.write(root/'assets/stores/missile.json',dict(parameters=dict(mass_kg=dict(value=5),fuel_kg=dict(value=7))))
        mass=fits.derive_mass_preflight(self.population,root=root)
        row=mass['rows'][0]
        self.assertEqual((row['dry_mass_kg'],row['internal_fuel_capacity_kg'],row['scene_fuel_kg'],row['stores_mass_kg'],row['loaded_mass_kg'],row['margin_kg']),(10.,100.,50.,24.,84.,16.))
        original=copy.deepcopy(mass)
        fits.write(root/'assets/stores/missile.json',dict(parameters=dict(mass_kg=dict(value=6),fuel_kg=dict(value=7))))
        self.assertNotEqual(fits.derive_mass_preflight(self.population,root=root),original)
        fits.write(root/'assets/stores/missile.json',dict(parameters=dict(mass_kg=dict(value=5),fuel_kg=dict(value=7))))
        xml_path=root/'aircraft/carrier/carrier.xml';xml_path.write_text(xml_path.read_text(encoding='utf-8').replace('>100<','>101<'),encoding='utf-8')
        changed=fits.derive_mass_preflight(self.population,root=root)
        self.assertEqual(changed['rows'][0]['scene_fuel_kg'],50.5)
        self.assertNotEqual(changed['rows'][0]['xml_sha256'],original['rows'][0]['xml_sha256'])

    def test_installed_operational_json_changes_invalidate_admission(self):
        for name in self.runtime_inputs:
            path=self.prefix/name;original=path.read_bytes();path.write_bytes(original+b'changed')
            with self.subTest(path=name),self.assertRaises(ValueError):fits.validate_receipt(self.receipt,self.population,self.producer)
            path.write_bytes(original)

    def test_known_world_and_state_module_identities_are_required(self):
        bad=copy.deepcopy(self.receipt)
        for name in ('world','state'):bad['python'].pop(name,None)
        with self.assertRaises(ValueError):fits.validate_receipt(bad,self.population,self.producer)

    def test_installed_world_and_state_byte_changes_invalidate_admission(self):
        for name in ('fsim/world.py','fsim/_state.py'):
            path=self.prefix/name;original=path.read_bytes();path.write_bytes(original+b'changed')
            with self.subTest(path=name),self.assertRaises(ValueError):fits.validate_receipt(self.receipt,self.population,self.producer)
            path.write_bytes(original)

    def test_runtime_input_map_is_complete_and_source_copy_bound(self):
        for name in self.runtime_inputs:
            bad=copy.deepcopy(self.receipt);bad['runtime_inputs'].pop(name)
            with self.subTest(path=name),self.assertRaises(ValueError):fits.validate_receipt(bad,self.population,self.producer)
        bad=copy.deepcopy(self.receipt)
        bad['python']['world']['sha256']='0'*64
        with self.assertRaises(ValueError):fits.validate_receipt(bad,self.population,self.producer)

    def test_changed_or_unbound_complete_loadout_mass_receipt_fails(self):
        for field in ('loaded_mass_kg','xml_sha256'):
            bad=copy.deepcopy(self.receipt);bad['mass_preflight']=copy.deepcopy(self.receipt['mass_preflight'])
            bad['mass_preflight']['rows'][0][field]=0
            with self.subTest(field=field),self.assertRaises(ValueError):fits.validate_receipt(bad,self.population,self.producer)

    def test_launch_boundary_has_no_decision_and_ended_body_cannot_continue(self):
        scene,raw=self.make_scene('armed');last=raw['releases'][-1];last['launch_time_s']=raw['times'][0]
        raw['pre_release_states']=[s for s in raw['pre_release_states'] if s['entity']!='3' or s['time_s']>=last['launch_time_s']]
        first=next(c for c in raw['controls'] if c['entity']=='3')
        first.update(controller_available=False,controller_time_s=None)
        self.assertEqual(fits.validate_scene(scene,raw,[self.binding],'armed',self.epoch),2)
        first.update(controller_available=True,controller_time_s=last['launch_time_s']-fits.POLICY['dt_s'])
        with self.assertRaises(ValueError):fits.validate_scene(scene,raw,[self.binding],'armed',self.epoch)
        scene,raw=self.make_scene('armed');raw['outcomes'][0]['end']='expired'
        with self.assertRaises(ValueError):fits.validate_scene(scene,raw,[self.binding],'armed',self.epoch)

    def test_negative_prelaunch_stale_frozen_and_dropout_decisions_fail(self):
        mutations=[lambda r:[c.update(controller_time_s=-1000.) for c in r['controls']],
            lambda r:[x.update(launch_time_s=-1000.) for x in r['releases']],
            lambda r:[c.update(controller_time_s=0.) for c in r['controls']],
            lambda r:[c.update(controller_time_s=r['times'][0]-fits.POLICY['dt_s']) for c in r['controls']],
            lambda r:[c.update(controller_available=False) for c in r['controls'][2:]],
            lambda r:r['releases'][0].update(launch_time_s=r['pre_release_times'][0])]
        for mutate in mutations:
            scene,raw=self.make_scene('armed');mutate(raw)
            with self.subTest(mutation=mutations.index(mutate)),self.assertRaises(ValueError):fits.validate_scene(scene,raw,[self.binding],'armed',self.epoch)

    def test_dual_rail_pre_release_timeline_is_complete_and_unique(self):
        scene,raw=self.make_scene('armed')
        first=next(s for s in raw['pre_release_states'] if s['entity']=='2' and s['time_s']>raw['releases'][0]['launch_time_s'])
        mutations=[lambda r:r['pre_release_states'].remove(first),
            lambda r:r['pre_release_states'].append(copy.deepcopy(first)),
            lambda r:next(s for s in r['pre_release_states'] if s==first).update(time_s=0.),
            lambda r:r['pre_release_states'].append(dict(first,entity='99'))]
        for mutate in mutations:
            changed=copy.deepcopy(raw);mutate(changed)
            with self.subTest(mutation=mutations.index(mutate)),self.assertRaises(ValueError):fits.validate_scene(scene,changed,[self.binding],'armed',self.epoch)


if __name__=='__main__':unittest.main()
