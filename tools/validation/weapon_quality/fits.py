"""Execute and admit every authored loadout using only an installed public SDK.

The armed scenes are explicitly referee integration: Vehicle.strike supplies
its own scenario consent. They are not actor/C2 permission evidence. No scene
requires an invented hit, seeker lock, performance envelope, or pod function.
"""
import argparse
from collections import Counter
import ctypes
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys
import tomllib
import xml.etree.ElementTree as ET

from acceptance import allowed, canonical_product_map, reconcile_population, verify_artifact
from contract import APPROVED_ROTOR_POLICY

ROOT=Path(__file__).resolve().parents[3]
SCHEMA='fsim.sim-m03.installed-authored-fits.v1'
# Frozen before execution. Short M02 untrimmed flight and the existing M03
# nominal guided launch recipe; these are integration scenes, not flight limits.
POLICY=dict(dt_s=1/120,frame_skip=4,seed=41,workers=1,terrain=False,fuel_fraction=.5,
    jettison=dict(latitude_deg=37.6,longitude_deg=-122.4,altitude_msl_m=1500.,heading_deg=0.,airspeed_ms=90.),
    armed=dict(latitude_deg=40.,longitude_deg=0.,altitude_msl_m=6000.,heading_deg=90.,airspeed_ms=250.),
    station_steps=1350,release_steps=1350,aftermath_steps=60,
    control_contract='Actual native armed uncued initialization and finite control; no acquisition or intercept claim',
    scope='Referee integration; separate fleet.py provides actor-permitted release and modeled C2 consent')


def require(value,reason):
    if not value:raise ValueError(reason)


def sha(path):return hashlib.sha256(allowed(path).read_bytes()).hexdigest()
def read(path):return json.loads(allowed(path).read_text(encoding='utf-8'))
def digest(value):return hashlib.sha256(json.dumps(value,sort_keys=True,allow_nan=False,separators=(',',':')).encode()).hexdigest()


def write(path,value):
    path=allowed(path);path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_name(path.name+'.writing')
    temporary.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    temporary.replace(path)


def identity(row):return (row['design'],row['loadout'],row['payload'],row['station'],row['card'])
def loadout_identity(row):return (row['design'],row['loadout'])


def derive_mass_preflight(population,root=ROOT):
    """Independent source/XML loading arithmetic, not measured native mass."""
    root=allowed(root);rows=[]
    def kg(node):
        require(node is not None,'Missing generated mass/capacity')
        require(node.get('unit','LBS').upper() in ('KG','LBS'),'Unknown generated mass unit')
        value=float(node.text)*(1 if node.get('unit','LBS').upper()=='KG' else .45359237)
        require(math.isfinite(value) and value>=0,'Invalid generated mass/capacity');return value
    for loadout in population['loadouts']:
        design=loadout['design'];folder=root/'aircraft'/design;spec_path=folder/(design+'.toml');xml_path=folder/(design+'.xml')
        spec=tomllib.loads(spec_path.read_text(encoding='utf-8'));xml=ET.parse(xml_path).getroot()
        dry=kg(xml.find('mass_balance/emptywt'))+sum(kg(p.find('weight')) for p in xml.findall('mass_balance/pointmass'))
        capacity=sum(kg(t.find('capacity')) for t in xml.findall('propulsion/tank') if t.get('type')=='FUEL')
        stores=[]
        for binding in population['bindings']:
            if loadout_identity(binding)!=loadout_identity(loadout):continue
            card_path=root/'assets/stores'/(binding['card']+'.json');card=read(card_path);parameters=card['parameters']
            mass=parameters['mass_kg']['value'];fuel=parameters.get('fuel_kg',{}).get('value',0.)
            require(finite([mass,fuel]) and mass>0 and fuel>=0,'Invalid complete store mass/fuel')
            stores.append(dict(payload=binding['payload'],station=binding['station'],card=binding['card'],count=binding['count'],
                dry_mass_kg=mass,fuel_kg=fuel,sha256=sha(card_path)))
        stores_mass=sum(s['count']*(s['dry_mass_kg']+s['fuel_kg']) for s in stores)
        limit=spec.get('mass',{}).get('max_takeoff',spec.get('mass',{}).get('mtow'))
        require(limit is None or finite([limit]) and limit>0,'Invalid declared MTOW')
        loaded=dry+capacity*POLICY['fuel_fraction']+stores_mass
        margin=limit-loaded if limit is not None else None
        row=dict(design=design,loadout=loadout['loadout'],spec_sha256=sha(spec_path),xml_sha256=sha(xml_path),
            xml_path=xml_path.relative_to(root).as_posix(),stores=stores,dry_mass_kg=dry,internal_fuel_capacity_kg=capacity,
            scene_fuel_kg=capacity*POLICY['fuel_fraction'],stores_mass_kg=stores_mass,loaded_mass_kg=loaded,mtow_kg=limit,
            margin_kg=margin,limit_availability='available' if limit is not None else 'unavailable',
            passed=margin>=0 if margin is not None else None,
            reason=None if limit is not None else 'No mass.max_takeoff or mass.mtow is declared in '+spec_path.relative_to(root).as_posix())
        require(row['passed'] is not False,'Fixed global fuel policy exceeds declared MTOW: '+repr(loadout_identity(loadout)))
        rows.append(row)
    return dict(schema='fsim.sim-m03.authored-fit-mass-preflight.v1',fuel_fraction=POLICY['fuel_fraction'],rows=rows,
        scope='Source/XML loading arithmetic; unavailable declared limits are not passed native mass evidence')


def derive_population(root=ROOT):
    """Derive from TOML, authored fits, station cards and store cards, not audit rows."""
    root=allowed(root);airframes=[];loadouts=[];bindings=[];sources={};cards={}
    def source(path):
        relative=path.relative_to(root).as_posix();sources[relative]=sha(path);return relative
    for path in sorted((root/'assets/stores').glob('*.json')):
        card=read(path);require(card['id']==path.stem and card['id'] not in cards,'Duplicate/misnamed source card')
        source(path);cards[card['id']]=card
    for directory in sorted((root/'aircraft').iterdir()):
        if directory.name.casefold()=='.codex':continue
        path=directory/(directory.name+'.toml')
        if not path.is_file():continue
        aircraft=tomllib.loads(path.read_text(encoding='utf-8'))['aircraft']
        source(path)
        if aircraft.get('category') in ('store','missile'):continue
        design=directory.name
        category='rotorcraft' if aircraft.get('kind') in ('helicopter','multirotor') else 'fixed-wing'
        fit_path=directory/'payloads.json';fit=read(fit_path) if fit_path.is_file() else {}
        if fit_path.is_file():source(fit_path)
        require(not fit or fit['aircraft']==design,'Fit/airframe identity differs')
        payloads={}
        for payload in fit.get('payloads',[]):
            card_path=root/'assets/payloads'/(payload['card']+'.json');payload_card=read(card_path);source(card_path)
            if payload_card.get('family')=='store_station':
                require(payload['name'] not in payloads,'Duplicate station payload name')
                payloads[payload['name']]=payload_card
        airframes.append(dict(design=design,category=category,fit_path=fit_path.relative_to(root).as_posix(),
            fit_sha256=sha(fit_path) if fit_path.is_file() else None,authored_loadouts=len(fit.get('loadouts',[]))))
        for loadout in fit.get('loadouts',[]):
            lrow=dict(design=design,loadout=loadout['name'],source=loadout.get('source'),note=loadout.get('note'),stations=[])
            for station in loadout.get('stations',[]):
                payload=station.get('payload')
                if payload is None:
                    require(len(payloads)==1,'An omitted station payload is ambiguous');payload=next(iter(payloads))
                require(payload in payloads,'Unknown authored station payload')
                card=cards[station['store']];count=station.get('count',1)
                require(type(count) is int and count>0,'Authored count must be a positive integer')
                stations=[s for s in payloads[payload]['stations'] if s['number']==station['station']]
                require(len(stations)==1,'Unknown or duplicate authored station')
                require(card['class'] in stations[0]['takes'] and count<=stations[0].get('capacity',1),'Incompatible authored station/count')
                components=card['component_models']['components']
                # Applicability is copied exactly from source; unavailable is a failure,
                # not a substitute for a not_applicable reason.
                require(all(c['availability'] in ('available','not_applicable') for c in components.values()),'Required component unavailable')
                require(all(c.get('reason') for c in components.values() if c['availability']=='not_applicable'),'Missing exact inapplicability reason')
                row=dict(design=design,loadout=loadout['name'],payload=payload,station=station['station'],card=card['id'],
                    count=count,model=card['model'],store_class=card['class'],jettisonable=card.get('jettisonable',False),
                    guided=components['guidance']['availability']=='available',
                    armed=any(components[k]['availability']=='available' for k in ('guidance','fuze','warhead')),
                    components={k:dict(availability=v['availability'],reason=v.get('reason')) for k,v in components.items()})
                require(row['jettisonable'] and bool(row['model']),'An authored nonjettisonable/nonflyable store needs an explicit operation contract')
                lrow['stations'].append(dict(number=row['station'],card=row['card'],count=count,payload=payload))
                bindings.append(row)
            loadouts.append(lrow)
    reconcile_population({identity(r) for r in bindings},bindings,('design','loadout','payload','station','card'))
    reconcile_population({loadout_identity(r) for r in loadouts},loadouts,('design','loadout'))
    require(not any(a['category']=='rotorcraft' and a['authored_loadouts'] for a in airframes),'Rotor dynamic scope requires a separate accepted assignment')
    population=dict(airframes=airframes,loadouts=loadouts,bindings=bindings,sources=sources,
        counts=dict(airframes=len(airframes),airframes_with_loadouts=sum(a['authored_loadouts']>0 for a in airframes),
            loadouts=len(loadouts),station_bindings=len(bindings),physical_stores=sum(b['count'] for b in bindings),
            aam_station_bindings=sum(b['store_class']=='aam' for b in bindings),
            armed_station_bindings=sum(b['armed'] for b in bindings),armed_physical_stores=sum(b['count'] for b in bindings if b['armed'])),
        inherited_rotor_scope=dict(policy_sha256=APPROVED_ROTOR_POLICY,deferred_scenarios=28,dynamic_acceptance_added=False))
    population['mass_preflight']=derive_mass_preflight(population,root)
    for row in population['mass_preflight']['rows']:sources[row['xml_path']]=row['xml_sha256']
    for relative in ('python/fsim/__init__.py','python/fsim/world.py','python/fsim/_state.py'):source(root/relative)
    return population


def derive_runtime_inputs(population,prefix,python):
    """Explicit consumed JSON and known imported modules; no filesystem walk."""
    prefix=allowed(prefix).resolve();result={}
    require(set(python)=={'facade','native','world','state'},'Actual facade/native/world/state module objects are required')
    for relative,expected in population['sources'].items():
        source=Path(relative);runtime=None
        if relative.startswith('assets/stores/'):runtime=Path('share/flightsim/stores')/source.name
        elif relative.startswith('assets/payloads/'):runtime=Path('share/flightsim/payloads')/source.name
        elif relative.startswith('aircraft/') and source.name=='payloads.json':runtime=Path('share/flightsim')/source
        elif relative in ('python/fsim/__init__.py','python/fsim/world.py','python/fsim/_state.py'):runtime=source.relative_to('python')
        if runtime is not None:result[runtime.as_posix()]=dict(source=relative,path=str(prefix/runtime),sha256=expected)
    for name,relative in (('facade','fsim/__init__.py'),('world','fsim/world.py'),('state','fsim/_state.py')):
        require(relative in result and python[name]=={k:result[relative][k] for k in ('path','sha256')},'Actual imported Python module differs from its expected source copy')
    return result


def verify_runtime_inputs(runtime,population,prefix,python):
    require(runtime==derive_runtime_inputs(population,prefix,python),'Incomplete or changed explicit runtime input map')
    for item in runtime.values():verify_artifact(item['path'],item['sha256'])


def runtime_epoch(population,producer,modules,python,runtime):
    return digest(dict(sources=population['sources'],producer=producer,modules=modules,python=python,runtime_inputs=runtime))


def compare_coverage(population,coverage):
    require(coverage.get('inventory_passed') is True,'Coverage audit did not pass')
    fits=reconcile_population({r['design'] for r in population['airframes']},coverage['fits'],('design',))
    actual=[]
    for airframe in population['airframes']:
        fit=fits[airframe['design']]
        require(all(fit[k]==airframe[k] for k in ('category','fit_path','fit_sha256')),'Coverage source identity differs')
        for loadout in fit['declared_loadouts']:
            actual.append((airframe['design'],loadout['name'],[(s['number'],s['card'],s['count']) for s in loadout['stations']]))
    expected=[(l['design'],l['loadout'],[(s['number'],s['card'],s['count']) for s in l['stations']]) for l in population['loadouts']]
    require(sorted(actual)==sorted(expected),'Coverage loadout/count denominator differs from independent source')
    source_cards={Path(p).stem:h for p,h in population['sources'].items() if p.startswith('assets/stores/')}
    audit_cards=reconcile_population(set(source_cards),coverage['stores'],('card',))
    require(all(audit_cards[name]['card_sha256']==h for name,h in source_cards.items()),'Coverage store epoch differs')


def validate_inventory(binding_rows,inventory):
    expected=Counter((b['payload'],b['station'],b['card']) for b in binding_rows for _ in range(b['count']))
    actual=Counter((s['payload_name'],s['station'],s['store']) for s in inventory)
    require(actual==expected and len({s['index'] for s in inventory})==len(inventory),'Exact authored inventory/count/index reconciliation failed')
    require(all(s['flyable'] and s['jettisonable'] and s['status']=='loaded' and s['weapon']=='0' for s in inventory),'Fresh authored loadout has unavailable/already released stores')


def unsigned(value,positive=True):
    return isinstance(value,str) and re.fullmatch(r'0|[1-9][0-9]*',value) is not None and (not positive or int(value)>0) and int(value)<2**64


def finite(values):return all(type(x) in (int,float) and math.isfinite(x) for x in values)


def body_state(world,body):
    state=body.state
    return dict(time_s=world.time,entity=str(body.id),diverged=bool(state.diverged),
        values=[*map(float,state.position_ecef),*map(float,state.velocity_body_ms),*map(float,state.angular_rate_body_rad_s),
            float(state.alpha_rad),float(state.beta_rad),float(state.mach),float(state.load_factor),float(state.airspeed_true_ms),float(body.mass_kg),float(state.altitude_msl_m)])


def valid_body(state):
    return unsigned(state['entity']) and state['diverged'] is False and len(state['values'])==16 and finite(state['values']) and state['values'][14]>0


def validate_scene(scene,raw,bindings,mode,epoch):
    """Pure independent admission: flags and summary counts cannot stand in for traces."""
    require(scene.get('passed') is True and raw.get('mode')==mode and raw.get('policy')==POLICY,'Missing/changed executed scene policy')
    require(raw.get('epoch_sha256')==epoch and raw.get('scope')==POLICY['scope'],'Wrong source/product epoch or referee scope')
    require(raw.get('initial_conditions')==dict(POLICY[mode],fuel_fraction=POLICY['fuel_fraction']),'Unbound or changed global scene fuel/initial conditions')
    require(raw['design']==bindings[0]['design'] and raw['loadout']==bindings[0]['loadout'],'Wrong authored loadout identity')
    validate_inventory(bindings if mode=='jettison' else raw['loadout_bindings'],raw['inventory_before'])
    require(mode=='jettison' or raw['loadout_bindings']==scene['loadout_bindings'],'Changed armed initialization loadout')
    inventory={s['index']:s for s in raw['inventory_before']}
    after={s['index']:s for s in raw['inventory_after']}
    require(len(after)==len(raw['inventory_after']) and set(after)==set(inventory),'Native inventory was dropped or duplicated after release')
    expected=Counter((b['payload'],b['station'],b['card']) for b in bindings for _ in range(b['count']))
    releases=raw['releases'];actual=Counter((r['payload_name'],r['station'],r['store']) for r in releases)
    require(actual==expected and len({r['entity'] for r in releases})==len(releases),'Missing/duplicate physical release')
    require(len({r['index'] for r in releases})==len(releases),'Store released more than once')
    require(len({(r['slot'],r['generation']) for r in releases})==len(releases),'Live release slot/generation was reused')
    require(all(c['entity'] in {r['entity'] for r in releases} for c in raw['controls']),'Unrelated native body supplied control evidence')
    require(type(raw['release_wait_steps']) is int and 0<raw['release_wait_steps']<=POLICY['release_steps'],'Unbounded or absent native release execution')
    require(type(raw['station_wait_steps']) is int and 0<=raw['station_wait_steps']<=POLICY['station_steps'],'Unbounded station warmup')
    pre_times=raw['pre_release_times'];step=POLICY['dt_s']*POLICY['frame_skip']
    require(len(pre_times)==1+raw['station_wait_steps']+raw['release_wait_steps'] and finite(pre_times) and abs(pre_times[0])<1e-9,'Warmup/release native execution was omitted')
    require(all(abs(b-a-step)<1e-7 for a,b in zip(pre_times,pre_times[1:])),'Warmup/release native steps changed')
    pre_states=raw['pre_release_states']
    require(all(valid_body(s) and s['time_s'] in pre_times for s in pre_states),'Invalid native body during warmup/release')
    pre_pairs=[(s['time_s'],s['entity']) for s in pre_states]
    require(len(set(pre_pairs))==len(pre_pairs),'Duplicate warmup/release native body observation')
    release_start=pre_times[raw['station_wait_steps']]
    for release in releases:
        launch=release['launch_time_s']
        require(finite([launch]) and launch>=0 and release_start<launch+1e-9 and launch<=pre_times[-1]+1e-9,'Launch lies outside its actual release interval')
        require(any(abs(launch-t)<1e-9 for t in pre_times[raw['station_wait_steps']+1:]),'Launch is not an actual post-step native release boundary')
    # flyReleases executes after integration and the time increment. A new body
    # is already queryable at its launch boundary, but has not integrated yet.
    expected_pre={(t,raw['launcher']) for t in pre_times}
    expected_pre.update((t,r['entity']) for r in releases for t in pre_times if t+1e-9>=r['launch_time_s'])
    require(set(pre_pairs)==expected_pre,'Missing, unknown or prelaunch body in warmup/ripple timeline')
    times=raw['times'];require(len(times)==POLICY['aftermath_steps']+1 and finite(times),'Missing complete native aftermath')
    require(times[0]==pre_times[-1],'Native aftermath is detached from actual release execution')
    require(all(abs(b-a-step)<1e-7 for a,b in zip(times,times[1:])),'Omitted/reordered native integration step')
    states=raw['states'];require(states,'No native body observations')
    observed={};seen=set()
    for state in states:
        require(valid_body(state) and state['time_s'] in times,'Invalid native body state')
        key=(state['time_s'],state['entity']);require(key not in seen,'Duplicate native body observation');seen.add(key)
        observed.setdefault(state['entity'],set()).add(state['time_s'])
    require(unsigned(raw['launcher']) and observed.get(raw['launcher'])==set(times),'Launcher aftermath missing')
    require(set(observed)=={raw['launcher']}|{r['entity'] for r in releases},'Unknown or missing body in live aftermath')
    for release in releases:
        require(all(unsigned(release[k]) for k in ('entity','activity','generation')) and type(release['slot']) is int and release['slot']>=0,'Invalid release correlation/generation')
        require(release['launcher']==raw['launcher'] and release['jettisoned'] is (mode=='jettison') and release['target']=='0','Unarmed release relabeled armed or correlation changed')
        before=inventory[release['index']];final=after[release['index']]
        require(release['activity']==raw['activities'][release['payload_name']]==release['expected_activity'],'Native release activity does not match submitted request')
        require((before['payload_name'],before['station'],before['store'])==(release['payload_name'],release['station'],release['store']),'Release used wrong authored station/store')
        require(all(final[k]==before[k] for k in ('payload_name','station','store')),'Native released inventory identity changed')
        require(final['weapon']==release['entity'] and final['status']==('jettisoned' if mode=='jettison' else 'released'),'Release did not consume exact native store')
        require(release['type']==next(b['model'] for b in bindings if b['card']==release['store']),'Wrong released native model')
        require(0<=release['launch_time_s']<=times[0] and finite([release['launch_time_s']]),'Invalid launch epoch')
        samples=[s for s in raw['controls'] if s['entity']==release['entity']]
        require(samples and observed.get(release['entity'])==set(times),'Released body lacks complete finite native integration')
        require(len(samples)==len(times) and {c['sample_time_s'] for c in samples}==set(times),'A native control observation was omitted or duplicated')
        for control in samples:
            require(control['generation']==release['generation'] and control['slot']==release['slot'] and control['sample_time_s'] in times,'Control entity/generation/epoch differs')
            require(len(control['commands'])==3 and finite(control['commands']) and all(abs(c)<=1+1e-8 for c in control['commands']),'Invalid executed native control')
        if mode=='armed' and bindings[0]['guided']:
            executed=[]
            for control in sorted(samples,key=lambda c:c['sample_time_s']):
                t=control['sample_time_s'];decision=control['controller_time_s'];launch=release['launch_time_s']
                if abs(t-launch)<1e-9:
                    require(control['controller_available'] is False and decision is None,'A launch-boundary body claimed a pre-integration controller decision')
                    continue
                require(control['controller_available'] is True and finite([decision]) and decision>=0 and decision>=launch-1e-9,'Missing or prelaunch native decision after integration')
                require(abs(decision-(t-POLICY['dt_s']))<1e-7,'Stale native controller decision; final substep was not observed')
                if executed:require(abs(decision-executed[-1]['controller_time_s']-step)<1e-7,'Native controller decision did not progress with integration')
                executed.append(control)
            require(executed,'No actual armed finite controller decision')
            require(all(c['available'] is True and c['fins'] and finite([c['controller_time_s']]) and c['controller_time_s']<=c['sample_time_s'] for c in executed),'Unavailable/unexecuted finite controller')
            for control in executed:
                require(len({f['id'] for f in control['fins']})==len(control['fins']),'Duplicate physical fins')
                for fin in control['fins']:
                    require(fin['id']>0 and fin['valid'] is True and finite([fin[k] for k in ('actual_rad','demand_rad','minimum_rad','maximum_rad','rate_rad_s')]) and fin['rate_rad_s']>0,'Invalid actual physical fin')
                    require(fin['minimum_rad']-1e-7<=fin['actual_rad']<=fin['maximum_rad']+1e-7 and fin['minimum_rad']<=fin['demand_rad']<=fin['maximum_rad'],'Native fin violates physical stops')
        else:require(not any(c['controller_available'] for c in samples),'Inapplicable inert/unguided controller became active')
    released_indices={r['index'] for r in releases}
    require(all(after[i]['status']==inventory[i]['status'] and after[i]['weapon']==inventory[i]['weapon'] for i in inventory if i not in released_indices),'Unselected authored store was consumed')
    outcomes=reconcile_population({r['entity'] for r in releases},raw['outcomes'],('entity',))
    for release in releases:
        outcome=outcomes[release['entity']]
        require(all(outcome[k]==release[k] for k in ('jettisoned','activity','store','station')),'Native final outcome identity changed')
        require(outcome['end']=='in_flight','Early native termination fails this fixed live-body integration scene')
    return len(releases)


def validate_receipt(receipt,population,producer,artifact_reader=read):
    require(receipt.get('schema')==SCHEMA and receipt.get('passed') is True,'Incomplete authored-fit receipt')
    require(producer.get('schema')=='fsim.sim-m02.production-install.v1' and producer.get('product_count')==len(producer.get('products',{})),'Invalid installed production contract')
    require(receipt['population']==population and receipt['policy']==POLICY,'Changed source denominator or scenario')
    require(receipt['producer_sha256']==digest(producer),'Wrong production epoch')
    require(receipt['products']==canonical_product_map(producer['products']),'Installed generated product set differs')
    require(receipt['source_sha256']==digest(population['sources']),'Changed source epoch')
    verify_runtime_inputs(receipt['runtime_inputs'],population,receipt['prefix'],receipt['python'])
    require(receipt['mass_preflight']==population['mass_preflight'] and receipt['mass_preflight']['fuel_fraction']==POLICY['fuel_fraction'],'Changed source-derived complete-loadout mass/fuel preflight')
    for row in receipt['mass_preflight']['rows']:
        require(receipt['products'].get(row['design']+'/'+row['design']+'.xml')==row['xml_sha256'],'Mass preflight differs from current installed generated XML')
    epoch=runtime_epoch(population,producer,receipt['loaded_modules'],receipt['python'],receipt['runtime_inputs'])
    require(epoch==receipt['epoch_sha256'],'Receipt epoch differs')
    require(receipt['prefix']==producer['prefix'],'Wrong installed prefix')
    modules=receipt['loaded_modules'];require(len({m['name'] for m in modules})==len(modules),'Duplicate native module')
    require(any(m['name']=='libfsim.dll' and m['sha256']==producer['native_dll_sha256'] for m in modules) and any('JSBSim' in m['name'] for m in modules),'Actual SDK/JSBSim modules not bound to producer')
    prefix=allowed(receipt['prefix']).resolve()
    for item in modules+list(receipt['python'].values()):
        require(allowed(item['path']).resolve().is_relative_to(prefix),'Loaded module outside installed prefix')
        require(re.fullmatch('[0-9a-f]{64}',item['sha256']) is not None,'Invalid loaded-module hash')
        verify_artifact(item['path'],item['sha256'])
    for relative,expected in receipt['products'].items():verify_artifact(prefix/'share/flightsim/aircraft'/relative,expected)
    loads=reconcile_population({loadout_identity(l) for l in population['loadouts']},receipt['loadouts'],('design','loadout'))
    armed=reconcile_population({identity(b) for b in population['bindings'] if b['armed']},receipt['armed'],('design','loadout','payload','station','card'))
    for key,scene in loads.items():
        bindings=[b for b in population['bindings'] if loadout_identity(b)==key]
        verify_artifact(scene['raw_path'],scene['raw_sha256']);raw=artifact_reader(scene['raw_path'])
        validate_scene(scene,raw,bindings,'jettison',epoch)
    for key,scene in armed.items():
        binding=next(b for b in population['bindings'] if identity(b)==key)
        require(scene['binding']==binding and scene['loadout_bindings']==[b for b in population['bindings'] if loadout_identity(b)==loadout_identity(binding)],'Changed exact armed fit')
        verify_artifact(scene['raw_path'],scene['raw_sha256']);raw=artifact_reader(scene['raw_path'])
        validate_scene(scene,raw,[binding],'armed',epoch)
    return dict(loadouts=len(loads),armed_station_bindings=len(armed),physical_jettisons=sum(b['count'] for b in population['bindings']),armed_physical_stores=sum(b['count'] for b in population['bindings'] if b['armed']))


def plain(value):
    if hasattr(value,'_asdict'):return {k:plain(v) for k,v in value._asdict().items()}
    if isinstance(value,(tuple,list)):return [plain(v) for v in value]
    return value


def native_inventory(vehicle):
    payloads={p.index:p.name for p in vehicle.payloads}
    return [dict(index=s.index,payload_name=payloads[s.payload],station=s.station,store=s.store,
        flyable=s.flyable,jettisonable=s.jettisonable,status=s.status,weapon=str(s.weapon)) for s in vehicle.stores]


def run_scene(fsim,prefix,bindings,all_bindings,mode,epoch,raw_path):
    """Fixed bounded native calls. Save partial raw observations on every exit."""
    first=bindings[0];raw=dict(design=first['design'],loadout=first['loadout'],mode=mode,policy=POLICY,
        scope=POLICY['scope'],epoch_sha256=epoch,loadout_bindings=all_bindings,inventory_before=[],inventory_after=[],
        initial_conditions=dict(POLICY[mode],fuel_fraction=POLICY['fuel_fraction']),releases=[],controls=[],states=[],times=[],outcomes=[],pre_release_times=[],pre_release_states=[],station_wait_steps=0,release_wait_steps=0)
    write(raw_path,raw)
    scene=dict(design=first['design'],loadout=first['loadout'],passed=False,raw_path=str(raw_path),
        loadout_bindings=all_bindings)
    if mode=='armed':scene.update({k:first[k] for k in ('payload','station','card')});scene['binding']=first
    with fsim.World('m03-authored-fit',publish=False,terrain=POLICY['terrain'],workers=1,pin_workers=False,
            seed=POLICY['seed'],dt=POLICY['dt_s'],frame_skip=POLICY['frame_skip'],jsbsim_root=str(prefix/'share/flightsim/jsbsim')) as world:
        try:
            vehicle=world.create_vehicle('authored-carrier','jsbsim:'+first['design'],**raw['initial_conditions']);raw['launcher']=str(vehicle.id)
            vehicle.set_protection('off')
            vehicle.command_actuator(aileron=0,elevator=0,rudder=0,throttle=1.,gear_down=0)
            vehicle.set_property('propulsion/set-running',-1.)
            if mode=='armed':vehicle.submit(fsim.Level.VELOCITY,airspeed_ms=250,vertical_speed_ms=0,heading_rad=math.pi/2)
            require(not vehicle.stores,'New authored carrier did not start empty')
            vehicle.load_loadout(first['loadout']);raw['inventory_before']=native_inventory(vehicle)
            validate_inventory(all_bindings,raw['inventory_before'])
            def observe_pre_release():
                raw['pre_release_times'].append(world.time)
                raw['pre_release_states'].extend(body_state(world,body) for body in world.vehicles())
            observe_pre_release()
            selected=[s for s in raw['inventory_before'] if any((s['payload_name'],s['station'],s['store'])==(b['payload'],b['station'],b['card']) for b in bindings)]
            by_payload={}
            for store in selected:by_payload.setdefault(store['payload_name'],[]).append(store['index'])
            if mode=='armed':
                for payload in by_payload:vehicle.set_payload_state(payload,'operate')
                for _ in range(POLICY['station_steps']):
                    world.step();raw['station_wait_steps']+=1;observe_pre_release()
                    if all(p.state==fsim.PayloadState.OPERATE for p in vehicle.payloads if p.name in by_payload):break
                else:raise ValueError('Actual authored station did not become operational within frozen budget')
            activities={}
            for payload,indices in by_payload.items():
                activity=(vehicle.jettison(stores=indices,emergency=True,payload=payload) if mode=='jettison' else
                    vehicle.strike(stores=indices,consent=True,target=None,payload=payload,ripple_s=.25))
                activities[payload]=str(activity)
            raw['activities']=activities
            for _ in range(POLICY['release_steps']):
                world.step();raw['release_wait_steps']+=1;observe_pre_release()
                current=native_inventory(vehicle)
                if all(s['weapon']!='0' for s in current if s['index'] in {x['index'] for x in selected}):break
                for activity in activities.values():
                    status=vehicle.strike_status(int(activity))
                    if status is not None and status.phase in ('failed','canceled'):raise ValueError('Native release failed: '+repr(status))
            else:raise ValueError('Actual authored release did not complete within frozen budget')
            raw['inventory_after']=native_inventory(vehicle)
            records={r.id:r for r in world.weapons}
            for store in selected:
                final=next(s for s in raw['inventory_after'] if s['index']==store['index']);record=records[int(final['weapon'])]
                control=world.weapon_controls(record.id);require(control is not None,'Native released body lacks generation identity')
                raw['releases'].append(dict(index=store['index'],payload_name=store['payload_name'],station=record.station,
                    store=record.store,entity=str(record.id),launcher=str(record.launcher),activity=str(record.activity),
                    expected_activity=activities[store['payload_name']],target=str(record.target),type=record.type,
                    jettisoned=record.jettisoned,launch_time_s=record.launch_time_s,slot=control.slot,generation=control.generation))
            def observe():
                raw['times'].append(world.time)
                for body in world.vehicles():
                    raw['states'].append(body_state(world,body))
                for release in raw['releases']:
                    control=world.weapon_controls(int(release['entity']))
                    if control is not None:
                        sample=plain(control);sample['entity']=str(sample['entity']);raw['controls'].append(sample)
            observe()
            for _ in range(POLICY['aftermath_steps']):world.step();observe()
            raw['inventory_after']=native_inventory(vehicle)
            raw['outcomes']=[dict(entity=str(r.id),end=r.end,jettisoned=r.jettisoned,
                activity=str(r.activity),store=r.store,station=r.station) for r in world.weapons]
            scene['passed']=True
            validate_scene(scene,raw,bindings,mode,epoch)
        except Exception as error:
            scene['passed']=False;raw['error']=str(error);raise
        finally:
            # Preserve invalid numeric evidence in failed rows without making JSON invalid.
            def safe(value):
                if isinstance(value,float) and not math.isfinite(value):return None
                if isinstance(value,dict):return {k:safe(v) for k,v in value.items()}
                if isinstance(value,(list,tuple)):return [safe(v) for v in value]
                return value
            write(raw_path,safe(raw));scene['raw_sha256']=sha(raw_path)
    return scene


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('prefix','coverage','producer','output'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();prefix=allowed(args.prefix).resolve();output=allowed(args.output).resolve()
    require(not output.exists(),'Fresh authored-fit receipt required')
    require(prefix.is_relative_to(Path('D:/ENV').resolve()),'Installed SDK belongs under D:/ENV')
    population=derive_population();coverage=read(args.coverage);compare_coverage(population,coverage);producer=read(args.producer)
    require(producer.get('schema')=='fsim.sim-m02.production-install.v1' and Path(producer['prefix']).resolve()==prefix,'Wrong installed producer')
    products=canonical_product_map(producer['products']);require(producer['product_count']==len(products),'Wrong product denominator')
    for relative,expected in products.items():verify_artifact(prefix/'share/flightsim/aircraft'/relative,expected)
    for relative,expected in population['sources'].items():
        if relative.startswith('assets/stores/'):verify_artifact(prefix/'share/flightsim/stores'/Path(relative).name,expected)
        elif relative.startswith('assets/payloads/'):verify_artifact(prefix/'share/flightsim/payloads'/Path(relative).name,expected)
        elif relative.endswith('/payloads.json'):verify_artifact(prefix/'share/flightsim/aircraft'/Path(relative).relative_to('aircraft'),expected)
    for key in tuple(os.environ):
        if key.startswith('FSIM_') and key!='FSIM_MAPS_DIR':os.environ.pop(key)
    os.environ['FSIM_DLL_DIR']=str(prefix/'bin')
    os.environ['FSIM_AIRCRAFT_PATH']=os.pathsep.join(str(prefix/'share/flightsim/aircraft'/p) for p in ('','stores'))
    # Explicit installed DLL load prevents the facade's development fallback.
    dll_directory=os.add_dll_directory(str(prefix/'bin'));library=ctypes.WinDLL(str(prefix/'bin/libfsim.dll'))
    sys.path.insert(0,str(prefix));import fsim
    fsim.set_log_level('off')
    sys.path.insert(0,str(ROOT/'tools/validation/radar'));from provenance import loaded_modules
    world_module=sys.modules[fsim.World.__module__]
    state_module=sys.modules[world_module.VehicleState.__module__]
    require(world_module.__name__=='fsim.world' and state_module.__name__=='fsim._state','Unexpected public implementation/state module owner')
    python={name:dict(path=str(path:=Path(module.__file__).resolve()),sha256=sha(path)) for name,module in
        (('facade',fsim),('native',fsim._native),('world',world_module),('state',state_module))}
    require(all(Path(item['path']).is_relative_to(prefix) for item in python.values()),'Python package outside installed product')
    modules=loaded_modules(prefix);verify_artifact(prefix/'bin/libfsim.dll',producer['native_dll_sha256'])
    runtime=derive_runtime_inputs(population,prefix,python);verify_runtime_inputs(runtime,population,prefix,python)
    epoch=runtime_epoch(population,producer,modules,python,runtime)
    result=dict(schema=SCHEMA,passed=False,prefix=producer['prefix'],population=population,policy=POLICY,
        source_sha256=digest(population['sources']),producer_sha256=digest(producer),products=products,
        loaded_modules=modules,python=python,runtime_inputs=runtime,mass_preflight=population['mass_preflight'],epoch_sha256=epoch,loadouts=[],armed=[],
        coverage=dict(path=str(args.coverage.resolve()),sha256=sha(args.coverage)),
        producer=dict(path=str(args.producer.resolve()),sha256=sha(args.producer)),
        installed_aircraft_search_roots=os.environ['FSIM_AIRCRAFT_PATH'].split(os.pathsep))
    artifacts=output.parent/(output.stem+'-records');require(not artifacts.exists(),'Fresh raw scene directory required');artifacts.mkdir(parents=True)
    write(output,result)
    try:
        for index,loadout in enumerate(population['loadouts']):
            bindings=[b for b in population['bindings'] if loadout_identity(b)==loadout_identity(loadout)]
            print('authored loadout '+loadout['design']+' / '+loadout['loadout'],flush=True)
            result['active_scene']=dict(mode='jettison',design=loadout['design'],loadout=loadout['loadout'],raw_path=str(artifacts/(str(index)+'-jettison.json')));write(output,result)
            result['loadouts'].append(run_scene(fsim,prefix,bindings,bindings,'jettison',epoch,artifacts/(str(index)+'-jettison.json')));write(output,result)
            for binding_index,binding in enumerate(bindings):
                if not binding['armed']:continue
                print('authored armed '+repr(identity(binding)),flush=True)
                result['active_scene']=dict(mode='armed',identity=list(identity(binding)),raw_path=str(artifacts/(str(index)+'-'+str(binding_index)+'-armed.json')));write(output,result)
                result['armed'].append(run_scene(fsim,prefix,[binding],bindings,'armed',epoch,artifacts/(str(index)+'-'+str(binding_index)+'-armed.json')));write(output,result)
        require(derive_population()==population,'Authored source changed during execution')
        verify_runtime_inputs(runtime,population,prefix,python)
        for relative,expected in products.items():verify_artifact(prefix/'share/flightsim/aircraft'/relative,expected)
        for item in modules+list(python.values()):verify_artifact(item['path'],item['sha256'])
        result.pop('active_scene',None)
        result['passed']=True;result['admitted']=validate_receipt(result,population,producer);write(output,result)
        print(json.dumps(result['admitted']),flush=True)
    except Exception as error:result['passed']=False;result['error']=str(error);write(output,result);raise


if __name__=='__main__':main()
