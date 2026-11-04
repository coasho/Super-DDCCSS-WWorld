"""Execute every AAM through installed C++, C99 and Python SDK consumers."""
import argparse
import ast
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

from acceptance import allowed, verify_artifact, WorldHealthV1, canonical_product_map
from component_matrix import derive_cells, binding, bound, read as strict_read
from fleet import carrier_for

ROOT=Path(__file__).resolve().parents[3]
LANGUAGES=('cpp','c99','python')


def checked_path(value):return allowed(allowed(value).resolve())
def read(value):return strict_read(checked_path(value))
def sha(value):return hashlib.sha256(checked_path(value).read_bytes()).hexdigest()
def require(value,message):
    if not value:raise ValueError(message)
def save(path,value):checked_path(path).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n',encoding='utf-8')


def facade_paths(root=ROOT):
    """Follow explicit local SDK imports without enumerating directories."""
    package=checked_path(root/'python/fsim');pending=[package/'__init__.py'];visited=set()
    def add(module):
        module=checked_path(module);require(module.is_relative_to(package),'Facade import escapes current source')
        candidates=(module/'__init__.py',) if module==package else (module.with_suffix('.py'),module/'__init__.py')
        for p in candidates:
            p=checked_path(p);require(p.is_relative_to(package),'Facade import escapes current source')
            if p.is_file() and p not in visited:pending.append(p)
    while pending:
        p=checked_path(pending.pop())
        if p in visited:continue
        visited.add(p);require(len(visited)<=512,'Bounded explicit facade import closure exceeded')
        for node in ast.walk(ast.parse(p.read_text(encoding='utf-8'))):
            if isinstance(node,ast.ImportFrom):
                if node.level:
                    base=p.parent
                    for _ in range(node.level-1):base=base.parent
                    module=base.joinpath(*(node.module or '').split('.')) if node.module else base
                elif node.module and (node.module=='fsim' or node.module.startswith('fsim.')):
                    module=package.joinpath(*node.module.split('.')[1:])
                else:continue
                add(module)
                for alias in node.names:
                    if alias.name!='*':add(module/alias.name)
            elif isinstance(node,ast.Import):
                for alias in node.names:
                    if alias.name.startswith('fsim.'):add(package.joinpath(*alias.name.split('.')[1:]))
    return sorted(visited)


def freeze_sources(coverage,root=ROOT):
    from full_chain import scene_input_paths
    names={row['path'] for cell in derive_cells(coverage,root) for row in cell['sources']}
    names.update('tools/validation/weapon_quality/'+name for name in
        ('component_public.py','test_component_public.py','aam_public_capture.h','aam_public.cpp','aam_public.c',
         'installed.py','CMakeLists.txt','fleet.py','acceptance.py'))
    names.add('tools/validation/weapon_quality/full_chain.py')
    names.update('include/fsim/'+name for name in
        ('World.h','Agent.h','MissionSystems.h','WeaponControls.h','fsim_c.h','VehicleState.h','InitialConditions.h'))
    names.update(('python/fsim/world.py','python/fsim/__init__.py','tools/validation/radar/provenance.py'))
    names.update(p.relative_to(checked_path(root)).as_posix() for p in facade_paths(root))
    for fit in coverage['fits']:
        if not fit.get('fit_sha256'):continue
        fit_path=allowed(fit['fit_path']);names.add(fit_path.as_posix())
        for payload in read(root/fit_path).get('payloads',[]):
            name='assets/payloads/'+payload['card']+'.json';allowed(name)
            require((root/name).is_file(),'Missing actual carrier payload source');names.add(name)
    carriers={carrier_for(read(root/r['card_path']),coverage)['carrier'] for r in coverage['stores'] if r['class']=='aam'}
    names.update(scene_input_paths(coverage,root,airframes=tuple(sorted(carriers))))
    return dict(schema='fsim.sim-m03.component-public-epoch.v1',
        sources=[binding(root/name,root) for name in sorted(names)])


def verify_installed(installed,prefix,root=ROOT):
    require(installed.get('schema')=='fsim.sim-m03.installed-consumers.v1' and installed.get('passed') is True and
        checked_path(installed['prefix'])==checked_path(prefix),'Passing matching installed receipt required')
    source_rows={}
    for row in installed['provenance']+installed['component_client_sources']:
        source=checked_path(row['source']);product=checked_path(row['installed'])
        require(source.is_relative_to(checked_path(root)),'Installed source identity is outside current source root')
        if row in installed['provenance']:require(product.is_relative_to(checked_path(prefix)),'Installed resource outside product')
        verify_artifact(source,row['sha256']);verify_artifact(product,row['sha256']);source_rows[source]=row
    require(all(path in source_rows for path in facade_paths(root)),'Installed receipt omits used facade sources')
    for name in ('aam_public.cpp','aam_public.c','aam_public_capture.h','CMakeLists.txt'):
        require(checked_path(root/'tools/validation/weapon_quality'/name) in source_rows,'Unbound public consumer source')
    producer_path=bound(installed['producer'],root);production=read(producer_path)
    require(checked_path(production['prefix'])==checked_path(prefix),'Installed model products use another prefix')
    products=canonical_product_map(production['products']);require(products,'No installed native model products')
    require(production.get('schema')=='fsim.sim-m02.production-install.v1' and
        production.get('product_count')==len(products),'Normal complete producer manifest required')
    for name,digest in products.items():verify_artifact(prefix/'share/flightsim/aircraft'/name,digest)
    for name in ('libfsim.dll','libJSBSim.dll'):
        product=checked_path(prefix/'bin'/name)
        require(any(checked_path(r['installed'])==product for r in installed['provenance']),'Unbound installed SDK binary')
        if name=='libfsim.dll':require(sha(product)==production.get('native_dll_sha256'),'Installed SDK differs from normal producer')
    pyd=checked_path(installed['python_loaded_modules']['native_python'])
    require(pyd.is_relative_to(checked_path(prefix)),'Python extension outside installed product')
    verify_artifact(pyd,installed['python_loaded_modules']['native_python_sha256'])


def load_csv(path):
    with checked_path(path).open(encoding='utf-8',newline='') as stream:result=list(csv.DictReader(stream))
    require(result and all(None not in row for row in result),'Empty/malformed actual public observations')
    return result


def validate_case(capture,directory,language,model,native_model,epoch_sha,prefix,prepared,root=ROOT):
    """Reconcile captured native results, raw events and actual loaded products."""
    require(capture.get('schema')=='fsim.sim-m03.public-aam-capture.v1','Actual public capture schema required')
    require(capture.get('language')==language and capture.get('model')==model and
        capture.get('native_model')==native_model,'Foreign public model/language identity')
    require(capture.get('input_epoch_sha256')==epoch_sha,'Public producer input epoch mismatch')
    for field in ('weapon','target','checks','observations','control_samples','tracking_samples','burning_samples','damage_events'):
        require(type(capture.get(field)) is int and capture[field]>0,'Actual positive public '+field+' required')
    require(capture.get('terminal')=='detonated','Actual public native detonation required')
    launch,stop=capture['launch_s'],capture['stop_s']
    require(all(type(v) in (int,float) and math.isfinite(v) for v in (launch,stop)) and
        40<=stop-launch<40+1/30+1e-8,'Complete unchanged public40s encounter required')
    actual=capture.get('modules',[])
    require(type(actual) is list and len(actual)==(4 if language=='python' else 3),
        'Actual executable, SDK modules and Python extension when used required')
    observed={}
    for item in actual:
        path=checked_path(item['path']);name=path.name.casefold()
        require(name not in observed,'Duplicate loaded public module')
        expected=prepared.get(name);require(expected is not None,'Unprepared public producer module')
        require(item.get('sha256')==expected['sha256'],'Capture was produced by other module bytes')
        if name.endswith(('.dll','.pyd')):require(path.is_relative_to(checked_path(prefix)),'Public module outside installed product')
        else:
            expected_path=allowed(expected['path'])
            require(path==checked_path(expected_path if expected_path.is_absolute() else root/expected_path),'Public process executable changed')
        verify_artifact(path,expected['sha256']);observed[name]=binding(path,root)
    require(set(observed)==set(prepared),'Incomplete actual loaded public product')
    files=capture.get('files',[])
    require(type(files) is list and len(files)==2 and {r.get('path') for r in files}=={'trace.csv','events.csv'},
        'Exactly both closed public output bindings required')
    for item in files:verify_artifact(directory/allowed(item['path']),item['sha256'])
    trace=load_csv(directory/'trace.csv');events=load_csv(directory/'events.csv')
    require(len(trace)==capture['observations'],'Public observation count differs from actual trace')
    counts=dict(control_samples=0,tracking_samples=0,burning_samples=0);last=-1;ended=False
    for row in trace:
        require(row['model']==model and int(row['weapon'])==capture['weapon'],'Foreign public trace lifetime')
        current,age=float(row['time']),float(row['age'])
        require(math.isfinite(current) and math.isfinite(age) and current>last and
            abs((current-age)-launch)<1e-8,'Invalid public native sample clock')
        require(abs((current-(last if last>=0 else launch))-1/30)<1e-8,
            'Missing/reordered fixed-cadence public sample')
        last=current
        for field,column in (('control_samples','controller'),('tracking_samples','tracking'),('burning_samples','burning')):
            require(row[column] in ('0','1'),'Nonboolean native observation')
            counts[field]+=int(row[column])
        require(int(row['fin_count'])>=0,'Invalid public fin count')
        flight,terminal=('in_flight','detonated') if language=='python' else ('0','3')
        require(row['terminal'] in (flight,terminal) and (not ended or row['terminal']==terminal),
            'Invalid/reversed native terminal transition')
        ended=ended or row['terminal']==terminal
    require(abs(last-stop)<1e-8 and all(counts[k]==capture[k] for k in counts),'Public summary is not its actual trace')
    terminal='detonated' if language=='python' else '3'
    require(trace[-1]['terminal']==terminal,'Public capture terminal differs from actual native trace')
    damages=0;seen=set()
    for row in events:
        require(row['model']==model and int(row['weapon'])==capture['weapon'],'Foreign public event lifetime')
        serial=int(row['serial']);require(serial>0 and serial not in seen,'Duplicate/invalid public event');seen.add(serial)
        event_time=float(row['time']);require(math.isfinite(event_time) and event_time<=stop,'Future/nonfinite public event')
        damages+=row['kind']=='damage' and int(row['source'])==capture['weapon'] and int(row['target'])==capture['target']
    require(damages==capture['damage_events'],'No matching actual public target damage events')
    return dict(language=language,model=model,native_model=native_model,checks=capture['checks'],
        observations=len(trace),modules=list(observed.values()),damage_events=damages,
        raw=[binding(directory/name,root) for name in ('capture.json','trace.csv','events.csv')])


def invocation(language,model,card,selection,executable,prefix,directory,epoch_sha):
    if language=='python':
        return [str(executable),'-B',str(checked_path(ROOT/'tools/validation/weapon_quality/component_public.py')),
            '--python-scene','--prefix',str(prefix),'--output',str(directory),'--card',model,
            '--carrier',selection['carrier'],'--station',str(selection['station']),'--epoch-sha',epoch_sha]
    return [str(executable),str(prefix),model,card['model'],selection['carrier'],str(selection['station']),
        str(int(card['seeker']=='semi_active_radar')),epoch_sha,str(directory)]


def sdk_environment(prefix):
    return dict(FSIM_DLL_DIR=str(prefix/'bin'),FSIM_AIRCRAFT_PATH=os.pathsep.join(
        str(prefix/'share/flightsim/aircraft'/part) for part in ('','stores')))


def admit(coverage,run_path,installed_path,root=ROOT):
    """Read-only reconstruction of all72 executions and consumed product bytes."""
    run_path=checked_path(run_path);installed_path=checked_path(installed_path);record=read(run_path)
    require(record.get('schema')=='fsim.sim-m03.component-public-run.v1' and record.get('passed') is True and
        record.get('failures')==[],'Complete passing installed component run required')
    require(bound(record['installed'],root)==installed_path,'Public run uses another installed producer')
    installed=read(installed_path);prefix=checked_path(installed['prefix']);verify_installed(installed,prefix,root)
    epoch_path=bound(record['source_epoch'],root);epoch=read(epoch_path)
    require(epoch==freeze_sources(coverage,root),'Public component input epoch changed')
    cells=derive_cells(coverage,root);models=sorted({c['model'] for c in cells})
    expected={(m,l) for m in models for l in LANGUAGES};executions=record['executions']
    pairs=[(r['model'],r['language']) for r in executions]
    require(len(pairs)==72 and len(set(pairs))==72 and set(pairs)==expected,'Actual72 distinct public executions required')
    observations=[];raw=[binding(epoch_path,root),binding(installed_path,root),binding(run_path,root)]
    dlls={name.casefold():binding(prefix/'bin'/name,root) for name in ('libfsim.dll','libJSBSim.dll')}
    for entry in executions:
        model,language=entry['model'],entry['language'];directory=run_path.parent/language/model
        card=read(root/'assets/stores'/(model+'.json'))
        verify_artifact(prefix/'share/flightsim/stores'/(model+'.json'),sha(root/'assets/stores'/(model+'.json')))
        selection=carrier_for(card,coverage);client=installed['component_clients'][language]
        executable=bound(client,root);prepared={**dlls,executable.name.casefold():binding(executable,root)}
        if language=='python':
            pyd=prefix/'fsim/_native.pyd';prepared[pyd.name.casefold()]=binding(pyd,root)
        require(entry['exit_code']==0 and type(entry['exit_code']) is int,'Failed public consumer process')
        require(entry['command']==invocation(language,model,card,selection,executable,prefix,directory,sha(epoch_path)) and
            entry['environment']==sdk_environment(prefix),'Public consumer command/environment differs from fixed scene')
        require(entry['prepared']==list(prepared.values()),'Public prepared runtime differs from installed producer')
        require(type(entry['elapsed_s']) in (int,float) and math.isfinite(entry['elapsed_s']) and entry['elapsed_s']>=0,
            'Invalid actual public process duration')
        expected_files={'capture.json','trace.csv','events.csv','run.log'}|({'world-health.json'} if language=='python' else set())
        require(len(entry['raw'])==len(expected_files) and
            {bound(r,root) for r in entry['raw']}=={checked_path(directory/name) for name in expected_files},
            'Incomplete or foreign actual public process outputs')
        value=validate_case(read(directory/'capture.json'),directory,language,model,card['model'],sha(epoch_path),prefix,prepared,root)
        if language=='python':
            health=read(directory/'world-health.json');require(health['rows'] and health['invalid_entities']==[],
                'Invalid complete native public world')
            trace_epochs={float(r['time']) for r in load_csv(directory/'trace.csv')}
            health_epochs=set()
            for row in health['rows']:
                require(type(row) is list and len(row)==17 and row[1]>0 and row[3] is False and
                    all(type(v) in (int,float) and math.isfinite(v) for v in [row[0],*row[4:]]),
                    'Nonfinite/diverged actual public world state')
                require(row[0] in trace_epochs,'Public world state has no matching native observation');health_epochs.add(row[0])
            require(trace_epochs==health_epochs,'Incomplete actual public world observation epochs')
        observations.append(value);raw.extend(entry['raw'])
    require(record['models']==observations,'Public model summary differs from reconstructed executions')
    proof=dict(schema='fsim.sim-m03.component-public-installed.v1',passed=True,sources=epoch['sources'],raw=raw,models=[])
    for cell in cells:
        actual=[r for r in observations if r['model']==cell['model']]
        proof['models'].append(dict(**{k:cell[k] for k in ('model','component','implementation_id','mechanism_id','card_sha256','parameter_sha256')},
            passed=True,public_installed=dict(case='Actual installed SDK24-model C++/C99/Python native encounters',tests=3,
                assertions=sum(r['checks'] for r in actual),failures=0,errors=0,skipped=0,per_model_execution=True,
                native_model=cell['native_model'],languages=list(LANGUAGES),loaded_identity_verified=True,executions=actual)))
    return proof


def python_scene(args):
    """Public Python execution, with actor execution and referee evidence separate."""
    import fsim
    sys.path.insert(0,str(checked_path(ROOT/'tools/validation/radar')))
    from provenance import loaded_modules
    prefix=checked_path(args.prefix);output=checked_path(args.output)
    require(checked_path(fsim.__file__).is_relative_to(prefix),'Python facade outside installed product')
    card=read(prefix/'share/flightsim/stores'/(args.card+'.json'));checks=0
    def check(value,message):
        nonlocal checks
        checks+=1;require(value,message)
    with fsim.World('m03-installed-python-'+args.card,publish=False,terrain=False,workers=1,pin_workers=False,
        seed=19,dt=1/120,frame_skip=4,jsbsim_root=str(prefix/'share/flightsim/jsbsim')) as world:
        carrier=world.create_vehicle('launcher','jsbsim:'+args.carrier,latitude_deg=40,longitude_deg=0,
            altitude_msl_m=6000,heading_deg=90,airspeed_ms=250)
        launcher=world.create_agent(carrier,'launcher')
        launcher.submit(fsim.Level.VELOCITY,airspeed_ms=250,vertical_speed_ms=0,heading_rad=math.pi/2)
        c2=world.create_c2_node('approver',39.9,0,altitude_m=200);net=world.create_net('consent')
        check(world.join_net(carrier,net) and world.join_net(c2,net),'Consent network refused')
        approver=world.create_agent(c2,'approver');check(approver.declare_role(fsim.ROLE_QB,'APPROVER'),'Role refused')
        payload=next(p.index for p in launcher.payloads if p.family=='store_station')
        radar=next(p.index for p in launcher.payloads if p.family=='air_radar')
        carrier.load_store(args.station,args.card,payload=payload)
        for p in (payload,radar):launcher.set_payload_state(p,'operate')
        for _ in range(1350):
            world.step()
            if sum(p.state==fsim.PayloadState.OPERATE for p in launcher.payloads if p.index in (payload,radar))==2:break
        else:raise ValueError('Native payload warmup failed')
        state=carrier.state
        target=world.create_vehicle('opponent','jsbsim:f16c',latitude_deg=math.degrees(state.latitude_rad),
            longitude_deg=math.degrees(state.longitude_rad)+math.degrees(5000/(6371000*math.cos(state.latitude_rad))),
            altitude_msl_m=6000,heading_deg=270,airspeed_ms=250)
        opponent=world.create_agent(target,'opponent')
        opponent.submit(fsim.Level.VELOCITY,airspeed_ms=250,vertical_speed_ms=0,heading_rad=3*math.pi/2)
        launcher.amti_search(radar,bars=1,azimuth_half_width_rad=math.radians(15),range_scale_m=60000)
        track=None
        for _ in range(600):
            world.step();track=next((t for t in launcher.amti_tracks(radar) if t.updates>=3),None)
            if track is not None:break
        check(track is not None,'No actual permitted radar track')
        designation=launcher.designate(radar,track.id);check(designation,'Designation refused')
        launcher.amti_track(track.id,radar,illuminate=card['seeker']=='semi_active_radar',range_scale_m=60000)
        activity=launcher.strike(args.card,target=designation,payload=payload)
        check(not world.weapons,'Actor strike bypassed consent');answered=False
        for _ in range(1350):
            world.step()
            for report in approver.report_consents().items:
                request=report.value
                if request.strike==activity:
                    check(report.meta.source=='dms' and not world.weapons,'Consent bypassed modeled delivery')
                    check(approver.answer_consent(request.source,request.request_id),'Answer refused');answered=True;break
            if answered:break
        check(answered,'No delivered consent')
        for _ in range(600):
            if world.weapons:break
            world.step()
        check(len(world.weapons)==1,'No correlated actual release')
        released=world.weapons[0];weapon=released.id;launch=released.launch_time_s
        check(released.store==args.card and not released.jettisoned and released.launcher==carrier.id,'Foreign/unarmed release')
        check(world.vehicle(weapon).type==card['model'],'Wrong native model identity')
        health=WorldHealthV1();samples=controls=tracking=burning=0
        with (output/'trace.csv').open('w',encoding='utf-8',newline='') as file:
            writer=csv.writer(file);writer.writerow(('model','weapon','time','age','tracking','burning','controller','fin_count','terminal'))
            while world.time<launch+40:
                world.step();row=next(r for r in world.weapons if r.id==weapon);samples+=1
                tracking+=bool(row.tracking);burning+=bool(row.burning)
                control=world.weapon_controls(weapon);available=bool(control and control.controller_available);controls+=available
                writer.writerow((args.card,weapon,world.time,world.time-launch,int(row.tracking),int(row.burning),int(available),len(control.fins) if control else 0,row.end))
                health.observe(world);check(not health.invalid,'Native world diverged')
        events=world.engagement_events();damage=sum(e.entity==weapon and e.target==target.id and e.kind=='damage' for e in events)
        with (output/'events.csv').open('w',encoding='utf-8',newline='') as file:
            writer=csv.writer(file);writer.writerow(('model','weapon','serial','time','kind','source','target'))
            for e in events:writer.writerow((args.card,weapon,e.serial,e.time_s,e.kind,e.entity,e.target))
        final=next(r for r in world.weapons if r.id==weapon)
        check(samples and tracking and burning and controls,'Missing actual native acquisition/control/propulsion')
        check(final.end=='detonated' and damage,'Native detonation/target damage absent')
        capture=dict(schema='fsim.sim-m03.public-aam-capture.v1',language='python',model=args.card,native_model=card['model'],
            input_epoch_sha256=args.epoch_sha,weapon=int(weapon),target=int(target.id),checks=checks,observations=samples,
            control_samples=controls,tracking_samples=tracking,burning_samples=burning,damage_events=damage,
            launch_s=launch,stop_s=world.time,terminal=final.end,
            modules=[dict(path=str(checked_path(p)),sha256=sha(p)) for p in (sys.executable,fsim._native.__file__)]+
                [dict(path=r['path'],sha256=r['sha256']) for r in loaded_modules(prefix)],
            files=[dict(path=name,sha256=sha(output/name)) for name in ('trace.csv','events.csv')])
        save(output/'world-health.json',dict(rows=health.rows,invalid_entities=sorted(health.invalid)))
        save(output/'capture.json',capture)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ('prefix','output'):parser.add_argument('--'+key,type=Path,required=True)
    parser.add_argument('--coverage',type=Path);parser.add_argument('--installed',type=Path)
    parser.add_argument('--python-scene',action='store_true')
    for key in ('card','carrier','epoch-sha'):parser.add_argument('--'+key)
    parser.add_argument('--station',type=int);parser.add_argument('--only-model')
    args=parser.parse_args()
    for p in (args.prefix,args.output,args.coverage,args.installed):
        if p is not None:checked_path(p)
    if args.python_scene:return python_scene(args)
    require(args.coverage and args.installed,'Current coverage and installed producer receipt required')
    execute(args)


def execute(args):
    prefix=checked_path(args.prefix);output=checked_path(args.output)
    require(not output.exists(),'Fresh installed component output required');output.mkdir(parents=True)
    coverage=read(args.coverage);installed=read(args.installed)
    verify_installed(installed,prefix)
    clients=installed['component_clients'];epoch=freeze_sources(coverage);epoch_path=output/'source-epoch.json';save(epoch_path,epoch)
    environment={key:value for key,value in os.environ.items() if not key.startswith('FSIM_')}
    environment.update(TMP='D:/ENV/tmp',TEMP='D:/ENV/tmp',TORCH_EXTENSIONS_DIR='D:/ENV/tmp',
        PYTHONDONTWRITEBYTECODE='1',PYTHONPATH=str(prefix),**sdk_environment(prefix),
        PATH=str(prefix/'bin')+os.pathsep+str(Path(os.environ.get('SystemRoot','C:/Windows'))/'System32'))
    expected_dlls={name.casefold():binding(prefix/'bin'/name) for name in ('libfsim.dll','libJSBSim.dll')}
    record=dict(schema='fsim.sim-m03.component-public-run.v1',passed=False,engineering_acceptance=False,
        source_epoch=binding(epoch_path),installed=binding(args.installed),executions=[],models=[],failures=[])
    all_models=sorted(row['card'] for row in coverage['stores'] if row['class']=='aam')
    require(len(all_models)==24,'Exact authored24 AAM population required')
    selected=[args.only_model] if args.only_model else all_models
    require(set(selected)<=set(all_models),'Unknown actual public model')
    started=time.perf_counter()
    for model in selected:
        card=read(prefix/'share/flightsim/stores'/(model+'.json'))
        require(sha(prefix/'share/flightsim/stores'/(model+'.json'))==sha(ROOT/'assets/stores'/(model+'.json')),'Installed card differs from current source')
        selection=carrier_for(card,coverage)
        for language in LANGUAGES:
            directory=output/language/model;directory.mkdir(parents=True)
            executable=checked_path(clients[language]['path']);verify_artifact(executable,clients[language]['sha256'])
            prepared={**expected_dlls,executable.name.casefold():binding(executable)}
            if language=='python':
                extension=prefix/'fsim/_native.pyd';prepared[extension.name.casefold()]=binding(extension)
            command=invocation(language,model,card,selection,executable,prefix,directory,sha(epoch_path))
            entry=dict(language=language,model=model,command=command,environment={k:environment[k] for k in environment if k.startswith('FSIM_')},
                prepared=list(prepared.values()),exit_code=None);record['executions'].append(entry);tick=time.perf_counter()
            try:
                with (directory/'run.log').open('wb') as log:
                    process=subprocess.Popen(command,cwd=output,env=environment,stdout=log,stderr=subprocess.STDOUT)
                    try:entry['exit_code']=process.wait(timeout=240)
                    except subprocess.TimeoutExpired:
                        subprocess.run(['taskkill','/PID',str(process.pid),'/T','/F'],stdout=log,stderr=subprocess.STDOUT,timeout=30)
                        process.wait(timeout=30);raise
                require(entry['exit_code']==0,'Actual installed '+language+' '+model+' failed')
                value=validate_case(read(directory/'capture.json'),directory,language,model,card['model'],sha(epoch_path),prefix,prepared)
                for source in epoch['sources']:verify_artifact(ROOT/source['path'],source['sha256'])
                record['models'].append(value)
            except Exception as error:record['failures'].append(dict(language=language,model=model,error=str(error)))
            finally:
                entry['elapsed_s']=time.perf_counter()-tick
                entry['raw']=[binding(directory/name) for name in ('capture.json','trace.csv','events.csv','world-health.json','run.log')
                    if (directory/name).is_file()]
                record['elapsed_s']=time.perf_counter()-started;save(output/'run.json',record)
    verify_installed(installed,prefix)
    record['passed']=len(record['models'])==72 and not record['failures'] and set(selected)==set(all_models)
    save(output/'run.json',record)
    if record['passed']:
        proof=admit(coverage,output/'run.json',args.installed)
        save(output/'components.json',proof)
    print(json.dumps(dict(passed=record['passed'],executed=len(record['executions']),passed_cases=len(record['models']),failures=record['failures'],receipt=binding(output/'run.json'))))
    require(record['passed'] or args.only_model and not record['failures'],'Complete actual installed component population failed')


if __name__=='__main__':main()
