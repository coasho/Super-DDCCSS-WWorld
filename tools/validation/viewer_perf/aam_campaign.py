"""All-AAM native kill-chain diagnostics and live full-scene visualization; no solver/calibration.

Run through run.ps1 -Mode python. Scenarios are frozen before execution; launch
refusals and misses remain results. Equipment accuracy is tracked separately.
"""
import argparse
import hashlib
import html
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import time
import sys

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools/models/hangar'))
from hangar import equipment_models


def read(path):return json.loads(Path(path).read_text(encoding='utf-8'))
def write(path,value):Path(path).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n',encoding='utf-8')
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def summarize(recording,evidence):
    """Use recorded native state; a low-speed short encounter is not max-Mach proof."""
    import fsim
    import numpy as np
    weapon=evidence['weapons'][0];slot=weapon['weapon_slot'];target=evidence['target_slot']
    rows=[];world_times=[];closest=math.inf
    reader=fsim.Recording(recording)
    for frame_time,samples in reader:
        missiles=samples[samples['slot']==slot]
        if not len(missiles):continue
        state=missiles[0]['state']
        # Recording time and lifecycle bounds use the World clock. The
        # vehicle's sim_time is local age and can restart when a slot is reused.
        if float(frame_time)<weapon['launch_time_s']-1e-9:continue
        if weapon['end_time_s'] is not None and float(frame_time)>=weapon['end_time_s']-1e-9:continue
        rows.append(state.copy());world_times.append(float(frame_time))
        opponents=samples[samples['slot']==target]
        if len(opponents):closest=min(closest,float(np.linalg.norm(state['position_ecef']-opponents[0]['state']['position_ecef'])))
    if not rows:raise ValueError('Released weapon has no native recorded flight samples')
    def peak(key):
        values=np.asarray([r[key] for r in rows],float)
        return float(np.max(np.abs(values))) if np.isfinite(values).all() else None
    functional=evidence.get('target_response',{}).get('functional_events',[])
    events=[e for e in evidence.get('engagements',[]) if e['entity']==weapon['id']]
    counts={kind:sum(e['kind']==kind for e in events) for kind in sorted({e['kind'] for e in events})}
    reverse_index=next((i for i,r in enumerate(rows) if math.cos(r['alpha_rad'])*math.cos(r['beta_rad'])<=0),None)
    reverse=None if reverse_index is None else rows[reverse_index]
    reverse_state=None if reverse is None else dict(world_time_s=world_times[reverse_index],weapon_age_s=float(reverse['sim_time']),
        alpha_deg=math.degrees(float(reverse['alpha_rad'])),beta_deg=math.degrees(float(reverse['beta_rad'])),
        mach=float(reverse['mach']),load_factor=float(reverse['load_factor']),
        body_rates_rad_s=[float(v) for v in reverse['angular_rate_body_rad_s']])
    # Public vehicle events are not native fragment Hit events or proof of destruction.
    hits=[]
    for i in range(reader.frame_count):
        hits.extend(reader.events(i))
    return dict(flight_samples=len(rows),max_mach=peak('mach'),max_alpha_deg=math.degrees(peak('alpha_rad')),
        max_beta_deg=math.degrees(peak('beta_rad')),max_load_factor=peak('load_factor'),
        max_speed_ms=peak('airspeed_true_ms'),diverged=any(bool(r['diverged']) for r in rows),
        crashed=any(bool(r['crashed']) for r in rows),closest_recorded_range_m=None if not math.isfinite(closest) else closest,
        launch_s=weapon['launch_time_s'],end_s=weapon['end_time_s'],terminal_kind=weapon['terminal_kind'],
        fuze_miss_m=weapon['miss_m'],acquired=weapon['acquired'],activated=weapon['activated'],supported=weapon['supported'],
        target_functional_events=functional,recorded_event_count=len(hits),seeker=weapon['seeker'],
        native_event_counts=counts,native_destroyed=any(e['destroyed'] for e in events),first_reverse_flow=reverse_state,
        engine_count=max(int(r['engine_count']) for r in rows),
        peak_engine_thrust_n=[max(float(r['thrust_n'][i]) for r in rows) for i in range(max(int(r['engine_count']) for r in rows))])


def environment(build):
    env=os.environ.copy();env['PATH']=str(build/'bin')+os.pathsep+env.get('PATH','')
    for key in ('TMP','TEMP','TORCH_EXTENSIONS_DIR'):env[key]='D:/ENV/tmp'
    return env


def native(out,build,source_revision=None):
    from hangar import showcase,storestages
    cards=[read(p) for p in sorted((ROOT/'assets/stores').glob('*.json')) if read(p).get('class')=='aam']
    if len(cards)!=24:raise ValueError('The declared campaign roster must contain all24 current AAMs')
    manifest=dict(schema='fsim.aam-kill-chain-campaign.v1',head=source_revision or subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        scope='Native diagnostic behavior and card implementation; not exact-model calibration',models=[],
        executables={name:dict(path=str(build/'bin'/name),sha256=sha(build/'bin'/name)) for name in ['flightsim-showcase.exe','flightsim-viewer.exe','libfsim.dll']})
    scenarios=[('head-on',5000.,270.,0.),('tail-turn',3000.,90.,300.)]
    for card in cards:
        model=card['model'].removeprefix('jsbsim:')
        try:carrier='f16c';station=showcase._station(carrier,card['id'])
        except SystemExit:
            carrier,station,_=storestages.carrier_for(card)
        for scenario,distance,heading,offset in scenarios:
            folder=out/model/scenario;folder.mkdir(parents=True,exist_ok=True)
            scene=showcase.script(model,carrier,station,12.,40.,6000.,target='f16c',target_range=distance,
                target_altitude_offset=offset,seed=17)
            scene['target']['heading_deg']=heading;scene['name']='AAM '+model+' '+scenario
            scene_path=folder/'scene.json';write(scene_path,scene)
            recording=folder/'native.fsrec';evidence=Path(str(recording)+'.engagement.json')
            row=dict(model=model,card=card['id'],scenario=scenario,carrier=carrier,station=station,run_status='pending',
                scene=str(scene_path),recording=str(recording),evidence=str(evidence),scene_sha256=sha(scene_path),
                card_sha256=sha(ROOT/'assets/stores'/(card['id']+'.json')),
                xml_sha256=sha(ROOT/'aircraft/stores'/model/(model+'.xml')),
                card_description={key:card.get(key) for key in ['designation','summary','seeker','guidance','motor','guidance_capabilities']},
                providers=card.get('component_models',{}).get('components',{}),equipment_accuracy='uncalibrated')
            command=[str(build/'bin/flightsim-showcase.exe'),str(scene_path),str(recording)]
            row['command']=command
            try:
                if shutil.disk_usage(out).free<3*1024**3:raise RuntimeError('Disk reserve below3GiB; stop before producing temporary data')
                with (folder/'native.log').open('w',encoding='utf-8') as log:
                    result=subprocess.run(command,cwd=ROOT,env=environment(build),stdout=log,stderr=subprocess.STDOUT,timeout=120)
                row['exit_code']=result.returncode
                if result.returncode:raise RuntimeError((folder/'native.log').read_text(encoding='utf-8',errors='replace')[-1200:])
                row['native']=summarize(recording,read(evidence));row['run_status']='ok'
            except Exception as error:row['run_status']='not_released_or_execution_error';row['error']=str(error)
            manifest['models'].append(row);write(out/'manifest.json',manifest)
            print(model,scenario,row['run_status'],row.get('native',{}).get('terminal_kind',''),flush=True)
    return manifest


def viewer_command(build,world,prefix):
    """Mandatory presentation; no reduced scene or speed selection exists."""
    return [str(build/'bin/flightsim-viewer.exe'),'--world',world,'--fullscreen',
        '--width','3840','--height','2160','--offline','--camera','chase',
        '--chase-distance','20','--chase-azimuth','240','--chase-elevation','20',
        '--imagery','satellite','--elevation','terrarium','--clouds','scattered','--sun-utc','20',
        '--screenshot',str(prefix),'--screenshot-after','15','--screenshot-frames','4',
        '--screenshot-every','10','--stats','48','--trace','2']


def visual(out,build,manifest):
    """All24 models, one full-scene fullscreen live native40-second World each."""
    from PIL import Image
    names=sorted({r['model'] for r in manifest['models']})
    if len(names)!=24:raise ValueError('All24 AAMs are required; no partial visual campaign')
    superseded=[]
    for row in manifest['models']:
        if 'visual' in row:
            previous=row.pop('visual')
            if previous.get('mode')!='fullscreen-live-real-time':
                superseded.append(dict(model=row['model'],scenario=row['scenario'],visual=previous))
        for path in Path(row['recording']).parent.glob('visual_*.png'):
            if path.resolve().is_relative_to(out):path.unlink()
    if superseded:write(out/'superseded-windowed-visuals.json',dict(status='invalidated',reason='Windowed accelerated reduced-scene run is rejected by user; no acceptance credit.',models=superseded))
    for model in names:
        candidates=[r for r in manifest['models'] if r['model']==model and r['run_status']=='ok']
        if not candidates:raise RuntimeError('No releasable native case for '+model)
        row=next((r for r in candidates if r['scenario']=='tail-turn'),candidates[0])
        folder=Path(row['recording']).parent/'fullscreen-live';folder.mkdir(exist_ok=True)
        scene=read(row['scene']);live=folder/'live.fsrec';prefix=folder/'fullscreen.png'
        command=viewer_command(build,scene['name'],prefix)
        native_command=[str(build/'bin/flightsim-showcase.exe'),row['scene'],str(live),'--live']
        result=dict(command=command,native_command=native_command,status='running',mode='fullscreen-live-real-time',
            resolution=[3840,2160],time_factor=1.,effects='mandatory-complete-scene',recording=str(live),
            viewer_sha256=sha(build/'bin/flightsim-viewer.exe'),native_sha256=sha(build/'bin/flightsim-showcase.exe'))
        row['visual']=result;write(out/'manifest.json',manifest)
        source=window=None
        try:
            for old in folder.glob('fullscreen_*.png'):old.unlink()
            vlog=folder/'viewer.log'
            with vlog.open('w',encoding='utf-8') as vl,(folder/'native.log').open('w',encoding='utf-8') as nl:
                window=subprocess.Popen(command,cwd=ROOT,env=environment(build),stdout=vl,stderr=subprocess.STDOUT)
                deadline=time.monotonic()+60
                while 'stats t=' not in vlog.read_text(encoding='utf-8',errors='replace'):
                    if window.poll() is not None:raise RuntimeError('Full-scene Viewer exited before first rendered frame')
                    if time.monotonic()>deadline:raise RuntimeError('Full-scene Viewer first-frame timeout')
                    time.sleep(.1)
                start=time.monotonic()
                source=subprocess.Popen(native_command,cwd=ROOT,env=environment(build),stdout=nl,stderr=subprocess.STDOUT)
                code=source.wait(timeout=90);elapsed=time.monotonic()-start
                viewer_code=window.wait(timeout=30)
            if code or viewer_code:raise RuntimeError('Native/Viewer exit codes '+str(code)+'/'+str(viewer_code))
            if elapsed<float(scene['duration_s'])-.1:raise RuntimeError('Native World did not run in real time')
            captures=sorted(folder.glob('fullscreen_*.png'))
            if len(captures)!=4:raise RuntimeError('Missing full-duration visual captures')
            for path in captures:
                with Image.open(path) as image:
                    image.load()
                    if image.size!=(3840,2160):raise RuntimeError('Capture is not native fullscreen resolution')
            actual=read(str(live)+'.engagement.json')
            result.update(status='captured',captures=[str(p) for p in captures],native_exit=code,viewer_exit=viewer_code,
                native_wall_seconds=elapsed,simulation_seconds=actual['duration_s'],native_frames=actual['recorded_frames'],
                matches_preflight=actual==read(row['evidence']),native=summarize(live,actual))
        except Exception as error:result.update(status='error',error=str(error))
        finally:
            for process in (source,window):
                if process is not None and process.poll() is None:
                    process.terminate()
                    try:process.wait(timeout=5)
                    except subprocess.TimeoutExpired:process.kill();process.wait()
        write(out/'manifest.json',manifest);print('fullscreen-live',model,result['status'],result.get('error',''),flush=True)


def assess(rows):
    """Report observed behavior and declared mechanisms, without inventing a fidelity gate."""
    first=rows[0];model=first['model'];p=first['providers'];d=first['card_description']
    card_path=ROOT/'assets/stores'/(first['card']+'.json')
    if sha(card_path)!=first['card_sha256']:raise ValueError('Card changed since the frozen campaign: '+model)
    card=read(card_path)
    if p!=card['component_models']['components']:
        raise ValueError('Reported providers differ from the frozen card: '+model)
    p={kind:dict(part) for kind,part in p.items()}
    for kind,part in p.items():
        if part['availability']=='available':
            part['implementation_id']=equipment_models.normalize(card,kind,part['implementation_id'])
    parameters=card.get('parameters',{})
    g_key=p['guidance'].get('parameter_refs',{}).get('max_g','')
    card_kinematics={key:parameters[key] for key in ['speed_mach','range_km',g_key] if key in parameters}
    native=[r['native'] for r in rows]
    legacy=p['seeker']['implementation_id']=='fsim.seeker.encounter.v1'
    guidance=('Legacy encounter seeker and Cartesian PN: the declared seeker family is selected, '
        'but detector/servo and hardware discrimination are not established.' if legacy else
        'Independent '+d['seeker']+' lifecycle and measurement-history PN; finite V3 RF/radiometric '
        'sensing, sampled pointing/exposure/processing and modeled detector covariance. Hardware calibration is separate.'
        if p['seeker']['implementation_id'].endswith('.v3') else
        'Retained ideal V2 measurement-history PN with unavailable servo dynamics; no hardware calibration.')
    gaps={
      'aim120c':'Finite native uplink refreshes only delivered platform fusion. These close encounters do not measure radio or equipment range accuracy.',
      'aim120d':'Resolved inertial/GNSS aiding and exact-fit two-way transport execute; separate public fixtures verify delivered assessment and loss lifecycle. These scenes do not calibrate hardware.',
      'aim7m':'Illumination-dependent SARH is selected; this pair does not validate loss-of-illumination rejection or CW/PD receiver fidelity.',
      'aim9m':'All-aspect detection is not an all-aspect kill guarantee; inspect the separate native outcome of each case.',
      'aim9x':'The card explicitly discloses absent jet-vane TVC in the fin-only plant; these forward launch cases do not validate high off-boresight capability.',
      'asraam':'A 128x128 IIR detector and lock-on-after-launch performance are not established by geometric extent association or predesignated forward shots.',
      'irist':'The card describes TVC; the current physical plant is fin-only. Stability is evaluated from the current native histories separately.',
      'meteor':'The card describes a throttleable ducted rocket; the bound plant is a generated solid rocket. This is a propulsion mechanism mismatch.',
      'micaem':'An active-radar lifecycle does not establish calibrated RF receiver performance or thrust-vector control.',
      'micair':'Geometric image-extent association does not establish a dual-waveband imaging detector or thrust-vector control.',
      'pl10':'LOAL and physical imaging-detector performance are not demonstrated by predesignated forward shots.',
      'pl15':'A finite commanded reserve pulse executes through native fuel/impulse accounting; separate30km cases exercise actual delayed ignition. Equipment long-range performance remains uncalibrated.',
      'r37m':'A finite commanded reserve pulse executes through native fuel/impulse accounting; separate30km cases exercise actual delayed ignition. Equipment very-long-range performance remains uncalibrated.',
      'r77m':'An active-radar measurement lifecycle does not implement or validate an AESA receiver.',
    }
    if p['propulsion'].get('implementation_id') in ('fsim.propulsion.jsbsim_ducted_rocket.v1','fsim.propulsion.jsbsim_ducted_rocket.v2'):
        gaps['meteor']='Separate native solid booster and atmosphere-fed throttled sustainer execute an engineering thermodynamic cycle. Inlet/combustor/nozzle maps and equipment range/energy performance remain uncalibrated.'
    if p['autopilot'].get('bindings',{}).get('thrust_vector')=='jsbsim_nozzle_v1':
        gaps[model]={'aim9x':'High off-boresight acquisition and jet-vane hardware curves remain uncalibrated.',
            'irist':'Imaging detector, vector hardware curves and full maneuver envelope remain uncalibrated.',
            'micaem':'RF receiver performance and vector hardware curves remain uncalibrated.',
            'micair':'Dual-band radiometric imagery and vector hardware curves remain uncalibrated.',
            'pl10':'The authored equivalent thrust-vector mechanism executes; PL-10 hardware curves and high off-boresight performance remain uncalibrated.'}.get(model,
                'Equivalent thrust vectoring executes with disclosed assumptions; exact hardware response remains uncalibrated.')
        guidance+=' Finite native equivalent thrust vectoring is selected; physical nozzle forces/CG moments are modeled with explicitly assumed angle/rate inputs.'
    control=('Physical authored fin IDs, actuator lag/rates/stops and actual recorded pose are used. ')
    if p['autopilot'].get('implementation_id')=='fsim.control.actuator_aware_cascade.v1':
        control+='The selected controller includes actual actuator state and local incidence/Mach derivatives; the original instantaneous cascade remains separately available. '
    elif p['autopilot'].get('implementation_id')=='fsim.control.operating_point_bounded.v1':
        control+='The selected operating-point controller consumes actual actuator state and bounded local stability/control derivatives. '
    maximum=max(n['max_alpha_deg'] for n in native)
    if maximum>90:control+='Recorded reverse-flow incidence occurs; the controller explicitly refuses reverse-flow inversion. This is loss of control, not successful fin control.'
    elif maximum>30:control+='A large transient incidence excursion occurs; successful detonation does not validate stability.'
    else:control+='No large incidence departure was observed in these two short encounters; maximum maneuver envelope remains untested.'
    audits=[r.get('fin_render_audit') for r in rows]
    if all(audits):
        control+=' Recorded peak actual fin '+format(math.degrees(max(a['peak_actual_rad'] for a in audits)),'.2f')+' deg; saturated actual-angle samples '+str(sum(a['saturated_angles'] for a in audits))+'/'+str(sum(a['fin_angles'] for a in audits))+'; controller unavailable '+str(sum(a['controller_unavailable_samples'] for a in audits))+'/'+str(sum(a['samples'] for a in audits))+'. One initial release sample per case is not a sustained controller outage.'
    outcome_note=(' Head-on: '+native[0]['terminal_kind']+'; tail: '+native[1]['terminal_kind']+'.'
        if len(native)==2 else '')
    return dict(model=model,designation=d['designation'],card_description=d,card_kinematics=card_kinematics,
      seeker_provider=first['providers']['seeker']['implementation_id'],guidance_provider=first['providers']['guidance']['implementation_id'],
      seeker_mechanism=p['seeker']['implementation_id'],guidance_mechanism=p['guidance']['implementation_id'],
      aerodynamics='Shared VLM/JSBSim engineering estimate with accepted M02 method checks. Equipment-specific measured accuracy remains uncalibrated; no accuracy percentage is inferred from encounters.',
      control=control,guidance=guidance,card_gap=gaps.get(model,
        'The declared '+d['seeker']+' family and solid propulsion are selected. The stated equipment range, maneuverability, countermeasure resistance and dynamic response remain unvalidated.')+outcome_note,
      head_on=next(r['native'] for r in rows if r['scenario']=='head-on'),
      tail_turn=next(r['native'] for r in rows if r['scenario']=='tail-turn'),
      fin_render=[r.get('fin_render_audit') for r in rows],
      visual=next((r['visual'] for r in rows if 'visual' in r),None))


def report(out,manifest):
    audit=read(out/'fin-render-audit.json') if (out/'fin-render-audit.json').is_file() else {'models':[]}
    for row in manifest['models']:
        row['native']=summarize(Path(row['recording']),read(row['evidence']))
        if row.get('visual',{}).get('status')=='captured':
            v=row['visual'];v['native']=summarize(Path(v['recording']),read(v['recording']+'.engagement.json'))
        row['fin_render_audit']=next((a for a in audit['models'] if a['model']==row['model'] and a['scenario']==row['scenario']),None)
    findings=[assess([r for r in manifest['models'] if r['model']==model]) for model in sorted({r['model'] for r in manifest['models']})]
    write(out/'results.json',dict(**manifest,findings=findings))
    def esc(v):return html.escape(str(v))
    body=[]
    for f in findings:
        def encounter(n):return n['terminal_kind']+'; CPA '+format(n['fuze_miss_m'],'.2f')+' m; Mach '+format(n['max_mach'],'.2f')+'; max |alpha| '+format(n['max_alpha_deg'],'.1f')+' deg; max |load| '+format(n['max_load_factor'],'.1f')+'; track losses '+str(n.get('native_event_counts',{}).get('track_loss',0))+'; native damage events '+str(n.get('native_event_counts',{}).get('damage',0))+'; native destroyed '+str(n.get('native_destroyed',False))
        v=f['visual'] or {};captures=v.get('captures',[])
        pictures=''.join('<a href="'+esc(Path(path).relative_to(out).as_posix())+'"><img width="320" src="'+esc(Path(path).relative_to(out).as_posix())+'"></a>' for path in captures)
        fin=f['fin_render'];synchronization=('Actual fin angles and independent hinge matrices agree; actuator rate/stop checks agree.' if all(a and a['angle_mismatches']==0 and a['matrix_error_max']==0 and a['rate_violations']==0 and a['limit_violations']==0 for a in fin) else 'See actual fin audit; no blanket synchronization claim.')
        nominal=f['card_kinematics'].get('speed_mach',{})
        card_speed=('Card nominal Mach '+str(nominal.get('value','unspecified'))+'; conditions not matched; not an accuracy reference.')
        cells=[f['designation']+'; '+card_speed,encounter(f['head_on']),encounter(f['tail_turn']),f['aerodynamics'],f['control'],f['guidance']+' '+f['card_gap'],synchronization,v.get('status','not captured')]
        body.append('<tr>'+''.join('<td>'+esc(x)+'</td>' for x in cells)+'<td>'+pictures+'</td></tr>')
    (out/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>All24 AAM live diagnostics</title><style>body{font:15px system-ui}table{border-collapse:collapse}td,th{border:1px solid #aaa;padding:8px;vertical-align:top}td{min-width:160px}img{display:block}</style>'
        '<h1>All24 AAM native kill-chain and fullscreen live diagnostics</h1><p>Two frozen 40-second native SDK encounters per model: head-on 5 km and tail 3 km with 300 m altitude offset; launch requested at 12 s, 6000 m, 250 m/s, seed17. One complete fullscreen live 1x run per model. This is behavior diagnosis, not real-equipment calibration. Detonation, fragment Hit, functional loss and destruction are distinct. Short forward shots do not validate card maximum range, high off-boresight, maximum g, datalink resilience or countermeasure resistance.</p>'
        '<p>Every visual uses Earth imagery, terrain, scattered volumetric clouds, sunlight/shadows, native exhaust, vapour/contrails and the native Flow detonation/fire/smoke pipeline. A pipeline being enabled does not create a physical event that did not occur.</p>'+
        ('<p>Supplemental full-scene pixel checks: '+ '; '.join('<a href="'+name+'">'+label+'</a>' for name,label in [('aircraft-shadow-verified.png','aircraft contact shadow'),('event-strike-aim120c/strike_hit_plus_0.3s.png','native event-relative fragment hit and target state')] if (out/name).is_file())+'. These supplement the live runs below.</p>' if any((out/name).is_file() for name in ['aircraft-shadow-verified.png','event-strike-aim120c/strike_hit_plus_0.3s.png']) else '')+
        '<table><tr>'+''.join('<th>'+x+'</th>' for x in ['Model','Head-on','Tail with altitude offset','Aerodynamics','Control','Card / guidance comparison','Actual fin to geometry','Live presentation','Native fullscreen captures'])+'</tr>'+''.join(body)+'</table>',encoding='utf-8')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,required=True);parser.add_argument('--build',type=Path,default=ROOT/'build/ucrt64-release')
    parser.add_argument('--stage',choices=['native','visual','report'],required=True)
    args=parser.parse_args();out=args.out.resolve();out.mkdir(parents=True,exist_ok=True);build=args.build.resolve()
    if args.stage=='native':manifest=native(out,build)
    else:manifest=read(out/'manifest.json')
    if args.stage=='visual':visual(out,build,manifest)
    if args.stage=='report':report(out,manifest)


if __name__=='__main__':main()
