"""Every authored AAM through installed public Agent release and native flight.

All scenario inputs are fixed here before execution. Referee-only flight metrics
are separate from actor observations and return telemetry. Misses remain native
outcomes. The client performs no sensor, guidance, flight, fuze or damage model.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time

from acceptance import allowed, verify_artifact, WorldHealthV1

ROOT=Path(__file__).resolve().parents[3]


def read(path):return json.loads(allowed(path).read_text(encoding='utf-8'))
def sha(path):return hashlib.sha256(allowed(path).read_bytes()).hexdigest()
def require(value,reason):
    if not value:raise ValueError(reason)


def carrier_for(card,coverage):
    """Select a radar-equipped carrier for the fixed permitted RF-cue scene.

    A compatibility fallback remains labeled; it is not evidence of a real fit.
    The SDK still validates every load and release authoritatively.
    """
    candidates=[];mass=sum(card.get('parameters',{}).get(k,{}).get('value',0) for k in ('mass_kg','fuel_kg'))
    for fit in coverage['fits']:
        if fit['category']!='fixed-wing':continue
        source=read(ROOT/fit['fit_path']) if fit['fit_sha256'] else {}
        payload_cards=[read(path) for payload in source.get('payloads',[])
            if (path:=ROOT/'assets/payloads'/(payload['card']+'.json')).is_file()]
        if not any(payload.get('family')=='air_radar' for payload in payload_cards):continue
        authored={s['number'] for loadout in fit['declared_loadouts'] for s in loadout['stations'] if s['card']==card['id']}
        for payload in source.get('payloads',[]):
            path=ROOT/'assets/payloads'/(payload['card']+'.json')
            if not path.is_file():continue
            station_card=read(path)
            if station_card.get('family')!='store_station':continue
            for station in station_card.get('stations',[]):
                if card['class'] not in station.get('takes',[]) or mass>station.get('max_kg',0):continue
                if station.get('capacity',1)<1:continue
                rank=0 if station['number'] in authored else 1
                preferred={'aim-120d':'f16c','meteor':'gripen'}.get(card['id'])
                candidates.append((0 if fit['design']==preferred else 1,rank,fit['design'],station['number']))
    require(candidates,'No maintained legal carrier/station for '+card['id'])
    _,rank,carrier,station=min(candidates)
    return dict(carrier=carrier,station=station,declared_loadout=rank==0,
        scene_prerequisite='Own fitted air radar supplies the predeclared RF designation; radarless carriers remain in the authored applicability inventory',
        fit_scope='authored selected loadout' if rank==0 else 'native class/mass compatibility; real installation not inferred')


def scene(fsim,row,card,selection,workers,*,distance=5000.,duration=40.,frame_skip=4,countermeasure=True,remove_launcher_after_s=None):
    from explosives import clean
    ticks=[];started=time.perf_counter()
    with fsim.World('m03-public-'+row['card'],publish=False,terrain=False,workers=workers,pin_workers=False,
            seed=19,dt=1/120,frame_skip=frame_skip,jsbsim_root=str(scene.prefix/'share/flightsim/jsbsim')) as w:
        own=w.create_vehicle('launcher','jsbsim:'+selection['carrier'],latitude_deg=40,longitude_deg=0,
            altitude_msl_m=6000,heading_deg=90,airspeed_ms=250)
        launcher=w.create_agent(own,'launcher')
        launcher.submit(fsim.Level.VELOCITY,airspeed_ms=250,vertical_speed_ms=0,heading_rad=math.pi/2)
        c2=w.create_c2_node('approver',39.9,0,altitude_m=200);net=w.create_net('consent')
        require(w.join_net(own,net) and w.join_net(c2,net),'Consent endpoints could not join')
        approver=w.create_agent(c2,'approver');require(approver.declare_role(fsim.ROLE_QB,'APPROVER'),'C2 role refused')
        station=next(p.index for p in launcher.payloads if p.family=='store_station')
        radar=next(p.index for p in launcher.payloads if p.family=='air_radar')
        own.load_store(selection['station'],row['card'],payload=station)
        for payload in (station,radar):launcher.set_payload_state(payload,'operate')
        for _ in range(math.ceil(45/w.step_seconds)):
            w.step()
            if sum(p.state==fsim.PayloadState.OPERATE for p in launcher.payloads if p.index in (station,radar))==2:break
        else:raise ValueError('Native radar/station did not finish warmup')
        state=own.state
        target=w.create_vehicle('opponent','jsbsim:f16c',latitude_deg=math.degrees(state.latitude_rad),
            longitude_deg=math.degrees(state.longitude_rad)+math.degrees(distance/(6371000*math.cos(state.latitude_rad))),
            altitude_msl_m=6000,heading_deg=270,airspeed_ms=250)
        opponent=w.create_agent(target,'opponent')
        opponent.submit(fsim.Level.VELOCITY,airspeed_ms=250,vertical_speed_ms=0,heading_rad=3*math.pi/2)
        dispenser=next((p.index for p in opponent.payloads if p.family=='countermeasures'),None)
        if dispenser is not None:opponent.set_payload_state(dispenser,'operate')
        launcher.amti_search(radar,bars=1,azimuth_half_width_rad=math.radians(15),range_scale_m=60000)
        track=None
        for _ in range(math.ceil(20/w.step_seconds)):
            w.step();track=next((t for t in launcher.amti_tracks(radar) if t.updates>=3),None)
            if track is not None:break
        require(track is not None,'Own permitted radar did not confirm a track')
        links=fsim.WeaponLinksV1(launcher)
        fused=next((e for e in launcher.fused_tracks_v1() if any(s.payload==radar and s.track==track.id and
                    s.kind==fsim.FusionSourceKindV1.LOCAL_RADAR for s in e.sources)),None)
        require(fused is not None,'Confirmed track has no admitted platform fusion')
        designation=links.designate_fused(fused.id)
        require(designation,'Own fused designation refused')
        # SARH needs its actual illuminator association, not the fused ID being
        # misread as a local radar-track number. Existing native radar STT owns it.
        launcher.amti_track(track.id,radar,illuminate=card['seeker']=='semi_active_radar',range_scale_m=60000)
        if card['seeker']=='semi_active_radar':
            designation=launcher.designate(radar,track.id)
            require(designation,'Own SARH illumination designation refused')
        activity=launcher.strike(row['card'],target=designation,payload=station)
        require(not w.weapons,'Actor strike bypassed native consent')
        answered=False
        for _ in range(math.ceil(45/w.step_seconds)):
            w.step()
            for report in approver.report_consents().items:
                request=report.value
                if request.strike!=activity:continue
                require(report.meta.source=='dms' and not w.weapons,'Consent was not actually delivered')
                require(approver.answer_consent(request.source,request.request_id),'C2 answer refused')
                answered=True;break
            if answered:break
        require(answered,'C2 received no native request')
        weapon=None
        for _ in range(math.ceil(20/w.step_seconds)):
            w.step();status=launcher.strike_status(activity)
            if status and status.weapons:
                weapon=int(status.weapons[0]);break
        require(weapon is not None,'Delivered consent did not release the requested store')
        records=[r for r in w.weapons if r.id==weapon]
        require(len(records)==1 and not records[0].jettisoned,'Release correlation is not an armed native weapon')
        launch=records[0].launch_time_s
        capability=links.capability(weapon);receipt=None
        if capability.direction==fsim.WeaponLinkDirectionV1.TWO_WAY:
            receipt=links.assess(weapon,101,duration_s=duration+5);require(receipt.accepted,'Fitted assessment refused')
        else:require(not links.assess(weapon,101).accepted,'Unsupported return fit acquired a transmitter')
        samples=[];messages=[];dispensed=False;controls=0;acquired=False;second_ignition=None;second_thrust=False
        feedback_closed=None
        launcher_removed=False;diagnostics=[];first_reverse=None;health=WorldHealthV1()
        max_alpha=max_beta=max_load=max_mach=0.;reverse=False
        def observe_world():
            # Observe the entire native aftermath as well as the live weapon.
            # A terminal missile cannot hide a diverging target or detached body.
            health.observe(w)
        observe_world()
        while w.time<launch+duration:
            age=w.time-launch
            if remove_launcher_after_s is not None and not launcher_removed and age>=remove_launcher_after_s:
                own.remove();launcher_removed=True
            if countermeasure and not dispensed and age>=1.5 and dispenser is not None:
                before=opponent.countermeasures();kind='flare' if 'infrared' in card['seeker'] else 'chaff'
                opponent.dispense(kind);after=opponent.countermeasures()
                require(sum(before)-sum(after)==1,'Native cartridge release did not consume exactly one store')
                dispensed=True
            record=next(r for r in w.weapons if r.id==weapon);acquired=acquired or record.tracking
            require(record.end in ('in_flight','impact','removed','detonated','expired'),'Unknown native weapon terminal state')
            if record.end=='in_flight':
                missile=w.vehicle(weapon);s=missile.state
                values=[float(x) for x in s.position_ecef]+[float(x) for x in s.velocity_body_ms]+[
                    float(s.alpha_rad),float(s.beta_rad),float(s.load_factor),float(s.mach)]
                require(all(math.isfinite(x) for x in values) and not s.diverged,'Nonfinite/diverged native weapon')
                reverse=reverse or math.cos(s.alpha_rad)*math.cos(s.beta_rad)<=0
                max_alpha=max(max_alpha,abs(s.alpha_rad));max_beta=max(max_beta,abs(s.beta_rad))
                max_load=max(max_load,abs(s.load_factor));max_mach=max(max_mach,s.mach)
                control=w.weapon_controls(weapon)
                diagnostic=dict(time_s=w.time,age_s=age,alpha_rad=s.alpha_rad,beta_rad=s.beta_rad,
                    airspeed_ms=s.airspeed_true_ms,mach=s.mach,load_factor=s.load_factor,
                    angular_rate_body_rad_s=list(s.angular_rate_body_rad_s),fuel_kg=s.fuel_kg,
                    thrust_n=list(s.thrust_n)[:s.engine_count],phase=int(record.phase),tracking=record.tracking,
                    control=clean(control))
                diagnostics.append(diagnostic)
                if first_reverse is None and math.cos(s.alpha_rad)*math.cos(s.beta_rad)<=0:first_reverse=diagnostic
                if control and control.controller_available:controls+=1
                if row['card'] in ('pl-15','r-37m'):
                    ignition=missile.get_property('fsim/motor-pulse/second-ignition-age-s')
                    if ignition>=0:
                        second_ignition=float(ignition)
                        second_thrust=second_thrust or sum(s.thrust_n[:s.engine_count])>1
                samples.append([w.time,*values,int(record.phase),record.tracking,record.supported,
                    record.holding,record.burning,control.controller_available if control else False])
            if receipt and feedback_closed is None:
                reports=links.feedback(weapon,receipt.assessment,since=messages[-1][0] if messages else 0)
                for report in reports:
                    require(report.sample_time_s<=report.sent_time_s<report.received_time_s,'Feedback has invalid physical timing')
                    require(report.navigation_valid and report.navigation_covariance[0]>=0,'Onboard covariance unavailable')
                    messages.append([str(report.sequence),report.sample_time_s,report.sent_time_s,report.received_time_s,
                        report.tracking,report.active,report.supported,report.end_of_flight,list(report.state_ecef),
                        report.navigation_time_s,list(report.navigation_covariance),report.navigation_gnss_aided])
                status=links.status(weapon,receipt.assessment)
                if status.state in (fsim.WeaponAssessmentStateV1.SOFT_LOST,fsim.WeaponAssessmentStateV1.LOST,
                        fsim.WeaponAssessmentStateV1.END_OF_FLIGHT,fsim.WeaponAssessmentStateV1.CANCELED,
                        fsim.WeaponAssessmentStateV1.EXPIRED):
                    feedback_closed=dict(state=int(status.state),reason=int(status.reason),observed_s=w.time)
            tick=time.perf_counter();w.step();ticks.append(time.perf_counter()-tick)
            observe_world()
        final=next(r for r in w.weapons if r.id==weapon)
        raw=dict(samples=samples,messages=messages,diagnostics=diagnostics,first_reverse=first_reverse,
            world_health=health.rows,invalid_entities=sorted(health.invalid))
        fingerprint=hashlib.sha256(json.dumps(raw,allow_nan=False,separators=(',',':')).encode()).hexdigest()
        artifacts=scene.artifacts/(row['card']+'-'+str(workers)+'w-'+str(frame_skip)+'skip-'+str(int(distance))+'m'+
            ('-launcher-loss' if remove_launcher_after_s is not None else '')+'.json')
        artifacts.parent.mkdir(parents=True,exist_ok=True)
        artifacts.write_text(json.dumps(raw,allow_nan=False,separators=(',',':'))+'\n',encoding='utf-8')
        require(samples and acquired and controls,'Actual model did not exercise acquisition and finite physical control')
        require(not reverse,'Observed reverse-flow departure in the fixed admitted public scene; '+str(artifacts))
        require(not health.invalid,'Native target or detached body diverged during the complete encounter; '+str(artifacts))
        if countermeasure:require(dispensed,'Current target dispenser was not exercised')
        if receipt:require(messages,'Admitted return channel never delivered a report')
        if remove_launcher_after_s is not None:require(launcher_removed,'Launcher-loss control did not execute')
        if distance>=30000 and row['card'] in ('pl-15','r-37m'):
            require(second_ignition is not None and second_thrust,'Actual commanded variant did not ignite its reserved pulse; '+str(artifacts))
        return dict(passed=True,card=row['card'],workers=workers,frame_skip=frame_skip,distance_m=distance,duration_s=duration,
            launch_time_s=launch,terminal=final.end,terminal_time_s=final.end_time_s if math.isfinite(final.end_time_s) else None,
            closest_m=final.miss_m if math.isfinite(final.miss_m) else None,acquired=acquired,finite_control_samples=controls,
            observations=len(samples),countermeasure_dispensed=dispensed,delivered_feedback=len(messages),
            feedback_closed=feedback_closed,
            launcher_removed=launcher_removed,remove_launcher_after_s=remove_launcher_after_s,
            return_direction=int(capability.direction),return_direction_source=capability.direction_source,
            second_ignition_age_s=second_ignition,second_pulse_actual_thrust=second_thrust,reverse_flow=reverse,
            complete_world_finite=True,observed_world_states=len(health.rows),
            peak_alpha_deg=math.degrees(max_alpha),peak_beta_deg=math.degrees(max_beta),peak_load_g=max_load,peak_mach=max_mach,
            report_fingerprint=fingerprint,raw_observations=str(artifacts),raw_sha256=sha(artifacts),wall_seconds=time.perf_counter()-started,
            native_step_mean_s=sum(ticks)/len(ticks),native_step_max_s=max(ticks),
            scope='Non-isolated public SDK diagnostic; flight truth metrics remain outside actor feedback')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('prefix','coverage','output'):p.add_argument('--'+key,type=Path,required=True)
    a=p.parse_args();prefix=allowed(a.prefix).resolve();output=allowed(a.output).resolve()
    if output.exists():raise ValueError('Fresh fleet receipt required')
    coverage=read(a.coverage);require(coverage['inventory_passed'],'Incomplete applicability')
    for key in tuple(os.environ):
        if key.startswith('FSIM_') and key!='FSIM_MAPS_DIR':os.environ.pop(key)
    os.environ['FSIM_DLL_DIR']=str(prefix/'bin')
    os.environ['FSIM_AIRCRAFT_PATH']=os.pathsep.join(str(prefix/'share/flightsim/aircraft'/part) for part in ('','stores'))
    sys.path.insert(0,str(prefix))
    import fsim
    require(Path(fsim.__file__).resolve().is_relative_to(prefix),'Facade did not load from installed product')
    sys.path.insert(0,str(ROOT/'tools/validation/radar'))
    from provenance import loaded_modules
    scene.prefix=prefix
    scene.artifacts=output.parent/'installed-fleet-records'
    result=dict(schema='fsim.sim-m03.installed-fleet.v1',passed=False,inventory_sha256=sha(a.coverage),
        prefix=str(prefix),loaded_modules=loaded_modules(prefix),models=[],long_range=[],launcher_loss=[],
        installed_aircraft_search_roots=os.environ['FSIM_AIRCRAFT_PATH'].split(os.pathsep))
    def save():output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    try:
        for row in coverage['stores']:
            if row['class']!='aam':continue
            verify_artifact(ROOT/row['card_path'],row['card_sha256']);card=read(ROOT/row['card_path']);selection=carrier_for(card,coverage)
            print('installed model '+row['card'],flush=True)
            first=scene(fsim,row,card,selection,1);second=scene(fsim,row,card,selection,4)
            require(first['report_fingerprint']==second['report_fingerprint'],'Worker schedule changed native reports: '+row['card'])
            result['models'].append(dict(card=row['card'],selection=selection,one_worker=first,four_workers=second,replay_passed=True));save()
            if row['card'] in ('pl-15','r-37m'):
                long=scene(fsim,row,card,selection,1,distance=30000,duration=65,countermeasure=False)
                result['long_range'].append(long);save()
                loss=scene(fsim,row,card,selection,1,distance=30000,duration=65,countermeasure=False,
                    remove_launcher_after_s=1.)
                result['launcher_loss'].append(loss);save()
        require({row['card'] for row in result['models']}=={r['card'] for r in coverage['stores'] if r['class']=='aam'},'Incomplete current AAM population')
        result['passed']=True;save();print(json.dumps({k:v for k,v in result.items() if k not in ('models','long_range','cadence')}))
    except Exception as error:
        result['error']=str(error);save();raise


if __name__=='__main__':main()
