"""Installed Agent/C2 cruise and armed/inert bomb diagnostics.

Fixed inputs precede execution. Referee state/events stay in this diagnostic
process; actor target data is genuinely delivered radar-link data. Live mode
publishes the complete native scene and advances at real time (1x).
"""
import argparse
import ctypes
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time

from acceptance import allowed, WorldHealthV1

ROOT=Path(__file__).resolve().parents[3]
SCENARIOS=dict(cruise=dict(store='cj-10a',carrier='h6k',station=1,altitude_m=6000,speed_ms=200,seconds=40),
               bomb=dict(store='mk82',carrier='f16c',station=3,altitude_m=1500,speed_ms=200,seconds=90,aftermath_s=8))
ORIGIN=dict(latitude_deg=36.25,longitude_deg=-115.03)
C2_ENDPOINT=dict(latitude_deg=36.20,longitude_deg=-115.03,antenna_clearance_m=200.)


def require(value,reason):
    if not value:raise ValueError(reason)


def clean(value):
    if isinstance(value,ctypes.Structure):return {name:clean(getattr(value,name)) for name,*_ in value._fields_}
    if isinstance(value,ctypes.Array):return [clean(v) for v in value]
    if hasattr(value,'_asdict'):return clean(value._asdict())
    if isinstance(value,dict):return {k:(str(v) if k in ('instance','activity','serial','request_id','assessment','sequence','track') else clean(v)) for k,v in value.items()}
    if isinstance(value,(tuple,list)):return [clean(v) for v in value]
    if isinstance(value,float) and not math.isfinite(value):return None
    return value


class Clock:
    def __init__(self,world,live):self.world=world;self.live=live;self.wall=time.perf_counter();self.epoch=world.time;self.health=None
    def step(self):
        self.world.step()
        if self.health is not None:self.health.observe(self.world)
        if self.live:
            wait=self.wall+self.world.time-self.epoch-time.perf_counter()
            if wait>0:time.sleep(wait)
    def restart(self):self.wall=time.perf_counter();self.epoch=self.world.time


def until(clock,seconds,predicate,reason):
    for _ in range(math.ceil(seconds/clock.world.step_seconds)):
        clock.step();value=predicate()
        if value:return value
    raise ValueError(reason)


def hold(fsim,agent,speed):agent.submit(fsim.Level.VELOCITY,airspeed_ms=speed,vertical_speed_ms=0,heading_rad=math.pi/2)


def create_approver_endpoint(world):
    # Unlike the flat-ground fleet fixture, this scene enables actual terrain.
    # C2 altitude is an absolute native height, not antenna clearance. Derive
    # it from the same native ground used by DMS propagation at the fixed site.
    lat=C2_ENDPOINT['latitude_deg'];lon=C2_ENDPOINT['longitude_deg']
    ground=world.terrain(math.radians(lat),math.radians(lon))
    require(ground is not None and math.isfinite(ground),'Approver site native terrain unavailable')
    altitude=ground+C2_ENDPOINT['antenna_clearance_m']
    require(math.isfinite(altitude) and altitude>ground,'Approver antenna height is invalid')
    node=world.create_c2_node('approver',lat,lon,altitude_m=altitude)
    require(node,'Approver endpoint creation refused')
    return node,dict(**C2_ENDPOINT,native_ground_m=ground,altitude_m=altitude,
        height_contract='Absolute native terrain height plus fixed200m antenna clearance; native LOS remains enabled')


def retire_coordinate(agent,designation):
    # Python Vehicle.undesignate (also delegated by Agent) is a command with no
    # return value. Confirm the actual owner-scoped designation state instead
    # of interpreting None as refusal or inventing a successful boolean.
    before=next((row for row in agent.designations() if row[0]==designation),None)
    require(before is not None,'Delivered coordinate is absent from owner designation history')
    agent.undesignate(designation)
    after=next((row for row in agent.designations() if row[0]==designation),None)
    require(after is not None and not after[3],'Delivered coordinate remained valid after retirement')
    return dict(designation=str(designation),previously_valid=bool(before[3]),valid_after=bool(after[3]))


def authorize(clock,agent,approver,activity):
    def answer():
        for report in approver.report_consents().items:
            request=report.value
            if request.strike!=activity:continue
            require(report.meta.source=='dms','Consent did not traverse modeled DMS')
            require(not clock.world.weapons,'Weapon released before C2 answer')
            require(approver.answer_consent(request.source,request.request_id),'Native C2 answer refused')
            return dict(source=str(request.source),request=str(request.request_id),received_s=clock.world.time,source_kind=report.meta.source)
    consent=until(clock,45,answer,'No delivered native C2 consent request')
    def released():
        status=agent.strike_status(activity)
        return int(status.weapons[0]) if status and status.weapons else None
    return until(clock,25,released,'Delivered consent did not release store'),consent


def run_scene(fsim,prefix,case,*,live=False,world_name=None,start_file=None,record=None):
    policy=SCENARIOS[case];frames=[];events=[];cursor=0
    options=dict(publish=live,terrain=True,terrain_offline=True,workers=1,pin_workers=False,seed=73,
        dt=1/120,frame_skip=4,jsbsim_root=str(prefix/'share/flightsim/jsbsim'))
    if record is not None:options['record_path']=str(record)
    with fsim.World(world_name or 'm03-explosive-'+case,**options) as w:
        clock=Clock(w,live)
        carrier=w.create_vehicle('launcher','jsbsim:'+policy['carrier'],**ORIGIN,
            altitude_msl_m=policy['altitude_m'],heading_deg=90,airspeed_ms=policy['speed_ms'])
        agent=w.create_agent(carrier,'launcher');hold(fsim,agent,policy['speed_ms'])
        c2,consent_endpoint=create_approver_endpoint(w);approver=w.create_agent(c2,'approver')
        net=w.create_net('consent-and-measurements',latency_s=.01,jitter_s=0,data_rate_bps=1e6)
        require(w.join_net(carrier,net) and w.join_net(c2,net),'Net enrollment refused')
        require(approver.declare_role(fsim.ROLE_QB,'APPROVER'),'Approver role refused')
        station=next(p.index for p in agent.payloads if p.family=='store_station')
        carrier.load_store(policy['station'],policy['store'],payload=station)
        agent.set_payload_state(station,'operate')
        if live:
            w.focus_viewer(carrier);clock.step()
            if start_file:
                deadline=time.perf_counter()+300
                while not start_file.exists():
                    if time.perf_counter()>deadline:raise TimeoutError('Viewer start-file handshake timed out')
                    time.sleep(.05)
                clock.restart()
        until(clock,45,lambda:any(p.index==station and p.state==fsim.PayloadState.OPERATE for p in agent.payloads),'Station warmup incomplete')
        designation=None;aim=None;donor=None;observed=None;inert=None;inert_agent=None
        if case=='cruise':
            # The donor observes eastward. The carrier lies north of its narrow
            # search and cannot be mistaken for the selected measured contact.
            state=carrier.state;lat=math.degrees(state.latitude_rad);lon=math.degrees(state.longitude_rad)
            donor=w.create_vehicle('donor','jsbsim:f16c',latitude_deg=lat-.02,longitude_deg=lon,
                altitude_msl_m=6000,heading_deg=90,airspeed_ms=200)
            sensor=w.create_agent(donor,'donor');hold(fsim,sensor,200)
            observed=w.create_vehicle('coordinate-source','jsbsim:f16c',latitude_deg=lat-.02,longitude_deg=lon+.18,
                altitude_msl_m=6000,heading_deg=270,airspeed_ms=200)
            target_agent=w.create_agent(observed,'coordinate-source')
            target_agent.submit(fsim.Level.VELOCITY,airspeed_ms=200,vertical_speed_ms=0,heading_rad=3*math.pi/2)
            require(w.join_net(donor,net),'Donor net refused')
            radar=next(p.index for p in sensor.payloads if p.family=='air_radar')
            sensor.set_payload_state(radar,'operate')
            until(clock,45,lambda:any(p.index==radar and p.state==fsim.PayloadState.OPERATE for p in sensor.payloads),'Donor radar warmup incomplete')
            sensor.amti_search(radar,bars=1,azimuth_half_width_rad=.2,range_scale_m=60000)
            sent=-math.inf
            def received():
                nonlocal sent
                tracks=[t for t in sensor.amti_tracks(radar) if t.updates>=3]
                if tracks and w.time-sent>=.2:sensor.publish_tracks(radar);sent=w.time
                for report in agent.link_tracks():
                    if report.source!=sensor.system or report.payload!=radar or report.updates<3:continue
                    require(report.time_s<=report.received_s<=w.time,'Invalid delivered coordinate epochs')
                    target=agent.designate_link(report.source,report.payload,report.track)
                    if target:return target,clean(report)
            designation,received_report=until(clock,40,received,'No genuinely delivered donor coordinate')
            # A designation is an estimate, not a vehicle with VehicleInfo.
            aim=list(map(float,w.states([designation])[0]['position_ecef']))
        else:
            # Spatially separate natural-impact negative control; no damage
            # injection or target coordinates are supplied to either bomb.
            state=carrier.state
            inert=w.create_vehicle('inert-carrier','jsbsim:f16c',latitude_deg=math.degrees(state.latitude_rad)+.04,
                longitude_deg=math.degrees(state.longitude_rad),altitude_msl_m=1500,heading_deg=90,airspeed_ms=200)
            inert_agent=w.create_agent(inert,'inert-carrier');hold(fsim,inert_agent,200)
            inert_station=next(p.index for p in inert_agent.payloads if p.family=='store_station')
            inert.load_store(3,'mk82',payload=inert_station);inert_agent.set_payload_state(inert_station,'operate')
            until(clock,45,lambda:any(p.index==inert_station and p.state==fsim.PayloadState.OPERATE for p in inert_agent.payloads),'Negative-control station warmup incomplete')
        # Acceptance of declare_role queues its message. Only the platform's
        # delivered authority proves the modeled C2 declaration arrived.
        def declared_approver():
            operators=agent.authority().operators
            return operators if any(operator[0]==approver.system and operator[1]==fsim.ROLE_QB for operator in operators) else None
        role_operators=until(clock,45,declared_approver,'C2 approver role did not arrive through native DMS')
        consent_endpoint['delivered_role_s']=w.time
        consent_endpoint['delivered_operators']=clean(role_operators)
        activity=agent.strike(policy['store'],target=designation,payload=station)
        require(not w.weapons,'Actor release bypassed consent')
        weapon,consent=authorize(clock,agent,approver,activity)
        if live:w.focus_viewer(w.vehicle(weapon))
        links=fsim.WeaponLinksV1(agent);cap=links.capability(weapon)
        require(not links.assess(weapon,901).accepted,'Unguided/inertial-only store invented return telemetry')
        launch=next(r for r in w.weapons if r.id==weapon).launch_time_s
        health=WorldHealthV1();health.observe(w);clock.health=health
        initial_live=w.vehicle(weapon).state;initial=type(initial_live).from_buffer_copy(initial_live)
        initial_mass=w.vehicle(weapon).mass_kg
        before=math.dist(initial.position_ecef,aim) if aim else None
        coordinate_retirement=retire_coordinate(agent,designation) if case=='cruise' else None
        inert_weapon=None
        if case=='bomb':
            inert_activity=inert_agent.jettison(payload=inert_station)
            def dropped():
                status=inert_agent.strike_status(inert_activity)
                return int(status.weapons[0]) if status and status.weapons else None
            inert_weapon=until(clock,20,dropped,'Inert native jettison did not release')
            require(next(r for r in w.weapons if r.id==inert_weapon).jettisoned,'Negative control armed itself')
        peak_thrust=0.;fuel_min=initial.fuel_kg;mass_min=initial_mass;after=before;starved=False;both_ended_at=None
        while w.time<launch+policy['seconds']:
            rows={r.id:r for r in w.weapons}
            for identity in [weapon]+([inert_weapon] if inert_weapon else []):
                row=rows[identity]
                if row.end!='in_flight':continue
                vehicle=w.vehicle(identity);state=vehicle.state
                require(not state.diverged and all(math.isfinite(x) for x in state.position_ecef),'Invalid explosive native state')
                frames.append(dict(time_s=w.time,weapon=str(identity),state=clean(state),mass_kg=vehicle.mass_kg,record=clean(row)))
                if identity==weapon:
                    peak_thrust=max(peak_thrust,max(state.thrust_n[:state.engine_count],default=0))
                    fuel_min=min(fuel_min,state.fuel_kg);mass_min=min(mass_min,vehicle.mass_kg)
                    if aim:after=math.dist(state.position_ecef,aim)
                    if case=='cruise':
                        require(not row.tracking and not row.supported and row.holding==0,'Coordinate cruise invented terminal sensing/support')
                        if w.time-launch>=30 and not starved:
                            vehicle.set_property('propulsion/tank[0]/contents-lbs',0);starved=True
            for event in w.engagement_events(cursor):
                cursor=max(cursor,event.serial);events.append(clean(event))
            if case=='bomb' and all(rows[i].end!='in_flight' for i in (weapon,inert_weapon)):
                if both_ended_at is None:both_ended_at=w.time
                if w.time>=both_ended_at+policy['aftermath_s']:break
            clock.step()
        rows={r.id:r for r in w.weapons}
        # Consume terminal events emitted by the final captured native interval.
        for event in w.engagement_events(cursor):cursor=max(cursor,event.serial);events.append(clean(event))
        if case=='cruise':
            require(rows[weapon].end=='in_flight' and peak_thrust>0,'Cruise did not sustain actual native flight')
            require(initial.fuel_kg>0 and fuel_min<initial.fuel_kg and mass_min<initial_mass,'Native turbofan fuel/mass did not deplete')
            require(after<before-5000,'Native cruise made insufficient fixed-coordinate progress')
            final=w.vehicle(weapon).state
            require(starved and final.fuel_kg<1e-6 and sum(final.thrust_n[:final.engine_count])<1,'Diagnostic native tank exhaustion did not remove thrust')
            require(not any(e['entity']==weapon and e['kind'] in ('acquisition','activation','detonation') for e in events),'Cruise acquired an unsupported terminal mode')
        else:
            require(rows[weapon].end!='in_flight' and rows[inert_weapon].end!='in_flight','Bombs did not reach natural terminal impact')
            require(any(e['entity']==weapon and e['kind']=='detonation' for e in events),'Armed natural terrain impact did not detonate')
            require(not any(e['entity']==inert_weapon and e['kind']=='detonation' for e in events),'Inert jettison detonated')
            require(both_ended_at is not None and w.time>=both_ended_at+policy['aftermath_s'],
                'Complete native post-impact presentation interval was not observed')
        native_seconds=w.time-clock.epoch;wall_seconds=time.perf_counter()-clock.wall
        if live:require(.95<=native_seconds/wall_seconds<=1.02,'Native live scene did not sustain real time1x')
        return dict(passed=not bool(health.invalid),case=case,inputs=dict(**policy,**ORIGIN),consent_endpoint=consent_endpoint,consent=consent,weapon=str(weapon),inert_weapon=str(inert_weapon) if inert_weapon else None,
            capability=clean(cap),initial_fuel_kg=initial.fuel_kg,minimum_fuel_kg=fuel_min,initial_mass_kg=initial_mass,
            minimum_mass_kg=mass_min,peak_thrust_n=peak_thrust,initial_distance_m=before,final_distance_m=after,
            diagnostic_tank_exhaustion=starved,terminal=clean(rows[weapon]),events=events,frames=frames,
            delivered_coordinate=received_report if case=='cruise' else None,coordinate_retirement=coordinate_retirement,
            terminal_aftermath_seconds=w.time-both_ended_at if both_ended_at is not None else None,
            launch_time_s=launch,duration_s=w.time-launch,frame_skip=4,
            world_health=health.rows,invalid_entities=sorted(health.invalid),
            complete_world_finite=not bool(health.invalid),observed_world_states=len(health.rows),
            native_simulation_seconds=native_seconds,native_wall_seconds=wall_seconds,real_time_factor=native_seconds/wall_seconds,
            scope='Non-isolated public SDK diagnostic; referee states/events never feed actor targeting')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--prefix',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
    p.add_argument('--case',choices=('all','cruise','bomb'),default='all');p.add_argument('--live',action='store_true')
    p.add_argument('--world');p.add_argument('--start-file',type=Path);p.add_argument('--record',type=Path)
    a=p.parse_args();prefix=allowed(a.prefix).resolve();output=allowed(a.output).resolve()
    require(not output.exists(),'Fresh explosive receipt required')
    require(not a.live or a.case!='all','A live Viewer scenario selects one explicit case')
    for path in (a.start_file,a.record):
        if path is not None:allowed(path)
    for key in tuple(os.environ):
        if key.startswith('FSIM_') and key!='FSIM_MAPS_DIR':os.environ.pop(key)
    os.environ['FSIM_DLL_DIR']=str(prefix/'bin')
    os.environ['FSIM_AIRCRAFT_PATH']=os.pathsep.join(str(prefix/'share/flightsim/aircraft'/part) for part in ('','stores'))
    sys.path.insert(0,str(prefix))
    import fsim
    require(Path(fsim.__file__).resolve().is_relative_to(prefix),'Facade outside installed product')
    sys.path.insert(0,str(ROOT/'tools/validation/radar'));from provenance import loaded_modules
    result=dict(schema='fsim.sim-m03.installed-explosives.v1',passed=False,live=a.live,cases=[],
        prefix=str(prefix),modules=loaded_modules(prefix),inputs=SCENARIOS,source_cards={},
        installed_aircraft_search_roots=os.environ['FSIM_AIRCRAFT_PATH'].split(os.pathsep))
    for name in ('cj-10a','mk82'):
        source=ROOT/'assets/stores'/(name+'.json');installed=prefix/'share/flightsim/stores'/(name+'.json')
        expected=hashlib.sha256(source.read_bytes()).hexdigest()
        require(hashlib.sha256(installed.read_bytes()).hexdigest()==expected,'Installed explosive card differs from current source')
        result['source_cards'][name]=dict(source=str(source),installed=str(installed),sha256=expected)
    def save():output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    try:
        for case in (('cruise','bomb') if a.case=='all' else (a.case,)):
            print('installed explosive '+case,flush=True)
            result['cases'].append(run_scene(fsim,prefix,case,live=a.live,world_name=a.world,start_file=a.start_file,record=a.record));save()
            require(result['cases'][-1]['passed'],'Native world diverged during the complete explosive lifecycle; '+str(output))
        result['passed']=True;save()
    except Exception as error:result['error']=str(error);save();raise


if __name__=='__main__':main()
