"""Live native radar/link fusion, through public Agent calls, at real time 1x.

Run with fsim --preset ucrt64-release python examples/simulation/python/radar_fusion.py
and attach the normal Viewer to --world radar-fusion. This is an observational,
non-isolated diagnostic scene. Native measurements, delivery and fusion remain
the SDK's; neither this example nor the Viewer estimates target truth.
"""
import argparse
import datetime
import json
import math
from pathlib import Path
import time

import fsim


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world',default='radar-fusion')
    parser.add_argument('--mode',choices=('fighter','aew'),default='fighter')
    parser.add_argument('--seconds',type=float,default=65)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--start-file',type=Path,help='Optional viewer-ready synchronization, before the first step')
    args=parser.parse_args()
    if args.seconds<60:raise ValueError('The complete mode/loss demonstration requires at least60s')
    if args.output:
        if args.output.exists():raise ValueError('Use a fresh trace path')
        args.output.parent.mkdir(parents=True,exist_ok=True)
    world=fsim.World(args.world,publish=True,seed=3,workers=4,pin_workers=False)
    world.set_environment(epoch_utc_seconds=datetime.datetime(2026,6,21,20,tzinfo=datetime.timezone.utc).timestamp())
    lat,lon=37.2,-122.4
    def vehicle(name,kind,north,east,heading,speed=200):
        v=world.create_vehicle(name,'jsbsim:'+kind,latitude_deg=lat+north/111.2,
            longitude_deg=lon+east/(111.2*math.cos(math.radians(lat))),altitude_msl_m=6000,
            heading_deg=heading,airspeed_ms=speed)
        v.submit_hsa(heading_rad=math.radians(heading),speed=speed,altitude_m=6000)
        return v
    own=vehicle('sensor','f16c' if args.mode=='fighter' else 'e7a',0,0,90)
    receiver=vehicle('link receiver','f16c',-2,0,90)
    target=vehicle('air target','f16c',0,20,270)
    if args.mode=='aew':
        vehicle('rear target','f16c',0,-20,90,400)
        vehicle('crossing target','f16c',20,0,180)
    net=world.create_net('fusion demonstration',latency_s=.2)
    for v in (own,receiver):world.join_net(v,net)
    sensor=world.create_agent(own,'sensor');wing=world.create_agent(receiver,'receiver')
    sensor.set_payload_state('radar','operate');world.focus_viewer(own);world.step()
    print(json.dumps(dict(event='ready',world=args.world,mode=args.mode,source='native SDK',isolation='non-isolated observational scene')),flush=True)
    if args.start_file:
        deadline=time.perf_counter()+120
        while not args.start_file.exists():
            if time.perf_counter()>deadline:raise TimeoutError('Viewer-ready handshake timed out')
            time.sleep(.05)
    origin=world.time;start=time.perf_counter();events=[];samples=[];stage=0;last_sample=-1;last_publish=-1;selected=None;receiver_search=False
    def event(name):
        row=dict(time_s=world.time,event=name);events.append(row);print(json.dumps(row),flush=True)
    pattern=dict(azimuth_half_width_rad=math.radians(3),bars=2,elevation_centre_rad=-own.state.euler_rad[1],range_scale_m=100000)
    def entities(rows):
        result=[]
        for entity in rows:
            row=entity._asdict();row['id']=str(entity.id)
            row['standard_identity']=entity.standard_identity if entity.identity_known else None
            row['sources']=[dict(source._asdict(),track=str(source.track)) for source in entity.sources]
            result.append(row)
        return result
    while world.time-origin<args.seconds:
        now=world.time
        if stage==0 and own.payload('radar').state==fsim.PayloadState.OPERATE:
            sensor.amti_search(mode='surveillance' if args.mode=='aew' else 'range_while_search',signal_level=True,**pattern)
            event('native signal search');stage=1
        if args.mode=='fighter':
            if stage==1 and now>=8:
                tracks=sensor.amti_tracks()
                if tracks:
                    selected=min(tracks,key=lambda t:t.range_m).id;sensor.amti_track_v1(selected,signal_level=True)
                    event('native signal STT');stage=2
            if stage==2 and now>=16:
                sensor.amti_track_v1(selected,mode='track_while_scan',signal_level=True,
                    azimuth_half_width_rad=pattern['azimuth_half_width_rad'],bars=2,range_scale_m=100000)
                event('native signal TWS');stage=3
            if stage==3 and now>=24:
                sensor.amti_search(mode='velocity_search',signal_level=True,**pattern)
                event('native velocity-only search');stage=4
        if int(now)!=last_publish and now<35:
            last_publish=int(now);sensor.publish_tracks(destinations=[wing.system])
        if now>=18 and not receiver_search:
            wing.set_payload_state('radar','operate');world.step()
            wing.amti_search(azimuth_half_width_rad=math.radians(15),bars=2,
                elevation_centre_rad=-receiver.state.euler_rad[1],signal_level=True,range_scale_m=100000)
            receiver_search=True;event('receiver radar joins delivered-link fusion')
        if now>=12 and now<12+world.step_seconds:world.focus_viewer(receiver);event('focus delivered-link knowledge')
        if now>=22 and now<22+world.step_seconds:world.focus_viewer(own);event('focus local knowledge')
        if now>=35 and stage<10:
            sensor.set_payload_state('radar','off');wing.set_payload_state('radar','off');stage=10;event('sensors off; native reports coast then expire')
            world.focus_viewer(receiver)
        world.step()
        if int(world.time)!=last_sample:
            last_sample=int(world.time)
            row=dict(time_s=world.time,wall_s=time.perf_counter()-start,
                sensor=entities(sensor.fused_tracks_v1()),receiver=entities(wing.fused_tracks_v1()),
                sensor_status=sensor.fusion_status_v1(),receiver_status=wing.fusion_status_v1())
            samples.append(row);print(json.dumps(row,allow_nan=False),flush=True)
        ahead=world.time-origin-(time.perf_counter()-start)
        if ahead>0:time.sleep(ahead)
    elapsed=time.perf_counter()-start
    record=dict(schema='fsim.sim-m01.live-example.v1',mode=args.mode,world=args.world,events=events,samples=samples,
        start_time_s=origin,end_time_s=world.time,simulated_s=world.time-origin,elapsed_s=elapsed,real_time_factor=(world.time-origin)/elapsed,
        source='native SDK Agent reports; no client estimation',isolation='non-isolated observational scene')
    if args.output:args.output.write_text(json.dumps(record,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps({key:value for key,value in record.items() if key not in ('events','samples')}),flush=True)


if __name__=='__main__':main()
