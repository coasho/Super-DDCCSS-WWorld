"""Serial public-SDK functional sampling for every declared air-radar fit.

This is functional sampling, not an equipment-accuracy calibration or a full
angle/envelope validation. No card, model, scene threshold or reference changes.
Run through tools/launch/fsim.ps1 python (or an installed Python SDK).
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time

import fsim
from provenance import loaded_modules


def check(value,message):
    if not value:raise AssertionError(message)


def sample(fit,limit):
    started=time.perf_counter();checks=[];design=fit['design']
    w=fsim.World('sim-m01-'+design,publish=False,workers=1,pin_workers=False,terrain=False,seed=3)
    own=w.create_vehicle('own','jsbsim:'+design,latitude_deg=40,longitude_deg=0,
        altitude_msl_m=5000,airspeed_ms=200,heading_deg=90)
    sensor=fit['air_radars'][0]['payload']
    status=own.radar_status(sensor);check(status is not None,'declared air radar has no native status')
    distance=40000 if status.rotodome else 8000
    target=w.create_vehicle('target','jsbsim:f16c',latitude_deg=40,
        longitude_deg=distance/(6371e3*math.cos(math.radians(40)))*180/math.pi,
        altitude_msl_m=5000,airspeed_ms=200,heading_deg=270)
    own.submit_hsa(heading_rad=math.pi/2,speed=200,altitude_m=5000)
    target.submit_hsa(heading_rad=3*math.pi/2,speed=200,altitude_m=5000)
    agent=w.create_agent(own,'sensor')
    agent.set_payload_state(sensor,'operate');w.step()
    def advance_until(predicate,seconds):
        stop=w.time+seconds
        while w.time<stop and not predicate():
            if time.perf_counter()-started>limit:raise TimeoutError('per-design functional sampling wall-time budget exceeded')
            w.step()
        return predicate()
    def tracks():return [t for t in own.amti_tracks(sensor) if t.truth==target.id]
    mode='surveillance' if status.rotodome else 'range_while_search'
    pattern=dict(azimuth_half_width_rad=math.radians(3),bars=2,elevation_centre_rad=-own.state.euler_rad[1],range_scale_m=100000)
    agent.amti_search(sensor,mode=mode,**pattern)
    check(advance_until(lambda:bool(tracks()),80 if status.rotodome else 6),'detection-level track unavailable')
    local=agent.fused_tracks_v1();check(local,'local native track missing from fusion')
    check(all(not t.identity_known for t in local),'unsupported fused identity')
    checks.append(dict(stage='detection_track_and_fusion',passed=True,sim_time_s=w.time,tracks=len(tracks())))
    if status.rotodome:
        before=own.radar_status(sensor).detections
        agent.amti_search(sensor,mode=mode,signal_level=True,**pattern)
        check(advance_until(lambda:any(d.truth==target.id for d in own.amti_detections(sensor,since=before)),20),
              'signal surveillance target not measured')
        checks.append(dict(stage='signal_surveillance',passed=True,sim_time_s=w.time))
    else:
        track=tracks()[0];before=own.radar_status(sensor).detections;command_time=w.time
        agent.amti_track_v1(track.id,sensor,signal_level=True,range_scale_m=100000)
        check(advance_until(lambda:any(t.id==track.id and t.last_update_s>command_time for t in tracks()),1),
              'signal STT did not update its designated native track')
        check(own.radar_status(sensor).signal_level,'STT did not use signal receiver')
        checks.append(dict(stage='signal_stt',passed=True,sim_time_s=w.time))
        command_time=w.time
        agent.amti_track_v1(track.id,sensor,mode='track_while_scan',signal_level=True,
            azimuth_half_width_rad=math.radians(3),bars=2,range_scale_m=100000)
        check(advance_until(lambda:any(t.id==track.id and t.last_update_s>command_time for t in tracks()),1),
              'signal TWS did not update its designated native track')
        check(own.radar_status(sensor).signal_level,'TWS did not use signal receiver')
        checks.append(dict(stage='signal_tws',passed=True,sim_time_s=w.time))
        # Exercise a fresh acquisition after the documented power-off file clear.
        agent.set_payload_state(sensor,'off');agent.set_payload_state(sensor,'operate');w.step()
        agent.amti_acquire_v1(sensor,signal_level=True,elevation_centre_rad=-own.state.euler_rad[1])
        check(advance_until(lambda:own.radar_status(sensor).task==fsim.AmtiTask.TRACK,6),'fresh signal acquisition did not confirm')
        checks.append(dict(stage='signal_cued_acquisition',passed=True,sim_time_s=w.time))
        before=own.radar_status(sensor).detections;command_time=w.time
        agent.amti_search(sensor,mode='velocity_search',signal_level=True,**pattern)
        check(advance_until(lambda:any(d.truth==target.id for d in own.amti_detections(sensor,since=before)),1),
              'high-PRF target not measured')
        check(all(t.last_update_s<=command_time for t in own.amti_tracks(sensor)),
              'ambiguous high-PRF ranges updated Cartesian tracks')
        checks.append(dict(stage='signal_velocity_search',passed=True,sim_time_s=w.time))
    check(agent.fusion_status_v1().rejected_inputs==0,'native fusion rejected live sensor reports')
    return dict(design=design,card=fit['air_radars'][0]['card'],rotodome=status.rotodome,passed=True,checks=checks,
        elapsed_s=time.perf_counter()-started,basis='simulated',equipment_calibration=False)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--inventory',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--max-design-wall-s',type=float,default=120)
    a=p.parse_args();inventory=json.loads(a.inventory.read_text(encoding='utf-8'));started=time.perf_counter();rows=[]
    if a.output.exists():raise ValueError('Preserve previous sampling; use a new output path')
    package=Path(fsim.__file__).parent;modules=loaded_modules(package.parent)
    result=dict(schema='fsim.sim-m01.public-functional-sampling.v1',basis='simulated',equipment_calibration=False,
        inventory_sha256=hashlib.sha256(a.inventory.read_bytes()).hexdigest(),python_package=str(package),
        sdk_sha256=next(m['sha256'] for m in modules if m['name']=='libfsim.dll'),loaded_modules=modules,
        package_sdk_sha256=hashlib.sha256((package/'bin/libfsim.dll').read_bytes()).hexdigest(),rows=rows)
    def save():
        result['elapsed_s']=time.perf_counter()-started
        result['checked_stages']=sum(len(r.get('checks',[])) for r in rows)
        result['passed']=len(rows)==len(inventory['fits']) and all(r.get('passed',False) for r in rows)
        a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    for fit in inventory['fits']:
        tick=time.perf_counter()
        if not fit['air_radars']:
            rows.append(dict(design=fit['design'],passed=True,checks=[],
                applicability='no_air_radar_in_declared_fit',verified_equipment_absence=False))
        else:
            try:rows.append(sample(fit,a.max_design_wall_s))
            except Exception as error:rows.append(dict(design=fit['design'],passed=False,error=str(error),elapsed_s=time.perf_counter()-tick))
        save();print(json.dumps(rows[-1]),flush=True)
    save();print(json.dumps({k:v for k,v in result.items() if k!='rows'}),flush=True)
    raise SystemExit(not result['passed'])


if __name__=='__main__':main()
