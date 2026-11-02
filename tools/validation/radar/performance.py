"""Current public-SDK radar/fusion throughput; no hardware-fidelity claim."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics
import time

import fsim
from provenance import loaded_modules


def sample(radar_on):
    w=fsim.World('sim-m01-throughput',publish=False,terrain=False,workers=4,pin_workers=False,seed=3)
    vehicles=[]
    for side in range(2):
        for row in range(4):
            v=w.create_vehicle(str(side)+'-'+str(row),'jsbsim:f16c',latitude_deg=40+.02*row,
                longitude_deg=.15*side,altitude_msl_m=5000,airspeed_ms=200,heading_deg=90+180*side)
            v.submit_hsa(heading_rad=math.radians(90+180*side),speed=200,altitude_m=5000);vehicles.append(v)
    if radar_on:
        for v in vehicles:v.set_payload_state('radar','operate')
        w.step()
        for v in vehicles:v.amti_search(azimuth_half_width_rad=math.radians(25),bars=2)
    w.step(120)
    times=[]
    for _ in range(600):
        start=time.perf_counter();w.step();times.append(time.perf_counter()-start)
    pictures=[v.fused_tracks_v1() for v in vehicles]
    return dict(aircraft=8,workers=4,radar_on=radar_on,steps=len(times),simulated_s=len(times)*w.step_seconds,
        elapsed_s=sum(times),real_time_factor=len(times)*w.step_seconds/sum(times),
        mean_step_s=statistics.mean(times),p99_step_s=sorted(times)[math.ceil(.99*len(times))-1],
        fused_entities=sum(map(len,pictures)),rejected_inputs=sum(v.fusion_status_v1().rejected_inputs for v in vehicles))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise ValueError('Preserve previous measurement')
    package=Path(fsim.__file__).parent;start=time.perf_counter();modules=loaded_modules(package.parent)
    result=dict(schema='fsim.sim-m01.throughput.v1',basis='simulated',equipment_calibration=False,
        sdk_sha256=next(m['sha256'] for m in modules if m['name']=='libfsim.dll'),loaded_modules=modules,
        package_sdk_sha256=hashlib.sha256((package/'bin/libfsim.dll').read_bytes()).hexdigest(),
        method='Same eight-aircraft deterministic flight population; 120 warmup/600 timed steps, four workers, no Viewer',
        cases=[sample(False),sample(True)])
    result['elapsed_s']=time.perf_counter()-start
    result['native_input_acceptance']=all(row['rejected_inputs']==0 for row in result['cases'])
    a.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8');print(json.dumps(result))
    raise SystemExit(not result['native_input_acceptance'])


if __name__=='__main__':main()
