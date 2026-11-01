"""Publish a real-time native orbit-to-hover run for the normal Viewer entry.

This public-SDK client only submits commands and records observations. It does
not implement a flight model or replace the original native acceptance fixture.
"""
import argparse
import ctypes
import datetime
import hashlib
import json
import math
from pathlib import Path
import re
import time


def loaded_module(name):
    kernel=ctypes.WinDLL('kernel32',use_last_error=True)
    kernel.GetModuleHandleW.argtypes=[ctypes.c_wchar_p];kernel.GetModuleHandleW.restype=ctypes.c_void_p
    kernel.GetModuleFileNameW.argtypes=[ctypes.c_void_p,ctypes.c_wchar_p,ctypes.c_uint];kernel.GetModuleFileNameW.restype=ctypes.c_uint
    buffer=ctypes.create_unicode_buffer(32768);handle=kernel.GetModuleHandleW(name)
    if not handle or not kernel.GetModuleFileNameW(handle,buffer,len(buffer)):raise RuntimeError('Unknown loaded module '+name)
    path=Path(buffer.value)
    return dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def main(args):
    if not re.fullmatch('[a-z0-9_-]+',args.model):raise ValueError('Invalid authored model identity')
    if '.codex' in {part.casefold() for part in Path(args.output).parts}:raise ValueError('Excluded output')
    out=Path(args.output).resolve();root=Path(__file__).resolve().parents[3]
    if not out.is_relative_to(root/'build'):raise ValueError('Evidence must stay in the owned build tree')
    out.mkdir(exist_ok=True,parents=True)
    import fsim
    product=root/'aircraft'/args.model/(args.model+'.xml')
    original=product.read_bytes();refused=False
    poison=fsim.World(args.world+'-provenance',publish=False,workers=1)
    try:
        product.write_text('<invalid-normal-entry-product/>',encoding='utf-8')
        try:poison.create_vehicle('must-refuse',type='jsbsim:'+args.model,altitude_msl_m=100)
        except fsim.Error:refused=True
    finally:
        product.write_bytes(original);poison.close()
    if not refused:raise RuntimeError('Normal entry silently used another product')
    world=fsim.World(args.world,publish=True,dt=1/120,frame_skip=4,workers=1,pin_workers=False,seed=17,terrain=True)
    world.set_environment(epoch_utc_seconds=datetime.datetime(2026,11,1,20,tzinfo=datetime.timezone.utc).timestamp(),
                          time_factor=1,cloud_cover=.3,cloud_base_m=1800)
    rows=[];metadata=dict(schema='fsim.sim-m02.normal-flight-entry.v1',model=args.model,world=args.world,
        playback='real time 1x',exact_product_poison_refused=refused,
        role='Public SDK command/observation client; original native fixture remains separate')
    try:
        probe=world.create_vehicle('profile',type='jsbsim:'+args.model,altitude_msl_m=100)
        roll=probe.profile_value('hover/roll_attitude_deg');pitch=probe.profile_value('hover/pitch_attitude_deg')
        if not all(math.isfinite(v) for v in (roll,pitch)):raise ValueError('Native identified hover profile required')
        probe.remove()
        vehicle=world.create_vehicle('native-hover',type='jsbsim:'+args.model,latitude_deg=37.6188,longitude_deg=-122.375,
            altitude_msl_m=100,heading_deg=0,airspeed_ms=0,roll_deg=roll,pitch_deg=pitch)
        vehicle.submit(fsim.Level.VELOCITY,north_ms=0,east_ms=0,vertical_speed_ms=0)
        world.step()
        metadata.update(python_module=fsim.__file__,libraries={name:loaded_module(name) for name in ('libfsim.dll','libJSBSim.dll')},
            product_path=str(product),product_sha256=hashlib.sha256(original).hexdigest(),environment=world.environment)
        (out/'ready.json').write_text(json.dumps(metadata,indent=2)+'\n',encoding='utf-8')
        # Bounded scene-creation time, without advancing native simulation.
        time.sleep(20)
        activity=None;changed=False;target=None;start=time.monotonic();sim_start=world.time;dt=world.step_seconds
        for index in range(round((15+150)/dt)):
            tick=time.monotonic();t=index*dt
            if activity is None and t>=15:
                state=vehicle.state;target=(state.latitude_rad+80/6371008.8,state.longitude_rad,state.altitude_msl_m)
                activity=vehicle.submit_pattern(latitude_rad=target[0],longitude_rad=target[1],radius_m=60)
            if activity is not None and not changed and t>=45:
                activity.update(pattern=fsim.PatternKind.HOVER,duration_s=10);changed=True
            world.step();state=vehicle.state
            if index%3==0:
                rows.append(dict(time_s=t,latitude_rad=state.latitude_rad,longitude_rad=state.longitude_rad,
                    altitude_m=state.altitude_msl_m,velocity_ned_ms=list(state.velocity_ned_ms),
                    euler_rad=list(state.euler_rad),crashed=bool(state.crashed),diverged=bool(state.diverged),
                    activity_state=str(activity.state) if activity else None))
            if state.crashed or state.diverged:raise RuntimeError('Native flight ended before planned observation')
            time.sleep(max(0.,dt-(time.monotonic()-tick)))
        metadata.update(native_seconds=world.time-sim_start,wall_seconds=time.monotonic()-start,target=target,
            final_activity_state=str(activity.state),observations=rows)
        (out/'normal-entry-flight.json').write_text(json.dumps(metadata,indent=2,allow_nan=False)+'\n',encoding='utf-8')
        return 0
    finally:world.close()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',required=True);parser.add_argument('--world',required=True);parser.add_argument('--output',required=True)
    raise SystemExit(main(parser.parse_args()))
