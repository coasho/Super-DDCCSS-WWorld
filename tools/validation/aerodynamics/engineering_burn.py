"""Run the original declared-motor flight gate without changing its observable.

Every design receives an applicability record. An authored motor makes the
original burn trajectory required; implementation limitations cannot waive it.
This is separate from table/component completeness and from coefficient sampling.
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
import tomllib
import traceback
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[3]


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main(args):
    started=time.monotonic();build=Path(args.build).resolve();out=Path(args.output).resolve()
    out.parent.mkdir(parents=True,exist_ok=True)
    sys.path.insert(0,str(ROOT/'tools/models/hangar'))
    sys.path.insert(0,str(build/'python'))
    os.environ.update(FSIM_DLL_DIR=str(build/'python/fsim/bin'),
        FSIM_AIRCRAFT_PATH=str(build/'share/flightsim/aircraft')+';'+str(build/'share/flightsim/aircraft/stores'))
    directory=os.add_dll_directory(str(build/'python/fsim/bin'))
    import fsim
    from hangar import engineering,pipeline,storestages
    fsim.set_log_level('off')
    kernel=ctypes.WinDLL('kernel32',use_last_error=True)
    kernel.GetModuleHandleW.argtypes=[ctypes.c_wchar_p];kernel.GetModuleHandleW.restype=ctypes.c_void_p
    kernel.GetModuleFileNameW.argtypes=[ctypes.c_void_p,ctypes.c_wchar_p,ctypes.c_uint];kernel.GetModuleFileNameW.restype=ctypes.c_uint
    handle=kernel.GetModuleHandleW('libfsim.dll');buf=ctypes.create_unicode_buffer(32768)
    if not handle or not kernel.GetModuleFileNameW(handle,buf,len(buf)):raise RuntimeError('Missing actual loaded DLL provenance')
    dll=Path(buf.value).resolve()
    if not dll.is_relative_to(build):raise ValueError('Original motor flight refused a foreign native DLL')
    records=[];declarations=engineering.roster()
    def publish():
        required=[r for r in records if r['applicability']=='required']
        result=dict(schema='fsim.sim-m02.original-motor-flight.v1',basis='simulated',
            scope='Original motor burn trajectory, mass, timing and speed-gain acceptance; no propulsive-integral substitution',
            declared_designs=len(declarations),completed_designs=len(records),required_cases=len(required),
            passed_cases=sum(r['passed'] for r in required),failed_cases=sum(not r['passed'] for r in required),
            not_applicable_designs=len(records)-len(required),elapsed_seconds=time.monotonic()-started,
            loaded_dll=str(dll),loaded_dll_sha256=sha(dll),python_module=fsim.__file__,
            acceptance_source=str(Path(storestages.__file__).resolve()),acceptance_source_sha256=sha(storestages.__file__),
            command=[sys.executable,*sys.argv],designs=records)
        out.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
        return result
    for item in declarations:
        path=Path(item['path']);spec=tomllib.loads(path.read_text(encoding='utf-8'))
        required='motor' in spec
        row=dict(name=item['name'],category=item['category'],applicability='required' if required else 'not_applicable',
            reason='Authored motor requires the original burn-flight gate' if required else 'Authored configuration has no motor; other propulsion uses its native topology checks',
            source_sha256=sha(path),basis='simulated',passed=not required)
        tick=time.monotonic()
        if required:
            if time.monotonic()-started>1200:
                row.update(timed_out=True,error='Original motor-flight invocation exceeded1200seconds')
            else:
                try:
                    design=pipeline.Design(str(path),log=lambda _:None)
                    card=storestages.card_of(design)
                    row['card_sha256']=sha(ROOT/'assets/stores'/(card['id']+'.json'))
                    product=build/'share/flightsim/aircraft/stores'/item['name']/(item['name']+'.xml')
                    source_product=ROOT/'aircraft/stores'/item['name']/(item['name']+'.xml')
                    if sha(product)!=sha(source_product):raise ValueError('Original motor flight refused stale staged XML')
                    tree=ET.parse(product)
                    properties={(n.text or '').strip():n.get('value') for n in tree.findall('flight_control/property')}
                    expected_version=2 if spec.get('integration',{}).get('version')==2 and spec.get('integration',{}).get('mode')=='derived' else 1
                    if int(properties.get('fsim/integration-v1/version','0'))!=expected_version:raise ValueError('Source/staged numerical version mismatch')
                    expected_method=4 if expected_version==2 else 1
                    substeps=int(properties['fsim/integration-v1/substeps'])
                    original_world=storestages._world
                    observed=[]
                    class ObservedWorld:
                        def __init__(self,world):self.world=world
                        def __getattr__(self,key):return getattr(self.world,key)
                        def create_vehicle(self,*args,**kwargs):
                            vehicle=self.world.create_vehicle(*args,**kwargs)
                            observed.append(dict(rotational_rate_method=vehicle.get_property('simulation/integrator/rate/rotational'),
                                internal_dt_s=vehicle.get_property('simulation/dt')))
                            return vehicle
                    def observe_world(*args,**kwargs):
                        module,world=original_world(*args,**kwargs)
                        return module,ObservedWorld(world)
                    # No save/report or generated-source mutation: execute the
                    # exact original function, observing without altering it.
                    storestages._world=observe_world
                    try:result=storestages._burn(design,card)
                    finally:storestages._world=original_world
                    row.update(result)
                    row['failed_checks']=[c for c in result['checks'] if c['status']=='fail']
                    row['passed']=bool(result['checks']) and not row['failed_checks'] and all(c['status']=='pass' for c in result['checks'])
                    row['runtime_product']=str(product);row['runtime_product_sha256']=sha(product)
                    row['native_numerical_resolution']=observed
                    if len(observed)!=1 or observed[0]['rotational_rate_method']!=expected_method or not math.isclose(observed[0]['internal_dt_s'],1/(120*substeps),rel_tol=1.e-12,abs_tol=1.e-15):raise ValueError('Actual native numerical method/resolution mismatch')
                    if any(not math.isfinite(float(v)) for sample in result['burn'] for v in sample):raise ValueError('Nonfinite original burn trajectory')
                except Exception as error:
                    row.update(passed=False,error=str(error),traceback=traceback.format_exc())
        row['elapsed_seconds']=time.monotonic()-tick;records.append(row);publish()
        print(item['name'],row['applicability'],row['passed'],round(row['elapsed_seconds'],3),flush=True)
    result=publish()
    if len(declarations)!=88 or len(records)!=len(declarations):raise ValueError('Incomplete declared original motor-flight applicability roster')
    return int(bool(result['failed_cases']))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--build',required=True);p.add_argument('--output',required=True)
    raise SystemExit(main(p.parse_args()))
