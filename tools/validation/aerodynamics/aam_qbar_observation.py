"""All-roster native pressure-scaling observation and noninterference evidence.

The fixture now uses the explicitly approved first-force observation boundary.
The former end-of-public-step comparison remains a separate diagnostic.
Every native force sample is retained; the reference comparison requires an
explicitly equal first-sample aerodynamic state, with the original1e-8 bound.
"""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main(args):
    root=Path(__file__).resolve().parents[3]
    out=Path(args.output).resolve()
    if '.codex' in {p.casefold() for p in out.parts} or not out.is_relative_to(root/'build'):
        raise ValueError('Evidence must remain in the owned build tree')
    out.mkdir(parents=True,exist_ok=True)
    sys.path.insert(0,str(root/'tools/models/hangar'))
    from hangar.engineering import roster,cohort
    designs=[row for row in roster() if cohort(row)=='aam']
    if len(designs)!=24:raise ValueError('Incomplete maintained AAM roster')
    executable=root/'build'/args.preset/'bin/fsim-native-flight-probe.exe'
    environment=os.environ.copy()
    environment.update(TMP='D:/ENV/tmp',TEMP='D:/ENV/tmp',TORCH_EXTENSIONS_DIR='D:/ENV/tmp')
    started=time.monotonic();results=[]
    def write():
        report=dict(schema='fsim.aam-qbar-observation.v1',basis='simulated',preset=args.preset,
            executable_sha256=sha(executable),elapsed_seconds=time.monotonic()-started,
            source_sha256={p:sha(root/p) for p in (
                'src/simulation/flight/NativeStepObservationV1.h',
                'src/simulation/flight/JsbsimModel.cpp',
                'tools/validation/native_flight_probe/main.cpp',
                'tools/validation/aerodynamics/aam_qbar_observation.py',
                'tests/simulation/test_missile_control_native.cpp')},
            declared_designs=24,completed_designs=len(results),passed=sum(r['passed'] for r in results),
            tool_modifies_fixture=False,original_end_step_assertion_pass_claimed=False,
            tolerance=1e-8,scope='Same first evaluated native force state; full public-step propagation and terminal values retained',
            results=results)
        (out/'observation.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    for design in designs:
        name=design['name'];trace=out/(name+'.csv');log=out/(name+'.log');tick=time.monotonic()
        command=[str(executable),'--qbar-model',name,'--aircraft-dir',str(root/'aircraft/stores'),'--output',str(trace)]
        with log.open('wb') as stream:
            process=subprocess.Popen(command,cwd=root,env=environment,stdout=stream,stderr=subprocess.STDOUT)
            try:code=process.wait(timeout=60)
            except subprocess.TimeoutExpired:
                subprocess.run(['taskkill','/PID',str(process.pid),'/T','/F'],stdout=stream,stderr=subprocess.STDOUT,timeout=15)
                process.wait(timeout=15);code=124
        record=dict(design=name,command=command,exit_code=code,elapsed_seconds=time.monotonic()-tick,passed=False)
        results.append(record)
        if code:
            write();continue
        certificate=json.loads(Path(str(trace)+'.json').read_text(encoding='utf-8'))
        library=Path(certificate['loaded_jsbsim_library'])
        if library.name.casefold()!='libjsbsim.dll' or not library.is_file():
            raise ValueError('Cannot bind the actually loaded JSBSim library')
        certificate['loaded_jsbsim_sha256']=sha(library)
        rows=list(csv.DictReader(trace.open(encoding='utf-8')))
        grouped={plant:[r for r in rows if r['plant']==plant] for plant in ('ordinary','thin')}
        expected=certificate['substeps_per_plant']
        numeric=[float(v) for r in rows for k,v in r.items() if k not in ('plant','stage')]
        checks=dict(all_values_finite=all(map(math.isfinite,numeric)),all_substeps_present=True,
            first_state_equal=True,first_qbar_response=True,first_actual_fin_nonzero=True,
            first_six_loads_scale=True,terminal_properties_equal_unobserved=True,
            public_state_bitwise_unaffected=certificate['observed_unobserved_bitwise_identity'],
            observer_scope_restored=certificate['observer_scope_restored'])
        final=[];first={}
        for plant,values in grouped.items():
            native=[r for r in values if r['stage'].startswith('native-')]
            checks['all_substeps_present'] &= [r['stage'] for r in native]==['native-'+str(k+1) for k in range(expected)]
            first[plant]=native[0]
            end=next(r for r in values if r['stage']=='after')
            plain=next(r for r in values if r['stage']=='unobserved-after')
            for key in end:
                if key not in ('plant','stage'):checks['terminal_properties_equal_unobserved'] &= float(end[key])==float(plain[key])
            final.append(end)
        a,b=first['ordinary'],first['thin']
        state_fields=['velocities/mach','aero/alpha-rad','aero/beta-rad',
                      'fcs/elevator-pos-rad','fcs/aileron-pos-rad','fcs/rudder-pos-rad']
        checks['first_state_equal']=all(float(a[k])==float(b[k]) for k in state_fields)
        qa,qb=float(a['aero/qbar-psf']),float(b['aero/qbar-psf'])
        checks['first_qbar_response']=qa>0 and 0<qb<.75*qa
        checks['first_actual_fin_nonzero']=float(a['fcs/elevator-pos-rad'])!=0 and abs(float(a['aero/force/fsim-fin-z-lbs']))>1e-6
        loads=['aero/force/fsim-fin-'+axis+'-lbs' for axis in 'xyz']+['aero/moment/fsim-fin-'+axis+'-lbsft' for axis in 'lmn']
        comparisons=[]
        for key in loads:
            x,y=float(a[key])/qa,float(b[key])/qb
            difference=abs(x-y);limit=1e-8*max(1.,abs(x))
            checks['first_six_loads_scale'] &= difference<=limit
            comparisons.append(dict(property=key,ordinary_normalized=x,thin_normalized=y,difference=difference,limit=limit))
        x=float(final[0]['aero/force/fsim-fin-z-lbs'])/float(final[0]['aero/qbar-psf'])
        y=float(final[1]['aero/force/fsim-fin-z-lbs'])/float(final[1]['aero/qbar-psf'])
        record.update(passed=all(checks.values()),checks=checks,certificate=certificate,first_six_loads=comparisons,
            original_terminal_comparison=dict(difference=abs(x-y),limit=1e-8*max(1.,abs(x)),passed=abs(x-y)<=1e-8*max(1.,abs(x))),
            product_sha256=sha(root/'aircraft/stores'/name/(name+'.xml')),trace_sha256=sha(trace))
        write()
    print(json.dumps(dict(designs=len(results),passed=sum(r['passed'] for r in results),elapsed_seconds=time.monotonic()-started)))
    return int(len(results)!=24 or not all(r['passed'] for r in results))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preset',choices=['ucrt64-headless','ucrt64-release'],required=True)
    parser.add_argument('--output',required=True)
    raise SystemExit(main(parser.parse_args()))
