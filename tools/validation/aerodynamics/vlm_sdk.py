"""Measure generated Hangar tables through the actual SDK flight plant.

The existing Hangar writer tolerance is 0.002 in normalized force/moment
coefficients. This measures implementation fidelity, not equipment accuracy.
Run with the selected SDK's Python directory on PYTHONPATH.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import ctypes
import hashlib
import itertools
import json
import math
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np

from hangar import airframe, pipeline
from hangar.verify import TableModel, body_forces

FT=.3048
LBF=4.4482216152605
PSF=LBF/FT**2
ROOT=Path(__file__).resolve().parents[3]
_library_handles=[]


def load_native(expected_bin):
    """Select explicit build bytes before importing the extension module."""
    if expected_bin is None:return
    import os
    directory=Path(expected_bin).resolve()
    _library_handles.append(os.add_dll_directory(str(directory)))
    # AddDllDirectory ordering is unspecified. Loading the absolute library
    # first prevents a package's stale same-named copy from winning the search.
    _library_handles.append(ctypes.WinDLL(str(directory/'libfsim.dll'),winmode=0x100|0x1000))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def native_library(expected_bin=None):
    kernel=ctypes.WinDLL('kernel32',use_last_error=True)
    kernel.GetModuleHandleW.argtypes=[ctypes.c_wchar_p]
    kernel.GetModuleHandleW.restype=ctypes.c_void_p
    kernel.GetModuleFileNameW.argtypes=[ctypes.c_void_p,ctypes.c_wchar_p,ctypes.c_uint]
    for name in ('libfsim.dll','fsim.dll'):
        handle=kernel.GetModuleHandleW(name)
        if handle:
            buffer=ctypes.create_unicode_buffer(32768)
            if kernel.GetModuleFileNameW(handle,buffer,len(buffer)):
                path=Path(buffer.value)
                result=dict(path=str(path),sha256=sha(path))
                if expected_bin is not None:
                    expected=Path(expected_bin)/path.name
                    if not expected.is_file() or sha(expected)!=result['sha256']:
                        raise RuntimeError('Loaded SDK library differs from the selected build; rebuild python_package')
                    result['expected_build_path']=str(expected.resolve())
                return result
    raise RuntimeError('Cannot identify the loaded SDK library')


def condition(world, vehicle, alpha, beta, mach, rates=(0.,0.,0.)):
    """Use writable JSBSim initial conditions; body velocity outputs are read-only."""
    speed=mach*vehicle.get_property('atmosphere/a-fps')
    a,b=math.radians(alpha),math.radians(beta)
    for key,value in zip(('u','v','w'),(speed*math.cos(a)*math.cos(b),speed*math.sin(b),speed*math.sin(a)*math.cos(b))):
        vehicle.set_property('ic/'+key+'-fps',value)
    for key,value in zip('pqr',rates):vehicle.set_property('ic/'+key+'-rad_sec',value)
    vehicle.set_property('simulation/reset',0.)
    world.step(1)


def sample(vehicle, d, table):
    g=vehicle.get_property
    alpha,beta=g('aero/alpha-rad'),g('aero/beta-rad')
    speed=max(g('velocities/vt-fps')*FT,1.e-9)
    qbar=g('aero/qbar-psf')*PSF
    if not math.isfinite(qbar) or qbar<=0:raise ValueError('Invalid SDK dynamic pressure')
    controls={key:g(prop) for key,prop in (
        ('elevator','fcs/elevator-pos-deg'),('aileron','fcs/left-aileron-pos-deg'),
        ('rudder','fcs/rudder-pos-deg'),('flap','fcs/flap-pos-deg')) if key in table['controls']}
    pieces={i:g(piece['property']) for i,piece in enumerate(table.get('native_controls',[]))}
    a=d.aircraft
    coefficient=TableModel(table).evaluate(math.degrees(alpha),math.degrees(beta),
        p=g('velocities/p-aero-rad_sec')*a.b/(2*speed),
        q=g('velocities/q-aero-rad_sec')*a.c/(2*speed),
        r=g('velocities/r-aero-rad_sec')*a.b/(2*speed),
        adot=g('aero/alphadot-rad_sec')*a.c/(2*speed),controls_deg=controls,
        pieces_rad=pieces,mach=g('velocities/mach'),cl2=g('aero/cl-squared'),
        h_b=g('aero/h_b-mac-ft'),aspect_ratio=a.b*a.b/a.S)
    # Generated retractable-gear drag is separate from the neutral tables.
    from hangar.drag import gear_drag
    gear=gear_drag(a)
    if gear:
        fraction=g('gear/gear-pos-norm')
        coefficient['CD']+=fraction*gear[0]/a.S
        coefficient['Cm']+=fraction*gear[0]/a.S*(gear[1][2]-a.aero_point[2])/a.c
    force,moment=body_forces(coefficient,alpha,beta,qbar,a.S,a.b,a.c)
    cg=np.array([g('inertia/cg-'+axis+'-in') for axis in 'xyz'])*.0254
    arm=(a.aero_point-cg)*np.array([-1.,1.,-1.])
    moment+=np.cross(arm,force)
    observed_force=np.array([g('forces/fb'+axis+'-aero-lbs') for axis in 'xyz'])*LBF
    observed_moment=np.array([g('moments/'+axis+'-aero-lbsft') for axis in 'lmn'])*LBF*FT
    # Multirotor rotor drag/torque are physical propulsion loads, retained by
    # the generator in wind axes. Report the airframe table residual separately.
    if a.spec.get('aircraft',{}).get('category')=='multirotor':
        loads={key:g('aero/propulsion/'+key.lower()) for key in ('DRAG','SIDE','LIFT')}
        f,m=body_forces(dict(CD=loads['DRAG'],CY=loads['SIDE'],CL=loads['LIFT'],Cl=0.,Cm=0.,Cn=0.),alpha,beta,1.,1.,1.,1.)
        observed_force-=f*LBF
        observed_moment-=np.cross(arm,f*LBF)
        for i,names in enumerate(getattr(d,'propulsion_moments',[])):
            observed_moment[i]-=sum(g(key) for key in names)*LBF*FT
    error=np.r_[(observed_force-force)/(qbar*a.S),
                (observed_moment-moment)/(qbar*a.S*np.array([a.b,a.c,a.b]))]
    if not np.isfinite(error).all():raise ValueError('Nonfinite SDK comparison')
    return dict(alpha_deg=math.degrees(alpha),beta_deg=math.degrees(beta),mach=g('velocities/mach'),
                error=error.tolist(),controls_deg=controls,physical_controls_rad=pieces)


def validate(name, full_grid=False):
    import fsim
    started=time.monotonic()
    d=airframe.design(pipeline.find_design(name));a=d.aircraft
    table=d.calibrated(d.tables())
    store=bool(a.spec['aircraft'].get('card'))
    xml=ROOT/'aircraft'/('stores/'+name if store else name)/(name+'.xml')
    if a.category=='multirotor':
        tree=ET.parse(xml)
        d.propulsion_moments=[[fn.get('name') for fn in tree.findall("./aerodynamics/axis[@name='%s']/function"%axis)
                               if not fn.get('name','').startswith('aero/coefficient/')]
                              for axis in ('ROLL','PITCH','YAW')]
    w=fsim.World('vlm-'+name,dt=1.e-5,frame_skip=1,publish=False,workers=1,
                 pin_workers=False,seed=3,terrain=False)
    rows=[]
    try:
        v=w.create_vehicle('probe',type='jsbsim:'+name,latitude_deg=37.6,longitude_deg=-122.4,
                           altitude_msl_m=10000.,airspeed_ms=150.)
        v.set_protection('off')
        if v.get_property('fsim/aerodynamics/vlm-version')!=1:raise ValueError('Wrong aerodynamic resource loaded')
        alphas=table['alpha'] if full_grid else [-180.,-90.,-20.,0.,10.,90.,180.]
        betas=table['beta'] if full_grid else [-90.,-20.,0.,20.,90.]
        max_mach=float(table['mach']['mach'][-1])
        machs=sorted(set([.3,min(.9,max_mach),min(1.2,max_mach),max_mach]))
        for alpha,beta,mach in itertools.product(alphas,betas,machs):
            condition(w,v,float(alpha),float(beta),mach)
            rows.append(sample(v,d,table))
        # Exercise both signed demand, actual finite actuator positions and rates.
        for sign in (-1.,1.):
            v.command_actuator(elevator=.25*sign,aileron=.2*sign,rudder=.2*sign,throttle=0.)
            condition(w,v,5.,3.,.5,rates=(.01*sign,-.015*sign,.02*sign))
            w.step(20000)
            rows.append(sample(v,d,table))
        errors=np.abs(np.array([r['error'] for r in rows]))
        maximum=errors.max(axis=0)
        worst=int(np.argmax(errors.max(axis=1)))
        return dict(status='passed' if float(maximum.max())<=.002 else 'failed',conditions=len(rows),
            max_coefficient_error=dict(zip(('CX','CY','CZ','Cl','Cm','Cn'),maximum.tolist())),
            worst=rows[worst],xml_sha256=sha(xml),source_sha256=sha(Path(d.path)),
            control_cases=rows[-2:],
            calibration_sha256=sha(Path(d.dir)/'calibration.toml') if (Path(d.dir)/'calibration.toml').is_file() else None,
            physical_accuracy_error_percent=None,physical_accuracy_status='No matched independent equipment dataset',
            elapsed_seconds=time.monotonic()-started)
    finally:w.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',action='append',required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--full-grid',action='store_true')
    parser.add_argument('--native-bin',type=Path)
    parser.add_argument('--jobs',type=int,default=1,help='Concurrent isolated model processes for --model all')
    args=parser.parse_args()
    if args.jobs<1:parser.error('--jobs must be positive')
    load_native(args.native_bin)
    import fsim
    fsim.set_log_level('off')
    models=args.model
    fleet=models==['all']
    if models==['all']:
        models=sorted(p.name for p in (ROOT/'aircraft').iterdir()
                      if p.name.casefold()!='.codex' and (p/(p.name+'.toml')).is_file())
    report=dict(schema='fsim.vlm-sdk-validation.v1',sdk_executed=True,native_library=native_library(args.native_bin),
                coefficient_tolerance=.002,full_grid=args.full_grid,jobs=args.jobs if fleet else 1,models={})
    args.out.parent.mkdir(parents=True,exist_ok=True)
    started=time.monotonic()
    def execute(name):
        try:
            if fleet:
                # A native model-load exception must be recorded for that
                # model, rather than aborting the remaining roster.
                child=args.out.parent/(args.out.stem+'-'+name+'.json')
                log=child.with_suffix('.log')
                command=[sys.executable,'-u','-B',str(Path(__file__).resolve()),'--model',name,'--out',str(child)]
                if args.full_grid:command.append('--full-grid')
                if args.native_bin:command.extend(['--native-bin',str(args.native_bin)])
                with log.open('w',encoding='utf-8') as stream:
                    result=subprocess.run(command,stdout=stream,stderr=subprocess.STDOUT,timeout=300)
                if result.returncode not in (0,1) or not child.is_file():
                    raise RuntimeError('SDK child exit '+str(result.returncode)+'; see '+str(log))
                child_report=json.loads(child.read_text(encoding='utf-8'))
                if name not in child_report['models']:raise RuntimeError('Incomplete SDK child report')
                row=child_report['models'][name]
                if result.returncode and row['status']=='passed':raise RuntimeError('Nonzero SDK child despite a passing row')
            else:row=validate(name,args.full_grid)
        except Exception as error:row=dict(status='failed',reason=type(error).__name__+': '+str(error))
        return row
    def record(name,row):
        report['models'][name]=row
        report['models']=dict(sorted(report['models'].items()))
        report['elapsed_seconds']=time.monotonic()-started
        args.out.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8')
        print(json.dumps(dict(model=name,**row)),flush=True)
    if fleet and args.jobs>1:
        with ThreadPoolExecutor(max_workers=min(args.jobs,len(models))) as pool:
            pending={pool.submit(execute,name):name for name in models}
            for future in as_completed(pending):record(pending[future],future.result())
    else:
        for name in models:record(name,execute(name))
    failed=any(r['status']!='passed' for r in report['models'].values())
    report['status']='failed' if failed else 'passed'
    args.out.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    return int(failed)


if __name__=='__main__':raise SystemExit(main())
