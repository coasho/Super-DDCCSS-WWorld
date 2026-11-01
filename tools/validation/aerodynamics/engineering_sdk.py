"""Installed native force/performance sampling for every declared design.

All designs use the same nondimensional Mach/alpha/beta suite. This is native
model execution, not measured equipment accuracy. Only installed SDK assets are
opened by a worker. The independent reference oracle is test code, not physics.
"""
import argparse
import ctypes
import hashlib
import importlib.util
import itertools
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[3]
ALPHA=(-180.,-90.,-15.,0.,15.,90.,180.)
BETA=(-90.,-15.,0.,15.,90.)
ALTITUDE=(1500.,10000.)
MACH_FRACTIONS=(.2,.6,1.)
LIMIT=2e-3  # Existing Hangar native force/moment coefficient predicate.
DEADLINE=1200


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_text(encoding='utf-8'))
def write(path,value):Path(path).write_text(json.dumps(value,indent=2,allow_nan=False,default=float)+'\n',encoding='utf-8')


def declared_integration(receipt):
    integration=receipt['integration']
    method=integration.get('rotational_rate_method',1)
    dt=integration['maximum_internal_dt_s']
    if type(method) is not int or method not in (1,4):raise ValueError('Unsupported declared rotational integration method')
    if isinstance(dt,bool) or not isinstance(dt,(int,float)) or not math.isfinite(dt) or dt<=0:raise ValueError('Missing positive finite declared native timestep')
    return method,float(dt)


def observe_integration(vehicle,receipt):
    method,dt=declared_integration(receipt)
    actual=dict(rotational_rate_method=float(vehicle.get_property('simulation/integrator/rate/rotational')),
                internal_dt_s=float(vehicle.get_property('simulation/dt')))
    if actual['rotational_rate_method']!=method or not math.isfinite(actual['internal_dt_s']) or actual['internal_dt_s']<=0 or not math.isclose(actual['internal_dt_s'],dt,rel_tol=1.e-12,abs_tol=1.e-15):
        raise ValueError('Sampled native integrator/timestep differs from the generated receipt: '+str(actual))
    return actual


def restore_sampling_integration(vehicle,receipt):
    # The raw IC reset calls InitializeModels and restores Euler. RunIC seeds
    # every derivative history at dt0 independently of the selector, so restore
    # the declared method immediately, before any advancing step. This is not
    # a hot switch during flight, nor a change to the SDK reset contract.
    method,_=declared_integration(receipt)
    raw=float(vehicle.get_property('simulation/integrator/rate/rotational'))
    if not math.isfinite(raw):raise ValueError('Nonfinite raw-reset integration selector')
    vehicle.set_property('simulation/integrator/rate/rotational',float(method))
    return dict(raw_reset_rotational_rate_method=raw,before_step=observe_integration(vehicle,receipt))


def worker(args):
    import numpy as np
    started=time.monotonic();receipt=read(args.receipt);sdk=Path(args.prefix).resolve()
    data=sdk/'share/flightsim';folder=data/'aircraft'/receipt['product_directory']
    hashes={relative:sha(data/'aircraft'/relative) for relative in receipt['products']}
    if hashes!=receipt['products']:raise ValueError('Installed performance sweep refused stale generated assets')
    reference_path=Path(args.receipt).resolve().parent/receipt['reference_bundle']['name']
    if sha(reference_path)!=receipt['reference_bundle']['sha256']:raise ValueError('Stale test reference bundle')
    bundle=read(reference_path);tables=bundle['tables'];ref=bundle['reference']
    xml=ET.parse(folder/(folder.name+'.xml'));S,b,c=ref['S'],ref['b'],ref['c']
    source=ROOT/'tools/models/hangar/hangar/engineering_runtime.py'
    spec=importlib.util.spec_from_file_location('engineering_test_oracle',source)
    oracle=importlib.util.module_from_spec(spec);spec.loader.exec_module(oracle)
    tables=oracle.prepare_reference_tables(tables)
    sys.path.insert(0,str(sdk))
    os.environ['FSIM_AIRCRAFT_PATH']=str(data/'aircraft')+';'+str(data/'aircraft/stores')
    os.environ['FSIM_DLL_DIR']=str(sdk/'fsim/bin')
    installed_directory=os.add_dll_directory(str(sdk/'bin'))
    installed_library=ctypes.WinDLL(str(sdk/'bin/libfsim.dll'))
    import fsim
    fsim.set_log_level('off')
    kernel=ctypes.WinDLL('kernel32',use_last_error=True)
    kernel.GetModuleHandleW.argtypes=[ctypes.c_wchar_p];kernel.GetModuleHandleW.restype=ctypes.c_void_p
    kernel.GetModuleFileNameW.argtypes=[ctypes.c_void_p,ctypes.c_wchar_p,ctypes.c_uint];kernel.GetModuleFileNameW.restype=ctypes.c_uint
    buf=ctypes.create_unicode_buffer(32768);handle=kernel.GetModuleHandleW('libfsim.dll')
    if not handle or not kernel.GetModuleFileNameW(handle,buf,len(buf)):raise RuntimeError('No actual loaded DLL provenance')
    dll=Path(buf.value).resolve()
    if not dll.is_relative_to(sdk) or sha(dll)!=receipt['native_dll_sha256']:raise ValueError('Wrong native performance DLL')
    top=max(float(x) for x in tables['mach']['mach'])
    rows=[];failures=[]
    world=fsim.World('engineering-envelope',publish=False,workers=1,jsbsim_root=str(data/'jsbsim'))
    try:
        v=world.create_vehicle('subject',type='jsbsim:'+receipt['name'],latitude_deg=37.6,longitude_deg=-122.4,altitude_msl_m=1500,airspeed_ms=60)
        nodes={n.get('name') for n in xml.findall('.//function')}
        for altitude,fraction,alpha,beta in itertools.product(ALTITUDE,MACH_FRACTIONS,ALPHA,BETA):
            v.reset(latitude_deg=37.6,longitude_deg=-122.4,altitude_msl_m=altitude,airspeed_ms=60)
            v.set_protection('off');v.command_actuator(throttle=.5,gear_down=0)
            g=v.get_property
            speed=top*fraction*float(g('atmosphere/a-fps'))
            a,bb=math.radians(alpha),math.radians(beta)
            for axis,value in zip('uvw',(speed*math.cos(a)*math.cos(bb),speed*math.sin(bb),speed*math.sin(a)*math.cos(bb))):
                v.set_property('ic/'+axis+'-fps',value)
            v.set_property('simulation/reset',0.)
            integration=restore_sampling_integration(v,receipt)
            world.step(1)
            integration['after_step']=observe_integration(v,receipt)
            state={key:float(g(prop))*scale for key,prop,scale in [('alpha','aero/alpha-rad',1),('beta','aero/beta-rad',1),('mach','velocities/mach',1),
                ('V','velocities/vt-fps',oracle.FT),('altitude_m','position/h-sl-meters',1),('qbar','aero/qbar-psf',oracle.PSF),('hb','aero/h_b-mac-ft',1),
                ('p','velocities/p-aero-rad_sec',1),('q','velocities/q-aero-rad_sec',1),('r','velocities/r-aero-rad_sec',1),('adot','aero/alphadot-rad_sec',1)]}
            for rate,length in [('p',b),('q',c),('r',b),('adot',c)]:state[rate+'_hat']=state[rate]*length/(2*max(state['V'],1))
            ctl={ch:float(g({'elevator':'fcs/elevator-pos-deg','aileron':'fcs/left-aileron-pos-deg','rudder':'fcs/rudder-pos-deg','flap':'fcs/flap-pos-deg'}[ch])) for ch in tables['controls']}
            pieces={i:float(g(tab['property'])) for i,tab in enumerate(tables.get('native_controls',[]))}
            coeff=oracle.reference(tables,state,ctl,pieces)
            if bundle.get('gear_drag'):
                area,at=bundle['gear_drag'];gear=float(g('gear/gear-pos-norm'))
                coeff['CD']+=gear*area/S;coeff['Cm']+=gear*area/S*(at[2]-ref['arp'][2])/c
            F,M=oracle.forces(coeff,state['alpha'],state['beta'],state['qbar']*S,b,c)
            cg=np.array([g('inertia/cg-'+axis+'-in') for axis in 'xyz'])*.0254
            arm=(np.array(ref['arp'])-cg)*np.array([-1,1,-1]);M+=np.cross(arm,F)
            rotor={key:float(g('aero/propulsion/'+key)) if 'aero/propulsion/'+key in nodes else 0. for key in ('drag','side','lift')}
            rf,_=oracle.forces(dict(CD=rotor['drag'],CY=rotor['side'],CL=rotor['lift'],Cl=0,Cm=0,Cn=0),state['alpha'],state['beta'],1,b,c)
            F+=rf*oracle.LBF;M+=np.cross(arm,rf*oracle.LBF)
            for axis,index in [('ROLL',0),('PITCH',1),('YAW',2)]:
                for fn in xml.findall("aerodynamics/axis[@name='%s']/function"%axis):
                    name=fn.get('name','')
                    if name and not name.startswith('aero/coefficient/'):M[index]+=float(g(name))*oracle.LBF*oracle.FT
            nativeF=np.array([g('forces/fb'+axis+'-aero-lbs') for axis in 'xyz'])*oracle.LBF
            nativeM=np.array([g('moments/'+axis+'-aero-lbsft') for axis in 'lmn'])*oracle.LBF*oracle.FT
            qS=state['qbar']*S
            errors=np.r_[(F-nativeF)/qS,(M-nativeM)/(qS*np.array([b,c,b]))]
            finite=np.isfinite(list(state.values())).all() and np.isfinite(errors).all() and not v.state.diverged
            maximum=float(np.max(np.abs(errors))) if np.isfinite(errors).all() else None
            ok=bool(finite and maximum<=LIMIT)
            row=dict(request=dict(altitude_m=altitude,mach_fraction=fraction,mach=top*fraction,alpha_deg=alpha,beta_deg=beta),actual=state,
                native_integration=integration,
                controls_deg=ctl,physical_fins_rad=pieces,
                basis='simulated',force_n=nativeF.tolist(),moment_nm=nativeM.tolist(),mass_kg=float(g('inertia/weight-lbs'))*.45359237,cg_m=cg.tolist(),
                native_total_aero_body_coefficients=dict(CA=-nativeF[0]/qS,CY=nativeF[1]/qS,CN=-nativeF[2]/qS,Cl=nativeM[0]/qS/b,Cm=nativeM[1]/qS/c,Cn=nativeM[2]/qS/b),
                six_component_error=errors.tolist(),maximum_error=maximum,passed=ok)
            rows.append(row)
            if not ok:failures.append(len(rows)-1)
    finally:world.close()
    result=dict(schema='fsim.sim-m02.native-envelope.v1',validation_scope='native dynamic sampling; six-component consistency at realized states',name=receipt['name'],basis='simulated',case_count=len(rows),expected_cases=len(ALTITUDE)*len(MACH_FRACTIONS)*len(ALPHA)*len(BETA),
        source_sha256=sha(__file__),oracle_sha256=sha(source),loaded_dll=str(dll),loaded_dll_sha256=sha(dll),products=hashes,
        coefficient_limit=LIMIT,failures=failures,maximum_error=max((r['maximum_error'] for r in rows if r['maximum_error'] is not None),default=None),
        elapsed_seconds=time.monotonic()-started,rows=rows,
        request_realization_maximum_absolute_deltas=dict(
            altitude_m=max(abs(r['actual']['altitude_m']-r['request']['altitude_m']) for r in rows),
            mach=max(abs(r['actual']['mach']-r['request']['mach']) for r in rows),
            alpha_deg=max(abs((math.degrees(r['actual']['alpha'])-r['request']['alpha_deg']+180)%360-180) for r in rows),
            beta_deg=max(abs(math.degrees(r['actual']['beta'])-r['request']['beta_deg']) for r in rows)),
        limitation='210 requested conditions are sampling, not all table knots. Comparisons use realized states after one native step; request deltas are reported, not claimed as attained envelope coverage. No trimmed flight, high-angle equipment validity, measured top-speed/climb or long-duration stability claim.')
    write(args.output,result)
    return int(bool(failures))


def campaign(args):
    matrix=read(Path(args.matrix)/'matrix.json');out=Path(args.output).resolve();out.mkdir(parents=True,exist_ok=True)
    sdk=Path(args.prefix).resolve();environment=os.environ.copy()
    for key in ('PYTHONPATH','FSIM_DLL_DIR','FSIM_AIRCRAFT_PATH'):environment.pop(key,None)
    environment.update(TMP='D:/ENV/tmp',TEMP='D:/ENV/tmp',TORCH_EXTENSIONS_DIR='D:/ENV/tmp',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    declarations=[r['name'] for r in matrix['designs']]
    write(out/'batch-plan.json',dict(deadline_seconds=DEADLINE,designs=declarations,cases_per_design=210))
    records=[]
    for name in declarations:
        receipt=Path(args.matrix).resolve()/name/'generation.json'
        command=[sys.executable,'-I','-B',str(Path(__file__).resolve()),'worker','--prefix',str(sdk),'--receipt',str(receipt),'--output',str(out/(name+'.json'))]
        started=time.monotonic();record=dict(name=name,command=command,deadline_seconds=DEADLINE)
        try:
            with (out/(name+'.log')).open('w',encoding='utf-8') as log:
                process=subprocess.run(command,cwd='D:/ENV/tmp',env=environment,stdout=log,stderr=subprocess.STDOUT,timeout=DEADLINE)
            record['exit_code']=process.returncode
            if (out/(name+'.json')).is_file():
                r=read(out/(name+'.json'));record.update(case_count=r['case_count'],failures=r['failures'],maximum_error=r['maximum_error'])
        except subprocess.TimeoutExpired:record['timeout']=True
        except Exception as error:record['error']=str(error)
        record['elapsed_seconds']=time.monotonic()-started
        record['passed']=record.get('exit_code')==0 and record.get('case_count')==210 and record['elapsed_seconds']<=DEADLINE
        records.append(record);write(out/'summary.json',dict(declared_count=len(declarations),completed_count=len(records),designs=records))
        print(name,record['passed'],record.get('case_count'),record['elapsed_seconds'],flush=True)
    return int(not all(r['passed'] for r in records))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('mode',choices=['campaign','worker'])
    p.add_argument('--prefix',default='D:/ENV/sim-m02-engineering-20261030');p.add_argument('--matrix');p.add_argument('--receipt');p.add_argument('--output',required=True)
    args=p.parse_args();return worker(args) if args.mode=='worker' else campaign(args)


if __name__=='__main__':sys.exit(main())
