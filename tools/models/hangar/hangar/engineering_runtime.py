"""Standalone installed public-SDK consumer with a test-only reference bundle.

The product's interpolation reference is a test oracle, not a client simulator.
Run with Python -I from a directory outside the source/build trees.
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
import xml.etree.ElementTree as ET

import numpy as np

COEFFS = ('CD','CY','CL','Cl','Cm','Cn')
FT, LBF, PSF = .3048, 4.448222, 47.880259


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def interp2(a,b,data,x,y):
    return _interpolate2(data,_stencil2(a,b,x,y))


def _stencil2(a,b,x,y):
    x,y=np.clip(x,a[0],a[-1]),np.clip(y,b[0],b[-1])
    i=int(np.clip(np.searchsorted(a,x)-1,0,len(a)-2));j=int(np.clip(np.searchsorted(b,y)-1,0,len(b)-2))
    u,v=(x-a[i])/(a[i+1]-a[i]),(y-b[j])/(b[j+1]-b[j])
    return i,j,u,v


def _interpolate2(data,stencil):
    i,j,u,v=stencil
    return (1-u)*(1-v)*data[i][j]+u*(1-v)*data[i+1][j]+(1-u)*v*data[i][j+1]+u*v*data[i+1][j+1]


def prepare_reference_tables(value):
    """Reuse exact loaded float64 arrays; this changes storage, not values."""
    def number(x):return isinstance(x,float) or (type(x) is int and abs(x)<=2**53)
    if isinstance(value,dict):return {key:prepare_reference_tables(v) for key,v in value.items()}
    if isinstance(value,list):
        if value and all(number(x) for x in value):
            array=np.asarray(value,dtype=np.float64);array.setflags(write=False);return array
        if value and all(isinstance(x,list) and len(x)==len(value[0]) and all(number(y) for y in x) for x in value):
            array=np.asarray(value,dtype=np.float64);array.setflags(write=False);return array
        return [prepare_reference_tables(v) for v in value]
    return value


def reference(t, s, ctl, pieces):
    a,b=t['alpha'],t['beta'];al,be=map(math.degrees,(s['alpha'],s['beta']))
    mt=t['mach'];factors={};stencils={}
    def K(k):
        if k not in factors:factors[k]=float(np.interp(s['mach'],mt['mach'],mt[k]))
        return factors[k]
    def at(xgrid,ygrid,data,x,y):
        key=(id(xgrid),id(ygrid),float(x).hex(),float(y).hex())
        if key not in stencils:stencils[key]=_stencil2(xgrid,ygrid,x,y)
        return _interpolate2(data,stencils[key])
    out={k:at(a,b,t['base'][k],al,be) for k in COEFFS}
    induced=at(a,b,t['base']['CDi'],al,be)
    cl=out['CL'];lift=float(np.interp(s['hb'],t['ground_effect']['h_b'],t['ground_effect']['lift']))
    out['CL']*=K('K_L')*lift
    for k in ('CY','Cl','Cn'):out[k]*=K('K_Y')
    out['Cm']+=cl*K('dCm_dCL')
    for ch,tab in ({} if t.get('native_controls') else t['controls']).items():
        for k in COEFFS:
            out[k]+=at(a,tab['deflection'],tab[k],al,ctl[ch])*K('K_'+ch)*(lift if k=='CL' and ch=='flap' else 1)
        induced+=at(a,tab['deflection'],tab['CDi'],al,ctl[ch])*K('K_'+ch)
    for i,tab in enumerate(t.get('native_controls',[])):
        delta=float(np.clip(pieces[i],tab['lo'],tab['hi']))
        for k in COEFFS:out[k]+=at(a,tab['deflection'],tab['values'][k],al,delta)*K('K_'+tab['channel'])
        induced+=at(a,tab['deflection'],tab['values']['CDi'],al,delta)*K('K_'+tab['channel'])
    for rate in ('p','q','r'):
        for k in COEFFS:out[k]+=float(np.interp(al,a,t['rates'][rate][k]))*s[rate+'_hat']*K('K_L')
        induced+=float(np.interp(al,a,t['rates'][rate]['CDi']))*s[rate+'_hat']*K('K_L')
    for k in COEFFS:out[k]+=t['alphadot'][k]*s['adot_hat']*K('K_L')
    drag=float(np.interp(s['hb'],t['ground_effect']['h_b'],t['ground_effect']['drag']))
    out['CD']+=K('dCD0')+induced*(K('induced_factor')*drag-1)
    return out


def forces(c,al,be,qs,b,chord):
    ca,sa,cb,sb=math.cos(al),math.sin(al),math.cos(be),math.sin(be)
    x=np.array([ca*cb,sb,sa*cb]); y=np.array([-ca*sb,cb,-sa*sb]); z=np.array([-sa,0,ca])
    return qs*(-c['CD']*x+c['CY']*y-c['CL']*z),qs*np.array([c['Cl']*b,c['Cm']*chord,c['Cn']*b])


def main():
    p=argparse.ArgumentParser();p.add_argument('--sdk',required=True);p.add_argument('--data',required=True)
    p.add_argument('--receipt',required=True);p.add_argument('--output',required=True)
    p.add_argument('--sampling-output');args=p.parse_args()
    start=time.monotonic();sdk=Path(args.sdk).resolve();data=Path(args.data).resolve()
    receipt=json.loads(Path(args.receipt).read_text());checks=[]
    def check(name,ok,details=None):checks.append(dict(name=name,passed=bool(ok),details=details))
    actual={}
    for relative,expected in receipt['products'].items():
        path=data/'aircraft'/relative
        found=sha(path) if path.is_file() else None
        actual[relative]=found;check('fresh product/'+relative,found==expected)
    if not all(c['passed'] for c in checks):
        raise ValueError('Missing or stale installed product; refuse source fallback')
    folder=data/'aircraft'/receipt['product_directory']
    card_info=receipt.get('store_card')
    if card_info:
        installed_card=data/'stores'/(card_info['id']+'.json')
        check('fresh equipment card',installed_card.is_file() and sha(installed_card)==card_info['sha256'])
    reference_path=Path(args.receipt).resolve().parent/receipt['reference_bundle']['name']
    check('fresh test reference bundle',sha(reference_path)==receipt['reference_bundle']['sha256'])
    bundle=json.loads(reference_path.read_text(encoding='utf-8'))
    t=prepare_reference_tables(bundle['tables']);ref=bundle['reference'];S,b,chord=ref['S'],ref['b'],ref['c']
    xml=ET.parse(folder/(folder.name+'.xml'))
    sys.path.insert(0,str(sdk))
    # Load the installed SDK DLL explicitly so its own share/flightsim tree is
    # authoritative even when Python is a separate installed component.
    installed_dll=sdk/'bin/libfsim.dll'
    installed_directories=[]
    if installed_dll.is_file():
        installed_directories.append(os.add_dll_directory(str(installed_dll.parent)))
        installed_library=ctypes.WinDLL(str(installed_dll))
    import fsim
    fsim.set_log_level('off')
    check('isolated public Python import',Path(fsim.__file__).resolve().is_relative_to(sdk),fsim.__file__)
    kernel=ctypes.WinDLL('kernel32',use_last_error=True)
    kernel.GetModuleHandleW.argtypes=[ctypes.c_wchar_p];kernel.GetModuleHandleW.restype=ctypes.c_void_p
    kernel.GetModuleFileNameW.argtypes=[ctypes.c_void_p,ctypes.c_wchar_p,ctypes.c_uint];kernel.GetModuleFileNameW.restype=ctypes.c_uint
    buf=ctypes.create_unicode_buffer(32768);handle=kernel.GetModuleHandleW('libfsim.dll')
    if not handle or not kernel.GetModuleFileNameW(handle,buf,len(buf)):raise RuntimeError('Cannot establish loaded native DLL provenance')
    dll=Path(buf.value).resolve();check('isolated loaded DLL',dll.is_relative_to(sdk),str(dll))
    check('current built DLL identity',sha(dll)==receipt['native_dll_sha256'],sha(dll))
    # Before the first successful load, poison this exact installed XML. A
    # consumer that silently falls back to any source/bundled copy must fail.
    target=folder/(folder.name+'.xml');original_xml=target.read_bytes()
    refused=False
    try:
        target.write_text('<not-a-valid-jsbsim-model/>',encoding='utf-8')
        invalid=fsim.World('engineering-poison',publish=False,workers=1,jsbsim_root=str(data/'jsbsim'))
        try:
            invalid.create_vehicle('invalid',type='jsbsim:'+receipt['name'],altitude_msl_m=1500,airspeed_ms=60)
        except fsim.Error:
            refused=True
        finally:invalid.close()
    finally:target.write_bytes(original_xml)
    check('poisoned exact runtime product refused without fallback',refused)
    check('poisoned product restored exactly',sha(target)==hashlib.sha256(original_xml).hexdigest())
    traces=[]
    prop_nodes={node.get('name') for node in xml.findall('.//function')}
    native_fins=t.get('native_controls',[])
    # Common physical initial conditions and command changes for all designs.
    # Dynamic safety/accuracy at every Mach/angle knot is not inferred from it.
    for case,(speed,command) in enumerate([(60.,0.),(90.,.2),(120.,-.2)]):
        world=fsim.World('engineering-consumer',publish=False,workers=1,jsbsim_root=str(data/'jsbsim'))
        try:
            v=world.create_vehicle('subject',type='jsbsim:'+receipt['name'],latitude_deg=37.6,longitude_deg=-122.4,
                                   altitude_msl_m=1500,heading_deg=0,airspeed_ms=speed)
            expected_integrator=receipt.get('integration',{}).get('rotational_rate_method',1)
            check('declared rotational-rate integrator/'+str(case),v.get_property('simulation/integrator/rate/rotational')==expected_integrator)
            v.set_protection('off')
            if xml.findall('propulsion/engine'):v.set_property('propulsion/set-running',-1.)
            v.command_actuator(aileron=command,elevator=-command,rudder=command,throttle=1.,gear_down=0)
            rows=[]
            for sample in range(4):
                world.step(30)
                g=v.get_property
                s={key:float(g(prop))*scale for key,prop,scale in [('alpha','aero/alpha-rad',1),('beta','aero/beta-rad',1),
                    ('mach','velocities/mach',1),('V','velocities/vt-fps',FT),('qbar','aero/qbar-psf',PSF),('hb','aero/h_b-mac-ft',1),
                    ('mass_kg','inertia/weight-lbs',.45359237),('p','velocities/p-aero-rad_sec',1),('q','velocities/q-aero-rad_sec',1),
                    ('r','velocities/r-aero-rad_sec',1),('adot','aero/alphadot-rad_sec',1)]}
                for rate,length in [('p',b),('r',b),('q',chord),('adot',chord)]:s[rate+'_hat']=s[rate]*length/(2*max(s['V'],1))
                ctl={ch:float(g({'elevator':'fcs/elevator-pos-deg','aileron':'fcs/left-aileron-pos-deg','rudder':'fcs/rudder-pos-deg','flap':'fcs/flap-pos-deg'}[ch])) for ch in t['controls']}
                pieces={i:float(g(tab['property'])) for i,tab in enumerate(native_fins)}
                c=reference(t,s,ctl,pieces)
                # Gear is commanded retracted; account for any transit with the
                # generated native gear terms rather than dropping the row.
                gear=bundle.get('gear_drag')
                if gear:
                    fraction=float(g('gear/gear-pos-norm'))
                    c['CD']+=fraction*gear[0]/S
                    c['Cm']+=fraction*gear[0]/S*(gear[1][2]-ref['arp'][2])/chord
                F,M=forces(c,s['alpha'],s['beta'],s['qbar']*S,b,chord)
                nativeF=np.array([g('forces/fb'+axis+'-aero-lbs') for axis in 'xyz'])*LBF
                nativeM=np.array([g('moments/'+axis+'-aero-lbsft') for axis in 'lmn'])*LBF*FT
                cg=np.array([g('inertia/cg-'+axis+'-in') for axis in 'xyz'])*.0254
                arm=(np.array(ref['arp'])-cg)*np.array([-1,1,-1]);M+=np.cross(arm,F)
                # Rotor propulsion written into aerodynamic axes is an exact
                # native load contribution, not an aerodynamic tolerance waiver.
                rotor={k:float(g('aero/propulsion/'+k)) if 'aero/propulsion/'+k in prop_nodes else 0. for k in ('drag','side','lift')}
                rf,rm=forces(dict(CD=rotor['drag'],CY=rotor['side'],CL=rotor['lift'],Cl=0,Cm=0,Cn=0),s['alpha'],s['beta'],1,b,chord)
                F+=rf*LBF;M+=np.cross(arm,rf*LBF)
                physical_moments={}
                for axis,index in [('ROLL',0),('PITCH',1),('YAW',2)]:
                    for function in xml.findall("aerodynamics/axis[@name='%s']/function"%axis):
                        name=function.get('name','')
                        if name and not name.startswith('aero/coefficient/'):
                            value=float(g(name))*LBF*FT
                            M[index]+=value;physical_moments[name]=value
                qs=s['qbar']*S
                errorF=(F-nativeF)/qs;errorM=(M-nativeM)/(qs*np.array([b,chord,b]))
                inertia=[float(g('inertia/'+key+'-slugs_ft2')) for key in ('ixx','iyy','izz')]
                native_state=v.state
                check('unchanged native outer clock/'+str((case,sample)),math.isclose(float(native_state.sim_time),(sample+1)*30*world.step_seconds,rel_tol=0,abs_tol=1e-9))
                rigid_body={field:[float(x) for x in getattr(native_state,field)] for field in ('position_ecef','velocity_ned_ms','euler_rad','angular_rate_body_rad_s')}
                check('finite 6DOF propagation/'+str((case,sample)),all(math.isfinite(x) for values in rigid_body.values() for x in values) and not native_state.diverged,
                      {'diverged':bool(native_state.diverged),'crashed':bool(native_state.crashed)})
                propulsion=[]
                for index,engine in enumerate(xml.findall('propulsion/engine')):
                    prop=folder/'Engines'/(engine.get('file')+'.xml')
                    kind=ET.parse(prop).getroot().tag
                    item=dict(index=index,kind=kind,thrust_n=float(g('propulsion/engine[%d]/thrust-lbs'%index))*LBF)
                    if kind in ('rocket_engine','turbine_engine','turboprop_engine','piston_engine'):
                        item['native_fuel_flow_demand_kg_s']=float(g('propulsion/engine[%d]/fuel-flow-rate-pps'%index))*.45359237
                        item['fuel_flow_scope']='Raw native engine demand; may retain its last value after starvation; not a tank-depletion ledger'
                    propulsion.append(item)
                check('finite propulsion trace/'+str((case,sample)),all(math.isfinite(val) for eng in propulsion for val in eng.values() if isinstance(val,(int,float))))
                trace=dict(sample=sample,state=s,rigid_body=rigid_body,controls_deg=ctl,physical_fins_rad=pieces,inertia_slug_ft2=inertia,cg_m=cg.tolist(),
                    coefficients=c,force_native_n=nativeF.tolist(),moment_native_nm=nativeM.tolist(),
                    force_coefficient_error=errorF.tolist(),moment_coefficient_error=errorM.tolist(),
                    propulsion=propulsion,physical_moments_nm=physical_moments,instantaneous_drag_power_w=-float(np.dot(nativeF,np.array([math.cos(s['alpha'])*math.cos(s['beta']),math.sin(s['beta']),math.sin(s['alpha'])*math.cos(s['beta'])])))*s['V'])
                check('finite rigid-body/performance trace/'+str((case,sample)),all(math.isfinite(x) for x in list(s.values())+list(ctl.values())+list(pieces.values())+inertia+cg.tolist()+nativeF.tolist()+nativeM.tolist()))
                check('positive mass and inertia/'+str((case,sample)),s['mass_kg']>0 and min(inertia)>0)
                check('six-component native table agreement/'+str((case,sample)),max(np.max(np.abs(errorF)),np.max(np.abs(errorM)))<=2e-3,
                      {'maximum':float(max(np.max(np.abs(errorF)),np.max(np.abs(errorM)))),'existing_coefficient_limit':2e-3})
                rows.append(trace)
            # Vehicle.reset preserves activities/overrides and is not the full
            # episode-reset contract. Keep this comparison as diagnostic data.
            v.reset(latitude_deg=37.6,longitude_deg=-122.4,altitude_msl_m=1500,heading_deg=0,airspeed_ms=speed)
            check('reset restores declared rotational-rate integrator/'+str(case),v.get_property('simulation/integrator/rate/rotational')==expected_integrator)
            v.set_protection('off')
            if xml.findall('propulsion/engine'):v.set_property('propulsion/set-running',-1.)
            v.command_actuator(aileron=command,elevator=-command,rudder=command,throttle=1.,gear_down=0)
            world.step(30)
            again=[v.get_property('velocities/vt-fps')*FT,v.get_property('inertia/weight-lbs')*.45359237]
            original=[rows[0]['state']['V'],rows[0]['state']['mass_kg']]
            vehicle_reset=dict(initial=original,preserved_state_reset=again)
            # Reconstruction repeatability smoke check, observing speed/mass.
            # This does not prove equality of every native subsystem state.
            world.close()
            world=fsim.World('engineering-consumer',publish=False,workers=1,jsbsim_root=str(data/'jsbsim'))
            v=world.create_vehicle('subject',type='jsbsim:'+receipt['name'],latitude_deg=37.6,longitude_deg=-122.4,
                altitude_msl_m=1500,heading_deg=0,airspeed_ms=speed)
            v.set_protection('off')
            if xml.findall('propulsion/engine'):v.set_property('propulsion/set-running',-1.)
            v.command_actuator(aileron=command,elevator=-command,rudder=command,throttle=1.,gear_down=0)
            world.step(30)
            full=[v.get_property('velocities/vt-fps')*FT,v.get_property('inertia/weight-lbs')*.45359237]
            check('World reconstruction speed/mass repeatability/'+str(case),np.allclose(full,original,rtol=0,atol=1e-9),{'initial':original,'reconstructed':full})
            traces.append(dict(case=case,speed_ms=speed,command=command,rows=rows,vehicle_reset_diagnostic=vehicle_reset))
        finally:world.close()
    release=None
    if card_info:
        carrier=card_info['carrier']
        check('required release fixture exists',bool(carrier))
        if carrier:
            world=fsim.World('engineering-release',publish=False,workers=1,jsbsim_root=str(data/'jsbsim'))
            try:
                aircraft=world.create_vehicle('carrier',type='jsbsim:'+carrier[0],latitude_deg=37.6,longitude_deg=-122.4,altitude_msl_m=1500,airspeed_ms=150)
                world.step();aircraft.load_store(carrier[1],card_info['id']);world.step(2)
                before=[dict(station=x.station,mass_kg=x.mass_kg,status=str(x.status),weapon=x.weapon) for x in aircraft.stores]
                aircraft.jettison(emergency=True);world.step(120)
                after=[dict(station=x.station,mass_kg=x.mass_kg,status=str(x.status),weapon=x.weapon) for x in aircraft.stores]
                body=next((x for x in after if x['station']==carrier[1] and x['weapon']),None)
                check('native store released as an independent body',any(x['station']==carrier[1] for x in before) and bool(body))
                released_mass=world.vehicle(body['weapon']).mass_kg if body else None
                check('released body has positive finite mass',released_mass is not None and math.isfinite(released_mass) and released_mass>0)
                release=dict(carrier=carrier,before=before,after=after,released_mass_kg=released_mass)
            finally:world.close()
    output=dict(schema='fsim.sim-m02.engineering-runtime.v1',name=receipt['name'],sdk=str(sdk),data=str(data),release=release,
        python_module=fsim.__file__,loaded_dll=str(dll),loaded_dll_sha256=sha(dll),products=actual,
        checks=checks,traces=traces,elapsed_seconds=time.monotonic()-start,
        limitation='Short untrimmed native flights and instantaneous drag power; not measured top-speed/climb/trim accuracy or full-envelope dynamic stability.')
    Path(args.output).write_text(json.dumps(output,indent=2,allow_nan=False,default=float)+'\n',encoding='utf-8')
    if args.sampling_output:
        import importlib.util
        from types import SimpleNamespace
        source=Path(__file__).resolve().parents[3]/'validation/aerodynamics/engineering_sdk.py'
        spec=importlib.util.spec_from_file_location('engineering_native_sampling',source)
        sampling=importlib.util.module_from_spec(spec);spec.loader.exec_module(sampling)
        return sampling.worker(SimpleNamespace(receipt=args.receipt,prefix=args.sdk,output=args.sampling_output))
    return 0


if __name__=='__main__':sys.exit(main())
