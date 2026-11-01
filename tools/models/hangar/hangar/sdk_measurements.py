"""Controlled measurements of generated products through the public SDK.

Fixtures copy declared XML dependencies only. Candidate parameters are applied
to the shared writer in memory; iterations do not rewrite production assets.
"""
import ctypes
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np

from . import airframe, jsbsim, pipeline
from .reference_campaign import sha

FT=.3048
LBF=4.4482216152605
PSF=LBF/FT**2
KT=.5144444444444445


def reference_altitude(flight, vehicle, conditions):
    """Resolve a pressure-altitude test condition using the SDK atmosphere.

    Off-standard temperature changes the relation between geometric and
    pressure altitude. Do not compare a POH pressure-altitude row at the same
    numerical geometric altitude. This only positions the test, not its forces.
    """
    geometric=conditions['altitude_m']
    target=conditions.get('pressure_altitude_m')
    for _ in range(8 if target is not None else 1):
        vehicle.set_property('ic/h-sl-ft',geometric/FT)
        vehicle.set_property('simulation/reset',0.)
        flight.world.set_environment(temperature_sl_k=conditions['temperature_c']+.0065*geometric+273.15,
                                     pressure_sl_pa=101325.,wind_speed_ms=0.,turbulence=0.)
        flight.world.step(1)
        if target is None:break
        difference=target-vehicle.get_property('atmosphere/pressure-altitude')*FT
        if abs(difference)<.01:break
        geometric+=difference
    return geometric


def native_library():
    kernel=ctypes.WinDLL('kernel32',use_last_error=True)
    kernel.GetModuleHandleW.argtypes=[ctypes.c_wchar_p];kernel.GetModuleHandleW.restype=ctypes.c_void_p
    kernel.GetModuleFileNameW.argtypes=[ctypes.c_void_p,ctypes.c_wchar_p,ctypes.c_uint]
    for name in ('libfsim.dll','fsim.dll'):
        handle=kernel.GetModuleHandleW(name)
        if handle:
            buffer=ctypes.create_unicode_buffer(32768)
            if kernel.GetModuleFileNameW(handle,buffer,len(buffer)):
                return dict(path=buffer.value,sha256=sha(buffer.value))
    raise RuntimeError('Loaded SDK library identity is unavailable')


class Measurements:
    def __init__(self,name,scratch):
        self.name=name;self.root=Path(pipeline.repo_root());self.scratch=Path(scratch).resolve()
        self.source=Path(pipeline.find_design(name));self.design=airframe.design(self.source)
        self.kind='store' if self.design.store else self.design.aircraft.category
        self.product=self.root/'aircraft'/('stores/'+name if self.design.store else name)/(name+'.xml')
        self.original=self.product.read_text(encoding='utf-8')
        self.initial=dict(self.design.aircraft.calibration)
        self.last=[]

    def fixture(self,parameters,neutral=True,mass_kg=None,rotor_rpm=None,
                geometry_scale=1.,reference_reynolds=None,reference_length_m=None):
        import copy
        design_values={k.removeprefix('design:'):v for k,v in parameters.items() if k.startswith('design:')}
        force_parameters={k:v for k,v in parameters.items() if not k.startswith('design:')}
        scaled=geometry_scale!=1.
        if scaled:
            if design_values:raise ValueError('Scaled fixtures cannot also fit physical design dimensions')
            from .store_tunnel import fixture
            root=fixture(self,force_parameters,geometry_scale,reference_reynolds,reference_length_m)
            dependencies=self.product.parent/'Engines'
        elif design_values:
            spec=copy.deepcopy(self.design.aircraft.spec)
            for path,value in design_values.items():
                fields=path.split('.');node=spec
                for part in fields[:-1]:node=node[part]
                node[fields[-1]]=value
            # The normal writer evaluates the physical coefficient; this is
            # neither a substitute force model nor a per-equipment formula.
            from .rotorcraft import multi,heli
            writer={'multirotor':multi,'helicopter':heli}.get(self.kind)
            if writer is None:
                raise ValueError('Physical design bindings currently require a rotor propulsion writer')
            folder=self.scratch/'candidate';folder.mkdir(parents=True,exist_ok=True)
            (folder/(self.name+'.toml')).write_text(self.source.read_text(encoding='utf-8'),encoding='utf-8')
            (folder/'calibration.toml').write_text('[calibration]\n'+''.join(
                k+' = '+json.dumps(v,allow_nan=False)+'\n' for k,v in force_parameters.items()),encoding='utf-8')
            out=folder/'out';out.mkdir(exist_ok=True)
            cache=self.source.parent/'out/tables.pkl'
            if cache.is_file():shutil.copyfile(cache,out/'tables.pkl')
            writer.write(spec,str(folder))
            root=ET.fromstring((folder/(self.name+'.xml')).read_text(encoding='utf-8'))
            dependencies=folder/'Engines'
        else:
            root=ET.fromstring(self.original)
            dependencies=self.product.parent/'Engines'
        if not scaled and not design_values and (force_parameters!=self.initial or self.kind not in ('helicopter','multirotor')):
            self.design.aircraft.calibration=force_parameters
            aero=ET.fromstring(jsbsim.aerodynamics_xml(self.design.calibrated(self.design.tables()),self.design.aircraft))
            root.remove(root.find('aerodynamics'));root.append(aero)
        if rotor_rpm is not None:
            # Virtual bench: impose the measured shaft RPM, while retaining
            # the generated SDK thrust/torque equations and engine resources.
            for channel in root.findall('./flight_control/channel'):
                for node in list(channel):
                    if node.tag=='lag_filter' and '/omega[' in node.get('name',''):
                        index=list(channel).index(node);replacement=ET.Element('fcs_function',name=node.get('name'))
                        ET.SubElement(ET.SubElement(replacement,'function'),'value').text=str(rotor_rpm*2*math.pi/60)
                        channel.remove(node);channel.insert(index,replacement)
        if neutral:
            for node in root.findall('flight_control'):root.remove(node)
            fcs=ET.SubElement(root,'flight_control',name='Neutral reference controls')
            declared=set()
            for node in root.findall('./aerodynamics//property')+root.findall('./aerodynamics//independentVar'):
                prop=(node.text or '').strip().removeprefix('-')
                if prop.startswith('fcs/') and prop not in declared:
                    ET.SubElement(fcs,'property',value='0').text=prop;declared.add(prop)
        if mass_kg is not None:
            balance=root.find('mass_balance')
            weights=[balance.find('emptywt')]+balance.findall('pointmass/weight')
            factors={'KG':1.,'LBS':.45359237}
            dry=sum(float(n.text)*factors[n.get('unit','LBS')] for n in weights)
            tanks=root.findall('propulsion/tank')
            capacities=[float(t.findtext('capacity'))*factors[t.find('capacity').get('unit','LBS')] for t in tanks]
            fuel=mass_kg-dry
            if not 0<=fuel<=sum(capacities):raise ValueError('Reference weight cannot be represented by authored payload and fuel')
            for tank,capacity in zip(tanks,capacities):
                tank.find('contents').set('unit','KG');tank.find('contents').text=str(fuel*capacity/sum(capacities))
        name='reference-'+self.name
        folder=self.scratch/'aircraft'/name;folder.mkdir(parents=True,exist_ok=True)
        for directory in ('engine','systems'):(self.scratch/directory).mkdir(exist_ok=True)
        for node in root.findall('./propulsion/engine')+root.findall('./propulsion/engine/thruster'):
            filename=node.get('file')
            if not filename:continue
            candidates=[dependencies/(filename+'.xml'),self.root/'third_party/jsbsim/engine'/(filename+'.xml')]
            source=next((p for p in candidates if p.is_file()),None)
            if source is None:raise ValueError('Missing XML dependency: '+filename)
            destination=folder/'Engines'/source.name;destination.parent.mkdir(exist_ok=True);shutil.copyfile(source,destination)
        for node in root.findall('system')+root.findall('autopilot')+root.findall('flight_control'):
            filename=node.get('file')
            if not filename:continue
            candidates=[self.product.parent/'Systems'/(filename+'.xml'),self.root/'third_party/jsbsim/systems'/(filename+'.xml')]
            source=next((p for p in candidates if p.is_file()),None)
            if source is None:raise ValueError('Missing system dependency: '+filename)
            destination=self.scratch/'systems'/source.name;destination.parent.mkdir(exist_ok=True);shutil.copyfile(source,destination)
        path=folder/(name+'.xml');ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)
        self.fixture_sha256=sha(path)
        return name,root

    def static(self,parameters,cases,neutral=True):
        import fsim
        rotor_rpm=cases[0]['conditions'].get('rotor_rpm')
        c0=cases[0]['conditions']
        name,xml=self.fixture(parameters,neutral if rotor_rpm is None else False,rotor_rpm=rotor_rpm,
                              geometry_scale=c0.get('geometry_scale',1.),reference_reynolds=c0.get('reynolds_number'),
                              reference_length_m=c0.get('reference_chord_m'))
        world=fsim.World('reference-'+self.name,dt=1.e-5,frame_skip=1,publish=False,workers=1,
                         pin_workers=False,terrain=False,seed=907,jsbsim_root=str(self.scratch))
        rows=[]
        try:
            v=world.create_vehicle('probe','jsbsim:'+name,latitude_deg=0.,longitude_deg=0.,altitude_msl_m=1000.,airspeed_ms=100.)
            v.set_protection('off');g=v.get_property
            for case in cases:
                c=case['conditions'];alpha,beta=map(math.radians,(c['alpha_deg'],c['beta_deg']))
                v.set_property('ic/h-sl-ft',c['altitude_m']/FT)
                v.set_property('simulation/reset',0.);world.step(1)
                if 'density_kg_m3' in c:
                    # q and V specify tunnel density. A declared temperature
                    # closes the ideal-gas atmosphere; it is not a measurement.
                    temperature=c['temperature_k']
                    world.set_environment(temperature_sl_k=temperature,
                        pressure_sl_pa=c['density_kg_m3']*287.05287*temperature,
                        wind_speed_ms=0.,turbulence=0.)
                    world.step(1)
                if rotor_rpm is not None:
                    v.set_property('propulsion/set-running',-1.);world.step(2)
                speed=math.sqrt(2*c['qbar_pa']/PSF/g('atmosphere/rho-slugs_ft3')) if 'qbar_pa' in c else c['mach']*g('atmosphere/a-fps')
                if 'density_kg_m3' in c and 'qbar_pa' in c:
                    speed=math.sqrt(2*c['qbar_pa']/c['density_kg_m3'])/FT
                if case.get('metric') == 'CD0':
                    # A public zero-lift polar intercept is not CD at alpha=0.
                    # Find actual zero wind-axis lift using SDK force samples.
                    lo,hi=math.radians(-15.),math.radians(15.)
                    for _ in range(18):
                        alpha=(lo+hi)/2
                        for key,value in zip('uvw',(speed*math.cos(alpha),0.,speed*math.sin(alpha))):
                            v.set_property('ic/'+key+'-fps',value)
                        v.set_property('simulation/reset',0.);world.step(1)
                        actual_alpha=math.radians(g('aero/alpha-deg'))
                        lift=g('forces/fbx-aero-lbs')*math.sin(actual_alpha)-g('forces/fbz-aero-lbs')*math.cos(actual_alpha)
                        if lift>0:hi=alpha
                        else:lo=alpha
                for key,value in zip('uvw',(speed*math.cos(alpha)*math.cos(beta),speed*math.sin(beta),speed*math.sin(alpha)*math.cos(beta))):
                    v.set_property('ic/'+key+'-fps',value)
                for key in 'pqr':v.set_property('ic/'+key+'-rad_sec',0.)
                v.set_property('simulation/reset',0.)
                if 'density_kg_m3' in c:
                    world.set_environment(temperature_sl_k=c['temperature_k'],
                        pressure_sl_pa=c['density_kg_m3']*287.05287*c['temperature_k'],
                        wind_speed_ms=0.,turbulence=0.)
                world.step(1)
                qbar=g('aero/qbar-psf')*PSF
                if qbar<=0:raise ValueError('Nonpositive measured dynamic pressure')
                force=np.array([g('forces/fb'+a+'-aero-lbs') for a in 'xyz'])*LBF
                moment=np.array([g('moments/'+a+'-aero-lbsft') for a in 'lmn'])*LBF*FT
                location=xml.find("metrics/location[@name='AERORP']")
                scale={'M':1.,'IN':.0254,'FT':FT}[location.get('unit','IN')]
                arp=np.array([float(location.findtext(a)) for a in 'xyz'])*scale
                if 'moment_ref_x_m' in c:
                    arp=np.array([c['moment_ref_'+a+'_m'] for a in 'xyz'])
                cg=np.array([g('inertia/cg-'+a+'-in') for a in 'xyz'])*.0254
                moment-=np.cross((arp-cg)*np.array([-1.,1.,-1.]),force)
                a,b=map(math.radians,(g('aero/alpha-deg'),g('aero/beta-deg')))
                area=c.get('reference_area_m2',g('metrics/Sw-sqft')*FT**2)
                span=c.get('reference_span_m',g('metrics/bw-ft')*FT)
                chord=c.get('reference_chord_m',g('metrics/cbarw-ft')*FT)
                drag=-float(force@np.array([math.cos(a)*math.cos(b),math.sin(b),math.sin(a)*math.cos(b)]))/qbar
                lift=float(force@np.array([math.sin(a),0.,-math.cos(a)]))/qbar
                values=dict(drag_area_m2=drag,CD0=drag/area,CD=drag/area,CL=lift/area,CX=force[0]/qbar/area,CY=force[1]/qbar/area,CZ=force[2]/qbar/area,
                            Cl=moment[0]/qbar/area/span,Cm=moment[1]/qbar/area/chord,Cn=moment[2]/qbar/area/span)
                if rotor_rpm is not None:
                    values['static_thrust_n']=sum(g(f'propulsion/engine[{i}]/thrust-lbs')*LBF for i in range(len(xml.findall('./propulsion/engine'))))
                if not all(math.isfinite(x) for x in values.values()):raise ValueError('Nonfinite SDK loads')
                rows.append(dict(values=values,drag_area_m2=drag,actual=dict(alpha_deg=g('aero/alpha-deg'),
                    measurement_fixture_xml_sha256=self.fixture_sha256,
                    geometry_scale=c.get('geometry_scale',1.),solver_reynolds=c.get('reynolds_number'),
                    beta_deg=g('aero/beta-deg'),mach=g('velocities/mach'),altitude_m=g('position/h-sl-ft')*FT,
                    CL=values['CL'],qbar_pa=qbar,density_kg_m3=g('atmosphere/rho-slugs_ft3')*14.5939029372/FT**3,
                    temperature_c=g('atmosphere/T-R')/1.8-273.15)))
        finally:world.close()
        return rows

    def climb(self,parameters,case):
        from . import flight as F
        c=case['conditions'];name,_=self.fixture(parameters,neutral=False,mass_kg=c['mass_kg'])
        f=F.Flight(name,jsbsim_root=str(self.scratch),lat=0.,lon=0.)
        try:
            v=f.spawn(c['altitude_m'],c['airspeed_kcas']*KT)
            target_alt=reference_altitude(f,v,c)
            initial_alt=max(100.,target_alt-case['value']*FT/60*45.)
            guess=c['airspeed_kcas']*KT/(max(.2,1-2.25577e-5*initial_alt)**2.12794)
            v.set_property('ic/h-sl-ft',initial_alt/FT)
            v.set_property('ic/vt-fps',guess/FT)
            v.set_property('simulation/reset',0.)
            f.world.set_environment(temperature_sl_k=c['temperature_c']+.0065*target_alt+273.15,
                                    pressure_sl_pa=101325.,wind_speed_ms=0.,turbulence=0.)
            f.world.step(1)
            desired_tas=guess*c['airspeed_kcas']/v.get_property('velocities/vc-kts')
            v.set_property('ic/vt-fps',desired_tas/FT);v.set_property('simulation/reset',0.);f.world.step(1)
            f.world.set_environment(temperature_sl_k=c['temperature_c']+.0065*target_alt+273.15,
                                    pressure_sl_pa=101325.,wind_speed_ms=0.,turbulence=0.)
            f.world.step(1)
            # JSBSim reset also resets propulsion. Restore the airborne-start
            # contract before trimming instead of measuring an engine-out glide.
            v.set_property('propulsion/set-running',-1.)
            v.set_property('fcs/mixture-cmd-norm',1.)
            tr=f.trim(v)
            ap=F.Autopilot(f.dt,tr['pitch_deg'],throttle=1.)
            ap.i_outer=tr['pitch_deg']
            samples=[]
            def control(t,s,vehicle):
                cas=vehicle.get_property('velocities/vc-kts')
                target=s.airspeed_true_ms*c['airspeed_kcas']/max(cas,1.)
                ap.command(vehicle,s,ap.airspeed(s,target))
                if t>=40.:
                    samples.append((-s.velocity_ned_ms[2],cas,s.altitude_msl_m,
                                    vehicle.get_property('inertia/weight-lbs')*.45359237,
                                    vehicle.get_property('atmosphere/T-R')/1.8-273.15,
                                    vehicle.get_property('atmosphere/pressure-altitude')*FT,
                                    vehicle.get_property('fcs/mixture-cmd-norm[0]')))
            history=f.run(v,60.,control)
            mean=np.mean(samples,axis=0)
            if not np.isfinite(mean).all():raise ValueError('Climb run produced nonfinite values')
            if abs(mean[1]-c['airspeed_kcas'])>3.:raise ValueError('Climb speed did not track the reference condition')
            actual=dict(airspeed_kcas=float(mean[1]),altitude_m=float(mean[2]),mass_kg=float(mean[3]),temperature_c=float(mean[4]),
                        measurement_fixture_xml_sha256=self.fixture_sha256,
                        pressure_altitude_m=float(mean[5]),mixture_command=float(mean[6]),
                        mixture_control='Authored automatic pressure schedule; not manual peak-RPM optimization',
                        elapsed_seconds=float(history['t'][-1]),settled_speed=True)
            return float(mean[0])*60/FT,actual
        finally:f.close()

    def measure(self,parameters,cases):
        if not cases:return []
        self.last=[];values=[]
        if all(c['metric']=='static_thrust_n' for c in cases):
            for case in cases:
                row=self.static(parameters,[case],neutral=False)[0]
                values.append(row['values']['static_thrust_n']);self.last.append(dict(row['actual'],imposed_rotor_rpm=case['conditions']['rotor_rpm']))
            return values
        if all(c['metric']!='climb_rate_fpm' for c in cases):
            if any(c['conditions'].get('geometry_scale',1.)!=1. for c in cases):
                # Different tunnel scales require different geometry/XML.
                rows=[self.static(parameters,[c])[0] for c in cases]
            else:rows=self.static(parameters,cases)
            self.last=[r['actual'] for r in rows]
            return [r['values'][c['metric']] for r,c in zip(rows,cases)]
        for case in cases:
            if case['metric']=='climb_rate_fpm':value,actual=self.climb(parameters,case)
            else:
                row=self.static(parameters,[case])[0];value,actual=row['values'][case['metric']],row['actual']
            values.append(value);self.last.append(actual)
        return values

    def probe(self,parameters):
        cases=[dict(conditions=dict(alpha_deg=alpha,beta_deg=0.,mach=.3,altitude_m=1000.)) for alpha in (0.,5.)]
        # Rotor FCS functions also define physical propulsion variables, so
        # retain them for diagnostic loads; no external accuracy claim follows.
        rotor=self.kind in ('helicopter','multirotor')
        first=self.static(parameters,cases,neutral=not rotor)
        repeated=self.static(parameters,cases,neutral=not rotor)
        return dict(reference_error_percent=None,reason='Diagnostic SDK loads; no public reference assigned',
                    first=first,repeated=repeated)
