"""Versioned axial reference for the declared atmosphere-fed propulsion type.

RocketPy supplies the booster mass/thrust model. SciPy integrates a separately
written conservation-of-mass/enthalpy/momentum cycle and finite fuel valve.
This checks the authored engineering equations, not measured hardware data.
"""
import math
import xml.etree.ElementTree as ET

IMPLEMENTATION = 'fsim.propulsion.jsbsim_ducted_rocket.v1'
VERSION = 'fsim.reference.ducted-axial.v1'
PREFIX = 'fsim/ducted-rocket/'
LB, LBF, G0 = .45359237, 4.4482216152605, 9.80665
UNITS = {'capture_area_m2':'m^2', 'fuel_flow_max_kg_s':'kg/s',
         'heating_value_j_kg':'J/kg', 'combustion_efficiency':'1',
         'pressure_recovery':'1', 'air_cp_j_kg_k':'J/(kg*K)',
         'gas_cp_j_kg_k':'J/(kg*K)', 'gas_gamma':'1',
         'fuel_air_ratio_max':'1', 'lightoff_mach':'Mach',
         'target_mach':'Mach', 'valve_minimum':'1', 'speed_gain':'1',
         'valve_lag_s':'s', 'valve_rate_s_1':'1/s'}


def configuration(card, root):
    from hangar.components import resolve, number
    row = resolve(card, 'propulsion')
    if row is None or row.get('implementation_id') != IMPLEMENTATION:
        return None
    if row['availability'] != 'available' or not set(UNITS).issubset(row['parameters']):
        raise ValueError('Incomplete typed ducted propulsion descriptor')
    values = {k:number(row, k, u) for k,u in UNITS.items()}
    if any(not math.isfinite(v) or v <= 0 for v in values.values()):
        raise ValueError('Invalid typed cycle parameter')
    if values['valve_rate_s_1'] < 1/values['valve_lag_s']:
        raise ValueError('This axial reference requires a provably nonbinding valve slew limit')
    tanks = root.findall('propulsion/tank')
    engines = root.findall('propulsion/engine')
    if len(tanks) != 2 or len(engines) != 2 or [e.findtext('feed','').strip() for e in engines] != ['0','1']:
        raise ValueError('Typed propulsion requires distinct booster and fuel ledgers')
    masses = []
    for tank in tanks:
        node = tank.find('contents')
        if node is None or node.get('unit') != 'LBS':
            raise ValueError('Unsupported typed tank units')
        masses.append(float(node.text)*LB)
    values.update(booster_mass_kg=masses[0], fuel_mass_kg=masses[1], descriptor=row,
                  booster_engine_file=engines[0].get('file'))
    return values


def propulsion_functions(root):
    """Verify the exact signed cycle transfer; exclude it only from CA extraction."""
    def leaf(tag, text):
        n=ET.Element(tag);n.text=str(text);return n
    def op(tag, *children):
        n=ET.Element(tag);n.extend(children);return n
    def signature(n):
        return n.tag, tuple(sorted(n.attrib.items())), (n.text or '').strip(), tuple(signature(c) for c in n)
    found=[]
    for axis, sign, trig in (
        ('DRAG',-1,op('product',op('cos',leaf('property','aero/alpha-rad')),op('cos',leaf('property','aero/beta-rad')))),
        ('SIDE',-1,op('product',op('cos',leaf('property','aero/alpha-rad')),op('sin',leaf('property','aero/beta-rad')))),
        ('LIFT',1,op('sin',leaf('property','aero/alpha-rad')))):
        name=PREFIX+'ram-drag-'+axis.lower()
        nodes=root.findall("aerodynamics/axis[@name='%s']/function[@name='%s']"%(axis,name))
        expected=op('function',op('product',leaf('value',sign/LBF),
            op('min',leaf('value',0),leaf('property',PREFIX+'engine[1]/net-thrust-n')),trig))
        expected.set('name',name)
        # XML number spelling is immaterial; values must be numerically exact.
        for n in nodes+[expected]:
            for v in n.iter('value'):v.text=repr(float(v.text))
        if len(nodes)!=1 or signature(nodes[0])!=signature(expected):
            raise ValueError('Malformed or missing signed propulsion transfer: '+name)
        found.append(name)
    all_transfers=[n.get('name') for n in root.findall('aerodynamics/axis/function')
                   if n.get('name','').startswith(PREFIX)]
    if sorted(all_transfers)!=sorted(found):
        raise ValueError('Unknown or duplicate typed propulsion force: '+repr(all_transfers))
    return tuple(found)


def flight(data, atmosphere, times, initial, *, max_step, spherical):
    import numpy as np
    from scipy.integrate import solve_ivp
    from rocketpy import Function, SolidMotor
    from reference import phase_curve, radial_gravity
    c=data['ducted'];m=data['motor'];phase=m['phases'][0]
    booster_xml=ET.fromstring(data['generated']['Engines/'+c['booster_engine_file']+'.xml'])
    buildup=float(booster_xml.findtext('builduptime'))
    curve=phase_curve([phase],buildup_s=buildup)
    fraction=c['booster_mass_kg']/m['propellant_kg']
    boost=SolidMotor(thrust_source=curve,burn_time=(0,curve[-1][0]),dry_mass=0,
        dry_inertia=(0,0,0),nozzle_radius=math.sqrt(m['exit_area']/math.pi),
        grain_number=1,grain_density=m['grain_density_kg_m3'],grain_outer_radius=m['radius'],
        grain_initial_inner_radius=m['bore']/2,grain_initial_height=m['length']*fraction,
        grain_separation=0,grains_center_of_mass_position=0,center_of_dry_mass_position=0,
        nozzle_position=0,reference_pressure=0,only_radial_burn=True)
    drag=Function(data['drag'],interpolation='linear',extrapolation='constant')
    area=math.pi*data['radius']**2
    sound=initial['sound_ms'];rho=atmosphere['density_kg_m3'];temp=atmosphere['temperature_k']
    def rhs(t,y):
        height,speed,fuel,valve,impulse=y
        mach=abs(speed)/sound
        demand=min(1.,max(0.,c['valve_minimum']+c['speed_gain']*(c['target_mach']-mach))) if t>=phase['duration_s'] and mach>=c['lightoff_mach'] else 0.
        dv=max(-c['valve_rate_s_1'],min(c['valve_rate_s_1'],(demand-valve)/c['valve_lag_s']))
        flow=c['fuel_flow_max_kg_s']*min(1.,max(0.,valve)) if fuel>0 and valve>=1e-6 else 0.
        air=rho*abs(speed)*c['capture_area_m2'] if mach>=c['lightoff_mach'] and flow>0 else 0.
        burned=min(flow,air*c['fuel_air_ratio_max'])
        total_temp=(air*c['air_cp_j_kg_k']*temp*(1+.2*mach*mach)+burned*c['combustion_efficiency']*c['heating_value_j_kg'])/max((air+flow)*c['gas_cp_j_kg_k'],1e-12)
        pr=max(1.,(1+.2*mach*mach)**3.5*c['pressure_recovery'])
        exhaust=math.sqrt(max(0.,2*c['gas_cp_j_kg_k']*total_temp*(1-pr**(-(c['gas_gamma']-1)/c['gas_gamma']))))
        cycle=(air+flow)*exhaust-air*abs(speed)
        thrust=float(boost.thrust(t)) if 0<=t<curve[-1][0] else 0.
        mass=data['dry_mass']+max(0.,float(boost.propellant_mass(t)))+max(0.,fuel)
        force=thrust-atmosphere['pressure_pa']*m['exit_area'] if thrust>0 else 0.
        force=max(0.,force)+cycle
        gravity=radial_gravity(height) if spherical else initial['gravity_ms2']
        return [speed,(force-.5*rho*speed*abs(speed)*area*float(drag(mach)))/mass-gravity,-flow,dv,thrust+cycle]
    solution=solve_ivp(rhs,(0,times[-1]),[8000.,250.,c['fuel_mass_kg'],0.,0.],
        t_eval=times,max_step=max_step,rtol=1e-9,atol=1e-9)
    if not solution.success or len(solution.t)!=len(times) or not np.isfinite(solution.y).all():
        raise ValueError('Typed axial reference did not complete')
    rows=[[t,data['dry_mass']+max(0.,float(boost.propellant_mass(t)))+max(0.,solution.y[2,i]),
           float(solution.y[1,i]),float(solution.y[0,i])] for i,t in enumerate(times)]
    return rows,float(boost.total_impulse),curve[-1][0],dict(
        method=VERSION,basis='simulated',cycle_impulse_scope='booster vacuum plus signed cycle momentum',
        combined_impulse_ns=float(solution.y[4,-1]),remaining_fuel_kg=float(solution.y[2,-1]),
        comparison_horizon_s=times[-1],actual_sustainer_burnout_claimed=False,
        valve_reference='continuous lag; rate>=1/tau proves slew cannot bind for unit-bounded demand; native finite-step convergence retained',
        sources=['https://www.grc.nasa.gov/www/k-12/airplane/ramjet.html',
                 'https://docs.rocketpy.org/en/latest/reference/classes/motors/SolidMotor.html'])
