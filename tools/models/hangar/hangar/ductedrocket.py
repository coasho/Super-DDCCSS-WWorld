"""Atmosphere-fed ducted-rocket engineering cycle, integrated only by JSBSim.

The fully expanded equivalent nozzle uses conservation of mass, total enthalpy
and net momentum (NASA Glenn ramjet thrust equation). It is not a measured
Meteor inlet/combustor/nozzle map. Every cycle/controller input is explicit in
the selected card; absent inputs refuse generation.
"""
import copy
import math
import xml.etree.ElementTree as ET

IMPLEMENTATION='fsim.propulsion.jsbsim_ducted_rocket.v1'
IMPLEMENTATION_V2='fsim.propulsion.jsbsim_ducted_rocket.v2'
ROLES_V2_EXTRA={'inlet_residence_s':'s'}
ROLES={'capture_area_m2':'m^2','fuel_flow_max_kg_s':'kg/s','heating_value_j_kg':'J/kg',
       'combustion_efficiency':'1','pressure_recovery':'1','air_cp_j_kg_k':'J/(kg*K)',
       'gas_cp_j_kg_k':'J/(kg*K)','gas_gamma':'1','fuel_air_ratio_max':'1',
       'lightoff_mach':'Mach','target_mach':'Mach','valve_minimum':'1',
       'speed_gain':'1','valve_lag_s':'s','valve_rate_s_1':'1/s'}
LB=.45359237
LBF=4.4482216152605


def inputs(card):
    from . import components
    row=components.resolve(card,'propulsion')
    if row is None or row.get('implementation_id') not in (IMPLEMENTATION,IMPLEMENTATION_V2):return None
    values={key:components.number(row,key,unit) for key,unit in ROLES.items()}
    if any(not math.isfinite(v) or v<=0 for v in values.values()):
        raise ValueError('ducted rocket: cycle inputs must be finite and positive')
    if (values['combustion_efficiency']>1 or values['pressure_recovery']>1 or
        values['gas_gamma']<=1 or values['gas_gamma']>=2 or
        values['fuel_air_ratio_max']>=1 or values['valve_minimum']>1 or
        values['lightoff_mach']<=1 or values['target_mach']<=values['lightoff_mach']):
        raise ValueError('ducted rocket: cycle/controller inputs outside the supported domain')
    if row.get('implementation_id')==IMPLEMENTATION_V2:
        values['inlet_residence_s']=components.number(row,'inlet_residence_s','s')
        if not math.isfinite(values['inlet_residence_s']) or values['inlet_residence_s']<=0 or not all(math.isfinite(v) for v in (1/values['inlet_residence_s'],2*values['inlet_residence_s'],values['inlet_residence_s']/LB)):
            raise ValueError('ducted rocket v2: residence time must be positive representable')
    return values


def cycle(c, *, density, speed, temperature, mach, fuel_flow):
    """Independent-use SI reference evaluator; no flight or fuel integration."""
    values=(density,speed,temperature,mach,fuel_flow)
    if any(not math.isfinite(x) or x<0 for x in values):raise ValueError('invalid cycle condition')
    air=density*speed*c['capture_area_m2'] if mach>=c['lightoff_mach'] and fuel_flow>0 else 0.
    inlet_temperature=temperature*(1+.2*mach*mach)
    burned=min(fuel_flow,air*c['fuel_air_ratio_max'])
    total_temperature=(air*c['air_cp_j_kg_k']*inlet_temperature+
        burned*c['combustion_efficiency']*c['heating_value_j_kg'])/max((air+fuel_flow)*c['gas_cp_j_kg_k'],1e-12)
    pressure_ratio=max(1.,(1+.2*mach*mach)**3.5*c['pressure_recovery'])
    exit_speed=math.sqrt(max(0.,2*c['gas_cp_j_kg_k']*total_temperature*
        (1-pressure_ratio**(-(c['gas_gamma']-1)/c['gas_gamma']))))
    return dict(air_kg_s=air,burned_fuel_kg_s=burned,total_temperature_k=total_temperature,
        exit_speed_m_s=exit_speed,net_thrust_n=(air+fuel_flow)*exit_speed-air*speed)


def install(products,card,design,motor):
    """Split the existing grain into booster/fuel and emit a native cycle/valve."""
    c=inputs(card)
    if c is None:return products
    if 'inlet_residence_s' in c:return _install_v2(products,card,design,motor,c)
    if len(motor['phases'])!=2:raise ValueError('ducted rocket requires one booster and one fuel-ledger phase')
    from .storecontrol import _op as op,_property as prop,_value as val
    from .storebodies import _ignition_time_thrust_rows,_IGNITION_TIME_THRUST
    name=card['model'].removeprefix('jsbsim:');out=copy.deepcopy(products)
    root=ET.fromstring(out[name+'.xml']);native=root.find('propulsion');fcs=root.find('flight_control')
    full=native.find('tank');full_mass=motor['propellant_kg'];full_length=motor['length'];center=motor['x']
    booster_mass=1000*motor['phases'][0]['thrust_kn']*motor['phases'][0]['duration_s']/(motor['isp']*9.80665)
    fuel_mass=full_mass-booster_mass
    if not 0<booster_mass<full_mass:raise ValueError('ducted rocket requires separate positive booster and fuel masses')
    native.remove(full)
    # Contiguous pieces of the same original uniform annular grain preserve
    # its launch mass, centroid and all three moments of inertia exactly.
    length_boost=full_length*booster_mass/full_mass;length_fuel=full_length-length_boost
    for mass,length,x in ((booster_mass,length_boost,center+.5*length_fuel),
                          (fuel_mass,length_fuel,center-.5*length_boost)):
        tank=copy.deepcopy(full);tank.find('location/x').text='%.17g'%(x/.0254)
        tank.find('grain_config/length').text='%.17g'%(length/.0254)
        for tag in ('capacity','contents'):tank.find(tag).text='%.17g'%(mass/LB)
        native.append(tank)
    engine=copy.deepcopy(native.find('engine'));engine.set('file',name+'_sustainer')
    engine.find('feed').text='1';engine.find('thruster').set('file',name+'_sustainer_nozzle')
    native.insert(1,engine)
    boost=ET.fromstring(out['Engines/'+name+'_motor.xml']);table=boost.find('thrust_table')
    table.set('name',_IGNITION_TIME_THRUST)
    table.find('tableData').text='\n'+'\n'.join('%.17g %.17g'%row for row in
        _ignition_time_thrust_rows(motor['phases'][:1],buildup_s=float(boost.findtext('builduptime'))))+'\n'
    out['Engines/'+name+'_motor.xml']=ET.tostring(boost,encoding='unicode')+'\n'
    channel=ET.SubElement(fcs,'channel',name='Finite ducted-rocket fuel valve and thermodynamic cycle')
    prefix='fsim/ducted-rocket/'
    def p(leaf):return prop(prefix+'engine[1]/'+leaf)
    def function(leaf,expr):
        node=ET.SubElement(channel,'fcs_function',name=prefix+(leaf if leaf=='valve-demand' else 'engine[1]/'+leaf))
        ET.SubElement(node,'function').append(expr)
    # Automatic own-speed fuel management is explicit engineering control,
    # never a target-truth input. Booster-only mode and valve reset are native.
    gate=op('and',op('ge',prop('simulation/sim-time-sec'),val(motor['phases'][0]['duration_s'])),
            op('ge',prop('velocities/mach'),val(c['lightoff_mach'])))
    demand=op('max',val(0),op('min',val(1),op('sum',val(c['valve_minimum']),
        op('product',val(c['speed_gain']),op('difference',val(c['target_mach']),prop('velocities/mach'))))))
    function('valve-demand',op('ifthen',gate,demand,val(0)))
    actuator=ET.SubElement(channel,'actuator',name=prefix+'valve')
    ET.SubElement(actuator,'input').text=prefix+'valve-demand'
    ET.SubElement(actuator,'lag').text='%.17g'%(1/c['valve_lag_s'])
    ET.SubElement(actuator,'rate_limit').text='%.17g'%c['valve_rate_s_1']
    clip=ET.SubElement(actuator,'clipto');ET.SubElement(clip,'min').text='0';ET.SubElement(clip,'max').text='1'
    ET.SubElement(actuator,'output').text='fcs/throttle-pos-norm[1]'
    function('fuel-kg-s',op('product',prop('propulsion/engine[1]/fuel-flow-rate-pps'),val(LB)))
    function('air-kg-s',op('ifthen',op('and',op('gt',p('fuel-kg-s'),val(0)),
        op('ge',prop('velocities/mach'),val(c['lightoff_mach']))),op('product',
            prop('atmosphere/rho-slugs_ft3'),val(515.3788183931961),
            prop('velocities/vt-fps'),val(.3048*c['capture_area_m2'])),val(0)))
    function('inlet-temperature-k',op('product',prop('atmosphere/T-R'),val(5/9),op('sum',val(1),
        op('product',val(.2),prop('velocities/mach'),prop('velocities/mach')))))
    function('burned-fuel-kg-s',op('min',p('fuel-kg-s'),op('product',p('air-kg-s'),val(c['fuel_air_ratio_max']))))
    function('total-temperature-k',op('quotient',op('sum',op('product',p('air-kg-s'),
        val(c['air_cp_j_kg_k']),p('inlet-temperature-k')),op('product',p('burned-fuel-kg-s'),
        val(c['combustion_efficiency']*c['heating_value_j_kg']))),op('max',val(1e-12),
            op('product',op('sum',p('air-kg-s'),p('fuel-kg-s')),val(c['gas_cp_j_kg_k'])))))
    function('pressure-ratio',op('max',val(1),op('product',val(c['pressure_recovery']),op('pow',
        op('sum',val(1),op('product',val(.2),prop('velocities/mach'),prop('velocities/mach'))),val(3.5)))))
    function('exit-speed-m-s',op('sqrt',op('max',val(0),op('product',val(2*c['gas_cp_j_kg_k']),
        p('total-temperature-k'),op('difference',val(1),op('pow',p('pressure-ratio'),
            val(-(c['gas_gamma']-1)/c['gas_gamma'])))))))
    function('net-thrust-n',op('difference',op('product',op('sum',p('air-kg-s'),p('fuel-kg-s')),
        p('exit-speed-m-s')),op('product',p('air-kg-s'),prop('velocities/vt-fps'),val(.3048))))
    sustainer=ET.Element('rocket_engine',name=name+' atmosphere-fed engineering sustainer')
    # Engine pre-functions must own the cycle: FGRocket evaluates its Isp
    # during load, before the airframe FCS has been constructed. Defining
    # these quantities only in a later FCS would leave forward references.
    for node in list(channel):
        if node.tag=='fcs_function' and node.get('name')!=prefix+'valve-demand':
            channel.remove(node);function=node.find('function')
            function.set('name',node.get('name').replace('engine[1]','engine[#]'))
            function.set('type','pre');sustainer.append(function)
    isp=ET.SubElement(sustainer,'isp');ET.SubElement(isp,'function').append(op('quotient',
        op('max',val(0),p('net-thrust-n')),op('max',val(1e-12),op('product',p('fuel-kg-s'),val(9.80665)))))
    ET.SubElement(sustainer,'propflowmax',unit='LBS/SEC').text='%.17g'%(c['fuel_flow_max_kg_s']/LB)
    ET.SubElement(sustainer,'mixtureratio').text='0'
    ET.SubElement(sustainer,'minthrottle').text='0.000001'
    out['Engines/'+name+'_sustainer.xml']=ET.tostring(sustainer,encoding='unicode')+'\n'
    # Net momentum thrust already includes free-stream ram drag and fully
    # expanded pressure. A zero-area equivalent nozzle avoids subtracting
    # ambient pressure a second time; it is not the geometric nozzle area.
    out['Engines/'+name+'_sustainer_nozzle.xml']='<nozzle name="Fully expanded equivalent cycle nozzle"><area unit="FT2">0</area></nozzle>\n'
    # Where the cycle has net drag, do not discard it through FGNozzle's
    # positive-thrust clamp. Express that axial propulsion load in wind axes.
    aero=root.find('aerodynamics')
    negative=op('min',val(0),p('net-thrust-n'))
    # Tw2b.T * (Fx,0,0) = (Fx*cos(alpha)*cos(beta),
    # -Fx*cos(alpha)*sin(beta), -Fx*sin(alpha)). JSBSim negates
    # the authored DRAG and LIFT components before applying Tw2b.
    for axis,sign,trig in (('DRAG',-1,op('product',op('cos',prop('aero/alpha-rad')),op('cos',prop('aero/beta-rad')))),
                      ('SIDE',-1,op('product',op('cos',prop('aero/alpha-rad')),op('sin',prop('aero/beta-rad')))),
                      ('LIFT',1,op('sin',prop('aero/alpha-rad')))):
        node=aero.find("axis[@name='"+axis+"']")
        if node is None:node=ET.SubElement(aero,'axis',name=axis)
        f=ET.SubElement(node,'function',name=prefix+'ram-drag-'+axis.lower())
        f.append(op('product',val(sign/LBF),copy.deepcopy(negative),trig))
    ET.indent(root,space='    ')
    out[name+'.xml']=ET.tostring(root,encoding='unicode')+'\n'
    return out


def inlet_equilibrium_v2(c, *, density,velocity,temperature,mach):
    """SI steady control-volume oracle, not a second flight integrator."""
    if len(velocity)!=3 or any(not math.isfinite(x) for x in (*velocity,density,temperature,mach)) or density<0 or temperature<=0 or mach<0:
        raise ValueError('invalid inlet condition')
    air=density*c['capture_area_m2']*max(velocity[0],0.) if mach>=c['lightoff_mach'] else 0.
    residence=c['inlet_residence_s']
    enthalpy=c['air_cp_j_kg_k']*temperature+.5*sum(v*v for v in velocity)
    return dict(air_kg_s=air,air_mass_kg=residence*air,enthalpy_j=residence*air*enthalpy,
                momentum_body_kg_m_s=[-residence*air*v for v in velocity])


def _install_v2(products,card,design,motor,c):
    """Convective finite inventory; matched mass/enthalpy/momentum filters.

    The native lag is trapezoidal. Positive inventories require the admitted
    native substep <= 2*residence; root admission checks this before execution.
    This is a lumped transit inventory, not a pressure-resolved combustor.
    """
    legacy=copy.deepcopy(card)
    legacy['component_models']['components']['propulsion']['implementation_id']=IMPLEMENTATION
    out=install(products,legacy,design,motor)
    from .storecontrol import _op as op,_property as prop,_value as val
    name=card['model'].removeprefix('jsbsim:');root=ET.fromstring(out[name+'.xml'])
    engine=ET.fromstring(out['Engines/'+name+'_sustainer.xml'])
    for node in list(engine):
        if node.tag=='function':engine.remove(node)
    fcs=root.find('flight_control');prefix='fsim/ducted-inlet-v2/'
    channel=ET.SubElement(fcs,'channel',name='Finite convective inlet air inventory v2')
    def p(leaf):return prop(prefix+leaf)
    def f(leaf,expr):
        node=ET.SubElement(channel,'fcs_function',name=prefix+leaf)
        ET.SubElement(node,'function').append(expr)
    def e(leaf,expr):
        node=ET.SubElement(engine,'function',name=prefix+'engine[#]/'+leaf,type='pre');node.append(expr)
    def ep(leaf):return prop(prefix+'engine[1]/'+leaf)
    tau=c['inlet_residence_s']
    ET.SubElement(fcs,'property',value='2').text=prefix+'version'
    ET.SubElement(fcs,'property',value='1').text=prefix+'engine[1]/finite-fuel-v2'
    for key,value in c.items():ET.SubElement(fcs,'property',value='%.17g'%value).text=prefix+'input-'+key
    ET.SubElement(fcs,'property',value='%.17g'%tau).text=prefix+'residence-s'
    ET.SubElement(fcs,'property',value='%.17g'%(2*tau)).text=prefix+'maximum-native-substep-s'
    velocity=[op('product',prop('velocities/'+axis+'-aero-fps'),val(.3048)) for axis in 'uvw']
    gate=op('and',op('ge',prop('simulation/sim-time-sec'),val(motor['phases'][0]['duration_s'])),
        op('ge',prop('velocities/mach'),val(c['lightoff_mach'])))
    f('capture-kg-s',op('ifthen',gate,op('product',prop('atmosphere/rho-slugs_ft3'),
        val(515.3788183931961*c['capture_area_m2']),op('max',val(0),copy.deepcopy(velocity[0]))),val(0)))
    ht=op('sum',op('product',val(c['air_cp_j_kg_k']*5/9),prop('atmosphere/T-R')),
        op('product',val(.5),op('sum',*(op('product',copy.deepcopy(v),copy.deepcopy(v)) for v in velocity))))
    f('incoming-enthalpy-w',op('product',p('capture-kg-s'),ht))
    for axis,v in zip('xyz',velocity):f('incoming-momentum-'+axis+'-n',op('product',p('capture-kg-s'),v))
    fluxes=[('capture-kg-s','outflow-kg-s'),('incoming-enthalpy-w','outflow-enthalpy-w')]+[
        ('incoming-momentum-'+a+'-n','outflow-momentum-'+a+'-n') for a in 'xyz']
    for source,target in fluxes:
        # Engine loads before FCS: declare zero inventory for Isp's load-time
        # evaluation. The FCS filter subsequently owns this same property.
        ET.SubElement(engine,'property',value='0').text=prefix+target
        node=ET.SubElement(channel,'lag_filter',name=prefix+target+'-filter')
        ET.SubElement(node,'input').text=prefix+source
        ET.SubElement(node,'c1').text='%.17g'%(1/tau)
        ET.SubElement(node,'output').text=prefix+target
    balance=root.find('mass_balance');mass_index=len(balance.findall('pointmass'))
    mass=ET.SubElement(balance,'pointmass',name='Finite resident inlet air v2')
    ET.SubElement(mass,'weight',unit='LBS').text='0'
    location=ET.SubElement(mass,'location',unit='M')
    stations=[s for inlet in design.get('intake',[]) for s in inlet.get('stations',[])]
    centre=sum(float(s['x']) for s in stations)/len(stations) if stations else float(motor['x'])
    for axis,value in zip('xyz',(centre,0,0)):ET.SubElement(location,axis).text='%.17g'%value
    ET.SubElement(fcs,'property',value=str(mass_index)).text=prefix+'resident-pointmass-index'
    f('resident-air-mass-kg',op('product',val(tau),p('outflow-kg-s')))
    f('resident-enthalpy-j',op('product',val(tau),p('outflow-enthalpy-w')))
    for axis in 'xyz':f('resident-momentum-'+axis+'-kg-m-s',op('product',val(-tau),p('outflow-momentum-'+axis+'-n')))
    node=ET.SubElement(channel,'fcs_function',name=prefix+'resident-air-native-weight')
    ET.SubElement(node,'function').append(op('product',val(tau/LB),p('outflow-kg-s')))
    ET.SubElement(node,'output').text='inertia/pointmass-weight-lbs[%d]'%mass_index
    # Native actual consumed fuel, not requested throttle, supplies heat.
    e('fuel-kg-s',op('product',prop('propulsion/engine[1]/fuel-flow-rate-pps'),val(LB)))
    e('burned-fuel-kg-s',op('min',ep('fuel-kg-s'),op('product',p('outflow-kg-s'),val(c['fuel_air_ratio_max']))))
    e('total-temperature-k',op('quotient',op('sum',p('outflow-enthalpy-w'),op('product',ep('burned-fuel-kg-s'),
        val(c['combustion_efficiency']*c['heating_value_j_kg']))),op('max',val(1e-12),
        op('product',op('sum',p('outflow-kg-s'),ep('fuel-kg-s')),val(c['gas_cp_j_kg_k'])))))
    # Quasi-steady isentropic pressure closure on resident specific total
    # enthalpy, with declared recovery; no separate pressure-work source.
    e('pressure-ratio',op('max',val(1),op('product',val(c['pressure_recovery']),op('pow',op('quotient',
        p('outflow-enthalpy-w'),op('max',val(1e-12),op('product',p('outflow-kg-s'),
            val(c['air_cp_j_kg_k']*5/9),prop('atmosphere/T-R')))),val(3.5)))))
    e('exit-speed-m-s',op('sqrt',op('max',val(0),op('product',val(2*c['gas_cp_j_kg_k']),ep('total-temperature-k'),
        op('difference',val(1),op('pow',ep('pressure-ratio'),val(-(c['gas_gamma']-1)/c['gas_gamma'])))))))
    e('gross-thrust-n',op('product',op('sum',p('outflow-kg-s'),ep('fuel-kg-s')),ep('exit-speed-m-s')))
    isp=engine.find('isp/function');isp.clear();isp.append(op('quotient',ep('gross-thrust-n'),
        op('max',val(1e-12),op('product',ep('fuel-kg-s'),val(9.80665)))))
    # Remove v1 net-negative-only force. V2 always owns the full ram and
    # stored-relative-momentum transport reaction in the native body frame.
    for axis in root.findall('aerodynamics/axis'):
        for node in list(axis):
            if node.get('name','').startswith('fsim/ducted-rocket/ram-drag-'):axis.remove(node)
    reactions=root.find('external_reactions')
    if reactions is None:reactions=ET.SubElement(root,'external_reactions')
    angular=[prop('velocities/'+a+'i-rad_sec') for a in 'pqr']
    for j,axis in enumerate('xyz'):
        k,l=(j+1)%3,(j+2)%3
        # -Omega x P_res, where P_res=-tau*outgoing captured momentum.
        correction=op('product',val(tau),op('difference',op('product',copy.deepcopy(angular[k]),p('outflow-momentum-'+'xyz'[l]+'-n')),
            op('product',copy.deepcopy(angular[l]),p('outflow-momentum-'+'xyz'[k]+'-n'))))
        terms=[op('product',val(-1),p('outflow-momentum-'+axis+'-n')),correction]
        if j==0:terms.append(op('ifthen',op('le',ep('fuel-kg-s'),val(1e-12)),ep('gross-thrust-n'),val(0)))
        # One owned native pre-function freezes the exact force-epoch N
        # value. Both the actual force and producer read that same cache.
        reaction=ET.SubElement(reactions,'function',name=prefix+'ram-reaction-'+axis+'-n',type='pre')
        reaction.append(op('sum',*terms))
        force=ET.SubElement(reactions,'force',name=prefix+'ram-reaction-'+axis,frame='BODY')
        ET.SubElement(force,'function').append(op('product',val(1/LBF),p('ram-reaction-'+axis+'-n')))
        net=ET.SubElement(reactions,'function',name=prefix+'net-propulsion-force-'+axis+'-n',type='pre')
        net.append(op('sum',op('product',val(LBF),prop('forces/fb'+axis+'-prop-lbs')),p('ram-reaction-'+axis+'-n')))
        where=ET.SubElement(force,'location',unit='M')
        direction=ET.SubElement(force,'direction')
        for i,a in enumerate('xyz'):
            ET.SubElement(where,a).text='%.17g'%(centre if i==0 else 0)
            ET.SubElement(direction,a).text=str(int(i==j))
    ET.indent(root,space='    ');ET.indent(engine,space='    ')
    out[name+'.xml']=ET.tostring(root,encoding='unicode')+'\n'
    out['Engines/'+name+'_sustainer.xml']=ET.tostring(engine,encoding='unicode')+'\n'
    return out
