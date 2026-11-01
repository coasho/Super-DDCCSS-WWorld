"""Versioned liquid-fuel store plant, using the common native aircraft engine.

All sizing is explicit engineering source input. No design identity selects
this path. The normal JSBSim turbine owns thrust, spool, fuel use and starvation.
"""
import math
import xml.etree.ElementTree as ET

ROLES={'thrust_dry_kn':'kN','bypass_ratio':'1','tsfc_dry':'lb/(lbf*h)',
       'fuel_capacity_kg':'kg','fuel_cg_x_m':'m','dry_mass_kg':'kg',
       'cruise_speed_ms':'m/s','governor_time_s':'s'}


def dry_inertia(design):
    mass=float(design['mass']['empty']);g=design['mass']['gyration']
    length=max(b['stations'][-1]['x'] for b in design['body'])-min(b['stations'][0]['x'] for b in design['body'])
    inertia=[mass*(float(value)*length)**2 for value in g]
    if len(inertia)!=3 or not all(math.isfinite(v) and v>0 for v in inertia) or 2*max(inertia)>sum(inertia):
        raise ValueError('turbofan: dry inertia must be a physical positive tensor')
    return inertia


def full_mass_properties(design):
    dry=float(design['mass']['empty']);x=float(design['mass']['empty_cg'][0])
    tanks=design['mass']['tank'];fuel=[float(t['capacity'])*float(t.get('fill',1.)) for t in tanks]
    full=dry+sum(fuel);cg=(dry*x+sum(m*t['position'][0] for m,t in zip(fuel,tanks)))/full
    shift=dry*(x-cg)**2+sum(m*(t['position'][0]-cg)**2 for m,t in zip(fuel,tanks))
    inertia=dry_inertia(design);inertia[1]+=shift;inertia[2]+=shift
    return full,cg,inertia


def component_proposal(card,design):
    """Explicit source-authoring proposal; generation never installs it silently."""
    import copy
    from . import components
    engines=design.get('engine',[])
    if len(engines)!=1 or engines[0].get('type')!='turbofan' or design.get('store_propulsion',{}).get('version')!=1:
        raise ValueError('A versioned single-turbofan source is required')
    result=copy.deepcopy(card);rows=result['component_models']['components'];parameters=result['parameters']
    source='project-virtual-cruise-v1';name=result['model'][7:]
    result['sources'][source]={'evidence_kind':'game_reference','title':'Explicit project cruise configuration v1',
        'url':'aircraft/'+name+'/'+name+'.toml','version':'1',
        'note':'Engineering design inputs and derived controller, not variant hardware measurements. Engine uses shared JSBSim/Mattingly lapse model; wing uses common VLM. Static authorized aimpoint only; no terminal seeker.'}
    def parameter(key,value,unit):
        parameters[key]={'value':float(value),'unit':unit,'basis':'assumed','source':source,
            'uncertainty':abs(float(value))*.5,'note':'Explicit virtual configuration v1; 50% engineering uncertainty allowance, not a confidence interval or measured value.'}
        return key
    def descriptor(implementation,refs,bindings):
        return dict(availability='available',implementation_id=implementation,configuration_version=1,
            parameter_refs=refs,bindings=bindings,basis='assumed',source=source,
            note='Explicit versioned project design; actual native execution is required.',
            validity='Estimated virtual configuration, without exact-variant performance calibration.')
    controls=copy.deepcopy(result)
    for kind in ('autopilot','allocator','actuator'):controls['component_models']['components'].pop(kind,None)
    proposal=components.migration(controls,design);parameters.update(proposal['parameters'])
    for kind in ('autopilot','allocator','actuator'):rows[kind]=proposal['components'][kind]
    engine=engines[0];tank=design['mass']['tank'][0];config=design['store_propulsion']
    values={key:engine[key] for key in ('thrust_dry_kn','bypass_ratio','tsfc_dry')}
    values.update(fuel_capacity_kg=tank['capacity'],fuel_cg_x_m=tank['position'][0],dry_mass_kg=design['mass']['empty'],
                  cruise_speed_ms=config['cruise_speed_ms'],governor_time_s=config['governor_time_s'])
    rows['propulsion']=descriptor(components.TURBOFAN_PROPULSION,
        {key:parameter('plant_turbofan_'+key,value,ROLES[key]) for key,value in values.items()},
        dict(model=result['model'],design='aircraft/'+name+'/'+name+'.toml',engine='Engines/'+name+'_engine0.xml',nozzle='Engines/'+name+'_nozzle0.xml'))
    guide={'response_min_s':(2.,'s'),'response_max_s':(20.,'s'),'max_g':(2.,'1'),'life_s':(6000.,'s')}
    rows['guidance']=descriptor('fsim.guidance.inertial_waypoint.v1',
        {key:parameter('component_inertial_'+key,value,unit) for key,(value,unit) in guide.items()},
        dict(navigation='frozen_authorized_static_aimpoint'))
    result['guidance_capabilities']=dict(version=1,inertial_midcourse=True,launcher_updates=False,basis='assumed',source=source,
        note='Formal static inertial aimpoint configuration. No terminal seeker is installed; no post-release target observation is claimed.')
    result['guidance']='Frozen authorized static aimpoint; native finite aerodynamic control'
    effects=components.contact_effects_v1(result,arming_s=1.,life_s=6000.,source=source,terrain_contact=True)
    parameters.update(effects['parameters'])
    for kind in ('fuze','warhead'):rows[kind]=effects['component_models']['components'][kind]
    return result


def inputs(card,design):
    from . import components
    row=components.resolve(card,'propulsion')
    declared='store_propulsion' in design or bool(design.get('engine'))
    if not declared:
        if row is not None and row['implementation_id']==components.TURBOFAN_PROPULSION:
            raise ValueError('turbofan: selected component requires its explicit source declaration')
        return None
    config=design.get('store_propulsion')
    engines=design.get('engine',[])
    if not isinstance(config,dict) or type(config.get('version')) is not int or config['version']!=1:
        raise ValueError('turbofan: unsupported or missing explicit source version')
    if not isinstance(engines,list) or len(engines)!=1 or not isinstance(engines[0],dict) or engines[0].get('type')!='turbofan':
        raise ValueError('turbofan: version 1 requires exactly one authored turbofan')
    if design.get('motor'):
        raise ValueError('turbofan: version 1 cannot discard mixed rocket motor hardware')
    if row is None or row['availability']!='available' or row['implementation_id']!=components.TURBOFAN_PROPULSION:
        raise ValueError('turbofan: authored engine requires its explicit available propulsion component')
    components._available(row,components.TURBOFAN_PROPULSION,ROLES,('model','design','engine','nozzle'),card)
    values={key:components.number(row,key,unit) for key,unit in ROLES.items()}
    if not all(math.isfinite(value) and value>0 for value in values.values()):raise ValueError('turbofan: positive finite inputs required')
    engines=design.get('engine',[]);tanks=design.get('mass',{}).get('tank',[])
    config=design.get('store_propulsion',{})
    if len(engines)!=1 or engines[0]['type']!='turbofan' or len(tanks)!=1 or config.get('version')!=1:
        raise ValueError('turbofan: one authored engine/tank and versioned control settings required')
    authored={key:engines[0][key] for key in ('thrust_dry_kn','bypass_ratio','tsfc_dry')}
    authored.update(fuel_capacity_kg=tanks[0]['capacity'],fuel_cg_x_m=tanks[0]['position'][0],
                    dry_mass_kg=design['mass']['empty'],cruise_speed_ms=config['cruise_speed_ms'],governor_time_s=config['governor_time_s'])
    if any(values[key]!=float(authored[key]) for key in values):raise ValueError('turbofan: card/source configuration mismatch')
    if tanks[0].get('fill',1)!=1 or tanks[0]['position'][1:]!=[0.,0.]:raise ValueError('turbofan: source fuel must be full and axial')
    full=values['dry_mass_kg']+values['fuel_capacity_kg']
    if full!=float(card['parameters']['mass_kg']['value']):raise ValueError('turbofan: canonical launch mass differs from dry mass plus fuel')
    name=card['model'][7:]
    if row['bindings']['engine']!='Engines/'+name+'_engine0.xml' or row['bindings']['nozzle']!='Engines/'+name+'_nozzle0.xml':
        raise ValueError('turbofan: engine/nozzle binding mismatch')
    return values


def install(files,card,aircraft):
    values=inputs(card,aircraft.spec)
    if values is None:return files
    from . import propulsion,jsbsim,storebodies
    name=aircraft.name;root=ET.fromstring(files[name+'.xml'])
    engine=aircraft.engines[0];mass=aircraft.spec['mass']
    class Tanks:
        tanks=mass['tank']
        @staticmethod
        def fuel(t):return t['capacity']*t.get('fill',1.)
    old=root.find('propulsion');index=list(root).index(old);root.remove(old)
    root.insert(index,ET.fromstring(jsbsim.propulsion_xml(aircraft,Tanks(),[(name+'_engine0',name+'_nozzle0')])))
    balance=root.find('mass_balance');balance.find('emptywt').set('unit','KG');balance.find('emptywt').text='%.17g'%values['dry_mass_kg']
    inertia=dry_inertia(aircraft.spec)
    for axis,value in zip(('ixx','iyy','izz'),inertia):
        node=balance.find(axis);node.set('unit','KG*M2');node.text='%.17g'%value
    cg=balance.find("location[@name='CG']");cg.set('unit','M')
    for axis,value in zip('xyz',mass['empty_cg']):cg.find(axis).text='%.17g'%value
    files[name+'.xml']=ET.tostring(root,encoding='unicode')
    files['Engines/'+name+'_engine0.xml']=propulsion.turbofan_xml(engine)
    files['Engines/'+name+'_nozzle0.xml']=propulsion.nozzle_xml(engine.name)
    return files


def install_control(xml,card,design):
    values=inputs(card,design)
    if values is None:return xml
    root=ET.fromstring(xml);fcs=root.find('flight_control')
    prefix='fsim/cruise-governor-v1/'
    ET.SubElement(fcs,'property',value='1').text=prefix+'version'
    # Nominal longitudinal m*dv/dt=T*throttle-D. Pole placement gives
    # Kp=m/(T*tau), Ki=Kp/tau: s^2+s/tau+1/tau^2 (zeta=.5,
    # omega_n=1/tau), before native spool/lapse. Saturation freezes outward integration;
    # an inert master or empty tank resets it. Gains are derived, not fitted.
    full=values['dry_mass_kg']+values['fuel_capacity_kg']
    kp=full/(1000*values['thrust_dry_kn']*values['governor_time_s']);ki=kp/values['governor_time_s']
    channel=ET.SubElement(fcs,'channel',name='Native liquid-fuel cruise speed governor')
    error=ET.SubElement(channel,'fcs_function',name=prefix+'error-ms');function=ET.SubElement(error,'function')
    difference=ET.SubElement(function,'difference');ET.SubElement(difference,'value').text='%.17g'%values['cruise_speed_ms']
    product=ET.SubElement(difference,'product');ET.SubElement(product,'value').text='.3048';ET.SubElement(product,'property').text='velocities/vtrue-fps'
    trigger=ET.SubElement(channel,'switch',name=prefix+'integrator-trigger');ET.SubElement(trigger,'default',value='0')
    ET.SubElement(trigger,'test',value='-1',logic='OR').text='fcs/throttle-cmd-norm lt 0.5\npropulsion/tank[0]/contents-lbs le 0'
    test=ET.SubElement(trigger,'test',value='1',logic='OR')
    ET.SubElement(test,'test',logic='AND').text=prefix+'pid ge 1\n'+prefix+'error-ms gt 0'
    ET.SubElement(test,'test',logic='AND').text=prefix+'pid le 0\n'+prefix+'error-ms lt 0'
    pid=ET.SubElement(channel,'pid',name=prefix+'pid');ET.SubElement(pid,'input').text=prefix+'error-ms'
    ET.SubElement(pid,'kp').text='%.17g'%kp;ET.SubElement(pid,'ki',type='trap').text='%.17g'%ki;ET.SubElement(pid,'kd').text='0'
    ET.SubElement(pid,'trigger').text=prefix+'integrator-trigger'
    clip=ET.SubElement(pid,'clipto');ET.SubElement(clip,'min').text='0';ET.SubElement(clip,'max').text='1'
    command=ET.SubElement(channel,'fcs_function',name=prefix+'throttle');function=ET.SubElement(command,'function');product=ET.SubElement(function,'product')
    ET.SubElement(product,'property').text='fcs/throttle-cmd-norm';ET.SubElement(product,'property').text=prefix+'pid'
    ET.SubElement(command,'output').text='fcs/throttle-pos-norm'
    ET.indent(root,space='    ')
    return ET.tostring(root,encoding='unicode')
