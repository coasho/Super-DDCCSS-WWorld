"""One aircraft aerodynamic model/table/XML path for every authored airframe.

Physical mass, propulsion and actuator configurations remain separate inputs.
This module performs wiring and normalization; force calculations belong solely
to aero.model, aero.tables, aero.mach and the common JSBSim writer.
"""
import hashlib
import json
import math
from pathlib import Path
import re
import tomllib
import xml.etree.ElementTree as ET

import numpy as np

from . import jsbsim, pipeline
from .aero import tables as table_methods
from .files import atomic_write_text
from .verify import TableModel, body_forces


def design(path, log=lambda message: None, spec=None):
    from .geometry import Aircraft
    d=pipeline.Design(str(path), log=log,
                      aircraft=Aircraft(spec,str(path)) if spec is not None else None)
    if spec is not None:
        calibration=Path(path).parent/'calibration.toml'
        if calibration.is_file():
            d.aircraft.calibration=tomllib.loads(calibration.read_text(encoding='utf-8')).get('calibration',{})
    seen=set()
    for surface in d.aircraft.surfaces:
        for control in surface.controls:
            key=(surface.name,control.name)
            if key in seen:raise ValueError('Duplicate physical fin identity: '+str(key))
            seen.add(key)
    return d


def _body(table, aircraft, alpha=0., beta=0., mach=0., **kwargs):
    coefficient = TableModel(table).evaluate(math.degrees(alpha), math.degrees(beta), mach=mach, aspect_ratio=aircraft.b**2/aircraft.S, **kwargs)
    force, moment = body_forces(coefficient, alpha, beta, 1., 1., aircraft.b, aircraft.c)
    return np.array([-force[0], force[1], -force[2], moment[0]/aircraft.b,
                     moment[1]/aircraft.c, moment[2]/aircraft.b])


def coefficients(path, *, table=None):
    d = design(path)
    table = d.calibrated(d.tables()) if table is None else table
    a = d.aircraft
    grid = table['mach']['mach'].tolist()
    result = dict(S=a.S, d=a.c, ref=float(a.aero_point[0]), mach=grid,
                  method='vlm', vlm_panels=int(table.get('_panels', 0)))
    for key in ('ca', 'cna', 'cma', 'cyb', 'cnb', 'cmq', 'cnr', 'clp'):
        result[key] = []
    h = 1.e-4
    for mach in grid:
        pitch = (_body(table,a,h,mach=mach)-_body(table,a,-h,mach=mach))/(2*h)
        yaw = (_body(table,a,beta=h,mach=mach)-_body(table,a,beta=-h,mach=mach))/(2*h)
        result['ca'].append(float(_body(table,a,mach=mach)[0]))
        for key, value in (('cna',pitch[2]),('cma',pitch[4]),('cyb',-yaw[1]),('cnb',-yaw[5])):
            result[key].append(float(value))
        for rate, key, axis in (('p','clp',3),('q','cmq',4),('r','cnr',5)):
            value = (_body(table,a,mach=mach,**{rate:h})-_body(table,a,mach=mach,**{rate:-h}))/(2*h)
            result[key].append(float(value[axis]))
    return result


def stage(d, force=False):
    table = d.tables(force=force)
    a = d.aircraft
    values = coefficients(d.path, table=table)
    return d.save('aero', dict(method='vlm', kind='airframe',
        reference=dict(area_m2=a.S, span_m=a.b, chord_m=a.c, origin_m=a.aero_point.tolist()),
        grid=dict(alpha=table['alpha'].tolist(), beta=table['beta'].tolist(), mach=values['mach']),
        physical_controls=len(table.get('native_controls', [])),
        derivatives=table_methods.derivatives(table), coefficients=values,
        checks=[pipeline.info('aerodynamic model', 'shared aircraft VLM'),
                pipeline.info('lifting panels', values['vlm_panels']),
                pipeline.info('base angle conditions', len(table['alpha'])*len(table['beta']))]))


def portable(table, aircraft, origin):
    def plain(value):
        if isinstance(value, dict):return {str(k):plain(v) for k,v in value.items()}
        if isinstance(value, np.ndarray):return value.tolist()
        if isinstance(value, np.generic):return value.item()
        if isinstance(value, (list,tuple)):return [plain(v) for v in value]
        return value
    # Carried stores are stowed at neutral. Released models include the complete
    # physical-actuator response in XML; both use the same base/rate/Mach data.
    selected={key:table[key] for key in ('alpha','beta','base','rates','alphadot','ground_effect','mach')}
    return dict(schema='fsim.airframe-aerodynamics.v1',method='vlm',
        reference=dict(area_m2=aircraft.S,span_m=aircraft.b,chord_m=aircraft.c,
                       aero_point_body_m=(-np.asarray(aircraft.aero_point-origin)*np.array([1.,-1.,1.])).tolist()),
        tables=plain(selected))


def install(xml, path, *, card=None, spec=None, propulsion_body_loads=False):
    d=design(path,spec=spec);a=d.aircraft;table=d.calibrated(d.tables())
    aerodynamic=jsbsim.aerodynamics_xml(table,a)
    root=ET.fromstring(xml)
    old=root.find('aerodynamics')
    if old is not None:root.remove(old)
    root.append(ET.fromstring(aerodynamic))
    if propulsion_body_loads:
        # Capture only the shared airframe wrench, before native rotor loads
        # are appended. Rotor reaction torque is control authority and must
        # never be cancelled as an airframe disturbance by the allocation.
        aero=root.find('aerodynamics')
        from .storecontrol import _op,_property
        for axis in ('DRAG','SIDE','LIFT','ROLL','PITCH','YAW'):
            terms=[_property(f.get('name')) for f in aero.findall("axis[@name='%s']/function"%axis)]
            fn=ET.SubElement(aero,'function',name='fsim/rotor-airframe-v1/'+axis.lower(),type='post')
            fn.append(_op('sum',*terms))
        fcs=root.find('flight_control')
        ET.SubElement(fcs,'property',value='1').text='fsim/rotor-airframe-v1/version'
    if propulsion_body_loads and old is not None:
        if _propulsion_loads(root.find('aerodynamics'),old):
            ET.SubElement(root.find('flight_control'),'property',value='1').text='fsim/rotor-passive-v1/version'
    metrics=root.find('metrics')
    for tag,value in (('wingarea',a.S/.3048**2),('wingspan',a.b/.3048),('chord',a.c/.3048)):
        node=metrics.find(tag);node.text='%.17g'%value
    arp=metrics.find("location[@name='AERORP']")
    if arp is None:arp=ET.SubElement(metrics,'location',name='AERORP')
    arp.set('unit','M')
    for key,value in zip('xyz',a.aero_point):
        node=arp.find(key)
        if node is None:node=ET.SubElement(arp,key)
        node.text='%.17g'%value
    fcs=root.find('flight_control')
    if fcs is None:fcs=ET.SubElement(root,'flight_control',name='Airframe actuator inputs')
    if a.spec.get('aircraft',{}).get('kind') in ('helicopter','multirotor'):
        from . import actuator_models
        actuator_models.install(root,spec or a.spec)
    ET.indent(root,space='    ')
    xml=ET.tostring(root,encoding='unicode')
    if card is not None:
        from . import storecontrol,components,storecontacts
        xml=storecontrol.install(xml,a,table,card)
        xml=storecontacts.install(xml,a)
        xml=components.stamp(xml,card)
    from . import integration
    return integration.install(xml,a,table)


def _rotor_command_actuators(fcs, *, additional_lag_s=0.1):
    """Estimated command shaper v1: explicit lag, slew and physical stops.

    A standalone servo defaults to a 0.1 s engineering response. The physical
    rotor pipeline supplies zero additional lag, retaining its existing native
    rotor/motor response states rather than adding an unmeasured second pole.
    The common slew is 2 normalized units/s. No design-name gain is used.
    """
    if not math.isfinite(additional_lag_s) or additional_lag_s<0:
        raise ValueError('Additional command lag must be finite and nonnegative')
    ET.SubElement(fcs,'property',value='%.17g'%additional_lag_s).text='fsim/engineering-actuator-v1/additional-lag-s'
    commands=set()
    for node in fcs.iter():
        text=(node.text or '').strip()
        if node.tag in ('property','input') and (text in ('fcs/aileron-cmd-norm','fcs/elevator-cmd-norm','fcs/rudder-cmd-norm') or
                text.startswith('fcs/throttle-cmd-norm')):
            commands.add(text)
            node.text='fsim/engineering-actuator-v1/'+text[4:]
    channel=ET.Element('channel',name='Engineering command actuators v1 (estimated)')
    for command in sorted(commands):
        actuator=ET.SubElement(channel,'actuator',name='fsim/engineering-actuator-v1/'+command[4:])
        ET.SubElement(actuator,'input').text=command
        # In JSBSim, <lag> is a pole rate, not a time constant. A zero pole
        # freezes the first sample; identity means omitting the lag element.
        # Record the explicit zero time constant above instead of emitting
        # an invalid zero-pole interpretation.
        if additional_lag_s:
            ET.SubElement(actuator,'lag').text='%.17g'%(1./additional_lag_s)
        ET.SubElement(actuator,'rate_limit').text='2'
        clip=ET.SubElement(actuator,'clipto')
        ET.SubElement(clip,'min').text='0' if 'throttle' in command else '-1'
        ET.SubElement(clip,'max').text='1'
    fcs.insert(0,channel)


def _propulsion_loads(aero, old):
    """Keep physical rotor loads while expressing body forces in wind axes.

    JSBSim prohibits mixing X/Y/Z with DRAG/SIDE/LIFT. This is a coordinate
    transform of the existing rotor propulsion loads, not an airframe model.
    """
    from copy import deepcopy
    from .storecontrol import _op,_property,_value
    body={key:[] for key in 'XYZ'}
    passive={key:[] for key in 'XYZ'}
    retained=[]
    for node in old:
        if node.tag!='axis':retained.append(deepcopy(node));continue
        axis=node.get('name')
        for fn in node:
            if axis in body:
                copied=deepcopy(fn);copied.set('type','pre');retained.append(copied)
                body[axis].append(_property(fn.get('name')))
                if fn.findtext('description','').strip()=='fsim.passive-rotor-force.v1':
                    passive[axis].append(_property(fn.get('name')))
            else:
                destination=aero.find("axis[@name='%s']"%axis)
                if destination is None:destination=ET.SubElement(aero,'axis',name=axis)
                destination.append(deepcopy(fn))
    # JSBSim caches pre-functions in evaluation order. Repeated insert(0)
    # reversed provider/consumer chains and introduced an extra frame delay.
    # Preserve the authored sequence, including its existing state semantics.
    aero[:0]=retained
    x,y,z=(_op('sum',*body[key]) for key in 'XYZ')
    ca=lambda:_op('cos',_property('aero/alpha-rad'))
    sa=lambda:_op('sin',_property('aero/alpha-rad'))
    cb=lambda:_op('cos',_property('aero/beta-rad'))
    sb=lambda:_op('sin',_property('aero/beta-rad'))
    loads={'DRAG':_op('product',_value(-1),_op('sum',_op('product',deepcopy(x),ca(),cb()),
            _op('product',deepcopy(y),sb()),_op('product',deepcopy(z),sa(),cb()))),
        'SIDE':_op('sum',_op('product',_value(-1),deepcopy(x),ca(),sb()),_op('product',deepcopy(y),cb()),
                   _op('product',_value(-1),deepcopy(z),sa(),sb())),
        'LIFT':_op('sum',_op('product',deepcopy(x),sa()),_op('product',_value(-1),deepcopy(z),ca()))}
    for axis,expression in loads.items():
        fn=ET.SubElement(aero.find("axis[@name='%s']"%axis),'function',name='aero/propulsion/'+axis.lower())
        fn.append(expression)
    present=any(passive.values())
    if present:
        for axis,terms in passive.items():
            fn=ET.SubElement(aero,'function',name='fsim/rotor-passive-v1/'+axis.lower()+'-lbs',type='post')
            fn.append(_op('sum',*terms))
    return present


def publish_store(card, d, table):
    root=Path(pipeline.repo_root())
    name=card['model'].removeprefix('jsbsim:')
    origin=np.asarray(d.aircraft.spec['mass']['empty_cg'],float)
    payload=portable(table,d.aircraft,origin)
    text=json.dumps(payload,sort_keys=True,separators=(',',':'),allow_nan=False)+'\n'
    digest=hashlib.sha256(text.encode()).hexdigest()
    relative='aero/vlm-carriage-'+digest+'.json'
    destination=root/'aircraft/stores'/name/relative
    destination.parent.mkdir(parents=True,exist_ok=True)
    atomic_write_text(destination,text)
    card.setdefault('sources',{})['hangar-vlm']=dict(
        title='Shared Hangar aircraft VLM tables',url=relative,evidence_kind='game_reference',
        note='Same base, rate and Mach data as the released JSBSim model; no equipment calibration claim.')
    card.setdefault('parameters',{})['airframe_aerodynamics_version']=dict(
        value=1,unit='1',source='hangar-vlm',basis='derived',uncertainty=0,
        note='Versioned shared-table runtime binding; zero uncertainty is not an accuracy claim.')
    point=_body(table,d.aircraft,mach=.3)
    card['parameters']['drag_area_m2']=dict(value=float(point[0]*d.aircraft.S),unit='m2',
        source='hangar-vlm',basis='derived',uncertainty=0,
        note='Neutral Mach .3 drag area from the shared model; descriptive, not a separate force model.')
    for key in ('munk_volume_m3','side_slope_m2','side_arm_m','drag_cd'):
        card['parameters'].pop(key,None)
    atomic_write_text(root/'assets/stores'/(card['id']+'.json'),json.dumps(card,indent=2,ensure_ascii=False)+'\n')
    # Only generated, content-addressed carriage products in this exact model
    # directory are eligible for retirement after the new card is published.
    for old in destination.parent.glob('vlm-carriage-*.json'):
        if old!=destination and re.fullmatch(r'vlm-carriage-[0-9a-f]{64}\.json',old.name) and not old.is_symlink():
            old.unlink()
