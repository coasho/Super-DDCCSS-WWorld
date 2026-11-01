"""Finite equivalent vectoring nozzle, with explicit uncalibrated inputs."""
import math
import xml.etree.ElementTree as ET

BINDING='jsbsim_nozzle_v1'


def inputs(card):
    from . import components
    row=components.resolve(card,'autopilot')
    if row is None or 'thrust_vector' not in row['bindings']:return None
    if row['bindings']['thrust_vector']!=BINDING or row['bindings'].get('execution_mode')!='configured':
        raise ValueError('thrust vector: unsupported control binding')
    limit=components.number(row,'tvc_limit_rad','rad')
    rate=components.number(row,'tvc_rate_rad_s','rad/s')
    if not 0<limit<math.pi/2 or rate<=0:raise ValueError('thrust vector: finite positive angle/rate required')
    return dict(limit=limit,rate=rate)


def install(fcs,aircraft,control,card):
    selected=inputs(card)
    if selected is None:return
    from .storecontrol import _op as op,_property as prop,_value as val
    # Virtual axis signs follow the authored aerodynamic control moment.
    # Positive native nozzle pitch/yaw at an aft axial nozzle both produce
    # negative pitch/yaw torque (JSBSim FGForce's structural/body convention).
    gains={}
    for channel,key in (('elevator','cm-elevator'),('rudder','cn-rudder')):
        slopes=control['derivatives'][key]
        if not slopes or any(not math.isfinite(v) or v*slopes[0]<=0 for v in slopes):
            raise ValueError('thrust vector: signed aerodynamic authority must be nonsingular throughout the authored domain')
        low,high=control['channels'][channel]
        if low!=-high or high<=0:raise ValueError('thrust vector: explicit symmetric virtual axes required')
        gains[channel]=-math.copysign(selected['limit']/high,slopes[0])
    motor=aircraft.spec.get('motor')
    if motor is None or motor['exit'][1:]!=[0.,0.]:raise ValueError('thrust vector requires an authored axial motor/nozzle')
    prefix='fsim/thrust-vector-control/'
    metadata=dict(version=1,limit_rad=selected['limit'],rate_rad_s=selected['rate'],
        pitch_gain=gains['elevator'],yaw_gain=gains['rudder'],
        exit_x_body_m=float(aircraft.spec['mass']['empty_cg'][0])-float(motor['exit'][0]))
    for key,value in metadata.items():ET.SubElement(fcs,'property',value='%.17g'%value).text=prefix+key
    channel=ET.SubElement(fcs,'channel',name='Finite equivalent thrust-vector nozzle (engineering assumptions)')
    for axis,virtual in (('pitch','elevator'),('yaw','rudder')):
        name=prefix+axis
        function=ET.SubElement(channel,'fcs_function',name=name+'-demand-rad')
        ET.SubElement(function,'function').append(op('product',prop(prefix+axis+'_gain'),prop('fcs/'+virtual+'-response-rad')))
        actuator=ET.SubElement(channel,'actuator',name=name+'-actuator')
        ET.SubElement(actuator,'input').text=name+'-demand-rad'
        # The finite virtual-channel lag is already applied exactly once.
        ET.SubElement(actuator,'rate_limit').text=prefix+'rate_rad_s'
        clip=ET.SubElement(actuator,'clipto')
        ET.SubElement(clip,'min').text='%.17g'%-selected['limit']
        ET.SubElement(clip,'max').text='%.17g'%selected['limit']
        ET.SubElement(actuator,'output').text='propulsion/engine/'+axis+'-angle-rad'
