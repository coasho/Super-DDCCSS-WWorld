"""Physical rotor actuator evidence and explicitly estimated pitch servos.

Motor speed response remains the authored native ODE. No extra normalized
command slew is added. Helicopter servo options are a versioned project design,
not measured hardware data. All limits use physical angular units.
"""
import math
import xml.etree.ElementTree as ET

PREFIX='fsim/actuator-response-v1'
VERSION='fsim.rotor-actuator-model.v1'
DT=1/120


def _property(fcs,name,value):
    ET.SubElement(fcs,'property',value='%.17g'%value).text=PREFIX+'/'+name


def _rows(fcs,rows):
    _property(fcs,'version',1);_property(fcs,'count',len(rows))
    channel=ET.SubElement(fcs,'channel',name='Physical actuator evidence v1')
    for i,row in enumerate(rows):
        for key in ('kind','low','high','tau_min','tau_max','rate_bound'):_property(fcs,key+'[%d]'%i,row[key])
        node=ET.SubElement(channel,'fcs_function',name=PREFIX+'/state[%d]'%i)
        ET.SubElement(ET.SubElement(node,'function'),'property').text=row['state']


def install(root,spec):
    fcs=root.find('flight_control');kind=spec['aircraft']['kind'];rows=[]
    byname={n.get('name'):n for n in fcs.iter() if n.get('name')}
    if kind=='multirotor':
        from .rotorcraft.multi import rotors
        motors,_,_,_=rotors(spec);name=spec['aircraft']['name']
        for i in range(len(motors)):
            state='fcs/%s/omega[%d]'%(name,i)
            response=byname[state];command=byname[response.findtext('input').strip()]
            high=float(command.findtext('function/product/value'))
            poles=[float(n.text) for n in byname[response.findtext('c1').strip()].findall('function/ifthen/value')]
            if len(poles)!=2 or not all(math.isfinite(p) and p>0 for p in poles):raise ValueError('Missing bounded native motor lag branches')
            low_tau,high_tau=1/max(poles),1/min(poles)
            if DT>2*low_tau:raise ValueError('Native motor recurrence is not monotone at its declared step')
            rows.append(dict(kind=1,low=0.,high=high,tau_min=low_tau,tau_max=high_tau,rate_bound=high/low_tau,state=state))
    elif kind=='helicopter':
        config=spec.get('control_actuators',{})
        if not config:
            _rows(fcs,[]);return  # identity default; completeness audit rejects absent pitch servos
        if set(config)!={'version','method','phase_target_deg','cyclic_limit_deg','collective_limits_deg','pedal_limits_deg'} or type(config.get('version')) is not int or config['version']!=1 or config['method']!='phase-budget-servo':
            raise ValueError('Unsupported physical pitch actuator configuration')
        phase=float(config['phase_target_deg'])
        if not math.isfinite(phase) or not 0<phase<90:raise ValueError('Invalid actuator design phase target')
        from .rotorcraft import heli
        data=heli.rotor_data(spec);controls=spec['controls']
        flap=controls.get('flap_lag_s',data['flap_lag'])
        cyclic=math.radians(float(config['cyclic_limit_deg']))
        bounds={
            'longitudinal':(-cyclic,cyclic,flap),'lateral':(-cyclic,cyclic,flap),
            'collective':(*map(math.radians,map(float,config['collective_limits_deg'])),controls.get('collective_lag_s',flap)),
            'antitorque':(*map(math.radians,map(float,config['pedal_limits_deg'])),data['tr_flap_lag'])}
        for channel in list(fcs.findall('channel')):
            for source in list(channel):
                for output in list(source.findall('output')):
                    target=(output.text or '').strip()
                    if not target.startswith('propulsion/engine[') or not target.endswith('-ctrl-rad'):continue
                    role=target.rsplit('/',1)[1].removesuffix('-ctrl-rad')
                    if role not in bounds:raise ValueError('Unrecognized native rotor pitch route')
                    lo,hi,response_tau=bounds[role]
                    if not all(math.isfinite(v) for v in (lo,hi,response_tau)) or not lo<hi or response_tau<=0:raise ValueError('Missing finite physical pitch actuator bounds')
                    tau=max(DT,math.tan(math.radians(phase))*response_tau)
                    state=PREFIX+'/servo[%d]'%len(rows)
                    actuator=ET.Element('actuator',name=state)
                    ET.SubElement(actuator,'description').text='Estimated post-mix physical pitch servo; phase target %.9g deg, realized %.9g deg at native response pole; native first-frame initialization is not a running slew sample.'%(phase,math.degrees(math.atan(tau/response_tau)))
                    ET.SubElement(actuator,'input').text=source.get('name')
                    ET.SubElement(actuator,'lag').text='%.17g'%(1/tau)
                    ET.SubElement(actuator,'rate_limit').text='%.17g'%((hi-lo)/tau)
                    clip=ET.SubElement(actuator,'clipto');ET.SubElement(clip,'min').text='%.17g'%lo;ET.SubElement(clip,'max').text='%.17g'%hi
                    ET.SubElement(actuator,'output').text=target
                    source.remove(output);channel.insert(list(channel).index(source)+1,actuator)
                    rows.append(dict(kind=2,low=lo,high=hi,tau_min=tau,tau_max=tau,rate_bound=(hi-lo)/tau,state=state))
        if len(rows)!=4:raise ValueError('Missing one of four native helicopter pitch actuator routes')
    else:raise ValueError('Unsupported rotor actuator topology')
    _rows(fcs,rows)


def audit(root):
    """Check each physical state, bounds, positive lag and its actual routing."""
    fcs=root.find('flight_control');checks=[];rows=[]
    def check(name,passed):checks.append(dict(name='physical actuator/'+name,passed=bool(passed)))
    props={(n.text or '').strip():n.get('value') for n in fcs.findall('property')}
    def value(key):return float(props.get(PREFIX+'/'+key,'nan'))
    version,count=value('version'),value('count')
    valid=version==1 and math.isfinite(count) and count==int(count) and 0<count<=64
    check('explicit state inventory',valid)
    if not valid:return rows,checks
    byname={n.get('name'):n for n in fcs.iter() if n.get('name')}
    for i in range(int(count)):
        row={key:value(key+'[%d]'%i) for key in ('kind','low','high','tau_min','tau_max','rate_bound')}
        finite=all(math.isfinite(v) for v in row.values())
        check(str(i)+'/finite dimensions',finite)
        if not finite:continue
        check(str(i)+'/stops and lag',row['low']<row['high'] and 0<row['tau_min']<=row['tau_max'])
        expected=(row['high']-row['low'])/row['tau_min']
        check(str(i)+'/derived physical rate',row['rate_bound']==expected)
        read=byname.get(PREFIX+'/state[%d]'%i)
        state=read.findtext('function/property','').strip() if read is not None else ''
        node=byname.get(state);check(str(i)+'/native state binding',node is not None)
        if node is None:continue
        if row['kind']==1:
            command=byname.get(node.findtext('input','').strip())
            poles=byname.get(node.findtext('c1','').strip())
            values=[float(n.text) for n in poles.findall('function/ifthen/value')] if poles is not None else []
            check(str(i)+'/both native lag branches',node.tag=='lag_filter' and len(values)==2 and all(p>0 for p in values) and row['tau_min']==1/max(values) and row['tau_max']==1/min(values))
            high=float(command.findtext('function/product/value','nan')) if command is not None else math.nan
            check(str(i)+'/native speed stop',row['low']==0 and row['high']==high)
            clip_source=command.findtext('function/product/sqrt/property','').strip() if command is not None else ''
            clip=byname.get(clip_source)
            bounded=clip is not None and any(n.findtext('value','').strip()=='0' and n.findtext('min/value','').strip()=='1' for n in clip.findall('.//max'))
            check(str(i)+'/bounded motor command',bounded)
            check(str(i)+'/monotone recurrence',DT<=2*row['tau_min'])
            unit,rate_unit='rad/s','rad/s^2'
        elif row['kind']==2:
            check(str(i)+'/actual servo lag and slew',node.tag=='actuator' and float(node.findtext('lag','nan'))==1/row['tau_min'] and float(node.findtext('rate_limit','nan'))==row['rate_bound'])
            check(str(i)+'/actual mechanical stops',float(node.findtext('clipto/min','nan'))==row['low'] and float(node.findtext('clipto/max','nan'))==row['high'])
            target=node.findtext('output','').strip()
            writers=[n for n in fcs.iter('output') if (n.text or '').strip()==target]
            check(str(i)+'/sole native pitch writer',target.startswith('propulsion/engine[') and target.endswith('-ctrl-rad') and len(writers)==1)
            unit,rate_unit='rad','rad/s'
        else:check(str(i)+'/known physical response',False);continue
        rows.append(dict(row,state=state,readback=PREFIX+'/state[%d]'%i,unit=unit,rate_unit=rate_unit,basis='estimated',version=VERSION,
            method='Authored native bounded motor response' if row['kind']==1 else 'Explicit post-mix first-order pitch servo',
            derivation='Bounded first-order response; rate range/tau_min; native initialization treated separately',
            source='actuator_models.py; rotorcraft/multi.py or source control_actuators configuration',
            interpolation='Native discrete state propagation',extrapolation='Clipped mechanical/input bounds; no hardware calibration claim'))
    return rows,checks
