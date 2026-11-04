"""Finite actuator bindings for the common aircraft VLM tables.

No independent fin force, CP, slender-body or offline-table model is selected
here. The existing native actuator identities/limits/lag/mixers are preserved.
"""
import hashlib
import copy
import json
import math
import re
import xml.etree.ElementTree as ET
import numpy as np
from .equipment_models import normalize as equipment_mechanism

CHANNELS = ("elevator", "rudder", "aileron")
OUTPUTS = {"elevator": "elevator", "rudder": "rudder", "aileron": "left-aileron"}
LAG = 25.0
RATE = 4.0
FT = 0.3048

def _number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("missile control: %s must be finite numeric" % label)
    try:
        result = float(value)
    except (ValueError, OverflowError) as error:
        raise ValueError("missile control: %s must be finite numeric" % label) from error
    if not math.isfinite(result):
        raise ValueError("missile control: %s must be finite" % label)
    return result

def actuator_profile(card=None):
    """Only native actuator inputs; full execution validation belongs to MS.

    The content fingerprint identifies the values actually used by this FCS,
    rather than an unrelated warhead/source edit to the same card.
    """
    from . import components
    selected = components.actuator(card or {})
    if selected is not None:
        return selected
    parameters = (card or {}).get("parameters", {})
    if not isinstance(parameters, dict):
        raise ValueError("missile control: card parameters must be an object")
    if "weapon_execution_version" not in parameters:
        if any(name in parameters for name in ("control_actuator_lag_s", "control_actuator_rate_rad_s")):
            raise ValueError("missile control: actuator configuration needs weapon_execution_version")
        return {"version": 0, "lag_coefficient": LAG, "rate_rad_s": RATE}

    def parameter(name, unit):
        row = parameters.get(name)
        if not isinstance(row, dict) or row.get("unit") != unit:
            raise ValueError("missile control: %s needs explicit %s units" % (name, unit))
        return _number(row.get("value"), name)

    for name in ("weapon_execution_version", "control_algorithm", "control_allocator"):
        if parameter(name, "1") != 1:
            raise ValueError("missile control: unsupported " + name)
    lag = parameter("control_actuator_lag_s", "s")
    rate = parameter("control_actuator_rate_rad_s", "rad/s")
    if lag <= 0 or rate <= 0 or not math.isfinite(1 / lag):
        raise ValueError("missile control: actuator lag/rate must be positive representable native inputs")
    consumed = {"version": 1, "card": (card or {}).get("id", ""), "control_algorithm": 1,
                "control_allocator": 1, "actuator_lag_s": lag, "actuator_rate_rad_s": rate}
    digest = hashlib.sha256(json.dumps(consumed, sort_keys=True, separators=(",", ":")).encode("utf-8")).hexdigest()
    return {"version": 1, "lag_coefficient": 1 / lag, "rate_rad_s": rate, "sha256": digest}

def _value(x):
    node = ET.Element("value")
    node.text = "%.17g" % x
    return node

def _property(name):
    node = ET.Element("property")
    node.text = name
    return node

def _op(tag, *children):
    if tag == "sum" and len(children) < 2:
        return children[0] if children else _value(0)
    node = ET.Element(tag)
    node.extend(children)
    return node

def _table(mach, values):
    node = ET.Element("table")
    ET.SubElement(node, "independentVar", lookup="row").text = "velocities/mach"
    ET.SubElement(node, "tableData").text = "\n" + "\n".join("%.17g %.17g" % pair for pair in zip(mach, values)) + "\n"
    return node

def _function(name, expression):
    node = ET.Element("function", name=name, type="pre")
    node.append(expression)
    return node


def control_jacobian(tables, channel, mach, *, alpha=0., beta=0., direction=1):
    """Exact one-sided derivative of the emitted neutral fin interpolant.

    A zero knot can have distinct left/right slopes. Select each fin's
    interval using the signed mixer gain, rather than differentiating across
    that knot. This changes derivative metadata only, never table values.
    """
    if direction not in (-1,1):raise ValueError('Control derivative direction must be -1 or 1')
    from .verify import body_forces
    from .aero.tables import COEFFS
    coefficient={key:0. for key in COEFFS}
    mt=tables['mach'];grid=mt['mach']
    induced_factor=float(np.interp(mach,grid,mt['induced_factor']))
    for piece in tables.get('native_controls',[]):
        gain=piece['gains'].get(channel,0.)
        if not gain:continue
        movement=direction*gain
        if (movement>0 and piece['hi']<=0) or (movement<0 and piece['lo']>=0):continue
        knots=np.asarray(piece['deflection'],float)
        zero=int(np.searchsorted(knots,0.))
        if zero>=len(knots) or knots[zero]!=0.:raise ValueError('Physical fin table requires an explicit neutral knot')
        j=zero if movement>0 else zero-1
        if j<0 or j+1>=len(knots):raise ValueError('Physical fin derivative interval is unavailable')
        scale=gain*float(np.interp(mach,grid,mt['K_'+piece['channel']]))/(knots[j+1]-knots[j])
        values={key:scale*(float(np.interp(math.degrees(alpha),tables['alpha'],data[:,j+1]))-
                          float(np.interp(math.degrees(alpha),tables['alpha'],data[:,j])))
                for key,data in piece['values'].items()}
        for key in COEFFS:coefficient[key]+=values[key]
        coefficient['CD']+=values['CDi']*(induced_factor-1.)
    force,moment=body_forces(coefficient,alpha,beta,1.,1.,1.,1.)
    return np.array([-force[0],force[1],-force[2],*moment])


def describe(aircraft, tables, *, local=False):
    from . import rolleron
    tables=rolleron.without_authority(aircraft,tables)
    physical=tables.get('native_controls',[])
    if not physical:return None
    fins=[dict(id=p['id'],lo=p['lo'],hi=p['hi'],
               gains={ch:p['gains'].get(ch,0.) for ch in CHANNELS}) for p in physical]
    limits={ch:tuple(math.radians(v) for v in aircraft.channel_limits(ch))
            if ch in aircraft.channels() else (0.,0.) for ch in CHANNELS}
    for ch in CHANNELS:
        if not any(fin['gains'][ch] for fin in fins):limits[ch]=(0.,0.)
    from .airframe import _body
    grid=tables['mach']['mach'].tolist();h=1.e-4
    names={'cz-alpha':[], 'cy-beta':[], 'cm-alpha':[], 'cn-beta':[],
           'cm-q':[], 'cn-r':[], 'cl-p':[], 'cz-elevator':[], 'cy-rudder':[],
           'cm-elevator':[], 'cn-rudder':[], 'cl-aileron':[]}
    for mach in grid:
        alpha=(_body(tables,aircraft,h,mach=mach)-_body(tables,aircraft,-h,mach=mach))/(2*h)
        beta=(_body(tables,aircraft,beta=h,mach=mach)-_body(tables,aircraft,beta=-h,mach=mach))/(2*h)
        for key,value in [('cz-alpha',-alpha[2]),('cy-beta',beta[1]),('cm-alpha',alpha[4]),('cn-beta',beta[5])]:names[key].append(float(value))
        for rate,key,index in [('q','cm-q',4),('r','cn-r',5),('p','cl-p',3)]:
            v=(_body(tables,aircraft,mach=mach,**{rate:h})-_body(tables,aircraft,mach=mach,**{rate:-h}))/(2*h)
            names[key].append(float(v[index]))
        for channel,pairs in [('elevator',[('cz-elevator',2,-1),('cm-elevator',4,1)]),
                              ('rudder',[('cy-rudder',1,1),('cn-rudder',5,1)]),('aileron',[('cl-aileron',3,1)])]:
            v=control_jacobian(tables,channel,mach)
            for key,index,sign in pairs:names[key].append(float(v[index]*sign))
    result=dict(fins=fins,channels=limits,mach=grid,derivatives=names)
    if local:
        angles=np.radians(np.arange(-30.,30.01,2.)).tolist()
        local_values={key:[] for key in names if key not in ('cl-p','cl-aileron')}
        for angle in angles:
            current={key:[] for key in local_values}
            for mach in grid:
                for axis,force,moment,control,rate,sign in (
                        ('alpha',2,4,'elevator','q',-1),('beta',1,5,'rudder','r',1)):
                    kw={axis:angle,'mach':mach}
                    def value(**extra):return _body(tables,aircraft,**(kw|extra))
                    slope=(value(**{axis:angle+h})-value(**{axis:angle-h}))/(2*h)
                    for key,index,multiplier in ((('cz-alpha' if axis=='alpha' else 'cy-beta'),force,sign),
                            (('cm-alpha' if axis=='alpha' else 'cn-beta'),moment,1)):
                        current[key].append(float(multiplier*slope[index]))
                    delta=control_jacobian(tables,control,mach,**{axis:angle})
                    current['cz-elevator' if axis=='alpha' else 'cy-rudder'].append(float(sign*delta[force]))
                    current['cm-elevator' if axis=='alpha' else 'cn-rudder'].append(float(delta[moment]))
                    damping=(value(**{rate:h})-value(**{rate:-h}))/(2*h)
                    current['cm-q' if axis=='alpha' else 'cn-r'].append(float(damping[moment]))
            for key in local_values:local_values[key].append(current[key])
        result.update(angles=angles,local_derivatives=local_values)
    return result


def install(xml, aircraft, tables, card):
    implementation=(card or {}).get('component_models',{}).get('components',{}).get('autopilot',{}).get('implementation_id')
    if implementation:implementation=equipment_mechanism(card,'autopilot',implementation)
    operating=implementation=='fsim.control.operating_point_bounded.v1'
    local=operating or implementation=='fsim.control.actuator_aware_cascade.v1'
    control=describe(aircraft,tables,local=local)
    if control is None:return xml
    root=ET.fromstring(xml)
    old_controls=ET.tostring(root.find('flight_control'),encoding='unicode')
    origin_inches=float(aircraft.spec['mass']['empty_cg'][0])/.0254
    baseline=dict(S=aircraft.S,d=aircraft.c)
    from . import components
    components.control(card or {})
    dynamics = actuator_profile(card)
    lag, rate = dynamics["lag_coefficient"], dynamics["rate_rad_s"]
    # Controller derivatives and generated force/moment normalization must use
    # the same area and diameter, including on the older build-up path whose
    # passive-only XML rounded these metrics to five decimals.
    for tag, number in (("wingarea", baseline["S"] / FT ** 2),
                        ("wingspan", aircraft.b / FT), ("chord", baseline["d"] / FT)):
        xml = re.sub(r'(<%s unit="[A-Z0-9]+">)[^<]*(</%s>)' % (tag, tag),
                     lambda match: match[1] + " %.17g " % number + match[2], xml, count=1)
    fcs = ET.Element("flight_control", name="Finite missile controls (engineering estimates)")
    metadata = {"version": 2 if local else 1, "normalization-version": 1,
                "reference-version":1,"control-derivative-method":2,"control-derivative-direction":1,
                "aero-reference-x-m":-(float(aircraft.aero_point[0])-float(aircraft.spec['mass']['empty_cg'][0])),
                "surface-limit-rad": max(max(abs(v) for v in limits) for limits in control["channels"].values()),
                "surface-rate-rad_s": rate, "channel-rate-rad_s": rate, "lag-coefficient-s-1": lag,
                "fin-count": len(control["fins"])}
    if dynamics["version"] == 1:
        metadata["actuator-version"] = 1
    metadata.update({ch + "-limit-rad": max(abs(v) for v in bounds) for ch, bounds in control["channels"].items()})
    for channel, (low, high) in control["channels"].items():
        metadata[channel + "-min-rad"] = low
        metadata[channel + "-max-rad"] = high
    for i, fin in enumerate(control["fins"]):
        metadata["fin-%d-id" % i] = fin["id"]
        metadata["fin-%d-min-rad" % i] = fin["lo"]
        metadata["fin-%d-max-rad" % i] = fin["hi"]
        metadata["fin-%d-rate-rad_s" % i] = rate
    for name, number in metadata.items():
        ET.SubElement(fcs, "property", value="%.17g" % number).text = "fsim/missile-control/" + name
    for axis, number in zip("xyz", (origin_inches, 0, 0)):
        ET.SubElement(fcs, "property", value="%.17g" % number).text = "fsim/model/origin-%s-in" % axis
    channels = ET.SubElement(fcs, "channel", name="Finite channel actuators and authored fin mixers")
    for channel in CHANNELS:
        low, high = control["channels"][channel]
        scale = ET.SubElement(channels, "aerosurface_scale", name="fcs/%s-control" % channel)
        ET.SubElement(scale, "input").text = "fcs/%s-cmd-norm" % channel
        limits = ET.SubElement(scale, "range")
        ET.SubElement(limits, "min").text = "%.17g" % low
        ET.SubElement(limits, "max").text = "%.17g" % high
        actuator = ET.SubElement(channels, "actuator", name="fcs/%s-actuator" % channel)
        if operating:
            # Expose the native pre-rate lag state for the new predictor. Old
            # selections retain the original actuator XML and initialization.
            channels.remove(actuator)
            filter_node=ET.SubElement(channels,'lag_filter',name='fcs/%s-operating-lag'%channel)
            ET.SubElement(filter_node,'input').text='fcs/%s-control'%channel
            ET.SubElement(filter_node,'c1').text='fsim/missile-control/lag-coefficient-s-1'
            ET.SubElement(filter_node,'output').text='fcs/%s-lag-state'%channel
            channels.append(actuator)
            ET.SubElement(actuator,'input').text='fcs/%s-lag-state'%channel
        else:
            ET.SubElement(actuator, "input").text = "fcs/%s-control" % channel
            ET.SubElement(actuator, "lag").text = "fsim/missile-control/lag-coefficient-s-1" if dynamics["version"] else "%.17g" % lag
        ET.SubElement(actuator, "rate_limit").text = "fsim/missile-control/channel-rate-rad_s" if dynamics["version"] else "%.17g" % rate
        stops = ET.SubElement(actuator, "clipto")
        ET.SubElement(stops, "min").text = "%.17g" % low
        ET.SubElement(stops, "max").text = "%.17g" % high
        ET.SubElement(actuator, "output").text = "fcs/%s-response-rad" % channel
    from . import rolleron
    physical=[dict(row,_alpha=tables['alpha']) for row in tables.get('native_controls',[])]
    passive=rolleron.install(fcs,channels,aircraft,physical)
    if passive:
        for i,fin in enumerate(control['fins']):
            ET.SubElement(fcs,'property',value=str(2 if i in passive else 1)).text='fsim/missile-control/fin-%d-actuation-kind-v1'%i
        for i,mechanics in passive.items():
            for suffix,value in (('min-rad',-mechanics['limit-rad']),('max-rad',mechanics['limit-rad']),('rate-rad_s',mechanics['rate-rad-s'])):
                key='fsim/missile-control/fin-%d-%s'%(i,suffix)
                next(node for node in fcs.findall('property') if node.text==key).set('value','%.17g'%value)
    for i, fin in enumerate(control["fins"]):
        if i in passive:continue
        mixer = ET.SubElement(channels, "fcs_function", name="fcs/fin-%d-demand-rad" % i)
        expression = ET.SubElement(mixer, "function")
        expression.append(_op("sum", *(_op("product", _value(gain), _property("fcs/%s-response-rad" % ch))
            for ch, gain in fin["gains"].items() if gain)))
        stops = ET.SubElement(mixer, "clipto")
        ET.SubElement(stops, "min").text = "%.17g" % fin["lo"]
        ET.SubElement(stops, "max").text = "%.17g" % fin["hi"]
        actuator = ET.SubElement(channels, "actuator", name="fcs/fin-%d-actuator" % i)
        ET.SubElement(actuator, "input").text = "fcs/fin-%d-demand-rad" % i
        ET.SubElement(actuator, "rate_limit").text = "fsim/missile-control/fin-%d-rate-rad_s" % i if dynamics["version"] else "%.17g" % rate
        stops = ET.SubElement(actuator, "clipto")
        ET.SubElement(stops, "min").text = "%.17g" % fin["lo"]
        ET.SubElement(stops, "max").text = "%.17g" % fin["hi"]
        ET.SubElement(actuator, "output").text = "fcs/fin-%d-delta-rad" % i
    mixing = np.asarray([[fin["gains"][channel] for channel in CHANNELS] for fin in control["fins"]], dtype=float)
    equivalent_axes = np.linalg.pinv(mixing)
    equivalent_axes[np.abs(equivalent_axes) < 1e-12] = 0
    for channel, row in zip(CHANNELS, equivalent_axes):
        report = ET.SubElement(channels, "fcs_function", name="fcs/%s-position" % channel)
        expression = ET.SubElement(report, "function")
        expression.append(_op("sum", *(_op("product", _value(float(gain)),
            _property("fcs/fin-%d-delta-rad" % i)) for i, gain in enumerate(row) if gain)))
        ET.SubElement(report, "output").text = "fcs/%s-pos-rad" % OUTPUTS[channel]
    alias = ET.SubElement(channels, "pure_gain", name="fcs/aileron-position-alias")
    ET.SubElement(alias, "input").text = "fcs/left-aileron-pos-rad"
    ET.SubElement(alias, "gain").text = "1"
    ET.SubElement(alias, "output").text = "fcs/aileron-pos-rad"
    present={node.text for node in fcs.findall('property')}
    for old in ET.fromstring(old_controls).findall("property"):
        if not (old.text or "").startswith("fsim/missile-control/") and old.text not in present:
            fcs.append(old)
            present.add(old.text)
    from . import thrustvector
    thrustvector.install(fcs,aircraft,control,card or {})
    ET.indent(fcs, space="    ")
    fingerprint = "<!-- Native actuator card profile SHA-256: %s -->\n" % dynamics["sha256"] if dynamics["version"] else ""
    root=ET.fromstring(xml)
    previous=root.find('flight_control')
    position=list(root).index(previous)
    root.remove(previous)
    root.insert(position,fcs)
    if fingerprint:
        root.insert(position,ET.Comment('Native actuator card profile SHA-256: '+dynamics['sha256']))
    aero=root.find('aerodynamics')
    for name, values in control['derivatives'].items():
        table=_table(control['mach'],values)
        if name in control.get('local_derivatives',{}):
            table=ET.Element('table')
            ET.SubElement(table,'independentVar',lookup='row').text='aero/alpha-rad' if name.startswith(('cz','cm')) else 'aero/beta-rad'
            ET.SubElement(table,'independentVar',lookup='column').text='velocities/mach'
            ET.SubElement(table,'tableData').text='\n'+ ' '.join('%.17g'%v for v in control['mach'])+'\n'+ '\n'.join(
                ' '.join('%.17g'%v for v in [angle]+row) for angle,row in zip(control['angles'],control['local_derivatives'][name]))+'\n'
        aero.insert(0,_function('fsim/control/'+name,table))
    if operating:_operating_point(root,control,tables)
    ET.indent(root,space='    ')
    return ET.tostring(root,encoding='unicode')


def _operating_point(root,control,tables):
    """Expose unchanged native piecewise loads, never install another force."""
    fcs=root.find('flight_control');aero=root.find('aerodynamics')
    prefix='fsim/control-operating-point-v1/'
    ET.SubElement(fcs,'property',value='1').text=prefix+'version'
    ET.SubElement(fcs,'property',value=str(len(control['fins']))).text=prefix+'fin-count'
    def property_value(name,value):
        ET.SubElement(fcs,'property',value='%.17g'%value).text=prefix+name
    def sliced(expression,fin,knot):
        result=copy.deepcopy(expression)
        for parent in result.iter():
            for table in list(parent):
                if table.tag!='table':continue
                column=table.find("independentVar[@lookup='column']")
                if column is None or column.text!='fcs/fin-%d-delta-rad'%fin:continue
                rows=[list(map(float,line.split())) for line in table.findtext('tableData').splitlines() if line.strip()]
                static=ET.Element('table');variable=copy.deepcopy(table.find("independentVar[@lookup='row']"));static.append(variable)
                ET.SubElement(static,'tableData').text='\n'+'\n'.join('%.17g %.17g'%(row[0],row[knot+1]) for row in rows[1:])+'\n'
                parent.remove(table);parent.append(static)
        return result
    def expression(name):
        source=root.find(".//function[@name='%s']"%name)
        if source is None:raise ValueError('Missing physical operating-point source '+name)
        return next(child for child in source if child.tag!='description')
    induced=expression('aero/function/CDi-low')
    for i,fin in enumerate(control['fins']):
        stem='fin-%d/'%i;piece=tables['native_controls'][i];knots=list(map(float,piece['deflection']))
        property_value(stem+'knot-count',len(knots))
        for ch in CHANNELS:property_value(stem+'mix-'+ch,fin['gains'].get(ch,0.))
        for j,knot in enumerate(knots):
            property_value(stem+'knot-%d-rad'%j,knot)
            cdi_parts=[]
            for child in induced:
                if any(n.text=='fcs/fin-%d-delta-rad'%i for n in child.findall('.//independentVar')):
                    cdi_parts.append(sliced(child,i,j))
            cdi_name=prefix+stem+'knot-%d/CDi'%j
            aero.append(_function(cdi_name,_op('sum',*cdi_parts)))
            for key in ('CD','CY','CL','Cl','Cm','Cn'):
                value=sliced(expression('aero/coefficient/%s_fin_%d'%(key,i)),i,j)
                if key=='CD':
                    corrections=[]
                    for name in ('CD_induced_mach','CD_ground'):
                        part=copy.deepcopy(expression('aero/coefficient/'+name))
                        for node in part.findall('.//property'):
                            if node.text=='aero/function/CDi-low':node.text=cdi_name
                        corrections.append(part)
                    value=_op('sum',value,*corrections)
                aero.append(_function(prefix+stem+'knot-%d/'%j+key,value))
