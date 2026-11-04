"""SIM-M02: one bounded geometry/card -> tables -> SDK engineering campaign.

Measurement calibration is deliberately not an admission criterion. No design
name is used in the acceptance rules. Workers are disposable deadline units.
"""
import argparse
import csv
from concurrent.futures import ThreadPoolExecutor
import hashlib
import html
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback
import xml.etree.ElementTree as ET

import numpy as np

from . import pipeline
from .aero.tables import COEFFS

VERSION = 'fsim.sim-m02.engineering.v1'
DEADLINE_SECONDS = 7200  # Operational bound, not the 1,200-second reporting target.
CAMPAIGN_TIMEOUT_SECONDS = DEADLINE_SECONDS
ROOT = Path(pipeline.repo_root())
SOURCES = {
    'vlm': {'url': 'https://web.mit.edu/drela/Public/web/avl/avl_doc.txt',
            'title': 'Drela and Youngren, AVL 3.40 User Primer',
            'scope': 'VLM, nondimensional rotation derivatives, quasi-steady and PG limitations; independent method reference, not the solver used'},
    'axes': {'url': 'https://jsbsim-team.github.io/jsbsim-reference-manual/user/concepts/frames-of-reference/',
             'title': 'JSBSim reference manual, frames of reference', 'scope': 'body, wind, structural frames'},
}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False, default=lambda x:
                    x.tolist() if isinstance(x, np.ndarray) else x.item() if isinstance(x, np.generic) else str(x)) + '\n', encoding='utf-8')


def declared_component_state(descriptor):
    """Project declarations decide applicability; missing code never waives it.

    Only an explicit versioned not_applicable source declaration means that
    this configuration does not install the component. Text saying 'inert'
    and an unavailable implementation cannot create that declaration.
    """
    availability=descriptor['availability']
    if availability not in ('available','unavailable','not_applicable'):
        raise ValueError('Unknown declared component availability')
    required=availability!='not_applicable'
    present=availability=='available'
    return dict(applicability='required' if required else 'not_applicable',
                present=present,availability=availability), not required or present


def engineering_status(failed_checks, *, timed_out=False, calibration_complete=False):
    """Both complete labels are admitted; optional measurement is not a gate."""
    if timed_out or failed_checks:return 'blocked'
    return 'engineering-complete' if calibration_complete else 'engineering-complete-uncalibrated'


def roster():
    result = []
    for folder in sorted((ROOT / 'aircraft').iterdir()):
        if folder.name.casefold() == '.codex':
            continue
        path = folder / (folder.name + '.toml')
        if path.is_file():
            import tomllib
            spec = tomllib.loads(path.read_text(encoding='utf-8'))
            identity = spec['aircraft']
            category = ('store' if identity.get('category') in ('store', 'missile') else
                        'rotorcraft' if identity.get('kind') in ('helicopter', 'multirotor') else 'fixed-wing')
            result.append(dict(name=folder.name, path=str(path), category=category))
    counts = {key: sum(r['category'] == key for r in result) for key in ('fixed-wing', 'rotorcraft', 'store')}
    if counts != {'fixed-wing': 31, 'rotorcraft': 4, 'store': 53}:
        raise ValueError('Incomplete declared roster: ' + str(counts))
    return result


def cohort(record):
    if record['category']!='store':return record['category']
    import tomllib
    identity=tomllib.loads(Path(record['path']).read_text(encoding='utf-8'))['aircraft']
    card=json.loads((ROOT/'assets/stores'/(identity['card']+'.json')).read_text(encoding='utf-8'))
    return 'aam' if card['class']=='aam' else 'stores'


def table_audit(t, a):
    """All shapes are derived from declared axes; no missing column is zero-filled."""
    inventory = []
    checks = []

    def check(name, ok, details=None):
        checks.append(dict(name=name, passed=bool(ok), details=details))

    def grid(name, values):
        v = np.asarray(values, float)
        check(name, v.ndim == 1 and len(v) >= 2 and np.isfinite(v).all() and np.all(np.diff(v) > 0))
        if not checks[-1]['passed']:
            raise ValueError('Missing valid required grid: ' + name)
        return len(v)

    def table(name, values, shape, method, basis, derivation, axes):
        v = np.asarray(values, float)
        check(name, v.shape == shape and np.isfinite(v).all(), {'shape': list(v.shape), 'required': list(shape)})
        inventory.append(dict(name=name, shape=list(v.shape), value_count=int(v.size), axes=axes, method=method, basis=basis,
                              version=VERSION, derivation=derivation, source=['vlm', 'axes'],
                              interpolation='linear in 1D; bilinear in 2D; additive first-order derivative build-up',
                              extrapolation='angle/Mach/control/height table lookup clamps to end knots; rates and alpha-dot continue linearly',
                              minimum=float(v.min()) if v.size and np.isfinite(v).all() else None,
                              maximum=float(v.max()) if v.size and np.isfinite(v).all() else None))

    na, nb = grid('alpha grid', t['alpha']), grid('beta grid', t['beta'])
    check('full numerical angle domain', t['alpha'][0] == -180 and t['alpha'][-1] == 180 and t['beta'][0] == -90 and t['beta'][-1] == 90)
    axes = {'alpha_deg': [float(t['alpha'][0]), float(t['alpha'][-1])], 'beta_deg': [float(t['beta'][0]), float(t['beta'][-1])]}
    for k in (*COEFFS, 'CDi'):
        table('base/' + k, t['base'][k], (na, nb), 'shared VLM + body/friction/separated-flow engineering closure', 'simulated',
              'aero/model.py:AeroModel.evaluate; aero/tables.py:build; VLM circulation plus explicit estimated closures', axes)
    # Export the requested body-axis force coefficients as derived data, while
    # retaining native wind-axis tables as the single execution authority.
    al, be = np.radians(t['alpha'])[:, None], np.radians(t['beta'])[None, :]
    cd, cy, cl = (t['base'][k] for k in ('CD','CY','CL'))
    body = dict(CA=cd*np.cos(al)*np.cos(be)+cy*np.cos(al)*np.sin(be)-cl*np.sin(al),
                CN=cd*np.sin(al)*np.cos(be)+cy*np.sin(al)*np.sin(be)+cl*np.cos(al),
                CY=-cd*np.sin(be)+cy*np.cos(be), **{k:t['base'][k] for k in ('Cl','Cm','Cn')})
    for k, values in body.items():
        table('body-base/'+k, values, (na,nb), 'exact wind-to-body coordinate transform', 'simulated',
              'CA=-Fx/qS, CN=-Fz/qS, CY=Fy/qS from native wind axes; moments retain body axes', axes)
    t['body_base'] = body
    for ch in a.channels():
        ctl = t['controls'][ch]
        nd = grid('control grid/' + ch, ctl['deflection'])
        lo, hi = a.channel_limits(ch)
        check('control boundary/' + ch, ctl['deflection'][0] <= lo and ctl['deflection'][-1] >= hi)
        for k in (*COEFFS, 'CDi'):
            table('control/' + ch + '/' + k, ctl[k], (na, nd), 'VLM control increment at beta=0', 'simulated',
                  'evaluate(alpha,delta)-evaluate(alpha,0); common superposition, no cross-term claim', axes | {'deflection_deg': [lo, hi]})
    for i, ctl in enumerate(t.get('native_controls', [])):
        nd = grid('physical control grid/' + str(i), ctl['deflection'])
        for k in (*COEFFS, 'CDi'):
            table('physical-control/' + str(i) + '/' + k, ctl['values'][k], (na, nd), 'individual VLM fin increment', 'simulated',
                  'aero/tables.py:build, same physical fin registry and mixer used by native JSBSim', axes | {'deflection_rad': [ctl['lo'], ctl['hi']]})
    for rate in ('p', 'q', 'r'):
        for k in (*COEFFS, 'CDi'):
            table('rate/' + rate + '/' + k, t['rates'][rate][k], (na,), 'VLM central difference', 'simulated',
                  'dC/d(rate_hat)=[C(+0.03)-C(-0.03)]/0.06; existing despike consistency rule retained',
                  axes | {'rate_hat_reference_interval': [-.03, .03]})
    for k in COEFFS:
        table('alpha-dot/' + k, [t['alphadot'][k]], (1,), 'first-order tail downwash lag; quasi-steady zero closure otherwise', 'estimated',
              'aero/tables.py:alphadot; CLad=2*CLat*St/S*lt/c*deps, Cmad=-CLad*lt/c; remaining four derivatives explicitly zero',
              {'alpha_dot_hat': [-.03, .03], 'continuation': 'linear, no high-frequency fidelity claim'})
    nm = grid('Mach grid', t['mach']['mach'])
    top = float(a.spec.get('analysis', {}).get('max_mach', .9 if not any(e.type == 'turbofan' for e in a.engines) else 2.6))
    check('declared Mach domain', t['mach']['mach'][0] == 0 and t['mach']['mach'][-1] >= top, {'declared': [0, top]})
    for key, values in t['mach'].items():
        if key != 'mach':
            scalar = np.asarray(values).ndim == 0
            table('Mach/' + key, [values] if scalar else values, (1,) if scalar else (nm,), 'subsonic PG VLM; transonic fairing; supersonic linear/slender-body engineering estimate', 'estimated',
                  'aero/mach.py:mach_effects and _drag; physical geometry parameters, no new fitted scales', {'Mach': [0, float(t['mach']['mach'][-1])]})
    ng = grid('height/span grid', t['ground_effect']['h_b'])
    for key in ('lift', 'drag'):
        table('ground-effect/' + key, t['ground_effect'][key], (ng,), 'McCormick-form engineering ground-effect closure', 'estimated',
              'aero/tables.py:ground_effect; drag=(16*h/b)^2/(1+(16*h/b)^2), lifting-surface-weighted lift; body-only identity',
              {'height/span': [float(t['ground_effect']['h_b'][0]), float(t['ground_effect']['h_b'][-1])]})
    # A shared held-out interpolation suite checks every design at the same
    # nondimensional points, including boundaries and linear rate continuation.
    from .verify import TableModel, body_forces
    tm = TableModel(t)
    for al, be, mh, rate in [(-180, -90, 0, -.03), (180, 90, top, .03), (0, 0, top/2, 0), (7.25, -2.5, top*.37, .015), (-11.75, 4.5, top*.73, -.015)]:
        c = tm.evaluate(al, be, p=rate, q=rate, r=rate, adot=rate, mach=mh)
        f, m = body_forces(c, math.radians(al), math.radians(be), 1, 1, a.b, a.c)
        check('common reference point/' + str((al, be, mh, rate)), np.isfinite(list(c.values())).all() and np.isfinite(f).all() and np.isfinite(m).all())
    def equal(left,right):
        return all(math.isclose(left[k],right[k],rel_tol=1e-12,abs_tol=1e-12) for k in COEFFS)
    for axis,beyond,boundary in [('alpha_deg',-181.,-180.),('alpha_deg',181.,180.),('beta_deg',-91.,-90.),('beta_deg',91.,90.),('mach',top+10.,float(t['mach']['mach'][-1]))]:
        state=dict(alpha_deg=7.25,beta_deg=-2.5,mach=0.)
        state[axis]=beyond;outside=tm.evaluate(**state)
        state[axis]=boundary;edge=tm.evaluate(**state)
        check('reference endpoint clamp/'+axis+'/'+str(beyond),equal(outside,edge))
    for rate in ('p','q','r'):
        plus=tm.evaluate(0,0,**{rate:.03});minus=tm.evaluate(0,0,**{rate:-.03})
        expected={k:float(np.interp(0,t['alpha'],t['rates'][rate][k])) for k in COEFFS}
        # The reference retains the declared clamped ground-effect endpoint;
        # it must include its circulation-only drag contribution as well.
        expected['CD']+=float(np.interp(0,t['alpha'],t['rates'][rate]['CDi']))*(float(t['ground_effect']['drag'][-1])-1.)
        observed={k:(plus[k]-minus[k])/.06 for k in COEFFS}
        check('reference central rotation derivative/'+rate,equal(observed,expected))
    check('reference orthogonal force transformation',np.allclose(body['CA']**2+body['CN']**2+body['CY']**2,cd**2+cl**2+cy**2,rtol=1e-12,atol=1e-12))
    return inventory, checks


def _operating_point_actuator_audit(root):
    """Validate the actual versioned serial FCS, without adding a second lag.

    The controller reads these scalar states/bindings. A positive unrelated
    filter, named function, or another healthy channel cannot satisfy them.
    """
    try:
        fcs=root.find('flight_control')
        if fcs is None:raise ValueError('Missing native FCS')
        properties={}
        for node in fcs.findall('property'):
            properties.setdefault((node.text or '').strip(),[]).append(node)
        def scalar(text):
            text=(text or '').strip()
            try:value=float(text)
            except ValueError:
                nodes=properties.get(text,[])
                if len(nodes)!=1:raise ValueError('Missing/ambiguous scalar property '+text)
                value=float(nodes[0].get('value','nan'))
            if not math.isfinite(value):raise ValueError('Nonfinite native actuator scalar '+text)
            return value
        def property_value(name):
            nodes=properties.get(name,[])
            if len(nodes)!=1:raise ValueError('Missing/ambiguous scalar property '+name)
            value=float(nodes[0].get('value','nan'))
            if not math.isfinite(value):raise ValueError('Nonfinite native actuator property '+name)
            return value
        def positive(value,label):
            if value<=0 or not math.isfinite(1/value):raise ValueError('Nonpositive/unrepresentable '+label)
            return value
        def leaf(node):
            if node is None or len(node) or node.attrib or not (node.text or '').strip():
                raise ValueError('Malformed native scalar/input leaf')
            return node.text.strip()
        def number(node):
            value=float(leaf(node))
            if not math.isfinite(value):raise ValueError('Nonfinite normalization/mixer literal')
            return value
        def shape(node,required,optional=(),attributes=()):
            if node is None:raise ValueError('Missing version1 native component/child')
            if set(node.attrib)-set(attributes) or (node.text or '').strip():
                raise ValueError('Unsupported version1 native component shape '+node.tag)
            tags=[child.tag for child in node]
            if set(tags)-set(required)-set(optional) or any(tags.count(tag)!=1 for tag in required) or any(tags.count(tag)>1 for tag in optional):
                raise ValueError('Unsupported/duplicate version1 child in '+node.tag)
        def component(tag,name):
            nodes=[node for node in fcs.iter() if node.get('name')==name]
            if len(nodes)!=1 or nodes[0].tag!=tag:raise ValueError('Missing/ambiguous '+name)
            return nodes[0]
        def output(node,name,required=True):
            explicit=node.find('output')
            if (required or explicit is not None) and leaf(explicit)!=name:raise ValueError('Broken native output '+name)
            writers=[n for n in fcs.iter() if n is not node and (n.get('name')==name or any((o.text or '').strip()==name for o in n.findall('output')))]
            if writers:raise ValueError('Ambiguous native output '+name)
        parents={child:parent for parent in fcs.iter() for child in parent}
        order={node:index for index,node in enumerate(n for ch in fcs.findall('channel') for n in ch)}
        def serial(*nodes):
            parent=parents.get(nodes[0])
            if any(parents.get(node) is not parent for node in nodes) or parent.tag!='channel' or set(parent.attrib)-{'name'}:
                raise ValueError('Disconnected/conditional native actuator stages')
            if any(order[a]>=order[b] for a,b in zip(nodes,nodes[1:])):raise ValueError('Reversed native actuator stage order')
        def bounds(node,container,low,high,label,literal=False):
            bound=node.find(container);shape(bound,('min','max'))
            values=[number(bound.find(key)) if literal else scalar(leaf(bound.find(key))) for key in ('min','max')]
            if values!=[low,high]:raise ValueError('Native stop/scale mismatch '+label)
        def normalization(scale,low,high,label):
            # FGGain.cpp admits other mappings. This version1 predictor is
            # specifically zero-centred [-1,1], unit gain, with signed range.
            shape(scale,('input','range'),('domain','gain','zero_centered','output','description'),('name',))
            if scale.find('domain') is not None:bounds(scale,'domain',-1.,1.,label,True)
            if scale.find('gain') is not None and number(scale.find('gain'))!=1:raise ValueError('Unsupported version1 scale gain '+label)
            centered=scale.find('zero_centered')
            if centered is not None and leaf(centered) not in ('true','1'):raise ValueError('Unsupported version1 zero-centred mapping '+label)
            bounds(scale,'range',low,high,label,True)
            output(scale,scale.get('name'),False)
        def bounded(node,rate,low,high,label):
            shape(node,('input','rate_limit','clipto','output'),('description',),('name',))
            leaf(node.find('input'))
            if scalar(leaf(node.find('rate_limit')))!=rate:raise ValueError('Missing/inconsistent symmetric native rate '+label)
            bounds(node,'clipto',low,high,label)
        def mixer_weights(mixer,metadata):
            shape(mixer,('function','clipto'),('output','description'),('name',))
            function=mixer.find('function')
            if function.attrib or (function.text or '').strip() or len(function)!=1:raise ValueError('Unsupported mixer function wrapper')
            expression=function[0];weights={axis:0. for axis in ('aileron','elevator','rudder')}
            terms=[]
            if expression.tag=='value':
                if number(expression)!=0:raise ValueError('Unmodeled constant mixer offset')
            elif expression.tag=='sum':
                if expression.attrib or (expression.text or '').strip() or not 1<=len(expression)<=3 or any(n.tag!='product' for n in expression):
                    raise ValueError('Unsupported native linear mixer sum')
                terms=list(expression)
            elif expression.tag=='product':terms=[expression]
            else:raise ValueError('Unsupported nonlinear native mixer')
            seen=set()
            for term in terms:
                shape(term,('value','property'))
                source=leaf(term.find('property'))
                axes=[axis for axis in weights if source=='fcs/'+axis+'-response-rad']
                if len(axes)!=1 or axes[0] in seen:raise ValueError('Unknown/duplicate native mixer axis')
                axis=axes[0];seen.add(axis);weights[axis]=number(term.find('value'))
            for axis,value in weights.items():
                if value!=property_value(metadata+'/mix-'+axis):raise ValueError('Native mixer coefficient/profile mismatch '+axis)
            output(mixer,mixer.get('name'),False)
        prefix='fsim/missile-control/'
        if property_value('fsim/control-operating-point-v1/version')!=1:raise ValueError('Unsupported/missing composed actuator version')
        if property_value(prefix+'actuator-version')!=1 or property_value(prefix+'normalization-version')!=1:
            raise ValueError('Unsupported/missing finite actuator metadata')
        pole=positive(property_value(prefix+'lag-coefficient-s-1'),'lag pole')
        rate=positive(property_value(prefix+'channel-rate-rad_s'),'channel rate')
        channel_stages=[]
        for axis in ('aileron','elevator','rudder'):
            stem='fcs/'+axis
            scale=component('aerosurface_scale',stem+'-control')
            lag=component('lag_filter',stem+'-operating-lag')
            actuator=component('actuator',stem+'-actuator')
            serial(scale,lag,actuator)
            shape(lag,('input','c1','output'),('description',),('name',))
            if leaf(scale.find('input'))!=stem+'-cmd-norm' or leaf(lag.find('input'))!=stem+'-control':
                raise ValueError('Disconnected native scale/lag input '+axis)
            if positive(scalar(leaf(lag.find('c1'))),'lag pole')!=pole:raise ValueError('Native lag/profile mismatch '+axis)
            output(lag,stem+'-lag-state')
            if leaf(actuator.find('input'))!=stem+'-lag-state':raise ValueError('Disconnected native lag/actuator input '+axis)
            low,high=property_value(prefix+axis+'-min-rad'),property_value(prefix+axis+'-max-rad')
            if low>0 or high<0 or low>high or low==high and (axis!='aileron' or low!=0):raise ValueError('Invalid signed channel stops '+axis)
            normalization(scale,low,high,axis);bounded(actuator,rate,low,high,axis)
            output(actuator,stem+'-response-rad');channel_stages.append(actuator)
        count=property_value(prefix+'fin-count')
        if count<1 or count>32 or count!=int(count) or property_value('fsim/control-operating-point-v1/fin-count')!=count:
            raise ValueError('Invalid physical fin count')
        for index in range(int(count)):
            stem='fcs/fin-%d'%index;metadata=prefix+'fin-%d'%index
            mixer=component('fcs_function',stem+'-demand-rad');actuator=component('actuator',stem+'-actuator')
            serial(mixer,actuator)
            if any(order[node]>=order[mixer] for node in channel_stages):raise ValueError('Physical mixer precedes native channel response')
            mixer_weights(mixer,'fsim/control-operating-point-v1/fin-%d'%index)
            if leaf(actuator.find('input'))!=stem+'-demand-rad':raise ValueError('Broken physical fin input')
            low,high=property_value(metadata+'-min-rad'),property_value(metadata+'-max-rad')
            if low>high:raise ValueError('Invalid physical fin stops')
            bounds(mixer,'clipto',low,high,stem)
            bounded(actuator,positive(property_value(metadata+'-rate-rad_s'),'fin rate'),low,high,stem)
            output(actuator,stem+'-delta-rad')
        return True,'All three serial channel lag/rate/stop paths and every post-mix finite fin consume their actual versioned native bindings'
    except (ValueError,TypeError,KeyError,OverflowError) as error:
        return False,'Invalid versioned composed actuator chain: '+str(error)


def structure(xml, product, card, category, required_powered=False, extra_xml=(), design_spec=None):
    """Component applicability follows declared geometry/equipment, never names."""
    root = ET.fromstring(xml)
    checks, components, xml_tables = [], {}, []

    def require(name, present, detail):
        checks.append(dict(name=name, passed=bool(present), details=detail))
        components[name] = dict(applicability='required', present=bool(present), derivation=detail, basis='estimated')

    def declared(name, descriptor):
        state,passed=declared_component_state(descriptor)
        components[name]=dict(state,derivation=descriptor,basis='estimated')
        checks.append(dict(name=name,passed=passed,details=descriptor))

    require('6DOF', root.tag == 'fdm_config', 'Native JSBSim rigid-body state propagation; SDK owns integration')
    tail_damping = (design_spec or {}).get('rotor', {}).get('tail', {})
    if 'damping_lb_per_fps' in tail_damping:
        marker = root.find("flight_control/property[.='fsim/rotor-tail-damping-v1/available']")
        reaction = root.find("external_reactions/force[@name='passive-tail-damping-v1']")
        require('authored-tail-rotor-damping', marker is not None and marker.get('value') == '1' and reaction is not None,
                'Versioned source yaw derivative transferred to the current physical tail port from a source-bound actual native hover point; passive lowpass engineering estimate')
    mass = root.find('mass_balance')
    require('mass-CG-inertia', mass is not None and all(mass.find(x) is not None for x in ('emptywt', 'ixx', 'iyy', 'izz', "location[@name='CG']")), 'JSBSim mass_balance plus tanks/pointmass recomputation; fixed mass is the unpowered/no-consumable limit')
    require('six aerodynamic axes', {axis.get('name') for axis in root.findall('aerodynamics/axis')} >= {'DRAG', 'SIDE', 'LIFT', 'ROLL', 'PITCH', 'YAW'}, 'Wind CD,CY,CL rotated into body CA=-Fx/qS,CY=Fy/qS,CN=-Fz/qS; Cl,Cm,Cn about AERORP then shifted to current CG')
    engines = root.findall('propulsion/engine')
    files = [product / (product.name + '.xml')]
    for path in sorted((product / 'Engines').glob('*.xml')):
        files.append(path)
    files.extend(extra_xml)
    require('propulsion-massflow-nozzle', (bool(engines) or not required_powered) and all(
        (product/'Engines'/(e.get('file')+'.xml')).is_file() and e.find('thruster') is not None and
        (product/'Engines'/(e.find('thruster').get('file')+'.xml')).is_file() for e in engines),
        'Powered designs require generated engine and nozzle/propeller/rotor definitions; unpowered designs use explicit zero thrust/zero mass flow/coast')
    components['boost-sustain-coast'] = dict(applicability='required' if card and card.get('powerable') else 'not_applicable', present=True,
        derivation='Generated engine thrust schedules/propellant state; coast is exhausted/disabled engine; no boost/sustain requirement for non-rocket engines', basis='estimated')
    control = root.find('flight_control')
    actuators = root.findall('.//actuator')
    controlled = bool(actuators or root.findall('.//lag_filter'))
    require('control-mixer-autopilot', category == 'store' and not card.get('guidance') or control is not None,
            'Generated FCS/physical mixer; fixed-wing/rotorcraft SDK controller profile; guided store component declarations')
    # A physical rotor/motor already propagates response lag. Its versioned
    # front-end shaper records zero ADDITIONAL lag, preserving that state.
    # JSBSim <lag>0</lag> freezes a sample; omission is identity. Require real
    # existing response filters before accepting the composed representation.
    additional=root.find("flight_control/property[.='fsim/engineering-actuator-v1/additional-lag-s']")
    response_lags=[]
    for node in root.findall('.//lag_filter'):
        pole=node.findtext('c1','').strip()
        try:
            valid=math.isfinite(float(pole)) and float(pole)>0
        except ValueError:
            # Motor response may switch between declared spin-up/down poles.
            valid=bool(pole) and any(n.get('name')==pole or n.tag=='property' and (n.text or '').strip()==pole for n in root.iter())
        if valid:response_lags.append(node.get('name'))
    composed_lag=category=='rotorcraft' and additional is not None and float(additional.get('value','nan'))==0. and bool(response_lags)
    bounded_actuators = [n for n in actuators if all(n.find(tag) is not None for tag in ('rate_limit','clipto/min','clipto/max')) and (n.find('lag') is not None or composed_lag)]
    if category=='rotorcraft':
        from .actuator_models import audit as actuator_audit
        actuator_rows,actuator_checks=actuator_audit(root)
        checks.extend(actuator_checks)
        actuator_complete=bool(actuator_rows) and all(c['passed'] for c in actuator_checks)
    else:actuator_complete=not controlled or bool(bounded_actuators)
    actuator_detail='Per-physical-state native motor lag/command bounds and derived angular acceleration bound, or post-mix pitch servo lag/rate/stops. No extra guessed rotor command slew; uncontrolled inert designs are the identity limit.'
    selected=(card or {}).get('component_models',{}).get('components',{}).get('autopilot',{}).get('implementation_id')
    if selected:
        from .equipment_models import normalize as equipment_mechanism
        selected=equipment_mechanism(card,'autopilot',selected)
    operating=root.find("flight_control/property[.='fsim/control-operating-point-v1/version']")
    if selected=='fsim.control.operating_point_bounded.v1' or operating is not None:
        actuator_complete,actuator_detail=_operating_point_actuator_audit(root)
    require('actuator-lag-rate-stops',actuator_complete,actuator_detail)
    components['actuator-lag-rate-stops']['applicability'] = 'required' if controlled else 'not_applicable'
    if card:
        definitions = card.get('component_models', {}).get('components', {})
        from .components import resolve
        for kind in ('autopilot','allocator','actuator','propulsion','aerodynamics'):
            resolved=resolve(card,kind)
            if resolved is None:require('declared-'+kind,False,'Missing explicit component declaration')
            else:declared('declared-'+kind,resolved)
        for kind in ('seeker', 'guidance', 'fuze', 'warhead', 'damage', 'release', 'reset'):
            declaration = definitions.get(kind)
            if declaration:
                from .components import resolve
                resolved = resolve(card,kind)
                declared(kind,resolved)
            elif kind in ('release', 'reset', 'damage'):
                require(kind, True, 'Native SDK store release/entity damage/vehicle reset implementation; reset preserves documented SDK authority semantics')
            else:
                # Missing declarations cannot establish an approved unarmed
                # configuration. The frozen roles need an explicit project
                # available/unavailable/not-installed declaration.
                require(kind,False,'Missing explicit project applicability/implementation declaration for '+kind)
    else:
        for kind in ('seeker', 'guidance', 'fuze', 'warhead', 'damage', 'release', 'reset'):
            components[kind] = dict(applicability='required' if kind in ('damage', 'reset') else 'not_applicable', present=True,
                 derivation='Native SDK vehicle damage/reset' if kind in ('damage', 'reset') else 'Airframe-only design; equipment modeled separately', basis='estimated')
    consistent=all(check['passed']==(components[check['name']]['applicability']=='not_applicable' or components[check['name']]['present'])
                   for check in checks if check['name'] in components)
    checks.append(dict(name='component applicability/check consistency',passed=consistent,
        details='Explicit not-installed declarations pass; unavailable required implementations remain absent and fail.'))
    for path in files:
        doc = ET.parse(path)
        checksum=sha(path)
        for index, node in enumerate(doc.findall('.//table')+doc.findall('.//thrust_table')):
            from .engineering_tables import table as parse_native_table
            try:
                dimensions=parse_native_table(node)
                valid=True
            except (ValueError,KeyError) as error:
                valid=False;dimensions=dict(error=str(error))
            checks.append(dict(name='native table/'+path.name+'/'+str(index),passed=valid,details=dimensions))
            xml_tables.append(dict(name=path.name+'/table/'+str(index), method='shared generated JSBSim expression/table', basis='estimated',
                version=VERSION, derivation='Current card/design -> shared Hangar engine/control/aerodynamic writer; includes existing scoped calibration where declared',
                source=str(path.relative_to(ROOT)), sha256=checksum, dimensions=dimensions,
                interpolation='JSBSim linear/bilinear/trilinear according to independentVar', extrapolation='clamped table endpoints'))
    return components, checks, xml_tables


def generation_inputs(record):
    """Explicit geometry, control/profile and carried-equipment input closure.

    Optional absence is bound too: adding an identification/profile file must
    invalidate reuse just as editing one does. External file inputs are refused.
    """
    import tomllib
    inputs={};presence={};visited=set()
    def bind(path):
        path=Path(path).resolve()
        if not path.is_relative_to(ROOT.resolve()) or '.codex' in {p.casefold() for p in path.parts}:
            raise ValueError('Generation input escapes allowed project source: '+str(path))
        relative=path.relative_to(ROOT.resolve()).as_posix()
        presence[relative]=path.is_file()
        if presence[relative]:inputs[relative]=sha(path)
    def design(path):
        path=Path(path).resolve()
        if path in visited:return
        visited.add(path);bind(path)
        directory=path.parent
        bind(path.with_suffix('.liquid-geometry-v1.json'))
        for name in ('calibration.toml','paint.toml','autopilot.toml','hover.toml','tail-damping.json','flight-calibration.json',
                     'out/fly.json','out/performance.json','out/ground.json','payloads.json'):
            bind(directory/name)
        spec=tomllib.loads(path.read_text(encoding='utf-8'))
        if spec.get('aircraft',{}).get('category') in ('missile','store'):
            bind(ROOT/'assets/stores'/(spec['aircraft']['card']+'.json'))
        for surface in spec.get('surface',[]):
            for part in [surface,*surface.get('sections',[])]:
                foil=part.get('airfoil')
                if isinstance(foil,str) and (directory/foil).is_file():bind(directory/foil)
        for foil in directory.glob('*.dat'):bind(foil)
        fit=directory/'payloads.json'
        if not fit.is_file():return
        payload=json.loads(fit.read_text(encoding='utf-8'))
        for item in payload.get('payloads',[]):bind(ROOT/'assets/payloads'/(item.get('card','')+'.json'))
        for loadout in payload.get('loadouts',[]):
            for station in loadout.get('stations',[]):
                card=ROOT/'assets/stores'/(station.get('store','')+'.json');bind(card)
                if card.is_file():
                    model=json.loads(card.read_text(encoding='utf-8')).get('model','')
                    if model.startswith('jsbsim:'):
                        name=model.removeprefix('jsbsim:');design(ROOT/'aircraft'/name/(name+'.toml'))
    design(record['path'])
    bind(Path(__file__).with_name('public_data.json'))
    return inputs,presence


def mass_partition_audit(xml_path, damage_path=None, design_spec=None):
    """Read actual serialized products; never call the fitting equations.

    Products are transformed to body axes. Every retained subset is tested at
    its own CG, including the covariance/triangle condition for a physical
    nonnegative mass distribution, rather than just positive diagonals.
    """
    import itertools
    balance=ET.parse(xml_path).getroot().find('mass_balance')
    if balance is None:raise ValueError('Missing generated mass balance')
    def scalar(name, factors, default=None):
        element=balance.find(name)
        if element is None:
            if default is not None:return default
            raise ValueError('Missing generated mass quantity '+name)
        unit=element.get('unit','').upper()
        if unit not in factors:raise ValueError('Unsupported generated mass unit '+unit)
        value=float(element.text)*factors[unit]
        if not math.isfinite(value):raise ValueError('Nonfinite generated mass quantity '+name)
        return value
    mass=scalar('emptywt',{'KG':1.,'LBS':.45359237,'':.45359237})
    inertia_units={'KG*M2':1.,'SLUG*FT2':1.3558179483314004,'':1.3558179483314004}
    a=[scalar(k,inertia_units,0. if k in ('ixy','ixz','iyz') else None)
       for k in ('ixx','iyy','izz','ixy','ixz','iyz')]
    negative=balance.get('negated_crossproduct_inertia','true')!='false'
    sign=-1. if negative else 1.
    tensor=np.array([[a[0],sign*a[3],-sign*a[4]],
                     [sign*a[3],a[1],sign*a[5]],[-sign*a[4],sign*a[5],a[2]]])
    location=balance.find("location[@name='CG']")
    if location is None:raise ValueError('Missing generated empty CG')
    distance={'M':1.,'IN':.0254,'FT':.3048,'':.0254}
    if location.get('unit','').upper() not in distance:raise ValueError('Unsupported generated CG unit')
    cg=np.array([float(location.find(k).text) for k in ('x','y','z')])*distance[location.get('unit','').upper()]
    if mass<=0 or not np.isfinite(cg).all():raise ValueError('Invalid generated empty mass/CG')
    roundoff=1024*np.finfo(float).eps*max(float(np.linalg.norm(tensor,ord=2)),1.)
    if not math.isfinite(roundoff):raise ValueError('Nonfinite generated inertia roundoff budget')
    def physical(value, tolerance=roundoff):
        if not math.isfinite(tolerance) or tolerance<0 or not np.isfinite(value).all():return False
        with np.errstate(over='ignore',invalid='ignore'):
            covariance=.5*np.trace(value)*np.eye(3)-value
        if not np.isfinite(covariance).all():return False
        return bool(np.linalg.eigvalsh(value).min()>=-tolerance and
                    np.linalg.eigvalsh(covariance).min()>=-tolerance)
    if not physical(tensor):raise ValueError('Generated empty inertia is not a physical mass tensor')
    result=dict(version='fsim.mass-partition.audit.v1',passed=True,xml_sha256=sha(Path(xml_path)),
        mass_kg=mass,cg_structural_m=cg.tolist(),body_tensor_kgm2=tensor.tolist(),
        primary_tensor_physical=True,section_count=0,complement_count=0)
    spec=(design_spec or {}).get('mass',{})
    result['source_policy']={k:spec[k] for k in ('gyration_reference','inertia_reference','retired_gyration',
        'retired_inertia_tensor_kgm2','inertia_reference_reason','inertia_uncertainty_fraction') if k in spec}
    if damage_path is None:return result
    data=json.loads(Path(damage_path).read_text(encoding='utf-8'))
    result['damage_sha256']=sha(Path(damage_path))
    origin=np.asarray(data['origin_design_m'],float)
    if not np.allclose(origin,cg,rtol=0,atol=64*np.finfo(float).eps*max(1.,float(np.max(np.abs(cg))))):
        raise ValueError('Damage and native empty origins differ')
    terms=[];amounts=[];firsts=[]
    def pai(amount,r):return amount*(np.dot(r,r)*np.eye(3)-np.outer(r,r))
    for section in data['sections']:
        amount=float(section['mass_kg']);r=np.asarray(section['cg'],float);v=section['inertia']
        own=np.array([[v[0],-v[3],-v[4]],[-v[3],v[1],-v[5]],[-v[4],-v[5],v[2]]])
        if not math.isfinite(amount) or amount<0 or not np.isfinite(r).all() or not physical(own):
            raise ValueError('Nonphysical serialized section '+section['name'])
        if amount==0 and float(np.max(np.abs(own)))>roundoff:
            raise ValueError('Zero-mass serialized section carries finite own inertia: '+section['name'])
        amounts.append(amount);firsts.append(amount*r);terms.append(own+pai(amount,r))
    # Exact dimensionless solver tolerance mapped to dimensional moments;
    # plus floating-point summation roundoff. This is not a physical clamp.
    method=data.get('mass_partition',{})
    scale=np.asarray(method.get('normalization_scale_m',[1.,1.,1.]),float)
    if scale.shape!=(3,) or not np.isfinite(scale).all() or (scale<1.).any():
        raise ValueError('Invalid generated moment normalization scale')
    epsilon=float(method.get('dimensionless_moment_tolerance',0.))
    if epsilon not in (0.,2e-12):raise ValueError('Unknown generated moment tolerance')
    tensor_budget=roundoff
    mass_budget=1024*np.finfo(float).eps*mass
    first_budget=mass_budget*max(float(np.max(np.abs(cg))),1.)
    if epsilon:
        # Bound metadata against independently serialized physical geometry.
        # A global coordinate extent is conservative for each normalization
        # axis, while preventing arbitrary finite metadata from inflating the
        # unchanged dimensionless numerical tolerance.
        geometry_bound=0.
        geometry_points=0
        def include_geometry(points):
            nonlocal geometry_bound,geometry_points
            value=np.asarray(points,float)
            if value.ndim!=2 or value.shape[1]!=3 or not len(value) or not np.isfinite(value).all():
                raise ValueError('Invalid serialized geometry for moment normalization')
            geometry_bound=max(geometry_bound,float(np.max(np.abs(value))))
            geometry_points+=len(value)
        def include_box(record):
            centre=np.asarray(record['centre'],float);half=np.asarray(record['half'],float)
            if centre.shape!=(3,) or half.shape!=(3,) or not np.isfinite(centre).all() or not np.isfinite(half).all() or (half<0).any():
                raise ValueError('Invalid serialized geometry box for moment normalization')
            with np.errstate(over='ignore',invalid='ignore'):
                include_geometry([centre-half,centre+half])
        for panel in data.get('panels',[]):
            if 'corners' in panel:include_geometry(panel['corners'])
            elif 'centre' in panel and 'half' in panel:include_box(panel)
        for mesh in data.get('contact_geometry',{}).get('meshes',[]):
            include_geometry(mesh['vertices'])
        for component in data.get('components',[]):
            include_box(component)
        if not geometry_points:raise ValueError('Missing serialized geometry proof for moment normalization')
        geometry_bound=max(1.,geometry_bound)
        if np.max(scale)>geometry_bound+64*np.finfo(float).eps*geometry_bound:
            raise ValueError('Moment normalization scale exceeds serialized geometry bound')
        result['normalization_geometry_bound_m']=geometry_bound
        # Do not square a scale before applying a zero tolerance. Even finite
        # metadata can overflow intermediate arithmetic and create NaN/Inf.
        maximum_scale=float(np.max(scale))
        tensor_budget+=(4*epsilon*mass*maximum_scale)*maximum_scale
        first_budget+=epsilon*mass*maximum_scale
    if not all(math.isfinite(value) for value in (mass_budget,first_budget,tensor_budget)):
        raise ValueError('Nonfinite generated moment comparison budget')
    with np.errstate(over='ignore',invalid='ignore'):
        total_mass=sum(amounts)
        total_first=sum(firsts,np.zeros(3))
        total_tensor=sum(terms,np.zeros((3,3)))
    if not math.isfinite(total_mass) or not np.isfinite(total_first).all() or not np.isfinite(total_tensor).all():
        raise ValueError('Nonfinite serialized aggregate mass moments')
    with np.errstate(over='ignore',invalid='ignore'):
        mass_error=abs(total_mass-mass)
        first_error=float(np.max(np.abs(total_first)))
        tensor_error=float(np.max(np.abs(total_tensor-tensor)))
    if not all(math.isfinite(value) for value in (mass_error,first_error,tensor_error)):
        raise ValueError('Nonfinite serialized aggregate moment errors')
    if mass_error>mass_budget or first_error>first_budget or tensor_error>tensor_budget:
        raise ValueError('Serialized sections do not conserve native empty mass/first/full second moments: '+
            str(dict(mass_error=mass_error,first_error=first_error,tensor_error=tensor_error,budget=tensor_budget)))
    for size in range(len(terms)):
        for removed in itertools.combinations(range(len(terms)),size):
            remaining=mass-sum(amounts[i] for i in removed)
            first=-sum((firsts[i] for i in removed),np.zeros(3))
            own=tensor-sum((terms[i] for i in removed),np.zeros((3,3)))
            if remaining<=mass_budget:
                # A numerical vacuum cannot retain finite moments. Check the
                # actual subtraction before skipping an empty complement.
                if (remaining < -mass_budget or not np.isfinite(first).all() or not np.isfinite(own).all() or
                    float(np.max(np.abs(first)))>first_budget or float(np.max(np.abs(own)))>tensor_budget):
                    raise ValueError('Zero-mass retained section complement carries finite moments '+str(removed))
                continue
            own-=pai(remaining,first/remaining)
            if not physical(own,tensor_budget):raise ValueError('Nonphysical retained section complement '+str(removed))
            result['complement_count']+=1
    result.update(section_count=len(terms),mass_error_kg=mass_error,first_moment_error_kgm=first_error,
        tensor_error_kgm2=tensor_error,tensor_numeric_budget_kgm2=tensor_budget,method=method)
    return result


def generate(record, out, geometry_threads=1):
    if type(geometry_threads) is not int or not 1<=geometry_threads<=16:raise ValueError('Geometry threads must be an integer from 1 to 16')
    from . import airframe, storestages, rotorcraft, pieces
    start = time.monotonic()
    d = pipeline.Design(record['path'], log=lambda *x: None)
    t = d.tables(force=True)  # No stale cache can satisfy this campaign.
    table_seconds=time.monotonic()-start
    xml_start=time.monotonic()
    if record['category'] == 'store':
        storestages.build(d)
        card = storestages.card_of(d)
        product = Path(storestages.flight_model(card)[0]).parent
    elif record['category'] == 'rotorcraft':
        rotorcraft.Rotorcraft(record['path'], log=lambda *x: None).build()
        card, product = None, Path(record['path']).parent
    else:
        d.build_with(t)
        card, product = None, Path(record['path']).parent
    runtime_xml_seconds=time.monotonic()-xml_start
    auxiliary_start=time.monotonic();detached=[]
    from .resources import thread_reservation,cpu_lease
    with thread_reservation(geometry_threads),cpu_lease(geometry_threads):
        # Complete production owns the normal visual/material model as well
        # as flight XML. A previous GLB or liquid sidecar cannot satisfy a
        # forced current-source campaign. Normal model writers also own the
        # damage, detached definitions and required occupancy generation.
        if record['category']=='store':
            model_result=storestages.model(d,force=True)
        elif record['category']=='rotorcraft':
            model_result=rotorcraft.Rotorcraft(record['path'],log=lambda *x:None).model()
        else:
            model_result=d.model(force=True)
    if record['category']=='fixed-wing' and any(s.kind=='wing' for s in d.aircraft.surfaces):
        damage_path=product/(product.name+'.damage.json')
        if not damage_path.is_file():raise ValueError('Normal model omitted required damage product')
        current_damage=json.loads(damage_path.read_text(encoding='utf-8'))
        detached=[damage_path]
        for section in pieces.PARTS:
            piece_name=pieces.piece_name(d.aircraft.name,section)
            piece_path=product/'pieces'/piece_name/(piece_name+'.xml')
            expected_piece=pieces.piece_xml(current_damage,section)
            if expected_piece is None:
                if piece_path.exists():raise ValueError('Obsolete generated section XML survived the normal model writer')
            else:
                if not piece_path.is_file() or piece_path.read_text(encoding='utf-8')!=expected_piece:
                    raise ValueError('Normal model did not publish the exact current section definition')
                detached.append(piece_path)
    auxiliary_seconds=time.monotonic()-auxiliary_start
    # The runtime coefficients include existing explicit calibration, never a
    # new fit or a manual edit of generated XML.
    tables = d.calibrated(t)
    inventory, checks = table_audit(tables, d.aircraft)
    xml = product / (product.name + '.xml')
    from . import integration
    resolution=integration.settings(ET.parse(xml).getroot(),d.aircraft,tables)
    required_powered=bool(card.get('powerable')) if card else record['category']=='rotorcraft' or bool(d.aircraft.engines)
    components, structural, xml_inventory = structure(xml.read_text(encoding='utf-8'), product, card, record['category'],required_powered,[p for p in detached if p.suffix=='.xml'],design_spec=d.aircraft.spec)
    structural.extend(dict(name='normal-model-'+check['name'],passed=check['status']!='fail')
                      for check in model_result['checks'])
    damage_product=next((p for p in detached if p.name.endswith('.damage.json')),None)
    mass_partition=mass_partition_audit(xml,damage_product,d.aircraft.spec)
    structural.append(dict(name='serialized-mass-partition',passed=mass_partition['passed']))
    inputs,input_presence = generation_inputs(record)
    if card:
        cp = ROOT / 'assets/stores' / (card['id'] + '.json')
        if inputs.get(cp.relative_to(ROOT).as_posix())!=sha(cp):
            raise ValueError('Current store card is absent from the generation input closure')
    source_files = sorted(Path(__file__).parent.glob('*.py'))
    for directory in ('aero','geometry','rotorcraft','shape'):
        source_files += sorted((Path(__file__).parent/directory).glob('*.py'))
    source_files += sorted((Path(__file__).parent.parent/'native').glob('*.cpp'))
    source_files += sorted((Path(__file__).parent.parent/'native').glob('*.h'))
    source_files += sorted((Path(__file__).parent.parent/'native').glob('*.hpp'))
    source_files += sorted((Path(__file__).parent.parent/'native').glob('CMakeLists.txt'))
    generator = {str(p.relative_to(ROOT)): sha(p) for p in source_files}
    geometry_backend=None
    if record['category']=='fixed-wing' and any(s.kind=='wing' for s in d.aircraft.surfaces):
        from .shape import meshkit
        backend=Path(meshkit.library()._name).resolve()
        geometry_backend=dict(path=str(backend),sha256=sha(backend),
            method='Native signed-distance geometry fitting',basis='simulated',
            version=VERSION,source='tools/models/hangar/native')
    a = d.aircraft
    from .drag import gear_drag
    from .carriage import _CONTROL_DESIGN_REPORTS
    control_design=[value for key,value in _CONTROL_DESIGN_REPORTS.items() if key[0]==a.name]
    bundle = dict(control_design=control_design, schema=VERSION, name=record['name'], category=record['category'], basis='simulated', sources=SOURCES,
        auxiliary_inventory=[dict(name=str(p.relative_to(product)),method='Current geometry -> native-fitted shared damage and detached-body writers',basis='estimated',version=VERSION,
            source=str(Path(record['path']).relative_to(ROOT)),derivation='damage.py:write and pieces.py:write; geometry-derived mass/CG/inertia, strip load shares and explicit tumbling-body estimates; no new measured data') for p in detached],
        input_sha256=inputs, input_presence=input_presence, generator_sha256=generator, geometry_backend=geometry_backend, table_inputs_hash=d.table_inputs_hash(), regenerated=True,
        reference=dict(S=a.S, b=a.b, c=a.c, arp=a.aero_point.tolist()), gear_drag=gear_drag(a), tables=tables,
        components=components, mass_partition=mass_partition, integration=resolution, lifting_panels=int(t.get('_panels',0)), inventory=inventory+xml_inventory, checks=checks+structural,
        limitations=['Finite full-angle tables are engineering continuation, not measured high-angle fidelity.',
            'Control/rate/Mach effects use a separated additive build-up; no tensor cross-term or high-frequency unsteady flow claim.',
            'No CFD is run; all CFD tables count is zero. No new calibration or arbitrary per-design scale is introduced.',
            'Short native traces validate execution/forces and instantaneous performance, not measured maximum-performance accuracy.'],
        generation_seconds=time.monotonic()-start, calibration='optional separate public-reference ledger')
    # Test reference data is not a public native flight-model asset. Bind it
    # beside the receipt; physics loads only regenerated installed XML/carriage.
    reference_path=out/'engineering-aero-v1.json'
    dump(reference_path, bundle)
    legacy=product/'engineering-aero-v1.json'
    if legacy.is_file():
        old=json.loads(legacy.read_text(encoding='utf-8'))
        if old.get('schema')!=VERSION or old.get('name')!=record['name']:raise ValueError('Unowned audit sidecar at native runtime path')
        legacy.unlink()
    from .runtime_products import stage_built_project_sdks
    stage_built_project_sdks(record['path'])
    visual=Path(record['path']).parent;name=record['name']
    model_products=[visual/(name+'.glb'),visual/(name+'.glb.manifest')]
    if record['category']!='store':model_products.append(visual/(name+'.liquid-v1.json'))
    if any(not p.is_file() for p in model_products):raise ValueError('Normal model omitted a required visual/material product')
    from . import method_backends
    backend_start=time.monotonic();backend_closure=method_backends.snapshot()
    backend_seconds=time.monotonic()-backend_start
    products = [xml,*detached,*model_products] + sorted((product/'Engines').glob('*.xml')) + sorted((product/'aero').glob('vlm-carriage-*.json'))
    receipt = dict(record, product_directory=str(product.relative_to(ROOT/'aircraft')), class_name=card.get('class') if card else None,
        input_sha256=inputs, input_presence=input_presence, generator_sha256=generator, geometry_backend=geometry_backend, table_inputs_hash=d.table_inputs_hash(),
        products={str(p.relative_to(ROOT/'aircraft')): sha(p) for p in products}, checks=checks+structural,
        control_design=control_design, components=components, mass_partition=mass_partition, integration=resolution,
        normal_model=dict(regenerated=True,checks=model_result.get('checks',[]),
            products=[str(p.relative_to(ROOT/'aircraft')) for p in model_products],method_backends=backend_closure),
        lifting_panels=int(t.get('_panels',0)), table_count=len(inventory)+len(xml_inventory), generation_seconds=time.monotonic()-start,
        native_dll_sha256=sha(ROOT/'build/ucrt64-release/python/fsim/bin/libfsim.dll'),
        reference_bundle=dict(name=reference_path.name,sha256=sha(reference_path),scope='Test-only reference; not a native runtime asset'))
    receipt['generation_phase_worker_seconds']=dict(forced_aerodynamic_tables=table_seconds,runtime_xml=runtime_xml_seconds,
        fresh_model_material_and_detached_products=auxiliary_seconds,method_backend_inventory=backend_seconds,
        audit_hashing_publication=receipt['generation_seconds']-table_seconds-runtime_xml_seconds-auxiliary_seconds-backend_seconds)
    from .shape import meshkit
    receipt['geometry_query_cache']=dict(version='fsim.meshkit.exact-point-cache.v1',capacity_per_scene=65536,
        key='Exact native float64 input bytes; immutable scene lifetime',large_scan_bypass_points=4096,
        statistics_scope='Closed scenes only; hits include within-query exact duplicate elimination',
        **meshkit.probe_cache_stats())
    receipt['geometry_threads']=geometry_threads
    if card:
        receipt['store_card'] = dict(id=card['id'],sha256=sha(cp),carrier=storestages.carrier_for(card))
    dump(out/'generation.json', receipt)
    return receipt


def run_command(command, log, remaining, cwd=ROOT, env=None):
    if remaining <= 0:
        raise subprocess.TimeoutExpired(command, 0)
    with Path(log).open('w', encoding='utf-8') as stream:
        child = subprocess.Popen(command, cwd=cwd, env=env, stdout=stream, stderr=subprocess.STDOUT)
        try:
            code=child.wait(timeout=remaining)
        except subprocess.TimeoutExpired:
            # CMake install may own a wheel-builder child. Reap the owned tree
            # before releasing the serial slot; never leave a timed-out writer.
            if os.name=='nt':
                killed=subprocess.run(['taskkill','/PID',str(child.pid),'/T','/F'],stdout=stream,stderr=subprocess.STDOUT,timeout=10)
                if killed.returncode and child.poll() is None:
                    raise RuntimeError('Cannot terminate deadline-owned process tree; serial slot remains occupied')
            else:
                child.kill()
            child.wait(timeout=10)
            raise
    if code:
        raise RuntimeError('Command failed ('+str(code)+'): '+str(log))


def generation_order(records,profile=None):
    """Cohort priority is invariant; optional observed cost only orders peers."""
    priority={'aam':0,'fixed-wing':1,'stores':2,'rotorcraft':3}
    costs={}
    if profile is not None:
        if profile.get('schema')!='fsim.sim-m02.scheduling-profile.v1':raise ValueError('Unsupported scheduling profile')
        costs=profile.get('generation_wall_seconds',{})
        if set(costs)!={r['name'] for r in records}:raise ValueError('Scheduling profile must cover exactly the declared roster')
        if any(type(v) not in (int,float) or not math.isfinite(v) or v<=0 for v in costs.values()):
            raise ValueError('Scheduling costs must be finite positive wall seconds')
    return sorted(records,key=lambda r:(priority[cohort(r)],-costs.get(r['name'],0),r['name']))


def reusable_generation(folder,record):
    """Resume only a byte-bound generation completed by this exact method.

    An interrupted writer has no complete receipt and is regenerated. Reuse
    includes every input, generator file, emitted product and reference bundle;
    a table cache or a status label alone can never satisfy this contract.
    """
    receipt=folder/'generation.json'
    if not receipt.is_file():return None
    try:
        row=json.loads(receipt.read_text(encoding='utf-8'))
        if row['name']!=record['name'] or row['category']!=record['category']:return None
        model=row.get('normal_model',{})
        if not isinstance(model,dict) or model.get('regenerated') is not True:return None
        if not isinstance(model.get('products'),list) or not model['products'] or any(not isinstance(name,str) for name in model['products']):return None
        if not isinstance(model.get('checks'),list) or not model['checks'] or any(not isinstance(c,dict) or
            not isinstance(c.get('name'),str) or not c['name'].strip() or c.get('status') not in ('pass','info','warn') for c in model['checks']):return None
        if not isinstance(row.get('products'),dict) or any(not isinstance(name,str) for name in row['products']):return None
        required_visual={record['name']+'/'+record['name']+suffix for suffix in
            (('.glb','.glb.manifest') if record['category']=='store' else ('.glb','.glb.manifest','.liquid-v1.json'))}
        declared_visual=[name.replace('\\','/') for name in model['products']]
        current_products={name.replace('\\','/'):digest for name,digest in row['products'].items()}
        if len({name.casefold() for name in current_products})!=len(row['products']):return None
        if len(declared_visual)!=len(required_visual) or set(declared_visual)!=required_visual or not required_visual.issubset(current_products):return None
        current_inputs,current_presence=generation_inputs(record)
        if row.get('input_sha256')!=current_inputs or row.get('input_presence')!=current_presence:return None
        from . import method_backends
        if not method_backends.verify(model.get('method_backends')):return None
        if not row['checks'] or not all(c['passed'] for c in row['checks']):return None
        for relative,present in row['input_presence'].items():
            if (ROOT/relative).is_file()!=present:return None
        for kind in ('input_sha256','generator_sha256'):
            for relative,expected in row[kind].items():
                path=Path(relative)
                if path.is_absolute() or '..' in path.parts or '.codex' in {x.casefold() for x in path.parts}:return None
                if not (ROOT/path).is_file() or sha(ROOT/path)!=expected:return None
        for relative,expected in row['products'].items():
            path=Path(relative)
            if path.is_absolute() or '..' in path.parts or '.codex' in {x.casefold() for x in path.parts}:return None
            product=ROOT/'aircraft'/path
            if not product.is_file() or sha(product)!=expected:return None
        reference=Path(row['reference_bundle']['name'])
        if reference.name!=str(reference) or sha(folder/reference)!=row['reference_bundle']['sha256']:return None
        backend=row.get('geometry_backend')
        if backend and sha(backend['path'])!=backend['sha256']:return None
        if row['native_dll_sha256']!=sha(ROOT/'build/ucrt64-release/python/fsim/bin/libfsim.dll'):return None
        return row
    except (KeyError,OSError,ValueError,TypeError):return None


def production(args):
    """Generate and install the complete production roster without test execution.

    Table shape/finiteness and mandatory-component rejection are part of the
    production writer. No test executable, flight sampler, regression runner or
    consumer probe is invoked by this entry point.
    """
    started=time.monotonic();records=roster();profile=None
    if args.scheduling_profile:profile=json.loads(Path(args.scheduling_profile).read_text(encoding='utf-8'))
    records=generation_order(records,profile)
    if not 1<=args.generation_workers<=16 or not 1<=args.geometry_threads<=16 or args.generation_workers*args.geometry_threads>16:
        raise ValueError('Production native CPU reservation must not exceed16')
    from . import resources
    cpu_pool=resources.CpuLeasePool(args.generation_workers,args.geometry_threads)
    out=Path(args.output).resolve();out.mkdir(parents=True,exist_ok=True)
    prefix=Path(args.prefix).resolve()
    if not prefix.is_relative_to(Path('D:/ENV')):raise ValueError('Install prefix must be under D:/ENV')
    batches=[]
    for kind in ('aam','fixed-wing','stores','rotorcraft'):
        group=[r for r in records if cohort(r)==kind]
        batches.extend(group[i:i+8] for i in range(0,len(group),8))
    batch_ids={r['name']:i+1 for i,group in enumerate(batches) for r in group}
    dump(out/'batch-plan.json',dict(schema=VERSION,mode='production-only',
        batches=[dict(id=i+1,designs=[r['name'] for r in group]) for i,group in enumerate(batches)],
        priority_order=['aam','fixed-wing','stores','rotorcraft'],
        scope='Reporting partitions only; complete aggregate production wall time is reported'))
    env=os.environ.copy();env.update(TMP='D:/ENV/tmp',TEMP='D:/ENV/tmp',TORCH_EXTENSIONS_DIR='D:/ENV/tmp',
        OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',FSIM_HANGAR_THREADS='1',
        FSIM_ENGINEERING_DEADLINE_OWNER='campaign',PYTHONDONTWRITEBYTECODE='1',
        HANGAR_MESHKIT_DIR=str(ROOT/'build/ucrt64-release/python'))
    env['PYTHONPATH']=os.pathsep.join(filter(None,[str(Path(__file__).parent.parent),env.get('PYTHONPATH','')]))
    phases=[];commands=[]
    def invoke(argv,log,cpus=None):
        tick=time.monotonic()
        remaining=CAMPAIGN_TIMEOUT_SECONDS-(time.monotonic()-started)
        if cpus is None:run_command(list(map(str,argv)),log,remaining,ROOT,env)
        else:
            code=resources.run_leased(list(map(str,argv)),log,remaining,cpus,cwd=ROOT,env=env)
            if code:raise RuntimeError('Command failed ('+str(code)+'): '+str(log))
        commands.append(dict(argv=list(map(str,argv)),log=str(log),elapsed_seconds=time.monotonic()-tick))
    def one(record):
        tick=time.monotonic();folder=out/record['name'];folder.mkdir(exist_ok=True)
        row=dict(record,status='blocked',reasons=[],commands=[],batch=batch_ids[record['name']])
        try:
            resumed=reusable_generation(folder,record) if args.resume else None
            if resumed is None:
                with cpu_pool.lease() as cpus:
                    invoke([sys.executable,'-B','-m','hangar','engineering','generate','--design',record['name'],
                        '--geometry-threads',args.geometry_threads,'--output',folder],folder/'generation.log',cpus)
            row.update(json.loads((folder/'generation.json').read_text(encoding='utf-8')))
            row['reasons']=[c['name'] for c in row['checks'] if not c['passed']]
            row['status']=engineering_status(row['reasons'])
            row['generation_resumed']=resumed is not None
        except Exception as exc:
            row['reasons']=['Required production component/table unavailable: '+str(exc)]
            (folder/'failure.txt').write_text(traceback.format_exc(),encoding='utf-8')
        row.update(generation_wall_seconds=time.monotonic()-tick,validation_wall_seconds=0.,
            runtime_evidence_scope='Generation-only stage; dynamic execution is reported separately',
            elapsed_seconds=time.monotonic()-tick)
        dump(folder/'result.json',row)
        print('produced '+record['name']+' '+row['status']+' '+format(row['elapsed_seconds'],'.2f')+'s',flush=True)
        return row
    with resources.cpu_lease(len(cpu_pool.cpus),cpus=cpu_pool.cpus):
        with ThreadPoolExecutor(max_workers=args.generation_workers) as pool:rows=list(pool.map(one,records))
    generation_seconds=time.monotonic()-started
    # All production writers join before the sole installer acquires the tree.
    if any(row['status']=='blocked' for row in rows):
        publish(out,rows,records)
        raise ValueError('Production roster contains a missing required component/table; installation refused')
    install_tick=time.monotonic();cmake=Path('D:/ENV/DevLanguages/Cpp/msys2/ucrt64/bin/cmake.exe')
    for component in ('sdk','python'):
        invoke([cmake,'--install',ROOT/'build/ucrt64-release','--component',component,'--prefix',prefix],out/('install-'+component+'.log'))
    installation_seconds=time.monotonic()-install_tick
    installed={};source={}
    for row in rows:
        for relative,expected in row['products'].items():
            current=sha(ROOT/'aircraft'/relative);delivered=sha(prefix/'share/flightsim/aircraft'/relative)
            if current!=expected or delivered!=expected:raise ValueError('Production package checksum mismatch: '+relative)
            source[relative]=current;installed[relative]=delivered
    dll=sha(ROOT/'build/ucrt64-release/bin/libfsim.dll')
    if sha(prefix/'bin/libfsim.dll')!=dll:raise ValueError('Installed native library checksum mismatch')
    dump(out/'installed-production-manifest.json',dict(schema='fsim.sim-m02.production-install.v1',
        prefix=str(prefix),native_dll_sha256=dll,products=installed,product_count=len(installed),
        scope='Passive source/package byte identity only; no SDK execution or test performed'))
    publish(out,rows,records)
    dump(out/'campaign-timing.json',dict(schema='fsim.sim-m02.production-generation.v1',design_count=len(rows),
        elapsed_seconds=time.monotonic()-started,generation_phase_seconds=generation_seconds,
        single_install_phase_seconds=installation_seconds,serial_native_validation_phase_seconds=0.,
        dynamic_validation_executed=False,commands=commands,
        scope='Production generation and installation only. No tests, native sampling or consumer probes.'))
    print('production delivered '+str(len(rows))+' designs '+format(time.monotonic()-started,'.3f')+'s',flush=True)
    return 0


def campaign(args):
    # One aggregate clock includes every generation, installation, native check
    # and publication. Batches are reporting partitions, never timing waivers.
    started=time.monotonic()
    if not 1<=args.generation_workers<=16:raise ValueError('Generation workers must be between 1 and 16')
    if not 1<=args.geometry_threads<=16 or args.generation_workers*args.geometry_threads>16:
        raise ValueError('Generation workers times geometry threads must not exceed the 16-thread native CPU reservation')
    from . import resources
    cpu_pool=resources.CpuLeasePool(args.generation_workers,args.geometry_threads)
    records=roster();all_records=records
    order={'aam':0,'fixed-wing':1,'stores':2,'rotorcraft':3}
    scheduling=None;profile=None
    if getattr(args,'scheduling_profile',None):
        source=Path(args.scheduling_profile).resolve()
        if '.codex' in {p.casefold() for p in source.parts}:raise ValueError('Excluded scheduling input')
        profile=json.loads(source.read_text(encoding='utf-8'))
        scheduling=dict(path=str(source),sha256=sha(source),version=profile.get('schema'),scope='Scheduling only; no physical parameters, acceptance predicates or job omission')
    records=generation_order(records,profile)
    if args.cohort!='all':records=[r for r in records if cohort(r)==args.cohort]
    if args.design:
        selected=set(args.design.split(','));records=[r for r in records if r['name'] in selected]
        if len(records)!=len(selected):raise ValueError('Unknown design selection')
    out=Path(args.output).resolve();out.mkdir(parents=True,exist_ok=True)
    prefix=Path(args.prefix).resolve()
    if not prefix.is_relative_to(Path('D:/ENV')):raise ValueError('Isolated installation must be under D:/ENV')
    env=os.environ.copy()
    env.update(TMP='D:/ENV/tmp',TEMP='D:/ENV/tmp',TORCH_EXTENSIONS_DIR='D:/ENV/tmp',MPLCONFIGDIR='D:/ENV/cache/matplotlib',XDG_CACHE_HOME='D:/ENV/cache',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1',FSIM_ENGINEERING_DEADLINE_OWNER='campaign',FSIM_HANGAR_THREADS='1')
    # Geometry fitting must use the same compiled backend as normal Hangar
    # builds/tests. A missing Python search path must never silently switch
    # damage products to the unfitted fallback.
    env['HANGAR_MESHKIT_DIR']=str(ROOT/'build/ucrt64-release/python')
    batches=[]
    for kind in order:
        selected=[r for r in records if cohort(r)==kind]
        batches.extend(selected[i:i+8] for i in range(0,len(selected),8))
    previous=json.loads((out/'matrix.json').read_text(encoding='utf-8')) if args.append and (out/'matrix.json').is_file() else None
    retained=[r for r in (previous or {}).get('designs',[]) if r['name'] not in {r['name'] for r in records}]
    offset=max((r['batch'] for r in retained),default=0)
    batch_ids={r['name']:i+1+offset for i,group in enumerate(batches) for r in group}
    head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    dump(out/'batch-plan.json',dict(schema=VERSION,reference_target_seconds=1200,
        operational_campaign_timeout_seconds=CAMPAIGN_TIMEOUT_SECONDS,generation_workers=args.generation_workers,native_threads_per_design=1,geometry_threads_per_design=args.geometry_threads,table_threads_per_design=1,native_workers=1,
        requested_cohort=args.cohort,priority_order=list(order),main_head=head,
        scheduling_profile=scheduling,
        batches=[dict(id=i+1+offset,designs=[r['name'] for r in group]) for i,group in enumerate(batches)],
        phases=['parallel-disjoint-force-generation-and-full-static-audit','join-all-writers','single-sdk-python-install','serial-native-and-installed-flight-plus-native-sampling','publication']))
    commands=[]
    def invoke(command,log,owned,cwd=ROOT,environment=env,cpus=None):
        owned.append(dict(argv=[str(x) for x in command],log=str(log),cwd=str(cwd)))
        remaining=CAMPAIGN_TIMEOUT_SECONDS-(time.monotonic()-started)
        if cpus is None:run_command([str(x) for x in command],log,remaining,cwd,environment)
        else:
            code=resources.run_leased([str(x) for x in command],log,remaining,cpus,cwd=cwd,env=environment)
            if code:raise RuntimeError('Command failed ('+str(code)+'): '+str(log))
    def generate_one(record):
        tick=time.monotonic();folder=out/record['name'];folder.mkdir(exist_ok=True)
        result=dict(record,batch=batch_ids[record['name']],status='blocked',reasons=[],commands=[],generation_start_offset_seconds=tick-started)
        try:
            retained_generation=reusable_generation(folder,record) if getattr(args,'resume',False) else None
            if retained_generation:
                result['resumed_generation']=dict(receipt_sha256=sha(folder/'generation.json'),
                    original_generation_seconds=retained_generation['generation_seconds'],
                    scope='Previously completed current-source forced generation; every input/method/product/reference hash rechecked')
            else:
                with cpu_pool.lease() as cpus:
                    invoke([sys.executable,'-B','-m','hangar','engineering','generate','--design',record['name'],'--geometry-threads',str(args.geometry_threads),'--output',folder],folder/'generation.log',result['commands'],cpus=cpus)
            result.update(json.loads((folder/'generation.json').read_text(encoding='utf-8')))
        except subprocess.TimeoutExpired:
            result['timeout']=True;result['reasons'].append('Operational campaign timeout during generation')
        except Exception as exc:
            result['reasons'].append('Missing valid required generated table/component: '+str(exc));(folder/'failure.txt').write_text(traceback.format_exc(),encoding='utf-8')
        result['generation_wall_seconds']=time.monotonic()-tick
        print('generated '+record['name']+' '+format(result['generation_wall_seconds'],'.2f')+'s',flush=True)
        return result
    with resources.cpu_lease(len(cpu_pool.cpus),cpus=cpu_pool.cpus):
        with ThreadPoolExecutor(max_workers=args.generation_workers) as executor:
            generated=list(executor.map(generate_one,records))
    generation_phase=time.monotonic()-started
    build=ROOT/'build/ucrt64-release';cmake=Path('D:/ENV/DevLanguages/Cpp/msys2/ucrt64/bin/cmake.exe')
    install_tick=time.monotonic();install_error=None
    try:
        for component in ('sdk','python'):
            invoke([cmake,'--install',build,'--component',component,'--prefix',prefix],out/('install-'+component+'.log'),commands)
    except Exception as exc:install_error='Missing valid required installed products: '+str(exc)
    install_phase=time.monotonic()-install_tick
    probe=Path(__file__).with_name('engineering_runtime.py');validation_tick=time.monotonic()
    work=Path('D:/ENV/tmp/sim-m02-consumer');work.mkdir(parents=True,exist_ok=True)
    for result in generated:
        tick=time.monotonic();folder=out/result['name']
        if install_error:result['reasons'].append(install_error)
        if not result['reasons']:
            try:
                worker_env=env.copy()
                for key in ('PYTHONPATH','FSIM_DLL_DIR','FSIM_DATA_DIR','FSIM_AIRCRAFT_PATH','JSBSIM_ROOT'):worker_env.pop(key,None)
                for mode,sdk,data in [('native',build/'python',build/'share/flightsim'),('installed',prefix,prefix/'share/flightsim')]:
                    worker_env.update(FSIM_AIRCRAFT_PATH=str(data/'aircraft')+';'+str(data/'aircraft/stores'),FSIM_DLL_DIR=str(sdk/'fsim/bin'),PATH=str(sdk/'fsim/bin')+';'+os.environ.get('SystemRoot','C:/Windows')+'/System32')
                    command=[sys.executable,'-I','-B',probe,'--sdk',sdk,'--data',data,'--receipt',folder/'generation.json','--output',folder/(mode+'.json')]
                    if mode=='installed':command.extend(['--sampling-output',folder/'envelope.json'])
                    # The parent owns restoration even if the worker is killed.
                    target=data/'aircraft'/result['product_directory']/(result['name']+'.xml');original=target.read_bytes()
                    try:invoke(command,folder/(mode+'.log'),result['commands'],work,worker_env)
                    finally:
                        if target.read_bytes()!=original:target.write_bytes(original)
                result['runtime']={mode:json.loads((folder/(mode+'.json')).read_text(encoding='utf-8')) for mode in ('native','installed')}
                result['envelope']=json.loads((folder/'envelope.json').read_text(encoding='utf-8'))
                failed=[c['name'] for c in result['checks'] if not c['passed']]
                for mode,value in result['runtime'].items():failed.extend(mode+'/'+c['name'] for c in value['checks'] if not c['passed'])
                if result['envelope']['case_count']!=210 or result['envelope']['failures']:failed.append('required native sampled execution consistency')
                result['reasons']=['Missing valid required dimension/component/table: '+x for x in failed];result['status']=engineering_status(failed)
            except subprocess.TimeoutExpired:
                result['timeout']=True;result['reasons'].append('Operational campaign timeout during validation')
            except Exception as exc:
                result['reasons'].append('Missing valid required native/installed execution evidence: '+str(exc));(folder/'failure.txt').write_text(traceback.format_exc(),encoding='utf-8')
        result['validation_wall_seconds']=time.monotonic()-tick
        result['elapsed_seconds']=result['generation_wall_seconds']+result['validation_wall_seconds']
        result['batch_commands']=commands
        dump(folder/'result.json',result)
        print('validated '+result['name']+' '+result['status']+' '+format(result['validation_wall_seconds'],'.2f')+'s',flush=True)
    validation_phase=time.monotonic()-validation_tick
    results=retained+generated;publish(out,results,all_records)
    elapsed=time.monotonic()-started
    dump(out/'campaign-timing.json',dict(schema=VERSION,main_head=head,design_count=len(records),elapsed_seconds=elapsed,
        generation_phase_seconds=generation_phase,single_install_phase_seconds=install_phase,serial_native_validation_phase_seconds=validation_phase,
        publication_and_overhead_seconds=elapsed-generation_phase-install_phase-validation_phase,
        reference_target_seconds=1200,exceeds_twenty_minutes=elapsed>1200,operational_campaign_timeout_seconds=CAMPAIGN_TIMEOUT_SECONDS,
        baseline_seconds=2151.704024,speedup=2151.704024/elapsed,commands=commands,
        scheduling_profile=scheduling,
        optimization=['same knots, alpha-major traversal retains wake interpolation operators','one bounded '+str(args.generation_workers)+'-worker queue; table/BLAS threads1, geometry threads'+str(args.geometry_threads)+'; aggregate native reservation at most16','one SDK/Python installation after all generation writers join','installed flight and sampling share one isolated process','publish full matrix once','exact-byte immutable-scene point reuse; every original fit point and threshold retained','native union bounds use inline storage for small graphs without changing evaluation order',
            'observed longest-first generation inside each priority cohort' if scheduling else 'default identity scheduling; no cost profile']))
    for i,group in enumerate(batches,offset+1):
        batch_dir=out/('batch-%02d'%i);batch_dir.mkdir(exist_ok=True)
        rows=[r for r in generated if r['batch']==i]
        dump(batch_dir/'timing.json',dict(batch=i,designs=[r['name'] for r in group],
            generation_worker_seconds=sum(r['generation_wall_seconds'] for r in rows),serial_validation_seconds=sum(r['validation_wall_seconds'] for r in rows),
            limitation='Reporting partition; worker time is not wall time. Use campaign-timing.json for aggregate wall time.'))
    print('aggregate '+str(len(records))+' designs '+format(elapsed,'.3f')+'s',flush=True)
    return int(any(r['status']=='blocked' for r in results))


def publish(out, results, declared):
    closure={r['name']:r.get('mass_partition') for r in results}
    dump(out/'mass-partition-closure.json',dict(schema='fsim.mass-partition.closure.v1',
        declared_count=len(declared),design_count=len(closure),
        complete=set(closure)=={r['name'] for r in declared} and all(v and v.get('passed') for v in closure.values()),
        designs=closure))
    counts = {s: sum(r['status']==s for r in results) for s in ('engineering-complete','engineering-complete-uncalibrated','blocked')}
    dump(out/'matrix.json', dict(schema=VERSION, declared_count=len(declared), completed_count=len(results), counts=counts,
        reference_target_seconds=1200, sources=SOURCES, designs=results))
    with (out/'matrix.csv').open('w', newline='', encoding='utf-8') as f:
        columns=['6DOF','mass-CG-inertia','six aerodynamic axes','propulsion-massflow-nozzle','boost-sustain-coast','actuator-lag-rate-stops','control-mixer-autopilot','seeker','guidance','fuze','warhead','damage','release','reset']
        w=csv.writer(f);w.writerow(['design','category','status','batch','seconds','table_count','lifting_panels','internal_substeps',*columns,'generation_and_structure_checks','native_checks','installed_checks','reasons'])
        for r in results:
            fields=[('NA' if r.get('components',{}).get(c,{}).get('applicability')=='not_applicable' else r.get('components',{}).get(c,{}).get('present',False)) for c in columns]
            runtime=r.get('runtime',{})
            w.writerow([r['name'],r['category'],r['status'],r['batch'],r['elapsed_seconds'],r.get('table_count',0),r.get('lifting_panels'),r.get('integration',{}).get('substeps'),*fields,
                bool(r.get('checks')) and all(c['passed'] for c in r['checks']),
                all(c['passed'] for c in runtime['native']['checks']) if runtime.get('native') else 'not run',
                all(c['passed'] for c in runtime['installed']['checks']) if runtime.get('installed') else 'not run','; '.join(r['reasons'])])
    rows=[]
    for r in results:
        name=html.escape(r['name'])
        rows.append('<tr>'+''.join('<td>'+str(x)+'</td>' for x in [name,html.escape(r['category']),r['status'],format(r['elapsed_seconds'],'.2f'),r.get('table_count',0),r.get('lifting_panels',''),r.get('integration',{}).get('substeps','')])+
            '<td>'+html.escape('; '.join(r['reasons']))+'</td><td>'+ ' / '.join('<a href="'+name+'/'+file+'">'+label+'</a>' for file,label in [('result.json','full matrix row'),('generation.json','products and inputs'),('native.json','native'),('installed.json','installed')])+'</td></tr>')
    samples=sum(r.get('envelope',{}).get('case_count',0) for r in results)
    execution=('Native dynamic checks are sampling ('+str(samples)+' requested conditions total), not full-envelope validation.'
               if samples else 'No native dynamic sampling or consumer execution was performed in this production-only delivery.')
    (out/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>SIM-M02 all-design engineering completeness</title><style>body{font:15px system-ui;margin:24px}table{border-collapse:collapse}th,td{border:1px solid #aaa;padding:6px;vertical-align:top}</style><h1>SIM-M02 engineering completeness</h1><p>'+html.escape(str(counts))+'</p><p>One shared VLM/engineering pipeline. No CFD campaign or new measurement fitting. All table cells receive static checks. '+execution+' Aggregate wall time is in campaign-timing.json. A zero lifting-panel count is the body-only physical limit, not an invented wing.</p><p><a href="matrix.json">Complete JSON matrix</a> / <a href="matrix.csv">Component CSV matrix</a> / <a href="batch-plan.json">Declared batch plan</a></p><table><tr><th>Design</th><th>Category</th><th>Status</th><th>Seconds</th><th>Tables</th><th>VLM panels</th><th>Native substeps</th><th>Missing required components/tables</th><th>Evidence</th></tr>'+''.join(rows)+'</table>',encoding='utf-8')


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['campaign','generate','production'])
    parser.add_argument('--output', required=True)
    parser.add_argument('--prefix', default='D:/ENV/sim-m02-engineering-20261030')
    parser.add_argument('--design')
    parser.add_argument('--scheduling-profile',help='Versioned observed wall costs; default preserves original ordering and all numerical behavior')
    parser.add_argument('--geometry-threads',type=int,default=2,help='Verified geometry CPU lease (default2); explicit1 stays serial; workers times this value must not exceed16')
    parser.add_argument('--cohort',choices=['all','aam','fixed-wing','stores','rotorcraft'],default='all')
    parser.add_argument('--append',action='store_true',help='retain completed disjoint cohorts in the same full-roster matrix')
    parser.add_argument('--resume',action='store_true',help='Reuse only completed generation receipts whose full input/method/product/reference hashes remain current; all native execution is rerun')
    parser.add_argument('--generation-workers',type=int,default=8,help='Bounded disjoint generation queue; one nested native thread per design')
    args=parser.parse_args(argv)
    if args.mode=='production':return production(args)
    if args.mode=='generate':
        if os.environ.get('FSIM_ENGINEERING_DEADLINE_OWNER') != 'campaign':
            env=os.environ.copy();env['FSIM_ENGINEERING_DEADLINE_OWNER']='campaign'
            from . import resources
            folder=Path(args.output);folder.mkdir(parents=True,exist_ok=True)
            log=folder/'standalone-generation.log'
            try:
                with resources.cpu_lease(args.geometry_threads) as cpus:
                    print('Leased standalone generation log: '+str(log),flush=True)
                    code=resources.run_leased([sys.executable,'-B','-m','hangar','engineering',*(argv if argv is not None else sys.argv[1:])],
                        log,DEADLINE_SECONDS,cpus,env=env)
                if log.is_file():print(log.read_text(encoding='utf-8'),end='',flush=True)
                return code
            except subprocess.TimeoutExpired:
                dump(Path(args.output)/'timeout.json', dict(status='blocked',deadline_seconds=DEADLINE_SECONDS,
                    reason='Standalone generation exceeded hard deadline'))
                return 1
        record=next(r for r in roster() if r['name']==args.design)
        folder=Path(args.output);folder.mkdir(parents=True,exist_ok=True)
        generate(record,folder,args.geometry_threads)
        return 0
    return campaign(args)


if __name__=='__main__':
    sys.exit(main())
