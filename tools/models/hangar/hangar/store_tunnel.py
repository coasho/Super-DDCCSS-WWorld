"""Geometrically scaled, unpowered body-store wind-tunnel SDK fixtures.

Geometry and force equations still use the normal aircraft table/XML writer.
Mass/inertia scaling is a fixture convention, not measured model mass. Matching
Re uses solver_speed = Re * NU0 / reference_length; the SDK flow is independent.
No coefficient observations enter this transformation.
"""
import copy
import hashlib
import json
import math
import xml.etree.ElementTree as ET

from . import airframe, jsbsim
from .aero.model import NU0
from .files import atomic_write_text
from .reference_campaign import sha


def fixture(runner, parameters, scale, reynolds, length):
    spec=copy.deepcopy(runner.design.aircraft.spec)
    root=ET.fromstring(runner.original)
    if not runner.design.store or any(spec.get(k) for k in ('surface','intake','strut','gear','engine','motor')) or root.findall('./propulsion/engine'):
        raise ValueError('Scaled tunnel fixtures require an unpowered body-only store')
    if not math.isfinite(scale) or not 0.<scale<=1.:
        raise ValueError('Invalid tunnel geometry scale')
    if any(not isinstance(v,(int,float)) or not math.isfinite(v) or v<=0. for v in (reynolds,length)):
        raise ValueError('Positive source Reynolds and scaled reference length are required')
    # A wind-tunnel balance is not a box resting on a runway. Ground contact
    # at the sea-level reference altitude must not alter the imposed flow.
    ground=root.find('ground_reactions')
    if ground is not None:ground.clear()
    for body in spec['body']:
        for station in body['stations']:
            for key in ('x','w','top','bottom'):station[key]*=scale
    reference=spec['reference']
    reference['area']*=scale**2
    for key in ('span','chord'):reference[key]*=scale
    reference['aero_point']=[x*scale for x in reference['aero_point']]
    spec['mass']['empty']*=scale**3
    spec['mass']['empty_cg']=[x*scale for x in spec['mass']['empty_cg']]
    for key in ('length','span'):spec['dimensions'][key]*=scale
    for key in ('width','height'):
        if key in spec.get('store',{}):spec['store'][key]*=scale
    spec.setdefault('analysis',{})['speed']=reynolds*NU0/length
    key=hashlib.sha256(json.dumps(spec,sort_keys=True).encode()).hexdigest()
    folder=runner.scratch/('scale-'+key[:16]);folder.mkdir(parents=True,exist_ok=True)
    design=airframe.design(folder/(runner.name+'.toml'),spec=spec)
    design.aircraft.calibration=dict(parameters)
    for node in root.findall('.//location'):
        for child in node:
            if child.text:child.text=format(float(child.text)*scale,'.17g')
    metrics=root.find('metrics')
    for tag,power in (('wingarea',2),('wingspan',1),('chord',1)):
        node=metrics.find(tag);node.text=format(float(node.text)*scale**power,'.17g')
    balance=root.find('mass_balance')
    for node in balance:
        power=3 if node.tag=='emptywt' else 5 if node.tag in ('ixx','iyy','izz','ixy','ixz','iyz') else 0
        if power:node.text=format(float(node.text)*scale**power,'.17g')
    root.remove(root.find('aerodynamics'))
    root.append(ET.fromstring(jsbsim.aerodynamics_xml(design.calibrated(design.tables()),design.aircraft)))
    atomic_write_text(runner.scratch/'scaled-fixture.json',json.dumps(dict(
        source_sha256=sha(runner.source),geometry_scale=scale,solver_reynolds=reynolds,
        reference_length_m=length,solver_speed_ms=spec['analysis']['speed'],
        kinematic_viscosity_m2_s=NU0,geometry_sha256=key,
        mass_convention='Uniform geometric scaling: mass s^3, inertia s^5; not measured tunnel-model mass.'),indent=2)+'\n')
    return root
