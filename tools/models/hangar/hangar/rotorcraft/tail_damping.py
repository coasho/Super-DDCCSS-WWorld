"""Published yaw-derivative transfer to a passive physical tail-rotor port.

The excess is frozen at a source-bound native hover operating point. It matches
the declared normal-flow DC yaw derivative, not an entire frequency response or
all lateral/roll derivatives. The positive excess is instantaneous; its unknown
high-frequency accuracy remains an explicit engineering-estimate limitation.
"""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from .. import pipeline
from ..storecontrol import _op, _property, _value

VERSION = 'fsim.rotor-tail-damping.v1'
PREFIX = 'fsim/rotor-tail-damping-v1'


def declaration(spec):
    tail = spec.get('rotor', {}).get('tail', {})
    if 'damping_lb_per_fps' not in tail:
        return None
    config = tail.get('damping_model', {})
    if (set(config) != {'version', 'method', 'source', 'basis'}
            or type(config.get('version')) is not int or config['version'] != 1
            or config['method'] != 'native-hover-passive-dc-yaw-excess'
            or config['basis'] != 'estimated' or not config['source'].strip()):
        raise ValueError('Authored tail damping requires a versioned sourced transfer method')
    target = float(tail['damping_lb_per_fps'])
    arms = tail.get('damping_arm_ft', [])
    if len(arms) != 2 or not all(math.isfinite(v) and v > 0 for v in [target, *arms]):
        raise ValueError('Tail damping requires finite positive source derivative and arms')
    return config


def source_hashes(path):
    import os
    root = Path(pipeline.repo_root())
    path = Path(path).resolve()
    here = Path(__file__).resolve()
    files = [Path(path).resolve(), here, here.with_name('heli.py'), here.with_name('fly.py'),
             here.with_name('__init__.py'), here.parents[1] / 'actuator_models.py',
             root / 'cmake/JsbsimPatches.cmake', root / 'third_party/jsbsim/src/models/propulsion/FGRotor.cpp']
    for directory, folders, names in os.walk(here.parents[1]):
        folders[:] = [name for name in folders if name not in ('.codex', '__pycache__')]
        files += [Path(directory) / name for name in names if Path(name).suffix in ('.py', '.dat', '.csv')]
    files += [p for p in (Path(path).parent / 'calibration.toml',) if p.is_file()]
    return {p.relative_to(root).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(set(files))}


def derive(spec, point):
    from .heli import RHO0
    declaration(spec)
    tail = spec['rotor']['tail']
    if not .01 <= tail.get('tip_loss', 1.) <= 1. or not 1e-6 <= tail.get('inflow_lag_s', .1) <= 2.:
        raise ValueError('Tail derivative declaration exceeds native rotor parameter bounds')
    if tail.get('ground_effect_g1', 0):
        raise ValueError('Tail derivative transfer does not support ground-effect coupling')
    radius = float(tail['radius_ft'])
    chord = float(tail.get('chord_ft', tail.get('solidity', 0) * math.pi * radius / tail['blades']))
    rho, rpm, mu, lam, ct = (point[k] for k in ('rho', 'rpm', 'mu', 'lambda', 'ct'))
    if not all(math.isfinite(v) for v in (rho, rpm, mu, lam, ct)) or rho <= 0 or rpm <= 0 or ct <= 0:
        raise ValueError('Tail derivative needs a finite positive-thrust native operating point')
    solidity = tail['blades'] * chord / (math.pi * radius)
    tip = rpm * 2 * math.pi / 60 * radius
    c_lambda = .5 * tail['lift_slope'] * solidity * (.5 * tail.get('tip_loss', 1.)**2 + .25 * mu**2)
    s = math.hypot(mu, lam)
    if s <= 0:
        raise ValueError('Singular native inflow operating point')
    # Differentiate the exact regularized native momentum expression.
    denominator = 2 * s + 1e-15
    h = c_lambda / denominator - ct * (2 * lam / s) / denominator**2
    if not math.isfinite(h) or h <= 0:
        raise ValueError('Hover inflow derivative does not have the supported stable passive pole')
    k_inst = rho * math.pi * radius**2 * tip * c_lambda
    k_dc = k_inst / (1 + h)
    pole = math.exp(-point['native_dt_s'] / tail.get('inflow_lag_s', .1))
    native_pole = pole - (1 - pole) * h
    if not -1 < native_pole < 1:
        raise ValueError('Native inflow recurrence is unstable at the recorded step')
    cg, hub = spec['mass']['cg_in'], tail['hub_in']
    arm = [-(hub[0] - cg[0]) / 12, (hub[1] - cg[1]) / 12, -(hub[2] - cg[2]) / 12]
    cant = math.radians(tail.get('cant_deg', 0))
    normal = [0., (1. if tail.get('thrust', 'right') == 'right' else -1.) * math.cos(cant), -math.sin(cant)]
    yaw_port = arm[0] * normal[1]
    if abs(yaw_port) <= 1e-12:
        raise ValueError('Physical tail port has no yaw authority')
    nominal_tip = tail.get('tip_speed_fps', tail.get('rpm', 0) * 2 * math.pi / 60 * radius)
    target_yaw = tail['damping_lb_per_fps'] * (rho / RHO0) * (tip / nominal_tip) * tail['damping_arm_ft'][0]**2
    target_normal = target_yaw / yaw_port**2
    excess = target_normal - k_dc
    if not math.isfinite(excess) or excess < 0:
        raise ValueError('Source yaw derivative cannot be obtained by passive supplementation at this port')
    return dict(normal=normal, physical_arm_ft=arm, native_instantaneous_lb_per_fps=k_inst,
        native_dc_lb_per_fps=k_dc, target_normal_dc_lb_per_fps=target_normal,
        excess_dc_lb_per_fps=excess, native_linearized_inflow_time_s=tail.get('inflow_lag_s', .1) / (1 + h),
        native_inflow_pole=native_pole, inflow_slope=h,
        target_yaw_damping_lb_ft_s=target_yaw,
        source_arms_ft=tail['damping_arm_ft'], reference_cg_in=spec['mass']['cg_in'],
        limitation='Normal-flow DC yaw transfer at reference CG only. Instantaneous positive excess adds damping at all frequencies; lateral/roll and transient matching are not claimed. Power accounting is relative to the air, including wind energy.')


def identify(r, vehicle):
    import fsim
    import ctypes
    tail = 'propulsion/engine[1]/'
    point = dict(rho=vehicle.get_property('atmosphere/rho-slugs_ft3'),
        rpm=vehicle.get_property(tail + 'rotor-rpm'), mu=vehicle.get_property(tail + 'advance-ratio'),
        **{'lambda': vehicle.get_property(tail + 'inflow-ratio')},
        ct=vehicle.get_property(tail + 'thrust-coefficient'), native_dt_s=vehicle.get_property('simulation/dt'))
    directory = Path(r.dir)
    acquired = [directory / (r.name + '.xml'), *sorted((directory / 'Engines').glob('*.xml'))]
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    kernel.GetModuleHandleW.argtypes = [ctypes.c_wchar_p]
    kernel.GetModuleHandleW.restype = ctypes.c_void_p
    kernel.GetModuleFileNameW.argtypes = [ctypes.c_void_p, ctypes.c_wchar_p, ctypes.c_uint]
    kernel.GetModuleFileNameW.restype = ctypes.c_uint
    buffer = ctypes.create_unicode_buffer(32768)
    handle = kernel.GetModuleHandleW('libfsim.dll')
    if not handle or not kernel.GetModuleFileNameW(handle, buffer, len(buffer)):
        raise RuntimeError('Cannot establish loaded native DLL provenance')
    native = Path(buffer.value)
    acquired += [native, Path(fsim._native.__file__)]
    result = dict(schema=VERSION, basis='estimated', operating_point_basis='simulated',
        acquisition_inputs={str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest() for p in acquired},
        source_hashes=source_hashes(r.path), source=declaration(r.spec)['source'], operating_point=point,
        derivation=derive(r.spec, point))
    (Path(r.dir) / 'tail-damping.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    return result


def install(root, spec, directory):
    if declaration(spec) is None:
        return
    fcs = root.find('flight_control')
    def prop(name, value):
        ET.SubElement(fcs, 'property', value='%.17g' % value).text = PREFIX + '/' + name
    prop('required', 1); prop('version', 1)
    path = Path(directory) / 'tail-damping.json'
    if not path.is_file():
        prop('available', 0)
        return
    reference = json.loads(path.read_text(encoding='utf-8'))
    source = Path(directory) / (spec['aircraft']['name'] + '.toml')
    if reference.get('schema') != VERSION or reference.get('source_hashes') != source_hashes(source):
        prop('available', 0)
        return
    data = derive(spec, reference['operating_point'])
    if data != reference['derivation']:
        raise ValueError('Tail damping derived parameters differ from their source-bound operating point')
    prop('available', 1); prop('excess-dc-lb-per-fps', data['excess_dc_lb_per_fps'])
    # Fixed structural force port, current CG: body arm signs match JSBSim.
    arm = []
    for index, axis in enumerate('xyz'):
        difference = _op('difference', _value(spec['rotor']['tail']['hub_in'][index]), _property('inertia/cg-' + axis + '-in'))
        arm.append(_op('product', _value((-1. if axis in 'xz' else 1.) / 12.), difference))
    # Evaluate in ExternalReactions, after MassBalance and Auxiliary. FCS
    # functions would see the previous native step's CG and air velocity.
    import copy
    def ar(axis): return copy.deepcopy(arm['xyz'.index(axis)])
    qy = _op('sum', _property('velocities/v-aero-fps'),
             _op('product', _property('velocities/r-aero-rad_sec'), ar('x')),
             _op('product', _value(-1.), _property('velocities/p-aero-rad_sec'), ar('z')))
    qz = _op('sum', _property('velocities/w-aero-fps'),
             _op('product', _property('velocities/p-aero-rad_sec'), ar('y')),
             _op('product', _value(-1.), _property('velocities/q-aero-rad_sec'), ar('x')))
    q = _op('sum', _op('product', _value(data['normal'][1]), qy), _op('product', _value(data['normal'][2]), qz))
    reaction = root.find('external_reactions')
    if reaction is None: reaction = ET.SubElement(root, 'external_reactions')
    force = ET.SubElement(reaction, 'force', name='passive-tail-damping-v1', frame='BODY')
    ET.SubElement(force, 'description').text = VERSION + ': frozen instantaneous passive excess; published DC yaw derivative, simulated hover point; transient response estimated'
    location = ET.SubElement(force, 'location', unit='IN')
    for axis, value in zip('xyz', spec['rotor']['tail']['hub_in']): ET.SubElement(location, axis).text = '%.17g' % value
    direction = ET.SubElement(force, 'direction')
    for axis, value in zip('xyz', data['normal']): ET.SubElement(direction, axis).text = '%.17g' % value
    ET.SubElement(force, 'function').append(_op('product', _value(-data['excess_dc_lb_per_fps']), q))
