"""Executed method references for six SIM-M03 flight/control component families.

This tool never changes cards, generated models, runtime code, or native state.
Ports are explicitly labelled; only imported Hangar functions and separately
supplied native dumps count as production execution. Output is exact JSON.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import itertools
import importlib
import json
import math
import os
import re
from pathlib import Path
import sys
import tomllib
import xml.etree.ElementTree as ET
from acceptance import allowed

ROOT = Path(__file__).resolve().parents[3]
FAMILIES = ('guidance', 'autopilot', 'allocator', 'actuator', 'aerodynamics', 'propulsion')
G0 = 9.80665


def guard_path(path):
    return allowed(path)


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=False,
                      allow_nan=False).encode('utf-8')


def digest(path):
    return hashlib.sha256(guard_path(path).read_bytes()).hexdigest()


def parameter_payload(card, family):
    descriptor = card['component_models']['components'][family]
    return dict(descriptor=descriptor, parameters={role: card['parameters'][ref]
                for role, ref in descriptor.get('parameter_refs', {}).items()})


def parameter_hash(card, family):
    return hashlib.sha256(canonical(parameter_payload(card, family))).hexdigest()


def roster(root=ROOT):
    """Catalogue identities intersected with actual AAM cards; never aliases."""
    root=guard_path(root)
    catalogue = json.loads((root/'tools/models/hangar/hangar/data/equipment_models.json').read_text(encoding='utf-8'))
    rows = []
    for model in catalogue['models']:
        path = root/model['card_path']
        if any(part.casefold() == '.codex' for part in path.parts):
            raise ValueError('Excluded source identity')
        card = json.loads(path.read_text(encoding='utf-8'))
        if card.get('class') != 'aam':
            continue
        if card['id'] != model['store_id'] or card['model'] != model['native_model']:
            raise ValueError('Catalogue/card identity mismatch')
        if any(card['component_models']['components'][f]['availability'] != 'available' for f in FAMILIES):
            raise ValueError('Missing applicable component')
        rows.append((model, card, path))
    if len(rows) != 24 or len({c['id'] for _, c, _ in rows}) != 24:
        raise ValueError('The independently enumerated AAM population is not 24 unique cards')
    return sorted(rows, key=lambda row: row[1]['id'])


def resolved(card, family):
    return {role: row['value'] for role, row in parameter_payload(card, family)['parameters'].items()}


def residual(name, actual, expected, tolerance, **extra):
    import numpy as np
    a, b = np.asarray(actual, dtype=float), np.asarray(expected, dtype=float)
    error = float(np.max(np.abs(a-b)))
    if not math.isfinite(error) or error > tolerance:
        raise AssertionError((name, error, tolerance, actual, expected))
    return dict(name=name, actual=a.tolist(), expected=b.tolist(), max_absolute_error=error,
                tolerance=tolerance, passed=True, **extra)


def measured_pn_port(direction, rate, velocity, up, navigation_gain, scale_speed, maximum_g):
    """Source-bound algebra port of MeasuredPnGuidanceV2; not native execution."""
    import numpy as np
    d, rate, velocity, up = map(lambda x: np.asarray(x, dtype=float), (direction, rate, velocity, up))
    heading = velocity/np.linalg.norm(velocity)
    steering = navigation_gain*scale_speed*np.cross(np.cross(d, rate), heading)
    steering -= heading*np.dot(heading, steering)
    command = steering+G0*up/np.linalg.norm(up)
    return command*min(1., maximum_g*G0/np.linalg.norm(command))


def pn_geometric_reference(direction, rate, velocity, up, gain, scale_speed, maximum_g, step=1e-3):
    """Different route: symmetric differentiated unit bearing and triple product."""
    import numpy as np
    d, rate, velocity, up = map(lambda x: np.asarray(x, dtype=float), (direction, rate, velocity, up))
    plus, minus = d+step*rate, d-step*rate
    derivative = (plus/np.linalg.norm(plus)-minus/np.linalg.norm(minus))/(2*step)
    h = velocity/np.linalg.norm(velocity)
    steering = gain*scale_speed*(derivative*np.dot(d, h)-d*np.dot(derivative, h))
    steering -= h*np.dot(h, steering)
    command = steering+G0*up/np.linalg.norm(up)
    return command*min(1., maximum_g*G0/np.linalg.norm(command))


def cascade_port(control, *, matrix_reference=False):
    """Current finite cascade versus independent force/moment trim solve.

    Synthetic signed SI derivatives define a disclosed stable three-axis plant.
    The card's exact response and actuator parameters remain model specific.
    """
    import numpy as np
    tau = control['lag_s']; rate_time = max(control['rate_time_min_s'], control['rate_lag_multiplier']*tau)
    angle_time = max(control['incidence_time_min_s'], control['incidence_rate_multiplier']*rate_time)
    mass, speed = 100., 500.
    rates = np.array([.03, .02, -.015]); inertia = np.array([1., 100., 100.])
    gyro = np.cross(rates, inertia*rates)
    states = (.03, -.02); demands = (-30., 12.); paths = (-(demands[0]+G0)/speed, demands[1]/speed)
    axes = [(-10000., -2000., -5000., -10000., -2000.),
            (-10000., 2000., 5000., -10000., -2000.)]
    references = []
    for axis, (fa, fd, ma, md, mr) in enumerate(axes):
        force, path, coupling = mass*demands[axis], paths[axis], gyro[axis+1]
        if matrix_reference:
            angle, _ = np.linalg.solve([[fa, fd], [ma, md]], [force, coupling-mr*path])
        else:
            angle = (force+fd*(mr*path-coupling)/md)/(fa-fd*ma/md)
        references.append(float(angle))
    limit = math.radians(control['max_incidence_deg']); norm = math.hypot(*references)
    if norm > limit:
        references = [x*limit/norm for x in references]
        # Once incidence is constrained, force and moment must share the same
        # achievable path. The matrix route solves for [surface,path] directly;
        # the separate algebra route eliminates surface from those equations.
        paths = list(paths)
        for axis,(fa,fd,ma,md,mr) in enumerate(axes):
            sign=1 if axis==0 else -1;gravity=G0 if axis==0 else 0.
            if matrix_reference:
                _,paths[axis]=np.linalg.solve([[fd,sign*mass*speed],[md,mr]],
                    [-fa*references[axis]-mass*gravity,gyro[axis+1]-ma*references[axis]])
            else:
                force=(fa-fd*ma/md)*references[axis]+fd*gyro[axis+1]/md
                paths[axis]=-sign*(force+mass*gravity)/(mass*speed-sign*fd*mr/md)
    commands = []
    for axis, (_, _, ma, md, mr) in enumerate(axes):
        wanted_rate = paths[axis]+(1 if axis == 0 else -1)*(references[axis]-states[axis])/angle_time
        moment = inertia[axis+1]*(wanted_rate-rates[axis+1])/rate_time+gyro[axis+1]
        delta = (moment-ma*states[axis]-mr*rates[axis+1])/md
        commands.append(float(np.clip(delta/.3, -1, 1)))
    roll = float(np.clip((-inertia[0]*rates[0]/rate_time+gyro[0]+100*rates[0])/(1000*.3), -1, 1))
    return [roll, *commands]


def actuator_aware_gains(control):
    """Source algebra port for PL-10 pole compensation; independent eigensolve below."""
    import numpy as np
    lag = 1/control['lag_s']; fast = 1/max(control['rate_time_min_s'], control['rate_lag_multiplier']*control['lag_s'])
    slow = 1/max(control['incidence_time_min_s'], control['incidence_rate_multiplier']/fast)
    p2, p1, p0 = lag+fast+slow, lag*fast+lag*slow+fast*slow, lag*fast*slow
    # angle_dot=aa*angle+b*rate+c*delta; rate_dot=d*angle+e*rate+f*delta.
    aa, b, c, d, e, f = -.2, 1., -.04, -50., -20., -100.
    det = aa*e-b*d; kd = (p2+aa+e)/lag-1
    u, v, y, z = c, f, b*f-c*e, c*d-aa*f
    rhs1 = (p1-det+lag*(1+kd)*(aa+e))/lag
    rhs0 = (p0-lag*(1+kd)*det)/lag
    ka = (rhs1*z-v*rhs0)/(u*z-v*y); kr = (u*rhs0-rhs1*y)/(u*z-v*y)
    A = np.array([[aa, b, c], [d, e, f], [0., 0., -lag]])
    B = np.array([[0.], [0.], [lag]])
    return A, B, np.array([ka, kr, kd]), np.array([-lag, -fast, -slow])


def actuator_aware_reference(control):
    """Independent state-space pole placement (SciPy YT), then equilibrium feedback."""
    import numpy as np
    from scipy.signal import place_poles
    rates=np.array([.03,.02,-.015]);j=np.array([1.,100.,100.]);gyro=np.cross(rates,j*rates)
    tr=max(control['rate_time_min_s'],control['rate_lag_multiplier']*control['lag_s'])
    ta=max(control['incidence_time_min_s'],control['incidence_rate_multiplier']*tr)
    poles=[-1/control['lag_s'],-1/tr,-1/ta]
    axes=[(-10000.,-2000.,-5000.,-10000.,-2000.,1.,.03,-30.,G0),
          (-10000.,2000.,5000.,-10000.,-2000.,-1.,-.02,12.,0.)]
    references=[];paths=[]
    for i,(fa,fd,ma,md,mr,sign,angle,demand,gravity) in enumerate(axes):
        path=-sign*(demand+gravity)/500.;paths.append(path)
        references.append(np.linalg.solve([[fa,fd],[ma,md]],[100*demand,gyro[i+1]-mr*path])[0])
    ratio=min(1.,math.radians(control['max_incidence_deg'])/np.linalg.norm(references))
    output=[cascade_port(control,matrix_reference=True)[0]]
    for i,(fa,fd,ma,md,mr,sign,angle,demand,gravity) in enumerate(axes):
        reference=references[i]*ratio;path=paths[i];moment_residual=-gyro[i+1]
        if ratio<1:
            force=(fa-fd*ma/md)*reference-fd*moment_residual/md
            path=-sign*(force+100*gravity)/(50000-sign*fd*mr/md)
        trim=-(ma*reference+mr*path+moment_residual)/md
        A=np.array([[fa/50000,sign,fd/50000],[ma/100,mr/100,md/100],[0,0,-1/control['lag_s']]])
        B=np.array([[0.],[0.],[1/control['lag_s']]])
        K=place_poles(A,B,poles).gain_matrix[0]
        command=trim-K@np.array([angle-reference,rates[i+1]-path,-trim])
        output.append(float(np.clip(command/.3,-1,1)))
    return output


def lag_rate_stops_port(inputs, tau, rate, dt, low, high):
    """Initialized-zero Tustin lag, independent rate history, then hard stops."""
    values = []; previous_input = previous_lag = previous_rate = 0.
    a, b = dt/(2*tau+dt), (2*tau-dt)/(2*tau+dt)
    for command in inputs:
        lag = a*(command+previous_input)+b*previous_lag
        actual_rate = min(previous_rate+rate*dt, max(previous_rate-rate*dt, lag))
        output = min(high, max(low, actual_rate))
        values.append(output)
        previous_input, previous_lag, previous_rate = command, lag, actual_rate
    return values


def bounded_least_squares_reference(B, wanted):
    """Exhaust all 3^3 active faces; independent SciPy TRF comparison below."""
    import numpy as np
    B, wanted = np.asarray(B, float), np.asarray(wanted, float)
    best = None
    for state in itertools.product((-1, 0, 1), repeat=3):
        fixed = [i for i, value in enumerate(state) if value]; free = [i for i, value in enumerate(state) if not value]
        command = np.array(state, dtype=float)
        if free:
            rhs = wanted-B[:, fixed]@command[fixed]
            command[free] = np.linalg.lstsq(B[:, free], rhs, rcond=None)[0]
        if np.any(np.abs(command) > 1+1e-13):
            continue
        command = np.clip(command, -1, 1); cost = float(np.sum((B@command-wanted)**2))
        if best is None or cost < best[0]:
            best = cost, command
    if best is None:
        raise AssertionError('No feasible box face')
    return best


CONTROL_ENTRIES={1:('finCommands','normalized_fcs_input'),
                 2:('actuatorAwareCommands','normalized_fcs_input'),
                 3:('OperatingPointAutopilotV1::commandsOperatingPointV1/operatingPointInputCommandsV1',
                    'finite_fcs_input_endpoint')}


def selected_control_identity(actual, algorithm):
    """A legacy diagnostic cannot stand in for the selected provider."""
    if algorithm not in CONTROL_ENTRIES:raise ValueError('Unsupported selected control mechanism')
    entry,stage=CONTROL_ENTRIES[algorithm]
    if (actual.get('selected_control_entry')!=entry or actual.get('selected_control_target_stage')!=stage or
        actual.get('operating_legacy_entry')!='operatingPointCommandsV1' or
        actual.get('operating_legacy_target_stage')!='instantaneous_virtual_angle'):
        raise ValueError('Selected control entry or signal stage differs from the actual mechanism')
    if actual.get('control_benchmark_mass_kg')!=100. or actual.get('control_benchmark_speed_m_s')!=500.:
        raise ValueError('Control method benchmark inputs differ; this is not a model-native flight')
    if not isinstance(actual.get('control_benchmark_scope'),str) or 'Synthetic' not in actual['control_benchmark_scope']:
        raise ValueError('Synthetic control benchmark scope is not explicitly disclosed')
    selected=actual.get('selected_operating_point')
    if (algorithm==3 and not isinstance(selected,dict)) or (algorithm!=3 and selected is not None):
        raise ValueError('Selected operating-point observation is missing or belongs to another mechanism')
    return entry,stage


def operating_benchmark_reference(control, stage):
    """Independent selected-stage target and SciPy constrained solution.

    The unchanged disclosed native bench has zero channel history, h=.01,
    one native substep and identity physical fins. This is method arithmetic,
    not the variant's mass, aerodynamics or a whole aircraft trajectory.
    """
    import numpy as np
    from scipy import signal,optimize
    nominal=np.asarray(cascade_port(control,matrix_reference=True),float)
    gain=np.array([1000.,-10000.,-10000.]);inertia=np.array([1.,100.,100.])
    base=np.array([-3.,-190.,-70.]);dt=.01;rate=control['rate_rad_s']*dt
    numerator,denominator,_=signal.cont2discrete(([1.],[control['lag_s'],1.]),dt,method='bilinear')
    endpoint=np.array([signal.lfilter(numerator[0],denominator,[.3*u])[0] for u in nominal])
    endpoint=np.clip(endpoint,-rate,rate);endpoint=np.clip(endpoint,-.3,.3)
    if stage=='instantaneous_virtual_angle':target=base+gain*.3*nominal
    elif stage=='finite_fcs_input_endpoint':target=base+gain*endpoint
    else:raise ValueError('Unknown independent operating-point target stage')
    bound=min(rate,.3*float(numerator[0,0]))
    fit=optimize.lsq_linear(np.diag(gain/inertia),(target-base)/inertia,
        bounds=(-bound,bound),tol=1e-13)
    return dict(nominal_input=nominal,target=target,virtual_endpoint=endpoint,
        predicted_moment=base+gain*fit.x,residual=float(np.linalg.norm((base+gain*fit.x-target)/inertia)),
        input_equivalence_domain=bool(np.all(np.abs(.3*nominal*float(numerator[0,0]))<rate)),
        scope='Unchanged 100kg/500m/s signed linear method bench; actual model-native flight is a separate gate')


def native_fin_matrix(xml):
    """Read actual generated physical-fin mixer expressions, not selector names."""
    import numpy as np
    tree = ET.parse(xml); rows = []; bounds = []
    for node in tree.findall('.//fcs_function'):
        if not node.get('name', '').startswith('fcs/fin-') or not node.get('name', '').endswith('-demand-rad'):
            continue
        gain = [0., 0., 0.]
        expression = node.find('function')
        if expression is None or len(expression) != 1:
            raise ValueError('Malformed authored mixer expression')
        terms = list(expression[0]) if expression[0].tag == 'sum' else [expression[0]]
        for term in terms:
            if term.tag == 'value' and float(term.text) == 0:
                continue  # Explicit passive/absent command authority.
            if term.tag != 'product':
                raise ValueError('Unsupported nonlinear authored mixer')
            name = term.findtext('property', '').strip()
            axis = next((i for i, channel in enumerate(('aileron', 'elevator', 'rudder'))
                         if name == 'fcs/'+channel+'-response-rad'), None)
            if axis is None:
                raise ValueError('Unsupported authored mixer term')
            gain[axis] += float(term.findtext('value'))
        rows.append(gain); bounds.append([float(node.findtext('./clipto/min')), float(node.findtext('./clipto/max'))])
    if not rows:
        raise ValueError('Actual model has no physical-fin mixers')
    return np.array(rows), bounds


def source_rows(paths):
    return [dict(path=str(p.relative_to(ROOT)).replace('\\', '/'), sha256=digest(p)) for p in sorted(set(paths))]


def read_reference_json(path):
    def unique(pairs):
        out={}
        for key,value in pairs:
            if key in out:raise ValueError('Duplicate reference JSON key: '+key)
            out[key]=value
        return out
    return json.loads(guard_path(path).read_text(encoding='utf-8'),object_pairs_hook=unique,
        parse_constant=lambda value: (_ for _ in ()).throw(ValueError('Nonfinite reference JSON: '+value)))


def source_epoch(root=ROOT):
    """Explicit current production/reference input closure; no tree enumeration."""
    from component_matrix import native_paths
    root=guard_path(root)
    paths={'tests/simulation/test_component_reference_native.cpp','tests/CMakeLists.txt','cmake/JsbsimPatches.cmake',
        'tools/validation/weapon_quality/component_reference_run.py',
        'tools/validation/weapon_quality/component_references_kinematics.py',
        'tools/validation/weapon_quality/test_component_references_kinematics.py',
        'tools/validation/weapon_quality/component_matrix.py','tools/validation/weapon_quality/acceptance.py',
        'tools/models/hangar/hangar/data/equipment_models.json',
        'tools/models/hangar/hangar/aero/vlm.py','tools/models/hangar/hangar/aero/section.py',
        'tools/models/hangar/hangar/geometry/aircraft.py','tools/models/hangar/hangar/geometry/surface.py',
        'tools/models/hangar/hangar/geometry/airfoil.py','tools/models/hangar/hangar/geometry/body.py',
        'tools/models/hangar/hangar/storecontrol.py','tools/models/hangar/hangar/storebodies.py',
        'tools/models/hangar/hangar/components.py','tools/models/hangar/hangar/ductedrocket.py',
        'tools/models/hangar/hangar/rolleron.py','tools/models/hangar/hangar/motorinputs.py',
        'src/simulation/flight/JsbsimModel.cpp','src/simulation/flight/PassiveRolleronV1.cpp',
        'src/simulation/mission_systems/weapon/Autopilot.cpp','src/simulation/mission_systems/weapon/OperatingPointControlV1.cpp',
        'src/simulation/mission_systems/weapon/PulseManagerV1.cpp',
        'third_party/jsbsim/src/FGFDMExec.cpp','third_party/jsbsim/src/models/FGPropulsion.cpp',
        'third_party/jsbsim/src/models/flight_control/FGActuator.cpp','third_party/jsbsim/src/models/flight_control/FGFilter.cpp',
        'third_party/jsbsim/src/models/propulsion/FGRocket.cpp','third_party/jsbsim/src/models/propulsion/FGNozzle.cpp',
        'third_party/jsbsim/src/models/propulsion/FGTank.cpp',
        'tests/simulation/test_finite_rocket_feed_native.cpp','tools/models/hangar/tests/test_finite_rocket_feed.py',
        'src/foundation/transport/third_party/picosha2/picosha2.h',
        'src/foundation/transport/third_party/picosha2/LICENSE','src/foundation/transport/third_party/picosha2/provenance.json'}
    for family in FAMILIES:paths.update(native_paths(family,root))
    for _,card,path in roster(root):
        model=card['model'].removeprefix('jsbsim:');xml=root/'aircraft/stores'/model/(model+'.xml')
        paths.update((path.relative_to(root).as_posix(),f'aircraft/{model}/{model}.toml',xml.relative_to(root).as_posix()))
        tree=ET.parse(guard_path(xml))
        for node in tree.findall('./propulsion/engine')+tree.findall('./propulsion/engine/thruster'):
            paths.add(f'aircraft/stores/{model}/Engines/'+node.get('file')+'.xml')
    pending=[root/p for p in paths if Path(p).suffix in ('.cpp','.h')]
    visited=set()
    while pending:
        path=guard_path(pending.pop())
        if path in visited:continue
        visited.add(path)
        if len(visited)>2048:raise ValueError('Reference include closure exceeds its explicit bound')
        for name in re.findall(r'^\s*#include\s+"([^"]+)"',path.read_text(encoding='utf-8'),re.M):
            guard_path(name)
            if Path(name).is_absolute():raise ValueError('Absolute quoted reference include')
            candidates=(path.parent/name,root/'src'/name,root/'include'/name,root/'third_party/jsbsim/src'/name)
            for candidate in candidates:
                candidate=guard_path(candidate)
                if not candidate.is_file():continue
                candidate=guard_path(candidate.resolve())
                if not candidate.is_relative_to(root.resolve()):raise ValueError('Reference include escapes source root')
                relative=candidate.relative_to(root.resolve()).as_posix()
                if relative not in paths:paths.add(relative);pending.append(candidate)
                implementation=candidate.with_suffix('.cpp')
                if implementation.is_file():
                    rel=implementation.relative_to(root.resolve()).as_posix()
                    if rel not in paths:paths.add(rel);pending.append(implementation)
                break
    return dict(schema='fsim.sim-m03.component-kinematics-inputs.v1',sources=[
        dict(path=p,sha256=digest(root/guard_path(p))) for p in sorted(paths)])


def validate_capture_run(native_paths,epoch_path,receipt_path,root=ROOT):
    """Require current sources, capture-side epoch/exe and real Catch execution."""
    root=guard_path(root);epoch_path=guard_path(epoch_path);receipt_path=guard_path(receipt_path)
    native_paths=[guard_path(p) for p in native_paths]
    root=root.resolve();epoch_path=epoch_path.resolve();receipt_path=receipt_path.resolve()
    native_paths=[p.resolve() for p in native_paths]
    if read_reference_json(epoch_path)!=source_epoch(root):raise ValueError('Native input epoch is stale or incomplete')
    receipt=read_reference_json(receipt_path)
    if receipt.get('schema')!='fsim.sim-m03.component-reference-native-run.v1' or receipt.get('passed') is not True:
        raise ValueError('Passing actual reference run receipt required')
    def bound_path(binding):
        if not isinstance(binding,dict) or set(binding)!={'path','sha256'}:raise ValueError('Exact run path/hash binding required')
        path=guard_path(binding['path']);path=path if path.is_absolute() else root/path;path=guard_path(path.resolve())
        if not (path.is_relative_to(root) or path.is_relative_to(Path('D:/ENV').resolve())):raise ValueError('Foreign reference evidence root')
        if binding['sha256']!=digest(path):raise ValueError('Reference run bytes changed')
        return path
    matches=[]
    for execution in receipt.get('executions',[]):
        raw={}
        for binding in execution.get('raw',[]):
            path=bound_path(binding)
            if path in raw:raise ValueError('Duplicate reference run raw path')
            raw[path]=binding['sha256']
        if all(path in raw for path in native_paths):matches.append((execution,raw))
    if len(matches)!=1:raise ValueError('Exactly one matching actual native execution required')
    execution,raw=matches[0];command=execution.get('command');environment=execution.get('environment',{})
    if execution.get('exit_code')!=0 or not isinstance(command,list) or len(command)!=6 or command[1:5]!=['[component-reference-native]','--reporter','junit','--out']:
        raise ValueError('Exact successful Catch reference command required')
    executable=guard_path(command[0]).resolve();report=guard_path(command[5]).resolve()
    if report not in raw or epoch_path not in raw:raise ValueError('Missing actual Catch report or pre-execution epoch binding')
    if guard_path(environment.get('FSIM_COMPONENT_REFERENCE_EPOCH','')).resolve()!=epoch_path:
        raise ValueError('Actual native epoch environment mismatch')
    if guard_path(environment.get('FSIM_COMPONENT_REFERENCE_JSON','')).resolve()!=native_paths[0]:
        raise ValueError('Actual native output environment mismatch')
    if len(native_paths)>1 and guard_path(environment.get('FSIM_COMPONENT_PROPULSION_JSON','')).resolve()!=native_paths[1]:
        raise ValueError('Actual powered native output environment mismatch')
    runtime={bound_path(binding):binding['sha256'] for binding in execution.get('runtime',[])}
    if len(runtime)!=len(execution.get('runtime',[])) or runtime.get(executable)!=digest(executable):raise ValueError('Unique actual executable runtime binding required')
    # Both headers belong to the same actual Catch invocation. Even a caller
    # comparing only the control families must bind the powered header's
    # observed modules; this does not turn its numeric residuals into a pass.
    capture_paths=[guard_path(environment[key]).resolve() for key in
                   ('FSIM_COMPONENT_REFERENCE_JSON','FSIM_COMPONENT_PROPULSION_JSON')]
    if len(set(capture_paths))!=2 or any(path not in raw for path in capture_paths):
        raise ValueError('Both actual native capture headers must be bound by this run')
    captures=[]
    for path in capture_paths:
        capture=read_reference_json(path);identity=capture.get('executable',{})
        if capture.get('input_epoch_sha256')!=digest(epoch_path):raise ValueError('Capture-side source epoch mismatch')
        if bound_path(identity)!=executable or identity['sha256']!=runtime[executable]:raise ValueError('Capture-side executable mismatch')
        captures.append(capture)
    from component_reference_run import verify_execution_runtime
    verify_execution_runtime(execution,captures,executable,'kinematics',root)
    report_tree=ET.parse(guard_path(report));report_root=report_tree.getroot()
    suites=[report_root] if report_root.tag=='testsuite' else report_root.findall('testsuite')
    if len(suites)!=1:raise ValueError('Exactly one actual Catch suite required')
    suite=suites[0]
    if any(suite.get(key)!='0' for key in ('errors','failures','skipped')) or int(suite.get('tests','0'))<=0:
        raise ValueError('Positive actual Catch assertions and explicit zero failure counters required')
    cases=suite.findall('testcase');expected={
        'All actual AAM card configurations execute numeric guidance and control method references',
        'Every actual AAM engine executes an independent powered propulsion reference bench'}
    if len(cases)!=2 or {c.get('name') for c in cases}!=expected or any(c.get('status')!='run' for c in cases):
        raise ValueError('Both exact actual native reference cases required')
    if any(report_root.iter(tag) and list(report_root.iter(tag)) for tag in ('failure','error','skipped')):
        raise ValueError('Failed/skipped Catch reference evidence')
    filters=suite.findall('./properties/property[@name="filters"]')
    if len(filters)!=1 or filters[0].get('value')!='[component-reference-native]':raise ValueError('Actual Catch filter mismatch')
    return dict(passed=True,cases=2,assertions=int(suite.get('tests')),run_receipt_sha256=digest(receipt_path),
        epoch_sha256=digest(epoch_path),executable=dict(path=str(executable),sha256=runtime[executable]))


def native_comparisons(result, native, root=ROOT):
    """Compare real outputs, never replace unavailable observations by a port."""
    import numpy as np
    from scipy import optimize, linalg
    if native.get('schema') != 'fsim.sim-m03.component-native.v1':
        raise ValueError('Unsupported actual native output')
    records = {r['model']: r for r in native['models']}
    if set(records) != {c['id'] for _, c, _ in roster(root)}:
        raise ValueError('Native population differs from independently derived population')
    for equipment, card, _ in roster(root):
        name=card['id']; actual=records[name]
        rows={f: next(r for r in result['families'][f] if r['model']==name) for f in FAMILIES}
        def add(f, check):
            rows[f]['method']['residuals'].append(check)
            rows[f]['method']['tests']+=1;rows[f]['method']['assertions']+=1
        expected_algorithm={'fsim.control.load_feedback_cascade':1,'fsim.control.actuator_aware_cascade.v1':2,
                            'fsim.control.operating_point_bounded.v1':3}[equipment['components']['autopilot']['mechanism_id']]
        if actual['autopilot_code']!=expected_algorithm:raise ValueError('Actual native selected autopilot mechanism differs from catalogue/card')
        entry,stage=selected_control_identity(actual,expected_algorithm)
        rows['autopilot']['method']['selected_native_entry']=entry
        rows['autopilot']['method']['selected_signal_stage']=stage
        rows['autopilot']['method']['control_benchmark_scope']=actual['control_benchmark_scope']
        rows['autopilot']['method']['model_native_flight_admission']=False
        declared_guidance=resolved(card,'guidance')
        add('guidance',residual('Actual native navigation gain resolves this exact card',actual['navigation_gain'],declared_guidance['navigation_gain'],0.))
        add('guidance',residual('Actual native acceleration limit resolves this exact card',actual['max_g'],declared_guidance['max_g'],0.))
        add('guidance', residual('Actual native measured PN versus independent geometric reference',
            actual['guidance_acceleration'], rows['guidance']['method']['residuals'][0]['expected'], 1e-7))
        controls=dict(resolved(card,'autopilot'),**resolved(card,'actuator'))
        add('actuator',residual('Actual native card-selected actuator time constant',actual['lag_s'],controls['lag_s'],0.))
        add('actuator',residual('Actual native card-selected actuator rate',actual['rate_rad_s'],controls['rate_rad_s'],0.))
        add('autopilot', residual('Actual native cascade versus independent 2x2 trim and Euler coupling',
            actual['cascade'],cascade_port(controls,matrix_reference=True),1e-12))
        if equipment['components']['autopilot']['mechanism_id']=='fsim.control.actuator_aware_cascade.v1':
            add('autopilot',residual('Actual actuator-aware C++ control versus independent SciPy state-space pole placement',
                actual['actuator_aware'],actuator_aware_reference(controls),1e-10))
        if expected_algorithm==1:
            add('autopilot',residual('Selected cascade entry returns its actual FCS input',
                actual['selected_control_commands'],actual['cascade'],0.))
        elif expected_algorithm==2:
            add('autopilot',residual('Selected actuator-aware entry returns its actual FCS input',
                actual['selected_control_commands'],actual['actuator_aware'],0.))
        # Homogeneous disclosed bench: exact one-step native actuator prediction
        # independently gives a diagonal box-constrained least-squares problem.
        a=.01/(2*controls['lag_s']+.01); rate=controls['rate_rad_s']*.01
        predicted=np.clip(a*.3*np.array([.5,-.4,.3]),-rate,rate)
        add('autopilot',residual('Actual operating-point fixed-command predictor versus Tustin/rate analytic law',
            actual['predicted_fixed_command_fins'],predicted,1e-12))
        gain=np.array([1000.,-10000.,-10000.]);inertia=np.array([1.,100.,100.])
        base=np.array([-3.,-190.,-70.]);target=np.array(actual['operating_requested_moment'])
        legacy=operating_benchmark_reference(controls,'instantaneous_virtual_angle')
        add('autopilot',residual('Retained legacy diagnostic requested moment has instantaneous virtual-angle units',
            target,legacy['target'],1e-8,scope='Legacy helper diagnostic; not the selected R77M provider proof'))
        bound=min(rate,.3*a)
        optimum=optimize.lsq_linear(np.diag(gain/inertia),(target-base)/inertia,
            bounds=(-bound,bound),tol=1e-13)
        expected_cost=float(np.linalg.norm((base+gain*optimum.x-target)/inertia))
        add('autopilot',residual('Actual bounded operating-point residual versus independent constrained linear least squares',
            actual['operating_residual'],expected_cost,1e-8,
            scope='Disclosed linear orthogonal bench only; no nonlinear global-optimality claim'))
        if expected_algorithm==3:
            selected=actual['selected_operating_point']
            reference=operating_benchmark_reference(controls,'finite_fcs_input_endpoint')
            if not reference['input_equivalence_domain']:
                raise ValueError('The declared linear input-equivalence bench reaches a rate boundary')
            add('autopilot',residual('Selected actual R77M provider finite input-stage target versus SciPy bilinear channel response',
                selected['requested_moment'],reference['target'],1e-8,scope=reference['scope']))
            add('autopilot',residual('Selected actual R77M physical allocation versus independent bounded least squares',
                selected['residual'],reference['residual'],1e-8,scope=reference['scope']))
            add('autopilot',residual('Selected actual R77M provider preserves nominal input on a matching unsaturated linear FCS',
                selected['commands'],reference['nominal_input'],1e-8,scope=reference['scope']))
            add('autopilot',residual('Selected actual R77M finite endpoint moment versus independent constrained solution',
                selected['predicted_moment'],reference['predicted_moment'],1e-7,scope=reference['scope']))
            add('autopilot',residual('Selected control capture is the genuine input-stage provider result',
                actual['selected_control_commands'],selected['commands'],0.))
        ca,sa,cb,sb=math.cos(.17),math.sin(.17),math.cos(-.09),math.sin(-.09)
        wind_basis=np.array([[ca*cb,-ca*sb,-sa],[sb,cb,0.],[sa*cb,-sa*sb,ca]])
        force=wind_basis@np.array([-12.,3.,-40.])
        transported=np.array([2.,-4.,6.])-np.cross([.1,-.2,.05],force)
        add('aerodynamics',residual('Actual native wind-force/body-CG moment conversion versus independent orthonormal basis transport',
            actual['body_cg_moment'],transported,1e-12,inputs=dict(DYL_N=[12,3,40],moment_Nm=[2,-4,6],
                alpha_rad=.17,beta_rad=-.09,reference_to_CG_body_m=[.1,-.2,.05])))
        model=card['model'].removeprefix('jsbsim:');xml=root/'aircraft/stores'/model/(model+'.xml')
        tree=ET.parse(xml); plant=actual['native_plant'];samples=plant['samples'];outer_dt=plant['dt_s']
        substep_node=tree.find('.//property[.="fsim/integration-v1/substeps"]')
        substeps=int(float(substep_node.get('value'))) if substep_node is not None else 1
        if substeps<1:raise ValueError('Invalid actual native integration subdivision')
        dt=outer_dt/substeps
        matrix,bounds=native_fin_matrix(xml)
        active=[int(f['index']) for f in plant['fins'] if f['kind']==1]
        if len(active)!=len(matrix):raise ValueError('Native driven-fin identities disagree with XML mixers')
        channels=('aileron','elevator','rudder'); expected_channels=[]
        for axis,ch in enumerate(channels):
            scale=tree.find(f'.//aerosurface_scale[@name="fcs/{ch}-control"]')
            lo=float(scale.findtext('./range/min'));hi=float(scale.findtext('./range/max'))
            demand=[s['command'][axis]*(hi if s['command'][axis]>=0 else -lo) for s in samples for _ in range(substeps)]
            expected_channels.append(lag_rate_stops_port(demand,controls['lag_s'],controls['rate_rad_s'],dt,lo,hi))
        native_step_channels=np.array(expected_channels).T
        expected_outer=native_step_channels[substeps-1::substeps]
        measured_channels=np.array([s['response'] for s in samples])
        add('actuator',residual('Actual FGActuator channels versus independent Tustin/rate/stops recurrence',
            measured_channels,expected_outer,1e-11,inputs=dict(outer_dt_s=outer_dt,actual_native_substeps=substeps,native_dt_s=dt)))
        demands=np.clip(native_step_channels@matrix.T,np.array(bounds)[:,0],np.array(bounds)[:,1])
        fins=np.zeros(len(active));expected_fins=[]
        for demand in demands:
            fins=np.clip(demand,fins-controls['rate_rad_s']*dt,fins+controls['rate_rad_s']*dt)
            fins=np.clip(fins,np.array(bounds)[:,0],np.array(bounds)[:,1]);expected_fins.append(fins.copy())
        add('allocator',residual('Actual authored native mixer plus physical-fin rate/stops versus independent XML matrix',
            np.array([s['fins'] for s in samples])[:,active],np.array(expected_fins)[substeps-1::substeps],1e-11))
        for passive in plant['passive_reference']:
            sign,radius,area,iw,drive_c,ratio,drag_c,bearing,ih,stiffness,damping,aerok,limit,rate=passive['config']
            q=.5*passive['density_kg_m3']*passive['speed_ms']**2
            drive=drive_c*q*area*radius;wind=drive*radius/(ratio*passive['speed_ms'])
            stiffness+=q*aerok
            K=np.array([[-(wind+bearing)/iw,0,0,drive/iw],[0,0,1,0],
                        [-iw*passive['roll_rad_s']/ih,-stiffness/ih,-damping/ih,0],[0,0,0,0.]])
            initial=np.array([0.,0.,0.,1.]);decay=max((wind+bearing)/iw,damping/ih,math.sqrt(stiffness/ih))
            count=max(1,math.ceil(.001/min(.001,.1/max(decay,1.))));h=.001/count
            Z=K*h;P=np.eye(4)+Z+Z@Z/2+Z@Z@Z/6+Z@Z@Z@Z/24
            discrete=np.linalg.matrix_power(P,10*count)@initial
            continuous=linalg.expm(K*passive['time_s'])@initial
            # This case deliberately stays away from mechanical constraints;
            # refuse it rather than compare a constrained trajectory to an LTI law.
            if abs(discrete[1])>=limit or abs(discrete[2])>=rate:
                raise ValueError('Passive linear reference reaches a physical stop/rate boundary')
            mapping=np.array([1,sign,sign]);expected=discrete[:3]*mapping;exact=continuous[:3]*mapping
            floating=1e-10*max(1.,float(np.max(abs(exact))))
            add('actuator',residual('Actual passive rolleron dynamics versus independent RK4 matrix polynomial',
                passive['state'],expected,floating,inputs=passive))
            allowance=float(np.max(abs(expected-exact)))+floating
            add('actuator',residual('Actual passive wheel/hinge versus continuous NACA-type state-space exponential',
                passive['state'],exact,allowance,discretization_error_bound=float(np.max(abs(expected-exact))),
                difference_reason='Fourth-order native time integration; independently computed matrix-polynomial truncation'))
        if equipment['components']['propulsion']['mechanism_id']=='fsim.propulsion.jsbsim_commanded_dual_pulse.v1':
            p=resolved(card,'propulsion')
            start=p['phase_0_duration_s']+p['phase_1_duration_s']+p['buildup_s']*(1-2/math.pi)
            add('propulsion',residual('Actual commandable dual-pulse native state sequence versus independent hybrid schedule',
                actual['pulse_states'],[1,2,3,3],0.,inputs=dict(second_start_s=start,
                    hold_limit_s=p['pulse_hold_limit_s'],reserve_speed_ms=p['pulse_reserve_speed_m_s'])))
            add('propulsion',residual('Native dual-pulse eligible ignition epoch versus sine-buildup integral',
                actual['pulse_start_s'],start,1e-12))
        for f in FAMILIES:rows[f]['native_numeric_coupling']=f in ('guidance','autopilot','allocator','actuator','propulsion')
    result['native_coupling_passed']=True
    result['retained_control_validation_context']=control_validation_context(root)
    return result


def control_validation_context(root=ROOT):
    """Link retained unit/World evidence without admitting it as this bench.

    These existing dated artifacts are supporting context only, not another
    collector, a mandatory clean-build input, or a source of method residuals.
    Fresh method/native/public/visual gates remain separately required.
    """
    root=guard_path(root);base='build/architecture-reviews/sim-m03-production-20261103/'
    definitions=[dict(kind='actual_axial_control_unit',models=['pl-10','r-77m'],
        scope='20 actual native invariant cases; force/moment/capped-incidence laws, not a flight',
        paths=['axial-control/green-first.xml','axial-control/green-first-LastTest.log'],
        sources={'src/simulation/mission_systems/weapon/Autopilot.cpp':'d01242837e55c61c6a8478236ad5bad92f5d8fd949cb811b03df2d198112107b',
                 'tests/simulation/test_missile_autopilot.cpp':'b782f7284e41a9dbc4bc322240f1aa6ec25a84d7f6a0c7bdf61b0aa7e55eb3c1'}),
        dict(kind='actual_world_total_axial_pre',models=['pl-10'],
            scope='Actual PL10 World PRE inputs, all 9 stages plus detonation/damage; same receipt retains failed R77 before input-stage repair',
            paths=['control-total-axial/actual-first-receipt.json','control-total-axial/actual-first-output/pl-10/control-pre.json'],
            sources={'src/simulation/runtime/world/World.cpp':'24d83bd7dfd469be7293b5badcf1d7d100406442c28f1cc457ceecb630c370aa',
                     'src/simulation/mission_systems/weapon/Autopilot.cpp':'d01242837e55c61c6a8478236ad5bad92f5d8fd949cb811b03df2d198112107b'}),
        dict(kind='actual_world_input_stage_pre',models=['r-77m'],
            scope='Actual R77 World input-stage all 9 stages plus detonation/damage; 624 exact PRE calls; not the 100kg method bench',
            paths=['control-input-stage/r77-first-receipt.json','control-input-stage/r77-first-output/r-77m/trace.csv'],
            sources={'src/simulation/mission_systems/weapon/OperatingPointControlV1.cpp':'4577e1a187137c1db449faa67100c5c98a3649580c247a4701e83cd2b8f9070c',
                     'src/simulation/mission_systems/components/implementations/autopilot/OperatingPointAutopilotV1.cpp':'e47bff21be6cabfd5cce5dbd84e0b4459b25196f680097f32e4eba4a0acf6caf',
                     'src/simulation/mission_systems/weapon/Autopilot.cpp':'d01242837e55c61c6a8478236ad5bad92f5d8fd949cb811b03df2d198112107b',
                     'src/simulation/runtime/world/World.cpp':'24d83bd7dfd469be7293b5badcf1d7d100406442c28f1cc457ceecb630c370aa'})]
    result=[]
    for item in definitions:
        artifacts=[]
        for suffix in item.pop('paths'):
            path=guard_path(root/(base+suffix))
            artifacts.append(dict(path=base+suffix,present=path.is_file(),sha256=digest(path) if path.is_file() else None))
        bindings=[]
        for name,expected in item.pop('sources').items():
            actual=digest(root/name)
            bindings.append(dict(path=name,recorded_sha256=expected,current_sha256=actual,matches=actual==expected))
        result.append(dict(**item,artifacts=artifacts,source_bindings=bindings,
            current_source_matches=all(v['matches'] for v in bindings),admitted_by_this_numeric_bench=False))
    return result


def finite_feed_reference(requested_lbs, tanks, feeds):
    """Forward native priority/share/drain law from PRE state, never POST data.

    The supplied physical batch uses min(share, usable), while the separately
    predicted POST ledger retains native binary64 subtraction. A short share
    is not redistributed; an empty
    tier may fall through, but a nonempty first tier cannot borrow the next.
    """
    if not math.isfinite(requested_lbs) or requested_lbs<0:
        raise ValueError('Invalid independent requested fuel batch')
    if len(set(feeds))!=len(feeds) or any(type(k) is not int or not 0<=k<len(tanks) for k in feeds):
        raise ValueError('Invalid independent feed identities')
    for tank in tanks:
        if any(not math.isfinite(tank[k]) or tank[k]<0 for k in ('contents','unusable')):
            raise ValueError('Invalid independent tank material state')
    candidates=[k for k in feeds if tanks[k]['selected'] and 0<tanks[k]['priority']<=len(tanks)
                and tanks[k]['contents']>tanks[k]['unusable']]
    priority=min((tanks[k]['priority'] for k in candidates),default=0)
    group=[k for k in candidates if tanks[k]['priority']==priority]
    post=[tank['contents'] for tank in tanks]
    supplied=0.
    for k in group:
        post[k]=max(tanks[k]['unusable'],tanks[k]['contents']-requested_lbs/len(group))
        supplied+=min(requested_lbs/len(group),tanks[k]['contents']-tanks[k]['unusable'])
    if group and all(tanks[k]['contents']-tanks[k]['unusable']>=requested_lbs/len(group) for k in group):
        supplied=requested_lbs
    return dict(priority=priority,group=group,post=post,supplied=supplied)


def powered_comparisons(result, native, root=ROOT):
    """Actual engine force, consumption, hybrid pulse and inlet inventory laws."""
    import numpy as np
    if native.get('schema')!='fsim.sim-m03.propulsion-reference-native.v1':
        raise ValueError('Unsupported actual powered engine capture')
    records={r['model']:r for r in native['models']}
    if set(records)!={c['id'] for _,c,_ in roster(root)}:raise ValueError('Powered native population mismatch')
    for equipment,card,_ in roster(root):
        row=next(r for r in result['families']['propulsion'] if r['model']==card['id'])
        observed=records[card['id']];values=np.array(observed['samples']);columns=observed['columns']
        if not len(values) or not np.all(np.isfinite(values)):raise ValueError('Missing/nonfinite powered observations')
        def v(name):return values[:,columns.index(name)]
        def add(name,actual,expected,tolerance,**inputs):
            checked=residual(name,actual,expected,tolerance)
            actual,expected=np.broadcast_arrays(np.asarray(actual,float),np.asarray(expected,float))
            worst=np.unravel_index(np.argmax(abs(actual-expected)),actual.shape)
            check=dict(name=name+' (maximum absolute residual)',actual=checked['max_absolute_error'],expected=0.,
                tolerance=tolerance,passed=True,worst_observation=dict(index=[int(index) for index in worst],actual=float(actual[worst]),expected=float(expected[worst])),
                native_observations=len(values),inputs=inputs)
            row['method']['residuals'].append(check);row['method']['tests']+=1;row['method']['assertions']+=1
        dt=observed['native_dt_s'];eps=np.finfo(float).eps
        n=observed['native_samples'];initial=observed['initial_fuel_kg']
        fuel_allowance=8*eps*n*max(1.,initial)
        add('Actual native tank consumption versus integral of measured engine mass flow',
            initial-observed['final_fuel_kg'],observed['flow_integral_kg'],fuel_allowance,
            tolerance_basis='8*float64_eps*native_samples*max(1,initial_fuel_kg)',native_dt_s=dt)
        model=card['model'].removeprefix('jsbsim:');folder=root/'aircraft/stores'/model
        xml=ET.parse(folder/(model+'.xml'));engines=xml.findall('./propulsion/engine')
        age=v('native_time_s')-dt
        for k,engine in enumerate(engines):
            flow=v(f'engine{k}_flow_kg_s');vacuum=v(f'engine{k}_vacuum_thrust_n');isp=v(f'engine{k}_isp_s')
            if np.max(vacuum)<=0 or np.max(flow)<=0:raise ValueError('An actual native engine never produced force or consumed fuel')
            allowance=64*eps*max(1.,float(np.max(vacuum)))
            feed_state=None
            if k==0:
                policy=xml.findall('./flight_control/property[.="fsim/rocket-feed-v1/engine[0]/version"]')
                if len(policy)!=1 or float(policy[0].get('value'))!=1:
                    raise ValueError('Actual explicit AAM engine lacks finite supply policy')
                feed=observed.get('feed_reference',{})
                if feed.get('schema')!='fsim.sim-m03.native-feed-forward.v1' or feed.get('checked_native_steps')!=n or feed.get('violations')!=0:
                    raise ValueError('Missing/refused actual every-native-step feed oracle')
                samples={item['state'][0]:item for item in feed.get('samples',[])}
                if len(samples)!=len(feed.get('samples',[])) or not set(v('native_sample'))<=samples.keys():
                    raise ValueError('Incomplete/duplicate actual retained PRE feed observations')
                feeds=[int(node.text) for node in engine.findall('feed')]
                if feed.get('feeds')!=feeds:raise ValueError('Actual native feeds differ from authored source')
                selected=[samples[index] for index in v('native_sample')]
                feed_state=np.array([item['state'] for item in selected])
                predicted=[];predicted_tanks=[];actual_tanks=[];predicted_force=[];groups=[];priorities=[]
                for item in selected:
                    state=item['state'];tanks=[dict(contents=t[0],unusable=t[1],priority=int(t[2]),selected=bool(t[3])) for t in item['tanks']]
                    request=state[1]/state[6]*dt
                    batch=finite_feed_reference(request,tanks,feeds)
                    predicted.append(batch['supplied']);groups.append(len(batch['group']));priorities.append(batch['priority'])
                    predicted_tanks.extend(batch['post'][j] for j in feeds)
                    actual_tanks.extend(item['tanks'][j][4] for j in feeds)
                    full=bool(batch['group']) and all(tanks[j]['contents']-tanks[j]['unusable']>=request/len(batch['group']) for j in batch['group'])
                    predicted_force.append(state[1] if full else batch['supplied']/dt*state[6])
                predicted=np.array(predicted)
                add('Every actual native step independent forward supply versus measured flow integral',
                    observed['engine_flow_integrals_kg'][0],feed['expected_consumed_kg'],fuel_allowance)
                for leaf,tolerance in [('maximum_supply_residual_kg',fuel_allowance),
                                       ('maximum_tank_residual_kg',fuel_allowance),
                                       ('maximum_flow_residual_kg_s',64*eps*max(1.,float(np.max(flow)))),
                                       ('maximum_force_residual_n',allowance)]:
                    add('Every actual native step '+leaf,feed[leaf],0.,tolerance)
                add('Native requested batch versus source-independent nominal force/Isp/dt',
                    feed_state[:,2]*.45359237,feed_state[:,1]/feed_state[:,6]*dt*.45359237,fuel_allowance)
                add('Native supplied batch versus PRE-state priority/equal-share/shortfall reference',
                    feed_state[:,3]*.45359237,predicted*.45359237,fuel_allowance)
                add('Native finite flow versus PRE-state independently predicted physical supply',
                    flow,predicted*.45359237/dt,64*eps*max(1.,float(np.max(flow))))
                add('Native finite force versus independent full or clipped physical supply',
                    vacuum,np.array(predicted_force)*4.4482216152605,allowance)
                add('Native per-feed POST contents versus independent forward native subtraction',
                    np.array(actual_tanks)*.45359237,np.array(predicted_tanks)*.45359237,fuel_allowance)
                add('Native selected feed group size versus independent priority reference',feed_state[:,11],groups,0.)
                add('Native selected feed group priority versus independent priority reference',feed_state[:,12],priorities,0.)
                add('Native Starved status versus PRE feed eligibility',feed_state[:,7],np.array(groups)==0,0.)
            add('Actual native engine%d vacuum force versus NASA Isp mass-flow relation'%k,vacuum,flow*isp*G0,allowance,
                equation='F_vac=Isp*g0*actual_mass_flow',engine=k)
            nozzle=ET.parse(folder/'Engines'/(engine.find('thruster').get('file')+'.xml'))
            area=nozzle.getroot().find('area');area_ft2=float(area.text)
            if area.get('unit')=='M2':area_ft2/=.3048**2
            elif area.get('unit')!='FT2':raise ValueError('Unsupported explicit nozzle area unit')
            expected=np.maximum(0.,vacuum-v(f'engine{k}_pressure_psf')*area_ft2*4.4482216152605)
            add('Actual native engine%d nozzle output versus pressure-area momentum balance'%k,
                v(f'engine{k}_delivered_thrust_n'),expected,allowance,area_ft2=area_ft2)
            engine_xml=ET.parse(folder/'Engines'/(engine.get('file')+'.xml'));table=engine_xml.getroot().find('thrust_table')
            isp_node=engine_xml.getroot().find('isp')
            if isp_node is not None and not len(isp_node):
                add('Actual native engine%d constant Isp versus independently parsed authored engine source'%k,
                    isp,np.full(len(isp),float(isp_node.text)),0.)
            if table is not None:
                knots=np.array([[float(x) for x in line.split()] for line in table.findtext('tableData').strip().splitlines()])
                axis=table.get('name')
                if axis=='fsim/commanded-dual-pulse-v1':coordinate=v('fsim/motor-pulse/virtual-age-s')
                elif axis=='fsim/ignition-time-thrust-v1':coordinate=age
                else:
                    if feed_state is None:raise ValueError('Missing independent burned-fuel coordinate')
                    coordinate=feed_state[:,9]
                base=np.interp(coordinate,knots[:,0],knots[:,1])*4.4482216152605
                buildup=float(engine_xml.getroot().findtext('builduptime','0'))
                if buildup>0:base*=np.where(age<=buildup,np.sin(age/buildup*math.pi/2),1.)
                # Explicit hold overrides the positive next-pulse table value.
                if axis=='fsim/commanded-dual-pulse-v1':
                    eligible=float(xml.find('.//property[.="fsim/motor-pulse/second-start-s"]').get('value'))
                    hold=(age>=eligible)&(v('fsim/motor-pulse/second-ignition-age-s')<0)
                    base=np.where(hold,0.,base)
                if feed_state is not None:base=np.where(feed_state[:,13]!=0.,0.,base)
                # Native repeated clock addition is bounded separately from
                # represented source knots; this allowance is propagation of
                # the native absolute clock error through the sine ramp.
                clock_bound=8*eps*n*max(1.,float(np.max(age)))
                slope_bound=float(np.max(knots[:,1]))*4.4482216152605*math.pi/(2*buildup) if buildup else 0.
                add('Actual native engine%d authored table and startup curve versus independent interpolation'%k,
                    feed_state[:,1]*4.4482216152605 if feed_state is not None else vacuum,
                    base,allowance+clock_bound*slope_bound,table_axis=axis,
                    clock_roundoff_bound_s=clock_bound,startup_time_s=buildup)
        mechanism=equipment['components']['propulsion']['mechanism_id']
        if mechanism=='fsim.propulsion.jsbsim_commanded_dual_pulse.v1':
            p=resolved(card,'propulsion');start=p['phase_0_duration_s']+p['phase_1_duration_s']+p['buildup_s']*(1-2/math.pi)
            age_after=v('fsim/motor-pulse/native-age-s');ignition=v('fsim/motor-pulse/second-ignition-age-s')
            waiting=(age_after>=start+dt)&(ignition<0)
            burning=ignition>=0
            if not np.any(waiting) or not np.any(burning):raise ValueError('Powered dual-pulse capture omitted hold or actual second ignition')
            add('Actual native held reserve has zero vacuum force',v('engine0_vacuum_thrust_n')[waiting],np.zeros(np.count_nonzero(waiting)),0.)
            add('Actual native held reserve consumes no propellant',v('engine0_flow_kg_s')[waiting],np.zeros(np.count_nonzero(waiting)),0.)
            expected=np.where(burning,start+age_after-dt-ignition,np.where(age_after-dt>=start,start,age_after-dt))
            add('Actual native commanded pulse virtual clock versus independent hybrid time mapping',
                v('fsim/motor-pulse/virtual-age-s'),expected,8*eps*n*max(1.,float(np.max(age_after))),
                eligible_s=start,manual_bench_command_after_hold_s=.05)
            if np.max(v('engine0_vacuum_thrust_n')[burning])<=0:raise ValueError('Second pulse never delivered positive force')
        if mechanism=='fsim.propulsion.jsbsim_ducted_rocket.v2':
            from hangar import ductedrocket
            config=ductedrocket.inputs(card)
            prefix='fsim/ducted-inlet-v2/';tau=float(v(prefix+'residence-s')[0]);lb=.45359237
            if np.max(v(prefix+'outflow-kg-s'))<=0 or np.max(v(prefix+'engine[1]/burned-fuel-kg-s'))<=0:
                raise ValueError('Actual finite inlet/combustor never carried air and burning fuel')
            fluxes=[('capture-kg-s','outflow-kg-s','resident-air-mass-kg',1.),
                    ('incoming-enthalpy-w','outflow-enthalpy-w','resident-enthalpy-j',1.)]+[
                    (f'incoming-momentum-{a}-n',f'outflow-momentum-{a}-n',f'resident-momentum-{a}-kg-m-s',-1.) for a in 'xyz']
            adjacent=np.diff(v('native_sample'))==1
            a=dt/(2*tau+dt);b=(2*tau-dt)/(2*tau+dt)
            for incoming,outgoing,inventory,sign in fluxes:
                output=v(prefix+outgoing);forcing=v(prefix+incoming);expected=a*(forcing[1:]+forcing[:-1])+b*output[:-1]
                tolerance=128*eps*max(1.,float(np.max(abs(output))))
                add('Actual V2 '+outgoing+' transient versus independent trapezoidal control-volume equation',
                    output[1:][adjacent],expected[adjacent],tolerance,native_dt_s=dt,residence_s=tau)
                add('Actual V2 '+inventory+' versus matched finite residence inventory',v(prefix+inventory),sign*tau*output,
                    128*eps*max(1.,float(np.max(abs(sign*tau*output)))))
            weight_column=next(c for c in columns if c.startswith('inertia/pointmass-weight-lbs['))
            add('Actual native pointmass input versus finite resident inlet air mass',v(weight_column)*lb,v(prefix+'resident-air-mass-kg'),
                128*eps*max(1.,float(np.max(v(prefix+'resident-air-mass-kg')))))
            add('Actual finite native reserved fuel versus current consumed mass flow',
                v(prefix+'engine[1]/reserved-fuel-lbs')*lb/dt,v('engine1_flow_kg_s'),
                128*eps*max(1.,float(np.max(v('engine1_flow_kg_s')))))
            gross=(v(prefix+'outflow-kg-s')+v(prefix+'engine[1]/fuel-kg-s'))*v(prefix+'engine[1]/exit-speed-m-s')
            add('Actual finite ducted native gross thrust versus control-volume exit momentum',v(prefix+'engine[1]/gross-thrust-n'),gross,
                128*eps*max(1.,float(np.max(abs(gross)))))
            air=v(prefix+'outflow-kg-s');fuel=v(prefix+'engine[1]/fuel-kg-s');enthalpy=v(prefix+'outflow-enthalpy-w')
            burned=np.minimum(fuel,air*config['fuel_air_ratio_max'])
            temperature=(enthalpy+burned*config['combustion_efficiency']*config['heating_value_j_kg'])/np.maximum(1e-12,(air+fuel)*config['gas_cp_j_kg_k'])
            ratio=np.maximum(1.,config['pressure_recovery']*(enthalpy/np.maximum(1e-12,
                air*config['air_cp_j_kg_k']*v('force_epoch_temperature_k')))**3.5)
            exit_speed=np.sqrt(np.maximum(0.,2*config['gas_cp_j_kg_k']*temperature*(1-ratio**(-(config['gas_gamma']-1)/config['gas_gamma']))))
            add('Actual finite ducted combustor temperature versus independent residence enthalpy plus actual consumed-fuel heat',
                v(prefix+'engine[1]/total-temperature-k'),temperature,128*eps*max(1.,float(np.max(abs(temperature)))))
            add('Actual finite ducted nozzle exit speed versus independent isentropic equivalent expansion',
                v(prefix+'engine[1]/exit-speed-m-s'),exit_speed,256*eps*max(1.,float(np.max(exit_speed))))
            omega=np.column_stack([v('force_epoch_'+a+'_rad_s') for a in 'pqr'])
            momentum=np.column_stack([v(prefix+'outflow-momentum-'+a+'-n') for a in 'xyz'])
            ram=-momentum+tau*np.cross(omega,momentum)
            ram[:,0]+=np.where(fuel<=1e-12,gross,0.)
            for j,axis in enumerate('xyz'):
                add('Actual finite ducted '+axis+' ram reaction versus independent moving-body control-volume momentum transport',
                    v(prefix+'ram-reaction-'+axis+'-n'),ram[:,j],256*eps*max(1.,float(np.max(abs(ram[:,j])))))
            for axis in 'xyz':
                expected=v('forces/fb'+axis+'-prop-lbs')*4.4482216152605+v(prefix+'ram-reaction-'+axis+'-n')
                add('Actual finite ducted net '+axis+' native propulsion force versus same-epoch native engine and ram reaction',
                    v(prefix+'net-propulsion-force-'+axis+'-n'),expected,128*eps*max(1.,float(np.max(abs(expected)))))
            net=ram.copy();net[:,0]+=sum(v(f'engine{k}_delivered_thrust_n') for k in range(len(engines)))
            add('Actual finite ducted native net body force versus independent nozzle and moving-control-volume balance',
                np.column_stack([v(prefix+'net-propulsion-force-'+a+'-n') for a in 'xyz']),net,
                512*eps*max(1.,float(np.max(abs(net)))))
        row['powered_native_numeric_coupling']=True
    result['powered_native_coupling_passed']=True
    return result


def execute(root=ROOT):
    import numpy as np
    import scipy
    from scipy import integrate, optimize, signal, linalg
    sys.path.insert(0, str(root/'tools/models/hangar'))
    from hangar.aero import vlm
    from hangar.geometry.aircraft import Aircraft
    from hangar import storebodies, components, ductedrocket
    models = roster(root); rows = {family: [] for family in FAMILIES}
    extra = {family: [] for family in FAMILIES}
    # A primary constrained allocation problem checked by two numerical routes.
    B = np.array([[1., .25, -.1], [.1, 1., .4], [.3, -.2, .8]])
    for wanted in ([.2, -.3, .4], [2., -1.5, .8], [-2., 3., -1.]):
        cost, u = bounded_least_squares_reference(B, wanted)
        independent_fit = optimize.lsq_linear(B, wanted, bounds=(-1., 1.), tol=1e-13, method='trf', max_iter=200)
        extra['allocator'].append(residual('27-face objective versus independent SciPy TRF', cost,
                                         np.sum((B@independent_fit.x-wanted)**2), 1e-10, inputs=dict(B=B.tolist(), wanted=wanted)))
    # Actual Hangar finite-segment velocity versus numerical Biot-Savart integral.
    a, b = np.array([-.4, .2, -.1]), np.array([.9, 1.2, .4])
    for point in ([.2, -.8, .7], [2., .3, -.4], [-1., 2., 1.]):
        x = np.array(point); actual = vlm._segments(x[None, :], a[None, :], b[None, :], np.zeros((1, 1)))[0, 0]
        reference = np.array([integrate.quad(lambda t: np.cross(b-a, x-a-t*(b-a))[k]/
            (4*math.pi*np.linalg.norm(x-a-t*(b-a))**3), 0., 1., epsabs=1e-13, epsrel=1e-13)[0] for k in range(3)])
        extra['aerodynamics'].append(residual('Actual Hangar Biot-Savart versus independent quadrature', actual, reference, 1e-12,
                                             inputs=dict(a=a.tolist(), b=b.tolist(), point=point, core_radius=0.)))
    # A disclosed small collocation problem also executes the production kernel
    # in a coupled solve; this is not a regenerated whole-aircraft polar.
    starts=np.array([[-.5,-1.,0.],[-.4,0.,.1],[-.3,1.,-.1]])
    ends=starts+np.array([.8,.25,.05]);points=starts+np.array([.3,-.2,.7])
    normals=np.array([[0.,0.,1.],[.1,0.,1.],[-.1,.1,1.]])
    influence=vlm._segments(points,starts,ends,np.zeros((3,3)))
    A=np.einsum('ijk,ik->ij',influence,normals);independent_A=np.zeros((3,3))
    for i in range(3):
        for j in range(3):
            independent_A[i,j]=integrate.quad(lambda t: np.dot(normals[i],np.cross(ends[j]-starts[j],
                points[i]-starts[j]-t*(ends[j]-starts[j])))/(4*math.pi*np.linalg.norm(points[i]-starts[j]-t*(ends[j]-starts[j]))**3),
                0,1,epsabs=1e-13,epsrel=1e-13)[0]
    rhs=np.array([.1,-.03,.07]);solution=np.linalg.solve(A,rhs)
    extra['aerodynamics'].append(residual('Production VLM finite-vortex collocation solve versus independent quadrature matrix',
        solution,np.linalg.solve(independent_A,rhs),1e-10,inputs=dict(starts=starts.tolist(),ends=ends.tolist(),
            points=points.tolist(),normals=normals.tolist(),rhs=rhs.tolist(),condition_number=float(np.linalg.cond(A)))))
    for equipment, card, cardpath in models:
        native = card['model'].removeprefix('jsbsim:'); designpath = root/'aircraft'/native/(native+'.toml')
        design = tomllib.loads(designpath.read_text(encoding='utf-8')); xml = root/'aircraft/stores'/native/(native+'.xml')
        g, c, servo = resolved(card, 'guidance'), resolved(card, 'autopilot'), resolved(card, 'actuator')
        control = dict(c, **servo)
        v = np.array([550., 30., -20.]); scale_speed = 450. if card['seeker'] == 'active_radar' else float(np.linalg.norm(v))
        pn_input = dict(direction=[1., 0., 0.], rate=[0., .02, -.01], velocity=v.tolist(), up=[0., 0., 1.],
                        navigation_gain=g['navigation_gain'], scale_speed=scale_speed, maximum_g=g['max_g'])
        actual = measured_pn_port(**pn_input)
        reference = pn_geometric_reference(pn_input['direction'], pn_input['rate'], v, pn_input['up'],
            g['navigation_gain'], scale_speed, g['max_g'])
        checks = {'guidance': [residual('PN port versus differentiated bearing reference', actual, reference, 1e-7, inputs=pn_input)],
                  'autopilot': [residual('Cascade algebra versus independent force/moment trim matrix solve',
                    cascade_port(control), cascade_port(control, matrix_reference=True), 1e-12, inputs=control)]}
        if equipment['components']['autopilot']['mechanism_id'] == 'fsim.control.actuator_aware_cascade.v1':
            A, binput, gain, poles = actuator_aware_gains(control)
            checks['autopilot'].append(residual('Actuator-aware characteristic poles versus independent eigensolve',
                np.sort(np.linalg.eigvals(A-binput@gain[None, :])), np.sort(poles), 1e-9,
                inputs=dict(A=A.tolist(), B=binput.tolist(), gain=gain.tolist())))
            checks['autopilot'].append(residual('Actuator-aware pole gains versus independent SciPy Yang-Tits placement',
                gain,signal.place_poles(A,binput,poles).gain_matrix[0],1e-10))
        matrix, limits = native_fin_matrix(xml); channel = np.array([.03, -.04, .02])
        passive = [{k: v for k, v in surface.get('rolleron', {}).items()}
                   for surface in design.get('surface', []) if surface.get('rolleron', {}).get('version') == 1]
        direct = np.array([sum(float(a)*float(b) for a, b in zip(row, channel)) for row in matrix])
        checks['allocator'] = [residual('Actual native XML authored mixer versus matrix linear map', direct, matrix@channel, 1e-15,
                                       inputs=dict(matrix=matrix.tolist(), channel_rad=channel.tolist(), fin_limits_rad=limits))]
        dt = .002; commands = [0.]*4+[.3]*50+[-.25]*50+[0.]*20
        # rate/stops disabled by large finite bounds for the exact linear comparison;
        # a separate finite rate/stops trajectory records active nonlinear limits.
        direct = lag_rate_stops_port(commands, servo['lag_s'], 1e6, dt, -1., 1.)
        num, den, _ = signal.cont2discrete(([1.], [servo['lag_s'], 1.]), dt, method='bilinear')
        reference = signal.lfilter(num[0], den, commands)
        finite = lag_rate_stops_port(commands, servo['lag_s'], servo['rate_rad_s'], dt, -.1, .2)
        checks['actuator'] = [residual('Native-law Tustin port versus independent SciPy bilinear transfer', direct, reference, 1e-13,
             inputs=dict(tau_s=servo['lag_s'], dt_s=dt, commands=commands)),
             residual('Finite rate bound', max(abs(b-a)/dt for a, b in zip([0.]+finite, finite)),
                      min(servo['rate_rad_s'], max(abs(b-a)/dt for a, b in zip([0.]+finite, finite))), 1e-10)]
        selected_design, settings = components.motor(card, design)
        mass = card['parameters']['mass_kg']['value']; radius = card['parameters']['diameter_m']['value']/2
        motor = storebodies.motor_of(selected_design, mass, design['mass']['empty_cg'][0], radius, settings=settings)
        phases = selected_design['motor']['phase']; impulse = math.fsum(max(1000*p['thrust_kn'], settings['coast_thrust_n'])*p['duration_s'] for p in phases)
        reference_mass = impulse/(settings['isp_s']*G0)
        checks['propulsion'] = [residual('Actual motor_of mass ledger versus NASA impulse/Isp relation', motor['propellant_kg'], reference_mass, 1e-11,
            inputs=dict(phases=phases, coast_n=settings['coast_thrust_n'], isp_s=settings['isp_s'], launch_mass_kg=mass))]
        final_mass = mass-reference_mass; exhaust = settings['isp_s']*G0
        numerical_dv = integrate.quad(lambda remaining: exhaust/remaining, final_mass, mass, epsabs=1e-9)[0]
        checks['propulsion'].append(residual('Ideal variable-mass integration versus NASA rocket equation', numerical_dv,
            exhaust*math.log(mass/final_mass), 1e-8, inputs=dict(initial_mass_kg=mass, final_mass_kg=final_mass, equivalent_exit_ms=exhaust)))
        if equipment['components']['propulsion']['mechanism_id']=='fsim.propulsion.jsbsim_commanded_dual_pulse.v1':
            buildup=settings['buildup_s'];duration=phases[0]['duration_s']
            lost=integrate.quad(lambda t: 1-math.sin(math.pi*t/(2*buildup)),0,buildup,epsabs=1e-14)[0]
            checks['propulsion'].append(residual('Commandable two-pulse ignition epoch versus independent buildup-area quadrature',
                duration+phases[1]['duration_s']+buildup*(1-2/math.pi),duration+phases[1]['duration_s']+lost,1e-12,
                inputs=dict(phases=phases,buildup_s=buildup),scope='Policy epoch and finite impulse; native state sequence separately required'))
        if equipment['components']['propulsion']['mechanism_id'] == 'fsim.propulsion.jsbsim_ducted_rocket.v2':
            cycle = ductedrocket.inputs(card); state = dict(density=.7, speed=850., temperature=240., mach=2.8, fuel_flow=.3)
            output = ductedrocket.cycle(cycle, **state)
            momentum = (output['air_kg_s']+state['fuel_flow'])*output['exit_speed_m_s']-output['air_kg_s']*state['speed']
            checks['propulsion'].append(residual('Actual ducted cycle versus NASA net momentum thrust', output['net_thrust_n'], momentum, 1e-10,
                inputs=dict(configuration=cycle, state=state), scope='Fully expanded equivalent nozzle; no OEM Meteor cycle validation'))
            equilibrium=ductedrocket.inlet_equilibrium_v2(cycle,density=.7,velocity=[850.,20.,-5.],temperature=240.,mach=2.8)
            capture=.7*cycle['capture_area_m2']*850.;tau=cycle['inlet_residence_s']
            independent=[tau*capture,tau*capture*(cycle['air_cp_j_kg_k']*240.+.5*(850**2+20**2+5**2)),
                         *[-tau*capture*v for v in [850.,20.,-5.]]]
            checks['propulsion'].append(residual('Actual V2 finite-inlet inventory versus independent residence control-volume balance',
                [equilibrium['air_mass_kg'],equilibrium['enthalpy_j'],*equilibrium['momentum_body_kg_m_s']],independent,1e-8,
                inputs=dict(density=.7,velocity=[850.,20.,-5.],temperature=240.,mach=2.8,residence_s=tau)))
            times=np.arange(101)*min(.001,tau/10);dt_in=float(times[1]);forcing=[0.]+[capture]*100
            discrete=lag_rate_stops_port(forcing,tau,1e12,dt_in,-1e12,1e12)
            state_reference=signal.dlsim(signal.cont2discrete(([1.],[tau,1.]),dt_in,method='bilinear'),forcing)[1][:,0]
            checks['propulsion'].append(residual('Finite-inlet transient Tustin inventory flux versus independent state-space discretization',
                discrete,state_reference,1e-10,inputs=dict(tau_s=tau,dt_s=dt_in,capture_kg_s=capture)))
        # Individually parameter-sensitive surface geometry, not24 copies of
        # an invented common aerodynamic coefficient table.
        surface=design['surface'][0];sections=surface['sections']
        a=np.array(sections[0]['le'],float)+[.25*sections[0]['chord'],0,0]
        b=np.array(sections[-1]['le'],float)+[.25*sections[-1]['chord'],0,0]
        chord=.5*(sections[0]['chord']+sections[-1]['chord'])
        point=.5*(a+b)+np.array([.5*chord,-.3*chord,.7*chord])
        actual_v=vlm._segments(point[None,:],a[None,:],b[None,:],np.zeros((1,1)))[0,0]
        independent_v=np.array([integrate.quad(lambda t: np.cross(b-a,point-a-t*(b-a))[k]/
            (4*math.pi*np.linalg.norm(point-a-t*(b-a))**3),0.,1.,epsabs=1e-12,epsrel=1e-12)[0] for k in range(3)])
        checks['aerodynamics'] = list(extra['aerodynamics'])+[residual('Actual per-model quarter-chord geometry VLM kernel versus independent Biot-Savart integral',
            actual_v,independent_v,1e-10,inputs=dict(surface=surface['name'],a=a.tolist(),b=b.tolist(),
                point=point.tolist(),source_design_sha256=digest(designpath),scope='Off-filament zero-core primitive, not a whole missile polar'))]
        aircraft=Aircraft(design,str(designpath));lattice=vlm.Lattice(aircraft,density=.05)
        production_vlm=vlm.VLM(lattice,mach=0.,wake_deg=[0.,1.])
        influence=production_vlm._velocity(production_vlm._cp,production_vlm._same_cp,0.)
        normal_matrix=np.einsum('mnk,mk->mn',influence,lattice.normal)
        independent_solve=linalg.lstsq(normal_matrix,-lattice.normal,lapack_driver='gelsy')[0]
        condition=float(np.linalg.cond(normal_matrix))
        allowance=128*np.finfo(float).eps*condition*max(1.,float(np.max(abs(independent_solve))))
        checks['aerodynamics'].append(residual('Actual per-model production VLM solve versus independent pivoted-QR boundary-condition solve',
            production_vlm._sets[0]['g_v'],independent_solve,allowance,
            inputs=dict(design_sha256=digest(designpath),mach=0.,wake_angle_deg=0.,panels=lattice.n_panels,
                lattice_density=.05,condition_number=condition,allowance_basis='128*float64_eps*cond(A)*max(1,abs(Gamma))'),
            scope='Actual source geometry and actual production coupled VLM; disclosed minimum lattice, not production-table resolution/convergence/calibration'))
        for family in FAMILIES:
            if family == 'allocator':
                checks[family] += extra['allocator']
            descriptor = card['component_models']['components'][family]
            rows[family].append(dict(model=card['id'], component=family, implementation_id=descriptor['implementation_id'],
                mechanism_id=equipment['components'][family]['mechanism_id'], card_sha256=digest(cardpath),
                parameter_sha256=parameter_hash(card, family), parameters=parameter_payload(card, family)['parameters'],
                bindings=descriptor.get('bindings', {}), passed=True,
                passive_rolleron_profiles=passive,
                method=dict(case='source-bound independent numerical references', tests=len(checks[family]),
                            assertions=len(checks[family]), residuals=checks[family]),
                actual_artifacts=source_rows([cardpath, designpath, xml]+[
                    root/'aircraft/stores'/native/'Engines'/(node.get('file')+'.xml')
                    for node in ET.parse(xml).findall('./propulsion/engine')+ET.parse(xml).findall('./propulsion/engine/thruster')]),
                production_execution_scope=('Actual imported Hangar '+('VLM primitive' if family == 'aerodynamics' else 'motor/cycle generation' if family == 'propulsion' else 'XML mixer data')
                    if family in ('allocator', 'aerodynamics', 'propulsion') else 'Source-bound equation port; native dump independently required')))
    references={}
    for name,module in [('numpy',np),('scipy',scipy),('scipy.integrate',integrate),
                        ('scipy.optimize',optimize),('scipy.signal',signal),('actual_hangar_vlm',vlm),
                        ('actual_hangar_motor',storebodies),('actual_hangar_ducted',ductedrocket)]:
        path=Path(module.__file__)
        references[name]={'path':str(path),'sha256':digest(path),'version':getattr(module,'__version__',None)}
    # Bind the actual open-source algorithms and native numerical kernels,
    # not just package __init__.py files or metadata version strings.
    for name in ('scipy.integrate._quadpack_py','scipy.integrate._quadpack',
                 'scipy.optimize._lsq.lsq_linear','scipy.optimize._lsq.trf_linear',
                 'scipy.signal._ltisys','scipy.signal._lti_conversion','scipy.signal._signaltools','scipy.signal._sigtools',
                 'scipy.linalg._decomp_qr','scipy.linalg._flapack','numpy._core._multiarray_umath','numpy.linalg._umath_linalg'):
        module=importlib.import_module(name);path=Path(module.__file__)
        references[name]={'path':str(path),'sha256':digest(path)}
    return dict(schema='fsim.sim-m03.kinematics-reference-run.v1', model_count=24, families=rows,
        reference_execution={'python':sys.version,'executable':sys.executable,'executable_sha256':digest(sys.executable),'modules':references},
        family_methods_passed=True, native_coupling_passed=False,
        scope='Executed numerical method checks; not native/full-chain/model/installed/visual/calibration admission')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--cpus', default='4,6', help='Explicit bounded OS CPU identities for this method run')
    parser.add_argument('--freeze-epoch',type=Path,help='Freeze current exact inputs without executing numerical cases')
    parser.add_argument('--native-epoch',type=Path,help='Actual pre-execution input epoch')
    parser.add_argument('--run-receipt',type=Path,help='Root-owned actual Catch command/output/runtime receipt')
    parser.add_argument('--native', type=Path, help='Actual root-executed component-native.v1 dump')
    parser.add_argument('--propulsion-native', type=Path, help='Actual root-executed powered engine native dump')
    args = parser.parse_args()
    for path in (args.output,args.native,args.propulsion_native,args.freeze_epoch,args.native_epoch,args.run_receipt):
        if path is not None:guard_path(path)
    if args.freeze_epoch:
        if any((args.output,args.native,args.propulsion_native,args.native_epoch,args.run_receipt)):
            raise ValueError('Epoch freezing is a separate source-only command')
        args.freeze_epoch.parent.mkdir(parents=True,exist_ok=True)
        args.freeze_epoch.write_text(json.dumps(source_epoch(),indent=2,allow_nan=False)+'\n',encoding='utf-8')
        print(json.dumps(dict(epoch=str(args.freeze_epoch),sha256=digest(args.freeze_epoch))))
        return
    if not args.output:raise ValueError('An explicit output path is required')
    run=None
    if any((args.native,args.propulsion_native,args.native_epoch,args.run_receipt)):
        if not all((args.native,args.native_epoch,args.run_receipt)):
            raise ValueError('Native capture requires matching actual run receipt and pre-execution epoch')
        native_paths=[args.native]+([args.propulsion_native] if args.propulsion_native else [])
        run=validate_capture_run(native_paths,args.native_epoch,args.run_receipt)
    sys.path.insert(0, str(ROOT/'tools/models/hangar'))
    from hangar import resources
    cpus = tuple(int(value) for value in args.cpus.split(','))
    if not 1 <= len(cpus) <= 2:
        raise ValueError('At most two explicitly reserved CPUs')
    with resources.cpu_lease(len(cpus), cpus=cpus):
        result = execute()
        if args.native:
            result=native_comparisons(result,read_reference_json(args.native))
            result['native_raw']={'path':str(args.native),'sha256':digest(args.native)}
        if args.propulsion_native:
            result=powered_comparisons(result,read_reference_json(args.propulsion_native))
            result['powered_native_raw']={'path':str(args.propulsion_native),'sha256':digest(args.propulsion_native)}
    if run:
        result['native_run_binding']=run
        result['native_run_raw']={'path':str(args.run_receipt),'sha256':digest(args.run_receipt)}
        result['native_epoch_raw']={'path':str(args.native_epoch),'sha256':digest(args.native_epoch)}
    result['execution_cpus'] = list(cpus)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True, ensure_ascii=False, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps(dict(passed=True, families=6, model_projections=144, native_coupling_passed=result['native_coupling_passed'],
                         output=str(args.output), sha256=digest(args.output))))


if __name__ == '__main__':
    main()
