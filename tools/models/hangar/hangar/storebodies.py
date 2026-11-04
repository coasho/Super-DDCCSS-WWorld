"""Physical mass, contact and propulsion shells. Aerodynamics use the shared aircraft VLM writer."""
import math
import os
from . import storeaero
from .motorinputs import validate_motor
from .storemodels import FT, IN, LB, RING, SLUGFT2, _value, model_name
G0 = 9.80665
LBF = 4.4482216152605
ISP = {'aluminised': 255.0}
ISP_PLAIN = 245.0
COAST_N = 2.0
BUILDUP = 0.05
_IGNITION_TIME_THRUST = 'fsim/ignition-time-thrust-v1'
GRAIN_DENSITY = 1750.0
GRAIN_RADIUS = 0.9
GRAIN_PORT = 0.15
GRAIN_AFT = 0.03
CLASSES = ('aam', 'bomb', 'cruise_missile')

def point_contact_nose(design):
    """An authored point tip in design structural metres, or unavailable.

    A finite-width/height tip or ambiguous/mirrored foremost body does not define
    this point sensor. Do not infer a tip from card length or launch CG.
    """

    def number(value):
        return isinstance(value, (int, float)) and (not isinstance(value, bool)) and math.isfinite(value)
    candidates = []
    for body in design.get('body', []):
        stations = body.get('stations', [])
        origin = body.get('origin', [0, 0, 0])
        if not stations or not isinstance(origin, (list, tuple)) or len(origin) != 3:
            return None
        first = stations[0]
        values = [first.get(key) for key in ('x', 'w', 'top', 'bottom')] + [first.get('y', 0)] + list(origin)
        if not all((number(value) for value in values)):
            return None
        tip = (first['x'] + origin[0], first.get('y', 0) + origin[1], first['top'] + origin[2])
        if not all((number(value) for value in tip)):
            return None
        candidates.append((tip, first, body.get('mirror', False)))
    if not candidates:
        return None
    foremost = min((row[0][0] for row in candidates))
    candidates = [row for row in candidates if row[0][0] == foremost]
    if len(candidates) != 1:
        return None
    tip, first, mirrored = candidates[0]
    return tip if not mirrored and first['w'] == 0 and (first['top'] == first['bottom']) else None

def _ignition_time_thrust_rows(phases, *, coast_n=COAST_N, buildup_s=BUILDUP):
    """A validated pulsed motor's vacuum thrust (lbf) against native ignition time (s).

    A consumed-mass axis can skip a low-thrust coast in one integration step. The
    versioned native time table advances through it regardless of its tiny mass.
    Preserve the existing 2 N coast and its grain/impulse. The first native sine
    ramp delays every subsequent boundary by its lost effective burn time.
    """
    if phases[0]['duration_s'] < buildup_s:
        raise ValueError('motor: ignition-time table requires the first phase to cover buildup')
    forces = [max(1000.0 * p['thrust_kn'], coast_n) / LBF for p in phases]
    deficit = buildup_s * (1.0 - 2.0 / math.pi)
    rows, elapsed = ([(0.0, forces[0])], 0.0)
    for index, phase in enumerate(phases):
        elapsed += phase['duration_s']
        boundary = elapsed + deficit
        before = math.nextafter(boundary, -math.inf)
        if not math.isfinite(boundary) or before <= rows[-1][0]:
            raise ValueError('motor: ignition-time table knots collapse at double precision')
        rows.append((before, forces[index]))
        rows.append((boundary, forces[index + 1] if index + 1 < len(forces) else 0.0))
    return rows

def _inertia(design, m, length):
    """(ixx, iyy) kg m^2 of the whole store about its CG: a missile's radii of gyration are over its length
    (missiles.py); a store's design is weighed by the mass model (storedesigns.py)."""
    if design['aircraft'].get('category') == 'missile':
        g = design['mass']['gyration']
        return (m * (g[0] * length) ** 2, m * (g[1] * length) ** 2)
    from . import pipeline
    from .geometry.aircraft import Aircraft
    from .mass import MassModel
    name = design['aircraft']['name']
    e = MassModel(Aircraft.load(os.path.join(pipeline.repo_root(), 'aircraft', name, name + '.toml'))).empty()
    return (e['ixx'], e['iyy'])

def motor_of(design, m, cg, radius, *, settings=None):
    """Its motor as its flight model has it, from its design's phases: {propellant_kg, burn_s, rows [(lb burnt, lbf)],
    grain: radius, bore, length, x (m), exit_x, exit_area}; None without one."""
    if 'motor' not in design:
        return None
    motor = design['motor']
    validate_motor(motor)
    from .motorinputs import _number

    def quantity(value, label, positive=False):
        value = _number(value, label)
        if positive and value <= 0.0:
            raise ValueError('motor: %s must be positive' % label)
        return value
    m = quantity(m, 'launch mass', True)
    cg = quantity(cg, 'launch CG')
    radius = quantity(radius, 'body radius', True)
    phases = (motor or {}).get('phase', [])
    if not phases:
        return None
    settings = settings or {}
    isp = settings.get('isp_s', ISP.get(motor.get('smoke'), ISP_PLAIN))
    coast = settings.get('coast_thrust_n', COAST_N)
    density = settings.get('grain_density_kg_m3', GRAIN_DENSITY)
    grain_radius = settings.get('grain_radius_fraction', GRAIN_RADIUS)
    grain_port = settings.get('grain_port_area_fraction', GRAIN_PORT)
    grain_aft = settings.get('grain_aft_clearance_m', GRAIN_AFT)
    rows, burnt, impulse, burn_s = ([], 0.0, 0.0, 0.0)
    for i, p in enumerate(phases):
        newtons = quantity(max(1000.0 * p['thrust_kn'], coast), 'simulated phase thrust', True)
        phase_impulse = quantity(newtons * p['duration_s'], 'simulated phase impulse', True)
        impulse = quantity(impulse + phase_impulse, 'total simulated impulse', True)
        burn_s = quantity(burn_s + p['duration_s'], 'total simulated burn duration', True)
        rows.append((quantity((burnt + (1e-06 if i else 0.0)) / LB, 'thrust-table consumed mass'), quantity(newtons / LBF, 'thrust-table force', True)))
        burnt = quantity(burnt + phase_impulse / (isp * G0), 'propellant mass', True)
        rows.append((quantity(burnt / LB, 'thrust-table consumed mass', True), newtons / LBF))
    previous = emitted_previous = -math.inf
    for consumed, force in rows:
        emitted_consumed = float('%.7f' % consumed)
        emitted_force = float('%.3f' % force)
        if consumed <= previous or emitted_consumed <= emitted_previous:
            raise ValueError('motor: consumed-mass thrust table must increase strictly at the emitted precision')
        if not math.isfinite(emitted_consumed) or not math.isfinite(emitted_force) or emitted_force <= 0.0:
            raise ValueError('motor: thrust table must remain finite with positive force at the emitted precision')
        previous, emitted_previous = (consumed, emitted_consumed)
    if burnt >= m:
        raise ValueError('motor: propellant mass must be less than launch mass')
    empty = quantity(m - burnt, 'remaining structure mass', True)
    R = quantity(grain_radius * radius, 'grain radius', True)
    volume_per_length = quantity(density * math.pi * R * R * (1.0 - grain_port), 'grain mass per length', True)
    length = quantity(burnt / volume_per_length, 'grain length', True)
    bore = quantity(2.0 * R * math.sqrt(grain_port), 'grain bore', True)
    quantity(length * length, 'grain squared length', True)
    x = quantity(motor['exit'][0] - motor.get('cone_length', 0.0) - grain_aft - 0.5 * length, 'grain centre')
    try:
        exit_area = quantity(0.25 * math.pi * motor['exit_diameter'] ** 2, 'nozzle exit area', True)
    except OverflowError as error:
        raise ValueError('motor: nozzle exit area is not finite') from error
    empty_cg = quantity((quantity(m * cg, 'launch mass moment') - quantity(burnt * x, 'grain mass moment')) / empty, 'remaining structure CG')
    if design.get('aircraft', {}).get('category') == 'missile':
        try:
            nose = min((quantity(body['stations'][0]['x'], 'body nose') for body in design['body']))
            tail = max((quantity(body['stations'][-1]['x'], 'body tail') for body in design['body']))
            body_length = quantity(tail - nose, 'body length', True)
            gyration = design['mass']['gyration']
            if len(gyration) != 3:
                raise ValueError('motor: gyration must have three positive coordinates')
            for value in gyration:
                quantity(value, 'gyration', True)
            if x - 0.5 * length < nose or x + 0.5 * length > tail:
                raise ValueError('motor: derived grain does not fit inside the body')
            ixx, iyy = _inertia(design, m, body_length)
            quantity(ixx, 'launch roll inertia', True)
            quantity(iyy, 'launch pitch inertia', True)
            rb = 0.5 * bore
            quantity(ixx - 0.5 * burnt * (R * R + rb * rb), 'remaining structure roll inertia', True)
            quantity(iyy - burnt * (3.0 * (R * R + rb * rb) + length * length) / 12.0 - burnt * (x - cg) ** 2 - empty * (empty_cg - cg) ** 2, 'remaining structure pitch inertia', True)
        except (KeyError, IndexError, TypeError, OverflowError, ValueError) as error:
            if isinstance(error, ValueError) and str(error).startswith('motor:'):
                raise
            raise ValueError('motor: invalid body geometry or structure inertia') from error
    return dict(isp=isp, propellant_kg=burnt, burn_s=burn_s, rows=rows, radius=R, bore=bore, length=length, x=x, exit_x=motor['exit'][0], exit_area=exit_area, phases=phases, grain_density_kg_m3=density)

def card_motor_of(card, design, m, cg, radius):
    from . import components
    selected, settings = components.motor(card, design)
    return motor_of(selected, m, cg, radius, settings=settings)

def _finite_conversion(value, label):
    if not math.isfinite(value):
        raise ValueError('%s must remain finite after unit conversion' % label)
    return value

def files(card):
    """The store's flight model as its files, {path under aircraft/stores/<model>/: text} - its definition and, with a
    motor, its engine's and its nozzle's (None: not a missile or a bomb, or it has no design)."""
    name = model_name(card)
    if card.get('class') not in CLASSES or not name:
        return None
    geo = storeaero.geometry(card)
    if geo is None or geo.get('design') is None:
        return None
    design = geo['design']
    from . import components
    from .geometry import Aircraft
    aircraft = Aircraft.load(geo['design_path'])
    b = dict(S=aircraft.S, d=aircraft.c)
    precise = True
    S, d = (b['S'], b['d'])
    area_ft2, diameter_ft = (S / FT ** 2, d / FT)
    r = 0.5 * geo['diameter'] # physical body radius is independent of reference chord
    m = _value(card, 'mass_kg')
    cg = float(design['mass']['empty_cg'][0])
    x0 = cg - geo['cg']
    nose = min((body['stations'][0]['x'] for body in design['body']))
    tail = max((body['stations'][-1]['x'] for body in design['body']))
    ixx, iyy = _inertia(design, m, tail - nose)
    design, motor_settings = components.motor(card, design)
    motor = motor_of(design, m, cg, r, settings=motor_settings)
    empty, empty_cg = (m, cg)
    if motor:
        mp, R, rb, lg, xg = (motor['propellant_kg'], motor['radius'], 0.5 * motor['bore'], motor['length'], motor['x'])
        empty = m - mp
        empty_cg = (m * cg - mp * xg) / empty
        ixx -= 0.5 * mp * (R * R + rb * rb)
        iyy -= mp * (3.0 * (R * R + rb * rb) + lg * lg) / 12.0 + mp * (xg - cg) ** 2 + empty * (empty_cg - cg) ** 2
    k = 1.0 / IN

    def loc(what, x):
        if precise:
            return '        <location name="%s" unit="IN"> <x> %.17g </x> <y> 0 </y> <z> 0 </z> </location>' % (what, _finite_conversion(x * k, what))
        return '        <location name="%s" unit="IN"> <x> %.3f </x> <y> 0 </y> <z> 0 </z> </location>' % (what, x * k)
    spring, damping = (20.0 * m / LB, 2.0 * m / LB)

    def contacts(what, x):
        out = []
        for i in range(RING):
            a = 2.0 * math.pi * (i + 0.5) / RING
            out.append('        <contact type="STRUCTURE" name="%s %d">\n            <location unit="IN"> <x> %.3f </x> <y> %.3f </y> <z> %.3f </z> </location>\n            <static_friction> 0.8 </static_friction>\n            <dynamic_friction> 0.5 </dynamic_friction>\n            <spring_coeff unit="LBS/FT"> %.0f </spring_coeff>\n            <damping_coeff unit="LBS/FT/SEC"> %.0f </damping_coeff>\n        </contact>' % (what, i + 1, x * k, r * math.cos(a) * k, r * math.sin(a) * k, spring / RING, damping / RING))
        return '\n'.join(out)
    if motor:
        propulsion = '    <propulsion>\n        <engine file="%s_motor">\n            <feed> 0 </feed>\n            <thruster file="%s_nozzle">\n                <location unit="IN"> <x> %.3f </x> <y> 0 </y> <z> 0 </z> </location>\n                <orient unit="DEG"> <roll> 0 </roll> <pitch> 0 </pitch> <yaw> 0 </yaw> </orient>\n            </thruster>\n        </engine>\n        <tank type="FUEL">\n            <location unit="IN"> <x> %.3f </x> <y> 0 </y> <z> 0 </z> </location>\n            <radius unit="IN"> %.4f </radius>\n            <grain_config type="CYLINDRICAL">\n                <length unit="IN"> %.4f </length>\n                <bore_diameter unit="IN"> %.4f </bore_diameter>\n            </grain_config>\n            <capacity unit="LBS"> %.5f </capacity>\n            <contents unit="LBS"> %.5f </contents>\n        </tank>\n    </propulsion>' % (name, name, motor['exit_x'] * k, motor['x'] * k, motor['radius'] * k, motor['length'] * k, motor['bore'] * k, motor['propellant_kg'] / LB, motor['propellant_kg'] / LB)
        controls = '    <flight_control name="none">\n        <!-- no controls. Its model\'s origin in this frame (in): its CG as it is launched, its motor full - its picture\'s\n             origin and what a station holds; the CG above is its structure\'s, burnt out -->\n        <property value="%.3f">fsim/model/origin-x-in</property>\n        <property value="0">fsim/model/origin-y-in</property>\n        <property value="0">fsim/model/origin-z-in</property>\n    </flight_control>' % (cg * k)
    else:
        propulsion = '    <propulsion/>'
        controls = '    <flight_control name="none"/>'
    xml = '<?xml version="1.0"?>\n<!-- Written by hangar (tools/hangar/hangar/storemodels.py) from the store card %s and its design: the store flying on its\n     own once it is released or jettisoned (MS-7e1, MS-7g). Its aerodynamics use the shared aircraft VLM tables and writer. Do not edit. -->\n<fdm_config name="%s" version="2.0" release="ALPHA">\n    <fileheader>\n        <author> hangar </author>\n        <description> %s, released </description>\n    </fileheader>\n    <metrics>\n        <wingarea unit="FT2"> %s </wingarea>\n        <wingspan unit="FT"> %s </wingspan>\n        <chord unit="FT"> %s </chord>\n        <htailarea unit="FT2"> 0 </htailarea>\n        <htailarm unit="FT"> 0 </htailarm>\n        <vtailarea unit="FT2"> 0 </vtailarea>\n        <vtailarm unit="FT"> 0 </vtailarm>\n%s\n%s\n%s\n    </metrics>\n    <mass_balance negated_crossproduct_inertia="false">\n        <ixx unit="SLUG*FT2"> %.5f </ixx>\n        <iyy unit="SLUG*FT2"> %.5f </iyy>\n        <izz unit="SLUG*FT2"> %.5f </izz>\n        <emptywt unit="LBS"> %.4f </emptywt>\n%s\n    </mass_balance>\n    <ground_reactions>\n%s\n%s\n    </ground_reactions>\n%s\n%s\n\n</fdm_config>\n' % (card['id'], name, card.get('designation', card['id']), ('%.17g' if precise else '%.5f') % area_ft2, ('%.17g' if precise else '%.5f') % diameter_ft, ('%.17g' if precise else '%.5f') % diameter_ft, loc('AERORP', cg), loc('EYEPOINT', cg), loc('VRP', cg), ixx / SLUGFT2, iyy / SLUGFT2, iyy / SLUGFT2, empty / LB, loc('CG', empty_cg), contacts('nose', x0 + 0.1 * geo['length']), contacts('tail', x0 + 0.9 * geo['length']), propulsion, controls)
    from . import storecontrol
    tip = point_contact_nose(design)
    metadata = '\n        <property value="%d">fsim/model/nose-version</property>' % (1 if tip else 0)
    if tip is not None:
        for axis, coordinate in zip('xyz', tip):
            metadata += '\n        <property value="%.17g">fsim/model/nose-%s-in</property>' % (_finite_conversion(coordinate * k, 'contact nose'), axis)
    xml = xml.replace('</flight_control>', metadata + '\n    </flight_control>', 1)
    pulse=(motor_settings or {}).get('pulse_control_v1')
    if pulse:
        phases=design['motor']['phase'];buildup=motor_settings['buildup_s']
        first=phases[0]['duration_s']+buildup*(1-2/math.pi)
        second=first+phases[1]['duration_s']
        values={'version':1,'first-end-s':first,'second-start-s':second,
            'second-end-s':second+phases[2]['duration_s'],'hold-limit-s':pulse['pulse_hold_limit_s'],
            'reserve-speed-m_s':pulse['pulse_reserve_speed_m_s'],'terminal-lead-s':pulse['pulse_terminal_lead_s'],
            'second-command':0,'second-ignition-age-s':-1,'native-age-s':0,'virtual-age-s':0}
        metadata=''.join('\n        <property value="%.17g">fsim/motor-pulse/%s</property>'%(value,key) for key,value in values.items())
        xml=xml.replace('</flight_control>',metadata+'\n    </flight_control>',1)
    if motor and motor_settings is not None and card.get('class')=='aam':
        # Explicit current AAM motor profiles opt into supply-backed native
        # force. Legacy/absent profiles keep their original native execution.
        # Meteor's booster is engine0; its sustainer already owns V2 batches.
        xml=xml.replace('</flight_control>',
            '\n        <property value="1">fsim/rocket-feed-v1/engine[0]/version</property>\n    </flight_control>',1)
    out = {name + '.xml': xml}
    if motor:
        table_name = 'propulsion/thrust_prop_remain'
        table_text = '\n'.join(('            %.7f %.3f' % row for row in motor['rows']))
        if any((phase['thrust_kn'] == 0.0 for phase in design['motor']['phase'])):
            table_name = _IGNITION_TIME_THRUST
            table_text = '\n'.join(('            %.17g %.17g' % row for row in _ignition_time_thrust_rows(design['motor']['phase'], coast_n=(motor_settings or {}).get('coast_thrust_n', COAST_N), buildup_s=(motor_settings or {}).get('buildup_s', BUILDUP))))
        if pulse:table_name='fsim/commanded-dual-pulse-v1'
        out['Engines/%s_motor.xml' % name] = '<?xml version="1.0"?>\n<!-- Written by hangar (storemodels.py) from the design of %s: its motor\'s phases there (assumed: none is published),\n     %.1f kg of propellant at %.0f s (assumed), burning %.1f s. Do not edit. -->\n<rocket_engine name="%s motor">\n    <isp> %.1f </isp>\n    <builduptime> %.2f </builduptime>\n    <thrust_table name="%s" type="internal">\n        <tableData>\n%s\n        </tableData>\n    </thrust_table>\n</rocket_engine>\n' % (name, motor['propellant_kg'], motor['isp'], motor['burn_s'], name, motor['isp'], (motor_settings or {}).get('buildup_s', BUILDUP), table_name, table_text)
        if motor_settings:
            for tag, value, precision in (('isp', motor['isp'], 1), ('builduptime', motor_settings['buildup_s'], 2)):
                if float('%.*f' % (precision, value)) != value:
                    import re
                    key = 'Engines/%s_motor.xml' % name
                    out[key] = re.sub('(<%s>)[^<]*(</%s>)' % (tag, tag), lambda match: match[1] + ' %.17g ' % value + match[2], out[key], count=1)
        out['Engines/%s_nozzle.xml' % name] = '<?xml version="1.0"?>\n<!-- Written by hangar (storemodels.py): the nozzle\'s exit of %s\'s design. Do not edit. -->\n<nozzle name="%s nozzle">\n    <area unit="FT2"> %.6f </area>\n</nozzle>\n' % (name, name, motor['exit_area'] / FT ** 2)
    from . import airframe
    from . import turbofanstore
    out=turbofanstore.install(out,card,aircraft)
    out[name + '.xml'] = airframe.install(out[name + '.xml'], geo['design_path'], card=card)
    out[name + '.xml'] = turbofanstore.install_control(out[name + '.xml'],card,design)
    from . import ductedrocket
    out=ductedrocket.install(out,card,design,motor)
    return out
