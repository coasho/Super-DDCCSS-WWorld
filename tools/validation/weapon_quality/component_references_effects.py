"""Independent primary-equation references for current seeker/fuze/warhead methods.

No simulation or provider is implemented here. Inputs are observations emitted
by the unchanged, selected SDK providers. A passed artifact requires a passing
native report, current complete input epoch, and independently recomputed
numeric residuals. Equation verification does not establish OEM calibration.
"""
import argparse
from decimal import Decimal, localcontext
from fractions import Fraction
import hashlib
import json
import math
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET
from acceptance import allowed

ROOT = Path(__file__).resolve().parents[3]
FAMILIES = ('seeker', 'fuze', 'warhead')
BASE = Path('docs/research/sim-m03/component-methods')
REFERENCES = BASE/'references'
SELF = 'tools/validation/weapon_quality/component_references_effects.py'
FIXTURE = 'tests/simulation/test_component_effect_reference_native.cpp'
# Numerical verification budgets declared before the native observations.
# They describe implementation arithmetic, never equipment uncertainty.
TOLERANCES = dict(planck_relative=1e-6, charge_relative=1e-9,
                  radar_relative=2e-12, gurney_relative=2e-12,
                  geometry_relative=128*sys.float_info.epsilon,
                  energy_relative=1e-10)
PRIMARY = {
    'planck': dict(title='NIST SP250-43, Radiance Temperature Calibrations (1998)',
        url='https://nvlpubs.nist.gov/nistpubs/Legacy/SP/nistspecialpublication250-43.pdf',
        locator='Section 2, equation (4), printed page 6 (PDF page16)',
        scope='Vacuum spectral radiance law; integrated independently with the exact-SI constants, not the historical rounded constants.'),
    'si': dict(title='BIPM SI Brochure, ninth edition, v4.01',
        url='https://www.bipm.org/documents/20126/41483022/SI-Brochure-9.pdf',
        locator='Section2.2, defining constants h,c,k, English tables2 and4',
        scope='Exact SI h=6.62607015e-34 J s, c=299792458 m/s, k=1.380649e-23 J/K; no fitted radiometric coefficient.'),
    'radar': dict(title="MIT Lincoln Laboratory, O'Donnell, Introduction to Radar Systems: The Radar Equation (2002)",
        url='https://www.ll.mit.edu/media/6946', locator='Slides5-10,13: power-density, effective area, kTB and track radar equation',
        scope='Free-space received power and thermal noise. Energy integration and bistatic Rt/Rr separation are explicitly derived here, not OEM receiver data.'),
    'cpa': dict(title='David Eberly, Geometric Tools, DistPointSegment.h, file version8.0.2025.05.10',
        url='https://raw.githubusercontent.com/davideberly/GeometricTools/master/GTE/Mathematics/DistPointSegment.h',
        locator='DCPQuery operator(), lines42-85; non-unit segment projection and clamping',
        scope='Independent exact-rational relative-motion segment projection. Arming, actual-turn and lifetime eligibility are additional explicit native contracts, not claims made by this geometry reference.'),
    'gurney': dict(title='R.W.Gurney, BRL Report405, The Initial Velocities of Fragments from Bombs, Shell, Grenades (1943)',
        url='https://www.pdf-archive.com/2019/11/16/untitled-pdf-document/untitled-pdf-document.pdf',
        locator='Original report, equations(2.1),(2.3),(2.4), pp4-5; public-release original reproduced by archive',
        scope='Uniform cylindrical case and linear radial gas velocity; energy per explosive mass. Finite zones and connected rods are separately identified engineering source extensions, not measured validation from Gurney.'),
}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=False,
                      allow_nan=False).encode('utf-8')


def sha(path):
    result = hashlib.sha256()
    with allowed(path).open('rb') as stream:
        for block in iter(lambda: stream.read(65536), b''):
            result.update(block)
    return result.hexdigest()


def read(path):
    def unique(pairs):
        out = {}
        for k, v in pairs:
            require(k not in out, 'Duplicate JSON field '+k)
            out[k] = v
        return out
    return json.loads(allowed(path).read_text(encoding='utf-8'), object_pairs_hook=unique,
        parse_constant=lambda _: (_ for _ in ()).throw(ValueError('Nonfinite JSON')))


def bind(path, root=ROOT):
    root = allowed(root)
    path = allowed(allowed(path).resolve())
    require(path.is_relative_to(root.resolve()) or path.is_relative_to(Path('D:/ENV').resolve()),
            'Evidence is outside the project and DENV roots')
    return dict(path=path.relative_to(root.resolve()).as_posix() if path.is_relative_to(root.resolve())
                else path.as_posix(), sha256=sha(path))


def inventory(root=ROOT):
    root = allowed(root)
    # Reuse the independently maintained matrix's exact descriptor+FULL record
    # hash and current native-input inventory; do not invent another identity.
    import component_matrix as matrix
    stores = []
    for path in sorted((root/'assets/stores').glob('*.json')):
        card = read(path)
        if card.get('class') == 'aam':
            stores.append(dict(card=card['id'], **{'class': 'aam'},
                card_path=path.relative_to(root).as_posix(), card_sha256=sha(path),
                model=card['model'].removeprefix('jsbsim:')))
    return [c for c in matrix.derive_cells(dict(stores=stores), root) if c['component'] in FAMILIES]


def source_epoch(root=ROOT):
    root = allowed(root)
    cells = inventory(root)
    paths = {r['path'] for cell in cells for r in cell['sources']}
    paths |= {SELF, FIXTURE, 'tests/CMakeLists.txt',
        'src/foundation/transport/third_party/picosha2/picosha2.h',
        'src/foundation/transport/third_party/picosha2/LICENSE',
        'src/foundation/transport/third_party/picosha2/provenance.json',
        'tools/validation/weapon_quality/component_reference_run.py',
        'tools/validation/weapon_quality/component_matrix.py',
        'tools/validation/weapon_quality/acceptance.py',
        'src/simulation/mission_systems/weapon/Composition.cpp',
        'src/simulation/mission_systems/weapon/MotionHistory.h',
        'src/simulation/mission_systems/weapon/Fuze.h',
        'src/simulation/mission_systems/weapon/FuzeDelay.h',
        'src/simulation/mission_systems/components/implementations/seeker/ThermalImageReceiverV1.cpp',
        'src/simulation/mission_systems/components/implementations/seeker/RadioReceiverV1.cpp',
        'src/simulation/mission_systems/components/support/ReceiverConfigurationV1.cpp',
        'src/simulation/mission_systems/components/support/WarheadSnapshotV1.h',
        'include/fsim/Rng.h', 'src/simulation/mission_systems/rf/Antenna.cpp'}
    # Explicit include closure of these already-scoped implementation inputs.
    # This does not enumerate source trees or follow excluded directories.
    pending = [root/p for p in paths if Path(p).suffix in ('.h', '.cpp')]
    while pending:
        path = pending.pop()
        for name in re.findall(r'^\s*#include\s+"([^"]+)"', path.read_text(encoding='utf-8'), re.M):
            allowed(name)
            candidates = (path.parent/name, root/'src'/name, root/'include'/name)
            target = next((allowed(p.resolve()) for p in candidates if allowed(p).is_file()), None)
            if target is not None and target.is_relative_to(root.resolve()):
                relative = target.relative_to(root.resolve()).as_posix()
                if relative not in paths:
                    paths.add(relative);pending.append(target)
    return dict(schema='fsim.sim-m03.component-effects-inputs.v1',
                sources=[bind(root/p, root) for p in sorted(paths)])


def finite(value, name):
    require(type(value) in (int, float) and math.isfinite(value), 'Finite number required: '+name)
    return float(value)


def planck_radiance(temperature, emissivity, low_um, high_um):
    """50-digit exponential-series antiderivative of NIST's wavelength law.

    Distinct from native Simpson quadrature. Terms integrate x^3 exp(-n*x)
    analytically. A positive geometric tail bound (<1e-42 of a unit integral)
    terminates the sum; all authored test bands have lower x>0. No fitting.
    """
    require(temperature > 0 and 0 <= emissivity <= 1 and 0 < low_um < high_um,
            'Invalid Planck reference domain')
    with localcontext() as context:
        context.prec = 55
        D = lambda x: Decimal(str(x))
        h, c, k = D('6.62607015e-34'), D('299792458'), D('1.380649e-23')
        t = D(temperature)
        a, b = h*c/(D(high_um)*D('1e-6')*k*t), h*c/(D(low_um)*D('1e-6')*k*t)
        total = Decimal(0)
        def tail(x, n):
            return (-n*x).exp()*(x**3/n+3*x*x/n**2+6*x/n**3+6/n**4)
        for ordinal in range(1, 10001):
            n = Decimal(ordinal)
            total += tail(a, n)-tail(b, n)
            # For m>=n+1, coefficients decrease and exp(-m*a) is geometric.
            bound = tail(a, n+1)/(1-(-a).exp())
            if bound < D('1e-42'):
                return float(D(emissivity)*2*(k*t)**4/(h**3*c**2)*total)
        raise ValueError('Planck reference series did not converge')


def radar_energy(m):
    # Factor the link as incident flux -> scattered flux -> effective aperture,
    # independently of the native collapsed lambda^2/(4pi)^3 expression.
    wavelength = 299792458./m['frequency_hz']
    incident = m['power_w']*m['transmitter_gain']/(4*math.pi*m['transmitter_range_m']**2)
    scattered = incident*m['rcs_m2']/(4*math.pi*m['receiver_range_m']**2)
    aperture = m['receiver_gain']*wavelength**2/(4*math.pi)
    attenuation = 10**((m['system_loss_db']+m['receiver_excess_loss_db']+
                       m['transmitter_excess_loss_db'])/10)
    received_energy = scattered*aperture*m['dwell_s']/attenuation
    return received_energy/(1.380649e-23*m['temperature_k']*10**(m['noise_figure_db']/10))


def detector_charge(m):
    x0 = (m['column']-.5*m['width'])*m['pixel_pitch_rad']
    y0 = (m['row']-.5*m['height'])*m['pixel_pitch_rad']
    x1, y1 = x0+m['pixel_pitch_rad'], y0+m['pixel_pitch_rad']
    def primitive(x, y):
        x, y = math.tan(x), math.tan(y)
        return math.atan2(x*y, math.sqrt(1+x*x+y*y))
    omega = primitive(x1, y1)-primitive(x0, y1)-primitive(x1, y0)+primitive(x0, y0)
    photons = (m['radiance_w_m2_sr']*omega*m['aperture_m2']*m['transmission']*
        m['integration_s']*m['wavelength_m']/(6.62607015e-34*299792458.))
    response = -math.expm1(-m['integration_s']/m['response_s'])
    return response*photons*m['quantum_efficiency']+m['dark_electrons_s']*m['integration_s']


def segment_reference(before, after):
    # Fraction retains the EXACT supplied binary64 endpoints; no copied native
    # floating-point projection or artificial geometric epsilon.
    a, b = [Fraction(x) for x in before], [Fraction(x) for x in after]
    d = [y-x for x, y in zip(a, b)]
    square = sum(x*x for x in d)
    u = max(Fraction(0), min(Fraction(1), -sum(x*y for x, y in zip(a, d))/square)) if square else Fraction(0)
    return float(u), math.sqrt(float(sum((x+u*y)**2 for x, y in zip(a, d))))


def cylinder_reference(gurney, case, charge):
    require(gurney > 0 and case >= 0 and charge > 0, 'Invalid Gurney domain')
    with localcontext() as context:
        context.prec = 55
        g, m, c = [Decimal(str(x)) for x in (gurney, case, charge)]
        charge_energy = g*g*c/2
        return float((charge_energy/(m/2+c/4)).sqrt())


def residual(name, actual, expected, tolerance):
    a, e, t = [finite(v, name) for v in (actual, expected, tolerance)]
    require(t >= 0, 'Negative tolerance')
    require(abs(a-e) <= t, f'{name}: actual={a:.17g}, expected={e:.17g}, tolerance={t:.17g}')
    return dict(name=name, actual=a, expected=e, tolerance=t)


def scaled(name, actual, expected, relative, absolute=1e-12):
    return residual(name, actual, expected, absolute+relative*abs(expected))


def compare_seeker(observation):
    result = []
    laws = set()
    for index, m in enumerate(observation):
        law = m['law'];laws.add(law)
        if law == 'planck_band_radiance':
            expected = planck_radiance(m['temperature_k'], m['emissivity'], m['low_um'], m['high_um'])
            relative = TOLERANCES['planck_relative']
        elif law == 'radar_energy':
            expected = radar_energy(m);relative = TOLERANCES['radar_relative']
        elif law == 'uniform_detector_charge':
            expected = detector_charge(m);relative = TOLERANCES['charge_relative']
        else:
            raise ValueError('Unknown seeker numerical law')
        result.append(scaled(f'{law}[{index}]', m['actual'], expected, relative))
    require(laws == {'radar_energy'} or laws == {'planck_band_radiance', 'uniform_detector_charge'},
            'Incomplete physical receiver comparison')
    if 'radar_energy' in laws:
        require(len(observation) == 2 and {m['semi_active'] for m in observation} == {True, False},
                'Both actual RF path geometries required')
    else:
        planck = [m for m in observation if m['law'] == 'planck_band_radiance']
        require(len(planck) in (4, 8) and {m['temperature_k'] for m in planck} == {300., 600., 1000., 2000.},
                'Every declared thermal band and temperature case required')
    return result


def compare_fuze(row):
    require(len(row['fuze']) == 2, 'Crossing and radius-miss cases required')
    result = []
    for i, m in enumerate(row['fuze']):
        u, miss = segment_reference((m['x0_m'], m['y_m'], 0), (m['x1_m'], m['y_m'], 0))
        result += [scaled(f'cpa_fraction[{i}]', m['fraction'], u, 64*sys.float_info.epsilon),
                   scaled(f'cpa_miss[{i}]', m['miss_m'], miss, 64*sys.float_info.epsilon),
                   residual(f'radius_decision[{i}]', int(m['triggered']), int(miss <= m['radius_m']), 0)]
    require({m['scale'] for m in row['fuze']} == {.5, 1.5}, 'Changed CPA reference cases')
    e = row['fuze_endpoint']
    result += [scaled('approach_endpoint_miss', e['miss_m'], .5*e['radius_m'], 64*sys.float_info.epsilon),
               residual('approach_endpoint_trigger', int(e['triggered']), int(e['model_version'] == 0), 0)]
    return result


def compare_warhead(w):
    base = cylinder_reference(w['gurney_ms'], w['case_kg'], w['charge_kg'])
    speed = min(base, 1400.) if w['rod'] and w['model_version'] == 0 else base
    result = [scaled('gurney_cylinder_ms', w['actual'], base, TOLERANCES['gurney_relative']),
              scaled('source_speed_ms', w['source_speed_ms'], speed, TOLERANCES['gurney_relative'])]
    e = w['emission'];n = w['fragment_count']
    require(type(n) is int and n > 0, 'Positive integral source population')
    connected = e['expanding'] > 0
    energy = .5*w['case_kg']*speed**2
    if connected:
        # Integral of the affine endpoint velocity on each homogeneous rod:
        # m/6*(v0^2+v0.v1+v1^2). It is NOT a point-mass ring energy claim.
        energy *= (2+math.cos(2*math.pi/n))/3
    result += [residual('emitted_count', e['count'], n, 0),
        scaled('emitted_mass_kg', e['mass_kg'], w['case_kg'], 64*sys.float_info.epsilon*n),
        scaled('emitted_energy_j', e['energy_j'], energy, TOLERANCES['energy_relative'], 1e-6),
        scaled('minimum_launch_speed_ms', e['minimum_speed_ms'], speed, 1e-12),
        scaled('maximum_launch_speed_ms', e['maximum_speed_ms'], speed, 1e-12),
        residual('rod_segment_count', e['segments'], n if w['rod'] else 0, 0)]
    tolerance = TOLERANCES['geometry_relative']*max(1., e['length_m'], e['outer_radius_m'])
    result += [residual('outer_radius_violation_m', max(0., e['maximum_radius_m']-e['outer_radius_m']), 0, tolerance),
        residual('inner_radius_violation_m', 0 if connected else max(0., e['inner_radius_m']-e['minimum_radius_m']), 0, tolerance),
        residual('axial_envelope_violation_m', max(0., e['maximum_axial_m']-.5*e['length_m']), 0, tolerance)]
    if connected:
        outer, length = e['outer_radius_m'], e['length_m']
        breakup = (n*length/(2*math.pi)-outer)/speed
        material = n*length*math.sin(math.pi/n)/math.pi
        result += [residual('connected_population', e['expanding'], n, 0),
            residual('connected_endpoint_gap_m', e['ring_gap_m'], 0, 0),
            scaled('ring_expansion_per_s', e['expansion_per_s'], speed/outer, 1e-12),
            scaled('ring_breakup_s', e['breakup_s'], breakup, 1e-12),
            scaled('minimum_material_length_m', e['minimum_material_length_m'], material, 1e-11),
            scaled('maximum_material_length_m', e['maximum_material_length_m'], material, 1e-11),
            residual('closed_ring_momentum_ns', e['momentum_norm_ns'], 0, 128*sys.float_info.epsilon*n*w['case_kg']*speed)]
    return result


def passing_native_report(path):
    document = ET.parse(allowed(path)).getroot()
    suites = [document] if document.tag == 'testsuite' else document.findall('./testsuite')
    require(len(suites) == 1, 'Exactly one actual Catch suite required')
    suite = suites[0]
    require(suite.get('name') in ('fsim_component_effect_reference_native_test.exe',
                                'fsim_component_effect_reference_native_test'), 'Wrong Catch suite')
    count = suite.get('tests', '')
    require(re.fullmatch('[1-9][0-9]*', count) is not None, 'Positive actual Catch assertion count required')
    require(all(suite.get(k) == '0' for k in ('failures', 'errors', 'skipped')),
            'Explicit passing Catch counters required')
    require(suite.get('disabled', '0') == '0', 'Disabled Catch execution')
    cases = suite.findall('./testcase')
    require(len(cases) == 1 and cases[0].get('name') ==
            'Actual selected AAM effects methods expose primary-reference observations'
            and cases[0].get('status') == 'run', 'Exact executed native reference case required')
    require(not document.findall('.//failure') and not document.findall('.//error')
            and not document.findall('.//skipped'), 'Native report failed or skipped')
    return len(cases)


def bound_file(item, root=ROOT):
    root = allowed(root)
    require(type(item) is dict and set(item) == {'path', 'sha256'}, 'Exact path/hash record required')
    path = allowed(item['path'])
    path = allowed((path if path.is_absolute() else root/path).resolve())
    require(path.is_relative_to(root.resolve()) or path.is_relative_to(Path('D:/ENV').resolve()), 'Foreign evidence root')
    require(type(item['sha256']) is str and re.fullmatch('[0-9a-f]{64}', item['sha256'])
            and sha(path) == item['sha256'], 'Changed/malformed artifact binding')
    return path


def validate_run(receipt_path, native_path, report_path, epoch_path, native, root=ROOT):
    root = allowed(root)
    receipt_path = allowed(receipt_path)
    for path in (native_path, report_path, epoch_path):allowed(path)
    native_path, report_path, epoch_path = [allowed(allowed(p).resolve())
                                          for p in (native_path, report_path, epoch_path)]
    receipt = read(allowed(receipt_path))
    require(receipt.get('schema') == 'fsim.sim-m03.component-reference-native-run.v1'
            and receipt.get('passed') is True, 'Actual passing native execution receipt required')
    require(native.get('input_epoch_sha256') == sha(epoch_path), 'Capture is from a different source epoch')
    executor = bound_file(native['executable'], root)
    wanted = {path:sha(path) for path in (native_path, report_path, epoch_path)}
    matches = []
    for execution in receipt.get('executions', []):
        records = execution.get('raw', [])
        # Guard all supplied metadata paths before even resolving them. Only
        # the matching execution is read/hashed below, avoiding cross-kind I/O.
        paths = [allowed((allowed(item['path']) if allowed(item['path']).is_absolute()
                          else root/allowed(item['path'])).resolve()) for item in records]
        if native_path in paths:
            require(len(paths) == len(set(paths)), 'Duplicate execution raw binding')
            current = {bound_file(item, root):item['sha256'] for item in records}
            require(all(current.get(path) == digest for path, digest in wanted.items()),
                    'Run receipt does not bind the actual capture/Catch report/pre-epoch pair')
            matches.append(execution)
    require(len(matches) == 1, 'Exactly one current capture execution required')
    execution = matches[0]
    require(type(execution.get('exit_code')) is int and execution['exit_code'] == 0,
            'Native execution did not exit zero')
    runtimes = {bound_file(item, root):item['sha256'] for item in execution.get('runtime', [])}
    require(runtimes.get(executor) == native['executable']['sha256'], 'Producer executable was not bound by the actual run')
    command = execution.get('command')
    require(type(command) is list and len(command) == 6 and
            command[1:5] == ['[component-effects-reference]', '--reporter', 'junit', '--out'],
            'Exact actual Catch command required')
    require(allowed(allowed(command[0]).resolve()) == executor and
            allowed(allowed(command[5]).resolve()) == report_path, 'Native executable/report command mismatch')
    environment = execution.get('environment', {})
    require(allowed(allowed(environment['FSIM_COMPONENT_EFFECT_REFERENCE_EPOCH']).resolve()) == epoch_path and
            allowed(allowed(environment['FSIM_COMPONENT_EFFECT_REFERENCE_DIR']).resolve()) == native_path.parent,
            'Run environment does not identify the actual epoch/output')
    from component_reference_run import verify_execution_runtime
    verify_execution_runtime(execution, [native], executor, 'effects', root)
    return receipt


def compare_capture(native, cells, root=ROOT):
    require(native.get('schema') == 'fsim.sim-m03.component-effects-native.v1'
            and native.get('complete') is True, 'Incomplete/foreign native method capture')
    observations = native['models']
    expected = {c['model'] for c in cells}
    require(len(observations) == 24 and len({r['model'] for r in observations}) == 24
            and {r['model'] for r in observations} == expected, 'Exact24 native method population required')
    results = {}
    for c in cells:
        r = next(r for r in observations if r['model'] == c['model'])
        require(r['card_sha256'] == c['card_sha256'], 'Stale native card')
        implementation = r[c['component']+'_implementation']
        require(implementation == (c['implementation_id'] if c['component'] == 'seeker' else c['mechanism_id']),
                'Native provider differs from actual selection')
        bindings = validate_inputs(c, r, root)
        f = c['component']
        results[(c['model'], f)] = bindings + (compare_seeker(r['seeker']) if f == 'seeker' else compare_fuze(r) if f == 'fuze' else compare_warhead(r['warhead']))
    return results


def project_warhead(cell, root=ROOT):
    """Independent card-only geometry/mass projection; no native observations.

    These are the simulator's declared equivalent-source assumptions, not
    independently sourced OEM geometry. Decimal arithmetic avoids copying the
    production operation rounding; the predeclared numerical budgets apply.
    """
    root = allowed(root)
    card = read(root/cell['card_path'])
    require(sha(root/cell['card_path']) == cell['card_sha256'], 'Stale projection card')
    p = {name: Decimal(str(r['value'])) for name, r in cell['parameters'].items()}
    require(card['warhead_kind'] in ('fragmentation', 'rod'), 'Unknown authored warhead kind')
    with localcontext() as context:
        context.prec = 55
        pi = Decimal('3.14159265358979323846264338327950288419716939937510582')
        total, charge = p['warhead_kg'], p['explosive_kg']
        version = int(p.get('warhead_model_version', 0))
        require(version in (0, 1, 2), 'Unknown authored source version')
        finite_source = version != 0
        rod = card['warhead_kind'] == 'rod'
        structure = p['warhead_structure_fraction'] if finite_source else Decimal('.15')
        ratio = p['warhead_diameter_ratio'] if finite_source else Decimal('.9')
        rho_case = p['warhead_case_density_kg_m3'] if finite_source else Decimal(7850)
        rho_charge = p['warhead_explosive_density_kg_m3'] if finite_source else Decimal(1700)
        rho_structure = p['warhead_structure_density_kg_m3'] if finite_source else Decimal(2780)
        require(total > charge > 0 and 0 <= structure < 1 and ratio > 0,
                'Invalid authored source mass/geometry')
        case = (1-structure)*(total-charge)
        outer = ratio*p['diameter_m']/2
        # Cylinder volume closure, followed by the selected case-wall law.
        volume = charge/rho_charge+case/rho_case+structure*(total-charge)/rho_structure
        length = volume/(pi*outer*outer)
        if finite_source:
            require(not rod and p['warhead_fragment_source'] == version, 'Wrong finite source/kind')
            annulus = case/(rho_case*pi*length)
            inner = (outer*outer-annulus).sqrt()
            thickness = annulus/(outer+inner)
        else:
            thickness = case/(rho_case*pi*(2*outer)*max(length, Decimal('.001')))
            inner = max(Decimal(0), outer-thickness)
        nominal = rho_case*thickness*thickness*(length if rod else thickness)
        raw_count = case/nominal
        # All supported masses are positive: C++ lround is positive half-up.
        count = int((raw_count+Decimal('.5')).to_integral_value(rounding='ROUND_FLOOR'))
        fragment = case/count if finite_source else nominal
        area = (thickness*thickness/2+thickness*length if rod else
                Decimal('1.5')*(fragment/rho_case)**(Decimal(2)/3) if finite_source else
                Decimal('1.5')*thickness*thickness)
        tnt = p['warhead_tnt_equivalent_kg'] if finite_source else charge*(p['gurney_kms']*1000/2440)**2
        if version:
            require(cell['mechanism_id'] == ('native.warhead.finite_zoned_case.v2' if version == 2 else
                    'native.warhead.side_case.v1'), 'Authored source differs from mechanism cohort')
        else:
            # A rod family label does NOT select a connected ring. The
            # independently admitted equipment catalogue makes that explicit
            # choice; zoned V0 also contains real discrete-rod selections.
            require(cell['mechanism_id'] == 'native.warhead.zoned_case.v1' or
                    (rod and cell['mechanism_id'] == 'native.warhead.connected_ring.v1'),
                    'Authored source differs from mechanism cohort')
        return dict(case_kg=float(case), gurney_ms=float(p['gurney_kms']*1000),
            charge_kg=float(charge), model_version=version, fragment_source=version,
            rod=rod, fragment_count=count, fragment_kg=float(fragment), fragment_area_m2=float(area),
            tnt_kg=float(tnt), outer_radius_m=float(outer), inner_radius_m=float(inner), length_m=float(length))


def validate_inputs(cell, row, root=ROOT):
    """Bind numeric observations to actual resolved policies and fixed cases."""
    p = {name: r['value'] for name, r in cell['parameters'].items()}
    def equal(actual, expected, name):
        require(type(actual) in (int, float) and math.isfinite(actual) and actual == expected,
                'Changed/malformed reference input '+name)
    family = cell['component']
    if family == 'seeker':
        observations = row['seeker']
        if cell['bindings']['receiver_model'] == 'rf_signal.v1':
            require(len(observations) == 2, 'Missing RF branch')
            for m in observations:
                require(m['law'] == 'radar_energy' and type(m['semi_active']) is bool, 'Foreign RF law')
                for key, role in dict(frequency_hz='frequency_hz', temperature_k='temperature_k',
                    noise_figure_db='noise_figure_db', system_loss_db='system_loss_db', dwell_s='receiver_rf_dwell_s').items():
                    equal(m[key], p[role], key)
                for key, expected in dict(receiver_range_m=5000., rcs_m2=1., receiver_excess_loss_db=.3,
                    transmitter_range_m=10000. if m['semi_active'] else 5000.,
                    transmitter_excess_loss_db=.5 if m['semi_active'] else .3,
                    power_w=1000. if m['semi_active'] else p['emitter_power_w']).items():
                    equal(m[key], expected, key)
                require(m['receiver_gain'] > 0, 'Finite receiver gain required')
                equal(m['transmitter_gain'], 100. if m['semi_active'] else m['receiver_gain'], 'transmitter gain')
        else:
            bands = {(p['ir_band_low_um'], p['ir_band_high_um'])}
            if cell['bindings']['receiver_model'] == 'thermal_dual_image.v1':
                bands.add((p['ir_band2_low_um'], p['ir_band2_high_um']))
            planck = [m for m in observations if m['law'] == 'planck_band_radiance']
            expected = {(low, high, t) for low, high in bands for t in (300., 600., 1000., 2000.)}
            require(len(planck) == len(expected) and {(m['low_um'], m['high_um'], m['temperature_k']) for m in planck} == expected,
                    'Missing/foreign/duplicate authored band comparison')
            for m in planck:equal(m['emissivity'], .8, 'emissivity')
            detector = [m for m in observations if m['law'] == 'uniform_detector_charge']
            require(len(detector) == 1 and type(detector[0]['imaging']) is bool, 'Missing detector cohort comparison')
            m = detector[0]
            require(m['imaging'] == (cell['bindings']['mode'] == 'imaging_infrared'), 'Wrong detector cohort')
            for key, role in dict(width='receiver_image_width', height='receiver_image_height',
                pixel_pitch_rad='receiver_pixel_pitch_rad', aperture_m2='receiver_aperture_area_m2',
                transmission='receiver_optical_transmission', integration_s='receiver_integration_s',
                response_s='receiver_pixel_response_s', quantum_efficiency='receiver_quantum_efficiency',
                wavelength_m='receiver_effective_wavelength_m', dark_electrons_s='receiver_dark_current_electrons_s').items():
                equal(m[key], p[role], key)
            equal(m['column'], p['receiver_image_width']//2, 'column')
            equal(m['row'], p['receiver_image_height']//2, 'row')
            equal(m['radiance_w_m2_sr'], 1., 'constant radiance')
    elif family == 'fuze':
        for m in row['fuze']:
            for key, role in dict(radius_m='radius_m', arming_s='arming_s', life_s='life_s').items():
                equal(m[key], p[role], key)
            equal(m['delay_s'], p.get('delay_s', 0.), 'delay')
            equal(m['x0_m'], 100., 'own before');equal(m['x1_m'], -100., 'own after')
            equal(m['y_m'], m['scale']*p['radius_m'], 'own offset')
            equal(m['age_s'], p['arming_s']+.25, 'armed reference age')
            equal(m['duration_s'], 1., 'duration')
        equal(row['fuze_endpoint']['model_version'], p['model_version'], 'endpoint law')
        equal(row['fuze_endpoint']['radius_m'], p['radius_m'], 'endpoint radius')
    else:
        w = row['warhead']
        equal(w['gurney_ms'], p['gurney_kms']*1000., 'explosive Gurney input')
        equal(w['charge_kg'], p['explosive_kg'], 'charge mass')
        require(w['case_kg'] > 0 and type(w['rod']) is bool, 'Invalid native warhead model')
        connected = cell['mechanism_id'] == 'native.warhead.connected_ring.v1'
        require((w['emission']['expanding'] > 0) == connected, 'Foreign rod topology')
        expected = project_warhead(cell, root)
        result = []
        for name in ('model_version', 'fragment_source', 'fragment_count', 'rod'):
            if name == 'rod':
                result.append(residual('authored_'+name, int(w[name]), int(expected[name]), 0))
            else:
                result.append(residual('authored_'+name, w[name], expected[name], 0))
        for name in ('case_kg', 'fragment_kg', 'fragment_area_m2', 'tnt_kg'):
            result.append(scaled('authored_'+name, w[name], expected[name], TOLERANCES['gurney_relative']))
        for name in ('outer_radius_m', 'inner_radius_m', 'length_m'):
            result.append(scaled('authored_'+name, w['emission'][name], expected[name],
                                 TOLERANCES['geometry_relative']))
        return result
    return []


def family_theory(family, mechanism):
    if family == 'seeker':
        return dict(equations='L=epsilon integral[2hc^2/(lambda^5(expm1(hc/lambda/k/T)))]dlambda; Q=L*Omega*A*t*tau*QE*lambda_eff/(hc); SNR=Pr*t/(k*T*F).',
            limits='Finite grey bands/rectangular projected patches, Gaussian PSF, effective photon wavelength, explicit detector/RF timing and assumed geometry. These method comparisons do not validate ECCM, Pd/Pfa or OEM acquisition envelope.',
            delta=mechanism+': original selected physical receiver; IR quadrant charge versus imaging segmentation; SARH requires real separate illumination and temporal PRF coincidence; ARH uses own transmitter. All receive coefficients retain original card uncertainty.',
            primary_sources=[PRIMARY[k] for k in ('planck', 'si', 'radar')])
    if family == 'fuze':
        return dict(equations='r(u)=r0+u*d; u*=clamp(-r0.d/(d.d),0,1); miss=norm(r(u*)); initiate only according to selected armed/live CPA law.',
            limits='Only supplied captured piecewise-linear path segments; geometric CPA alone does not grant pose, arming, target identity, terrain contact or detonation authority. Lifetime end is exclusive.',
            delta=mechanism+': legacy accepts the constrained interval minimum; turning_cpa requires an actual stationary/corner turn in the armed live interval and retains contiguous exact boundary history. Fixed-delay latch is one-shot and separately native tested.',
            primary_sources=[PRIMARY['cpa']])
    return dict(equations='E*C=(M/2+C/4)*v^2; v=sqrt(2E)/sqrt(M/C+1/2). Emitted sum(m)=M; point E=sum(m*v^2/2); rod E=sum(m/6*(v0.v0+v0.v1+v1.v1)).',
        limits='Gurney uniform cylinder and linear gas-velocity approximation, not finite explosive chemistry, weld mechanics, OEM lethality or fragment measurements. Blast and THOR penetration have separate empirical domains; these comparisons do not calibrate them.',
        delta=mechanism+': actual selected finite annular/zone or connected-ring emission. Legacy rods retain the authored 1400 m/s cap; finite source uses declared mass, station and Cd. A connected homogenized polygon unfolds only until its declared length budget; no hardware weld-count claim.',
        primary_sources=[PRIMARY['gurney']])


def generate(native_path, report_path, epoch_path, output, root=ROOT, run_receipt=None):
    root = allowed(root)
    native_path, report_path, epoch_path, output = [allowed(p) for p in
                                                  (native_path, report_path, epoch_path, output)]
    require(run_receipt is not None, 'Actual producer run receipt is mandatory')
    run_receipt = allowed(run_receipt)
    current = source_epoch(root)
    require(read(epoch_path) == current, 'Native input epoch is stale or incomplete')
    passing_native_report(report_path)
    cells = inventory(root)
    native = read(native_path)
    validate_run(run_receipt, native_path, report_path, epoch_path, native, root)
    values = compare_capture(native, cells, root)
    raw = [bind(path, root) for path in (native_path, report_path, epoch_path, run_receipt)]
    output = Path(output);output.mkdir(parents=True, exist_ok=True)
    results = dict(schema='fsim.sim-m03.effects-reference-results.v1', passed=True,
        native=raw, tolerances=TOLERANCES, comparisons=[dict(model=m, component=f, residuals=r)
            for (m, f), r in sorted(values.items())])
    (output/'references').mkdir(exist_ok=True)
    result_path = output/'references/effects-results.v1.json'
    result_path.write_text(json.dumps(results, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    raw.append(bind(result_path, root))
    for family in FAMILIES:
        selected = [c for c in cells if c['component'] == family]
        rows = []
        for cell in selected:
            model, mechanism = cell['model'], cell['mechanism_id']
            peers = sorted(c['model'] for c in selected if c['mechanism_id'] == mechanism)
            card = read(root/cell['card_path'])
            rows.append(dict(**{k: cell[k] for k in ('model', 'component', 'implementation_id', 'mechanism_id', 'card_sha256', 'parameter_sha256')},
                passed=True, theory=family_theory(family, mechanism),
                method=dict(case='Actual selected-provider native capture versus independent primary equations',
                    tests=len(values[(model, family)]), assertions=len(values[(model, family)]),
                    residuals=values[(model, family)]),
                shared_equivalence=dict(id='effects-equations-v1:'+mechanism, models=peers, mechanism_id=mechanism,
                    scope='The exact shared production numerical mechanism with each model\'s full original resolved parameter records; separate native observations for every model.',
                    limits='Only listed equations and cohort-specific controls; not equivalent OEM hardware, calibrated parameters, coupled scene outcomes, public or visual acceptance.'),
                parameters=cell['parameters'], parameter_refs=cell['parameter_refs'],
                parameter_sources={role:card.get('sources', {}).get(p.get('source'),
                    dict(unresolved_source_id=p.get('source'), basis=p['basis'], note=p.get('note', 'No independent equipment source')))
                    for role, p in cell['parameters'].items()},
                bindings=cell['bindings'], equipment_accuracy_calibrated=False))
        artifact = dict(schema='fsim.sim-m03.component-reference.v1', component=family, passed=True,
            scope='Primary-equation numerical verification and explicit per-model theory; not equipment calibration or full-program acceptance.',
            sources=current['sources'], raw=raw, models=rows)
        (output/(family+'.json')).write_text(json.dumps(artifact, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=ROOT)
    parser.add_argument('--freeze-epoch', type=Path)
    parser.add_argument('--native', type=Path)
    parser.add_argument('--native-report', type=Path)
    parser.add_argument('--native-epoch', type=Path)
    parser.add_argument('--run-receipt', type=Path)
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    args.root = allowed(args.root)
    if args.freeze_epoch:
        require(not any((args.native, args.native_report, args.native_epoch, args.output_dir, args.run_receipt)), 'Separate epoch command required')
        allowed(args.freeze_epoch).write_text(json.dumps(source_epoch(args.root), indent=2)+'\n', encoding='utf-8')
        return 0
    require(all((args.native, args.native_report, args.native_epoch, args.output_dir, args.run_receipt)), 'Native capture, passing report, pre-execution epoch, producer run receipt and output required')
    generate(args.native, args.native_report, args.native_epoch, args.output_dir, args.root, args.run_receipt)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
