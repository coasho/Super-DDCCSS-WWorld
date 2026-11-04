"""Admit actual all-aircraft liquid products, without regenerating geometry.

Independent checks cover source/native identity, units, capacities, ownership,
origins and exact AABB material accounting. Manifold containment is a hash-bound
producer receipt, not independently rerun geometry or equipment calibration.
"""
import argparse
from collections import defaultdict
import hashlib
import importlib.util
import json
import math
import multiprocessing as mp
from pathlib import Path
import re
import struct
import sys
import tomllib
import xml.etree.ElementTree as ET

from acceptance import allowed

ROOT = Path(__file__).resolve().parents[3]
EPS = sys.float_info.epsilon
POLICY = dict(law='quadratic_unresolved_tumbling_v1', drag_coefficient=1.0, basis='assumed',
              screening_uncertainty_fraction=1.0,
              note='Unresolved tumbling pressure-drag engineering policy; not FAA/OEM calibration or air Cd.')
METHODS = {'producer': 'tools/models/hangar/hangar/liquid.py',
           'geometry_body': 'tools/models/hangar/hangar/geometry/body.py',
           'geometry_aircraft_engine': 'tools/models/hangar/hangar/geometry/aircraft.py',
           'native_engine_order': 'tools/models/hangar/hangar/jsbsim.py',
           'exclusion_method': 'tools/models/hangar/hangar/vulnerability.py'}
SOURCE_PATHS = {*METHODS.values(), 'tools/models/hangar/hangar/model3d.py',
                'tools/models/hangar/hangar/resources.py',
                'tools/models/hangar/hangar/liquid_vessels.py',
                'tools/models/hangar/tests/test_liquid_vessels.py',
                'tools/models/hangar/tests/test_liquid_vessel_packing.py',
                'tools/models/hangar/tests/test_liquid_vessel_inputs.py',
                'tools/validation/weapon_quality/test_liquid_vessel_policy.py',
                'tools/models/hangar/hangar/shape/breakup.py', 'tools/models/hangar/hangar/files.py',
                'tools/models/hangar/hangar/pipeline.py', 'tools/models/hangar/hangar/rotorcraft/__init__.py',
                'tools/models/hangar/hangar/rotorcraft/heli.py',
                'tools/validation/weapon_quality/liquid_population.py',
                'tools/validation/weapon_quality/test_liquid_population.py'}


def require(value, reason):
    if not value: raise ValueError(reason)


def number(value, label, *, positive=False):
    require(type(value) in (int, float) and math.isfinite(value), 'Nonfinite/non-numeric '+label)
    require(not positive or value > 0, 'Nonpositive '+label)
    return float(value)


def integer(value, label):
    require(type(value) is int and value >= 0, 'Invalid integer '+label)
    return value


def vector(value, label):
    require(isinstance(value, list) and len(value) == 3, 'Invalid vector '+label)
    return [number(v, label) for v in value]


def close(a, b, tolerance, reason):
    require(math.isfinite(a) and math.isfinite(b) and abs(a-b) <= tolerance, reason)


def sha(path): return hashlib.sha256(allowed(path).read_bytes()).hexdigest()


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def read(path):
    def pairs(items):
        result = {}
        for k, v in items:
            require(k not in result, 'Duplicate JSON key: '+k); result[k] = v
        return result
    def invalid(value): raise ValueError('Nonfinite JSON token '+value)
    result = json.loads(allowed(path).read_text(encoding='utf-8'), object_pairs_hook=pairs, parse_constant=invalid)
    def finite(value):
        if type(value) in (int, float): number(value, 'JSON numeric value')
        elif isinstance(value, dict):
            for child in value.values(): finite(child)
        elif isinstance(value, list):
            for child in value: finite(child)
    finite(result)
    return result


def bind(path, role):
    path = allowed(path).resolve()
    require(path.is_file(), 'Missing '+role+': '+str(path))
    return dict(path=str(path), role=role, sha256=sha(path), bytes=path.stat().st_size)


def verify_sources(rows, expected):
    """Exact source epoch reconciliation, independently testable without geometry."""
    require(isinstance(rows, list) and all(isinstance(r, dict) and isinstance(r.get('role'), str) for r in rows),
            'Malformed producer source receipt')
    require(len({r['role'] for r in rows}) == len(rows), 'Duplicate producer source role')
    refs = {r['role']: r for r in rows}
    require(set(refs) == set(expected), 'Missing/extra producer source roles')
    for role, reference in expected.items():
        actual = refs[role]
        require(isinstance(actual.get('sha256'), str) and re.fullmatch('[0-9a-f]{64}', actual['sha256']) is not None,
                'Malformed producer source digest')
        require(actual['sha256'] == reference['sha256'] and actual.get('path') == reference['path'],
                'Stale/mixed producer source epoch: '+role)
    return refs


def validate_vessel_declaration(doc, design):
    """Independent exact source policy; no producer parser imported here."""
    expected = dict(schema='fsim.inferred-liquid-vessels.v1', design='rq4b', basis='theoretical_inference',
                    geometry='original_source_wing_root_strip_v1',
                    attachment='sealed_whole_native_tank_wing_module_v1',
                    tanks=[dict(index=0, section='left wing', surface='wing'),
                           dict(index=1, section='right wing', surface='wing')])
    require(design == 'rq4b' and isinstance(doc, dict) and doc == expected and
            all(type(t.get('index')) is int for t in doc.get('tanks', [])), 'Unsupported inferred vessel source declaration')
    return doc


def vessel_root_cut(spec):
    """Independently reproduce the unchanged structural cut's linear widths."""
    wings = [s for s in spec['surface'] if s.get('kind') == 'wing']
    require(len(wings) == 1 and wings[0]['name'] == 'wing', 'Wrong inferred source wing')
    root = wings[0]['sections'][0]; x = root['le'][0]+.5*root['chord']; widths = []
    require(root['le'][1] == 0 and wings[0].get('mirror', True) is True, 'Wrong source centreline/mirror')
    for body in spec.get('body', []):
        if body.get('kind', 'fuselage') != 'fuselage': continue
        origin = body.get('origin', [0, 0, 0]); rows = body['stations']
        for a, b in zip(rows, rows[1:]):
            if a['x']+origin[0] <= x <= b['x']+origin[0]:
                t = (x-origin[0]-a['x'])/(b['x']-a['x']); widths.append(.5*((1-t)*a['w']+t*b['w'])); break
    require(widths, 'Source wing root has no structural fuselage cut')
    return max(widths)+.05


def _vessel_summary(cells, anchor, fraction):
    """Independent scalar volume/first-moment oracle for body -Z reference up."""
    target = math.fsum(c['volume_m3'] for c in cells)*fraction
    low = min(c['low_m'][2] for c in cells); high = max(c['high_m'][2] for c in cells)
    if fraction < 1:
        for _ in range(100):
            trial = low+(high-low)*.5
            volume = math.fsum((c['high_m'][0]-c['low_m'][0])*(c['high_m'][1]-c['low_m'][1])*
                              max(0., c['high_m'][2]-max(c['low_m'][2], trial)) for c in cells)
            if volume >= target: low = trial
            else: high = trial
        surface = low+(high-low)*.5
    else: surface = low
    wet = []
    for c in cells:
        lo = list(c['low_m']); hi = c['high_m']; lo[2] = max(lo[2], surface)
        if hi[2] > lo[2]: wet.append((lo, hi, math.prod(b-a for a, b in zip(lo, hi))))
    volume = math.fsum(w[2] for w in wet)
    centre = [math.fsum(v*(lo[a]+hi[a])*.5 for lo, hi, v in wet)/volume for a in range(3)]
    return dict(volume_m3=volume, centroid_body_m=centre, offset_from_native_lump_m=[a-b for a, b in zip(centre, anchor)],
                offset_norm_m=math.sqrt(math.fsum((a-b)**2 for a, b in zip(centre, anchor))),
                low_m=[min(lo[a] for lo, _, _ in wet) for a in range(3)],
                high_m=[max(hi[a] for _, hi, _ in wet) for a in range(3)], surface_body_z_m=surface,
                reference_up_body=[0., 0., -1.], contents_fraction=fraction)


def validate_vessel_receipt(doc, expected):
    policy = expected.get('vessel_declaration'); receipt = doc.get('inferred_vessels_v1')
    if policy is None:
        require(receipt is None and all('eligibility_v1' not in c for c in doc['cells']), 'Undeclared inferred vessel product')
        return
    validate_vessel_declaration(policy, expected['design'])
    require(isinstance(receipt, dict) and receipt.get('declaration') == policy and
            receipt.get('attachment') == policy['attachment'], 'Missing/changed inferred vessel receipt')
    require(doc['ledger'].get('native_owner_interval_certified') is False and
            doc['ledger'].get('per_tank_vessel_eligibility_certified') is True,
            'Inferred root vessel cannot claim singleton structural skin ownership')
    packing = receipt.get('packing_v1')
    require(isinstance(packing, dict) and packing.get('method') == 'nested_eligible_shared_pool_v1' and
            packing.get('assignment_order') == [2, 3, 0, 1] and packing.get('definite_eligibility_masks') is True,
            'Missing/unsupported definite inferred shared-pool method')
    require(set(packing) == {'method', 'assignment_order', 'definite_eligibility_masks',
                            'target_volume_m3_by_tank', 'nested_certified_feasibility'} and
            all(type(i) is int for i in packing['assignment_order']), 'Malformed inferred packing identity')
    native = {t['index']: t for t in expected['tanks']}
    require(set(native) == {0, 1, 2, 3}, 'Shared pool requires the four unchanged native RQ tanks')
    require(all(native[i]['section'] == ('left wing' if i in (0, 2) else 'right wing') for i in range(4)),
            'Shared pool native mechanical owners changed')
    targets = packing.get('target_volume_m3_by_tank')
    require(isinstance(targets, list) and [r.get('tank') for r in targets] == [0, 1, 2, 3],
            'Incomplete/reordered original per-tank resolution')
    demand = {i: number(t['capacity_kg'], 'native capacity', positive=True)/
                 number(t['density_kg_m3'], 'native density', positive=True) for i, t in native.items()}
    for row in targets:
        integer(row['tank'], 'original target tank')
        close(number(row['target_m3'], 'original single-tank target', positive=True), demand[row['tank']]/64,
              16*EPS*max(1., demand[row['tank']]/64), 'Changed original per-tank resolution')
    feasibility = packing.get('nested_certified_feasibility')
    require(isinstance(feasibility, list) and len(feasibility) == 2, 'Missing nested certified-volume feasibility')
    for row, (restricted, flexible) in zip(feasibility, ((2, 0), (3, 1))):
        integer(row['restricted_tank'], 'restricted native tank'); integer(row['flexible_tank'], 'flexible native tank')
        require(row.get('restricted_tank') == restricted and row.get('flexible_tank') == flexible,
                'Changed nested feasibility identities')
        dr, total = demand[restricted], math.fsum((demand[restricted], demand[flexible]))
        close(number(row['restricted_required_m3'], 'restricted demand', positive=True), dr,
              16*EPS*max(1., dr), 'Wrong restricted demand')
        close(number(row['total_required_m3'], 'nested total demand', positive=True), total,
              16*EPS*max(1., total), 'Wrong nested total demand')
        ordinary = number(row['ordinary_certified_m3'], 'certified ordinary pool', positive=True)
        combined = number(row['total_certified_m3'], 'certified nested pool', positive=True)
        require(ordinary >= dr and combined >= total and combined >= ordinary,
                'Nested certified pool cannot fulfill restricted and total demands')
    cut = number(receipt['structural_root_cut_design_y_m'], 'structural root cut', positive=True)
    close(cut, expected['vessel_root_cut'], 32*EPS*max(1., cut), 'Structural root cut differs from source')
    mesh = receipt['source_wing']
    require(mesh['method'] == 'original_uninflated_wing_primitive_root_clip_v1' and mesh['surface'] == 'wing' and
            mesh['skin_inflation_m'] == 0 and mesh['root_extension_m'] == 0, 'Source-wing geometry enlarged or replaced')
    for key in ('vertices_sha256', 'triangles_sha256'):
        require(isinstance(mesh[key], str) and re.fullmatch('[0-9a-f]{64}', mesh[key]), 'Malformed source-wing mesh digest')
    for key in ('vertices', 'triangles'): require(0 < integer(mesh[key], key) <= 2097152, 'Source-wing mesh budget exceeded')
    for key in ('cell_m', 'error_m', 'volume_m3'): number(mesh[key], key, positive=True)
    require(mesh['error_m'] == .25*mesh['cell_m'], 'Source-wing mesher error differs from existing method')
    selected = {r['index']: r['section'] for r in policy['tanks']}; by_tank = defaultdict(list)
    for c in doc['cells']:
        kind = c.get('eligibility_v1'); require(kind in ('native_owner_interval_v1', 'inferred_original_wing_root_v1'),
                                              'Missing or unknown per-tank final-slab certificate')
        by_tank[c['tank']].append(c)
        require(c['tank'] in native, 'Unknown inferred pool tank')
        eligible = (0, 2) if native[c['tank']]['section'] == 'left wing' else (1, 3)
        target = demand[c['tank']]/64 if kind == 'inferred_original_wing_root_v1' else min(demand[i]/64 for i in eligible)
        require(number(c['volume_m3'], 'inferred slab volume', positive=True) <= target*(1+1e-12),
                'Inferred cell coarsens an eligible original per-tank resolution')
        if kind == 'inferred_original_wing_root_v1':
            require(c['tank'] in selected and c['section'] == selected[c['tank']], 'Root volume borrowed by restricted native tank')
            oy = expected['origin'][1]; lower, upper = (-cut-oy, -oy) if c['tank'] == 0 else (-oy, cut-oy)
            require(lower <= c['low_m'][1] < c['high_m'][1] <= upper, 'Root cell leaves declared original half-strip')
    for row in feasibility:
        indices = (row['restricted_tank'], row['flexible_tank'])
        owned = [c for i in indices for c in by_tank[i]]
        ordinary = math.fsum(c['volume_m3'] for c in owned if c['eligibility_v1'] == 'native_owner_interval_v1')
        total = math.fsum(c['volume_m3'] for c in owned)
        rounding = 512*EPS*max(1., total)*max(1, len(owned))
        require(ordinary <= row['ordinary_certified_m3']+rounding and total <= row['total_certified_m3']+rounding,
                'Allocated material exceeds claimed certified shared pool')
    rows = receipt['tanks']; require(isinstance(rows, list) and [r['index'] for r in rows] == [0, 1], 'Missing/reordered inferred native tanks')
    for row in rows:
        i = integer(row['index'], 'inferred tank'); require(row['section'] == selected[i], 'Inferred native mechanical owner changed')
        owned = by_tank[i]; require(owned, 'Inferred native vessel has no material')
        for key, kind in [('ordinary_m3', 'native_owner_interval_v1'), ('root_m3', 'inferred_original_wing_root_v1')]:
            value = math.fsum(c['volume_m3'] for c in owned if c['eligibility_v1'] == kind)
            close(number(row[key], key), value, 64*EPS*max(1., value), 'Wrong ordinary/root capacity split')
        for key, fraction in [('full', 1.), ('half_reference_up', .5)]:
            anchor = next(t['anchor_body_m'] for t in expected['tanks'] if t['index'] == i)
            actual = row[key]; oracle = _vessel_summary(owned, anchor, fraction)
            require(set(actual) == set(oracle), 'Incomplete spatial/native lump discrepancy report')
            for field, value in oracle.items():
                if isinstance(value, list):
                    given = vector(actual[field], field)
                    require(all(abs(a-b) <= 512*EPS*max(1., abs(b)) for a, b in zip(given, value)), 'Wrong actual-cell '+field)
                else: close(number(actual[field], field), value, 512*EPS*max(1., abs(value)), 'Wrong actual-cell '+field)
    require(isinstance(receipt.get('centroid_scope'), str) and 'native point masses unchanged' in receipt['centroid_scope'],
            'Missing native lump/actual spatial material limitation')
    require(all(isinstance(doc['uncertainty'].get(k), str) and doc['uncertainty'][k].strip()
                for k in ('inferred_vessel_attachment', 'native_lump_discrepancy')), 'Missing inferred fracture/mass uncertainty')


def population(coverage, root):
    """Read maintained TOML identities independently of supplied coverage rows."""
    rows = {}
    for folder in sorted(allowed(root/'aircraft').iterdir()):
        if folder.name.casefold() == '.codex': continue
        path = folder/(folder.name+'.toml')
        if not path.is_file(): continue
        spec = tomllib.loads(path.read_text(encoding='utf-8')); identity = spec['aircraft']
        if identity.get('category') in ('store', 'missile'): continue
        name = identity['name']; require(name == folder.name and name not in rows, 'Invalid maintained aircraft identity')
        category = 'rotorcraft' if identity.get('kind') in ('helicopter', 'multirotor') else 'fixed-wing'
        rows[name] = dict(design=name, category=category, path=path, spec=spec)
    require(len(rows) == 35 and sum(r['category'] == 'fixed-wing' for r in rows.values()) == 31 and
            sum(r['category'] == 'rotorcraft' for r in rows.values()) == 4, 'Incomplete maintained all35 aircraft roster')
    fits = coverage['fits']; names = [r['design'] for r in fits]
    require(len(names) == len(set(names)) == 35 and set(names) == set(rows), 'Coverage differs from maintained all35 roster')
    require(all(r['category'] == rows[r['design']]['category'] for r in fits), 'Coverage aircraft category mismatch')
    return rows


def location(element):
    require(element is not None, 'Missing native structural location')
    factor = {'M': 1., 'IN': .0254, 'FT': .3048}.get(element.get('unit', 'IN').upper())
    require(factor is not None, 'Unsupported native length unit')
    return [number(float(element.findtext(k)), 'native location')*factor for k in ('x', 'y', 'z')]


def expected_model(name, spec, xml, damage):
    """Independently derive native capacities/density/origin and real ownership."""
    rotor = spec['aircraft'].get('kind') in ('helicopter', 'multirotor')
    engine = spec.get('engine', [])
    density = 720. if isinstance(engine, list) and engine and all(e['type'] == 'piston' for e in engine) else 800.
    authored = []
    if spec['aircraft'].get('kind') == 'helicopter':
        cap = number(spec.get('fuel', {}).get('capacity_lb', 0.), 'rotor capacity')
        require(cap >= 0, 'Negative rotor capacity')
        origin = [number(v, 'rotor CG')*.0254 for v in spec['mass']['cg_in']]
        if cap > 0: authored.append(dict(index=0, capacity_kg=cap*.45359237, position=origin, density_kg_m3=800.))
        native_cg = location(xml.find("mass_balance/location[@name='CG']"))
        require(all(abs(a-b) <= 1e-9 for a, b in zip(origin, native_cg)), 'Authored/native rotor CG differs')
    else:
        origin = [0., 0., 0.] if rotor else location(xml.find("metrics/location[@name='VRP']"))
        for i, t in enumerate(spec.get('mass', {}).get('tank', [])):
            cap = number(t.get('capacity', 0.), 'authored capacity'); require(cap >= 0, 'Negative capacity')
            if cap > 0: authored.append(dict(index=i, capacity_kg=cap, density_kg_m3=density,
                                           position=vector(t.get('position', [0, 0, 0]), 'authored anchor')))
    native = []
    for index, tank in enumerate(xml.findall('propulsion/tank')):
        require(tank.get('type', 'FUEL') == 'FUEL', 'Unexpected native nonfuel tank')
        capacity = tank.find('capacity'); unit = capacity.get('unit', 'LBS').upper()
        factor = {'KG': 1., 'LBS': .45359237}.get(unit); require(factor is not None, 'Unsupported native fuel unit')
        value = number(float(capacity.text), 'native capacity'); require(value >= 0, 'Negative native capacity')
        if value > 0: native.append(dict(index=index, capacity_kg=value*factor, position=location(tank.find('location')), unit=unit))
    require([t['index'] for t in native] == [t['index'] for t in authored], 'Authored/native positive tank IDs differ')
    for source, actual in zip(authored, native):
        # Fixed-wing writer explicitly serializes KG capacity to two decimals.
        expected = source['capacity_kg'] if rotor else float('%.2f' % source['capacity_kg'])
        close(actual['capacity_kg'], expected, 32*EPS*max(1., expected), 'Native tank capacity/unit differs from source')
        require(all(abs(a-b) <= 1e-8 for a, b in zip(source['position'], actual['position'])), 'Native tank point differs from source')
    owners = {}; component_rows = defaultdict(list)
    if damage is not None:
        require(damage['aircraft'] == name, 'Damage aircraft identity mismatch')
        require(all(abs(a-b) <= 1e-9 for a, b in zip(vector(damage['origin_design_m'], 'damage origin'), origin)),
                'Native/damage structural origin mismatch')
        for section in damage['sections']:
            for tank in section['tanks']:
                index = integer(tank['index'], 'damage tank index'); require(index not in owners, 'Duplicate damage tank owner')
                owners[index] = (section['name'], number(tank['capacity_kg'], 'damage capacity', positive=True))
        for component in damage['components']:
            if component['kind'] == 'fuel':
                i = integer(component['tank'], 'fuel component index')
                vector(component['centre'], 'fuel probe centre')
                require(all(v > 0 for v in vector(component['half'], 'fuel probe half extent')), 'Degenerate native fuel probe')
                component_rows[i].append(component)
        require(set(owners) == {t['index'] for t in authored} == set(component_rows), 'Incomplete native fuel owner/component population')
    elif not rotor:
        raise ValueError('A maintained fixed wing requires its real damage product')
    for t in authored:
        i = t['index']; t['section'] = owners[i][0] if damage is not None else 'airframe'
        if damage is not None:
            close(owners[i][1], t['capacity_kg'], 5.0001e-6, 'Damage/native authored capacity differs')
            probes = component_rows[i]; kg = math.fsum(number(p['fuel_kg'], 'fuel probe mass', positive=True) for p in probes)
            close(kg, t['capacity_kg'], 5.0001e-6*len(probes), 'Nominal fuel probe mass population differs')
            by_section = defaultdict(float)
            for p in probes: by_section[p['section']] += p['fuel_kg']
            require(t['section'] in by_section and by_section[t['section']] >= max(by_section.values())-5.0001e-6*len(probes),
                    'Native owner differs from dominant mounted fuel components')
        p = t.pop('position'); t['anchor_body_m'] = [origin[0]-p[0], p[1]-origin[1], origin[2]-p[2]]
    return dict(design=name, origin=origin, tanks=authored, rotor=rotor, native_tanks=native)


def nonoverlap(boxes):
    """Independent exact AABB test with sweep and orthogonal spatial buckets."""
    if len(boxes) < 2: return 0
    low = [min(b[0][a] for b in boxes) for a in range(3)]; high = [max(b[1][a] for b in boxes) for a in range(3)]
    axis = max(range(3), key=lambda a: (high[a]-low[a])/math.fsum(b[1][a]-b[0][a] for b in boxes))
    other = [a for a in range(3) if a != axis]; bins = max(1, math.ceil(len(boxes)**(1/3)))
    grid = defaultdict(set); owned = {}; active = set(); checks = 0
    def buckets(box):
        ranges = []
        for a in other:
            width = (high[a]-low[a])/bins
            # Broad phase expands rounding uncertainty; the final exact AABB
            # predicate still permits common boundaries and forbids interiors.
            first = max(0, min(bins-1, math.floor(math.nextafter((box[0][a]-low[a])/width, -math.inf))))
            last = max(first, min(bins-1, math.floor(math.nextafter((box[1][a]-low[a])/width, math.inf))))
            ranges.append(range(first, last+1))
        return [(i, j) for i in ranges[0] for j in ranges[1]]
    events = sorted([(b[0][axis], 1, i) for i, b in enumerate(boxes)]+[(b[1][axis], 0, i) for i, b in enumerate(boxes)])
    for _, start, i in events:
        if not start:
            for key in owned.pop(i): grid[key].remove(i)
            active.remove(i); continue
        keys = buckets(boxes[i]); candidates = set()
        for key in keys: candidates.update(grid[key])
        for j in candidates:
            checks += 1; require(checks <= 33554432, 'Independent AABB comparison budget exceeded')
            require(any(min(boxes[i][1][a], boxes[j][1][a]) <= max(boxes[i][0][a], boxes[j][0][a]) for a in range(3)),
                    'Actual liquid cell interiors overlap')
        owned[i] = keys; active.add(i)
        for key in keys: grid[key].add(i)
    return checks


def validate_native_exclusions(doc, native_damage, origin):
    """Audit actual native solid geometry, not only producer clearance flags."""
    origin = vector(list(origin), 'native material origin')
    payload = dict(origin_design_m=origin, native_damage_present=native_damage is not None, components=[])
    declared = {}
    for row in doc['exclusions']:
        if 'native_component' in row:
            index = integer(row['native_component'], 'native exclusion index')
            require(index not in declared, 'Duplicate native material exclusion'); declared[index] = row
    if native_damage is None:
        require(not declared, 'A missing native recipient cannot supply solid components')
        return payload
    require(math.sqrt(math.fsum((a-b)**2 for a, b in zip(vector(native_damage['origin_design_m'], 'native solid origin'), origin))) <= 1e-9,
            'Native material exclusion frame mismatch')
    expected_indices = set()
    for index, component in enumerate(native_damage['components']):
        if component['kind'] == 'fuel': continue
        require(component['kind'] in ('engine', 'crew', 'control') and component['shape'] in ('box', 'cylinder'),
                'Unknown native nonfuel solid kind/shape')
        centre, half = vector(component['centre'], 'native solid centre'), vector(component['half'], 'native solid half extent')
        require(all(h > 0 for h in half), 'Degenerate native solid')
        lo, hi = [x-h for x, h in zip(centre, half)], [x+h for x, h in zip(centre, half)]
        require(index in declared and declared[index]['low_m'] == lo and declared[index]['high_m'] == hi,
                'Actual native solid geometry differs from material exclusion')
        for cell in doc['cells']:
            require(any(min(cell['high_m'][a], hi[a]) <= max(cell['low_m'][a], lo[a]) for a in range(3)),
                    'Actual material overlaps actual native nonfuel solid interior')
        expected_indices.add(index); payload['components'].append([index, component])
    require(set(declared) == expected_indices, 'Native material exclusion population is incomplete')
    return payload


def validate_document(doc, expected):
    require(doc['schema'] == 'fsim.liquid-occupancy.v1' and doc['design'] == expected['design'] and
            doc['frame'] == 'body_origin_forward_right_down_m', 'Liquid schema/identity/frame mismatch')
    require(doc['fluid_policy_v1'] == POLICY, 'Missing or changed predeclared fluid engineering policy')
    require(doc['method']['version'] == 1 and all(abs(a-b) <= 1e-9 for a, b in
            zip(vector(doc['method']['origin_design_m'], 'material origin'), expected['origin'])), 'Material/native origin mismatch')
    tanks = doc['tanks']; cells = doc['cells']; require(len(cells) <= 65536, 'Native material cell bound exceeded')
    require(len(tanks) == len(expected['tanks']), 'Missing or extra native liquid tanks')
    actual_tanks = {}
    for t in tanks:
        i = integer(t['index'], 'material tank'); require(i not in actual_tanks, 'Duplicate native liquid tank'); actual_tanks[i] = t
    require(set(actual_tanks) == {t['index'] for t in expected['tanks']}, 'Wrong native material tank identities')
    for e in expected['tanks']:
        t = actual_tanks[e['index']]
        for k in ('capacity_kg', 'density_kg_m3'):
            close(number(t[k], k, positive=True), e[k], 32*EPS*max(1., e[k]), 'Wrong authored '+k)
        require(t['section'] == e['section'], 'Material tank owner differs from native owner')
        require(all(abs(a-b) <= 1e-9 for a, b in zip(vector(t['anchor_body_m'], 'material anchor'), e['anchor_body_m'])),
                'Material/native authored anchor mismatch')
    ledger = doc['ledger']
    for key in ('capacity_conserved', 'containment_certified', 'exclusions_certified', 'disjoint_certified'):
        require(ledger[key] is True, 'Producer did not certify '+key)
    if not tanks:
        require(expected.get('vessel_declaration') is None and 'inferred_vessels_v1' not in doc,
                'Liquid-inapplicable product cannot declare an inferred vessel')
        require(not cells and not ledger['tanks'] and isinstance(doc['no_liquid_reason'], str) and doc['no_liquid_reason'].strip(),
                'Liquid-inapplicable model lacks complete explicit absence')
        require(doc['uncertainty']['distribution'] == 'liquid-inapplicable', 'Wrong no-liquid uncertainty scope')
        return dict(liquid=False, tanks=0, cells=0, independent_pair_checks=0)
    require(cells and ledger['single_native_tank_owner'] is True and ledger['authoritative_boundary_certified'] is True and
            (ledger['native_owner_interval_certified'] is True or expected.get('vessel_declaration') is not None) and
            ledger['global_aabb_nonoverlap_verified'] is True,
            'Missing original-boundary/single-owner receipt')
    boxes = []; volumes = defaultdict(list); max_diagonal = 0.
    for i, c in enumerate(cells):
        require(integer(c['id'], 'stable cell id') == i, 'Missing, duplicate or reordered material cell identity')
        tank = integer(c['tank'], 'cell native tank'); require(tank in actual_tanks, 'Cell has unknown native tank')
        require(c['section'] == actual_tanks[tank]['section'], 'Cell leaves its complete native tank owner')
        lo, hi = vector(c['low_m'], 'cell low'), vector(c['high_m'], 'cell high')
        require(all(a < b for a, b in zip(lo, hi)), 'Nonpositive material box extent')
        actual = math.prod(b-a for a, b in zip(lo, hi)); given = number(c['volume_m3'], 'cell volume', positive=True)
        close(actual, given, 256*EPS*max(actual, given), 'Cell material volume differs from actual spatial box')
        volumes[tank].append(actual); boxes.append((lo, hi)); max_diagonal = max(max_diagonal, math.sqrt(math.fsum((b-a)**2 for a, b in zip(lo, hi))))
    require(set(volumes) == set(actual_tanks), 'A required liquid tank has no actual cells')
    checks = nonoverlap(boxes); entries = ledger['tanks']
    require(len(entries) == len(actual_tanks) and len({e['tank'] for e in entries}) == len(entries), 'Incomplete producer volume ledger')
    for row in entries:
        i = integer(row['tank'], 'ledger tank'); require(i in actual_tanks, 'Unknown ledger tank')
        actual = math.fsum(volumes[i]); t = actual_tanks[i]; required = t['capacity_kg']/t['density_kg_m3']
        tolerance = 512*EPS*max(1., required)*max(1, len(volumes[i]))
        close(actual, required, tolerance, 'Physical-density capacity is not conserved')
        for key, value in [('required_m3', required), ('actual_m3', actual), ('error_m3', actual-required), ('tolerance_m3', tolerance)]:
            close(number(row[key], key), value, 32*EPS*max(abs(value), sys.float_info.min), 'Changed or inconsistent producer ledger '+key)
    exclusions = doc['exclusions']
    for e in exclusions:
        require(isinstance(e['basis'], str) and e['basis'], 'Missing exclusion estimate provenance')
        lo, hi = vector(e['low_m'], 'exclusion low'), vector(e['high_m'], 'exclusion high')
        require(all(a < b for a, b in zip(lo, hi)), 'Degenerate declared equipment exclusion')
        require(all(any(min(b[1][a], hi[a]) <= max(b[0][a], lo[a]) for a in range(3)) for b in boxes),
                'Actual liquid cell intersects declared equipment exclusion')
    uncertainty = doc['uncertainty']
    require(all(isinstance(uncertainty.get(k), str) and uncertainty[k].strip() for k in ('distribution', 'boundary', 'density', 'dynamics')),
            'Missing physical model uncertainty')
    require('estimated' in uncertainty['distribution'] and 'estimated' in uncertainty['density'], 'Unadmitted equipment accuracy claim')
    close(number(uncertainty['maximum_cell_diagonal_m'], 'cell uncertainty scale', positive=True), max_diagonal,
          32*EPS*max(1., max_diagonal), 'Wrong spatial uncertainty scale')
    limits = doc['limits']
    for k, v in limits.items(): number(v, k, positive=True)
    require(len(cells) <= integer(limits['max_cells'], 'max cells') <= 65536, 'Cell resource budget exceeded')
    for field, limit in [('screened_cells', 'max_screened_cells'), ('assignment_edges', 'max_assignment_edges')]:
        require(0 < integer(ledger[field], field) <= integer(limits[limit], limit), 'Invalid producer resource count')
    section_rows = ledger['sections']; owners = {t['section'] for t in tanks}
    require(len(section_rows) == len(owners) and {r['section'] for r in section_rows} == owners, 'Incomplete producer owner receipts')
    for row in section_rows:
        for flag in ('capacity_conserved', 'containment_certified', 'exclusions_certified', 'disjoint_certified'):
            require(row[flag] is True, 'Owner did not certify '+flag)
        expected_ids = {t['index'] for t in tanks if t['section'] == row['section']}
        require(len(row['tanks']) == len(expected_ids) and {r['tank'] for r in row['tanks']} == expected_ids,
                'Incomplete owner-local tank ledger')
        require(all(r == next(g for g in entries if g['tank'] == r['tank']) for r in row['tanks']), 'Owner/global volume ledger mismatch')
    for field in ('screened_cells', 'assignment_edges'):
        require(sum(integer(r[field], 'owner '+field) for r in section_rows) == ledger[field], 'Owner/global resource count mismatch')
    validate_vessel_receipt(doc, expected)
    return dict(liquid=True, tanks=len(tanks), cells=len(cells), independent_pair_checks=checks,
                screened_cells=ledger['screened_cells'], assignment_edges=ledger['assignment_edges'],
                measured_liquid_wall_seconds=None, cost_scope='Packing counters are producer receipts; wall cost not recorded in deterministic product')


def authoritative_receipt(path):
    raw = allowed(path).read_bytes(); require(len(raw) <= 268435456 and len(raw) >= 28, 'Invalid GLB byte budget')
    require(struct.unpack_from('<III', raw) == (0x46546c67, 2, len(raw)), 'Invalid GLB header')
    n, kind = struct.unpack_from('<II', raw, 12); require(kind == 0x4e4f534a, 'Missing GLB JSON')
    doc = json.loads(raw[20:20+n]); binary_size, binary_kind = struct.unpack_from('<II', raw, 20+n)
    require(binary_kind == 0x004e4942 and 28+n+binary_size == len(raw), 'Invalid GLB binary chunk')
    binary = raw[28+n:]; extra = doc['extras']['fsim_liquid_boundary_v1']
    require(extra['schema'] == 'fsim.liquid-boundary.v1' and extra['frame'] == 'gltf_model_origin_m', 'Missing original unsplit boundary schema')
    require(extra['node'] in doc['scenes'][doc.get('scene', 0)]['nodes'], 'Authoritative accessor frame is not model root')
    def accessor(index, kind):
        integer(index, 'GLB accessor'); a = doc['accessors'][index]; require(a['type'] == kind and 'sparse' not in a, 'Wrong authority accessor')
        width = 3 if kind == 'VEC3' else 1; size = {5121: 1, 5123: 2, 5125: 4, 5126: 4}[a['componentType']]*width
        require((kind == 'VEC3') == (a['componentType'] == 5126), 'Wrong authority numeric storage')
        v = doc['bufferViews'][a['bufferView']]; count = integer(a['count'], 'authority count'); require(0 < count <= 6291456, 'Authority count budget exceeded')
        offset = v.get('byteOffset', 0)+a.get('byteOffset', 0); stride = v.get('byteStride', size)
        require(v.get('buffer', 0) == 0 and stride >= size and offset >= 0 and
                a.get('byteOffset', 0)+(count-1)*stride+size <= v['byteLength'] and offset+(count-1)*stride+size <= len(binary),
                'Authority accessor exceeds source bytes')
        data = b''.join(binary[offset+i*stride:offset+i*stride+size] for i in range(count))
        return data, count
    p, count = accessor(extra['positions'], 'VEC3'); t, indices = accessor(extra['indices'], 'SCALAR')
    require(indices % 3 == 0, 'Incomplete original triangle')
    return dict(positions=extra['positions'], indices=extra['indices'], node=extra['node'],
                positions_sha256=hashlib.sha256(p).hexdigest(), indices_sha256=hashlib.sha256(t).hexdigest(),
                vertices=count, triangles=indices//3, status='Error.NoError')


def dependency_path():
    spec = importlib.util.find_spec('manifold3d'); require(spec is not None and spec.origin, 'Cannot bind producer Manifold dependency')
    return allowed(Path(spec.origin))


def admit(coverage, coverage_sha256, root=ROOT):
    root = allowed(root).resolve(); require(re.fullmatch('[0-9a-f]{64}', coverage_sha256) is not None, 'Coverage SHA256 required')
    require(coverage.get('inventory_passed') is True, 'Authored population inventory did not pass')
    population_rows = population(coverage, root); bindings = {}; results = []
    def add(path, role):
        row = bind(path, role); previous = bindings.get(row['path'])
        if previous: require(previous['sha256'] == row['sha256'], 'Inputs changed within admission epoch')
        else: bindings[row['path']] = row
        return row['sha256']
    for source in sorted(SOURCE_PATHS): add(root/source, 'method source')
    dependency = dependency_path(); dependency_sha = add(dependency, 'resolved producer dependency binary; not native loaded-module evidence')
    for name, row in sorted(population_rows.items()):
        folder = root/'aircraft'/name; product = folder/(name+'.liquid-v1.json'); glb = folder/(name+'.glb')
        xml_path = folder/(name+'.xml'); damage_path = folder/(name+'.damage.json')
        for path, role in [(row['path'], 'authored design'), (product, 'actual liquid product'), (glb, 'actual GLB'), (xml_path, 'actual native XML')]: add(path, role)
        damage = read(damage_path) if damage_path.is_file() else None
        if damage is not None: add(damage_path, 'actual native damage')
        expected = expected_model(name, row['spec'], ET.fromstring(xml_path.read_bytes()), damage)
        vessel_source = folder/(name+'.liquid-geometry-v1.json')
        if vessel_source.is_symlink(): raise ValueError('Inferred vessel source cannot be a symlink')
        if vessel_source.exists():
            require(vessel_source.is_file() and vessel_source.stat().st_size <= 16384, 'Inferred vessel source bound exceeded')
            add(vessel_source, 'authored inferred vessel declaration')
            expected['vessel_declaration'] = validate_vessel_declaration(read(vessel_source), name)
            expected['vessel_root_cut'] = vessel_root_cut(row['spec'])
        doc = read(product); checked = validate_document(doc, expected)
        declaration = dict(file=name+'.liquid-v1.json', schema='fsim.liquid-occupancy.v1', required=True)
        if damage is not None:
            require(damage.get('liquid_geometry_v1') == declaration if checked['liquid'] else 'liquid_geometry_v1' not in damage,
                    'Real damage product has missing/stale required liquid declaration')
        required = dict(METHODS); required.pop('exclusion_method')
        required['normal_model_stage'] = 'tools/models/hangar/hangar/'+('rotorcraft/__init__.py' if row['spec']['aircraft'].get('kind') in ('helicopter', 'multirotor') else 'pipeline.py')
        source_expectations = {role: dict(path=Path(path).name, sha256=sha(root/path)) for role, path in required.items()}
        source_expectations.update(design=dict(path=row['path'].name, sha256=sha(row['path'])),
            effective_design_spec=dict(path='canonical-design-spec', sha256=hashlib.sha256(canonical(row['spec'])).hexdigest()))
        if checked['liquid']:
            execution_resources=root/'tools/models/hangar/hangar/resources.py'
            spawn_runtime=allowed(Path(mp.__file__).parent/'popen_spawn_win32.py')
            source_expectations.update(
                execution_resources=dict(path=execution_resources.name,sha256=add(execution_resources,'actual execution resources')),
                spawn_runtime=dict(path=spawn_runtime.name,sha256=add(spawn_runtime,'actual CPython Windows spawn source')))
            if expected.get('vessel_declaration') is not None:
                from hangar.shape import meshkit
                require(Path(meshkit.__file__).resolve() == (root/'tools/models/hangar/hangar/shape/meshkit.py').resolve(),
                        'Foreign inferred vessel mesher namespace')
                library = meshkit.library(); require(library is not None, 'Inferred vessel mesher backend unavailable')
                method_roles = dict(inferred_vessel_declaration=vessel_source,
                    inferred_vessel_method=root/'tools/models/hangar/hangar/liquid_vessels.py',
                    inferred_vessel_geometry=root/'tools/models/hangar/hangar/shape/airframe.py',
                    inferred_vessel_mesher=root/'tools/models/hangar/hangar/shape/meshkit.py',
                    inferred_vessel_section_method=root/'tools/models/hangar/hangar/geometry/surface.py',
                    inferred_vessel_airfoil_method=root/'tools/models/hangar/hangar/geometry/airfoil.py',
                    inferred_vessel_airfoil_coordinates=folder/'lrn1015.dat',
                    inferred_vessel_mesher_binary=allowed(Path(library._name)))
                wings = [s for s in row['spec']['surface'] if s.get('kind') == 'wing']
                require(wings[0].get('airfoil') == 'lrn1015.dat' and
                        all(r.get('airfoil', 'lrn1015.dat') == 'lrn1015.dat' for r in wings[0]['sections']),
                        'Inferred vessel original airfoil input changed')
                for role, path in method_roles.items():
                    source_expectations[role] = dict(path=path.name, sha256=add(path, role))
            native_payload = validate_native_exclusions(doc, damage, expected['origin'])
            owners = [(t['index'], t['section']) for t in doc['tanks']]
            owner_sha = hashlib.sha256(json.dumps(owners, separators=(',', ':')).encode()).hexdigest()
            source_expectations.update(glb=dict(path=glb.name, sha256=sha(glb)),
                manifold_binary=dict(path=dependency.name, sha256=dependency_sha),
                exclusion_method=dict(path=Path(METHODS['exclusion_method']).name, sha256=sha(root/METHODS['exclusion_method'])),
                native_partition_method=dict(path='breakup.py', sha256=sha(root/'tools/models/hangar/hangar/shape/breakup.py')),
                native_material_exclusion=dict(path='canonical-native-material-exclusion',
                    sha256=hashlib.sha256(canonical(native_payload)).hexdigest()),
                native_tank_ownership=dict(path='canonical-ownership', sha256=owner_sha))
            receipt = doc['boundary_receipt']
            require(receipt['authority'] == 'original_unsplit_airframe_accessors_v1' and
                    receipt['authoritative_accessors'] == authoritative_receipt(glb), 'Original-boundary byte receipt differs from actual GLB')
            number(receipt['volume_m3'], 'producer original hull volume', positive=True)
        verify_sources(doc['sources'], source_expectations)
        results.append(dict(design=name, category=row['category'], **checked))
    require(sum(r['liquid'] for r in results) == 32 and sum(r['tanks'] for r in results) == 149 and
            sum(not r['liquid'] for r in results) == 3, 'Actual derived all35/32-liquid/149-native-tank population differs')
    for b in bindings.values(): require(sha(b['path']) == b['sha256'], 'Inputs/products changed during admission')
    return dict(schema='fsim.sim-m03.liquid-population-admission.v1', passed=True,
                coverage_sha256=coverage_sha256, population=len(results), liquid_models=32, native_tanks=149,
                no_liquid_models=[r['design'] for r in results if not r['liquid']], models=results,
                artifacts=sorted(bindings.values(), key=lambda b: b['path']),
                scope='Independent source/native identity, units, ownership, origin, numerical material volume/nonoverlap and declared AABB exclusions; hash-bound producer Manifold receipts',
                independent_geometric_containment=False, equipment_calibration_admitted=False,
                limits='No occupancy regeneration, independent Manifold rerun, runtime contents/kernel, installation or equipment-accuracy acceptance')


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--coverage', type=Path, required=True)
    parser.add_argument('--root', type=Path, default=ROOT); parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    try: result = admit(read(args.coverage), sha(args.coverage), root=args.root); code = 0
    except (ValueError, KeyError, OSError, TypeError, IndexError, ET.ParseError) as refused:
        result = dict(schema='fsim.sim-m03.liquid-population-admission.v1', passed=False, error=str(refused)); code = 2
    allowed(args.output).parent.mkdir(parents=True, exist_ok=True)
    allowed(args.output).write_text(json.dumps(result, sort_keys=True, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    return code


if __name__ == '__main__': raise SystemExit(main())
