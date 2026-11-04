"""Admit actual all-model native liquid capture/transit/reset receipts.

No simulation or geometry production runs here. The static population ledger
and aggregate green suites cannot replace model/tank/state runtime rows.
"""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from acceptance import allowed, reconcile_population, verify_artifact
from external_tanks import git_epoch
from liquid_population import population, expected_model

ROOT = Path(__file__).resolve().parents[3]
PRESETS = {'ucrt64-headless', 'ucrt64-release'}
SCHEMA = 'fsim.liquid-native-population.v1'
MODES = {(0., False), (.5, False), (.5, True), (1., True)}
SOURCE_PATHS = {
    'tests/simulation/test_liquid_population_native.cpp', 'tests/CMakeLists.txt',
    'tools/validation/weapon_quality/liquid_native_population.py',
    'tools/validation/weapon_quality/test_liquid_native_population.py',
    'tools/validation/weapon_quality/liquid_population.py',
    'tools/validation/weapon_quality/external_tanks.py',
    'tools/validation/weapon_quality/acceptance.py',
    'tools/validation/weapon_quality/campaign.py',
    'tools/validation/weapon_quality/report.py',
    'docs/research/sim-m03/native-liquid-population.md',
    'src/simulation/runtime/world/World.h', 'src/simulation/runtime/world/World.cpp',
    'src/simulation/runtime/world/WorldDamage.cpp',
    'src/simulation/runtime/world/WorldLiquidMaterials.h',
    'src/simulation/runtime/world/WorldLiquidMaterials.cpp',
    'src/simulation/mission_systems/damage/Damage.h',
    'src/simulation/mission_systems/damage/Damage.cpp',
    'src/simulation/mission_systems/damage/LiquidSceneV1.h',
    'src/simulation/mission_systems/damage/FluidTransitV1.h',
    'src/simulation/flight/JsbsimModel.cpp',
    'third_party/jsbsim/src/models/propulsion/FGTank.cpp',
    'third_party/jsbsim/src/models/propulsion/FGTank.h',
    'third_party/jsbsim/src/models/FGPropulsion.cpp',
}


def require(value, message):
    if not value:
        raise ValueError(message)


def sha(path):
    import hashlib
    return hashlib.sha256(allowed(path).read_bytes()).hexdigest()


def bind(path):
    return dict(path=str(allowed(path).resolve()), sha256=sha(path))


def bound(row, root=ROOT):
    path = allowed(row['path'])
    path = path if path.is_absolute() else root/path
    verify_artifact(path, row['sha256'])
    return path


def number(value, label):
    require(type(value) in (int, float) and math.isfinite(value), 'Finite numeric '+label+' required')
    return value


def integer(value, label):
    require(type(value) in (int, float) and number(value, label) >= 0 and int(value) == value,
            'Nonnegative integer '+label+' required')
    return int(value)


def identity(value):
    require(type(value) is str and value.isascii() and value.isdecimal() and value[0] != '0'
            and 0 < int(value) <= 2**64-1, 'Exact positive decimal identity required')
    return int(value)


def read(path):
    def unique(pairs):
        out = {}
        for key, value in pairs:
            require(key not in out, 'Duplicate JSON key')
            out[key] = value
        return out
    return json.loads(allowed(path).read_text(encoding='utf-8'), object_pairs_hook=unique,
                      parse_constant=lambda value: (_ for _ in ()).throw(ValueError('Nonfinite JSON '+value)))


def models(coverage, root=ROOT):
    result = {}
    for name, source in population(coverage, root).items():
        folder = root/'aircraft'/name
        damage_path = folder/(name+'.damage.json')
        damage = read(damage_path) if damage_path.is_file() else None
        expected = expected_model(name, source['spec'], ET.parse(folder/(name+'.xml')).getroot(), damage)
        doc = read(folder/(name+'.liquid-v1.json'))
        require(doc['design'] == name, 'Foreign material product')
        result[name] = dict(tanks={t['index']: t for t in expected['tanks']},
                            cells={c['id']+1: c for c in doc['cells']}, unsectioned=damage is None,
                            source_paths={source['path'].relative_to(root).as_posix(),
                                          'aircraft/'+name+'/'+name+'.xml',
                                          'aircraft/'+name+'/'+name+'.liquid-v1.json'} |
                                         ({'aircraft/'+name+'/'+name+'.damage.json'} if damage is not None else set()))
    require(len(result) == 35 and sum(bool(m['tanks']) for m in result.values()) == 32
            and sum(len(m['tanks']) for m in result.values()) == 149, 'Changed native liquid population')
    return result


def validate_model(name, rows, expected):
    require(rows and all(r.get('design') == name for r in rows), 'Foreign/missing model row')
    require(rows[-1].get('kind') == 'complete', 'Incomplete native model run')
    body = rows[:-1]
    tanks = expected['tanks']
    if not tanks:
        require(len(body) == 1 and body[0].get('kind') == 'no-liquid', 'Unexpected no-liquid records')
        require(body[0].get('loaded') is True and body[0].get('reset') is True
                and body[0].get('native_tanks') == 0, 'Absent native no-liquid/reset evidence')
    else:
        require(all(r.get('kind') in ('transit', 'reset') for r in body), 'Foreign native record')
        transit = [dict(r, fraction=number(r['fraction'], 'fraction')) for r in body if r['kind'] == 'transit']
        for r in transit:
            require(type(r['wet']) is bool and type(r['tank']) is int, 'Typed native tank/state identity required')
        indexed = reconcile_population({(i, f, w) for i in tanks for f, w in MODES}, transit,
                                       ('tank', 'fraction', 'wet'))
        for (tank, fraction, wet), row in indexed.items():
            original = tanks[tank]
            for key in ('source', 'source_generation', 'victim', 'victim_generation', 'cell'):
                identity(row[key])
            cell = expected['cells'].get(int(row['cell']))
            require(cell is not None and cell['tank'] == tank, 'Selected cell does not belong to actual native tank')
            require(row['source_ordinal'] == '1' and row['advanced'] is True and row['inside_selected_cell'] is True
                    and row['freeze_oracle'] is True and row['unsectioned'] is expected['unsectioned'], 'Missing actual source/capture/transit evidence')
            require(number(row['dt_s'], 'duration') == .0001, 'Changed diagnostic duration')
            require(integer(row['captured_tanks'], 'captured tanks') == len(tanks)
                    and integer(row['captured_cells'], 'captured cells') == len(expected['cells']), 'Incomplete actual freeze')
            capacity = number(row['capacity_kg'], 'capacity')
            require(abs(capacity-original['capacity_kg']) <= 5.0001e-6, 'Changed native tank capacity')
            require(number(row['density_kg_m3'], 'density') == original['density_kg_m3'], 'Changed material density')
            require(abs(number(row['actual_kg'], 'native contents')-capacity*fraction) <= 1e-8,
                    'Native current contents differ from prescribed state')
            energy = number(row['energy_j'], 'fluid work')
            require(energy > 0 if wet else energy == 0, 'Missing wet work or false dry work')
            require(integer(row['wall_contacts'], 'wall contacts') == 0, 'Inside-cell transit fabricated wall contact')
            require(number(row['reaction_budget_ns'], 'reaction budget') == 2e-5 and
                    0 <= number(row['reaction_error_ns'], 'reaction error') <= 2e-5, 'Native reaction mismatch')
        resets = reconcile_population(set(tanks), [r for r in body if r['kind'] == 'reset'], ('tank',))
        for row in resets.values():
            require(type(row['tank']) is int and row['stale_capture_refused'] is True and row['fresh_freeze'] is True,
                    'Missing actual reset/fresh freeze')
            require(number(row['kg_budget'], 'reset budget') == 1e-9 and
                    abs(number(row['actual_kg'], 'reset kg')-number(row['fresh_kg'], 'fresh kg')) <= 1e-9,
                    'Native reset does not match fresh')
            require(integer(row['captured_tanks'], 'reset tanks') == len(tanks) and
                    integer(row['captured_cells'], 'reset cells') == len(expected['cells']), 'Incomplete reset population')
    require(integer(rows[-1]['tanks'], 'complete tanks') == len(tanks)
            and integer(rows[-1]['transit_rows'], 'complete transit rows') == len(tanks)*4
            and integer(rows[-1]['reset_rows'], 'complete reset rows') == len(tanks), 'Wrong completion denominator')
    return dict(design=name, tanks=len(tanks), transit_rows=len(tanks)*4, reset_rows=len(tanks), passed=True)


def source_paths(coverage, root=ROOT):
    return SOURCE_PATHS | set().union(*(m['source_paths'] for m in models(coverage, root).values()))


def freeze_sources(coverage, coverage_hash, root=ROOT, git_source_root=None):
    return dict(schema='fsim.liquid-native.source-epoch.v1', main_head=git_epoch(git_source_root or root),
                coverage_sha256=coverage_hash, sources=[dict(path=p, sha256=sha(root/p))
                for p in sorted(source_paths(coverage, root))], presets=[])


def capture_preset(coverage, preset, report, root=ROOT):
    require(preset in PRESETS, 'Unknown native preset')
    expected = models(coverage, root)
    cases = ET.parse(report).getroot().findall('.//testcase')
    names = {'liquid_population_native.'+name for name in expected} | {'liquid_population_methods'}
    selected = [c for c in cases if c.get('name') in names]
    reconcile_population(names, [dict(name=c.get('name')) for c in selected], ('name',))
    require(all(c.find(tag) is None for c in selected for tag in ('failure', 'error', 'skipped')), 'Native liquid case failed/skipped')
    binary = root/'build'/preset/'tests/fsim_liquid_population_native_test.exe'
    result = dict(preset=preset, binary=bind(binary), test_report=bind(report), models=[])
    for name in sorted(expected):
        path = root/'build'/preset/'validation/liquid_native_population'/(name+'.jsonl')
        # Use the same strict JSON reader for each bounded line.
        require(path.stat().st_size <= 16*1024*1024, 'Native receipt exceeds byte budget')
        lines = path.read_text(encoding='utf-8').splitlines()
        require(0 < len(lines) <= 1024, 'Native receipt exceeds row budget')
        rows = [json.loads(line, parse_constant=lambda v: (_ for _ in ()).throw(ValueError(v))) for line in lines]
        header = rows[0]
        require(header['kind'] == 'runtime' and header['schema'] == SCHEMA and header['design'] == name, 'Foreign runtime receipt')
        require(bound(header['binary'], root).resolve() == binary.resolve(), 'Foreign fixture binary')
        runtime = bound(header['jsbsim'], root)
        require(runtime.resolve() == (root/'build'/preset/'bin/libJSBSim.dll').resolve(), 'Foreign actual JSBSim module')
        folder = root/'aircraft'/name
        for suffix in ('.xml', '.liquid-v1.json', '.damage.json'):
            product = folder/(name+suffix)
            require((suffix in header) == product.is_file(), 'Missing/extra native product binding')
            if product.is_file():
                require(bound(header[suffix], root).resolve() == product.resolve(), 'Foreign model product')
        validate_model(name, rows[1:], expected[name])
        result['models'].append(dict(design=name, raw=bind(path), runtime=bind(runtime)))
    return result


def admit(coverage, coverage_hash, epoch, root=ROOT, git_source_root=None):
    require(epoch['schema'] == 'fsim.liquid-native.source-epoch.v1' and epoch['coverage_sha256'] == coverage_hash,
            'Foreign native liquid epoch')
    require(epoch['main_head'] == git_epoch(git_source_root or root), 'Changed main source epoch')
    sources = reconcile_population(source_paths(coverage, root), epoch['sources'], ('path',))
    for row in sources.values():
        bound(row, root)
    presets = reconcile_population(PRESETS, epoch['presets'], ('preset',))
    for preset, run in presets.items():
        require(capture_preset(coverage, preset, bound(run['test_report'], root), root) == run, 'Stale/mixed native liquid capture')
    return dict(schema=SCHEMA, passed=True, main_head=epoch['main_head'], coverage_sha256=coverage_hash,
                population=35, liquid_models=32, native_tanks=149, transit_rows_per_preset=596,
                reset_rows_per_preset=149, sources=epoch['sources'], presets=epoch['presets'],
                equipment_calibration_admitted=False,
                scope='Actual stationary native material capture/transit/reset; approved rotor flight deferral unchanged; no lethality/flight/OEM validation')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('coverage', 'source-epoch', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--root', type=Path, default=ROOT)
    parser.add_argument('--git-source-root', type=Path)
    args = parser.parse_args()
    require(not args.output.exists(), 'Fresh native population admission required')
    result = admit(read(args.coverage), sha(args.coverage), read(args.source_epoch), args.root, args.git_source_root)
    result['source_epoch'] = bind(args.source_epoch)
    allowed(args.output).parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')


if __name__ == '__main__':
    main()
