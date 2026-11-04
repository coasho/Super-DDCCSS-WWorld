"""Current-byte admission for the mandatory 24 AAM x 9 component matrix.

Availability, an equipment selector, shared numerical code and calibration are
different claims. No cell passes until all of its actual evidence gates pass.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import tomllib
import xml.etree.ElementTree as ET

from acceptance import allowed, reconcile_population, verify_artifact

ROOT = Path(__file__).resolve().parents[3]
COMPONENTS = ('seeker', 'guidance', 'autopilot', 'allocator', 'actuator',
              'fuze', 'warhead', 'propulsion', 'aerodynamics')
GATES = ('theory', 'method', 'native', 'public_installed', 'visual')
SCHEMAS = dict(theory='fsim.sim-m03.component-reference.v1',
               method='fsim.sim-m03.component-reference.v1',
               native='fsim.sim-m03.component-native.v1',
               public_installed='fsim.sim-m03.component-public-installed.v1',
               visual='fsim.sim-m03.component-visual.v1')
CATALOGUE = 'tools/models/hangar/hangar/data/equipment_models.json'
REGISTRATION = 'src/simulation/mission_systems/components/registration/'


def require(value, message):
    if not value:
        raise ValueError(message)


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'),
                      ensure_ascii=False, allow_nan=False).encode('utf-8')


def digest(value):
    return hashlib.sha256(canonical(value)).hexdigest()


def sha(path):
    return hashlib.sha256(allowed(path).read_bytes()).hexdigest()


def read(path):
    def unique(pairs):
        result = {}
        for key, value in pairs:
            require(key not in result, 'Duplicate JSON field: '+key)
            result[key] = value
        return result
    return json.loads(allowed(path).read_text(encoding='utf-8'), object_pairs_hook=unique,
                      parse_constant=lambda value: (_ for _ in ()).throw(ValueError('Nonfinite JSON: '+value)))


def binding(path, root=ROOT):
    path = allowed(path).resolve()
    require(path.is_relative_to(root.resolve()) or path.is_relative_to(Path('D:/ENV').resolve()),
            'Artifact is outside the admitted source/DENV evidence roots')
    return dict(path=path.relative_to(root.resolve()).as_posix() if path.is_relative_to(root.resolve()) else str(path), sha256=sha(path))


def bound(row, root=ROOT):
    require(type(row) is dict and set(row) == {'path', 'sha256'}, 'Exact path/hash binding required')
    path = allowed(Path(row['path']))
    path = path if path.is_absolute() else root/path
    path = allowed(path).resolve()
    require(path.is_relative_to(root.resolve()) or path.is_relative_to(Path('D:/ENV').resolve()), 'Foreign evidence root')
    require(type(row['sha256']) is str and re.fullmatch('[0-9a-f]{64}', row['sha256']), 'SHA256 required')
    verify_artifact(path, row['sha256'])
    return path


def parameter_payload(card, component):
    descriptor = card['component_models']['components'][component]
    refs = descriptor.get('parameter_refs', {})
    require(type(refs) is dict and all(type(k) is str and type(v) is str for k, v in refs.items()),
            'Named parameter references required')
    parameters = {role: card['parameters'][ref] for role, ref in refs.items()}
    for role, parameter in parameters.items():
        require(type(parameter) is dict and 'value' in parameter and parameter.get('unit')
                and parameter.get('basis'), 'Incomplete parameter '+role)
        canonical(parameter)  # Refuse nonfinite values without rewriting basis/uncertainty.
    return dict(descriptor=descriptor, parameters=parameters)


def parameter_hash(card, component):
    return digest(parameter_payload(card, component))


def design_values(spec, component):
    if component in ('allocator', 'actuator'):
        return dict(surfaces=[dict(name=s.get('name'), controls=s['controls'])
                             for s in spec.get('surface', []) if s.get('controls')],
                    integration=spec.get('integration'), control=spec.get('control'))
    keys = (('body', 'surface', 'reference', 'analysis', 'dimensions', 'integration', 'overrides')
            if component == 'aerodynamics' else ('motor', 'engine', 'mass', 'reference')
            if component == 'propulsion' else ('mass', 'dimensions', 'reference'))
    return {key: spec[key] for key in keys if key in spec}


def xml_values(path, component):
    tree = ET.parse(path).getroot()
    query = {'allocator': './/flight_control', 'actuator': './/actuator',
             'propulsion': './propulsion', 'aerodynamics': './aerodynamics'}.get(component)
    if query is None:
        return []
    def record(node):
        text = (node.text or '').strip()
        encoded = text.encode('utf-8')
        # Preserve scalar/configuration values verbatim. The unchanged primary
        # XML already owns large generated grids; duplicating them in every
        # closure/delta projection obscures review and is not new evidence.
        return dict(tag=node.tag, attributes=dict(node.attrib), text=text if len(encoded) <= 1024 else None,
                    external_text=(dict(sha256=hashlib.sha256(encoded).hexdigest(), bytes=len(encoded),
                                        lines=len(text.splitlines()), authority='hash-bound primary XML, same node')
                                   if len(encoded) > 1024 else None),
                    children=[record(c) for c in node])
    return [record(node) for node in tree.findall(query)]


def native_paths(component, root=ROOT):
    """Actual registration/include inputs, not a guessed OEM component name."""
    path = REGISTRATION+component.title()+'ProvidersV1.cpp'
    require((root/path).is_file(), 'Missing native registration: '+path)
    paths = {path, 'src/contracts/equipment/EquipmentModels.h',
             REGISTRATION+'EquipmentModels.cpp',
             'src/simulation/mission_systems/components/support/EquipmentSelectionV1.h'}
    text = (root/path).read_text(encoding='utf-8')
    for include in re.findall(r'^#include "([^"]+)"', text, re.M):
        candidate = 'src/'+include
        if (root/candidate).is_file():
            paths.add(candidate)
            cpp = str(Path(candidate).with_suffix('.cpp')).replace('\\', '/')
            if (root/cpp).is_file():
                paths.add(cpp)
    # Native plant functions are implemented by the actual JSBSim adapter and
    # its source generator; empty parameter_refs does not establish no inputs.
    if component in ('allocator', 'actuator', 'propulsion', 'aerodynamics'):
        paths |= {'src/simulation/flight/JsbsimModel.cpp', 'tools/models/hangar/hangar/jsbsim.py'}
    if component == 'fuze':
        paths |= {'src/simulation/mission_systems/weapon/FuzeAdapter.cpp'}
    if component == 'warhead':
        paths |= {'src/simulation/mission_systems/damage/WarheadAdapter.cpp',
                  'src/simulation/mission_systems/damage/Damage.cpp'}
    require(all((root/p).is_file() for p in paths), 'Missing native implementation dependency')
    return paths


def head_bytes(path, root=ROOT, baseline='HEAD', git_source_root=None):
    require(baseline == 'HEAD' or re.fullmatch('[0-9a-f]{40}', baseline), 'Immutable baseline hash required')
    result = subprocess.run(['git', 'show', baseline+':'+path], cwd=allowed(git_source_root or root), capture_output=True, timeout=15)
    return result.stdout if result.returncode == 0 else None


def derive_cells(coverage, root=ROOT):
    declared = [r for r in coverage['stores'] if r['class'] == 'aam']
    require(len(declared) == 24 and len({r['card'] for r in declared}) == 24, 'Exact24 AAM inventory required')
    catalogue = read(root/CATALOGUE)
    entries = {r['store_id']: r for r in catalogue['models']}
    require(len(entries) == len(catalogue['models']), 'Duplicate equipment catalogue identity')
    actual = {}
    # Explicit one-directory inventory; excluded directories are never entered.
    for path in sorted((root/'assets/stores').glob('*.json')):
        card = read(path)
        if card.get('class') == 'aam':
            require(card['id'] == path.stem and card['id'] not in actual, 'Foreign/duplicate AAM identity')
            actual[card['id']] = (card, path)
    require(set(actual) == {r['card'] for r in declared}, 'Missing/extra maintained AAM')
    coverage_rows = {r['card']: r for r in declared}
    cells = []
    native = {component: native_paths(component, root) for component in COMPONENTS}
    for model, (card, path) in sorted(actual.items()):
        coverage_row = coverage_rows[model]
        require(path.relative_to(root).as_posix() == coverage_row['card_path']
                and sha(path) == coverage_row['card_sha256'], 'Stale AAM coverage/card epoch')
        equipment = entries[model]
        require(equipment['native_model'] == card['model'] == 'jsbsim:'+coverage_row['model'], 'Native model identity differs')
        require(set(card['component_models']['components']) == set(COMPONENTS), 'Missing/extra AAM component')
        native_name = coverage_row['model']
        design_path = 'aircraft/'+native_name+'/'+native_name+'.toml'
        spec = tomllib.loads((root/design_path).read_text(encoding='utf-8'))
        primary_xml = 'aircraft/stores/'+native_name+'/'+native_name+'.xml'
        require((root/primary_xml).is_file(), 'Actual generated native model is absent')
        for component in COMPONENTS:
            descriptor = card['component_models']['components'][component]
            identity = equipment['components'][component]
            require(descriptor['availability'] == identity['availability'] == 'available', 'A required component is unavailable')
            require(descriptor['implementation_id'] == identity['selector'] and
                    identity['selector'].startswith('equipment.'), 'Aliased/foreign component selection')
            inputs = set(native[component]) | {CATALOGUE, path.relative_to(root).as_posix(), design_path, primary_xml}
            design = descriptor.get('bindings', {}).get('design')
            if design:
                require(type(design) is str and design == design_path and (root/allowed(Path(design))).is_file(), 'Missing/foreign component design input')
                inputs.add(design)
            payload = parameter_payload(card, component)
            cells.append(dict(model=model, native_model=card['model'], component=component,
                implementation_id=descriptor['implementation_id'], mechanism_id=identity['mechanism_id'],
                hardware_model_name=identity.get('hardware_model_name'), card_sha256=sha(path),
                card_path=path.relative_to(root).as_posix(), parameter_sha256=digest(payload),
                input_basis=descriptor['basis'], validity=descriptor.get('validity'),
                parameters=payload['parameters'], parameter_refs=descriptor.get('parameter_refs', {}),
                bindings=descriptor.get('bindings', {}),
                design_input=dict(path=design_path, sha256=sha(root/design_path), values=design_values(spec, component)),
                native_input=dict(path=primary_xml, sha256=sha(root/primary_xml), values=xml_values(root/primary_xml, component),
                    mutable_until_forced_production=True),
                sources=[binding(root/p, root) for p in sorted(inputs)]))
    require(len(cells) == 216, 'Exact216 component cells required')
    return cells


def deltas(cells, root=ROOT, baseline='HEAD', git_source_root=None):
    """Actual source/card differences, never fitted-equipment improvement claims."""
    git_root = allowed(git_source_root or root)
    if baseline == 'HEAD':
        baseline = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=git_root, text=True, timeout=15).strip()
    require(re.fullmatch('[0-9a-f]{40}', baseline), 'Pinned source baseline required')
    subprocess.run(['git', 'cat-file', '-e', baseline+'^{commit}'], cwd=git_root, check=True,
                   capture_output=True, timeout=15)
    paths = sorted({r['path'] for c in cells for r in c['sources']})
    source_deltas = []
    for path in paths:
        before = head_bytes(path, root, baseline, git_root)
        now = (root/path).read_bytes()
        old = hashlib.sha256(before).hexdigest() if before is not None else None
        # Git stores normalized text; distinguish an EOL-only checkout from a
        # substantive change, while retaining BOTH literal-byte digests.
        normalized_before = before.replace(b'\r\n', b'\n') if before is not None else None
        normalized_now = now.replace(b'\r\n', b'\n')
        source_deltas.append(dict(path=path, baseline_sha256=old, current_sha256=hashlib.sha256(now).hexdigest(),
            baseline_present=before is not None, substantive_change=normalized_before != normalized_now))
    rows = []
    cache = {}
    for cell in cells:
        path = cell['card_path']
        if path not in cache:
            raw = head_bytes(path, root, baseline, git_root)
            cache[path] = json.loads(raw) if raw is not None else None
        before = cache[path]
        prior = None
        if before is not None and cell['component'] in before.get('component_models', {}).get('components', {}):
            prior = parameter_payload(before, cell['component'])
        current = parameter_payload(read(root/path), cell['component'])
        names = set(current['parameters']) | (set(prior['parameters']) if prior else set())
        parameters = [dict(role=name, baseline=prior['parameters'].get(name) if prior else None,
                           current=current['parameters'].get(name),
                           changed=(prior['parameters'].get(name) if prior else None) != current['parameters'].get(name))
                      for name in sorted(names)]
        design_path = cell['design_input']['path']
        if design_path not in cache:
            raw = head_bytes(design_path, root, baseline, git_root)
            cache[design_path] = tomllib.loads(raw.decode('utf-8')) if raw is not None else None
        prior_design = design_values(cache[design_path], cell['component']) if cache[design_path] is not None else None
        rows.append(dict(model=cell['model'], component=cell['component'], baseline_descriptor=prior['descriptor'] if prior else None,
                         current_descriptor=current['descriptor'], parameters=parameters,
                         descriptor_changed=(prior['descriptor'] if prior else None) != current['descriptor'],
                         design_delta=dict(path=design_path, baseline=prior_design, current=cell['design_input']['values'],
                                           changed=prior_design != cell['design_input']['values']),
                         native_input=cell['native_input'],
                         equipment_accuracy_improvement_claim=False))
    return dict(schema='fsim.sim-m03.parameter-deltas.v1', baseline=baseline,
                scope='Exact maintained descriptor/parameter/source changes; not calibration or outcome fitting',
                cells=rows, source_deltas=source_deltas)


def finite(value, label):
    require(type(value) in (int, float) and math.isfinite(value), 'Finite numeric '+label+' required')
    return value


def execution(row, key):
    value = row[key]
    require(type(value.get('case')) is str and value['case'].strip(), 'Identifiable executed case required')
    for field in ('tests', 'assertions'):
        require(type(value.get(field)) is int and value[field] > 0, 'Nonzero discovered '+field+' required')
    require(not value.get('skipped', 0) and not value.get('expected_failures', 0)
            and not value.get('failures', 0) and not value.get('errors', 0), 'Failed/skipped evidence cannot pass')
    return value


def validate_gate(cell, gate, ref, root=ROOT, identities=None):
    require(gate in GATES, 'Unknown component gate')
    require(type(ref) is dict and set(ref) == {'artifact', 'row'}, 'Artifact and model-row selector required')
    artifact_path = bound(ref['artifact'], root)
    artifact = read(artifact_path)
    require(artifact['schema'] == SCHEMAS[gate] and artifact.get('passed') is True, 'Wrong/failed component proof schema')
    pairs = [(r['model'], r['component']) for r in artifact['models']]
    require(len(set(pairs)) == len(pairs), 'Duplicate component proof row')
    if identities is not None:
        require(set(pairs).issubset(identities), 'Unknown model/component proof row')
    require(type(ref['row']) is int and 0 <= ref['row'] < len(artifact['models']), 'Exact model-row index required')
    row = artifact['models'][ref['row']]
    for key in ('model', 'component', 'implementation_id', 'mechanism_id', 'card_sha256', 'parameter_sha256'):
        require(row.get(key) == cell[key], 'Missing/stale/foreign '+key+' evidence')
    require(row.get('passed') is True, 'Component evidence did not pass')
    bindings = artifact.get('sources', [])
    require(bindings and len({r['path'] for r in bindings}) == len(bindings), 'Missing/duplicate proof sources')
    current = {r['path']: r['sha256'] for r in cell['sources']}
    proof = {r['path']: r['sha256'] for r in bindings}
    require(all(proof.get(path) == value for path, value in current.items()), 'Incomplete/stale component input epoch')
    for item in bindings:
        bound(item, root)
    raw = artifact.get('raw', [])
    require(type(raw) is list and raw, 'Actual retained proof output required')
    for item in raw:
        bound(item, root)
    if gate == 'theory':
        theory = row['theory']
        require(all(type(theory.get(k)) is str and theory[k].strip() for k in ('equations', 'limits', 'delta')),
                'Mechanism equations, applicability and theory delta required')
        citations = theory.get('primary_sources')
        require(type(citations) is list and citations, 'Primary method sources required')
        require(all(type(r) is dict and all(type(r.get(k)) is str and r[k].strip() for k in ('title', 'url', 'locator', 'scope'))
                    and r['url'].startswith('https://') for r in citations), 'Exact primary-source attribution required')
    elif gate == 'method':
        method = execution(row, 'method')
        residuals = method.get('residuals')
        require(type(residuals) is list and residuals, 'Independent numerical residuals required')
        for r in residuals:
            require(type(r.get('name')) is str and r['name'].strip(), 'Named residual required')
            actual, expected, tolerance = (finite(r[k], k) for k in ('actual', 'expected', 'tolerance'))
            require(tolerance >= 0 and abs(actual-expected) <= tolerance, 'Original independent method residual failed')
        equivalent = row.get('shared_equivalence')
        require(type(equivalent) is dict and equivalent.get('mechanism_id') == cell['mechanism_id']
                and cell['model'] in equivalent.get('models', []), 'Explicit shared-mechanism equivalence required')
        require(len(set(equivalent['models'])) == len(equivalent['models'])
                and all(type(equivalent.get(k)) is str and equivalent[k].strip() for k in ('id', 'scope', 'limits')),
                'Shared-method domain/differences must be explicit')
        if identities is not None:
            require(all((model, cell['component']) in identities and
                        identities[model, cell['component']]['mechanism_id'] == cell['mechanism_id']
                        for model in equivalent['models']), 'Shared-method equivalence crossed a different mechanism/model')
    elif gate == 'native':
        native = execution(row, 'native')
        require(native.get('native_model') == cell['native_model'] and native.get('per_model_execution') is True,
                'Actual per-model native execution required')
        require(type(native.get('coupled_components')) is list and len(native['coupled_components']) == 9
                and set(native['coupled_components']) == set(COMPONENTS), 'Complete native component chain required')
        require(native.get('presets') == ['ucrt64-headless', 'ucrt64-release'], 'Both actual native presets required')
    elif gate == 'public_installed':
        public = execution(row, gate)
        require(public.get('native_model') == cell['native_model'] and public.get('per_model_execution') is True,
                'Installed per-model public execution required')
        require(set(public.get('languages', [])) == {'cpp', 'c99', 'python'} and public.get('loaded_identity_verified') is True,
                'Actual installed C++/C99/Python provenance required')
    else:
        visual = execution(row, gate)
        require(visual.get('native_model') == cell['native_model'] and visual.get('reviewed') is True
                and visual.get('fullscreen') is True and type(visual.get('rate')) in (int, float)
                and visual.get('rate') == 1, 'Actual reviewed fullscreen1x model required')
        require(set(visual.get('enabled_effects', [])) == {'earth', 'terrain', 'clouds', 'sunlight', 'shadows', 'exhaust', 'contrails', 'native_effects'},
                'Complete visual scene required')
        bound(visual['image'], root)
    return dict(passed=True, artifact=ref['artifact'], row=ref['row'], details=row[gate],
                shared_equivalence=row.get('shared_equivalence') if gate == 'method' else None)


def admit_cells(cells, manifest, root=ROOT):
    require(manifest.get('schema') == 'fsim.sim-m03.component-evidence.v1', 'Component evidence manifest schema required')
    supplied = reconcile_population({(c['model'], c['component']) for c in cells}, manifest['cells'], ('model', 'component'))
    identities = {(c['model'], c['component']): c for c in cells}
    require(len(identities) == len(cells), 'Duplicate expected component identity')
    result = []
    for cell in cells:
        row = supplied[cell['model'], cell['component']]
        require(set(row.get('evidence', {})) == set(GATES), 'All five component gates required')
        gates = {gate: validate_gate(cell, gate, row['evidence'][gate], root, identities) for gate in GATES}
        result.append(dict(**cell, gates=gates, status='engineering-complete-uncalibrated', passed=True))
    return result


def progress_cells(cells, manifest, root=ROOT):
    """Publish each verified item immediately without promoting missing gates."""
    require(manifest.get('schema') == 'fsim.sim-m03.component-evidence.v1', 'Component evidence manifest schema required')
    identities = {(c['model'], c['component']): c for c in cells}
    supplied = {}
    for row in manifest['cells']:
        key = (row['model'], row['component'])
        require(key in identities and key not in supplied, 'Unknown/duplicate component progress identity')
        require(type(row.get('evidence')) is dict and set(row['evidence']).issubset(GATES), 'Unknown evidence gate')
        supplied[key] = row
    result = []
    for cell in cells:
        row = supplied.get((cell['model'], cell['component']), {'evidence': {}})
        gates = {}
        for gate in GATES:
            gates[gate] = (validate_gate(cell, gate, row['evidence'][gate], root, identities) if gate in row['evidence'] else
                           dict(passed=False, reason='Current model-identifiable proof required'))
        passed = all(g['passed'] for g in gates.values())
        result.append(dict(**cell, gates=gates, passed=passed,
                           status='engineering-complete-uncalibrated' if passed else 'pending-evidence'))
    return result


def calibration_ledger(cells, supplied, root=ROOT):
    names = {c['model'] for c in cells}
    require(type(supplied) is list, 'Separate calibration rows required')
    rows = reconcile_population(names, supplied, ('model',))
    result = []
    for model in sorted(names):
        row = rows[model]
        require(row.get('status') in ('uncalibrated', 'calibrated'), 'Explicit calibration status required')
        if row['status'] == 'calibrated':
            artifact = read(bound(row['evidence'], root))
            require(artifact.get('schema') == 'fsim.sim-m03.equipment-calibration.v1' and artifact.get('model') == model
                    and artifact.get('passed') is True and artifact.get('equipment_specific') is True,
                    'Equipment-specific calibration admission required')
            require(artifact.get('card_sha256') == next(c['card_sha256'] for c in cells if c['model'] == model),
                    'Calibration is for another card epoch')
            require(artifact.get('holdout_unchanged') is True and artifact.get('parameter_diff')
                    and finite(artifact['uncertainty'], 'calibration uncertainty') > 0,
                    'Independent holdout/parameter delta/nonzero uncertainty required')
            require(0 <= finite(artifact['after_error'], 'after error') < finite(artifact['before_error'], 'before error'),
                    'Actual held-out improvement required')
            for item in artifact.get('sources', []) + artifact.get('raw', []):
                bound(item, root)
            require(artifact.get('sources') and artifact.get('raw'), 'Actual calibration provenance required')
        else:
            require(type(row.get('reason')) is str and row['reason'].strip(), 'Uncalibrated reason required')
        result.append(row)
    return result


def build(coverage, manifest=None, root=ROOT, baseline='HEAD', git_source_root=None):
    cells = derive_cells(coverage, root)
    calibrated = [dict(model=model, status='uncalibrated', reason='No admitted equipment-specific held-out calibration receipt supplied.')
                  for model in sorted({c['model'] for c in cells})]
    if manifest is None:
        rows = [dict(**c, passed=False, status='pending-evidence', gates={g: dict(passed=False, reason='Current model-identifiable proof required') for g in GATES}) for c in cells]
    else:
        rows = progress_cells(cells, manifest, root)
        calibrated = calibration_ledger(cells, manifest['calibration'], root)
    complete = len(rows) == 216 and all(r['passed'] for r in rows)
    changes = deltas(cells, root, baseline, git_source_root)
    for source in {r['path']: r for c in cells for r in c['sources']}.values():
        bound(source, root)
    statements = {c['model']: read(root/c['card_path']).get('simulation_model', {}).get('calibration') for c in cells}
    return dict(schema='fsim.sim-m03.component-matrix.v1', passed=complete, engineering_acceptance=complete,
        models=24, components=list(COMPONENTS), cells=rows, passed_cells=sum(r['passed'] for r in rows),
        passed_gates={gate: sum(r['gates'][gate]['passed'] for r in rows) for gate in GATES},
        input_diagnostics=dict(available_cells=len(cells),
            component_basis_counts={basis: sum(c['input_basis'] == basis for c in cells) for basis in sorted({c['input_basis'] for c in cells})},
            card_calibration_statements=statements,
            absent_calibration_statements=sum(not statement for statement in statements.values()),
            scope='Input diagnostics only; neither proof nor calibration admission'),
        calibration=dict(required_records=24, records=calibrated,
            admitted=sum(r['status'] == 'calibrated' for r in calibrated), separate_from_engineering=True),
        limits='Component closure is not whole-program admission; missing model execution never becomes shared-method equivalence.'), changes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--coverage', type=Path, required=True)
    parser.add_argument('--evidence', type=Path)
    parser.add_argument('--root', type=Path, default=ROOT)
    parser.add_argument('--baseline', default='HEAD')
    parser.add_argument('--git-source-root', type=Path,
                        help='Explicit original main repository for immutable baseline reads from a clean source export')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--parameter-deltas', type=Path, required=True)
    args = parser.parse_args()
    result, changes = build(read(args.coverage), read(args.evidence) if args.evidence else None,
                            args.root, args.baseline, args.git_source_root)
    for path, value in ((args.output, result), (args.parameter_deltas, changes)):
        allowed(path).parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)+'\n', encoding='utf-8')
    return 0 if result['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
