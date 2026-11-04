"""Capture a fresh source-bound native method run, including its real Catch report."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET
import re

from acceptance import allowed

ROOT = Path(__file__).resolve().parents[3]
CASES = dict(effects={
    'Actual selected AAM effects methods expose primary-reference observations'},
    kinematics={
        'All actual AAM card configurations execute numeric guidance and control method references',
        'Every actual AAM engine executes an independent powered propulsion reference bench'})
MODULE_NAMES = {'libjsbsim.dll', 'libfsim.dll'}


def require(value, text):
    if not value:
        raise ValueError(text)


def path(value):
    return allowed(allowed(value).resolve())


def sha(value):
    h = hashlib.sha256()
    with path(value).open('rb') as stream:
        for block in iter(lambda: stream.read(1048576), b''):
            h.update(block)
    return h.hexdigest()


def binding(value):
    value = path(value)
    return dict(path=str(value), sha256=sha(value))


def bindings(records, root=ROOT):
    """Validate exact byte bindings, including duplicate and excluded paths."""
    require(type(records) is list, 'Runtime binding list required')
    result = {}
    for item in records:
        require(type(item) is dict and set(item) == {'path', 'sha256'}, 'Exact runtime path/hash required')
        require(type(item['sha256']) is str and re.fullmatch('[0-9a-f]{64}', item['sha256']), 'Runtime SHA256 required')
        supplied = allowed(item['path'])
        actual = path(supplied if supplied.is_absolute() else path(root)/supplied)
        require(actual.is_relative_to(path(root)) or actual.is_relative_to(path('D:/ENV')),
                'Foreign runtime evidence root')
        require(actual not in result, 'Duplicate runtime path')
        require(sha(actual) == item['sha256'], 'Runtime bytes changed: '+str(actual))
        result[actual] = item['sha256']
    return result


def staged_runtime(executable, kind):
    """Prepare expected app-directory products, without claiming they loaded."""
    executable = path(executable)
    result = [binding(executable)]
    for name in ('libJSBSim.dll', 'libfsim.dll'):
        candidate = path(executable.parent/name)
        if candidate.is_file():
            result.append(binding(candidate))
    if kind == 'kinematics':
        require(any(path(item['path']).name.casefold() == 'libjsbsim.dll' for item in result),
                'Powered reference requires its staged native JSBSim product')
    return result


def observed_modules(native, prepared, executable, require_jsbsim=False, root=ROOT):
    executable = path(executable)
    expected = bindings(prepared, root)
    require(expected.get(executable) == sha(executable), 'Prepared executable identity required')
    for candidate in expected:
        require(candidate == executable or (candidate.parent == executable.parent and candidate.name.casefold() in MODULE_NAMES),
                'Expected runtime must be an explicit staged app-directory product')
    actual = bindings(native.get('runtime_modules'), root)
    require(len(actual) <= 2, 'Unknown loaded native module population')
    names = set()
    for module, checksum in actual.items():
        name = module.name.casefold()
        require(name in MODULE_NAMES and name not in names, 'Duplicate/unknown loaded native module')
        require(module.parent == executable.parent and expected.get(module) == checksum,
                'Loaded native module is not the prepared staged product')
        names.add(name)
    require(not require_jsbsim or 'libjsbsim.dll' in names, 'Powered capture did not observe native JSBSim')
    return actual


def verify_execution_runtime(execution, captures, executable, kind, root=ROOT):
    """Actual union may exclude staged-but-unused DLLs; it may not invent use."""
    executable = path(executable)
    prepared = execution.get('staged_runtime')
    require(bindings(prepared, root) == bindings(staged_runtime(executable, kind), root), 'Prepared runtime changed since execution')
    require(type(captures) is list and len(captures) == (2 if kind == 'kinematics' else 1),
            'All capture-side module inventories required')
    expected = {executable: sha(executable)}
    schemas = set()
    for native in captures:
        schema = native.get('schema');schemas.add(schema)
        identity = native.get('executable', {})
        require(bindings([identity], root) == {executable: expected[executable]}, 'Capture executable differs from actual run')
        modules = observed_modules(native, prepared, executable,
            require_jsbsim=(schema == 'fsim.sim-m03.propulsion-reference-native.v1'), root=root)
        expected.update(modules)
    require(schemas == ({'fsim.sim-m03.component-native.v1', 'fsim.sim-m03.propulsion-reference-native.v1'}
                        if kind == 'kinematics' else {'fsim.sim-m03.component-effects-native.v1'}),
            'Foreign/missing capture-side runtime cohort')
    require(bindings(execution.get('runtime'), root) == expected,
            'Actual runtime union differs from capture-side loaded modules')
    return [dict(path=str(module), sha256=checksum) for module, checksum in sorted(expected.items(), key=lambda item: str(item[0]).casefold())]


def save(value, document):
    path(value).write_text(json.dumps(document, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def catch_counts(report, kind):
    root = ET.parse(path(report)).getroot()
    suites = root.findall('testsuite') if root.tag == 'testsuites' else [root]
    require(len(suites) == 1, 'Exactly one real Catch suite required')
    suite = suites[0]
    for name in ('errors', 'failures', 'skipped'):
        require(suite.get(name) == '0', 'Failed, missing or skipped Catch execution')
    assertions = int(suite.get('tests', '0'))
    cases = suite.findall('testcase')
    require(assertions > 0 and len(cases) == len(CASES[kind]), 'Positive complete native population required')
    require({c.get('name') for c in cases} == CASES[kind], 'Exact native case identities required')
    require(all(c.get('status') == 'run' and not any(c.find(k) is not None for k in
                ('failure', 'error', 'skipped')) for c in cases), 'Unexecuted native case')
    return dict(cases=len(cases), assertions=assertions, failures=0, errors=0, skipped=0)


def verify_capture(capture, epoch, executable, prepared):
    value = json.loads(path(capture).read_text(encoding='utf-8'))
    require(value.get('input_epoch_sha256') == sha(epoch), 'Capture belongs to another pre-execution epoch')
    identity = value.get('executable', {})
    require(path(identity['path']) == path(executable) and identity.get('sha256') == sha(executable),
            'Capture came from a different executable')
    observed_modules(value, prepared, executable,
        require_jsbsim=(value.get('schema') == 'fsim.sim-m03.propulsion-reference-native.v1'))
    require(isinstance(value.get('models'), list) and len(value['models']) == 24,
            'Actual24-model native capture required')
    names = [r.get('model') for r in value['models'] if isinstance(r, dict)]
    require(len(names) == 24 and len(set(names)) == 24 and all(isinstance(n, str) and n for n in names),
            'Distinct actual native model identities required')
    before = json.loads(path(epoch).read_text(encoding='utf-8'))
    expected = set()
    for source in before['sources']:
        p = allowed(source['path'])
        if p.is_absolute():
            if not p.is_relative_to(ROOT):
                continue
            p = p.relative_to(ROOT)
        if p.parts[:2] == ('assets', 'stores') and p.suffix == '.json':
            expected.add(p.stem)
    require(len(expected) == 24 and set(names) == expected, 'Captured models differ from frozen authored inputs')
    return value


def capture_environment(kind, output, epoch):
    if kind == 'effects':
        return ('[component-effects-reference]',
                dict(FSIM_COMPONENT_EFFECT_REFERENCE_EPOCH=str(epoch), FSIM_COMPONENT_EFFECT_REFERENCE_DIR=str(output)),
                [output/'component-effects-native.json'])
    return ('[component-reference-native]',
            dict(FSIM_COMPONENT_REFERENCE_EPOCH=str(epoch), FSIM_COMPONENT_REFERENCE_JSON=str(output/'kinematics.json'),
                 FSIM_COMPONENT_PROPULSION_JSON=str(output/'kinematics.json.propulsion.json')),
            [output/'kinematics.json', output/'kinematics.json.propulsion.json'])


def run(kind, executable, directory):
    executable, directory = path(executable), path(directory)
    require(kind in CASES and executable.is_file(), 'Known native reference and executable required')
    directory.mkdir(parents=True, exist_ok=True)
    output = directory/('run-'+uuid.uuid4().hex)
    output.mkdir()
    epoch, report, raw = output/'input-epoch.json', output/'catch.xml', output/'native.log'
    checker = ROOT/'tools/validation/weapon_quality'/('component_references_'+kind+'.py')
    prepared = [sys.executable, '-B', str(checker), '--freeze-epoch', str(epoch)]
    env = os.environ.copy()
    env.update(TMP='D:/ENV/tmp', TEMP='D:/ENV/tmp', TORCH_EXTENSIONS_DIR='D:/ENV/tmp', PYTHONDONTWRITEBYTECODE='1')
    with (output/'epoch.log').open('wb') as stream:
        subprocess.run(prepared, cwd=ROOT, env=env, stdout=stream, stderr=subprocess.STDOUT, timeout=120, check=True)
    before = json.loads(epoch.read_text(encoding='utf-8'))
    selected, supplied, captures = capture_environment(kind, output, epoch)
    env.update(supplied)
    command = [str(executable), selected, '--reporter', 'junit', '--out', str(report)]
    prepared_runtime = staged_runtime(executable, kind)
    started = time.perf_counter()
    execution = dict(command=command, environment=supplied, exit_code=None,
                     staged_runtime=prepared_runtime, runtime=[], raw=[])
    receipt = dict(schema='fsim.sim-m03.component-reference-native-run.v1', passed=False,
                   engineering_acceptance=False, kind=kind, executions=[execution], prepare_command=prepared)
    try:
        with raw.open('wb') as stream:
            process = subprocess.Popen(command, cwd=ROOT, env=env, stdout=stream, stderr=subprocess.STDOUT)
            try:
                execution['exit_code'] = process.wait(timeout=180)
            except subprocess.TimeoutExpired:
                subprocess.run(['taskkill', '/PID', str(process.pid), '/T', '/F'], stdout=stream,
                               stderr=subprocess.STDOUT, timeout=30, check=False)
                process.wait(timeout=30)
                raise
        require(execution['exit_code'] == 0, 'Actual native reference failed; retained '+str(output))
        receipt['counts'] = catch_counts(report, kind)
        headers = [verify_capture(capture, epoch, executable, prepared_runtime) for capture in captures]
        for source in before['sources']:
            p = path(source['path']) if Path(source['path']).is_absolute() else path(ROOT/source['path'])
            require(sha(p) == source['sha256'], 'Native input changed during execution: '+str(p))
        actual_runtime = {path(executable): sha(executable)}
        for header in headers:
            actual_runtime.update(observed_modules(header, prepared_runtime, executable,
                require_jsbsim=(header.get('schema') == 'fsim.sim-m03.propulsion-reference-native.v1')))
        execution['runtime'] = [dict(path=str(p), sha256=h) for p, h in sorted(actual_runtime.items(), key=lambda item: str(item[0]).casefold())]
        verify_execution_runtime(execution, headers, executable, kind)
        receipt['passed'] = True
    finally:
        execution['elapsed_s'] = time.perf_counter()-started
        execution['raw'] = [binding(p) for p in [*captures, report, epoch, raw] if p.is_file()]
        save(output/'run.json', receipt)
    save(directory/'latest.json', dict(receipt=binding(output/'run.json'), passed=True))
    print(json.dumps(dict(passed=True, kind=kind, **receipt['counts'], receipt=binding(output/'run.json'))))
    return receipt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--kind', choices=tuple(CASES), required=True)
    parser.add_argument('--executable', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    a = parser.parse_args()
    run(a.kind, a.executable, a.output)


if __name__ == '__main__':
    main()
