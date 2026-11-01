"""Serial complete UCRT64 headless/Release verification, with raw evidence.

No CTest selectors, tolerance changes or substituted expected values. A source
export can use an existing installed dependency prefix as a read-only input.
"""
import argparse
import ast
import hashlib
import json
import os
import re
from pathlib import Path, PureWindowsPath
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET


def fixture_input_paths(source):
    """Read the unchanged compatibility fixture's literal input declaration."""
    assignments=[node for node in ast.walk(ast.parse(source)) if isinstance(node,ast.Assign)
        and any(isinstance(target,ast.Name) and target.id=='paths' for target in node.targets)]
    if len(assignments)!=1:raise ValueError('Expected one explicit fixture input declaration')
    paths=ast.literal_eval(assignments[0].value)
    if not isinstance(paths,(tuple,list)) or not paths or len(paths)>32:raise ValueError('Invalid fixture input declaration')
    for name in paths:
        if not isinstance(name,str):raise ValueError('Non-string fixture input')
        path=Path(name)
        windows=PureWindowsPath(name)
        parts=(*path.parts,*windows.parts)
        if not name or not path.parts or path.anchor or windows.anchor or '..' in parts or '.codex' in {p.casefold() for p in parts}:raise ValueError('Excluded fixture input')
    return tuple(paths)


def main(args):
    root=Path(args.source).resolve();out=Path(args.output).resolve();out.mkdir(parents=True,exist_ok=True)
    if '.codex' in {p.casefold() for p in root.parts}:raise ValueError('Excluded source')
    environment=os.environ.copy()
    for key in ('FSIM_DATA_DIR','FSIM_AIRCRAFT_PATH','FSIM_DLL_DIR','JSBSIM_ROOT','PYTHONPATH','HANGAR_MESHKIT_DIR'):environment.pop(key,None)
    environment.update(TMP='D:/ENV/tmp',TEMP='D:/ENV/tmp',TORCH_EXTENSIONS_DIR='D:/ENV/tmp',MPLCONFIGDIR='D:/ENV/cache/matplotlib',XDG_CACHE_HOME='D:/ENV/cache',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1',FSIM_HANGAR_THREADS='1')
    shell=environment.get('FSIM_BUILD_POWERSHELL') or shutil.which('pwsh')
    if not shell:raise ValueError('Set FSIM_BUILD_POWERSHELL to the existing PowerShell launcher host')
    launcher=[shell,'-NoProfile','-ExecutionPolicy','Bypass','-File',str(root/'tools/launch/fsim.ps1')]
    ctest=Path('D:/ENV/DevLanguages/Cpp/msys2/ucrt64/bin/ctest.exe')
    started=time.monotonic();stages=[];suites=[];checkout_fixture={}
    def run(name,command,limit=7200,capture=None,env=None):
        tick=time.monotonic();log=out/(name+'.log')
        with log.open('wb') as stream:
            process=subprocess.Popen([str(x) for x in command],cwd=root,env=environment if env is None else env,stdout=stream,stderr=subprocess.STDOUT)
            try:code=process.wait(timeout=limit)
            except subprocess.TimeoutExpired:
                subprocess.run(['taskkill','/PID',str(process.pid),'/T','/F'],stdout=stream,stderr=subprocess.STDOUT,timeout=15)
                process.wait(timeout=15);code=-1
        record=dict(name=name,argv=[str(x) for x in command],cwd=str(root),log=str(log),exit_code=code,elapsed_seconds=time.monotonic()-tick)
        if name.endswith('-full-unittest'):
            raw=log.read_text(encoding='utf-8',errors='replace')
            counts=re.findall(r'Ran (\d+) tests? in ([\d.]+)s',raw)
            record['unittest_cases']=int(counts[-1][0]) if counts else None
            summary=re.findall(r'^(?:OK|FAILED).*$',raw,re.MULTILINE)
            record['unittest_summary']=summary[-1] if summary else None
        stages.append(record);write()
        print(name,code,round(record['elapsed_seconds'],3),flush=True)
        return code
    def write():
        (out/'verification.json').write_text(json.dumps(dict(source=str(root),elapsed_seconds=time.monotonic()-started,stages=stages,suites=suites,checkout_fixture=checkout_fixture),indent=2)+'\n',encoding='utf-8')
    for preset in args.presets:
        environment['HANGAR_MESHKIT_DIR']=str(root/'build'/preset/'python')
        configure=launcher+['--preset',preset,'configure','-DFSIM_PYTHON_EXECUTABLE='+sys.executable,'-DFSIM_CHECK_PYTHON='+sys.executable,'-DFSIM_BUILD_PYTHON=ON','-DFSIM_BUILD_TESTS=ON']
        if args.dependencies:configure.append('-DCMAKE_PREFIX_PATH='+str(Path(args.dependencies).resolve()))
        if run(preset+'-configure',configure):continue
        if run(preset+'-build',launcher+['--preset',preset,'build','--parallel','8']):continue
        build=root/'build'/preset
        discovery=subprocess.run([str(ctest),'--test-dir',str(build),'--show-only=json-v1'],cwd=root,env=environment,stdout=subprocess.PIPE,stderr=subprocess.PIPE,check=True,timeout=120)
        manifest=json.loads(discovery.stdout);(out/(preset+'-discovery.json')).write_bytes(discovery.stdout)
        expected=len(manifest['tests'])
        if not expected:raise ValueError('No tests discovered')
        unsafe=[]
        for test in manifest['tests']:
            properties={p['name']:p['value'] for p in test.get('properties',[])}
            executable=Path(test['command'][0]).name.casefold() if test.get('command') else ''
            readonly='slot-read-only' in properties.get('LABELS',[])
            needs_slot=(executable.startswith(('python','cmake')) and not readonly) or 'render' in properties.get('LABELS',[])
            if needs_slot and not properties.get('RUN_SERIAL',False):unsafe.append(test['name'])
        (out/(preset+'-slot-audit.json')).write_text(json.dumps(dict(discovered=expected,unsafe_script_or_gpu_cases=unsafe,hangar_threads=1,cpu_jobs=args.jobs),indent=2)+'\n',encoding='utf-8')
        if unsafe:raise ValueError('Unsafe simultaneous CMake/GUI fixtures: '+str(unsafe))
        inventory_file=out/(preset+'-hangar-method-inventory.json')
        inventory_env=environment.copy()
        inventory_env.update(PYTHONPATH=str(root/'tools/models/hangar')+';'+str(build/'python'),FSIM_DLL_DIR=str(build/'python/fsim/bin'))
        inventory_code="""import json,sys,unittest
from pathlib import Path
suite=unittest.defaultTestLoader.discover(str(Path(sys.argv[1])/'tests'),top_level_dir=sys.argv[1])
def walk(s):
    for t in s:
        if isinstance(t,unittest.TestSuite):yield from walk(t)
        else:yield t.id()
ids=sorted(walk(suite));classes={}
for identity in ids:classes.setdefault(identity.rsplit('.',1)[0].removeprefix('tests.'),[]).append(identity)
Path(sys.argv[2]).write_text(json.dumps(dict(test_ids=ids,classes=classes,loader_errors=unittest.defaultTestLoader.errors),indent=2)+'\\n',encoding='utf-8')
"""
        run(preset+'-hangar-method-inventory',[sys.executable,'-B','-c',inventory_code,root/'tools/models/hangar',inventory_file],limit=120,env=inventory_env)
        method_inventory=json.loads(inventory_file.read_text(encoding='utf-8'))
        junit=out/(preset+'.junit.xml')
        run(preset+'-full-ctest',launcher+['--preset',preset,'test','-j',str(args.jobs),'--no-tests=error','--output-on-failure',
            '--test-output-size-passed','8388608','--test-output-size-failed','8388608','--output-junit',junit])
        if junit.is_file():
            tree=ET.parse(junit);cases=tree.findall('.//testcase')
            methods=[]
            registered={t['name'].removeprefix('hangar ') for t in manifest['tests'] if t['name'].startswith('hangar ')}
            for case in cases:
                if not case.get('name','').startswith('hangar '):continue
                class_name=case.get('name').removeprefix('hangar ');output=case.findtext('system-out','')
                observed=sorted(set(re.findall(r'^test_[A-Za-z0-9_]+ \((tests\.[^)]+)\)',output,re.MULTILINE)))
                ran=re.findall(r'Ran (\d+) tests? in [\d.]+s',output)
                skipped=re.findall(r'skipped=(\d+)',output)
                skipped_ids=sorted(set(re.findall(r'^test_[A-Za-z0-9_]+ \((tests\.[^)]+)\)\s+\.\.\.\s+skipped',output,re.MULTILINE)))
                expected_ids=method_inventory['classes'].get(class_name,[])
                methods.append(dict(class_name=class_name,expected_ids=expected_ids,observed_ids=observed,
                    completed_methods=int(ran[-1]) if ran else None,skipped=int(skipped[-1]) if skipped else 0,skipped_ids=skipped_ids,
                    exact_ids_match=bool(expected_ids) and sorted(observed)==sorted(expected_ids) and bool(ran) and int(ran[-1])==len(expected_ids)))
            method_coverage=registered==set(method_inventory['classes']) and len(methods)==len(registered) and not method_inventory['loader_errors'] and all(m['exact_ids_match'] for m in methods)
            (out/(preset+'-hangar-method-results.json')).write_text(json.dumps(dict(exact_coverage=method_coverage,loader_classes=sorted(method_inventory['classes']),registered_classes=sorted(registered),classes=methods),indent=2)+'\n',encoding='utf-8')
            suites.append(dict(preset=preset,discovered=expected,executed_records=len(cases),
                native_geometry_backend=str(build/'python/hangar_meshkit.dll'),
                native_geometry_backend_sha256=hashlib.sha256((build/'python/hangar_meshkit.dll').read_bytes()).hexdigest() if (build/'python/hangar_meshkit.dll').is_file() else None,
                test_names_match=sorted(t['name'] for t in manifest['tests'])==sorted(c.get('name') for c in cases),
                failed=sum(c.find('failure') is not None or c.find('error') is not None for c in cases),
                skipped=sum(c.find('skipped') is not None for c in cases),junit=str(junit),
                hangar_method_coverage_complete=method_coverage,hangar_methods_completed=sum(m['completed_methods'] or 0 for m in methods),hangar_underlying_unittest_skipped=sum(m['skipped'] for m in methods),
                hangar_methods_executed=sum(m['completed_methods'] or 0 for m in methods)-sum(m['skipped'] for m in methods),
                skipped_hangar_method_ids=sorted({identity for m in methods for identity in m['skipped_ids']}),
                uncovered_nested_skips=sum(m['skipped'] for m in methods),
                junit_sha256=hashlib.sha256(junit.read_bytes()).hexdigest()))
            write()
    # Hangar methods already execute in CTest; reconcile exact loader and verbose
    # execution IDs instead of repeating an expensive monolithic serial suite.
    selected=root/'build'/args.presets[-1]
    environment.update(PYTHONPATH=str(root/'tools/models/hangar')+';'+str(selected/'python'),
        FSIM_DLL_DIR=str(selected/'python/fsim/bin'),FSIM_AIRCRAFT_PATH=str(selected/'share/flightsim/aircraft')+';'+str(selected/'share/flightsim/aircraft/stores'))
    # The original source-checkout fixture genuinely requires Git. Execute the
    # unchanged test on main, guarded by the exact source/input bytes, while
    # retaining every export skip in its original per-preset record.
    identity='tests.test_bootstrap_snapshot.SourceCheckout.test_git_checkout_preserves_byte_bound_products_and_the_real_index'
    if any(identity in s['skipped_hangar_method_ids'] for s in suites) and args.git_checkout:
        checkout=Path(args.git_checkout).resolve();tick=time.monotonic()
        checkout_fixture.update(source=str(checkout),expected_method=identity,passed=False,files=[])
        try:
            if '.codex' in {p.casefold() for p in checkout.parts} or not (checkout/'.git').is_dir():raise ValueError('A main checkout, not another worktree, is required')
            branch=subprocess.check_output(['git','-C',str(checkout),'branch','--show-current'],text=True,timeout=30).strip()
            head=subprocess.check_output(['git','-C',str(checkout),'rev-parse','HEAD'],text=True,timeout=30).strip()
            if branch!='main' or (args.expected_head and head!=args.expected_head):raise ValueError('Unexpected source checkout branch/HEAD')
            checkout_fixture.update(branch=branch,head=head)
            fixture_source='tools/models/hangar/tests/test_bootstrap_snapshot.py'
            paths=(fixture_source,'tools/models/hangar/tests/__init__.py','tools/models/hangar/tests/paths.py','tools/bootstrap/snapshot.py','tools/bootstrap/project.py',
                *fixture_input_paths((root/fixture_source).read_text(encoding='utf-8')))
            for relative in paths:
                source=checkout/relative;exported=root/relative
                a=hashlib.sha256(source.read_bytes()).hexdigest();b=hashlib.sha256(exported.read_bytes()).hexdigest()
                checkout_fixture['files'].append(dict(path=relative,checkout_sha256=a,export_sha256=b,equal=a==b))
                if a!=b:raise ValueError('Checkout fixture source/input differs from tested export: '+relative)
            fixture_env=environment.copy();fixture_env['PYTHONPATH']=str(checkout/'tools/models/hangar')
            code=run('git-checkout-fixture-full-unittest',[sys.executable,'-B','-m','unittest',identity,'-v'],env=fixture_env,limit=120)
            raw=(out/'git-checkout-fixture-full-unittest.log').read_text(encoding='utf-8')
            observed=set(re.findall(r'^test_[A-Za-z0-9_]+ \((tests\.[^)]+)\)',raw,re.MULTILINE))
            checkout_fixture.update(observed_method_ids=sorted(observed),unittest_exit_code=code,
                unittest_cases=stages[-1].get('unittest_cases'),unittest_summary=stages[-1].get('unittest_summary'))
            checkout_fixture['passed']=bool(code==0 and observed=={identity} and re.search(r'Ran 1 test in [\d.]+s',raw) and not re.search(r'\bskipped\b',raw))
            checkout_fixture['elapsed_seconds']=time.monotonic()-tick
        except Exception as error:
            checkout_fixture.update(error=str(error),elapsed_seconds=time.monotonic()-tick)
            stages.append(dict(name='git-checkout-fixture-preconditions',exit_code=1,error=str(error)))
        for suite in suites:
            covered=[identity] if checkout_fixture['passed'] and identity in suite['skipped_hangar_method_ids'] else []
            suite['externally_executed_nested_method_ids']=covered
            suite['uncovered_nested_skips']=suite['hangar_underlying_unittest_skipped']-len(covered)
        write()
    original_burn=out/'original-motor-flight.json'
    run('original-motor-flight',[sys.executable,'-B',root/'tools/validation/aerodynamics/engineering_burn.py',
        '--build',selected,'--output',original_burn],limit=1200)
    if original_burn.is_file():
        original=json.loads(original_burn.read_text(encoding='utf-8'))
        stages[-1]['native_design_cases']=original['required_cases']
        stages[-1]['failed_native_design_cases']=original['failed_cases']
        stages[-1]['applicability_design_records']=original['completed_designs']
    for label,directory in [('aerodynamics','tools/validation/aerodynamics')]:
        run(label+'-full-unittest',[sys.executable,'-B','-m','unittest','discover','-s',directory,'-p','test_*.py','-v'])
    write()
    return int(len(suites)!=len(args.presets) or any(s['discovered']!=s['executed_records'] or not s['test_names_match'] or not s['hangar_method_coverage_complete'] or s['failed'] or s['skipped'] or s['uncovered_nested_skips'] for s in suites) or any(s['exit_code'] for s in stages))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--source',required=True);parser.add_argument('--output',required=True);parser.add_argument('--dependencies')
    parser.add_argument('--presets',nargs='+',choices=['ucrt64-headless','ucrt64-release'],default=['ucrt64-headless','ucrt64-release'])
    parser.add_argument('--jobs',type=int,default=16,help='Read-only CPU cases; CMake writers and render cases remain RUN_SERIAL')
    parser.add_argument('--git-checkout',help='Original main checkout for the unchanged Git-only source fixture; exact fixture/input bytes must match')
    parser.add_argument('--expected-head',help='Pinned main HEAD for the original Git-only source fixture')
    raise SystemExit(main(parser.parse_args()))
