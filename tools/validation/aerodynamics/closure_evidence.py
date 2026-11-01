"""Source/binary binding and nested-method accounting for closure receipts."""
import csv
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def derived_output(relative,protected):
    """Only documented build/report outputs, never tracked or declared inputs."""
    if relative in protected:return False
    parts=Path(relative).parts
    return ((len(parts)>=4 and parts[0]=='aircraft' and parts[2]=='out')
        or relative.startswith('examples/training/rust_trainer/target/'))


def fingerprint(root,preset,inventory,protected_outputs):
    files=set()
    derived={}
    for name in ('src','include','cmake','tests','tools','python','aircraft','assets','third_party','deps','examples','docs','.agents','.claude','.github'):
        for folder,dirs,names in os.walk(root/name,followlinks=False):
            dirs[:]=sorted(d for d in dirs if d.casefold() not in ('.codex','.git','__pycache__'))
            if Path(folder)==root/'assets':dirs[:]=[d for d in dirs if d!='maps']
            for name in names:
                path=Path(folder)/name
                if '.codex' in {p.casefold() for p in path.resolve().parts}:
                    raise ValueError('Excluded source binding path')
                relative=path.relative_to(root).as_posix()
                if derived_output(relative,protected_outputs):
                    derived[relative]=sha(path)
                    continue
                files.add(path)
    for name in ('CMakeLists.txt','CMakePresets.json','run.ps1','build.ps1','setup.ps1','stop.ps1',
                 '.gitattributes','AGENTS.md','README.md','docs/data/sim-m02-deferred-rotor-issues-20261101.json'):
        files.add(root/name)
    sources={p.relative_to(root).as_posix():sha(p) for p in sorted(files) if p.is_file()}
    build=root/'build'/preset
    executables={Path(t['command'][0]) for t in inventory['tests'] if t.get('command')}
    runtime={str(p.resolve()):sha(p) for p in sorted(executables) if p.is_file()}
    for relative in ('bin/libfsim.dll','bin/libJSBSim.dll','tests/libfsim.dll','tests/libJSBSim.dll',
                     'python/fsim/_native.pyd','python/fsim/bin/libfsim.dll','python/fsim/bin/libJSBSim.dll',
                     'bin/flightsim.exe','bin/flightsim-viewer.exe','bin/libfsim_vision.dll','python/fsim/_vision.pyd'):
        path=build/relative
        if path.is_file():runtime[str(path.resolve())]=sha(path)
    for folder in (build/'bin',build/'tests',build/'python/fsim/bin'):
        if folder.is_dir():
            for path in folder.iterdir():
                if path.name.casefold()=='.codex':continue
                if path.is_file() and path.suffix.casefold() in ('.dll','.exe','.pyd','.spv'):
                    runtime[str(path.resolve())]=sha(path)
    return dict(source_root=str(root),preset=preset,source_files=sources,runtime_files=runtime,
        derived_output_files=derived,
        cmake_cache_sha256=sha(build/'CMakeCache.txt'),compile_commands_sha256=sha(build/'compile_commands.json'))


def production_preflight(root,preset,production,git_root):
    """Bind fresh generation and all actual inputs before functional execution."""
    production=Path(production).resolve()
    matrix=json.loads((production/'matrix.json').read_text(encoding='utf-8'))
    rows=matrix['designs']
    if len(rows)!=88 or len({r['name'] for r in rows})!=88:
        raise ValueError('Current complete production roster is required')
    git_root=Path(git_root or root).resolve()
    # The shared index must still equal HEAD; filter excluded directories in
    # Git's pathspec before obtaining the immutable tracked-input inventory.
    subprocess.run(['git','diff','--cached','--quiet','HEAD','--'],cwd=git_root,check=True)
    tracked=subprocess.check_output(['git','ls-files','--','aircraft','examples/training/rust_trainer',
        ':(exclude,glob)**/.codex/**',':(exclude,glob).codex/**'],cwd=git_root).decode().splitlines()
    protected=set()
    for name in tracked:
        parts=Path(name).parts
        if '.codex' in {p.casefold() for p in parts}:continue
        if ((len(parts)>=4 and parts[0]=='aircraft' and parts[2]=='out')
                or name.startswith('examples/training/rust_trainer/target/')):protected.add(name)
    source_hashes={};presence={};receipts={};caches={}
    for row in rows:
        receipt=production/row['name']/'generation.json'
        generation=json.loads(receipt.read_text(encoding='utf-8'))
        if not all(c['passed'] for c in generation['checks']):raise ValueError('Required generated component/table failed')
        receipts[row['name']]=sha(receipt)
        for key in ('input_sha256','generator_sha256'):
            for name,digest in generation[key].items():
                path=Path(name)
                if path.is_absolute() or '..' in path.parts or '.codex' in {p.casefold() for p in path.parts}:
                    raise ValueError('Excluded production input')
                if sha(root/path)!=digest:raise ValueError('Production input changed: '+name)
                source_hashes[path.as_posix()]=digest
        for name,present in generation['input_presence'].items():
            path=Path(name)
            if path.is_absolute() or '..' in path.parts or '.codex' in {p.casefold() for p in path.parts}:
                raise ValueError('Excluded optional production input')
            if (root/path).is_file()!=present:raise ValueError('Production input presence changed: '+name)
            presence[path.as_posix()]=present
            if len(path.parts)>=4 and path.parts[0]=='aircraft' and path.parts[2]=='out':protected.add(path.as_posix())
        for name,digest in generation['products'].items():
            if sha(root/'aircraft'/name)!=digest:raise ValueError('Generated runtime product changed: '+name)
        cache=root/'aircraft'/row['name']/'out/tables.pkl'
        import pickle
        with cache.open('rb') as stream:cached=pickle.load(stream)
        if cached.get('_key')!=generation['table_inputs_hash']:
            raise ValueError('Missing current-source generated intermediate table cache: '+row['name'])
        caches[row['name']]=dict(path=str(cache),sha256=sha(cache),table_inputs_hash=cached['_key'])
    runtime={}
    for name in ('libfsim.dll','libJSBSim.dll'):
        paths=[root/'build'/preset/relative/name for relative in ('bin','tests','python/fsim/bin')]
        hashes={str(p):sha(p) for p in paths}
        if len(set(hashes.values()))!=1:raise ValueError('Stale staged runtime dependency: '+name)
        runtime.update(hashes)
    return dict(production=str(production),generation_receipts=receipts,source_hashes=source_hashes,
        optional_input_presence=presence,protected_output_inputs=sorted(protected),
        current_generated_intermediate_caches=caches,canonical_runtime_identity=runtime,
        output_policy='Only non-source derived aircraft/<design>/out and the documented Cargo target tree are mutable; HEAD-tracked files and every declared input including optional absence remain protected')


def rust_fixture_runtime(root,preset,inventory):
    """Cargo intentionally stages this preset's DLLs beside its test binary."""
    if not any(t['name']=='rust_example' for t in inventory['tests']):return None
    rows=[]
    for name in ('libfsim.dll','libJSBSim.dll'):
        canonical=root/'build'/preset/'bin'/name
        staged=root/'examples/training/rust_trainer/target/release'/name
        rows.append(dict(path=str(staged),sha256=sha(staged),canonical=str(canonical),
            canonical_sha256=sha(canonical),matches=sha(staged)==sha(canonical)))
    return dict(scope='Fixture-built derived Cargo runtime; not a maintained source or production DLL mutation',
        files=rows,passed=all(r['matches'] for r in rows))


def git_bridge(root,git_root,out,environment):
    method='tests.test_bootstrap_snapshot.SourceCheckout.test_git_checkout_preserves_byte_bound_products_and_the_real_index'
    source_method='hangar test_bootstrap_snapshot.SourceCheckout'
    paths=('.gitattributes','README.md','tools/bootstrap/dependencies.lock.json','aircraft/c17a/c17a.xml',
           'tools/bootstrap/snapshot.py','tools/models/hangar/tests/test_bootstrap_snapshot.py',
           'tools/models/hangar/tests/paths.py','tools/models/hangar/tests/__init__.py')
    if (root/'.git').is_dir():return None
    if git_root is None:raise ValueError('An ordinary source export requires --git-source-root for its unchanged Git-only method')
    git_root=Path(git_root).resolve()
    if '.codex' in {p.casefold() for p in git_root.parts}:raise ValueError('Excluded Git source root')
    if subprocess.check_output(['git','branch','--show-current'],cwd=git_root).decode().strip()!='main':
        raise ValueError('Git-only source check must execute on main')
    binding={p:sha(root/p) for p in paths}
    if any(sha(git_root/p)!=digest for p,digest in binding.items()):
        raise ValueError('Git-only fixture or declared inputs differ from the clean export')
    receipt=out/'git-source-method.json'
    # One unchanged method is independent of preset. Reuse only a complete
    # current-input receipt supplied explicitly by the parent workflow.
    if receipt.exists():
        value=json.loads(receipt.read_text(encoding='utf-8'))
        if value.get('inputs')==binding and value.get('passed') and value.get('git_root')==str(git_root):
            return value
    env=environment.copy();env['FSIM_PROJECT_ROOT']=str(git_root)
    before=sha(git_root/'.git/index')
    command=[sys.executable,'-B','-m','unittest','-v',method]
    process=subprocess.run(command,cwd=git_root/'tools/models/hangar',env=env,capture_output=True,timeout=120)
    log=out/'git-source-method.log';log.write_bytes(process.stdout+process.stderr)
    text=(process.stdout+process.stderr).decode('utf-8',errors='replace')
    after=sha(git_root/'.git/index')
    value=dict(method=method,ctest_case=source_method,git_root=str(git_root),inputs=binding,command=command,
        exit_code=process.returncode,real_index_before=before,real_index_after=after,
        passed=process.returncode==0 and before==after and 'Ran 1 test' in text and 'skipped=' not in text,
        log_sha256=sha(log))
    receipt.write_text(json.dumps(value,indent=2)+'\n',encoding='utf-8')
    if not value['passed']:raise ValueError('Unchanged Git-only source method failed on main')
    return value


def nested_accounting(cases,policy,preset,bridge):
    deferred=set(policy['python_deferred_methods'])
    result=dict(unittest_methods=0,user_deferred_methods=[],platform_optional_skips=[],source_export_bridged_methods=[],unexpected_skips=[])
    for case in cases:
        text=case.findtext('system-out','')+'\n'+case.findtext('system-err','')
        counts=re.findall(r'Ran (\d+) tests? in ',text)
        if counts:result['unittest_methods']+=int(counts[-1])
        if case.get('name','').startswith(('python_sdk ','hangar ')) and (not counts or int(counts[-1])==0):
            result['unexpected_skips'].append(dict(case=case.get('name'),reason='Missing positive unittest method summary'))
        summaries=re.findall(r'(?:OK|FAILED) \([^\n)]*skipped=(\d+)[^\n)]*\)',text)
        skipped=int(summaries[-1]) if summaries else 0
        markers=re.findall(r'FSIM_SCOPE_DEFERRED_METHOD: ([\w.]+)',text)
        unexpected=set(markers)-deferred
        if unexpected or len(markers)!=len(set(markers)):
            result['unexpected_skips'].append(dict(case=case.get('name'),invalid_deferred_markers=markers))
        result['user_deferred_methods'].extend(markers)
        remaining=skipped-len(markers)
        if remaining<0:
            result['unexpected_skips'].append(dict(case=case.get('name'),summary_count=skipped,markers=markers));continue
        if remaining and bridge and case.get('name')==bridge['ctest_case'] and remaining==1:
            result['source_export_bridged_methods'].append(bridge['method']);remaining=0
        if remaining and preset=='ucrt64-headless' and case.get('name') in ('python_sdk test_vision','python_sdk test_package_imports'):
            result['platform_optional_skips'].append(dict(case=case.get('name'),methods=remaining,reason='Renderer intentionally absent from headless configuration'))
            remaining=0
        if remaining:result['unexpected_skips'].append(dict(case=case.get('name'),unaccounted_methods=remaining))
        if re.search(r'test cases?:[^\n]*\bskipped\b',text):
            result['unexpected_skips'].append(dict(case=case.get('name'),reason='Unexecuted native Catch case'))
    if set(result['user_deferred_methods'])!=deferred:
        result['unexpected_skips'].append(dict(missing_declared_deferred_methods=sorted(deferred-set(result['user_deferred_methods']))))
    return result


def fleet_coverage(root,cases):
    with (root/'docs/data/sim-m02-engineering-completeness.csv').open(encoding='utf-8-sig',newline='') as stream:
        expected={row['design'] for row in csv.DictReader(stream) if row['category']=='fixed-wing'}
    shards=[];populations=[]
    for case in cases:
        if re.fullmatch(r'fleet: .* - shard \d+ of 16',case.get('name','')):
            text=case.findtext('system-out','')
            names=re.findall(r'FSIM_SCOPE_FLEET_FIXED_WING: ([\w-]+)',text)
            shards.append(dict(case=case.get('name'),fixed_wing_designs=names))
            populations.extend(names)
    return dict(shards=shards,fixed_wing_designs=sorted(set(populations)),
        passed=len(shards)==16 and len(expected)==31 and len(populations)==31 and set(populations)==expected)
