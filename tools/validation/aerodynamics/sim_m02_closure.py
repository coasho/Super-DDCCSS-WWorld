"""Run the complete preset inventory except explicitly user-deferred scenarios.

The explicit family scope defers rotor flight populations. Fixed-wing physical
inputs, assertion bounds and complete inventory reconciliation are retained.
"""
import argparse
import hashlib
import json
import os
import re
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET
from closure_evidence import fingerprint,git_bridge,nested_accounting,fleet_coverage,production_preflight,rust_fixture_runtime


APPROVED_POLICY_SHA256 = 'd52e0e76afdb93ddbe05513f8a04253812271cf902f865d92f03527bb11e3645'

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main(args):
    for value in (args.source_root or Path(__file__).resolve().parents[3],args.output,args.maps_dir,args.git_source_root):
        if value is not None and '.codex' in {p.casefold() for p in Path(value).parts}:
            raise ValueError('Excluded directory')
    root=Path(args.source_root).resolve() if args.source_root else Path(__file__).resolve().parents[3]
    out=Path(args.output).resolve()
    if any(p.casefold()=='.codex' for p in (*root.parts,*out.parts)):
        raise ValueError('Excluded directory')
    out.mkdir(parents=True,exist_ok=True)
    policy_path=root/'docs/data/sim-m02-deferred-rotor-issues-20261101.json'
    policy=json.loads(policy_path.read_text(encoding='utf-8'))
    excluded=policy['excluded_tests']
    if sha(policy_path)!=APPROVED_POLICY_SHA256 or policy.get('scope_id')!='sim-m02-nonrotor-v1':
        raise ValueError('The reviewed user-authorized policy differs')
    if len(excluded)!=28 or len(set(excluded))!=28:
        raise ValueError('The approved rotor-only exclusion set is incomplete')
    environment=os.environ.copy()
    environment.update(TMP='D:/ENV/tmp',TEMP='D:/ENV/tmp',TORCH_EXTENSIONS_DIR='D:/ENV/tmp',FSIM_TEST_FUNCTIONAL_SCOPE=policy['scope_id'])
    shell=args.powershell or environment.get('FSIM_BUILD_POWERSHELL')
    if not shell or not Path(shell).is_file():
        raise ValueError('Pass the existing PowerShell launcher host with --powershell')
    prefix=[shell,'-NoProfile','-File',str(root/'tools/launch/fsim.ps1'),'--preset',args.preset,'test']
    started=time.monotonic()
    inventory_command=prefix+['--show-only=json-v1']
    inventory=subprocess.run(inventory_command,cwd=root,env=environment,capture_output=True,timeout=120)
    raw=inventory.stdout.decode('utf-8-sig')
    (out/'inventory-raw.log').write_bytes(inventory.stdout+inventory.stderr)
    if inventory.returncode:
        raise RuntimeError('CTest inventory failed')
    data=json.loads(raw[raw.index('{'):])
    (out/'inventory.json').write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
    names=[t['name'] for t in data['tests']]
    if len(set(names))!=len(names) or not set(excluded).issubset(names):
        raise ValueError('Missing deferred scenarios or duplicate CTest names')
    selected=[n for n in names if n not in excluded]
    if not selected:
        raise ValueError('Empty closure suite')
    selection=out/'selected-tests.txt'
    selection.write_text('\n'.join(selected)+'\n',encoding='utf-8')
    junit=out/'junit.xml'
    command=prefix+['--no-tests=error','--parallel',str(args.jobs),'--timeout','1200',
                    '--test-output-size-passed','8388608','--test-output-size-failed','8388608',
                    '--tests-from-file',str(selection),'--output-junit',str(junit)]
    record=dict(schema='fsim.sim-m02.closure-regression.v1',preset=args.preset,
        source_root=str(root),policy_sha256=sha(policy_path),runner_sha256=sha(Path(__file__)),functional_scope=policy['scope_id'],discovered=len(names),selected=len(selected),
        excluded=len(excluded),excluded_names=excluded,excluded_status='deferred, not passed',
        command=command,inventory_command=inventory_command,inventory_only=args.inventory_only)
    report=out/'regression.json'
    def write():
        record['elapsed_seconds']=time.monotonic()-started
        report.write_text(json.dumps(record,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    write()
    if args.inventory_only:
        print(json.dumps(record));return 0
    maps=Path(args.maps_dir or environment.get('FSIM_MAPS_DIR') or root/'assets/maps').resolve()
    if '.codex' in {p.casefold() for p in maps.parts} or not (maps/'s3.amazonaws.com/elevation-tiles-prod/terrarium').is_dir():
        raise ValueError('The existing project terrain map set is required; pass --maps-dir explicitly for a clean export')
    environment['FSIM_MAPS_DIR']=str(maps)
    record['maps_dir']=str(maps)
    production=production_preflight(root,args.preset,args.production,args.git_source_root)
    production_path=out/'production-input-binding.json'
    production_path.write_text(json.dumps(production,indent=2)+'\n',encoding='utf-8')
    record['production_input_binding_sha256']=sha(production_path)
    protected=set(production['protected_output_inputs'])
    binding=fingerprint(root,args.preset,data,protected)
    binding_path=out/'source-binary-binding.json'
    binding_path.write_text(json.dumps(binding,indent=2)+'\n',encoding='utf-8')
    record['source_binary_binding_sha256']=sha(binding_path)
    bridge=git_bridge(root,args.git_source_root,out,environment)
    record['git_only_method_bridge']=bridge
    write()
    junit.unlink(missing_ok=True)
    with (out/'raw.log').open('wb') as stream:
        process=subprocess.Popen(command,cwd=root,env=environment,stdout=stream,stderr=subprocess.STDOUT)
        try:
            record['exit_code']=process.wait(timeout=10800)
        except subprocess.TimeoutExpired:
            subprocess.run(['taskkill','/PID',str(process.pid),'/T','/F'],stdout=stream,stderr=subprocess.STDOUT,timeout=30)
            process.wait(timeout=30)
            record.update(exit_code=124,operational_timeout=True)
    if junit.exists():
        cases=ET.parse(junit).getroot().findall('.//testcase')
        actual=[c.get('name') for c in cases]
        record.update(executed=len(cases),inventory_reconciled=len(actual)==len(selected) and set(actual)==set(selected),
            failed=[c.get('name') for c in cases if c.find('failure') is not None or c.find('error') is not None],
            skipped=[c.get('name') for c in cases if c.find('skipped') is not None],junit_sha256=sha(junit))
        record['nested_methods']=nested_accounting(cases,policy,args.preset,bridge)
        record['fleet_coverage']=fleet_coverage(root,cases)
    current=fingerprint(root,args.preset,data,protected)
    record['derived_output_changes']=[dict(path=p,before=binding['derived_output_files'].get(p),after=current['derived_output_files'].get(p))
        for p in sorted(set(binding['derived_output_files'])|set(current['derived_output_files']))
        if binding['derived_output_files'].get(p)!=current['derived_output_files'].get(p)]
    record['rust_fixture_runtime']=rust_fixture_runtime(root,args.preset,data)
    record['source_changes']=[p for p in sorted(set(binding['source_files'])|set(current['source_files'])) if current['source_files'].get(p)!=binding['source_files'].get(p)]
    record['runtime_changes']=[p for p in sorted(set(binding['runtime_files'])|set(current['runtime_files'])) if current['runtime_files'].get(p)!=binding['runtime_files'].get(p)]
    record['configuration_changes']=[key for key in ('cmake_cache_sha256','compile_commands_sha256') if current[key]!=binding[key]]
    record['passed']=(not record['configuration_changes'] and record.get('exit_code')==0 and record.get('inventory_reconciled',False)
        and (record['rust_fixture_runtime'] is None or record['rust_fixture_runtime']['passed'])
        and not record.get('failed') and not record.get('skipped') and not record['source_changes'] and not record['runtime_changes']
        and not record.get('nested_methods',{}).get('unexpected_skips',True) and record.get('fleet_coverage',{}).get('passed',False))
    write();print(json.dumps(record))
    return 0 if record['passed'] else 1


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preset',choices=('ucrt64-headless','ucrt64-release'),required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--source-root')
    parser.add_argument('--production',required=True,help='Fresh complete generation epoch; binds source, profiles, runtime products and current intermediate caches')
    parser.add_argument('--powershell')
    parser.add_argument('--git-source-root',help='The original main checkout for the unchanged Git-only export method')
    parser.add_argument('--maps-dir',help='Existing project map set, shared as runtime reference data only')
    parser.add_argument('--jobs',type=int,choices=range(1,17),default=16)
    parser.add_argument('--inventory-only',action='store_true')
    raise SystemExit(main(parser.parse_args()))
