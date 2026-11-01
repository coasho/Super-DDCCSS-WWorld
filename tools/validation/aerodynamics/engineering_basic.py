"""One bounded basic-usability pass over all generated native/installed designs.

Short execution only; no envelope sweep, fitting, full regression or retry loop.
"""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools/models/hangar'))
from hangar import engineering as e


def main(args):
    started=time.monotonic()
    for value in (args.output,args.production):
        if '.codex' in {part.casefold() for part in Path(value).parts}:raise ValueError('Excluded path')
    out=Path(args.output).resolve();production=Path(args.production).resolve()
    for path in (out,production):
        if '.codex' in {part.casefold() for part in path.parts} or not path.is_relative_to(ROOT/'build'):
            raise ValueError('Basic evidence must remain in the owned build tree')
    out.mkdir(exist_ok=True,parents=True)
    matrix=json.loads((production/'matrix.json').read_text(encoding='utf-8'))
    value=json.loads((production/'installed-production-manifest.json').read_text(encoding='utf-8'))['prefix']
    if '.codex' in {part.casefold() for part in Path(value).parts}:raise ValueError('Excluded SDK prefix')
    prefix=Path(value).resolve()
    if not prefix.is_relative_to(Path('D:/ENV')):raise ValueError('SDK prefix must remain under D:/ENV')
    declared={r['name']:r for r in e.roster()}
    if len(matrix['designs'])!=88 or {r['name'] for r in matrix['designs']}!=set(declared):raise ValueError('Incomplete basic-usability roster')
    for row in matrix['designs']:
        category=declared[row['name']]['category']
        expected=Path('stores')/row['name'] if category=='store' else Path(row['name'])
        if row['category']!=category or Path(row['product_directory'])!=expected:
            raise ValueError('Unexpected consumer product path: '+row['name'])
    env=os.environ.copy();env.update(TMP='D:/ENV/tmp',TEMP='D:/ENV/tmp',TORCH_EXTENSIONS_DIR='D:/ENV/tmp',
        OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1')
    shell=env.get('FSIM_BUILD_POWERSHELL') or shutil.which('pwsh') or shutil.which('powershell')
    if not shell:raise ValueError('Use the existing UCRT64 launcher PowerShell host')
    commands=[];failures=[];counts=dict(designs=0,native_consumers=0,installed_consumers=0,
        short_flight_samples=0,released_store_bodies=0,envelope_sampling_conditions=0)
    shutil.copy2(production/'matrix.json',out/'input-production-matrix.json')
    binding=dict(wrapper_sha256=e.sha(__file__),probe_sha256=e.sha(ROOT/'tools/models/hangar/hangar/engineering_runtime.py'),
        installed_manifest_sha256=e.sha(production/'installed-production-manifest.json'),
        input_matrix_sha256=e.sha(out/'input-production-matrix.json'),
        generation_receipts={row['name']:e.sha(production/row['name']/'generation.json') for row in matrix['designs']})
    def run(name,argv,environment=env,cwd=ROOT,timeout=180):
        tick=time.monotonic();log=out/(name+'.log')
        with log.open('wb') as stream:
            p=subprocess.Popen(list(map(str,argv)),cwd=cwd,env=environment,stdout=stream,stderr=subprocess.STDOUT)
            try:code=p.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                subprocess.run(['taskkill','/PID',str(p.pid),'/T','/F'],stdout=stream,stderr=subprocess.STDOUT,timeout=15)
                code=p.wait(timeout=15)
        commands.append(dict(name=name,argv=list(map(str,argv)),cwd=str(cwd),log=str(log),exit_code=code,elapsed_seconds=time.monotonic()-tick))
        return code
    build=ROOT/'build/ucrt64-release';probe=ROOT/'tools/models/hangar/hangar/engineering_runtime.py'
    work=Path('D:/ENV/tmp/sim-m02-consumer');work.mkdir(exist_ok=True,parents=True)
    def restore(originals):
        errors=[]
        for target,original in originals.items():
            try:
                if not target.is_file() or target.read_bytes()!=original:target.write_bytes(original)
            except OSError as error:errors.append('XML restoration failed/'+str(target)+': '+str(error))
        return errors
    batches={}
    if args.consumer_execution=='batched-v1':
        batch_probe=Path(__file__).with_name('engineering_consumer_batch.py')
        binding.update(consumer_execution='batched-v1',batch_worker_sha256=e.sha(batch_probe))
        for mode,sdk,data in [('native',build/'python',build/'share/flightsim'),('installed',prefix,prefix/'share/flightsim')]:
            worker=env.copy()
            for key in ('PYTHONPATH','FSIM_DLL_DIR','FSIM_DATA_DIR','FSIM_AIRCRAFT_PATH','JSBSIM_ROOT'):worker.pop(key,None)
            worker.update(FSIM_AIRCRAFT_PATH=str(data/'aircraft')+';'+str(data/'aircraft/stores'),
                FSIM_DLL_DIR=str(sdk/'fsim/bin'),PATH=str(sdk/'fsim/bin')+';'+os.environ.get('SystemRoot','C:/Windows')+'/System32')
            manifest=out/(mode+'-batch.json')
            manifest.unlink(missing_ok=True);run_id=uuid.uuid4().hex
            originals={data/'aircraft'/row['product_directory']/(row['name']+'.xml'):None for row in matrix['designs']}
            originals={path:path.read_bytes() for path in originals}
            try:
                code=run(mode+'-batch',[sys.executable,'-I','-B',batch_probe,'--production',production,
                    '--sdk',sdk,'--data',data,'--probe',probe,'--mode',mode,'--output',manifest,'--run-id',run_id],worker,work,timeout=180*88)
            finally:
                failures.extend(dict(mode=mode,error=error) for error in restore(originals))
            try:
                batch=json.loads(manifest.read_text(encoding='utf-8'))
                if (batch.get('schema')!='fsim.sim-m02.consumer-batch.v1' or batch.get('run_id')!=run_id
                    or batch.get('mode')!=mode or Path(batch['sdk'])!=sdk or Path(batch['data'])!=data
                    or batch.get('probe_sha256')!=e.sha(probe) or batch.get('worker_sha256')!=e.sha(batch_probe)):
                    raise ValueError('Consumer batch provenance mismatch')
                records=batch['records']
                if (len(records)>88 or len({r['name'] for r in records})!=len(records)
                    or any(r['name'] not in {row['name'] for row in matrix['designs']} or type(r['exit_code']) is not int for r in records)):
                    raise ValueError('Consumer batch record identities differ from declared roster')
            except (ValueError,KeyError,TypeError,OSError) as error:
                failures.append(dict(mode=mode,error='Invalid current batch manifest: '+str(error)));batch={'records':[]}
            batches[mode]={record['name']:record for record in batch['records']}
            if code or set(batches[mode])!={row['name'] for row in matrix['designs']}:
                failures.append(dict(mode=mode,error='Incomplete or failed isolated consumer batch',exit_code=code))
        binding['batch_manifests']={mode:e.sha(out/(mode+'-batch.json')) for mode in batches if (out/(mode+'-batch.json')).is_file()}
    for row in matrix['designs']:
        folder=production/row['name'];row['runtime']={};errors=[]
        for mode,sdk,data in [('native',build/'python',build/'share/flightsim'),('installed',prefix,prefix/'share/flightsim')]:
            worker=env.copy()
            for key in ('PYTHONPATH','FSIM_DLL_DIR','FSIM_DATA_DIR','FSIM_AIRCRAFT_PATH','JSBSIM_ROOT'):worker.pop(key,None)
            worker.update(FSIM_AIRCRAFT_PATH=str(data/'aircraft')+';'+str(data/'aircraft/stores'),FSIM_DLL_DIR=str(sdk/'fsim/bin'),
                PATH=str(sdk/'fsim/bin')+';'+os.environ.get('SystemRoot','C:/Windows')+'/System32')
            argv=[sys.executable,'-I','-B',probe,'--sdk',sdk,'--data',data,'--receipt',folder/'generation.json','--output',folder/(mode+'.json')]
            target=data/'aircraft'/row['product_directory']/(row['name']+'.xml');original=target.read_bytes()
            try:
                if args.consumer_execution=='batched-v1':
                    observed=batches.get(mode,{}).get(row['name'],{})
                    result=folder/(mode+'.json')
                    code=observed.get('exit_code',1)
                    if not result.is_file() or e.sha(result)!=observed.get('output_sha256'):code=1
                else:code=run(row['name']+'-'+mode,argv,worker,work)
            finally:
                errors.extend(restore({target:original}))
            if code:
                errors.append(mode+' basic consumer exited '+str(code));continue
            value=json.loads((folder/(mode+'.json')).read_text(encoding='utf-8'));row['runtime'][mode]=value
            errors.extend(mode+'/'+c['name'] for c in value['checks'] if not c['passed'])
            counts[mode+'_consumers']+=1
            counts['short_flight_samples']+=sum(len(trace['rows']) for trace in value['traces'])
            counts['released_store_bodies']+=bool(value.get('release'))
        counts['designs']+=1
        row['basic_usability']=dict(passed=not errors,errors=errors,scope='Three short untrimmed command cases, finite six-axis execution, reset/reconstruction and release where installed; no envelope sweep')
        row['runtime_evidence_scope']='Current basic native and installed consumer execution; no envelope sampling or full regression'
        row['reasons']=[c['name'] for c in row['checks'] if not c['passed']]+errors
        row['status']=e.engineering_status(row['reasons'])
        if errors:failures.append(dict(design=row['name'],errors=errors))
        e.dump(folder/'result.json',row)
        print('basic '+row['name']+' '+('passed' if not errors else 'failed'),flush=True)
    # Existing native component fixtures only. Original scenarios/assertions and
    # timeouts remain unchanged. No GUI fixture or competing writer is selected.
    selected=['contact_fuze_native','payload_native','weapon_guidance','fuze','missile_autopilot',
              'payload_plant_adapters','payload_effect_adapters','payload_components','weapon_employment_core','package_consumer']
    junit=out/'basic-native.junit.xml'
    code=run('basic-native',[shell,'-NoProfile','-ExecutionPolicy','Bypass','-File',ROOT/'tools/launch/fsim.ps1',
        '--preset','ucrt64-release','test','--no-tests=error','-j','1','--output-on-failure','--output-junit',junit,
        '-R','^('+'|'.join(selected)+')$'],timeout=600)
    doc=ET.parse(junit);cases=doc.findall('.//testcase')
    if {c.get('name') for c in cases}!=set(selected):raise ValueError('Basic native fixtures did not execute exactly the declared nonzero set')
    native=dict(executed=len(cases),failed=sum(c.find('failure') is not None or c.find('error') is not None for c in cases),
        skipped=sum(c.find('skipped') is not None for c in cases),junit=str(junit),exit_code=code)
    if code or native['failed'] or native['skipped']:failures.append(dict(native_components=native))
    e.publish(production,matrix['designs'],e.roster())
    index=production/'index.html'
    index.write_text(index.read_text(encoding='utf-8').replace(
        'No native dynamic sampling or consumer execution was performed in this production-only delivery.',
        'Basic native and installed consumer execution was performed separately; no envelope sweep or full regression was run.'),encoding='utf-8')
    binding['output_matrix_sha256']=e.sha(production/'matrix.json')
    binding['runtime_results']={row['name']+'/'+mode+'.json':e.sha(production/row['name']/(mode+'.json'))
        for row in matrix['designs'] for mode in ('native','installed') if (production/row['name']/(mode+'.json')).is_file()}
    result=dict(schema='fsim.sim-m02.basic-usability.v1',elapsed_seconds=time.monotonic()-started,
        counts=counts,native_components=native,failures=failures,commands=commands,binding=binding,
        scope='All88 native/installed basic usability and declared core component fixtures. No full regression or dynamic full-envelope claim.')
    e.dump(out/'basic-usability.json',result)
    print(json.dumps(dict(counts=counts,native_components=native,failed=len(failures),elapsed_seconds=result['elapsed_seconds']),indent=2),flush=True)
    return int(bool(failures))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--production',required=True);p.add_argument('--output',required=True)
    p.add_argument('--consumer-execution',choices=('isolated-per-design','batched-v1'),default='isolated-per-design')
    raise SystemExit(main(p.parse_args()))
