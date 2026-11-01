"""Replay an applied source patch through generation and complete native gates.

One entry point owns sequential configure/build/install/test operations. The
aggregate clock includes the full regressions; the narrower generation/native
campaign clock is reported separately. Failed gates never become a pass.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time


def read(path):return json.loads(Path(path).read_text(encoding='utf-8'))
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main(args):
    start=time.monotonic()
    if not 1200<=args.timeout_seconds<=14400:raise ValueError('Operational clean-replay timeout must be bounded1200..14400seconds')
    root=Path(args.source).resolve();out=Path(args.output).resolve();out.mkdir(parents=True,exist_ok=True)
    prefix=Path(args.prefix).resolve();manifest=read(args.manifest)
    for path in (root,out,prefix):
        if '.codex' in {p.casefold() for p in path.parts}:raise ValueError('Excluded path')
    if (root/'.git').exists() or str(root)!=manifest['source_export']:
        raise ValueError('Expected the actual ordinary archive-plus-patch export')
    if manifest['byte_verification_failures'] or not manifest['real_index_unchanged']:
        raise ValueError('Patch application not verified')
    if not prefix.is_relative_to(Path('D:/ENV')):raise ValueError('Install prefix must be under D:/ENV')
    if prefix.exists() and any(prefix.iterdir()):raise ValueError('Clean replay requires a new empty installed prefix')
    for entry in manifest['files']:
        path=root/entry['path']
        if entry.get('deleted'):
            if path.exists():raise ValueError('Unexpected retained source: '+entry['path'])
        elif not path.is_file() or sha(path)!=entry['sha256']:
            raise ValueError('Source export drift before execution: '+entry['path'])
    env=os.environ.copy()
    for key in ('PYTHONPATH','FSIM_DLL_DIR','FSIM_DATA_DIR','FSIM_AIRCRAFT_PATH','JSBSIM_ROOT','HANGAR_MESHKIT_DIR'):env.pop(key,None)
    env.update(TMP='D:/ENV/tmp',TEMP='D:/ENV/tmp',TORCH_EXTENSIONS_DIR='D:/ENV/tmp',
        MPLCONFIGDIR='D:/ENV/cache/matplotlib',XDG_CACHE_HOME='D:/ENV/cache',
        OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1')
    if args.maps:
        maps=Path(args.maps).resolve()
        terrain=maps/'s3.amazonaws.com/elevation-tiles-prod/terrarium'
        if not terrain.is_dir():raise ValueError('Missing actual terrain map dependency')
        env['FSIM_MAPS_DIR']=str(maps)
        inputs=[]
        for here,dirs,names in os.walk(terrain,followlinks=False):
            dirs[:]=sorted(d for d in dirs if d.casefold()!='.codex' and not (Path(here)/d).is_symlink())
            for name in sorted(names):
                path=Path(here)/name
                if path.is_file() and not path.is_symlink():inputs.append(dict(path=path.relative_to(maps).as_posix(),sha256=sha(path),bytes=path.stat().st_size))
        if not inputs:raise ValueError('Empty terrain dependency cannot satisfy map tests')
        (out/'map-dependency.json').write_text(json.dumps(dict(root=str(maps),read_only=True,files=inputs),indent=2)+'\n',encoding='utf-8')
    shell=env.get('FSIM_BUILD_POWERSHELL') or shutil.which('pwsh')
    if not shell:raise ValueError('Use the existing launcher PowerShell host')
    launcher=[shell,'-NoProfile','-ExecutionPolicy','Bypass','-File',str(root/'tools/launch/fsim.ps1'),'--preset','ucrt64-release']
    stages=[];failures=[];source_drift=[];restorations=[]
    report_files=('freshness-and-counts.json','table-method-inventory.json','table-method-inventory.csv','native-table-inventory.json','retained-profile-inputs.json')
    previous=out/'intermediate-eight-worker-audit'
    previous.mkdir(exist_ok=True)
    for filename in report_files:
        old=out/filename
        if old.is_file():
            archived=previous/filename
            if archived.exists() and sha(archived)!=sha(old):raise ValueError('Historical report archive collision: '+filename)
            shutil.copy2(old,archived);old.unlink()
    def save():
        value=dict(source_export=str(root),base_head=manifest['base_head'],patch_sha256=manifest['patch_sha256'],
            elapsed_seconds=time.monotonic()-start,reference_target_seconds=1200,
            exceeds_twenty_minutes=time.monotonic()-start>1200,operational_timeout_seconds=args.timeout_seconds,
            stages=stages,failed_gates=failures,unexpected_source_drift=source_drift,restored_runtime_xml=restorations,
            scope='Entire clean replay including configure/build, forced all-design generation, full-table reload audits and unfiltered headless/Release regressions; no selected-gate substitution')
        (out/'clean-pipeline.json').write_text(json.dumps(value,indent=2)+'\n',encoding='utf-8')
    def restore_owned_xml():
        plan=out/'replay-final/batch-plan.json'
        if not plan.is_file():return
        names=[name for batch in read(plan)['batches'] for name in batch['designs']]
        for name in names:
            if not name or not all(c.isascii() and (c.isalnum() or c in '_-') for c in name):raise ValueError('Unexpected declared design identity')
            receipt=out/'replay-final'/name/'generation.json'
            if not receipt.is_file():continue
            for relative,expected in read(receipt)['products'].items():
                relative=Path(relative)
                if relative.is_absolute() or '..' in relative.parts or '.codex' in {p.casefold() for p in relative.parts}:raise ValueError('Excluded receipt product')
                if relative.name!=name+'.xml':continue
                source=root/'aircraft'/relative
                if not source.resolve().is_relative_to(root/'aircraft') or not source.is_file() or sha(source)!=expected:continue
                for data in (root/'build/ucrt64-release/share/flightsim',prefix/'share/flightsim'):
                    target=data/'aircraft'/relative
                    if target.is_file() and sha(target)!=expected:
                        target.write_bytes(source.read_bytes());restorations.append(str(target))
    def run(name,command,environment=env,permitted=(0,)):
        tick=time.monotonic();log=out/(name+'.log');record=dict(name=name,argv=[str(x) for x in command],cwd=str(root),log=str(log))
        with log.open('wb') as stream:
            process=subprocess.Popen(record['argv'],cwd=root,env=environment,stdout=stream,stderr=subprocess.STDOUT)
            try:code=process.wait(timeout=max(1,args.timeout_seconds-(time.monotonic()-start)))
            except subprocess.TimeoutExpired:
                subprocess.run(['taskkill','/PID',str(process.pid),'/T','/F'],stdout=stream,stderr=subprocess.STDOUT,timeout=15)
                process.wait(timeout=15);code=-1;record['timeout']=True
        record.update(exit_code=code,elapsed_seconds=time.monotonic()-tick);stages.append(record)
        if code:failures.append(dict(stage=name,exit_code=code,log=str(log)))
        save();print(name,code,round(record['elapsed_seconds'],3),flush=True)
        if code not in permitted:raise RuntimeError('Required gate failed: '+name)
    try:
        configure=launcher+['configure','-DFSIM_PYTHON_EXECUTABLE='+sys.executable,'-DFSIM_CHECK_PYTHON='+sys.executable,
            '-DFSIM_BUILD_PYTHON=ON','-DFSIM_BUILD_TESTS=ON','-DCMAKE_PREFIX_PATH='+str(Path(args.dependencies).resolve())]
        run('clean-release-configure',configure)
        run('clean-release-build',launcher+['build','--parallel','8'])
        generation_env=env.copy();generation_env['PYTHONPATH']=str(root/'tools/models/hangar')
        campaign=[sys.executable,'-B','-m','hangar','engineering','campaign','--cohort','all',
            '--generation-workers',str(args.generation_workers),'--geometry-threads',str(args.geometry_threads),'--output',out/'replay-final','--prefix',prefix]
        if args.scheduling_profile:
            profile=Path(args.scheduling_profile)
            if profile.is_absolute() or '..' in profile.parts or '.codex' in {p.casefold() for p in profile.parts}:
                raise ValueError('Scheduling profile must be a source-relative replay input')
            campaign.extend(['--scheduling-profile',root/profile])
        run('clean-campaign',campaign,generation_env,permitted=(0,1))
        matrix=read(out/'replay-final/matrix.json')
        if matrix['completed_count']!=88:raise ValueError('Incomplete clean generation roster')
        (out/'canonical-epoch.json').write_text(json.dumps(dict(directory='replay-final',prefix=str(prefix),
            campaign_stage='clean-campaign',audit_stages=['clean-pre-regression-audit','clean-post-regression-audit']),indent=2)+'\n',encoding='utf-8')
        audit=[sys.executable,'-B',str(root/'tools/validation/aerodynamics/engineering_delivery.py'),
            '--matrix',out/'replay-final','--prefix',prefix,'--output',out]
        if (out/'baseline/matrix.json').is_file():audit.extend(['--baseline',out/'baseline'])
        run('clean-pre-regression-audit',audit,permitted=(0,1))
        if (out/'freshness-and-counts.json').is_file():shutil.copy2(out/'freshness-and-counts.json',out/'clean-pre-regression-freshness-and-counts.json')
        verification=[sys.executable,'-B',str(root/'tools/validation/aerodynamics/engineering_verify.py'),
            '--source',root,'--output',out/'regression-replay','--dependencies',args.dependencies,'--jobs',str(args.jobs)]
        if manifest.get('source_checkout'):
            verification.extend(['--git-checkout',manifest['source_checkout'],'--expected-head',manifest['base_head']])
        run('clean-full-verification',verification,permitted=(0,1))
        run('clean-post-regression-audit',audit,permitted=(0,1))
        allowed={str(Path('aircraft')/p).replace('\\','/') for r in matrix['designs'] for p in r['products']}
        allowed.update('assets/stores/'+r['store_card']['id']+'.json' for r in matrix['designs'] if r.get('store_card'))
        for entry in manifest['files']:
            path=root/entry['path']
            if entry['path'] in allowed:continue
            if entry.get('deleted'):
                if path.exists():source_drift.append(entry['path'])
            elif not path.is_file() or sha(path)!=entry['sha256']:source_drift.append(entry['path'])
        if source_drift:failures.append(dict(stage='source-freeze',paths=source_drift))
    except Exception as error:
        try:restore_owned_xml()
        except Exception as restore_error:failures.append(dict(stage='runtime-restoration',error=str(restore_error)))
        failures.append(dict(stage='pipeline',error=str(error)));save();raise
    save()
    return int(bool(failures) or bool(matrix['counts']['blocked']))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',required=True);p.add_argument('--manifest',required=True)
    p.add_argument('--output',required=True);p.add_argument('--prefix',required=True);p.add_argument('--dependencies',required=True)
    p.add_argument('--generation-workers',type=int,default=16);p.add_argument('--jobs',type=int,default=16)
    p.add_argument('--geometry-threads',type=int,default=1)
    p.add_argument('--timeout-seconds',type=int,default=7200,help='Bounded operational ceiling for all builds, generation and both complete regressions; performance target remains1200seconds')
    p.add_argument('--maps',help='Existing read-only map set, fingerprinted as an external replay dependency')
    p.add_argument('--scheduling-profile',help='Source-relative versioned generation cost profile; no default scheduling override')
    raise SystemExit(main(p.parse_args()))
