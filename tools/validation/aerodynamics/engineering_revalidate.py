"""Revalidate unchanged generated products after an execution-only native repair.

Historical generation receipts and source hashes remain intact. A separate
execution binding names the current DLL. No aerodynamic regeneration is claimed.
Every authored input/product is rehashed before the installed SDK is executed.
"""
import argparse
import copy
import json
import os
from pathlib import Path
import shutil
import sys
import time

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools/models/hangar'))
from hangar import engineering as e


def checked_path(root,relative):
    relative=Path(relative)
    if relative.is_absolute() or '..' in relative.parts or '.codex' in {x.casefold() for x in relative.parts}:
        raise ValueError('Invalid receipt path')
    return root/relative


def verify_generation(row,source):
    """Changed table-generation code is never silently reused."""
    for relative,expected in row['input_sha256'].items():
        if e.sha(checked_path(ROOT,relative))!=expected:raise ValueError('Changed authored input: '+relative)
    writers=[]
    for relative,expected in row['generator_sha256'].items():
        if e.sha(checked_path(source,relative))!=expected:raise ValueError('Historical generation source drift: '+relative)
        current=e.sha(checked_path(ROOT,relative))
        if current==expected:continue
        # This source-input writer is not called by engineering.generate. Its
        # generated TOML is already checked byte-for-byte as an authored input.
        # Check every supported output as well, so input reconstruction remains
        # reproducible. Aerodynamic solvers/writers may never take this path.
        if Path(relative).as_posix()!='tools/models/hangar/hangar/storedesigns.py':
            raise ValueError('Changed table-generation source requires regeneration: '+relative)
        from hangar import storedesigns
        for name in storedesigns.cards():
            text,paint=storedesigns.design(name)
            for filename,wanted in ((name+'.toml',text),('paint.toml',paint)):
                if (ROOT/'aircraft'/name/filename).read_text(encoding='utf-8')!=wanted:
                    raise ValueError('Reconstructed authored source differs: '+name+'/'+filename)
        writers.append(dict(path=relative,generation_sha256=expected,current_sha256=current,
            reason='Source-input writer only; every current generated TOML/paint output independently equals its authored input'))
    for relative,expected in row['products'].items():
        if e.sha(checked_path(ROOT/'aircraft',relative))!=expected:raise ValueError('Changed generated product: '+relative)
    card=row.get('store_card')
    if card and e.sha(ROOT/'assets/stores'/(card['id']+'.json'))!=card['sha256']:raise ValueError('Changed store card')
    return writers


def main(args):
    started=time.monotonic();old=Path(args.evidence).resolve();source=Path(args.generation_source).resolve()
    out=Path(args.output).resolve();prefix=Path(args.prefix).resolve()
    for path in (old,source,out,prefix):
        if '.codex' in {p.casefold() for p in path.parts}:raise ValueError('Excluded path')
    if out.exists():raise ValueError('Execution evidence epoch must be new')
    if not prefix.is_relative_to(Path('D:/ENV')) or prefix.exists():raise ValueError('A new D:/ENV install prefix is required')
    matrix=json.loads((old/'matrix.json').read_text(encoding='utf-8'))
    roster=e.roster()
    if {x['name'] for x in matrix['designs']}!={x['name'] for x in roster}:raise ValueError('Incomplete roster')
    out.mkdir(parents=True);rows=[];commands=[]
    # Finish verification before the first install mutation.
    for row in matrix['designs']:
        changes=verify_generation(row,source)
        receipt=old/row['name']/'generation.json'
        original=json.loads(receipt.read_text(encoding='utf-8'))
        for key in ('products','input_sha256','generator_sha256','native_dll_sha256'):
            if row[key]!=original[key]:raise ValueError('Matrix/receipt mismatch: '+row['name']+'/'+key)
        folder=out/row['name'];folder.mkdir()
        shutil.copy2(receipt,folder/'source-generation.json')
        reference=old/row['name']/row['reference_bundle']['name']
        if e.sha(reference)!=row['reference_bundle']['sha256']:raise ValueError('Changed reference bundle')
        shutil.copy2(reference,folder/reference.name)
        value=copy.deepcopy(row);value['runtime']={};value.pop('envelope',None)
        value['execution_binding']=dict(schema='fsim.sim-m02.execution-binding.v1',
            generation_source_export=str(source),source_receipt=str(receipt),source_receipt_sha256=e.sha(receipt),
            original_native_dll_sha256=original['native_dll_sha256'],reconstructed_input_writers=changes,
            scope='Unchanged geometry/card/table products reused; current native/installed execution rerun. Original generation receipt retained.')
        value['native_dll_sha256']=e.sha(ROOT/'build/ucrt64-release/python/fsim/bin/libfsim.dll')
        value['execution_binding']['native_dll_sha256']=value['native_dll_sha256']
        e.dump(folder/'generation.json',value);rows.append(value)
    env=os.environ.copy();env.update(TMP='D:/ENV/tmp',TEMP='D:/ENV/tmp',TORCH_EXTENSIONS_DIR='D:/ENV/tmp',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    def run(command,log,cwd=ROOT,environment=env):
        commands.append(dict(argv=list(map(str,command)),log=str(log),cwd=str(cwd)))
        e.run_command(list(map(str,command)),log,7200-(time.monotonic()-started),cwd,environment)
    cmake=Path('D:/ENV/DevLanguages/Cpp/msys2/ucrt64/bin/cmake.exe');build=ROOT/'build/ucrt64-release'
    install_started=time.monotonic()
    for component in ('sdk','python'):
        run([cmake,'--install',build,'--component',component,'--prefix',prefix],out/('install-'+component+'.log'))
    install_seconds=time.monotonic()-install_started
    probe=ROOT/'tools/models/hangar/hangar/engineering_runtime.py'
    work=Path('D:/ENV/tmp/sim-m02-consumer');work.mkdir(exist_ok=True,parents=True)
    validation_started=time.monotonic()
    for row in rows:
        tick=time.monotonic();folder=out/row['name'];row['commands']=[]
        worker_env=env.copy()
        for key in ('PYTHONPATH','FSIM_DLL_DIR','FSIM_DATA_DIR','FSIM_AIRCRAFT_PATH','JSBSIM_ROOT'):worker_env.pop(key,None)
        for mode,sdk,data in [('native',build/'python',build/'share/flightsim'),('installed',prefix,prefix/'share/flightsim')]:
            worker_env.update(FSIM_AIRCRAFT_PATH=str(data/'aircraft')+';'+str(data/'aircraft/stores'),
                FSIM_DLL_DIR=str(sdk/'fsim/bin'),PATH=str(sdk/'fsim/bin')+';'+os.environ.get('SystemRoot','C:/Windows')+'/System32')
            command=[sys.executable,'-I','-B',probe,'--sdk',sdk,'--data',data,'--receipt',folder/'generation.json','--output',folder/(mode+'.json')]
            if mode=='installed':command+=['--sampling-output',folder/'envelope.json']
            target=data/'aircraft'/row['product_directory']/(row['name']+'.xml');before=target.read_bytes()
            try:run(command,folder/(mode+'.log'),work,worker_env)
            finally:
                if target.read_bytes()!=before:target.write_bytes(before)
        row['runtime']={mode:json.loads((folder/(mode+'.json')).read_text(encoding='utf-8')) for mode in ('native','installed')}
        row['envelope']=json.loads((folder/'envelope.json').read_text(encoding='utf-8'))
        failed=[c['name'] for c in row['checks'] if not c['passed']]
        for mode,value in row['runtime'].items():failed += [mode+'/'+c['name'] for c in value['checks'] if not c['passed']]
        if row['envelope']['case_count']!=210 or row['envelope']['failures']:failed.append('required sampled native consistency')
        row.update(reasons=failed,status=e.engineering_status(failed),validation_wall_seconds=time.monotonic()-tick)
        row['elapsed_seconds']=row['generation_wall_seconds']+row['validation_wall_seconds']
        e.dump(folder/'result.json',row)
        print(row['name'],row['status'],flush=True)
    e.publish(out,rows,roster)
    shutil.copy2(old/'batch-plan.json',out/'batch-plan.json')
    timing=json.loads((old/'campaign-timing.json').read_text(encoding='utf-8'))
    e.dump(out/'source-campaign-timing.json',timing)
    e.dump(out/'campaign-timing.json',dict(schema='fsim.sim-m02.execution-revalidation.v1',design_count=len(rows),
        elapsed_seconds=time.monotonic()-started,single_install_phase_seconds=install_seconds,
        serial_native_validation_phase_seconds=time.monotonic()-validation_started,generation_phase_seconds=0.,
        generation_reused=True,source_generation_seconds=timing['elapsed_seconds'],source_evidence=str(old),commands=commands))
    return int(any(r['status']=='blocked' for r in rows))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--evidence',required=True);p.add_argument('--generation-source',required=True)
    p.add_argument('--output',required=True);p.add_argument('--prefix',required=True)
    raise SystemExit(main(p.parse_args()))
