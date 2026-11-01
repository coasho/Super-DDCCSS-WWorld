"""Refresh declared simulated component inputs without a flight-fitness suite.

This preparation performs the existing native reference acquisition only. It
does not run rotor manoeuvres, fly.run(), performance tests or write_hover().
All required roles and the existing reference-trim condition remain unchanged.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools/models/hangar'))


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def worker(name,out):
    from hangar.engineering import roster
    from hangar.rotorcraft import Rotorcraft,fly,tail_damping
    record=next(r for r in roster() if r['name']==name and r['category']=='rotorcraft')
    r=Rotorcraft(record['path'])
    if tail_damping.declaration(r.spec) is None:raise ValueError('No declared component to refresh')
    target=Path(r.dir)/'tail-damping.json'
    before=sha(target) if target.is_file() else None
    if target.is_file():shutil.copyfile(target,out/'source-reference-before.json')
    hover=Path(r.dir)/'hover.toml';hover_before=sha(hover) if hover.is_file() else None
    r.build()
    directory=Path(r.dir);xml=directory/(r.name+'.xml')
    acquisition={}
    for source in [xml,*sorted((directory/'Engines').glob('*.xml'))]:
        relative=source.relative_to(directory);saved=out/'bootstrap'/relative
        saved.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(source,saved)
        acquisition[relative.as_posix()]=dict(path=str(saved),sha256=sha(saved))
    contacts=ET.parse(xml).getroot().findall('ground_reactions/contact')
    session=fly.Session(r)
    try:
        trim,vehicle=session.trim(0.)
        if trim is None or not trim['steady']:
            raise RuntimeError('Required component reference did not meet the unchanged native steady-trim condition')
        state=vehicle.state
        contact_wow={}
        for index,contact in enumerate(contacts):
            prefix='gear' if contact.get('type','BOGEY').upper()=='BOGEY' else 'contact'
            prop=prefix+'/unit['+str(index)+']/WOW'
            contact_wow[prop]=vehicle.get_property(prop)
        certificate=dict(crashed=bool(state.crashed),diverged=bool(state.diverged),on_ground=bool(state.on_ground),
            altitude_msl_m=float(state.altitude_msl_m),altitude_agl_m=float(state.altitude_agl_m),
            euler_rad=list(state.euler_rad),contact_wow=contact_wow,trim=trim)
        certificate['airborne_valid']=(not any(certificate[k] for k in ('crashed','diverged','on_ground'))
            and math.isfinite(certificate['altitude_agl_m']) and certificate['altitude_agl_m']>0
            and all(math.isfinite(v) and v==0 for v in contact_wow.values()))
        if not certificate['airborne_valid']:
            (out/'invalid-reference-state.json').write_text(json.dumps(certificate,indent=2)+'\n',encoding='utf-8')
            raise RuntimeError('A declared hover component reference must be airborne, not ground-supported/crashed/diverged')
        reference=tail_damping.identify(r,vehicle)
        reference['acquisition_certificate']=certificate
        target.write_text(json.dumps(reference,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    finally:session.close()
    r.build()
    if (sha(hover) if hover.is_file() else None)!=hover_before:
        raise ValueError('Reference preparation changed the deferred rotor controller identification')
    return dict(name=name,method='Existing native bootstrap trim and tail_damping.identify',
        basis='estimated',operating_point_basis='simulated',before_sha256=before,
        after_sha256=sha(target),reference=reference,bootstrap_files=acquisition,hover_identification_unchanged=True,
        functional_fitness_test_executed=False)


def main(args):
    from hangar.engineering import roster
    from hangar.rotorcraft import Rotorcraft,tail_damping
    out=Path(args.output).resolve()
    if not out.is_relative_to(ROOT/'build') or '.codex' in {p.casefold() for p in out.parts}:
        raise ValueError('Preparation evidence must remain in the owned build tree')
    out.mkdir(parents=True,exist_ok=True)
    if args.worker_design:
        value=worker(args.worker_design,out)
        (out/'reference.json').write_text(json.dumps(value,indent=2,allow_nan=False)+'\n',encoding='utf-8')
        return 0
    started=time.monotonic();results=[]
    env=os.environ.copy()
    env.update(TMP='D:/ENV/tmp',TEMP='D:/ENV/tmp',TORCH_EXTENSIONS_DIR='D:/ENV/tmp',
        OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1',
        PYTHONPATH=os.pathsep.join(map(str,[ROOT/'build/ucrt64-release/python',ROOT/'tools/models/hangar',ROOT/'.local/packages'])),
        FSIM_AIRCRAFT_PATH=os.pathsep.join(map(str,[ROOT/'aircraft',ROOT/'aircraft/stores'])),
        FSIM_DLL_DIR=str(ROOT/'build/ucrt64-release/python/fsim/bin'))
    for record in roster():
        if record['category']!='rotorcraft':continue
        r=Rotorcraft(record['path']);declaration=tail_damping.declaration(r.spec)
        if declaration is None:continue
        path=Path(r.dir)/'tail-damping.json'
        if path.is_file():
            previous=json.loads(path.read_text(encoding='utf-8'))
            if (previous.get('schema')==tail_damping.VERSION and previous.get('source_hashes')==tail_damping.source_hashes(r.path)
                    and previous.get('acquisition_certificate',{}).get('airborne_valid') is True):
                results.append(dict(name=r.name,refreshed=False,source_reference_current=True));continue
        folder=out/r.name;folder.mkdir(exist_ok=True)
        command=[sys.executable,'-B',str(Path(__file__)), '--worker-design',r.name,'--output',str(folder)]
        tick=time.monotonic()
        with (folder/'raw.log').open('wb') as log:
            process=subprocess.Popen(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT)
            try:code=process.wait(timeout=300)
            except subprocess.TimeoutExpired:
                subprocess.run(['taskkill','/PID',str(process.pid),'/T','/F'],stdout=log,stderr=subprocess.STDOUT,timeout=15)
                process.wait(timeout=15);code=124
        results.append(dict(name=r.name,refreshed=code==0,exit_code=code,command=command,elapsed_seconds=time.monotonic()-tick))
        if code:break
    report=dict(scope='Declared component input preparation, not rotor functional acceptance',results=results,
        elapsed_seconds=time.monotonic()-started,passed=all(r.get('exit_code',0)==0 for r in results))
    (out/'component-preparation.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report))
    return int(not report['passed'])


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True)
    parser.add_argument('--worker-design',help=argparse.SUPPRESS)
    raise SystemExit(main(parser.parse_args()))
