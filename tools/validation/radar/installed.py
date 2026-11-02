"""Install a fresh SDK/Python product, verify provenance and run M01 consumers.

Uses the current UCRT64 preset output and existing toolchain, sequentially.
The prefix must not exist: a previous package cannot satisfy this run.
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

ROOT=Path(__file__).resolve().parents[3]


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--build-dir',type=Path,required=True)
    p.add_argument('--prefix',type=Path,required=True);p.add_argument('--inventory',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--resume-install',type=Path,help='Prior producer receipt; every source/installed hash must still match')
    a=p.parse_args()
    for path in (a.build_dir,a.prefix,a.inventory,a.output):
        if any(part.casefold()=='.codex' for part in path.parts):raise ValueError('Excluded path')
    build=a.build_dir.resolve();prefix=a.prefix.resolve();out=a.output.resolve();out.parent.mkdir(parents=True,exist_ok=True)
    if out.exists():raise ValueError('Use a new receipt path; earlier attempts must remain intact')
    if not prefix.is_relative_to(ROOT/'build/architecture-reviews'):raise ValueError('Fresh feature-owned build evidence prefix required')
    previous=None
    if a.resume_install:
        previous=json.loads(a.resume_install.read_text(encoding='utf-8'))
        if previous.get('prefix')!=str(prefix) or previous.get('build')!=str(build) or not previous.get('provenance'):
            raise ValueError('Resume receipt does not identify this produced package')
        for pair in previous['provenance']:
            if sha(Path(pair['source']))!=pair['sha256'] or sha(Path(pair['installed']))!=pair['sha256']:
                raise ValueError('Resume source/product provenance changed')
    else:prefix.mkdir(parents=True,exist_ok=False)
    env=os.environ.copy()
    for key in list(env):
        if key.startswith('FSIM_'):env.pop(key)
    for key in ('TMP','TEMP','TORCH_EXTENSIONS_DIR'):env[key]='D:/ENV/tmp'
    ucrt=Path(os.environ.get('FSIM_UCRT64_ROOT','D:/ENV/DevLanguages/Cpp/msys2/ucrt64'))
    env['PATH']=str(ucrt/'bin')+os.pathsep+env['PATH']
    cmake=shutil.which('cmake',path=env['PATH']);ninja=shutil.which('ninja',path=env['PATH'])
    if not cmake or not ninja:raise RuntimeError('Existing UCRT64 CMake/Ninja required')
    record=dict(schema='fsim.sim-m01.installed-consumers.v1',build=str(build),prefix=str(prefix),commands=[],passed=False)
    if previous:record['producer_receipt']=dict(path=str(a.resume_install),sha256=sha(a.resume_install),elapsed_s=previous['elapsed_s'])
    started=time.perf_counter()
    def save():
        record['elapsed_s']=time.perf_counter()-started
        out.write_text(json.dumps(record,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    def run(label,command,environment=env,timeout=600):
        log=out.parent/(out.stem+'-'+label+'.log');tick=time.perf_counter()
        with log.open('wb') as output:
            process=subprocess.Popen(list(map(str,command)),cwd=out.parent,env=environment,stdout=output,stderr=subprocess.STDOUT)
            try:code=process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                subprocess.run(['taskkill','/PID',str(process.pid),'/T','/F'],stdout=output,stderr=subprocess.STDOUT)
                process.wait();code=-1
        record['commands'].append(dict(label=label,command=list(map(str,command)),exit_code=code,elapsed_s=time.perf_counter()-tick,log=str(log)))
        save()
        if code:raise RuntimeError(label+' failed; see '+str(log))
    try:
        if not previous:
            for component in ('sdk','python'):
                run('installed-'+component,[cmake,'--install',build,'--component',component,'--prefix',prefix])
        checks=[]
        pairs=[(build/'bin/libfsim.dll',prefix/'bin/libfsim.dll'),
               (build/'bin/libfsim.dll',prefix/'fsim/bin/libfsim.dll'),
               (build/'bin/libJSBSim.dll',prefix/'bin/libJSBSim.dll'),
               (build/'bin/libJSBSim.dll',prefix/'fsim/bin/libJSBSim.dll'),
               (build/'python/fsim/_native.pyd',prefix/'fsim/_native.pyd')]
        for name in ('__init__.py','world.py','agent.py','fusion.py'):
            pairs.append((ROOT/'python/fsim'/name,prefix/'fsim'/name))
        for name in ('Fusion.h','fsim_fusion.h'):pairs.append((ROOT/'include/fsim'/name,prefix/'include/fsim'/name))
        inventory=json.loads(a.inventory.read_text(encoding='utf-8'))
        for row in inventory['radars']:
            card=row['card']+'.json'
            for data in (prefix/'share/flightsim',prefix/'fsim/share/flightsim'):
                pairs.append((ROOT/'assets/payloads'/card,data/'payloads'/card))
        for source,installed in pairs:
            expected=sha(source);actual=sha(installed)
            if expected!=actual:raise RuntimeError('Installed bytes differ: '+str(installed))
            checks.append(dict(source=str(source),installed=str(installed),sha256=actual))
        record['provenance']=checks;save()
        consumer=out.parent/'installed-consumer-build'
        run(('resumed-' if previous else '')+'installed-consumer-configure',[cmake,'-S',ROOT/'tools/validation/radar','-B',consumer,'-G','Ninja',
            '-DCMAKE_TOOLCHAIN_FILE='+str(ROOT/'cmake/toolchains/ucrt64.cmake'),'-DFSIM_UCRT64_ROOT='+str(ucrt),
            '-DCMAKE_MAKE_PROGRAM='+ninja,'-Dfsim_DIR='+str(prefix/'lib/cmake/fsim'),'-DCMAKE_PREFIX_PATH='+str(prefix),
            '-DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF','-DCMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY=OFF','-DCMAKE_BUILD_TYPE=Release'])
        run(('resumed-' if previous else '')+'installed-consumer-build',[cmake,'--build',consumer,'--parallel','1'])
        isolated=env.copy();system_root=env.get('SYSTEMROOT') or env.get('SystemRoot')
        if not system_root:raise RuntimeError('Windows system root unavailable')
        isolated['PATH']=str(prefix/'bin')+os.pathsep+os.path.join(system_root,'System32')
        for executable in ('sim_m01_cpp.exe','sim_m01_c.exe'):
            run('installed-'+executable,[prefix/'bin'/executable,prefix],isolated,120)
        python=json.loads((build/'python/fsim-python.json').read_text(encoding='utf-8'))['executable']
        isolated['PYTHONPATH']=str(prefix);isolated['FSIM_DLL_DIR']=str(prefix/'bin')
        identity=out.parent/(out.stem+'-loaded-modules.json')
        run('installed-native-identity',[python,ROOT/'tools/validation/radar/provenance.py','--expected-prefix',prefix,'--output',identity],isolated,120)
        record['loaded_modules']=json.loads(identity.read_text(encoding='utf-8'))
        for module in record['loaded_modules']['modules']:
            if module['sha256']!=sha(build/'bin'/module['name']):raise RuntimeError('Actually loaded module differs from produced binary')
        save()
        run('installed-python-fusion',[python,ROOT/'tests/compatibility/python/test_fusion.py'],isolated,120)
        run('installed-fleet',[python,ROOT/'tools/validation/radar/fleet.py','--inventory',a.inventory.resolve(),
            '--output',out.parent/'installed-fleet-results.json'],isolated,600)
        record['passed']=True;save();print(json.dumps({k:v for k,v in record.items() if k not in ('commands','provenance')}))
    except Exception as error:
        record['error']=str(error);save();raise


if __name__=='__main__':main()
