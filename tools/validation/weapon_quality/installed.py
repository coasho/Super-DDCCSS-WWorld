"""Install and execute public-only M03 consumers with actual module provenance."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

from acceptance import allowed, canonical_product_map
from component_public import facade_paths

ROOT=Path(__file__).resolve().parents[3]


def sha(path):return hashlib.sha256(allowed(path).read_bytes()).hexdigest()
def read(path):return json.loads(allowed(path).read_text(encoding='utf-8'))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('build-dir','prefix','output'):p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--producer',type=Path);a=p.parse_args()
    for path in (ROOT,a.build_dir,a.prefix,a.output):allowed(path)
    build=a.build_dir.resolve();prefix=a.prefix.resolve();output=a.output.resolve()
    if output.exists():raise ValueError('Fresh installed-consumer receipt required')
    if not prefix.is_relative_to(Path('D:/ENV')):raise ValueError('Installed runtime prefix belongs under D:/ENV')
    if prefix.exists():
        if a.producer is None:raise ValueError('Existing prefix requires its same-run producer receipt')
        producer=read(a.producer)
        if producer.get('schema')!='fsim.sim-m02.production-install.v1' or producer.get('product_count')!=len(producer.get('products',{})) or not producer.get('products'):raise ValueError('Invalid passive production manifest')
        if Path(producer['prefix']).resolve()!=prefix or producer['native_dll_sha256']!=sha(build/'bin/libfsim.dll'):
            raise ValueError('Existing product does not match its actual producer')
        if sha(prefix/'bin/libfsim.dll')!=producer['native_dll_sha256']:raise ValueError('Produced installed DLL changed')
        for relative,expected in canonical_product_map(producer['products']).items():
            if sha(prefix/'share/flightsim/aircraft'/relative)!=expected:raise ValueError('Produced installed model changed')
    else:prefix.mkdir(parents=True)
    output.parent.mkdir(parents=True,exist_ok=True);started=time.perf_counter()
    artifacts=output.parent/(output.stem+'-artifacts');artifacts.mkdir(exist_ok=False)
    env={key:value for key,value in os.environ.items() if not key.startswith('FSIM_')}
    env.update(TMP='D:/ENV/tmp',TEMP='D:/ENV/tmp',TORCH_EXTENSIONS_DIR='D:/ENV/tmp')
    ucrt=Path('D:/ENV/DevLanguages/Cpp/msys2/ucrt64');env['PATH']=str(ucrt/'bin')+os.pathsep+env.get('PATH','')
    cmake=ucrt/'bin/cmake.exe';ninja=ucrt/'bin/ninja.exe'
    record=dict(schema='fsim.sim-m03.installed-consumers.v1',passed=False,build=str(build),prefix=str(prefix),commands=[])
    if a.producer:record['producer']=dict(path=str(a.producer),sha256=sha(a.producer))
    def save():
        record['elapsed_s']=time.perf_counter()-started
        output.write_text(json.dumps(record,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    def run(label,command,environment=env,cwd=None,timeout=900):
        log=output.parent/(output.stem+'-'+label+'.log');tick=time.perf_counter()
        with log.open('wb') as stream:
            process=subprocess.Popen(list(map(str,command)),cwd=cwd or output.parent,env=environment,stdout=stream,stderr=subprocess.STDOUT)
            try:code=process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                subprocess.run(['taskkill','/PID',str(process.pid),'/T','/F'],stdout=stream,stderr=subprocess.STDOUT,timeout=30)
                process.wait(timeout=30);code=124
        record['commands'].append(dict(label=label,command=list(map(str,command)),exit_code=code,elapsed_s=time.perf_counter()-tick,
            log=str(log),sha256=sha(log)));save()
        if code:raise RuntimeError('Installed '+label+' failed; '+str(log))
    try:
        cache=(build/'CMakeCache.txt').read_text(encoding='utf-8')
        render=next((line.split('=',1)[1] for line in cache.splitlines() if line.startswith('FSIM_WITH_RENDER:BOOL=')),None)
        if render not in ('ON','OFF'):raise ValueError('Missing explicit render build configuration')
        record['render_enabled']=render=='ON'
        for component in ('sdk','python','tools','examples')+ (('viewer',) if render=='ON' else ()):
            run('install-'+component,[cmake,'--install',build,'--component',component,'--prefix',prefix])
        provenance=[]
        pairs=[(build/'bin/libfsim.dll',prefix/'bin/libfsim.dll'),(build/'bin/libfsim.dll',prefix/'fsim/bin/libfsim.dll'),
            (build/'bin/libJSBSim.dll',prefix/'bin/libJSBSim.dll'),(build/'python/fsim/_native.pyd',prefix/'fsim/_native.pyd'),
            ]
        if render=='ON':pairs.append((build/'bin/flightsim-viewer.exe',prefix/'bin/flightsim-viewer.exe'))
        for name in ('WeaponLink.h','fsim_weapon_link.h','Fusion.h'):pairs.append((ROOT/'include/fsim'/name,prefix/'include/fsim'/name))
        for name in ('__init__.py','weapon_link.py','weapon_link_process.py','world.py','agent.py','catalog.py'):pairs.append((ROOT/'python/fsim'/name,prefix/'fsim'/name))
        for source in facade_paths(ROOT):pairs.append((source,prefix/'fsim'/source.relative_to(ROOT/'python/fsim')))
        pairs=list(dict.fromkeys(pairs))
        catalogue=ROOT/'tools/models/hangar/hangar/data/equipment_models.json'
        pairs.extend((catalogue,data/'equipment_models.json') for data in (prefix/'share/flightsim',prefix/'fsim/share/flightsim'))
        for path in sorted((ROOT/'assets/stores').glob('*.json')):
            pairs.extend((path,data/'stores'/path.name) for data in (prefix/'share/flightsim',prefix/'fsim/share/flightsim'))
        for source,installed in pairs:
            expected=sha(source)
            if sha(installed)!=expected:raise ValueError('Installed bytes differ: '+str(installed))
            provenance.append(dict(source=str(source),installed=str(installed),sha256=expected))
        record['provenance']=provenance;save()
        consumer=artifacts/'standalone-consumer-source';consumer.mkdir(exist_ok=False)
        for name in ('CMakeLists.txt','consumer.cpp','consumer.c','aam_public.cpp','aam_public.c','aam_public_capture.h'):
            shutil.copy2(Path(__file__).parent/name,consumer/name)
        for source,name in ((ROOT/'tests/compatibility/test_weapon_links.cpp','functional_cpp.inc'),
                            (ROOT/'tests/compatibility/test_weapon_links_c.c','functional_c.inc')):
            shutil.copy2(source,consumer/name)
        consumer_build=artifacts/'standalone-consumer-build'
        run('consumer-configure',[cmake,'-S',consumer,'-B',consumer_build,'-G','Ninja',
            '-DCMAKE_TOOLCHAIN_FILE='+str(ROOT/'cmake/toolchains/ucrt64.cmake'),'-DFSIM_UCRT64_ROOT='+str(ucrt),
            '-DCMAKE_MAKE_PROGRAM='+str(ninja),'-Dfsim_DIR='+str(prefix/'lib/cmake/fsim'),'-DCMAKE_PREFIX_PATH='+str(prefix),
            '-DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF','-DCMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY=OFF','-DCMAKE_BUILD_TYPE=Release'])
        run('consumer-build',[cmake,'--build',consumer_build,'--parallel','1'])
        isolated=env.copy();isolated['PATH']=str(prefix/'bin')+os.pathsep+str(Path(os.environ.get('SystemRoot','C:/Windows'))/'System32')
        isolated['PYTHONPATH']=str(prefix);isolated['FSIM_DLL_DIR']=str(prefix/'bin')
        isolated['FSIM_AIRCRAFT_PATH']=os.pathsep.join(str(prefix/'share/flightsim/aircraft'/part) for part in ('','stores'))
        record['installed_aircraft_search_roots']=isolated['FSIM_AIRCRAFT_PATH'].split(os.pathsep);save()
        run_dir=artifacts/'standalone-runtime';run_dir.mkdir(exist_ok=False)
        modules=artifacts/'installed-cpp-modules.json'
        run('cpp',[consumer_build/'sim_m03_cpp.exe',prefix,modules],isolated,run_dir,240)
        loaded=read(modules)
        for module in loaded['modules']:
            if not allowed(module['path']).resolve().is_relative_to(prefix):raise ValueError('C++ module outside installed prefix')
            module['sha256']=sha(Path(module['path']))
            if module['sha256']!=sha(build/'bin'/module['name']):raise ValueError('Actual C++ process loaded another product')
        record['cpp_loaded_modules']=loaded;save()
        c_modules=artifacts/'installed-c99-modules.json'
        run('c99',[consumer_build/'sim_m03_c.exe',prefix,c_modules],isolated,run_dir,240)
        record['c99_loaded_modules']=read(c_modules)
        for module in record['c99_loaded_modules']['modules']:
            if not allowed(module['path']).resolve().is_relative_to(prefix):raise ValueError('C99 module outside installed prefix')
            module['sha256']=sha(Path(module['path']))
            if module['sha256']!=sha(build/'bin'/module['name']):raise ValueError('Actual C99 process loaded another product')
        interpreter=read(build/'python/fsim-python.json')['executable']
        record['component_clients']={language:dict(path=str(path),sha256=sha(path)) for language,path in
            (('cpp',consumer_build/'sim_m03_aam_cpp.exe'),('c99',consumer_build/'sim_m03_aam_c99.exe'),
             ('python',Path(interpreter)))}
        record['component_client_sources']=[dict(source=str(Path(__file__).parent/name),installed=str(consumer/name),
            sha256=sha(consumer/name)) for name in ('CMakeLists.txt','aam_public.cpp','aam_public.c','aam_public_capture.h')]
        save()
        identity=artifacts/'installed-python-modules.json'
        script=r"""import hashlib,json,pathlib,sys,unittest
prefix=pathlib.Path(sys.argv[1]).resolve()
sys.path.insert(0,sys.argv[2]);sys.path.insert(0,sys.argv[3])
import fsim
from provenance import loaded_modules
import test_weapon_links
import test_catalog
suite=unittest.defaultTestLoader.loadTestsFromModule(test_weapon_links)
suite.addTests(unittest.defaultTestLoader.loadTestsFromModule(test_catalog))
result=unittest.TextTestRunner(verbosity=2).run(suite)
facade=pathlib.Path(fsim.__file__).resolve();native=pathlib.Path(fsim._native.__file__).resolve()
if not facade.is_relative_to(prefix) or not native.is_relative_to(prefix):raise RuntimeError('Python product outside prefix')
record=dict(python_package=str(facade),native_python=str(native),native_python_sha256=hashlib.sha256(native.read_bytes()).hexdigest(),modules=loaded_modules(prefix),tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),skips=len(result.skipped))
pathlib.Path(sys.argv[4]).write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
sys.exit(0 if result.wasSuccessful() and result.testsRun>0 and not result.skipped else 1)
"""
        run('python',[interpreter,'-B','-c',script,prefix,ROOT/'tests/compatibility/python',ROOT/'tools/validation/radar',identity],isolated,run_dir,600)
        record['python_loaded_modules']=read(identity)
        if record['python_loaded_modules']['native_python_sha256']!=sha(build/'python/fsim/_native.pyd'):raise ValueError('Python extension differs from producer')
        for module in record['python_loaded_modules']['modules']:
            if module['sha256']!=sha(build/'bin'/module['name']):raise ValueError('Actual Python process loaded another product')
        record['passed']=True;save();print(json.dumps({k:v for k,v in record.items() if k not in ('commands','provenance')}))
    except Exception as error:
        record['error']=str(error);save();raise


if __name__=='__main__':main()
