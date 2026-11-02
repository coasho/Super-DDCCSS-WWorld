"""Rebuild both normal presets and run the complete SIM-M01 engineering gates.

One serial owner; fresh output, installed prefix and raw logs. No model cards,
physical assertions, reference tolerances or public defaults are changed.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

from contract import METHOD_TESTS, TEST_PATTERN

ROOT=Path(__file__).resolve().parents[3]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--deps-prefix',type=Path,default=ROOT/'build/deps-ucrt64/install')
    parser.add_argument('--catch2-source',type=Path)
    parser.add_argument('--jobs',type=int,choices=range(1,17),default=8)
    parser.add_argument('--powershell',default='powershell')
    args=parser.parse_args();out=args.output.resolve()
    if out.exists() or not out.is_relative_to(ROOT/'build/architecture-reviews'):
        raise ValueError('Use a fresh output under this source root build/architecture-reviews')
    for path in (ROOT,out,args.deps_prefix,args.catch2_source):
        if path is not None and any(part.casefold()=='.codex' for part in path.parts):raise ValueError('Excluded path')
    out.mkdir(parents=True);start=time.perf_counter()
    env=os.environ.copy();env.update(TMP='D:/ENV/tmp',TEMP='D:/ENV/tmp',TORCH_EXTENSIONS_DIR='D:/ENV/tmp')
    record=dict(schema='fsim.sim-m01.engineering-campaign.v1',source_root=str(ROOT),commands=[],passed=False,
        regression_scope='Complete declared radar/track/RF/public-interface dependency selector; not whole-project regression')
    source_files=[]
    for directory in ('src/simulation/mission_systems/radar','src/simulation/mission_systems/track','include/fsim','python/fsim','tools/validation/radar'):
        for parent,children,files in os.walk(ROOT/directory):
            children[:]=[name for name in children if name.casefold() not in ('.codex','__pycache__')]
            source_files.extend(Path(parent)/name for name in files if Path(name).suffix in ('.h','.cpp','.c','.py'))
    source={str(path.relative_to(ROOT)):sha(path) for path in sorted(source_files)}
    record['source_files']=source
    def save():
        record['elapsed_s']=time.perf_counter()-start
        (out/'campaign.json').write_text(json.dumps(record,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    def run(label,command,timeout=1200):
        log=out/(label+'.log');tick=time.perf_counter();command=list(map(str,command))
        print(label,flush=True)
        with log.open('wb') as stream:
            process=subprocess.Popen(command,cwd=ROOT,env=env,stdout=stream,stderr=subprocess.STDOUT)
            try:code=process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                subprocess.run(['taskkill','/PID',str(process.pid),'/T','/F'],stdout=stream,stderr=subprocess.STDOUT,timeout=30)
                process.wait(timeout=30);code=124
        record['commands'].append(dict(label=label,command=command,exit_code=code,elapsed_s=time.perf_counter()-tick,log=str(log)))
        save()
        if code:raise RuntimeError(label+' failed; raw log: '+str(log))
        return log
    def launcher(preset,*arguments):
        return [args.powershell,'-NoProfile','-ExecutionPolicy','Bypass','-File',ROOT/'tools/launch/fsim.ps1','--preset',preset,*arguments]
    try:
        run('artifact-checker',[sys.executable,ROOT/'tools/validation/radar/test_acceptance.py'])
        inventory=out/'coverage.json'
        run('audit',[sys.executable,ROOT/'tools/validation/radar/audit.py','--output',inventory])
        for preset in ('ucrt64-headless','ucrt64-release'):
            configure=['-DCMAKE_PREFIX_PATH='+str(args.deps_prefix.resolve()),'-DFSIM_PYTHON_EXECUTABLE='+sys.executable,
                '-DFETCHCONTENT_BASE_DIR=D:/ENV/tmp/sim-m01-closure-cmake-deps']
            if args.catch2_source:configure.append('-DFETCHCONTENT_SOURCE_DIR_CATCH2='+str(args.catch2_source.resolve()))
            run(preset+'-configure',launcher(preset,'configure',*configure))
            run(preset+'-build',launcher(preset,'build','--parallel',args.jobs))
            log=run(preset+'-inventory',launcher(preset,'test','--show-only=json-v1'),120)
            raw=log.read_text(encoding='utf-8-sig');population=json.loads(raw[raw.index('{'):])
            selected=[row['name'] for row in population['tests'] if re.search(TEST_PATTERN,row['name'])]
            if not selected or len(selected)!=len(set(selected)) or not set(METHOD_TESTS).issubset(selected):
                raise ValueError('Incomplete or duplicate M01 dependency test inventory')
            junit=out/(preset+'-junit.xml')
            run(preset+'-tests',launcher(preset,'test','--no-tests=error','--parallel',args.jobs,'--timeout','240',
                '-R',TEST_PATTERN,'--output-on-failure','--output-junit',junit),2400)
            cases=ET.parse(junit).getroot().findall('.//testcase')
            names=[case.get('name') for case in cases]
            passed=len(names)==len(selected) and set(names)==set(selected) and all(
                case.find('failure') is None and case.find('error') is None and case.find('skipped') is None for case in cases)
            data=dict(passed=passed,preset=preset,inventory=population,executed=len(cases),junit=str(junit),junit_sha256=sha(junit),
                elapsed_s=record['commands'][-1]['elapsed_s'])
            (out/(preset+'-regression.json')).write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
            if not passed:raise ValueError('Dependency regression population did not pass')
        run('installed',[sys.executable,ROOT/'tools/validation/radar/installed.py','--build-dir',ROOT/'build/ucrt64-release',
            '--prefix',out/'sdk','--inventory',inventory,'--output',out/'installed.json'])
        env['PYTHONPATH']=str(out/'sdk');env['FSIM_DLL_DIR']=str(out/'sdk/bin')
        run('performance',[sys.executable,ROOT/'tools/validation/radar/performance.py','--output',out/'performance.json'])
        env.pop('FSIM_DLL_DIR',None);env.pop('PYTHONPATH',None)
        run('normal-entry',launcher('ucrt64-release','python',ROOT/'tools/validation/radar/normal_entry.py','--output',out/'normal.json'))
        run('report',[sys.executable,ROOT/'tools/validation/radar/report.py','--coverage',inventory,
            '--functional',out/'installed-fleet-results.json','--installed',out/'installed.json','--performance',out/'performance.json',
            '--normal',out/'normal.json','--headless',out/'ucrt64-headless-regression.json','--release',out/'ucrt64-release-regression.json',
            '--output',out,'--stem','sim-m01'])
        changed=[path for path,hash_value in source.items() if sha(ROOT/path)!=hash_value]
        if changed:raise ValueError('Source changed during verification: '+str(changed))
        record.update(passed=True,source_changed=False,installed_prefix=str(out/'sdk'))
        save();print(json.dumps({k:v for k,v in record.items() if k not in ('source_files','commands')}),flush=True)
    except Exception as error:
        record['error']=str(error);save();raise


if __name__=='__main__':main()
