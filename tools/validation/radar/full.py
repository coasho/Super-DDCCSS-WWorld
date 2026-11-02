"""Complete M01 quality campaign: full presets, installed consumers and live UI.

Runs serially in a clean ordinary main-source export or clean main checkout.
The existing approved M02 rotor deferral is unchanged. No M01 exclusions exist.
Actual PNG review remains explicit and must be joined by the delivery report.
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

ROOT=Path(__file__).resolve().parents[3]


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--production',type=Path,required=True);p.add_argument('--git-source-root',type=Path,required=True)
    p.add_argument('--deps-prefix',type=Path,required=True);p.add_argument('--catch2-source',type=Path,required=True)
    p.add_argument('--maps-dir',type=Path,required=True);p.add_argument('--powershell',required=True)
    p.add_argument('--jobs',type=int,choices=range(1,17),default=8);a=p.parse_args();out=a.output.resolve()
    if out.exists() or any(x.casefold()=='.codex' for path in (ROOT,out,a.production,a.git_source_root,a.deps_prefix,a.catch2_source,a.maps_dir) for x in path.parts):raise ValueError('Fresh allowed output required')
    out.mkdir(parents=True);start=time.perf_counter();env=os.environ.copy()
    env.update(TMP='D:/ENV/tmp',TEMP='D:/ENV/tmp',TORCH_EXTENSIONS_DIR='D:/ENV/tmp',PSExecutionPolicyPreference='Bypass',
        FSIM_MAPS_DIR=str(a.maps_dir.resolve()))
    record=dict(schema='fsim.sim-m01.full-campaign.v1',source_root=str(ROOT),commands=[],automated_passed=False,
        full_regression={},visual_review_required=True)
    def save():
        record['elapsed_s']=time.perf_counter()-start;(out/'full-campaign.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
    def run(label,command,timeout):
        command=list(map(str,command));log=out/(label+'.log');print(label,flush=True);tick=time.perf_counter()
        with log.open('wb') as stream:
            process=subprocess.Popen(command,cwd=ROOT,env=env,stdout=stream,stderr=subprocess.STDOUT)
            try:code=process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                subprocess.run(['taskkill','/PID',str(process.pid),'/T','/F'],stdout=stream,stderr=subprocess.STDOUT,timeout=30);process.wait(timeout=30);code=124
        record['commands'].append(dict(label=label,command=command,exit_code=code,elapsed_s=time.perf_counter()-tick,log=str(log),sha256=sha(log)));save()
        if code:raise RuntimeError(label+' failed; '+str(log))
    try:
        campaign=out/'engineering'
        run('engineering',[sys.executable,ROOT/'tools/validation/radar/campaign.py','--output',campaign,'--deps-prefix',a.deps_prefix,
            '--catch2-source',a.catch2_source,'--powershell',a.powershell,'--jobs',a.jobs],7200)
        run('applicability',[sys.executable,ROOT/'tools/validation/radar/applicability.py','--inventory',campaign/'coverage.json','--output',out/'applicability.json'],120)
        cache=(ROOT/'build/ucrt64-headless/CMakeCache.txt').read_text(encoding='utf-8')
        cargo=re.search(r'^FSIM_CARGO:FILEPATH=(.+)$',cache,re.MULTILINE)
        if not cargo or not Path(cargo.group(1)).is_file():raise ValueError('The discovered Rust consumer toolchain is required')
        # Cargo.lock is generated for this dependency-free example and ignored
        # by Git. Initialize it before immutable full-regression input binding.
        run('cargo-lock-preflight',[cargo.group(1),'generate-lockfile','--offline','--manifest-path',
            ROOT/'examples/training/rust_trainer/Cargo.toml'],120)
        for preset in ('ucrt64-headless','ucrt64-release'):
            run(preset+'-installed-equipment-preflight',[a.powershell,'-NoProfile','-ExecutionPolicy','Bypass','-File',ROOT/'tools/launch/fsim.ps1',
                '--preset',preset,'test','--no-tests=error','--timeout','600','-R','^equipment_package_consumer$','--output-on-failure'],900)
        for preset in ('ucrt64-headless','ucrt64-release'):
            target=out/preset
            run(preset,[sys.executable,ROOT/'tools/validation/aerodynamics/sim_m02_closure.py','--source-root',ROOT,'--preset',preset,
                '--output',target,'--production',a.production,'--git-source-root',a.git_source_root,'--maps-dir',a.maps_dir,
                '--powershell',a.powershell,'--jobs',a.jobs],11000)
            value=json.loads((target/'regression.json').read_text(encoding='utf-8'))
            cases=ET.parse(target/'junit.xml').getroot().findall('.//testcase')
            if not value['passed'] or len(cases)!=value['selected'] or value['excluded']!=28:raise ValueError('Full regression population differs')
            record['full_regression'][preset]=dict(discovered=value['discovered'],executed=len(cases),excluded=value['excluded'],
                receipt_sha256=sha(target/'regression.json'),junit_sha256=sha(target/'junit.xml'),passed=True);save()
        run('visual',[sys.executable,ROOT/'tools/validation/radar/visual.py','--output',out/'visual','--powershell',a.powershell],600)
        record['automated_passed']=True;save();print(json.dumps(record));return 0
    except Exception as error:
        record['error']=str(error);save();raise


if __name__=='__main__':raise SystemExit(main())
