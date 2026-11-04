"""Complete M03 build, generation, regression, installed and visual campaign.

Run from the normal source or an ordinary source export. Fresh output and a
fresh D:/ENV product prefix are required. This runner has no pass-on-skip path.
Actual image review and final artifact admission remain explicit joined gates.
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

from acceptance import allowed, reconcile_inventory, reconcile_population
from contract import APPROVED_ROTOR_POLICY
from maps import snapshot as map_snapshot, freeze as freeze_maps, canonical as map_bytes
from countermeasures import SOURCE_PATHS as COUNTERMEASURE_SOURCES
from countermeasures import capture_preset as capture_countermeasure_preset
from effects_population import freeze_sources as freeze_effects_sources, capture_preset as capture_effects_preset
from external_tanks import freeze_sources as freeze_external_sources, capture_preset as capture_external_preset
from backend_epoch import freeze as freeze_method_environment, environment as method_environment
from full_chain import freeze_sources as freeze_chain_sources, prepare_preset as prepare_chain_preset

ROOT=Path(__file__).resolve().parents[3]


def sha(path):return hashlib.sha256(allowed(path).read_bytes()).hexdigest()
def read(path):return json.loads(allowed(path).read_text(encoding='utf-8'))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('output','prefix','git-source-root','deps-prefix','catch2-source','maps-dir','python-test-path'):
        p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--powershell',type=Path,required=True)
    p.add_argument('--jobs',type=int,choices=range(1,17),default=8)
    a=p.parse_args()
    for path in (ROOT,a.output,a.prefix,a.git_source_root,a.deps_prefix,a.catch2_source,a.maps_dir,a.powershell,a.python_test_path):allowed(path)
    if not a.python_test_path.is_dir():raise ValueError('Existing complete Python test dependencies required')
    out=a.output.resolve();prefix=a.prefix.resolve()
    if out.exists() or prefix.exists() or not prefix.is_relative_to(Path('D:/ENV')):
        raise ValueError('Fresh output and fresh D:/ENV product prefix required')
    maps_root=a.maps_dir.resolve()
    if any(left.is_relative_to(right) or right.is_relative_to(left) for left in (out,prefix) for right in (maps_root,)):
        raise ValueError('Frozen map inputs must be separate from campaign/product outputs')
    initial_maps=map_snapshot(maps_root)
    out.mkdir(parents=True);started=time.perf_counter()
    maps_start=out/'maps-start.json';maps_start.write_bytes(map_bytes(initial_maps))
    branch=subprocess.check_output(['git','branch','--show-current'],cwd=a.git_source_root,text=True,timeout=15).strip()
    if branch!='main':raise ValueError('The declared source checkout must remain on main')
    source_revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=a.git_source_root,text=True,timeout=15).strip()
    env={key:value for key,value in os.environ.items() if not key.startswith('FSIM_') or key in ('FSIM_UCRT64_ROOT','FSIM_FLOW_DIR')}
    env.update(TMP='D:/ENV/tmp',TEMP='D:/ENV/tmp',TORCH_EXTENSIONS_DIR='D:/ENV/tmp',
        MPLCONFIGDIR='D:/ENV/cache/fsim-m03/matplotlib',XDG_CACHE_HOME='D:/ENV/cache/fsim-m03',
        PSExecutionPolicyPreference='Bypass',FSIM_MAPS_DIR=str(maps_root),PYTHONDONTWRITEBYTECODE='1',
        PYTHONPATH=str(a.python_test_path.resolve()))
    record=dict(schema='fsim.sim-m03.complete-campaign.v1',source_root=str(ROOT),prefix=str(prefix),
        method_environment=freeze_method_environment(ROOT,a.python_test_path),
        git_source_root=str(a.git_source_root.resolve()),
        source_revision=source_revision,map_inputs=dict(root=str(maps_root),policy='offline-frozen-exact-tree',
            start=dict(path=str(maps_start),sha256=sha(maps_start))),
        commands=[],automated_passed=False,engineering_acceptance=False,full_regression={},
        actual_visual_review_required=True)
    def save():
        record['elapsed_s']=time.perf_counter()-started
        (out/'campaign.json').write_text(json.dumps(record,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    def run(label,command,timeout,extra_env=None):
        command=list(map(str,command));log=out/(label+'.log');tick=time.perf_counter()
        process_env=env.copy()
        if extra_env:process_env.update(extra_env)
        print(label,flush=True)
        with log.open('wb') as stream:
            process=subprocess.Popen(command,cwd=ROOT,env=process_env,stdout=stream,stderr=subprocess.STDOUT)
            try:code=process.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                subprocess.run(['taskkill','/PID',str(process.pid),'/T','/F'],stdout=stream,stderr=subprocess.STDOUT,timeout=30)
                process.wait(timeout=30);code=124
        record['commands'].append(dict(label=label,command=command,exit_code=code,elapsed_s=time.perf_counter()-tick,
            log=str(log),sha256=sha(log),environment=dict(extra_env or {})));save()
        if code:raise RuntimeError(label+' failed; '+str(log))
        return log
    def launcher(preset,*args):
        return [a.powershell,'-NoProfile','-ExecutionPolicy','Bypass','-File',ROOT/'tools/launch/fsim.ps1','--preset',preset,*args]
    try:
        run('admission-negative-controls',[sys.executable,ROOT/'tools/validation/weapon_quality/test_acceptance.py'],120)
        run('independent-navigation-reference',[sys.executable,ROOT/'tools/validation/weapon_quality/navigation_reference.py',
            '--output',out/'navigation-reference.json'],120)
        run('independent-strapdown-reference',[sys.executable,ROOT/'docs/research/sim-m03/navigation_strapdown_reference.py',
            '--out',out/'strapdown-reference.json'],120)
        run('applicability',[sys.executable,ROOT/'tools/validation/weapon_quality/audit.py','--output',out/'coverage.json'],120)
        for preset in ('ucrt64-headless','ucrt64-release'):
            run(preset+'-configure',launcher(preset,'configure','-DCMAKE_PREFIX_PATH='+str(a.deps_prefix.resolve()),
                '-DFETCHCONTENT_SOURCE_DIR_CATCH2='+str(a.catch2_source.resolve()),'-DFSIM_PYTHON_EXECUTABLE='+sys.executable,
                '-DFSIM_PYTHON_TEST_PATH='+str(a.python_test_path.resolve())),1200)
            run(preset+'-build',launcher(preset,'build','--parallel',a.jobs),3600)
        production=out/'production'
        run('complete-production',launcher('ucrt64-release','hangar','engineering','production','--output',production,
            '--prefix',prefix,'--generation-workers',min(8,a.jobs),'--geometry-threads',2),7500,
            extra_env=method_environment(record['method_environment'],ROOT,parent=env))
        generated=read(production/'matrix.json')
        if len(generated['designs'])!=88 or any(row['status']=='blocked' for row in generated['designs']):
            raise ValueError('Incomplete required production roster')
        coverage=read(out/'coverage.json')
        expected_designs={r['model'] for r in coverage['stores']}|{r['design'] for r in coverage['fits']}
        reconcile_population(expected_designs,generated['designs'],('name',))
        mass_closure=read(production/'mass-partition-closure.json')
        if not (mass_closure['complete'] and mass_closure['declared_count']==88
                and mass_closure['design_count']==88 and set(mass_closure['designs'])==expected_designs):
            raise ValueError('Complete actual generated mass/first/full second-moment closure required')
        record['production']=dict(path=str(production),matrix_sha256=sha(production/'matrix.json'),
            mass_partition_sha256=sha(production/'mass-partition-closure.json'),
            timing=read(production/'campaign-timing.json'));save()
        run('liquid-population',[sys.executable,ROOT/'tools/validation/weapon_quality/liquid_population.py',
            '--coverage',out/'coverage.json','--root',ROOT,'--output',out/'liquid-population.json'],300)
        # Generation changes native products. Both normal runtime staging targets
        # must consume the produced bytes before regression or SDK use.
        for preset in ('ucrt64-headless','ucrt64-release'):
            run(preset+'-stage-produced',launcher(preset,'build','--parallel',a.jobs),1800)
        cache=(ROOT/'build/ucrt64-headless/CMakeCache.txt').read_text(encoding='utf-8')
        cargo=re.search(r'^FSIM_CARGO:FILEPATH=(.+)$',cache,re.MULTILINE)
        if not cargo or not Path(cargo.group(1)).is_file():raise ValueError('Existing discovered Rust consumer toolchain required')
        run('cargo-lock-preflight',[cargo.group(1),'generate-lockfile','--offline','--manifest-path',
            ROOT/'examples/training/rust_trainer/Cargo.toml'],120)
        for preset in ('ucrt64-headless','ucrt64-release'):
            run(preset+'-installed-equipment-preflight',launcher(preset,'test','--no-tests=error','--timeout','600',
                '-R','^equipment_package_consumer$','--output-on-failure'),900)
        policy=ROOT/'docs/data/sim-m02-deferred-rotor-issues-20261101.json'
        if sha(policy)!=APPROVED_ROTOR_POLICY:raise ValueError('User-approved M02 rotor scope changed')
        # Freeze physical receiver sources before execution. A post-run source
        # digest alone cannot prove which bytes produced the captured detector.
        cm_sources=COUNTERMEASURE_SOURCES|{row['card_path'] for row in coverage['stores'] if row['class']=='aam'}
        cm_epoch=dict(schema='fsim.sim-m03.countermeasure-source-epoch.v1',main_head=source_revision,
            coverage_sha256=sha(out/'coverage.json'),sources=[dict(path=name,sha256=sha(ROOT/name)) for name in sorted(cm_sources)],presets=[])
        cm_epoch_path=out/'countermeasure-source-epoch.json'
        cm_epoch_path.write_text(json.dumps(cm_epoch,indent=2,allow_nan=False)+'\n',encoding='utf-8')
        effects_epoch=freeze_effects_sources(coverage,sha(out/'coverage.json'),ROOT,a.git_source_root)
        effects_epoch_path=out/'effects-source-epoch.json'
        effects_epoch_path.write_text(json.dumps(effects_epoch,indent=2,allow_nan=False)+'\n',encoding='utf-8')
        external_epoch=freeze_external_sources(coverage,sha(out/'coverage.json'),ROOT,a.git_source_root)
        external_epoch_path=out/'external-tanks-source-epoch.json'
        external_epoch_path.write_text(json.dumps(external_epoch,indent=2,allow_nan=False)+'\n',encoding='utf-8')
        chain_epoch_path=out/'native-full-chain-source-epoch.json'
        chain_epoch_path.write_text(json.dumps(freeze_chain_sources(coverage,ROOT),indent=2,allow_nan=False)+'\n',encoding='utf-8')
        # Both product expectations and the immutable source epoch precede
        # either full regression. The native captures independently attest
        # which executable/modules produced their six actual output files.
        chain_runs={preset:dict(prepare_chain_preset(chain_epoch_path,preset,ROOT),junit=str(out/preset/'junit.xml'))
                    for preset in ('ucrt64-headless','ucrt64-release')}
        chain_runs_path=out/'native-full-chain-runs.json'
        chain_runs_path.write_text(json.dumps(chain_runs,indent=2,allow_nan=False)+'\n',encoding='utf-8')
        record['native_full_chain']=dict(source_epoch=dict(path=str(chain_epoch_path),sha256=sha(chain_epoch_path)),
            runs=dict(path=str(chain_runs_path),sha256=sha(chain_runs_path)));save()
        for preset in ('ucrt64-headless','ucrt64-release'):
            folder=out/preset
            run(preset+'-full',[sys.executable,ROOT/'tools/validation/aerodynamics/sim_m02_closure.py','--source-root',ROOT,
                '--preset',preset,'--output',folder,'--production',production,'--git-source-root',a.git_source_root,
                '--maps-dir',maps_root,'--powershell',a.powershell,'--jobs',a.jobs],11000,
                extra_env=dict(FSIM_AAM_CHAIN_EPOCH=str(chain_epoch_path)))
            result=read(folder/'regression.json')
            if not result.get('passed') or result.get('excluded')!=28 or result.get('policy_sha256')!=APPROVED_ROTOR_POLICY:
                raise ValueError('Incomplete full regression or changed approved scope')
            record['full_regression'][preset]=dict(path=str(folder/'regression.json'),sha256=sha(folder/'regression.json'),
                discovered=result['discovered'],executed=result['executed'],deferred=result['excluded'],
                nested=result['nested_methods']);save()
            cm_epoch['presets'].append(capture_countermeasure_preset(coverage,preset,folder/'junit.xml',
                out/'countermeasure'/preset/'countermeasure-summary-skip4.csv'))
            cm_epoch_path.write_text(json.dumps(cm_epoch,indent=2,allow_nan=False)+'\n',encoding='utf-8')
            effects_epoch['presets'].append(capture_effects_preset(coverage,preset,folder/'junit.xml'))
            effects_epoch_path.write_text(json.dumps(effects_epoch,indent=2,allow_nan=False)+'\n',encoding='utf-8')
            external_epoch['presets'].append(capture_external_preset(coverage,preset,folder/'junit.xml'))
            external_epoch_path.write_text(json.dumps(external_epoch,indent=2,allow_nan=False)+'\n',encoding='utf-8')
        run('native-full-chain',[sys.executable,ROOT/'tools/validation/weapon_quality/full_chain.py',
            '--coverage',out/'coverage.json','--source-epoch',chain_epoch_path,'--runs',chain_runs_path,
            '--output',out/'native-full-chain.json'],240)
        record['native_full_chain']['proof']=dict(path=str(out/'native-full-chain.json'),sha256=sha(out/'native-full-chain.json'));save()
        run('native-external-tanks',[sys.executable,ROOT/'tools/validation/weapon_quality/external_tanks.py',
            '--coverage',out/'coverage.json','--source-epoch',external_epoch_path,'--git-source-root',a.git_source_root,
            '--output',out/'external-tanks.json'],120)
        run('native-authored-effects',[sys.executable,ROOT/'tools/validation/weapon_quality/effects_population.py',
            '--coverage',out/'coverage.json','--source-epoch',effects_epoch_path,'--git-source-root',a.git_source_root,
            '--output',out/'authored-effects.json'],240)
        run('native-countermeasure-receivers',[sys.executable,ROOT/'tools/validation/weapon_quality/countermeasures.py',
            '--coverage',out/'coverage.json','--source-epoch',cm_epoch_path,'--git-source-root',a.git_source_root,
            '--output',out/'countermeasure-receivers.json'],120)
        run('native-cadence',[sys.executable,ROOT/'tools/validation/weapon_quality/cadence.py','--coverage',out/'coverage.json',
            '--output',out/'cadence'],240)
        run('installed',[sys.executable,ROOT/'tools/validation/weapon_quality/installed.py','--build-dir',ROOT/'build/ucrt64-release',
            '--prefix',prefix,'--producer',production/'installed-production-manifest.json','--output',out/'installed.json'],2400)
        run('installed-component-chain',[sys.executable,ROOT/'tools/validation/weapon_quality/component_public.py',
            '--prefix',prefix,'--coverage',out/'coverage.json','--installed',out/'installed.json',
            '--output',out/'component-public'],20000)
        record['component_public']={key:dict(path=str(path),sha256=sha(path)) for key,path in
            (('run',out/'component-public/run.json'),('proof',out/'component-public/components.json'))};save()
        run('installed-authored-fits',[sys.executable,ROOT/'tools/validation/weapon_quality/fits.py','--prefix',prefix,
            '--coverage',out/'coverage.json','--producer',production/'installed-production-manifest.json',
            '--output',out/'installed-authored-fits.json'],3600)
        run('installed-fleet',[sys.executable,ROOT/'tools/validation/weapon_quality/fleet.py','--prefix',prefix,
            '--coverage',out/'coverage.json','--output',out/'installed-fleet.json'],3600)
        run('installed-explosives',[sys.executable,ROOT/'tools/validation/weapon_quality/explosives.py','--prefix',prefix,
            '--output',out/'explosives.json'],600)
        run('normal-entry',launcher('ucrt64-release','python',ROOT/'tools/validation/weapon_quality/normal_entry.py',
            '--output',out/'normal.json'),240)
        run('full-native-and-visual',launcher('ucrt64-release','python',ROOT/'tools/validation/weapon_quality/visual.py',
            '--output',out/'visual','--source-revision',source_revision),3600)
        run('explosives-visual',launcher('ucrt64-release','python',ROOT/'tools/validation/weapon_quality/visual_explosives.py',
            '--prefix',prefix,'--output',out/'visual/explosives.json'),900)
        final_maps=out/'maps.json';final_maps.write_bytes(map_bytes(freeze_maps(maps_root,initial_maps,maps_start)))
        record['maps']=dict(path=str(final_maps),sha256=sha(final_maps))
        record['automated_passed']=True;save()
        print(json.dumps({k:v for k,v in record.items() if k!='commands'}),flush=True)
    except Exception as error:
        record['error']=str(error);save();raise


if __name__=='__main__':main()
