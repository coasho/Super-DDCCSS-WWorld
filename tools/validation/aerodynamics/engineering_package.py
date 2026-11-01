"""Package the current all-design production/basic-use delivery without reruns."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import zipfile

ROOT=Path(__file__).resolve().parents[3]


def read(path):return json.loads(Path(path).read_text(encoding='utf-8'))
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def write(path,value):Path(path).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n',encoding='utf-8')


def archive_tree(source,target):
    tick=time.monotonic();files=[]
    with zipfile.ZipFile(target,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6,allowZip64=True) as archive:
        for here,dirs,names in os.walk(source,followlinks=False):
            dirs[:]=sorted(d for d in dirs if d.casefold() not in ('.codex','.git','__pycache__') and not (Path(here)/d).is_symlink())
            for name in sorted(names):
                path=Path(here)/name
                if path.is_symlink():continue
                relative=path.relative_to(source).as_posix();archive.write(path,relative)
                files.append(dict(path=relative,bytes=path.stat().st_size,sha256=sha(path)))
    return dict(path=str(target),sha256=sha(target),bytes=target.stat().st_size,files=files,elapsed_seconds=time.monotonic()-tick)


def main(args):
    started=time.monotonic();out=Path(args.output).resolve();production=out/'production'
    if not out.is_relative_to(ROOT/'build') or '.codex' in {p.casefold() for p in out.parts}:
        raise ValueError('Delivery output must be an owned build artifact directory')
    basic=read(out/'basic/basic-usability.json')
    if basic['failures'] or basic['counts']['native_consumers']!=88 or basic['counts']['installed_consumers']!=88:
        raise ValueError('A complete passing all-design basic-use pass is required before packaging')
    env=os.environ.copy();env.update(TMP='D:/ENV/tmp',TEMP='D:/ENV/tmp',TORCH_EXTENSIONS_DIR='D:/ENV/tmp')
    records=[]
    def run(name,argv,limit=1200):
        tick=time.monotonic();log=out/(name+'.log')
        with log.open('wb') as stream:
            child=subprocess.Popen(list(map(str,argv)),cwd=ROOT,env=env,stdout=stream,stderr=subprocess.STDOUT)
            try:code=child.wait(timeout=limit)
            except subprocess.TimeoutExpired:
                subprocess.run(['taskkill','/PID',str(child.pid),'/T','/F'],stdout=stream,stderr=subprocess.STDOUT,timeout=15)
                child.wait(timeout=15);code=-1
        record=dict(name=name,argv=list(map(str,argv)),cwd=str(ROOT),log=str(log),exit_code=code,elapsed_seconds=time.monotonic()-tick)
        records.append(record);write(out/'packaging-commands.json',records)
        print(name,code,record['elapsed_seconds'],flush=True)
        if code:raise ValueError('Required packaging stage failed: '+name)
    run('production-inventory',[sys.executable,'-B',Path(__file__).with_name('engineering_production_delivery.py'),
        '--production',production,'--output',out])
    report=read(out/'freshness-and-counts.json');matrix=read(production/'matrix.json');timing=read(production/'campaign-timing.json')
    if matrix['counts']['blocked']:raise ValueError('Cannot package a blocked design as complete')
    counts=report['counts'];core=basic['native_components']
    paragraph=(f"Current production delivery: {matrix['counts']['engineering-complete-uncalibrated']} engineering-complete-uncalibrated, "
        f"{matrix['counts']['engineering-complete']} engineering-complete and {matrix['counts']['blocked']} blocked designs. "
        "All 31 fixed wings, four rotorcraft and 53 stores (including all24 AAMs) use the same source-driven method and component rules. "
        f"Aggregate production generation/install: {timing['elapsed_seconds']:.3f} seconds; "
        f"current basic usability: {basic['elapsed_seconds']:.3f} seconds. "
        f"Current basic execution covers 88 native and 88 installed consumers, {counts['basic_flight_observations']} short-flight observations "
        f"and {core['executed']} core/package fixtures ({core['failed']} failed, {core['skipped']} skipped). "
        f"Production integrity inventories {counts['table_inventory_entries']} logical/serialized table entries and "
        f"{counts['native_xml_coefficient_cells']} XML coefficient cells. "
        "These are separately counted from document consistency predicates. The latest user scope is bounded basic usability; "
        "no new full regression or210-condition envelope sweep is claimed. Exact-model measurement calibration remains optional. "
        "Evidence: `build/architecture-reviews/sim-m02-delivery-20261031/`.")
    begin='<!-- SIM-M02 current generated verification begin -->';end='<!-- SIM-M02 current generated verification end -->'
    for relative in ('docs/status.md','docs/hangar-system.md','docs/sdk/missile-flight-validation.md',
                     'docs/sdk/weapon-fidelity-validation.md','docs/architecture/modules/simulation/plan.md'):
        path=ROOT/relative;text=path.read_text(encoding='utf-8');block=begin+'\n'+paragraph+'\n'+end
        if begin in text:
            a=text.index(begin);b=text.index(end,a)+len(end);text=text[:a]+block+text[b:]
        else:text+='\n\n'+block+'\n'
        if '\ufffd' in text:raise ValueError('Invalid replacement character in '+relative)
        path.write_text(text,encoding='utf-8')
    shutil.copy2(production/'matrix.csv',ROOT/'docs/data/sim-m02-engineering-completeness.csv')
    review_path=Path(args.review).resolve() if args.review else out/'independent-review.json'
    if '.codex' in {p.casefold() for p in review_path.parts}:raise ValueError('Excluded review path')
    review=read(review_path)
    if not review.get('sources'):raise ValueError('A source-bound actual independent review receipt is required')
    for relative,expected in review['sources'].items():
        path=Path(relative)
        if path.is_absolute() or '..' in path.parts or '.codex' in {p.casefold() for p in path.parts}:
            raise ValueError('Invalid independent review source path')
        if sha(ROOT/path)!=expected:raise ValueError('Independent review does not cover the current source: '+relative)
    if review_path!=out/'independent-review.json':shutil.copy2(review_path,out/'independent-review.json')
    run('head-plus-patch',[sys.executable,'-B',Path(__file__).with_name('engineering_replay.py'),
        '--matrix',production/'matrix.json','--output',out,'--export',Path(args.export).resolve()])
    patch=read(out/'patch-manifest.json');prefix=Path(report['prefix'])
    sdk_manifest=out/'sdk-package-manifest.json';sdk_archive=out/'sim-m02-sdk.zip';sdk=None
    if sdk_manifest.is_file() and sdk_archive.is_file():
        previous=read(sdk_manifest)
        if sha(sdk_archive)==previous['sha256']:
            current={}
            for here,dirs,names in os.walk(prefix,followlinks=False):
                dirs[:]=sorted(d for d in dirs if d.casefold() not in ('.codex','.git','__pycache__') and not (Path(here)/d).is_symlink())
                for name in names:
                    path=Path(here)/name
                    if not path.is_symlink():current[path.relative_to(prefix).as_posix()]=sha(path)
            if current=={entry['path']:entry['sha256'] for entry in previous['files']}:sdk=previous
    if sdk is None:
        sdk=archive_tree(prefix,sdk_archive);write(sdk_manifest,sdk)
    slow=max(matrix['designs'],key=lambda r:r['generation_seconds'])
    slow_phases=slow['generation_phase_worker_seconds']
    build_receipt=ROOT/'build/architecture-reviews/sim-m02-closure-20261031/repaired/contact-rate-repair/commands.json'
    build_seconds=read(build_receipt)['commands'][0]['elapsed_seconds'] if build_receipt.is_file() else None
    metrics=dict(production_seconds=timing['elapsed_seconds'],basic_usability_seconds=basic['elapsed_seconds'],
        packaging_seconds_so_far=time.monotonic()-started,preceding_native_build_seconds=build_seconds,
        slowest_generation_design=slow['name'],slowest_generation_worker_seconds=slow['generation_seconds'],
        slowest_generation_worker_phases=slow_phases,
        comparison='Historical controlled unchanged-workload optimization2048.546->1734.571s is separate. Current polar method and verification scope differ; no false same-workload speedup claim.')
    write(out/'delivery-timing.json',metrics)
    text=f'''# SIM-M02 production delivery

{paragraph}

## Actual products

- Installed SDK/native/Python assets: `{prefix}`.
- SDK archive: [sim-m02-sdk.zip](sim-m02-sdk.zip), {sdk['bytes']} bytes, SHA256 `{sdk['sha256']}`.
- One source/product patch: [sim-m02.patch](sim-m02.patch), SHA256 `{patch['patch_sha256']}`.
- Base main HEAD: `{patch['base_head']}`. No commit, push, branch or worktree was created.
- Actual ordinary HEAD-archive plus binary-patch application: `{patch['source_export']}`; {len(patch['byte_verification_failures'])} byte mismatches; real index unchanged: `{patch['real_index_unchanged']}`.
- Patch changes {patch['patch_changed_file_count']} files. [Exact file list](patch-files.txt), [diff stat](patch-stat.txt), [complete manifest](patch-manifest.json).
- [All88 matrix](production/matrix.csv), [machine-readable matrix](production/matrix.json), [per-table method inventory](table-method-inventory.csv), [source versions](method-source-versions.json).

## Production changes

The native contact response uses actual integration coefficients and prior derivatives for version2 assets. Legacy assets retain their original branch. Shared section polars preserve source reflection symmetry, including signed flap limits; every aerodynamic product was regenerated. Source reconstruction retains common integration metadata. Calibration inventory can recognize an exact byte-bound physical source with only added numerical metadata, without changing any fitted coefficients or measured data. Generic armed terrain-contact fuze/warhead and inertial-waypoint/turbofan providers remain real native implementations; applicability never depends on an implementation being unavailable.

All generated tables declare method, derivation/source, immutable implementation version, bounds and measured/estimated/simulated basis. VLM plus explicit engineering closures is the baseline; body-only designs use its zero-lifting-panel limit. No CFD or newly measured table is claimed. Existing scoped performance overrides are retained and disclosed, not new calibration evidence.

## Current basic usability

[Actual commands/results](basic/basic-usability.json): 88/88 native and88/88 installed design consumers, {counts['basic_flight_observations']} short-flight observations, {counts['basic_released_store_bodies']} independent released-store body observations; {core['executed']}/{core['executed']} core/package CTest records, {core['failed']} failed, {core['skipped']} skipped. [Native JUnit](basic/basic-native.junit.xml).

Every consumer loads the current DLL and regenerated installed assets, refuses a poisoned exact XML instead of falling back to source, observes finite six-axis propagation/control/propulsion, and exercises reset/reconstruction and release where applicable. These are basic functional observations, not independent equipment-accuracy tests. No210-condition sweep or new unfiltered full headless/Release regression was run in this user-selected scope. Older outcomes were not relabeled as current passes; original assertions and reference data remain unchanged.

[Production table/package inventory](freshness-and-counts.json) covers {counts['native_xml_tables']} XML tables, {counts['native_xml_coefficient_cells']} coefficient cells and {counts['native_xml_axis_knots']} axis knots. {counts['consistency_predicates']} document consistency predicates are separate from execution and physics. [Independent source review](independent-review.json) is source review, not an independently executed flight campaign.

## Timing and reproduction

Production generation/install: {timing['elapsed_seconds']:.3f}s ({timing['generation_phase_seconds']:.3f}s generation, {timing['single_install_phase_seconds']:.3f}s install). Basic usability: {basic['elapsed_seconds']:.3f}s. Slowest generation worker: `{slow['name']}`, {slow['generation_seconds']:.3f}s; phases: `{json.dumps(slow_phases)}`. [Exact timing](delivery-timing.json). Reporting groups do not replace aggregate wall time. Historical controlled optimization and this changed-method workload are explicitly separate.

```powershell
$env:TMP='D:/ENV/tmp'
$env:TEMP='D:/ENV/tmp'
$env:TORCH_EXTENSIONS_DIR='D:/ENV/tmp'
$env:PYTHONPATH=(Join-Path $PWD 'tools/models/hangar')
$env:FSIM_BUILD_POWERSHELL=(Get-Process -Id $PID).Path
.\\setup.ps1
.\\tools\\launch\\fsim.ps1 --preset ucrt64-release configure -DFSIM_PYTHON_EXECUTABLE=D:/ENV/python/3.13/python.exe -DFSIM_CHECK_PYTHON=D:/ENV/python/3.13/python.exe -DFSIM_BUILD_PYTHON=ON -DFSIM_BUILD_TESTS=ON "-DCMAKE_PREFIX_PATH=$PWD/build/deps-ucrt64/install"
.\\tools\\launch\\fsim.ps1 --preset ucrt64-release build --parallel 8
& 'D:/ENV/python/3.13/python.exe' -B -m hangar engineering production --generation-workers 16 --geometry-threads 1 --output build/sim-m02-reproduce/production --prefix D:/ENV/sim-m02-reproduce --scheduling-profile tools/validation/aerodynamics/profiles/native-generation-20261031.json
& 'D:/ENV/python/3.13/python.exe' -B tools/validation/aerodynamics/engineering_basic.py --production build/sim-m02-reproduce/production --output build/sim-m02-reproduce/basic
& 'D:/ENV/python/3.13/python.exe' -B tools/validation/aerodynamics/engineering_package.py --output build/sim-m02-reproduce --export build/replay-src --review '{out/'independent-review.json'}'
```

Use a new output/install/export directory for a fresh reproduction. The source-bound review receipt is supplied in the evidence bundle; reuse is refused if any reviewed source changed. Production `--resume` accepts only fully current input/method/product/reference hashes. Native configure/build/test ownership remains exclusive and serial. The SDK archive is an installed SDK package; the existing project game/Viewer entrypoints remain available in the main workspace. No current visual-presentation acceptance or measured equipment fidelity is asserted by this package. Final archive-inclusive timing is in `evidence-archive.json`, which is generated alongside the ZIP to avoid self-referential archive hashes.
'''
    if '\ufffd' in text:raise ValueError('Invalid replacement character in delivery')
    (out/'delivery.md').write_text(text,encoding='utf-8')
    (out/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>SIM-M02 delivery</title><h1>SIM-M02 delivery</h1><p>'+paragraph+'</p><ul><li><a href="sim-m02-sdk.zip">Installed SDK package</a></li><li><a href="sim-m02.patch">One HEAD+patch source/product delivery</a></li><li><a href="delivery.md">Delivery details and commands</a></li><li><a href="production/index.html">All88 matrix</a></li><li><a href="table-method-inventory.csv">Complete table inventory</a></li><li><a href="evidence.zip">Evidence archive</a></li></ul>',encoding='utf-8')
    from engineering_report import archive_evidence
    archive_evidence(out)
    metrics['packaging_elapsed_seconds']=time.monotonic()-started
    metrics['current_generation_basic_and_packaging_seconds']=timing['elapsed_seconds']+basic['elapsed_seconds']+metrics['packaging_elapsed_seconds']
    archive_metadata=read(out/'evidence-archive.json')
    archive_metadata['delivery_timing']=metrics
    write(out/'evidence-archive.json',archive_metadata)
    print(json.dumps(dict(statuses=matrix['counts'],patch=patch['patch'],sdk=str(out/'sim-m02-sdk.zip'),
        elapsed_seconds=time.monotonic()-started),indent=2),flush=True)
    return 0


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',required=True);p.add_argument('--export',required=True)
    p.add_argument('--review',help='Actual independent source-review receipt; defaults to output/independent-review.json')
    raise SystemExit(main(p.parse_args()))
