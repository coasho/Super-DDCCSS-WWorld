"""Export exact HEAD plus one binary patch without touching the real Git index.

Includes explicitly listed SDK/Hangar prerequisites already present in main's
dirty tree. Viewer presentation changes are excluded, including their shared
CMake/scope hunks. No branch, commit or Git worktree is created.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tarfile
import time

ROOT=Path(__file__).resolve().parents[3]
SKIP={'.codex','.git','__pycache__','out','build','.local','.cache'}
ROOTS=['tools/models/hangar/hangar','tools/models/hangar/native','tools/models/hangar/tests','tools/models/hangar/data',
       'tools/validation/aerodynamics','tools/validation/missile','tools/validation/testing','tools/validation/native_flight_probe',
       'src/simulation','tests/simulation','tests/compatibility/python','tools/bootstrap','tools/tests']
FILES=['CMakeLists.txt','.gitattributes','.gitignore','README.md','setup.ps1','build.ps1','run.ps1','stop.ps1',
       'tools/CMakeLists.txt','assets/payloads/h6k-store-station.json',
       'docs/data/hangar-vlm-errors-status.csv','docs/data/hangar-vlm-errors.html','docs/data/hangar-aam-bindings.json',
       'cmake/Assets.cmake','cmake/StageDesigns.cmake','cmake/JsbsimPatches.cmake','cmake/SerialVerification.cmake',
       'third_party/jsbsim/COPYING','third_party/vsgXchange/src/bin/bin.cpp',
       'cmake/TestDistComponents.cmake','cmake/ArchitectureChecks.cmake','python/CMakeLists.txt',
       'tools/architecture/policy.json','tests/CMakeLists.txt','docs/status.md',
       'docs/hangar-system.md','docs/sdk/missile-flight-validation.md',
       'docs/sdk/weapon-fidelity-validation.md','docs/architecture/modules/simulation/plan.md',
       'docs/superpowers/plans/2026-10-30-sim-m02-engineering-completeness.md',
       'docs/data/sim-m02-engineering-completeness.csv','docs/README.md',
       'docs/architecture/README.md','docs/architecture/migration/document-register.csv',
       'docs/development.md','docs/resources.md','docs/rotorcraft.md',
       'docs/hangar.md','docs/hangar-stores-build.md','docs/hangar-stores-build.json',
       'docs/sdk/missile-control-validation.md','docs/sdk/representative-weapon-models.md']


def git(*args,env=None,input=None):
    return subprocess.run(['git',*args],cwd=ROOT,env=env,input=input,stdout=subprocess.PIPE,stderr=subprocess.PIPE,check=True).stdout


def walk(root):
    for here,dirs,files in os.walk(root,followlinks=False):
        dirs[:]=sorted(d for d in dirs if d.casefold() not in SKIP and not (Path(here)/d).is_symlink())
        for name in sorted(files):
            path=Path(here)/name
            if not path.is_symlink():yield path


def prepared(relative):
    content=(ROOT/relative).read_bytes()
    if relative=='tools/validation/missile/data/aim7m_drev_tm1999_050.json':
        if not git('ls-tree','HEAD','--',relative):
            # This reference is a pre-existing untracked prerequisite, not a
            # HEAD modification. Package its unchanged approved working bytes.
            return content
        original=git('show','HEAD:'+relative)
        if content.replace(b'\r\n',b'\n')!=original.replace(b'\r\n',b'\n'):
            raise ValueError('Protected DREV reference differs from HEAD')
        # Preserve the reference byte-for-byte, including original line endings.
        return original
    if relative=='tests/CMakeLists.txt':
        start=content.index(b'    add_executable(fsim_mandatory_presentation_test')
        end=content.index(b'    add_executable(fsim_render_tests',start)
        content=content[:start]+content[end:]
        # Preserve HEAD's original native timeout in the delivered replay.
        # Main's pre-existing scheduling edit is retained in the user's tree.
        content=content.replace(
            b'set_tests_properties(missile_control_native PROPERTIES LABELS "native-engagement" TIMEOUT 300 PROCESSORS 2)',
            b'set_tests_properties(missile_control_native PROPERTIES LABELS "native-engagement" TIMEOUT 120)')
    if relative=='tools/validation/testing/scopes.toml':
        content=b''.join(line for line in content.splitlines(keepends=True) if b'tests/visualization/test_mandatory_presentation.cpp' not in line)
    return content


def main(args):
    started=time.monotonic()
    out=Path(args.output).resolve();out.mkdir(parents=True,exist_ok=True)
    export=Path(args.export).resolve()
    if export.exists():raise ValueError('Replay export must be a new empty path')
    if not export.is_relative_to(ROOT/'build'):raise ValueError('Export is an ordinary source directory under this main checkout build tree')
    head=git('rev-parse','HEAD').decode().strip()
    if git('branch','--show-current').decode().strip()!='main':raise ValueError('main required')
    real_index=ROOT/'.git/index';index_before=hashlib.sha256(real_index.read_bytes()).hexdigest()
    matrix=json.loads(Path(args.matrix).read_text(encoding='utf-8'))
    paths=set(FILES)
    for root in ROOTS:
        paths.update(p.relative_to(ROOT).as_posix() for p in walk(ROOT/root))
    # Preserve all tracked authored changes/deletions in these explicit roots.
    scopes=ROOTS+['aircraft','assets/stores']+FILES
    paths.update(git('diff','--name-only','-z','HEAD','--',*scopes,':(exclude,icase,glob)**/.codex/**').decode().split('\0'))
    candidates=[]
    for name in sorted(paths-{''}):
        if any(part.casefold()=='.codex' for part in Path(name).parts):raise ValueError('Excluded directory in allowlist')
        if (ROOT/name).is_file():candidates.append(name)
    # Ignore caches/downloads, but explicitly include receipt-listed products.
    ignored=subprocess.run(['git','check-ignore','--stdin','-z'],cwd=ROOT,input=('\0'.join(candidates)+'\0').encode(),stdout=subprocess.PIPE).stdout
    paths.difference_update(ignored.decode().split('\0'))
    # Explicit reviewed prerequisites can themselves be ignored by Git (the
    # bootstrap-provisioned upstream auxiliary source/license files are). The
    # ordinary cache filter must not silently drop this declared input closure.
    paths.update(FILES)
    for row in matrix['designs']:
        paths.update(row['input_sha256'])
        paths.update('aircraft/'+p for p in row['products'])
        directory=ROOT/'aircraft'/row['name']
        for pattern in ('*.toml','*.dat','*.glb','*.glb.manifest','*.damage.json','rcs.bin','payloads.json'):
            paths.update(p.relative_to(ROOT).as_posix() for p in directory.glob(pattern))
        for relative in ('flight-calibration.json','out/fly.json','out/performance.json','out/ground.json'):
            if (directory/relative).is_file():paths.add((directory/relative).relative_to(ROOT).as_posix())
        if (directory/'pieces').is_dir():
            paths.update(p.relative_to(ROOT).as_posix() for p in walk(directory/'pieces') if p.suffix=='.xml')
        product=ROOT/'aircraft'/row['product_directory']
        for pattern in ('*.toml','*.dat'):
            paths.update(p.relative_to(ROOT).as_posix() for p in product.glob(pattern))
    # Raw receipts/logs/review form a separate evidence archive. Keeping them
    # outside the source patch avoids self-referential patch hashes and allows
    # the delivered patch to contain exactly source, inputs and runtime products.
    paths={Path(name).as_posix() for name in paths if name}
    for name in paths:
        relative=Path(name)
        if relative.is_absolute() or '..' in relative.parts or '.codex' in {p.casefold() for p in relative.parts}:
            raise ValueError('Nonlocal/excluded patch path: '+name)
    index=out/'temporary-index';env=os.environ.copy();env['GIT_INDEX_FILE']=str(index)
    auxiliary=[]
    lock=json.loads((ROOT/'tools/bootstrap/dependencies.lock.json').read_text(encoding='utf-8'))
    for item in lock.get('source_files',[]):
        if item['destination'] not in FILES:continue
        payload=(ROOT/item['destination']).read_bytes();digest=hashlib.sha256(payload).hexdigest()
        if digest not in [item['sha256'],*item.get('accepted_sha256',[])]:
            raise ValueError('Unpinned bootstrap auxiliary input: '+item['destination'])
        auxiliary.append(dict(path=item['destination'],sha256=digest,version=item['version'],source=item['url']))
    if index.exists():index.unlink()
    git('read-tree',head,env=env)
    entries=[];updates=[]
    for name in sorted(paths):
        path=ROOT/name
        if path.is_file():
            content=prepared(name);blob=git('hash-object','-w','--no-filters','--stdin',input=content).decode().strip()
            entry=git('ls-tree',head,'--',name).decode();mode=entry.split()[0] if entry else '100644'
            updates.append(f'{mode} {blob}\t{name}\n')
            entries.append(dict(path=name,sha256=hashlib.sha256(content).hexdigest(),bytes=len(content),prepared_shared_hunks=name in ('tests/CMakeLists.txt','tools/validation/testing/scopes.toml')))
        else:
            updates.append('0 '+'0'*40+'\t'+name+'\n');entries.append(dict(path=name,deleted=True))
    git('update-index','--index-info',env=env,input=''.join(updates).encode())
    changed=[p for p in git('diff','--cached','--no-renames','--name-only','-z',head,'--',env=env).decode().split('\0') if p]
    (out/'patch-stat.txt').write_bytes(git('diff','--cached','--no-renames','--stat',head,'--',env=env))
    (out/'patch-files.txt').write_bytes(git('diff','--cached','--no-renames','--name-status',head,'--',env=env))
    patch=out/'sim-m02.patch'
    with patch.open('wb') as stream:
        subprocess.run(['git','diff','--cached','--no-renames','--binary','--full-index',head,'--'],cwd=ROOT,env=env,stdout=stream,check=True)
    archive=Path('D:/ENV/tmp')/('sim-m02-head-'+head[:12]+'.tar')
    subprocess.run(['git','archive','--format=tar','--output='+str(archive),head,'.',':(exclude,icase,glob)**/.codex/**'],cwd=ROOT,check=True)
    export.mkdir(parents=True)
    with tarfile.open(archive) as source:
        members=(m for m in source if '.codex' not in {part.casefold() for part in Path(m.name).parts})
        source.extractall(export,members=members,filter='data')
    apply_env={k:v for k,v in os.environ.items() if not k.upper().startswith('GIT_')}
    # Treat this directory as an export even though it is nested below main.
    # Git apply must not discover and target the enclosing dirty checkout.
    apply_env['GIT_CEILING_DIRECTORIES']=str(export.parent)
    for apply_args in (['--check'],[]):
        subprocess.run(['git','-c','core.autocrlf=false','-c','core.safecrlf=false',
            'apply','--binary','--whitespace=nowarn',*apply_args,str(patch)],cwd=export,env=apply_env,check=True)
    failures=[]
    for entry in entries:
        path=export/entry['path']
        if entry.get('deleted'):
            if path.exists():failures.append(entry['path'])
        elif not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=entry['sha256']:failures.append(entry['path'])
    index_after=hashlib.sha256(real_index.read_bytes()).hexdigest()
    if index.exists():index.unlink()
    result=dict(base_head=head,patch=str(patch),patch_sha256=hashlib.sha256(patch.read_bytes()).hexdigest(),patch_bytes=patch.stat().st_size,
        elapsed_seconds=time.monotonic()-started,bootstrap_auxiliary_inputs=auxiliary,
        source_checkout=str(ROOT),source_export=str(export),export_is_git_worktree=False,real_index_unchanged=index_before==index_after,
        byte_verification_failures=failures,snapshot_file_count=len(entries),patch_changed_file_count=len(changed),patch_changed_paths=changed,files=entries,
        excluded_unrelated=['assets/config/viewer.json','src/visualization/**','src/applications/viewer/**','tests/visualization/**','mandatory_presentation CMake/scope hunks','pre-existing missile-control timeout increase (replay retains HEAD 120 seconds)'],
        prerequisites='Includes pre-existing SDK/Hangar native provider, carriage, integration, authored-design and bootstrap changes. This is a dependency snapshot, not a claim that every hunk was authored in this correction.')
    (out/'patch-manifest.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k!='files'},indent=2))
    return int(bool(failures) or index_before!=index_after)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--matrix',required=True);p.add_argument('--output',required=True);p.add_argument('--export',required=True)
    raise SystemExit(main(p.parse_args()))
