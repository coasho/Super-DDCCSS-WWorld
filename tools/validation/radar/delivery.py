"""HEAD plus one exact-byte M01 patch; ordinary export, no branch/worktree.

Only explicitly owned paths enter the temporary index. Unrelated changes stay
in the real workspace and never enter the patch. Receipts are outside it.
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
GUIDE='docs/sdk/mission-systems.md'
MARKER=b'\n<!-- SIM-M01 V1 extension begin -->'


def sha(data):return hashlib.sha256(data).hexdigest()
def git(*args,env=None,input=None):
    return subprocess.run(['git',*args],cwd=ROOT,env=env,input=input,stdout=subprocess.PIPE,stderr=subprocess.PIPE,check=True).stdout


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--window',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--export',type=Path,required=True);a=p.parse_args()
    for path in (a.window,a.output,a.export):
        if any(s.casefold()=='.codex' for s in path.parts):raise ValueError('Excluded path')
    out=a.output.resolve();out.mkdir(parents=True,exist_ok=True);export=a.export.resolve()
    if export.exists() or not export.is_relative_to(ROOT/'build'):raise ValueError('New ordinary export under build required')
    window=json.loads(a.window.read_text(encoding='utf-8'));head=git('rev-parse','HEAD').decode().strip()
    if head!=window['base_head'] or git('branch','--show-current').decode().strip()!='main':raise ValueError('Recorded main HEAD required')
    paths=window['scope']
    if any(Path(n).is_absolute() or '..' in Path(n).parts or any(s.casefold()=='.codex' for s in Path(n).parts) for n in paths):
        raise ValueError('Nonlocal/excluded owned path')
    logical=set(git('diff','--name-only','-z',head,'--',*paths).decode().split('\0'))
    index=out/'private-index';env=os.environ.copy();env['GIT_INDEX_FILE']=str(index)
    if index.exists():raise ValueError('Private index already exists')
    index_before=sha((ROOT/'.git/index').read_bytes());git('read-tree',head,env=env)
    entries=[];updates=[];skipped=[];preexisting=window['preexisting']['files']
    for name in sorted(set(paths)):
        path=ROOT/name;tracked=git('ls-tree',head,'--',name)
        if not path.exists() and not tracked:continue
        content=path.read_bytes() if path.is_file() else None
        if name in preexisting:
            if (sha(content) if content is not None else None)==preexisting[name]:skipped.append(name);continue
            if name!=GUIDE or content is None or MARKER not in content:raise ValueError('Review mixed-owner edits before packaging: '+name)
            prefix,extension=content.split(MARKER,1)
            if sha(prefix)!=window.get('guide_prefix_sha256',preexisting[name]):raise ValueError('Preexisting guide bytes changed')
            # A continuation may replace its already committed extension. Keep
            # the original HEAD guide prefix and exactly one owned marker.
            content=git('show',head+':'+name).split(MARKER,1)[0]+MARKER+extension
        elif tracked and name not in logical:continue
        if content is None:
            updates.append('0 '+'0'*40+'\t'+name+'\n');entries.append(dict(path=name,deleted=True));continue
        mode=tracked.decode().split()[0] if tracked else '100644'
        blob=git('hash-object','-w','--no-filters','--stdin',input=content).decode().strip()
        updates.append(f'{mode} {blob}\t{name}\n');entries.append(dict(path=name,sha256=sha(content),bytes=len(content),prepared_shared_hunks=name==GUIDE))
    git('update-index','--index-info',env=env,input=''.join(updates).encode())
    patch=out/'sim-m01.patch';patch.write_bytes(git('diff','--cached','--binary','--full-index','--no-renames',head,'--',env=env))
    (out/'patch-stat.txt').write_bytes(git('diff','--cached','--stat','--no-renames',head,'--',env=env))
    (out/'patch-files.txt').write_bytes(git('diff','--cached','--name-status','--no-renames',head,'--',env=env))
    archive=Path('D:/ENV/tmp')/('sim-m01-head-'+str(time.time_ns())+'.tar')
    git('archive','--format=tar','--output='+str(archive),head,'.',':(exclude,icase,glob)**/.codex/**')
    export.mkdir(parents=True)
    with tarfile.open(archive) as source:
        members=(m for m in source if not any(s.casefold()=='.codex' for s in Path(m.name).parts) and
            not any(s.casefold()=='.codex' for s in Path(m.linkname).parts))
        source.extractall(export,members=members,filter='data')
    apply_env={k:v for k,v in os.environ.items() if not k.upper().startswith('GIT_')};apply_env['GIT_CEILING_DIRECTORIES']=str(export.parent)
    for extra in (['--check'],[]):
        subprocess.run(['git','-c','core.autocrlf=false','-c','core.safecrlf=false','apply','--binary','--whitespace=nowarn',*extra,str(patch)],
            cwd=export,env=apply_env,check=True)
    failures=[]
    for row in entries:
        path=export/row['path']
        if row.get('deleted'):
            if path.exists():failures.append(row['path'])
        elif not path.is_file() or sha(path.read_bytes())!=row['sha256']:failures.append(row['path'])
    unchanged=sha((ROOT/'.git/index').read_bytes())==index_before
    index.unlink()
    result=dict(schema='fsim.sim-m01.patch-replay.v1',base_head=head,patch=str(patch),patch_sha256=sha(patch.read_bytes()),
        bytes=patch.stat().st_size,source_export=str(export),export_is_git_worktree=False,real_index_unchanged=unchanged,
        replay_byte_failures=failures,files=entries,excluded_preexisting=skipped,
        mixed_guide_policy='Only the appended M01 marker block is applied to HEAD; preexisting guide edits remain in the real workspace')
    (out/'patch-manifest.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('files','excluded_preexisting')}))
    raise SystemExit(bool(failures) or not unchanged)


if __name__=='__main__':main()
