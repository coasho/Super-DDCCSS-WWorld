"""Explicit normal-production method environment, independent of report shell."""
import json
import os
from pathlib import Path
import subprocess
import sys

from acceptance import allowed

SCHEMA='fsim.sim-m03.method-environment.v1'
KEYS={'schema','source_root','python_executable','python_test_path','ucrt64_root',
      'meshkit_directory','mplconfigdir','xdg_cache_home','system_root'}
SCRIPT="""import json,sys
from pathlib import Path
from hangar import method_backends
expected=Path(sys.argv[1])/'tools/models/hangar/hangar/method_backends.py'
if Path(method_backends.__file__).resolve()!=expected.resolve():
    raise ValueError('Backend verifier imported from another source export')
print(json.dumps(method_backends.snapshot(),sort_keys=True))
"""


def path(value):
    return allowed(value).resolve()


def freeze(source,dependencies,interpreter=sys.executable,ucrt64_root=None):
    source=path(source)
    record=dict(schema=SCHEMA,source_root=str(source),python_executable=str(path(interpreter)),
        python_test_path=str(path(dependencies)),
        ucrt64_root=str(path(ucrt64_root or os.environ.get('FSIM_UCRT64_ROOT','D:/ENV/DevLanguages/Cpp/msys2/ucrt64'))),
        meshkit_directory=str(source/'build/ucrt64-release/python'),
        mplconfigdir=str(path('D:/ENV/cache/fsim-m03/matplotlib')),
        xdg_cache_home=str(path('D:/ENV/cache/fsim-m03')),
        system_root=str(path(os.environ.get('SystemRoot','C:/Windows'))))
    environment(record,source,parent={})
    return record


def environment(record,source,*,parent=None):
    source=path(source)
    if not isinstance(record,dict) or set(record)!=KEYS or record['schema']!=SCHEMA:
        raise ValueError('Complete explicit method environment required')
    if path(record['source_root'])!=source or path(record['meshkit_directory'])!=source/'build/ucrt64-release/python':
        raise ValueError('Method environment belongs to another source/model backend')
    for key in ('python_executable','python_test_path','ucrt64_root','mplconfigdir','xdg_cache_home'):
        if not path(record[key]).is_relative_to(path('D:/ENV')):
            raise ValueError('Method tools, dependencies and caches must stay under D:/ENV')
    system=path(record['system_root']);python=path(record['python_executable'])
    result=dict(os.environ if parent is None else parent)
    # Exact source/dependency selection replaces inherited Python paths. Keep
    # caller PATH after declared tools for production Git/CMake invocations;
    # verification uses an empty parent and needs only the declared tools/OS.
    roots=[path(record['ucrt64_root'])/'bin',python.parent,system/'System32',system]
    result.update(PYTHONPATH=os.pathsep.join(map(str,[source/'tools/models/hangar',
        path(record['meshkit_directory']),path(record['python_test_path'])])),
        HANGAR_MESHKIT_DIR=record['meshkit_directory'],MPLCONFIGDIR=record['mplconfigdir'],
        XDG_CACHE_HOME=record['xdg_cache_home'],MPLBACKEND='Agg',
        PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',PYTHONUTF8='1',
        TMP='D:/ENV/tmp',TEMP='D:/ENV/tmp',TORCH_EXTENSIONS_DIR='D:/ENV/tmp',
        SystemRoot=str(system),WINDIR=str(system),
        PATH=os.pathsep.join([*map(str,roots),result.get('PATH','')]))
    return result


def capture(record,source,*,timeout=180):
    source=path(source)
    configured=environment(record,source,parent={})
    result=subprocess.run([str(path(record['python_executable'])),'-B','-c',SCRIPT,str(source)],
        cwd=source,env=configured,stdout=subprocess.PIPE,stderr=subprocess.PIPE,
        timeout=timeout,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0),check=False)
    if result.returncode:
        raise ValueError('Current backend verification failed: '+result.stderr.decode('utf-8',errors='replace')[-6000:])
    snapshot=json.loads(result.stdout.decode('utf-8'))
    if not isinstance(snapshot,dict) or snapshot.get('schema')!='fsim.hangar.method-backends.v1':
        raise ValueError('Current backend verifier returned no complete snapshot')
    return snapshot
