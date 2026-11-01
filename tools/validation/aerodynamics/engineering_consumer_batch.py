"""Execute the unchanged consumer probe in one isolated process per SDK.

Every design retains its own poison/refusal checks, Worlds and reconstruction.
Only Python/native-library startup is shared. No simultaneous Worlds or probes.
"""
import argparse
import contextlib
import gc
import hashlib
import json
from pathlib import Path
import os
import re
import runpy
import sys
import subprocess
import time
import importlib.util

_scope_spec=importlib.util.spec_from_file_location('engineering_scope',Path(__file__).with_name('engineering_scope.py'))
_scope_module=importlib.util.module_from_spec(_scope_spec)
_scope_spec.loader.exec_module(_scope_module)
functional_rows=_scope_module.functional_rows


# A separate minimal watchdog can enforce the deadline while native code owns
# the Python GIL. It waits on the exact parent process handle, without polling.
WATCHDOG = r'''
import ctypes,sys,time
from ctypes import wintypes as w
k=ctypes.WinDLL('kernel32',use_last_error=True)
k.OpenProcess.argtypes=[w.DWORD,w.BOOL,w.DWORD];k.OpenProcess.restype=w.HANDLE
k.GetProcessTimes.argtypes=[w.HANDLE,ctypes.POINTER(w.FILETIME),ctypes.POINTER(w.FILETIME),ctypes.POINTER(w.FILETIME),ctypes.POINTER(w.FILETIME)]
k.WaitForSingleObject.argtypes=[w.HANDLE,w.DWORD];k.WaitForSingleObject.restype=w.DWORD
k.TerminateProcess.argtypes=[w.HANDLE,w.UINT]
k.CloseHandle.argtypes=[w.HANDLE]
k.OpenEventW.argtypes=[w.DWORD,w.BOOL,w.LPCWSTR];k.OpenEventW.restype=w.HANDLE
k.SetEvent.argtypes=[w.HANDLE]
h=k.OpenProcess(0x101001,False,int(sys.argv[1]))
if not h:sys.exit(2)
try:
    created,exited,kernel,user=(w.FILETIME() for _ in range(4))
    if not k.GetProcessTimes(h,ctypes.byref(created),ctypes.byref(exited),ctypes.byref(kernel),ctypes.byref(user)):sys.exit(3)
    identity=(created.dwHighDateTime<<32)|created.dwLowDateTime
    if identity!=int(sys.argv[2]):sys.exit(4)
    event=k.OpenEventW(2,False,sys.argv[4])
    if not event:sys.exit(6)
    ready=k.SetEvent(event);k.CloseHandle(event)
    if not ready:sys.exit(7)
    remaining=max(0,int((float(sys.argv[3])-time.monotonic())*1000))
    status=k.WaitForSingleObject(h,remaining)
    if status==258:k.TerminateProcess(h,124)
    elif status!=0:sys.exit(5)
finally:k.CloseHandle(h)
'''


def process_identity():
    import ctypes
    from ctypes import wintypes as w
    kernel=ctypes.WinDLL('kernel32',use_last_error=True)
    kernel.GetCurrentProcess.restype=w.HANDLE
    kernel.GetProcessTimes.argtypes=[w.HANDLE,*([ctypes.POINTER(w.FILETIME)]*4)]
    values=[w.FILETIME() for _ in range(4)]
    if not kernel.GetProcessTimes(kernel.GetCurrentProcess(),*(ctypes.byref(v) for v in values)):
        raise RuntimeError('Cannot establish bounded consumer identity')
    return (values[0].dwHighDateTime<<32)|values[0].dwLowDateTime


def deadline_guard(identity,deadline,stream):
    import ctypes
    import uuid
    from ctypes import wintypes as w
    kernel=ctypes.WinDLL('kernel32',use_last_error=True)
    kernel.CreateEventW.argtypes=[ctypes.c_void_p,w.BOOL,w.BOOL,w.LPCWSTR];kernel.CreateEventW.restype=w.HANDLE
    kernel.WaitForSingleObject.argtypes=[w.HANDLE,w.DWORD];kernel.WaitForSingleObject.restype=w.DWORD
    kernel.CloseHandle.argtypes=[w.HANDLE]
    name='Local\\fsim-consumer-'+uuid.uuid4().hex
    event=kernel.CreateEventW(None,False,False,name)
    if not event:raise RuntimeError('Cannot establish consumer watchdog handshake')
    guard=None
    try:
        guard=subprocess.Popen([sys.executable,'-I','-B','-c',WATCHDOG,str(os.getpid()),str(identity),str(deadline),name],
            stdout=stream,stderr=subprocess.STDOUT)
        if kernel.WaitForSingleObject(event,10000)!=0:
            if guard.poll() is None:guard.terminate()
            guard.wait(timeout=10)
            raise RuntimeError('Consumer watchdog did not become ready')
        return guard
    finally:kernel.CloseHandle(event)


def main(args):
    started=time.monotonic()
    if os.name!='nt':raise ValueError('This bounded batch worker requires the Windows UCRT64 runtime')
    if not re.fullmatch('[0-9a-f]{32}',args.run_id):raise ValueError('Invalid fresh consumer run identity')
    identity=process_identity()
    for value in (args.production,args.output,args.sdk,args.data,args.probe):
        if '.codex' in {part.casefold() for part in Path(value).parts}:raise ValueError('Excluded path')
    production=Path(args.production).resolve();output=Path(args.output).resolve()
    sdk=Path(args.sdk).resolve();data=Path(args.data).resolve();probe=Path(args.probe).resolve()
    for path in (production,output,sdk,data,probe):
        if '.codex' in {part.casefold() for part in path.parts}:raise ValueError('Excluded path')
    rows=json.loads((production/'matrix.json').read_text(encoding='utf-8'))['designs']
    if len(rows)!=88 or len({row['name'] for row in rows})!=88:raise ValueError('Exactly88 unique designs required')
    rows,deferred=functional_rows(rows,args.functional_scope)
    records=[]
    def publish():
        temporary=output.with_name(output.name+'.'+args.run_id+'.tmp')
        value=json.dumps(dict(schema='fsim.sim-m02.consumer-batch.v1',mode=args.mode,run_id=args.run_id,
            functional_scope=args.functional_scope,deferred_designs=[r['name'] for r in deferred],
            scope_implementation_sha256=hashlib.sha256(Path(__file__).with_name('engineering_scope.py').read_bytes()).hexdigest(),
            probe_sha256=hashlib.sha256(probe.read_bytes()).hexdigest(),sdk=str(sdk),data=str(data),
            worker_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            elapsed_seconds=time.monotonic()-started,records=records),indent=2)+'\n'
        with temporary.open('w',encoding='utf-8',newline='\n') as stream:
            stream.write(value);stream.flush();os.fsync(stream.fileno())
        os.replace(temporary,output)
    publish()
    for row in rows:
        name=row['name'];folder=production/name
        if not re.fullmatch(r'[A-Za-z0-9_-]+',name):raise ValueError('Invalid consumer design identity')
        target=data/'aircraft'/row['product_directory']/(name+'.xml')
        if '.codex' in {part.casefold() for part in target.parts}:raise ValueError('Excluded target')
        if not folder.resolve().is_relative_to(production):raise ValueError('Consumer output escapes production root')
        if not target.resolve().is_relative_to(data):raise ValueError('Consumer target escapes data root')
        original=target.read_bytes();result=folder/(args.mode+'.json')
        # A failed invocation can never reuse a prior result file.
        result.unlink(missing_ok=True)
        tick=time.monotonic();code=0;error=None
        log=output.parent/(name+'-'+args.mode+'.log')
        argv=[str(probe),'--sdk',str(sdk),'--data',str(data),'--receipt',str(folder/'generation.json'),'--output',str(result)]
        previous=sys.argv
        # Preserve the original180s deadline even when native code owns GIL.
        # The parent wrapper restores XML and rejects incomplete manifests.
        guard=None
        try:
            with log.open('w',encoding='utf-8',buffering=1) as stream,contextlib.redirect_stdout(stream),contextlib.redirect_stderr(stream):
                sys.argv=argv
                guard=deadline_guard(identity,tick+180.,stream)
                try:runpy.run_path(str(probe),run_name='__main__')
                except SystemExit as exit_code:
                    code=exit_code.code if isinstance(exit_code.code,int) else (0 if exit_code.code is None else 1)
                except Exception as exc:
                    import traceback
                    traceback.print_exc();code=1;error=str(exc)
        except Exception as exc:
            code=1;error=str(exc)
        finally:
            sys.argv=previous
            try:
                if not target.is_file() or target.read_bytes()!=original:target.write_bytes(original)
                gc.collect()
                if guard is not None and guard.poll() is not None:
                    code=1;error='Consumer deadline watchdog exited unexpectedly'
                records.append(dict(name=name,exit_code=code,error=error,argv=argv,log=str(log),
                    elapsed_seconds=time.monotonic()-tick,
                    output_sha256=hashlib.sha256(result.read_bytes()).hexdigest() if result.is_file() else None))
                publish()
            finally:
                if guard is not None:
                    if guard.poll() is None:guard.terminate()
                    guard.wait(timeout=10)
        print('consumer '+args.mode+' '+name+' '+str(code),flush=True)
    return int(any(record['exit_code'] for record in records))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('production','sdk','data','probe','output'):parser.add_argument('--'+name,required=True)
    parser.add_argument('--mode',choices=('native','installed'),required=True)
    parser.add_argument('--run-id',required=True)
    parser.add_argument('--functional-scope',choices=('all','sim-m02-nonrotor-v1'),default='all')
    raise SystemExit(main(parser.parse_args()))
