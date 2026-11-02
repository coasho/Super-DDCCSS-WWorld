"""Actual loaded SDK/JSBSim identity for Windows native consumer evidence."""
import argparse
import ctypes
import hashlib
import json
from pathlib import Path


def loaded_modules(expected_prefix):
    prefix=Path(expected_prefix).resolve()
    kernel=ctypes.WinDLL('kernel32',use_last_error=True)
    kernel.GetModuleHandleW.argtypes=[ctypes.c_wchar_p];kernel.GetModuleHandleW.restype=ctypes.c_void_p
    kernel.GetModuleFileNameW.argtypes=[ctypes.c_void_p,ctypes.c_wchar_p,ctypes.c_uint32]
    kernel.GetModuleFileNameW.restype=ctypes.c_uint32
    result=[]
    for name in ('libfsim.dll','libJSBSim.dll','JSBSim.dll'):
        handle=kernel.GetModuleHandleW(name)
        if not handle:continue
        buffer=ctypes.create_unicode_buffer(32768)
        length=kernel.GetModuleFileNameW(handle,buffer,len(buffer))
        if not length or length>=len(buffer):raise OSError('Cannot identify loaded '+name)
        path=Path(buffer.value)
        if any(p.casefold()=='.codex' for p in path.parts) or not path.is_relative_to(prefix):
            raise RuntimeError('Loaded native module is outside the expected product: '+str(path))
        path=path.resolve()
        if not path.is_relative_to(prefix):raise RuntimeError('Redirected native product module')
        result.append(dict(name=name,path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    if not any(m['name']=='libfsim.dll' for m in result) or not any('JSBSim' in m['name'] for m in result):
        raise RuntimeError('Required SDK/JSBSim native modules not loaded')
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--expected-prefix',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    import fsim
    if not Path(fsim.__file__).resolve().is_relative_to(a.expected_prefix.resolve()):raise RuntimeError('Python facade outside product')
    native=Path(fsim._native.__file__).resolve()
    if not native.is_relative_to(a.expected_prefix.resolve()):raise RuntimeError('Native Python module outside product')
    record=dict(schema='fsim.sim-m01.loaded-modules.v1',python_package=fsim.__file__,native_python=str(native),
        modules=loaded_modules(a.expected_prefix))
    a.output.write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8');print(json.dumps(record))


if __name__=='__main__':main()
