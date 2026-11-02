"""Normal-entry fullscreen, real-time, complete-scene M01 observational runs.

Automated receipts are necessary; actual PNG inspection is a separate required
review. This driver never adds a headless, reduced-effect or accelerated branch.
"""
import argparse
import ctypes
from ctypes import wintypes
import hashlib
import json
import os
from pathlib import Path
import struct
import subprocess
import time
from PIL import ImageGrab

ROOT=Path(__file__).resolve().parents[3]


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def windows():
    user=ctypes.windll.user32;rows=[]
    callback=ctypes.WINFUNCTYPE(wintypes.BOOL,wintypes.HWND,wintypes.LPARAM)
    def visit(handle,_):
        if not user.IsWindowVisible(handle):return True
        title=ctypes.create_unicode_buffer(512);user.GetWindowTextW(handle,title,512)
        if 'flightsim' not in title.value.lower():return True
        rect=wintypes.RECT();user.GetWindowRect(handle,ctypes.byref(rect))
        rows.append(dict(title=title.value,handle=int(handle),rect=[rect.left,rect.top,rect.right,rect.bottom]))
        return True
    user.EnumWindows(callback(visit),0);return rows


def present_viewer():
    """Keep only this diagnostic Viewer above unrelated desktop windows."""
    user=ctypes.windll.user32;kernel=ctypes.windll.kernel32
    user.GetForegroundWindow.restype=wintypes.HWND
    user.SetForegroundWindow.argtypes=[wintypes.HWND]
    user.BringWindowToTop.argtypes=[wintypes.HWND]
    kernel.GetCurrentThreadId.restype=wintypes.DWORD
    user.SetWindowPos.argtypes=[wintypes.HWND,wintypes.HWND,ctypes.c_int,ctypes.c_int,ctypes.c_int,ctypes.c_int,wintypes.UINT]
    user.GetWindowThreadProcessId.argtypes=[wintypes.HWND,ctypes.POINTER(wintypes.DWORD)]
    rows=windows()
    if len(rows)!=1:raise RuntimeError('The single Viewer GUI slot is required')
    handle=rows[0]['handle']
    if not user.SetWindowPos(handle,wintypes.HWND(-1),0,0,0,0,0x0001|0x0002|0x0040):raise RuntimeError('Cannot keep the diagnostic Viewer visible')
    foreground=user.GetForegroundWindow();owner=user.GetWindowThreadProcessId(foreground,None) if foreground else 0
    current=kernel.GetCurrentThreadId();attached=bool(owner and owner!=current and user.AttachThreadInput(current,owner,True))
    try:
        user.BringWindowToTop(handle);user.SetForegroundWindow(handle)
    finally:
        if attached:user.AttachThreadInput(current,owner,False)
    if user.GetForegroundWindow()!=handle:raise RuntimeError('Actual Viewer foreground ownership is required')
    return rows[0]


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--powershell',default='powershell');a=p.parse_args();out=a.output.resolve()
    if out.exists() or any(x.casefold()=='.codex' for x in out.parts):raise ValueError('Fresh allowed output required')
    out.mkdir(parents=True);env=os.environ.copy()
    env.update(TMP='D:/ENV/tmp',TEMP='D:/ENV/tmp',TORCH_EXTENSIONS_DIR='D:/ENV/tmp')
    ctypes.windll.user32.SetProcessDPIAware()
    dimensions=[ctypes.windll.user32.GetSystemMetrics(i) for i in (0,1)]
    record=dict(schema='fsim.sim-m01.visual.v1',source_root=str(ROOT),primary_resolution=dimensions,
        binaries={name:sha(ROOT/'build/ucrt64-release/bin'/name) for name in ('flightsim-viewer.exe','libfsim.dll')},
        scenarios=[],automated_passed=False,actual_capture_review_required=True,
        isolation='non-isolated observational SDK diagnostic',physical_calibration=False,
        capture_method='Actual primary display through the complete native lifecycle; Viewer stays open through expiry')
    def save():(out/'visual.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
    prefix=[a.powershell,'-NoProfile','-ExecutionPolicy','Bypass','-File',str(ROOT/'tools/launch/fsim.ps1'),'--preset','ucrt64-release']
    save()
    for mode in ('fighter','aew'):
        ready=out/(mode+'-start');trace=out/(mode+'-trace.json');world='radar-fusion-'+mode
        scene=prefix+['python',str(ROOT/'examples/simulation/python/radar_fusion.py'),'--world',world,'--mode',mode,
            '--seconds','65','--output',str(trace),'--start-file',str(ready)]
        viewer=prefix+['viewer','--world',world,'--fullscreen','--clouds','scattered','--camera','chase',
            '--chase-distance','90' if mode=='aew' else '40','--chase-elevation','15',
            '--stats','70']
        row=dict(mode=mode,scene_command=scene,viewer_command=viewer);record['scenarios'].append(row);save()
        processes=[]
        try:
            with (out/(mode+'-scene.log')).open('wb') as s,(out/(mode+'-viewer.log')).open('wb') as v:
                producer=subprocess.Popen(scene,cwd=ROOT,env=env,stdout=s,stderr=subprocess.STDOUT);processes.append(producer)
                deadline=time.monotonic()+120
                while '"event": "ready"' not in (out/(mode+'-scene.log')).read_text(encoding='utf-8',errors='replace'):
                    if producer.poll() is not None or time.monotonic()>deadline:raise RuntimeError('Scene startup failed; see raw log')
                    time.sleep(.1)
                display=subprocess.Popen(viewer,cwd=ROOT,env=env,stdout=v,stderr=subprocess.STDOUT);processes.append(display)
                deadline=time.monotonic()+120
                while True:
                    shown=windows()
                    if shown:break
                    if display.poll() is not None or time.monotonic()>deadline:raise RuntimeError('Viewer startup failed; see raw log')
                    time.sleep(.1)
                row['actual_windows']=shown
                if not any(r['rect']==[0,0,*dimensions] for r in shown):raise RuntimeError('Viewer is not fullscreen at primary resolution')
                present_viewer()
                ready.write_text('normal Viewer window verified\n',encoding='utf-8')
                capture_start=time.monotonic()
                for index,elapsed in enumerate((10,20,32,46,62)):
                    while time.monotonic()-capture_start<elapsed:
                        if producer.poll() is not None or display.poll() is not None:raise RuntimeError('Complete visual lifecycle exited early')
                        time.sleep(max(0,min(.1,elapsed-(time.monotonic()-capture_start))))
                    if not any(r['rect']==[0,0,*dimensions] for r in windows()):raise RuntimeError('Actual Viewer lost primary fullscreen presentation')
                    presented=present_viewer();time.sleep(.15)
                    image=ImageGrab.grab(all_screens=False)
                    if ctypes.windll.user32.GetForegroundWindow()!=presented['handle']:raise RuntimeError('Foreground changed during actual display capture')
                    if list(image.size)!=dimensions:raise RuntimeError('Actual primary capture resolution changed')
                    image.save(out/(mode+'_'+str(index).zfill(4)+'.png'))
                row['scene_exit']=producer.wait(timeout=150);row['viewer_exit']=display.wait(timeout=150)
            if row['scene_exit'] or row['viewer_exit']:raise RuntimeError('Normal entry failed')
            data=json.loads(trace.read_text(encoding='utf-8'))
            row['real_time_factor']=data['real_time_factor'];row['trace_sha256']=sha(trace)
            if not .95<=data['real_time_factor']<=1.02:raise RuntimeError('Scene did not sustain real time 1x')
            if not any(s['sensor'] for s in data['samples']) or not any(s['receiver'] for s in data['samples']):raise RuntimeError('Missing actual native sensor/link fusion')
            if not any({source['kind'] for source in e['sources']}=={0,1} for s in data['samples'] for e in s['receiver']):raise RuntimeError('Missing actual multi-source radar/link covariance intersection')
            if not any(e['state']==2 for s in data['samples'] for e in s['receiver']):raise RuntimeError('No actual lost fused entity')
            if data['samples'][-1]['receiver']:raise RuntimeError('Expired fusion retention not observed')
            images=[]
            for image in sorted(out.glob(mode+'_*.png')):
                size=list(struct.unpack('>II',image.read_bytes()[16:24]))
                if size!=dimensions:raise RuntimeError('Actual capture resolution differs from primary display')
                images.append(dict(path=str(image),sha256=sha(image),resolution=size))
            if len(images)!=5:raise RuntimeError('Incomplete capture sequence')
            row.update(images=images,automated_passed=True);save()
        finally:
            for process in processes:
                if process.poll() is None:subprocess.run(['taskkill','/PID',str(process.pid),'/T','/F'],capture_output=True,timeout=30)
    record['automated_passed']=all(r['automated_passed'] for r in record['scenarios']);save()
    print(json.dumps(record));return 0


if __name__=='__main__':raise SystemExit(main())
