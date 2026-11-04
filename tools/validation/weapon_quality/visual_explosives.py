"""Actual primary fullscreen 1x presentation of both other explosive stores."""
import argparse
import ctypes
import json
import os
from pathlib import Path
import subprocess
import sys
import time

from acceptance import allowed
from visual import load, sha, write, require, configure_windows, visible_window, stop_owned

ROOT=Path(__file__).resolve().parents[3]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--prefix',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
    a=p.parse_args();out=allowed(a.output).resolve();prefix=allowed(a.prefix).resolve()
    require(not out.exists(),'Fresh visual receipt required');out.parent.mkdir(parents=True,exist_ok=True)
    from PIL import ImageGrab
    helper=load('m03_effect_primary_display',ROOT/'tools/validation/radar/visual.py')
    configure_windows();ctypes.windll.user32.SetProcessDPIAware()
    dimensions=[ctypes.windll.user32.GetSystemMetrics(i) for i in (0,1)]
    receipt=dict(schema='fsim.sim-m03.explosives-visual.v1',automated_passed=False,models=[],
        primary_resolution=dimensions,actual_capture_review_required=True,
        actual_capture_method='Primary ImageGrab, verified foreground PID/fullscreen, full1x native lifecycle')
    def save():write(out,receipt)
    save();environment=os.environ.copy()
    environment.update(TMP='D:/ENV/tmp',TEMP='D:/ENV/tmp',TORCH_EXTENSIONS_DIR='D:/ENV/tmp')
    environment['PATH']=str(prefix/'bin')+os.pathsep+environment.get('PATH','')
    for case,card in (('cruise','cj-10a'),('bomb','mk82')):
        folder=out.parent/('effects-live-'+case);folder.mkdir()
        ready=folder/'start';native_receipt=folder/'native.json';recording=folder/'live.fsrec';world='m03-fullscene-'+case
        producer=[sys.executable,'-B',str(ROOT/'tools/validation/weapon_quality/explosives.py'),
            '--prefix',str(prefix),'--output',str(native_receipt),'--case',case,'--live','--world',world,
            '--start-file',str(ready),'--record',str(recording)]
        viewer=[str(prefix/'bin/flightsim-viewer.exe'),'--world',world,'--fullscreen',
            '--offline','--camera','chase','--chase-distance','35','--chase-elevation','20','--chase-azimuth','240',
            '--imagery','satellite','--elevation','terrarium','--clouds','scattered','--sun-utc','20','--stats','420']
        row=dict(card=card,model=case,automated_passed=False,images=[],producer_command=producer,viewer_command=viewer)
        receipt['models'].append(row);save();source=display=None
        cleanup=folder/'cleanup.log';slog=folder/'native.log';vlog=folder/'viewer.log'
        try:
            require(not helper.windows(),'Another Viewer owns the single GUI slot')
            with slog.open('wb') as sl,vlog.open('wb') as vl:
                source=subprocess.Popen(producer,cwd=ROOT,env=environment,stdout=sl,stderr=subprocess.STDOUT,
                    creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
                # The scene waits after its first publication. The Viewer can
                # attach and render before the native real-time clock resumes.
                display=subprocess.Popen(viewer,cwd=ROOT,env=environment,stdout=vl,stderr=subprocess.STDOUT,
                    creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
                deadline=time.monotonic()+120
                while 'stats t=' not in vlog.read_text(encoding='utf-8',errors='replace'):
                    require(source.poll() is None and display.poll() is None,'Native or Viewer startup failed')
                    require(time.monotonic()<deadline,'First actual presentation timeout');time.sleep(.1)
                visible_window(helper,display,dimensions);ready.write_text('Actual primary fullscreen verified\n',encoding='utf-8')
                started=time.monotonic();next_capture=0.;index=0
                while True:
                    require(display.poll() is None,'Viewer exited before complete native lifecycle')
                    shown=visible_window(helper,display,dimensions);elapsed=time.monotonic()-started
                    require(elapsed<300,'Bounded real-time lifecycle timeout')
                    ended=source.poll() is not None
                    if elapsed>=next_capture or ended:
                        image=ImageGrab.grab(all_screens=False)
                        require(ctypes.windll.user32.GetForegroundWindow()==shown['handle'],'Foreground changed during actual capture')
                        require(list(image.size)==dimensions,'Actual primary resolution changed')
                        path=folder/('primary-'+str(index).zfill(2)+'.png');image.save(path)
                        row['images'].append(dict(path=str(path),sha256=sha(path),resolution=dimensions,wall_elapsed_s=elapsed,window=shown))
                        index+=1;next_capture=elapsed+(2 if case=='bomb' else 10);save()
                    if ended:break
                    time.sleep(.1)
                require(source.returncode==0,'Public explosive scene failed; see raw log')
                data=json.loads(native_receipt.read_text(encoding='utf-8'));require(data['passed'] and data['live'],'Missing actual native live evidence')
                require(len(row['images'])>=3,'Complete lifecycle images missing')
                row.update(automated_passed=True,native=dict(path=str(native_receipt),sha256=sha(native_receipt)),
                    recording=dict(path=str(recording),sha256=sha(recording)),native_exit=source.returncode,
                    wall_seconds=elapsed,mode='actual-primary-fullscreen-foreground-live1x',
                    native_log=dict(path=str(slog),sha256=sha(slog)))
                ctypes.windll.user32.PostMessageW(shown['handle'],0x0010,0,0);display.wait(timeout=30)
                require(display.returncode==0,'Viewer close failed');save()
        except BaseException as error:row['error']=str(error);save();raise
        finally:
            stop_owned(source,cleanup);stop_owned(display,cleanup)
    receipt['automated_passed']=True;save();print(json.dumps(dict(receipt=str(out),automated_passed=True)))


if __name__=='__main__':main()
