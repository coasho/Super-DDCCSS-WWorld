"""Complete M03 AAM presentation receipt through the current native SDK.

Run through the normal Release Python entry. This driver preserves all 48 frozen
preflight scenes, observes 24 foreground primary-fullscreen live 1x runs, and
replays actual native fins. Pixel review remains a separate explicit requirement.
"""
import argparse
import ctypes
from ctypes import wintypes
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

from acceptance import allowed, verify_artifact

ROOT=Path(__file__).resolve().parents[3]
FIN_CASE='all authored AAM physical fins synchronize to native recording snapshots'
CAPTURE_SECONDS=(3.,14.,24.,38.)


def sha(path):return hashlib.sha256(allowed(path).read_bytes()).hexdigest()
def read(path):return json.loads(allowed(path).read_text(encoding='utf-8'))
def write(path,data):allowed(path).write_bytes((json.dumps(data,indent=2,allow_nan=False)+'\n').encode('utf-8'))
def require(value,reason):
    if not value:raise ValueError(reason)


def load(name,path):
    specification=importlib.util.spec_from_file_location(name,allowed(path))
    module=importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def configure_windows():
    """Declare pointer-sized Win32 handles before using shared UI helpers."""
    user=ctypes.windll.user32
    user.IsWindowVisible.argtypes=[wintypes.HWND]
    user.IsWindowVisible.restype=wintypes.BOOL
    user.GetWindowTextW.argtypes=[wintypes.HWND,wintypes.LPWSTR,ctypes.c_int]
    user.GetWindowRect.argtypes=[wintypes.HWND,ctypes.POINTER(wintypes.RECT)]
    user.GetWindowRect.restype=wintypes.BOOL
    user.GetWindowThreadProcessId.argtypes=[wintypes.HWND,ctypes.POINTER(wintypes.DWORD)]
    user.GetWindowThreadProcessId.restype=wintypes.DWORD
    user.GetForegroundWindow.restype=wintypes.HWND
    user.AttachThreadInput.argtypes=[wintypes.DWORD,wintypes.DWORD,wintypes.BOOL]
    user.AttachThreadInput.restype=wintypes.BOOL
    user.PostMessageW.argtypes=[wintypes.HWND,wintypes.UINT,wintypes.WPARAM,wintypes.LPARAM]
    user.PostMessageW.restype=wintypes.BOOL


def revision(explicit):
    if explicit:
        require(re.fullmatch('[0-9a-fA-F]{40}',explicit),'An explicit export revision must be a full40-digit Git SHA')
        return explicit.lower()
    branch=subprocess.check_output(['git','branch','--show-current'],cwd=ROOT,text=True,timeout=15).strip()
    require(branch=='main','The live checkout must be on main; exports require --source-revision')
    return subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True,timeout=15).strip()


def frozen_population(manifest):
    rows=manifest.get('models',[]);require(len(rows)==48,'Exactly48 frozen native cases are required')
    names={r['model'] for r in rows};require(len(names)==24,'Exactly24 current AAM identities are required')
    require({(r['model'],r['scenario']) for r in rows}=={(m,s) for m in names for s in ('head-on','tail-turn')},
            'Missing or duplicate frozen head-on/tail case')
    for row in rows:
        require(row.get('run_status')=='ok','Frozen native case failed: '+row['model']+'/'+row['scenario'])
        scene=read(row['scene']);target=scene['target'];start=scene['start']
        expected=(5000.,270.,0.) if row['scenario']=='head-on' else (3000.,90.,300.)
        require((target['range_m'],target['heading_deg'],target.get('altitude_offset_m',0))==expected,'Frozen encounter changed')
        require(scene['duration_s']==40 and scene['seed']==17 and start['alt_m']==6000,'Frozen duration/seed/altitude changed')
        require(scene['carrier']['speed_ms']==250 and scene['missiles'][0]['launch_s']==12,'Frozen speed/release changed')
        verify_artifact(row['scene'],row['scene_sha256'])
        verify_artifact(ROOT/'assets/stores'/(row['card']+'.json'),row['card_sha256'])
        verify_artifact(ROOT/'aircraft/stores'/row['model']/(row['model']+'.xml'),row['xml_sha256'])
    return sorted(names)


def replay_fins(build,manifest,environment,label):
    """Use the real render test target discovered in tests/CMakeLists.txt."""
    binary=build/'bin/fsim_render_tests.exe';require(binary.is_file(),'Build the real fsim_render_tests target first')
    folder=manifest.parent;log=folder/(label+'-fin-replay.log');junit=folder/(label+'-fin-replay.junit.xml')
    env=environment.copy();env['FSIM_AAM_CHAIN_MANIFEST']=str(manifest)
    command=[str(binary),FIN_CASE,'--reporter','junit','--out',str(junit)]
    with log.open('wb') as output:
        result=subprocess.run(command,cwd=ROOT,env=env,stdout=output,stderr=subprocess.STDOUT,timeout=300)
    require(result.returncode==0,'Actual fin replay failed; see '+str(log))
    document=ET.parse(junit).getroot();cases=document.findall('.//testcase')
    require(cases,'Fin replay executed zero cases')
    require(not document.findall('.//failure') and not document.findall('.//error') and not document.findall('.//skipped'),
            'Fin replay failed or skipped a case')
    audit=folder/'fin-render-audit.json';data=read(audit)
    require(data.get('source')=='actual-v8-recording-through-production-VehicleVisuals','Synthetic fins cannot satisfy this replay')
    require(data.get('distinct_models')==24,'Fin replay omitted a current variant')
    expected={(r['model'],r['scenario']) for r in read(manifest)['models']}
    require(len(data['models'])==len(expected) and {(r['model'],r['scenario']) for r in data['models']}==expected,
            'Fin replay case population changed')
    for row in data['models']:
        require(row['samples']>0 and row['fin_angles']>0,'Missing actual native fin measurements')
        require(row['angle_mismatches']==0 and row['rate_violations']==0 and row['limit_violations']==0 and row['matrix_error_max']<1e-12,
                'Actual fin angle/matrix/stop/rate mismatch')
    return dict(path=str(audit),sha256=sha(audit),command=command,executed_cases=len(cases),
                binary_sha256=sha(binary),log=dict(path=str(log),sha256=sha(log)),junit=dict(path=str(junit),sha256=sha(junit)))


def visible_window(helper,display,dimensions):
    rows=helper.windows();require(len(rows)==1,'Exactly one Viewer GUI slot is required')
    window=rows[0];require(window['rect']==[0,0,*dimensions],'Actual Viewer lost primary fullscreen bounds')
    pid=wintypes.DWORD();ctypes.windll.user32.GetWindowThreadProcessId(window['handle'],ctypes.byref(pid))
    require(pid.value==display.pid,'A different process occupies the Viewer GUI slot')
    shown=helper.present_viewer()
    require(shown['handle']==window['handle'],'Foreground presentation changed window identity')
    return window


def stop_owned(process,log):
    if process is None or process.poll() is not None:return
    with log.open('ab') as stream:
        subprocess.run(['taskkill','/PID',str(process.pid),'/T','/F'],stdout=stream,stderr=subprocess.STDOUT,timeout=30)
    process.wait(timeout=10)


def live_case(root_output,build,row,dimensions,helper,campaign,environment,save):
    from PIL import Image, ImageGrab
    folder=root_output/'live'/row['model'];folder.mkdir(parents=True)
    scene=read(row['scene']);recording=folder/'live.fsrec'
    viewer=[str(build/'bin/flightsim-viewer.exe'),'--world',scene['name'],'--fullscreen',
        '--width',str(dimensions[0]),'--height',str(dimensions[1]),'--offline','--camera','chase',
        '--chase-distance','20','--chase-azimuth','240','--chase-elevation','20',
        '--imagery','satellite','--elevation','terrarium','--clouds','scattered','--sun-utc','20',
        '--stats','90','--trace','2']
    native=[str(build/'bin/flightsim-showcase.exe'),row['scene'],str(recording),'--live']
    result=dict(card=row['card'],model=row['model'],scenario=row['scenario'],automated_passed=False,images=[],
        mode='fullscreen-primary-foreground-live-real-time',isolation='non-isolated SDK diagnostic',
        scene=dict(path=row['scene'],sha256=row['scene_sha256']),viewer_command=viewer,native_command=native,
        expected_simulation_seconds=40,actual_capture_review_required=True,presentation_observations=[])
    save(result)
    source=display=None;started=None
    viewer_log=folder/'viewer.log';native_log=folder/'native.log';cleanup_log=folder/'cleanup.log'
    flags=getattr(subprocess,'CREATE_NO_WINDOW',0)
    try:
        require(not helper.windows(),'Another Viewer already owns the GUI slot')
        with viewer_log.open('wb') as vl,native_log.open('wb') as nl:
            display=subprocess.Popen(viewer,cwd=ROOT,env=environment,stdout=vl,stderr=subprocess.STDOUT,creationflags=flags)
            deadline=time.monotonic()+60
            while 'stats t=' not in viewer_log.read_text(encoding='utf-8',errors='replace'):
                require(display.poll() is None,'Viewer exited before actual presentation')
                require(time.monotonic()<deadline,'Viewer first-present timeout')
                time.sleep(.1)
            shown=visible_window(helper,display,dimensions)
            result['viewer_window']=shown;source=subprocess.Popen(native,cwd=ROOT,env=environment,stdout=nl,
                stderr=subprocess.STDOUT,creationflags=flags);started=time.monotonic()
            for index,due in enumerate(CAPTURE_SECONDS):
                while time.monotonic()-started<due:
                    require(source.poll() is None and display.poll() is None,'Complete live lifecycle exited before capture')
                    shown=visible_window(helper,display,dimensions)
                    result['presentation_observations'].append(dict(wall_elapsed_s=time.monotonic()-started,
                        rect=shown['rect'],handle=shown['handle'],foreground=True))
                    time.sleep(min(.25,max(0,due-(time.monotonic()-started))))
                shown=visible_window(helper,display,dimensions)
                image=ImageGrab.grab(all_screens=False)
                require(ctypes.windll.user32.GetForegroundWindow()==shown['handle'],'Foreground changed during primary capture')
                require(list(image.size)==dimensions,'Primary screen capture resolution changed')
                path=folder/('primary-'+str(index).zfill(2)+'.png');image.save(path)
                with Image.open(path) as decoded:decoded.load();require(list(decoded.size)==dimensions,'PNG decode/resolution failed')
                result['images'].append(dict(path=str(path),sha256=sha(path),resolution=dimensions,
                    wall_elapsed_s=time.monotonic()-started,window=shown))
                save(result)
            deadline=started+90
            while source.poll() is None:
                require(display.poll() is None,'Viewer closed before the native lifecycle ended')
                require(time.monotonic()<deadline,'Real-time native process timeout')
                visible_window(helper,display,dimensions);time.sleep(.1)
            result['native_exit']=source.returncode;result['native_wall_seconds']=time.monotonic()-started
            require(source.returncode==0,'Live native process failed')
            shown=visible_window(helper,display,dimensions)
            # Close only this owned Viewer normally after the complete40s scene.
            require(ctypes.windll.user32.PostMessageW(shown['handle'],0x0010,0,0),'Owned Viewer close request failed')
            result['viewer_exit']=display.wait(timeout=15)
        require(result['viewer_exit']==0,'Viewer did not close normally')
        actual_path=Path(str(recording)+'.engagement.json');actual=read(actual_path)
        require(actual==read(row['evidence']),'Live native outcome differs from its unchanged preflight')
        duration=actual['duration_s'];factor=duration/result['native_wall_seconds']
        require(duration==40 and result['native_wall_seconds']>=duration-.1 and .95<=factor<=1.02,'Actual live pacing was not1x')
        require(len(result['images'])==4,'Incomplete actual primary capture sequence')
        result.update(automated_passed=True,preflight_outcome_equal=True,real_time_factor=factor,
            recording=dict(path=str(recording),sha256=sha(recording)),evidence=dict(path=str(actual_path),sha256=sha(actual_path)),
            native=campaign.summarize(recording,actual),viewer_log=dict(path=str(viewer_log),sha256=sha(viewer_log)),
            native_log=dict(path=str(native_log),sha256=sha(native_log)))
        return result
    except BaseException as error:
        result['error']=str(error);save(result);raise
    finally:
        stop_owned(source,cleanup_log);stop_owned(display,cleanup_log)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--source-revision')
    parser.add_argument('--build',type=Path,default=ROOT/'build/ucrt64-release')
    args=parser.parse_args(argv)
    require(os.name=='nt','Actual primary foreground presentation requires Windows')
    output=allowed(args.output).resolve();build=allowed(args.build).resolve()
    allowed(output);allowed(build);require(not output.exists(),'A fresh output directory is required; retain earlier failures')
    source_revision=revision(args.source_revision);output.mkdir(parents=True)
    campaign=load('m03_frozen_aam_campaign',ROOT/'tools/validation/viewer_perf/aam_campaign.py')
    helper=load('m03_primary_presentation',ROOT/'tools/validation/radar/visual.py')
    environment=campaign.environment(build)
    environment.update(TMP='D:/ENV/tmp',TEMP='D:/ENV/tmp',TORCH_EXTENSIONS_DIR='D:/ENV/tmp')
    configure_windows()
    ctypes.windll.user32.SetProcessDPIAware()
    dimensions=[ctypes.windll.user32.GetSystemMetrics(index) for index in (0,1)]
    require(all(value>0 for value in dimensions),'Primary resolution is unavailable')
    require(not helper.windows(),'The single Viewer GUI slot must be released first')
    binaries={name:dict(path=str(build/'bin'/name),sha256=sha(build/'bin'/name)) for name in
              ('flightsim-showcase.exe','flightsim-viewer.exe','fsim_render_tests.exe','libfsim.dll')}
    receipt=dict(schema='fsim.sim-m03.visual.v1',source_revision=source_revision,source_root=str(ROOT),
        source_state='Working source and products are separately bound by hashes',primary_resolution=dimensions,binaries=binaries,
        automated_passed=False,models=[],actual_capture_review_required=True,
        scope='Observational presentation, outcome equality and actual fin replay; equipment calibration is separate',
        capture_method='PIL ImageGrab actual primary pixels with owned PID/fullscreen/foreground checks',
        driver_sha256=sha(Path(__file__)),campaign_sha256=sha(ROOT/'tools/validation/viewer_perf/aam_campaign.py'),
        presentation_helper_sha256=sha(ROOT/'tools/validation/radar/visual.py'))
    def save() :write(output/'visual.json',receipt)
    save()
    try:
        native_folder=output/'native';native_folder.mkdir()
        manifest=campaign.native(native_folder,build,source_revision=source_revision)
        names=frozen_population(manifest)
        for row in manifest['models']:
            row['recording_sha256']=sha(row['recording']);row['evidence_sha256']=sha(row['evidence'])
        native_manifest=native_folder/'manifest.json';write(native_manifest,manifest)
        receipt['native_manifest']=dict(path=str(native_manifest),sha256=sha(native_manifest));save()
        native_fins=replay_fins(build,native_manifest,environment,'native48')
        live_manifest=dict(schema='fsim.aam-live-fin-replay.v1',source_revision=source_revision,models=[])
        for model in names:
            row=next(r for r in manifest['models'] if r['model']==model and r['scenario']=='tail-turn')
            for field,path in (('card_sha256',ROOT/'assets/stores'/(row['card']+'.json')),('xml_sha256',ROOT/'aircraft/stores'/model/(model+'.xml'))):
                verify_artifact(path,row[field])
            def update(value):
                found=next((index for index,r in enumerate(receipt['models']) if r['model']==model),None)
                if found is None:receipt['models'].append(value)
                else:receipt['models'][found]=value
                save()
            result=live_case(output,build,row,dimensions,helper,campaign,environment,update)
            update(result)
            live_manifest['models'].append(dict(model=model,card=row['card'],scenario='fullscreen-live',run_status='ok',
                recording=result['recording']['path'],evidence=result['evidence']['path']))
            print('M03 actual fullscreen live',model,'passed',flush=True)
        live_path=output/'live/manifest.json';write(live_path,live_manifest)
        live_fins=replay_fins(build,live_path,environment,'live24')
        combined=output/'fin-audit.json';write(combined,dict(schema='fsim.sim-m03.visual-fins.v1',
            native48=native_fins,live24=live_fins,automated_passed=True))
        receipt['fin_audit']=dict(path=str(combined),sha256=sha(combined))
        frozen_population(manifest)
        for binary in binaries.values():verify_artifact(binary['path'],binary['sha256'])
        for row in manifest['models']:
            verify_artifact(row['recording'],row['recording_sha256']);verify_artifact(row['evidence'],row['evidence_sha256'])
        require(len(receipt['models'])==24 and all(r['automated_passed'] for r in receipt['models']),'Incomplete live model population')
        for row in receipt['models']:
            for image in row['images']:verify_artifact(image['path'],image['sha256'])
            for kind in ('recording','evidence'):verify_artifact(row[kind]['path'],row[kind]['sha256'])
        receipt.update(automated_passed=True,native_cases=48,live_cases=24,actual_primary_captures=96)
        save();print(json.dumps(dict(receipt=str(output/'visual.json'),automated_passed=True,actual_capture_review_required=True)))
        return 0
    except BaseException as error:
        receipt['error']=str(error);receipt['automated_passed']=False;save();raise


if __name__=='__main__':raise SystemExit(main())
