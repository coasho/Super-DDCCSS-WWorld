"""One bounded normal-entry SDK publisher and full-scene real-time Viewer run."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import time


def main(args):
    root=Path(__file__).resolve().parents[3]
    out=Path(args.output).resolve()
    if '.codex' in {x.casefold() for x in out.parts} or not out.is_relative_to(root/'build'):
        raise ValueError('Evidence must stay in the owned build tree')
    out.mkdir(parents=True,exist_ok=True)
    ready=out/'ready.json';ready.unlink(missing_ok=True)
    env=os.environ.copy()
    env.update(TMP='D:/ENV/tmp',TEMP='D:/ENV/tmp',TORCH_EXTENSIONS_DIR='D:/ENV/tmp')
    shell=env.get('FSIM_BUILD_POWERSHELL') or shutil.which('pwsh') or shutil.which('powershell')
    if not shell:raise RuntimeError('Existing PowerShell launcher required')
    # -Command is constant; argv is passed through environment JSON rather than
    # interpolated shell text. Both processes use the normal run.ps1 entry.
    command="& ./run.ps1 -Mode $env:SIM_M02_MODE -Preset ucrt64-release -Arguments @(ConvertFrom-Json $env:SIM_M02_ARGUMENTS)"
    base=[shell,'-NoProfile','-Command',command]
    publisher_args=['tools/validation/aerodynamics/normal_flight_entry.py','--model',args.model,
                    '--world',args.world,'--output',str(out)]
    viewer_args=['--world',args.world,'--fullscreen','--camera','chase','--chase-distance','35',
                 '--screenshot',str(out/'native-flight.png'),'--screenshot-after','120',
                 '--screenshot-frames','2','--screenshot-every','30','--trace','0']
    started=time.monotonic();record=dict(publisher_argv=publisher_args,viewer_argv=viewer_args)
    def stop(process,stream):
        if process.poll() is None:
            subprocess.run(['taskkill','/PID',str(process.pid),'/T','/F'],stdout=stream,stderr=subprocess.STDOUT,timeout=15)
            process.wait(timeout=15)
    with (out/'publisher.log').open('wb') as publish_log,(out/'viewer.log').open('wb') as viewer_log:
        publish_env=env|dict(SIM_M02_MODE='python',SIM_M02_ARGUMENTS=json.dumps(publisher_args))
        publisher=subprocess.Popen(base,cwd=root,env=publish_env,stdout=publish_log,stderr=subprocess.STDOUT,
                                   creationflags=subprocess.CREATE_NO_WINDOW)
        viewer=None
        try:
            deadline=time.monotonic()+90
            while not ready.is_file():
                if publisher.poll() is not None:raise RuntimeError('Publisher failed before native readiness')
                if time.monotonic()>deadline:raise TimeoutError('Native publisher readiness deadline')
                time.sleep(.2)
            view_env=env|dict(SIM_M02_MODE='viewer',SIM_M02_ARGUMENTS=json.dumps(viewer_args))
            viewer=subprocess.Popen(base,cwd=root,env=view_env,stdout=viewer_log,stderr=subprocess.STDOUT)
            record['viewer_exit_code']=viewer.wait(timeout=260)
            record['publisher_exit_code']=publisher.wait(timeout=90)
            if record['viewer_exit_code'] or record['publisher_exit_code']:raise RuntimeError('Normal entry failed')
            images=sorted(out.glob('native-flight_*.png'))
            if len(images)!=2:raise RuntimeError('Expected two actual Viewer captures')
            record['images']=[str(x) for x in images]
        finally:
            if viewer is not None:stop(viewer,viewer_log)
            stop(publisher,publish_log)
            record['elapsed_seconds']=time.monotonic()-started
            (out/'normal-entry-launch.json').write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
    return 0


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',default='uh1h');parser.add_argument('--world',default='sim-m02-normal-flight')
    parser.add_argument('--output',required=True)
    raise SystemExit(main(parser.parse_args()))
