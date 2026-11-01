"""Portable SDK-derived flight calibration, separate from aerodynamic forces.

Flight/performance stage reports are temporary. Their admitted source records
must survive a clone, otherwise build silently discards performance tables.
No airframe coefficients or force equations are supplied by this module.
"""
import argparse
from functools import lru_cache
import hashlib
import json
import warnings
from pathlib import Path

from . import pipeline
from .files import atomic_write_text


@lru_cache(maxsize=3)
def methods_hash(scope=None):
    """Hash actual method dependencies; legacy records retain the full scope.

    A fixed-wing profile never executes rotorcraft sampling/writers. Changing
    those must invalidate rotor records without relabelling fixed-wing data.
    """
    if scope not in (None,'fixed-wing','rotorcraft'):
        raise ValueError('Unknown flight-calibration method scope')
    root=Path(pipeline.repo_root())
    home=Path(__file__).parent
    paths=[]
    for package in ('aero','geometry','rotorcraft'):
        if package=='rotorcraft' and scope=='fixed-wing':continue
        paths.extend((home/package).glob('*.py'))
    paths.extend(home/name for name in ('airframe.py','jsbsim.py','fcs.py','propulsion.py'))
    # Native rotor equations are authored in this patch, not the vendored
    # source below. A rebuilt DLL must not reuse old rotor flight metadata.
    if scope != 'fixed-wing':
        paths.append(root/'cmake/JsbsimPatches.cmake')
        paths.extend((root/'src/simulation/flight_control').glob('*.cpp'))
        paths.extend((root/'src/simulation/flight_control').glob('*.h'))
        paths.append(root/'include/fsim/RotorControllers.h')
    # The upstream dynamics are part of the calibration's method identity.
    for folder,dirs,names in __import__('os').walk(root/'third_party/jsbsim/src/models'):
        dirs[:]=[n for n in dirs if n.casefold()!='.codex']
        paths.extend(Path(folder)/n for n in names if Path(n).suffix in ('.cpp','.h'))
    h=hashlib.sha256()
    for path in sorted(paths):
        h.update(path.relative_to(root).as_posix().encode())
        h.update(path.read_bytes().replace(b'\r\n',b'\n'))
    return h.hexdigest()


def source_hash(directory):
    directory=Path(directory)
    h=hashlib.sha256()
    for name in (directory.name+'.toml','calibration.toml'):
        path=directory/name
        if path.is_file():h.update(path.read_bytes().replace(b'\r\n',b'\n'))
    return h.hexdigest()


def read(directory):
    directory=Path(directory)
    path=directory/'flight-calibration.json'
    if not path.is_file():return {}
    data=json.loads(path.read_text(encoding='utf-8'))
    if (data.get('schema')!='fsim.flight-calibration.v1' or data.get('model')!=directory.name or
        data.get('source_sha256')!=source_hash(directory) or data.get('methods_sha256')!=methods_hash(data.get('method_scope'))):
        warnings.warn('Flight calibration is stale: '+str(path)+'; runtime metadata remains unknown until its flight/performance stages are rerun',RuntimeWarning)
        return {}
    return data['outputs']


def export(directory):
    directory=Path(directory)
    outputs={}
    for stage in ('fly','performance','ground'):
        path=directory/'out'/(stage+'.json')
        if path.is_file():
            raw=json.loads(path.read_text(encoding='utf-8'))
            if stage=='ground':
                outputs[stage]={k:raw[k] for k in ('turn_radius_m','yaw_accel_rad_s2') if k in raw}
            elif stage=='performance':
                outputs[stage]={k:raw[k] for k in ('tables','checks','time','hangar') if k in raw}
            elif 'results' in raw:
                design=raw.get('results',{}).get('design',{})
                outputs[stage]={'results':{'design':{k:design[k] for k in
                    ('stall','stall_landing','fighter','climb','max_speed_ms') if k in design}}}
            else:
                outputs[stage]={k:raw[k] for k in ('hover','hover_trim','trim_table','performance','plant','checks','time','hangar') if k in raw}
    if not outputs:raise ValueError('No SDK flight data to export: '+str(directory))
    import tomllib
    spec=tomllib.loads((directory/(directory.name+'.toml')).read_text(encoding='utf-8'))
    scope='rotorcraft' if spec['aircraft'].get('kind') in ('helicopter','multirotor') else 'fixed-wing'
    data=dict(schema='fsim.flight-calibration.v1',model=directory.name,
              source_sha256=source_hash(directory),methods_sha256=methods_hash(scope),method_scope=scope,
              scope='Retained SDK-derived runtime metadata, including original check results. '
                    'Not an independently matched equipment accuracy certification.',outputs=outputs)
    text=json.dumps(pipeline._json(data),indent=1,allow_nan=False)+'\n'
    atomic_write_text(directory/'flight-calibration.json',text)
    return dict(model=directory.name,stages=sorted(outputs),bytes=len(text.encode()))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',action='append',required=True)
    args=parser.parse_args()
    paths={p.name:p for p in (Path(pipeline.repo_root())/'aircraft').iterdir()
           if p.name.casefold()!='.codex' and (p/(p.name+'.toml')).is_file()}
    names=sorted(paths) if args.model==['all'] else args.model
    import tomllib
    for name in names:
        directory=paths[name]
        spec=tomllib.loads((directory/(name+'.toml')).read_text(encoding='utf-8'))
        if spec['aircraft'].get('card'):continue
        print(json.dumps(export(directory)))


if __name__=='__main__':main()
