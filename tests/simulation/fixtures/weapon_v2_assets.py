"""Stage explicit retained V2 profiles for unchanged legacy native assertions.

Only flat payload/store JSON files are read. Source/installed assets are immutable.
The executable's module-relative ../share/flightsim tree selects these fixtures.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'tools/models/hangar'))
from hangar import equipment_models


def digest(data):
    return hashlib.sha256(data).hexdigest()


def stage(source, destination):
    source=source.resolve();destination=destination.resolve()
    if destination.name!='flightsim' or destination.parent.name!='share' or destination.parent.parent.name!='weapon-v2-fixture':
        raise ValueError('Destination must be the dedicated weapon-v2-fixture/share/flightsim tree')
    if destination==source or source in destination.parents or destination in source.parents:
        raise ValueError('Fixture destination must be separate from immutable source assets')
    selections={'aim-9m','aim-9x','aim-7m','aim-120c'}
    manifest={'schema':'fsim.weapon-v2-native-fixture.v1','source':str(source),
              'destination':str(destination),'python':sys.executable,'files':[]}
    converted=set()
    for category in ('payloads','stores'):
        inputs=sorted((source/category).glob('*.json'))
        if not inputs:raise ValueError('Missing flat fixture input population: '+category)
        folder=destination/category;folder.mkdir(parents=True,exist_ok=True)
        expected={p.name for p in inputs}
        for old in folder.glob('*.json'):
            if old.name not in expected:
                raise ValueError('Unexpected stale fixture JSON: '+str(old))
        for path in inputs:
            if path.is_symlink() or not path.is_file():raise ValueError('Fixture inputs must be ordinary files')
            raw=path.read_bytes();output=raw;selection='unchanged'
            if category=='stores' and path.stem in selections:
                card=json.loads(raw)
                for kind, part in card['component_models']['components'].items():
                    if part['availability']=='available':
                        part['implementation_id']=equipment_models.normalize(card,kind,part['implementation_id'])
                sensor=card['component_models']['components']['seeker']
                if not sensor['implementation_id'].endswith('.v3'):raise ValueError('Expected explicit current V3 input')
                sensor['implementation_id']=sensor['implementation_id'][:-3]+'.v2'
                for role,ref in list(sensor['parameter_refs'].items()):
                    if ref.startswith('sensor_v3_') or role.startswith(('receiver_', 'scene_')):
                        del sensor['parameter_refs'][role];card['parameters'].pop(ref)
                # A retained V2 detector cannot carry a V3 physical receiver
                # or its navigation-frame/waveform contract. Its immediate
                # observations and legacy ownship policy are tested separately.
                for binding in ('receiver_model','navigation_frame','rf_aperture','rf_waveform'):
                    sensor['bindings'].pop(binding,None)
                for parameter in list(card['parameters']):
                    if parameter=='own_navigation_version' or parameter.startswith('own_nav_'):
                        del card['parameters'][parameter]
                sensor['bindings']['measurement_model']='ideal_geometric'
                sensor['bindings']['servo_model']='unavailable'
                sensor['note']='Explicit retained V2 native compatibility fixture; ideal detector observables and immediate reads. Current V3 acceptance runs separately.'
                sensor['validity']='Legacy V2 compatibility behavior; no hardware calibration or V3 engineering acceptance claim.'
                output=(json.dumps(card,indent=2,ensure_ascii=False)+'\n').encode('utf-8')
                selection=sensor['implementation_id'];converted.add(path.stem)
            target=folder/path.name
            if not target.exists() or target.read_bytes()!=output:target.write_bytes(output)
            manifest['files'].append({'source':str(path),'destination':str(target),
                'input_sha256':digest(raw),'output_sha256':digest(output),'selection':selection})
    if converted!=selections:raise ValueError('The four explicit legacy fixture identities are incomplete')
    destination.mkdir(parents=True,exist_ok=True)
    (destination/'v2-fixture-manifest.json').write_bytes((json.dumps(manifest,indent=2)+'\n').encode('utf-8'))
    print('Staged4 explicit V2 fixtures; immutable source hashes and all copied inputs recorded.',flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-assets',required=True,type=Path)
    parser.add_argument('--destination',required=True,type=Path)
    args=parser.parse_args();stage(args.source_assets,args.destination)
