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


def digest(data):
    return hashlib.sha256(data).hexdigest()


SELECTIONS = ('aim-9m', 'aim-9x', 'aim-7m', 'aim-120c')
LEGACY_CASE = ('aim-9m', 'aim-9x', 'aim-7m')


def allowed(path):
    path=Path(path)
    if any(part.lower()=='.codex' for part in path.parts):
        raise ValueError('Excluded path')
    path=path.resolve()
    if any(part.lower()=='.codex' for part in path.parts):
        raise ValueError('Excluded resolved path')
    return path


def retained_warhead(card):
    """Explicit compatibility mechanism, never the current production V3 field.

    The three legacy cases used only the original four scalar references. The
    source-only V3 fields remain in production cards and are not this fixture's
    physics. No numeric value is copied from an old fixture or receipt.
    """
    if card['id'] not in LEGACY_CASE:return
    part=card['component_models']['components']['warhead']
    if part['implementation_id']=='native.warhead.zoned_case.v1':return
    if part['implementation_id']!='native.warhead.authored_field.v3':
        raise ValueError('Unexpected current warhead for retained legacy case')
    if card['parameters']['warhead_model_version']['value']!=3:
        raise ValueError('Expected exact production V3 input before compatibility projection')
    owned=set(part['parameter_refs'].values())
    for name in list(card['parameters']):
        if name.startswith('warhead_') and name!='warhead_kg':
            if name not in owned:raise ValueError('Unowned extra legacy-warhead parameter: '+name)
            del card['parameters'][name]
    for name in ('warhead_kg','explosive_kg','gurney_kms','diameter_m'):
        if name not in card['parameters']:raise ValueError('Missing original legacy scalar: '+name)
    card['component_models']['components']['warhead']={
        'availability':'available','implementation_id':'native.warhead.zoned_case.v1',
        'configuration_version':1,
        'parameter_refs':{name:name for name in ('warhead_kg','explosive_kg','gurney_kms','diameter_m')},
        'bindings':{'burst_reference':'captured_weapon_cg',
                    'blast':'kingery_bulmash.swisdak_1994.free_air.v1',
                    'penetration':'thor.aluminium_2024_t3.v1','assessment':'native.fragment_blast.v1'},
        'basis':'assumed','note':'Existing simulator implementation; no equipment calibration.',
        'validity':'Reduced-order game simulation; equipment performance and damage effectiveness unvalidated.'}
    # Retain current source discussion but distinguish it from fixture selection.
    card['simulation_model']['calibration']=(
        'Retained native compatibility fixture: seeker V2 and native.warhead.zoned_case.v1 '
        'use the original scalar inputs and captured-CG burst. The following production-card '
        'model discussion does not select this fixture runtime: '+card['simulation_model']['calibration'])


def write_frozen(destination,folder,project_root,generator):
    folder=allowed(folder);folder.mkdir(parents=True,exist_ok=True)
    result={'schema':'fsim.weapon-v2-native-fixture-source.v1',
        'generator':'tests/simulation/fixtures/weapon_v2_assets.py',
        'generator_sha256':digest(generator.read_bytes()),
        'scope':'Exactly four explicit retained V2 detector profiles with original nonseeker mechanisms and numeric inputs. Current production acceptance is separate.',
        'verification_scope':'Current input, generated fixture, generator and normalization dependencies are checked before native fixture staging.',
        'files':[],'source_immutability_verified':True,'dependencies':{}}
    for name in SELECTIONS:
        raw=(destination/'stores'/(name+'.json')).read_bytes();card=json.loads(raw)
        target=folder/(name+'.json');target.write_bytes(raw)
        logical='tests/simulation/fixtures/weapon-v2/'+name+'.json'
        source=project_root/'assets/stores'/(name+'.json')
        result['files'].append({'input':'assets/stores/'+name+'.json',
            'input_sha256':digest(source.read_bytes()),'fixture':logical,
            'fixture_sha256':digest(raw),'selected_seeker':card['component_models']['components']['seeker']['implementation_id']})
    for kind,path in [('catalogue','tools/models/hangar/hangar/data/equipment_models.json'),
                      ('normalizer','tools/models/hangar/hangar/equipment_models.py')]:
        result['dependencies'][kind]={'path':path,'sha256':digest((project_root/path).read_bytes())}
    (folder/'manifest.json').write_bytes((json.dumps(result,indent=2)+'\n').encode('utf-8'))


def stage(source, destination, project_root=ROOT, frozen_directory=None):
    source=allowed(source);destination=allowed(destination);project_root=allowed(project_root)
    if frozen_directory is not None:
        frozen_directory=allowed(frozen_directory)
        if (frozen_directory==source or source in frozen_directory.parents or frozen_directory in source.parents or
            frozen_directory==destination or destination in frozen_directory.parents or frozen_directory in destination.parents):
            raise ValueError('Frozen fixtures must be separate from source and runtime assets')
        if frozen_directory.parts[-4:]!=('tests','simulation','fixtures','weapon-v2'):
            raise ValueError('Frozen fixture output must be the explicit tests/simulation/fixtures/weapon-v2 directory')
    sys.path.insert(0,str(project_root/'tools/models/hangar'))
    from hangar import equipment_models
    if allowed(equipment_models.__file__)!=project_root/'tools/models/hangar/hangar/equipment_models.py':
        raise ValueError('Normalizer loaded from a different source root')
    dependencies=[project_root/'tools/models/hangar/hangar/data/equipment_models.json',
                  project_root/'tools/models/hangar/hangar/equipment_models.py']
    dependency_before={str(path):digest(path.read_bytes()) for path in dependencies}
    if destination.name!='flightsim' or destination.parent.name!='share' or destination.parent.parent.name!='weapon-v2-fixture':
        raise ValueError('Destination must be the dedicated weapon-v2-fixture/share/flightsim tree')
    if destination==source or source in destination.parents or destination in source.parents:
        raise ValueError('Fixture destination must be separate from immutable source assets')
    selections=set(SELECTIONS)
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
                retained_warhead(card)
                sensor=card['component_models']['components']['seeker']
                if not sensor['implementation_id'].endswith('.v3'):raise ValueError('Expected explicit current V3 input')
                sensor['implementation_id']=sensor['implementation_id'][:-3]+'.v2'
                for role,ref in list(sensor['parameter_refs'].items()):
                    if ref.startswith('sensor_v3_') or role.startswith(('receiver_', 'scene_')):
                        del sensor['parameter_refs'][role];card['parameters'].pop(ref)
                # A retained V2 detector cannot carry a V3 physical receiver
                # or its navigation-frame/waveform contract. Its immediate
                # observations and legacy ownship policy are tested separately.
                for binding in ('receiver_model','navigation_frame','rf_aperture','rf_waveform',
                                'reticle_pattern','reticle_design'):
                    sensor['bindings'].pop(binding,None)
                # Current cards explicitly opt into a seeker-independent V0
                # proximity scene. These frozen compatibility cases retain
                # their original main-seeker-dependent fuze selection.
                card['component_models']['components']['fuze']['bindings'].pop('target_detection_v1',None)
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
    for row in manifest['files']:
        if digest(Path(row['source']).read_bytes())!=row['input_sha256']:
            raise ValueError('Source changed during compatibility staging')
    if any(digest(Path(path).read_bytes())!=value for path,value in dependency_before.items()):
        raise ValueError('Normalization dependency changed during compatibility staging')
    if frozen_directory is not None:
        if source!=project_root/'assets':raise ValueError('Frozen manifest requires this project source assets')
        write_frozen(destination,frozen_directory,project_root,Path(__file__))
    destination.mkdir(parents=True,exist_ok=True)
    (destination/'v2-fixture-manifest.json').write_bytes((json.dumps(manifest,indent=2)+'\n').encode('utf-8'))
    print('Staged4 explicit V2 fixtures; immutable source hashes and all copied inputs recorded.',flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-assets',required=True,type=Path)
    parser.add_argument('--destination',required=True,type=Path)
    parser.add_argument('--project-root',type=Path,default=ROOT)
    parser.add_argument('--frozen-directory',type=Path)
    args=parser.parse_args();stage(args.source_assets,args.destination,args.project_root,args.frozen_directory)
