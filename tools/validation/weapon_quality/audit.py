"""Derive complete M03 store/fit applicability from independent authored inputs.

Inventory success establishes input consistency only. Native execution,
independent methods, installed use and actual presentation remain separate.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys

from acceptance import allowed
from contract import COMPONENTS, required_population

ROOT=Path(__file__).resolve().parents[3]


def sha(path):return hashlib.sha256(allowed(path).read_bytes()).hexdigest()
def read(path):return json.loads(allowed(path).read_text(encoding='utf-8'))


def audit():
    sys.path.insert(0,str(ROOT/'tools/models/hangar'))
    from hangar.engineering import roster
    from hangar.equipment_models import catalogue as equipment_catalogue
    named_models={row['store_id']:row for row in equipment_catalogue()}
    designs=roster();stores={};problems=[]
    design_models={row['name']:row for row in designs if row['category']=='store'}
    for path in sorted((ROOT/'assets/stores').glob('*.json')):
        card=read(path);name=card['id'];model=card['model'].removeprefix('jsbsim:')
        if name in stores:raise ValueError('Duplicate card ID')
        if model not in design_models:problems.append(name+': no maintained store design')
        if name!=path.stem:problems.append(name+': filename/card identity differs')
        declarations=card.get('component_models',{}).get('components',{})
        named=named_models.get(name)
        if named is None or named['native_model']!=card['model'] or named['card_designation']!=card['designation']:
            problems.append(name+': missing/mismatched actual equipment model identity')
        if set(declarations)!=set(COMPONENTS):problems.append(name+': nine exact component declarations required')
        components={}
        for kind in COMPONENTS:
            value=declarations.get(kind,{})
            state=value.get('availability')
            required=state!='not_applicable'
            present=state=='available'
            if state not in ('available','unavailable','not_applicable'):
                problems.append(name+'/'+kind+': absent/invalid applicability')
            if required and not present:problems.append(name+'/'+kind+': required implementation unavailable')
            if not required and not value.get('reason'):problems.append(name+'/'+kind+': missing explicit configuration reason')
            if card['class']=='aam' and not required:problems.append(name+'/'+kind+': AAM component cannot be waived')
            refs=value.get('parameter_refs',{})
            for role,reference in refs.items():
                p=card.get('parameters',{}).get(reference)
                if not isinstance(p,dict):problems.append(name+'/'+kind+'/'+role+': missing parameter');continue
                number=p.get('value');uncertainty=p.get('uncertainty')
                if type(number) not in (int,float) or not math.isfinite(number):
                    problems.append(name+'/'+reference+': nonfinite value')
                if type(uncertainty) not in (int,float) or not math.isfinite(uncertainty) or uncertainty<0:
                    problems.append(name+'/'+reference+': invalid uncertainty')
                if p.get('basis') not in ('published','derived','assumed') or not p.get('unit'):
                    problems.append(name+'/'+reference+': missing basis/unit')
                if p.get('basis') in ('published','derived') and p.get('source') not in card.get('sources',{}):
                    problems.append(name+'/'+reference+': unknown source')
                if p.get('basis')!='published' and not p.get('note'):
                    problems.append(name+'/'+reference+': missing method/assumption note')
            components[kind]=dict(applicability='required' if required else 'not_applicable',
                present=present,implementation=value.get('implementation_id'),
                configuration_version=value.get('configuration_version'),
                parameter_refs=refs,bindings=value.get('bindings',{}),reason=value.get('reason'))
            actual=named['components'][kind] if named else None
            if present and (not actual or actual['availability']!='available' or value.get('implementation_id')!=actual['selector']):
                problems.append(name+'/'+kind+': configured component does not select its named equipment model')
            if actual and actual['availability']=='available':
                components[kind].update(model_name=actual['name'],mechanism=actual['mechanism_id'],
                    hardware_model_name=actual['hardware_model_name'],hardware_identity=actual['hardware_identity'])
        stores[name]=dict(card=name,model=model,designation=card['designation'],**{'class':card['class']},
            card_path=path.relative_to(ROOT).as_posix(),card_sha256=sha(path),
            design_path=design_models.get(model,{}).get('path'),components=components,
            equipment_calibration='uncalibrated',equipment_calibration_required=False,
            calibration_limit='No admitted independent exact-variant equipment-accuracy receipt',
            sources=card.get('sources',{}))
        if named:stores[name].update(model_id=named['model_id'],equipment_name=named['designation'])
    matched=[row['model'] for row in stores.values()]
    if set(stores)!=set(named_models):problems.append('Equipment catalogue/current store population differs')
    if len(matched)!=len(set(matched)) or set(matched)!=set(design_models):
        problems.append('Design/card population is not an exact one-to-one binding')
    fits=[]
    for design in designs:
        if design['category']=='store':continue
        path=ROOT/'aircraft'/design['name']/'payloads.json'
        fit=read(path) if path.is_file() else {}
        loadouts=[]
        for loadout in fit.get('loadouts',[]):
            stations=[]
            for station in loadout.get('stations',[]):
                card=station.get('store')
                if card not in stores:problems.append(design['name']+': loadout names an unknown store '+str(card))
                stations.append(dict(number=station.get('station'),card=card,count=station.get('count',1)))
            loadouts.append(dict(name=loadout['name'],stations=stations,source=loadout.get('source'),note=loadout.get('note')))
        fits.append(dict(design=design['name'],category=design['category'],
            fit_path=path.relative_to(ROOT).as_posix(),fit_sha256=sha(path) if path.is_file() else None,
            declared_loadouts=loadouts,verified_equipment_absence=False,
            scope='Authored selected configuration; no station compatibility is relabeled a real equipment installation'))
    result=dict(schema='fsim.sim-m03.current-coverage.v1',stores=list(stores.values()),fits=fits,
        store_designs=len(design_models),airframes=len(fits),
        classes=dict(sorted(Counter(row['class'] for row in stores.values()).items())),
        aam_models=sum(row['class']=='aam' for row in stores.values()),
        explosive_models=sum(row['components']['warhead']['applicability']=='required' for row in stores.values()),
        inventory_passed=not problems,problems=problems,engineering_acceptance=False,
        closure_rule='Complete engineering components/functions/methods/integration and reproducible delivery; empirical equipment accuracy is separate')
    result['required_population']=required_population(result)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    output=allowed(a.output);value=audit();output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in value.items() if k not in ('stores','fits','required_population')}))
    raise SystemExit(0 if value['inventory_passed'] else 2)
