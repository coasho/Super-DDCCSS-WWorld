"""Admit actual native fuel functions for every maintained external tank.

No simulation is executed here. N/A seeker/fuze/warhead entries, internal tank
occupancy and aggregate green suites cannot replace this population's rows.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

from acceptance import allowed, reconcile_population, verify_artifact

ROOT=Path(__file__).resolve().parents[3]
PRESETS={'ucrt64-release','ucrt64-headless'}
CASE='All authored external tanks transfer remaining fuel and preserve native release mass CG and ground fire'
SOURCE_PATHS={
    'tests/simulation/test_external_tank_population.cpp','tests/simulation/mode_flights.h','tests/CMakeLists.txt',
    'tools/validation/weapon_quality/external_tanks.py','tools/validation/weapon_quality/test_external_tanks.py',
    'docs/research/sim-m03/external-tank-functions.md',
    'tools/validation/weapon_quality/campaign.py','tools/validation/weapon_quality/report.py',
    'tools/validation/weapon_quality/README.md',
    'tools/validation/weapon_quality/acceptance.py','tools/validation/weapon_quality/contract.py',
    'src/simulation/mission_systems/HostStores.cpp','src/simulation/mission_systems/Host.h',
    'src/simulation/mission_systems/Cards.cpp','src/simulation/mission_systems/Cards.h',
    'src/simulation/runtime/stores/StoreFlightBinding.cpp','src/simulation/runtime/stores/StoreFlightBinding.h',
    'src/simulation/runtime/world/World.cpp','src/simulation/runtime/world/WorldDamage.cpp',
    'src/simulation/flight/JsbsimModel.cpp','src/simulation/mission_systems/damage/Impact.cpp',
    'tools/models/hangar/hangar/storestages.py','tools/models/hangar/hangar/jsbsim.py',
}

def require(value,message):
    if not value: raise ValueError(message)
def sha(path):return hashlib.sha256(allowed(path).read_bytes()).hexdigest()
def bind(path):return dict(path=str(allowed(path).resolve()),sha256=sha(path))
def bound(row,root=ROOT):
    path=allowed(row['path']);path=path if path.is_absolute() else root/path
    verify_artifact(path,row['sha256']);return path
def number(value,label,*,positive=False):
    require(type(value) in (int,float) and math.isfinite(value),'Nonfinite/nonnumeric '+label)
    require(not positive or value>0,'Nonpositive '+label);return float(value)
def vector(value,label):
    require(type(value) is list and len(value)==3,'Three-vector required: '+label)
    return [number(v,label) for v in value]
def norm(value):return math.sqrt(math.fsum(x*x for x in value))
def near(a,b,budget,label):
    a=number(a,label);b=number(b,label);budget=number(budget,label+' budget')
    require(budget>=0 and abs(a-b)<=budget,label+' mismatch')
def arithmetic(a,b,scale,label):near(a,b,512*sys.float_info.epsilon*max(1,scale),label)

def authored(coverage,root=ROOT):
    declared=[row for row in coverage['stores'] if row['class']=='tank'];require(len(declared)==19,'Exact19 external tank inventory required')
    cards={}
    for path in sorted((root/'assets/stores').glob('*.json')):
        card=json.loads(path.read_text(encoding='utf-8'))
        if card.get('class')=='tank':
            require(card['id']==path.stem and card['id'] not in cards,'Duplicate/foreign tank identity');cards[card['id']]=(card,path)
    require(set(cards)=={r['card'] for r in declared},'Coverage differs from independently read tank cards')
    fits=[];dependencies=set()
    for row in coverage['fits']:
        path=root/row['fit_path'];verify_artifact(path,row['fit_sha256']);dependencies.add(row['fit_path'])
        dependencies.add('aircraft/'+row['design']+'/'+row['design']+'.xml')
        doc=json.loads(path.read_text(encoding='utf-8'));require(doc['aircraft']==row['design'],'Foreign authored fit')
        for payload in doc.get('payloads',[]):
            dependencies.add('assets/payloads/'+payload['card']+'.json')
        fits.append((row['design'],path,doc))
    result={}
    for row in declared:
        card,path=cards[row['card']];verify_artifact(path,row['card_sha256'])
        require(card['model']=='jsbsim:'+row['model'],'Tank model identity mismatch')
        mounts=[]
        for design,fit_path,fit in fits:
            for loadout in fit.get('loadouts',[]):
                for station in loadout['stations']:
                    if station['store']==card['id']:mounts.append(dict(carrier=design,station=station['station'],fit=str(fit_path)))
        require(mounts or card['id']=='ptb-1150','Unreviewed unauthored diagnostic mount')
        result[card['id']]=dict(card=card['id'],model=card['model'],model_name=row['model'],
            fuel_kg=number(card['parameters']['fuel_kg']['value'],'capacity',positive=True),
            dry_kg=number(card['parameters']['mass_kg']['value'],'dry mass',positive=True),
            card_path=str(path),mounts=mounts)
    return result,dependencies

def xml_material(path):
    doc=ET.parse(path).getroot();mass=doc.find('mass_balance');require(mass is not None,'Missing native mass balance')
    dry=mass.find('emptywt');require(dry is not None and dry.get('unit')=='LBS','Unresolved native empty mass units')
    dry_kg=float(dry.text)*.45359237
    def location(node):
        require(node is not None and node.get('unit') in ('IN','M'),'Unresolved native mass location')
        factor=.0254 if node.get('unit')=='IN' else 1
        return [float(node.findtext(axis))*factor for axis in ('x','y','z')]
    points=mass.findall('pointmass');require(len(points)==1 and points[0].get('name')=='fuel','Native tank fuel pointmass must be uniquely identified')
    weight=points[0].find('weight');require(weight is not None and weight.get('unit')=='LBS' and float(weight.text)==0,'Released tank model must start empty')
    require(doc.find('propulsion/engine') is None,'External-tank function fixture cannot admit a powered weapon')
    return dict(dry_kg=dry_kg,dry_cg=location(mass.find("location[@name='CG']")),fuel_at=location(points[0].find('location')))

def validate_run(row,expected,empty,root=ROOT):
    require(all(type(row[key]) is int for key in ('station','frame_skip','impact_count','fire_count','detonation_count','transfer_calls')),'Native integer required')
    require(row['empty_control'] is empty and row['frame_skip']==1,'Wrong native control/cadence')
    require(row['initial_altitude_m']==120 and row['initial_airspeed_ms']==180 and row['initial_pitch_deg']==-60 and
            number(row['release_down_ms'],'actual release downspeed')>100,'Native hard-impact premise not established')
    if expected['mounts']:
        require(row['fit_basis']=='authored_loadout' and any(m['carrier']==row['carrier'] and m['station']==row['station'] for m in expected['mounts']),
                'Tank did not use an actual authored configuration')
    else:
        require(expected['card']=='ptb-1150' and row['carrier']=='f16c' and row['station']==4 and
                row['fit_basis']=='generic_diagnostic_class_mass_mount_not_OEM','Diagnostic mount mislabeled OEM/foreign')
    capacity=expected['fuel_kg'];external0=number(row['external_before_transfer_kg'],'external before',positive=True)
    carrier_xml=bound(row['carrier_xml'],root)
    require(carrier_xml.name==row['carrier']+'.xml' and sha(carrier_xml)==sha(root/'aircraft'/row['carrier']/(row['carrier']+'.xml')),
            'Loaded carrier differs from current authored native plant')
    external1=number(row['external_after_transfer_kg'],'external after');inside0=number(row['internal_before_transfer_kg'],'internal before')
    inside1=number(row['internal_after_transfer_kg'],'internal after');native_capacity=number(row['internal_capacity_kg'],'internal capacity',positive=True)
    dt=number(row['transfer_duration_s'],'component interval',positive=True)
    require(0<=external1<external0<=capacity+1e-7 and 0<=inside0<inside1<=native_capacity+1e-7,'Invalid actual fuel state')
    require(native_capacity-inside0>external0,'Empty control lacked native receiving capacity')
    native_dt=number(row['native_dt_s'],'native timestep',positive=True)
    requested=capacity/10+1 if empty else 1
    require(row['transfer_calls']==math.ceil(requested/native_dt) and 0<row['transfer_calls']<=100000,'Wrong bounded SDK transfer calls')
    near(dt,row['transfer_calls']*native_dt,1e-12*max(1,dt),'Predeclared component interval')
    transferred=min(external0,native_capacity-inside0,10*dt)
    near(external0-external1,transferred,1e-7*max(1,capacity),'Native10kg/s transfer law')
    near(inside1-inside0,transferred,1e-7*max(1,capacity),'Transferred mass conservation')
    fuel=number(row['released_fuel_kg'],'released remaining fuel')
    remaining=number(row['external_before_release_kg'],'external before release')
    require(0<=fuel<=remaining+1e-7 and remaining<=external1+1e-7,'Release created fuel')
    require((empty and fuel<=1e-8) or (not empty and fuel>0),'Wrong remaining-fuel control')
    before,after=row['before_release'],row['after_release']
    mb=number(before['mass_kg'],'before mass',positive=True);ma=number(after['mass_kg'],'after mass',positive=True)
    cb=vector(before['cg_structural_m'],'before CG');ca=vector(after['cg_structural_m'],'after CG')
    fb=number(before['internal_fuel_kg'],'before internal fuel');fa=number(after['internal_fuel_kg'],'after internal fuel')
    pb=vector(before['internal_fuel_moment_kg_m'],'before internal moment');pa=vector(after['internal_fuel_moment_kg_m'],'after internal moment')
    station=vector(row['station_structural_m'],'actual station');store=number(row['store_before_release_kg'],'station mass',positive=True)
    near(store,expected['dry_kg']+remaining,1e-6*max(1,store),'Remaining native station mass')
    mass_error=mb-ma-(fb-fa)-store;mass_budget=1e-6*max(1,mb,ma)
    moment_error=norm([mb*cb[k]-ma*ca[k]-(pb[k]-pa[k])-store*station[k] for k in range(3)])
    moment_budget=2e-6*max(1,mb*norm(cb),ma*norm(ca))
    arithmetic(row['release_mass_error_kg'],mass_error,mb+ma,'Derived release mass error')
    arithmetic(row['release_moment_error_kg_m'],moment_error,mb*norm(cb)+ma*norm(ca),'Derived release first moment error')
    arithmetic(row['release_mass_budget_kg'],mass_budget,1,'Fixed release mass budget')
    arithmetic(row['release_moment_budget_kg_m'],moment_budget,1,'Fixed release moment budget')
    near(mass_error,0,mass_budget,'Native release mass conservation');near(moment_error,0,moment_budget,'Native release CG conservation')
    require(row['impact_count']==1 and row['detonation_count']==0 and row['continuing_kg_s']==0,'Wrong inert native terminal path')
    require(number(row['impact_energy_j'],'impact energy',positive=True)>0 and 0<number(row['impact_time_s'],'impact time')<=21,'No bounded actual impact')
    fire=number(row['fire_kg'],'fire fuel');require(row['fire_count']==(0 if empty else 1),'Wrong native fire control')
    require(fire==0 if empty else 0<fire<=fuel*(1+1e-12),'Native fire exceeds actual remaining fuel/control')
    path=bound(row['loaded_xml'],root);expected_xml=root/'aircraft/stores'/expected['model_name']/(expected['model_name']+'.xml')
    require(path.name==expected['model_name']+'.xml','Foreign loaded tank model path')
    require(sha(path)==sha(expected_xml),'Loaded native tank differs from current generated source XML')
    material=xml_material(path);mass=number(row['released_mass_kg'],'released mass',positive=True)
    near(material['dry_kg'],expected['dry_kg'],.005+1e-6*expected['dry_kg'],'Generated native dry mass/card')
    cg=vector(row['released_cg_structural_m'],'released CG');at=vector(row['fuel_point_structural_m'],'fuel point location')
    near(mass,material['dry_kg']+fuel,1e-7*max(1,mass),'Actual native fuel pointmass')
    for k in range(3):
        near(at[k],material['fuel_at'][k],1e-10,'Declared native fuel point location')
        near(mass*cg[k],material['dry_kg']*material['dry_cg'][k]+fuel*at[k],1e-7*max(1,mass*norm(cg)),'Native XML composite CG')
    return dict(released_fuel_kg=fuel,released_mass_kg=mass,cg=cg,fuel_at=at,xml=row['loaded_xml'])

def validate_tank(row,expected,root=ROOT):
    require(row['card']==expected['card'] and row['model']==expected['model'],'Wrong tank identity')
    require(bound(row['card_source'],root).resolve()==Path(expected['card_path']).resolve(),'Foreign tank source')
    near(row['capacity_kg'],expected['fuel_kg'],0,'Current card capacity');near(row['dry_card_mass_kg'],expected['dry_kg'],0,'Current card dry mass')
    fit=bound(row['fit_source'],root);require(fit.resolve()==(root/'aircraft'/row['wet']['carrier']/'payloads.json').resolve(),'Foreign carrier fit')
    require(row['wet']['carrier']==row['empty']['carrier'] and row['wet']['station']==row['empty']['station'],'Control mount changed')
    wet=validate_run(row['wet'],expected,False,root);empty=validate_run(row['empty'],expected,True,root)
    error=wet['released_mass_kg']-empty['released_mass_kg']-wet['released_fuel_kg'];budget=1e-7*max(1,wet['released_mass_kg'])
    moment=norm([wet['released_mass_kg']*wet['cg'][k]-empty['released_mass_kg']*empty['cg'][k]-wet['released_fuel_kg']*wet['fuel_at'][k] for k in range(3)])
    moment_budget=1e-7*max(1,wet['released_mass_kg']*norm(wet['cg']),empty['released_mass_kg']*norm(empty['cg']))
    for key,value,scale in (('wet_empty_mass_error_kg',error,wet['released_mass_kg']),('wet_empty_mass_budget_kg',budget,1),
                            ('wet_empty_moment_error_kg_m',moment,wet['released_mass_kg']*norm(wet['cg'])),('wet_empty_moment_budget_kg_m',moment_budget,1)):
        arithmetic(row[key],value,scale,key)
    near(error,0,budget,'Wet/empty native mass control');near(moment,0,moment_budget,'Wet/empty native CG control')
    return dict(card=expected['card'],model=expected['model'],carrier=row['wet']['carrier'],fit_basis=row['wet']['fit_basis'],passed=True)

def raw(path):
    require(path.stat().st_size<=4*1024*1024,'Oversized external-tank receipt')
    rows=[json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()]
    require(len(rows)==21 and rows[0].get('kind')=='runtime' and rows[0].get('schema')=='fsim.external-tank-functions.v1' and rows[0].get('seed')=='17' and
            rows[-1]=={'kind':'complete','cards':19},'Missing/partial external-tank population completion')
    require(all(row.get('kind')=='tank' for row in rows[1:-1]),'Foreign receipt record')
    return rows[0],rows[1:-1]

def actual_test_report(path):
    doc=ET.parse(path).getroot()
    if doc.tag=='Catch2TestRun':
        cases=doc.findall('.//TestCase');require(len(cases)==1 and cases[0].get('name')==CASE,'Exact external-tank native case required')
        total=cases[0].find('OverallResult');require(total is not None and total.get('success')=='true','External-tank test failed')
        totals=doc.find('OverallResults');require(totals is not None and int(totals.get('successes','0'))>0 and int(totals.get('failures','0'))==0,'Nonzero native assertions required')
    else:
        cases=[c for c in doc.iter('testcase') if c.get('name')=='external_tank_population'];require(len(cases)==1,'Exact external-tank CTest required')
        require(all(cases[0].find(tag) is None for tag in ('failure','error','skipped')),'Native tank CTest failed/skipped')
        require(re.search(r'All tests passed.*?\b[1-9][0-9,]* assertions? in 1 test case', '\n'.join(cases[0].itertext()), re.S),'Nonzero complete native tank execution required')

def source_paths(coverage,root):
    models,dependencies=authored(coverage,root)
    return SOURCE_PATHS|dependencies|{r['card_path'] for r in coverage['stores'] if r['card'] in models}|{
        'aircraft/stores/'+r['model_name']+'/'+r['model_name']+'.xml' for r in models.values()}
def git_epoch(root):
    require(subprocess.check_output(['git','branch','--show-current'],cwd=root,text=True,timeout=15).strip()=='main','Main-only epoch required')
    return subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True,timeout=15).strip()
def freeze_sources(coverage,coverage_hash,root=ROOT,git_source_root=None):
    return dict(schema='fsim.external-tanks.source-epoch.v1',main_head=git_epoch(git_source_root or root),coverage_sha256=coverage_hash,
                sources=[dict(path=p,sha256=sha(root/p)) for p in sorted(source_paths(coverage,root))],presets=[])
def capture_preset(coverage,preset,report,root=ROOT):
    require(preset in PRESETS,'Unknown tank preset');authored(coverage,root);actual_test_report(report)
    folder=root/'build'/preset/'validation/external_tanks';path=folder/'external-tanks.ndjson';header,rows=raw(path)
    binary=root/'build'/preset/'tests/fsim_external_tank_population_test.exe'
    require(bound(header['binary'],root).resolve()==binary.resolve(),'Foreign native tank producer')
    jsbsim=bound(header['jsbsim'],root);require(jsbsim.parent.resolve()==binary.parent.resolve() and jsbsim.name=='libJSBSim.dll','Foreign actual native module')
    return dict(preset=preset,binary=bind(binary),runtime=bind(jsbsim),raw=bind(path),test_report=bind(report))
def admit(coverage,coverage_hash,epoch,root=ROOT,git_source_root=None):
    require(epoch['schema']=='fsim.external-tanks.source-epoch.v1' and epoch['coverage_sha256']==coverage_hash,'Wrong external-tank epoch')
    require(epoch['main_head']==git_epoch(git_source_root or root),'Changed main epoch')
    sources=reconcile_population(source_paths(coverage,root),epoch['sources'],('path',))
    for row in sources.values():bound(row,root)
    models,_=authored(coverage,root);runs=reconcile_population(PRESETS,epoch['presets'],('preset',));accepted=[];artifacts=[]
    for preset,run in runs.items():
        report=bound(run['test_report'],root);actual_test_report(report);fresh=capture_preset(coverage,preset,report,root)
        require(fresh==run,'Stale/mixed external-tank runtime capture')
        header,rows=raw(bound(run['raw'],root));population=reconcile_population(models,rows,('card',))
        for card in sorted(models):accepted.append(dict(preset=preset,**validate_tank(population[card],models[card],root)))
        artifacts.extend(run[key] for key in ('binary','runtime','raw','test_report'))
    return dict(schema='fsim.external-tanks.population.v1',passed=True,main_head=epoch['main_head'],coverage_sha256=coverage_hash,
                models=accepted,sources=epoch['sources'],artifacts=artifacts,
                scope='19 existing external-tank native fuel functions in both presets; no carried/free fragment vulnerability, OEM mount validation, M10 task/MDF or full-program acceptance')
def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('coverage','source-epoch','output'):parser.add_argument('--'+name,required=True,type=Path)
    parser.add_argument('--root',type=Path,default=ROOT);parser.add_argument('--git-source-root',type=Path);args=parser.parse_args()
    require(not args.output.exists(),'Fresh external-tank admission output required')
    result=admit(json.loads(args.coverage.read_text(encoding='utf-8')),sha(args.coverage),json.loads(args.source_epoch.read_text(encoding='utf-8')),
                 root=args.root,git_source_root=args.git_source_root)
    result['source_epoch']=bind(args.source_epoch)
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
if __name__=='__main__':main()
