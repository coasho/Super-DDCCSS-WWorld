"""Admit exact authored explosive execution, independently rechecking raw material.

This collector never runs a native test or supplies damage. Freeze its source
epoch before execution, capture each preset afterward, then admit both. Native
method absorption, real component holes and actual CJ10 terminal initiation have
different predicates; a generic green suite cannot substitute for those rows.
"""
import argparse
import csv
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
PRESETS={'ucrt64-headless','ucrt64-release'}
CASES={
    'Every authored explosive executes its selected finite scene and native component damage',
    'CJ10 armed native terrain initiation executes its warhead while inert releases remain inert'}
SOURCE_PATHS={
    'tests/simulation/test_weapon_effects_population.cpp',
    'tools/validation/weapon_quality/effects_population.py',
    'tools/validation/weapon_quality/test_effects_population.py',
    'tests/CMakeLists.txt',
    'src/simulation/mission_systems/damage/Damage.cpp',
    'src/simulation/mission_systems/damage/Damage.h',
    'src/simulation/mission_systems/damage/State.cpp',
    'src/simulation/mission_systems/damage/State.h',
    'src/simulation/mission_systems/damage/FluidTransitV1.h',
    'src/simulation/mission_systems/damage/LiquidSceneV1.h',
    'src/simulation/mission_systems/damage/WarheadAdapter.cpp',
    'src/simulation/mission_systems/components/support/WarheadSnapshotV1.h',
    'src/simulation/mission_systems/components/registration/WarheadProvidersV1.cpp',
    'src/simulation/mission_systems/components/registration/FuzeProvidersV1.cpp',
    'src/simulation/mission_systems/components/registration/EffectFactoriesV1.cpp',
    'src/simulation/mission_systems/components/implementations/warhead/ZonedCaseWarheadV1.cpp',
    'src/simulation/mission_systems/components/implementations/warhead/FiniteZonedCaseWarheadV2.cpp',
    'src/simulation/mission_systems/components/implementations/fuze/SweptPathFuzeV1.cpp',
    'src/simulation/mission_systems/weapon/FuzeAdapter.cpp',
    'src/simulation/runtime/world/World.cpp',
    'src/simulation/flight/JsbsimModel.cpp',
    'src/simulation/runtime/stores/StoreFlightBinding.cpp',
    'src/simulation/runtime/stores/StoreFlightBinding.h',
    'src/simulation/runtime/world/World.h',
    'src/simulation/runtime/world/WorldDamage.cpp',
    'src/simulation/runtime/world/WorldWeaponEffectsDiagnostics.cpp',
    'src/simulation/runtime/world/WorldLiquidMaterials.cpp',
    'src/simulation/runtime/world/WorldLiquidMaterials.h',
    'src/simulation/runtime/world/WorldEngagements.cpp',
    'aircraft/f16c/f16c.damage.json','aircraft/f16c/f16c.liquid-v1.json','aircraft/f16c/f16c.glb','aircraft/f16c/f16c.xml','aircraft/f16c/payloads.json',
    'aircraft/h6k/h6k.xml','aircraft/h6k/payloads.json','aircraft/stores/cj10a/cj10a.xml'}
POPULATION_FIELDS=tuple('card selection mechanism version segments connected emitted case_kg emitted_kg mass_tolerance_kg reference_energy_j absorbed_energy_j energy_tolerance_j momentum_error_ns momentum_tolerance_ns absorbed_kg native_scene_calls native_emitted native_emitted_kg native_fragment_hits native_fuel_hole_m2 native_finite blast_tnt_kg'.split())
SOURCE_FIELDS=tuple('ordinal mass_kg area_m2 cd segment x y z vx vy vz ex ey ez evx evy evz expansion_per_s material_length_m'.split())
TERMINAL_FIELDS=tuple('card frame_skip armed jettisoned arming_s impact_age_s detonations scene_calls emitted emitted_kg case_kg terminal finite_worlds'.split())
RUNTIME_FIELDS=('role','path')


def require(value,message):
    if not value:raise ValueError(message)
def sha(path):return hashlib.sha256(allowed(path).read_bytes()).hexdigest()
def bind(path):return dict(path=str(allowed(path).resolve()),sha256=sha(path))
def bound(row,root=ROOT):
    path=allowed(row['path']);path=path if path.is_absolute() else root/path
    verify_artifact(path,row['sha256']);return path
def read_csv(path,fields):
    with allowed(path).open(newline='',encoding='utf-8') as stream:
        reader=csv.DictReader(stream);require(tuple(reader.fieldnames or ())==fields,'Wrong raw effects CSV schema')
        rows=list(reader)
    require(rows and all(None not in row and all(value is not None for value in row.values()) for row in rows),'Empty or malformed raw effects rows')
    return rows
def number(row,key,*,integer=False):
    value=float(row[key]);require(math.isfinite(value),'Nonfinite effects value: '+key)
    if integer:require(value.is_integer(),'Noninteger effects value: '+key);return int(value)
    return value
def close(actual,expected,tolerance,message):
    require(math.isfinite(tolerance) and tolerance>=0 and abs(actual-expected)<=tolerance,message)
def definition(card):
    d=card['component_models']['components']['warhead'];p=card['parameters']
    def value(key,default=None):return p[key]['value'] if key in p else default
    version=value('warhead_model_version',0)
    total=value('warhead_kg');charge=value('explosive_kg');structure=value('warhead_structure_fraction',.15)
    case=(total-charge)*(1-structure);density=value('warhead_case_density_kg_m3',7850.)
    diameter=value('diameter_m')*value('warhead_diameter_ratio',.9)
    volume=charge/value('warhead_explosive_density_kg_m3',1700.)+case/density+structure*(total-charge)/value('warhead_structure_density_kg_m3',2780.)
    length=volume/(math.pi*(diameter/2)**2)
    if version:
        annulus=case/(density*math.pi*length);outer=diameter/2
        thickness=annulus/(outer+math.sqrt(outer*outer-annulus))
    else:thickness=case/(density*math.pi*diameter*max(length,.001))
    rod=card.get('warhead_kind')=='rod'
    equivalent_mass=density*thickness**2*(length if rod else thickness)
    count=math.floor(case/equivalent_mass+.5)
    speed=value('gurney_kms')*1000/math.sqrt(case/charge+.5)
    if rod:speed=min(speed,1400.)
    fuze=card['component_models']['components']['fuze'];arming=value(fuze['parameter_refs']['arming_s'])
    tnt=value('warhead_tnt_equivalent_kg') if version else charge*(value('gurney_kms')*1000/2440.)**2
    # Native equivalents are derived material, not measured hardware counts.
    mechanism=('native.warhead.finite_zoned_case.v2' if version==2 else
        'native.warhead.connected_ring.v1' if d['bindings'].get('assessment')=='native.fragment_blast.v1'
        and card['id'] in {'r-27r','r-27er','r-27t','r-27et'} else 'native.warhead.zoned_case.v1')
    return dict(card=card['id'],selection=d['implementation_id'],mechanism=mechanism,
        version=version,case_kg=case,rod=rod,count=count,speed=speed,diameter=diameter,length=length,arming_s=arming,
        tnt_kg=tnt,cd=value('warhead_fragment_cd',1.3))
def authored(coverage,root=ROOT):
    rows=[row for row in coverage['stores'] if row['components']['warhead']['applicability']=='required']
    require(len(rows)==26,'Exact26 authored explosives required')
    independent={}
    for path in sorted((root/'assets/stores').glob('*.json')):
        card=json.loads(path.read_text(encoding='utf-8'))
        if card['component_models']['components']['warhead']['availability']=='available':
            require(card['id'] not in independent,'Duplicate independently enumerated card');independent[card['id']]=path
    require(len(independent)==26 and set(independent)=={r['card'] for r in rows},'Coverage differs from independently enumerated authored explosives')
    result={}
    for row in rows:
        path=allowed(root/row['card_path']);require(path.resolve()==independent[row['card']].resolve(),'Card outside independently enumerated source')
        verify_artifact(path,row['card_sha256'])
        card=json.loads(path.read_text(encoding='utf-8'));require(card['id']==row['card'],'Wrong card identity')
        data=definition(card);require(data['selection']==row['components']['warhead']['implementation'],'Coverage/provider mismatch')
        require(row['card'] not in result,'Duplicate explosive card');result[row['card']]=data
    return result


def source_reference(rows):
    """Uniform material line integrals, independently evaluated with fsum."""
    masses=[];energy=[];moments=[[],[],[]];scales=[]
    for index,row in enumerate(rows,1):
        require(number(row,'ordinal',integer=True)==index,'Missing/reordered immutable source ordinal')
        mass=number(row,'mass_kg');require(mass>0,'Nonpositive source material mass')
        require(number(row,'area_m2')>0 and number(row,'cd')>0,'Nonphysical material area/drag')
        segment=number(row,'segment',integer=True);require(segment in (0,1),'Invalid material topology')
        for key in SOURCE_FIELDS[5:]:number(row,key)
        velocity=[number(row,'v'+axis) for axis in 'xyz'];end=[number(row,'ev'+axis) for axis in 'xyz']
        mean=[(a+b)/2 if segment else a for a,b in zip(velocity,end)]
        relative=[a-b for a,b in zip(mean,(17,-8,3))]
        energy.append(.5*mass*math.fsum(v*v for v in relative)+
            (mass*math.fsum((a-b)**2 for a,b in zip(velocity,end))/24 if segment else 0))
        for axis in range(3):moments[axis].append(mass*relative[axis])
        masses.append(mass);scales.append(mass*math.sqrt(math.fsum(v*v for v in relative)))
    return dict(mass=math.fsum(masses),energy=math.fsum(energy),momentum=[math.fsum(x) for x in moments],
        momentum_scale=math.fsum(scales),count=len(rows))


def validate_model(row,expected,method,native):
    require(row['card']==expected['card'] and row['selection']==expected['selection'] and row['mechanism']==expected['mechanism'],'Wrong selected effects provider')
    require(number(row,'version',integer=True)==expected['version'],'Wrong source model version')
    ref=source_reference(method);count=number(row,'emitted',integer=True);require(count==ref['count']==expected['count'] and count>0,'Material population count differs')
    budget=512*sys.float_info.epsilon*count
    mass_tolerance=budget*max(1.,expected['case_kg']);energy_tolerance=budget*max(1.,ref['energy'])
    momentum_tolerance=budget*max(1.,ref['momentum_scale'])
    for key,expected_tolerance in (('mass_tolerance_kg',mass_tolerance),('energy_tolerance_j',energy_tolerance),('momentum_tolerance_ns',momentum_tolerance)):
        close(number(row,key),expected_tolerance,16*sys.float_info.epsilon*max(1.,expected_tolerance),'Changed analytical roundoff budget')
    for key in ('case_kg','emitted_kg','absorbed_kg','native_emitted_kg'):
        close(number(row,key),expected['case_kg'],mass_tolerance,'Case material not conserved: '+key)
    close(ref['mass'],expected['case_kg'],mass_tolerance,'Raw material differs from authored case mass')
    for key in ('reference_energy_j','absorbed_energy_j'):
        close(number(row,key),ref['energy'],energy_tolerance,'Uniform material kinetic energy mismatch')
    require(0<=number(row,'momentum_error_ns')<=momentum_tolerance,'Fragment momentum failed')
    require(number(row,'native_scene_calls',integer=True)==1 and number(row,'native_emitted',integer=True)==count,'Actual selected native scene not executed exactly once')
    require(number(row,'native_fragment_hits',integer=True)>0 and number(row,'native_fuel_hole_m2')>0 and number(row,'native_finite',integer=True)==1,'Missing real native fragment/component damage or finite state')
    close(number(row,'blast_tnt_kg'),expected['tnt_kg'],budget*max(1.,expected['tnt_kg']),'Wrong authored blast equivalent')
    segments=sum(number(p,'segment',integer=True) for p in method)
    connected=sum(number(p,'expansion_per_s')>0 for p in method)
    require(number(row,'segments',integer=True)==segments==(count if expected['rod'] else 0),'Wrong finite rod topology')
    require(number(row,'connected',integer=True)==connected==(count if expected['mechanism']=='native.warhead.connected_ring.v1' else 0),'Wrong connected material topology')
    for particle in method:
        close(number(particle,'cd'),expected['cd'],budget,'Wrong authored material drag')
        for prefix,velocity_prefix in (('', 'v'),('e','ev')) if expected['rod'] else (('', 'v'),):
            at=[number(particle,prefix+axis) for axis in 'xyz'];velocity=[number(particle,velocity_prefix+axis) for axis in 'xyz']
            spin=[-3*at[2]-4*at[1],4*at[0]-2*at[2],2*at[1]+3*at[0]]
            relative=[v-b-w for v,b,w in zip(velocity,(100,20,-30),spin)]
            close(math.sqrt(math.fsum(v*v for v in relative)),expected['speed'],budget*max(1.,expected['speed']),'Source violated authored Gurney/rod-speed law')
            require(abs(at[0])<=expected['length']/2+budget and math.hypot(at[1],at[2])<=expected['diameter']/2+budget,'Material outside authored finite envelope')
        close(number(particle,'mass_kg'),expected['case_kg']/count,mass_tolerance,'Wrong per-element material mass')
    require(len(native)==count,'Native physical source population differs')
    native_mass=[]
    for index,p in enumerate(native,1):
        require(number(p,'ordinal',integer=True)==index,'Wrong actual native material ordinal')
        for key in SOURCE_FIELDS:number(p,key)
        require(number(p,'mass_kg')>0 and number(p,'area_m2')>0 and number(p,'cd')>0,'Invalid actual native material')
        native_mass.append(number(p,'mass_kg'))
    close(math.fsum(native_mass),expected['case_kg'],mass_tolerance,'Native source duplicated/lost material')
    require(sum(number(p,'segment',integer=True) for p in native)==segments and
        sum(number(p,'expansion_per_s')>0 for p in native)==connected,'Actual native source topology differs')
    return dict(card=row['card'],passed=True,emitted=count,case_kg=ref['mass'],absorbed_energy_j=ref['energy'],
        native_fragment_hits=number(row,'native_fragment_hits',integer=True),native_fuel_hole_m2=number(row,'native_fuel_hole_m2'))


def validate_terminal(rows,expected):
    indexed=reconcile_population({(skip,armed) for skip in (1,4) for armed in (0,1)},
        [dict(row,frame_skip=number(row,'frame_skip',integer=True),armed=number(row,'armed',integer=True)) for row in rows],('frame_skip','armed'))
    result=[]
    for (skip,armed),row in indexed.items():
        require(row['card']=='cj-10a' and number(row,'jettisoned',integer=True)==1-armed,'Wrong native release arming provenance')
        require(number(row,'arming_s')==expected['arming_s'] and number(row,'impact_age_s')>=expected['arming_s'],'Terrain fixture did not pass actual authored arming')
        require(number(row,'finite_worlds',integer=True)>0,'Native terminal lifecycle was unobserved')
        require(number(row,'detonations',integer=True)==armed and number(row,'scene_calls',integer=True)==armed,'Missing armed terrain initiation or inert detonation')
        terminal=number(row,'terminal',integer=True);require(terminal in range(5),'Unknown native end')
        require((terminal==3)==bool(armed),'Native terminal outcome differs')
        if armed:
            require(number(row,'emitted',integer=True)==expected['count'],'Armed terminal did not emit its exact material population')
            tolerance=512*sys.float_info.epsilon*number(row,'emitted',integer=True)*max(1.,expected['case_kg'])
            close(number(row,'emitted_kg'),expected['case_kg'],tolerance,'Terminal source material mismatch')
            close(number(row,'case_kg'),expected['case_kg'],tolerance,'Wrong terminal authored case')
        else:require(number(row,'emitted',integer=True)==0 and number(row,'emitted_kg')==0,'Inert control emitted explosive material')
        result.append(dict(frame_skip=skip,armed=bool(armed),passed=True,detonations=armed))
    return result


def actual_test_report(path):
    doc=ET.parse(path).getroot()
    if doc.tag=='Catch2TestRun':
        cases=doc.findall('.//TestCase');require({c.get('name') for c in cases}==CASES and len(cases)==2,'Both exact native effects Catch cases required')
        for case in cases:
            total=case.find('OverallResult');require(total is not None and total.get('success')=='true','Native effects case failed')
        totals=doc.find('OverallResults');require(totals is not None and int(totals.get('successes','0'))>0 and int(totals.get('failures','0'))==0,'Actual nonzero native assertions required')
        return 'catch2-xml'
    cases=[c for c in doc.iter('testcase') if c.get('name')=='weapon_effects_population']
    require(len(cases)==1,'Exact native effects CTest entry required')
    case=cases[0];require(all(case.find(tag) is None for tag in ('failure','error','skipped')),'Native effects CTest failed/skipped')
    output='\n'.join(case.itertext())
    require('All tests passed' in output and re.search(r'\b[1-9][0-9,]* assertions? in 2 test cases',output),'Both native cases and nonzero assertions required')
    return 'ctest-junit'


def validate_runtime(observed,binary,runtime,root=ROOT):
    modules=reconcile_population({'binary','jsbsim'},observed,('role',))
    actual_binary=allowed(modules['binary']['path']).resolve()
    actual_jsbsim=allowed(modules['jsbsim']['path']).resolve()
    require(actual_binary==binary.resolve(),'Observed process is not the bound native test producer')
    require(actual_jsbsim.name=='libJSBSim.dll' and actual_jsbsim.parent==actual_binary.parent,
        'Observed JSBSim module belongs to another native runtime')
    require(len(runtime)==1 and bound(runtime[0],root).resolve()==actual_jsbsim,
        'Runtime binding differs from actual process module path')
    return dict(binary=str(actual_binary),jsbsim=str(actual_jsbsim),method='GetModuleHandleW/GetModuleFileNameW inside native test process',
        sdk='Simulation session is statically linked into the bound test binary; no loaded libfsim.dll is claimed')


def freeze_sources(coverage,coverage_hash,root=ROOT,git_source_root=None):
    models=authored(coverage,root);source_paths=SOURCE_PATHS|{row['card_path'] for row in coverage['stores'] if row['card'] in models}
    ancestry=allowed(git_source_root) if git_source_root else root
    require(subprocess.check_output(['git','branch','--show-current'],cwd=ancestry,text=True,timeout=15).strip()=='main','Main-only source epoch required')
    return dict(schema='fsim.sim-m03.effects-source-epoch.v1',main_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ancestry,text=True,timeout=15).strip(),
        coverage_sha256=coverage_hash,sources=[dict(path=p,sha256=sha(root/p)) for p in sorted(source_paths)],presets=[])


def capture_preset(coverage,preset,report,root=ROOT):
    require(preset in PRESETS,'Unknown effects preset');models=authored(coverage,root);actual_test_report(report)
    folder=root/'build'/preset/'validation/weapon_effects_population'
    binary=root/'build'/preset/'tests/fsim_weapon_effects_population_test.exe'
    observed=read_csv(folder/'runtime.csv',RUNTIME_FIELDS)
    modules=reconcile_population({'binary','jsbsim'},observed,('role',));runtime=[bind(allowed(modules['jsbsim']['path']))]
    validate_runtime(observed,binary,runtime,root)
    return dict(preset=preset,binary=bind(binary),runtime=runtime,runtime_observation=bind(folder/'runtime.csv'),test_report=bind(report),
        summary=bind(folder/'population.csv'),terminal=bind(folder/'cruise-terminal.csv'),
        materials=[dict(card=card,method=bind(folder/('method-'+card+'.csv')),native=bind(folder/('native-'+card+'.csv'))) for card in sorted(models)])


def admit(coverage,coverage_hash,epoch,root=ROOT,git_source_root=None):
    require(epoch['schema']=='fsim.sim-m03.effects-source-epoch.v1' and epoch['coverage_sha256']==coverage_hash,'Wrong effects source epoch')
    ancestry=allowed(git_source_root) if git_source_root else root
    require(subprocess.check_output(['git','branch','--show-current'],cwd=ancestry,text=True,timeout=15).strip()=='main','Main-only source epoch required')
    require(subprocess.check_output(['git','rev-parse','HEAD'],cwd=ancestry,text=True,timeout=15).strip()==epoch['main_head'],'Changed main epoch')
    models=authored(coverage,root);sources=reconcile_population(SOURCE_PATHS|{row['card_path'] for row in coverage['stores'] if row['card'] in models},epoch['sources'],('path',))
    for row in sources.values():bound(row,root)
    presets=reconcile_population(PRESETS,epoch['presets'],('preset',));result=[];terminals=[];artifacts=[]
    for preset,run in presets.items():
        binary=bound(run['binary'],root);require(binary.name=='fsim_weapon_effects_population_test.exe' and binary.resolve().is_relative_to((root/'build'/preset).resolve()),'Wrong native effects producer')
        actual_test_report(bound(run['test_report'],root))
        folder=(root/'build'/preset/'validation/weapon_effects_population').resolve()
        require(bound(run['runtime_observation'],root).resolve()==folder/'runtime.csv','Runtime observation belongs to another preset')
        validate_runtime(read_csv(bound(run['runtime_observation'],root),RUNTIME_FIELDS),binary,run['runtime'],root)
        require(bound(run['summary'],root).resolve()==folder/'population.csv' and
            bound(run['terminal'],root).resolve()==folder/'cruise-terminal.csv','Raw effects receipts belong to another preset')
        rows=reconcile_population(models,read_csv(bound(run['summary'],root),POPULATION_FIELDS),('card',))
        materials=reconcile_population(models,run['materials'],('card',))
        for card in sorted(models):
            source=materials[card]
            require(bound(source['method'],root).resolve()==folder/('method-'+card+'.csv') and
                bound(source['native'],root).resolve()==folder/('native-'+card+'.csv'),'Material belongs to another model/preset')
            checked=validate_model(rows[card],models[card],read_csv(bound(source['method'],root),SOURCE_FIELDS),read_csv(bound(source['native'],root),SOURCE_FIELDS))
            result.append(dict(preset=preset,**checked,materials=source));artifacts.extend([source['method'],source['native']])
        terminal=validate_terminal(read_csv(bound(run['terminal'],root),TERMINAL_FIELDS),models['cj-10a'])
        terminals.extend(dict(preset=preset,**row) for row in terminal)
        artifacts.extend([run['binary'],*run['runtime'],run['runtime_observation'],run['test_report'],run['summary'],run['terminal']])
    for row in epoch['sources']+artifacts:bound(row,root)
    return dict(schema='fsim.sim-m03.effects-population.v1',passed=True,main_head=epoch['main_head'],coverage_sha256=coverage_hash,
        models=result,terminal=terminals,sources=epoch['sources'],artifacts=artifacts,
        scope='Exact26 selected material/absorption/native-component execution in both presets; independent CJ10 armed/inert terrain initiation at frameSkip1/4. No measured equipment lethality or full-envelope accuracy claim.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('coverage','source-epoch','output'):parser.add_argument('--'+name,required=True,type=Path)
    parser.add_argument('--git-source-root',type=Path);args=parser.parse_args()
    output=allowed(args.output);require(not output.exists(),'Fresh effects population receipt required')
    coverage=allowed(args.coverage);epoch=allowed(args.source_epoch)
    result=admit(json.loads(coverage.read_text(encoding='utf-8')),sha(coverage),json.loads(epoch.read_text(encoding='utf-8')),git_source_root=args.git_source_root)
    result['source_epoch']=bind(epoch);output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print('52 exact-card native effects receipts and8 CJ10 armed/inert terminal receipts admitted')


if __name__=='__main__':main()
