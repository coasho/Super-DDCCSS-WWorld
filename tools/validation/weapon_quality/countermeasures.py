"""Admit hash-bound all-model native countermeasure receiver evidence.

This joins actual native receipts. It never simulates a receiver or upgrades
coupling into deception, hit probability, or equipment calibration.
"""
import argparse
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET

from acceptance import allowed, verify_artifact
from cadence import read_summary_shards

ROOT=Path(__file__).resolve().parents[3]
PRESETS={'ucrt64-headless','ucrt64-release'}
CASE='Every configured AAM receives an actual native countermeasure in its physical detector'
PROBE_FIELDS=('channel','receiver','time_s','launch_age_s','decoy_age_s','range_m','matched',
    'full_replay_max_error','positive_expected_electrons','max_expected_delta','saturated_pixels',
    'max_measured_delta','retained_cell_max_delta','native_energy_snr','residual_energy_snr',
    'semi_active','illumination_waveform','illumination_clock','proof')
SOURCE_PATHS={
    'tests/simulation/test_weapon_sensor_native.cpp',
    'tools/validation/weapon_quality/countermeasures.py',
    'tools/validation/weapon_quality/test_countermeasures.py',
    'tools/validation/weapon_quality/cadence.py',
    'tools/models/hangar/hangar/data/equipment_models.json',
    'tools/models/hangar/hangar/equipment_models.py',
    'src/contracts/equipment/EquipmentModels.h.in',
    'src/contracts/equipment/EquipmentModels.h',
    'cmake/EquipmentModels.cmake',
    'src/simulation/mission_systems/components/ProviderTableV1.h',
    'src/simulation/mission_systems/components/ProviderRegistryV1.cpp',
    'src/simulation/mission_systems/components/registration/EquipmentModels.cpp',
    'src/simulation/mission_systems/components/registration/EquipmentModels.h',
    'src/simulation/mission_systems/components/support/EquipmentSelectionV1.h',
    'src/simulation/mission_systems/weapon/Seeker.cpp',
    'src/simulation/mission_systems/weapon/Guidance.cpp',
    'src/simulation/runtime/world/WorldEngagements.cpp',
    'src/simulation/runtime/world/World.cpp',
    'src/simulation/mission_systems/countermeasure/Dispenser.cpp',
    'src/simulation/mission_systems/components/implementations/seeker/EngineeringSeekerV3.cpp',
    'src/simulation/mission_systems/components/implementations/seeker/ThermalImageReceiverV1.cpp',
    'src/simulation/mission_systems/components/implementations/seeker/RadioReceiverV1.cpp',
    'src/simulation/mission_systems/radar/Signal.cpp'}
SOURCE_PATHS.update({
    'include/fsim/MissionSystems.h','src/simulation/runtime/world/World.h',
    'src/simulation/mission_systems/countermeasure/Dispenser.h',
    'src/simulation/mission_systems/components/interfaces/SeekerExecutionV2.h',
    'src/simulation/mission_systems/components/interfaces/ThermalSeekerExecutionV1.h',
    'src/simulation/mission_systems/components/interfaces/ThermalDualBandSeekerExecutionV1.h',
    'src/simulation/mission_systems/components/interfaces/RadioSeekerExecutionV1.h',
    'src/simulation/mission_systems/components/interfaces/SeekerSceneClockV1.h',
    'src/simulation/mission_systems/components/interfaces/SeekerLaunchNavigationV1.h',
    'src/simulation/mission_systems/components/implementations/seeker/EngineeringSeekerV3.h',
    'src/simulation/mission_systems/components/implementations/seeker/ThermalImageReceiverV1.h',
    'src/simulation/mission_systems/components/implementations/seeker/ThermalDualBandReceiverV1.h',
    'src/simulation/mission_systems/components/implementations/seeker/RadioReceiverV1.h',
    'src/simulation/mission_systems/components/implementations/seeker/RadioTemporalCoincidenceV1.cpp',
    'src/simulation/mission_systems/components/implementations/seeker/RadioTemporalCoincidenceV1.h',
    'src/simulation/mission_systems/components/support/ReceiverConfigurationV1.cpp',
    'src/simulation/mission_systems/components/support/ReceiverConfigurationV1.h',
    'src/simulation/mission_systems/radar/Signal.h',
    'src/simulation/mission_systems/radar/SignalDiagnosticsV1.h'})


def require(value,message):
    if not value:raise ValueError(message)


def sha(path):return hashlib.sha256(allowed(path).read_bytes()).hexdigest()


def exact(expected,rows,key):
    names=[row[key] for row in rows]
    require(len(names)==len(set(names)) and set(names)==set(expected),'Missing or duplicate exact population: '+key)
    return {row[key]:row for row in rows}


def binding(row,root=ROOT):
    path=allowed(row['path']);path=path if path.is_absolute() else root/path
    verify_artifact(path,row['sha256']);return path


def number(value):
    result=float(value);require(math.isfinite(result),'Nonfinite native evidence');return result


def integer(value):
    result=int(value);require(str(result)==str(value),'Noncanonical integral native evidence');return result


def require_epoch(epoch,coverage_hash):
    require(epoch.get('schema')=='fsim.sim-m03.countermeasure-source-epoch.v1','Wrong countermeasure epoch schema')
    require(epoch.get('coverage_sha256')==coverage_hash,'Coverage/source epoch differs')


def validate_model(summary,probes,model):
    require(len(summary)==16,'Invalid complete native summary shape')
    require(summary[:4]==[model['card'],model['implementation'],'4',model['family']],'Wrong native model/provider/family/cadence')
    before,after=map(integer,summary[4:6]);require(before>0 and after>=0 and before-after==1,'Exactly one native cartridge required')
    release,dispense=map(number,summary[6:8]);require(release>=0 and dispense-release>=1.5-1e-9,'Dispensation preceded frozen launch age')
    require(integer(summary[8])==len(probes)==model['channels'],'Missing/extra receiver channel proofs')
    require(summary[9:11]==['1',str(int(model['channels']==2))],'Incomplete native physical channel proof')
    require(all(integer(v)>1 for v in summary[11:14]),'Native sensing, decision and tracking not exercised')
    require(summary[14] in ('in_flight','impact','removed','detonated','expired'),'Invalid native end flag')
    miss=number(summary[15]) if summary[15] else None
    require(miss is None or miss>=0,'Invalid native miss distance')
    channels=exact(set(range(model['channels'])),[{**p,'channel':integer(p['channel'])} for p in probes],'channel')
    for channel,row in channels.items():
        require(set(row)==set(PROBE_FIELDS),'Unexpected/incomplete raw receiver fields')
        numeric={key:number(value) for key,value in row.items() if key not in ('receiver','proof')}
        require(numeric['matched']==1 and numeric['range_m']>0 and numeric['decoy_age_s']>=.1,'Missing unambiguous actual native decoy geometry')
        require(numeric['launch_age_s']>=1.5 and numeric['time_s']>=dispense+.1-1e-8 and
            abs(numeric['time_s']-numeric['launch_age_s']-release)<=1/30+1e-8,'Wrong native receiver/release epochs')
        require(abs(numeric['time_s']-numeric['decoy_age_s']-dispense)<=1/30+1e-8,'Wrong actual decoy release epoch')
        require(numeric['full_replay_max_error']==0,'Paired kernel did not reproduce actual detector')
        require(all(numeric[k]>=0 for k in ('positive_expected_electrons','max_expected_delta','saturated_pixels',
            'max_measured_delta','retained_cell_max_delta','native_energy_snr','residual_energy_snr')),'Negative detector evidence')
        if model['family']=='flare':
            require(row['receiver']=='thermal' and row['proof']=='preclip_charge','Wrong physical detector kind')
            require(numeric['positive_expected_electrons']>0 and numeric['max_expected_delta']>0,'No actual thermal charge contribution')
            require(numeric['retained_cell_max_delta']==numeric['native_energy_snr']==numeric['residual_energy_snr']==0,'Mixed receiver evidence')
        else:
            require(row['receiver']=='radio' and row['proof']=='retained_first_interval','Wrong physical detector kind')
            require(numeric['retained_cell_max_delta']>0 and numeric['native_energy_snr']>0,'No actual RF kernel/cell contribution')
            ratio=numeric['residual_energy_snr']/numeric['native_energy_snr']
            require(ratio>0 and abs(ratio/1e-200-1)<1e-8,'Counterfactual lost common RNG echo slot')
            require(numeric['positive_expected_electrons']==numeric['max_expected_delta']==0,'Mixed receiver evidence')
            require(numeric['semi_active']==int(model['semi']),'Wrong active/passive physical receiver')
            if model['semi']:require(numeric['illumination_waveform']==numeric['illumination_clock']==1,'Missing actual SARH waveform/clock')
    return dict(card=model['card'],family=model['family'],channels=list(channels.values()),terminal=summary[14],miss_m=miss,
        inventory_before=before,inventory_after=after,release_s=release,dispense_s=dispense,passed=True)


def actual_test_report(path,models=None):
    report=ET.parse(path).getroot()
    if report.tag=='Catch2TestRun':
        cases=[case for case in report.findall('.//TestCase') if case.get('name')==CASE]
        require(len(cases)==1,'Exactly one actual countermeasure Catch case required')
        totals=cases[0].find('OverallResult');require(totals is not None and totals.get('success')=='true','Native Catch case failed')
        assertions=cases[0].find('OverallResults')
        if assertions is None:assertions=report.find('OverallResults')
        require(assertions is not None and int(assertions.get('successes','0'))>0 and int(assertions.get('failures','0'))==0,'Missing actual nonzero Catch assertions')
        return 'catch2-xml'
    prefix='sensor_native.'+CASE
    cases=[case for case in report.iter('testcase') if case.get('name') in (CASE,prefix) or case.get('name','').startswith(prefix+'.')]
    sharded=any(case.get('name','').startswith(prefix+'.') for case in cases)
    if sharded:
        require(models is not None,'Exact authored models required for shard report')
        exact({prefix+'.'+model for model in models},[dict(name=case.get('name')) for case in cases],'name')
    else:require(len(cases)==1,'Exactly one actual countermeasure CTest JUnit case required')
    for case in cases:
        require(all(case.find(tag) is None for tag in ('failure','error','skipped')),'Native CTest case failed/skipped')
        output='\n'.join(case.itertext())
        require('All tests passed' in output and re.search(r'\b[1-9][0-9,]* assertions?\b',output),'Missing actual nonzero Catch assertion summary')
    return 'ctest-junit-shards' if sharded else 'ctest-junit'


def admit(coverage,coverage_hash,epoch,root=ROOT,*,git_source_root=None):
    require_epoch(epoch,coverage_hash)
    ancestry=allowed(git_source_root) if git_source_root is not None else root
    head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ancestry,text=True,timeout=15).strip()
    require(subprocess.check_output(['git','branch','--show-current'],cwd=ancestry,text=True,timeout=15).strip()=='main','Main-only source epoch required')
    require(epoch.get('main_head')==head,'Native epoch belongs to another main HEAD')
    stores=[row for row in coverage['stores'] if row['class']=='aam']
    require(len(stores)==24,'All24 authored AAM models required')
    exact({row['card'] for row in stores},stores,'card')
    sources={row['path']:row for row in epoch['sources']}
    require(len(sources)==len(epoch['sources']) and SOURCE_PATHS<=set(sources),'Missing/duplicate required native source epochs')
    for row in epoch['sources']:
        path=allowed(row['path']);require(not path.is_absolute() and '..' not in path.parts,'Repository-relative source epoch required')
        binding(row,root)
    models={};families=Counter()
    for row in stores:
        path=row['card_path'];require(path in sources and sources[path]['sha256']==row['card_sha256'],'Wrong authored card/source epoch')
        card=json.loads(binding(sources[path],root).read_text(encoding='utf-8'))
        provider=card['component_models']['components']['seeker']['implementation_id']
        require(row['components']['seeker']['implementation']==provider,'Coverage/provider epoch differs')
        receiver=card['component_models']['components']['seeker']['bindings']['receiver_model']
        expected_receiver={'active_radar':{'rf_signal.v1'},'semi_active_radar':{'rf_signal.v1'},
            'infrared':{'thermal_quadrant.v1'},'imaging_infrared':{'thermal_image.v1','thermal_dual_image.v1'}}
        require(card['seeker'] in expected_receiver and receiver in expected_receiver[card['seeker']],'Unknown authored physical receiver')
        require((row['card']=='mica-ir')==(receiver=='thermal_dual_image.v1'),'MICA requires actual authored dual-band physical receiver')
        family='flare' if 'infrared' in card['seeker'] else 'chaff';families[card['seeker']]+=1
        models[row['card']]=dict(card=row['card'],implementation=provider,family=family,channels=2 if receiver=='thermal_dual_image.v1' else 1,semi=card['seeker']=='semi_active_radar')
    require(families=={'active_radar':8,'semi_active_radar':4,'infrared':7,'imaging_infrared':5},'Authored all24 seeker-family population differs')
    presets=exact(PRESETS,epoch['presets'],'preset');result=[];artifacts=[]
    for preset,run in presets.items():
        binary=binding(run['binary'],root);require(binary.name.lower()=='fsim_weapon_sensor_native_test.exe','Wrong native test producer')
        require(binary.resolve().is_relative_to((root/'build'/preset).resolve()),'Native producer belongs to another preset')
        require(run['runtime'],'Actual staged SDK and JSBSim runtime bindings required')
        runtime_names={Path(row['path']).name.lower() for row in run['runtime']}
        require(any('fsim' in name and name.endswith('.dll') for name in runtime_names) and
            any('jsbsim' in name and name.endswith('.dll') for name in runtime_names),'Missing staged SDK/JSBSim runtime')
        for row in run['runtime']:binding(row,root)
        report=binding(run['test_report'],root);report_kind=actual_test_report(report,models)
        summary_path=binding(run['summary'],root)
        require(summary_path.name=='countermeasure-summary-skip4.csv','Wrong separate CM summary artifact')
        with summary_path.open(newline='',encoding='utf-8') as stream:summaries=list(csv.reader(stream))
        if report_kind=='ctest-junit-shards':
            inputs=exact(models,run.get('summary_shards',[]),'card')
            raw=[]
            for model in sorted(models):
                shard=binding(inputs[model],root);data=shard.read_bytes()
                parsed=list(csv.reader(data.decode('utf-8').splitlines()))
                require(len(parsed)==1 and parsed[0][0]==model and data.endswith(b'\n'),'Wrong raw model summary shard')
                raw.append(data);artifacts.append(inputs[model])
            require(b''.join(raw)==summary_path.read_bytes(),'Joined summary differs from native shard bytes')
        require(all(len(row)==16 for row in summaries),'Invalid raw summary shape')
        summary_rows=exact(models,[dict(card=row[0],values=row) for row in summaries],'card')
        traces=exact(models,run['traces'],'card')
        for model,definition in models.items():
            path=binding(traces[model],root);require(path.name==model+'-countermeasure-skip4.csv','Wrong per-model CM probe artifact')
            with path.open(newline='',encoding='utf-8') as stream:
                reader=csv.DictReader(stream);require(tuple(reader.fieldnames or ())==PROBE_FIELDS,'Invalid raw probe schema');probes=list(reader)
            result.append(dict(preset=preset,**validate_model(summary_rows[model]['values'],probes,definition),trace=traces[model],summary=run['summary']))
        artifacts.extend([run['binary'],*run['runtime'],run['test_report'],run['summary'],*run['traces']])
    require(len(result)==48,'Complete two-preset24-model population required')
    # Recheck source and artifacts after parsing: mid-admission mutation fails.
    for row in epoch['sources']+artifacts:binding(row,root)
    return dict(schema='fsim.sim-m03.countermeasure-receivers.v1',passed=True,main_head=head,coverage_sha256=coverage_hash,
        models=result,sources=epoch['sources'],artifacts=artifacts,
        scope='Actual native chaff/flare coupling at frozen frameSkip4; no deception, hit, or hardware-accuracy claim',
        rf_counterfactual='Same native pre-exposure state, echo count/geometric order and first-interval noise/phase draws; matched chaff RCS times 1e-200, retained first-interval float-cell contribution suppressed',
        thermal_counterfactual='Same native pre-exposure state and actual pointing; current matched flare patch removed; expected charge compared before full-well clipping')


def capture_preset(coverage,preset,report,output,root=ROOT):
    """Bind an executed full regression; do not execute or simulate anything."""
    require(preset in PRESETS,'Unknown native preset')
    cards={row['card'] for row in coverage['stores'] if row['class']=='aam'}
    require(len(cards)==24,'All24 authored AAM models required')
    source=root/'build'/preset/'validation/weapon_sensor_native'
    _,raw,summaries=read_summary_shards(source,cards,4,countermeasure=True)
    require(actual_test_report(report,cards)=='ctest-junit-shards','Actual independently discovered native shards required')
    output=allowed(output);require(not output.exists(),'Fresh joined native summary required')
    output.parent.mkdir(parents=True,exist_ok=True);output.write_bytes(raw)
    def bind(path):return dict(path=str(path.resolve()),sha256=sha(path))
    return dict(preset=preset,binary=bind(root/'build'/preset/'tests/fsim_weapon_sensor_native_test.exe'),
        runtime=[bind(root/'build'/preset/'tests'/name) for name in ('libfsim.dll','libJSBSim.dll')],
        test_report=bind(report),summary=bind(output),summary_shards=summaries,
        traces=[dict(card=card,**bind(source/card/(card+'-countermeasure-skip4.csv'))) for card in sorted(cards)])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--coverage',required=True,type=Path);parser.add_argument('--source-epoch',required=True,type=Path)
    parser.add_argument('--git-source-root',type=Path)
    parser.add_argument('--output',required=True,type=Path);args=parser.parse_args()
    output=allowed(args.output);require(not output.exists(),'Fresh countermeasure joined receipt required')
    coverage_path=allowed(args.coverage);epoch_path=allowed(args.source_epoch)
    result=admit(json.loads(coverage_path.read_text(encoding='utf-8')),sha(coverage_path),json.loads(epoch_path.read_text(encoding='utf-8')),
        git_source_root=args.git_source_root)
    result['source_epoch']=dict(path=str(epoch_path),sha256=sha(epoch_path))
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print('48 actual native model/preset countermeasure receiver receipts admitted')


if __name__=='__main__':main()
