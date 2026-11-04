"""Join executed M03 evidence and independent reviews without waiving a gate."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from acceptance import allowed, verify_artifact, reconcile_inventory, reconcile_source_review, admit_models, reconcile_population, reconcile_world_health, reconcile_model_products, canonical_product_key, canonical_product_map
from contract import required_population, APPROVED_ROTOR_POLICY
from fits import derive_population as derive_fit_population, compare_coverage as compare_fit_coverage, validate_receipt as validate_fit_receipt
from maps import verify_campaign
from countermeasures import admit as admit_countermeasures
from effects_population import admit as admit_effects_population
from liquid_population import admit as admit_liquid_population
from external_tanks import admit as admit_external_tanks
from backend_epoch import capture as capture_method_backend
from component_matrix import derive_cells, admit_cells, calibration_ledger, GATES as COMPONENT_GATES
from full_chain import admit as admit_full_chain
from component_public import admit as admit_component_public


def read(path):return json.loads(allowed(path).read_text(encoding='utf-8'))
def sha(path):return hashlib.sha256(allowed(path).read_bytes()).hexdigest()
def require(value,message):
    if not value:raise ValueError(message)
def binding(path):return dict(path=str(path),sha256=sha(path))


def join_component_closure(coverage, evidence_path, source):
    """Recheck current per-cell proofs before any full-program acceptance."""
    manifest=read(evidence_path)
    cells=derive_cells(coverage,source)
    rows=admit_cells(cells,manifest,source)
    ledger=calibration_ledger(cells,manifest['calibration'],source)
    require(len(rows)==216 and all(row['passed'] for row in rows),
        'All216 actual component cells must close')
    require(all(sum(row['gates'][gate]['passed'] for row in rows)==216 for gate in COMPONENT_GATES),
        'Complete theory/method/native/public-installed/visual component population required')
    return dict(schema='fsim.sim-m03.component-closure.v1',passed=True,
        evidence=binding(evidence_path),models=24,cells=216,
        passed_gates={gate:216 for gate in COMPONENT_GATES},records=rows,
        calibration=dict(required_records=24,records=ledger,separate_from_engineering=True,
            admitted=sum(row['status']=='calibrated' for row in ledger)))


def join_native_chain(coverage,root,source,campaign,components):
    """Current campaign output is mandatory for every component native gate."""
    root=allowed(root).resolve();source=allowed(source).resolve()
    expected={'source_epoch':root/'native-full-chain-source-epoch.json',
              'runs':root/'native-full-chain-runs.json','proof':root/'native-full-chain.json'}
    metadata=campaign['native_full_chain']
    for key,path in expected.items():
        require(allowed(metadata[key]['path']).resolve()==path,'Native chain binding belongs to another campaign')
        verify_artifact(path,metadata[key]['sha256'])
    runs=read(expected['runs']);epoch=read(expected['source_epoch'])
    proof=read(expected['proof'])
    require(proof==admit_full_chain(coverage,epoch,runs,source),'Native chain receipt changed after actual captures')
    require(len(proof['models'])==216,'All216 current native chain rows required')
    for preset,run in runs.items():
        junit=root/preset/'junit.xml';regression_path=root/preset/'regression.json'
        require(allowed(run['junit']).resolve()==junit,'Native chain uses foreign full-regression output')
        regression=read(regression_path)
        verify_artifact(junit,regression['junit_sha256'])
        recorded=campaign['full_regression'][preset]
        require(allowed(recorded['path']).resolve()==regression_path,'Native chain regression belongs to another campaign')
        verify_artifact(regression_path,recorded['sha256'])
        commands=[row for row in campaign['commands'] if row['label']==preset+'-full']
        require(len(commands)==1 and commands[0]['exit_code']==0 and
                allowed(commands[0]['environment']['FSIM_AAM_CHAIN_EPOCH']).resolve()==expected['source_epoch'],
                'Full native regression did not receive this pre-execution epoch')
    identities={(row['model'],row['component']):index for index,row in enumerate(proof['models'])}
    require(len(identities)==216 and len(components['records'])==216,'Duplicate/missing native component identities')
    for row in components['records']:
        native=row['gates']['native']
        require(allowed(source/native['artifact']['path']).resolve()==expected['proof'] and
                native['artifact']['sha256']==metadata['proof']['sha256'] and
                native['row']==identities[row['model'],row['component']],
                'Component native gate does not reference the current campaign chain')
    return dict(passed=True,proof=binding(expected['proof']),source_epoch=binding(expected['source_epoch']),
                runs=binding(expected['runs']),models=24,cells=216)


def join_component_public(coverage,root,source,campaign,components):
    """Every public component row must use this campaign's actual72 processes."""
    expected={key:root/'component-public'/name for key,name in (('run','run.json'),('proof','components.json'))}
    for key,path in expected.items():
        item=campaign['component_public'][key]
        require(allowed(item['path']).resolve()==path,'Public component output belongs to another campaign')
        verify_artifact(path,item['sha256'])
    commands=[r for r in campaign['commands'] if r['label']=='installed-component-chain']
    require(len(commands)==1 and commands[0]['exit_code']==0 and '--only-model' not in commands[0]['command'],
        'Complete actual installed component command required')
    proof=read(expected['proof'])
    require(proof==admit_component_public(coverage,expected['run'],root/'installed.json',source),
        'Public component evidence changed after actual installed execution')
    identities={(row['model'],row['component']):i for i,row in enumerate(proof['models'])}
    require(len(identities)==216,'All216 public component rows required')
    for row in components['records']:
        item=row['gates']['public_installed']
        require(allowed(source/item['artifact']['path']).resolve()==expected['proof'] and
            item['artifact']['sha256']==campaign['component_public']['proof']['sha256'] and
            item['row']==identities[row['model'],row['component']],
            'Component public gate does not use the current installed campaign')
    return dict(passed=True,proof=binding(expected['proof']),run=binding(expected['run']),models=24,executions=72,cells=216)


def join_external_tanks(coverage,root,source,git_source_root):
    external_path=root/'external-tanks.json';external=read(external_path)
    external_epoch=root/'external-tanks-source-epoch.json'
    verify_artifact(external_epoch,external['source_epoch']['sha256'])
    require(Path(external['source_epoch']['path']).resolve()==external_epoch.resolve(),'External tank epoch belongs to another campaign')
    joined_external=admit_external_tanks(coverage,sha(root/'coverage.json'),read(external_epoch),source,
        git_source_root=git_source_root)
    require(all(external.get(key)==value for key,value in joined_external.items()),'External tank joined receipt changed')
    return external_path


def external_tank_gate_evidence(card,gate,expected_tanks,external_path):
    return [binding(external_path)] if card in expected_tanks and gate=='applicability' else []


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('campaign','visual-review','source-review','source-manifest','component-evidence','output'):
        p.add_argument('--'+key,required=True,type=Path)
    p.add_argument('--produced-overlay',type=Path)
    a=p.parse_args();out=allowed(a.output).resolve()
    require(not out.exists(),'Fresh final receipt required')
    root=allowed(a.campaign).resolve();campaign=read(root/'campaign.json')
    require(campaign['automated_passed'] is True,'Complete automated campaign required')
    require(campaign['engineering_acceptance'] is False,'Automated evidence cannot self-certify independent review')
    map_binding=campaign['maps'];map_path=root/'maps.json'
    require(Path(map_binding['path']).resolve()==map_path.resolve(),'Map receipt belongs to another campaign')
    verify_artifact(map_path,map_binding['sha256'])
    map_receipt=read(map_path)
    verify_campaign(Path(map_receipt['root']),map_path,campaign['map_inputs'])
    labels=[row['label'] for row in campaign['commands']]
    required={'admission-negative-controls','independent-navigation-reference','independent-strapdown-reference',
        'native-countermeasure-receivers','native-authored-effects','native-external-tanks','native-full-chain','liquid-population','applicability','complete-production',
        'cargo-lock-preflight','native-cadence','installed','installed-component-chain','installed-authored-fits','installed-fleet','installed-explosives','normal-entry',
        'full-native-and-visual','explosives-visual'}
    required.update(preset+'-'+step for preset in ('ucrt64-headless','ucrt64-release')
        for step in ('configure','build','stage-produced','installed-equipment-preflight','full'))
    require(len(labels)==len(set(labels)) and set(labels)==required,'Incomplete or duplicated campaign command population')
    for command in campaign['commands']:
        require(command['exit_code']==0,'Failed campaign command')
        verify_artifact(command['log'],command['sha256'])
    coverage=read(root/'coverage.json');require(coverage['inventory_passed'],'Incomplete current population')
    require(len(coverage['stores'])==53 and len(coverage['fits'])==35 and coverage['aam_models']==24
            and coverage['explosive_models']==26,'Changed declared roster requires reviewed contract')
    expected_aam={r['card'] for r in coverage['stores'] if r['class']=='aam'}
    expected_explosive={r['card'] for r in coverage['stores'] if r['components']['warhead']['applicability']=='required'}
    expected_tanks={r['card'] for r in coverage['stores'] if r['class']=='tank'}
    require(len(expected_tanks)==19,'Changed external tank roster requires reviewed contract')
    source=Path(campaign['source_root'])
    components=join_component_closure(coverage,a.component_evidence,source)
    native_chain=join_native_chain(coverage,root,source,campaign,components)
    public_components=join_component_public(coverage,root,source,campaign,components)
    liquid_path=root/'liquid-population.json';liquids=read(liquid_path)
    require(liquids==admit_liquid_population(coverage,sha(root/'coverage.json'),source),
        'Current complete liquid material population differs from captured production')
    strapdown=read(root/'strapdown-reference.json')
    require(strapdown.get('schema')=='fsim.strapdown-independent-reference.v1' and strapdown.get('passed') is True,
        'Current independent coupled strapdown reference did not pass')
    cm_path=root/'countermeasure-receivers.json';cm=read(cm_path)
    cm_epoch=root/'countermeasure-source-epoch.json'
    verify_artifact(cm_epoch,cm['source_epoch']['sha256'])
    require(Path(cm['source_epoch']['path']).resolve()==cm_epoch.resolve(),'Countermeasure epoch belongs to another campaign')
    joined_cm=admit_countermeasures(coverage,sha(root/'coverage.json'),read(cm_epoch),source,
        git_source_root=Path(campaign['git_source_root']))
    require(all(cm.get(key)==value for key,value in joined_cm.items()),'Countermeasure joined receipt changed')
    effects_path=root/'authored-effects.json';effects=read(effects_path)
    effects_epoch=root/'effects-source-epoch.json'
    verify_artifact(effects_epoch,effects['source_epoch']['sha256'])
    require(Path(effects['source_epoch']['path']).resolve()==effects_epoch.resolve(),'Effects epoch belongs to another campaign')
    joined_effects=admit_effects_population(coverage,sha(root/'coverage.json'),read(effects_epoch),source,
        git_source_root=Path(campaign['git_source_root']))
    require(all(effects.get(key)==value for key,value in joined_effects.items()),'Authored effects joined receipt changed')
    external_path=join_external_tanks(coverage,root,source,Path(campaign['git_source_root']))
    production=root/'production';matrix=read(production/'matrix.json')
    verify_artifact(production/'matrix.json',campaign['production']['matrix_sha256'])
    mass_path=production/'mass-partition-closure.json';mass_closure=read(mass_path)
    verify_artifact(mass_path,campaign['production']['mass_partition_sha256'])
    expected_designs={r['model'] for r in coverage['stores']}|{r['design'] for r in coverage['fits']}
    produced=reconcile_population(expected_designs,matrix['designs'],('name',))
    current_backends=capture_method_backend(campaign['method_environment'],source)
    producer_path=production/'installed-production-manifest.json';producer=read(producer_path)
    producer_products=canonical_product_map(producer['products'])
    reconcile_model_products(matrix['designs'],mass_closure['designs'],producer['products'])
    require(mass_closure['complete'] and mass_closure['declared_count']==88 and mass_closure['design_count']==88
            and set(mass_closure['designs'])==expected_designs,'Incomplete generated whole/partitioned mass closure')
    for name,row in produced.items():
        require(row.get('normal_model',{}).get('regenerated') is True and
            row['normal_model'].get('method_backends')==current_backends,
            'Normal production method/backend epoch differs from current actual bytes')
        audit=mass_closure['designs'][name]
        require(row['status']!='blocked' and audit==row['mass_partition'] and audit['passed']
                and audit['primary_tensor_physical'],'A generated mass tensor failed independent artifact admission')
        directory=source/'aircraft'/canonical_product_key(row['product_directory'])
        verify_artifact(directory/(name+'.xml'),audit['xml_sha256'])
        if row['category']=='fixed-wing':
            require(audit['section_count']>0 and audit['complement_count']>0,'A detachable airframe omitted its partition/complements')
            verify_artifact(directory/(name+'.damage.json'),audit['damage_sha256'])
    for row in coverage['stores']:verify_artifact(source/row['card_path'],row['card_sha256'])
    for fit in coverage['fits']:
        if fit['fit_sha256']:verify_artifact(source/fit['fit_path'],fit['fit_sha256'])
    regressions={}
    for preset in ('ucrt64-headless','ucrt64-release'):
        path=root/preset/'regression.json';r=read(path)
        require(r['passed'] and r['exit_code']==0 and r['excluded']==28
                and r['policy_sha256']==APPROVED_ROTOR_POLICY,'Full original regression scope did not pass')
        inventory=read(root/preset/'inventory.json')
        expected=[t['name'] for t in inventory['tests'] if t['name'] not in r['excluded_names']]
        require({'weapon_quality_external_tanks','weapon_quality_liquid_vessel_policy',
                 'weapon_quality_model_methods'}.issubset(expected),
                'Required current fuel/geometry method gates are absent from the full regression')
        junit=root/preset/'junit.xml';verify_artifact(junit,r['junit_sha256'])
        cases=ET.parse(junit).getroot().findall('.//testcase')
        count=reconcile_inventory(expected,[c.get('name') for c in cases],r['failed'],r['skipped'])
        require(not r['nested_methods']['unexpected_skips'],'Unapproved nested method skip')
        require(not r['source_changes'] and not r['runtime_changes'] and not r['configuration_changes'],
                'Source or runtime changed during full regression')
        regressions[preset]=dict(**binding(path),discovered=r['discovered'],passed=count,
            deferred=r['excluded'],nested_methods=r['nested_methods'])
    installed=read(root/'installed.json');fleet=read(root/'installed-fleet.json');normal=read(root/'normal.json')
    fit_population=derive_fit_population(source);compare_fit_coverage(fit_population,coverage)
    authored_fits=read(root/'installed-authored-fits.json')
    admitted_fits=validate_fit_receipt(authored_fits,fit_population,producer)
    require(Path(installed['producer']['path']).resolve()==producer_path.resolve(),
            'Installed consumer refers to another production epoch')
    verify_artifact(producer_path,installed['producer']['sha256'])
    require(producer['prefix']==campaign['prefix'],'Installed producer prefix differs from campaign')
    explosives=read(root/'explosives.json')
    require(all(x['passed'] for x in (installed,fleet,normal,explosives)),'Installed/public/normal entry gate failed')
    prefix=Path(campaign['prefix']);expected_model_roots=[str(prefix/'share/flightsim/aircraft'),str(prefix/'share/flightsim/aircraft/stores')]
    require(all(row.get('installed_aircraft_search_roots')==expected_model_roots for row in (installed,fleet,explosives)),
            'Installed consumers must explicitly select installed model roots ahead of development-source fallbacks')
    cases=explosives['cases']
    reconcile_population({'cruise','bomb'},cases,('case',))
    require(all(r['passed'] for r in cases),'Both actual explosive cases must pass')
    for case in cases:reconcile_world_health(case,case)
    require(next(r for r in cases if r['case']=='bomb')['terminal_aftermath_seconds']>=8,
            'Native bomb aftermath was not observed')
    cadence=read(root/'cadence/cadence.json')
    require(cadence['passed'] and len(cadence['models'])==144,'Complete native cadence matrix required')
    reconcile_population({(card,p,s) for card in expected_aam for p in regressions for s in (1,2,4)},
                         cadence['models'],('card','preset','frame_skip'))
    reconcile_population({(p,s) for p in regressions for s in (1,2,4)},cadence['summaries'],('preset','frame_skip'))
    for summary in cadence['summaries']:
        verify_artifact(summary['path'],summary['sha256'])
        inputs=reconcile_population(expected_aam,summary['source_shards'],('card',))
        raw=[]
        for card in sorted(inputs):
            item=inputs[card];verify_artifact(item['path'],item['sha256']);raw.append(Path(item['path']).read_bytes())
        require(b''.join(raw)==Path(summary['path']).read_bytes(),'Cadence summary differs from actual native shards')
    for row in cadence['models']:
        require(row['passed'],'A native cadence model failed')
        verify_artifact(row['trace']['path'],row['trace']['sha256'])
        verify_artifact(row['summary']['path'],row['summary']['sha256'])
        if row['frame_skip']!=1:require(row['versus_frame_skip1']['decision_pairs']>0,'Missing cross-rate native comparison')
    aam=reconcile_population(expected_aam,fleet['models'],('card',))
    for row in aam.values():
        require(row['replay_passed'] and row['one_worker']['passed'] and row['four_workers']['passed'],
                'Per-model worker replay failed')
        for run in (row['one_worker'],row['four_workers']):
            require(run['complete_world_finite'] and run['observed_world_states']>0,
                    'A complete native aftermath was not observed')
            verify_artifact(run['raw_observations'],run['raw_sha256'])
            reconcile_world_health(run,read(run['raw_observations']))
    reconcile_population({'pl-15','r-37m'},fleet['long_range'],('card',))
    for row in fleet['long_range']:
        require(row['passed'] and row['complete_world_finite'] and row['observed_world_states']>0
                and row['second_ignition_age_s'] is not None and row['second_pulse_actual_thrust'],
                'Actual commanded pulse never executed')
        verify_artifact(row['raw_observations'],row['raw_sha256'])
        reconcile_world_health(row,read(row['raw_observations']))
    reconcile_population({'pl-15','r-37m'},fleet['launcher_loss'],('card',))
    for row in fleet['launcher_loss']:
        require(row['passed'] and row['complete_world_finite'] and row['observed_world_states']>0
                and row['launcher_removed'] and row['remove_launcher_after_s']==1.
                and row['second_ignition_age_s'] is not None and row['second_pulse_actual_thrust'],
                'Live onboard navigation and reserved propulsion did not survive launcher removal')
        verify_artifact(row['raw_observations'],row['raw_sha256'])
        reconcile_world_health(row,read(row['raw_observations']))
    visual_path=root/'visual/visual.json';visual=read(visual_path)
    effect_visual_path=root/'visual/explosives.json';effect_visual=read(effect_visual_path)
    require(visual['automated_passed'] and effect_visual['automated_passed'],'Complete actual presentation required')
    for field in ('native_manifest','fin_audit'):
        verify_artifact(visual[field]['path'],visual[field]['sha256'])
    native_manifest=read(visual['native_manifest']['path'])
    expected_native={(r['card'],r['model'],scenario) for r in coverage['stores'] if r['class']=='aam'
                     for scenario in ('head-on','tail-turn')}
    reconcile_population(expected_native,native_manifest['models'],('card','model','scenario'))
    for row in native_manifest['models']:
        require(row['run_status']=='ok','A frozen native scene failed')
        for field in ('recording','evidence'):verify_artifact(row[field],row[field+'_sha256'])
    fin_audit=read(visual['fin_audit']['path']);require(fin_audit['automated_passed'],'Fin replay failed')
    for phase in ('native48','live24'):
        record=fin_audit[phase];verify_artifact(record['path'],record['sha256'])
        for field in ('log','junit'):verify_artifact(record[field]['path'],record[field]['sha256'])
        measured=read(record['path'])
        require(measured['source']=='actual-v8-recording-through-production-VehicleVisuals','Actual fin replay source required')
        expected_fins={(model,scene) for _,model,scene in expected_native} if phase=='native48' else {
            (model,'fullscreen-live') for _,model,_ in expected_native}
        reconcile_population(expected_fins,measured['models'],('model','scenario'))
        for row in measured['models']:
            require(row['samples']>0 and row['fin_angles']>0 and row['angle_mismatches']==0 and row['rate_violations']==0 and
                    row['limit_violations']==0 and row['matrix_error_max']<1e-12,'Actual fin replay predicate failed')
    aam_visuals=reconcile_population(expected_aam,visual['models'],('card',))
    other_visuals=reconcile_population(expected_explosive-expected_aam,effect_visual['models'],('card',))
    visuals={**aam_visuals,**other_visuals}
    images={}
    for card,row in visuals.items():
        require(row['automated_passed'] and row['images'],'Model presentation failed: '+card)
        for field in (('recording','evidence') if card in aam else ('recording','native')):
            verify_artifact(row[field]['path'],row[field]['sha256'])
        for entry in row['images']:
            verify_artifact(entry['path'],entry['sha256'])
            require(entry['resolution']==visual['primary_resolution'],'Actual primary resolution differs')
            require(entry['path'] not in images,'Duplicate visual capture')
            images[entry['path']]=entry['sha256']
    review=read(a.visual_review)
    require(review['passed'] and review['reviewer'] and review['method']=='actual image inspection',
            'Actual image review required')
    reviewed={r['path']:r for r in review['images']}
    require(len(reviewed)==len(review['images']) and set(reviewed)==set(images),'Incomplete actual capture review')
    require(all(row['passed'] and row['sha256']==images[path] for path,row in reviewed.items()),
            'A capture failed review or its bytes changed')
    code_review=read(a.source_review)
    require(code_review['passed'] and code_review['reviewers'] and code_review['unresolved_findings']==[],
            'Independent complete source review required')
    require(code_review['source_root']==str(source),'Source review belongs to another delivery')
    manifest=read(a.source_manifest)
    require(manifest['passed'] and manifest['source_export']==str(source),'Exact replayed source inventory required')
    reconcile_source_review(manifest['files'],code_review['files'])
    require(code_review['source_manifest_sha256']==sha(a.source_manifest),'Source review inventory bytes changed')
    produced={}
    if a.produced_overlay:
        overlay=read(a.produced_overlay);producer_path=root/'production/installed-production-manifest.json';producer=read(producer_path)
        require(overlay['schema']=='fsim.sim-m03.produced-overlay.v1' and
            overlay['prepare_receipt_sha256']==sha(a.source_manifest) and overlay['campaign_sha256']==sha(root/'campaign.json') and
            overlay['producer_sha256']==sha(producer_path),'Produced overlay lacks same-run provenance')
        snapshot_path=a.source_manifest.parent/'source-files.json'
        verify_artifact(snapshot_path,manifest['source_manifest_sha256'])
        snapshot={row['path']:row for row in read(snapshot_path)}
        for row in overlay['files']:
            name=row['path'];original=snapshot.get(name)
            require(name.startswith('aircraft/') and name not in produced and original is not None and
                row['prepared_sha256']==original['sha256'] and row['prepared_sha256']!=row['produced_sha256'] and
                producer_products.get(canonical_product_key(name.removeprefix('aircraft/')))==row['produced_sha256'],
                'Overlay changes an unproved or authored source path')
            raw=subprocess.check_output(['git','cat-file','blob',original['oid']],cwd=allowed(manifest['source_root']),timeout=30)
            require(hashlib.sha256(raw).hexdigest()==row['prepared_sha256'],'Original prepared bytes changed')
            verify_artifact(source/name,row['produced_sha256']);produced[name]=row['produced_sha256']
        reviewed_products=code_review.get('produced_files',[])
        reconcile_source_review([dict(path=name,sha256=digest,deleted=False)
            for name,digest in produced.items()],reviewed_products)
    for row in code_review['files']:
        verify_artifact(source/row['path'],produced.get(row['path'],row['sha256']))
    population=required_population(coverage);rows=[]
    for card,expected in population.items():
        gates={}
        for gate in expected['gates']:
            evidence=[binding(root/'coverage.json')]
            if gate=='methods':evidence += [binding(root/'navigation-reference.json'),binding(root/'strapdown-reference.json')]+[binding(root/p/'regression.json') for p in regressions]
            if gate=='native':evidence += [binding(root/p/'regression.json') for p in regressions]
            if card in aam and gate in ('methods','native'):evidence += [binding(cm_path)]
            if card in expected_explosive and gate in ('methods','native'):evidence += [binding(effects_path)]
            evidence += external_tank_gate_evidence(card,gate,expected_tanks,external_path)
            if gate in ('public','installed'):
                evidence += [binding(root/'installed.json'),binding(root/('installed-fleet.json' if card in aam else 'explosives.json'))]
                evidence += [binding(root/'installed-authored-fits.json')]
            if gate=='visual':evidence += [binding(visual_path if card in aam else effect_visual_path),binding(a.visual_review)]
            gates[gate]=dict(passed=True,evidence=evidence)
        rows.append(dict(model=card,status='engineering-complete-uncalibrated',unclosed_gates=[],
            components=expected['components'],gates=gates,equipment_calibrated=False,
            uncertainty='Engineering laws and declared finite mechanisms verified; independent equipment-specific accuracy is uncalibrated.'))
    count=admit_models(population,rows)
    result=dict(schema='fsim.sim-m03.complete-acceptance.v1',passed=True,engineering_acceptance=True,
        source_root=str(source),campaign=binding(root/'campaign.json'),coverage=binding(root/'coverage.json'),
        source_review=binding(a.source_review),visual_review=binding(a.visual_review),
        source_manifest=binding(a.source_manifest),native_cadence=binding(root/'cadence/cadence.json'),
        strapdown_reference=binding(root/'strapdown-reference.json'),countermeasure_receivers=binding(cm_path),
        authored_effects=binding(effects_path),liquid_population=binding(liquid_path),external_tank_functions=binding(external_path),
        component_closure=components,
        native_full_chain=native_chain,
        installed_component_chain=public_components,
        mass_partition=binding(mass_path),maps=map_binding,
        installed_consumer=binding(root/'installed.json'),
        produced_overlay=binding(a.produced_overlay) if a.produced_overlay else None,
        stores=count,airframe_configurations=len(fit_population['airframes']),models=rows,full_regression=regressions,
        authored_fits=dict(**binding(root/'installed-authored-fits.json'),**admitted_fits),
        inherited_scope=dict(policy_sha256=APPROVED_ROTOR_POLICY,deferred_rotor_scenarios=28,
            disposition='User-accepted SIM-M02 scope; excluded scenarios are not counted as passed'),
        equipment_calibration=False,unclosed_engineering_gates=[])
    out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k!='models'}))


if __name__=='__main__':main()
