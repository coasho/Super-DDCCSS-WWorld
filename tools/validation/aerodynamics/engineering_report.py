"""Render the SIM-M02 delivery from actual current evidence, including gaps."""
import argparse
import json
import os
import hashlib
import time
import zipfile
import xml.etree.ElementTree as ET
from pathlib import Path


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8')) if Path(path).is_file() else None


def archive_evidence(out):
    started=time.monotonic();files=[]
    for here,dirs,names in os.walk(out,followlinks=False):
        dirs[:]=sorted(d for d in dirs if not d.startswith('candidate-') and d.casefold() not in ('.codex','.git','.local','__pycache__','source-export','source-final'))
        for name in sorted(names):
            path=Path(here)/name
            if path.is_symlink():continue
            if name in ('evidence-archive.json','archive-contents.json') or path.suffix not in ('.json','.log','.csv','.md','.html','.txt','.ps1','.py','.patch','.xml','.png'):continue
            # Historical attempts retain their manifests/logs, but delivery has
            # exactly one source patch at the archive root.
            if path.suffix=='.patch' and path!=out/'sim-m02.patch':continue
            files.append(path)
    contents=[dict(path=p.relative_to(out).as_posix(),bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in files]
    manifest=out/'archive-contents.json';manifest.write_text(json.dumps(dict(files=contents),indent=2)+'\n',encoding='utf-8')
    archive=out/'evidence.zip'
    with zipfile.ZipFile(archive,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as target:
        for path in files+[manifest]:target.write(path,path.relative_to(out).as_posix())
    metadata=dict(path=str(archive),bytes=archive.stat().st_size,sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),payload_files=len(files)+1,elapsed_seconds=time.monotonic()-started)
    (out/'evidence-archive.json').write_text(json.dumps(metadata,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(metadata,indent=2))


def update_docs(out,audit,matrix,verification,epoch='replay-final'):
    root=Path(__file__).resolve().parents[3]
    counts=audit['counts']; statuses=matrix['counts']
    suites='; '.join('%s: %d/%d records, %d failed, %d skipped'%(s['preset'],s['executed_records'],s['discovered'],s['failed'],s['skipped']) for s in verification.get('suites',[]))
    text=('Current evidence: %d engineering-complete-uncalibrated, %d engineering-complete, %d blocked designs. '
          'These labels describe the common engineering matrix, separately from whole-program regression acceptance. '
          'Missing measurements do not block engineering completion. '
          'Aggregate forced generation/native campaign %.3f seconds. '
          'Full static audit covers %d serialized table entries and %d XML coefficient values; '
          'native dynamic validation is sampling (%d requested conditions), not full-envelope validation. '
          '%d consistency predicates are reported separately from physical tests. '
          'Fresh unfiltered regressions: %s. Evidence directory: `%s`.')%(
        statuses['engineering-complete-uncalibrated'],statuses['engineering-complete'],statuses['blocked'],
        audit['campaign_timing']['elapsed_seconds'],counts['table_inventory_entries'],counts['native_xml_table_cells'],
        counts['native_dynamic_sampling_conditions'],counts['consistency_predicates'],suites,out.relative_to(root).as_posix())
    controlled=read(out/'optimization-baseline.json') or {}
    resolution_change=read(out/'numerical-resolution-change.json')
    if controlled and not resolution_change:
        before=controlled['campaign_timing']['elapsed_seconds'];after=audit['campaign_timing']['elapsed_seconds']
        text+=' Same-workload before/after campaign: %.3f / %.3f seconds (%.2f%% reduction).'%(before,after,100*(1-after/before))
    elif resolution_change:
        text+=' The earlier identical-discretization optimization measured 2048.546 -> 1734.571 seconds. Current propagation uses the separately documented source-derived version2 numerical method, so its final clock is disclosed without treating it as an identical-workload speedup.'
    pipeline=read(out/'clean-pipeline.json') or {}
    if pipeline:text+=' Entire clean replay including configure/build and both full regressions: %.3f seconds.'%pipeline['elapsed_seconds']
    text+=' The older 2151.704-second campaign has a different native-fitting/build/regression scope and is not the controlled comparison. Reporting groups never replace aggregate clocks.'
    begin='<!-- SIM-M02 current generated verification begin -->';end='<!-- SIM-M02 current generated verification end -->'
    for name in ('docs/status.md','docs/hangar-system.md','docs/sdk/missile-flight-validation.md','docs/sdk/weapon-fidelity-validation.md','docs/architecture/modules/simulation/plan.md'):
        path=root/name; value=path.read_text(encoding='utf-8'); block=begin+'\n'+text+'\n'+end
        if begin in value:
            a=value.index(begin);b=value.index(end,a)+len(end);value=value[:a]+block+value[b:]
        else:value+='\n\n'+block+'\n'
        if '\ufffd' in value:raise ValueError('Replacement character in '+name)
        path.write_text(value,encoding='utf-8')
    (root/'docs/data/sim-m02-engineering-completeness.csv').write_bytes((out/epoch/'matrix.csv').read_bytes())


def main(args):
    out=Path(args.output).resolve(); audit=read(out/'freshness-and-counts.json')
    if not audit:raise ValueError('Current full-table audit required')
    epoch_record=read(out/'canonical-epoch.json') or dict(directory='replay-final')
    epoch=epoch_record['directory']
    if Path(epoch).is_absolute() or len(Path(epoch).parts)!=1:raise ValueError('Nonlocal evidence epoch')
    matrix=read(out/epoch/'matrix.json'); timing=read(out/epoch/'campaign-timing.json')
    clean=read(out/'clean-pipeline.json') or {}; verification=read(out/'regression-replay/verification.json') or {}
    patch=read(out/'patch-manifest.json'); review=read(out/'independent-review.json')
    original_burn=read(out/'regression-replay/original-motor-flight.json')
    resolution_change=read(out/'numerical-resolution-change.json')
    comparable=read(out/'optimization-baseline.json')
    equality=read(out/'optimization-equality.json')
    evaluator=read(out/'reference-evaluator-equivalence.json')
    attempts=read(out/'optimization-attempts.json') or dict(attempts=[])
    phases=[dict(design=r['name'],stage=stage,worker_wall_seconds=seconds) for r in matrix['designs'] for stage,seconds in r.get('generation_phase_worker_seconds',{}).items()]
    phases.sort(key=lambda r:r['worker_wall_seconds'],reverse=True)
    (out/'slowest-generation-stages.json').write_text(json.dumps(dict(scope='Actual per-design worker wall times, including contention; not additive campaign wall time',stages=phases),indent=2)+'\n',encoding='utf-8')
    stages=clean.get('stages',[])
    calls=[s for s in stages if s['name'] in ('clean-campaign','clean-pre-regression-audit','clean-post-regression-audit')]
    reload_wall=sum(s['elapsed_seconds'] for s in calls if s['name']!='clean-campaign')
    whole=sum(s['elapsed_seconds'] for s in calls)
    counts=audit['counts']; statuses=matrix['counts']; baseline=2151.704024
    suites=verification.get('suites',[])
    regression_ok=len(suites)==2 and all(s['failed']==s['skipped']==s.get('uncovered_nested_skips',s['hangar_underlying_unittest_skipped'])==0 and s['discovered']==s['executed_records'] and s['test_names_match'] and s['hangar_method_coverage_complete'] for s in suites) and all(s['exit_code']==0 for s in verification.get('stages',[]))
    evaluator_ok=bool(evaluator and evaluator.get('passed') and evaluator.get('design_count')==matrix['completed_count'] and evaluator.get('recorded_states')==counts['native_dynamic_sampling_conditions']+counts['short_flight_samples'])
    complete=matrix['completed_count']==88 and not statuses['blocked'] and not audit['failed_checks'] and regression_ok and bool(review and review.get('accepted')) and bool(patch and not patch['byte_verification_failures']) and not clean.get('failed_gates',[True]) and (not comparable or bool(equality and equality.get('passed'))) and evaluator_ok
    export_command=read(out/'patch-export-command.json') or {}
    (out/'whole-tool-timing.json').write_text(json.dumps(dict(final_campaign_seconds=timing['elapsed_seconds'],all_reload_command_seconds=reload_wall,generation_validation_reload_command_seconds=whole,entire_clean_pipeline_seconds=clean.get('elapsed_seconds'),candidate_patch_export_seconds=export_command.get('elapsed_seconds'),baseline_campaign_seconds=baseline,same_workload_baseline=comparable,reported_attempts=attempts,commands=calls),indent=2)+'\n',encoding='utf-8')
    lines=['# SIM-M02 engineering delivery','',
        '**Whole SIM-M02 acceptance: %s.**'%('complete' if complete else 'not established; see actual failed or missing gates below'),
        'Current common engineering matrix: **%d engineering-complete-uncalibrated, %d engineering-complete, %d blocked**.'%(statuses['engineering-complete-uncalibrated'],statuses['engineering-complete'],statuses['blocked']),
        'Both completion labels satisfy the engineering matrix. Measurement absence is optional calibration debt, never a component waiver. Required unavailable roles remain required / present=false / failed / blocked. Applicability follows the authored configuration, never implementation capacity.',
        'All 31 fixed-wing, four rotorcraft and 53 stores use the same checks, including all 24 AAMs. No model-specific acceptance branch, new measured value or CFD campaign is claimed.','',
        '## Aggregate timing','', '| Scope | Seconds |','|---|---:|',
        '| Rejected baseline: entire 12-group generation/native campaign | %.3f |'%baseline,
        '| Current forced 88-design generation + installation + native/installed validation + publication | %.3f |'%timing['elapsed_seconds'],
        '| Pre/post full static reload CLI commands | %.3f |'%reload_wall,
        '| Current campaign and reload commands including startup | %.3f |'%whole,
        '| Entire clean replay including configure/build and complete regressions | %s |'%clean.get('elapsed_seconds','missing'),'',
        'The historical 2151.704-second campaign is a different workload: it did not include the current native-fitted damage regeneration, clean builds or full regressions. Its clock is disclosed without treating the difference as an optimization speedup.',
        'Generation %.3f s; one install %.3f s; serial native validation %.3f s; publication %.3f s.'%(timing['generation_phase_seconds'],timing['single_install_phase_seconds'],timing['serial_native_validation_phase_seconds'],timing['publication_and_overhead_seconds']),
        'Campaign exceeds the 1200-second performance target: **%s**. Reporting partitions never replace aggregate wall time. Remaining dominant costs are actual full-grid generation/damage fitting and serial native/installed execution.'%(timing['elapsed_seconds']>1200),
        'Optimizations: '+ '; '.join(timing['optimization'])+'. Exact wake identity replaces traversal-dependent approximate cache reuse; original numerical acceptance limits remain unchanged.',
        '[Aggregate clock](whole-tool-timing.json), [phase timing](%s/campaign-timing.json), [every exact command and raw log](clean-pipeline.json).'%epoch,'',
        '[All generation stage costs](slowest-generation-stages.json). Slowest worker phase: '+(('%s / %s: %.3f seconds'%(phases[0]['design'],phases[0]['stage'],phases[0]['worker_wall_seconds'])) if phases else 'not recorded')+'. Worker times include contention and are not added to campaign wall time.', '',
        '## Static checks and native sampling','',
        '%d serialized inventory entries; %d native XML table nodes; %d XML coefficient values; %d XML axis knots; %d logical coefficient values; %d carriage serialized values. These representations are not independent physical observations.'%(counts['table_inventory_entries'],counts['native_xml_tables'],counts['native_xml_table_cells'],counts['native_xml_serialized_axis_knots'],counts['bundle_numeric_table_values'],counts['carriage_serialized_table_values']),
        'Every emitted table receives finite/grid/missing-cell/shape/boundary/source/method/version/basis checks. CFD tables: zero; newly measured tables: zero. Basis counts: '+json.dumps(audit['tables_by_basis'])+'. Method families: '+json.dumps(audit['tables_by_method_family'])+'.',
        '%d requested native dynamic sampling conditions (210/design), %d short native/installed six-axis rows, %d reconstruction smoke checks, %d released-body observations. Requests and realized states are both retained. This is sampling, never all-knot dynamic or full-envelope validation.'%(counts['native_dynamic_sampling_conditions'],counts['short_flight_samples'],counts['speed_mass_reconstruction_smoke_checks'],counts['released_store_body_observations']),
        '%d consistency predicates are separate applicability/check/status agreement checks, not independent physical tests. Full-static reload predicates: %d. Static/provenance/execution-summary audit failures: %d.'%(counts['consistency_predicates'],counts['replayed_full_static_predicates'],len(audit['failed_checks'])),
        '[Full table inventory](table-method-inventory.csv), [per-table provenance](table-method-inventory.json), [complete audit](freshness-and-counts.json), [all 88 matrix rows](%s/matrix.csv).'%epoch,'',
        '## Fresh unfiltered regressions','', '| Preset | Discovered | Records | Failed | CTest skips | Nested methods reported/executed | Nested skips | Uncovered skips | Exact IDs |','|---|---:|---:|---:|---:|---:|---:|---:|---|']
    if comparable and not resolution_change:
        before=comparable['campaign_timing']['elapsed_seconds']
        lines[lines.index('## Static checks and native sampling'):lines.index('## Static checks and native sampling')]=[
            'Same fresh-fitted 88-design workload: **%.3f -> %.3f seconds (%.2f%% reduction)**. Both runs regenerate all aerodynamic grids, native-fitted damage/pieces, install once, and run the identical native/installed sampling suite. The optimized run reallocates the same maximum16 native CPU-thread reservation and orders peers by explicit observed cost.'%(before,timing['elapsed_seconds'],100*(1-timing['elapsed_seconds']/before)),
            '[Before-run timing and source provenance](optimization-baseline.json), [all-design numerical/product equality](optimization-equality.json). Equality established: **%s**.'%bool(equality and equality.get('passed')),'']
    if attempts.get('attempts'):
        details=['All measured optimization attempts are disclosed; these are complete campaign clocks, not per-batch clocks.','',
            '| Attempt | Generation workers | Geometry threads | Generation s | Install s | Native s | Campaign s |',
            '|---|---:|---:|---:|---:|---:|---:|']
        for attempt in attempts['attempts']:
            t=attempt['campaign_timing']
            details.append('| %s | %d | %d | %.3f | %.3f | %.3f | %.3f |'%(attempt['id'],attempt['generation_workers'],attempt['geometry_threads'],t['generation_phase_seconds'],t['single_install_phase_seconds'],t['serial_native_validation_phase_seconds'],t['elapsed_seconds']))
        details+=['','Sum of the listed campaign invocations: %.3f seconds. Clean-build/full-regression clocks are reported separately in [the complete attempt ledger](optimization-attempts.json); prior attempts are not hidden by selecting a shorter final run.'%sum(a['campaign_timing']['elapsed_seconds'] for a in attempts['attempts']),'']
        at=lines.index('## Static checks and native sampling');lines[at:at]=details
    if evaluator:
        at=lines.index('## Fresh unfiltered regressions')
        lines[at:at]=['Reference-evaluator optimization: %d recorded states / %d binary64 coefficient comparisons across %d designs; zero-difference result **%s**. These are evaluator-equivalence comparisons, not physical tests. [Exact inputs, source hashes and results](reference-evaluator-equivalence.json).'%(evaluator['recorded_states'],evaluator['coefficient_comparisons'],evaluator['design_count'],evaluator['passed']),'']
    for suite in suites:
        lines.append('| %s | %d | %d | %d | %d | %d/%d | %d | %d | %s |'%(suite['preset'],suite['discovered'],suite['executed_records'],suite['failed'],suite['skipped'],suite['hangar_methods_completed'],suite.get('hangar_methods_executed',suite['hangar_methods_completed']-suite['hangar_underlying_unittest_skipped']),suite['hangar_underlying_unittest_skipped'],suite.get('uncovered_nested_skips',suite['hangar_underlying_unittest_skipped']),suite['test_names_match'] and suite['hangar_method_coverage_complete']))
    for stage in verification.get('stages',[]):
        if 'unittest_cases' in stage:lines.append('- %s: %s cases; exit %d; %s.'%(stage['name'],stage['unittest_cases'],stage['exit_code'],stage['unittest_summary']))
    if original_burn:
        lines+=['','Original declared-motor flight: %d required design cases, %d passed, %d failed; %d/%d design applicability records. The exact original trajectory/mass/burn-time/speed-gain predicates execute separately from coefficient sampling and are never added to CTest totals.'%(
            original_burn['required_cases'],original_burn['passed_cases'],original_burn['failed_cases'],original_burn['completed_designs'],original_burn['declared_designs']),
            '[All original motor-flight results](regression-replay/original-motor-flight.json).']
    if resolution_change:
        lines+=['','The final source contains a separate numerical-resolution correction: every authored design selects the same version2 derived fixed-substep / AB3 rotational-rate policy, including AERORP-to-source-CG-hull transfer. No geometry, mass, force coefficient or controller gain is fitted. Absent configuration and version1 assets retain their existing behavior.',
            'This changes internal propagation work and runtime XML. Final aggregate timing is therefore not an identical-discretization optimization comparison to the earlier identity-step epoch. The controlled optimization result remains the preceding 2048.546 -> 1734.571 second campaign; its complete timing/source/equality evidence is preserved. Final current-source timing and all additional attempts are disclosed separately.',
            '[Uniform source-option changes](numerical-resolution-change.json), [bounded original-scene refinement](numerical-refinement.json).']
    lines+=['','Full regressions use no selector and --no-tests=error. Nested unittest methods and consistency predicates are never added to the CTest record total. Source exports have no Git metadata; any source-checkout-only skip remains explicit.',
        '[Exact discovery, commands, raw output and JUnit](regression-replay/verification.json).', '', '## Independent review','',json.dumps(review,ensure_ascii=True,indent=2) if review else 'Final independent review is missing.','',
        '## HEAD plus one patch replay','']
    if patch:
        lines+=['Base HEAD `%s`; patch SHA256 `%s`; %d bytes; %d changed paths; %d explicit snapshot paths; %d byte-verification failures; main index unchanged %s.'%(patch['base_head'],patch['patch_sha256'],patch['patch_bytes'],patch['patch_changed_file_count'],patch['snapshot_file_count'],len(patch['byte_verification_failures']),patch['real_index_unchanged']),
            'Actual ordinary archive-plus-patch export: `%s`. No branch, worktree, commit or push. Required pre-existing SDK/Hangar/bootstrap prerequisites are included; unrelated Viewer changes are excluded.'%patch['source_export'],
            '[One binary patch](sim-m02.patch), [hash/path manifest](patch-manifest.json), [changed files](patch-files.txt), [diff statistics](patch-stat.txt).']
    else:lines.append('Applied clean patch evidence is missing.')
    lines+=['','## Remaining failed or missing gates','']
    failures=clean.get('failed_gates',[])
    lines.extend('- '+json.dumps(f,ensure_ascii=True) for f in failures)
    if original_burn:
        for row in original_burn['designs']:
            if row['applicability']=='required' and not row['passed']:
                lines.append('- Original motor-flight case `'+row['name']+'` failed: '+', '.join(c['name'] for c in row.get('failed_checks',[]))+(('; '+row['error']) if row.get('error') else '')+'.')
    if not evaluator_ok:lines.append('- Complete recorded-state evaluator-equivalence evidence is missing or failed.')
    for suite in suites:
        junit=Path(suite['junit'])
        if not junit.is_file():continue
        failed=[c for c in ET.parse(junit).findall('.//testcase') if c.find('failure') is not None or c.find('error') is not None]
        for case in failed:lines.append('- '+suite['preset']+': `'+case.get('name','unknown').replace('`','\\`')+'` failed; raw case output is retained in `'+junit.relative_to(out).as_posix()+'`.')
    if (out/'original-regression-review.json').is_file():
        lines.append('- [Original physical/control failure review](original-regression-review.json), with unchanged-scenario traces and rejected control-experiment evidence under `physical-diagnostics/`. No original assertion is waived.')
    if (out/'real-checkout-only-test.json').is_file():
        lines.append('- [Initial Git-checkout-only fixture](real-checkout-only-test.json) passed separately on main; this does not erase or reclassify its no-Git-export skip.')
    if verification.get('checkout_fixture'):
        lines.append('- The unchanged original Git-only fixture was executed separately on main only after exact source/input hashes matched the tested export. Its actual result and per-preset uncovered skip counts remain in `regression-replay/verification.json`; no export skip is renamed to a pass. Any unexecuted/uncovered skip rejects whole acceptance.')
    if not review or not review.get('accepted'):lines.append('- Independent final acceptance is not established.')
    missing=[]
    for row in matrix['designs']:
        for role,c in row['components'].items():
            if c['applicability']=='required' and not c['present']:missing.append(row['name']+'/'+role)
    lines.append('- Missing required components: '+(', '.join(missing) or 'none in the current matrix')+'.')
    lines+=['','## Engineering limits','',
        '- High-angle/Mach/unsteady/additive control-rate continuations are labeled engineering estimates or simulations, not measured equipment accuracy.',
        '- Short native flights and instantaneous drag power do not certify trimmed maximum performance or long-duration stability.',
        '- Virtual cruise configuration geometry, propulsion, mass distribution and control estimates are explicit source inputs; no exact-variant or terminal-seeker fidelity claim.',
        '- Point fuel-mass approximation and reduced rotor response models retain documented uncertainty. Reported supplied-engine fuel flow is not an exact tank-depletion ledger.',
        '- Optional measured-calibration debt is retained separately. No new visual acceptance is claimed.','',
        '## All designs','', '| Design | Category | Status | Missing required components |','|---|---|---|---|']
    for row in matrix['designs']:
        roles=[k for k,v in row['components'].items() if v['applicability']=='required' and not v['present']]
        lines.append('| %s | %s | %s | %s |'%(row['name'],row['category'],row['status'],', '.join(roles) or '-'))
    text='\n'.join(lines)+'\n'
    if '\ufffd' in text:raise ValueError('Replacement character in delivery')
    (out/'delivery.md').write_text(text,encoding='utf-8')
    if args.update_docs:update_docs(out,audit,matrix,verification,epoch)
    if args.archive:archive_evidence(out)
    print(out/'delivery.md')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',required=True);p.add_argument('--update-docs',action='store_true');p.add_argument('--archive',action='store_true');main(p.parse_args())
