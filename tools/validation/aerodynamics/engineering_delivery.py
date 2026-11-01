"""Reproducible full-table/provenance audit; native sampling counts stay separate.

This audits emitted data, not independent physical accuracy. No reference data,
solver tolerance or engineering applicability is changed by this reporter.
"""
import argparse
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from engineering_inventory import inspect


def read(path):
    def invalid(value):
        raise ValueError('Nonfinite JSON number: ' + value)
    return json.loads(Path(path).read_text(encoding='utf-8'), parse_constant=invalid)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def sampling_integration_current(sample,declared):
    """Reload actual numerical configuration evidence; no FDM execution."""
    try:
        expected=declared.get('rotational_rate_method',1);dt=declared['maximum_internal_dt_s']
        value=sample['native_integration'];raw=value['raw_reset_rotational_rate_method']
        if type(expected) is not int or expected not in (1,4) or isinstance(dt,bool) or not math.isfinite(dt) or dt<=0:return False
        if isinstance(raw,bool) or not math.isfinite(raw):return False
        for key in ('before_step','after_step'):
            actual=value[key];actual_dt=actual['internal_dt_s'];method=actual['rotational_rate_method']
            if isinstance(method,bool) or method!=expected or isinstance(actual_dt,bool) or not math.isfinite(actual_dt) or actual_dt<=0:return False
            if not math.isclose(actual_dt,dt,rel_tol=1.e-12,abs_tol=1.e-15):return False
        return True
    except (KeyError,TypeError,ValueError):return False


def write(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def cells(dimensions):
    return dimensions.get('value_count', 0) + sum(cells(s) for s in dimensions.get('slices', []))


def knots(dimensions):
    return sum(a['points'] for a in dimensions.get('axes', [])) + sum(knots(s) for s in dimensions.get('slices', []))


def numeric_values(value):
    if isinstance(value, dict):return sum(numeric_values(v) for v in value.values())
    if isinstance(value, list):return sum(numeric_values(v) for v in value)
    if isinstance(value, (int,float)) and not isinstance(value,bool):
        if not math.isfinite(value):raise ValueError('Nonfinite serialized value')
        return 1
    return 0


def arrays(value):
    if isinstance(value,dict):return {k:arrays(v) for k,v in value.items()}
    if isinstance(value,list):
        if not value:return []
        try:return np.asarray(value,dtype=float)
        except (TypeError,ValueError):return [arrays(v) for v in value]
    return value


def compare(old, new, path, differences):
    if isinstance(old, dict) and isinstance(new, dict):
        if old.keys() != new.keys():
            differences.append(dict(path=path, reason='different keys'))
        for key in old.keys() & new.keys():
            compare(old[key], new[key], path + '/' + key, differences)
    elif isinstance(old, list) and isinstance(new, list):
        a, b = np.asarray(old), np.asarray(new)
        if a.dtype.kind in 'fiu' and b.dtype.kind in 'fiu':
            if a.shape != b.shape or not np.array_equal(a, b):
                differences.append(dict(path=path, old_shape=list(a.shape), new_shape=list(b.shape),
                    max_abs=float(np.max(np.abs(a-b))) if a.shape == b.shape else None))
        else:
            if len(old) != len(new):differences.append(dict(path=path, reason='different lengths'))
            for i, (a, b) in enumerate(zip(old, new)):compare(a, b, path + '/' + str(i), differences)
    elif old != new:
        differences.append(dict(path=path, old=old, new=new))


def audit(args):
    started = time.monotonic()
    matrix_dir = Path(args.matrix).resolve();out = Path(args.output).resolve();out.mkdir(parents=True, exist_ok=True)
    matrix = read(matrix_dir/'matrix.json');prefix = Path(args.prefix).resolve()
    checks = [];consistency = [];inventory = [];xml_rows = [];comparison = [];profile_inputs=[];counts = Counter();basis = Counter();families=Counter()
    def check(name, passed):checks.append(dict(name=name, passed=bool(passed)))
    def consistent(name, passed):consistency.append(dict(name=name, passed=bool(passed)))
    check('complete unique roster',matrix['completed_count']==matrix['declared_count']==88 and len({r['name'] for r in matrix['designs']})==88)
    dll = sha(ROOT/'build/ucrt64-release/bin/libfsim.dll')
    check('installed DLL current',sha(prefix/'bin/libfsim.dll')==dll)
    old_matrix = read(Path(args.baseline)/'matrix.json') if args.baseline else None
    old_rows = {r['name']:r for r in old_matrix['designs']} if old_matrix else {}
    from hangar import engineering, pipeline
    for row in matrix['designs']:
        name=row['name'];named={c['name']:c['passed'] for c in row['checks']}
        directory=ROOT/'aircraft'/name
        profile_inputs.append(dict(name=name,inputs=[dict(path=str((directory/p).relative_to(ROOT)).replace('\\','/'),sha256=sha(directory/p))
            for p in ('autopilot.toml','hover.toml','flight-calibration.json','out/fly.json','out/performance.json','out/ground.json') if (directory/p).is_file()],
            basis='retained identified-controller and flight-planning inputs; not new measurements or freshly trimmed maximum-performance evidence'))
        for role, component in row['components'].items():
            if name in old_rows and isinstance(component.get('derivation'),dict) and 'availability' in component['derivation']:
                consistent(name+'/'+role+'/source-declaration-preserved',component['derivation']==old_rows[name]['components'][role]['derivation'])
            if role not in named:continue
            consistent(name+'/'+role+'/applicability-check', named[role] == (component['applicability']=='not_applicable' or component['present']))
            if component.get('availability')=='unavailable':
                consistent(name+'/'+role+'/required-absent-blocked', component['applicability']=='required' and not component['present'] and not named[role] and row['status']=='blocked')
        consistent(name+'/completion-label',row['status']=='blocked' if row['reasons'] else row['status'] in ('engineering-complete','engineering-complete-uncalibrated'))
        generation_source=ROOT
        binding=row.get('execution_binding')
        if binding:
            # Execution-only repairs retain the actual historical generation
            # source/receipt. Verify input/product identity and reject changed
            # aerodynamic code rather than claiming a fresh table generation.
            from engineering_revalidate import verify_generation
            generation_source=Path(binding['generation_source_export'])
            source_receipt=matrix_dir/name/'source-generation.json'
            check(name+'/execution-binding/source-receipt',sha(source_receipt)==binding['source_receipt_sha256'])
            original=read(source_receipt)
            for key in ('input_sha256','generator_sha256','products'):
                check(name+'/execution-binding/'+key,row[key]==original[key])
            check(name+'/execution-binding/current-DLL',binding['native_dll_sha256']==row['native_dll_sha256']==dll)
            check(name+'/execution-binding/generation-DLL',binding['original_native_dll_sha256']==original['native_dll_sha256'])
            changes=verify_generation(row,generation_source)
            check(name+'/execution-binding/input-reconstruction',changes==binding['reconstructed_input_writers'])
        for kind in ('input_sha256','generator_sha256'):
            base=generation_source if kind=='generator_sha256' else ROOT
            for relative, expected in row[kind].items():check(name+'/'+kind+'/'+relative,sha(base/relative)==expected)
        backend=row.get('geometry_backend')
        if backend:
            check(name+'/native-geometry-backend',Path(backend['path']).is_file() and sha(backend['path'])==backend['sha256'])
        carriage=[]
        for location,base in [('source',ROOT/'aircraft'),('native',ROOT/'build/ucrt64-release/share/flightsim/aircraft'),('installed',prefix/'share/flightsim/aircraft')]:
            check(name+'/'+location+'/no-test-reference-in-runtime-assets',not (base/row['product_directory']/'engineering-aero-v1.json').exists())
        for relative, expected in row['products'].items():
            for location, base in [('source',ROOT/'aircraft'),('native',ROOT/'build/ucrt64-release/share/flightsim/aircraft'),('installed',prefix/'share/flightsim/aircraft')]:
                check(name+'/'+location+'/'+relative,sha(base/relative)==expected)
            if Path(relative).suffix=='.xml':
                tables=inspect(prefix/'share/flightsim/aircraft'/relative)
                xml_rows.extend(dict(design=name,**t) for t in tables)
                counts['native_xml_table_cells']+=sum(cells(t) for t in tables)
                counts['native_xml_serialized_axis_knots']+=sum(knots(t) for t in tables)
                for t in tables:check(name+'/all-XML-table/'+relative+'/'+str(t['index']),t['passed'])
            elif Path(relative).suffix=='.json':
                payload=read(prefix/'share/flightsim/aircraft'/relative)
                counts['auxiliary_runtime_json_numeric_values_scanned']+=numeric_values(payload)
                if payload.get('schema')=='fsim.airframe-aerodynamics.v1':carriage.append((relative,payload))
        reference_path=matrix_dir/name/row['reference_bundle']['name']
        check(name+'/current-test-reference-bundle',sha(reference_path)==row['reference_bundle']['sha256'])
        bundle=read(reference_path)
        counts['serialized_bundle_numeric_values_scanned']+=numeric_values(bundle)
        design=pipeline.Design(str(ROOT/'aircraft'/name/(name+'.toml')),log=lambda *args:None)
        materialized=arrays(bundle['tables'])
        original_body=materialized.get('body_base')
        _, static_checks=engineering.table_audit(materialized,design.aircraft)
        counts['replayed_full_static_predicates']+=len(static_checks)
        for c in static_checks:check(name+'/replayed-static/'+c['name'],c['passed'])
        check(name+'/all-derived-body-cells',all(np.array_equal(original_body[k],materialized['body_base'][k]) for k in original_body))
        check(name+'/forced-generation',bundle['regenerated'])
        for item in bundle['inventory']:
            check(name+'/table-metadata/'+item['name'], all(item.get(k) for k in ('method','source','version','basis','derivation','interpolation','extrapolation')) and item['basis'] in ('measured','estimated','simulated'))
            if 'dimensions' in item:
                properties=[v['property'] for v in item['dimensions'].get('independent_variables',[])]
                family=('native emission of shared aerodynamic tables' if any(p.startswith('aero/alpha-') for p in properties) else 'native emission of engine/control/engineering lookup')
            elif item['basis']=='simulated':
                family='VLM with explicit engineering closures' if row['lifting_panels'] else 'body-only engineering model (zero lifting panels)'
            else:family='engineering estimate/continuation'
            entry=dict(design=name,method_family=family,**item);inventory.append(entry);basis[item['basis']]+=1;families[family]+=1
            if 'shape' in item:counts['bundle_numeric_table_values']+=int(np.prod(item['shape']))
        for relative,payload in carriage:
            selected={'alpha','beta','base','rates','alphadot','ground_effect','mach'}
            delta=[]
            compare({k:bundle['tables'][k] for k in selected},payload['tables'],'carriage',delta)
            check(name+'/all-carriage-cells-and-grids/'+relative,not delta)
            for item in bundle['inventory']:
                if not item['name'].startswith(('base/','rate/','alpha-dot/','Mach/','ground-effect/')):continue
                entry=dict(item,design=name,name='carriage/'+relative+'/'+item['name'],
                    method_family='carriage emission of shared tables',method='Exact serialization of the same shared neutral-stowed tables',
                    source=dict(runtime_product=relative,derivation_sources=item['source']),version=payload['schema'],
                    derivation='Same logical table '+item['name']+'; every cell and grid compared exactly with the regenerated test reference')
                inventory.append(entry);basis[entry['basis']]+=1;families[entry['method_family']]+=1
                counts['carriage_serialized_table_values']+=int(item.get('value_count',0))
        # Full shape/finite/axis/domain checks are generated from the actual
        # arrays by table_audit; XML grammar is independently parsed above.
        counts['generation_reference_structure_predicates']+=len(row['checks'])
        counts['required_component_failures']+=sum(not c['passed'] for c in row['checks'])
        for mode in ('native','installed'):
            runtime=read(matrix_dir/name/(mode+'.json'))
            check(name+'/'+mode+'/current-DLL',runtime['loaded_dll_sha256']==dll)
            check(name+'/'+mode+'/checks',all(c['passed'] for c in runtime['checks']))
            counts['native_and_installed_execution_predicates']+=len(runtime['checks'])
            counts['short_flight_samples']+=sum(len(t['rows']) for t in runtime['traces'])
            counts['speed_mass_reconstruction_smoke_checks']+=sum(c['name'].startswith('World reconstruction speed/mass repeatability/') for c in runtime['checks'])
            counts['released_store_body_observations']+=bool(runtime['release'])
        sampling=read(matrix_dir/name/'envelope.json')
        check(name+'/native-sampling',sampling['case_count']==210 and not sampling['failures'])
        check(name+'/current-sampling-tool',sampling['source_sha256']==sha(ROOT/'tools/validation/aerodynamics/engineering_sdk.py'))
        check(name+'/current-sampling-oracle',sampling['oracle_sha256']==sha(ROOT/'tools/models/hangar/hangar/engineering_runtime.py'))
        check(name+'/sampling/current-DLL',sampling['loaded_dll_sha256']==dll)
        check(name+'/sampling/current-products',sampling['products']==row['products'])
        check(name+'/complete-sampling-rows',len(sampling['rows'])==210)
        for index,sample in enumerate(sampling['rows']):
            check(name+'/sampled-native-integration/'+str(index),sampling_integration_current(sample,row['integration']))
            counts['sampled_native_integration_consistency_predicates']+=1
        counts['native_dynamic_sampling_conditions']+=sampling['case_count']
        if name in old_rows:
            check(name+'/historical-lattice-size-preserved',row['lifting_panels']==old_rows[name]['lifting_panels'])
            old_folder=(Path(args.baseline_prefix)/'share/flightsim/aircraft'/old_rows[name]['product_directory']
                if args.baseline_prefix else Path(args.baseline)/name)
            changes=[]
            old_tables=read(old_folder/'engineering-aero-v1.json')['tables']
            # Private cache fingerprints necessarily change with source. The
            # historical comparison includes the corrected wake-cache behavior;
            # it is disclosed numerical change, not an equivalence assertion.
            compare({k:v for k,v in old_tables.items() if not k.startswith('_')},
                {k:v for k,v in bundle['tables'].items() if not k.startswith('_')},'tables',changes)
            axis_names={'alpha','beta','deflection','mach','h_b'}
            structural=[d for d in changes if 'reason' in d or d.get('old_shape')!=d.get('new_shape') or d['path'].rsplit('/',1)[-1] in axis_names]
            check(name+'/historical-grid-and-layout-preserved',not structural)
            comparison.append(dict(name=name,numerically_identical=not changes,differences=changes))
    counts['native_xml_tables']=len(xml_rows);counts['table_inventory_entries']=len(inventory)
    counts['consistency_predicates']=len(consistency);counts['provenance_static_execution_summary_predicates']=len(checks)
    elapsed=time.monotonic()-started
    timing=read(matrix_dir/'campaign-timing.json')
    result=dict(schema='fsim.sim-m02.delivery-audit.v2',statuses=matrix['counts'],design_count=88,counts=dict(counts),tables_by_basis=dict(basis),tables_by_method_family=dict(families),
        campaign_timing=timing,post_campaign_audit_seconds=elapsed,generation_validation_and_audit_seconds=timing['elapsed_seconds']+elapsed,
        current_native_dll_sha256=dll,checks=checks,consistency_predicates=consistency,
        failed_checks=[c for c in checks+consistency if not c['passed']],
        blocked=[dict(name=r['name'],components={k:v for k,v in r['components'].items() if v.get('availability')=='unavailable'},reasons=r['reasons']) for r in matrix['designs'] if r['status']=='blocked'],
        scope='Full emitted-table static checks and execution consistency. Counts are not independent physical accuracy tests. Native dynamic validation is sampling.',
        grid_and_value_comparison=dict(scope='Historical wake-cache correction deltas; not numerical equivalence. No acceptance tolerance changed. Exact cache/traversal tests and original native/reference/regression gates are separate.',compared_designs=len(comparison),identical_designs=sum(r['numerically_identical'] for r in comparison),designs=comparison))
    write(out/'freshness-and-counts.json',result)
    write(out/'table-method-inventory.json',dict(sources=matrix['sources'],tables=inventory))
    write(out/'retained-profile-inputs.json',dict(designs=profile_inputs))
    write(out/'native-table-inventory.json',dict(design_count=88,table_count=len(xml_rows),value_count=counts['native_xml_table_cells'],tables=xml_rows))
    with (out/'table-method-inventory.csv').open('w',newline='',encoding='utf-8') as stream:
        writer=csv.writer(stream);writer.writerow(['design','table','basis','method_family','method','version','source','derivation','axes','interpolation','extrapolation'])
        for t in inventory:writer.writerow([t['design'],t['name'],t['basis'],t['method_family'],t['method'],t['version'],json.dumps(t['source']),t['derivation'],json.dumps(t.get('axes',t.get('dimensions'))),t['interpolation'],t['extrapolation']])
    print(json.dumps(dict(statuses=result['statuses'],counts=dict(counts),failed_checks=len(result['failed_checks']),elapsed_seconds=elapsed,identical_designs=result['grid_and_value_comparison']['identical_designs']),indent=2))
    return int(bool(result['failed_checks']))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--matrix',required=True);parser.add_argument('--prefix',required=True);parser.add_argument('--output',required=True)
    parser.add_argument('--baseline');parser.add_argument('--baseline-prefix')
    raise SystemExit(audit(parser.parse_args()))
