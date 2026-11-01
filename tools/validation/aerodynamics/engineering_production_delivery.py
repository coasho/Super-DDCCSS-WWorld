"""Publish complete production artifacts and passive table/package inventory.

No test, simulator, consumer, sampling or regression process is executed.
"""
import argparse
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys
import time

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools/models/hangar'))
from engineering_inventory import inspect
from engineering_delivery import cells,knots,numeric_values,compare
from engineering_scope import functional_rows


def read(path):
    def invalid(value):raise ValueError('Nonfinite JSON value: '+value)
    return json.loads(Path(path).read_text(encoding='utf-8'),parse_constant=invalid)


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def write(path,value):Path(path).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n',encoding='utf-8')


def audit_control_design(name,condition):
    """Reconcile recorded sizing with authored loadouts; this is consistency."""
    from hangar.carriage import CONTROL_DESIGN_MODEL_SET,CONTROL_DESIGN_SOURCES
    from hangar.aero.mach import MACH_SUB
    if (condition.get('method')!=CONTROL_DESIGN_MODEL_SET or condition.get('basis')!='estimated'
            or condition.get('sources')!=CONTROL_DESIGN_SOURCES):
        raise ValueError('Invalid control-design method/source: '+name)
    requested=condition['conditions']['mach'];cap=float(MACH_SUB[-1])
    if not math.isfinite(requested) or requested<0:raise ValueError('Invalid requested design Mach')
    cases={}
    for alternative in condition['rows']:
        values=alternative['derivatives_m3_per_rad']
        valid=set(values)=={'nominal_shared','attached_recovery_design'}
        valid=valid and all(math.isfinite(v) for v in values.values())
        valid=valid and alternative['selected_case']==min(values,key=values.get)
        valid=valid and alternative['recovery_mach_domain']==[0.,cap]
        valid=valid and alternative['recovery_mach']==min(requested,cap)
        valid=valid and alternative['recovery_mach_hold']==(requested>cap)
        # These assumptions are required metadata; they are not physical proof.
        valid=valid and bool(alternative['frozen_contributions'])
        for relative,digest in alternative['source_sha256'].items():
            valid=valid and sha(ROOT/relative)==digest
        key=(alternative['station'],alternative['store'])
        if key in cases or not valid:raise ValueError('Invalid control-design alternative: '+name)
        cases[key]=values[alternative['selected_case']]
    fit=read(ROOT/'aircraft'/name/'payloads.json');places=None
    for item in fit.get('payloads',[]):
        path=ROOT/'assets/payloads'/(item.get('card','')+'.json')
        if path.is_file():
            card=read(path)
            if card.get('family')=='store_station' and 'stations' in card:
                places={s['number']:s for s in card['stations']}
    if places is None:raise ValueError('Control-design report has no authored station card')
    expected={};used=set();source_order=[]
    for loadout in fit.get('loadouts',[]):
        for item in loadout.get('stations',[]):
            station=item.get('station');store=item.get('store','');place=places.get(station)
            path=ROOT/'assets/stores'/(store+'.json')
            if place is None or not path.is_file():raise ValueError('Authored loadout requires unavailable store/station')
            if 'bay' in place.get('name',''):continue
            store=read(path)['id']
            key=(station,store)
            if key not in cases:raise ValueError('Missing authored control-design store: '+str(key))
            if key not in used:source_order.append(key)
            used.add(key);per_unit=cases[key];count=int(item.get('count',1));raw=per_unit*count
            if station not in expected or raw<expected[station]['selected_penalty_m3_per_rad']:
                expected[station]=dict(station=station,store=store,count=count,
                    per_unit_m3_per_rad=per_unit,selected_penalty_m3_per_rad=min(0.,raw))
    if used!=set(cases) or source_order!=list(cases):raise ValueError('Control-design alternatives differ from authored loadouts')
    if condition['station_selections']!=[expected[s] for s in sorted(expected)]:
        raise ValueError('Incorrect authored station winner/count/penalty: '+name)
    if condition['selected_station_sum_m3_per_rad']!=sum(s['selected_penalty_m3_per_rad'] for s in expected.values()):
        raise ValueError('Incorrect control-design aggregate: '+name)


def main(args):
    started=time.monotonic();production=Path(args.production).resolve();out=Path(args.output).resolve()
    out.mkdir(exist_ok=True,parents=True);matrix=read(production/'matrix.json')
    installed=read(production/'installed-production-manifest.json')
    prefix=Path(args.installed_prefix or installed['prefix'])
    if '.codex' in {p.casefold() for p in prefix.parts}:raise ValueError('Excluded installed prefix')
    if args.installed_prefix and not args.tables_only:
        raise ValueError('A different runtime epoch requires an explicitly table-only audit')
    basic_path=out/'basic/basic-usability.json'
    basic=read(basic_path) if basic_path.is_file() and not args.tables_only else None
    active_names=set();deferred_names=set()
    for dll in (() if args.tables_only else (prefix/'bin/libfsim.dll',ROOT/'build/ucrt64-release/bin/libfsim.dll',
                ROOT/'build/ucrt64-release/python/fsim/bin/libfsim.dll')):
        if sha(dll)!=installed['native_dll_sha256']:raise ValueError('Current production DLL differs from installation manifest: '+str(dll))
    if basic:
        binding=basic['binding']
        scope=binding.get('functional_scope','all')
        active,deferred=functional_rows(matrix['designs'],scope)
        active_names={r['name'] for r in active};deferred_names={r['name'] for r in deferred}
        if scope!='all':
            if (binding.get('scope_implementation_sha256')!=sha(Path(__file__).with_name('engineering_scope.py'))
                    or binding.get('active_designs')!=len(active)
                    or set(binding.get('deferred_designs',[]))!=deferred_names):
                raise ValueError('Basic functional population binding differs')
        expected_results={name+'/'+mode+'.json' for name in active_names for mode in ('native','installed')}
        if set(binding['runtime_results'])!=expected_results:
            raise ValueError('Basic runtime receipts do not cover exactly the active population')
        for actual,expected in ((sha(Path(__file__).with_name('engineering_basic.py')),binding['wrapper_sha256']),
            (sha(ROOT/'tools/models/hangar/hangar/engineering_runtime.py'),binding['probe_sha256']),
            (sha(production/'installed-production-manifest.json'),binding['installed_manifest_sha256']),
            (sha(out/'basic/input-production-matrix.json'),binding['input_matrix_sha256']),
            (sha(production/'matrix.json'),binding['output_matrix_sha256'])):
            if actual!=expected:raise ValueError('Basic consumer evidence belongs to another source/product epoch')
    counts=Counter();basis=Counter();families=Counter();inventory=[];xml_tables=[];products={};methods={};failures=[]
    consistency=[];control_design={}
    if matrix['declared_count']!=88 or matrix['completed_count']!=88 or len({x['name'] for x in matrix['designs']})!=88:
        raise ValueError('Incomplete declared production roster')
    for row in matrix['designs']:
        name=row['name'];folder=production/name
        receipt=read(folder/'generation.json')
        control_design[name]=receipt.get('control_design',[])
        for condition in control_design[name]:
            audit_control_design(name,condition)
        if basic and sha(folder/'generation.json')!=basic['binding']['generation_receipts'][name]:
            raise ValueError('Basic generation receipt drift: '+name)
        for key in ('input_sha256','input_presence','generator_sha256','products','components','reference_bundle'):
            if receipt[key]!=row[key]:raise ValueError('Receipt/production matrix mismatch: '+name+'/'+key)
        for relative,present in receipt['input_presence'].items():
            if (ROOT/relative).is_file()!=present:raise ValueError('Production input presence changed: '+relative)
        for key in ('input_sha256','generator_sha256'):
            for relative,expected in receipt[key].items():
                if sha(ROOT/relative)!=expected:raise ValueError('Changed production source: '+relative)
        implementation=hashlib.sha256(json.dumps(row['generator_sha256'],sort_keys=True).encode()).hexdigest()
        methods[name]=dict(implementation_sha256=implementation,source_versions=row['generator_sha256'],
            input_versions=row['input_sha256'],reference_bundle=row['reference_bundle'])
        bundle_path=folder/row['reference_bundle']['name']
        if sha(bundle_path)!=row['reference_bundle']['sha256']:raise ValueError('Changed generation bundle')
        bundle=read(bundle_path)
        counts['serialized_bundle_numeric_values']+=numeric_values(bundle)
        counts['mandatory_generation_predicates']+=len(row['checks'])
        failed=[c['name'] for c in row['checks'] if not c['passed']]
        if failed:failures.append(dict(design=name,failed_generation_contracts=failed))
        named={c['name']:c['passed'] for c in row['checks']}
        if basic and name in active_names:
            for mode in ('native','installed'):
                runtime=read(folder/(mode+'.json'))
                if sha(folder/(mode+'.json'))!=basic['binding']['runtime_results'][name+'/'+mode+'.json']:
                    raise ValueError('Basic runtime result drift: '+name+'/'+mode)
                if runtime['loaded_dll_sha256']!=installed['native_dll_sha256'] or runtime['products']!=row['products']:
                    raise ValueError('Basic consumer used a different product/DLL epoch: '+name+'/'+mode)
                if not all(c['passed'] for c in runtime['checks']):raise ValueError('Basic native usability failure: '+name+'/'+mode)
        elif basic and name in deferred_names:
            if row.get('basic_usability',{}).get('passed') is not None or not row.get('basic_usability',{}).get('deferred'):
                raise ValueError('Deferred design was counted as passed: '+name)
            for mode in ('native','installed'):
                runtime=read(folder/(mode+'.json'))
                if (runtime.get('schema')!='fsim.sim-m02.functional-deferred.v1'
                        or runtime.get('design')!=name or runtime.get('mode')!=mode
                        or runtime.get('executed') is not False or runtime.get('passed') is not None
                        or runtime.get('functional_scope')!=scope):
                    raise ValueError('Invalid deferred metadata: '+name+'/'+mode)
        for role,component in row['components'].items():
            if role not in named:continue
            correct=named[role]==(component['applicability']=='not_applicable' or component['present'])
            consistency.append(dict(design=name,component=role,consistent=correct))
            if not correct:raise ValueError('Inconsistent required-component declaration: '+name+'/'+role)
        for item in bundle['inventory']:
            if not all(item.get(k) for k in ('method','source','version','basis','derivation','interpolation','extrapolation')):
                raise ValueError('Missing table derivation: '+name+'/'+item['name'])
            if item['basis'] not in ('measured','estimated','simulated'):raise ValueError('Invalid table basis')
            if 'dimensions' in item:
                properties=[v['property'] for v in item['dimensions'].get('independent_variables',[])]
                family='native aerodynamic serialization' if any(p.startswith('aero/alpha-') for p in properties) else 'native engine/control serialization'
            elif item['basis']=='simulated':family='VLM and explicit engineering closures' if row['lifting_panels'] else 'body-only engineering limit'
            else:family='engineering estimate/continuation'
            inventory.append(dict(design=name,method_family=family,implementation_sha256=implementation,**item))
            basis[item['basis']]+=1;families[family]+=1
            if 'shape' in item:
                n=1
                for dim in item['shape']:n*=dim
                counts['logical_numeric_table_values']+=n
        for relative,expected in row['products'].items():
            for location,base in [('source',ROOT/'aircraft'),('native',ROOT/'build/ucrt64-release/share/flightsim/aircraft'),
                                   ('installed',prefix/'share/flightsim/aircraft')]:
                if sha(base/relative)!=expected:raise ValueError('Production product mismatch: '+location+'/'+relative)
            products[relative]=expected
            path=prefix/'share/flightsim/aircraft'/relative
            if path.suffix=='.xml':
                entries=inspect(path)
                for item in entries:
                    if not item['passed']:raise ValueError('Malformed production table: '+str(item))
                    xml_tables.append(dict(design=name,**item))
                    counts['native_xml_coefficient_cells']+=cells(item)
                    counts['native_xml_axis_knots']+=knots(item)
            elif path.suffix=='.json':
                payload=read(path);counts['runtime_json_numeric_values']+=numeric_values(payload)
                if payload.get('schema')=='fsim.airframe-aerodynamics.v1':
                    selected={'alpha','beta','base','rates','alphadot','ground_effect','mach'}
                    differences=[]
                    compare({key:bundle['tables'][key] for key in selected},payload['tables'],'carriage',differences)
                    if differences:raise ValueError('Carriage cells/grids differ from current generated source: '+relative)
                    for item in bundle['inventory']:
                        if not item['name'].startswith(('base/','rate/','alpha-dot/','Mach/','ground-effect/')):continue
                        entry=dict(item,design=name,name='carriage/'+relative+'/'+item['name'],
                            method_family='carriage serialization of shared tables',
                            method='Exact serialization of the same shared neutral-stowed tables',
                            source=dict(runtime_product=relative,derivation_sources=item['source']),version=payload['schema'],
                            implementation_sha256=implementation,
                            derivation='Same logical table '+item['name']+'; every serialized cell and grid compared exactly with regenerated source')
                        inventory.append(entry);basis[entry['basis']]+=1;families[entry['method_family']]+=1
                        counts['carriage_serialized_table_values']+=int(item.get('value_count',0))
    counts.update(designs=88,native_xml_tables=len(xml_tables),table_inventory_entries=len(inventory),
        unique_production_products=len(products),consistency_predicates=len(consistency),
        native_dynamic_samples=0,test_executions=0,cfd_tables=0)
    report=dict(schema='fsim.sim-m02.production-delivery.v1',counts=dict(counts),design_statuses=matrix['counts'],
        by_category=dict(Counter(x['category'] for x in matrix['designs'])),tables_by_basis=dict(basis),
        tables_by_method_family=dict(families),failures=failures,prefix=str(prefix),
        production_seconds=read(production/'campaign-timing.json')['elapsed_seconds'],
        passive_publication_seconds=time.monotonic()-started,
        scope='Current production/component/table/package inventory; basic usability results are separate from envelope sampling and full regression.',
        basic_usability=basic,whole_regression_pass_claimed=False,
        tables_only=args.tables_only,native_dll_identity_checked=not args.tables_only,
        runtime_evidence_scope='Not executed or imported in table-only mode' if args.tables_only else 'Original production runtime epoch')
    if basic:
        if basic['failures']:raise ValueError('Basic usability did not pass')
        if any(basic['counts'][key]!=len(active_names) for key in ('designs','native_consumers','installed_consumers')):
            raise ValueError('Basic counts differ from the executed population')
        report.update(functional_scope=scope,active_functional_designs=len(active_names),
            deferred_functional_designs=sorted(deferred_names),deferred_status='not executed, not passed')
        report['counts'].update(basic_native_designs=basic['counts']['native_consumers'],
            basic_installed_designs=basic['counts']['installed_consumers'],
            basic_flight_observations=basic['counts']['short_flight_samples'],
            basic_released_store_bodies=basic['counts']['released_store_bodies'],
            test_executions=basic['native_components']['executed'])
    write(out/'freshness-and-counts.json',report)
    write(out/'table-method-inventory.json',dict(sources=matrix['sources'],tables=inventory))
    write(out/'method-source-versions.json',methods)
    write(out/'control-design-inventory.json',dict(
        scope='Distinct control-sizing design conditions and two estimated flow idealizations; not physical tests or measured load bounds',
        designs=control_design,distinct_condition_count=sum(map(len,control_design.values()))))
    write(out/'native-table-inventory.json',dict(table_count=len(xml_tables),tables=xml_tables))
    write(out/'consistency-predicates.json',dict(scope='Internal document consistency only; not physical tests',predicates=consistency))
    with (out/'table-method-inventory.csv').open('w',newline='',encoding='utf-8') as stream:
        writer=csv.writer(stream);writer.writerow(['design','table','basis','method_family','method','version','implementation_sha256','source','derivation','interpolation','extrapolation'])
        for item in inventory:writer.writerow([item['design'],item['name'],item['basis'],item['method_family'],item['method'],item['version'],item['implementation_sha256'],json.dumps(item['source']),item['derivation'],item['interpolation'],item['extrapolation']])
    shutil.copy2(production/'matrix.csv',ROOT/'docs/data/sim-m02-engineering-completeness.csv')
    print(json.dumps(report,indent=2))
    return int(bool(failures))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--production',required=True);p.add_argument('--output',required=True)
    p.add_argument('--tables-only',action='store_true',help='Retain every table/source/product check; do not import runtime receipts or claim DLL/flight acceptance')
    p.add_argument('--installed-prefix',help='Explicit current product tree for table-only verification after a native-only rebuild')
    raise SystemExit(main(p.parse_args()))
