"""Publish complete production artifacts and passive table/package inventory.

No test, simulator, consumer, sampling or regression process is executed.
"""
import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import shutil
import sys
import time

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'tools/models/hangar'))
from engineering_inventory import inspect
from engineering_delivery import cells,knots,numeric_values,compare


def read(path):
    def invalid(value):raise ValueError('Nonfinite JSON value: '+value)
    return json.loads(Path(path).read_text(encoding='utf-8'),parse_constant=invalid)


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def write(path,value):Path(path).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n',encoding='utf-8')


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
    for dll in (() if args.tables_only else (prefix/'bin/libfsim.dll',ROOT/'build/ucrt64-release/bin/libfsim.dll',
                ROOT/'build/ucrt64-release/python/fsim/bin/libfsim.dll')):
        if sha(dll)!=installed['native_dll_sha256']:raise ValueError('Current production DLL differs from installation manifest: '+str(dll))
    if basic:
        binding=basic['binding']
        for actual,expected in ((sha(Path(__file__).with_name('engineering_basic.py')),binding['wrapper_sha256']),
            (sha(ROOT/'tools/models/hangar/hangar/engineering_runtime.py'),binding['probe_sha256']),
            (sha(production/'installed-production-manifest.json'),binding['installed_manifest_sha256']),
            (sha(out/'basic/input-production-matrix.json'),binding['input_matrix_sha256']),
            (sha(production/'matrix.json'),binding['output_matrix_sha256'])):
            if actual!=expected:raise ValueError('Basic consumer evidence belongs to another source/product epoch')
    counts=Counter();basis=Counter();families=Counter();inventory=[];xml_tables=[];products={};methods={};failures=[]
    consistency=[]
    if matrix['declared_count']!=88 or matrix['completed_count']!=88 or len({x['name'] for x in matrix['designs']})!=88:
        raise ValueError('Incomplete declared production roster')
    for row in matrix['designs']:
        name=row['name'];folder=production/name
        receipt=read(folder/'generation.json')
        if basic and sha(folder/'generation.json')!=basic['binding']['generation_receipts'][name]:
            raise ValueError('Basic generation receipt drift: '+name)
        for key in ('input_sha256','generator_sha256','products','components','reference_bundle'):
            if receipt[key]!=row[key]:raise ValueError('Receipt/production matrix mismatch: '+name+'/'+key)
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
        if basic:
            for mode in ('native','installed'):
                runtime=read(folder/(mode+'.json'))
                if sha(folder/(mode+'.json'))!=basic['binding']['runtime_results'][name+'/'+mode+'.json']:
                    raise ValueError('Basic runtime result drift: '+name+'/'+mode)
                if runtime['loaded_dll_sha256']!=installed['native_dll_sha256'] or runtime['products']!=row['products']:
                    raise ValueError('Basic consumer used a different product/DLL epoch: '+name+'/'+mode)
                if not all(c['passed'] for c in runtime['checks']):raise ValueError('Basic native usability failure: '+name+'/'+mode)
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
        report['counts'].update(basic_native_designs=basic['counts']['native_consumers'],
            basic_installed_designs=basic['counts']['installed_consumers'],
            basic_flight_observations=basic['counts']['short_flight_samples'],
            basic_released_store_bodies=basic['counts']['released_store_bodies'],
            test_executions=basic['native_components']['executed'])
    write(out/'freshness-and-counts.json',report)
    write(out/'table-method-inventory.json',dict(sources=matrix['sources'],tables=inventory))
    write(out/'method-source-versions.json',methods)
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
