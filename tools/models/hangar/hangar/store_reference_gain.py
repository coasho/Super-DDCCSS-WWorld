"""Repeat a frozen store calibration against its original public holdouts.

No fitting or publication occurs here. Current observations use the published
production aerodynamic XML; the baseline uses the recorded old parameters.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import tomllib
import xml.etree.ElementTree as ET

from . import aero_evidence, reference_campaign as C, storestages
from .files import atomic_write_text
from .sdk_measurements import Measurements, native_library


def case_digest(cases):
    return hashlib.sha256(json.dumps(cases,sort_keys=True,separators=(',',':')).encode()).hexdigest()


def measure_gain(model,evidence,out):
    source=C.roster()[model]
    if not storestages.is_store(source):raise ValueError('Store measurements only')
    catalog=C.load_catalog();record=catalog['models'][model]
    declared=json.loads(Path(evidence).read_text(encoding='utf-8'))
    if declared.get('schema')=='fsim.public-calibration-data.v1':
        declared=declared['model_audits'][model]['frozen_store_calibration']
    if declared['model']!=model or declared['original_cases_sha256']!=case_digest(record['cases']):
        raise ValueError('The frozen original reference partition changed')
    train=[c for c in record['cases'] if c['split']=='calibration']
    held=[c for c in record['cases'] if c['split']=='validation']
    if not train or not held:raise ValueError('Matched training and holdouts are required')
    parameters=tomllib.loads(source.with_name('calibration.toml').read_text(encoding='utf-8'))['calibration']
    if parameters!=declared['parameters']:raise ValueError('The frozen parameters changed')
    runner=Measurements(model,Path(out).parent/'runtime');cases=train+held
    scaled=any(c['conditions'].get('geometry_scale',1.)!=1. for c in cases)
    baseline=runner.measure(declared['initial_parameters'],cases);baseline_actual=copy.deepcopy(runner.last)
    original_fixture=runner.fixture
    published=ET.fromstring(runner.original).find('aerodynamics')
    def published_fixture(*args,**kwargs):
        name,xml=original_fixture(*args,**kwargs)
        # A scaled wind-tunnel model needs its own Reynolds-matched geometry
        # tables, regenerated with the retained production calibration. Never
        # replace them with full-scale coefficients and call that a match.
        if kwargs.get('geometry_scale',1.)!=1.:return name,xml
        xml.remove(xml.find('aerodynamics'));xml.append(copy.deepcopy(published))
        path=runner.scratch/'aircraft'/name/(name+'.xml')
        ET.ElementTree(xml).write(path,encoding='utf-8',xml_declaration=True)
        runner.fixture_sha256=C.sha(path)
        return name,xml
    runner.fixture=published_fixture
    current=runner.measure(parameters,cases)
    result=dict(schema='fsim.store-reference-gain.v1',model=model,
        original_cases_sha256=case_digest(record['cases']),reference_inputs_sha256=C.reference_hash(model,catalog),
        initial_parameters=declared['initial_parameters'],parameters=parameters,
        source_sha256=C.sha(source),calibration_sha256=C.sha(source.with_name('calibration.toml')),
        xml_sha256=C.sha(runner.product),native_library=native_library(),
        measurement_path=('Scaled body-store fixtures use the normal table/XML writer, retained production calibration and source Reynolds; full-scale publication is hashed and verified separately. These are scaled-model observations, not full-scale performance validation.' if scaled else
                          'Current forces use the published production aerodynamic XML in fresh SDK worlds; baseline uses the recorded old parameters, same controls and original reference conditions.'),
        scaled_tunnel_validation=scaled,
        observations=[dict(c,baseline_sdk_value=b,sdk_value=n,baseline_actual=ba,actual=a,source_record=catalog['sources'][c['source']])
                      for c,b,n,ba,a in zip(cases,baseline,current,baseline_actual,runner.last)])
    for label,subset,start in [('training',train,0),('validation',held,len(train))]:
        for prefix,values in [('baseline_',baseline),('',current)]:
            result[prefix+label]={metric:C.errors([v for c,v in zip(subset,values[start:]) if c['metric']==metric],
                [c['value'] for c in subset if c['metric']==metric]) for metric in sorted({c['metric'] for c in subset})}
    result['relative_reduction_percent']={metric:100*(1.-result['validation'][metric]['nrmse_percent']/old['nrmse_percent'])
        for metric,old in result['baseline_validation'].items() if old['nrmse_percent']}
    result['retention_threshold_percent']=2.
    result['kpi_threshold_percent']=10.
    decisions=[C.fidelity_gain(old['nrmse_percent'],result['validation'][metric]['nrmse_percent'])
               for metric,old in result['baseline_validation'].items()]
    result['gain_accepted']=any(row['retained'] for row in decisions)
    result['kpi_met']=any(row['kpi_met'] for row in decisions)
    card=storestages.card_of(runner.design);aero_evidence.verify_binding(runner.design,card)
    result['table_sha256']=C.sha(runner.product.parent/card['sources']['hangar-vlm']['url'])
    atomic_write_text(out,json.dumps(result,indent=2,allow_nan=False)+'\n')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',required=True)
    parser.add_argument('--evidence',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    import fsim
    fsim.set_log_level('off')
    result=measure_gain(args.model,args.evidence,args.out)
    print(json.dumps({k:result[k] for k in ('model','baseline_validation','validation','relative_reduction_percent','gain_accepted','kpi_met','table_sha256','xml_sha256')}))
    return 0 if result['gain_accepted'] else 1


if __name__=='__main__':raise SystemExit(main())
