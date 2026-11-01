"""SDK measurements, sourced calibration and untouched holdout evaluation.

The catalog is data, not equipment-specific solver code. Unsupported or missing
observations stay unknown. A generated calibration file is never evidence.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

from . import pipeline
from .files import atomic_write_text

CATALOG = Path(__file__).with_name('public_data.json')
METRICS = {'CD','CD0','CL','CY','CX','CZ','Cl','Cm','Cn','drag_area_m2','climb_rate_fpm','static_thrust_n'}
METRIC_UNITS={key:'1' for key in METRICS}
METRIC_UNITS.update(drag_area_m2='m2',climb_rate_fpm='ft/min',static_thrust_n='N')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def reference_hash(name,catalog):
    record=catalog['models'][name]
    used={c['source'] for c in record.get('cases',[])}
    return hashlib.sha256(json.dumps(dict(model=record,sources={k:catalog['sources'][k] for k in sorted(used)}),
                                     sort_keys=True,allow_nan=False).encode()).hexdigest()


def validate_record(record, sources):
    if not record.get('searches'):
        raise ValueError('Every model needs a recorded reference search')
    identities = {}
    ids = set()
    for case in record.get('cases', []):
        if case['id'] in ids:
            raise ValueError('Duplicate reference observation ID')
        ids.add(case['id'])
        if case['source'] not in sources or not sources[case['source']].get('url','').startswith('https://'):
            raise ValueError('Missing traceable public reference')
        if not case.get('locator') or not case.get('configuration') or not case.get('campaign'):
            raise ValueError('Missing page/table, configuration or campaign')
        if case['metric'] not in METRICS or not math.isfinite(case['value']):
            raise ValueError('Unsupported or nonfinite observation')
        if case['split'] not in ('calibration','validation','context'):
            raise ValueError('Invalid reference partition')
        needed = ('altitude_m','mass_kg','temperature_c','airspeed_kcas') if case['metric']=='climb_rate_fpm' else ('alpha_deg','beta_deg','mach','altitude_m')
        if case['metric']=='static_thrust_n':needed=needed+('rotor_rpm',)
        if any(k not in case['conditions'] for k in needed):
            raise ValueError('Reference operating conditions are incomplete')
        if not all(isinstance(v,(int,float)) and math.isfinite(v) for v in case['conditions'].values()):
            raise ValueError('Nonfinite reference condition')
        identity = (case['source'],case['campaign'],case['metric'],json.dumps(case['conditions'],sort_keys=True))
        if identity in identities and identities[identity]!=case['split']:
            raise ValueError('Calibration/validation observation leakage')
        identities[identity]=case['split']
    for name, limits in record.get('parameters',{}).items():
        if name not in ('extra_drag_area_m2','base_drag_scale','wave_drag_efficiency','korn_kappa','wave_drag_rise_exponent') and not name.startswith('design:'):
            raise ValueError('Unsupported SDK calibration binding: '+name)
        if len(limits)!=2 or not all(math.isfinite(v) for v in limits) or limits[0]>=limits[1]:
            raise ValueError('Invalid calibration parameter bounds: '+name)
        if name=='base_drag_scale' and limits[0]<=0:
            raise ValueError('Base drag scale bounds must be positive')
        physical={'wave_drag_efficiency':(1.2,6.),'korn_kappa':(.87,.95),'wave_drag_rise_exponent':(1.,4.)}
        if name in physical and (limits[0]<physical[name][0] or limits[1]>physical[name][1]):
            raise ValueError('Bounds exceed the existing method domain: '+name)


def errors(predicted, observed, uncertainties=None):
    if not observed:
        return dict(count=0,nrmse_percent=None,max_absolute_error=None)
    p,o=np.asarray(predicted,float),np.asarray(observed,float)
    if p.shape!=o.shape or not np.isfinite(p).all() or not np.isfinite(o).all():
        raise ValueError('Incomplete/nonfinite SDK observation')
    denominator=float(np.linalg.norm(o))
    result=dict(count=len(o),nrmse_percent=100*float(np.linalg.norm(p-o))/denominator if denominator else None,
                max_absolute_error=float(np.max(np.abs(p-o))))
    if uncertainties and all(u is not None for u in uncertainties):
        u=np.asarray(uncertainties,float)
        low_denom=float(np.linalg.norm(np.maximum(np.abs(o)-u,0.)))
        result['reference_reading_uncertainty_nrmse_interval_percent']=[
            100*float(np.linalg.norm(np.maximum(np.abs(p-o)-u,0.)))/float(np.linalg.norm(np.abs(o)+u)),
            100*float(np.linalg.norm(np.abs(p-o)+u))/low_denom if low_denom else None]
    return result


def fidelity_gain(baseline_percent, new_percent, retention_percent=2.):
    """Retention and the 10-percent fidelity KPI are separate decisions."""
    valid=all(isinstance(v,(int,float)) and math.isfinite(v) for v in (baseline_percent,new_percent))
    relative=100.*(1.-new_percent/baseline_percent) if valid and baseline_percent>0. else None
    return dict(relative_reduction_percent=relative,
                retained=relative is not None and new_percent<baseline_percent and
                         new_percent<=baseline_percent*(1.-retention_percent/100.),
                kpi_met=relative is not None and new_percent<=baseline_percent*.9)


def fit_parameters(measure, cases, initial, bounds, iterations=5):
    """Bounded finite-difference Gauss-Newton; only calibration cases enter here.

    Cached VLM tables are reused. Each objective/sensitivity comes from an SDK
    instance; holdouts never enter the optimizer or parameter selection.
    """
    from scipy.optimize import lsq_linear
    keys=list(bounds)
    x=np.array([np.clip(initial.get(k,0.),*bounds[k]) for k in keys],float)
    widths=np.array([bounds[k][1]-bounds[k][0] for k in keys])
    target=np.array([c['value'] for c in cases])
    scale=np.maximum(np.abs(target),max(float(np.max(np.abs(target)))*.05,1.e-12))
    history=[]
    def evaluate(candidate):
        parameters=dict(initial,**dict(zip(keys,map(float,candidate))))
        y=np.array(measure(parameters,cases),float)
        if not np.isfinite(y).all():raise ValueError('SDK measurement did not settle')
        residual=(y-target)/scale
        history.append(dict(parameters=parameters,objective=float(residual@residual),values=y.tolist()))
        return y,residual
    y,residual=evaluate(x)
    for _ in range(iterations):
        jac=[]
        for j,k in enumerate(keys):
            h=widths[j]*.01
            trial=x.copy();trial[j]=min(bounds[k][1],x[j]+h)
            if trial[j]==x[j]:trial[j]=max(bounds[k][0],x[j]-h)
            yp,_=evaluate(trial);jac.append((yp-y)/(((trial[j]-x[j])/widths[j])*scale))
        # Solve with the bounds, rather than clipping an unconstrained step.
        # Otherwise one active limit leaves coupled free parameters at the
        # wrong optimum and can prematurely stop the SDK fit. SciPy's BVLS
        # active set refits those parameters inside the same linearization.
        # https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.lsq_linear.html
        lower=np.array([bounds[k][0] for k in keys]);upper=lower+widths
        step=lsq_linear(np.asarray(jac).T,-residual,
            bounds=((lower-x)/widths,(upper-x)/widths),method='bvls',tol=1.e-10).x
        trial=np.array([np.clip(v+dv*width,*bounds[k]) for k,v,dv,width in zip(keys,x,step,widths)])
        if np.linalg.norm((trial-x)/widths)<1.e-8:break
        improved=False
        for fraction in (1.,.5,.25):
            candidate=x+fraction*(trial-x);yp,rp=evaluate(candidate)
            if rp@rp < residual@residual:
                x,y,residual=candidate,yp,rp;improved=True;break
        if not improved:break
    return dict(parameters=dict(initial,**dict(zip(keys,map(float,x)))),history=history)


def roster():
    directory=Path(pipeline.repo_root())/'aircraft'
    return {p.name:p/(p.name+'.toml') for p in sorted(directory.iterdir())
            if p.name.casefold()!='.codex' and (p/(p.name+'.toml')).is_file()}


def load_catalog(path=CATALOG):
    catalog=json.loads(Path(path).read_text(encoding='utf-8'))
    if catalog.get('schema')!='fsim.public-calibration-data.v1':raise ValueError('Unsupported reference catalog')
    if set(catalog['models'])!=set(roster()):raise ValueError('Reference catalog must account for every authored model')
    from .aero_evidence import validate_audit
    if set(catalog.get('model_audits',{}))!=set(catalog['models']):
        raise ValueError('Evidence audit must account for every authored model')
    for audit in catalog['model_audits'].values():validate_audit(audit)
    for record in catalog['models'].values():validate_record(record,catalog['sources'])
    return catalog


def persist_parameters(source, parameters, record, catalog_hash,reference_inputs_hash):
    """One per-design physical calibration file, rebuilt by the normal writer."""
    import tomllib
    from . import airframe, storestages, rotorcraft
    design_values={k.removeprefix('design:'):v for k,v in parameters.items() if k.startswith('design:')}
    if design_values:
        import re
        text=source.read_text(encoding='utf-8')
        for path,value in design_values.items():
            section,key=path.rsplit('.',1)
            pattern=r'(?ms)(^\['+re.escape(section)+r'\][^\n]*\n)(.*?)(?=^\[|\Z)'
            match=re.search(pattern,text)
            if not match:raise ValueError('Physical parameter section does not exist: '+section)
            body,count=re.subn(r'(?m)^(\s*'+re.escape(key)+r'\s*=\s*)[^\n]+',lambda m:m[1]+format(value,'.17g')+' # SDK reference fit; see calibration.toml',match[2])
            if count!=1:raise ValueError('Physical parameter is not uniquely authored: '+path)
            text=text[:match.start(2)]+body+text[match.end(2):]
        atomic_write_text(source,text)
    destination=source.parent/'calibration.toml'
    original=tomllib.loads(destination.read_text(encoding='utf-8'))
    # The old table-only DI fitter must not overwrite an SDK-derived result.
    reference=original.get('reference')
    parts=['# Fitted from declared calibration observations through the SDK.','[calibration]']
    parts += [key+' = '+json.dumps(value) for key,value in sorted(parameters.items()) if not key.startswith('design:')]
    parts += ['','[provenance]','schema = "fsim.hangar-calibration.v1"','method = "shared-aircraft-vlm"',
              'status = "sdk-reference-fit"','observed_source_sha256 = '+json.dumps(sha(source)),
              'catalog_sha256 = '+json.dumps(catalog_hash),
              'reference_inputs_sha256 = '+json.dumps(reference_inputs_hash),
              'fitted_parameter_names = '+json.dumps(sorted(record.get('parameters',{}))),
              'simulated_method_input_names = '+json.dumps(sorted(set(record.get('fixed_method_inputs',{})) & set(parameters))),
              'calibration_case_ids = '+json.dumps([c['id'] for c in record['cases'] if c['split']=='calibration']),
              'scope = "Declared observations only; independent errors are in the campaign report"']
    if reference:parts += ['','[historical_configuration_reference]']+[k+' = '+json.dumps(v) for k,v in reference.items()]
    if design_values:parts+=['','[physical_fit]']+[json.dumps(k)+' = '+json.dumps(v) for k,v in design_values.items()]
    atomic_write_text(destination,'\n'.join(parts)+'\n')
    if rotorcraft.kind_of(source):rotorcraft.run(str(source),['build'],log=lambda s:None)
    else:
        d=airframe.design(source)
        (storestages.build(d) if storestages.is_store(source) else d.build())


def stage_runtime_products(source):
    """Publish into the data tree selected by the actual loaded SDK module."""
    from . import runtime_products
    from .sdk_measurements import native_library
    library=Path(native_library()['path']).resolve()
    return runtime_products.stage_runtime_products(source,library.parent.parent/'share/flightsim')


def execute_model(name, catalog, scratch, apply=True):
    import tomllib
    import fsim  # Load the actual SDK before publishing to its project data tree.
    from .sdk_measurements import Measurements, native_library
    source=roster()[name];record=catalog['models'][name];started=time.monotonic()
    calibration=source.parent/'calibration.toml'
    calibration_record=tomllib.loads(calibration.read_text(encoding='utf-8'))
    initial=calibration_record.get('calibration',{})
    provenance=calibration_record.get('provenance',{})
    existing_fit=(provenance.get('status')=='sdk-reference-fit' and
                  provenance.get('observed_source_sha256')==sha(source) and
                  provenance.get('reference_inputs_sha256')==reference_hash(name,catalog))
    from .aero.mach import E_WD, KORN_KAPPA
    defaults={'extra_drag_area_m2':0.,'base_drag_scale':1.,'wave_drag_efficiency':E_WD,'korn_kappa':KORN_KAPPA,'wave_drag_rise_exponent':1.}
    for key in record.get('parameters',{}):
        if key.startswith('design:'):
            value=tomllib.loads(source.read_text(encoding='utf-8'))
            for part in key.removeprefix('design:').split('.'):value=value[part]
            initial[key]=float(value)
        else:initial.setdefault(key,defaults[key])
    runner=Measurements(name,scratch)
    train=[c for c in record.get('cases',[]) if c['split']=='calibration']
    validation=[c for c in record.get('cases',[]) if c['split']=='validation']
    all_cases=train+validation
    result=dict(model=name,status='unknown',unknown_reason=record.get('unknown_reason',''),
                kind=runner.kind,source_sha256=sha(source),sdk_executed=False,
                initial_parameters=initial,parameters=initial,training=None,validation=None,
                matched_equipment_error_percent=None,observations=[],searches=record['searches'],
                contextual_sources=record.get('contextual_sources',[]),
                repository_reference_review=record.get('repository_reference_review'))
    result['evidence_audit']=catalog.get('model_audits',{}).get(name)
    result['runtime_publication']=stage_runtime_products(source)
    result['calibration_executed']=bool(apply and train and record.get('parameters'))
    result['configuration_updated']=False
    result['existing_calibration_matches_references']=existing_fit
    # The baseline uses all observations, but is not passed to the optimizer.
    # No holdout enters parameter fitting, convergence or bounds. The frozen
    # candidate is independently assessed before any production write; a
    # candidate with no holdout gain is rejected, as required by the workflow.
    baseline=runner.measure(initial,all_cases) if all_cases else []
    frozen=initial
    if apply and train and record.get('parameters'):
        fitted=fit_parameters(runner.measure,train,initial,record['parameters'])
        frozen=fitted['parameters'];result['fit_history']=fitted['history']
        changed=frozen!=initial
        result['candidate_rejected']=False
        if changed:
            candidate=runner.measure(frozen,validation) if validation else []
            metrics=sorted({c['metric'] for c in train}&{c['metric'] for c in validation})
            old=baseline[len(train):]
            result['candidate_validation']={metric:errors(
                [v for c,v in zip(validation,candidate) if c['metric']==metric],
                [c['value'] for c in validation if c['metric']==metric]) for metric in metrics}
            old_errors={metric:errors([v for c,v in zip(validation,old) if c['metric']==metric],
                [c['value'] for c in validation if c['metric']==metric]) for metric in metrics}
            improved=[metric for metric in metrics if
                result['candidate_validation'][metric]['nrmse_percent'] is not None and
                old_errors[metric]['nrmse_percent'] is not None and
                fidelity_gain(old_errors[metric]['nrmse_percent'],result['candidate_validation'][metric]['nrmse_percent'],
                              2. if runner.kind=='store' else 0.)['retained']]
            result['candidate_parameters']=dict(frozen)
            result['improved_quantities']=improved
            if not improved:
                result['candidate_rejected']=True
                result['candidate_rejection_reason']='No original independent holdout quantity improved; no production inputs changed'
                frozen=initial;changed=False
        result['configuration_updated']=changed
        result['status']='calibrated-unvalidated'
        # Physical calibration also invalidates derived runtime profiles.
        # Regenerate them in the same operation, then publish normal products.
        from . import rotorcraft,storestages,flightcal
        if not changed:
            # Source/reference identity and every physical parameter are
            # unchanged. Retest all external observations in fresh SDK worlds,
            # retaining valid runtime profiles instead of rewriting provenance
            # and making those very profiles stale on a no-op fit.
            stages=flightcal.read(source.parent) if not storestages.is_store(source) else {}
            result['runtime_check_provenance']='No retained configuration change; valid source-bound profiles retained; external observations measured afresh'
        else:
            persist_parameters(source,frozen,record,sha(CATALOG),reference_hash(name,catalog))
            result['runtime_publication']=stage_runtime_products(source)
        if changed and storestages.is_store(source):
            _,stages=pipeline.run(str(source),['verify','fly'],log=print)
        elif changed and rotorcraft.kind_of(source):
            _,stages=rotorcraft.run(str(source),['fly','performance'],log=print)
            flightcal.export(source.parent)
            rotorcraft.run(str(source),['build'],log=lambda s:None)
        elif changed:
            _,stages=pipeline.run(str(source),['fly','ground','performance'],log=print)
            flightcal.export(source.parent)
            pipeline.run(str(source),['build'],log=lambda s:None)
        result['runtime_publication']=stage_runtime_products(source)
        result['post_fit_runtime_checks']={stage:[c for c in data.get('checks',[]) if c['status'] in ('warn','fail')]
                                          for stage,data in stages.items() if isinstance(data,dict)}
    elif train:
        result['status']='measured-unfitted'
        from . import flightcal,storestages
        if storestages.is_store(source):
            # A no-fit retest still executes the real SDK checks, but its
            # timestamped reports belong to this campaign's scratch evidence.
            # Canonical source/products/reports remain read-only.
            design=pipeline.Design(str(source),log=print)
            runtime=Path(scratch)/'runtime-checks'/name
            runtime.mkdir(parents=True,exist_ok=True);design.out=str(runtime)
            stages={stage:storestages.run(design,stage) for stage in ('verify','fly')}
            result['runtime_check_directory']=str(runtime)
            result['runtime_check_provenance']='Fresh SDK load, carriage and release'
        else:
            stages=flightcal.read(source.parent)
            result['runtime_check_provenance']='Source-bound retained flight/performance receipts; external observations are measured afresh'
        result['post_fit_runtime_checks']={stage:[c for c in data.get('checks',[]) if c['status'] in ('warn','fail')]
                                          for stage,data in stages.items() if isinstance(data,dict)}
    if all_cases:
        values=runner.measure(frozen,all_cases)
        result['observations']=[]
        for i,(case,predicted) in enumerate(zip(all_cases,values)):
            result['observations'].append(dict(case,baseline_sdk_value=baseline[i],sdk_value=predicted,
                signed_error=predicted-case['value'],relative_error_percent=100*(predicted-case['value'])/abs(case['value']) if case['value'] else None,
                actual=runner.last[i],source_record=catalog['sources'][case['source']]))
        # Never mix dimensions or metric scales into one headline error.
        for split,cases,offset in (('training',train,0),('validation',validation,len(train))):
            result['baseline_'+split]={metric:errors([v for c,v in zip(cases,baseline[offset:]) if c['metric']==metric],
                [c['value'] for c in cases if c['metric']==metric]) for metric in sorted({c['metric'] for c in cases})}
            result[split]={metric:errors([v for c,v in zip(cases,values[offset:]) if c['metric']==metric],
                                        [c['value'] for c in cases if c['metric']==metric],
                                        [c.get('reference_uncertainty',c.get('digitization_uncertainty_cd')) for c in cases if c['metric']==metric]) for metric in sorted({c['metric'] for c in cases})}
        if validation:
            result['status']='calibrated-and-retested' if result['calibration_executed'] else 'retested-existing-calibration' if existing_fit else 'measured-unfitted'
            result['validation_independence']=record.get('validation_independence','Disjoint observations; common source may share systematic errors')
    if any(c['status']=='fail' for checks in result.get('post_fit_runtime_checks',{}).values() for c in checks):
        result['status']='runtime-check-failed'
    if not train and runner.kind!='store':
        from . import flightcal
        retained=flightcal.read(source.parent)
        result['retained_model_self_checks']={stage:[c for c in data.get('checks',[]) if c['status'] in ('warn','fail')]
            for stage,data in retained.items() if isinstance(data,dict)}
    # Unknown models are still loaded and measured through the same SDK path.
    result['runtime_probe']=runner.probe(frozen)
    result['source_before_sha256']=result['source_sha256']
    if result.get('evidence_audit'):
        from copy import deepcopy
        from .aero_evidence import parameter_audit
        result['evidence_audit']=deepcopy(result['evidence_audit'])
        current_calibration=tomllib.loads(calibration.read_text(encoding='utf-8'))
        result['evidence_audit']['parameter_audit']=parameter_audit(current_calibration,
            fitted_parameters=record.get('parameters',{}) if existing_fit or result['calibration_executed'] else ())
    result.update(sdk_executed=True,parameters=frozen,native_library=native_library(),source_sha256=sha(source),
                  reference_inputs_sha256=reference_hash(name,catalog),
                  calibration_sha256=sha(calibration),xml_sha256=sha(runner.product),elapsed_seconds=time.monotonic()-started)
    from .aero_evidence import closure_status,retain_fit_evidence
    result['execution_status']={'unknown':'sdk-probed-no-reference',
        'calibrated-and-retested':'fit-retested','retested-existing-calibration':'reference-retested',
        'calibrated-unvalidated':'fit-without-holdout','measured-unfitted':'reference-measured-without-fit'}.get(result['status'],result['status'])
    if result['configuration_updated'] and result['execution_status']!='runtime-check-failed':
        proof=retain_fit_evidence(result,CATALOG)
        if proof:
            result['evidence_audit']['retained_calibration_change']=proof
            result['evidence_audit']['status']='calibrated'
            result['evidence_audit']['unknown_reason']=''
    result['status']=closure_status(result)
    result['round_outcome']=('Retained source-bound holdout improvement in the stated quantities only' if result['status']=='calibrated' and result['configuration_updated']
        else 'No new fidelity gain in this round; the earlier source-bound improvement remains retained' if result['status']=='calibrated'
        else 'Blocked: no admissible independently validated equipment calibration' if result['status']=='blocked'
        else 'Historical fit or configuration override retained; no retained fidelity gain in this round')
    return result


def publish(report, out):
    atomic_write_text(out,json.dumps(report,indent=2,allow_nan=False)+'\n')
    csv_path=Path(out).with_suffix('.csv')
    fields=['model','status','sdk_executed','metric','metric_unit','calibration_points','calibration_nrmse_percent',
            'calibration_max_absolute_error','validation_points','validation_nrmse_percent','validation_max_absolute_error',
            'matched_equipment_error_percent','validation_scope','unknown_reason','sources',
            'parameter_verification','historical_parameters','search_date','search_queries']
    with csv_path.open('w',encoding='utf-8',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        for row in report['models']:
            audit=row.get('evidence_audit') or {}
            metrics=sorted(set(row.get('training') or {})|set(row.get('validation') or {})) or ['']
            for metric in metrics:
                tr=(row.get('training') or {}).get(metric,{})
                va=(row.get('validation') or {}).get(metric,{})
                writer.writerow(dict(model=row['model'],status=row['status'],sdk_executed=row.get('sdk_executed',False),metric=metric,
                    metric_unit=METRIC_UNITS.get(metric,''),
                    calibration_points=tr.get('count',0),calibration_nrmse_percent=tr.get('nrmse_percent'),
                    calibration_max_absolute_error=tr.get('max_absolute_error'),
                    validation_points=va.get('count',0),validation_nrmse_percent=va.get('nrmse_percent'),
                    validation_max_absolute_error=va.get('max_absolute_error'),
                    matched_equipment_error_percent=row.get('matched_equipment_error_percent'),
                    validation_scope=row.get('validation_independence',''),unknown_reason=row.get('unknown_reason',''),
                    parameter_verification=audit.get('parameter_audit',{}).get('verification_status',''),
                    historical_parameters=json.dumps(audit.get('parameter_audit',{}).get('parameters',{}),sort_keys=True),
                    search_date=audit.get('search_date',''),search_queries='; '.join(audit.get('searches',[])),
                    sources='; '.join(sorted({o['source_record']['url'] for o in row.get('observations',[])}|
                        {report['sources'][key]['url'] for key in row.get('contextual_sources',[]) if key in report.get('sources',{})}))))
    # One row per model; the metric CSV above deliberately has several rows
    # when a model has different measured quantities.
    ledger=Path(out).with_name(Path(out).stem+'-status.csv')
    with ledger.open('w',encoding='utf-8',newline='') as stream:
        fields=['model','status','kind','matched_quantity','baseline_nrmse_percent','new_nrmse_percent',
                'parameter_diff','retained_baseline_nrmse_percent','retained_parameter_diff','sources','gap','round_outcome']
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        for row in report['models']:
            audit=row.get('evidence_audit') or {};proof=audit.get('retained_calibration_change') or {}
            quantities=sorted(row.get('validation') or {})
            old=row.get('baseline_validation')
            new=row.get('validation') or {}
            before=row.get('initial_parameters') or {};after=row.get('parameters') or {}
            difference={key:dict(before=before.get(key),after=after.get(key)) for key in sorted(set(before)|set(after))
                        if before.get(key)!=after.get(key)}
            writer.writerow(dict(model=row['model'],status=row['status'],kind=row.get('kind',''),
                matched_quantity='; '.join(quantities) or 'No matched quantity established',
                baseline_nrmse_percent=json.dumps({k:v.get('nrmse_percent') for k,v in (old or {}).items()}),
                new_nrmse_percent=json.dumps({k:v.get('nrmse_percent') for k,v in new.items()}),
                parameter_diff=json.dumps(difference,sort_keys=True),
                retained_baseline_nrmse_percent=json.dumps({k:v.get('nrmse_percent') for k,v in proof.get('baseline_validation',{}).items()}),
                retained_parameter_diff=json.dumps(proof.get('parameter_diff',{}),sort_keys=True),
                sources='; '.join(sorted({r['url'] for r in audit.get('source_reviews',[])})),
                gap=row.get('unknown_reason') or audit.get('unknown_reason',''),round_outcome=row.get('round_outcome','')))


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',action='append',default=[])
    parser.add_argument('--out',type=Path,default=Path(pipeline.repo_root())/'build/hangar-calibration/results.json')
    parser.add_argument('--no-fit',action='store_true',help='Repeat SDK measurements without changing parameters')
    parser.add_argument('--require-all',action='store_true',help='Return an incomplete status if any model lacks calibration or holdouts')
    parser.add_argument('--worker-timeout-seconds',type=float,default=3600.,
                        help='Per-model execution limit including regenerated flight/performance profiles (default: 3600)')
    parser.add_argument('--worker',action='store_true',help=argparse.SUPPRESS)
    args=parser.parse_args(argv);catalog=load_catalog()
    if not math.isfinite(args.worker_timeout_seconds) or args.worker_timeout_seconds<=0:
        parser.error('--worker-timeout-seconds must be finite and positive')
    if not args.model or args.model==['all']:
        from .storestages import is_store
        paths=roster()
        def priority(name):
            if is_store(paths[name]):return (0,name)
            import tomllib
            kind=tomllib.loads(paths[name].read_text(encoding='utf-8'))['aircraft'].get('kind')
            return (2 if kind in ('helicopter','multirotor') else 1,name)
        names=sorted(paths,key=priority)
    else:
        names=args.model
    if any(n not in roster() for n in names):raise ValueError('Unknown authored model')
    args.out.parent.mkdir(parents=True,exist_ok=True)
    if args.worker:
        import fsim
        fsim.set_log_level('off')
        row=execute_model(names[0],catalog,args.out.parent/'runtime'/names[0],not args.no_fit)
        atomic_write_text(args.out,json.dumps(row,indent=2,allow_nan=False)+'\n')
        return 0
    started=time.monotonic()
    report=dict(schema='fsim.sdk-reference-campaign.v1',catalog_sha256=sha(CATALOG),models=[],complete=False,
                sources=catalog['sources'],
                campaign_code_sha256=sha(__file__),
                measurement_code_sha256=sha(Path(__file__).with_name('sdk_measurements.py')),sdk_only=True,
                scope='Public-data calibration and fresh SDK holdouts; not full-envelope physical certification')
    for name in names:
        child=args.out.parent/'models'/(name+'.json');child.parent.mkdir(parents=True,exist_ok=True)
        command=[sys.executable,'-u','-B','-m','hangar.reference_campaign','--worker','--model',name,'--out',str(child)]
        if args.no_fit:command.append('--no-fit')
        try:
            with child.with_suffix('.log').open('w',encoding='utf-8') as log:
                completed=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,timeout=args.worker_timeout_seconds)
            if completed.returncode:raise RuntimeError('SDK worker exit '+str(completed.returncode)+'; see '+str(child.with_suffix('.log')))
            row=json.loads(child.read_text(encoding='utf-8'))
        except Exception as exc:
            row=dict(model=name,status='blocked',execution_status='failed',sdk_executed=False,unknown_reason=str(exc))
        report['models'].append(row);report['elapsed_seconds']=time.monotonic()-started;publish(report,args.out)
        print(name+': '+row['status'],flush=True)
    failed=sum(r.get('execution_status') in ('failed','runtime-check-failed') for r in report['models'])
    missing=[r['model'] for r in report['models'] if r['status']!='calibrated']
    covers_roster=set(names)==set(roster())
    report.update(complete=not missing and covers_roster,selected_models_complete=not missing,
                  output_catalog_sha256=sha(CATALOG),
                  covers_complete_roster=covers_roster,roster_total=len(roster()),
                  models_with_retained_self_check_failures=[r['model'] for r in report['models']
                    if any(c['status']=='fail' for checks in r.get('retained_model_self_checks',{}).values() for c in checks)],
                  models_without_complete_calibration=missing,failed_models=failed)
    publish(report,args.out)
    if covers_roster:
        from .reference_report import publish_public
        publish_public(report)
    print(json.dumps(dict(models=len(report['models']),failed=failed,complete=report['complete'],
                         calibrated=sum(r['status']=='calibrated' for r in report['models']),
                         historical=sum(r['status']=='historical' for r in report['models']),
                         blocked=sum(r['status']=='blocked' for r in report['models']),
                         calibration_executed=sum(bool(r.get('calibration_executed')) for r in report['models']),
                         models_without_complete_calibration=len(missing))))
    return 1 if failed else 2 if args.require_all and missing else 0


if __name__=='__main__':raise SystemExit(main())
