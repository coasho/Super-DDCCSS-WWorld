"""Trace simulated products to physical inputs; keep history unverified."""
import hashlib,json,math
from pathlib import Path

from . import __version__
from .files import atomic_write_text

CARD_INPUTS=('length_m','diameter_m','span_m','mass_kg','fuel_kg','width_m','height_m','analysis_speed_ms')

def retained_change(row):
    """Keep the measured baseline and parameter delta, not optimizer activity."""
    before=row.get('initial_parameters',{});after=row.get('parameters',{})
    difference={key:dict(before=before.get(key),after=after.get(key)) for key in sorted(set(before)|set(after))
                if before.get(key)!=after.get(key)}
    baseline=row.get('baseline_validation') or {};current=row.get('validation') or {}
    improved=[metric for metric in (row.get('training') or {}) if metric in baseline and metric in current and
        baseline[metric].get('nrmse_percent') is not None and current[metric].get('nrmse_percent') is not None and
        current[metric]['nrmse_percent']<baseline[metric]['nrmse_percent']]
    if not difference or not improved:return None
    from datetime import datetime,timezone
    return dict(schema='fsim.retained-calibration-change.v1',recorded_utc=datetime.now(timezone.utc).isoformat(),
        reference_inputs_sha256=row['reference_inputs_sha256'],
        initial_parameters=before,parameters=after,parameter_diff=difference,target_metrics=improved,
        baseline_validation=baseline,validation=current,native_library=row.get('native_library'),
        source_sha256=row.get('source_sha256'),calibration_sha256=row.get('calibration_sha256'),
        xml_sha256=row.get('xml_sha256'),observations=row.get('observations',[]),
        scope='Only the listed target quantities improved on the unchanged source holdouts; other axes and full-envelope accuracy are not implied.')

def retain_fit_evidence(row,catalog_path):
    proof=retained_change(row)
    if proof is None:return None
    catalog_path=Path(catalog_path);catalog=json.loads(catalog_path.read_text(encoding='utf-8'))
    from .reference_campaign import reference_hash
    if reference_hash(row['model'],catalog)!=proof['reference_inputs_sha256']:
        raise ValueError('Retained gain references differ from the current source partition')
    audit=catalog['model_audits'][row['model']]
    audit['retained_calibration_change']=proof
    audit['status']='calibrated'
    atomic_write_text(catalog_path,json.dumps(catalog,indent=2,ensure_ascii=False)+'\n')
    return proof

def closure_status(row):
    """A retained real-data gain, a historical fit, or an explicit data block.

    Running an optimizer or an SDK probe never establishes an improvement.
    Retained gain evidence must still match this configuration and reference
    partition, and the current independent holdout must beat its baseline.
    """
    if row.get('execution_status') in ('failed','runtime-check-failed'):return 'blocked'
    audit=row.get('evidence_audit') or {}
    if audit.get('status')=='blocked':return 'blocked'
    proof=audit.get('retained_calibration_change') or {}
    if (proof.get('reference_inputs_sha256')==row.get('reference_inputs_sha256') and
        proof.get('parameters')==row.get('parameters') and
        proof.get('initial_parameters')!=proof.get('parameters') and
        proof.get('target_metrics')):
        def improved(metric):
            before=proof.get('baseline_validation',{}).get(metric,{}).get('nrmse_percent')
            after=(row.get('validation') or {}).get(metric,{}).get('nrmse_percent')
            return (isinstance(before,(int,float)) and isinstance(after,(int,float)) and
                    math.isfinite(before) and math.isfinite(after) and after<before)
        if all(improved(metric) for metric in proof['target_metrics']):return 'calibrated'
    if audit.get('parameter_audit',{}).get('configuration_status')=='unfitted':return 'blocked'
    return 'historical'

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def card_inputs(card):
    return {key:card['parameters'][key]['value'] for key in CARD_INPUTS if key in card.get('parameters',{})}

def validate_card_geometry(d,card):
    """Use the existing missile authoring invariants before stamping provenance.

    Rebinding a new card to an unchanged old source is not regeneration. The
    dimensional rounding and span tolerances are the original card-figure
    checks in tests.test_missiles, not new aerodynamic acceptance thresholds.
    """
    from . import missiles,pipeline
    if d.name not in missiles.MISSILES:return {}
    a=d.aircraft;body=next(b for b in a.bodies if b.name=='body')
    values=dict(length_m=float(body.x[-1])-min(float(b.x[0]) for b in a.bodies),
        diameter_m=2.*float(max(body.top)),mass_kg=a.spec['mass']['empty'])
    if 'span_m' in card.get('parameters',{}):values['span_m']=pipeline.missile_span(a)
    for key,value in values.items():
        target=card['parameters'][key]['value']
        tolerance=.005*target if key=='span_m' else 0. if key=='mass_kg' else .0005
        if abs(value-target)>tolerance:
            raise ValueError('Source geometry differs from current card '+key+' for '+d.name)
    return values

def validate_binding(binding,*,card_inputs,design_sha256,table_inputs_hash,table_sha256):
    if not binding or binding.get('basis')!='simulated':raise ValueError('Missing input-bound simulated provenance')
    for key,value,label in [('card_inputs',card_inputs,'card geometry'),('design_sha256',design_sha256,'design'),
                            ('table_inputs_hash',table_inputs_hash,'aerodynamic inputs'),('table_sha256',table_sha256,'table bytes')]:
        if binding.get(key)!=value:raise ValueError('Stale VLM '+label+' binding')

def parameter_audit(calibration, fitted_parameters=None):
    status=calibration.get('provenance',{}).get('status','unfitted')
    history=status in ('configuration-reference-fit','existing-performance-fit')
    fitted=set(calibration.get('provenance',{}).get('fitted_parameter_names',()) if fitted_parameters is None else fitted_parameters)
    simulated=set(calibration.get('provenance',{}).get('simulated_method_input_names',())) if status=='sdk-reference-fit' else set()
    values=dict(calibration.get('calibration',{}))
    values.update({'design:'+key:value for key,value in calibration.get('physical_fit',{}).items()})
    unverified=sorted(key for key in values if history or status!='sdk-reference-fit' or key not in fitted|simulated)
    return dict(configuration_status=status,verification_status='unverified' if history else
                'scoped-sdk-reference-fit' if status=='sdk-reference-fit' else 'not-fitted',
                unverified_parameters=unverified,
                parameters={key:dict(value=value,basis='simulated_method_input' if key in simulated else 'historical_override' if history else
                    ('sdk_reference_fit' if key in fitted else 'historical_override') if status=='sdk-reference-fit' else 'assumed',
                    matched_error_percent=None) for key,value in values.items()},
                historical_reference=calibration.get('reference',{}),
                scope='A historical override is not a published measurement or a completed calibration.')

def validate_audit(audit):
    reviews=audit.get('source_reviews',[])
    if not reviews or any(not all(r.get(k) for k in ('url','locator','conditions','decision')) for r in reviews):
        raise ValueError('Every model needs an explicit source review')
    if audit.get('status') in ('unknown','blocked'):
        if not audit.get('unknown_reason'):raise ValueError('Specific unknown_reason is required')
        value=audit.get('estimation',{}).get('assumed_relative_half_width')
        if not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
            raise ValueError('Unknown estimates need a nonzero stated uncertainty')
        if audit.get('status')=='blocked' and (not audit.get('search_date') or not audit.get('searches') or
            not audit.get('model_specific_analysis') or any(not audit.get('estimation',{}).get(key)
                for key in ('tool','version','method','geometry_source','boundaries'))):
            raise ValueError('Blocked models need complete dated searches and a scoped estimation method')

def publish_binding(d,card):
    """Augment the shared publisher's reference before normal SDK staging.

    The content address binds table bytes; this additional record binds those
    bytes to the authored geometry and the card's physical input values.
    Output links/computed descriptive drag are excluded to avoid a hash cycle.
    """
    from . import pipeline
    validate_card_geometry(d,card)
    source=Path(d.path).resolve();root=Path(pipeline.repo_root()).resolve();ref=card['sources']['hangar-vlm']
    name=card['model'].removeprefix('jsbsim:')
    product=root/'aircraft/stores'/name/ref['url']
    table=json.loads(product.read_text(encoding='utf-8'))
    mach=table['tables']['mach']['mach'];alpha=table['tables']['alpha'];beta=table['tables']['beta']
    record=dict(basis='simulated',tool='Hangar shared aircraft VLM',version=__version__,
        design_sha256=sha(source),card_inputs=card_inputs(card),table_inputs_hash=d.table_inputs_hash(),
        table_sha256=sha(product),
        input_geometry=str(source.relative_to(root)).replace('\\','/'),
        domain=dict(mach=[min(mach),max(mach)],alpha_deg=[min(alpha),max(alpha)],beta_deg=[min(beta),max(beta)]),
        method_sources=['Katz and Plotkin, Low-Speed Aerodynamics, vortex lattice',
                        'Shared aero/model.py, tables.py and mach.py; engineering extensions are not equipment measurements'],
        solver_sha256=hashlib.sha256(b''.join(p.read_bytes().replace(b'\r\n',b'\n') for p in
            sorted((Path(__file__).parent/'aero').glob('*.py')))).hexdigest())
    catalog=json.loads((Path(__file__).parent/'public_data.json').read_text(encoding='utf-8'))
    record['generic_validation']=catalog.get('generic_validation',{'status':'pending','equipment_accuracy_claim':False})
    record['uncertainty']=catalog.get('model_audits',{}).get(d.name,{}).get('estimation',{
        'status':'not-quantified','scope':'No matched equipment error bound is established'})
    ref['simulation']=record
    descriptive=card['parameters']['drag_area_m2'];descriptive['basis']='assumed'
    descriptive['uncertainty']=max(abs(descriptive['value']),1.e-12)
    descriptive['note']='Engineering estimate A_D = CD_simulated*S_geometry at neutral Mach .3. Inputs, dated model-specific searches and method limits are in hangar-vlm.simulation and the evidence catalog. Assumed one-sigma screening uncertainty is |A_D| in m2; its coverage probability is not established. Equipment drag error remains unknown; the table evidence basis is simulated.'
    atomic_write_text(root/'assets/stores'/(card['id']+'.json'),json.dumps(card,indent=2,ensure_ascii=False)+'\n')
    return record

def verify_binding(d,card):
    from . import pipeline
    validate_card_geometry(d,card)
    ref=card['sources'].get('hangar-vlm',{});name=card['model'].removeprefix('jsbsim:')
    product=Path(pipeline.repo_root())/'aircraft/stores'/name/ref.get('url','')
    validate_binding(ref.get('simulation'),card_inputs=card_inputs(card),design_sha256=sha(d.path),
                     table_inputs_hash=d.table_inputs_hash(),table_sha256=sha(product))

def aam_inventory():
    """Read-only source/cache/published-product comparison for every AAM."""
    import pickle
    import numpy as np
    from . import missiles,pipeline,airframe
    root=Path(pipeline.repo_root())
    catalog=json.loads((Path(__file__).parent/'public_data.json').read_text(encoding='utf-8'))
    rows=[]
    for name,author in sorted(missiles.MISSILES.items()):
        source=root/'aircraft'/name/(name+'.toml')
        card_path=root/'assets/stores'/(author['card']+'.json')
        card=json.loads(card_path.read_text(encoding='utf-8'))
        d=pipeline.Design(str(source),log=lambda message:None)
        row=dict(model=name,card=author['card'],calibration_status=catalog['model_audits'][name]['status'],
            aerodynamic_gate=catalog['model_audits'][name].get('aerodynamic_gate',{}),
            card_sha256=sha(card_path),geometry_sha256=sha(source),
            card_geometry_inputs=card_inputs(card),table_inputs_key=d.table_inputs_hash(),
            has_specific_model_coefficient_data=bool(catalog['models'][name].get('cases')),
            data_gap=catalog['model_audits'][name]['unknown_reason'])
        try:
            row['source_geometry']=validate_card_geometry(d,card)
            verify_binding(d,card)
            canonical,_=missiles.design(name)
            row['canonical_geometry_matches']=source.read_text(encoding='utf-8')==canonical
            cache=source.parent/'out/tables.pkl'
            with cache.open('rb') as stream:saved=pickle.load(stream)
            row['base_cache_key']=saved.get('_key')
            ref=card['sources']['hangar-vlm'];binding=ref['simulation']
            product=root/'aircraft/stores'/name/ref['url']
            table=d.calibrated(d._with_wave_drag(saved))
            payload=airframe.portable(table,d.aircraft,np.asarray(d.aircraft.spec['mass']['empty_cg'],float))
            expected=(json.dumps(payload,sort_keys=True,separators=(',',':'),allow_nan=False)+'\n').encode()
            row.update(table_path=str(product.relative_to(root)).replace('\\','/'),
                published_table_sha256=sha(product),current_table_sha256=hashlib.sha256(expected).hexdigest(),
                card_reference_geometry_sha256=binding['design_sha256'],card_reference_table_sha256=binding['table_sha256'],
                xml_sha256=sha(root/'aircraft/stores'/name/(name+'.xml')))
            row['consistent']=(row['base_cache_key']==row['table_inputs_key'] and
                row['current_table_sha256']==row['published_table_sha256'])
        except Exception as exc:
            row.update(consistent=False,reason=str(exc))
        rows.append(row)
    return dict(schema='fsim.aam-geometry-table-binding.v1',models=rows,
        scope='Geometry/cache/card/table consistency only; not a real-data calibration or equipment fidelity claim')

def main():
    import argparse
    parser=argparse.ArgumentParser(description='Check every AAM against current source geometry and published tables')
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args();result=aam_inventory()
    atomic_write_text(args.out,json.dumps(result,indent=2,allow_nan=False)+'\n')
    good=sum(r['consistent'] for r in result['models'])
    print(json.dumps(dict(models=len(result['models']),consistent=good)))
    return 0 if good==len(result['models']) else 1

if __name__=='__main__':raise SystemExit(main())
