"""Configure reviewable calibration provenance without inventing fitted data.

Existing performance fits are retained. A missing independent reference leaves
an empty override section; this is an explicit uncalibrated model, not a fit.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import tomllib

from . import pipeline
from .files import atomic_write_text


def numerical_source_rebinding(path,expected):
    """Recognize a byte-bound physical source with only added integrator metadata.

    This retains an existing fit; it does not create measurements or refit any
    coefficient. The original calibration record remains unchanged. Changes to
    geometry, reference dimensions or any other physical section still fail.
    """
    text=Path(path).read_text(encoding='utf-8');spec=tomllib.loads(text)
    if 'integration' not in spec:return None
    from .integration import source_options
    source_options(path) # Require the supported versioned numerical contract.
    lines=text.splitlines(keepends=True)
    begin=next(i for i,line in enumerate(lines) if re.fullmatch(r'\s*\[integration\]\s*(?:#.*)?\n?',line))
    end=next((i for i in range(begin+1,len(lines)) if re.match(r'\s*\[',lines[i])),len(lines))
    while begin and (not lines[begin-1].strip() or lines[begin-1].lstrip().startswith('#')):begin-=1
    before=''.join(lines[:begin]).rstrip('\n');after=''.join(lines[end:])
    physical={k:v for k,v in spec.items() if k!='integration'}
    for separator in ('','\n','\n\n'):
        candidate=before+separator+after
        if hashlib.sha256(candidate.encode('utf-8')).hexdigest()==expected and tomllib.loads(candidate)==physical:
            return dict(previous_source_sha256=expected,current_source_sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest(),
                changed_sections=['integration'],scope='Existing reference fit retained; exact original physical source recovered. No new calibration or measurements.')
    return None


def configure(path):
    path=Path(path)
    spec=tomllib.loads(path.read_text(encoding='utf-8'))
    destination=path.parent/'calibration.toml'
    original=destination.read_text(encoding='utf-8') if destination.is_file() else '[calibration]\n# No independent fit is available; the shared model has no per-model overrides.\n'
    data=tomllib.loads(original)
    values=data.get('calibration',{})
    allowed={'extra_drag_area_m2','base_drag_scale','propeller_pitch_m','throttle_ratio','wave_drag_efficiency','korn_kappa','wave_drag_rise_exponent',
             'body_friction_reynolds_coupling','body_friction_temperature_k','body_friction_density_kg_m3'}
    if set(values)-allowed:raise ValueError('Unknown calibration override in '+str(destination))
    if data.get('provenance',{}).get('status')=='sdk-reference-fit':
        current_source=hashlib.sha256(path.read_bytes()).hexdigest()
        source_rebinding=None
        if data['provenance'].get('observed_source_sha256')!=current_source:
            source_rebinding=numerical_source_rebinding(path,data['provenance'].get('observed_source_sha256'))
            if source_rebinding is None:raise ValueError('SDK reference calibration is stale; rerun hangar calibrate: '+path.stem)
        from .reference_campaign import load_catalog,reference_hash
        catalog=load_catalog()
        expected=data['provenance'].get('reference_inputs_sha256')
        if expected and expected!=reference_hash(path.stem,catalog):
            raise ValueError('Reference observations changed; rerun hangar calibrate: '+path.stem)
        fitted=set(catalog['models'][path.stem].get('parameters',{}))
        simulated=set(data['provenance'].get('simulated_method_input_names',()))
        return dict(model=path.stem,status='sdk-reference-fit',overrides=values,
                    fitted_parameters=sorted(fitted),simulated_method_inputs=sorted(simulated),
                    unverified_parameters=sorted(set(values)-fitted-simulated),
                    targets=data['provenance'].get('calibration_case_ids',[]),
                    source_sha256=current_source,source_rebinding=source_rebinding,reference={})
    status='existing-performance-fit' if values else 'unfitted'
    reference=data.get('reference',{})
    if reference:
        if reference.get('kind')!='configuration-drag-index' or not reference.get('source_url'):
            raise ValueError('Unsupported or unsourced calibration reference')
        # Preserve historical overrides as historical data. A normal build
        # cannot fit new coefficients against a table-only surrogate; all
        # new calibration belongs to reference_campaign's SDK objective.
        status='configuration-reference-fit'
    source=hashlib.sha256(path.read_bytes()).hexdigest()
    previous=data.get('provenance',{})
    if (previous.get('schema')=='fsim.hangar-calibration.v1' and
        previous.get('method')=='shared-aircraft-vlm' and previous.get('status')==status and
        previous.get('observed_source_sha256')==source):
        # Inventorying unchanged coefficients must not rewrite metadata and
        # invalidate source-bound flight receipts on an otherwise no-op build.
        return dict(model=path.stem,status=status,overrides=values,targets=sorted(spec.get('targets',{})),
                    source_sha256=source,reference=reference)
    original=original.split('\n[provenance]',1)[0].rstrip()+'\n'
    targets=sorted(spec.get('targets',{}))
    metadata=('\n[provenance]\nschema = "fsim.hangar-calibration.v1"\nmethod = "shared-aircraft-vlm"\n'
              'status = '+json.dumps(status)+'\nobserved_source_sha256 = '+json.dumps(source)+'\n'
              'target_keys = '+json.dumps(targets)+'\n'
              'scope = "Declared reference targets only; no six-axis equipment accuracy certification"\n'
              'historical_fit_note = "Retained performance overrides are not newly validated equipment data"\n')
    atomic_write_text(destination,original+metadata)
    return dict(model=path.stem,status=status,overrides=values,targets=targets,source_sha256=source,reference=reference)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--model',action='append')
    args=parser.parse_args()
    directory=Path(pipeline.repo_root())/'aircraft'
    paths={p.name:p/(p.name+'.toml') for p in sorted(directory.iterdir())
           if p.name.casefold()!='.codex' and (p/(p.name+'.toml')).is_file()}
    selected=sorted(paths) if not args.model or args.model==['all'] else args.model
    if any(name not in paths for name in selected):raise ValueError('Unknown calibration model')
    rows=[configure(paths[name]) for name in selected]
    args.out.parent.mkdir(parents=True,exist_ok=True)
    atomic_write_text(args.out,json.dumps(dict(schema='fsim.hangar-calibration-inventory.v1',models=rows),indent=2)+'\n')
    print(json.dumps(dict(models=len(rows),
        sdk_reference_fits=sum(r['status']=='sdk-reference-fit' for r in rows),
        retained_historical_overrides=sum(r['status'] in ('existing-performance-fit','configuration-reference-fit') for r in rows),
        unfitted=sum(r['status']=='unfitted' for r in rows),
        scope='Configuration inventory only; calibration/retest completeness is reported by hangar calibrate')))


if __name__=='__main__':main()
