"""Admit equipment-specific radar error data before proposing calibration.

Input is an explicitly public, source-hashed measurement dataset. Absence is
not replaced by a generic accuracy fraction, GPS reference accuracy, another
radar's result, or a simulated comparison. This tool never edits payload cards.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
from urllib.parse import urlparse

VERSION='fsim.radar.equipment-calibration.v1'
CHANNELS={'azimuth_rad':'rad','elevation_rad':'rad','range_m':'m','range_rate_ms':'m/s'}


def load(path, card):
    path=Path(path)
    if any(p.casefold()=='.codex' for p in path.parts):raise ValueError('Excluded input directory')
    data=json.loads(path.read_text(encoding='utf-8'))
    if data.get('schema')!=VERSION or data.get('card')!=card or data.get('basis')!='measured':
        raise ValueError('Exact equipment card and measured dataset required')
    if not isinstance(data.get('equipment_variant'),str) or not data['equipment_variant'].strip():
        raise ValueError('Actual equipment variant is required')
    source=data.get('source',{})
    for name in ('title','author','date','location','public_release'):
        if not isinstance(source.get(name),str) or not source[name].strip():raise ValueError('Missing source '+name)
    url=urlparse(source.get('url',''))
    if url.scheme!='https' or not url.hostname or url.username or url.password:raise ValueError('Public primary-source URL required')
    relative=Path(source.get('local_file',''))
    if relative.is_absolute() or '..' in relative.parts or not relative.parts:raise ValueError('Source must be a local relative file')
    if any(p.casefold()=='.codex' for p in relative.parts):raise ValueError('Excluded source directory')
    document=path.parent/relative
    if any(p.is_symlink() or getattr(p,'is_junction',lambda:False)() for p in (document,*document.parents)):
        raise ValueError('Redirected source document')
    if hashlib.sha256(document.read_bytes()).hexdigest()!=source.get('sha256'):
        raise ValueError('Measurement source digest differs')
    channels=data.get('channels',{})
    if set(channels)!=set(CHANNELS):raise ValueError('All four radar measurement channels are required')
    for name,unit in CHANNELS.items():
        row=channels[name]
        if row.get('unit')!=unit:raise ValueError('Measurement units differ: '+name)
        if not isinstance(row.get('operating_conditions'),dict) or not row['operating_conditions']:
            raise ValueError('Measurement operating conditions are required: '+name)
        for split in ('calibration','validation'):
            samples=row.get(split,[])
            if not isinstance(samples,list) or len(samples)<3:raise ValueError('Independent '+split+' samples required: '+name)
            for sample in samples:
                for field in ('error','thermal_sigma','reference_sigma'):
                    v=sample.get(field)
                    if isinstance(v,bool) or not isinstance(v,(float,int)) or not math.isfinite(v):
                        raise ValueError('Finite measured error and uncertainty required')
                    if field!='error' and v<0:raise ValueError('Negative measurement uncertainty')
                if not isinstance(sample.get('sample_id'),str) or not sample['sample_id']:
                    raise ValueError('Traceable sample identity required')
        a=[s['sample_id'] for s in row['calibration']];b=[s['sample_id'] for s in row['validation']]
        if len(set(a+b))!=len(a+b):raise ValueError('Calibration/held-out sample identities overlap')
    return data


def evaluate(data):
    import numpy as np
    from scipy.stats import chi2
    results={}
    for name in CHANNELS:
        channel=data['channels'][name];train=channel['calibration'];held=channel['validation']
        errors=np.array([s['error'] for s in train]);bias=float(errors.mean())
        variance=float(errors.var(ddof=1))
        known=sum(s['thermal_sigma']**2+s['reference_sigma']**2 for s in train)/len(train)
        # One explicit residual floor is identifiable in a fixed operating
        # condition. It does not separately identify glint/radome mechanisms.
        floor=math.sqrt(max(0.,variance-known))
        squared=0.;rows=[]
        for sample in held:
            sigma2=floor*floor+sample['thermal_sigma']**2+sample['reference_sigma']**2
            if not sigma2>0:raise ValueError('Zero held-out predictive uncertainty: '+name)
            residual=sample['error']-bias;squared+=residual*residual/sigma2
            rows.append(dict(sample_id=sample['sample_id'],residual=residual,predictive_sigma=math.sqrt(sigma2)))
        # Frozen two-sided 95% normalized-error gate; original samples remain
        # held out. No optimizer may change this bound or reuse their errors.
        bounds=[float(chi2.ppf(p,len(held))) for p in (.025,.975)]
        results[name]=dict(basis='derived_from_measured',bias=bias,residual_floor=floor,
            calibration_samples=len(train),validation_samples=len(held),normalized_squared_error=squared,
            acceptance_bounds=bounds,passed=bounds[0]<=squared<=bounds[1],validation=rows,
            interpretation='Condition-specific residual floor; Gaussian plug-in gate omits parameter-estimation uncertainty and is not equipment acceptance')
    return dict(schema=VERSION,card=data['card'],equipment_variant=data['equipment_variant'],source=data['source'],
        method='Calibration-only bias/variance estimate; unchanged independent held-out normalized-error gate',
        source_assessment='provenance_verified_manual_admission_required',equipment_acceptance=False,
        passed=all(r['passed'] for r in results.values()),channels=results)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--dataset',type=Path,required=True)
    parser.add_argument('--card',required=True);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    if any(p.casefold()=='.codex' for p in args.output.parts):raise ValueError('Excluded output directory')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    pending=dict(schema=VERSION,card=args.card,passed=False,source_assessment='not_admitted')
    args.output.write_text(json.dumps(pending)+'\n',encoding='utf-8')
    try:result=evaluate(load(args.dataset,args.card))
    except (ValueError,OSError,TypeError,KeyError) as error:result=pending|dict(error=str(error))
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    raise SystemExit(not result['passed'])


if __name__=='__main__':main()
