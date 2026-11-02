"""Current radar roster and separate equipment-calibration uncertainty ledger.

Inventory success never establishes engineering acceptance: native functions,
methods, installed consumers and regressions are joined by report.py.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[3]


def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def audit(receipts):
    sys.path.insert(0,str(ROOT/'tools/models/hangar'))
    from hangar.engineering import roster
    designs=[r for r in roster() if r['category']!='store']
    cards={}
    for p in sorted((ROOT/'assets/payloads').glob('*.json')):
        c=json.loads(p.read_text(encoding='utf-8'))
        if c.get('family')=='air_radar':cards[p.stem]=(p,c)
    accepted={}
    if receipts:
        if any(p.casefold()=='.codex' for p in receipts.parts):raise ValueError('Excluded receipt directory')
        for p in receipts.glob('*.json'):
            d=json.loads(p.read_text(encoding='utf-8'))
            if d.get('schema')!='fsim.radar.equipment-calibration.v1':continue
            card=d.get('card')
            if card not in cards or card in accepted:raise ValueError('Duplicate or unknown calibration card')
            if d.get('equipment_acceptance') is True and d.get('independent_source_review',{}).get('accepted') is True:
                accepted[card]=dict(path=str(p),sha256=digest(p),receipt=d)
    fits=[]
    for design in designs:
        p=ROOT/'aircraft'/design['name']/'payloads.json';air=[]
        if p.is_file():
            fit=json.loads(p.read_text(encoding='utf-8'))
            air=[dict(payload=r.get('name'),card=r['card']) for r in fit.get('payloads',[]) if r.get('card') in cards]
        fits.append(dict(design=design['name'],category=design['category'],fit_path=str(p),
            fit_sha256=digest(p) if p.is_file() else None,air_radars=air,
            installation_assessment='declared_fit_only' if air else 'no_air_radar_declared_not_a_verified_absence_claim'))
    rows=[]
    for name,(path,card) in cards.items():
        parameters=card.get('parameters',{});calibration=accepted.get(name)
        sources=card.get('sources',{})
        issues=[]
        for parameter,value in parameters.items():
            if not isinstance(value,dict) or value.get('basis') not in ('published','derived','assumed'):
                issues.append('Missing parameter evidence classification: '+parameter);continue
            if value.get('source') not in sources:issues.append('Unknown parameter source: '+parameter)
            if not isinstance(value.get('unit'),str):issues.append('Missing parameter unit: '+parameter)
            if not isinstance(value.get('uncertainty'),(int,float)) or value['uncertainty']<0:
                issues.append('Missing nonnegative declared uncertainty: '+parameter)
        rows.append(dict(card=name,designation=card['designation'],card_path=str(path.relative_to(ROOT)),card_sha256=digest(path),
            fitted_designs=[r['design'] for r in fits if any(p['card']==name for p in r['air_radars'])],
            nonthermal_floors={k:parameters.get(k) for k in ('angle_floor_beamwidths','range_floor_resolutions','range_rate_floor_resolutions')},
            calibration=calibration,status='calibrated' if calibration else 'uncalibrated',
            blockers=issues,calibration_limit=None if calibration else 'No admitted variant-specific independent accuracy-calibration receipt',
            parameter_count=len(parameters),parameter_evidence_counts={basis:sum(isinstance(p,dict) and p.get('basis')==basis for p in parameters.values())
                for basis in ('published','derived','assumed')},
            numerical_or_simulated_reference_is_equipment_calibration=False))
    return dict(schema='fsim.sim-m01.current-coverage.v2',closure_rule='2026-11-02 user clarification: complete engineering functions and reproducible delivery required; equipment calibration is a separate uncertainty ledger',
        main_source='Current authored roster and payload cards; no representative or cloned-default coverage',
        airframes=len(fits),air_radar_cards=len(rows),fits=fits,radars=rows,
        equipment_calibrated=sum(r['status']=='calibrated' for r in rows),
        calibration_pending=sum(r['status']=='uncalibrated' for r in rows),
        inventory_passed=bool(rows) and all(not r['blockers'] and r['fitted_designs'] for r in rows),
        formal_acceptance=False,
        scope='Inventory only; functional, method, performance, regression and installed-product gates additionally required')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--calibration-receipts',type=Path);a=p.parse_args();started=time.perf_counter()
    if any(x.casefold()=='.codex' for x in a.output.parts):raise ValueError('Excluded output directory')
    result=audit(a.calibration_receipts);result['elapsed_seconds']=time.perf_counter()-started
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('fits','radars')}))
    raise SystemExit(0 if result['inventory_passed'] else 2)


if __name__=='__main__':main()
