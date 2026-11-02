"""Join complete M01 roster, calibration ledger and actual installed sampling.

CSV rows and hash checks are consistency evidence, never physical tests or
equipment calibration. This report cannot close SIM-M01 on sampling alone.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('coverage','functional','installed','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--stem',default='sim-m01');a=p.parse_args()
    coverage=json.loads(a.coverage.read_text(encoding='utf-8'));functional=json.loads(a.functional.read_text(encoding='utf-8'))
    installed=json.loads(a.installed.read_text(encoding='utf-8'))
    if not installed['passed'] or not functional['passed']:raise ValueError('Actual installed functional evidence must pass')
    if functional['inventory_sha256']!=digest(a.coverage):raise ValueError('Functional inventory differs from coverage')
    def unique(rows,key):
        result={row[key]:row for row in rows}
        if len(result)!=len(rows):raise ValueError('Duplicate '+key+' input')
        return result
    installed_modules=unique(installed['loaded_modules']['modules'],'name')
    functional_modules=unique(functional['loaded_modules'],'name')
    if set(functional_modules)!=set(installed_modules):raise ValueError('Native module population differs')
    for name,module_row in installed_modules.items():
        if functional_modules[name]['sha256']!=module_row['sha256']:raise ValueError('Different loaded native product: '+name)
    module=next(m for m in installed['loaded_modules']['modules'] if m['name']=='libfsim.dll')
    if functional['sdk_sha256']!=module['sha256']:raise ValueError('Functional population used a different SDK product')
    sampled=unique(functional['rows'],'design');covered=unique(coverage['fits'],'design')
    unique(coverage['radars'],'card')
    if set(sampled)!=set(covered):raise ValueError('Roster rows missing or extra')
    for name,fit in covered.items():
        row=sampled[name];stages=row.get('checks',[])
        if row.get('passed') is not True or any(stage.get('passed') is not True for stage in stages):
            raise ValueError('Failed or missing stage acceptance: '+name)
        if fit['air_radars']:
            if len(fit['air_radars'])!=1 or row.get('card')!=fit['air_radars'][0]['card']:raise ValueError('Unmatched declared fit: '+name)
            expected=['detection_track_and_fusion']+(['signal_surveillance'] if row['rotodome'] else
                ['signal_stt','signal_tws','signal_cued_acquisition','signal_velocity_search'])
            if [stage['stage'] for stage in stages]!=expected:raise ValueError('Incomplete common sampling stages: '+name)
        elif stages or row.get('verified_equipment_absence') is not False:
            raise ValueError('Undeclared fit misrepresented as exercised/equipment absence: '+name)
    if functional['checked_stages']!=sum(len(row.get('checks',[])) for row in sampled.values()):raise ValueError('Incorrect sampling count')
    a.output.mkdir(parents=True,exist_ok=True)
    def write(name,rows):
        with (a.output/(a.stem+'-'+name+'.csv')).open('w',newline='',encoding='utf-8') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    fits=[]
    for fit in coverage['fits']:
        row=sampled[fit['design']];stages=row.get('checks',[])
        fits.append(dict(design=fit['design'],category=fit['category'],
            declared_air_radars=';'.join(p['card'] for p in fit['air_radars']),
            sampling='passed' if stages else 'not_exercised_no_air_radar_in_authored_fit',
            stages=len(stages),basis='simulated' if stages else 'authored_fit_inventory',
            verified_equipment_absence=False,equipment_calibration='blocked' if stages else 'not_assessed_by_this_air_radar_package'))
    radars=[]
    for radar in coverage['radars']:
        rows=[sampled[name] for name in radar['fitted_designs']]
        if not rows or any(not row['passed'] or row.get('card')!=radar['card'] for row in rows):raise ValueError('Card lacks matched actual sampling')
        radars.append(dict(card=radar['card'],designation=radar['designation'],fitted_designs=';'.join(radar['fitted_designs']),
            functional_sampling='passed',stages=sum(len(row['checks']) for row in rows),basis='simulated',
            calibration_required=True,calibration_present=radar['calibration'] is not None,
            calibration_check='passed' if radar['calibration'] else 'failed',status=radar['status'],
            blocker=';'.join(radar['blockers']),card_sha256=radar['card_sha256']))
    write('fit-matrix',fits);write('radar-matrix',radars)
    record=dict(schema='fsim.sim-m01.delivery.v1',formal_acceptance=False,
        closure_rule=coverage['closure_rule'],airframes=len(fits),sampled_air_radar_fits=len(radars),
        native_sampling_stages=sum(row['stages'] for row in radars),
        equipment_calibrated=coverage['equipment_calibrated'],calibration_blocked=coverage['calibration_blocked'],
        calibration_cannot_be_replaced_by_functional_sampling=True,
        loaded_sdk=module,inputs=[dict(path=str(path),sha256=digest(path)) for path in (a.coverage,a.functional,a.installed)])
    (a.output/(a.stem+'-delivery.json')).write_text(json.dumps(record,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(record))


if __name__=='__main__':main()
