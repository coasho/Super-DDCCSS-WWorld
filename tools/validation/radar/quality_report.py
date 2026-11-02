"""Admit complete M01 delivery only from reconciled full and visual evidence."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
from contract import METHOD_TESTS

APPROVED_ROTOR_POLICY='d52e0e76afdb93ddbe05513f8a04253812271cf902f865d92f03527bb11e3645'


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_text(encoding='utf-8'))


def admit(campaign,review):
    root=Path(campaign);whole=read(root/'full-campaign.json');engineering=read(root/'engineering/sim-m01-delivery.json')
    if not whole.get('automated_passed') or not engineering.get('engineering_acceptance'):raise ValueError('Incomplete campaign')
    required={'engineering','applicability','cargo-lock-preflight','ucrt64-headless','ucrt64-release','visual',
        'ucrt64-headless-installed-equipment-preflight','ucrt64-release-installed-equipment-preflight'}
    if len(whole['commands'])!=len(required) or {c['label'] for c in whole['commands']}!=required:raise ValueError('Missing campaign command')
    if any(c['exit_code'] or sha(c['log'])!=c['sha256'] for c in whole['commands']):raise ValueError('Failed or changed command evidence')
    if any(sha(i['path'])!=i['sha256'] for i in engineering['inputs']):raise ValueError('Changed engineering evidence')
    policy_path=Path(whole['source_root'])/'docs/data/sim-m02-deferred-rotor-issues-20261101.json'
    if sha(policy_path)!=APPROVED_ROTOR_POLICY:raise ValueError('Approved source policy changed')
    policy=read(policy_path)
    full={}
    for preset in ('ucrt64-headless','ucrt64-release'):
        directory=root/preset;receipt=read(directory/'regression.json');inventory=read(directory/'inventory.json')
        cases=ET.parse(directory/'junit.xml').getroot().findall('.//testcase');names=[c.get('name') for c in cases]
        excluded=receipt['excluded_names'];population=[r['name'] for r in inventory['tests']]
        if not receipt['passed'] or receipt['policy_sha256']!=APPROVED_ROTOR_POLICY or len(excluded)!=28 or len(set(excluded))!=28 or set(excluded)!=set(policy['excluded_tests']):raise ValueError('Full regression or approved scope changed')
        if len(population)!=len(set(population)) or not set(excluded).issubset(population) or receipt['nested_methods']['unexpected_skips']:raise ValueError('Inventory or nested method accounting invalid')
        if not cases or len(set(names))!=len(names) or set(names)!=set(population)-set(excluded) or len(cases)!=receipt['executed']:raise ValueError('Incomplete full inventory execution')
        if not set(METHOD_TESTS).issubset(names) or set(METHOD_TESTS)&set(excluded):raise ValueError('M01 method coverage missing')
        if any(c.find('failure') is not None or c.find('error') is not None or c.find('skipped') is not None for c in cases):raise ValueError('Full regression contains failure or skip')
        if sha(directory/'junit.xml')!=receipt['junit_sha256'] or receipt['source_changes'] or receipt['runtime_changes'] or receipt['configuration_changes']:raise ValueError('Changed full regression inputs/results')
        expected=whole['full_regression'][preset]
        if sha(directory/'regression.json')!=expected['receipt_sha256'] or sha(directory/'junit.xml')!=expected['junit_sha256']:raise ValueError('Campaign/full receipt mismatch')
        full[preset]=dict(discovered=len(population),passed=len(cases),rotor_deferred=len(excluded),nested_methods=receipt['nested_methods']['unittest_methods'])
    applicability=read(root/'applicability.json');coverage=read(root/'engineering/coverage.json')
    if not applicability['passed'] or {f['design'] for f in applicability['fits']}!={f['design'] for f in coverage['fits']}:raise ValueError('Incomplete applicability roster')
    fits={f['design']:f for f in coverage['fits']}
    if any(not row['engineering_assignment_closed'] or row['fit_sha256']!=fits[row['design']]['fit_sha256'] for row in applicability['fits']):raise ValueError('Unreconciled selected fit')
    visual=read(root/'visual/visual.json');manual=read(review)
    if not visual['automated_passed'] or not manual.get('accepted') or manual['visual_receipt_sha256']!=sha(root/'visual/visual.json'):raise ValueError('Actual visual acceptance missing')
    images=[i for s in visual['scenarios'] for i in s['images']];reviewed={r['path']:r for r in manual['captures']}
    if len(visual['scenarios'])!=2 or {s['mode'] for s in visual['scenarios']}!={'fighter','aew'}:raise ValueError('Incomplete visual scenario population')
    for scene in visual['scenarios']:
        trace=root/'visual'/(scene['mode']+'-trace.json')
        if scene['scene_exit'] or scene['viewer_exit'] or not .95<=scene['real_time_factor']<=1.02 or sha(trace)!=scene['trace_sha256']:raise ValueError('Failed or changed real-time scene')
    if len(images)!=10 or len(reviewed)!=10 or set(reviewed)!={i['path'] for i in images}:raise ValueError('Incomplete actual capture review')
    for image in images:
        if sha(image['path'])!=image['sha256'] or reviewed[image['path']]['sha256']!=image['sha256'] or not reviewed[image['path']]['accepted']:raise ValueError('Changed or rejected capture')
    if visual['primary_resolution']!=manual['actual_primary_resolution'] or any(i['resolution']!=visual['primary_resolution'] for i in images):raise ValueError('Actual display resolution mismatch')
    return dict(schema='fsim.sim-m01.full-quality-delivery.v1',engineering_acceptance=True,m01_unclosed_gates=[],
        full_regression=full,airframes=len(fits),air_radar_fits=engineering['sampled_air_radar_fits'],stock_models=len(applicability['stock_models']),
        native_sampling_stages=engineering['native_sampling_stages'],cross_worker_replay_stages=engineering['cross_worker_replay_stages'],
        visual_scenarios=len(visual['scenarios']),actual_reviewed_captures=len(images),resolution=visual['primary_resolution'],
        real_time_factors=[s['real_time_factor'] for s in visual['scenarios']],
        equipment_accuracy=dict(admitted=engineering['equipment_calibrated'],uncalibrated=engineering['calibration_pending'],
            rule='User-approved separate empirical calibration ledger; no missing engineering gate is waived'),
        inputs=[dict(path=str(path),sha256=sha(path)) for path in (root/'full-campaign.json',root/'engineering/sim-m01-delivery.json',root/'applicability.json',root/'visual/visual.json',Path(review))])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--campaign',type=Path,required=True);p.add_argument('--review',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists() or any(x.casefold()=='.codex' for path in (a.campaign,a.review,a.output) for x in path.parts):raise ValueError('Fresh allowed paths required')
    result=admit(a.campaign,a.review);a.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8');print(json.dumps(result))
