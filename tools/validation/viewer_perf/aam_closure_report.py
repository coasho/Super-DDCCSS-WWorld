"""Join frozen before/after AAM diagnostics and explicit unresolved gates.

Requires every fresh native and fullscreen case. No simulation, fitting,
source-card mutation or acceptance-threshold changes occur here.
"""
import argparse
import html
import json
import re
from pathlib import Path
import xml.etree.ElementTree as ET
import aam_campaign as campaign


def read(p):return json.loads(p.read_text(encoding='utf-8'))
def esc(value):return html.escape(str(value))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--before',type=Path,required=True);p.add_argument('--after',type=Path,required=True)
    a=p.parse_args();before=a.before.resolve();after=a.after.resolve();out=after.parent
    old=read(before/'results.json');current=read(after/'manifest.json')
    # Apply the same corrected lifetime/clock filter to both raw campaigns.
    # Keep the original report and its recorded statistics immutable.
    for row in old['models']:
        row['native']=campaign.summarize(Path(row['recording']),read(Path(row['evidence'])))
    campaign.write(out/'before-recomputed.json',old)
    assert len(current['models'])==48 and all(r['run_status']=='ok' for r in current['models'])
    old_cases={(r['model'],r['scenario']):r for r in old['models']}
    assert set(old_cases)=={(r['model'],r['scenario']) for r in current['models']}
    assert all(r['scene_sha256']==old_cases[(r['model'],r['scenario'])]['scene_sha256'] for r in current['models'])
    visuals=[r['visual'] for r in current['models'] if 'visual' in r]
    assert len(visuals)==24 and all(v['status']=='captured' and v['matches_preflight'] for v in visuals)
    assert sum(len(v['captures']) for v in visuals)==96
    campaign.report(after,current);new=read(after/'results.json')
    assert all(n['native']['first_reverse_flow'] is None for n in new['models'])
    audit=read(after/'fin-render-audit.json');assert audit['distinct_models']==24 and len(audit['models'])==48
    assert all(r['fin_angles']>0 and r['angle_mismatches']==r['matrix_error_max']==r['rate_violations']==r['limit_violations']==0 for r in audit['models'])
    bindings=read(out/'bindings.json');assert len(bindings['models'])==24 and all(r['consistent'] for r in bindings['models'])
    native=ET.parse(out/'native-verified-aggregate.junit.xml').getroot()
    assert len(native.findall('testcase'))==12 and not native.findall('.//failure')
    gates=read(out/'python-gates.json');assert len(gates)==7 and all(r['exit_code']==0 for r in gates)
    python_count=sum(int(re.search(r'Ran (\d+) tests?',(out/('python-'+r['check']+'.log')).read_text()).group(1)) for r in gates)
    mechanisms=read(out/'propulsion-controls.json');assert len(mechanisms['checks'])==6
    references=[read(out/('reference-'+m+'.json')) for m in ('mk82','cdsa22')]
    assert all(r['gain_accepted'] for r in references)
    def metrics(rows):
        values=[r['native'] for r in rows]
        return dict(cases=len(values),reverse_flow=sum(n['first_reverse_flow'] is not None for n in values),
            peak_alpha_deg=max(n['max_alpha_deg'] for n in values),peak_load_g=max(n['max_load_factor'] for n in values),
            detonations=sum(n['terminal_kind']=='detonated' for n in values),
            in_flight=sum(n['terminal_kind']=='in_flight' for n in values),
            damage_cases=sum(n.get('native_event_counts',{}).get('damage',0)>0 for n in values),
            native_destroyed_cases=sum(n['native_destroyed'] for n in values))
    fixed=[
      ('Wingless Mach correction','A strake was incorrectly used as an aircraft main wing, reversing supersonic lift/damping. Wing-wake correction now requires a real upstream wing; coupled base VLM interaction is retained.'),
      ('PL-10 physical control','A separate finite-actuator controller and local incidence/Mach Jacobians remove the reproduced supersonic oscillation. Original response inputs and legacy providers remain supported.'),
      ('Measured terminal guidance','All24 exact variant bindings select measured seeker families and PN. IR/SARH terminal steering no longer consumes hidden target range/velocity.'),
      ('Navigation and lifetime','Admitted inertial seeds and enabled dated launcher estimates remain separate from terminal measurements. Invalid epochs and resets cannot retain or poison navigation/activation history.'),
      ('Actual fin telemetry','Selected native derivative metadata version 2 retains the unchanged public per-fin schema; wrong versions still refuse. Fresh production replay checks every real angle and hinge matrix.'),
      ('Declaration and report integrity','R-73 canonical radius now aliases its existing used radius without changing it. Reports use current outcomes. Typed ducted designs refuse missing/incompatible cycle profiles, and migration preserves specialized selections. Sibling generated stores are synchronized; original fitted parameters and measured holdouts are retained.')]
    engineering=[
      ('AIM-9X, IRIS-T, MICA EM/IR TVC','Finite native equivalent vectoring rotates existing thrust and produces its current-CG torque. Native force/moment, stops/rates and stuck-nozzle checks pass. Hardware angle/rate curves remain assumed with nonzero uncertainty.'),
      ('Meteor propulsion','Separate native solid booster and atmosphere-fed throttled sustainer replace the solid-only mechanism. Positive-airflow and no-fuel checks pass; mass/CG/inertia are retained. Inlet/combustor/nozzle maps remain uncalibrated.')]
    pending=[
      ('blocked_measurements','All24 exact-model aerodynamic calibration','Missing admitted specific-variant/configuration force and moment measurements with conditions and independent holdouts. Simulated tables are engineering estimates.'),
      ('blocked_measurements','Equipment response fidelity','Exact motor, vector/fin actuator, RF/IR detector, seeker-servo, countermeasure and lethality response data remain unavailable or uncalibrated. Ideal geometric sensing is explicitly retained.'),
      ('open_implementation','GPS, fitted weapon links and commanded pulse management','GPS error/aiding, fitted two-way weapon-link transport/feedback and commanded dual-pulse management are not implemented by these repairs. Existing unavailable actor feedback is preserved; a reviewed versioned contract and owner handoff are required for that public extension.'),
      ('pending_acceptance','Full equipment maneuver/energy envelope','These unchanged short forward encounters do not establish maximum range/Mach/g, high off-boresight, LOAL, ECM/ECCM or no-escape-zone performance. No all-equipment closure is claimed.')]
    result=dict(schema='fsim.aam-repair-evidence.v1',status='software_repairs_verified_equipment_closure_incomplete',
        head=current['head'],before=metrics(old['models']),after=metrics(new['models']),
        native_tests=12,python_tests=python_count,fullscreen_models=24,captures=96,
        actual_fin_angles=sum(r['fin_angles'] for r in audit['models']),
        fin_samples=sum(r['samples'] for r in audit['models']),
        controller_unavailable_samples=sum(r['controller_unavailable_samples'] for r in audit['models']),
        fixed=[dict(issue=k,repair=v) for k,v in fixed],engineering=[dict(issue=k,repair=v) for k,v in engineering],
        pending=[dict(status=s,issue=k,reason=v) for s,k,v in pending],
        visual_wall_seconds=[min(v['native_wall_seconds'] for v in visuals),max(v['native_wall_seconds'] for v in visuals)],
        executables=current['executables'],
        supplemental_event_strike=read(after/'event-strike-aim120c/result.json') if (after/'event-strike-aim120c/result.json').is_file() else None,
        supplemental_exclusions=[dict(case='PL-10 head-on',reason='Did not satisfy the separate crew-loss strike predicate; original native outcome preserved.')])
    campaign.write(out/'closure.json',result)
    def rows(items):return ''.join('<tr><th>'+esc(k)+'</th><td>'+esc(v)+'</td></tr>' for k,v in items)
    model_rows=[]
    for model in sorted({r['model'] for r in new['models']}):
        previous=[r for r in old['models'] if r['model']==model];now=[r for r in new['models'] if r['model']==model]
        def peaks(rs):return 'alpha %.2f deg / load %.2f g'%(max(r['native']['max_alpha_deg'] for r in rs),max(r['native']['max_load_factor'] for r in rs))
        def outcomes(rs):return '; '.join(r['scenario']+': '+r['native']['terminal_kind']+' (%.2f m)'%r['native']['fuze_miss_m'] for r in rs)
        visual=next(r['visual'] for r in now if 'visual' in r)
        images=''.join('<a href="'+Path(s).relative_to(out).as_posix()+'"><img width="240" loading="lazy" src="'+Path(s).relative_to(out).as_posix()+'"></a>' for s in visual['captures'])
        model_rows.append('<tr><th>'+esc(model)+'</th><td>'+esc(peaks(previous))+'<br>'+esc(outcomes(previous))+'</td><td>'+esc(peaks(now))+'<br>'+esc(outcomes(now))+'</td><td>'+images+'</td></tr>')
    text='''<!doctype html><meta charset="utf-8"><title>AAM repair evidence</title><style>body{font:16px system-ui;background:#101820;color:#edf2f7;margin:28px}a{color:#8dc9ff}table{border-collapse:collapse;width:100%;margin:20px 0}td,th{border:1px solid #566574;padding:10px;text-align:left;vertical-align:top}img{display:inline-block}.open{border:2px solid #e5b45f;padding:16px}code{overflow-wrap:anywhere}</style><h1>All24 AAM repair evidence</h1>
<p class="open">Software repairs are verified. Full equipment closure is incomplete: measured calibration, equipment mechanisms and envelope acceptance below remain open. Detonation, native damage and destruction retain different meanings. Misses are preserved.</p>
<p><a href="../aam-chain-20261030/index.html">Frozen original report</a> · <a href="after/index.html">Current per-model analysis and logs</a> · <a href="catalogue/report.html">Provider/source catalogue</a> · <a href="closure.json">Machine-readable issue ledger</a></p>'''
    text+='<p>Same 48 frozen scenes; no changed launch geometry, fuze radius, maximum-g input or forced hits. '+str(python_count)+' Python tests; 12 current native gates (11 plus one targeted fixture retest); '+str(result['actual_fin_angles'])+' actual fin angles audited; 24 complete fullscreen 1x cases and 96 native-resolution captures.</p>'
    text+='<h2>Before and after</h2><pre>'+esc(json.dumps(dict(before=result['before'],after=result['after']),indent=2))+'</pre>'
    text+='<h2>Repaired software defects</h2><table>'+rows(fixed)+'</table><h2>New engineering mechanisms</h2><table>'+rows(engineering)+'</table>'
    text+='<h2>Unresolved equipment gates</h2><table>'+''.join('<tr><th>'+esc(k)+'<br>'+esc(s)+'</th><td>'+esc(v)+'</td></tr>' for s,k,v in pending)+'</table>'
    text+='<p>Native nozzle/cycle evidence: <a href="propulsion-controls.json">six mechanism checks</a>. Geometry/source bindings: <a href="bindings.json">24 consistent models</a>. Retained measured gains: <a href="reference-mk82.json">MK82</a>, <a href="reference-cdsa22.json">A-22</a>. Native gates: <a href="native-verified-aggregate.junit.xml">JUnit with source provenance</a>. Every failed intermediate attempt remains beside these final artifacts.</p>'
    if result['supplemental_event_strike']:
        text+='<p>Supplemental complete live AIM-120C head-on event capture passed the original native detonation/fragment/crew-loss checks: <a href="after/event-strike-aim120c/strike_hit_plus_0.3s.png">hit plus 0.3 s target state</a>, <a href="after/event-strike-aim120c/result.json">timing and native outcome</a>. PL-10 head-on did not satisfy that separate crew-loss predicate and was retained as an exclusion. No crew loss or explosion was invented.</p>'
    text+='<h2>Every model</h2><table><tr><th>Model</th><th>Before</th><th>After</th><th>Full-scene live captures</th></tr>'+''.join(model_rows)+'</table>'
    (out/'index.html').write_text(text,encoding='utf-8')
    print(json.dumps({k:result[k] for k in ('status','before','after','python_tests','actual_fin_angles','visual_wall_seconds')},indent=2))


if __name__=='__main__':main()
