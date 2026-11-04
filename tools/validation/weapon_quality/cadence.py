"""Reconcile every real V3 World observer trace at all three decision rates."""
import argparse
import bisect
import csv
import hashlib
import json
import math
from pathlib import Path
import shutil

from acceptance import allowed, reconcile_population

ROOT=Path(__file__).resolve().parents[3]


def require(value,message):
    if not value:raise ValueError(message)
def sha(path):return hashlib.sha256(allowed(path).read_bytes()).hexdigest()


def read_summary_shards(source,cards,skip,*,countermeasure=False):
    """Join raw per-card bytes, retaining exact producer files and identities."""
    rows=[];parts=[];inputs=[]
    name='countermeasure-summary-skip4.csv' if countermeasure else 'summary-skip'+str(skip)+'.csv'
    for card in sorted(cards):
        path=allowed(source/card/name);raw=path.read_bytes();digest=hashlib.sha256(raw).hexdigest()
        parsed=list(csv.reader(raw.decode('utf-8').splitlines()))
        require(len(parsed)==1 and len(parsed[0])==(16 if countermeasure else 9) and parsed[0][0]==card,
            'Each native shard must contain exactly its actual model')
        require(raw.endswith(b'\n'),'Native summary row must terminate before joining')
        require(sha(path)==digest,'Native summary shard changed during collection')
        rows.extend(parsed);parts.append(raw);inputs.append(dict(card=card,path=str(path.resolve()),sha256=digest))
    require(len(rows)==len(set(cards)) and len(rows)>0,'Empty or duplicate native shard population')
    return rows,b''.join(parts),inputs


def analyze_trace(rows,period,delay,decision_interval):
    measurements={};decisions=[];tracking=0;delays=[];ages=[];covariances=[];sample_calls=0
    for row in rows:
        now=float(row['time_s']);age=float(row['launch_age_s'])
        require(math.isfinite(now) and math.isfinite(age),'Nonfinite native epoch')
        decision=int(row['decision']);tracked=int(row['tracking'])
        require(decision in (0,1) and tracked in (0,1),'Invalid native decision/tracking flag')
        point=tuple(float(row['point_'+axis]) for axis in ('x','y','z'))
        demand=tuple(float(row['demand_'+axis]) for axis in ('x','y','z'))
        require(all(math.isfinite(v) for v in point+demand),'Nonfinite actual sensor/control output')
        if decision:
            require(not decisions or age>decisions[-1]['age'],'Nonmonotonic native decision launch age')
            decisions.append(dict(age=age,point=point,demand=demand));tracking+=tracked
        else:sample_calls+=1
        if not row['measurement_s']:continue
        sample=float(row['measurement_s']);ready=float(row['processing_ready_s']);delivery=float(row['delivered_s'])
        require(all(math.isfinite(v) for v in (sample,ready,delivery)),'Nonfinite physical sensor chronology')
        require(sample<=ready<=delivery+1e-9 and delivery<=now+1e-9,'Invalid physical sensor chronology')
        require(abs(ready-sample-delay)<1e-8,'Processing delay differs from authored policy')
        require(-1e-9<=delivery-ready<decision_interval+1e-8,'Publication exceeds one decision quantization')
        if not decision:measurements.setdefault(row['instance'],set()).add(sample)
        delays.append(delivery-sample);ages.append(now-sample)
        if row['angular_variance_rad2']:
            covariance=float(row['angular_variance_rad2'])
            require(math.isfinite(covariance) and covariance>0,'Invalid modeled detector covariance');covariances.append(covariance)
    require(len(decisions)>1 and tracking>1 and delays and covariances,'Actual variant did not exercise sensing/guidance')
    intervals=[b-a for times in measurements.values() for a,b in zip(sorted(times),sorted(times)[1:])]
    require(intervals and all(v>=period-1e-8 for v in intervals),'Exposure cadence exceeds authored sampling rate')
    return dict(decision_samples=len(decisions),tracking_decisions=tracking,sensor_calls=sample_calls,
        unique_measurements=sum(map(len,measurements.values())),minimum_measurement_interval_s=min(intervals),
        maximum_measurement_interval_s=max(intervals),minimum_sample_age_s=min(ages),maximum_sample_age_s=max(ages),
        minimum_delivery_delay_s=min(delays),maximum_delivery_delay_s=max(delays),
        minimum_angular_variance_rad2=min(covariances)),decisions


def compare_decisions(reference,current,reference_summary,current_summary):
    """Compare actual traces on their common launch-age interval, without future interpolation."""
    require(reference and current,'Both native decision traces are required')
    epochs=[r['age'] for r in reference];pointing=[];demand=[];alignment=[];unavailable=0
    for row in current:
        if row['age']<epochs[0] or row['age']>epochs[-1]:continue
        prior=reference[bisect.bisect_right(epochs,row['age'])-1]
        alignment.append(row['age']-prior['age'])
        demand.append(math.dist(row['demand'],prior['demand']))
        a=row['point'];b=prior['point'];na=math.hypot(*a);nb=math.hypot(*b)
        if na>0 and nb>0:
            a=tuple(v/na for v in a);b=tuple(v/nb for v in b)
            cross=(a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])
            pointing.append(math.atan2(math.hypot(*cross),sum(x*y for x,y in zip(a,b))))
        else:unavailable+=1
    require(alignment,'Native cadence traces have no common decision launch-age interval')
    before=reference_summary['miss_m'];after=current_summary['miss_m']
    return dict(against_frame_skip=1,scope='Diagnostic decision quantization; stochastic exposures and trajectories can differ, not hardware accuracy or a convergence pass',
        alignment='Causal zero-order hold of the frameSkip1 decision at each coarser decision launch_age_s within the common recorded interval',
        common_launch_age_start_s=max(epochs[0],current[0]['age']),common_launch_age_end_s=min(epochs[-1],current[-1]['age']),
        decision_pairs=len(alignment),maximum_alignment_age_s=max(alignment),
        pointing_pairs=len(pointing),pointing_unavailable_pairs=unavailable,
        maximum_pointing_difference_rad=max(pointing) if pointing else None,
        rms_pointing_difference_rad=math.sqrt(sum(v*v for v in pointing)/len(pointing)) if pointing else None,
        maximum_demand_difference_m_s2=max(demand),rms_demand_difference_m_s2=math.sqrt(sum(v*v for v in demand)/len(demand)),
        reference_terminal=reference_summary['terminal'],current_terminal=current_summary['terminal'],
        terminal_equal=reference_summary['terminal']==current_summary['terminal'],reference_miss_m=before,current_miss_m=after,
        miss_difference_m=after-before if before is not None and after is not None else None)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--coverage',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
    a=p.parse_args();out=allowed(a.output).resolve();require(not out.exists(),'Fresh cadence evidence required')
    coverage=json.loads(allowed(a.coverage).read_text(encoding='utf-8'))
    expected={r['card'] for r in coverage['stores'] if r['class']=='aam'};require(len(expected)==24,'All24 models required')
    out.mkdir(parents=True);result=dict(schema='fsim.sim-m03.native-cadence.v1',passed=False,models=[],summaries=[],
        comparison_scope='Cross-rate diagnostics preserve observed outcomes; no additional physical hit or hardware-accuracy threshold')
    for preset in ('ucrt64-headless','ucrt64-release'):
        source=ROOT/'build'/preset/'validation/weapon_sensor_native';destination=out/preset;destination.mkdir()
        references={}
        for skip in (1,2,4):
            population,raw_summary,inputs=read_summary_shards(source,expected,skip)
            require(len(population)==24 and {r[0] for r in population}==expected,'Incomplete/duplicated cadence population')
            require(all(len(r)==9 for r in population),'Invalid native cadence summary shape')
            copied_summary=destination/('summary-skip'+str(skip)+'.csv');copied_summary.write_bytes(raw_summary)
            summary_binding=dict(path=str(copied_summary),sha256=sha(copied_summary))
            result['summaries'].append(dict(preset=preset,frame_skip=skip,source_shards=inputs,**summary_binding))
            summary_rows={r[0]:r for r in population}
            for card in sorted(expected):
                path=source/card/(card+'-skip'+str(skip)+'.csv');target=destination/path.name
                original_trace_hash=sha(path)
                raw=json.loads((ROOT/'assets/stores'/(card+'.json')).read_text(encoding='utf-8'))
                configuration=raw['component_models']['components']['seeker'];refs=configuration['parameter_refs']
                delay=raw['parameters'][refs['processing_delay_s']]['value'];period=raw['parameters'][refs['sample_period_s']]['value']
                native_summary=summary_rows[card]
                require(native_summary[1]==configuration['implementation_id'] and int(native_summary[2])==skip,'Actual native identity/cadence differs')
                miss=float(native_summary[8]) if native_summary[8] else None
                require(miss is None or math.isfinite(miss),'Invalid native terminal miss')
                terminal=dict(terminal=native_summary[7],miss_m=miss)
                decision_interval=skip/120
                with path.open(newline='',encoding='utf-8') as stream:
                    metrics,decisions=analyze_trace(csv.DictReader(stream),period,delay,decision_interval)
                require(int(native_summary[4])==metrics['sensor_calls'] and int(native_summary[5])==metrics['decision_samples'] and
                        int(native_summary[6])==metrics['tracking_decisions'],'Native summary and trace counters differ')
                shutil.copy2(path,target);require(sha(target)==original_trace_hash==sha(path),'Native trace copy changed bytes')
                result_row=dict(card=card,preset=preset,frame_skip=skip,decision_interval_s=decision_interval,
                    authored_exposure_s=period,authored_processing_s=delay,**metrics,**terminal,
                    summary=summary_binding,trace=dict(path=str(target),sha256=sha(target)),passed=True)
                if skip==1:references[card]=(decisions,terminal,result_row)
                else:
                    reference_decisions,reference_terminal,reference_metrics=references[card]
                    result_row['versus_frame_skip1']=compare_decisions(reference_decisions,decisions,reference_terminal,terminal)
                    for metric in ('maximum_sample_age_s','maximum_measurement_interval_s','maximum_delivery_delay_s'):
                        result_row['versus_frame_skip1'][metric+'_difference']=metrics[metric]-reference_metrics[metric]
                result['models'].append(result_row)
    reconcile_population({(card,preset,skip) for card in expected for preset in ('ucrt64-headless','ucrt64-release') for skip in (1,2,4)},
                         result['models'],('card','preset','frame_skip'))
    require(len(result['models'])==144,'Incomplete two-preset24-model3-cadence matrix')
    result['passed']=True;(out/'cadence.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print('144 actual native variant/cadence/preset traces reconciled')


if __name__=='__main__':main()
