"""Offline public-reference report; SDK/table residuals are deliberately absent."""
from html import escape
import json
from pathlib import Path


def render(report):
    from .reference_campaign import METRIC_UNITS
    rows=report['models'];qualified=[r for r in rows if r.get('observations')]
    complete=sum(r['status']=='calibrated' for r in rows)
    def number(value):return 'Unknown' if value is None else f'{value:.3g}%'
    def absolute(value):return 'Unknown' if value is None else f'{value:.4g}'
    body=[]
    for r in rows:
        metrics=sorted(set(r.get('training') or {})|set(r.get('validation') or {}))
        if not metrics:metrics=['No admissible calibration observations']
        details=[]
        for metric in metrics:
            tr=(r.get('training') or {}).get(metric,{})
            va=(r.get('validation') or {}).get(metric,{})
            interval=va.get('reference_reading_uncertainty_nrmse_interval_percent')
            band=f' (reading uncertainty: {number(interval[0])}–{number(interval[1])})' if interval else ''
            details.append(f'<tr><td>{escape(metric)} ({METRIC_UNITS.get(metric,"")})</td><td>{tr.get("count",0)}</td><td>{number(tr.get("nrmse_percent"))}</td><td>{absolute(tr.get("max_absolute_error"))}</td><td>{va.get("count",0)}</td><td>{number(va.get("nrmse_percent"))}{band}</td><td>{absolute(va.get("max_absolute_error"))}</td></tr>')
        refs={o['source']:o['source_record'] for o in r.get('observations',[])}
        for key in r.get('contextual_sources',[]):
            if key in report.get('sources',{}):refs.setdefault(key,report['sources'][key])
        sources=''.join(f'<li><a href="{escape(v["url"],quote=True)}">{escape(v["title"])}</a>: {escape(v["evidence"])}</li>' for v in refs.values()) or '<li>No qualifying numeric dataset found in this audit.</li>'
        audit=r.get('evidence_audit') or {}
        change=audit.get('retained_calibration_change',{})
        change_rows=[]
        for metric,before in change.get('baseline_validation',{}).items():
            after=(r.get('validation') or {}).get(metric,{}).get('nrmse_percent')
            change_rows.append('<tr><td>'+escape(metric)+'</td><td>'+number(before.get('nrmse_percent'))+
                '</td><td>'+number(after)+'</td></tr>')
        improvement=('<p><strong>Retained calibration change: original independent holdouts</strong></p><table><tr><th>Quantity</th><th>Before NRMSE</th><th>After NRMSE</th></tr>'+
            ''.join(change_rows)+'</table><p>'+escape(change.get('scope',''))+'</p>') if change_rows else ''
        parameter_audit=audit.get('parameter_audit',{})
        parameter_rows=''.join('<tr><td>'+escape(key)+'</td><td>'+escape(str(p['value']))+'</td><td>'+escape(p['basis'])+'</td><td>Unknown</td></tr>'
                               for key,p in parameter_audit.get('parameters',{}).items())
        reviewed=''.join('<li><a href="'+escape(v['url'],quote=True)+'">'+escape(v['title'])+'</a>; '+escape(v['locator'])+
                         '; conditions: '+escape(v['conditions'])+'; '+escape(v['decision'])+'</li>' for v in audit.get('source_reviews',[]))
        research=('<details><summary>Individual evidence audit and parameter status</summary><p>Search date: '+escape(audit.get('search_date','Unknown'))+
            '</p><ul>'+''.join('<li>'+escape(q)+'</li>' for q in audit.get('searches',[]))+'</ul><p>'+escape(audit.get('model_specific_analysis',''))+
            '</p><p>Parameter verification: <strong>'+escape(parameter_audit.get('verification_status','Unknown'))+'</strong></p>'+
            '<table><tr><th>Parameter</th><th>Retained value</th><th>Evidence basis</th><th>Matched error</th></tr>'+parameter_rows+'</table>'+
            '<ul>'+reviewed+'</ul><p>Estimated/simulated method and uncertainty: '+escape(json.dumps(audit.get('estimation',{}),ensure_ascii=True))+'</p></details>') if audit else ''
        issues=[escape(stage+': '+c['name']+' = '+str(c.get('value'))+' '+c.get('unit','')+'; expected '+str(c.get('expected','')))
                for stage,checks in r.get('retained_model_self_checks',{}).items() for c in checks if c['status']=='fail']
        self_checks='<p><strong>Retained source-bound model self-check failures (not external accuracy):</strong></p><ul>'+''.join('<li>'+s+'</li>' for s in issues)+'</ul>' if issues else ''
        body.append(f'''<article data-model="{escape(r['model'])}" data-calibrated="{'yes' if r in qualified else 'no'}">
<div class="model"><h2>{escape(r['model'].upper())}</h2><span>{escape(r['status'])}</span></div><p>{escape(r.get('round_outcome',''))}</p>{improvement}{self_checks}{research}
<table><thead><tr><th>Measured quantity (unit)</th><th>Fit points</th><th>Fit NRMSE</th><th>Fit max abs.</th><th>Holdout points</th><th>Holdout NRMSE</th><th>Holdout max abs.</th></tr></thead><tbody>{''.join(details)}</tbody></table>
<p>{escape(r.get('unknown_reason',''))}</p><p class="scope">{escape(r.get('validation_independence','No independent retest error is available.'))}</p>
<details><summary>Sources, conditions and measurement receipt</summary><ul>{sources}</ul>
<p>Every compared observation records its source page/table, requested conditions, actual SDK conditions and raw result in the downloadable JSON.</p>
<p>Matched full-equipment accuracy: <strong>Unknown</strong>. This report does not convert fit residuals into a fidelity score.</p>
<p>XML SHA-256: <code>{escape(r.get('xml_sha256','Not measured'))}</code></p></details></article>''')
    payload=json.dumps(dict(models=len(rows),calibrated=complete,complete=report.get('complete',False)))
    return f'''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Hangar · Public-data calibration errors</title><style>
*{{box-sizing:border-box}}body{{margin:0;background:#f1f5f8;color:#173348;font:15px/1.55 system-ui,sans-serif}}main{{max-width:1200px;margin:auto;padding:32px 24px}}h1{{font-size:32px;margin:8px 0}}h2{{font-size:21px;margin:0}}.eyebrow{{color:#087d83;font-size:12px;letter-spacing:2px;font-weight:750}}.warning{{border-left:4px solid #b97f16;background:#fff5df;padding:16px;margin:20px 0}}.metrics{{display:flex;gap:16px;flex-wrap:wrap}}.card,article{{background:white;border:1px solid #d6e1e8;border-radius:9px;padding:20px;margin:12px 0}}.card{{flex:1;min-width:180px}}.card strong{{display:block;font-size:30px}}article p,.scope{{color:#4b6475}}.model{{display:flex;justify-content:space-between;align-items:center;gap:16px}}.model span{{font-size:12px;color:#85530b;background:#fff1d6;padding:4px 8px;border-radius:5px}}table{{width:100%;border-collapse:collapse;margin:16px 0;font-size:13px}}th,td{{text-align:left;padding:9px;border-bottom:1px solid #e1e8ed}}th{{background:#f4f7f9}}a{{color:#086e7a}}code{{overflow-wrap:anywhere}}input,select{{font:inherit;padding:10px;border:1px solid #adc2cf;border-radius:5px;margin:8px 8px 8px 0;background:white}}input{{width:260px}}details{{font-size:13px}}.toolbar{{position:sticky;top:0;background:#f1f5f8;padding:8px 0;z-index:1}}.foot{{font-size:12px;color:#4b6475}}[hidden]{{display:none}}@media(max-width:650px){{main{{padding:20px 12px}}article{{overflow:auto}}h1{{font-size:27px}}.model{{align-items:start}}}}@media print{{.toolbar{{display:none}}article{{break-inside:avoid}}}}
</style><main><div class="eyebrow">HANGAR / SIM-M02</div><h1>Public-data calibration errors</h1>
<p>Actual SDK measurements versus traceable external references. Calibration and untouched holdouts are shown separately.</p>
<div class="metrics"><div class="card">Authored models accounted for<strong>{len(rows)}</strong></div><div class="card">Scoped calibration and retest<strong>{complete}</strong></div><div class="card">Still missing calibration/retest<strong>{len(rows)-complete}</strong></div></div>
<div class="warning"><strong>All-model calibration is {'complete' if report.get('complete') else 'NOT complete'}.</strong> Missing reference data never count as successful calibration. Known errors apply only to the stated quantities and conditions. No model has full-envelope matched-equipment qualification here.</div>
<p><a href="hangar-vlm-errors-status.csv">One row per model: calibrated / blocked / historical</a> · <a href="hangar-vlm-errors.csv">Metric CSV</a> · <a href="hangar-vlm-errors.json">Source and measurement JSON</a></p>
<div class="toolbar"><label>Model <input id="search" type="search" placeholder="F16C, MK82, UH60..."></label><label>Evidence <select id="filter"><option value="all">All models</option><option value="yes">Has measured reference comparisons</option><option value="no">Still missing calibration data</option></select></label><span id="count" role="status"></span></div>
{''.join(body)}<p class="foot">NRMSE = 100 × ||SDK − reference|| / ||reference||. Different quantities are never averaged into one accuracy score. Graph-derived observations carry a reading uncertainty. Generated by the common SDK calibration campaign; no SDK/table consistency values are included.</p>
<script type="application/json" id="summary">{payload}</script><script>
const input=document.getElementById('search'),filter=document.getElementById('filter'),articles=[...document.querySelectorAll('article')];function update(){{let n=0;for(const article of articles){{article.hidden=!(article.dataset.model.includes(input.value.trim().toLowerCase())&&(filter.value==='all'||article.dataset.calibrated===filter.value));if(!article.hidden)n++}}document.getElementById('count').textContent=n+' / '+articles.length+' shown'}}input.addEventListener('input',update);filter.addEventListener('change',update);update();
</script></main></html>'''


def publish_public(report):
    from . import pipeline
    from .reference_campaign import publish
    from .files import atomic_write_text
    folder=Path(pipeline.repo_root())/'docs/data';folder.mkdir(parents=True,exist_ok=True)
    publish(report,folder/'hangar-vlm-errors.json')
    atomic_write_text(folder/'hangar-vlm-errors.html',render(report))
