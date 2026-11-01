"""Publish the complete roster's measured SDK errors without inventing accuracy.

Inputs must come from vlm_sdk.py, reference_sdk.py and hangar.calibration.
The existing 0.002 wiring tolerance belongs to the SDK report, not a new gate.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import tomllib

ROOT = Path(__file__).resolve().parents[3]
AXES = ('CX', 'CY', 'CZ', 'Cl', 'Cm', 'Cn')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def inventory(sdk, calibration, reference):
    roster = {p.name: p / (p.name + '.toml') for p in (ROOT / 'aircraft').iterdir()
              if p.name.casefold() != '.codex' and (p / (p.name + '.toml')).is_file()}
    if set(sdk['models']) != set(roster):
        raise ValueError('SDK report does not cover the complete authored roster')
    fits = {r['model']: r for r in calibration['models']}
    if set(fits) != set(roster):
        raise ValueError('Calibration inventory does not cover the complete roster')
    for key, relative in (('candidate', 'aircraft/f16c/f16c.xml'),
                          ('reference', 'third_party/jsbsim/aircraft/f16/f16.xml'),
                          ('mk82', 'aircraft/stores/mk82/mk82.xml')):
        if reference['source_sha256'][key] != digest(ROOT / relative):
            raise ValueError('Reference comparison is stale: ' + key)
    rows = []
    for name, source in sorted(roster.items()):
        spec = tomllib.loads(source.read_text(encoding='utf-8'))
        store = bool(spec['aircraft'].get('card'))
        product = ROOT / 'aircraft' / ('stores/' + name if store else name) / (name + '.xml')
        row = sdk['models'][name]
        if row.get('source_sha256') != digest(source) or row.get('xml_sha256') != digest(product):
            raise ValueError('SDK comparison is stale or incomplete: ' + name)
        cal = source.parent / 'calibration.toml'
        if row.get('calibration_sha256') != digest(cal):
            raise ValueError('SDK calibration provenance is stale: ' + name)
        result = dict(model=name, kind='store' if store else spec['aircraft'].get('kind',spec['aircraft'].get('category', 'fixed-wing')),
                      sdk_status=row['status'], sdk_conditions=row['conditions'],
                      **{'sdk_max_abs_' + axis: row['max_coefficient_error'][axis] for axis in AXES},
                      calibration_status=fits[name]['status'],
                      matched_equipment_error_percent=None,
                      reference_nrmse_percent=None, reference_scope='No matched independent equipment dataset',
                      uncontrolled_motor_speed_fraction=None, uncontrolled_motor_note='',
                      flight_calibration_sha256=None,performance_selfcheck_failures='')
        if not store:
            record=source.parent/'flight-calibration.json'
            if record.is_file():
                result['flight_calibration_sha256']=digest(record)
                data=json.loads(record.read_text(encoding='utf-8'))
                failures=[r for r in data['outputs'].get('performance',{}).get('checks',[]) if r['status']=='fail']
                result['performance_selfcheck_failures']='; '.join(
                    r['name']+': '+str(r['value'])+' '+r.get('unit','')+'; expected '+r.get('expected','') for r in failures)
        if name in ('f16c', 'mk82'):
            comparison = reference['f16' if name == 'f16c' else 'mk82']
            result['reference_nrmse_percent'] = comparison['nrmse_percent']
            result['reference_scope'] = comparison['scope']
        if spec.get('motor'):
            flight = json.loads((source.parent / 'out/fly.json').read_text(encoding='utf-8'))
            result['uncontrolled_motor_speed_fraction'] = flight.get('legacy_speed_fraction')
            result['uncontrolled_motor_note'] = flight.get('legacy_speed_fraction_status', '')
        rows.append(result)
    return dict(schema='fsim.vlm-error-inventory.v1', sdk_executed=True,
                native_library=sdk['native_library'], coefficient_tolerance=sdk['coefficient_tolerance'],
                note='Absolute normalized coefficient wiring errors are not real-world accuracy percentages. '
                     'Reference NRMSE uses unmatched variants/geometries; unknown equipment errors remain null.',
                models=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sdk', type=Path, required=True)
    parser.add_argument('--calibration', type=Path, required=True)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--csv', type=Path, required=True)
    parser.add_argument('--html', type=Path, help='Optional self-contained interactive report')
    args = parser.parse_args()
    public = ROOT/'docs/data/hangar-vlm-errors'
    if args.csv.resolve()==public.with_suffix('.csv').resolve() or (args.html and args.html.resolve()==public.with_suffix('.html').resolve()):
        parser.error('The public error inventory is reserved for external-reference errors. Use a separate wiring filename here; regenerate the public report with hangar calibrate --model all.')
    read = lambda path: json.loads(path.read_text(encoding='utf-8'))
    report = inventory(read(args.sdk), read(args.calibration), read(args.reference))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    args.csv.parent.mkdir(parents=True, exist_ok=True)
    with args.csv.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(report['models'][0]))
        writer.writeheader()
        writer.writerows(report['models'])
    if args.html:
        template = Path(__file__).with_name('error_dashboard.html').read_text(encoding='utf-8')
        # Escape HTML delimiters so embedded report strings cannot close the data script.
        payload = json.dumps(report, allow_nan=False).replace('&', '\\u0026').replace('<', '\\u003c').replace('>', '\\u003e')
        args.html.parent.mkdir(parents=True, exist_ok=True)
        args.html.write_text(template.replace('REPORT_DATA_PLACEHOLDER', payload), encoding='utf-8')
    print(json.dumps(dict(models=len(report['models']),
                          passed=sum(r['sdk_status'] == 'passed' for r in report['models']))))
    return int(any(r['sdk_status'] != 'passed' for r in report['models']))


if __name__ == '__main__':
    raise SystemExit(main())
