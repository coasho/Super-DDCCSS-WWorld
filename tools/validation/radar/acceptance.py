"""Read actual native regression/performance receipts for engineering acceptance."""
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from contract import METHOD_TESTS, TEST_PATTERN


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def regression(path):
    data=json.loads(Path(path).read_text(encoding='utf-8'))
    if not data['passed'] or digest(data['junit'])!=data['junit_sha256']:
        raise ValueError('Failed or changed regression receipt')
    cases=ET.parse(data['junit']).getroot().findall('.//testcase')
    names=[row.get('name') for row in cases]
    selected=[row['name'] for row in data['inventory']['tests'] if re.search(TEST_PATTERN,row['name'])]
    if not cases or len(set(names))!=len(names) or set(names)!=set(selected) or len(names)!=data['executed']:
        raise ValueError('Missing, duplicate or extra dependency regression cases')
    if any(row.find('failure') is not None or row.find('error') is not None or row.find('skipped') is not None for row in cases):
        raise ValueError('Failed or skipped dependency regression')
    if not set(METHOD_TESTS).issubset(names):raise ValueError('Missing independent-method or boundary regression')
    return dict(executed=len(cases),elapsed_s=data['elapsed_s'],scope='Complete declared M01 dependency selector; not whole-project regression')


def engineering(args,installed,modules):
    result={name:regression(getattr(args,name)) for name in ('headless','release')}
    performance=json.loads(args.performance.read_text(encoding='utf-8'))
    normal=json.loads(args.normal.read_text(encoding='utf-8'))
    if not performance.get('passed') or not normal.get('passed') or normal.get('tests_run',0)<1:
        raise ValueError('Performance/normal launcher gate did not pass')
    required={'installed-sdk','installed-python','installed-consumer-configure','installed-consumer-build',
        'installed-sim_m01_cpp.exe','installed-sim_m01_c.exe','installed-native-identity','installed-python-fusion','installed-fleet'}
    labels=[row['label'] for row in installed['commands']]
    if len(labels)!=len(set(labels)) or not required.issubset(labels) or any(row['exit_code'] for row in installed['commands']):
        raise ValueError('Missing or failed fresh installation/consumer command')
    for item in installed['provenance']:
        if digest(item['source'])!=item['sha256'] or digest(item['installed'])!=item['sha256']:
            raise ValueError('Source or installed product changed since execution')
    for population in (performance['loaded_modules'],normal['loaded_modules']):
        observed={row['name']:row for row in population}
        if len(observed)!=len(population) or set(observed)!=set(modules) or any(observed[name]['sha256']!=row['sha256'] for name,row in modules.items()):
            raise ValueError('Different loaded native products across acceptance gates')
    cases=performance['cases']
    if len(cases)!=2 or [row['radar_on'] for row in cases]!=[False,True] or any(row['steps']!=600 or row['rejected_inputs'] or row['real_time_factor']<1 for row in cases) or cases[0]['fused_entities']!=0 or cases[1]['fused_entities']<=0:
        raise ValueError('Invalid native real-time/functional benchmark')
    return result
