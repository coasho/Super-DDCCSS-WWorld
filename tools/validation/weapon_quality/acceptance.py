"""Fail-closed admission of complete, current, hash-bound M03 evidence.

These checks reconcile evidence; they never substitute for executing a native
function or comparing its result against an independent physical reference.
"""
import hashlib
import math
import re
from pathlib import Path, PurePosixPath, PureWindowsPath


def allowed(path):
    path=Path(path)
    if any(part.casefold()=='.codex' for part in path.parts):
        raise ValueError('Excluded path')
    return path


def verify_artifact(path,expected):
    path=allowed(path)
    if not isinstance(expected,str) or not re.fullmatch('[0-9a-f]{64}',expected):
        raise ValueError('A complete SHA-256 binding is required')
    if not path.is_file():raise ValueError('Missing evidence artifact: '+str(path))
    actual=hashlib.sha256(path.read_bytes()).hexdigest()
    if actual!=expected:raise ValueError('Evidence bytes changed: '+str(path))
    return actual


def reconcile_inventory(expected,executed,failed,skipped):
    if not expected or len(expected)!=len(set(expected)):
        raise ValueError('A nonzero unique discovered selection is required')
    if len(executed)!=len(expected) or set(executed)!=set(expected):
        raise ValueError('Executed tests do not exactly match the discovered selection')
    if failed or skipped:raise ValueError('Failed or unexecuted selected tests')
    return len(executed)


def reconcile_population(expected,rows,fields):
    """Reject missing/duplicate identities before a map can discard a row."""
    expected=set(expected)
    if not expected or not fields:raise ValueError('A nonzero declared population and identity are required')
    identities=[]
    for row in rows:
        if any(field not in row for field in fields):raise ValueError('Missing population identity field')
        key=tuple(row[field] for field in fields)
        identities.append(key[0] if len(key)==1 else key)
    if len(identities)!=len(expected) or len(set(identities))!=len(identities) or set(identities)!=expected:
        raise ValueError('Complete unique exact evidence population is required')
    return dict(zip(identities,rows))


def reconcile_source_review(manifest,review):
    expected={row['path']:row['sha256'] for row in manifest if not row['deleted']}
    if any(not isinstance(row,dict) or row.get('passed') is not True for row in review):
        raise ValueError('Every reviewed source or produced file must explicitly pass')
    actual={row['path']:row['sha256'] for row in review}
    if not expected or len(actual)!=len(review) or actual!=expected:
        raise ValueError('Review must cover every changed delivery file exactly')
    return len(actual)


def reconcile_world_health(run,raw):
    """Independently admit every observed native interval through the aftermath."""
    rows=raw.get('world_health',[])
    if not rows or raw.get('invalid_entities')!=[] or run.get('complete_world_finite') is not True:
        raise ValueError('Complete finite native world observations are required')
    if run.get('observed_world_states')!=len(rows):raise ValueError('World observation count differs')
    identities=set();times=[]
    for row in rows:
        if len(row)!=17 or type(row[1]) is not int or row[1]<=0 or not row[2] or row[3] is not False:
            raise ValueError('Malformed or diverged native body observation')
        if any(type(x) not in (int,float) or not math.isfinite(x) for x in (row[0],*row[4:])):
            raise ValueError('Nonfinite native body observation')
        if row[4]<0 or row[-1]<=0:raise ValueError('Invalid native speed or mass')
        key=(row[0],row[1])
        if key in identities:raise ValueError('Duplicate native body observation')
        identities.add(key)
        if not times or times[-1]!=row[0]:times.append(row[0])
    step=run['frame_skip']/120.;launch=run['launch_time_s'];end=launch+run['duration_s'];epsilon=1e-7
    if not (launch-epsilon<=times[0]<=launch+step+epsilon and end-epsilon<=times[-1]<end+step+epsilon):
        raise ValueError('Native world observations omit launch or final aftermath')
    if any(abs(b-a-step)>epsilon for a,b in zip(times,times[1:])):
        raise ValueError('A native aftermath interval was omitted or reordered')
    return len(rows)


class WorldHealthV1:
    """Referee-only observations; never a source of actor targeting or control."""
    def __init__(self):self.rows=[];self.invalid=set()

    def observe(self,world):
        for vehicle in world.vehicles():
            state=vehicle.state
            row=[world.time,vehicle.id,vehicle.name,bool(state.diverged),state.airspeed_true_ms,
                *state.position_ecef,*state.velocity_body_ms,*state.angular_rate_body_rad_s,
                state.alpha_rad,state.beta_rad,vehicle.mass_kg]
            if state.diverged or not all(math.isfinite(float(x)) for x in row[4:]):
                self.invalid.add(vehicle.id)
                row[4:]=[float(x) if math.isfinite(float(x)) else None for x in row[4:]]
            self.rows.append(row)


def canonical_product_key(name):
    """Read native Windows or POSIX receipt identities without rewriting them."""
    if not isinstance(name,str) or not name or ':' in name or PureWindowsPath(name).drive:
        raise ValueError('A safe relative generated-product identity is required')
    normalized=name.replace('\\','/')
    parts=normalized.split('/')
    if (PurePosixPath(normalized).is_absolute() or any(p in ('','.','..') or p.casefold() in ('.codex','.git') for p in parts) or
            any(ord(c)<32 or ord(c)==127 for c in normalized) or any(p!=p.rstrip(' .') or
                re.fullmatch(r'(CON|PRN|AUX|NUL|CONIN\$|CONOUT\$|COM[1-9\u00b9\u00b2\u00b3]|LPT[1-9\u00b9\u00b2\u00b3])',p.split('.')[0],re.I) for p in parts)):
        raise ValueError('Unsafe generated-product path')
    return normalized


def canonical_product_map(products):
    if not isinstance(products,dict) or not products:raise ValueError('A nonzero generated-product map is required')
    result={};seen=set()
    for name,digest in products.items():
        key=canonical_product_key(name)
        if key.casefold() in seen:raise ValueError('Generated-product identities collide after normalization')
        if not isinstance(digest,str) or not re.fullmatch('[0-9a-f]{64}',digest):raise ValueError('Invalid generated-product SHA-256')
        seen.add(key.casefold());result[key]=digest
    return result


def reconcile_model_products(rows,audits,installed_products):
    """Join every generated product and mass-audit digest to the installed epoch."""
    expected={};seen={}
    for row in rows:
        if not isinstance(row,dict):raise ValueError('A generated model record is required')
        name=row.get('name');directory=PurePosixPath(canonical_product_key(row.get('product_directory')))
        if canonical_product_key(name)!=name or '/' in name:raise ValueError('Invalid generated model identity')
        category=row.get('category')
        if category not in ('store','fixed-wing','rotorcraft'):raise ValueError('Unknown generated model category')
        # Visuals occupy aircraft/<name>, including stores whose flight XML
        # occupies aircraft/stores/<name>. Derive this roster independently of
        # both receipt maps: matching omissions must not certify stale bytes.
        required={name+'/'+name+'.glb',name+'/'+name+'.glb.manifest'}
        if category!='store':required.add(name+'/'+name+'.liquid-v1.json')
        model=row.get('normal_model')
        if not isinstance(model,dict) or model.get('regenerated') is not True:
            raise ValueError('Current regenerated normal-model record is required')
        checks=model.get('checks')
        if (not isinstance(checks,list) or not checks or any(not isinstance(check,dict) or
                not isinstance(check.get('name'),str) or not check['name'].strip() or
                check.get('status') not in ('pass','warn','info') for check in checks)):
            raise ValueError('Current nonfailed normal-model checks are required')
        declared=model.get('products')
        if not isinstance(declared,list) or not declared:
            raise ValueError('The exact normal-model product roster is required')
        identities=[canonical_product_key(relative) for relative in declared]
        if len({relative.casefold() for relative in identities})!=len(identities):
            raise ValueError('Normal-model product identities collide after normalization')
        if set(identities)!=required:
            raise ValueError('Normal-model products differ from the required category roster')
        products=canonical_product_map(row.get('products'))
        if not required.issubset(products):
            raise ValueError('A required normal-model product has no generated byte binding')
        audit=audits[name]
        primary=(directory/(name+'.xml')).as_posix()
        if products.get(primary)!=audit['xml_sha256']:
            raise ValueError('Primary audit differs from generated product')
        if category=='fixed-wing':
            damage=(directory/(name+'.damage.json')).as_posix()
            if products.get(damage)!=audit['damage_sha256']:
                raise ValueError('Damage audit differs from generated product')
        for relative,digest in products.items():
            if relative.casefold() in seen and seen[relative.casefold()]!=relative:
                raise ValueError('Generated-product identities collide across models')
            if relative in expected and expected[relative]!=digest:raise ValueError('Conflicting generated product')
            expected[relative]=digest;seen[relative.casefold()]=relative
    if not expected or expected!=canonical_product_map(installed_products):
        raise ValueError('Installed product epoch differs from audited generation')
    return len(expected)


def admit_models(population,rows):
    if not population or not isinstance(population,dict):raise ValueError('A declared population is required')
    names=[row.get('model') for row in rows]
    if len(names)!=len(set(names)) or set(names)!=set(population):
        raise ValueError('Complete unique current model coverage is required')
    for row in rows:
        name=row['model'];expected=population[name]
        if row.get('status') not in ('engineering-complete','engineering-complete-uncalibrated'):
            raise ValueError('Incomplete engineering model: '+name)
        if row.get('unclosed_gates')!=[]:raise ValueError('Unclosed engineering gates: '+name)
        components=row.get('components',[])
        if len(components)!=len(set(components)) or set(components)!=set(expected['components']):
            raise ValueError('Missing, duplicate or invented applicable component: '+name)
        gates=row.get('gates',{})
        if set(gates)!=set(expected['gates']):raise ValueError('Incomplete or changed gate population: '+name)
        for gate,result in gates.items():
            if result.get('passed') is not True or not result.get('evidence'):
                raise ValueError('Unproved gate: '+name+'/'+gate)
        calibrated=row.get('equipment_calibrated')
        if type(calibrated) is not bool:raise ValueError('Explicit separate equipment accuracy state required')
        if row['status']=='engineering-complete' and not calibrated:
            raise ValueError('Calibrated engineering label requires equipment evidence')
        if calibrated and not row.get('equipment_calibration_evidence'):
            raise ValueError('Measured equipment accuracy requires independent calibration evidence')
        if row['status']=='engineering-complete-uncalibrated' and calibrated:
            raise ValueError('Inconsistent calibration label')
    return len(rows)
