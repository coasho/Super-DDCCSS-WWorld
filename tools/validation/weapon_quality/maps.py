"""Exact offline map input provenance; no downloads or simulation execution."""
import hashlib
import json
import os
from pathlib import Path, PurePosixPath, PureWindowsPath
import re
import stat

SCHEMA='fsim.sim-m03.maps.v1'
POLICY='offline-frozen-exact-tree'
EXCLUDED={'.codex','.git'}
REQUIRED={
    'terrain':'s3.amazonaws.com/elevation-tiles-prod/terrarium/',
    'imagery':'server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/'}


def canonical(value):
    return (json.dumps(value,indent=2,sort_keys=True,ensure_ascii=False,allow_nan=False)+'\n').encode('utf-8')


def safe(path):
    path=Path(path)
    if any(part.casefold() in EXCLUDED for part in path.parts):raise ValueError('Excluded map provenance path')
    resolved=path.resolve()
    if any(part.casefold() in EXCLUDED for part in resolved.parts):raise ValueError('Excluded resolved map provenance path')
    return resolved


def identity(name):
    parts=name.split('/')
    if (not name or '\\' in name or ':' in name or PurePosixPath(name).is_absolute() or PureWindowsPath(name).drive or
            PurePosixPath(name).as_posix()!=name or any(p in ('','.','..') or p.casefold() in EXCLUDED for p in parts) or
            any(ord(c)<32 or ord(c)==127 for c in name) or any(p!=p.rstrip(' .') or
                re.fullmatch(r'(CON|PRN|AUX|NUL|CONIN\$|CONOUT\$|COM[1-9\u00b9\u00b2\u00b3]|LPT[1-9\u00b9\u00b2\u00b3])',p.split('.')[0],re.I) for p in parts)):
        raise ValueError('Unsafe map inventory identity')
    return name


def linked(value):
    return stat.S_ISLNK(value.st_mode) or bool(getattr(value,'st_file_attributes',0)&0x400)


def fingerprint(value):
    return value.st_dev,value.st_ino,value.st_mode,value.st_size,value.st_mtime_ns,value.st_ctime_ns


def snapshot(root):
    root=safe(root)
    if not root.is_dir():raise ValueError('Existing exact map root required')
    files=[];seen=set();pending=[root]
    required={kind:dict(prefix=prefix,files=0,bytes=0) for kind,prefix in REQUIRED.items()}
    while pending:
        directory=pending.pop()
        with os.scandir(directory) as entries:
            # Exclude names before asking their type or traversing them.
            children=sorted((e for e in entries if e.name.casefold() not in EXCLUDED),key=lambda e:e.name)
        for entry in children:
            path=Path(entry.path);name=identity(path.relative_to(root).as_posix())
            key=name.casefold()
            if key in seen:raise ValueError('Case-colliding map inventory identity')
            seen.add(key);before=entry.stat(follow_symlinks=False)
            if linked(before):raise ValueError('Map links/reparse points are unsupported')
            if stat.S_ISDIR(before.st_mode):pending.append(path);continue
            if not stat.S_ISREG(before.st_mode):raise ValueError('Map special files are unsupported')
            # Windows DirEntry.stat exposes zero device/inode identifiers.
            # Compare fresh path stats on both sides of the read so real file
            # identity remains checked without comparing incompatible APIs.
            before=path.stat(follow_symlinks=False)
            if linked(before) or not stat.S_ISREG(before.st_mode):
                raise ValueError('Map input changed to a link or special file')
            digest=hashlib.sha256();amount=0;header=b''
            with path.open('rb') as stream:
                for block in iter(lambda:stream.read(1024*1024),b''):
                    if not header:header=block[:16]
                    digest.update(block);amount+=len(block)
            after=path.stat(follow_symlinks=False)
            if linked(after) or fingerprint(before)!=fingerprint(after) or amount!=before.st_size:
                raise ValueError('Map input changed while inventory was captured')
            files.append(dict(path=name,bytes=amount,sha256=digest.hexdigest()))
            for kind,prefix in REQUIRED.items():
                if not name.startswith(prefix) or path.suffix.lower() not in ('.png','.jpg','.jpeg'):continue
                valid=header.startswith(b'\x89PNG\r\n\x1a\n') if path.suffix.lower()=='.png' else header.startswith(b'\xff\xd8\xff')
                if not valid:raise ValueError('Required map tile is not a declared image input: '+name)
                required[kind]['files']+=1;required[kind]['bytes']+=amount
    files.sort(key=lambda row:row['path'])
    if not files or any(row['files']==0 or row['bytes']==0 for row in required.values()):
        raise ValueError('Nonzero actual terrain and satellite imagery map inputs required')
    return dict(schema=SCHEMA,passed=True,policy=POLICY,root=str(root),files=files,
        file_count=len(files),total_bytes=sum(row['bytes'] for row in files),required_inputs=required,
        inventory_sha256=hashlib.sha256(canonical(files)).hexdigest())


def verify_snapshot(root,receipt):
    if receipt.get('schema')!=SCHEMA or receipt.get('passed') is not True or receipt.get('policy')!=POLICY:
        raise ValueError('Complete exact offline map receipt required')
    if safe(receipt['root'])!=safe(root):raise ValueError('Supplied maps differ from the accepted map root')
    actual=snapshot(root)
    if actual['files']!=receipt.get('files') or actual['inventory_sha256']!=receipt.get('inventory_sha256'):
        raise ValueError('Accepted map inventory or bytes changed')
    return actual


def freeze(root,start,start_path):
    start_path=safe(start_path)
    raw=start_path.read_bytes()
    if json.loads(raw)!=start:raise ValueError('Initial map receipt changed during campaign')
    result=verify_snapshot(root,start)
    result.update(frozen=True,start_receipt=dict(path=str(start_path),sha256=hashlib.sha256(raw).hexdigest()),
        start_inventory_sha256=start['inventory_sha256'])
    return result


def verify_frozen(root,receipt_path):
    receipt_path=safe(receipt_path);receipt=json.loads(receipt_path.read_bytes())
    if receipt.get('frozen') is not True:raise ValueError('Frozen start/end map proof required')
    start_binding=receipt.get('start_receipt',{});start_path=safe(start_binding.get('path',''))
    if start_path!=receipt_path.parent/'maps-start.json':raise ValueError('Initial map proof belongs to another campaign')
    raw=start_path.read_bytes()
    if hashlib.sha256(raw).hexdigest()!=start_binding.get('sha256'):raise ValueError('Initial map receipt bytes changed')
    start=json.loads(raw)
    if (start.get('schema')!=SCHEMA or start.get('passed') is not True or start.get('policy')!=POLICY or
            safe(start['root'])!=safe(root) or start.get('files')!=receipt.get('files') or
            start.get('inventory_sha256')!=receipt.get('inventory_sha256') or
            receipt.get('start_inventory_sha256')!=receipt.get('inventory_sha256')):
        raise ValueError('Map inputs were not frozen across the actual campaign')
    verify_snapshot(root,receipt)
    return receipt


def verify_campaign(root,receipt_path,input_binding):
    """Join the final proof to the map inputs recorded before native execution."""
    if (not isinstance(input_binding,dict) or set(input_binding)!={'root','policy','start'} or
            not isinstance(input_binding.get('root'),str) or not input_binding['root'] or
            input_binding.get('policy')!=POLICY):
        raise ValueError('Original campaign map input binding is missing or malformed')
    start=input_binding.get('start')
    if (not isinstance(start,dict) or set(start)!={'path','sha256'} or
            not isinstance(start.get('path'),str) or not start['path'] or
            not isinstance(start.get('sha256'),str) or not re.fullmatch('[0-9a-f]{64}',start['sha256'])):
        raise ValueError('Original campaign map start binding is missing or malformed')
    receipt_path=safe(receipt_path)
    if safe(input_binding['root'])!=safe(root):
        raise ValueError('Accepted map root differs from original campaign map inputs')
    if safe(start['path'])!=receipt_path.parent/'maps-start.json':
        raise ValueError('Original campaign map start belongs to another input epoch')
    receipt=verify_frozen(root,receipt_path)
    if receipt.get('policy')!=input_binding['policy'] or receipt.get('start_receipt')!=start:
        raise ValueError('Final map proof differs from original campaign map start binding')
    return receipt
