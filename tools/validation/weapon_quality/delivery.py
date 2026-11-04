"""Exact HEAD + reviewed binary patch replay, then deterministic delivery packaging.

prepare --allowlist reviewed.json --output build/architecture-reviews/m03-prepare
        --export D:/ENV/tmp/m03-source
package --receipt <prepare.json> --report <completed.json> --artifact evidence=<tree>
        --sdk <installed-prefix> --maps <map-tree> --output <fresh-package-directory>

No real-index staging, branches, worktrees, native execution or recursive deletion.
All source blobs bypass Git line-ending/encoding filters, including CRLF changes.
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path, PurePosixPath, PureWindowsPath
import re
import stat
import subprocess
import tarfile
from maps import verify_campaign as verify_campaign_maps
from acceptance import canonical_product_key, canonical_product_map


ROOT = Path(__file__).resolve().parents[3]
ENV = Path('D:/ENV')
EXCLUDED = {'.codex', '.git'}
PATHSPECS = ('.', ':(exclude,icase,glob).codex/**', ':(exclude,icase,glob)**/.codex/**')
VERIFY_SCRIPT = '''"""Verify declared delivery bytes using only Python's standard library."""
import hashlib,json
from pathlib import Path,PurePosixPath
root=Path(__file__).resolve().parent
manifest=root/'manifest.json'
data=json.loads(manifest.read_text(encoding='utf-8'))
for row in data['files']:
    name=PurePosixPath(row['path'])
    if name.is_absolute() or any(p in ('.','..') or p.casefold() in ('.git','.codex') for p in name.parts):
        raise ValueError('Unsafe manifest identity')
    path=root/name
    probe=path
    while probe!=root:
        if probe.is_symlink() or (probe.exists() and getattr(probe.stat(),'st_file_attributes',0)&0x400):
            raise ValueError('Links are not delivery artifacts')
        probe=probe.parent
    if not path.is_file() or path.stat().st_size!=row['bytes']:
        raise ValueError('Missing or resized artifact: '+str(name))
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1048576),b''):h.update(block)
    if h.hexdigest()!=row['sha256']:raise ValueError('Changed artifact: '+str(name))
print(json.dumps(dict(passed=True,files=len(data['files']),manifest_sha256=hashlib.sha256(manifest.read_bytes()).hexdigest())))
'''


def sha(data):
    return hashlib.sha256(data).hexdigest()


def json_bytes(value):
    return (json.dumps(value, indent=2, sort_keys=True, ensure_ascii=False, allow_nan=False) + '\n').encode('utf-8')


def save(path, value):
    Path(path).write_bytes(json_bytes(value))


def relative_name(name):
    if not isinstance(name, str) or not name or '\\' in name:
        raise ValueError('Use a nonempty POSIX relative file path')
    p = PurePosixPath(name)
    if (p.is_absolute() or PureWindowsPath(name).drive or ':' in name or
            name != p.as_posix() or any(x in ('', '.', '..') or x.casefold() in EXCLUDED for x in name.split('/')) or
            any(ord(x) < 32 or ord(x) == 127 for x in name) or
            any(part != part.rstrip(' .') or re.fullmatch(r'(CON|PRN|AUX|NUL|CONIN\$|CONOUT\$|COM[1-9\u00b9\u00b2\u00b3]|LPT[1-9\u00b9\u00b2\u00b3])',
                part.split('.')[0], flags=re.IGNORECASE) for part in name.split('/'))):
        raise ValueError('Unsafe/excluded relative path: ' + name)
    return name


def safe_path(path, roots):
    p = Path(path)
    if any(x.casefold() in EXCLUDED for x in p.parts):
        raise ValueError('Excluded filesystem path')
    p = p.resolve()
    if any(x.casefold() in EXCLUDED for x in p.parts) or not any(p.is_relative_to(Path(r).resolve()) for r in roots):
        raise ValueError('Path is outside explicitly allowed project/environment roots: ' + str(p))
    return p


def is_link(path):
    s = path.lstat()
    return stat.S_ISLNK(s.st_mode) or bool(getattr(s, 'st_file_attributes', 0) & 0x400)


def regular_file(path, anchor):
    p = path
    while p != anchor:
        if p.exists() and is_link(p):
            raise ValueError('Links/reparse points are not source or artifact files: ' + str(p))
        p = p.parent
        if not p.is_relative_to(anchor):
            raise ValueError('File escaped its declared tree')
    if not path.is_file() or is_link(path):
        raise ValueError('A regular file is required: ' + str(path))


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def git_env(index=None):
    result = {k: v for k, v in os.environ.items() if not k.upper().startswith('GIT_')}
    result.update(GIT_CONFIG_NOSYSTEM='1', GIT_CONFIG_GLOBAL=os.devnull, GIT_ATTR_NOSYSTEM='1', GIT_OPTIONAL_LOCKS='0',
                  TMP='D:/ENV/tmp', TEMP='D:/ENV/tmp', TORCH_EXTENSIONS_DIR='D:/ENV/tmp')
    if index is not None:
        result['GIT_INDEX_FILE'] = str(index)
    return result


def git_command(*args):
    return ['git', '-c', 'core.autocrlf=false', '-c', 'core.safecrlf=false', '-c', 'core.attributesfile=' + os.devnull,
            '-c', 'color.ui=false', *map(str, args)]


def git(root, *args, index=None, data=None):
    return subprocess.run(git_command(*args), cwd=root, env=git_env(index), input=data,
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True).stdout


def index_tree(root, index):
    raw = git(root, 'ls-files', '--stage', '-z', '--', *PATHSPECS, index=index)
    result = {}
    for entry in raw.split(b'\0'):
        if not entry:
            continue
        meta, name = entry.split(b'\t', 1)
        mode, oid, stage = meta.decode('ascii').split()
        name = relative_name(name.decode('utf-8'))
        if stage != '0' or mode not in ('100644', '100755'):
            raise ValueError('Ordinary source export requires resolved regular blobs; unsupported entry: ' + name)
        result[name] = {'mode': mode, 'oid': oid}
    if len({name.casefold() for name in result}) != len(result):
        raise ValueError('HEAD has case-colliding source identities')
    return result


def export_head(root, tree, export):
    """Read exact immutable Git blob bytes, never checkout/archive attribute filters."""
    process = subprocess.Popen(git_command('cat-file', '--batch'), cwd=root, env=git_env(),
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        for name, row in sorted(tree.items()):
            destination = export / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            process.stdin.write((row['oid'] + '\n').encode('ascii'))
            process.stdin.flush()
            header = process.stdout.readline().decode('ascii').split()
            if len(header) != 3 or header[0] != row['oid'] or header[1] != 'blob':
                raise ValueError('Git did not return the expected source blob: ' + name)
            remaining = int(header[2])
            with destination.open('xb') as stream:
                while remaining:
                    block = process.stdout.read(min(1024 * 1024, remaining))
                    if not block:
                        raise ValueError('Truncated Git source blob: ' + name)
                    stream.write(block)
                    remaining -= len(block)
            if process.stdout.read(1) != b'\n':
                raise ValueError('Malformed Git batch separator')
            destination.chmod(0o755 if row['mode'] == '100755' else 0o644)
        process.stdin.close()
        if process.wait(timeout=30):
            raise RuntimeError('Git source export failed: ' + process.stderr.read().decode('utf-8', errors='replace'))
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=30)
        for stream in (process.stdin, process.stdout, process.stderr):
            if stream is not None: stream.close()


def read_allowlist(path):
    raw = path.read_bytes()
    names = json.loads(raw.decode('utf-8-sig'))
    if not isinstance(names, list) or not names:
        raise ValueError('The reviewed allowlist must be a nonempty JSON array')
    names = [relative_name(name) for name in names]
    if len({n.casefold() for n in names}) != len(names):
        raise ValueError('Duplicate/case-colliding allowlist paths')
    return sorted(names), sha(raw)


def prepare(root, allowlist, output, export, temporary_root=ENV / 'tmp'):
    root = Path(root).resolve()
    temporary_root = Path(temporary_root).resolve()
    roots = (root, temporary_root)
    allowlist = safe_path(allowlist, roots)
    output = safe_path(output, roots)
    export = safe_path(export, (temporary_root,))
    if export.parent != temporary_root or len(str(export)) > 96:
        raise ValueError('Use a short fresh direct child of D:/ENV/tmp for the ordinary source export')
    if output.exists() or export.exists() or output.is_relative_to(export) or export.is_relative_to(output):
        raise ValueError('Fresh separate output and ordinary export directories are required')
    # A .git file denotes a linked worktree, explicitly outside this workflow.
    if not (root / '.git').is_dir() or is_link(root / '.git'):
        raise ValueError('Use the original ordinary main checkout, never a worktree')
    if git(root, 'branch', '--show-current').decode().strip() != 'main':
        raise ValueError('The current branch must be main')
    head = git(root, 'rev-parse', '--verify', 'HEAD').decode().strip()
    real_index = root / '.git/index'
    index_before = digest(real_index)
    names, allowlist_sha = read_allowlist(allowlist)
    output.mkdir(parents=True)
    index = output / 'private-index'
    entries = []
    try:
        git(root, 'read-tree', head, index=index)
        head_tree = index_tree(root, index)
        updates = []
        for name in names:
            path = root / name
            previous = head_tree.get(name)
            if not path.exists():
                if previous is None:
                    raise ValueError('Allowlisted file is absent from both HEAD and workspace: ' + name)
                updates.append(b'0 ' + b'0' * len(head) + b'\t' + name.encode('utf-8') + b'\0')
                entries.append({'path': name, 'deleted': True, 'head_oid': previous['oid']})
                continue
            regular_file(path, root)
            content = path.read_bytes()
            oid = git(root, 'hash-object', '-w', '--no-filters', '--stdin', data=content).decode().strip()
            mode = previous['mode'] if previous else '100644'
            updates.append((mode + ' ' + oid + '\t' + name).encode('utf-8') + b'\0')
            entries.append({'path': name, 'deleted': False, 'bytes': len(content), 'sha256': sha(content), 'oid': oid,
                            'mode': mode, 'status': 'added' if previous is None else 'unchanged' if oid == previous['oid'] else 'modified'})
        git(root, 'update-index', '-z', '--index-info', index=index, data=b''.join(updates))
        patch = output / 'sim-m03.patch'
        # The private index starts at HEAD and changes ONLY reviewed entries.
        # Diff that immutable pair using bounded exclusion pathspecs instead of
        # putting the complete fleet's file identities on a Windows command line.
        common = ('--cached', '--no-renames', '--no-ext-diff', '--no-textconv', head, '--', *PATHSPECS)
        changed = git(root, 'diff', '--name-only', '-z', *common, index=index).split(b'\0')
        changed = {name.decode('utf-8') for name in changed if name}
        expected = {row['path'] for row in entries if row.get('deleted') or row['status'] != 'unchanged'}
        if changed != expected:
            raise ValueError('Private patch differs from the exact reviewed file population')
        patch.write_bytes(git(root, 'diff', '--binary', '--full-index', *common, index=index))
        (output / 'patch-stat.txt').write_bytes(git(root, 'diff', '--stat', *common, index=index))
        (output / 'patch-files.txt').write_bytes(git(root, 'diff', '--name-status', *common, index=index))
        save(output / 'allowlist.json', names)
        export.mkdir()
        export_head(root, head_tree, export)
        if patch.stat().st_size:
            env = git_env()
            env['GIT_CEILING_DIRECTORIES'] = str(export.parent)
            for check in (['--check'], []):
                subprocess.run(git_command('apply', '--binary', '--whitespace=nowarn', *check, str(patch)),
                               cwd=export, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True)
        tree = index_tree(root, index)
        snapshot = []
        for name, row in sorted(tree.items()):
            path = export / name
            regular_file(path, export)
            snapshot.append({'path': name, 'mode': row['mode'], 'oid': row['oid'], 'bytes': path.stat().st_size, 'sha256': digest(path)})
        failures = []
        for row in entries:
            path = export / row['path']
            if row['deleted']:
                if path.exists() or (root / row['path']).exists():
                    failures.append(row['path'])
            elif not path.is_file() or digest(path) != row['sha256'] or digest(root / row['path']) != row['sha256']:
                failures.append(row['path'])
        unchanged = digest(real_index) == index_before
        if not unchanged or git(root, 'rev-parse', 'HEAD').decode().strip() != head or failures:
            raise ValueError('Source/index changed or exact-byte replay failed: ' + repr(failures))
        save(output / 'source-files.json', snapshot)
        receipt = {'schema': 'fsim.sim-m03.patch-replay.v1', 'passed': True, 'base_head': head,
                   'source_root': str(root), 'source_export': str(export), 'export_is_git_worktree': False,
                   'real_index_unchanged': unchanged, 'real_index_sha256': index_before, 'allowlist_input_sha256': allowlist_sha,
                   'allowlist': names, 'files': entries, 'patch_sha256': digest(patch),
                   'source_manifest_sha256': digest(output / 'source-files.json'), 'replay_byte_failures': []}
        save(output / 'prepare.json', receipt)
        return receipt
    finally:
        # Only this invocation's one explicitly named private index is removed.
        if index.exists():
            index.unlink()
        if digest(real_index) != index_before:
            raise RuntimeError('The real Git index changed during preparation')


def walk_files(root):
    pending = [root]
    while pending:
        directory = pending.pop()
        with os.scandir(directory) as stream:
            children = sorted((entry for entry in stream if entry.name.casefold() not in EXCLUDED), key=lambda e: e.name)
        for entry in children:
            path = Path(entry.path)
            if is_link(path):
                raise ValueError('Artifact links/reparse points are unsupported: ' + str(path))
            if entry.is_dir(follow_symlinks=False):
                pending.append(path)
            elif entry.is_file(follow_symlinks=False):
                yield path
            else:
                raise ValueError('Artifact special files are unsupported: ' + str(path))


def completed_binding(receipt_path, receipt, report, sdk, roots):
    if (report.get('schema') != 'fsim.sim-m03.complete-acceptance.v1' or
            report.get('passed') is not True or report.get('engineering_acceptance') is not True):
        raise ValueError('Completed engineering acceptance report required')
    export = safe_path(receipt['source_export'], roots)
    if safe_path(report['source_root'], roots) != export:
        raise ValueError('Completed report belongs to another source export')
    def bound(binding):
        path = safe_path(binding['path'], roots)
        regular_file(path, next(Path(r).resolve() for r in roots if path.is_relative_to(Path(r).resolve())))
        if digest(path) != binding['sha256']:
            raise ValueError('Bound acceptance artifact changed: ' + str(path))
        return path, json.loads(path.read_text(encoding='utf-8-sig'))
    manifest_path, _ = bound(report['source_manifest'])
    if manifest_path != receipt_path or report['source_manifest']['sha256'] != digest(receipt_path):
        raise ValueError('Report is not bound to this exact preparation receipt')
    campaign_path, campaign = bound(report['campaign'])
    if (campaign.get('schema') != 'fsim.sim-m03.complete-campaign.v1' or
            campaign.get('automated_passed') is not True or
            safe_path(campaign['source_root'], roots) != export or safe_path(campaign['prefix'], roots) != sdk):
        raise ValueError('Campaign source/prefix or acceptance mismatch')
    installed_path, installed = bound(report['installed_consumer'])
    if installed_path != campaign_path.parent / 'installed.json':
        raise ValueError('Final report installed receipt is outside its campaign')
    if (installed.get('schema') != 'fsim.sim-m03.installed-consumers.v1' or
            installed.get('passed') is not True or safe_path(installed['prefix'], roots) != sdk or
            not installed.get('provenance')):
        raise ValueError('Current installed consumer provenance is required')
    producer_path, producer = bound(installed['producer'])
    if (producer_path != safe_path(campaign['production']['path'], roots) / 'installed-production-manifest.json' or
            producer.get('schema') != 'fsim.sim-m02.production-install.v1' or
            safe_path(producer['prefix'], roots) != sdk or not producer.get('products') or
            producer.get('product_count') != len(producer['products'])):
        raise ValueError('Installed producer must belong to this campaign')
    if digest(sdk / 'bin/libfsim.dll') != producer['native_dll_sha256']:
        raise ValueError('Installed native DLL differs from same-run producer')
    for name, expected in canonical_product_map(producer['products']).items():
        regular_file(sdk / 'share/flightsim/aircraft' / name, sdk)
        if digest(sdk / 'share/flightsim/aircraft' / name) != expected:
            raise ValueError('Installed generated product differs from same-run producer')
    seen = set()
    for row in installed['provenance']:
        current = safe_path(row['installed'], roots)
        source = safe_path(row['source'], roots)
        if not current.is_relative_to(sdk) or not source.is_relative_to(export):
            raise ValueError('Installed provenance escaped campaign source/prefix')
        regular_file(current, sdk); regular_file(source, export)
        if digest(current) != row['sha256'] or digest(source) != row['sha256']:
            raise ValueError('Installed/source producer bytes changed')
        seen.add(current.relative_to(sdk).as_posix())
    if not {'bin/libfsim.dll', 'fsim/_native.pyd'}.issubset(seen):
        raise ValueError('Installed provenance lacks native DLL or Python extension')
    reviews = {}
    for role in ('source_review', 'visual_review'):
        path, value = bound(report[role])
        if value.get('passed') is not True:
            raise ValueError('Passing independent review receipt required')
        reviews[role] = path
    return campaign_path, installed_path, producer_path, producer, reviews


def package(receipt_path, report_path, artifacts, sdk, maps, output, scope_roots=None, produced_overlay=None):
    roots = tuple(scope_roots or (ROOT, ENV))
    receipt_path = safe_path(receipt_path, roots)
    report_path = safe_path(report_path, roots)
    output = safe_path(output, roots)
    if output.exists():
        raise ValueError('A fresh package output directory is required')
    receipt = json.loads(receipt_path.read_text(encoding='utf-8-sig'))
    report = json.loads(report_path.read_text(encoding='utf-8-sig'))
    if receipt.get('schema') != 'fsim.sim-m03.patch-replay.v1' or receipt.get('passed') is not True or report.get('passed') is not True:
        raise ValueError('A passing replay receipt and completed passing report are required')
    export = safe_path(receipt['source_export'], roots)
    sdk = safe_path(sdk, roots)
    campaign_path, installed_path, producer_path, producer, reviews = completed_binding(receipt_path, receipt, report, sdk, roots)
    producer_products = canonical_product_map(producer['products'])
    campaign = json.loads(campaign_path.read_text(encoding='utf-8-sig'))
    maps = safe_path(maps, roots)
    map_binding = report.get('maps')
    if not isinstance(map_binding, dict) or map_binding != campaign.get('maps'):
        raise ValueError('Report and campaign must bind the same accepted map receipt')
    map_receipt_path = safe_path(map_binding['path'], roots)
    if map_receipt_path != campaign_path.parent / 'maps.json' or digest(map_receipt_path) != map_binding.get('sha256'):
        raise ValueError('Accepted map receipt changed or belongs to another campaign')
    map_receipt = verify_campaign_maps(maps, map_receipt_path, campaign.get('map_inputs'))
    expected_maps = {row['path']: row for row in map_receipt['files']}
    artifact_roots = [safe_path(directory, roots) for _, directory in artifacts]
    if not any(campaign_path.parent.is_relative_to(tree) for tree in artifact_roots):
        raise ValueError('An artifact tree must include the complete raw campaign output')
    provenance = receipt_path.parent
    manifest_path = provenance / 'source-files.json'
    if digest(manifest_path) != receipt['source_manifest_sha256'] or digest(provenance / 'sim-m03.patch') != receipt['patch_sha256']:
        raise ValueError('Preparation provenance changed')
    source_rows = json.loads(manifest_path.read_text(encoding='utf-8'))
    overlay_rows = {}
    overlay_path = None
    if produced_overlay is None and report.get('produced_overlay') is not None:
        raise ValueError('Final report requires its reviewed produced overlay argument')
    if produced_overlay is not None:
        overlay_path = safe_path(produced_overlay, roots)
        binding=report.get('produced_overlay')
        if (not isinstance(binding,dict) or safe_path(binding['path'],roots)!=overlay_path or
                binding.get('sha256')!=digest(overlay_path)):
            raise ValueError('Produced overlay argument differs from final report binding')
        overlay = json.loads(overlay_path.read_text(encoding='utf-8-sig'))
        if (overlay.get('schema') != 'fsim.sim-m03.produced-overlay.v1' or
                overlay.get('prepare_receipt_sha256') != digest(receipt_path) or
                overlay.get('campaign_sha256') != digest(campaign_path) or
                overlay.get('producer_sha256') != digest(producer_path)):
            raise ValueError('Produced overlay lacks exact same-run provenance')
        prepared = {row['path']: row for row in source_rows}
        for row in overlay['files']:
            name = relative_name(row['path'])
            product = canonical_product_key(name.removeprefix('aircraft/'))
            if (name in overlay_rows or not name.startswith('aircraft/') or name not in prepared or
                    row['prepared_sha256'] != prepared[name]['sha256'] or
                    producer_products.get(product) != row['produced_sha256'] or
                    digest(export / name) != row['produced_sha256'] or
                    row['prepared_sha256'] == row['produced_sha256']):
                raise ValueError('Overlay must name exact changed declared generated products')
            overlay_rows[name] = row
    inputs = {}
    identities = set()
    def add(logical, path, expected=None, mode='100644'):
        logical = relative_name(logical)
        if logical.casefold() in identities:
            raise ValueError('Duplicate archive identity: ' + logical)
        identities.add(logical.casefold())
        regular_file(path, next((Path(r).resolve() for r in roots if path.is_relative_to(Path(r).resolve())), path.parent))
        value = {'path': logical, 'sha256': digest(path), 'bytes': path.stat().st_size, 'mode': mode}
        if expected is not None and (value['sha256'] != expected['sha256'] or value['bytes'] != expected['bytes']):
            raise ValueError('Prepared source bytes changed: ' + logical)
        inputs[logical] = (path, value)
    for row in source_rows:
        name = relative_name(row['path'])
        if name in overlay_rows:
            # Original reviewed bytes are immutable Git blobs, never replaced by generated bytes.
            original = git(safe_path(receipt['source_root'], roots), 'cat-file', 'blob', row['oid'])
            if sha(original) != row['sha256'] or len(original) != row['bytes']:
                raise ValueError('Original prepared source object changed')
            retained = provenance / ('retained-' + row['sha256'])
            if retained.exists() and retained.read_bytes() != original:
                raise ValueError('Retained source collision')
            if not retained.exists(): retained.write_bytes(original)
            add('source/' + name, retained, row, row['mode'])
            add('produced-overlay/' + name, export / name)
        else:
            add('source/' + name, export / name, row, row['mode'])
    for row in receipt['files']:
        if row.get('deleted') and (export / relative_name(row['path'])).exists():
            raise ValueError('Prepared deletion was resurrected: ' + row['path'])
    for name in ('prepare.json', 'source-files.json', 'allowlist.json', 'sim-m03.patch', 'patch-stat.txt', 'patch-files.txt'):
        add('provenance/' + name, provenance / name)
    add('report/completed.json', report_path)
    add('report/campaign.json', campaign_path)
    add('report/installed.json', installed_path)
    add('report/installed-production-manifest.json', producer_path)
    add('report/maps.json', map_receipt_path)
    add('report/maps-start.json', safe_path(map_receipt['start_receipt']['path'], roots))
    for role, path in reviews.items():
        add('report/' + role.replace('_', '-') + '.json', path)
    if overlay_path is not None: add('provenance/produced-overlay.json', overlay_path)
    trees = [('sdk', safe_path(sdk, roots)), ('maps', maps)]
    for label, directory in artifacts:
        if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,63}', label):
            raise ValueError('Use a portable artifact label')
        trees.append(('artifacts/' + label, safe_path(directory, roots)))
    archived_maps = set()
    for prefix, tree in trees:
        if not tree.is_dir() or output.is_relative_to(tree) or tree.is_relative_to(output):
            raise ValueError('Artifact trees must exist and remain outside package output')
        for path in walk_files(tree):
            relative = path.relative_to(tree).as_posix()
            if prefix == 'maps':
                if relative not in expected_maps:raise ValueError('Map input was added after accepted inventory validation')
                add(prefix + '/' + relative, path, expected_maps[relative]);archived_maps.add(relative)
            else:add(prefix + '/' + relative, path)
    if archived_maps != set(expected_maps):raise ValueError('Accepted map inputs disappeared during packaging')
    output.mkdir(parents=True)
    (output / 'VERIFY.py').write_bytes(VERIFY_SCRIPT.encode('utf-8'))
    guide = ('# SIM-M03 delivery\n\n'
             'Extract into a fresh ordinary directory under D:/ENV/tmp. Run `python VERIFY.py` first.\n\n'
             '`source/` contains exact HEAD plus the reviewed binary patch, without Git/worktree metadata. '
             '`provenance/` retains the patch, allowlist and replay receipt. `sdk/`, `maps/` and `artifacts/` '
             'contain the supplied installed runtime and acceptance inputs/results.\n\n'
             '`report/maps.json` binds the exact offline terrain/imagery file tree frozen across the '
             'campaign; `report/maps-start.json` retains its initial inventory. The bundled maps files '
             'must match those accepted per-file hashes.\n\n'
             'When present, produced-overlay/ retains only explicitly declared same-run generated products. '
             'The original reviewed source remains in source/; provenance/produced-overlay.json binds both '
             'versions and the campaign producer. Apply those declared files after verification when '
             'reproducing the already-generated runtime instead of rerunning generation.\n\n'
             'Use the normal `source/run.ps1` and `source/tools/launch/fsim.ps1` entrypoints with the existing '
             'UCRT64 presets. Set FSIM_MAPS_DIR to this bundle\'s maps directory and TMP, TEMP and '
             'TORCH_EXTENSIONS_DIR to D:/ENV/tmp. Exact recorded generation/build/test commands and '
             'toolchain/dependency provenance are in `report/completed.json` and the artifact receipts.\n\n'
             'Archive metadata is sorted and uses zero timestamps/owners. Each declared file has an exact-byte '
             'SHA-256 entry in manifest.json; the archive digest is in the adjacent producer package.json.\n')
    (output / 'REPRODUCE.md').write_bytes(guide.encode('utf-8'))
    add('VERIFY.py', output / 'VERIFY.py')
    add('REPRODUCE.md', output / 'REPRODUCE.md')
    manifest = {'schema': 'fsim.sim-m03.delivery-manifest.v1', 'base_head': receipt['base_head'],
                'prepare_receipt_sha256': digest(receipt_path), 'completed_report_sha256': digest(report_path),
                'provenance_bindings': [dict(role=role, producer_path=str(path), archive_path=logical, sha256=digest(path))
                    for role, path, logical in (
                        ('source_manifest',receipt_path,'provenance/prepare.json'),
                        ('complete_report',report_path,'report/completed.json'),
                        ('campaign',campaign_path,'report/campaign.json'),
                        ('installed_consumer',installed_path,'report/installed.json'),
                        ('installed_producer',producer_path,'report/installed-production-manifest.json'),
                        ('maps',map_receipt_path,'report/maps.json'),
                        *((role,path,'report/'+role.replace('_','-')+'.json') for role,path in reviews.items()))],
                'files': [inputs[name][1] for name in sorted(inputs)], 'archive_format': 'tar.gz, sorted regular files, zero timestamps/owners'}
    manifest_content = json_bytes(manifest)
    save(output / 'manifest.json', manifest)
    archive = output / 'sim-m03-quality.tar.gz'
    with (output / 'sim-m03-quality.tar.gz.partial').open('xb') as raw:
        with gzip.GzipFile(fileobj=raw, mode='wb', filename='', mtime=0) as compressed:
            with tarfile.open(fileobj=compressed, mode='w', format=tarfile.PAX_FORMAT) as bundle:
                import io
                info = tarfile.TarInfo('manifest.json');info.size = len(manifest_content);info.mode = 0o644;info.mtime = 0
                bundle.addfile(info, io.BytesIO(manifest_content))
                for logical in sorted(inputs):
                    path, row = inputs[logical]
                    regular_file(path, next(Path(r).resolve() for r in roots if path.is_relative_to(Path(r).resolve())))
                    if path.stat().st_size != row['bytes']:
                        raise ValueError('Artifact changed during packaging: ' + logical)
                    info = tarfile.TarInfo(logical);info.size = row['bytes'];info.mode = 0o755 if row['mode'] == '100755' else 0o644;info.mtime = 0
                    with path.open('rb') as stream:
                        class CheckedReader:
                            def __init__(self, source):
                                self.source=source;self.hash=hashlib.sha256();self.count=0
                            def read(self, size=-1):
                                block=self.source.read(size);self.hash.update(block);self.count+=len(block);return block
                        checked=CheckedReader(stream)
                        bundle.addfile(info, checked)
                        if checked.count!=row['bytes'] or checked.hash.hexdigest()!=row['sha256'] or path.stat().st_size!=row['bytes']:
                            raise ValueError('Artifact changed while being archived: ' + logical)
    (output / 'sim-m03-quality.tar.gz.partial').replace(archive)
    result = {'schema': 'fsim.sim-m03.delivery-package.v1', 'passed': True, 'archive': str(archive),
              'archive_sha256': digest(archive), 'archive_bytes': archive.stat().st_size,
              'manifest_sha256': digest(output / 'manifest.json'), 'files': len(inputs), 'base_head': receipt['base_head']}
    save(output / 'package.json', result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='phase', required=True)
    first = commands.add_parser('prepare')
    first.add_argument('--allowlist', type=Path, required=True)
    first.add_argument('--output', type=Path, required=True)
    first.add_argument('--export', dest='source_export', type=Path, required=True)
    second = commands.add_parser('package')
    second.add_argument('--receipt', type=Path, required=True)
    second.add_argument('--report', type=Path, required=True)
    second.add_argument('--artifact', action='append', default=[], metavar='LABEL=PATH')
    second.add_argument('--sdk', type=Path, required=True)
    second.add_argument('--maps', type=Path, required=True)
    second.add_argument('--produced-overlay', type=Path)
    second.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.phase == 'prepare':
        result = prepare(ROOT, args.allowlist, args.output, args.source_export)
    else:
        artifacts = []
        for item in args.artifact:
            if '=' not in item:
                raise ValueError('--artifact requires LABEL=PATH')
            label, value = item.split('=', 1)
            artifacts.append((label, Path(value)))
        result = package(args.receipt, args.report, artifacts, args.sdk, args.maps, args.output, produced_overlay=args.produced_overlay)
    print(json.dumps(result, ensure_ascii=False, allow_nan=False))


if __name__ == '__main__':
    main()
