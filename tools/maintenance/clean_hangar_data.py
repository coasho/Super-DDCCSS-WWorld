"""Remove obsolete CFD intermediates while retaining registered raw evidence.

Run from the repository with --apply to delete; the default only inventories.
The active campaign and native execution receipts are dependency roots.
"""
import argparse
from functools import lru_cache
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import time
import tomllib


def walk(root, prune=None):
    if not root.is_dir() or root.is_symlink() or root.is_junction():
        return
    for base, dirs, files in os.walk(root, followlinks=False):
        parent = Path(base)
        dirs[:] = [name for name in dirs if name.lower() not in ('.codex', '.git')
                   and not (prune and prune(parent / name))
                   and not (parent / name).is_symlink()
                   and not (parent / name).is_junction()
                   and not (parent / name / '.git').exists()]
        for name in files:
            path = parent / name
            if not path.is_symlink():
                yield path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path.cwd())
    parser.add_argument('--active', action='append', default=[])
    parser.add_argument('--research-root', type=Path, action='append', default=[])
    parser.add_argument('--include-research-logs', action='store_true',
                        help='Only for closed research roots, never a live wrapper log directory')
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    root = args.root.absolute()
    if any(part.lower() == '.codex' for part in root.parts):
        raise ValueError('Excluded project root')
    root = root.resolve()
    report_path = args.report.absolute()
    if any(part.lower() == '.codex' for part in report_path.parts):
        raise ValueError('Excluded cleanup report path')
    args.report = report_path.resolve()
    if not args.report.is_relative_to(root):
        raise ValueError('Cleanup reports must stay in the project')
    campaigns = root / '.local/campaigns/hangar'
    if not (root / 'tools/models/hangar/hangar').is_dir():
        raise ValueError('Expected Hangar project root')
    if args.apply and not args.active and os.name == 'nt':
        registry = root / '.local/state/processes'
        if registry.is_dir():
            spec = importlib.util.spec_from_file_location('hangar_cleanup_owner', root / 'tools/bootstrap/project.py')
            owner = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(owner)
            for path in registry.glob('*.json'):
                record = json.loads(path.read_text(encoding='utf-8'))
                if record.get('state') != 'running' or record.get('pid') == os.getppid():
                    continue
                if owner.identity_matches(record, owner.process_identity(record['pid']), root):
                    plan = root / '.local/state/plans' / (record['token'] + '.json')
                    if json.loads(plan.read_text(encoding='utf-8')).get('lock') == 'native':
                        raise ValueError('A native job is active; explicitly protect its campaign with --active')
    protected = set()
    trees = {campaigns / name for name in args.active}
    for tree in trees:
        if (tree.parent != campaigns or len(tree.name) != 32
                or any(char not in '0123456789abcdef' for char in tree.name)):
            raise ValueError('Explicit hexadecimal campaign identifier required')
    if not campaigns.exists():
        print('No generated Hangar campaign data to clean.', flush=True)
        return
    plans = set()

    @lru_cache(maxsize=262144)
    def retain(value):
        if not isinstance(value, str) or len(value) < 4:
            return
        path = Path(value)
        if not path.is_absolute() or '.codex' in [s.lower() for s in path.parts]:
            return
        # Lexical exclusion happens before filesystem resolution.
        path = Path(os.path.abspath(path))
        if not path.is_relative_to(root):
            return
        protected.add(path)
        if path.name == 'plan.json':
            plans.add(path)

    def references(value):
        if isinstance(value, dict):
            for key, item in value.items():
                retain(key)
                references(item)
        elif isinstance(value, list):
            for item in value:
                references(item)
        else:
            retain(value)

    manifest_path = root / '.local/state/manifest.json'
    if manifest_path.is_file():
        manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
        references(manifest)
        solver = manifest.get('solver', {})
        for name in ('proof', 'solver_manifest'):
            if solver.get(name):
                path = Path(solver[name]).resolve()
                trees.add(path.parent)
                references(json.loads(path.read_text(encoding='utf-8')))

    records = 0
    for path in walk(root / '.local/cache/native-execution-data'):
        if path.suffix != '.json':
            continue
        data = json.loads(path.read_text(encoding='utf-8'))
        if data.get('schema') != 'fsim.native-execution-data.v1':
            raise ValueError('Unknown native execution receipt')
        references(data)
        row = data['source_row']
        folder = Path(row['folder'] if data['identity']['kind'] == 'initializer'
                      else row['passes'][-1]['folder']).resolve()
        if not folder.is_relative_to(campaigns):
            raise ValueError('Raw source outside campaign storage')
        trees.add(folder if data['identity']['kind'] == 'initializer' else folder.parent)
        records += 1

    # Per-point products can outlive incomplete responses. Their retained
    # histories, BDF2 levels and initializers are live; retired volume digests
    # are provenance only and must not protect the deleted fields.
    for path in walk(root / '.local/cache/native-point-products'):
        if path.suffix != '.json':
            continue
        data = json.loads(path.read_text(encoding='utf-8'))
        if data.get('schema') != 'fsim.published-native-pressure-point.v1':
            raise ValueError('Unknown published native pressure point')
        protected.add(path.resolve())
        references(data.get('retained_inputs_sha256', {}))

    # Independently admitted responses and installed model publications can
    # retain cold validation fields which intentionally never enter warm reuse.
    for path in walk(root / 'build/hangar/native-responses'):
        if path.suffix == '.json':
            data = json.loads(path.read_text(encoding='utf-8'))
            if data.get('schema') == 'fsim.hangar-native-response-product.v2':
                # A published response has already completed raw admission.
                # Its immutable bytes and explicit retained inputs are live;
                # retired time-field hashes remain provenance, not files to
                # keep forever. Index files pin their immutable product.
                protected.add(path.resolve())
                references(data.get('product', {}))
                references(data.get('retained_inputs_sha256', {}))
                continue
            if data.get('schema') != 'fsim.hangar-native-response-product.v1':
                raise ValueError('Unknown native response product')
            references(data)
    aircraft = root / 'aircraft'
    if aircraft.is_dir():
        for directory in aircraft.iterdir():
            if (directory.name.lower() in ('.codex', '.git') or not directory.is_dir()
                    or directory.is_symlink() or directory.is_junction()):
                continue
            for design in directory.glob('*.toml'):
                selector = tomllib.loads(design.read_text(encoding='utf-8')).get('cfd', {})
                if not selector.get('model'):
                    continue
                pending = [selector['model']]
                visited_products = set()
                while pending:
                    relative = Path(pending.pop())
                    if (relative.is_absolute() or '..' in relative.parts
                            or any(s.lower() == '.codex' for s in relative.parts)):
                        raise ValueError('Selected CFD artifact escapes its authored model')
                    path = (directory / relative).resolve()
                    if not path.is_relative_to(directory.resolve()):
                        raise ValueError('Linked CFD artifact escapes its authored model')
                    if path in visited_products:
                        continue
                    visited_products.add(path)
                    data = json.loads(path.read_text(encoding='utf-8'))
                    references(data)
                    for item in data.get('parts', []) + data.get('states', []):
                        if item.get('artifact', {}).get('path'):
                            pending.append(item['artifact']['path'])

    # Current plans can retain old seed fields or a mesh outside their campaign.
    for name in args.active:
        for path in walk(campaigns / name):
            if path.suffix == '.json':
                references(json.loads(path.read_text(encoding='utf-8')))
    visited = set()
    while plans - visited:
        path = (plans - visited).pop()
        references(json.loads(path.read_text(encoding='utf-8')))
        visited.add(path)

    # Prepared meshes are cheap, reusable authored inputs. Keep their full roots.
    for campaign in campaigns.iterdir():
        if not campaign.is_dir() or campaign.is_symlink() or campaign.is_junction():
            continue
        for path in walk(campaign / 'cfd', lambda p: p.name.startswith('case-')):
            if path.name not in ('mesh.json', 'prepared-mesh.json', 'receipt.json', 'mesh-manifest.json'):
                continue
            try:
                data = json.loads(path.read_text(encoding='utf-8'))
            except (ValueError, OSError):
                continue
            if data.get('schema') == 'fsim.hangar-native-mesh.v1':
                trees.add(path.parent)
                references(data)

    def kept(path):
        return path in protected or any(parent in trees for parent in path.parents)

    scan_roots = [(path / 'cfd', False) for path in campaigns.iterdir()
                  if path.name not in args.active and path.name.lower() not in ('.codex', '.git')
                  and path.is_dir() and not path.is_symlink() and not path.is_junction()]
    for research_argument in args.research_root:
        research = research_argument.absolute()
        if '.codex' in [p.lower() for p in research.parts]:
            raise ValueError('Excluded research root')
        research = research.resolve()
        authored_output = (research.is_relative_to(root / 'aircraft') and research.name == 'out'
                           and research.parent.parent == root / 'aircraft')
        retired_solver = research.parent == root / '.local/solvers'
        if retired_solver and manifest_path.is_file() and manifest.get('solver', {}).get('path'):
            if Path(manifest['solver']['path']).resolve().is_relative_to(research):
                raise ValueError('The selected solver build cannot be a cleanup root')
        if (not (research.is_relative_to(root / 'build/architecture-reviews') or authored_output or retired_solver)
                or (research / '.git').exists()):
            raise ValueError('Research cleanup requires generated reviews, aircraft out or an unselected solver build')
        scan_roots.append((research, True))
    candidates = []
    suffixes = {'.dat', '.su2', '.brep', '.vtu', '.vtk', '.stl', '.msh', '.cgns', '.tec', '.szplt', '.log'}
    research_suffixes = suffixes | {'.off', '.mesh', '.npz', '.a', '.o', '.obj', '.exe'}
    for directory, research in scan_roots:
        retired_solver_copy = directory.parent == root / '.local/solvers'
        for path in walk(directory, lambda p: p in trees or
                         (research and not retired_solver_copy
                          and p.name in ('source', 'current-source', 'workspace', 'checkout', 'include'))):
            duplicate_fleet_report = path.name == 'fleet-result.json'
            if (retired_solver_copy or path.suffix.lower() in (research_suffixes if research else suffixes) or duplicate_fleet_report) and not (research and path.suffix.lower() == '.log'
                    and not args.include_research_logs) and not kept(path):
                candidates.append((path, path.stat().st_size))

    # A pin disappearing during inventory is a hard failure, before any deletion.
    cleanup_roots = {path for path, _ in scan_roots}
    retained = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in protected if path.suffix.lower() in research_suffixes
                and not any(parent in trees for parent in path.parents)
                and (path in cleanup_roots or any(parent in cleanup_roots for parent in path.parents))
                and path.is_file()}
    report = dict(schema='fsim.hangar-storage-cleanup.v1', apply=args.apply,
                  active=args.active, execution_records=records, cleanup_roots=[str(p) for p, _ in scan_roots],
                  protected_trees=[str(p) for p in sorted(trees)],
                  protected_binary_sha256=retained,
                  candidate_files=len(candidates), logical_bytes=sum(size for _, size in candidates),
                  disk_free_before=shutil.disk_usage(root).free, deleted_files=0,
                  candidates=[dict(path=str(p), bytes=n) for p, n in candidates])
    args.report.parent.mkdir(parents=True, exist_ok=True)

    def save():
        args.report.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8', newline='\n')

    save()
    print(json.dumps({k: v for k, v in report.items() if k not in
                     ('candidates', 'protected_trees', 'protected_binary_sha256', 'cleanup_roots')}), flush=True)
    if args.apply:
        started = time.monotonic()
        @lru_cache(maxsize=32768)
        def resolved_parent(path):
            return path.resolve()
        for path, _ in candidates:
            absolute = resolved_parent(path.parent) / path.name
            if not any(parent in cleanup_roots for parent in absolute.parents) or kept(absolute) or path.is_symlink():
                raise ValueError('Deletion target escaped bounded obsolete CFD storage')
            path.unlink()
            report['deleted_files'] += 1
        for filename, digest in retained.items():
            if hashlib.sha256(Path(filename).read_bytes()).hexdigest() != digest:
                raise ValueError('Retained binary dependency changed during cleanup')
        report.update(disk_free_after=shutil.disk_usage(root).free,
                      elapsed_s=time.monotonic() - started, retained_hashes_verified=True)
        # The pre-deletion inventory is a transaction journal. Once verified,
        # keep counts/digests rather than another permanent bulk path listing.
        for name in ('candidates', 'protected_binary_sha256', 'protected_trees'):
            value = report.pop(name)
            report[name + '_count'] = len(value)
            report[name + '_digest'] = hashlib.sha256(
                json.dumps(value, sort_keys=True, separators=(',', ':')).encode('utf-8')).hexdigest()
        save()
        print(json.dumps({k: v for k, v in report.items() if k not in
                         ('candidates', 'protected_trees', 'protected_binary_sha256', 'cleanup_roots')}), flush=True)


if __name__ == '__main__':
    main()
