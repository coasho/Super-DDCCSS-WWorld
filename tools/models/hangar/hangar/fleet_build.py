"""Run the normal Hangar design pipeline with bounded model concurrency.

No external CFD solver is installed, invoked, or required by this entrypoint.
Each model keeps the same stage semantics as ``python -m hangar MODEL``.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import json
import os
from pathlib import Path
import subprocess
import sys
import time

from . import pipeline
from .files import atomic_write_text


def build(models, *, jobs, memory_limit_mib, out, force=False,
          runtime_threads=1, runtime_memory_mib=1024., stages=None):
    if not models or len(set(models)) != len(models):
        raise ValueError('A nonempty distinct model roster is required')
    if jobs < 1 or runtime_threads < 1 or runtime_memory_mib <= 0 or memory_limit_mib < runtime_memory_mib:
        raise ValueError('Invalid CPU or model-memory reservations')
    root = Path(pipeline.repo_root()).resolve()
    paths = [Path(pipeline.find_design(name)).resolve() for name in models]
    if any(not p.is_relative_to(root) or '.codex' in [part.casefold() for part in p.parts] for p in paths):
        raise ValueError('Only project design paths are allowed')
    out = Path(out).resolve()
    if out == root or not out.is_relative_to(root):
        raise ValueError('Fleet reports must remain inside the project')
    out.mkdir(parents=True, exist_ok=False)
    workers = min(len(paths), jobs // runtime_threads, int(memory_limit_mib // runtime_memory_mib))
    if workers < 1:
        raise ValueError('Resources cannot fit one model worker')
    started = time.monotonic()
    report = dict(schema='fsim.hangar-fleet-build.v3', method='authored-hangar',
                  status='running', workers=workers, models={})
    def save():
        report['elapsed_seconds'] = time.monotonic() - started
        atomic_write_text(out/'fleet-build.json', json.dumps(report, indent=2, allow_nan=False)+'\n')
    def execute(path):
        name = path.stem
        command = [sys.executable, '-u', '-B', '-m', 'hangar', str(path)]
        command += list(stages or [])
        if force:
            command.append('--force')
        env = dict(os.environ)
        for key in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS', 'FSIM_HANGAR_THREADS'):
            env[key] = str(runtime_threads)
        log = out/(name+'.log')
        tick = time.monotonic()
        with log.open('w', encoding='utf-8') as stream:
            result = subprocess.run(command, cwd=root, env=env, stdout=stream, stderr=subprocess.STDOUT)
        return name, dict(status='passed' if result.returncode == 0 else 'failed',
                          exit_code=result.returncode, log=str(log), elapsed_seconds=time.monotonic()-tick)
    save()
    with ThreadPoolExecutor(max_workers=workers) as executor:
        futures = {executor.submit(execute, path): path for path in paths}
        for future in as_completed(futures):
            path = futures[future]
            try:
                name, row = future.result()
            except Exception as error:
                name, row = path.stem, dict(status='failed', reason=type(error).__name__+': '+str(error))
            report['models'][name] = row
            save()
            print(json.dumps(dict(model=name, **row)), flush=True)
    report['status'] = 'passed' if all(row['status']=='passed' for row in report['models'].values()) else 'failed'
    save()
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', action='append', required=True)
    parser.add_argument('--jobs', type=int, required=True)
    parser.add_argument('--memory-limit-mib', type=float, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--force', action='store_true')
    parser.add_argument('--stage', action='append', choices=pipeline.STAGES)
    parser.add_argument('--runtime-threads', type=int, default=1)
    parser.add_argument('--runtime-memory-mib', type=float, default=1024.)
    args = parser.parse_args(argv)
    if args.model == ['all']:
        directory=Path(pipeline.repo_root())/'aircraft'
        args.model=sorted(p.name for p in directory.iterdir()
                          if p.name.casefold()!='.codex' and p.is_dir() and (p/(p.name+'.toml')).is_file())
    result = build(args.model, jobs=args.jobs, memory_limit_mib=args.memory_limit_mib,
                   out=args.out, force=args.force, runtime_threads=args.runtime_threads,
                   runtime_memory_mib=args.runtime_memory_mib, stages=args.stage)
    return 0 if result['status']=='passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
