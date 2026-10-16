"""Record the architecture baseline: the actual graphs the gate checks, with hashes (ARCH-01).

    python -B tools/architecture/baseline.py --date 2026-10-16 [--out docs/architecture/validation/arch-01-baseline.json]

It runs the gate's own analysis (check.analyze) and writes, deterministically for one tree and date: the HEAD and the
scoped working-tree changes it was taken from; each native module's target, library type, domain, kind, legacy and
destination directories and every source it owns with its SHA-256; the actual direct link graph (an allowance in the
policy is not evidence of a link), external links and the module-to-module include graph (how many files include
another module's headers); the legacy exceptions in use; each public header with its hash and the modules including
it; the Python modules with owner and hash, every literal import (eager or deferred, with its line), cycles, the
eager closure and dynamic-import sites; and the package outputs the build declares (native target types, the CPython
extension modules with their sources and links, and the install rules as written). The gate's verdict is part of the
record; a rejected tree is recorded as rejected (exit 1).

Static by design: this is not the configured CMake File API graph, the linker's view, an installed package or a
runtime import trace (README.md, "Reconciling with the configured build"). Standard library only."""

from __future__ import annotations

import argparse
import datetime
import hashlib
import json
from pathlib import Path
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import check  # noqa: E402

SCHEMA = "fsim.architecture.baseline.v1"
SCOPE = ["src", "include/fsim", "python/fsim", "python/src", "python/CMakeLists.txt", "cmake", "tools/architecture"]


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def digest(hashes):
    """One digest over (path, hash) pairs, independent of their order."""
    return hashlib.sha256("".join(f"{p}\n{h}\n" for p, h in sorted(hashes.items())).encode("utf-8")).hexdigest()


def git(repo, *args):
    try:
        run = subprocess.run(["git", *args], cwd=repo, capture_output=True, text=True, check=False)
    except OSError:
        return None
    return run.stdout if run.returncode == 0 else None


def package_outputs(repo, facts):
    """What the build declares it produces: native target types, CPython extensions, install rules as written."""
    out = {"native_targets": facts["target_types"], "python_extensions": [], "install_rules": {}}
    for relative in ("python/CMakeLists.txt", "cmake/Install.cmake"):
        path = repo / relative
        if not path.is_file():
            out["install_rules"][relative] = "absent"
            continue
        try:
            commands = list(check.cmake_commands(path.read_text(encoding="utf-8")))
        except ValueError as error:
            out["install_rules"][relative] = f"unparsed: {error}"
            continue
        rules = []
        sources = {}
        for command, args in commands:
            if command == "install":
                rules.append(" ".join(args))
            elif command == "fsim_python_module" and len(args) == 4:
                target, module, source, link = args
                out["python_extensions"].append({"target": target, "module": "fsim." + module, "sources": [source], "links": [link]})
            elif command == "target_sources" and args:
                sources.setdefault(args[0], []).extend(a for a in args[1:] if a not in {"PRIVATE", "PUBLIC", "INTERFACE"})
        for extension in out["python_extensions"]:
            extension["sources"] = sorted(set(extension["sources"] + sources.get(extension["target"], [])))
            extension["source_sha256"] = {s: sha256(repo / "python" / s) for s in extension["sources"] if (repo / "python" / s).is_file()}
        out["install_rules"][relative] = rules
    return out


def record(repo, policy_path, date):
    policy = json.loads(policy_path.read_text(encoding="utf-8"))
    issues, facts = check.analyze(repo, policy)
    modules = policy["modules"]
    files = {}
    for path, module in facts["source_owner"].items():
        files.setdefault(module, {})[path] = sha256(repo / path)
    native_modules = {}
    for name, module in sorted(modules.items()):
        owned = files.get(name, {})
        native_modules[name] = {
            "target": module["target"], "type": facts["target_types"].get(module["target"]), "aliases": module["aliases"],
            "domain": module["domain"], "kind": module["kind"], "headless": module["headless"],
            "directories": module["directories"], "destinations": module["destinations"], "explicit_files": module["files"],
            "files": owned, "digest": digest(owned),
        }
    public = {path: {"sha256": sha256(repo / path), "included_by": consumers} for path, consumers in facts["public_headers"].items()}
    python = facts["python"]
    python_modules = {name: {"path": path, "owner": python["owners"].get(name),
                             "sha256": sha256(repo / path) if path else None}
                      for name, path in python["modules"].items()}
    status = git(repo, "status", "--porcelain", "--", *SCOPE, ":(exclude,glob)**/.codex/**")
    head = git(repo, "rev-parse", "HEAD")
    return issues, {
        "schema": SCHEMA,
        "date": date,
        "head": head.strip() if head else None,
        "worktree_changes": sorted(line[3:] for line in status.splitlines()) if status is not None else None,
        "scope": SCOPE,
        "policy": {"schema": policy["schema"], "sha256": sha256(policy_path)},
        "checker": {"accepted": not issues, "issues": issues, "native_modules": facts["module_count"],
                    "native_source_and_public_files": facts["file_count"]},
        "native": {
            "modules": native_modules,
            "planned_modules": policy["planned_modules"],
            "links": facts["links"],
            "external_links": facts["external_links"],
            "includes": facts["includes"],
            "exceptions_applied": facts["exceptions_applied"],
            "unowned": sorted(path for path, module in facts["source_owner"].items() if module is None),
            "public_headers": public,
        },
        "python": {
            "modules": python_modules,
            "imports": python["edges"],
            "external_imports": python["external"],
            "cycles_including_deferred": python["cycles_all"],
            "eager_cycles": python["cycles_eager"],
            "eager_closure_of_package": python["eager_closure"],
            "dynamic_imports": python["dynamic_imports"],
            "legacy_backedges": policy["python"]["exceptions"],
        },
        "package_outputs": package_outputs(repo, facts),
        "digests": {
            "src": digest({p: h for owned in files.values() for p, h in owned.items()}),
            "include/fsim": digest({p: v["sha256"] for p, v in public.items()}),
            "python/fsim": digest({m["path"]: m["sha256"] for m in python_modules.values() if m["path"]}),
        },
        "limits": [
            "Static source declarations: every CMake branch is combined; this is not the configured File API graph of one preset",
            "Link edges are target_link_libraries declarations; include edges are literal includes resolved to an owning module",
            "Python imports are a literal AST scan; dynamic imports, native initialization and external packages' imports are not proven",
            "Package outputs are the build's declarations, not an installed tree; installed provenance is checked by the package gates",
            "The record is of the working tree named by head and worktree_changes, not of a clean commit unless worktree_changes is empty",
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--policy", type=Path, default=None)
    parser.add_argument("--date", required=True, help="the record's date in the client's timezone, YYYY-MM-DD")
    parser.add_argument("--out", type=Path, default=None, help="default: docs/architecture/validation/arch-01-baseline.json")
    args = parser.parse_args()
    datetime.date.fromisoformat(args.date)
    repo = args.repo.resolve()
    policy = (args.policy or repo / "tools/architecture/policy.json").resolve()
    out = args.out or repo / "docs/architecture/validation/arch-01-baseline.json"
    try:
        issues, result = record(repo, policy, args.date)
    except (OSError, ValueError, KeyError, TypeError) as error:
        print(f"architecture input error: {error}", file=sys.stderr)
        return 1
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=1, sort_keys=True) + "\n", encoding="utf-8", newline="\n")
    native = result["native"]
    print(f"{out}: {'accepted' if not issues else 'REJECTED (%d issues)' % len(issues)}; "
          f"{len(native['modules'])} native modules, {sum(len(m['files']) for m in native['modules'].values())} owned sources, "
          f"{len(native['public_headers'])} public headers, {len(result['python']['modules'])} Python modules; head {result['head']}")
    return 1 if issues else 0


if __name__ == "__main__":
    sys.exit(main())
