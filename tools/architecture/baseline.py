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
SCOPE = ["src", "include/fsim", "python/fsim", "python/CMakeLists.txt", "cmake", "tests", "examples", "tools"]


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


KEYWORDS = {"PRIVATE", "PUBLIC", "INTERFACE", "SYSTEM", "BEFORE", "AFTER", "debug", "optimized", "general"}


def declared_value(repo, base, value, kind):
    """One declared input or link as written, with what a static read can establish: a file under the declaring
    directory (with its hash), a target name, or an unresolved variable/generator expression - recorded, never
    guessed (its configured value is not static evidence)."""
    resolved = value.replace("${CMAKE_CURRENT_SOURCE_DIR}", base.as_posix()).replace("${CMAKE_SOURCE_DIR}", repo.as_posix())
    entry = {"declared": value}
    if "${" in resolved or "$<" in resolved:
        entry["resolved"] = False
        return entry
    entry["resolved"] = True
    path = (base / resolved) if not Path(resolved).is_absolute() else Path(resolved)
    if kind in {"source", "include"} and path.exists():
        rel = path.resolve().relative_to(repo.resolve()).as_posix() if path.resolve().is_relative_to(repo.resolve()) else path.as_posix()
        entry["path"] = rel
        if path.is_file():
            entry["sha256"] = sha256(path)
    elif kind == "source":
        entry["missing"] = True
    return entry


def cmake_declarations(repo, path, outer, out, seen):
    """The commands of one CMake file as (command, args, enclosing if() branches, helper, declaring directory): helper
    functions are expanded with their call's arguments, and an add_subdirectory() of a repository directory is followed
    into that directory's CMakeLists.txt, under the branches enclosing the call (ARCH-07: the extension modules are
    declared in their adapter directories, which python/CMakeLists.txt adds)."""
    if path in seen:
        raise ValueError(f"{path.relative_to(repo).as_posix()} is added twice")
    seen.add(path)
    commands = list(check.cmake_commands(path.read_text(encoding="utf-8")))
    functions, top, conditions, current = {}, [], [], None
    for command, args in commands:
        if current is not None:
            if command in {"endfunction", "endmacro"}:
                functions[current[0]] = (current[1], current[2])
                current = None
            else:
                current[2].append((command, args))
            continue
        if command in {"function", "macro"} and args:
            current = (args[0].lower(), args[1:], [])
        elif command == "if":
            conditions.append("if(" + " ".join(args) + ")")
        elif command == "elseif" and conditions:
            conditions[-1] = "elseif(" + " ".join(args) + ")"
        elif command == "else" and conditions:
            conditions[-1] = "else() of " + conditions[-1]
        elif command == "endif" and conditions:
            conditions.pop()
        else:
            top.append((command, args, outer + tuple(conditions)))
    for command, args, where in top:
        if command in functions:
            params, body = functions[command]
            mapping = dict(zip(params, args))
            for inner, inner_args in body:
                values = []
                for value in inner_args:
                    for name, actual in mapping.items():
                        value = value.replace("${" + name + "}", actual)
                    values.append(value)
                out.append((inner, values, where, command, path.parent))
        elif command == "add_subdirectory" and args:
            directory = args[0].replace("${CMAKE_SOURCE_DIR}", repo.as_posix()).replace("${CMAKE_CURRENT_SOURCE_DIR}",
                                                                                      path.parent.as_posix())
            target = Path(directory) if Path(directory).is_absolute() else path.parent / directory
            if "${" not in directory and target.resolve().is_relative_to(repo.resolve()) and (target / "CMakeLists.txt").is_file():
                cmake_declarations(repo, (target / "CMakeLists.txt").resolve(), where, out, seen)
            else:
                out.append((command, args, where, None, path.parent))
        else:
            out.append((command, args, where, None, path.parent))


def extension_declarations(repo, relative="python/CMakeLists.txt"):
    """Each CPython extension module the build declares (add_library MODULE), with every input and link as written,
    from python/CMakeLists.txt and the directories it adds: helper functions are expanded with their call's arguments
    (so a helper-added header or link is the extension's), and direct target_sources / target_link_libraries / include
    / definition / property declarations are added. Each declaration keeps the if() branches enclosing it (an added
    directory's, those around its add_subdirectory); every branch is included, as the architecture parser does."""
    path = repo / relative
    if not path.is_file():
        return "absent"
    expanded = []
    try:
        cmake_declarations(repo, path.resolve(), (), expanded, set())
    except ValueError as error:
        return f"unparsed: {error}"
    extensions = {}
    for command, args, where, helper, base in expanded:
        if command == "add_library" and len(args) > 1 and args[1] == "MODULE":
            extensions[args[0]] = {"target": args[0], "conditions": list(where), "helper": helper, "module": None,
                                   "declared_in": (base / "CMakeLists.txt").relative_to(repo.resolve()).as_posix()
                                   if base.is_relative_to(repo.resolve()) else base.as_posix(),
                                   "sources": [], "links": [], "include_directories": [], "compile_definitions": [],
                                   "properties": {}}
            extensions[args[0]]["sources"] += [declared_value(repo, base, a, "source") for a in args[2:]]
    for command, args, where, helper, base in expanded:
        if not args or args[0] not in extensions:
            continue
        extension = extensions[args[0]]
        values = [a for a in args[1:] if a not in KEYWORDS]
        if command == "target_sources":
            extension["sources"] += [declared_value(repo, base, a, "source") for a in values]
        elif command == "target_link_libraries":
            extension["links"] += [dict(declared_value(repo, base, a, "link"), by=helper or "direct") for a in values]
        elif command == "target_include_directories":
            extension["include_directories"] += [declared_value(repo, base, a, "include") for a in values]
        elif command == "target_compile_definitions":
            extension["compile_definitions"] += values
        elif command == "set_target_properties" and "PROPERTIES" in args:
            pairs = args[args.index("PROPERTIES") + 1:]
            extension["properties"].update(dict(zip(pairs[0::2], pairs[1::2])))
    for extension in extensions.values():
        name = extension["properties"].get("OUTPUT_NAME")
        extension["module"] = ("fsim." + name) if name and "${" not in name else None
        extension["sources"].sort(key=lambda e: e["declared"])
    return [extensions[name] for name in sorted(extensions)]


def package_outputs(repo, facts):
    """What the build declares it produces: native target types, CPython extensions, install rules as written."""
    out = {"native_targets": facts["target_types"], "python_extensions": extension_declarations(repo), "install_rules": {}}
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
        out["install_rules"][relative] = [" ".join(args) for command, args in commands if command == "install"]
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
    classified = policy["public_headers"]
    public = {path: dict({"sha256": sha256(repo / path), "included_by": consumers}, **classified.get(path, {"domain": None, "kind": None}))
              for path, consumers in facts["public_headers"].items()}
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
            "public_includes": facts["public_includes"],
            "public_exceptions_applied": facts["public_exceptions_applied"],
            "unowned": sorted(path for path, module in facts["source_owner"].items() if module is None),
            "public_headers": public,
        },
        "python": {
            "modules": python_modules,
            "imports": python["edges"],
            "initialization_imports": python["init_edges"],
            "external_imports": python["external"],
            "cycles_including_deferred": python["cycles_all"],
            "eager_cycles": python["cycles_eager"],
            "eager_closure_of_package": python["eager_closure"],
            "dynamic_imports": python["dynamic_imports"],
            "legacy_backedges": policy["python"]["exceptions"],
        },
        "layout": {
            "roots": policy["layout"]["roots"],
            "files": facts["layout"]["files"],
            "exceptions": policy["layout"]["exceptions"],
            "exceptions_applied": facts["layout"]["exceptions_applied"],
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
