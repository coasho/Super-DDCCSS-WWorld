"""The configured build graph against the source graph (README, "Reconciling with the configured build"; ARCH-10).

    python -B tools/architecture/configured.py --repo . --build build/ucrt64-headless [--json facts.json]

A configure of this tree answers its own CMake File API query: cmake/ArchitectureChecks.cmake asks for the code model
and the cache. The source parser (check.py) stays the ownership authority, because it reads every CMake branch; this
gate holds one preset's configured graph to it, by target name. The reply must be this build's (its source and build
directories) and come from CMake 4.2 or later (HC-43): native builds stay supported from CMake 3.25, but an older
CMake's configure leaves this gate failing with an actionable diagnostic and the preset's configured-graph
acceptance pending.

1. targets: a configured target with a source under src is a registered module target. A test target (one with a
   source under tests) may also compile an application module's sources, which have no library to link; those are
   reported by name. A library module's source is compiled by its own target alone.
2. sources: every configured source under src has its target's module as its owner.
3. links: every configured link between module targets (the code model's linkLibraries and interfaceLinkLibraries)
   is an edge of the source graph, and a link to any other target names a registered external target. A build-order
   dependency between module targets lies in the consumer's source closure.
4. branches: a module is configured exactly where its branch is enabled (HC-42). A module the policy marks
   non-headless belongs to the FSIM_WITH_RENDER branch and a Python extension module (its destinations under a
   `python` directory) to the FSIM_BUILD_PYTHON branch: absent when that option is off, present - like every other
   module - when it is on. A source edge between two configured modules is configured too, and a configured module's
   providers are configured.
5. include directories: none is src or an ancestor of it. A directory under src lies in the destinations of the module
   or of a module in its source closure, and an include view (build/<preset>/include-views/<target>) is that module's
   or a closure module's, whether or not the include is marked SYSTEM: SYSTEM changes compiler diagnostics, not
   ownership (HC-38). The others must be the public include root, vendor code (third_party), a SYSTEM directory or
   outside the repository.

Exit 0 accepted, 1 rejected, 2 no usable reply or another input error: a preset without a reply is pending, not
passed. Standard library only."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import check  # noqa: E402

MIN_CODEMODEL = (2, 9)  # (linkLibraries and interfaceLinkLibraries: the direct link items)
MIN_CMAKE = (4, 2)  # (the user's compatibility decision for this gate, HC-43; native builds keep CMake 3.25)
REQUIREMENT = ("architecture_configured needs CMake 4.2 or later (native builds remain supported from CMake 3.25); "
               "configure this preset with CMake 4.2+ to check its configured graph. Until then its configured-graph "
               "acceptance is pending, not passed")


class InputError(Exception):
    pass


def cache_version(build):
    """The CMake version that configured `build`, from its CMakeCache.txt, or None."""
    values = {}
    try:
        lines = (build / "CMakeCache.txt").read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return None
    for line in lines:
        for part in ("MAJOR", "MINOR", "PATCH"):
            if line.startswith(f"CMAKE_CACHE_{part}_VERSION:INTERNAL="):
                values[part] = line.split("=", 1)[1]
    return ".".join(values[p] for p in ("MAJOR", "MINOR", "PATCH")) if len(values) == 3 else None


def load_reply(build):
    """The newest reply index's code model, its targets, its cache entries and the CMake version that wrote it."""
    reply = build / ".cmake" / "api" / "v1" / "reply"
    indexes = sorted(reply.glob("index-*.json"))
    if not indexes:
        configured_with = cache_version(build)
        raise InputError(f"no CMake File API reply in {reply}"
                         + (f" (this build was configured with CMake {configured_with})" if configured_with else "")
                         + f": {REQUIREMENT}; with CMake 4.2+, configure this build again")
    index = json.loads(indexes[-1].read_text(encoding="utf-8"))
    version = index.get("cmake", {}).get("version", {})
    if (version.get("major", 0), version.get("minor", 0)) < MIN_CMAKE:
        raise InputError(f"this build's reply is from CMake {version.get('string', 'of an unknown version')}: {REQUIREMENT}")
    entries = index.get("reply", {})
    if "codemodel-v2" not in entries or "cache-v2" not in entries:
        raise InputError(f"the reply in {reply} has no code model or no cache: configure this build again")
    model = json.loads((reply / entries["codemodel-v2"]["jsonFile"]).read_text(encoding="utf-8"))
    codemodel = (model["version"]["major"], model["version"]["minor"])
    if codemodel < MIN_CODEMODEL:
        raise InputError("code model %d.%d has no direct link items; CMake must reply with %d.%d or later"
                         % (codemodel + MIN_CODEMODEL))
    cache = json.loads((reply / entries["cache-v2"]["jsonFile"]).read_text(encoding="utf-8"))
    targets = {}
    for configuration in model["configurations"]:
        for entry in configuration["targets"]:
            targets[entry["id"]] = json.loads((reply / entry["jsonFile"]).read_text(encoding="utf-8"))
    return model, targets, {entry["name"]: entry["value"] for entry in cache["entries"]}, version["string"]


def enabled(value):
    return str(value).upper() not in {"", "0", "OFF", "NO", "FALSE", "N", "IGNORE", "NOTFOUND"} \
        and not str(value).upper().endswith("-NOTFOUND")


def relative(path, root):
    try:
        return path.relative_to(root).as_posix()
    except ValueError:
        return None


def analyze(repo, build, policy):
    """(issues, facts) for one configured build."""
    repo, build = repo.resolve(), build.resolve()
    model, targets, cache, cmake_version = load_reply(build)
    if Path(model["paths"]["source"]).resolve() != repo:
        raise InputError(f"the reply is of {model['paths']['source']}, not {repo}")
    if Path(model["paths"]["build"]).resolve() != build:
        raise InputError(f"the reply is of the build {model['paths']['build']}, not {build}")  # (HC-42)
    for option in ("FSIM_WITH_RENDER", "FSIM_BUILD_PYTHON"):
        if option not in cache:
            raise InputError(f"the configure's cache has no {option}")
    source_issues, facts = check.analyze(repo, policy)
    issues = [f"source-graph: {issue}" for issue in source_issues]  # (check.py's own verdict comes first)
    modules = policy["modules"]
    module_of = {}
    for name, module in modules.items():
        module_of[module["target"]] = name
    owner = {check.fold(path): name for path, name in facts["source_owner"].items()}
    links = {name: set(providers) for name, providers in facts["links"].items()}

    def closure(name):
        seen, todo = set(), [name]
        while todo:
            current = todo.pop()
            if current not in seen:
                seen.add(current)
                todo.extend(links.get(current, ()))
        return seen

    def issue(code, place, detail):
        issues.append(f"{code}: {place}: {detail}")

    source_root = repo / "src"
    model_root = Path(model["paths"]["source"]).resolve()
    names = {target_id: target["name"] for target_id, target in targets.items()}
    configured, edges, test_compiled, external_used = set(), set(), {}, set()
    for target in targets.values():
        name = target["name"]
        sources = []
        for source in target.get("sources", []):
            path = Path(source["path"])
            path = (path if path.is_absolute() else model_root / path).resolve()
            sources.append(path)
        under_src = [relative(path, repo) for path in sources if relative(path, source_root) is not None]
        under_tests = any(relative(path, repo / "tests") is not None for path in sources)
        module = module_of.get(name)
        if module is None:
            if not under_src:
                continue
            foreign = sorted({owner.get(check.fold(path), "no owner") for path in under_src})
            libraries = [m for m in foreign if m not in modules or modules[m]["kind"] != "application"]
            if not under_tests or libraries:
                issue("unregistered-target", name, "compiles sources under src (%s) outside their module targets; only a "
                      "test target may compile an application's sources" % ", ".join(sorted(under_src)))
            else:
                test_compiled[name] = {"modules": foreign, "sources": sorted(under_src)}
            continue
        configured.add(module)
        for path in under_src:
            found = owner.get(check.fold(path))
            if found != module:
                issue("foreign-source", name, f"{path} is owned by {found or 'no module'}, not {module}")
        for key in ("linkLibraries", "interfaceLinkLibraries"):
            for item in target.get(key) or []:
                provider = names.get(item.get("id"))
                if provider is None:
                    continue  # (a library file or flag, not a target of this build)
                if provider in module_of:
                    edges.add((module, module_of[provider]))
                    if module_of[provider] not in links[module]:
                        issue("unknown-configured-link", name, f"links {provider} ({module_of[provider]}), which the "
                              "source graph does not declare: a hidden declaration")
                elif provider in policy["external_targets"]:
                    external_used.add(provider)
                else:
                    issue("unregistered-link", name, f"links {provider}, neither a module nor a registered external target")
        reach = closure(module)
        for item in target.get("dependencies") or []:
            provider = names.get(item.get("id"))
            if provider in module_of and module_of[provider] not in reach:
                issue("hidden-dependency", name, f"depends on {provider} ({module_of[provider]}) outside its source closure")
        allowed_dirs = [repo / directory for m in reach for directory in modules[m]["destinations"] + modules[m]["directories"]]
        views = {modules[m]["target"] for m in reach}
        for group in target.get("compileGroups", []):
            for include in group.get("includes", []):
                path = Path(include["path"]).resolve()
                inside = relative(path, repo)
                # (owned paths first: a SYSTEM mark changes diagnostics, not ownership - HC-38)
                if path == source_root or relative(source_root, path) is not None:
                    issue("broad-include-root", name, f"includes {path}, which holds the whole src tree")
                elif inside is None:
                    continue
                elif relative(path, source_root) is not None:
                    if not any(relative(path, directory) is not None for directory in allowed_dirs):
                        issue("include-outside-closure", name, f"includes {inside}, outside its and its providers' destinations")
                elif relative(path, build / "include-views") is not None:
                    view = relative(path, build / "include-views").split("/", 1)[0]
                    if view not in module_of:
                        issue("unknown-include-view", name, f"includes the view {view}, which is no module's")
                    elif view not in views:
                        issue("include-outside-closure", name, f"includes {module_of[view]}'s view outside its source closure")
                elif include.get("isSystem"):
                    continue  # (vendor or dependency-prefix directories inside the checkout)
                elif inside.split("/", 1)[0] not in {"include", "third_party"}:
                    issue("unclassified-include", name, f"includes {inside}: not a view, a destination, the public "
                          "include root or vendor code")
    render, python = enabled(cache["FSIM_WITH_RENDER"]), enabled(cache["FSIM_BUILD_PYTHON"])

    def disabled(module):
        """Why `module`'s branch is off in this configure, or None."""
        if all("python" in directory.split("/") for directory in modules[module]["destinations"]) and not python:
            return "FSIM_BUILD_PYTHON is off"
        if not modules[module]["headless"] and not render:
            return "FSIM_WITH_RENDER is off"
        return None

    absent = {}
    for module in sorted(modules):
        off = disabled(module)
        if module in configured and off:
            issue("disabled-branch-module", module, f"configured although {off}: this reply does not match its own "
                  "branch options")
        elif module not in configured and off:
            absent[module] = off
        elif module not in configured:
            issue("unconfigured-module", module, "not configured, though its branch is enabled here (FSIM_WITH_RENDER %s, "
                  "FSIM_BUILD_PYTHON %s)" % (cache["FSIM_WITH_RENDER"], cache["FSIM_BUILD_PYTHON"]))
    for consumer in sorted(configured):
        for provider in sorted(links[consumer]):
            if provider not in configured:
                issue("unconfigured-provider", consumer, f"its provider {provider} is not configured here")
            elif (consumer, provider) not in edges:
                issue("missing-configured-edge", consumer, f"the source graph's link to {provider} is not configured here")
    facts = {
        "build": build.as_posix(), "cmake": cmake_version,
        "codemodel": "%d.%d" % (model["version"]["major"], model["version"]["minor"]),
        "cache": {"FSIM_WITH_RENDER": cache["FSIM_WITH_RENDER"], "FSIM_BUILD_PYTHON": cache["FSIM_BUILD_PYTHON"],
                  "CMAKE_BUILD_TYPE": cache.get("CMAKE_BUILD_TYPE", "")},
        "targets": len(targets), "modules_configured": sorted(configured), "modules_absent": absent,
        "module_links": sorted(f"{c} -> {p}" for c, p in edges), "source_links": sum(len(p) for p in links.values()),
        "external_links": sorted(external_used), "test_compiled_application_sources": test_compiled,
    }
    return sorted(set(issues)), facts


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--build", type=Path, required=True)
    parser.add_argument("--policy", type=Path, default=Path(__file__).with_name("policy.json"))
    parser.add_argument("--json", type=Path, help="write the facts here")
    args = parser.parse_args()
    try:
        issues, facts = analyze(args.repo, args.build, json.loads(args.policy.read_text(encoding="utf-8")))
    except (InputError, OSError, ValueError, KeyError, TypeError) as error:
        print(f"configured graph input error: {error}", file=sys.stderr)
        return 2
    if args.json:
        args.json.write_text(json.dumps({"issues": issues, **facts}, indent=1) + "\n", encoding="utf-8")
    for line in issues:
        print(line)
    if issues:
        print(f"Configured graph rejected ({facts['build']}): {len(issues)} violation(s).")
        return 1
    absent = ", ".join(f"{m} ({why})" for m, why in facts["modules_absent"].items()) or "none"
    tested = "; ".join(f"{t} ({', '.join(v['modules'])})" for t, v in sorted(facts["test_compiled_application_sources"].items()))
    print(f"Configured graph accepted ({facts['build']}, CMake {facts['cmake']}, code model {facts['codemodel']}): "
          f"{facts['targets']} targets, "
          f"{len(facts['modules_configured'])} module targets configured; absent: {absent}.")
    print(f"Module links: {len(facts['module_links'])} configured, each a source-graph edge, and every source edge between "
          f"configured modules configured; external: {', '.join(facts['external_links']) or 'none'}.")
    print(f"Sources owned by their targets' modules; include directories within each module's source closure; "
          f"application sources compiled by tests: {tested or 'none'}.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
