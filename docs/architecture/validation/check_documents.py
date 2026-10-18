"""Validate only the architecture planning documents, registers and target DAGs."""
from __future__ import annotations

import csv
import fnmatch
import json
import os
from pathlib import Path
import re
import sys
from urllib.parse import unquote

HERE = Path(__file__).resolve().parents[1]
ROOT = HERE.parents[1]
LINK = re.compile(r"(?<!!)\[[^\]]+\]\(([^)]+)\)")
errors = []


def fail(place, message):
    errors.append(str(place) + ": " + message)


def walk(path):
    for directory, children, names in os.walk(path, followlinks=False):
        children[:] = [name for name in children if name != ".codex"]
        for name in names:
            yield Path(directory) / name


def destination(rule, source):
    """Expand paired glob captures; reject lossy or ambiguous path templates."""
    tokens = re.compile(r"(\*\*|\*|\?)")
    parts = tokens.split(rule["source_pattern"])
    expression = "".join("(.)" if part == "?" else "(.*)"
                         if part in {"*", "**"} else re.escape(part)
                         for part in parts)
    match = re.fullmatch(expression, source)
    target_parts = tokens.split(rule["target_pattern"])
    placeholders = sum(part in {"*", "**", "?"} for part in target_parts)
    if match is None or placeholders != len(match.groups()):
        fail("path-map.csv", "unpaired path template " + rule["id"])
        return None
    captures = iter(match.groups())
    return "".join(next(captures) if part in {"*", "**", "?"} else part
                   for part in target_parts)


def graph_check(nodes, kind):
    visiting, visited, order = set(), set(), []
    def visit(name):
        if name in visiting:
            fail(kind, "dependency cycle through " + name)
            return
        if name in visited:
            return
        visiting.add(name)
        for dependency in nodes[name]["dependencies"]:
            if dependency not in nodes:
                fail(kind, "unknown dependency " + name + " -> " + dependency)
                continue
            visit(dependency)
        visiting.remove(name)
        visited.add(name)
        order.append(name)
    for name, node in nodes.items():
        directories = node.get("directories", [node.get("directory", "")])
        if not directories or any(not directory or ".codex" in Path(directory).parts or ".." in Path(directory).parts
                                  for directory in directories):
            fail(kind, "invalid planned directory for " + name)
        for dependency in node["dependencies"]:
            if dependency not in nodes:
                continue
            provider = nodes[dependency]["domain"]
            domain = node["domain"]
            if kind == "native":
                allowed = {
                    "contracts": set(),
                    "foundation": {"contracts", "foundation"},
                    "simulation": {"contracts", "foundation", "simulation"},
                    "training": {"contracts", "foundation", "simulation", "training"},
                    "visualization": {"contracts", "foundation", "visualization"},
                }
            else:
                allowed = {
                    "extension": set(), "contracts": set(),
                    "simulation": {"extension", "contracts", "simulation"},
                    "training": {"extension", "contracts", "training"},
                    "visualization": {"extension", "contracts", "visualization"},
                }
            if domain in allowed and provider not in allowed[domain]:
                fail(kind, "forbidden domain edge " + name + " -> " + dependency)
        visit(name)
    return order


files = list(walk(HERE))
markdown = [path for path in files if path.suffix == ".md"]
if len(markdown) < 8:
    fail("discovery", "incomplete architecture document discovery")
doc_graph = {path.resolve(): set() for path in markdown}
for path in markdown:
    body = path.read_text(encoding="utf-8")
    if "\ufffd" in body:
        fail(path.relative_to(ROOT), "invalid replacement character")
    for match in LINK.finditer(body):
        target = match[1].strip().split(" ", 1)[0].strip("<>")
        if target.startswith(("http:", "https:", "mailto:", "#")):
            continue
        target = unquote(target.split("#", 1)[0])
        resolved = (path.parent / target).resolve()
        if ".codex" in resolved.parts:
            fail(path.relative_to(ROOT), "link enters excluded directory")
        elif not resolved.exists():
            fail(path.relative_to(ROOT), "missing local link: " + target)
        elif resolved in doc_graph:
            doc_graph[path.resolve()].add(resolved)

entry = (HERE / "README.md").resolve()
seen, todo = set(), [entry]
while todo:
    path = todo.pop()
    if path in seen:
        continue
    seen.add(path)
    todo.extend(doc_graph.get(path, ()))
for path in doc_graph:
    if path not in seen:
        fail(path.relative_to(ROOT), "unreachable from architecture README")

policy = json.loads((HERE / "validation/target-dependencies.json").read_text(encoding="utf-8"))
if policy.get("status") != "planned-not-current-enforcement":
    fail("policy", "must identify the graph as a target, not existing implementation acceptance")
native_order = graph_check(policy["native"], "native")
python_order = graph_check(policy["python"], "python")

# The native target graph names the registered modules (HC-24): the same names, targets, domains and directories as
# the architecture gate's policy, and it admits every edge that policy allows.
registered_policy = json.loads((ROOT / "tools/architecture/policy.json").read_text(encoding="utf-8"))
registered = {**registered_policy["modules"], **registered_policy["planned_modules"]}
for name in sorted(set(registered) ^ set(policy["native"])):
    fail("native", ("registered module missing from the target graph: " if name in registered
                    else "target graph module not registered in the policy: ") + name)
for name in sorted(set(registered) & set(policy["native"])):
    module, node = registered[name], policy["native"][name]
    if node["domain"] != module["domain"]:
        fail("native", "%s: domain %s, registered %s" % (name, node["domain"], module["domain"]))
    if node.get("directories") != module["destinations"]:
        fail("native", "%s: directories %s, registered destinations %s" % (name, node.get("directories"), module["destinations"]))
    if "target" in module and node.get("target") != module["target"]:
        fail("native", "%s: target %s, registered %s" % (name, node.get("target"), module["target"]))
    for dependency in module.get("allowed_dependencies", []):
        if dependency not in node["dependencies"]:
            fail("native", "%s: registered dependency %s is not in the target graph" % (name, dependency))

catalogs = {}
for filename in ("path-map.csv", "document-register.csv"):
    path = HERE / "migration" / filename
    with path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    catalogs[filename] = rows
    ids, patterns = set(), set()
    for row in rows:
        if row["id"] in ids:
            fail(filename, "duplicate id " + row["id"])
        ids.add(row["id"])
        key = (row["source_pattern"], row.get("priority", ""))
        if key in patterns:
            fail(filename, "duplicate source/priority " + row["source_pattern"])
        patterns.add(key)
        for field in ("source_pattern", "target_pattern"):
            parts = Path(row[field]).parts
            if ".codex" in parts or ".." in parts or Path(row[field]).is_absolute():
                fail(filename, "unsafe/excluded path " + row[field])
        source = row["source_pattern"]
        if "*" in source or "?" in source or "[" in source:
            prefix = re.split(r"[*?\[]", source, maxsplit=1)[0]
            directory = prefix if prefix.endswith("/") else prefix.rpartition("/")[0]
            if not (ROOT / directory).is_dir():
                fail(filename, "missing source root " + directory)
        elif not (ROOT / source).exists():
            fail(filename, "missing source " + source)
        if row["phase"] not in {"retain", *("ARCH-%02d" % n for n in range(11))}:
            fail(filename, "unknown phase " + row["phase"])

# Scoped coverage: all native implementation/public files and Python SDK sources
# need one winning path rule. Test/tool moves are catalogued migration units.
rules = catalogs["path-map.csv"]
coverage = 0
destinations = {}
for root in (ROOT / "src", ROOT / "include/fsim", ROOT / "python/src", ROOT / "python/fsim"):
    for path in walk(root):
        if path.suffix not in {".h", ".hpp", ".hh", ".inl", ".c", ".cpp", ".cc", ".cxx", ".py"}:
            continue
        relative = path.relative_to(ROOT).as_posix()
        matches = [r for r in rules if fnmatch.fnmatchcase(relative, r["source_pattern"])]
        if not matches:
            fail("path-map.csv", "unmapped source " + relative)
            continue
        best = max(int(r["priority"]) for r in matches)
        winners = [r for r in matches if int(r["priority"]) == best]
        if len(winners) != 1:
            fail("path-map.csv", "ambiguous winning owner for " + relative)
        else:
            target = destination(winners[0], relative)
            if target is not None:
                key = target.casefold()
                if key in destinations:
                    fail("path-map.csv", "destination collision: " + relative + " and "
                         + destinations[key] + " -> " + target)
                else:
                    destinations[key] = relative
        coverage += 1

if errors:
    for error in errors:
        print(error, file=sys.stderr)
    raise SystemExit(1)
print("Architecture plan accepted: %d Markdown documents, %d native and %d Python target owners; no target cycles." %
      (len(markdown), len(native_order), len(python_order)))
print("Catalogs: %d source units, %d document units; %d implementation/public files covered; local links valid." %
      (len(catalogs["path-map.csv"]), len(catalogs["document-register.csv"]), coverage))
print("Planning consistency only; no source migration, native build, runtime or GPU acceptance implied.")
