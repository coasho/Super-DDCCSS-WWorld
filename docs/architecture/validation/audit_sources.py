"""Read-only, scoped dependency evidence for the architecture planning documents."""
import argparse
import ast
import datetime
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[3]
PARSER = argparse.ArgumentParser(description=__doc__)
PARSER.add_argument("--date", required=True, help="Evidence date in the client timezone, YYYY-MM-DD")
ARGS = PARSER.parse_args()
datetime.date.fromisoformat(ARGS.date)
SPEC = importlib.util.spec_from_file_location("native_architecture_check", ROOT / "tools/architecture/check.py")
CHECK = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CHECK)


def components(graph):
    index, low, stack, active, result = {}, {}, [], set(), []
    def visit(v):
        index[v] = low[v] = len(index)
        stack.append(v)
        active.add(v)
        for w in sorted(graph[v]):
            if w not in index:
                visit(w)
                low[v] = min(low[v], low[w])
            elif w in active:
                low[v] = min(low[v], index[w])
        if low[v] == index[v]:
            part = []
            while True:
                w = stack.pop()
                active.remove(w)
                part.append(w)
                if w == v:
                    break
            if len(part) > 1 or v in graph[v]:
                result.append(sorted(part))
    for v in sorted(graph):
        if v not in index:
            visit(v)
    return sorted(result)


policy_path = ROOT / "tools/architecture/policy.json"
policy = json.loads(policy_path.read_text(encoding="utf-8"))
issues, module_count, source_count = CHECK.check(ROOT, policy)
if source_count < 10:
    raise RuntimeError("Incomplete source discovery: this execution context cannot read the source tree")
names = {}
for module, item in policy["modules"].items():
    for name in [item["target"], *item["aliases"]]:
        names[name] = module
graph = {m: set() for m in policy["modules"]}
for path in CHECK.files_under(ROOT / "src"):
    if path.name != "CMakeLists.txt":
        continue
    for command, args in CHECK.cmake_commands(path.read_text(encoding="utf-8")):
        if command != "target_link_libraries" or args[0] not in names:
            continue
        for value in args[1:]:
            candidates = value[len("$<LINK_LIBRARY:WHOLE_ARCHIVE,"):-1].split(",") if value.startswith("$<LINK_LIBRARY:WHOLE_ARCHIVE,") else value.split(";")
            for candidate in candidates:
                if candidate in names:
                    graph[names[args[0]]].add(names[candidate])

package = ROOT / "python/fsim"
modules = {}
for path in CHECK.files_under(package):
    if path.suffix == ".py":
        parts = list(path.relative_to(package).with_suffix("").parts)
        if parts[-1] == "__init__":
            parts.pop()
        modules[".".join(["fsim", *parts])] = path
python_module_count = len(modules)
for extension in ("fsim._native", "fsim._vision"):
    modules[extension] = None  # extension leaves, not a false backedge to the package facade
imports = []
python_graph = {m: set() for m in modules}


class Imports(ast.NodeVisitor):
    def __init__(self, module, path):
        self.module, self.path, self.depth = module, path, 0
    def visit_FunctionDef(self, node):
        self.depth += 1
        self.generic_visit(node)
        self.depth -= 1
    visit_AsyncFunctionDef = visit_FunctionDef
    def edge(self, target, node):
        if target in modules:
            python_graph[self.module].add(target)
            imports.append({"from": self.module, "to": target, "path": self.path.relative_to(ROOT).as_posix(), "line": node.lineno, "deferred": self.depth > 0})
    def visit_Import(self, node):
        for item in node.names:
            self.edge(item.name, node)
    def visit_ImportFrom(self, node):
        if node.level:
            base = self.module.split(".") if self.path.name == "__init__.py" else self.module.split(".")[:-1]
            if node.level > 1:
                base = base[:-(node.level - 1)]
            parent = ".".join(base + ([node.module] if node.module else []))
        else:
            parent = node.module or ""
        selected = False
        for item in node.names:
            child = parent + "." + item.name
            if child in modules:
                self.edge(child, node)
                selected = True
        if not selected:
            self.edge(parent, node)


for module, path in modules.items():
    if path is not None:
        Imports(module, path).visit(ast.parse(path.read_text(encoding="utf-8"), filename=str(path)))
eager = {m: set() for m in modules}
for item in imports:
    if not item["deferred"]:
        eager[item["from"]].add(item["to"])
result = {
    "schema": "fsim.architecture.audit.v1", "date": ARGS.date, "head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
    "scope": {"native": ["src", "include/fsim", "third_party header names"], "python": ["python/fsim"], "excluded_directory": ".codex", "configured_build_executed": False},
    "native": {"checker_modules": module_count, "checker_source_and_public_files": source_count, "checker_issues": issues,
               "direct_edges": {m: sorted(deps) for m, deps in sorted(graph.items())}, "cycles": components(graph)},
    "python": {"module_count": python_module_count, "native_extension_leaves": ["fsim._native", "fsim._vision"], "import_edges": imports, "eager_cycles": components(eager), "all_literal_import_cycles": components(python_graph),
               "limits": ["AST literal-import analysis only", "conditional branches are combined", "function-local imports marked deferred", "dynamic imports/native initialization not proven by this scan", "implicit package initialization is excluded from owner-level edges", "native extension aliases are treated as leaves, not root-package backedges"]},
    "policy_sha256": hashlib.sha256(policy_path.read_bytes()).hexdigest(),
    "limitations": ["Working-tree scan; not a clean-commit-only scan", "No native configure/build/runtime/GPU test was run", "A source graph is not a configured linker or CPython runtime proof"]
}
destination = ROOT / "docs/architecture/current-state/dependency-audit.json"
destination.parent.mkdir(parents=True, exist_ok=True)
destination.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
print(json.dumps({"native_modules": module_count, "native_files": source_count, "native_issues": issues, "native_cycles": result["native"]["cycles"],
                  "python_modules": python_module_count, "eager_cycles": result["python"]["eager_cycles"], "literal_import_cycles": result["python"]["all_literal_import_cycles"]}, indent=2))
