"""Python import ownership for the architecture gate (tools/architecture/check.py).

A literal-import AST scan of the public package (python/fsim): which module imports which, eagerly (module level,
class bodies and conditional branches included) or deferred (inside a function). It enforces:

- python-cycle: the module graph, deferred edges included, is acyclic once the policy's exact legacy backedges are
  set aside. A deferred import is an architecture edge even when initialization happens to work.
- python-exception: each legacy backedge exists, is deferred only, and still closes a cycle; otherwise it is stale.
- python-facade-backedge / python-forbidden-import: a private implementation package (fsim._simulation, ...) imports
  only what the policy lists for it - never the package root or a public facade module, not even function-locally.
- python-optional-eager: importing the package eagerly loads no optional module or optional external package.
- python-dynamic-import: importlib.import_module / __import__ appear only at registered sites.
- python-unknown-owner: every subpackage is a declared private package.

Compiled extensions (fsim._native, fsim._vision) are owner-level leaves; their native links are checked as native
targets. Dynamic imports, native initialization and external packages' own imports are not proven by this scan.
Standard library only."""

from __future__ import annotations

import ast
import os
from pathlib import Path

SPEC_KEYS = {"package_root", "package", "extensions", "private_packages", "optional_modules", "optional_packages",
             "dynamic_imports", "exceptions"}


def validate(spec):
    if set(spec) != SPEC_KEYS:
        raise ValueError("python policy keys: expected %s, got %s" % (sorted(SPEC_KEYS), sorted(spec)))
    package = spec["package"]
    for name in list(spec["private_packages"]) + spec["extensions"] + spec["optional_modules"]:
        if not name.startswith(package + "."):
            raise ValueError("python policy name outside the package: " + name)
    for name, allowed in spec["private_packages"].items():
        if not name.split(".")[1].startswith("_") or name.count(".") != 1:
            raise ValueError("a private package is a top-level underscored subpackage: " + name)
        for dependency in allowed:
            if dependency not in spec["private_packages"] and dependency not in spec["extensions"]:
                raise ValueError("python policy: %s may import unknown %s" % (name, dependency))
    for entry in spec["exceptions"]:
        if set(entry) != {"from", "to", "until", "reason"}:
            raise ValueError("python exception keys: " + str(sorted(entry)))
    for entry in spec["dynamic_imports"]:
        if set(entry) != {"module", "function", "reason"}:
            raise ValueError("python dynamic import keys: " + str(sorted(entry)))


def walk(root):
    """The .py files under `root`, never entering a .codex or __pycache__ directory."""
    for directory, children, files in os.walk(root, followlinks=False):
        children[:] = sorted(child for child in children if child not in {".codex", "__pycache__"})
        for name in sorted(files):
            if name.endswith(".py"):
                yield Path(directory) / name


def discover(repo, spec):
    """{module name: path} for the package; the extensions are leaves without a path."""
    root = repo / spec["package_root"]
    if not (root / "__init__.py").is_file():
        raise ValueError("python package root without __init__.py: " + spec["package_root"])
    modules = {}
    for path in walk(root):
        parts = list(path.relative_to(root).with_suffix("").parts)
        if parts[-1] == "__init__":
            parts.pop()
        modules[".".join([spec["package"], *parts])] = path
    for extension in spec["extensions"]:
        modules.setdefault(extension, None)
    return modules


class Imports(ast.NodeVisitor):
    """Literal imports of one module: package edges, external top-level names and dynamic-import sites."""

    def __init__(self, module, path, modules, package, aliases=()):
        self.module, self.path, self.modules, self.package = module, path, modules, package
        self.aliases = set(aliases)
        self.functions, self.edges, self.init_edges, self.external, self.dynamic = [], [], [], [], []

    def visit_FunctionDef(self, node):
        self.functions.append(node.name)
        self.generic_visit(node)
        self.functions.pop()

    visit_AsyncFunctionDef = visit_FunctionDef

    def visit_Lambda(self, node):
        self.functions.append("<lambda>")
        self.generic_visit(node)
        self.functions.pop()

    def record(self, target, node):
        deferred = bool(self.functions)
        if target in self.modules:
            self.edges.append({"from": self.module, "to": target, "line": node.lineno, "deferred": deferred})
            # Importing a.b.c runs each package on the way first: their initializers load what they import. These
            # are initialization edges, kept apart from explicit imports (the package root is always initialized
            # first and is no edge: counting it would invent a facade cycle for every submodule import).
            parts = target.split(".")
            for k in range(2, len(parts)):
                ancestor = ".".join(parts[:k])
                if ancestor in self.modules and self.modules[ancestor] is not None and ancestor != self.module:
                    self.init_edges.append({"from": self.module, "to": ancestor, "line": node.lineno, "deferred": deferred,
                                            "via": target})
        elif target.split(".")[0] != self.package:
            self.external.append({"name": target.split(".")[0], "line": node.lineno, "deferred": deferred})

    def visit_Import(self, node):
        for item in node.names:
            # `import a.b` runs `a` first: an edge to the deepest package module named, each package on the way included
            parts = item.name.split(".")
            found = False
            for k in range(len(parts), 0, -1):
                if ".".join(parts[:k]) in self.modules:
                    self.record(".".join(parts[:k]), node)
                    found = True
                    break
            if not found:
                self.record(item.name, node)

    def visit_ImportFrom(self, node):
        if node.level:
            base = self.module.split(".") if self.path.name == "__init__.py" else self.module.split(".")[:-1]
            if node.level > 1:
                base = base[:-(node.level - 1)]
            parent = ".".join(base + ([node.module] if node.module else []))
        else:
            parent = node.module or ""
        if node.level == 0 and node.module in {"importlib", "builtins"} and any(item.name == "*" for item in node.names):
            # a wildcard binds import_module / __import__ invisibly: their calls could not be found, so the form is refused
            self.site(node, "unsupported")
        for item in node.names:
            # `from p import m` imports module p.m when there is one, else a name of p (p itself is the edge)
            child = parent + "." + item.name
            self.record(child if child in self.modules else parent, node)

    def site(self, node, form):
        self.dynamic.append({"module": self.module, "function": ".".join(self.functions) or "<module>", "line": node.lineno,
                             "form": form})

    def visit_Call(self, node):
        func = node.func
        if isinstance(func, ast.Attribute) and func.attr in IMPORT_FUNCTIONS:
            self.site(node, "call")  # importlib.import_module(...), builtins.__import__(...), any module alias
            self.visit(func.value)
        elif isinstance(func, ast.Name) and (func.id == "__import__" or func.id in self.aliases):
            self.site(node, "call")  # __import__(...), or a bound name: from importlib import import_module [as x]
        elif isinstance(func, ast.Name) and func.id == "getattr" and len(node.args) >= 2 \
                and isinstance(node.args[1], ast.Constant) and node.args[1].value in IMPORT_FUNCTIONS:
            self.site(node, "unsupported")  # getattr(importlib, "import_module")
            self.visit(node.args[0])
        else:
            self.visit(func)
        for child in node.args + [keyword.value for keyword in node.keywords]:
            self.visit(child)

    # an import function referenced other than as a direct call (stored, passed or rebound) cannot be followed
    def visit_Name(self, node):
        if node.id == "__import__" or node.id in self.aliases:
            self.site(node, "unsupported")

    def visit_Attribute(self, node):
        if node.attr in IMPORT_FUNCTIONS:
            self.site(node, "unsupported")
        self.generic_visit(node)


IMPORT_FUNCTIONS = {"import_module", "__import__"}


def import_function_aliases(tree):
    """Names bound to an import function anywhere in a module: from importlib import import_module [as x], from
    builtins import __import__ [as y]. Scope-insensitive: a binding anywhere counts everywhere (conservative)."""
    names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.level == 0 and node.module in {"importlib", "builtins"}:
            for item in node.names:
                if item.name in IMPORT_FUNCTIONS:
                    names.add(item.asname or item.name)
    return names


def components(graph):
    """Strongly connected components with more than one member (or a self edge), each sorted."""
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


def check(repo, spec):
    """(issues, facts): the policy's rules over the package's literal imports."""
    validate(spec)
    issues = []

    def issue(code, place, detail):
        issues.append(f"{code}: {place}: {detail}")

    package = spec["package"]
    modules = discover(repo, spec)
    edges, init_edges, external, dynamic = [], [], {}, []
    for module, path in sorted((m, p) for m, p in modules.items() if p is not None):
        try:
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        except SyntaxError as error:
            issue("python-syntax", path.relative_to(repo).as_posix(), str(error))
            continue
        scan = Imports(module, path, modules, package, import_function_aliases(tree))
        scan.visit(tree)
        unique = {(e["to"], e["line"], e["deferred"]): e for e in scan.edges if e["to"] != module}
        edges += [unique[key] for key in sorted(unique)]
        unique = {(e["to"], e["line"], e["deferred"]): e for e in scan.init_edges}
        init_edges += [unique[key] for key in sorted(unique)]
        external[module] = scan.external
        dynamic += scan.dynamic

    def place(module, line=None):
        path = modules.get(module)
        where = path.relative_to(repo).as_posix() if path else module
        return f"{where}:{line}" if line else where

    # owners: the facade's own top-level modules, a declared private package, or an extension
    private = spec["private_packages"]
    owners = {}
    for module, path in modules.items():
        parts = module.split(".")
        if module in spec["extensions"]:
            owners[module] = module
        elif len(parts) == 1 or (len(parts) == 2 and path is not None and path.name != "__init__.py"):
            owners[module] = package
        elif ".".join(parts[:2]) in private:
            owners[module] = ".".join(parts[:2])
        else:
            owners[module] = None
            issue("python-unknown-owner", place(module), "subpackage %s is not a declared private package" % ".".join(parts[:2]))

    # private implementation packages import only what they list (themselves, other packages listed, extensions listed)
    for edge in edges:
        source, target = owners.get(edge["from"]), owners.get(edge["to"])
        if source in private and source != target:
            kind = "deferred" if edge["deferred"] else "eager"
            if target == package:
                issue("python-facade-backedge", place(edge["from"], edge["line"]),
                      f"{edge['from']} -> {edge['to']} ({kind}): a private implementation imports the public facade")
            elif target not in private[source]:
                issue("python-forbidden-import", place(edge["from"], edge["line"]),
                      f"{edge['from']} -> {edge['to']} ({kind}): {source} may import {', '.join(private[source]) or 'nothing'}")

    # cycles, deferred edges included, with the exact legacy backedges set aside
    excepted = {(entry["from"], entry["to"]) for entry in spec["exceptions"]}
    graph = {module: set() for module in modules}
    for edge in edges:
        if (edge["from"], edge["to"]) not in excepted:
            graph[edge["from"]].add(edge["to"])
    for part in components(graph):
        inside = sorted({"%s -> %s%s" % (e["from"], e["to"], " (deferred)" if e["deferred"] else "")
                         for e in edges if e["from"] in part and e["to"] in part and (e["from"], e["to"]) not in excepted})
        issue("python-cycle", place(part[0]), "; ".join(inside))
    with_all = {module: set() for module in modules}
    for edge in edges:
        with_all[edge["from"]].add(edge["to"])

    def reaches(start, goal):
        seen, pending = {start}, [start]
        while pending:
            for nxt in with_all[pending.pop()]:
                if nxt == goal:
                    return True
                if nxt not in seen:
                    seen.add(nxt)
                    pending.append(nxt)
        return start == goal

    for entry in spec["exceptions"]:
        found = [e for e in edges if (e["from"], e["to"]) == (entry["from"], entry["to"])]
        name = f"{entry['from']} -> {entry['to']}"
        if not found:
            issue("python-exception", "policy", f"stale legacy backedge {name}: no such import")
        elif any(not e["deferred"] for e in found):
            issue("python-exception", place(entry["from"], next(e["line"] for e in found if not e["deferred"])),
                  f"legacy backedge {name} is now eager")
        elif not reaches(entry["to"], entry["from"]):
            # the edge itself must close a cycle: its target reaches back to its source (being in some cycle is not enough)
            issue("python-exception", "policy", f"legacy backedge {name} no longer closes a cycle: remove the exception")

    # the package's eager closure loads no optional module or optional external package; an imported module's package
    # initializers run too (initialization edges)
    reached, pending = {package}, [package]
    while pending:
        current = pending.pop()
        for edge in edges + init_edges:
            if edge["from"] == current and not edge["deferred"] and edge["to"] not in reached:
                reached.add(edge["to"])
                pending.append(edge["to"])
    for module in sorted(reached):
        if module in spec["optional_modules"]:
            issue("python-optional-eager", place(module), f"importing {package} eagerly imports optional {module}")
        for item in external.get(module, []):
            if not item["deferred"] and item["name"] in spec["optional_packages"]:
                issue("python-optional-eager", place(module, item["line"]),
                      f"importing {package} eagerly imports optional package {item['name']} (through {module})")

    # dynamic imports at registered sites only
    registered = {(entry["module"], entry["function"]) for entry in spec["dynamic_imports"]}
    seen = set()
    for site in dynamic:
        key = (site["module"], site["function"])
        if site["form"] == "unsupported":
            issue("python-dynamic-import", place(site["module"], site["line"]),
                  f"unsupported dynamic-import form in {site['function']}: call importlib.import_module or a name bound by "
                  "`from importlib import import_module` (not a wildcard import) directly, at a registered site")
            continue
        seen.add(key)
        if key not in registered:
            issue("python-dynamic-import", place(site["module"], site["line"]),
                  f"unregistered dynamic import in {site['function']}")
    for module, function in sorted(registered - seen):
        issue("python-dynamic-import", "policy", f"stale registered dynamic import {module}:{function}")

    facts = {
        "modules": {m: (p.relative_to(repo).as_posix() if p else None) for m, p in sorted(modules.items())},
        "owners": {m: o for m, o in sorted(owners.items())},
        "edges": sorted(edges, key=lambda e: (e["from"], e["to"], e["line"])),
        "init_edges": sorted(init_edges, key=lambda e: (e["from"], e["to"], e["line"])),
        "external": {m: sorted({i["name"] for i in items}) for m, items in sorted(external.items())},
        "dynamic_imports": dynamic,
        "cycles_all": components(with_all),
        "cycles_eager": components({m: {e["to"] for e in edges if e["from"] == m and not e["deferred"]} for m in modules}),
        "eager_closure": sorted(reached),
    }
    return issues, facts
