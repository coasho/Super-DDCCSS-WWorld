"""Check all native source modules against their declared CMake dependency graph.

The deliberately limited source parser reads every conditional branch; it does
not configure CMake or evaluate variables. Unsupported source/link declarations
fail closed. See README.md for the accepted CMake subset and ownership policy.

Each module also has a domain (contracts, foundation, simulation, training,
visualization, adapters, applications) and a kind (contract, implementation,
final assembly, application). Their direction rules hold on links and on includes
of another module's header; the policy's few legacy exceptions are exact and
expire. Source ownership extends to the destination directories of the staged
migration (longest prefix wins; planned owners reserve theirs), and the Python
package's imports are checked by python_imports.py.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import python_imports  # noqa: E402


SOURCE_SUFFIXES = {".h", ".hpp", ".hh", ".inl", ".c", ".cpp", ".cc", ".cxx"}
GPU = re.compile(r"(?:^|::|/)(?:vsg\w*|vulkan)(?:\b|/)|^NvFlow", re.IGNORECASE)
INCLUDE = re.compile(r'^[\t ]*#[\t ]*include[\t ]*[<"]([^>"\n]+)[>"]', re.MULTILINE)
INCLUDE_DIRECTIVE = re.compile(r'^[\t ]*#[\t ]*include\b[^\n]*', re.MULTILINE)
# CMake tokens preserve quoted arguments and bracket arguments as one token.
CMAKE_TOKEN = re.compile(r'"(?:\\.|[^"\\])*"|\[(=*)\[.*?\]\1\]|[^\s()]+|[()]', re.DOTALL)
PRIVATE_NAMESPACES = "session|env|ipc"
PRIVATE_TYPE = re.compile(
    rf"\bnamespace\s+(?:(?:fsim\s*::\s*)?(?:{PRIVATE_NAMESPACES}))\b"
    rf"|::\s*(?:{PRIVATE_NAMESPACES})\b|\b(?:{PRIVATE_NAMESPACES})\s*::"
    rf"|\bnamespace\s+\w+\s*=\s*(?:{PRIVATE_NAMESPACES})\b")


def files_under(root):
    """Walk an explicit source root without entering excluded private directories."""
    for directory, children, files in os.walk(root, followlinks=False):
        children[:] = [child for child in children if child != ".codex"]
        for name in files:
            yield Path(directory) / name


def public_type_text(clean, place):
    """Retain native namespace references except the exact legacy World facade seams.

    This is a declaration guard, not a C++ type resolver or an actor sandbox. A
    future actor facade still needs runtime permission and isolation acceptance.
    """
    clean = re.sub(r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'', lambda match: blank(match[0]), clean)
    if place != "include/fsim/World.h":
        return clean

    depths, depth = [], 0
    for character in clean:
        depths.append(depth)
        depth += (character == "{") - (character == "}")
    spans = []

    def statement(source):
        tokens = re.findall(r"::|\w+|[^\w\s]", source)
        return re.compile(r"(?<!\w)" + r"\s*".join(re.escape(token) for token in tokens) + r"(?!\w)")

    # Preserve the existing declaration only, never a whole public header or namespace.
    forward = statement("namespace session { class World; }")
    for match in forward.finditer(clean):
        if depths[match.start()] == 1:
            spans.append(match.span())
            break
    world = re.search(r"\bclass\s+FSIM_API\s+World\s*\{", clean)
    if world and depths[world.start()] == 1:
        start = world.end()
        member_depth = depths[start - 1] + 1
        end = next((at for at in range(start, len(clean))
                    if clean[at] == "}" and depths[at] == member_depth), len(clean))
        seams = (
            "session::World& impl() noexcept { return *impl_; }",
            "static World* borrowed(session::World& w) { return new World(Borrow{}, w); }",
            "World(Borrow, session::World& borrowed);",
            "std::shared_ptr<session::World> impl_;",
        )
        for source in seams:
            for match in statement(source).finditer(clean, start, end):
                if depths[match.start()] == member_depth:
                    spans.append(match.span())
                    break
    for start, end in reversed(sorted(spans)):
        clean = clean[:start] + blank(clean[start:end]) + clean[end:]
    return clean


def blank(text):
    return re.sub(r"[^\n]", " ", text)


def without_comments(text, cmake=False):
    """Preserve line numbers and include strings; remove comments/raw literals."""
    if cmake:
        pattern = re.compile(r'"(?:\\.|[^"\\])*"|#\[(=*)\[.*?\]\1\]|#[^\n]*|\[(=*)\[.*?\]\2\]', re.DOTALL)
        return pattern.sub(lambda match: blank(match[0]) if match[0].startswith("#") else match[0], text)
    pattern = re.compile(r'R"([^ ()\\\t\r\n]{0,16})\(.*?\)\1"|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|//[^\n]*|/\*.*?\*/', re.DOTALL)
    return pattern.sub(lambda match: blank(match[0]) if match[0].startswith(("//", "/*", 'R"')) else match[0], text)


def cmake_commands(text):
    tokens = [match[0] for match in CMAKE_TOKEN.finditer(without_comments(text, cmake=True))]
    at = 0
    while at + 1 < len(tokens):
        name = tokens[at]
        if not re.fullmatch(r"[A-Za-z_]\w*", name) or tokens[at + 1] != "(":
            raise ValueError(f"unexpected CMake token {name}")
        at += 2
        depth, args = 1, []
        while at < len(tokens) and depth:
            token = tokens[at]
            at += 1
            depth += (token == "(") - (token == ")")
            if depth:
                if token.startswith('"'):
                    token = token[1:-1]
                args.append(token)
        if depth:
            raise ValueError(f"unclosed CMake command {name}")
        yield name.lower(), args
    if at != len(tokens):
        raise ValueError("incomplete CMake command")


KINDS = {"contract", "implementation", "assembly", "application"}
DOMAIN_KEYS = {"root", "kind", "may_depend_on"}
MODULE_KEYS = {"target", "aliases", "domain", "kind", "directories", "destinations", "files", "headless", "allowed_dependencies"}
PLANNED_KEYS = {"domain", "kind", "destinations", "phase"}
EXCEPTION_KEYS = {"from", "to", "rule", "until", "reason"}
POLICY_KEYS = {"schema", "domains", "modules", "planned_modules", "exceptions", "external_targets", "python"}
SCHEMA = "fsim.architecture.source-policy.v2"
PHASE = re.compile(r"ARCH-(0[2-9]|10)|MS-\d+")


def validate_policy(policy):
    """Structural policy errors fail the run as input errors: an unsupported declaration is never ignored."""
    if set(policy) != POLICY_KEYS or policy.get("schema") != SCHEMA:
        raise ValueError(f"policy must be {SCHEMA} with keys {sorted(POLICY_KEYS)}; got {sorted(policy)}")
    domains, modules, planned = policy["domains"], policy["modules"], policy["planned_modules"]
    roots = {}
    for name, domain in domains.items():
        if set(domain) != DOMAIN_KEYS or domain["kind"] not in KINDS:
            raise ValueError(f"domain {name}: keys {sorted(DOMAIN_KEYS)} and a kind of {sorted(KINDS)}")
        root = domain["root"]
        if not root.startswith("src/") or root.count("/") != 1 or root in roots:
            raise ValueError(f"domain {name}: root must be a distinct src/<name> directory: {root}")
        roots[root] = name
        for provider in domain["may_depend_on"]:
            if provider not in domains:
                raise ValueError(f"domain {name} may depend on unknown domain {provider}")
    owners = {}
    for group, entries, keys in (("module", modules, MODULE_KEYS), ("planned module", planned, PLANNED_KEYS)):
        for name, module in entries.items():
            if set(module) != keys:
                raise ValueError(f"{group} {name}: keys must be {sorted(keys)}; got {sorted(module)}")
            if name in modules and group == "planned module":
                raise ValueError(f"planned module {name} is already a module")
            domain = domains.get(module["domain"])
            if domain is None or domain["kind"] != module["kind"]:
                raise ValueError(f"{group} {name}: kind {module['kind']} does not belong to domain {module['domain']}")
            if group == "planned module" and not PHASE.fullmatch(module["phase"]):
                raise ValueError(f"planned module {name}: phase must name a later package: {module['phase']}")
            if group == "module" and not module["headless"] and module["kind"] == "implementation" \
                    and module["domain"] != "visualization":
                raise ValueError(f"module {name}: graphics (headless false) in the {module['domain']} domain")
            root = domain["root"]
            for destination in module["destinations"]:
                if destination != root and not destination.startswith(root + "/"):
                    raise ValueError(f"{group} {name}: destination {destination} is outside its domain root {root}")
            for directory in module.get("directories", []):
                if any(directory == r or directory.startswith(r + "/") for r in roots):
                    raise ValueError(f"module {name}: legacy directory {directory} lies in a domain root; list it as a destination")
            for path in module.get("directories", []) + module["destinations"] + module.get("files", []):
                if ".." in Path(path).parts or ".codex" in Path(path).parts or not path.startswith("src/"):
                    raise ValueError(f"{group} {name}: unsafe or non-source path {path}")
                if path in owners:
                    raise ValueError(f"duplicate-owner: {path} is claimed by {owners[path]} and {name}")
                owners[path] = name
    # An exact file claim keeps the existing legacy shared directories (Flow in src/world, the Viewer in src/app) but
    # never overrides another owner: inside a domain root it lies in the module's own destination, and in a legacy
    # directory it belongs to a module of the same domain. It cannot take a reserved or foreign file.
    directory_claims = [(path.rstrip("/"), name, group) for group, entries in (("module", modules), ("planned", planned))
                        for name, module in entries.items() for path in module.get("directories", []) + module["destinations"]]
    for name, module in modules.items():
        for path in module["files"]:
            covering = sorted((len(directory), owner, group) for directory, owner, group in directory_claims
                              if path.startswith(directory + "/"))
            inside = next((d for d, entry in domains.items() if path.startswith(entry["root"] + "/")), None)
            here = f"file-claim: {name} ({module['domain']}) claims {path}"
            if covering:
                _, owner, group = covering[-1]
                if group == "planned":
                    raise ValueError(f"{here}, reserved for planned module {owner} ({planned[owner]['phase']})")
                if inside and owner != name:
                    raise ValueError(f"{here} in the destination of {owner} ({modules[owner]['domain']})")
                if modules[owner]["domain"] != module["domain"]:
                    raise ValueError(f"{here} in the legacy directory of {owner} ({modules[owner]['domain']})")
            elif inside:
                raise ValueError(f"{here} inside the {inside} domain root, outside its own destinations")
    for module in modules.values():
        for dependency in module["allowed_dependencies"]:
            if dependency not in modules:
                raise ValueError(f"unknown policy dependency -> {dependency}")
    for entry in policy["exceptions"]:
        if set(entry) != EXCEPTION_KEYS or entry["rule"] not in {"domain", "kind"}:
            raise ValueError(f"exception keys must be {sorted(EXCEPTION_KEYS)} with rule domain|kind: {entry}")
        if entry["from"] not in modules or entry["to"] not in modules or not PHASE.fullmatch(entry["until"]):
            raise ValueError(f"exception {entry['from']} -> {entry['to']}: unknown module or phase {entry['until']}")
    return domains, modules, planned


def edge_rule(modules, domains, consumer, provider):
    """Which architecture rule a direct consumer -> provider edge breaks, if any: ("kind"|"domain", reason)."""
    c, p = modules[consumer], modules[provider]
    if p["kind"] == "application":
        return "kind", "nothing depends on an application"
    if c["kind"] in {"contract", "implementation"} and p["kind"] == "assembly":
        return "kind", "a domain implementation or contract depends on a final assembly"
    if c["kind"] == "contract" and p["kind"] != "contract":
        return "kind", "a contract depends on an implementation"
    if c["kind"] == "implementation" and p["kind"] == "implementation" and p["domain"] != c["domain"] \
            and p["domain"] not in domains[c["domain"]]["may_depend_on"]:
        return "domain", f"{c['domain']} may depend on {', '.join(domains[c['domain']]['may_depend_on']) or 'nothing'}, not {p['domain']}"
    return None


def check(repo, policy):
    """The historic entry point: (issues, native module count, native source/public file count)."""
    issues, facts = analyze(repo, policy)
    return issues, facts["module_count"], facts["file_count"]


def analyze(repo, policy):
    """(issues, facts): every rule over the source tree; facts are the actual graphs the rules were applied to."""
    domains, modules, planned = validate_policy(policy)
    targets, aliases = {}, {}
    for name, module in modules.items():
        if module["target"] in targets:
            raise ValueError(f"duplicate registered target {module['target']}")
        targets[module["target"]] = name
        for alias in module["aliases"]:
            if alias in aliases or alias in targets:
                raise ValueError(f"duplicate registered alias {alias}")
            aliases[alias] = name
    names = targets | aliases
    external = set(policy["external_targets"])
    issues, declared, declared_aliases = [], set(), set()
    graph = {name: set() for name in modules}
    source_declarations, link_declarations, include_declarations = [], [], []
    include_directories = {name: [] for name in modules}
    target_types, external_links = {}, {name: set() for name in modules}
    # ownership claims: exact files first, then the longest directory prefix (legacy or destination; planned owners too)
    claims = []
    for group, entries in (("module", modules), ("planned", planned)):
        for name, module in entries.items():
            for directory in module.get("directories", []) + module["destinations"]:
                claims.append((directory.rstrip("/"), name, group))

    def issue(code, place, detail):
        issues.append(f"{code}: {place}: {detail}")

    def owner(path):
        relative = path.relative_to(repo).as_posix()
        exact = [name for name, module in modules.items() if relative in module["files"]]
        if len(exact) == 1:
            return exact[0]
        matches = [(len(directory), name, group) for directory, name, group in claims if relative.startswith(directory + "/")]
        if not matches:
            root = next((d for d, entry in domains.items() if relative.startswith(entry["root"] + "/")), None)
            where = f" (inside the {root} domain root, outside every declared destination)" if root else ""
            issue("unknown-owner", relative, "source needs exactly one registered module" + where)
            return None
        longest = max(length for length, _, _ in matches)
        winners = sorted({(name, group) for length, name, group in matches if length == longest})
        if len(winners) != 1:
            issue("duplicate-owner", relative, "claimed by " + " and ".join(name for name, _ in winners))
            return None
        name, group = winners[0]
        if group == "planned":
            issue("planned-owner", relative, f"reserved for planned module {name} ({planned[name]['phase']}); "
                  "register its target in the policy before adding sources")
            return None
        return name

    def include_directory(cmake, argument):
        """Resolve the literal source/build include forms used by native targets."""
        install = re.fullmatch(r"\$<INSTALL_INTERFACE:([^<>]+)>", argument)
        if install:
            return None
        build = re.fullmatch(r"\$<BUILD_INTERFACE:([^<>]+)>", argument)
        if build:
            argument = build[1]
        for variable, value in (("CMAKE_SOURCE_DIR", repo), ("PROJECT_SOURCE_DIR", repo),
                                ("CMAKE_CURRENT_SOURCE_DIR", cmake.parent), ("CMAKE_CURRENT_LIST_DIR", cmake.parent)):
            argument = argument.replace("${" + variable + "}", value.as_posix())
        if any(character in argument for character in "$<>;"):
            raise ValueError(f"nonliteral include directory {argument}")
        directory = (cmake.parent / argument).resolve()
        if not directory.is_relative_to(repo):
            raise ValueError(f"include directory outside repository: {argument}")
        return directory

    for cmake in sorted(path for path in files_under(repo / "src") if path.name == "CMakeLists.txt"):
        place = cmake.relative_to(repo).as_posix()
        try:
            commands = list(cmake_commands(cmake.read_text(encoding="utf-8")))
        except ValueError as error:
            issue("unsupported-cmake", place, str(error))
            continue
        for command, args in commands:
            if command in {"link_libraries", "include_directories", "include", "function", "macro", "cmake_language"}:
                issue("unsupported-cmake", place, f"{command} can hide target declarations/dependencies")
            elif command in {"set_property", "set_target_properties"}:
                graph_properties = {"LINK_LIBRARIES", "INTERFACE_LINK_LIBRARIES", "INTERFACE_LINK_LIBRARIES_DIRECT",
                                    "IMPORTED_LINK_INTERFACE_LIBRARIES", "SOURCES", "INTERFACE_SOURCES",
                                    "INCLUDE_DIRECTORIES", "INTERFACE_INCLUDE_DIRECTORIES", "INTERFACE_SYSTEM_INCLUDE_DIRECTORIES"}
                if graph_properties.intersection(args):
                    issue("unsupported-cmake", place, f"graph property mutation: {' '.join(args)}")
            elif command == "set_source_files_properties" and "INCLUDE_DIRECTORIES" in args:
                try:
                    directory = include_directory(cmake, args[args.index("INCLUDE_DIRECTORIES") + 1])
                    # Existing NTIA compile wrappers use a vendor-only source path.
                    if directory is None or not directory.is_relative_to(repo / "third_party"):
                        raise ValueError("source include directory must be vendor-only")
                except (ValueError, IndexError) as error:
                    issue("unsupported-cmake", place, f"source include directory: {error}")
            if command in {"add_library", "add_executable"}:
                if not args:
                    issue("unsupported-cmake", place, f"empty {command}")
                    continue
                target = args[0]
                if len(args) > 1 and args[1] == "ALIAS":
                    if len(args) != 3 or target not in aliases or args[2] not in targets or aliases[target] != targets[args[2]]:
                        issue("unknown-target", place, f"unregistered alias {' '.join(args)}")
                    else:
                        declared_aliases.add(target)
                    continue
                if target not in targets:
                    issue("unknown-target", place, target)
                    continue
                declared.add(target)
                flags = {"STATIC", "SHARED", "MODULE", "OBJECT", "INTERFACE", "EXCLUDE_FROM_ALL", "WIN32", "MACOSX_BUNDLE"}
                kind = "EXECUTABLE" if command == "add_executable" else next((a for a in args[1:] if a in flags - {"EXCLUDE_FROM_ALL"}), "DEFAULT")
                target_types[target] = kind
                source_declarations.extend((cmake, target, source) for source in args[1:] if source not in flags)
            elif command == "target_sources":
                if args:
                    source_declarations.extend((cmake, args[0], source) for source in args[1:]
                                               if source not in {"PRIVATE", "PUBLIC", "INTERFACE"})
            elif command == "target_link_libraries":
                if args:
                    link_declarations.extend((place, args[0], link) for link in args[1:]
                                             if link not in {"PRIVATE", "PUBLIC", "INTERFACE", "debug", "optimized", "general"})
            elif command == "target_include_directories":
                if args:
                    include_declarations.extend((cmake, args[0], directory) for directory in args[1:]
                                                if directory not in {"PRIVATE", "PUBLIC", "INTERFACE", "SYSTEM", "BEFORE", "AFTER"})

    for target in sorted(set(targets) - declared):
        issue("missing-target", "src", target)
    for alias in sorted(set(aliases) - declared_aliases):
        issue("missing-alias", "src", alias)
    for cmake, target, argument in include_declarations:
        place = cmake.relative_to(repo).as_posix()
        if target not in targets:
            issue("unknown-target", place, target)
            continue
        try:
            directory = include_directory(cmake, argument)
            if directory is not None:
                include_directories[targets[target]].append(directory)
        except ValueError as error:
            issue("unsupported-cmake", place, str(error))
    declared_by = {}
    for cmake, target, source in source_declarations:
        place = cmake.relative_to(repo).as_posix()
        if target not in targets:
            issue("unknown-target", place, target)
            continue
        if "$" in source or ";" in source or source in {"(", ")"}:
            issue("unsupported-cmake", place, f"nonliteral source {source}")
            continue
        path = (cmake.parent / source).resolve()
        if not path.is_relative_to(repo / "src") or not path.is_file():
            issue("source-owner", place, f"{target} source is absent/outside src: {source}")
            continue
        relative = path.relative_to(repo).as_posix()
        declared_by.setdefault(relative, set()).add(target)
        actual = owner(path)
        if actual and actual != targets[target]:
            issue("source-owner", place, f"{actual} owns {relative}, declared by {targets[target]}")
    for relative, owners_of in sorted(declared_by.items()):
        if len(owners_of) > 1:
            issue("duplicate-source", relative, "listed by targets " + ", ".join(sorted(owners_of)))

    for place, target, argument in link_declarations:
        if target not in targets:
            issue("unknown-target", place, target)
            continue
        module = targets[target]
        whole = re.fullmatch(r"\$<LINK_LIBRARY:WHOLE_ARCHIVE,([^<>$]+)>", argument)
        if whole:
            links = whole[1].split(",")
        elif "$" in argument or "<" in argument or ">" in argument:
            issue("unsupported-cmake", place, f"nonliteral link {argument}")
            continue
        else:
            links = argument.split(";")
        for link in links:
            if modules[module]["headless"] and GPU.search(link):
                issue("headless-gpu", place, f"{module} links {link}")
            if link in names:
                dependency = names[link]
                graph[module].add(dependency)
                if dependency not in modules[module]["allowed_dependencies"]:
                    issue("forbidden-link", place, f"{module} -> {dependency}")
                if modules[module]["headless"] and not modules[dependency]["headless"]:
                    issue("headless-gpu", place, f"{module} -> {dependency}")
            elif link in external:
                external_links[module].add(link)
            else:
                issue("unknown-link", place, f"{module} links unregistered {link}")

    # an allowance is policy, not evidence: every allowed edge must be an actual link
    for module in sorted(modules):
        for dependency in sorted(set(modules[module]["allowed_dependencies"]) - graph[module]):
            issue("unused-allowance", "policy", f"{module} -> {dependency} is allowed but not linked; remove it")

    visited, active = set(), []

    def visit(module):
        if module in active:
            issue("dependency-cycle", "CMake graph", " -> ".join(active[active.index(module):] + [module]))
            return
        if module in visited:
            return
        active.append(module)
        for dependency in sorted(graph[module]):
            visit(dependency)
        active.pop()
        visited.add(module)

    for module in sorted(modules):
        visit(module)

    def closure(module):
        reached, pending = {module}, [module]
        while pending:
            for dependency in graph[pending.pop()] - reached:
                reached.add(dependency)
                pending.append(dependency)
        return reached

    reachable = {module: closure(module) for module in modules}
    search_directories = {module: list(dict.fromkeys(directory for dependency in sorted(reachable[module])
                                                    for directory in include_directories[dependency]))
                          for module in modules}
    vendor_headers = {path.name for path in files_under(repo / "third_party")
                      if path.is_file() and path.suffix in SOURCE_SUFFIXES}
    sources = [path for path in files_under(repo / "src") if path.is_file() and path.suffix in SOURCE_SUFFIXES]
    public = [path for path in files_under(repo / "include/fsim") if path.is_file() and path.suffix in SOURCE_SUFFIXES]
    source_owner, include_edges, public_consumers = {}, {}, {}
    for path in sorted(sources + public):
        place = path.relative_to(repo).as_posix()
        is_public = path.is_relative_to(repo / "include/fsim")
        module = None if is_public else owner(path)
        if not is_public:
            source_owner[place] = module
        clean = without_comments(path.read_text(encoding="utf-8"))
        if is_public:
            declarations = public_type_text(clean, place)
            for match in PRIVATE_TYPE.finditer(declarations):
                at = f"{place}:{clean.count(chr(10), 0, match.start()) + 1}"
                issue("public-private-type", at, match[0])
        for directive in INCLUDE_DIRECTIVE.finditer(clean):
            if not INCLUDE.match(directive[0]):
                at = f"{place}:{clean.count(chr(10), 0, directive.start()) + 1}"
                issue("unsupported-include", at, directive[0].strip())
        for match in INCLUDE.finditer(clean):
            include = match[1].replace("\\", "/")
            at = f"{place}:{clean.count(chr(10), 0, match.start()) + 1}"
            if GPU.search(include) and (is_public or module and modules[module]["headless"]):
                issue("headless-gpu", at, include)
            candidates = [(path.parent / include).resolve(), (repo / "src" / include).resolve(),
                          (repo / "include" / include).resolve(), (repo / include).resolve()]
            explicit_vendor_path = any(candidate.is_relative_to(repo / "third_party") for candidate in candidates)
            directories = search_directories[module] if module else [directory for paths in include_directories.values() for directory in paths]
            candidates.extend((directory / include).resolve() for directory in directories)
            private_headers = sorted({candidate for candidate in candidates
                                      if candidate.is_relative_to(repo / "src") and candidate.is_file()})
            third_party = explicit_vendor_path or any(candidate.is_relative_to(repo / "third_party") and candidate.is_file() for candidate in candidates)
            public_headers = sorted({candidate for candidate in candidates
                                     if candidate.is_relative_to(repo / "include/fsim") and candidate.is_file()})
            if module:
                for header in public_headers:
                    public_consumers.setdefault(header.relative_to(repo).as_posix(), set()).add(module)
            if is_public:
                if private_headers:
                    issue("public-private", at, include)
                if third_party or (not public_headers and Path(include).name in vendor_headers) or GPU.search(include):
                    issue("public-third-party", at, include)
            elif module:
                for private in private_headers:
                    dependency = owner(private)
                    if dependency and dependency not in reachable[module]:
                        issue("undeclared-include", at, f"{module} -> {dependency}: {include}")
                    if dependency and dependency != module:
                        include_edges.setdefault((module, dependency), set()).add(place)

    # domain and kind direction, for direct links and for every include of another module's header
    exceptions = {(entry["from"], entry["to"], entry["rule"]): entry for entry in policy["exceptions"]}
    used = set()
    edges = {(module, dependency, "link") for module in modules for dependency in graph[module]}
    edges |= {(module, dependency, "include") for module, dependency in include_edges}
    for module, dependency, how in sorted(edges):
        broken = edge_rule(modules, domains, module, dependency)
        if broken is None:
            continue
        rule, reason = broken
        if (module, dependency, rule) in exceptions:
            used.add((module, dependency, rule))
            continue
        where = "CMake graph" if how == "link" else sorted(include_edges[(module, dependency)])[0]
        issue("forbidden-" + rule, where, f"{module} ({modules[module]['domain']}) -> {dependency} "
              f"({modules[dependency]['domain']}) by {how}: {reason}")
    for key, entry in sorted(exceptions.items()):
        if key in used:
            continue
        if not any((entry["from"], entry["to"], how) in edges for how in ("link", "include")):
            issue("stale-exception", "policy", f"{entry['from']} -> {entry['to']} ({entry['rule']}, until {entry['until']}): "
                  "the edge no longer exists; remove the exception")
        else:
            issue("stale-exception", "policy", f"{entry['from']} -> {entry['to']} ({entry['rule']}, until {entry['until']}): "
                  "the edge breaks no rule; remove the exception")

    python_issues, python_facts = python_imports.check(repo, policy["python"])
    issues.extend(python_issues)
    facts = {
        "module_count": len(modules), "file_count": len(sources) + len(public),
        "links": {module: sorted(graph[module]) for module in sorted(modules)},
        "external_links": {module: sorted(external_links[module]) for module in sorted(modules)},
        "includes": {f"{m} -> {d}": len(files) for (m, d), files in sorted(include_edges.items())},
        "target_types": dict(sorted(target_types.items())),
        "source_owner": dict(sorted(source_owner.items())),
        "public_headers": {path.relative_to(repo).as_posix(): sorted(public_consumers.get(path.relative_to(repo).as_posix(), ()))
                           for path in sorted(public)},
        "exceptions_applied": sorted(f"{m} -> {d} ({r})" for m, d, r in used),
        "python": python_facts,
    }
    return sorted(set(issues)), facts


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--policy", type=Path, default=Path(__file__).with_name("policy.json"))
    args = parser.parse_args()
    try:
        issues, facts = analyze(args.repo.resolve(), json.loads(args.policy.read_text(encoding="utf-8")))
    except (OSError, ValueError, KeyError, TypeError) as error:
        print(f"architecture input error: {error}", file=sys.stderr)
        return 1
    for issue in issues:
        print(issue)
    if issues:
        print(f"Architecture rejected: {len(issues)} violation(s).")
        return 1
    python = facts["python"]
    print(f"Architecture accepted: {facts['module_count']} native modules, {facts['file_count']} source/public header files; "
          "all CMake branches scanned.")
    print(f"Domains and kinds held on links and includes; legacy exceptions in use: {', '.join(facts['exceptions_applied']) or 'none'}.")
    print(f"Python: {len(python['modules'])} modules (extensions as leaves), {len(python['edges'])} literal imports; "
          f"legacy deferred cycles set aside: {len(python['cycles_all'])}; eager cycles: {len(python['cycles_eager'])}.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
