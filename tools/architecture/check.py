"""Check all native source modules against their declared CMake dependency graph.

The deliberately limited source parser reads every conditional branch; it does
not configure CMake or evaluate variables. Unsupported source/link declarations
fail closed. See README.md for the accepted CMake subset and ownership policy.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import sys


SOURCE_SUFFIXES = {".h", ".hpp", ".hh", ".inl", ".c", ".cpp", ".cc", ".cxx"}
GPU = re.compile(r"(?:^|::|/)(?:vsg\w*|vulkan)(?:\b|/)|^NvFlow", re.IGNORECASE)
INCLUDE = re.compile(r'^[\t ]*#[\t ]*include[\t ]*[<"]([^>"\n]+)[>"]', re.MULTILINE)
INCLUDE_DIRECTIVE = re.compile(r'^[\t ]*#[\t ]*include\b[^\n]*', re.MULTILINE)
# CMake tokens preserve quoted arguments and bracket arguments as one token.
CMAKE_TOKEN = re.compile(r'"(?:\\.|[^"\\])*"|\[(=*)\[.*?\]\1\]|[^\s()]+|[()]', re.DOTALL)


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


def check(repo, policy):
    modules = policy["modules"]
    targets, aliases = {}, {}
    for name, module in modules.items():
        if module["target"] in targets:
            raise ValueError(f"duplicate registered target {module['target']}")
        targets[module["target"]] = name
        for alias in module["aliases"]:
            if alias in aliases or alias in targets:
                raise ValueError(f"duplicate registered alias {alias}")
            aliases[alias] = name
        for dependency in module["allowed_dependencies"]:
            if dependency not in modules:
                raise ValueError(f"unknown policy dependency {name} -> {dependency}")
    names = targets | aliases
    external = set(policy["external_targets"])
    issues, declared, declared_aliases = [], set(), set()
    graph = {name: set() for name in modules}
    source_declarations, link_declarations, include_declarations = [], [], []
    include_directories = {name: [] for name in modules}

    def issue(code, place, detail):
        issues.append(f"{code}: {place}: {detail}")

    def owner(path):
        relative = path.relative_to(repo).as_posix()
        exact = [name for name, module in modules.items() if relative in module["files"]]
        if exact:
            candidates = exact
        else:
            candidates = [name for name, module in modules.items()
                          if any(relative.startswith(directory.rstrip("/") + "/") for directory in module["directories"])]
        if len(candidates) != 1:
            issue("unknown-owner", relative, "source needs exactly one registered module")
            return None
        return candidates[0]

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

    for cmake in sorted((repo / "src").rglob("CMakeLists.txt")):
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
        actual = owner(path)
        if actual and actual != targets[target]:
            issue("source-owner", place, f"{actual} owns {path.relative_to(repo).as_posix()}, declared by {targets[target]}")

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
            elif link not in external:
                issue("unknown-link", place, f"{module} links unregistered {link}")

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
    vendor_headers = {path.name for path in (repo / "third_party").rglob("*")
                      if path.is_file() and path.suffix in SOURCE_SUFFIXES}
    sources = [path for path in (repo / "src").rglob("*") if path.is_file() and path.suffix in SOURCE_SUFFIXES]
    public = [path for path in (repo / "include/fsim").rglob("*") if path.is_file() and path.suffix in SOURCE_SUFFIXES]
    for path in sorted(sources + public):
        place = path.relative_to(repo).as_posix()
        is_public = path.is_relative_to(repo / "include/fsim")
        module = None if is_public else owner(path)
        clean = without_comments(path.read_text(encoding="utf-8"))
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
            if is_public:
                if private_headers:
                    issue("public-private", at, include)
                public_include = any(candidate.is_relative_to(repo / "include/fsim") and candidate.is_file() for candidate in candidates)
                if third_party or (not public_include and Path(include).name in vendor_headers) or GPU.search(include):
                    issue("public-third-party", at, include)
            elif module:
                for private in private_headers:
                    dependency = owner(private)
                    if dependency and dependency not in reachable[module]:
                        issue("undeclared-include", at, f"{module} -> {dependency}: {include}")
    return sorted(set(issues)), len(modules), len(sources) + len(public)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--policy", type=Path, default=Path(__file__).with_name("policy.json"))
    args = parser.parse_args()
    try:
        issues, modules, sources = check(args.repo.resolve(), json.loads(args.policy.read_text(encoding="utf-8")))
    except (OSError, ValueError, KeyError, TypeError) as error:
        print(f"architecture input error: {error}", file=sys.stderr)
        return 1
    for issue in issues:
        print(issue)
    if issues:
        print(f"Architecture rejected: {len(issues)} violation(s).")
        return 1
    print(f"Architecture accepted: {modules} native modules, {sources} source/public header files; all CMake branches scanned.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
