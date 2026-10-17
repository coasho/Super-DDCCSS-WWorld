"""Change-scoped test selection (docs/development.md, "Change-scoped testing").

    python tools/testing/scope.py                         plan for the working tree's changes against HEAD
    python tools/testing/scope.py --base origin/main      ... against another revision
    python tools/testing/scope.py --paths a.cpp b.py      ... for the paths given
    python tools/testing/scope.py --tier integration      the touched domains' whole labels (stress, package included)
    python tools/testing/scope.py --tier full             the whole preset suite
    python tools/testing/scope.py --run [--build] [--log-dir DIR] [--json]

It reads the changed paths, the rules of tools/testing/scopes.toml and each preset's own CTest inventory, and selects:
the tests the matching scopes name (by label or name); every CTest test built from a changed C/C++ test source or
whose command names a changed file; and every python_sdk test whose module imports - directly or through other
modules, shared test fixtures and the scripts it runs included - a changed Python file (the import closure of
python/tests and python/fsim), and the hangar test of a changed hangar test module. A matched `full` scope - or a
changed native source or build input no scope claims - escalates to the full suite of every preset it plans; a matched
`release` scope adds ucrt64-release. A `render_only` scope's paths are built only with the renderer, so they plan
ucrt64-release alone (ucrt64-headless only when another of their scopes - or another changed path - needs it); and any
changed path the presets' compile databases show is compiled only in ucrt64-release (a source, or a file in a directory
whose sources are), whatever claims it, plans ucrt64-release too - a renderer change never passes on the headless build
alone, which does not compile it. Such a path, and any path a `release` scope selecting the `render` label claims (the
shared Vision header, its Python wrapper, assets: headless keeps their coverage), needs renderer coverage: the
ucrt64-release plan selects every `render`-labelled test and fails without one.

It fails closed: a changed path nothing claims, a selection that is empty and no changes at all are errors (exit 3,
with their reasons and paths), not a passing run. With --build it builds the presets first, then lists each preset's
inventory afresh and resolves the selected names to that inventory's numbers immediately before running; after the run
it requires that exactly the selected tests ran (exit 4 otherwise). It prints each preset's selection, an estimated
wall time (a lower bound from measured times: RUN_SERIAL tests one after another, the rest packed over the jobs by their
PROCESSORS) and the exact command; with --run it runs them at the preset's jobs with --no-tests=error, each command's
raw output in its log and its measured times recorded for the next estimate (tools/testing/timing.py). Standard library
only."""
import argparse
import ast
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tomllib

sys.path.insert(0, str(Path(__file__).resolve().parent))
REPO = Path(__file__).resolve().parents[2]
RULES = Path(__file__).resolve().parent / "scopes.toml"
UCRT64 = os.environ.get("FSIM_UCRT64_ROOT", "D:/ENV/DevLanguages/Cpp/msys2/ucrt64")
EXCLUDED_DIRS = {".codex", ".git", "__pycache__"}  # never descended into


def find_ctest():
    """The UCRT64 toolchain's ctest, else the one on PATH."""
    import shutil
    candidate = Path(UCRT64) / "bin" / "ctest.exe"
    return str(candidate) if candidate.is_file() else (shutil.which("ctest") or "ctest")


CTEST = find_ctest()
HEADLESS, RELEASE = "ucrt64-headless", "ucrt64-release"
RENDER = "render"  # every test only the renderer builds: what a renderer change must run on ucrt64-release
STRESS = "stress"  # long contention cases: in the targeted tier only when a matched scope names this label or they depend on the change
NATIVE = (".c", ".cc", ".cpp", ".h", ".hpp", ".inl", ".cmake", ".txt", ".json", ".toml", ".xml", ".py", ".glsl", ".comp", ".vert", ".frag")
ESCALATING = ("src/", "include/", "python/src/", "deps/")


def glob_match(path, pattern):
    """A repository path against a scope pattern: `**/` crosses directories, `*` stays within one."""
    regex = re.escape(pattern).replace(r"\*\*/", "(?:.*/)?").replace(r"\*\*", ".*").replace(r"\*", "[^/]*").replace(r"\?", "[^/]")
    return re.fullmatch(regex, path) is not None


def load_rules(path=RULES):
    with open(path, "rb") as f:
        return tomllib.load(f)["scope"]


def walk(root, keep):
    """The files under `root` that `keep(name)` accepts, in a stable order. Excluded directories (.codex among them)
    are removed before the walk descends into them, never entered and filtered afterwards."""
    for here, dirs, files in os.walk(root):
        dirs[:] = sorted(d for d in dirs if d not in EXCLUDED_DIRS)
        for name in sorted(files):
            if keep(name):
                yield Path(here) / name


def changed_paths(base):
    """The working tree's changed and untracked paths against `base` (never inside a .codex directory)."""
    exclude = [":(exclude,glob)**/.codex/**", ":(exclude).codex"]
    diff = subprocess.run(["git", "diff", "--name-only", base, "--", "."] + exclude, cwd=REPO, capture_output=True, text=True, check=True).stdout
    untracked = subprocess.run(["git", "ls-files", "--others", "--exclude-standard", "--", "."] + exclude, cwd=REPO, capture_output=True,
                               text=True, check=True).stdout
    return sorted({p.strip() for p in (diff + untracked).splitlines() if p.strip()})


def inventory(preset=None, test_dir=None):
    """A preset's (or a build directory's) CTest tests in CTest's own order: [{index, name, labels, command, serial,
    processors}]; None when it has none."""
    where = ["--preset", preset] if preset else ["--test-dir", str(test_dir)]
    run = subprocess.run([CTEST] + where + ["--show-only=json-v1"], cwd=REPO, capture_output=True, text=True)
    if run.returncode != 0:
        return None
    tests = []
    for i, t in enumerate(json.loads(run.stdout)["tests"], 1):
        props = {p["name"]: p["value"] for p in t.get("properties", [])}
        tests.append({"index": i, "name": t["name"], "labels": props.get("LABELS", []), "command": t.get("command", []),
                      "serial": bool(props.get("RUN_SERIAL", False)), "processors": int(props.get("PROCESSORS", 1) or 1)})
    return tests


def test_executables(repo=REPO, builds=None):
    """{repository path of a test or example source: its CMake targets}. Each configured preset's compile_commands.json
    names the target every source under tests/ and examples/ is compiled into (CMakeFiles/<target>.dir/), whatever
    loop or helper declared it; without one, the add_executable calls with literal sources are read. A local header
    (`#include "x.h"`, beside its includer) belongs to every target of the sources including it, transitively."""
    repo = repo.resolve()
    builds = [repo / "build" / HEADLESS, repo / "build" / RELEASE] if builds is None else builds
    out = {}

    def add(path, target):
        try:
            rel = Path(path).resolve().relative_to(repo).as_posix()
        except ValueError:
            return
        if rel.startswith(("tests/", "examples/")):
            out.setdefault(rel, set()).add(target)

    for build in builds:
        commands = Path(build) / "compile_commands.json"
        if commands.is_file():
            for entry in json.loads(commands.read_text(encoding="utf-8", errors="replace")):
                m = re.search(r"[/\\]CMakeFiles[/\\]([^/\\]+)\.dir[/\\]", entry.get("output", "") + " " + entry.get("command", ""))
                if m:
                    add(entry["file"], m.group(1))
    if not out:
        for top in ("tests", "examples"):
            for lists in walk(repo / top, lambda name: name == "CMakeLists.txt"):
                text = re.sub(r"#[^\n]*", "", lists.read_text(encoding="utf-8", errors="replace"))
                for m in re.finditer(r"add_executable\(\s*([A-Za-z0-9_.-]+)([^)]*)\)", text):
                    for source in m.group(2).split():
                        if source.endswith((".c", ".cpp", ".cc")) and "$" not in source:
                            add(lists.parent / source, m.group(1))
    # local headers: the targets of the sources including them, to a fixed point
    includes = {}
    for top in ("tests", "examples"):
        for f in walk(repo / top, lambda name: name.endswith((".c", ".cc", ".cpp", ".h", ".hpp", ".inl"))):
            rel = f.resolve().relative_to(repo).as_posix()
            for name in re.findall(r'^\s*#\s*include\s+"([^"]+)"', f.read_text(encoding="utf-8", errors="replace"), re.M):
                header = (f.parent / name).resolve()
                if header.is_file() and repo in header.parents:
                    includes.setdefault(header.relative_to(repo).as_posix(), set()).add(rel)
    grown = True
    while grown:
        grown = False
        for header, includers in includes.items():
            targets = set().union(*(out.get(i, set()) for i in includers))
            if not targets <= out.get(header, set()):
                out.setdefault(header, set()).update(targets)
                grown = True
    return out


def release_only(repo=REPO, builds=None):
    """The repository paths compiled only where the renderer is: {"sources": first-party sources ucrt64-release's
    compile_commands.json compiles and ucrt64-headless's does not, "directories": directories holding such sources and no
    source ucrt64-headless compiles}; None when either preset's compile database is missing (only the rules then say)."""
    repo = repo.resolve()
    builds = {HEADLESS: repo / "build" / HEADLESS, RELEASE: repo / "build" / RELEASE} if builds is None else builds
    compiled = {}
    for preset, build in builds.items():
        commands = Path(build) / "compile_commands.json"
        if not commands.is_file():
            return None
        files = set()
        for entry in json.loads(commands.read_text(encoding="utf-8", errors="replace")):
            try:
                files.add(Path(entry["file"]).resolve().relative_to(repo).as_posix())
            except (KeyError, ValueError):
                continue
        compiled[preset] = {f for f in files if not f.startswith(("build/", "third_party/"))}
    sources = compiled[RELEASE] - compiled[HEADLESS]
    shared = {f.rsplit("/", 1)[0] for f in compiled[HEADLESS] if "/" in f}
    directories = {f.rsplit("/", 1)[0] for f in sources if "/" in f} - shared
    return {"sources": sources, "directories": directories}


def is_release_only(path, evidence):
    """Whether the compile databases show `path` built only with the renderer (see release_only)."""
    if not evidence:
        return False
    return path in evidence["sources"] or ("/" in path and path.rsplit("/", 1)[0] in evidence["directories"])


def _arg_path(arg):
    """The path an argument names (`-DEXE=<path>` included), as a lower-case POSIX string, else None."""
    value = arg.split("=", 1)[1] if arg.startswith("-") and "=" in arg else arg
    if "/" not in value and "\\" not in value:
        return None
    return Path(value).as_posix().rstrip("/").lower()


def python_dependents(repo=REPO):
    """{repository path of a Python file: the python/tests modules (stems) whose import closure holds it}. Edges: the
    imports of python/fsim and python/tests (absolute, relative and `from package import module`; importing `a.b` runs
    `a` first), and a test file's string naming a sibling script or module (`"isolated_mission_actor.py"`) - what it
    runs in another process. A test module depends on itself."""
    package, tests_dir = repo / "python" / "fsim", repo / "python" / "tests"
    modules = {}  # dotted name -> repository path
    for f in walk(package, lambda name: name.endswith(".py")):
        parts = ("fsim",) + f.relative_to(package).with_suffix("").parts
        modules[".".join(parts[:-1] if parts[-1] == "__init__" else parts)] = f
    siblings = {f.stem: f for f in tests_dir.glob("*.py")} if tests_dir.is_dir() else {}

    def resolve(dotted):
        """The files `import dotted` runs: each package on the way, then the module itself."""
        parts, found = dotted.split("."), []
        if parts[0] in siblings and len(parts) == 1:
            return [siblings[parts[0]]]
        for k in range(1, len(parts) + 1):
            if ".".join(parts[:k]) in modules:
                found.append(modules[".".join(parts[:k])])
        return found

    graph = {}
    for f in list(modules.values()) + list(siblings.values()):
        deps = set()
        try:
            tree = ast.parse(f.read_text(encoding="utf-8", errors="replace"))
        except SyntaxError:
            tree = None  # (a file that does not parse still selects itself and its importers)
        here = None
        if f in modules.values():
            here = next(name for name, path in modules.items() if path == f)
            if f.name != "__init__.py":
                here = here.rsplit(".", 1)[0]  # (the package a relative import starts from)
        for node in ast.walk(tree) if tree else ():
            if isinstance(node, ast.Import):
                for alias in node.names:
                    deps.update(resolve(alias.name))
            elif isinstance(node, ast.ImportFrom):
                base = node.module or ""
                if node.level and here:
                    anchor = here.split(".")
                    anchor = anchor[: len(anchor) - (node.level - 1)]
                    base = ".".join(anchor + ([node.module] if node.module else []))
                deps.update(resolve(base) if base else [])
                for alias in node.names:
                    if base:
                        deps.update(resolve(base + "." + alias.name))
            elif isinstance(node, ast.Constant) and isinstance(node.value, str) and f.parent == tests_dir:
                name = node.value[:-3] if node.value.endswith(".py") else node.value
                if name in siblings and name != f.stem:
                    deps.add(siblings[name])
        deps.discard(f)
        graph[f] = deps
    dependents = {}
    for f in siblings.values():
        if not f.name.startswith("test_"):
            continue
        seen, stack = {f}, [f]
        while stack:
            for dep in graph.get(stack.pop(), ()):
                if dep not in seen:
                    seen.add(dep)
                    stack.append(dep)
        for dep in seen:
            dependents.setdefault(dep.resolve().relative_to(repo.resolve()).as_posix(), set()).add(f.stem)
    return dependents


def costs(preset):
    """{test name: seconds}: the times tools/testing/timing.py recorded, else CTest's cost data."""
    import timing
    return timing.seconds(preset)


DATA = (".json", ".txt", ".csv", ".xml", ".toml", ".bin", ".dat", ".yaml", ".yml")


def data_readers(paths, candidates, read=None):
    """{changed file: the candidate sources that name it}. A test source or script reads a file under tests/, tools/ or
    examples/ when its text names it: by its repository path (`tests/equipment_package_consumer/equipment_python.py`,
    copied by a gate's CMake script) and - a data file - by its own name (`"aew_bomber_detections.txt"`) or its
    directory's repository path below a top-level one (`tests/fixtures/mission_training/engagement`, which a checker
    reads whole). `candidates` are repository paths of test sources, test modules and the scripts tests run."""
    read = read or (lambda p: (REPO / p).read_text(encoding="utf-8", errors="replace"))
    wanted = {}
    for p in paths:
        if not p.startswith(("tests/", "python/tests/", "tools/", "examples/")):
            continue
        keys = {p}
        if p.endswith(DATA):
            keys.add(Path(p).name)
            if p.count("/") >= 3:
                keys.add(p.rsplit("/", 1)[0])
        wanted[p] = keys
    out = {}
    if not wanted:
        return out
    for source in candidates:
        try:
            text = read(source).replace("\\", "/")
        except OSError:
            continue
        for path, keys in wanted.items():
            if source != path and any(key in text for key in keys):
                out.setdefault(path, set()).add(source)
    return out


def direct_tests(path, tests, executables, dependents):
    """The tests built from `path`, importing it (python_sdk; a hangar test module) or whose command names it or a
    repository directory holding it (a fixture directory - never the repository or a top-level directory, which would
    claim everything); and whether a compiled target holds it (a native build is needed)."""
    found = set()
    compiled = path in executables
    if compiled:
        targets = executables[path]
        targets = {targets} if isinstance(targets, str) else set(targets)
        found.update(t["name"] for t in tests if t["command"] and (
            Path(t["command"][0]).stem in targets
            or any(a.endswith(".exe") and Path(a).stem in targets for a in map(_arg_path, t["command"][1:]) if a)))
    modules = set(dependents.get(path, ()))
    if path.startswith("python/tests/test_") and path.endswith(".py"):
        modules.add(Path(path).stem)
    for module in modules:
        found.update(t["name"] for t in tests if t["name"] == "python_sdk " + module or t["name"].startswith("python_sdk %s." % module))
    if path.startswith("tools/hangar/tests/test_") and path.endswith(".py"):
        found.update(t["name"] for t in tests if t["name"].startswith("hangar %s." % Path(path).stem))
    absolute = (REPO / path).resolve().as_posix().lower()
    root = REPO.resolve().as_posix().lower()

    def names(arg):
        a = _arg_path(arg)
        return a is not None and (a == absolute or (absolute.startswith(a + "/") and a.startswith(root + "/") and "/" in a[len(root) + 1:]))
    found.update(t["name"] for t in tests if any(names(a) for a in t["command"][1:]))
    return found, compiled


def renders(rule):
    """Whether a scope requires renderer coverage: it runs on ucrt64-release and selects the `render` label. This is
    separate from `render_only`, which decides only whether ucrt64-headless is planned too."""
    return bool(rule.get("release")) and RENDER in rule.get("labels", [])


def plan(paths, rules, tests, executables, tier="targeted", dependents=None, readers=None, evidence=None, preset=None):
    """What `paths` need: {"scopes", "escalate" (reasons), "release" (reasons), "headless" (whether ucrt64-headless
    builds any of them), "renderer" (the paths that need renderer tests), "build", "selected" (test names), "unmatched",
    "errors" ([{"reason", "paths", ...}]: an unclaimed path, a required Python test module with no CTest test registered,
    no renderer test where a renderer change needs one, an empty selection or no changes)}. `evidence` (release_only)
    marks the paths compiled only with the renderer. Planned on ucrt64-release (`preset`), a path that needs renderer
    tests (that evidence, or a scope that `renders`, render-only or shared, claiming it) selects every `render`-labelled
    test of that inventory, whatever the tier, and an inventory without one is an error: generic matches (sdk_boundary,
    Python Vision) supplement that coverage, never stand in for it."""
    dependents, readers = dependents or {}, readers or {}
    selected, matched, escalate, release, unmatched, direct_all = set(), [], [], [], [], set()
    required = {}  # python/tests module -> the changed paths that need its tests (it exists: the import closure holds it)
    build = False
    headless = not paths
    renderer = []  # the changed paths needing renderer tests: a render scope claims them, or the compile databases say
    for path in paths:
        hits = [r for r in rules if any(glob_match(path, p) for p in r.get("paths", [])) and not any(glob_match(path, p) for p in r.get("except", []))]
        rendered = is_release_only(path, evidence)
        if rendered or any(renders(r) for r in hits):
            renderer.append(path)
        # ucrt64-headless builds the path unless every scope claiming it is render-only, or (claimed by none) the compile
        # databases show it built only with the renderer
        if any(not r.get("render_only") for r in hits) or (not hits and not rendered):
            headless = True
        if rendered and not any(r.get("release") for r in hits):
            release.append("%s (compiled only in %s)" % (path, RELEASE))
        for r in hits:
            if r["name"] not in matched:
                matched.append(r["name"])
            if r.get("full"):
                escalate.append("%s (%s)" % (path, r["name"]))
            if r.get("release"):
                release.append("%s (%s)" % (path, r["name"]))
            if r.get("build", True):
                build = True
            excluded = set() if tier != "targeted" else set(r.get("exclude_labels", []))
            for t in tests:
                labels = set(t["labels"])
                if labels & set(r.get("labels", [])) and not labels & excluded:
                    selected.add(t["name"])
            stem = Path(path).stem
            for expression in r.get("tests", []) + (r.get("integration", []) if tier != "targeted" else []):
                pattern = re.compile(expression.replace("{stem}", re.escape(stem)))
                selected.update(t["name"] for t in tests if pattern.search(t["name"]))
        if path.startswith("python/tests/test_"):
            build = True  # (the build checks the module glob again: an added or removed module changes the inventory)
        # beyond the rules: tests built from, run on or importing the changed file - or a test source reading it
        direct, claimed = set(), False
        for source in [path] + sorted(readers.get(path, ())):
            found, compiled = direct_tests(source, tests, executables, dependents)
            direct |= found
            claimed = claimed or compiled or source in dependents or bool(found)
            build = build or compiled
            for module in dependents.get(source, ()):
                required.setdefault(module, set()).add(path)
        selected |= direct
        direct_all |= direct
        if not any(r.get("claims", True) for r in hits) and not claimed:
            unmatched.append(path)
            if path.startswith(ESCALATING) or path.endswith(NATIVE[:7]) or Path(path).name == "CMakeLists.txt":
                escalate.append("%s (no scope claims it)" % path)
    if tier == "targeted" and not any(STRESS in r.get("labels", []) for r in rules if r["name"] in matched):
        # (contention cases run when concurrency code changed - a scope naming the label - or what they import or are built from did)
        stress = {t["name"] for t in tests if STRESS in t["labels"]}
        selected -= stress - direct_all
    if tier == "integration":  # the touched domains whole: every label any matched scope names, stress and package too
        labels = {lab for r in rules if r["name"] in matched for lab in r.get("labels", []) + r.get("integration_labels", [])}
        selected.update(t["name"] for t in tests if set(t["labels"]) & labels)
    if tier == "full" or escalate:
        selected = {t["name"] for t in tests}
    errors = []
    if preset == RELEASE and renderer:
        rendered_tests = {t["name"] for t in tests if RENDER in t["labels"]}
        selected |= rendered_tests
        if not rendered_tests:
            errors.append({"reason": "no-renderer-coverage", "paths": renderer,
                           "detail": "these paths need renderer tests (a render scope or the compile databases say so), and this %s "
                                     "inventory has no '%s' test: configure and build it with the renderer (FSIM_WITH_RENDER), "
                                     "then select again"
                                     % (RELEASE, RENDER)})
    escalated = {reason.split(" (", 1)[0] for reason in escalate}
    unclaimed = [path for path in unmatched if path not in escalated]
    if not paths and tier != "full":
        errors.append({"reason": "no-changes", "paths": [], "detail": "no changed paths: give --paths, or --tier full"})
    if unclaimed and tier != "full":
        errors.append({"reason": "unclaimed", "paths": unclaimed,
                       "detail": "no scope in tools/testing/scopes.toml, test source, test command or Python import claims these paths"})
    # A module the changes need must be registered: a test module added since the last configure has no CTest test until
    # a build reruns the glob, and the rest of the selection (sdk_boundary, say) passing would not have run it.
    names = {t["name"] for t in tests}
    unregistered = sorted(m for m in required if not any(n == "python_sdk " + m or n.startswith("python_sdk %s." % m) for n in names))
    if unregistered:
        errors.append({"reason": "unregistered", "modules": unregistered,
                       "paths": sorted(set().union(*(required[m] for m in unregistered))),
                       "detail": "no CTest test is registered for Python test module(s) %s: build the preset (--build) so CMake "
                                 "registers them, then select again" % ", ".join(unregistered)})
    if paths and not selected:
        errors.append({"reason": "empty-selection", "paths": list(paths), "scopes": list(matched),
                       "detail": "the changes select no test in this inventory"})
    return {"scopes": matched, "escalate": escalate, "release": release, "headless": headless, "renderer": renderer,
            "build": build or bool(escalate) or bool(release), "selected": sorted(selected), "unmatched": unmatched, "errors": errors}


def estimate(chosen, cost, jobs):
    """A lower bound on a selection's wall time from measured times, and how many tests have none: RUN_SERIAL tests run
    alone, one after another; the rest share the jobs by their PROCESSORS, no faster than the longest of them. It does
    not model the load one test puts on another or CTest's ordering."""
    known = [t for t in chosen if t["name"] in cost]
    unknown = len(chosen) - len(known)
    if not known:
        return None, unknown
    serial = sum(cost[t["name"]] for t in known if t.get("serial"))
    parallel = [t for t in known if not t.get("serial")]
    packed = max([cost[t["name"]] for t in parallel] + [sum(cost[t["name"]] * min(t.get("processors", 1), jobs) for t in parallel) / jobs])
    return serial + packed, unknown


def build_preset(preset):
    return subprocess.run([str(REPO / "fsim.cmd"), "--preset", preset, "build", "--parallel", "12"], cwd=REPO).returncode


def execute(command, log_path):
    """CTest's raw output straight to its log (read it there); its exit code."""
    with open(log_path, "wb") as log:
        return subprocess.run(command, cwd=REPO, stdout=log, stderr=subprocess.STDOUT).returncode


def candidate_readers(executables, dependents, repo=REPO):
    """Repository paths that may read a data file for a test: test sources, Python test modules and fixtures, the
    repository's tool scripts (tests run them) and the CMake scripts under tests/ and cmake/ (gates run them)."""
    out = set(executables) | set(dependents)
    out.update(f.resolve().relative_to(repo.resolve()).as_posix() for f in walk(repo / "tools", lambda name: name.endswith(".py")))
    for top in ("tests", "cmake"):
        out.update(f.resolve().relative_to(repo.resolve()).as_posix() for f in walk(repo / top, lambda name: name.endswith(".cmake")))
    return sorted(out)


def select(paths, rules, tier, jobs, run=False, build=False, log_dir=None, executables=None, dependents=None, readers=None,
           discover=None, builder=None, runner=None, measured=None, recorder=None, evidence=False):
    """The whole lifecycle, its effects injectable: build first (when asked and needed), then list each preset's
    inventory afresh, plan on it, resolve names to that inventory's numbers, run, and hold the run to the selection.
    The presets are those the paths need: ucrt64-headless unless every path is built only with the renderer,
    ucrt64-release when any needs it; a needed preset without an inventory, or whose build fails, is an error.
    Returns (exit status, report)."""
    import timing
    discover, builder, runner = discover or inventory, builder or build_preset, runner or execute
    measured, recorder = measured or costs, recorder or timing.record
    executables = test_executables() if executables is None else executables
    dependents = python_dependents() if dependents is None else dependents
    readers = data_readers(paths, candidate_readers(executables, dependents)) if readers is None else readers
    evidence = release_only() if evidence is False else evidence  # (None: no compile databases, the rules alone)
    report = {"paths": list(paths), "tier": tier, "presets": {}, "errors": [], "builds": {}}
    needs = plan(paths, rules, [], executables, tier, dependents, readers, evidence)  # (what the paths alone need: presets, a build)
    presets = ([HEADLESS] if needs["headless"] or not needs["release"] else []) + ([RELEASE] if needs["release"] else [])
    if run and build and needs["build"]:
        for preset in presets:
            rc = builder(preset)
            report["builds"][preset] = rc
            if rc != 0:
                report["errors"].append({"reason": "build-failed", "preset": preset, "paths": [], "detail": "exit %d" % rc})
                return rc, report
    for preset in presets:
        tests = discover(preset)  # (after any build: a reconfigure may have added, removed or reordered tests)
        if tests is None:
            report["errors"].append({"reason": "no-inventory", "preset": preset, "paths": [],
                                     "detail": "configure and build %s" % preset})
            continue
        p = plan(paths, rules, tests, executables, tier, dependents, readers, evidence, preset)  # (after any build: its inventory)
        names = set(p["selected"])
        chosen = [t for t in tests if t["name"] in names]
        wall, unknown = estimate(chosen, measured(preset), jobs)
        command = [CTEST, "--preset", preset, "--no-tests=error", "--output-on-failure", "-j", str(jobs)]
        if len(chosen) < len(tests):
            command += ["-I", "0,0,0," + ",".join(str(t["index"]) for t in chosen)]
        report["presets"][preset] = {"plan": p, "selected": [t["name"] for t in chosen], "of": len(tests), "estimate_s": wall,
                                     "without_times": unknown, "command": command}
        for error in p["errors"]:
            report["errors"].append(dict(error, preset=preset))
    if report["errors"]:
        return 3, report
    if not run:
        return 0, report
    status = 0
    for preset, entry in report["presets"].items():
        directory = Path(log_dir) if log_dir else REPO / "build" / preset / "scope-logs"
        directory.mkdir(parents=True, exist_ok=True)
        log_path = directory / ("scope-%s.log" % preset)
        rc = runner(entry["command"], log_path)
        ran = set(timing.parse(timing.read_log(log_path))) if log_path.is_file() else set()
        if log_path.is_file():
            recorder([log_path], preset)  # (what this run measured: the next plan's estimate)
        entry.update(log=str(log_path), exit=rc)
        missing, extra = sorted(set(entry["selected"]) - ran), sorted(ran - set(entry["selected"]))
        if missing or extra:
            report["errors"].append({"reason": "ran-other-tests", "preset": preset, "paths": [], "missing": missing, "extra": extra,
                                     "detail": "the run did not execute exactly the selected tests"})
            rc = rc or 4
        status = status or rc
    return status, report


def show(report):
    """The report as text."""
    out = []
    first = next(iter(report["presets"].values()), None)
    if first:
        p = first["plan"]
        out.append("%d changed path(s); scopes: %s" % (len(report["paths"]), ", ".join(p["scopes"]) or "none"))
        out += ["  escalates to the full suite: %s" % r for r in p["escalate"]]
        out += ["  needs %s too: %s" % (RELEASE, r) for r in p["release"]]
        out.append("native build needed: %s" % ("yes" if p["build"] else "no"))
    for preset, entry in report["presets"].items():
        wall = entry["estimate_s"]
        out.append("%s: %d of %d tests%s, estimated %s%s" % (
            preset, len(entry["selected"]), entry["of"], " (the whole suite)" if len(entry["selected"]) == entry["of"] else "",
            "at least %.0f s wall (measured times; RUN_SERIAL tests added, the rest packed over the jobs)" % wall if wall is not None else "unknown",
            " (%d without measured times)" % entry["without_times"] if entry["without_times"] else ""))
        out += ["    %s" % n for n in sorted(entry["selected"])[:60]]
        if len(entry["selected"]) > 60:
            out.append("    ... %d more" % (len(entry["selected"]) - 60))
        out.append("command: %s" % subprocess.list2cmdline(entry["command"]))
        if "exit" in entry:
            out.append("%s: log %s" % (preset, entry["log"]))
            out.append("%s: exit %d" % (preset, entry["exit"]))
    for error in report["errors"]:
        out.append("ERROR %s%s: %s%s" % (error["reason"], " (%s)" % error["preset"] if error.get("preset") else "", error["detail"],
                                         "".join("\n    %s" % path for path in error.get("paths", []))))
    return "\n".join(out)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--base", default="HEAD", help="the revision the working tree is compared with")
    ap.add_argument("--paths", nargs="+", help="these paths instead of the working tree's changes")
    ap.add_argument("--tier", choices=("targeted", "integration", "full"), default="targeted")
    ap.add_argument("--jobs", type=int, default=16, help="CTest parallelism (the presets' own: 16)")
    ap.add_argument("--run", action="store_true", help="run the selection (each preset's CTest, in parallel)")
    ap.add_argument("--build", action="store_true", help="with --run: build each needed preset first")
    ap.add_argument("--log-dir", default=None, help="with --run: where each command's raw output goes (build/<preset>/scope-logs)")
    ap.add_argument("--rules", default=str(RULES))
    ap.add_argument("--json", action="store_true", help="print the report as JSON")
    a = ap.parse_args(argv)
    paths = sorted(set(a.paths)) if a.paths else changed_paths(a.base)
    status, report = select(paths, load_rules(Path(a.rules)), a.tier, a.jobs, a.run, a.build, a.log_dir)
    print(json.dumps(report, indent=1, default=str) if a.json else show(report))
    return status


if __name__ == "__main__":
    sys.exit(main())
