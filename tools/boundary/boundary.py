"""The SDK's principals (ADR-31, docs/agent-boundary.md): every entry point of the SDK, classified.

    python tools/boundary/boundary.py check  --fsim-dll <libfsim.dll> [--vision-dll <libfsim_vision.dll>]
                                             --objdump <objdump> --cxxfilt <c++filt> --python-path <dir>
    python tools/boundary/boundary.py render (the same arguments): rewrites docs/sdk/boundary.md

The surface is read, not declared: the C and C++ entry points from the DLLs' export tables (objdump, then c++filt),
the Python SDK by importing it. Each is matched against docs/sdk/boundary.toml, whose rules give each a principal:

    agent     an agent (MA's decisions) may call it, on its own platform
    ma_core   the MA core alone, applying A-GRA's rules (and the range, standing in for C2 until AB-5)
    range     the experiment's author: building and changing the world
    referee   read only, truth among it: scoring, rewards, a critic
    platform  inside the simulated equipment: the control runtime, plug-ins and their interfaces
    value     no world: names, struct initialisers, unit and physics helpers, the SDK's version

`check` fails on an entry point no rule classifies, on a rule that classifies nothing, on a name two rules list, on an
entry point in a role-mixing scope (`explicit` in the file) classified by a glob rather than by its name, and on a
docs/sdk/boundary.md that is not what `render` writes.
"""
import argparse
import fnmatch
import importlib
import inspect
import os
import pkgutil
import re
import subprocess
import sys
import tomllib

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
TOML = os.path.join(REPO, "docs", "sdk", "boundary.toml")
DOC = os.path.join(REPO, "docs", "sdk", "boundary.md")
PRINCIPALS = ("agent", "ma_core", "range", "referee", "platform", "value")
# The vision library is optional (FSIM_BUILD_VISION): its names are not stale where it was not built.
VISION = ("fsim::vision::", "fsim_vision_", "fsim_camera_", "fsim.vision.")
SKIP_CPP = ("vtable for ", "typeinfo for ", "typeinfo name for ", "non-virtual thunk to ", "virtual thunk to ",
            "guard variable for ", "construction vtable for ", "VTT for ")


def exports(dll, objdump):
    out = subprocess.run([objdump, "-p", dll], capture_output=True, text=True, check=True).stdout
    names, table = [], False
    for line in out.splitlines():
        if "[Ordinal/Name Pointer] Table" in line:
            table = True
            continue
        if table:
            if not line.strip():
                break
            if re.match(r"\s*\[\s*\d+\]", line):  # a row: [   0] +base[   1] 0000 name (not the column header)
                names.append(line.split()[-1])
    return names


def native_surface(dlls, objdump, cxxfilt):
    """(surface, name) for every exported function: 'c' as exported, 'cpp' demangled without its parameters."""
    found = set()
    for dll in dlls:
        names = exports(dll, objdump)
        cpp = [n for n in names if n.startswith("_Z")]
        for n in names:
            if not n.startswith("_Z"):
                found.add(("c", n))
        demangled = subprocess.run([cxxfilt], input="\n".join(cpp), capture_output=True, text=True,
                                   check=True).stdout.splitlines()
        for d in demangled:
            if d.startswith(SKIP_CPP):
                continue
            found.add(("cpp", d.split("(", 1)[0].replace("[abi:cxx11]", "")))
    return found


def python_surface(path):
    """('python', module.Class.member or module.function) for the Python SDK's public calls. Data types (records,
    enumerations, exceptions) are not calls: the calls that return them are classified."""
    sys.path.insert(0, path)
    import enum
    import fsim
    found = set()
    modules = ["fsim"] + ["fsim." + m.name for m in pkgutil.iter_modules(fsim.__path__) if not m.name.startswith("_")]
    for name in modules:
        try:
            module = importlib.import_module(name)
        except ImportError:
            if name == "fsim.vision":  # (built without the vision library)
                continue
            raise
        for attr, obj in vars(module).items():
            if attr.startswith("_") or inspect.ismodule(obj):
                continue
            home = getattr(obj, "__module__", None) or name
            if not home.startswith("fsim"):
                continue
            if inspect.isclass(obj):
                if issubclass(obj, (enum.Enum, BaseException)) or (issubclass(obj, tuple) and hasattr(obj, "_fields")):
                    continue
                for member in vars(obj):
                    if not member.startswith("_"):
                        found.add(("python", "%s.%s.%s" % (home, obj.__name__, member)))
            elif callable(obj):
                found.add(("python", "%s.%s" % (home, getattr(obj, "__name__", attr))))
    # Calls bound per instance (World.step is the native method itself, for speed) are not on the class: a world and
    # a vehicle are made to read them.
    w = fsim.World("sdk-boundary", publish=False, workers=1, pin_workers=False, terrain=False)
    v = w.create_vehicle("v", "jsbsim:c172x", latitude_deg=0.0, longitude_deg=0.0, altitude_msl_m=1000.0,
                         airspeed_ms=50.0)
    for obj in (w, v):
        cls = type(obj)
        for member, value in getattr(obj, "__dict__", {}).items():  # (slots are on the class, read above)
            if not member.startswith("_") and callable(value):
                found.add(("python", "%s.%s.%s" % (cls.__module__, cls.__name__, member)))
    w.close()
    return found


def load_rules():
    with open(TOML, "rb") as f:
        data = tomllib.load(f)
    return data


def classify(surface, data, absent=()):
    """{(surface, name): rule} and the errors."""
    rules = data["rule"]
    errors = []
    exact = {}
    for i, r in enumerate(rules):
        if r.get("principal") not in PRINCIPALS:
            errors.append("rule %d (%s): principal %r is not one of %s" % (i, r.get("note", ""), r.get("principal"),
                                                                           ", ".join(PRINCIPALS)))
        for leak in r.get("leaks", []):
            if leak not in data["leak"]:
                errors.append("rule %d: leak %s is not in the leak register" % (i, leak))
        for n in r.get("names", []):
            if n in exact:
                errors.append("%s is listed by two rules" % n)
            exact[n] = i
    explicit = tuple(data["explicit"]["scopes"])
    result, used = {}, set()
    for key in sorted(surface):
        _, name = key
        if name in exact:
            result[key] = exact[name]
            used.add((exact[name], name))
            continue
        hit = next((i for i, r in enumerate(rules) for g in r.get("globs", []) if fnmatch.fnmatchcase(name, g)), None)
        if hit is None:
            errors.append("unclassified: %s %s" % key)
            continue
        if name.startswith(explicit):
            errors.append("%s %s is in a role-mixing scope: classify it by name, not by glob" % key)
        result[key] = hit
        used.add((hit, None))
    names = {n for _, n in surface}
    for i, r in enumerate(rules):
        for n in r.get("names", []):
            if n not in names and not n.startswith(absent):
                errors.append("stale: rule %d lists %s, which the SDK no longer has" % (i, n))
        if r.get("globs") and (i, None) not in used:
            errors.append("stale: rule %d's globs %s classify nothing" % (i, r["globs"]))
    return result, errors


C_SOURCES = ("src/sdk/fsim_world_c.cpp", "src/sdk/fsim.cpp")
GUARD = re.compile(r"FSIM_API\s+[^(;{]*?\b(fsim_\w+)\s*\(([^)]*)\)\s*\{\s*(FSIM_(?:NOT_AGENT|AGENT_\w+))\(")
DEFINITION = re.compile(r"FSIM_API\s+[^(;{]*?\b(fsim_\w+)\s*\(([^)]*)\)\s*\{")


def guards(result, data):
    """Every C function an fsim_world* reaches opens by saying whose call it is (FSIM_NOT_AGENT, FSIM_AGENT_*), as its
    principal says: an agent's handle is refused, or scoped, by the library itself (ADR-31, AB-2a)."""
    principal = {name: data["rule"][i]["principal"] for (s, name), i in result.items() if s == "c"}
    errors = []
    for path in C_SOURCES:
        text = open(os.path.join(REPO, path), encoding="utf-8").read()
        opened = {m.group(1): m.group(3) for m in GUARD.finditer(text)}
        for m in DEFINITION.finditer(text):
            name, params = m.group(1), m.group(2)
            if not re.search(r"\bfsim_world\s*\*\s*\w+", params) or name not in principal:
                continue
            guard = opened.get(name)
            if guard is None:
                errors.append("%s: %s takes an fsim_world* but does not open with its guard (FSIM_NOT_AGENT or FSIM_AGENT_*)"
                              % (path, name))
            elif (principal[name] == "agent") != guard.startswith("FSIM_AGENT_"):
                errors.append("%s: %s is %s's, but opens with %s" % (path, name, principal[name], guard))
    return errors


SURFACE = {"cpp": "C++", "c": "C", "python": "Python"}
TITLES = {
    "agent": "Agent - MA's decisions, on its own platform",
    "ma_core": "MA core - A-GRA's rules, applied for its agent",
    "range": "Range - the experiment's author",
    "referee": "Referee - read only, truth among it",
    "platform": "Platform - inside the simulated equipment",
    "value": "Value - no world",
}


def render(result, data):
    rules = data["rule"]
    lines = [
        "# The SDK's principals",
        "",
        "<!-- Written by tools/boundary/boundary.py render from boundary.toml; edit that file, not this one. -->",
        "",
        "Every entry point of the SDK - the C++ classes and functions and the C functions the DLLs export, the Python",
        "SDK's functions and members - belongs to one principal of [ADR-31](../agent-boundary.md). The table below is",
        "read from the DLLs and the Python package themselves (`tools/boundary/boundary.py`), so it cannot leave one out:",
        "ctest's `sdk_boundary` fails on an entry point no rule classifies, and on this page when it is out of date.",
        "",
        "An agent holds its own objects - fsim::Agent, fsim.Agent, an agent's C handle ([agents.md](agents.md)) - never",
        "the range's World or Vehicle, and the library refuses an agent's handle every call that is not an agent's: each",
        "C function an fsim_world* reaches opens with its guard, which `sdk_boundary` checks against this classification.",
        "A **leak** is an agent's call that reaches what ADR-31 keeps from agents; those still open are counted below.",
        "",
        "| Principal | C++ | C | Python |",
        "| --- | ---: | ---: | ---: |",
    ]
    counts = {p: {s: 0 for s in SURFACE} for p in PRINCIPALS}
    for (s, _), i in result.items():
        counts[rules[i]["principal"]][s] += 1
    for p in PRINCIPALS:
        lines.append("| %s | %d | %d | %d |" % (p, counts[p]["cpp"], counts[p]["c"], counts[p]["python"]))
    lines += ["", "## Leaks", "", "| Leak | What | Calls |", "| --- | --- | ---: |"]
    for code, text in data["leak"].items():
        n = sum(1 for (_, _), i in result.items() if code in rules[i].get("leaks", []))
        lines.append("| %s | %s | %d |" % (code, text, n))
    for p in PRINCIPALS:
        lines += ["", "## " + TITLES[p], ""]
        groups = {}
        for (s, name), i in result.items():
            if rules[i]["principal"] == p:
                groups.setdefault(i, []).append((s, name))
        for i in sorted(groups, key=lambda k: (rules[k].get("note", ""), k)):
            r = rules[i]
            leaks = ", ".join(r.get("leaks", []))
            head = "- **%s**" % r.get("note", "")
            if leaks:
                head += " *Leaks: %s.*" % leaks
            lines.append(head)
            for s in ("cpp", "c", "python"):
                names = sorted(n for t, n in groups[i] if t == s)
                if names:
                    lines.append("  - %s: %s" % (SURFACE[s], ", ".join("`%s`" % n for n in names)))
    return "\n".join(lines) + "\n"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("action", choices=("check", "render"))
    ap.add_argument("--fsim-dll", required=True)
    ap.add_argument("--vision-dll")
    ap.add_argument("--objdump", default="objdump")
    ap.add_argument("--cxxfilt", default="c++filt")
    ap.add_argument("--python-path", required=True)
    a = ap.parse_args()
    dlls = [a.fsim_dll] + ([a.vision_dll] if a.vision_dll else [])
    surface = native_surface(dlls, a.objdump, a.cxxfilt) | python_surface(a.python_path)
    data = load_rules()
    absent = () if a.vision_dll else VISION
    result, errors = classify(surface, data, absent)
    errors += guards(result, data)
    text = render(result, data)
    if a.action == "render":
        with open(DOC, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
    else:
        try:
            with open(DOC, encoding="utf-8") as f:
                if f.read() != text:
                    errors.append("docs/sdk/boundary.md is out of date: run tools/boundary/boundary.py render")
        except FileNotFoundError:
            errors.append("docs/sdk/boundary.md is missing: run tools/boundary/boundary.py render")
    for e in errors:
        print(e)
    print("%d entry points: %d C++, %d C, %d Python; %d errors" % (
        len(surface), sum(1 for s, _ in surface if s == "cpp"), sum(1 for s, _ in surface if s == "c"),
        sum(1 for s, _ in surface if s == "python"), len(errors)))
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
