"""The MA-training compatibility baseline (docs/sdk/ma-compatibility-baseline.md): what the SDK was before the mission
episode family is added, frozen so later packages can be held to it.

    python tools/ma_contract/baseline.py capture --out <dir> [--supplied <dir>] [--release-build <dir>]
    python tools/ma_contract/baseline.py verify  --out <dir>
    python tools/ma_contract/baseline.py run     --out <dir> [--sdk-prefix <dir>] [--python-package <dir>]

`capture` reads the checkout and its existing builds and writes, under <dir>: the checkout's HEAD and dirty diff, the
public interface's archive, the protected runtime inputs' hashes, every native and Python export, the CTest inventory,
the public types' layouts and vtables, and consumers frozen against today's headers - C++, C99 and Python - with what
each imports and the data it ran on. `verify` re-hashes what the manifest lists. `run` runs the frozen consumers again,
by default as captured. With `--sdk-prefix`, another build's installed SDK, the frozen executables and the frozen
Python module run against that prefix's libfsim.dll and data instead: old-header binaries on a new library.
`--python-package` runs the frozen Python consumer scripts on another package altogether.

It builds nothing in the checkout's build trees and changes no tracked file: it installs from an existing build into
<dir> and compiles the consumers there. Standard library only. Hold the native configure/build/test slot while it runs.
"""
import argparse
import datetime
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

REPO = Path(__file__).resolve().parents[2]
SCHEMA = "fsim.ma-baseline.v1"
DEFAULT_UCRT64 = "D:/ENV/DevLanguages/Cpp/msys2/ucrt64"
DEFAULT_TMP = "D:/ENV/tmp"
# Examples that reach the P2P and C2 paths through the public Python package alone: the frozen Python consumers.
PYTHON_CONSUMERS = ("agent.py", "links.py", "c2.py")
MANIFEST = "MANIFEST.json"

# Run inside the consumer's interpreter before its script: where the package and the platform's DLL really came from.
PY_PROVENANCE = r'''
import ctypes, json, os, runpy, sys
import fsim
k = ctypes.WinDLL("kernel32", use_last_error=True)
k.GetModuleHandleW.restype = ctypes.c_void_p
k.GetModuleHandleW.argtypes = [ctypes.c_wchar_p]
k.GetModuleFileNameW.argtypes = [ctypes.c_void_p, ctypes.c_wchar_p, ctypes.c_uint32]
def loaded(name):
    h = k.GetModuleHandleW(name)
    if not h:
        return None
    b = ctypes.create_unicode_buffer(32768)
    k.GetModuleFileNameW(h, b, 32768)
    return b.value.replace("\\", "/")
def value(name):
    v = getattr(fsim, name, None)
    return v() if callable(v) else v
print("provenance " + json.dumps({"package": os.path.dirname(fsim.__file__).replace("\\", "/"),
                                  "native": fsim._native.__file__.replace("\\", "/"),
                                  "libfsim": loaded("libfsim.dll"), "libJSBSim": loaded("libJSBSim.dll"),
                                  "abi": value("abi_version"), "version": value("__version__")},
                                 sort_keys=True), flush=True)
sys.argv = sys.argv[1:]
runpy.run_path(sys.argv[0], run_name="__main__")
'''

# The Python package as a consumer sees it: every public name, a call's signature, an enumeration's numbers, a
# record's fields. boundary.py lists the calls a principal owns; this is their shape.
PY_INVENTORY = r'''
import enum, importlib, inspect, json, pkgutil, sys
sys.path.insert(0, sys.argv[1])
import fsim
def sig(obj):
    try:
        return str(inspect.signature(obj))
    except (TypeError, ValueError):
        return None
def describe(obj):
    if inspect.isclass(obj):
        if issubclass(obj, enum.Enum):
            return {"kind": "enum", "members": {m.name: (m.value if isinstance(m.value, (int, str)) else repr(m.value)) for m in obj}}
        if issubclass(obj, tuple) and hasattr(obj, "_fields"):
            return {"kind": "record", "fields": list(obj._fields)}
        if issubclass(obj, BaseException):
            return {"kind": "exception", "bases": [b.__name__ for b in obj.__mro__[1:-1]]}
        members = {}
        for name, value in sorted(vars(obj).items()):
            if name.startswith("_") and name not in ("__init__",):
                continue
            if isinstance(value, property):
                members[name] = "property"
            elif isinstance(value, (staticmethod, classmethod)):
                members[name] = type(value).__name__ + (sig(value.__func__) or "")
            elif callable(value):
                members[name] = "method" + (sig(value) or "")
            else:
                members[name] = type(value).__name__
        return {"kind": "class", "members": members}
    if callable(obj):
        return {"kind": "function", "signature": sig(obj)}
    if isinstance(obj, (bool, int, float, str)):
        return {"kind": "constant", "type": type(obj).__name__, "value": obj if obj == obj else "nan"}
    return {"kind": "value", "type": type(obj).__name__}
out = {}
names = ["fsim"] + sorted("fsim." + m.name for m in pkgutil.iter_modules(fsim.__path__))
for name in names:
    try:
        module = importlib.import_module(name)
    except ImportError as e:
        out[name] = {"unavailable": str(e)}
        continue
    entries = {}
    private = name.rsplit(".", 1)[-1].startswith("_")
    for attr, obj in sorted(vars(module).items()):
        if attr.startswith("__") or inspect.ismodule(obj):
            continue
        if attr.startswith("_") and not private:
            continue
        home = getattr(obj, "__module__", None)
        if home is not None and not str(home).startswith("fsim") and not isinstance(obj, (bool, int, float, str)):
            continue
        entry = describe(obj)
        entry["home"] = home
        entries[attr] = entry
    out[name] = entries
json.dump(out, sys.stdout, indent=1, sort_keys=True)
'''

PY_SURFACE = r'''
import importlib.util, sys
spec = importlib.util.spec_from_file_location("boundary", sys.argv[1])
boundary = importlib.util.module_from_spec(spec)
spec.loader.exec_module(boundary)
for _, name in sorted(boundary.python_surface(sys.argv[2])):
    print(name)
'''


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


class Capture:
    def __init__(self, a):
        self.out = Path(a.out).resolve()
        self.ucrt = Path(a.ucrt64)
        self.tmp = a.tmp
        self.headless = Path(a.headless_build).resolve()
        self.release = Path(a.release_build).resolve() if a.release_build else None
        self.supplied = Path(a.supplied).resolve() if a.supplied else None
        self.python = a.python
        self.summary = {}
        self.failures = []
        self.logs = self.out / "logs"

    # ---- plumbing
    def tool(self, name):
        return str(self.ucrt / "bin" / (name + ".exe"))

    def env(self, path=None, extra=None):
        system = os.environ.get("SystemRoot", "C:/Windows")
        e = {k: v for k, v in os.environ.items() if k.upper() not in ("PATH", "PYTHONPATH", "FSIM_DLL_DIR", "FSIM_AIRCRAFT_PATH")}
        e["PATH"] = os.pathsep.join((path or [str(self.ucrt / "bin")]) + [system + "/System32", system])
        e["TMP"] = e["TEMP"] = e["TORCH_EXTENSIONS_DIR"] = self.tmp
        e.update(extra or {})
        return e

    def run(self, label, command, cwd=None, env=None, timeout=900, check=True):
        """A command's raw output straight to its log; its exit code."""
        self.logs.mkdir(parents=True, exist_ok=True)
        log = self.logs / (label + ".log")
        with open(log, "wb") as f:
            f.write(("> " + " ".join(str(c) for c in command) + "\n").encode())
            f.flush()
            try:
                rc = subprocess.run([str(c) for c in command], cwd=cwd, env=env or self.env(), stdout=f,
                                    stderr=subprocess.STDOUT, timeout=timeout).returncode
            except subprocess.TimeoutExpired:
                rc = "timeout"
            f.write(("exit=%s\n" % rc).encode())
        if check and rc != 0:
            self.failures.append("%s: exit %s (logs/%s.log)" % (label, rc, label))
        return rc

    def text(self, command, cwd=None, env=None, timeout=300):
        if str(command[0]) == "git":
            env = os.environ  # (git is the caller's own, wherever its PATH has it)
        return subprocess.run([str(c) for c in command], cwd=cwd or REPO, env=env or self.env(), capture_output=True,
                              text=True, timeout=timeout, check=True, encoding="utf-8", errors="replace").stdout

    def write(self, relative, text):
        path = self.out / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
        return path

    # ---- the checkout
    def git(self):
        head = self.text(["git", "rev-parse", "HEAD"]).strip()
        status = self.text(["git", "status", "--porcelain=v1"])
        self.write("git/head.txt", head + "\n")
        self.write("git/branch.txt", self.text(["git", "rev-parse", "--abbrev-ref", "HEAD"]))
        self.write("git/tree.txt", self.text(["git", "rev-parse", "HEAD^{tree}"]))
        self.write("git/status.txt", status)
        self.write("git/dirty.diff", self.text(["git", "diff", "HEAD", "--binary"]))
        self.write("git/tracked-blobs.txt", self.text(["git", "ls-files", "-s"]))
        archive = self.out / "git" / ("public-interface-%s.zip" % head[:7])
        subprocess.run(["git", "archive", "--format=zip", "-o", str(archive), "HEAD", "include", "python/fsim"], cwd=REPO,
                       check=True)
        # Tracked files a build reads: one of them changed, and the builds captured here are not HEAD's. (A file not
        # yet tracked reaches a build only through a tracked CMake file, which this would show.)
        interface_dirty = self.text(["git", "status", "--porcelain=v1", "--untracked-files=no", "--", "include", "python", "src",
                                     "tests", "cmake", "CMakeLists.txt", "CMakePresets.json", "aircraft", "assets", "third_party",
                                     "examples", "tools", "deps"]).strip()
        self.summary["git"] = {"head": head, "branch": self.text(["git", "rev-parse", "--abbrev-ref", "HEAD"]).strip(),
                               "dirty": [line[3:] for line in status.splitlines()],
                               "source_and_interface_dirty": interface_dirty.splitlines(),
                               "public_interface_archive": archive.name}
        if interface_dirty:
            self.failures.append("git: source, interface or build inputs differ from HEAD: the builds are not HEAD's")
        return head

    # ---- protected runtime inputs
    def inputs(self):
        if not self.supplied:
            self.summary["inputs"] = {"skipped": "no --supplied directory: no protected input list"}
            return
        listed = (self.supplied / "protected-runtime-inputs.sha256").read_text(encoding="utf-8").splitlines()
        lines, differ, missing = [], [], []
        for line in listed:
            digest, _, relative = line.partition("  ")
            path = REPO / relative
            if not path.is_file():
                missing.append(relative)
                continue
            now = sha256(path)
            lines.append("%s  %s" % (now, relative))
            if now != digest:
                differ.append(relative)
        self.write("inputs/protected-runtime-inputs.sha256", "\n".join(lines) + "\n")
        shutil.copyfile(self.supplied / "protected-runtime-inputs.sha256", self.out / "inputs" / "supplied-protected-runtime-inputs.sha256")
        self.summary["inputs"] = {"listed": len(listed), "hashed": len(lines), "differ_from_supplied": differ, "missing": missing}
        if differ or missing:
            self.failures.append("inputs: %d protected inputs differ from the supplied manifest, %d are missing" % (len(differ), len(missing)))

    # ---- exports
    def exported(self, binary):
        dump = self.text([self.tool("objdump"), "-p", binary])
        names, imports, table, dll = [], {}, False, None
        for line in dump.splitlines():
            if "[Ordinal/Name Pointer] Table" in line:
                table = True
                continue
            if table:
                if not line.strip():
                    table = False
                elif re.match(r"\s*\[\s*\d+\]", line):
                    names.append(line.split()[-1])
                continue
            m = re.match(r"\s*DLL Name: (\S+)", line)
            if m:
                dll = m.group(1)
                imports.setdefault(dll, [])
                continue
            if not line.strip():
                dll = None  # (a blank line ends one DLL's import rows)
                continue
            m = re.match(r"\s+[0-9a-f]+\s+(?:<none>|\d+)\s+(?:[0-9a-f]+\s+)?(\S+)\s*$", line)
            if dll and m and not line.strip().startswith("vma:"):
                imports[dll].append(m.group(1))
        return names, imports

    def demangle(self, names):
        if not names:
            return []
        return subprocess.run([self.tool("c++filt")], input="\n".join(names), capture_output=True, text=True, check=True,
                              env=self.env()).stdout.splitlines()

    def exports(self):
        binaries = {"libfsim.headless": self.headless / "bin" / "libfsim.dll",
                    "_native.headless": self.headless / "python" / "fsim" / "_native.pyd"}
        if self.release:
            binaries.update({"libfsim.release": self.release / "bin" / "libfsim.dll",
                             "libfsim_vision.release": self.release / "bin" / "libfsim_vision.dll",
                             "_native.release": self.release / "python" / "fsim" / "_native.pyd",
                             "_vision.release": self.release / "python" / "fsim" / "_vision.pyd"})
        summary, sets = {}, {}
        for label, binary in binaries.items():
            if not binary.is_file():
                summary[label] = {"missing": str(binary)}
                continue
            names, _ = self.exported(binary)
            demangled = self.demangle(names)
            self.write("exports/%s.exports.txt" % label, "\n".join(sorted(names)) + "\n")
            self.write("exports/%s.exports.demangled.txt" % label, "\n".join(sorted(demangled)) + "\n")
            vtables = sorted(d for d in demangled if d.startswith(("vtable for ", "typeinfo for ", "typeinfo name for ", "VTT for ")))
            self.write("exports/%s.vtables.txt" % label, "\n".join(vtables) + ("\n" if vtables else ""))
            sets[label] = set(names)
            summary[label] = {"path": binary.as_posix(), "sha256": sha256(binary), "bytes": binary.stat().st_size,
                              "exports": len(names), "c": sum(1 for n in names if not n.startswith("_Z")),
                              "cpp": sum(1 for n in names if n.startswith("_Z")), "vtable_and_typeinfo": len(vtables)}
        if "libfsim.release" in sets:
            summary["libfsim_headless_equals_release"] = sets["libfsim.headless"] == sets["libfsim.release"]
            if sets["libfsim.headless"] != sets["libfsim.release"]:
                self.failures.append("exports: the headless and release libfsim.dll export different names")
        if self.supplied:
            for label, name in (("libfsim.release", "native-exports.txt"), ("libfsim_vision.release", "vision-exports.txt")):
                dump = self.supplied / name
                if label not in sets or not dump.is_file():
                    continue
                supplied, table = set(), False
                for line in dump.read_text(encoding="utf-8", errors="replace").splitlines():
                    if "[Ordinal/Name Pointer] Table" in line:
                        table = True
                    elif table and not line.strip():
                        break
                    elif table and re.match(r"\s*\[\s*\d+\]", line):
                        supplied.add(line.split()[-1])
                summary[label]["supplied_dump"] = {"file": name, "exports": len(supplied),
                                                   "added_since": sorted(sets[label] - supplied),
                                                   "removed_since": sorted(supplied - sets[label])}
                if supplied != sets[label]:
                    self.failures.append("exports: %s differs from the supplied %s" % (label, name))
        stages = {"headless": self.headless / "python"}
        if self.release:
            stages["release"] = self.release / "python"
        for label, stage in stages.items():
            if not (stage / "fsim").is_dir():
                summary["python.%s" % label] = {"missing": str(stage)}
                continue
            surface = self.text([self.python, "-B", "-c", PY_SURFACE, REPO / "tools" / "boundary" / "boundary.py", stage],
                                env=self.env(), timeout=300)
            self.write("exports/python-calls.%s.txt" % label, surface)
            inventory = self.text([self.python, "-B", "-c", PY_INVENTORY, stage], env=self.env(), timeout=300)
            self.write("exports/python-inventory.%s.json" % label, inventory)
            parsed = json.loads(inventory)
            summary["python.%s" % label] = {"calls": len(surface.splitlines()), "modules": len(parsed),
                                            "names": sum(len(v) for v in parsed.values())}
        self.summary["exports"] = summary

    # ---- the test inventory
    def tests(self):
        summary = {}
        for preset, build in (("ucrt64-headless", self.headless), ("ucrt64-release", self.release)):
            if not build or not (build / "CMakeCache.txt").is_file():
                continue
            try:
                listing = self.text([self.tool("ctest"), "--preset", preset, "--show-only=json-v1"], timeout=600)
            except subprocess.CalledProcessError as e:
                self.failures.append("tests: ctest could not list %s: %s" % (preset, e.stderr[-400:]))
                continue
            self.write("tests/ctest-%s.json" % preset, listing)
            tests = json.loads(listing)["tests"]
            labels = {}
            for t in tests:
                found = [p["value"] for p in t.get("properties", []) if p["name"] == "LABELS"]
                for label in (found[0] if found else ["(none)"]):
                    labels[label] = labels.get(label, 0) + 1
            self.write("tests/ctest-%s.names.txt" % preset, "\n".join(t["name"] for t in tests) + "\n")
            summary[preset] = {"tests": len(tests), "labels": dict(sorted(labels.items()))}
            if not tests:
                self.failures.append("tests: %s lists no test" % preset)
        self.summary["tests"] = summary

    # ---- public types and vtables
    def dwarf(self, obj, keep):
        """Every complete struct, class, union and enumeration of an object file's debug information that `keep`
        accepts by its qualified name: sizes, bases, data members by offset, virtual functions by vtable slot, and
        enumerators by value."""
        proc = subprocess.Popen([self.tool("objdump"), "--dwarf=info", str(obj)], stdout=subprocess.PIPE, text=True,
                                encoding="utf-8", errors="replace", env=self.env())
        die = re.compile(r"\s*<(\d+)><([0-9a-f]+)>: Abbrev Number: \d+ \((DW_TAG_\w+)\)")
        attr = re.compile(r"\s*<[0-9a-f]+>\s+(DW_AT_\w+)\s*:\s?(.*)$")
        scopes = ("DW_TAG_namespace", "DW_TAG_structure_type", "DW_TAG_class_type", "DW_TAG_union_type")
        stack, dies = [], []
        for line in proc.stdout:
            m = die.match(line)
            if m:
                depth = int(m.group(1))
                del stack[depth:]
                entry = {"tag": m.group(3), "offset": int(m.group(2), 16), "attrs": {}, "children": [],
                         "scope": [d["attrs"].get("DW_AT_name") for d in stack[1:] if d["tag"] in scopes]}
                if stack and len(stack) == depth:
                    stack[-1]["children"].append(entry)
                stack.append(entry)
                if entry["tag"] in scopes[1:] + ("DW_TAG_enumeration_type",):
                    dies.append(entry)
                continue
            m = attr.match(line)
            if m and stack:
                value = m.group(2).strip()
                if m.group(1) == "DW_AT_name":
                    value = re.sub(r"^\(indirect (?:line )?string, offset: 0x[0-9a-f]+\): ", "", value)
                stack[-1]["attrs"].setdefault(m.group(1), value)
        proc.wait()

        def number(text):
            op = re.search(r"DW_OP_(?:plus_uconst|constu): (\d+)", text)
            return int(op.group(1)) if op else int(text.split()[0], 0)

        def qualified(entry):
            name = entry["attrs"].get("DW_AT_name")
            return None if name is None or None in entry["scope"] else "::".join(entry["scope"] + [name])
        named = {d["offset"]: qualified(d) for d in dies}
        types, enums = {}, {}
        for d in dies:
            name, a = qualified(d), d["attrs"]
            if "DW_AT_declaration" in a:
                continue
            if d["tag"] == "DW_TAG_enumeration_type":
                values = {c["attrs"]["DW_AT_name"]: number(c["attrs"]["DW_AT_const_value"]) for c in d["children"]
                          if c["tag"] == "DW_TAG_enumerator"}
                if name is None and values and None not in d["scope"]:  # (an unnamed enumeration: by its first name)
                    name = "::".join(d["scope"] + ["(unnamed: %s)" % next(iter(values))])
                if name and keep(name, values):
                    enums[name] = {"bytes": number(a["DW_AT_byte_size"]) if "DW_AT_byte_size" in a else None, "values": values}
                continue
            if name is None or "DW_AT_byte_size" not in a or not keep(name, ()):
                continue
            record = {"kind": d["tag"][7:-5], "size": number(a["DW_AT_byte_size"]), "bases": [], "members": [], "virtual": {}}
            for c in d["children"]:
                ca = c["attrs"]
                if c["tag"] == "DW_TAG_inheritance":
                    ref = re.search(r"<0x([0-9a-f]+)>", ca.get("DW_AT_type", ""))
                    record["bases"].append([named.get(int(ref.group(1), 16)) if ref else None,
                                            number(ca["DW_AT_data_member_location"]) if "DW_AT_data_member_location" in ca else None])
                elif c["tag"] == "DW_TAG_member" and "DW_AT_data_member_location" in ca:
                    member = [ca.get("DW_AT_name"), number(ca["DW_AT_data_member_location"])]
                    if "DW_AT_bit_size" in ca:
                        member.append("bits %s at %s" % (ca["DW_AT_bit_size"], ca.get("DW_AT_data_bit_offset", "?")))
                    record["members"].append(member)
                elif c["tag"] == "DW_TAG_subprogram" and "DW_AT_vtable_elem_location" in ca:
                    record["virtual"]["%s [%s]" % (ca.get("DW_AT_name"), ca.get("DW_AT_linkage_name", ""))] = number(ca["DW_AT_vtable_elem_location"])
            types[name] = record
        return {"types": types, "enums": enums}

    def types(self):
        work = self.out / "types"
        work.mkdir(parents=True, exist_ok=True)
        headers = sorted(p.name for p in (REPO / "include" / "fsim").glob("*.h"))
        (work / "all_public.cpp").write_text("".join('#include "fsim/%s"\n' % h for h in headers), encoding="utf-8")
        (work / "c_abi.c").write_text('#include "fsim/fsim_c.h"\n#include "fsim/fsim_vision_c.h"\n', encoding="utf-8")
        for stale in list(work.glob("*.class")) + list(work.glob("*.o")):
            stale.unlink()
        debug = ["-g", "-O0", "-fno-eliminate-unused-debug-types", "-c", "-I", REPO / "include"]
        ok = self.run("types-class-dump", [self.tool("g++"), "-std=c++17", "-fsyntax-only", "-fdump-lang-class", "-I",
                                           REPO / "include", "all_public.cpp"], cwd=work) == 0
        ok = self.run("types-cpp-debug", [self.tool("g++"), "-std=c++17"] + debug + ["all_public.cpp", "-o", "all_public.o"], cwd=work) == 0 and ok
        # The C header alone, as strict C99: what a C consumer compiles.
        ok = self.run("types-c99-debug", [self.tool("gcc"), "-std=c99", "-pedantic-errors", "-Wall", "-Wextra", "-Werror"] + debug
                      + ["c_abi.c", "-o", "c_abi.o"], cwd=work) == 0 and ok
        dumps = list(work.glob("*.class"))
        if not ok or len(dumps) != 1:
            self.failures.append("types: the public headers did not compile into layout metadata")
            return
        # The class hierarchy dump: each vtable's slots, thunks among them (compiler addresses removed).
        raw = re.sub(r" \(0x0x[0-9a-f]+\)", "", dumps[0].read_text(encoding="utf-8", errors="replace"))
        with gzip.GzipFile(work / "class-dump.all.txt.gz", "wb", mtime=0) as f:
            f.write(raw.encode("utf-8"))
        dumps[0].unlink()
        vtables, kept = {}, []
        for block in re.split(r"\n\s*\n", raw):
            lines = block.strip("\n").splitlines()
            m = re.match(r"(Vtable for|Class|Construction vtable for|VTT for) (.+)$", lines[0]) if lines else None
            if not m or not m.group(2).startswith(("fsim::", "fsim_")):
                continue
            kept.append(block.strip("\n"))
            if m.group(1) == "Vtable for":
                vtables[m.group(2)] = [line.split(None, 1)[1].strip() for line in lines if re.match(r"\d+\s", line)]
        self.write("types/public-class-dump.txt", "\n\n".join(kept) + "\n")
        self.write("types/public-vtables.json", json.dumps(vtables, indent=1, sort_keys=True) + "\n")
        cpp = self.dwarf(work / "all_public.o", lambda name, values: name.startswith("fsim::"))
        c = self.dwarf(work / "c_abi.o", lambda name, values: name.startswith(("fsim_", "FSIM_")) or any(v.startswith("FSIM_") for v in values))
        self.write("types/cpp-layout.json", json.dumps(cpp, indent=1, sort_keys=True) + "\n")
        self.write("types/c-abi-layout.json", json.dumps(c, indent=1, sort_keys=True) + "\n")
        for built in work.glob("*.o"):
            built.unlink()
        self.summary["types"] = {"headers": headers, "vtables": len(vtables),
                                 "cpp_types": len(cpp["types"]), "cpp_polymorphic": sum(1 for t in cpp["types"].values() if t["virtual"]),
                                 "cpp_enums": len(cpp["enums"]), "c_structs": len(c["types"]), "c_enums": len(c["enums"]),
                                 "c_enumerators": sum(len(e["values"]) for e in c["enums"].values())}

    # ---- frozen consumers
    def consumers(self):
        root = self.out / "consumers"
        prefix, pydist = root / "sdk-prefix", root / "python-dist"
        for owned in (prefix, pydist, root / "cpp-build"):
            if owned.exists():
                shutil.rmtree(owned)
        root.mkdir(parents=True, exist_ok=True)
        cmake = self.tool("cmake")
        ok = self.run("consumer-install-sdk", [cmake, "--install", self.headless, "--component", "sdk", "--prefix", prefix]) == 0
        ok = self.run("consumer-install-python", [cmake, "--install", self.headless, "--component", "python", "--prefix", pydist]) == 0 and ok
        if not ok:
            return
        # The SDK installs docs/sdk from the working tree. The frozen prefix is HEAD's package: a page the working
        # tree changed is put back to HEAD's bytes, and one HEAD does not have is taken out.
        installed = prefix / "share" / "doc" / "flightsim" / "sdk"
        tracked = set(self.text(["git", "ls-tree", "-r", "--name-only", "HEAD", "docs/sdk"]).splitlines())
        restored, removed = [], []
        for path in sorted(p for p in installed.rglob("*") if p.is_file()):
            name = "docs/sdk/" + path.relative_to(installed).as_posix()
            if name not in tracked:
                path.unlink()
                removed.append(name)
                continue
            blob = subprocess.run(["git", "show", "HEAD:" + name], cwd=REPO, capture_output=True, check=True).stdout
            if path.read_bytes() != blob:
                path.write_bytes(blob)
                restored.append(name)
        data = prefix / "share" / "flightsim"
        aircraft = "%s;%s" % ((data / "aircraft").as_posix(), (data / "aircraft" / "stores").as_posix())
        sealed = self.env(path=[str(prefix / "bin")], extra={"FSIM_AIRCRAFT_PATH": aircraft})
        results = {}

        # C++: the repository's package consumer, against the installed prefix alone.
        cache = (self.headless / "CMakeCache.txt").read_text(encoding="utf-8", errors="replace")
        toolchain = re.search(r"^CMAKE_TOOLCHAIN_FILE:[A-Z]+=(.+)$", cache, re.M).group(1)
        configured = self.run("consumer-cpp-configure", [
            cmake, "-S", REPO / "tests" / "package_consumer", "-B", root / "cpp-build", "-G", "Ninja",
            "-DCMAKE_TOOLCHAIN_FILE=" + toolchain, "-DFSIM_UCRT64_ROOT=" + self.ucrt.as_posix(),
            "-Dfsim_DIR=" + (prefix / "lib" / "cmake" / "fsim").as_posix(), "-DCMAKE_PREFIX_PATH=" + prefix.as_posix(),
            "-DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF", "-DCMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY=OFF", "-DCMAKE_BUILD_TYPE=Release"])
        if configured == 0 and self.run("consumer-cpp-build", [cmake, "--build", root / "cpp-build", "--target", "consumer"]) == 0:
            results["cpp"] = {"binary": "sdk-prefix/bin/consumer.exe", "source": "tests/package_consumer/main.cpp",
                              "exit": self.run("consumer-cpp-run", [prefix / "bin" / "consumer.exe", prefix, "smoke"], cwd=root,
                                               env=sealed, timeout=600)}

        # C99: the C ABI's own test, compiled as a consumer against the installed headers and import library.
        c99 = prefix / "bin" / "c_abi_consumer.exe"
        if self.run("consumer-c99-build", [self.tool("gcc"), "-std=c99", "-O2", "-I", prefix / "include", REPO / "tests" / "test_c_abi.c",
                                           "-o", c99, "-L", prefix / "lib", "-lfsim", "-lm"]) == 0:
            results["c99"] = {"binary": "sdk-prefix/bin/c_abi_consumer.exe", "source": "tests/test_c_abi.c",
                              "exit": self.run("consumer-c99-run", [c99, data / "jsbsim"], cwd=root, env=sealed, timeout=900)}

        # Python: the installed package (its wrappers and its stable-ABI module, built on today's C ABI) and the
        # examples that reach the P2P and C2 paths, frozen beside it.
        scripts = root / "python-consumers"
        scripts.mkdir(exist_ok=True)
        python = {}
        for name in PYTHON_CONSUMERS:
            shutil.copyfile(REPO / "examples" / "python" / name, scripts / name)
        (scripts / "_provenance.py").write_text(PY_PROVENANCE, encoding="utf-8")
        for name in PYTHON_CONSUMERS:
            label = "consumer-python-" + name[:-3]
            python[name] = self.run(label, [self.python, "-B", scripts / "_provenance.py", scripts / name], cwd=root,
                                    env=self.env(path=[], extra={"PYTHONPATH": str(pydist), "PYTHONHASHSEED": "0"}), timeout=900)
        results["python"] = {"package": "python-dist/fsim", "scripts": {k: {"exit": v} for k, v in python.items()}}

        # What each old binary needs of the library, and whether today's library gives it.
        exports, _ = self.exported(prefix / "bin" / "libfsim.dll")
        exports = set(exports)
        for label, binary in (("cpp", prefix / "bin" / "consumer.exe"), ("c99", c99), ("python", pydist / "fsim" / "_native.pyd")):
            if not binary.is_file():
                continue
            _, imports = self.exported(binary)
            needed = sorted(imports.get("libfsim.dll", []))
            self.write("consumers/%s.imports.txt" % label, "".join("%s: %s\n" % (dll, name) for dll in sorted(imports)
                                                                    for name in sorted(imports[dll])))
            results[label].update({"sha256": sha256(binary), "dlls": sorted(imports), "libfsim_imports": len(needed),
                                   "libfsim_imports_missing": sorted(set(needed) - exports)})
            if set(needed) - exports:
                self.failures.append("consumers: %s imports names libfsim.dll does not export" % label)

        def tree(base, relative_to):
            return "".join("%s  %s\n" % (sha256(p), p.relative_to(relative_to).as_posix())
                           for p in sorted(base.rglob("*")) if p.is_file() and "__pycache__" not in p.parts)
        self.write("consumers/sdk-binaries.sha256", tree(prefix / "bin", prefix))
        self.write("consumers/sdk-headers.sha256", tree(prefix / "include", prefix))
        self.write("consumers/sdk-runtime-data.sha256", tree(prefix / "share", prefix))
        self.write("consumers/python-package.sha256", tree(pydist, pydist))
        results["prefix"] = {"binaries": len(list((prefix / "bin").iterdir())),
                             "runtime_data_files": sum(1 for p in (prefix / "share" / "flightsim").rglob("*") if p.is_file()),
                             "document_files": sum(1 for p in (prefix / "share" / "doc").rglob("*") if p.is_file()),
                             "documents_restored_to_head": restored, "documents_not_in_head_removed": removed,
                             "installed_from": self.headless.as_posix()}
        if self.supplied and (self.supplied / "installed-cpp-binaries.sha256").is_file():
            wrong = []
            for line in (self.supplied / "installed-cpp-binaries.sha256").read_text(encoding="utf-8").splitlines():
                digest, _, name = line.partition("  ")
                path = self.supplied / "installed-cpp-consumer-bin" / name
                if not path.is_file() or sha256(path) != digest:
                    wrong.append(name)
            results["supplied_cpp_consumer"] = {"directory": (self.supplied / "installed-cpp-consumer-bin").as_posix(),
                                                "binaries": len((self.supplied / "installed-cpp-binaries.sha256").read_text().splitlines()),
                                                "runtime_data_listed": len((self.supplied / "installed-cpp-runtime-data.sha256").read_text().splitlines()),
                                                "hash_mismatches": wrong}
            if wrong:
                self.failures.append("consumers: the supplied C++ consumer does not match its manifest: %s" % ", ".join(wrong))
        self.summary["consumers"] = results

    # ---- the manifest
    def finish(self, head):
        tools = {}
        for name, command in (("gcc", [self.tool("gcc"), "--version"]), ("cmake", [self.tool("cmake"), "--version"]),
                              ("ninja", [self.tool("ninja"), "--version"]), ("objdump", [self.tool("objdump"), "--version"]),
                              ("python", [self.python, "--version"]), ("git", ["git", "--version"])):
            try:
                tools[name] = self.text(command).splitlines()[0].strip()
            except (OSError, subprocess.CalledProcessError) as e:
                tools[name] = "unavailable: %s" % e
        artifacts = {}
        skip = {"sdk-prefix", "python-dist", "cpp-build"}  # (listed by their own manifests above)
        for path in sorted(self.out.rglob("*")):
            relative = path.relative_to(self.out)
            if not path.is_file() or relative.name == MANIFEST or (len(relative.parts) > 1 and relative.parts[1] in skip):
                continue
            if relative.parts[0] == "logs" and (len(relative.parts) > 2 or re.match(r"\d\d-", relative.name)):
                continue  # (the operator's own numbered logs and later reruns: not this capture's, and still being written)
            artifacts[relative.as_posix()] = {"sha256": sha256(path), "bytes": path.stat().st_size}
        manifest = {"schema": SCHEMA, "captured_utc": datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
                    "repository": REPO.as_posix(), "head": head, "tools": tools, "summary": self.summary,
                    "failures": self.failures, "artifacts": artifacts}
        self.write(MANIFEST, json.dumps(manifest, indent=1, sort_keys=True) + "\n")
        return manifest


def capture(a):
    c = Capture(a)
    c.out.mkdir(parents=True, exist_ok=True)
    head = c.git()
    for step in (c.inputs, c.exports, c.tests, c.types, c.consumers):
        print("capture: %s" % step.__name__, flush=True)
        step()
    manifest = c.finish(head)
    print("%d artifacts; %d failures" % (len(manifest["artifacts"]), len(c.failures)))
    for failure in c.failures:
        print("FAILED " + failure)
    return 1 if c.failures else 0


def verify(a):
    out = Path(a.out).resolve()
    manifest = json.loads((out / MANIFEST).read_text(encoding="utf-8"))
    wrong = [name for name, entry in manifest["artifacts"].items()
             if not (out / name).is_file() or sha256(out / name) != entry["sha256"]]
    for listing, base in (("consumers/sdk-binaries.sha256", "consumers/sdk-prefix"), ("consumers/sdk-headers.sha256", "consumers/sdk-prefix"),
                          ("consumers/sdk-runtime-data.sha256", "consumers/sdk-prefix"), ("consumers/python-package.sha256", "consumers/python-dist")):
        if not (out / listing).is_file():
            continue
        for line in (out / listing).read_text(encoding="utf-8").splitlines():
            digest, _, relative = line.partition("  ")
            path = out / base / relative
            if not path.is_file() or sha256(path) != digest:
                wrong.append("%s/%s" % (base, relative))
    for name in wrong:
        print("CHANGED " + name)
    print("%s at %s: %d artifacts listed, %d changed or missing" % (manifest["schema"], manifest["head"][:7], len(manifest["artifacts"]), len(wrong)))
    return 1 if wrong else 0


def run(a):
    """The frozen consumers again: as captured, or (--sdk-prefix) the old binaries on another build's installed library."""
    a.headless_build, a.release_build, a.supplied = str(REPO / "build" / "ucrt64-headless"), None, None
    c = Capture(a)
    c.logs = c.out / "logs" / ("rerun-" + datetime.datetime.now().strftime("%Y%m%dT%H%M%S"))
    root = c.out / "consumers"
    frozen, pydist = root / "sdk-prefix", root / "python-dist"
    prefix = Path(a.sdk_prefix).resolve() if a.sdk_prefix else frozen
    data = prefix / "share" / "flightsim"
    if not (prefix / "bin" / "libfsim.dll").is_file() or not (data / "jsbsim").is_dir():
        print("%s is not an installed SDK prefix (bin/libfsim.dll, share/flightsim/jsbsim)" % prefix)
        return 1
    work, package = frozen / "bin", pydist
    extra = {"PYTHONHASHSEED": "0"}
    if a.sdk_prefix:
        # The old executables alone, in a directory of their own: Windows then finds libfsim.dll on PATH - the other
        # prefix's bin - and the library finds its data beside itself. The old Python module likewise: the frozen
        # package without the DLLs and data it was installed with.
        work = c.logs / "old-binaries"
        work.mkdir(parents=True, exist_ok=True)
        for exe in ("consumer.exe", "c_abi_consumer.exe"):
            if (frozen / "bin" / exe).is_file():
                shutil.copyfile(frozen / "bin" / exe, work / exe)
        package = c.logs / "old-python-package"
        shutil.copytree(pydist / "fsim", package / "fsim", ignore=shutil.ignore_patterns("bin", "share", "__pycache__"))
        extra["FSIM_DLL_DIR"] = str(prefix / "bin")
    if a.python_package:
        package = Path(a.python_package).resolve()
        extra.pop("FSIM_DLL_DIR", None)  # (another package altogether brings its own library)
    extra["PYTHONPATH"] = str(package)
    sealed = c.env(path=[str(prefix / "bin")], extra={"FSIM_AIRCRAFT_PATH": "%s;%s" % ((data / "aircraft").as_posix(), (data / "aircraft" / "stores").as_posix())})
    results = {}
    if (work / "consumer.exe").is_file():
        results["cpp"] = c.run("consumer-cpp-run", [work / "consumer.exe", prefix, "smoke"], cwd=root, env=sealed, timeout=600, check=False)
    if (work / "c_abi_consumer.exe").is_file():
        results["c99"] = c.run("consumer-c99-run", [work / "c_abi_consumer.exe", data / "jsbsim"], cwd=root, env=sealed, timeout=900, check=False)
    for name in PYTHON_CONSUMERS:
        results["python:" + name] = c.run("consumer-python-" + name[:-3], [c.python, "-B", root / "python-consumers" / "_provenance.py",
                                                                          root / "python-consumers" / name], cwd=root,
                                          env=c.env(path=[], extra=extra), timeout=900, check=False)
    for name, rc in results.items():
        print("%s: exit %s" % (name, rc))
    print("library: %s" % (prefix / "bin" / "libfsim.dll").as_posix())
    print("logs: %s" % c.logs.as_posix())
    return 0 if results and all(rc == 0 for rc in results.values()) else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("action", choices=("capture", "verify", "run"))
    ap.add_argument("--out", required=True, help="the baseline's directory (under D:/ENV)")
    ap.add_argument("--headless-build", default=str(REPO / "build" / "ucrt64-headless"))
    ap.add_argument("--release-build", default=None, help="a render build, for the vision library's exports")
    ap.add_argument("--supplied", default=None, help="a directory of supplied compatibility artifacts to check against")
    ap.add_argument("--ucrt64", default=os.environ.get("FSIM_UCRT64_ROOT", DEFAULT_UCRT64))
    ap.add_argument("--tmp", default=DEFAULT_TMP)
    ap.add_argument("--python", default=sys.executable)
    ap.add_argument("--sdk-prefix", default=None, help="run: another build's installed SDK to run the frozen binaries against")
    ap.add_argument("--python-package", default=None)
    a = ap.parse_args()
    return {"capture": capture, "verify": verify, "run": run}[a.action](a)


if __name__ == "__main__":
    sys.exit(main())
