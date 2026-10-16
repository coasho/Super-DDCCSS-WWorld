"""The compatibility comparison's own refusals (tools/ma_contract/baseline.py compare; M8).

    python tools/ma_contract/test_baseline.py --build <a build tree> [--ucrt64 <root>]

The comparator holds each frozen Python description to its contract - signatures and defaults, enumerator values,
record fields in order, class members and their kinds, exception bases, constants and the runtime owner of each name -
and admits additions alone. The command requires every artifact of each build it was given, captured fresh by that
invocation: a missing DLL, Python package or extension module fails it, and so does a work directory left by an
earlier run. Standard library only; the CLI cases capture the given build's real binaries into temporary directories
and leave the frozen baseline untouched."""
import argparse
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import baseline  # noqa: E402

ARGS = None

BEFORE = {
    "fsim.x": {
        "f": {"kind": "function", "signature": "(a, b=1)", "home": "fsim.x"},
        "E": {"kind": "enum", "members": {"A": 0, "B": 1}, "home": "fsim.x"},
        "R": {"kind": "record", "fields": ["a", "b"], "home": "fsim.x"},
        "C": {"kind": "class", "members": {"__init__": "method(self, x, y=0)", "m": "method(self, x)", "p": "property",
                                           "s": "staticmethod(a)"}, "home": "fsim._native"},
        "X": {"kind": "exception", "bases": ["Error"], "home": "fsim.x"},
        "K": {"kind": "constant", "type": "int", "value": 3, "home": None},
        "V": {"kind": "value", "type": "dict", "home": None},
    },
}


def changed(path, value):
    """BEFORE with one description changed (path: module, name, key[, member])."""
    now = copy.deepcopy(BEFORE)
    target = now[path[0]][path[1]]
    for key in path[2:-1]:
        target = target[key]
    if value is None:
        del target[path[-1]]
    else:
        target[path[-1]] = value
    return now


class ContractTest(unittest.TestCase):
    """Each frozen description against its contract: a change fails it, an addition does not."""

    def refused(self, now, name):
        failures = baseline.compare_inventory(BEFORE, now, "inventory")
        self.assertTrue(failures, "a changed %s passed" % name)
        self.assertTrue(any(name in f for f in failures), failures)

    def admitted(self, now):
        self.assertEqual(baseline.compare_inventory(BEFORE, now, "inventory"), [])

    def test_unchanged_passes(self):
        self.admitted(copy.deepcopy(BEFORE))

    def test_signatures_and_defaults(self):
        for signature in ("(a, b=2)", "(a)", "(b, a=1)", "(a, b)", "(a, b=1, c)", "(a, *, b=1)"):
            with self.subTest(signature=signature):
                self.refused(changed(("fsim.x", "f", "signature"), signature), "fsim.x.f")
        for signature in ("(a, b=1, c=None)", "(a, b=1, *, c=0)", "(a, b=1, **options)", "(a, b=1, *args)"):
            with self.subTest(signature=signature):
                self.admitted(changed(("fsim.x", "f", "signature"), signature))

    def test_enumerators(self):
        self.refused(changed(("fsim.x", "E", "members", "B"), 2), "fsim.x.E.B")
        self.refused(changed(("fsim.x", "E", "members", "B"), None), "fsim.x.E.B")
        self.admitted(changed(("fsim.x", "E", "members", "C"), 2))

    def test_record_fields_in_order(self):
        for fields in (["b", "a"], ["a"], ["a", "c"]):
            with self.subTest(fields=fields):
                self.refused(changed(("fsim.x", "R", "fields"), fields), "fsim.x.R")
        self.admitted(changed(("fsim.x", "R", "fields"), ["a", "b", "c"]))

    def test_class_members_and_kinds(self):
        self.refused(changed(("fsim.x", "C", "members", "p"), "method(self)"), "fsim.x.C.p")
        self.refused(changed(("fsim.x", "C", "members", "m"), "property"), "fsim.x.C.m")
        self.refused(changed(("fsim.x", "C", "members", "m"), "method(self, y)"), "fsim.x.C.m")
        self.refused(changed(("fsim.x", "C", "members", "s"), "classmethod(a)"), "fsim.x.C.s")
        self.refused(changed(("fsim.x", "C", "members", "__init__"), "method(self, x, y=1)"), "fsim.x.C.__init__")
        self.refused(changed(("fsim.x", "C", "members", "m"), None), "fsim.x.C.m")
        self.admitted(changed(("fsim.x", "C", "members", "n"), "method(self)"))
        self.admitted(changed(("fsim.x", "C", "members", "m"), "method(self, x, flag=False)"))

    def test_runtime_owners_and_kinds(self):
        self.refused(changed(("fsim.x", "C", "home"), "fsim.world"), "fsim.x.C")
        self.refused(changed(("fsim.x", "f", "home"), "fsim.y"), "fsim.x.f")
        self.refused(changed(("fsim.x", "E", "kind"), "class"), "fsim.x.E")

    def test_exceptions_constants_values_and_names(self):
        self.refused(changed(("fsim.x", "X", "bases"), ["RuntimeError"]), "fsim.x.X")
        self.refused(changed(("fsim.x", "K", "value"), 4), "fsim.x.K")
        self.refused(changed(("fsim.x", "V", "type"), "list"), "fsim.x.V")
        now = copy.deepcopy(BEFORE)
        del now["fsim.x"]["f"]
        self.refused(now, "fsim.x.f")
        now = copy.deepcopy(BEFORE)
        now["fsim.x"]["g"] = {"kind": "function", "signature": "()", "home": "fsim.x"}
        now["fsim.y"] = {}
        self.admitted(now)

    def test_modules(self):
        self.refused({"fsim.x": {"unavailable": "No module named fsim.x"}}, "fsim.x")
        self.refused({}, "fsim.x")


def synthetic_baseline(root, exports):
    """A frozen baseline with one export, empty types, and the Python surface of nothing: what a correct build passes."""
    for name, text in (("exports/libfsim.headless.exports.txt", "\n".join(exports) + "\n"),
                       ("exports/libfsim.headless.vtables.txt", ""),
                       ("exports/python-calls.headless.txt", ""),
                       ("exports/python-inventory.headless.json", "{}"),
                       ("types/cpp-layout.json", json.dumps({"types": {}, "enums": {}})),
                       ("types/c-abi-layout.json", json.dumps({"types": {}, "enums": {}})),
                       ("types/public-vtables.json", "{}")):
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")
    return root


class CommandTest(unittest.TestCase):
    """The real command against real and deliberately incomplete builds."""

    @classmethod
    def setUpClass(cls):
        cls.tmp = Path(tempfile.mkdtemp(prefix="ma-baseline-test-", dir=ARGS.tmp))
        cls.base = synthetic_baseline(cls.tmp / "baseline", ["fsim_abi_version"])
        cls.build = Path(ARGS.build).resolve()

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.tmp, ignore_errors=True)

    def compare(self, headless, work, release=None):
        command = [sys.executable, "-B", str(HERE / "baseline.py"), "compare", "--out", str(self.base), "--headless-build", str(headless),
                   "--work", str(work), "--ucrt64", ARGS.ucrt64, "--tmp", ARGS.tmp, "--python", sys.executable]
        if release:
            command += ["--release-build", str(release)]
        run = subprocess.run(command, capture_output=True, text=True, timeout=900)
        return run.returncode, run.stdout + run.stderr

    def fake(self, name, dll=False, package=False):
        """A build tree with only some of its artifacts."""
        root = self.tmp / name
        if dll:
            (root / "bin").mkdir(parents=True, exist_ok=True)
            shutil.copy2(self.build / "bin" / "libfsim.dll", root / "bin" / "libfsim.dll")
        if package:
            (root / "python" / "fsim").mkdir(parents=True, exist_ok=True)
            (root / "python" / "fsim" / "__init__.py").write_text("", encoding="utf-8")
        return root

    def test_a_complete_build_passes(self):
        code, out = self.compare(self.build, self.tmp / "work-complete")
        self.assertEqual(code, 0, out)

    def test_a_missing_dll_fails(self):
        code, out = self.compare(self.tmp / "no-such-build", self.tmp / "work-no-build")
        self.assertNotEqual(code, 0, out)
        self.assertIn("libfsim.headless", out)

    def test_a_missing_python_package_fails(self):
        code, out = self.compare(self.fake("dll-only", dll=True), self.tmp / "work-dll-only")
        self.assertNotEqual(code, 0, out)
        self.assertIn("python.headless", out)

    def test_a_missing_extension_module_fails(self):
        code, out = self.compare(self.fake("no-extension", dll=True, package=True), self.tmp / "work-no-extension")
        self.assertNotEqual(code, 0, out)
        self.assertIn("_native.headless", out)

    def test_a_selected_release_build_must_be_complete(self):
        code, out = self.compare(self.build, self.tmp / "work-release", release=self.tmp / "no-such-release")
        self.assertNotEqual(code, 0, out)
        self.assertIn("libfsim.release", out)

    def test_a_reused_work_directory_is_never_read(self):
        work = self.tmp / "work-reused"
        code, out = self.compare(self.build, work)
        self.assertEqual(code, 0, out)
        self.assertTrue((work / "exports" / "libfsim.headless.exports.txt").is_file())
        # the same work directory, now for a build that has nothing: the earlier capture must not stand in for it
        code, out = self.compare(self.tmp / "no-such-build-either", work)
        self.assertNotEqual(code, 0, out)
        self.assertFalse((work / "exports" / "libfsim.headless.exports.txt").is_file())

    def test_the_baseline_itself_is_never_the_work_directory(self):
        code, out = self.compare(self.build, self.base)
        self.assertNotEqual(code, 0, out)
        self.assertTrue((self.base / "exports" / "libfsim.headless.exports.txt").is_file())


def main():
    global ARGS
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--build", required=True, help="a build tree with bin/libfsim.dll and its staged python package")
    ap.add_argument("--ucrt64", default=os.environ.get("FSIM_UCRT64_ROOT", baseline.DEFAULT_UCRT64))
    ap.add_argument("--tmp", default=baseline.DEFAULT_TMP)
    ARGS, rest = ap.parse_known_args()
    unittest.main(argv=[sys.argv[0]] + rest, verbosity=2)


if __name__ == "__main__":
    main()
