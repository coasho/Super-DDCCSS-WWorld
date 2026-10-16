"""The change-scoped selector's own checks (tools/testing/scope.py, scopes.toml, timing.py).

    python tools/testing/test_scope.py [--test-dir <a configured build>]

The selection rules against synthetic inventories - labels and their exclusions per tier, names, escalation, the
render preset, tests built from, run on, importing or reading a changed file - and its failures: an unclaimed path,
an empty selection and no changes are errors, through the command itself. The lifecycle with its effects replaced:
a build before the inventory is listed, names resolved to the rebuilt inventory's numbers, and a run held to exactly
its selection. Discovery in small private trees: the Python import closure (shared fixtures and scripts a test runs
included), compile commands and local headers, and excluded directories never entered. Given a build, the rules file
against the real repository and that build's CTest inventory: every path pattern matches a tracked file, and every
label and test pattern a scope names selects at least one test (a render scope's patterns excepted on a build without
the renderer). No case here launches a native tool, build or CTest run except listing that given build's inventory.
Standard library only."""
import argparse
import contextlib
import io
import json
import os
import re
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import scope  # noqa: E402
import timing  # noqa: E402

ARGS = None

TESTS = [
    {"index": 1, "name": "ma_joint", "labels": ["ma-training"], "command": ["D:/b/tests/fsim_ma_joint_test.exe"]},
    {"index": 2, "name": "mission_training_stress batch", "labels": ["ma-training", "stress"], "command": ["D:/b/tests/fsim_mission_training_test.exe", "[concurrency][batch]"]},
    {"index": 3, "name": "mission_training", "labels": ["ma-training"], "command": ["D:/b/tests/fsim_mission_training_test.exe", "~[concurrency]"]},
    {"index": 4, "name": "mission_package_consumer", "labels": ["ma-training", "package"], "command": ["cmake.exe", "-P", "x.cmake"]},
    {"index": 5, "name": "python_sdk test_world", "labels": ["python_sdk"], "command": ["python.exe", "-m", "unittest", "test_world"]},
    {"index": 6, "name": "python_sdk test_mission_concurrency.A.test_x", "labels": ["python_sdk", "stress"], "command": ["python.exe"]},
    {"index": 7, "name": "hangar test_methods.Model3D", "labels": ["hangar"], "command": ["python.exe"]},
    {"index": 8, "name": "sdk_boundary", "labels": [], "command": ["python.exe", str(scope.REPO / "tools" / "boundary" / "boundary.py"), "check"]},
    {"index": 9, "name": "flight core", "labels": [], "command": ["D:/b/tests/fsim_core_test.exe"]},
    {"index": 10, "name": "crash_handler", "labels": [], "command": ["cmake.exe", "-DEXE=D:/b/tests/fsim_crash_test.exe", "-P", "crash.cmake"]},
    {"index": 11, "name": "fixture reader", "labels": [], "command": ["D:/b/tests/fsim_reader.exe", str(scope.REPO / "tests" / "fixtures" / "set")]},
    {"index": 12, "name": "python_sdk test_mission_training", "labels": ["python_sdk", "ma-training"], "command": ["python.exe"]},
    {"index": 13, "name": "vecenv", "labels": [], "command": ["D:/b/tests/fsim_vecenv_test.exe"]},
]
RULES = [
    {"name": "docs", "paths": ["docs/**", "**/*.md"], "tests": ["^sdk_boundary$"], "build": False},
    {"name": "gateway", "paths": ["src/sdk/Mission*"], "labels": ["ma-training"], "exclude_labels": ["stress"]},
    {"name": "joint", "paths": ["tools/joint.py"], "tests": ["^ma_joint$"], "integration": ["^mission_package_consumer$"],
     "integration_labels": ["stress"]},
    {"name": "concurrency", "paths": ["src/sdk/MissionBatch.cpp"], "labels": ["stress"]},
    {"name": "surface", "paths": ["include/fsim/Mission*.h"], "tests": ["^sdk_boundary$"], "release": True},
    {"name": "core", "paths": ["src/session/**", "cmake/**"], "except": ["cmake/RunMissionPackageConsumer.cmake"], "full": True},
    {"name": "consumers", "paths": ["cmake/RunMissionPackageConsumer.cmake"], "tests": ["^mission_package_consumer$"]},
    {"name": "stem", "paths": ["tools/fixtures/*.json"], "tests": ["^{stem}$"]},
    {"name": "nothing here", "paths": ["tools/orphan/**"], "tests": ["^no such test$"]},
    {"name": "package exports", "paths": ["python/fsim/**"], "tests": ["^sdk_boundary$"], "claims": False},
]
EXECUTABLES = {"tests/test_ma_joint.cpp": {"fsim_ma_joint_test"}, "tests/test_mission_training.cpp": {"fsim_mission_training_test"},
               "tests/test_crash.cpp": {"fsim_crash_test"}}
DEPENDENTS = {  # (python_dependents of a tree where test_mission_concurrency imports test_mission_training's fixtures)
    "python/fsim/world.py": {"test_world", "test_mission_training", "test_mission_concurrency"},
    "python/tests/test_mission_training.py": {"test_mission_training", "test_mission_concurrency"},
    "python/tests/test_mission_concurrency.py": {"test_mission_concurrency"},
    "python/tests/test_world.py": {"test_world"},
}


def selected(paths, tier="targeted", rules=RULES, readers=None):
    return scope.plan(paths, rules, TESTS, EXECUTABLES, tier, DEPENDENTS, readers)


def reasons(p):
    return {e["reason"]: e for e in p["errors"]}


class GlobTest(unittest.TestCase):
    def test_patterns(self):
        for path, pattern, expected in (("docs/a.md", "docs/**", True), ("docs/x/y/a.md", "docs/**", True), ("a.md", "**/*.md", True),
                                        ("docs/x/a.md", "**/*.md", True), ("src/sdk/MissionBatch.cpp", "src/sdk/Mission*", True),
                                        ("src/sdk/x/MissionBatch.cpp", "src/sdk/Mission*", False), ("src/env/a.h", "src/env/**", True),
                                        ("src/environment/a.h", "src/env/**", False), ("cmake/a.cmake", "cmake/*.cmake", True)):
            with self.subTest(path=path, pattern=pattern):
                self.assertEqual(scope.glob_match(path, pattern), expected)


class PlanTest(unittest.TestCase):
    def test_a_domain_change_selects_its_label_without_stress(self):
        p = selected(["src/sdk/MissionGateway.cpp"])
        self.assertEqual(p["selected"], ["ma_joint", "mission_package_consumer", "mission_training", "python_sdk test_mission_training"])
        self.assertFalse(p["escalate"])
        self.assertTrue(p["build"])
        self.assertEqual(p["errors"], [])

    def test_a_concurrency_change_adds_the_stress_tests(self):
        self.assertIn("mission_training_stress batch", selected(["src/sdk/MissionBatch.cpp"])["selected"])
        self.assertIn("python_sdk test_mission_concurrency.A.test_x", selected(["src/sdk/MissionBatch.cpp"])["selected"])

    def test_integration_takes_the_domains_whole_labels(self):
        self.assertIn("mission_training_stress batch", selected(["src/sdk/MissionGateway.cpp"], "integration")["selected"])

    def test_a_broad_pattern_never_pulls_in_the_stress_cases(self):
        rules = RULES + [{"name": "python", "paths": ["python/fsim/mission.py"], "tests": ["^python_sdk test_mission_concurrency"]}]
        p = scope.plan(["python/fsim/mission.py"], rules, TESTS, EXECUTABLES)
        self.assertEqual(p["selected"], ["sdk_boundary"])  # (the pattern names only the stress module; nothing imports mission.py here)
        p = scope.plan(["python/fsim/mission.py", "src/sdk/MissionBatch.cpp"], rules, TESTS, EXECUTABLES)
        self.assertIn("python_sdk test_mission_concurrency.A.test_x", p["selected"])  # (with concurrency code changed)

    def test_integration_adds_its_names_and_labels(self):
        self.assertEqual(selected(["tools/joint.py"])["selected"], ["ma_joint"])
        self.assertEqual(selected(["tools/joint.py"], "integration")["selected"],
                         ["ma_joint", "mission_package_consumer", "mission_training_stress batch", "python_sdk test_mission_concurrency.A.test_x"])

    def test_documentation_needs_no_build(self):
        p = selected(["docs/sdk/a.md"])
        self.assertEqual(p["selected"], ["sdk_boundary"])
        self.assertFalse(p["build"])

    def test_shared_core_and_unclaimed_native_code_escalate_to_the_full_suite(self):
        for path in ("src/session/World.cpp", "src/ms/Radar.cpp", "CMakeLists.txt", "cmake/Toolchain.cmake"):
            with self.subTest(path=path):
                p = selected([path])
                self.assertTrue(p["escalate"])
                self.assertEqual(len(p["selected"]), len(TESTS))
                self.assertEqual(p["errors"], [])

    def test_a_more_specific_scope_keeps_a_file_out_of_an_escalation(self):
        p = selected(["cmake/RunMissionPackageConsumer.cmake"])
        self.assertFalse(p["escalate"])
        self.assertEqual(p["selected"], ["mission_package_consumer"])

    def test_a_public_surface_change_needs_the_render_preset(self):
        self.assertTrue(selected(["include/fsim/MissionBatch.h"])["release"])

    def test_a_test_source_selects_every_test_built_from_it(self):
        self.assertEqual(selected(["tests/test_ma_joint.cpp"])["selected"], ["ma_joint"])
        self.assertEqual(selected(["tests/test_mission_training.cpp"])["selected"], ["mission_training", "mission_training_stress batch"])
        self.assertEqual(selected(["tests/test_crash.cpp"])["selected"], ["crash_handler"])  # (run by a CMake script: -DEXE=)

    def test_a_python_test_module_selects_its_tests(self):
        self.assertEqual(selected(["python/tests/test_world.py"])["selected"], ["python_sdk test_world"])
        self.assertEqual(selected(["python/tests/test_mission_concurrency.py"])["selected"], ["python_sdk test_mission_concurrency.A.test_x"])
        self.assertEqual(selected(["tools/hangar/tests/test_methods.py"])["selected"], ["hangar test_methods.Model3D"])

    def test_a_file_a_test_runs_selects_that_test(self):
        self.assertEqual(selected(["tools/boundary/boundary.py"])["selected"], ["sdk_boundary"])

    def test_a_file_in_a_fixture_directory_a_test_is_given_selects_that_test(self):
        self.assertEqual(selected(["tests/fixtures/set/case.json"])["selected"], ["fixture reader"])

    def test_a_stem_pattern(self):
        self.assertEqual(selected(["tools/fixtures/ma_joint.json"])["selected"], ["ma_joint"])

    # T1 - fail closed
    def test_unclaimed_python_code_is_an_error_not_an_empty_success(self):
        p = selected(["notes/plan.txt"])
        self.assertEqual(p["selected"], [])
        self.assertEqual(reasons(p)["unclaimed"]["paths"], ["notes/plan.txt"])
        self.assertEqual(reasons(p)["empty-selection"]["paths"], ["notes/plan.txt"])
        # a module nothing imports: the export check a `claims = false` scope adds does not make it covered
        p = selected(["python/fsim/orphan.py"])
        self.assertEqual(p["selected"], ["sdk_boundary"])
        self.assertFalse(p["escalate"])
        self.assertEqual(reasons(p)["unclaimed"]["paths"], ["python/fsim/orphan.py"])

    def test_a_scope_that_selects_nothing_is_an_error(self):
        p = selected(["tools/orphan/a.py"])
        self.assertEqual(p["scopes"], ["nothing here"])
        self.assertNotIn("unclaimed", reasons(p))
        self.assertEqual(reasons(p)["empty-selection"]["scopes"], ["nothing here"])

    def test_no_changes_is_an_error_unless_the_full_tier_is_asked_for(self):
        self.assertIn("no-changes", reasons(selected([])))
        self.assertEqual(selected([], "full")["errors"], [])
        self.assertEqual(len(selected([], "full")["selected"]), len(TESTS))

    def test_an_unclaimed_path_beside_a_claimed_one_still_fails(self):
        p = selected(["docs/sdk/a.md", "notes/plan.txt"])
        self.assertEqual(p["selected"], ["sdk_boundary"])
        self.assertEqual(reasons(p)["unclaimed"]["paths"], ["notes/plan.txt"])

    # T2 - dependency closure
    def test_a_production_python_module_selects_every_test_importing_it_stress_included(self):
        p = selected(["python/fsim/world.py"])
        self.assertEqual(p["selected"], ["python_sdk test_mission_concurrency.A.test_x", "python_sdk test_mission_training", "python_sdk test_world",
                                         "sdk_boundary"])
        self.assertEqual(p["errors"], [])

    def test_a_shared_fixture_module_selects_the_stress_cases_importing_it(self):
        p = selected(["python/tests/test_mission_training.py"])
        self.assertEqual(p["selected"], ["python_sdk test_mission_concurrency.A.test_x", "python_sdk test_mission_training"])

    def test_a_data_file_selects_the_tests_of_the_sources_reading_it(self):
        readers = {"tests/data/track/crosscheck.json": {"tests/test_crash.cpp"}}
        self.assertEqual(selected(["tests/data/track/crosscheck.json"], readers=readers)["selected"], ["crash_handler"])

    def test_the_legacy_environment_escalates_and_the_mission_episode_owner_does_not(self):
        rules = scope.load_rules()
        legacy = scope.plan(["src/env/VecEnv.cpp"], rules, TESTS, EXECUTABLES, "targeted", DEPENDENTS)
        self.assertTrue(legacy["escalate"])
        self.assertIn("vecenv", legacy["selected"])
        owner = scope.plan(["src/env/MissionEpisode.cpp"], rules, TESTS, EXECUTABLES, "targeted", DEPENDENTS)
        self.assertFalse(owner["escalate"])
        self.assertIn("MA episode owner (native)", owner["scopes"])
        self.assertNotIn("vecenv", owner["selected"])

    def test_estimates_add_serial_tests_and_pack_the_rest_by_processors(self):
        t = lambda name, serial=False, processors=1: {"name": name, "serial": serial, "processors": processors}  # noqa: E731
        self.assertEqual(scope.estimate([t("a"), t("b")], {"a": 100.0, "b": 10.0}, 16), (100.0, 0))
        self.assertEqual(scope.estimate([t("a"), t("b"), t("c")], {"a": 10.0, "b": 10.0}, 1), (20.0, 1))
        self.assertEqual(scope.estimate([t("c")], {}, 16), (None, 1))
        self.assertEqual(scope.estimate([t("a", serial=True), t("b"), t("d")], {"a": 50.0, "b": 10.0, "d": 10.0}, 16), (60.0, 0))
        self.assertEqual(scope.estimate([t("b", processors=2), t("d", processors=2)], {"b": 10.0, "d": 10.0}, 2), (20.0, 0))


def synthetic(names, start=1):
    return [{"index": start + k, "name": n, "labels": [], "command": ["x.exe"], "serial": False, "processors": 1} for k, n in enumerate(names)]


class LifecycleTest(unittest.TestCase):
    """scope.select with its effects replaced: what is built, listed, resolved and run, in which order."""

    RULES = [{"name": "domain", "paths": ["src/sdk/MissionGateway.cpp"], "tests": ["^b$"]}]

    def setUp(self):
        self.calls = []
        self.tmp = tempfile.TemporaryDirectory()
        self.inventories = {"before": synthetic(["a", "b", "c"]), "after": synthetic(["new 1", "new 2", "a", "c", "b"])}
        self.state = "before"

    def tearDown(self):
        self.tmp.cleanup()

    def builder(self, preset):
        self.calls.append("build " + preset)
        self.state = "after"  # (a reconfigure added two tests and reordered the rest)
        return 0

    def discover(self, preset):
        self.calls.append("list " + preset)
        return self.inventories[self.state]

    def runner(self, ran):
        def run(command, log):
            self.calls.append("run " + " ".join(command[-2:]))
            Path(log).write_text("".join(" %d/%d Test #%d: %s ....   Passed    1.00 sec\n" % (k + 1, len(ran), k + 1, n)
                                         for k, n in enumerate(ran)), encoding="utf-8")
            return 0
        return run

    def select(self, ran=("b",), build=True, run=True, paths=("src/sdk/MissionGateway.cpp",)):
        return scope.select(list(paths), self.RULES, "targeted", 16, run=run, build=build, log_dir=self.tmp.name, executables={},
                            dependents={}, readers={}, discover=self.discover, builder=self.builder, runner=self.runner(ran),
                            measured=lambda preset: {}, recorder=lambda logs, preset: 0)

    def test_the_build_comes_first_and_names_resolve_to_the_rebuilt_inventory(self):
        status, report = self.select()
        self.assertEqual(status, 0, report["errors"])
        self.assertEqual(self.calls, ["build ucrt64-headless", "list ucrt64-headless", "run -I 0,0,0,5"])
        self.assertEqual(report["presets"]["ucrt64-headless"]["selected"], ["b"])

    def test_without_a_build_the_current_inventory_is_used(self):
        status, report = self.select(build=False)
        self.assertEqual(status, 0, report["errors"])
        self.assertEqual(self.calls, ["list ucrt64-headless", "run -I 0,0,0,2"])

    def test_a_run_that_executed_other_tests_fails(self):
        status, report = self.select(ran=("b", "new 1"))
        self.assertEqual(status, 4)
        error = next(e for e in report["errors"] if e["reason"] == "ran-other-tests")
        self.assertEqual(error["extra"], ["new 1"])
        status, report = self.select(ran=())
        self.assertEqual(status, 4)

    def test_a_failing_build_stops_before_listing_or_running(self):
        self.builder = lambda preset: (self.calls.append("build " + preset), 2)[1]
        status, report = self.select()
        self.assertEqual(status, 2)
        self.assertEqual(self.calls, ["build ucrt64-headless"])

    def test_a_plan_error_runs_nothing(self):
        status, report = self.select(paths=("notes/plan.txt",))
        self.assertEqual(status, 3)
        self.assertNotIn("run", " ".join(self.calls))


class FixtureTest(unittest.TestCase):
    """A tiny independent CMake project (no compiler, no shared build tree): a test module added after configuring is
    registered by the next build - the glob is CONFIGURE_DEPENDS, as python/CMakeLists.txt's - and the selector,
    building first, resolves the new module's name to its number in the rebuilt inventory, where it sorts first."""

    LISTS = """cmake_minimum_required(VERSION 3.25)
project(selector_fixture NONE)
enable_testing()
file(GLOB modules CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/python/tests/test_*.py)
foreach(module ${modules})
    get_filename_component(name ${module} NAME_WE)
    add_test(NAME "python_sdk ${name}" COMMAND ${CMAKE_COMMAND} -E true)
endforeach()
"""

    def test_an_added_module_is_registered_by_the_build_and_selected_by_its_new_number(self):
        tools = Path(scope.CTEST).parent
        cmake, ninja = tools / "cmake.exe", tools / "ninja.exe"
        if not cmake.is_file() or not ninja.is_file():
            self.skipTest("no cmake and ninja beside %s" % scope.CTEST)
        with tempfile.TemporaryDirectory() as d:
            root, build = Path(d), Path(d) / "build"
            (root / "python" / "tests").mkdir(parents=True)
            (root / "CMakeLists.txt").write_text(self.LISTS, encoding="utf-8")
            (root / "python" / "tests" / "test_b.py").write_text("", encoding="utf-8")
            configure = subprocess.run([str(cmake), "-S", str(root), "-B", str(build), "-G", "Ninja", "-DCMAKE_MAKE_PROGRAM=%s" % ninja],
                                       capture_output=True, text=True, timeout=120)
            self.assertEqual(configure.returncode, 0, configure.stdout + configure.stderr)
            self.assertEqual([t["name"] for t in scope.inventory(test_dir=build)], ["python_sdk test_b"])
            (root / "python" / "tests" / "test_a.py").write_text("", encoding="utf-8")
            self.assertEqual([t["name"] for t in scope.inventory(test_dir=build)], ["python_sdk test_b"])  # (not yet built)
            calls = []

            def builder(preset):
                calls.append("build")
                return subprocess.run([str(cmake), "--build", str(build)], capture_output=True, timeout=120).returncode

            def discover(preset):
                calls.append("list")
                return scope.inventory(test_dir=build)

            def runner(command, log):
                calls.append(command[command.index("-I") + 1])
                command = [scope.CTEST, "--test-dir", str(build)] + command[command.index("--no-tests=error"):]
                with open(log, "wb") as out:
                    return subprocess.run(command, stdout=out, stderr=subprocess.STDOUT, timeout=120).returncode

            status, report = scope.select(["python/tests/test_a.py"], [], "targeted", 2, run=True, build=True, log_dir=d,
                                          executables={}, dependents={}, readers={}, discover=discover, builder=builder,
                                          runner=runner, measured=lambda preset: {}, recorder=lambda logs, preset: 0)
            self.assertEqual(status, 0, report["errors"])
            self.assertEqual(calls, ["build", "list", "0,0,0,1"])
            self.assertEqual(report["presets"]["ucrt64-headless"]["selected"], ["python_sdk test_a"])


class CommandTest(unittest.TestCase):
    """The real command line, with inventory listing replaced and every process launch refused."""

    def main(self, *argv):
        out = io.StringIO()
        refuse = mock.Mock(side_effect=AssertionError("no process may be started"))
        with mock.patch.object(scope, "inventory", lambda preset=None, test_dir=None: synthetic(["python_sdk test_world"])), \
                mock.patch.object(scope.subprocess, "run", refuse), mock.patch.object(scope.subprocess, "Popen", refuse), \
                contextlib.redirect_stdout(out):
            status = scope.main(list(argv))
        return status, out.getvalue()

    def test_an_unclaimed_python_path_exits_nonzero_with_its_reason(self):
        status, out = self.main("--paths", "python/fsim/no_such_module_xyz.py", "--json")
        self.assertEqual(status, 3)
        report = json.loads(out)
        unclaimed = [e for e in report["errors"] if e["reason"] == "unclaimed"]
        self.assertEqual(unclaimed[0]["paths"], ["python/fsim/no_such_module_xyz.py"])
        self.assertNotIn("exit", report["presets"]["ucrt64-headless"])

    def test_the_text_report_names_the_error_and_its_paths(self):
        status, out = self.main("--paths", "python/fsim/no_such_module_xyz.py", "--run")
        self.assertEqual(status, 3)
        self.assertIn("ERROR unclaimed", out)
        self.assertIn("python/fsim/no_such_module_xyz.py", out)


class DiscoveryTest(unittest.TestCase):
    """Discovery in small private trees."""

    def tree(self, files):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        root = Path(self.tmp.name)
        for name, text in files.items():
            (root / name).parent.mkdir(parents=True, exist_ok=True)
            (root / name).write_text(text, encoding="utf-8")
        return root

    def test_the_python_import_closure(self):
        root = self.tree({
            "python/fsim/__init__.py": "from .world import World\nfrom . import mission\n",
            "python/fsim/world.py": "import os\n",
            "python/fsim/mission.py": "from .world import World\n",
            "python/fsim/vision.py": "",
            "python/tests/test_world.py": "import fsim\n",
            "python/tests/test_mission_training.py": "import fsim.mission\nCRC = 1\n",
            "python/tests/test_mission_concurrency.py": "from test_mission_training import CRC\n",
            "python/tests/test_probe.py": "run([sys.executable, os.path.join(HERE, 'probe_script.py')])\n",
            "python/tests/probe_script.py": "",
            "python/tests/test_vision.py": "from fsim import vision\n",
        })
        d = scope.python_dependents(root)
        self.assertEqual(d["python/fsim/world.py"], {"test_world", "test_mission_training", "test_mission_concurrency", "test_vision"})
        self.assertEqual(d["python/fsim/vision.py"], {"test_vision"})
        self.assertEqual(d["python/tests/test_mission_training.py"], {"test_mission_training", "test_mission_concurrency"})
        self.assertEqual(d["python/tests/probe_script.py"], {"test_probe"})
        self.assertNotIn("python/tests/test_probe.py", d["python/tests/probe_script.py"])

    def test_compile_commands_and_local_headers(self):
        root = self.tree({
            "tests/test_a.cpp": '#include "support.h"\n',
            "tests/support.h": '#include "deeper.h"\n',
            "tests/deeper.h": "",
            "build/b/compile_commands.json": json.dumps([{"file": "tests/test_a.cpp", "command": "c++ -c tests/test_a.cpp",
                                                          "output": "tests/CMakeFiles/fsim_a_test.dir/test_a.cpp.obj"}]),
        })
        cwd = os.getcwd()
        os.chdir(root)  # (the compile command's relative file resolves against it, as CMake's directory would)
        try:
            found = scope.test_executables(root, [root / "build" / "b"])
        finally:
            os.chdir(cwd)
        self.assertEqual(found, {"tests/test_a.cpp": {"fsim_a_test"}, "tests/support.h": {"fsim_a_test"}, "tests/deeper.h": {"fsim_a_test"}})

    def test_excluded_directories_are_never_entered(self):
        root = self.tree({"tests/CMakeLists.txt": "add_executable(fsim_a_test test_a.cpp)\n", "tests/test_a.cpp": "",
                          "tests/.codex/CMakeLists.txt": "add_executable(hidden_test hidden.cpp)\n", "tests/.codex/hidden.cpp": "",
                          "tests/sub/.codex/x/CMakeLists.txt": "add_executable(deep_hidden_test h.cpp)\n"})
        entered, real = [], os.walk

        def recording(top, *a, **k):
            for here, dirs, files in real(top, *a, **k):
                entered.append(Path(here).as_posix())
                yield here, dirs, files
        with mock.patch.object(scope.os, "walk", recording):
            found = scope.test_executables(root, [])
        self.assertEqual(found, {"tests/test_a.cpp": {"fsim_a_test"}})
        self.assertTrue(entered)
        self.assertFalse([d for d in entered if ".codex" in d.split("/")], entered)

    def test_data_readers(self):
        texts = {"tests/test_radar.cpp": 'std::ifstream in(kSource / "tests" / "data" / "track" / "aew_bomber_detections.txt");',
                 "tools/check.py": 'FIXTURES = "tests/fixtures/mission_training/engagement"',
                 "cmake/RunGate.cmake": 'file(COPY_FILE "${_source}/tests/consumer/consumer.py" "${_work}/consumer.py")',
                 "tests/test_other.cpp": "nothing"}
        found = scope.data_readers(["tests/data/track/aew_bomber_detections.txt", "tests/fixtures/mission_training/engagement/a.json",
                                    "tests/consumer/consumer.py", "src/sdk/World.cpp"], list(texts), read=texts.__getitem__)
        self.assertEqual(found, {"tests/data/track/aew_bomber_detections.txt": {"tests/test_radar.cpp"},
                                 "tests/fixtures/mission_training/engagement/a.json": {"tools/check.py"},
                                 "tests/consumer/consumer.py": {"cmake/RunGate.cmake"}})


class TimingTest(unittest.TestCase):
    def test_ctest_output_is_read_with_names_outcomes_and_seconds(self):
        text = ("  3/18 Test   #4: ma_contract ......................   Passed    0.14 sec\n"
                "13/18 Test #613: mission_training .................***Exception: SegFault 55.01 sec\n"
                "12/17 Test #564: python_sdk test_mission_concurrency.PreopenedHandlesTest.test_x ...   Passed  219.30 sec\n"
                " 1/1 Test #9: hangar test_methods.Model3D ...........***Failed  368.62 sec\n")
        self.assertEqual(timing.parse(text), {"ma_contract": (0.14, "Passed"), "mission_training": (55.01, "Exception: SegFault"),
                                              "python_sdk test_mission_concurrency.PreopenedHandlesTest.test_x": (219.3, "Passed"),
                                              "hangar test_methods.Model3D": (368.62, "Failed")})

    def test_a_utf16_log_is_read(self):
        with tempfile.TemporaryDirectory() as d:
            log = Path(d) / "log.txt"
            log.write_bytes(" 1/1 Test #1: a ...   Passed    1.50 sec\n".encode("utf-16"))
            self.assertEqual(timing.parse(timing.read_log(log)), {"a": (1.5, "Passed")})

    def test_a_failure_or_timeout_never_replaces_the_passing_duration(self):
        with tempfile.TemporaryDirectory() as d:
            store, log = Path(d) / "times.json", Path(d) / "log.txt"
            for line in (" 1/1 Test #1: a ...   Passed  200.00 sec\n", " 1/1 Test #1: a ...***Timeout   3.00 sec\n",
                         " 1/1 Test #1: a ...***Failed   1.00 sec\n", " 1/1 Test #1: b ...***Failed   1.00 sec\n"):
                log.write_text(line, encoding="utf-8")
                timing.record([log], "none", store)
            entries = json.loads(store.read_text(encoding="utf-8"))
            self.assertEqual((entries["a"]["seconds"], entries["a"]["outcome"], entries["a"]["last_seconds"]), (200.0, "Failed", 1.0))
            self.assertNotIn("seconds", entries["b"])
            self.assertEqual({k: v for k, v in timing.seconds("none", store).items() if k in ("a", "b")}, {"a": 200.0})


class RulesTest(unittest.TestCase):
    """scopes.toml against the repository and a real build's inventory."""

    @classmethod
    def setUpClass(cls):
        cls.rules = scope.load_rules()
        cls.files = subprocess.run(["git", "ls-files", "--cached", "--others", "--exclude-standard", "--", ".", ":(exclude,glob)**/.codex/**"],
                                   cwd=scope.REPO, capture_output=True, text=True, check=True).stdout.splitlines()
        cls.tests = scope.inventory(test_dir=ARGS.test_dir) if ARGS.test_dir else None

    def test_every_scope_is_well_formed(self):
        keys = {"name", "paths", "tests", "labels", "exclude_labels", "release", "full", "build", "except", "integration", "integration_labels",
                "claims"}
        names = [r["name"] for r in self.rules]
        self.assertEqual(len(names), len(set(names)), "scope names repeat")
        for r in self.rules:
            with self.subTest(scope=r["name"]):
                self.assertLessEqual(set(r), keys)
                self.assertTrue(r.get("paths"))
                self.assertTrue(r.get("tests") or r.get("labels") or r.get("full"), "a scope selects something or escalates")
                for expression in r.get("tests", []):
                    re.compile(expression.replace("{stem}", "x"))

    def test_every_path_pattern_matches_a_tracked_file(self):
        for r in self.rules:
            for pattern in r["paths"] + r.get("except", []):
                with self.subTest(scope=r["name"], pattern=pattern):
                    self.assertTrue(any(scope.glob_match(f, pattern) for f in self.files), "matches no tracked file")

    def test_every_label_and_test_pattern_selects_a_test(self):
        if self.tests is None:
            self.skipTest("no --test-dir: the build's inventory is not checked")
        labels = {lab for t in self.tests for lab in t["labels"]}
        renderer = any("vision" in t["name"] for t in self.tests)
        for r in self.rules:
            for label in r.get("labels", []) + r.get("exclude_labels", []) + r.get("integration_labels", []):
                with self.subTest(scope=r["name"], label=label):
                    self.assertIn(label, labels)
            if r.get("release") and not renderer:
                continue  # (a render scope's patterns name tests a build without the renderer does not have)
            for expression in r.get("tests", []) + r.get("integration", []):
                if "{stem}" in expression:
                    continue
                with self.subTest(scope=r["name"], test=expression):
                    self.assertTrue(any(re.search(expression, t["name"]) for t in self.tests), "selects no test")

    def test_the_repository_python_closure_reaches_the_shared_fixture_and_the_wrappers(self):
        d = scope.python_dependents()
        self.assertIn("test_mission_concurrency", d["python/tests/test_mission_training.py"])
        for module in ("world", "agent", "vecenv", "__init__"):
            with self.subTest(module=module):
                self.assertIn("test_world", d["python/fsim/%s.py" % module])
                self.assertIn("test_mission_concurrency", d["python/fsim/%s.py" % module])


def main():
    global ARGS
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--test-dir", default=None, help="a configured build directory whose CTest inventory the rules are held to")
    ap.add_argument("--ctest", default=None, help="the ctest to list it with (the build's own)")
    ARGS, rest = ap.parse_known_args()
    if ARGS.ctest:
        scope.CTEST = ARGS.ctest
    unittest.main(argv=[sys.argv[0]] + rest, verbosity=2)


if __name__ == "__main__":
    main()
