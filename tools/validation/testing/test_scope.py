"""The change-scoped selector's own checks (tools/validation/testing/scope.py, scopes.toml, timing.py).

    python tools/validation/testing/test_scope.py [--test-dir <a configured build>]

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
    {"index": 8, "name": "sdk_boundary", "labels": [], "command": ["python.exe", str(scope.REPO / "tools" / "validation" / "boundary" / "boundary.py"), "check"]},
    {"index": 9, "name": "flight core", "labels": [], "command": ["D:/b/tests/fsim_core_test.exe"]},
    {"index": 10, "name": "crash_handler", "labels": [], "command": ["cmake.exe", "-DEXE=D:/b/tests/fsim_crash_test.exe", "-P", "crash.cmake"]},
    {"index": 11, "name": "fixture reader", "labels": [], "command": ["D:/b/tests/fsim_reader.exe", str(scope.REPO / "tests" / "training" / "fixtures" / "set")]},
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
EXECUTABLES = {"tests/training/test_ma_joint.cpp": {"fsim_ma_joint_test"}, "tests/training/test_mission_training.cpp": {"fsim_mission_training_test"},
               "tests/simulation/test_crash.cpp": {"fsim_crash_test"}}
DEPENDENTS = {  # (python_dependents of a tree where test_mission_concurrency imports test_mission_training's fixtures)
    "python/fsim/world.py": {"test_world", "test_mission_training", "test_mission_concurrency"},
    "tests/compatibility/python/test_mission_training.py": {"test_mission_training", "test_mission_concurrency"},
    "tests/compatibility/python/test_mission_concurrency.py": {"test_mission_concurrency"},
    "tests/compatibility/python/test_world.py": {"test_world"},
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


class ExternalFixtureTest(unittest.TestCase):
    fixture = "tools/validation/missile/native/OpenVspIntersectionFixture.cmake"
    scope_name = "external OpenVSP intersection fixture"

    def setUp(self):
        self.rules = scope.load_rules()
        self.inventory = [
            {"index": i, "name": name, "labels": [], "command": ["python", name]}
            for i, name in enumerate(("missile_aero_reference_checker", "missile_flight_reference_checker",
                                      "ma_contract", "unrelated"), 1)
        ]

    def test_external_fixture_runs_its_reference_guard_without_an_sdk_build(self):
        for tier in ("targeted", "integration"):
            with self.subTest(tier=tier):
                p = scope.plan([self.fixture], self.rules, self.inventory, {}, tier, preset=scope.HEADLESS)
                self.assertEqual(p["scopes"], [self.scope_name])
                self.assertEqual(p["selected"], ["missile_aero_reference_checker"])
                self.assertTrue(p["headless"])
                self.assertFalse(p["build"])
                self.assertFalse(p["release"])
                self.assertEqual(p["escalate"], [])
                self.assertEqual(p["errors"], [])

    def test_missing_external_guard_blocks_execution_even_with_other_selected_checks(self):
        inventory = [t for t in self.inventory if t["name"] != "missile_aero_reference_checker"]
        for paths in ([self.fixture], [self.fixture, "docs/status.md"]):
            for build_first in (False, True):
                with self.subTest(paths=paths, build_first=build_first), tempfile.TemporaryDirectory() as logs:
                    calls = []
                    status, report = scope.select(
                        paths, self.rules, "targeted", 1, run=True, build=build_first, log_dir=logs,
                        executables={}, dependents={}, readers={}, evidence=None,
                        discover=lambda preset: inventory,
                        builder=lambda preset: calls.append("build") or 0,
                        runner=lambda command, log: calls.append("run") or 0,
                        measured=lambda preset: {}, recorder=lambda logs, preset: 0)
                    self.assertEqual(status, 3, report)
                    self.assertIn("missing-required-tests", [e["reason"] for e in report["errors"]])
                    self.assertEqual(calls, [])

    def test_shared_sdk_cmake_still_escalates_both_presets_beside_an_external_fixture(self):
        shared = "src/simulation/mission_systems/CMakeLists.txt"
        for preset in (scope.HEADLESS, scope.RELEASE):
            with self.subTest(preset=preset):
                p = scope.plan([self.fixture, shared], self.rules, self.inventory, {}, preset=preset)
                self.assertTrue(p["build"])
                self.assertTrue(p["headless"])
                self.assertTrue(p["release"])
                self.assertIn(shared + " (build system)", p["escalate"])
                self.assertEqual(p["selected"], sorted(t["name"] for t in self.inventory))
                self.assertEqual(p["errors"], [])

    def test_external_exception_claims_only_the_known_fixture(self):
        def claims(path):
            return [r["name"] for r in self.rules
                    if any(scope.glob_match(path, pattern) for pattern in r["paths"])
                    and not any(scope.glob_match(path, pattern) for pattern in r.get("except", []))]
        self.assertIn(self.scope_name, claims(self.fixture))
        for path in ("tools/validation/missile/native/UnreviewedFixture.cmake",
                     "tools/validation/missile/native/vsp_triangle_boundary.cpp",
                     "src/simulation/mission_systems/CMakeLists.txt"):
            with self.subTest(path=path):
                self.assertNotIn(self.scope_name, claims(path))


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
        self.assertEqual(selected(["tests/training/test_ma_joint.cpp"])["selected"], ["ma_joint"])
        self.assertEqual(selected(["tests/training/test_mission_training.cpp"])["selected"], ["mission_training", "mission_training_stress batch"])
        self.assertEqual(selected(["tests/simulation/test_crash.cpp"])["selected"], ["crash_handler"])  # (run by a CMake script: -DEXE=)

    def test_a_python_test_module_selects_its_tests(self):
        self.assertEqual(selected(["tests/compatibility/python/test_world.py"])["selected"], ["python_sdk test_world"])
        self.assertEqual(selected(["tests/compatibility/python/test_mission_concurrency.py"])["selected"], ["python_sdk test_mission_concurrency.A.test_x"])
        self.assertEqual(selected(["tools/models/hangar/tests/test_methods.py"])["selected"], ["hangar test_methods.Model3D"])

    def test_a_file_a_test_runs_selects_that_test(self):
        self.assertEqual(selected(["tools/validation/boundary/boundary.py"])["selected"], ["sdk_boundary"])

    def test_a_file_in_a_fixture_directory_a_test_is_given_selects_that_test(self):
        self.assertEqual(selected(["tests/training/fixtures/set/case.json"])["selected"], ["fixture reader"])

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

    def test_an_unregistered_python_test_module_fails_though_sdk_boundary_is_selected(self):
        """A test module added since the last configure: the package scope still selects sdk_boundary, which exists and
        would pass - the selection must fail instead, naming the module, not run without it."""
        dependents = dict(DEPENDENTS, **{"tests/compatibility/python/test_new.py": {"test_new"}})
        rules = RULES + [{"name": "package tests", "paths": ["tests/compatibility/python/**"], "tests": ["^sdk_boundary$"], "claims": False}]  # (as scopes.toml)
        p = scope.plan(["tests/compatibility/python/test_new.py"], rules, TESTS, EXECUTABLES, "targeted", dependents)
        self.assertEqual(p["selected"], ["sdk_boundary"])
        self.assertNotIn("unclaimed", reasons(p))
        self.assertEqual(reasons(p)["unregistered"]["modules"], ["test_new"])
        self.assertEqual(reasons(p)["unregistered"]["paths"], ["tests/compatibility/python/test_new.py"])
        for tier in ("integration", "full"):
            with self.subTest(tier=tier):
                p = scope.plan(["tests/compatibility/python/test_new.py"], rules, TESTS, EXECUTABLES, tier, dependents)
                self.assertEqual(reasons(p)["unregistered"]["modules"], ["test_new"])

    def test_a_shared_fixture_an_unregistered_module_imports_fails_too(self):
        dependents = dict(DEPENDENTS, **{"tests/compatibility/python/test_mission_training.py": {"test_mission_training", "test_mission_concurrency", "test_new"}})
        p = scope.plan(["tests/compatibility/python/test_mission_training.py"], RULES, TESTS, EXECUTABLES, "targeted", dependents)
        self.assertEqual(reasons(p)["unregistered"]["modules"], ["test_new"])
        self.assertEqual(reasons(p)["unregistered"]["paths"], ["tests/compatibility/python/test_mission_training.py"])

    def test_a_deleted_test_module_is_not_required(self):
        p = selected(["tests/compatibility/python/test_gone.py"])  # (no longer in the import closure: the file is gone)
        self.assertNotIn("unregistered", reasons(p))

    def test_a_shared_fixture_module_selects_the_stress_cases_importing_it(self):
        p = selected(["tests/compatibility/python/test_mission_training.py"])
        self.assertEqual(p["selected"], ["python_sdk test_mission_concurrency.A.test_x", "python_sdk test_mission_training"])

    def test_a_data_file_selects_the_tests_of_the_sources_reading_it(self):
        readers = {"tests/simulation/data/track/crosscheck.json": {"tests/simulation/test_crash.cpp"}}
        self.assertEqual(selected(["tests/simulation/data/track/crosscheck.json"], readers=readers)["selected"], ["crash_handler"])

    def test_the_legacy_environment_escalates_and_the_mission_episode_owner_does_not(self):
        rules = scope.load_rules()
        legacy = scope.plan(["src/training/native/vecenv/VecEnv.cpp"], rules, TESTS, EXECUTABLES, "targeted",
                            DEPENDENTS)
        self.assertTrue(legacy["escalate"])
        self.assertIn("vecenv", legacy["selected"])
        owner = scope.plan(["src/training/native/episodes/runtime/MissionEpisode.cpp"], rules, TESTS, EXECUTABLES,
                           "targeted", DEPENDENTS)
        self.assertFalse(owner["escalate"])
        self.assertIn("MA episode owner (native)", owner["scopes"])
        self.assertNotIn("vecenv", owner["selected"])

    # HC-13: a change built only with the renderer is planned on ucrt64-release, never passed on the headless build alone
    RENDER_TESTS = [
        {"index": 1, "name": "sdk_boundary", "labels": [], "command": ["python.exe"]},
        {"index": 2, "name": "terrain profile: the camera", "labels": ["render"], "command": ["D:/r/tests/fsim_render_tests.exe"]},
        {"index": 3, "name": "vision", "labels": ["render"], "command": ["D:/r/tests/fsim_vision_test.exe"]},
        {"index": 4, "name": "flight core", "labels": [], "command": ["D:/r/tests/fsim_core_test.exe"]},
    ]
    RENDER_ROOTS = ["src/visualization/scene/VehicleVisuals.cpp", "src/visualization/ui/MonitorGui.cpp",
                    "src/visualization/vfx/flow/FlowSolver.cpp", "src/visualization/vfx/effects/Exhaust.h",
                    "src/visualization/scene/CMakeLists.txt", "src/visualization/vfx/flow/CMakeLists.txt",
                    "src/visualization/vision/CameraScene.cpp", "src/adapters/vision/Vision.cpp",
                    "src/applications/viewer/main.cpp", "src/applications/viewer/CMakeLists.txt", "src/adapters/vision/python/_vision.c",
                    "tools/models/rcs/Engine.cpp", "tools/validation/flow_calibrate/main.cpp", "examples/visualization/vision_capture/main.cpp",
                    "tests/visualization/test_terrain_render.cpp", "tests/visualization/test_vision_c_abi.c"]

    def render_plan(self, paths, evidence=None):
        return scope.plan(paths, scope.load_rules(), self.RENDER_TESTS, {}, "targeted", {}, {}, evidence)

    def test_every_render_only_root_plans_the_release_preset_alone(self):
        for path in self.RENDER_ROOTS:
            with self.subTest(path=path):
                p = self.render_plan([path])
                self.assertTrue(p["release"], "the release preset is planned")
                self.assertFalse(p["headless"], "the headless build does not compile it")
                self.assertTrue(p["build"])
                self.assertEqual(p["errors"], [])
                self.assertEqual(p["escalate"], [])
                self.assertLessEqual({"terrain profile: the camera", "vision"}, set(p["selected"]))

    def test_the_flow_module_is_also_a_shared_build_input(self):
        p = self.render_plan(["cmake/Flow.cmake"])  # (every configure reads it; only the renderer's build uses it)
        self.assertTrue(p["release"] and p["headless"] and p["escalate"])

    def test_controls_a_headless_change_and_a_mixed_one(self):
        p = self.render_plan(["src/simulation/flight/SimRunner.h"])
        self.assertTrue(p["headless"])
        self.assertEqual(p["release"], [])
        p = self.render_plan(["src/visualization/ui/MonitorGui.cpp", "src/simulation/flight/SimRunner.h"])
        self.assertTrue(p["headless"] and p["release"])

    def test_the_compile_databases_plan_the_release_preset_whatever_claims_a_path(self):
        evidence = {"sources": {"src/visualization/x/New.cpp", "tools/newtool/main.cpp"}, "directories": {"src/visualization/x"}}
        rules = [{"name": "tool", "paths": ["tools/newtool/**"], "tests": ["^flight core$"]}]
        # an unclaimed source compiled only with the renderer: the release preset alone, escalated there
        p = scope.plan(["src/visualization/x/New.cpp"], [], self.RENDER_TESTS, {}, "targeted", {}, {}, evidence)
        self.assertTrue(p["release"] and p["escalate"])
        self.assertFalse(p["headless"])
        # a header beside such sources is built with them
        p = scope.plan(["src/visualization/x/New.h"], [], self.RENDER_TESTS, {}, "targeted", {}, {}, evidence)
        self.assertTrue(p["release"])
        self.assertFalse(p["headless"])
        # a scope that forgot the renderer: still planned on ucrt64-release, and on ucrt64-headless as that scope says
        p = scope.plan(["tools/newtool/main.cpp"], rules, self.RENDER_TESTS, {}, "targeted", {}, {}, evidence)
        self.assertTrue(p["release"] and p["headless"])
        self.assertIn("compiled only in ucrt64-release", " ".join(p["release"]))
        # without the databases, the rules decide (an unclaimed native path escalates, as before)
        p = scope.plan(["src/visualization/x/New.cpp"], [], self.RENDER_TESTS, {}, "targeted", {}, {}, None)
        self.assertTrue(p["headless"] and p["escalate"])

    def test_the_release_plan_requires_and_selects_every_renderer_test(self):
        generic = [t for t in self.RENDER_TESTS if "render" not in t["labels"]]
        generic += [{"index": 5, "name": "python_sdk test_vision", "labels": [], "command": ["python.exe"]},
                    {"index": 6, "name": "python_sdk test_camera", "labels": [], "command": ["python.exe"]}]
        rules = scope.load_rules()
        for tier in ("targeted", "integration", "full"):
            with self.subTest(tier=tier):
                p = scope.plan(["src/visualization/ui/MonitorGui.cpp"], rules, generic, {}, tier, {}, {}, None, scope.RELEASE)
                self.assertEqual([(e["reason"], e["paths"]) for e in p["errors"]],
                                 [("no-renderer-coverage", ["src/visualization/ui/MonitorGui.cpp"])])
                p = scope.plan(["src/visualization/ui/MonitorGui.cpp"], rules, self.RENDER_TESTS, {}, tier, {}, {}, None, scope.RELEASE)
                self.assertEqual(p["errors"], [])
                self.assertLessEqual({"terrain profile: the camera", "vision"}, set(p["selected"]))
        # a shared renderer scope (headless kept) requires them as well; a generic Release scope does not
        for path in ("assets/clouds/shaders/bruneton/runtime.glsl", "include/fsim/fsim_vision_c.h", "python/fsim/vision.py"):
            with self.subTest(path=path):
                p = scope.plan([path], rules, generic, {}, "targeted", {}, {}, None, scope.RELEASE)
                self.assertTrue(p["headless"] and p["release"])
                self.assertEqual(p["renderer"], [path])
                self.assertEqual([e["reason"] for e in p["errors"]], ["no-renderer-coverage"])
                p = scope.plan([path], rules, generic, {}, "targeted", {}, {}, None, scope.HEADLESS)
                self.assertEqual(p["errors"], [])
                self.assertIn("sdk_boundary", p["selected"])
        p = scope.plan(["include/fsim/MissionBatch.h"], rules, generic, {}, "targeted", {}, {}, None, scope.RELEASE)
        self.assertTrue(p["release"])
        self.assertEqual(p["renderer"], [])
        self.assertTrue(scope.renders({"release": True, "labels": ["render"]}))
        self.assertFalse(scope.renders({"labels": ["render"]}) or scope.renders({"release": True, "tests": ["vision"]}))
        # the headless plan of a mixed change does not ask its inventory for renderer tests
        p = scope.plan(["src/visualization/ui/MonitorGui.cpp", "src/simulation/flight/SimRunner.h"], rules, generic, {},
                       "targeted", {}, {}, None, scope.HEADLESS)
        self.assertNotIn("no-renderer-coverage", [e["reason"] for e in p["errors"]])

    def test_release_only_reads_both_compile_databases(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp)
            for preset, files in ((scope.HEADLESS, ["src/a/A.cpp", "tests/t.cpp"]),
                                  (scope.RELEASE, ["src/a/A.cpp", "tests/t.cpp", "tests/r.cpp", "src/v/V.cpp"])):
                (repo / "build" / preset).mkdir(parents=True)
                (repo / "build" / preset / "compile_commands.json").write_text(
                    json.dumps([{"file": str(repo / f), "directory": str(repo)} for f in files]), encoding="utf-8")
            evidence = scope.release_only(repo)
            self.assertEqual(evidence["sources"], {"tests/r.cpp", "src/v/V.cpp"})
            self.assertEqual(evidence["directories"], {"src/v"})  # (tests/ also holds a headless source)
            self.assertTrue(scope.is_release_only("src/v/CMakeLists.txt", evidence))
            self.assertFalse(scope.is_release_only("tests/CMakeLists.txt", evidence))
            (repo / "build" / scope.RELEASE / "compile_commands.json").unlink()
            self.assertIsNone(scope.release_only(repo))

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

    def unregistered(self, build):
        """A new test module whose CTest test only the build registers; sdk_boundary is there all along."""
        self.inventories = {"before": synthetic(["sdk_boundary", "python_sdk test_world"]),
                            "after": synthetic(["sdk_boundary", "python_sdk test_new", "python_sdk test_world"])}
        rules = [{"name": "package", "paths": ["tests/compatibility/python/**"], "tests": ["^sdk_boundary$"], "claims": False}]
        return scope.select(["tests/compatibility/python/test_new.py"], rules, "targeted", 16, run=True, build=build, log_dir=self.tmp.name,
                            executables={}, dependents={"tests/compatibility/python/test_new.py": {"test_new"}}, readers={},
                            discover=self.discover, builder=self.builder,
                            runner=self.runner(("sdk_boundary", "python_sdk test_new")),
                            measured=lambda preset: {}, recorder=lambda logs, preset: 0)

    def test_run_without_build_fails_on_an_unregistered_module_and_runs_nothing(self):
        status, report = self.unregistered(build=False)
        self.assertEqual(status, 3)
        self.assertEqual(self.calls, ["list ucrt64-headless"])
        error = next(e for e in report["errors"] if e["reason"] == "unregistered")
        self.assertEqual((error["modules"], error["paths"], error["preset"]), (["test_new"], ["tests/compatibility/python/test_new.py"], "ucrt64-headless"))
        self.assertEqual(report["presets"]["ucrt64-headless"]["selected"], ["sdk_boundary"])  # (what would have run alone)

    def test_run_with_build_registers_and_runs_the_module(self):
        status, report = self.unregistered(build=True)
        self.assertEqual(status, 0, report["errors"])
        self.assertEqual(self.calls, ["build ucrt64-headless", "list ucrt64-headless", "run -I 0,0,0,1,2"])
        self.assertEqual(report["presets"]["ucrt64-headless"]["selected"], ["sdk_boundary", "python_sdk test_new"])


class RenderLifecycleTest(unittest.TestCase):
    """HC-13: select on the real rules, its effects replaced - a renderer change builds, lists and runs ucrt64-release, and a
    missing release preset or failed release build stops it rather than passing on ucrt64-headless."""

    def setUp(self):
        self.calls = []
        self.tmp = tempfile.TemporaryDirectory()
        self.inventories = {
            scope.HEADLESS: synthetic(["sdk_boundary", "flight core"]),
            scope.RELEASE: synthetic(["sdk_boundary", "flight core"]) + [
                {"index": 3, "name": "terrain profile: the camera", "labels": ["render"], "command": ["x.exe"], "serial": False, "processors": 1},
                {"index": 4, "name": "vision", "labels": ["render"], "command": ["x.exe"], "serial": False, "processors": 1}],
        }
        self.build_status = {scope.HEADLESS: 0, scope.RELEASE: 0}

    def tearDown(self):
        self.tmp.cleanup()

    def builder(self, preset):
        self.calls.append("build " + preset)
        return self.build_status[preset]

    def discover(self, preset):
        self.calls.append("list " + preset)
        return self.inventories.get(preset)

    def select(self, paths, ran):
        def run(command, log):
            self.calls.append("run " + command[2] + " " + command[-1])
            Path(log).write_text("".join(" %d/%d Test #%d: %s ....   Passed    1.00 sec\n" % (k + 1, len(ran[command[2]]), k + 1, n)
                                         for k, n in enumerate(ran[command[2]])), encoding="utf-8")
            return 0
        return scope.select(list(paths), scope.load_rules(), "targeted", 16, run=True, build=True, log_dir=self.tmp.name,
                            executables={}, dependents={}, readers={}, discover=self.discover, builder=self.builder,
                            runner=run, measured=lambda preset: {}, recorder=lambda logs, preset: 0, evidence=None)

    RENDER_RUN = ("sdk_boundary", "terrain profile: the camera", "vision")

    def test_scene_ui_and_flow_changes_build_and_run_the_release_preset_only(self):
        for path in ("src/visualization/scene/VehicleVisuals.cpp", "src/visualization/ui/MonitorGui.cpp",
                     "src/visualization/vfx/flow/FlowSolver.cpp"):
            with self.subTest(path=path):
                self.calls = []
                status, report = self.select([path], {scope.RELEASE: self.RENDER_RUN})
                self.assertEqual(status, 0, report["errors"])
                self.assertEqual(list(report["presets"]), [scope.RELEASE])
                self.assertEqual(self.calls, ["build ucrt64-release", "list ucrt64-release", "run ucrt64-release 0,0,0,1,3,4"])

    def test_without_a_release_inventory_a_renderer_change_fails_and_runs_nothing(self):
        del self.inventories[scope.RELEASE]
        status, report = self.select(["src/visualization/scene/VehicleVisuals.cpp"], {})
        self.assertEqual(status, 3)
        self.assertEqual([e["reason"] for e in report["errors"]], ["no-inventory"])
        self.assertNotIn("run", " ".join(self.calls))

    def test_a_failing_release_build_stops_before_listing_or_running(self):
        self.build_status[scope.RELEASE] = 2
        status, report = self.select(["src/visualization/ui/MonitorGui.cpp"], {})
        self.assertEqual(status, 2)
        self.assertEqual(self.calls, ["build ucrt64-release"])

    # A stale or renderer-disabled ucrt64-release inventory: it keeps the generic matches a render scope also names
    # (sdk_boundary, the Python Vision and camera tests) but has no renderer test. They must not stand in for it.
    GENERIC = ["sdk_boundary", "python_sdk test_vision", "python_sdk test_camera", "flight core"]

    def plan_only(self, paths, tier="targeted", evidence=None, rules=None):
        return scope.select(list(paths), rules or scope.load_rules(), tier, 16, run=False, executables={}, dependents={},
                            readers={}, discover=self.discover, measured=lambda preset: {}, evidence=evidence)

    def test_a_release_inventory_without_renderer_tests_fails_every_tier(self):
        self.inventories[scope.RELEASE] = synthetic(self.GENERIC)
        for tier in ("targeted", "integration", "full"):
            with self.subTest(tier=tier):
                status, report = self.plan_only(["src/visualization/ui/MonitorGui.cpp"], tier)
                self.assertEqual(status, 3)
                errors = [e for e in report["errors"] if e["reason"] == "no-renderer-coverage"]
                self.assertEqual([(e["preset"], e["paths"]) for e in errors],
                                 [(scope.RELEASE, ["src/visualization/ui/MonitorGui.cpp"])])

    def test_a_mixed_escalated_change_fails_without_renderer_tests(self):
        self.inventories[scope.RELEASE] = synthetic(self.GENERIC)
        status, report = self.plan_only(["src/visualization/ui/MonitorGui.cpp", "src/simulation/flight/SimRunner.h"])
        self.assertEqual(status, 3)
        self.assertEqual([e["reason"] for e in report["errors"]], ["no-renderer-coverage"])
        self.assertEqual(report["errors"][0]["paths"], ["src/visualization/ui/MonitorGui.cpp"])

    def test_the_compile_databases_require_renderer_tests_too(self):
        evidence = {"sources": {"tools/newtool/main.cpp"}, "directories": set()}
        rules = [{"name": "tool", "paths": ["tools/newtool/**"], "tests": ["^sdk_boundary$"]}]
        self.inventories[scope.RELEASE] = synthetic(self.GENERIC)
        status, report = self.plan_only(["tools/newtool/main.cpp"], evidence=evidence, rules=rules)
        self.assertEqual(status, 3)
        self.assertIn("no-renderer-coverage", [e["reason"] for e in report["errors"]])
        # with renderer tests in the inventory, a scope that names only generic tests still gets every one of them
        self.inventories[scope.RELEASE] = synthetic(self.GENERIC) + [
            {"index": 9, "name": "terrain profile: the camera", "labels": ["render"], "command": ["x.exe"], "serial": False, "processors": 1}]
        status, report = self.plan_only(["tools/newtool/main.cpp"], evidence=evidence, rules=rules)
        self.assertEqual(status, 0, report["errors"])
        self.assertIn("terrain profile: the camera", report["presets"][scope.RELEASE]["selected"])

    def test_without_renderer_tests_no_run_starts_with_or_without_a_build(self):
        self.inventories[scope.RELEASE] = synthetic(self.GENERIC)
        for build in (False, True):
            with self.subTest(build=build):
                self.calls = []
                status, report = scope.select(["src/visualization/scene/VehicleVisuals.cpp"], scope.load_rules(), "targeted", 16,
                                              run=True, build=build, log_dir=self.tmp.name, executables={}, dependents={},
                                              readers={}, discover=self.discover, builder=self.builder,
                                              runner=lambda command, log: self.calls.append("run") or 0,
                                              measured=lambda preset: {}, recorder=lambda logs, preset: 0, evidence=None)
                self.assertEqual(status, 3)
                self.assertEqual([e["reason"] for e in report["errors"]], ["no-renderer-coverage"])
                self.assertEqual(self.calls, (["build ucrt64-release"] if build else []) + ["list ucrt64-release"])

    def test_the_post_build_inventory_is_the_one_checked(self):
        # before the build the inventory lacks the renderer tests; the build (a reconfigure) registers them
        rendered = self.inventories[scope.RELEASE]
        self.inventories[scope.RELEASE] = synthetic(self.GENERIC)

        def builder(preset):
            self.calls.append("build " + preset)
            self.inventories[scope.RELEASE] = rendered
            return 0
        self.builder = builder
        status, report = self.select(["src/visualization/vfx/flow/FlowSolver.cpp"], {scope.RELEASE: self.RENDER_RUN})
        self.assertEqual(status, 0, report["errors"])
        self.assertEqual(self.calls[:2], ["build ucrt64-release", "list ucrt64-release"])

    # The shared renderer scope: paths both presets build or carry (the public Vision header, its Python wrapper, shader
    # and other assets). Headless keeps its coverage; on ucrt64-release they need renderer tests all the same.
    SHARED = ["assets/clouds/shaders/bruneton/runtime.glsl", "include/fsim/fsim_vision_c.h", "python/fsim/vision.py"]

    def test_a_shared_renderer_path_fails_every_tier_without_renderer_tests_and_keeps_headless(self):
        self.inventories = {scope.HEADLESS: synthetic(self.GENERIC), scope.RELEASE: synthetic(self.GENERIC)}
        for path in self.SHARED:
            for tier in ("targeted", "integration", "full"):
                with self.subTest(path=path, tier=tier):
                    status, report = self.plan_only([path], tier)
                    self.assertEqual(status, 3)
                    self.assertEqual([(e["reason"], e["preset"], e["paths"]) for e in report["errors"]],
                                     [("no-renderer-coverage", scope.RELEASE, [path])])
                    self.assertEqual(list(report["presets"]), [scope.HEADLESS, scope.RELEASE])
                    self.assertIn("sdk_boundary", report["presets"][scope.HEADLESS]["selected"])

    def test_a_mixed_escalated_shared_change_fails_without_renderer_tests(self):
        self.inventories = {scope.HEADLESS: synthetic(self.GENERIC), scope.RELEASE: synthetic(self.GENERIC)}
        for path in self.SHARED:
            with self.subTest(path=path):
                status, report = self.plan_only([path, "src/simulation/flight/SimRunner.h"])
                self.assertEqual(status, 3)
                self.assertEqual([(e["reason"], e["paths"]) for e in report["errors"]], [("no-renderer-coverage", [path])])
                self.assertEqual(len(report["presets"][scope.HEADLESS]["selected"]), len(self.GENERIC))  # (escalated)

    def test_a_shared_renderer_path_runs_nothing_without_renderer_tests_with_or_without_a_build(self):
        self.inventories = {scope.HEADLESS: synthetic(self.GENERIC), scope.RELEASE: synthetic(self.GENERIC)}
        for build in (False, True):
            with self.subTest(build=build):
                self.calls = []
                status, report = scope.select(["assets/clouds/shaders/bruneton/runtime.glsl"], scope.load_rules(), "targeted",
                                              16, run=True, build=build, log_dir=self.tmp.name, executables={}, dependents={},
                                              readers={}, discover=self.discover, builder=self.builder,
                                              runner=lambda command, log: self.calls.append("run") or 0,
                                              measured=lambda preset: {}, recorder=lambda logs, preset: 0, evidence=None)
                self.assertEqual(status, 3)
                self.assertEqual([e["reason"] for e in report["errors"]], ["no-renderer-coverage"])
                self.assertNotIn("run", self.calls)
                self.assertEqual([c for c in self.calls if c.startswith("build")],
                                 ["build " + scope.HEADLESS, "build " + scope.RELEASE] if build else [])

    def test_a_shared_renderer_path_on_a_valid_inventory_runs_both_presets(self):
        self.inventories[scope.HEADLESS] = synthetic(self.GENERIC)
        self.inventories[scope.RELEASE] = synthetic(self.GENERIC) + [
            {"index": 9, "name": "terrain profile: the camera", "labels": ["render"], "command": ["x.exe"], "serial": False, "processors": 1}]
        for path in self.SHARED:
            for tier in ("targeted", "integration", "full"):
                with self.subTest(path=path, tier=tier):
                    status, report = self.plan_only([path], tier)
                    self.assertEqual(status, 0, report["errors"])
                    self.assertIn("terrain profile: the camera", report["presets"][scope.RELEASE]["selected"])
                    self.assertIn("sdk_boundary", report["presets"][scope.HEADLESS]["selected"])
        # the post-build inventory decides: a build that registers the renderer tests passes the run
        rendered = self.inventories[scope.RELEASE]
        self.inventories[scope.RELEASE] = synthetic(self.GENERIC)

        def builder(preset):
            self.calls.append("build " + preset)
            if preset == scope.RELEASE:
                self.inventories[scope.RELEASE] = rendered
            return 0
        self.builder, self.calls = builder, []
        status, report = self.select(["include/fsim/fsim_vision_c.h"],
                                     {scope.HEADLESS: ("sdk_boundary", "python_sdk test_vision", "python_sdk test_camera"),
                                      scope.RELEASE: ("sdk_boundary", "python_sdk test_vision", "python_sdk test_camera",
                                                      "terrain profile: the camera")})
        self.assertEqual(status, 0, report["errors"])
        self.assertEqual(sum(c.startswith("run ") for c in self.calls), 2)

    def test_a_generic_release_scope_needs_no_renderer_tests(self):
        self.inventories = {scope.HEADLESS: synthetic(self.GENERIC), scope.RELEASE: synthetic(self.GENERIC)}
        for path in ("include/fsim/MissionBatch.h", "docs/sdk/boundary.toml"):  # (MA public surface; SDK principals)
            with self.subTest(path=path):
                status, report = self.plan_only([path])
                self.assertEqual(status, 0, report["errors"])
                self.assertEqual(list(report["presets"]), [scope.HEADLESS, scope.RELEASE])

    def test_a_valid_release_inventory_selects_every_renderer_test_in_every_tier(self):
        for tier in ("targeted", "integration", "full"):
            with self.subTest(tier=tier):
                status, report = self.plan_only(["src/visualization/ui/MonitorGui.cpp"], tier)
                self.assertEqual(status, 0, report["errors"])
                self.assertLessEqual({"terrain profile: the camera", "vision"}, set(report["presets"][scope.RELEASE]["selected"]))

    def test_a_mixed_change_plans_both_presets_and_a_headless_one_only_its_own(self):
        # HC-19: the shared-core header escalates the headless suite it plans; ucrt64-release runs the renderer change's
        # own selection (every renderer test with it), not its full suite
        status, report = self.select(["src/visualization/ui/MonitorGui.cpp", "src/simulation/flight/SimRunner.h"],
                                     {scope.HEADLESS: ("sdk_boundary", "flight core"),
                                      scope.RELEASE: ("sdk_boundary", "terrain profile: the camera", "vision")})
        self.assertEqual(status, 0, report["errors"])
        self.assertEqual(list(report["presets"]), [scope.HEADLESS, scope.RELEASE])
        self.assertNotIn("flight core", report["presets"][scope.RELEASE]["selected"])
        self.calls = []
        status, report = self.select(["src/simulation/flight/SimRunner.h"], {scope.HEADLESS: ("sdk_boundary", "flight core")})
        self.assertEqual(status, 0, report["errors"])
        self.assertEqual(list(report["presets"]), [scope.HEADLESS])


class PresetEscalationTest(unittest.TestCase):
    """HC-19: an escalation applies to the presets its own path plans, so a shared-core change beside a renderer change
    runs the headless suite and ucrt64-release's renderer selection, not both suites. HC-20: shared build inputs plan and
    escalate both presets. The real rules over synthetic inventories; plans only, nothing runs."""

    HEADLESS = synthetic(["sdk_boundary", "flight core", "vecenv", "testing_scope_checker"])
    RELEASE = synthetic(["sdk_boundary", "flight core", "vecenv", "testing_scope_checker"]) + [
        {"index": 5, "name": "terrain profile: the camera", "labels": ["render"], "command": ["x.exe"], "serial": False, "processors": 1},
        {"index": 6, "name": "vision", "labels": ["render"], "command": ["x.exe"], "serial": False, "processors": 1},
        {"index": 7, "name": "viewer_screenshot", "labels": ["render"], "command": ["x.exe"], "serial": False, "processors": 1}]
    RENDER = {"sdk_boundary", "terrain profile: the camera", "vision", "viewer_screenshot"}
    BUILD_INPUTS = ["CMakeLists.txt", "CMakePresets.json", "cmake/IncludeViews.cmake", "deps/CMakePresets.json",
                    "third_party/jsbsim/CMakeLists.txt", "src/CMakeLists.txt", "src/simulation/CMakeLists.txt",
                    "tests/CMakeLists.txt", "python/CMakeLists.txt"]

    def plan(self, paths, evidence=None):
        inventories = {scope.HEADLESS: self.HEADLESS, scope.RELEASE: self.RELEASE}
        return scope.select(list(paths), scope.load_rules(), "targeted", 16, run=False, executables={}, dependents={},
                            readers={}, discover=inventories.get, measured=lambda preset: {}, evidence=evidence)

    def names(self, inventory):
        return {t["name"] for t in inventory}

    def test_a_shared_core_change_beside_a_renderer_change_escalates_only_the_headless_suite(self):
        for core in ("src/simulation/runtime/world/World.cpp", "src/simulation/flight/SimRunner.h", "include/fsim/World.h"):
            with self.subTest(core=core):
                status, report = self.plan([core, "src/visualization/ui/MonitorGui.cpp"])
                self.assertEqual(status, 0, report["errors"])
                headless, release = report["presets"][scope.HEADLESS], report["presets"][scope.RELEASE]
                self.assertEqual(set(headless["selected"]), self.names(self.HEADLESS))
                self.assertEqual([r.split(" (")[0] for r in headless["plan"]["escalate"]], [core])
                self.assertEqual(release["plan"]["escalate"], [])
                self.assertEqual(set(release["selected"]), self.RENDER)  # (every renderer test, required coverage kept)

    def test_the_shared_core_change_alone_plans_the_headless_suite_only(self):
        status, report = self.plan(["src/simulation/runtime/world/World.cpp"])
        self.assertEqual(status, 0, report["errors"])
        self.assertEqual(list(report["presets"]), [scope.HEADLESS])
        self.assertEqual(set(report["presets"][scope.HEADLESS]["selected"]), self.names(self.HEADLESS))

    def test_shared_build_inputs_plan_and_escalate_both_presets(self):
        for path in self.BUILD_INPUTS:
            with self.subTest(path=path):
                status, report = self.plan([path])
                self.assertEqual(status, 0, report["errors"])
                self.assertEqual(list(report["presets"]), [scope.HEADLESS, scope.RELEASE])
                for preset, inventory in ((scope.HEADLESS, self.HEADLESS), (scope.RELEASE, self.RELEASE)):
                    self.assertEqual(set(report["presets"][preset]["selected"]), self.names(inventory), preset)
                    self.assertEqual(report["presets"][preset]["plan"]["escalate"], ["%s (build system)" % path])

    # HC-39: the module, example and tool CMake declarations both configures read, at their real depths
    NESTED = ["src/foundation/core/CMakeLists.txt", "src/foundation/platform/CMakeLists.txt", "src/simulation/flight/CMakeLists.txt",
              "src/simulation/communications/dms/CMakeLists.txt", "src/adapters/sdk/CMakeLists.txt",
              "src/adapters/sdk/python/CMakeLists.txt", "src/training/native/CMakeLists.txt", "src/applications/headless/CMakeLists.txt",
              "src/contracts/CMakeLists.txt", "examples/CMakeLists.txt", "examples/training/ppo_trainer/CMakeLists.txt",
              "tools/CMakeLists.txt", "tools/data/tile_prefetch/CMakeLists.txt", "tools/models/hangar/native/CMakeLists.txt"]
    RENDER_ONLY_CMAKE = ["src/visualization/ui/CMakeLists.txt", "src/visualization/vfx/flow/CMakeLists.txt",
                         "src/adapters/vision/CMakeLists.txt", "src/adapters/vision/python/CMakeLists.txt",
                         "src/applications/viewer/CMakeLists.txt", "examples/visualization/vision_capture/CMakeLists.txt",
                         "tools/models/rcs/CMakeLists.txt", "tools/validation/flow_calibrate/CMakeLists.txt"]

    def test_nested_shared_cmake_inputs_plan_and_escalate_both_presets(self):
        for path in self.NESTED:
            with self.subTest(path=path):
                status, report = self.plan([path])
                self.assertEqual(status, 0, report["errors"])
                self.assertEqual(list(report["presets"]), [scope.HEADLESS, scope.RELEASE])
                for preset, inventory in ((scope.HEADLESS, self.HEADLESS), (scope.RELEASE, self.RELEASE)):
                    self.assertEqual(set(report["presets"][preset]["selected"]), self.names(inventory), preset)
                    self.assertIn("%s (build system)" % path, report["presets"][preset]["plan"]["escalate"])

    def test_a_nested_shared_cmake_input_beside_a_renderer_change_escalates_both_suites(self):
        # (the review's probe: core's CMake file plus a UI source planned Release 68 of 780 with no escalation)
        status, report = self.plan(["src/foundation/core/CMakeLists.txt", "src/visualization/ui/MonitorGui.cpp"])
        self.assertEqual(status, 0, report["errors"])
        self.assertEqual(set(report["presets"][scope.RELEASE]["selected"]), self.names(self.RELEASE))
        self.assertEqual(set(report["presets"][scope.HEADLESS]["selected"]), self.names(self.HEADLESS))

    def test_render_only_cmake_inputs_keep_their_release_routing(self):
        for path in self.RENDER_ONLY_CMAKE:
            with self.subTest(path=path):
                status, report = self.plan([path])
                self.assertEqual(status, 0, report["errors"])
                self.assertEqual(list(report["presets"]), [scope.RELEASE])
                self.assertEqual(report["presets"][scope.RELEASE]["plan"]["escalate"], [])
                self.assertEqual(set(report["presets"][scope.RELEASE]["selected"]), self.RENDER)

    def test_a_build_input_beside_a_renderer_change_escalates_both_suites(self):
        status, report = self.plan(["CMakeLists.txt", "src/visualization/ui/MonitorGui.cpp"])
        self.assertEqual(status, 0, report["errors"])
        self.assertEqual(set(report["presets"][scope.RELEASE]["selected"]), self.names(self.RELEASE))
        self.assertEqual(set(report["presets"][scope.HEADLESS]["selected"]), self.names(self.HEADLESS))

    def test_a_build_script_a_test_scope_claims_is_neither_escalated_nor_planned_on_release(self):
        status, report = self.plan(["cmake/RunPackageConsumer.cmake"])  # (the installed SDK package scope)
        self.assertEqual(status, 3)  # (that scope's `package` label selects nothing in these inventories)
        self.assertEqual(list(report["presets"]), [scope.HEADLESS])
        self.assertEqual(report["presets"][scope.HEADLESS]["plan"]["escalate"], [])

    def test_an_unclaimed_renderer_source_escalates_release_and_leaves_the_headless_selection_alone(self):
        evidence = {"sources": {"tools/newtool/main.cpp"}, "directories": {"tools/newtool"}}
        status, report = self.plan(["tools/newtool/main.cpp", "tools/validation/testing/scope.py"], evidence)
        self.assertEqual(status, 0, report["errors"])  # (escalated on its own preset: claimed there, not 'unclaimed' here)
        headless, release = report["presets"][scope.HEADLESS], report["presets"][scope.RELEASE]
        self.assertEqual(headless["selected"], ["testing_scope_checker"])
        self.assertEqual(headless["plan"]["escalate"], [])
        self.assertEqual(release["plan"]["escalate"], ["tools/newtool/main.cpp (no scope claims it)"])
        self.assertEqual(set(release["selected"]), self.names(self.RELEASE))

    def test_the_full_tier_still_takes_every_planned_suite(self):
        inventories = {scope.HEADLESS: self.HEADLESS, scope.RELEASE: self.RELEASE}
        status, report = scope.select(["src/simulation/runtime/world/World.cpp", "src/visualization/ui/MonitorGui.cpp"],
                                      scope.load_rules(), "full", 16, run=False, executables={}, dependents={}, readers={},
                                      discover=inventories.get, measured=lambda preset: {}, evidence=None)
        self.assertEqual(status, 0, report["errors"])
        self.assertEqual(set(report["presets"][scope.RELEASE]["selected"]), self.names(self.RELEASE))

    def test_the_report_names_each_presets_own_escalations(self):
        status, report = self.plan(["src/simulation/runtime/world/World.cpp", "src/visualization/ui/MonitorGui.cpp"])
        text = scope.show(report)
        self.assertIn("escalates ucrt64-headless to the full suite: src/simulation/runtime/world/World.cpp", text)
        self.assertNotIn("escalates ucrt64-release", text)


class ArchitectureGatesRequiredTest(unittest.TestCase):
    """HC-40: the architecture checker's scope requires its gates - the configured graph and its checker among them - on
    each preset it plans. A nonempty architecture label is no evidence that they ran: a missing registration fails
    before any build is listed again or any runner starts."""

    PATHS = ["tools/architecture/configured.py", "tools/architecture/check.py", "cmake/ArchitectureChecks.cmake"]
    GATES = ["architecture_boundary", "architecture_checker", "architecture_configured", "architecture_configured_checker"]
    OTHER = ["sdk_boundary", "include_view_core"]

    def setUp(self):
        self.calls = []
        self.tmp = tempfile.TemporaryDirectory()
        self.inventories = {preset: self.inventory(self.GATES) for preset in (scope.HEADLESS, scope.RELEASE)}

    def tearDown(self):
        self.tmp.cleanup()

    def inventory(self, gates):
        tests = synthetic(self.OTHER + list(gates))
        for t in tests:
            t["labels"] = ["architecture"]
        return tests

    def discover(self, preset):
        self.calls.append("list " + preset)
        return self.inventories.get(preset)

    def plan_only(self, paths):
        return scope.select(list(paths), scope.load_rules(), "targeted", 16, run=False, executables={}, dependents={}, readers={},
                            discover=self.discover, measured=lambda preset: {}, evidence=None)

    def run_select(self, paths, builder):
        def run(command, log):
            preset = command[2]
            names = [t["name"] for t in self.inventories[preset] if str(t["index"]) in command[-1].split(",")[3:]] \
                if "-I" in command else [t["name"] for t in self.inventories[preset]]
            self.calls.append("run " + preset)
            Path(log).write_text("".join(" %d/%d Test #%d: %s ....   Passed    1.00 sec\n" % (k + 1, len(names), k + 1, n)
                                         for k, n in enumerate(names)), encoding="utf-8")
            return 0
        return scope.select(list(paths), scope.load_rules(), "targeted", 16, run=True, build=True, log_dir=self.tmp.name,
                            executables={}, dependents={}, readers={}, discover=self.discover, builder=builder,
                            runner=run, measured=lambda preset: {}, recorder=lambda logs, preset: 0, evidence=None)

    def test_a_valid_inventory_selects_every_gate_on_both_presets(self):
        for path in self.PATHS:
            with self.subTest(path=path):
                status, report = self.plan_only([path])
                self.assertEqual(status, 0, report["errors"])
                self.assertEqual(list(report["presets"]), [scope.HEADLESS, scope.RELEASE])
                for preset in (scope.HEADLESS, scope.RELEASE):
                    self.assertLessEqual(set(self.GATES), set(report["presets"][preset]["selected"]))

    def test_an_inventory_with_only_the_legacy_gate_fails_on_each_preset(self):
        # (the review's probe: each inventory holds architecture_boundary alone)
        self.inventories = {preset: self.inventory(["architecture_boundary"]) for preset in (scope.HEADLESS, scope.RELEASE)}
        status, report = self.plan_only(["tools/architecture/configured.py"])
        self.assertEqual(status, 3)
        errors = [e for e in report["errors"] if e["reason"] == "missing-required-tests"]
        self.assertEqual([(e["preset"], e["scope"]) for e in errors],
                         [(scope.HEADLESS, "architecture checker"), (scope.RELEASE, "architecture checker")])
        for e in errors:
            self.assertEqual(e["missing"], ["^architecture_checker$", "^architecture_configured$", "^architecture_configured_checker$"])

    def test_both_configured_gates_missing_fails(self):
        self.inventories[scope.RELEASE] = self.inventory(["architecture_boundary", "architecture_checker"])
        status, report = self.plan_only(["tools/architecture/configured.py"])
        self.assertEqual(status, 3)
        self.assertEqual([(e["preset"], e["missing"]) for e in report["errors"]],
                         [(scope.RELEASE, ["^architecture_configured$", "^architecture_configured_checker$"])])

    def test_one_missing_gate_is_enough_to_fail_on_either_preset(self):
        for preset in (scope.HEADLESS, scope.RELEASE):
            for missing in self.GATES:
                with self.subTest(preset=preset, missing=missing):
                    self.inventories = {p: self.inventory(self.GATES) for p in (scope.HEADLESS, scope.RELEASE)}
                    self.inventories[preset] = self.inventory([g for g in self.GATES if g != missing])
                    status, report = self.plan_only(["tools/architecture/configured.py"])
                    self.assertEqual(status, 3)
                    self.assertEqual([(e["preset"], e["missing"]) for e in report["errors"]], [(preset, ["^%s$" % missing])])

    def test_a_missing_gate_after_the_build_fails_before_any_runner_starts(self):
        self.inventories[scope.RELEASE] = self.inventory(["architecture_boundary", "architecture_checker"])

        def builder(preset):
            self.calls.append("build " + preset)
            return 0
        status, report = self.run_select(["tools/architecture/configured.py"], builder)
        self.assertEqual(status, 3)
        self.assertEqual([e["reason"] for e in report["errors"]], ["missing-required-tests"])
        self.assertFalse([c for c in self.calls if c.startswith("run")])

    def test_the_post_build_inventory_is_the_one_checked(self):
        # before the build the configured gates are unregistered; the build (a reconfigure) registers them
        self.inventories = {preset: self.inventory(["architecture_boundary", "architecture_checker"])
                            for preset in (scope.HEADLESS, scope.RELEASE)}

        def builder(preset):
            self.calls.append("build " + preset)
            self.inventories[preset] = self.inventory(self.GATES)
            return 0
        status, report = self.run_select(["tools/architecture/configured.py"], builder)
        self.assertEqual(status, 0, report["errors"])
        self.assertEqual([c for c in self.calls if c.startswith("run")], ["run " + scope.HEADLESS, "run " + scope.RELEASE])

    def test_a_documentation_change_does_not_require_the_gates(self):
        self.inventories = {preset: self.inventory(["architecture_boundary"]) + synthetic(["ma_contract", "sdk_guard_checker"], 20)
                            for preset in (scope.HEADLESS, scope.RELEASE)}
        status, report = self.plan_only(["docs/architecture/README.md"])
        self.assertEqual(status, 0, report["errors"])


class RequiredTestsTest(unittest.TestCase):
    """HC-31: a scope's required tests (`requires`) on the real rules, the demo publisher's scope: each planned preset
    must have every one - in every tier, before anything is built, listed again or run - and a selection of the scope's
    other tests is no stand-in for them."""

    PATHS = ["src/applications/demo/main.cpp", "src/contracts/world_stream/DemoControl.h", "src/applications/demo/DemoSteps.h",
             "tests/integration/test_demo_steps.cpp"]
    RELEASE_TESTS = ["demo_publisher", "demo_publisher_steps", "demo_publisher_lifecycle", "demo_publisher_schedules",
                     "include_view_demo", "contracts"]
    HEADLESS_TESTS = ["demo_publisher_schedules", "include_view_demo", "contracts"]
    OTHER = ["sdk_boundary", "flight core"]

    def setUp(self):
        self.calls = []
        self.tmp = tempfile.TemporaryDirectory()
        self.inventories = {scope.HEADLESS: synthetic(self.OTHER + self.HEADLESS_TESTS),
                            scope.RELEASE: synthetic(self.OTHER + self.RELEASE_TESTS)}

    def tearDown(self):
        self.tmp.cleanup()

    def discover(self, preset):
        self.calls.append("list " + preset)
        return self.inventories.get(preset)

    def builder(self, preset):
        self.calls.append("build " + preset)
        return 0

    def plan_only(self, paths, tier="targeted"):
        return scope.select(list(paths), scope.load_rules(), tier, 16, run=False, executables={}, dependents={}, readers={},
                            discover=self.discover, measured=lambda preset: {}, evidence=None)

    def run_select(self, paths, build):
        def run(command, log):
            preset = command[2]
            names = [t["name"] for t in self.inventories[preset] if str(t["index"]) in command[-1].split(",")[3:]]
            self.calls.append("run " + preset)
            Path(log).write_text("".join(" %d/%d Test #%d: %s ....   Passed    1.00 sec\n" % (k + 1, len(names), k + 1, n)
                                         for k, n in enumerate(names)), encoding="utf-8")
            return 0
        return scope.select(list(paths), scope.load_rules(), "targeted", 16, run=True, build=build, log_dir=self.tmp.name,
                            executables={}, dependents={}, readers={}, discover=self.discover, builder=self.builder,
                            runner=run, measured=lambda preset: {}, recorder=lambda logs, preset: 0, evidence=None)

    def test_a_valid_inventory_selects_every_required_test_on_both_presets_in_every_tier(self):
        for path in self.PATHS:
            for tier in ("targeted", "integration", "full"):
                with self.subTest(path=path, tier=tier):
                    status, report = self.plan_only([path], tier)
                    self.assertEqual(status, 0, report["errors"])
                    self.assertEqual(list(report["presets"]), [scope.HEADLESS, scope.RELEASE])
                    self.assertLessEqual(set(self.RELEASE_TESTS), set(report["presets"][scope.RELEASE]["selected"]))
                    self.assertLessEqual(set(self.HEADLESS_TESTS), set(report["presets"][scope.HEADLESS]["selected"]))

    def test_a_release_inventory_without_the_publisher_tests_fails_every_tier(self):
        # the review's probe: what the scope's own patterns still find there (the include probe, the contract test) is a
        # non-empty selection, and no stand-in
        self.inventories[scope.RELEASE] = synthetic(self.OTHER + self.HEADLESS_TESTS)
        for path in self.PATHS:
            for tier in ("targeted", "integration", "full"):
                with self.subTest(path=path, tier=tier):
                    status, report = self.plan_only([path], tier)
                    self.assertEqual(status, 3)
                    errors = [e for e in report["errors"] if e["reason"] == "missing-required-tests"]
                    self.assertEqual([(e["preset"], e["scope"]) for e in errors], [(scope.RELEASE, "demo publisher")])
                    self.assertEqual(errors[0]["missing"], ["^demo_publisher$", "^demo_publisher_steps$", "^demo_publisher_lifecycle$"])

    def test_one_missing_test_is_enough_to_fail(self):
        for missing in self.RELEASE_TESTS:
            with self.subTest(missing=missing):
                self.inventories[scope.RELEASE] = synthetic(self.OTHER + [n for n in self.RELEASE_TESTS if n != missing])
                status, report = self.plan_only(["src/applications/demo/main.cpp"])
                self.assertEqual(status, 3)
                self.assertEqual([e["missing"] for e in report["errors"]], [["^%s$" % missing]])

    def test_headless_keeps_its_required_coverage(self):
        self.inventories[scope.HEADLESS] = synthetic(self.OTHER + ["contracts"])
        status, report = self.plan_only(["src/contracts/world_stream/DemoControl.h"])
        self.assertEqual(status, 3)
        self.assertEqual([(e["preset"], e["missing"]) for e in report["errors"]],
                         [(scope.HEADLESS, ["^demo_publisher_schedules$", "^include_view_demo$"])])

    def test_without_the_required_tests_nothing_runs_with_or_without_a_build(self):
        self.inventories[scope.RELEASE] = synthetic(self.OTHER + self.HEADLESS_TESTS)
        for build in (False, True):
            with self.subTest(build=build):
                self.calls = []
                status, report = self.run_select(["src/applications/demo/main.cpp"], build)
                self.assertEqual(status, 3)
                self.assertEqual({e["reason"] for e in report["errors"]}, {"missing-required-tests"})
                self.assertFalse([c for c in self.calls if c.startswith("run")])
                self.assertEqual([c for c in self.calls if c.startswith("build")],
                                 ["build " + scope.HEADLESS, "build " + scope.RELEASE] if build else [])

    def test_a_build_that_registers_the_tests_runs_them_on_both_presets(self):
        complete = self.inventories[scope.RELEASE]
        self.inventories[scope.RELEASE] = synthetic(self.OTHER + self.HEADLESS_TESTS)

        def builder(preset):
            self.calls.append("build " + preset)
            if preset == scope.RELEASE:
                self.inventories[scope.RELEASE] = complete  # (the reconfigure registers them)
            return 0
        self.builder = builder
        status, report = self.run_select(["src/applications/demo/main.cpp"], True)
        self.assertEqual(status, 0, report["errors"])
        self.assertEqual([c for c in self.calls if c.startswith("run")], ["run " + scope.HEADLESS, "run " + scope.RELEASE])
        self.assertLessEqual(set(self.RELEASE_TESTS), set(report["presets"][scope.RELEASE]["selected"]))
        # and without that build the run stops: the inventory listed is the one checked
        self.inventories[scope.RELEASE] = synthetic(self.OTHER + self.HEADLESS_TESTS)
        self.calls = []
        status, report = self.run_select(["src/applications/demo/main.cpp"], False)
        self.assertEqual(status, 3)
        self.assertFalse([c for c in self.calls if c.startswith("run")])


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

            status, report = scope.select(["tests/compatibility/python/test_a.py"], [], "targeted", 2, run=True, build=True, log_dir=d,
                                          executables={}, dependents={"tests/compatibility/python/test_a.py": {"test_a"}}, readers={},
                                          discover=discover, builder=builder,
                                          runner=runner, measured=lambda preset: {}, recorder=lambda logs, preset: 0)
            self.assertEqual(status, 0, report["errors"])
            self.assertEqual(calls, ["build", "list", "0,0,0,1"])
            self.assertEqual(report["presets"]["ucrt64-headless"]["selected"], ["python_sdk test_a"])


class CommandTest(unittest.TestCase):
    """The real command line, with inventory listing replaced and every process launch refused."""

    def main(self, *argv, inventory=("python_sdk test_world",), dependents=None):
        out = io.StringIO()
        refuse = mock.Mock(side_effect=AssertionError("no process may be started"))
        closure = scope.python_dependents()
        closure.update(dependents or {})
        with mock.patch.object(scope, "inventory", lambda preset=None, test_dir=None: synthetic(list(inventory))), \
                mock.patch.object(scope, "python_dependents", lambda repo=None: closure), \
                mock.patch.object(scope.subprocess, "run", refuse), mock.patch.object(scope.subprocess, "Popen", refuse), \
                contextlib.redirect_stdout(out):
            status = scope.main(list(argv))
        return status, out.getvalue()

    def test_run_without_build_refuses_an_unregistered_new_test_module(self):
        """A new module in the tree (the closure holds it) and an inventory with sdk_boundary but no test for it: the real
        command, --run without --build (and the plan alone), exits 3 naming the module and starts no process."""
        new = "tests/compatibility/python/test_brand_new_xyz.py"
        for argv in (("--run",), ("--run", "--json"), ()):
            with self.subTest(argv=argv):
                status, out = self.main("--paths", new, *argv, inventory=("sdk_boundary", "python_sdk test_world"),
                                        dependents={new: {"test_brand_new_xyz"}})
                self.assertEqual(status, 3)
                if "--json" in argv:
                    error = next(e for e in json.loads(out)["errors"] if e["reason"] == "unregistered")
                    self.assertEqual((error["modules"], error["paths"]), (["test_brand_new_xyz"], [new]))
                    self.assertEqual(json.loads(out)["presets"]["ucrt64-headless"]["selected"], ["sdk_boundary"])
                else:
                    self.assertIn("ERROR unregistered", out)
                    self.assertIn("test_brand_new_xyz", out)

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
            "tests/compatibility/python/test_world.py": "import fsim\n",
            "tests/compatibility/python/test_mission_training.py": "import fsim.mission\nCRC = 1\n",
            "tests/compatibility/python/test_mission_concurrency.py": "from test_mission_training import CRC\n",
            "tests/compatibility/python/test_probe.py": "run([sys.executable, os.path.join(HERE, 'probe_script.py')])\n",
            "tests/compatibility/python/probe_script.py": "",
            "tests/compatibility/python/test_vision.py": "from fsim import vision\n",
        })
        d = scope.python_dependents(root)
        self.assertEqual(d["python/fsim/world.py"], {"test_world", "test_mission_training", "test_mission_concurrency", "test_vision"})
        self.assertEqual(d["python/fsim/vision.py"], {"test_vision"})
        self.assertEqual(d["tests/compatibility/python/test_mission_training.py"], {"test_mission_training", "test_mission_concurrency"})
        self.assertEqual(d["tests/compatibility/python/probe_script.py"], {"test_probe"})
        self.assertNotIn("tests/compatibility/python/test_probe.py", d["tests/compatibility/python/probe_script.py"])

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
        texts = {"tests/simulation/test_radar.cpp": 'std::ifstream in(kSource / "tests" / "simulation" / "data" / "track" / "aew_bomber_detections.txt");',
                 "tools/check.py": 'FIXTURES = "tests/training/fixtures/mission_training/engagement"',
                 "cmake/RunGate.cmake": 'file(COPY_FILE "${_source}/tests/consumer/consumer.py" "${_work}/consumer.py")',
                 "tests/test_other.cpp": "nothing"}
        found = scope.data_readers(["tests/simulation/data/track/aew_bomber_detections.txt", "tests/training/fixtures/mission_training/engagement/a.json",
                                    "tests/consumer/consumer.py", "src/sdk/World.cpp"], list(texts), read=texts.__getitem__)
        self.assertEqual(found, {"tests/simulation/data/track/aew_bomber_detections.txt": {"tests/simulation/test_radar.cpp"},
                                 "tests/training/fixtures/mission_training/engagement/a.json": {"tools/check.py"},
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
        cls.files = subprocess.run(["git", "ls-files", "--cached", "--others", "--exclude-standard", "--exclude=.codex", "--", ".",
                                    ":(exclude,glob)**/.codex/**"],
                                   cwd=scope.REPO, capture_output=True, text=True, check=True).stdout.splitlines()
        cls.tests = scope.inventory(test_dir=ARGS.test_dir) if ARGS.test_dir else None

    def test_every_scope_is_well_formed(self):
        keys = {"name", "paths", "tests", "labels", "exclude_labels", "release", "full", "build", "except", "integration", "integration_labels",
                "claims", "render_only", "requires"}
        names = [r["name"] for r in self.rules]
        self.assertEqual(len(names), len(set(names)), "scope names repeat")
        for r in self.rules:
            with self.subTest(scope=r["name"]):
                self.assertLessEqual(set(r), keys)
                self.assertTrue(r.get("paths"))
                self.assertTrue(r.get("tests") or r.get("labels") or r.get("full"), "a scope selects something or escalates")
                if r.get("render_only"):
                    self.assertTrue(r.get("release") and not r.get("full"), "a render-only scope runs on ucrt64-release, targeted")
                    self.assertTrue(scope.renders(r), "a render-only scope requires the renderer tests")
                if scope.RENDER in r.get("labels", []):
                    self.assertTrue(scope.renders(r), "a scope selecting the render label runs on ucrt64-release")
                for expression in r.get("tests", []):
                    re.compile(expression.replace("{stem}", "x"))
                for preset, expressions in r.get("requires", {}).items():
                    self.assertIn(preset, (scope.HEADLESS, scope.RELEASE))
                    self.assertTrue(expressions, "a requirement names tests")
                    for expression in expressions:
                        re.compile(expression)
                    if preset == scope.RELEASE:
                        self.assertTrue(r.get("release"), "a scope requiring ucrt64-release tests plans ucrt64-release")
                    else:
                        self.assertFalse(r.get("render_only"), "a render-only scope does not plan ucrt64-headless")

    def test_every_tracked_cmake_input_is_routed(self):
        """HC-39: a CMake declaration both configures read plans and escalates both presets; one only the renderer's
        branch configures plans ucrt64-release alone; test and package scripts and consumer projects belong to their own
        scopes, and a test's own script is named by the test that runs it. The exact external OpenVSP fixture is not an
        SDK build input and has its own required offline guard. Other CMake inputs cannot fall through to an ordinary
        headless-only reference scope."""
        inputs = [f for f in self.files if f.endswith(("CMakeLists.txt", ".cmake")) or f.endswith("CMakePresets.json")]
        self.assertGreater(len(inputs), 50)
        registrations = (scope.REPO / "tests" / "CMakeLists.txt").read_text(encoding="utf-8")
        for path in inputs:
            with self.subTest(path=path):
                p = scope.plan([path], self.rules, [], {}, "targeted", {}, {}, None)
                if any("(build system)" in reason for reason in p["escalate"]):
                    self.assertTrue(p["headless"] and p["release"], "a shared build input plans both presets")
                elif "render-only roots" in p["scopes"]:
                    self.assertFalse(p["headless"])
                    self.assertTrue(p["release"])
                elif set(p["scopes"]) & {"installed SDK package", "MA installed consumers", "architecture checker"}:
                    self.assertEqual(p["escalate"], [])
                elif "external OpenVSP intersection fixture" in p["scopes"]:
                    self.assertEqual(path, ExternalFixtureTest.fixture)
                    self.assertEqual(p["scopes"], [ExternalFixtureTest.scope_name])
                    self.assertFalse(p["build"])
                    self.assertFalse(p["release"])
                    self.assertEqual(p["escalate"], [])
                else:
                    self.assertTrue(path.startswith("tests/") and path in registrations,
                                    "a CMake input outside the build system, the render-only roots and the script scopes")

    def test_every_path_pattern_matches_a_tracked_file(self):
        for r in self.rules:
            for pattern in r["paths"] + r.get("except", []):
                with self.subTest(scope=r["name"], pattern=pattern):
                    self.assertTrue(any(scope.glob_match(f, pattern) for f in self.files), "matches no tracked file")

    def test_every_label_and_test_pattern_selects_a_test(self):
        if self.tests is None:
            self.skipTest("no --test-dir: the build's inventory is not checked")
        labels = {lab for t in self.tests for lab in t["labels"]}
        renderer = any(t["name"] == "vision" for t in self.tests)  # (the native Vision test: only a renderer build has it)
        for r in self.rules:
            if r.get("release") and not renderer:
                continue  # (a render scope's labels and patterns name tests a build without the renderer does not have)
            for label in r.get("labels", []) + r.get("exclude_labels", []) + r.get("integration_labels", []):
                with self.subTest(scope=r["name"], label=label):
                    self.assertIn(label, labels)
            for expression in r.get("tests", []) + r.get("integration", []):
                if "{stem}" in expression:
                    continue
                with self.subTest(scope=r["name"], test=expression):
                    self.assertTrue(any(re.search(expression, t["name"]) for t in self.tests), "selects no test")

    def test_what_only_the_renderer_builds_is_claimed_by_a_render_only_scope(self):
        """HC-13 against the real builds: every source ucrt64-release compiles and ucrt64-headless does not - and every
        tracked file in a directory holding only such sources (headers, nested CMake files) - is claimed by a render-only
        scope; and no source ucrt64-headless compiles is claimed by render-only scopes alone (it would lose its
        headless selection)."""
        evidence = scope.release_only()
        if evidence is None:
            self.skipTest("both presets' compile databases are needed (configure ucrt64-headless and ucrt64-release)")
        render = [r for r in self.rules if r.get("render_only")]
        claims = lambda path, rules: [r for r in rules if any(scope.glob_match(path, p) for p in r["paths"])
                                      and not any(scope.glob_match(path, p) for p in r.get("except", []))]
        tracked = [f for f in self.files if scope.is_release_only(f, evidence)]
        self.assertTrue(evidence["sources"], "the release build compiles renderer sources")
        for path in sorted(set(tracked) | evidence["sources"]):
            with self.subTest(path=path):
                self.assertTrue(claims(path, render), "built only with the renderer, claimed by no render-only scope")
        headless = {f for f in self.files if f.endswith((".c", ".cc", ".cpp")) and not scope.is_release_only(f, evidence)}
        for path in sorted(headless):
            hits = claims(path, self.rules)
            if hits and all(r.get("render_only") for r in hits):
                with self.subTest(path=path):
                    self.fail("a source outside the renderer's builds is claimed only by render-only scopes")

    def test_every_test_only_the_renderer_builds_carries_the_render_label(self):
        """The render-only scope selects by the `render` label: a test ucrt64-release has and ucrt64-headless does not must
        carry it, or a renderer change would not select it."""
        inventories = {p: scope.inventory(test_dir=scope.REPO / "build" / p) if (scope.REPO / "build" / p / "CTestTestfile.cmake").is_file()
                       else None for p in (scope.HEADLESS, scope.RELEASE)}
        if any(v is None for v in inventories.values()):
            self.skipTest("both presets' configured inventories are needed")
        headless = {t["name"] for t in inventories[scope.HEADLESS]}
        only = [t for t in inventories[scope.RELEASE] if t["name"] not in headless]
        self.assertTrue(only, "the release preset has renderer tests")
        for t in only:
            with self.subTest(test=t["name"]):
                self.assertIn("render", t["labels"])
        for t in inventories[scope.HEADLESS]:
            with self.subTest(test=t["name"]):
                self.assertNotIn("render", t["labels"], "the render label marks only what the renderer builds")

    def test_the_real_inventories_plan_a_renderer_change_on_the_release_preset(self):
        """The real rules on both presets' real inventories, planned only (nothing built or run): a scene, UI and Flow
        change each select render-labelled tests on ucrt64-release and plan no headless run."""
        builds = {p: scope.REPO / "build" / p for p in (scope.HEADLESS, scope.RELEASE)}
        inventories = {p: scope.inventory(test_dir=b) if (b / "CTestTestfile.cmake").is_file() else None for p, b in builds.items()}
        if any(v is None for v in inventories.values()) or not any("render" in t["labels"] for t in inventories[scope.RELEASE]):
            self.skipTest("both presets' configured inventories, the release one with its render-labelled tests, are needed")
        for path in ("src/visualization/ui/MonitorGui.cpp", "src/visualization/scene/VehicleVisuals.cpp",
                     "src/visualization/vfx/flow/FlowSolver.cpp"):
            with self.subTest(path=path):
                status, report = scope.select([path], self.rules, "targeted", 16, run=False, executables={}, dependents={},
                                              readers={}, discover=lambda preset: inventories[preset])
                self.assertEqual(status, 0, report["errors"])
                self.assertEqual(list(report["presets"]), [scope.RELEASE])
                chosen = report["presets"][scope.RELEASE]["selected"]
                rendered = {t["name"] for t in inventories[scope.RELEASE] if "render" in t["labels"]}
                self.assertLessEqual(rendered, set(chosen))
                self.assertLess(len(chosen), report["presets"][scope.RELEASE]["of"], "a targeted selection, not the suite")
        for path in ("assets/clouds/shaders/bruneton/runtime.glsl", "include/fsim/fsim_vision_c.h", "python/fsim/vision.py"):
            with self.subTest(path=path):  # the shared renderer scope: headless kept, every render test on ucrt64-release
                status, report = scope.select([path], self.rules, "targeted", 16, run=False, executables={}, dependents={},
                                              readers={}, discover=lambda preset: inventories[preset])
                self.assertEqual(status, 0, report["errors"])
                self.assertEqual(list(report["presets"]), [scope.HEADLESS, scope.RELEASE])
                self.assertIn("sdk_boundary", report["presets"][scope.HEADLESS]["selected"])
                self.assertLessEqual(rendered, set(report["presets"][scope.RELEASE]["selected"]))

    def test_the_real_headless_inventory_cannot_stand_in_for_the_release_one(self):
        """The review's probe: the real headless inventory (sdk_boundary and the Python Vision tests, no renderer test)
        given as the ucrt64-release one. Every tier, a mixed escalated change and a run, with or without a build, fail
        before any runner starts."""
        build = scope.REPO / "build" / scope.HEADLESS
        if not (build / "CTestTestfile.cmake").is_file():
            self.skipTest("the headless preset's configured inventory is needed")
        headless = scope.inventory(test_dir=build)
        self.assertFalse(any("render" in t["labels"] for t in headless))
        discover = lambda preset: headless  # noqa: E731
        cases = [(["src/visualization/ui/MonitorGui.cpp"], tier, False, False) for tier in ("targeted", "integration", "full")]
        cases += [(["src/visualization/ui/MonitorGui.cpp", "src/simulation/flight/SimRunner.h"], "targeted", False, False),
                  (["src/visualization/scene/VehicleVisuals.cpp"], "targeted", True, False),
                  (["src/visualization/vfx/flow/FlowSolver.cpp"], "targeted", True, True)]
        # the shared renderer scope (both presets planned): shader, public Vision header and Python wrapper
        shared = ["assets/clouds/shaders/bruneton/runtime.glsl", "include/fsim/fsim_vision_c.h", "python/fsim/vision.py"]
        cases += [([path], tier, False, False) for path in shared for tier in ("targeted", "integration", "full")]
        cases += [([shared[0], "src/simulation/flight/SimRunner.h"], "targeted", False, False),
                  ([shared[0]], "targeted", True, False), ([shared[0]], "targeted", True, True)]
        for paths, tier, run, build_first in cases:
            with self.subTest(paths=paths, tier=tier, run=run, build=build_first):
                calls = []
                with tempfile.TemporaryDirectory() as logs:
                    status, report = scope.select(paths, self.rules, tier, 16, run=run, build=build_first, log_dir=logs,
                                                  executables={}, dependents={}, readers={}, discover=discover,
                                                  builder=lambda preset: calls.append("build") or 0,
                                                  runner=lambda command, log: calls.append("run") or 0,
                                                  measured=lambda preset: {}, recorder=lambda logs, preset: 0)
                self.assertEqual(status, 3)
                self.assertIn("no-renderer-coverage", [e["reason"] for e in report["errors"]])
                self.assertNotIn("run", calls)

    def test_every_required_test_is_in_its_presets_real_inventory(self):
        """HC-31: each scope's required tests are registered in that preset's configured build."""
        found = 0
        for r in self.rules:
            for preset, expressions in r.get("requires", {}).items():
                build = scope.REPO / "build" / preset
                if not (build / "CTestTestfile.cmake").is_file():
                    continue
                names = [t["name"] for t in scope.inventory(test_dir=build)]
                for expression in expressions:
                    with self.subTest(scope=r["name"], preset=preset, test=expression):
                        self.assertTrue(any(re.search(expression, n) for n in names), "not in the inventory")
                        found += 1
        if not found:
            self.skipTest("no configured preset holds a required test")

    def test_the_real_headless_inventory_cannot_stand_in_for_the_release_one_for_the_demo_publisher(self):
        """HC-31, the review's probe: the real headless inventory given as the ucrt64-release one. A demo application or
        control-block change fails in every tier, and a run with or without a build starts no runner; the real inventories
        select every required test."""
        builds = {p: scope.REPO / "build" / p for p in (scope.HEADLESS, scope.RELEASE)}
        inventories = {p: scope.inventory(test_dir=b) if (b / "CTestTestfile.cmake").is_file() else None for p, b in builds.items()}
        if inventories[scope.HEADLESS] is None:
            self.skipTest("the headless preset's configured inventory is needed")
        paths = (["src/applications/demo/main.cpp"], ["src/contracts/world_stream/DemoControl.h"])
        for changed in paths:
            for tier, run, build_first in [(tier, False, False) for tier in ("targeted", "integration", "full")] + \
                    [("targeted", True, False), ("targeted", True, True)]:
                with self.subTest(paths=changed, tier=tier, run=run, build=build_first):
                    calls = []
                    with tempfile.TemporaryDirectory() as logs:
                        status, report = scope.select(changed, self.rules, tier, 16, run=run, build=build_first, log_dir=logs,
                                                      executables={}, dependents={}, readers={},
                                                      discover=lambda preset: inventories[scope.HEADLESS],
                                                      builder=lambda preset: calls.append("build") or 0,
                                                      runner=lambda command, log: calls.append("run") or 0,
                                                      measured=lambda preset: {}, recorder=lambda logs, preset: 0, evidence=None)
                    self.assertEqual(status, 3)
                    self.assertEqual([(e["reason"], e["preset"]) for e in report["errors"]],
                                     [("missing-required-tests", scope.RELEASE)])
                    self.assertNotIn("run", calls)
        if inventories[scope.RELEASE] is None:
            return
        required = [x for r in self.rules if r["name"] == "demo publisher" for x in r["requires"][scope.RELEASE]]
        for changed in paths:
            with self.subTest(paths=changed, inventory="real"):
                status, report = scope.select(changed, self.rules, "targeted", 16, run=False, executables={}, dependents={},
                                              readers={}, discover=lambda preset: inventories[preset], evidence=None)
                self.assertEqual(status, 0, report["errors"])
                chosen = report["presets"][scope.RELEASE]["selected"]
                for expression in required:
                    self.assertTrue(any(re.search(expression, n) for n in chosen), expression)

    def test_the_repository_python_closure_reaches_the_shared_fixture_and_the_wrappers(self):
        d = scope.python_dependents()
        self.assertIn("test_mission_concurrency", d["tests/compatibility/python/test_mission_training.py"])
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
