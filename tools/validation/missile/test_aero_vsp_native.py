"""Optional native-runner guards; every external process is mocked."""
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest import mock

_SPEC = importlib.util.spec_from_file_location("missile_aero_vsp_native", Path(__file__).with_name("aero_vsp_native.py"))
native = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(native)


def report():
    cases = []
    for name, level, unused in (("level0-identity", 0, False), ("level1-identity", 1, False),
                                ("level1-unused-nonidentity", 1, True), ("level2-unused-nonidentity", 2, True)):
        cases.append({"name": name, "level": level, "available_levels": 2,
                      "input_vertices": 5 if unused else 4, "reported_surface_count": 4,
                      "actual_surface_nodes": 4, "unused_vertex": unused, "nonidentity": unused,
                      "surfaces_unchanged": True, "other_fine_nodes_unchanged": True,
                      "roots_attached": True, "repeated_update_idempotent": True,
                      "all_coordinates_finite": True, "passed": True,
                      "deflect_wake_matches_fine": True, "undeflect_wake_matches_fine": True,
                      "deflect_fine_nodes_unchanged": True, "undeflect_fine_nodes_unchanged": True,
                      "zero_control_surfaces": 0, "checked_wake_nodes_per_path": 8,
                      "attachments": [{"te": 2, "wake": 5, "fine_te": 3 if unused else 2, "fine_wake": 6 if unused else 5},
                                      {"te": 3, "wake": 9, "fine_te": 4 if unused else 3, "fine_wake": 10 if unused else 9}]})
    return {"scope": "Public-library geometry attachment indexing regression; no flow",
            "all_passed": True, "model_accepted": False, "cases": cases}


class OptionalNativeRunner(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.compiler = self.base / "g++.exe"
        self.compiler.write_bytes(b"mock explicit compiler")
        self.include = self.base / "Solver"
        self.include.mkdir()
        self.source = self.include / "VSP_Geom.C"
        self.source.write_bytes(b"pinned source witness")
        (self.include / "VSP_Geom.H").write_bytes(b"public geometry header")
        (self.include / "utils.H").write_bytes(b"transitive public header")
        self.library = self.base / "libvspaero_lib.a"
        self.library.write_bytes(b"mock explicit library")
        self.obj = self.base / "VSP_Geom.C.obj"
        self.obj.write_bytes(b"explicit corrected object")
        self.output = self.base / "evidence"
        self.expected_source = hashlib.sha256(self.source.read_bytes()).hexdigest()
        self.patch = mock.patch.object(native, "EXPECTED_SOURCE_SHA256", self.expected_source)
        self.patch.start()
        self.addCleanup(self.patch.stop)
        self.commands = []
        self.value = report()
        self.after_compile = None
        self.after_run = None
        self.process_overrides = {}

    def fake_execute(self, command, cwd, log, timeout, env):
        self.commands.append(list(map(str, command)))
        stage = "version" if command[1:] == ["--version"] else "compile" if "-o" in command else "run"
        Path(log).write_text("mock raw " + stage + "\n")
        if stage == "compile":
            Path(command[command.index("-o") + 1]).write_bytes(b"compiled fixture witness")
            if self.after_compile:
                self.after_compile()
        if stage == "run":
            cases = Path(command[1])
            cases.mkdir()
            (cases / "report.json").write_text(json.dumps(self.value))
            if self.after_run:
                self.after_run(Path(command[0]))
        result = {"command": list(map(str, command)), "exit_code": 0, "timed_out": False,
                  "seconds": .01, "log": str(log)}
        result.update(self.process_overrides.get(stage, {}))
        return result

    def run_fixture(self, **kwargs):
        with mock.patch.object(native, "_execute", side_effect=self.fake_execute) as execute:
            result = native.run_validation(self.compiler, self.include, self.library, self.output, **kwargs)
        self.execute_count = execute.call_count
        self.assertEqual(result, json.loads((self.output / "manifest.json").read_text()))
        self.assertFalse(result["model_accepted"])
        return result

    def test_explicit_inputs_full_four_case_success_and_manifest(self):
        result = self.run_fixture(geom_object=self.obj)
        self.assertTrue(result["passed"])
        self.assertEqual(result["status"], "passed")
        self.assertEqual(self.execute_count, 3)
        command = self.commands[1]
        self.assertLess(command.index(str(self.obj.resolve())), command.index(str(self.library.resolve())))
        self.assertEqual(len(self.commands[2]), 2)
        paths = {entry["path"] for entry in result["inputs"]}
        for path in (self.compiler, self.library, self.source, self.include / "utils.H", self.obj):
            self.assertIn(str(path.resolve()), paths)
        self.assertTrue(result["inputs_unchanged"])
        self.assertEqual(result["executable"]["sha256_before_run"], result["executable"]["sha256_after_run"])
        self.assertEqual(result["report"]["sha256"], hashlib.sha256((self.output / "cases/report.json").read_bytes()).hexdigest())

    def test_geometry_override_is_never_selected_automatically(self):
        result = self.run_fixture()
        self.assertTrue(result["passed"])
        self.assertNotIn(str(self.obj.resolve()), self.commands[1])
        self.assertIsNone(result["geometry_object"])

    def test_exit_zero_cannot_hide_any_failed_native_guard(self):
        for field in ("surfaces_unchanged", "other_fine_nodes_unchanged", "roots_attached",
                      "repeated_update_idempotent", "all_coordinates_finite", "passed",
                      "deflect_wake_matches_fine", "undeflect_wake_matches_fine",
                      "deflect_fine_nodes_unchanged", "undeflect_fine_nodes_unchanged"):
            with self.subTest(field=field):
                self.output = self.base / field
                self.value = report()
                self.value["cases"][2][field] = False
                result = self.run_fixture()
                self.assertFalse(result["passed"])
                self.assertIn(field, result["error"])

    def test_missing_level2_wrong_level_and_identity_cannot_pass(self):
        mutations = (lambda r: r["cases"].pop(), lambda r: r["cases"][3].update(level=1),
                     lambda r: r["cases"][3].update(nonidentity=False),
                     lambda r: r["cases"][3].update(available_levels=1),
                     lambda r: r["cases"][0].update(zero_control_surfaces=1),
                     lambda r: r["cases"][3].update(checked_wake_nodes_per_path=4),
                     lambda r: r["cases"][2]["attachments"][0].update(fine_wake=5),
                     lambda r: r["cases"].reverse())
        for index, mutate in enumerate(mutations):
            with self.subTest(index=index):
                self.output = self.base / ("shape-%d" % index)
                self.value = report()
                mutate(self.value)
                self.assertFalse(self.run_fixture()["passed"])

    def test_boolean_substitutes_nonfinite_and_model_admission_are_rejected(self):
        mutations = (lambda r: r.update(all_passed=1), lambda r: r.update(model_accepted=True),
                     lambda r: r["cases"][0].update(level=False),
                     lambda r: r["cases"][0].update(roots_attached=1),
                     lambda r: r["cases"][0].update(extra=float("nan")))
        for index, mutate in enumerate(mutations):
            with self.subTest(index=index):
                self.output = self.base / ("invalid-%d" % index)
                self.value = report()
                mutate(self.value)
                self.assertFalse(self.run_fixture()["passed"])

    def test_original_source_pin_mismatch_stops_before_external_process(self):
        self.source.write_bytes(b"modified or wrong version")
        result = self.run_fixture()
        self.assertFalse(result["passed"])
        self.assertEqual(self.execute_count, 0)
        self.assertIn("VSP_Geom.C", result["error"])
        self.assertIn(str(self.source.resolve()), {item["path"] for item in result["inputs"]})
        self.assertNotEqual(result["original_source"]["sha256"], result["expected_original_source_sha256"])

    def test_changed_input_after_compile_is_rejected_before_execution(self):
        self.after_compile = lambda: self.library.write_bytes(b"changed library")
        result = self.run_fixture()
        self.assertFalse(result["passed"])
        self.assertEqual(self.execute_count, 2)
        self.assertFalse(result["inputs_unchanged"])

    def test_changed_header_or_executable_during_native_run_is_rejected(self):
        for change in ("header", "exe"):
            with self.subTest(change=change):
                self.output = self.base / ("changed-" + change)
                if change == "header":
                    self.after_run = lambda exe: (self.include / "utils.H").write_bytes(b"mutated")
                else:
                    self.after_run = lambda exe: exe.write_bytes(b"mutated executable")
                self.assertFalse(self.run_fixture()["passed"])

    def test_compile_failure_or_timeout_never_runs_fixture(self):
        for change in ({"exit_code": 1}, {"timed_out": True, "exit_code": 0}):
            with self.subTest(change=change):
                self.output = self.base / ("compile-%d" % len(self.commands))
                self.process_overrides = {"compile": change}
                result = self.run_fixture()
                self.assertFalse(result["passed"])
                self.assertEqual(self.execute_count, 2)

    def test_valid_report_cannot_hide_nonzero_exit_or_timeout(self):
        for change in ({"exit_code": 2}, {"timed_out": True, "exit_code": 0}):
            with self.subTest(change=change):
                self.output = self.base / ("run-%d" % len(self.commands))
                self.process_overrides = {"run": change}
                self.assertFalse(self.run_fixture()["passed"])

    def test_failed_native_report_and_attempted_commands_stay_in_manifest(self):
        self.process_overrides = {"run": {"exit_code": 2}}
        self.value["all_passed"] = False
        self.value["cases"][2]["roots_attached"] = False
        result = self.run_fixture()
        self.assertFalse(result["passed"])
        self.assertEqual(result["report"]["contents"], self.value)
        self.assertIn("sha256", result["report"])
        self.assertEqual(result["steps"][-1]["cwd"], str(self.output.absolute()))
        self.assertEqual(result["steps"][-1]["timeout_s"], 30.)
        self.assertEqual(result["executable"]["sha256_before_run"], result["executable"]["sha256_after_run"])

    def test_spawn_failure_retains_attempted_command_and_raw_log(self):
        with mock.patch.object(native.subprocess, "Popen", side_effect=OSError("mock launch refused")):
            result = native.run_validation(self.compiler, self.include, self.library, self.output)
        self.assertFalse(result["passed"])
        self.assertEqual(len(result["steps"]), 1)
        self.assertEqual(result["steps"][0]["command"], [str(self.compiler.resolve()), "--version"])
        self.assertIn("mock launch refused", result["steps"][0]["error"])
        self.assertTrue(Path(result["steps"][0]["log"]).exists())

    def test_report_replaced_during_publication_is_not_a_stale_pass(self):
        original = native._strict_report
        def replace_after_validation(value):
            checked = original(value)
            changed = report()
            changed["cases"][2]["deflect_wake_matches_fine"] = False
            (self.output / "cases/report.json").write_text(json.dumps(changed))
            return checked
        with mock.patch.object(native, "_strict_report", side_effect=replace_after_validation):
            result = self.run_fixture()
        self.assertFalse(result["passed"])
        self.assertFalse(result["report"]["contents"]["cases"][2]["deflect_wake_matches_fine"])

    def test_overflow_duplicate_and_truncated_json_fail_with_evidence(self):
        invalid = ('{"all_passed":true,"all_passed":false}', '{"extra":1e999}', '{"cases":')
        for index, text in enumerate(invalid):
            with self.subTest(index=index):
                self.output = self.base / ("bad-json-%d" % index)
                self.after_run = lambda exe, text=text: (self.output / "cases/report.json").write_text(text)
                result = self.run_fixture()
                self.assertFalse(result["passed"])
                self.assertIn("sha256", result["report"])

    def test_existing_output_is_rejected_without_overwrite(self):
        self.output.mkdir()
        marker = self.output / "keep"
        marker.write_bytes(b"original evidence")
        with mock.patch.object(native, "_execute") as execute, self.assertRaises(ValueError):
            native.run_validation(self.compiler, self.include, self.library, self.output)
        execute.assert_not_called()
        self.assertEqual(marker.read_bytes(), b"original evidence")

    def test_timeout_must_be_positive_finite_and_bounded(self):
        for value in (0, -1, float("inf"), float("nan"), 301, True):
            with self.subTest(value=value), mock.patch.object(native, "_execute") as execute:
                with self.assertRaises(ValueError):
                    native.run_validation(self.compiler, self.include, self.library, self.output, compile_timeout=value)
                execute.assert_not_called()
        self.assertFalse(self.output.exists())

    def test_process_timeout_terminates_tree_and_retains_raw_log(self):
        process = mock.Mock()
        process.communicate.side_effect = [subprocess.TimeoutExpired(["mock"], .1), (None, None)]
        process.returncode = -9
        with mock.patch.object(native.subprocess, "Popen", return_value=process), \
                mock.patch.object(native, "_terminate_tree") as terminate:
            result = native._execute(["mock"], self.base, self.base / "timeout.log", .1, {})
        terminate.assert_called_once_with(process, mock.ANY)
        self.assertTrue(result["timed_out"])
        self.assertEqual(result["exit_code"], -9)
        self.assertTrue((self.base / "timeout.log").exists())


if __name__ == "__main__":
    unittest.main()
