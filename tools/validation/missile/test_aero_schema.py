"""SU2 required-schema and continuation-safety regressions for reference_body (Task 02).

A malformed SU2 history (missing native Cauchy columns) must be rejected terminally,
before numerical nonconvergence is classified, and must never authorize continuation
from its own outgoing checkpoint.

The module under test is loaded from the explicit path in AERO_UNDER_TEST (default: the
project aero.py). Its own directory is put first on sys.path, so its sibling modules
resolve beside it. parse_su2() and su2_case() run unmodified; only the process boundary
bounded() is mocked. No solver, mesher or build is executed.

Run from the project root:
    fsim.cmd --preset ucrt64-headless python tools/validation/missile/test_aero_schema.py -v
"""
import hashlib
import importlib.util
import os
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest import mock

TARGET = Path(os.environ.get("AERO_UNDER_TEST",
                             str(Path(__file__).with_name("aero.py")))).resolve()
if not TARGET.is_file():
    raise FileNotFoundError("AERO_UNDER_TEST is not a file: %s" % TARGET)
sys.path.insert(0, str(TARGET.parent))
_SPEC = importlib.util.spec_from_file_location("aero_under_test", TARGET)
aero = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(aero)
print("module under test: %s sha256=%s" % (TARGET, hashlib.sha256(TARGET.read_bytes()).hexdigest()), flush=True)

REQUIRED_CAUCHY = ("Cauchy[CD]", "Cauchy[CL]", "Cauchy[CMy]")
OMISSIONS = [(name,) for name in REQUIRED_CAUCHY] + [REQUIRED_CAUCHY]
COLUMNS = ("Inner_Iter", "rms[Rho]", "CD", "CL", "CMy") + REQUIRED_CAUCHY
REFERENCE = {"area_m2": .1, "length_m": .1, "moment_x_m": 1}
MALFORMED_CHECKPOINT = b"reference_body malformed-pass outgoing checkpoint"


def history_text(omitted=(), rho=-7, rows=200, moment="CMy", cauchy_cd=0.0, cd=.01, cl=0.0, cm=0.0):
    """Finite synthetic SU2 history; `omitted` columns are absent from header and rows."""
    values = {"rms[Rho]": rho, "CD": cd, "CL": cl, "CMy": cm,
              "Cauchy[CD]": cauchy_cd, "Cauchy[CL]": 0.0, "Cauchy[CMy]": 0.0}
    names = [name for name in COLUMNS if name not in omitted]
    lines = [",".join('"%s"' % (moment if name == "CMy" else name) for name in names)]
    for iteration in range(rows):
        lines.append(",".join(str(iteration) if name == "Inner_Iter" else repr(float(values[name])) for name in names))
    return "\n".join(lines) + "\n"


def parse(text, **limits):
    with tempfile.TemporaryDirectory() as folder:
        path = Path(folder) / "history.csv"
        path.write_text(text, encoding="utf-8")
        return aero.parse_su2(path, 2.0, **limits)


class MissingRequiredSchemaIsTerminal(unittest.TestCase):
    def assertTerminalSchemaRejection(self, text, **limits):
        try:
            result = parse(text, **limits)
        except aero.NotConverged as error:  # NotConverged subclasses ValueError: catch it first
            self.fail("malformed required schema was classified as continuable NotConverged: %s" % error)
        except ValueError as error:
            self.assertIn("Cauchy", str(error), "terminal rejection is not the missing-schema rejection: %s" % error)
            return
        self.fail("malformed required schema was accepted with status %r" % (result.get("status"),))

    def test_each_and_all_missing_cauchy_columns_are_terminal_at_density_minus_7(self):
        for omitted in OMISSIONS:
            for limits in ({}, {"residual_limit": -9}):
                with self.subTest(omitted=omitted, limits=limits):
                    self.assertTerminalSchemaRejection(history_text(omitted), **limits)

    def test_missing_cauchy_columns_are_terminal_with_upper_case_moment_column(self):
        for omitted in OMISSIONS:
            with self.subTest(omitted=omitted):
                self.assertTerminalSchemaRejection(history_text(omitted, moment="CMY"))

    def test_missing_cauchy_columns_remain_terminal_when_density_is_converged(self):
        for omitted in OMISSIONS:
            with self.subTest(omitted=omitted):
                self.assertTerminalSchemaRejection(history_text(omitted, rho=-9))


class ValidOutputClassificationIsPreserved(unittest.TestCase):
    def test_complete_density_unconverged_history_is_continuable(self):
        for limits in ({}, {"residual_limit": -9}):
            with self.subTest(limits=limits), self.assertRaisesRegex(aero.NotConverged, "density residual"):
                parse(history_text(rho=-7), **limits)

    def test_complete_converged_history_is_accepted_for_both_moment_spellings(self):
        expected = aero.body_coefficients(.01, .02, -.03, 2.0)
        for moment in ("CMy", "CMY"):
            with self.subTest(moment=moment):
                result = parse(history_text(rho=-9, cd=.01, cl=.02, cm=-.03, moment=moment))
                self.assertEqual((result["status"], result["residual_log10"], result["iterations"],
                                  result["load_window_samples"], result["alpha_deg"], result["wind_cd"]),
                                 ("converged", -9.0, 200, 100, 2.0, .01))
                self.assertEqual(result["coefficients"], expected)
                self.assertEqual(result["native_cauchy"], {name: 0.0 for name in REQUIRED_CAUCHY})
                self.assertTrue(all(row["stable"] for row in result["load_stability"].values()))

    def test_caller_selected_residual_limit_is_respected(self):
        self.assertEqual(parse(history_text(rho=-7), residual_limit=-6)["status"], "converged")
        with self.assertRaisesRegex(aero.NotConverged, "density residual"):
            parse(history_text(rho=-9), residual_limit=-10)

    def test_complete_schema_with_unmet_native_cauchy_target_stays_continuable(self):
        with self.assertRaisesRegex(aero.NotConverged, "Cauchy targets"):
            parse(history_text(rho=-9, cauchy_cd=1e-9))

    def test_complete_schema_without_two_load_windows_stays_continuable(self):
        with self.assertRaisesRegex(aero.NotConverged, "two complete 100-sample windows"):
            parse(history_text(rho=-9, rows=150))


class ContinuationSafety(unittest.TestCase):
    maxDiff = None  # keep the complete observed-versus-required record in raw logs

    def run_case(self, outputs, passes=3):
        """Run the real su2_case(); process call i writes outputs[i] = (history text, checkpoint bytes)."""
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)
            (output / "mesh.su2").write_bytes(b"synthetic fixed mesh")
            calls = []

            def process(command, directory, log, timeout=120):
                directory = Path(directory)
                incoming = directory / "restart.dat"
                config = Path(command[-1]).read_text(encoding="ascii")
                calls.append({"restart_sol": "YES" if "RESTART_SOL= YES" in config else "NO",
                              "restart_input": incoming.read_bytes() if incoming.exists() else None})
                history, checkpoint = outputs[len(calls) - 1]
                (directory / "history.csv").write_text(history, encoding="utf-8")
                (directory / "restart-out.dat").write_bytes(checkpoint)
                Path(log).write_text("synthetic process boundary; no solver executed\n", encoding="utf-8")
                return {"exit_code": 0, "status": "executed", "command": [str(v) for v in command],
                        "seconds": 1.0, "log": str(log)}

            args = types.SimpleNamespace(su2=Path("su2-not-executed"), iterations=2200, su2_passes=passes,
                                         solver_timeout=300)
            with mock.patch.object(aero, "bounded", side_effect=process) as boundary:
                case = aero.su2_case(args, output, REFERENCE, 2.0)
            copies = sorted(path.relative_to(output).as_posix() for path in output.rglob("*")
                            if path.is_file() and path.read_bytes() == MALFORMED_CHECKPOINT)
            folders = sorted(path.name for path in (output / "su2-alpha-2").iterdir())
            outgoing = {name: aero.sha256(output / "su2-alpha-2" / name / "restart-out.dat") for name in folders}
            return case, calls, boundary.call_count, copies, folders, outgoing

    @staticmethod
    def observed(case, calls, count, copies, folders):
        return {"process_calls": count, "status": case["status"], "converged_result": "result" in case,
                "pass_statuses": [run.get("status") for run in case["passes"]], "pass_folders": folders,
                "restart_inputs": [call["restart_input"] for call in calls], "malformed_checkpoint_files": copies}

    def test_malformed_first_pass_is_terminal_and_its_checkpoint_is_never_reused(self):
        converged_second_pass = (history_text(rho=-9), b"hypothetical converged second-pass state")
        for omitted in OMISSIONS:
            with self.subTest(omitted=omitted):
                case, calls, count, copies, folders, _ = self.run_case(
                    [(history_text(omitted), MALFORMED_CHECKPOINT), converged_second_pass], passes=3)
                self.assertEqual(self.observed(case, calls, count, copies, folders),
                                 {"process_calls": 1, "status": "unavailable", "converged_result": False,
                                  "pass_statuses": ["unavailable"], "pass_folders": ["pass-0"],
                                  "restart_inputs": [None],
                                  "malformed_checkpoint_files": ["su2-alpha-2/pass-0/restart-out.dat"]})
                self.assertIn("Cauchy", case["reason"])
                self.assertIsNone(case["passes"][0]["input_restart_sha256"])

    def test_malformed_later_pass_is_terminal_after_valid_continuation(self):
        outputs = [(history_text(rho=-7), b"valid pass-0 state"),
                   (history_text(REQUIRED_CAUCHY, rho=-7), MALFORMED_CHECKPOINT),
                   (history_text(rho=-9), b"hypothetical converged pass-2 state")]
        case, calls, count, copies, folders, _ = self.run_case(outputs, passes=4)
        self.assertEqual(self.observed(case, calls, count, copies, folders),
                         {"process_calls": 2, "status": "unavailable", "converged_result": False,
                          "pass_statuses": ["not_converged", "unavailable"], "pass_folders": ["pass-0", "pass-1"],
                          "restart_inputs": [None, b"valid pass-0 state"],
                          "malformed_checkpoint_files": ["su2-alpha-2/pass-1/restart-out.dat"]})
        self.assertIn("Cauchy", case["reason"])

    def test_valid_continuation_uses_only_its_own_preceding_checkpoint_and_converges(self):
        outputs = [(history_text(rho=-7), b"valid pass-0 state"),
                   (history_text(rho=-7.5), b"valid pass-1 state"),
                   (history_text(rho=-9), b"converged pass-2 state")]
        case, calls, count, copies, folders, outgoing = self.run_case(outputs, passes=4)
        self.assertEqual(self.observed(case, calls, count, copies, folders),
                         {"process_calls": 3, "status": "converged", "converged_result": True,
                          "pass_statuses": ["not_converged", "not_converged", "converged"],
                          "pass_folders": ["pass-0", "pass-1", "pass-2"],
                          "restart_inputs": [None, b"valid pass-0 state", b"valid pass-1 state"],
                          "malformed_checkpoint_files": []})
        self.assertEqual([call["restart_sol"] for call in calls], ["NO", "YES", "YES"])
        self.assertEqual([run["input_restart_sha256"] for run in case["passes"]],
                         [None, outgoing["pass-0"], outgoing["pass-1"]])
        self.assertEqual((case["result"]["status"], case["result"]["mach"]), ("converged", 0.6))

    def test_converged_first_pass_needs_no_continuation(self):
        case, calls, count, copies, folders, _ = self.run_case([(history_text(rho=-9), b"converged state")])
        self.assertEqual((count, case["status"], [run["status"] for run in case["passes"]]),
                         (1, "converged", ["converged"]))


if __name__ == "__main__":
    unittest.main()
