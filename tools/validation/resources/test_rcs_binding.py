"""Asset binding regressions; synthetic bytes and metrics are not calibration."""
import importlib.util
import json
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest

target = Path(os.environ.get("RCS_BINDING_UNDER_TEST", str(Path(__file__).with_name("check_rcs.py"))))
spec = importlib.util.spec_from_file_location("rcs_binding_under_test", target)
checker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)


class Binding(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        folder = self.root / "fixture"
        folder.mkdir()
        self.model = folder / "fixture.glb"
        self.model.write_bytes(b"abc")
        self.table = folder / "rcs.bin"
        # Legacy native byte hash, source algorithm in tools/models/rcs/main.cpp.
        self.hash = "e16801510db89efd"
        self.header = {"aircraft": "fixture", "bands_hz": [1.0],
                       "provenance": {"model": "fixture.glb", "model_fnv1a": self.hash}}

    def write(self):
        raw = json.dumps(self.header).encode("utf-8")
        self.table.write_bytes(b"FSRCS\x00\x01\x00" + struct.pack("<I", len(raw)) + raw)

    def verified(self):
        self.header["provenance"]["model_fnv1a"] = "0000000000000000"
        self.header["provenance"]["verified"] = {
            "model": "fixture.glb", "model_fnv1a": self.hash, "bands_hz": [1.0], "bins": 2,
            "tolerance_db": .1, "max_db": .12, "p99_db": .05, "p95_db": .02,
            "noise_max_db": .04, "noise_p99_db": .02, "noise_p95_db": .01}

    def test_original_bake_matches(self):
        self.write()
        self.assertTrue(checker.check_binding(self.table, self.model)["passed"])

    def test_native_verification_can_bind_a_new_model_without_relabelling_the_bake(self):
        self.verified()
        self.write()
        result = checker.check_binding(self.table, self.model)
        self.assertTrue(result["passed"])
        self.assertEqual(result["binding"], "verification")

    def test_changed_model_cannot_reuse_either_stamp(self):
        self.verified()
        self.write()
        self.model.write_bytes(b"changed geometry")
        with self.assertRaises(ValueError):
            checker.check_binding(self.table, self.model)

    def test_a_matching_hash_cannot_hide_a_different_model_name(self):
        self.header["provenance"]["model"] = "other.glb"
        self.write()
        with self.assertRaises(ValueError):
            checker.check_binding(self.table, self.model)

    def test_failed_native_noise_criterion_is_not_a_valid_verification(self):
        for field in ("max_db", "p99_db", "p95_db"):
            with self.subTest(field=field):
                self.verified()
                self.header["provenance"]["verified"][field] = 1.0
                self.write()
                with self.assertRaises(ValueError):
                    checker.check_binding(self.table, self.model)

    def test_nonfinite_or_missing_metrics_are_rejected(self):
        for value in (None, float("nan"), -1, True):
            with self.subTest(value=value):
                self.verified()
                self.header["provenance"]["verified"]["noise_max_db"] = value
                self.write()
                with self.assertRaises(ValueError):
                    checker.check_binding(self.table, self.model)

    def test_verification_of_other_bands_is_not_reused(self):
        self.verified()
        self.header["provenance"]["verified"]["bands_hz"] = [2.0]
        self.write()
        with self.assertRaises(ValueError):
            checker.check_binding(self.table, self.model)

    def test_truncated_native_header_is_rejected(self):
        self.write()
        self.table.write_bytes(self.table.read_bytes()[:-1])
        with self.assertRaises(ValueError):
            checker.check_binding(self.table, self.model)

    def cli(self):
        output = self.root / "binding.json"
        run = subprocess.run([sys.executable, "-B", str(Path(__file__).with_name("check_rcs.py")),
                              "--aircraft-root", str(self.root), "--output", str(output)],
                             capture_output=True, text=True, timeout=30)
        return run, output

    def test_cli_publishes_a_current_success_and_nonempty_count(self):
        self.write()
        run, output = self.cli()
        self.assertEqual(run.returncode, 0, run.stderr)
        self.assertEqual(json.loads(run.stdout)["checked"], 1)
        self.assertTrue(json.loads(output.read_text())["passed"])

    def test_cli_replaces_an_old_success_when_current_data_fail(self):
        self.write()
        self.table.write_bytes(b"truncated")
        output = self.root / "binding.json"
        output.write_text('{"passed":true}')
        run, output = self.cli()
        self.assertEqual(run.returncode, 1, run.stderr)
        self.assertFalse(json.loads(run.stdout)["passed"])
        self.assertFalse(json.loads(output.read_text())["passed"])


if __name__ == "__main__":
    unittest.main()
