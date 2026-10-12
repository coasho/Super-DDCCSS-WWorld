"""The testbench keeps explicit viewer/build choices and browser state together."""
import importlib
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "testbench"))
entry = importlib.import_module("fsim_testbench.__main__")


class TestTestbenchEntry(unittest.TestCase):
    def setUp(self):
        self.scratch = tempfile.TemporaryDirectory(prefix="fsim-testbench-entry-", dir="D:/ENV/tmp")
        self.addCleanup(self.scratch.cleanup)
        self.root = Path(self.scratch.name)
        self.selected_bin = self.root / "build/ucrt64-headless/bin"
        self.release_viewer = self.root / "build/ucrt64-release/bin/flightsim-viewer.exe"
        self.release_viewer.parent.mkdir(parents=True)
        self.release_viewer.touch()
        self.addCleanup(patch.stopall)
        patch.object(entry, "ROOT", str(self.root)).start()
        patch.dict(os.environ, {"FSIM_BIN": str(self.selected_bin)}).start()

    def test_selected_build_without_viewer_never_uses_release_viewer(self):
        self.assertIsNone(entry.find_viewer())

    def test_missing_explicit_viewer_never_falls_back(self):
        os.environ["FSIM_BIN"] = str(self.release_viewer.parent)
        self.assertIsNone(entry.find_viewer(str(self.root / "missing-viewer.exe")))

    def test_existing_explicit_viewer_wins_over_selected_build(self):
        self.assertEqual(entry.find_viewer(str(self.release_viewer)), str(self.release_viewer))

    def test_headless_browser_profile_belongs_to_selected_build(self):
        app = entry.App(SimpleNamespace(no_viewer=True, viewer=None, browser=None))
        with patch.object(entry, "find_edge", return_value="edge.exe"), patch.object(entry.subprocess, "Popen") as launch:
            app.open_window("http://127.0.0.1:1234/")
        self.assertIn("--user-data-dir=" + str(self.selected_bin.parent / "logs/testbench-window"), launch.call_args.args[0])


if __name__ == "__main__":
    unittest.main()
