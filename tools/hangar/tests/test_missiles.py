"""The air-to-air missiles as hangar designs (hangar/missiles.py, ADR-30 MS-7e1): each design is what the table writes,
holds its card's published figures, and carries its motor to the viewer. A native encounter selects a real carrier
station and records the SDK release."""
import json
import importlib.util
import io
import os
import pathlib
import tempfile
import types
import unittest
from contextlib import redirect_stderr, redirect_stdout
from unittest import mock

import numpy as np

from hangar import missiles, model3d, pipeline, showcase
from hangar.geometry.aircraft import Aircraft, Motor
from hangar.pipeline import missile_span


def _design(name):
    return os.path.join(pipeline.repo_root(), "aircraft", name, name + ".toml")


class Missiles(unittest.TestCase):
    def test_written_as_committed(self):
        # the table is how each design was made: the committed files are what it writes
        for name in missiles.MISSILES:
            toml, paint = missiles.design(name)
            with open(_design(name), encoding="utf-8") as f:
                self.assertEqual(f.read(), toml, "%s is not what `python -m hangar missiles %s` writes" % (_design(name), name))
            with open(os.path.join(os.path.dirname(_design(name)), "paint.toml"), encoding="utf-8") as f:
                self.assertEqual(f.read(), paint, name)

    def test_card_figures_held(self):
        for name, row in missiles.MISSILES.items():
            a = Aircraft.load(_design(name))
            self.assertEqual(a.category, "missile")
            with open(os.path.join(pipeline.repo_root(), "assets", "stores", row["card"] + ".json"), encoding="utf-8") as f:
                par = json.load(f)["parameters"]
            body = next(b for b in a.bodies if b.name == "body")
            nose = min(float(b.x[0]) for b in a.bodies)  # (an infrared seeker's dome ahead of the body)
            self.assertAlmostEqual(float(body.x[-1]) - nose, par["length_m"]["value"], places=3, msg=name)
            self.assertAlmostEqual(float(2.0 * max(body.top)), par["diameter_m"]["value"], places=3, msg=name)
            if "span_m" in par:  # tip to tip across opposite fins
                self.assertAlmostEqual(missile_span(a), par["span_m"]["value"], delta=0.005 * par["span_m"]["value"], msg=name)
            self.assertEqual(a.spec["mass"]["empty"], par["mass_kg"]["value"], name)
            # every missile moves something: its fins, canards or wings, or its tail's ailerons
            self.assertTrue(any(s.controls for s in a.surfaces), name)

    def test_motor_to_the_viewer(self):
        a = Aircraft.load(_design("aim120c"))
        self.assertEqual(a.motor.smoke, "reduced")
        lines = model3d.engine_lines(a)
        rocket = [ln for ln in lines if ln.startswith("rocket ")]
        self.assertEqual(len(rocket), 1)
        _, index, smoke, exit_d, thrust = rocket[0].split()
        self.assertEqual((index, smoke), ("0", "reduced"))
        self.assertAlmostEqual(float(exit_d), a.motor.exit_diameter, places=4)
        self.assertAlmostEqual(float(thrust), max(t for _, t in a.motor.phases), places=3)
        self.assertFalse(any(ln.startswith("engine ") for ln in lines))  # no jet's line for it
        for bad in ({"exit": [1, 0, 0]}, {"exit": [1, 0, 0], "exit_diameter": 0.1, "smoke": "sooty"},
                    {"exit": [1, 0, 0], "exit_diameter": 0.1, "throat_diameter": 0.2}):
            with self.assertRaises(ValueError):
                Motor(bad)

    def test_showcase_script(self):
        s = showcase.script("aim120c")
        self.assertEqual(s.get("schema"), "native-engagement-v1")
        self.assertEqual(s["rate_hz"], 30)
        m = s["missiles"][0]
        # the F-16C's loadouts put the AIM-120C on its wingtip rails (stations 1 and 9): a rail launch
        self.assertIn(m["station_number"], (1, 9))
        self.assertEqual(m["card"], "aim-120c")
        self.assertEqual(m["launch_s"], 12.0)
        self.assertEqual(set(m), {"card", "station_number", "name", "launch_s"})
        self.assertEqual(s["target"]["type"], "jsbsim:f16c")
        self.assertEqual(s["target"]["range_m"], 5000.0)
        self.assertEqual(s["carrier"]["speed_ms"], 250.0)
        self.assertNotIn("thrust_n", s["carrier"])
        self.assertNotIn("n2", s["carrier"])

    def test_showcase_rejects_legacy_motion_controls(self):
        for field, value in (("detonate_s", 6.0), ("dive_s", 4.0), ("dive_deg", -25.0),
                             ("miss_m", (0.0, 5.0, 0.0)), ("separate", "left wing")):
            with self.subTest(field=field), self.assertRaises(TypeError):
                showcase.script("aim120c", **{field: value})

    def test_showcase_default_participants_have_distinct_native_names(self):
        s = showcase.script("aim120c")
        self.assertNotEqual(s["carrier"]["name"], s["target"]["name"], "World rejects duplicate vehicle names")

    def test_showcase_selects_station_that_can_take_sparrow_mass(self):
        s = showcase.script("aim7m")
        # The 225 kg AIM-7M exceeds the F-16's 192.8 kg wingtip limits; the outboard/mid rails can take it.
        self.assertIn(s["missiles"][0]["station_number"], (2, 3, 7, 8))

    def test_showcase_explicit_station_is_not_replaced(self):
        s = showcase.script("aim7m", station=1)
        self.assertEqual(s["missiles"][0]["station_number"], 1, "the native SDK must assess an explicit requested station")

    def test_showcase_target_is_initial_encounter_geometry(self):
        s = showcase.script("aim9m", "fa18c", target="mig29a", target_range=7000.0, target_altitude_offset=200.0, seed=7)
        self.assertEqual(s["target"], {"type": "jsbsim:mig29a", "name": "MIG29A-TARGET", "speed_ms": 250.0,
                                       "heading_deg": 270.0, "range_m": 7000.0, "altitude_offset_m": 200.0})
        self.assertEqual(s["seed"], 7)

    def test_showcase_cli_forwards_live_only_when_requested(self):
        scene = showcase.script("aim120c")
        for options, live in (([], False), (["--live"], True)):
            with self.subTest(live=live), tempfile.TemporaryDirectory() as folder:
                bin_dir = pathlib.Path(folder) / "bin"
                bin_dir.mkdir()
                exe = bin_dir / "flightsim-showcase.exe"
                exe.write_bytes(b"native tool fixture")
                transcript = io.StringIO()
                attach = 'fsim --preset ucrt64-release viewer --world "aim120c native encounter" --fullscreen --camera chase'

                def native_tool(command, **kwargs):
                    self.assertEqual(command[0], str(exe))
                    self.assertEqual(command[3:], ["--live"] if live else [])
                    self.assertEqual(kwargs, {"check": True})
                    with open(command[1], encoding="utf-8") as f:
                        self.assertEqual(json.load(f), scene, "live must use the same native scene")
                    self.assertEqual(attach in transcript.getvalue(), live, "live attach command must print before blocking")
                    with open(command[2] + ".engagement.json", "w", encoding="utf-8") as f:
                        json.dump({"schema": showcase.SCHEMA, "source": "native-sdk", "weapons": [
                            {"launch_time_s": 13.0, "terminal_kind": "detonated", "weapon_slot": 6}]}, f)
                    return types.SimpleNamespace(returncode=0)

                with mock.patch.object(showcase, "script", return_value=scene), \
                        mock.patch.object(showcase.pipeline, "repo_root", return_value=folder), \
                        mock.patch.dict(os.environ, {"FSIM_BIN": str(bin_dir), "FSIM_PRESET": "ucrt64-release"}), \
                        mock.patch.object(showcase.subprocess, "run", side_effect=native_tool) as child, \
                        redirect_stdout(transcript), redirect_stderr(transcript):
                    self.assertEqual(showcase.main(["aim120c"] + options), 0)
                self.assertEqual(child.call_count, 1, "Hangar must not spawn a second simulation or viewer")
                self.assertIn("play it: fsim --preset ucrt64-release viewer --replay", transcript.getvalue())


def _tool(name, relative):
    spec = importlib.util.spec_from_file_location(name, os.path.join(pipeline.repo_root(), relative))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class NativeShowcaseConsumers(unittest.TestCase):
    def test_screenshot_consumer_rejects_failed_viewer_process(self):
        perf = _tool("viewer_perf_test", "tools/viewer_perf/viewer_perf.py")
        with tempfile.TemporaryDirectory() as folder:
            cache = pathlib.Path(folder) / "shots"
            cache.mkdir()
            (cache / "reheat.fsrec").write_bytes(b"SDK recording fixture")
            args = types.SimpleNamespace(out=folder, shots="unit", scene=["reheat"], build=folder,
                                         exe="flightsim-viewer.exe", viewer=[])
            with mock.patch.object(perf.subprocess, "run", return_value=types.SimpleNamespace(returncode=7)):
                with self.assertRaisesRegex(RuntimeError, "viewer.*failed"):
                    perf.shots(args)

    def test_hit_recording_refuses_stale_mechanical_cache(self):
        perf = _tool("viewer_perf_test", "tools/viewer_perf/viewer_perf.py")
        with tempfile.TemporaryDirectory() as folder:
            cache = pathlib.Path(folder) / "shots"
            cache.mkdir()
            (cache / "hit.fsrec").write_bytes(b"obsolete mechanical recording")
            args = types.SimpleNamespace(out=folder, build=os.path.join(folder, "missing-build"))
            with self.assertRaisesRegex(RuntimeError, "native"):
                perf.recording("hit", args)

    def test_hit_reference_uses_native_target_and_fuze_time(self):
        perf = _tool("viewer_perf_test", "tools/viewer_perf/viewer_perf.py")
        data = {"target_id": 8, "target_slot": 5, "duration_s": 40.0,
                "weapons": [{"id": 12, "terminal_kind": "detonated", "end_time_s": 18.5,
                              "launch_time_s": 13.0, "holding_id": 8, "acquired": True}],
                "engagements": [{"kind": "detonation", "entity": 12, "target": 8, "time_s": 18.5},
                                {"kind": "damage", "entity": 12, "target": 8, "time_s": 18.5}]}
        times, cameras = perf.hit_reference(data)
        self.assertEqual(times, [18.55, 18.8, 19.3, 20.0, 21.5, 24.5, 28.5])
        self.assertEqual(cameras[0][:2], ["--follow", "5"])
        data["weapons"][0]["terminal_kind"] = "expired"
        with self.assertRaisesRegex(RuntimeError, "native.*hit"):
            perf.hit_reference(data)

    def test_hit_capture_plan_shows_actual_weapon_before_native_burst(self):
        perf = _tool("viewer_perf_test", "tools/viewer_perf/viewer_perf.py")
        data = {"carrier_slot": 2, "target_id": 8, "target_slot": 5, "duration_s": 40.0,
                "weapons": [{"id": 99, "weapon_slot": 9, "terminal_kind": "in_flight", "end_time_s": None},
                            {"id": 12, "weapon_slot": 6, "terminal_kind": "detonated", "end_time_s": 18.5,
                             "launch_time_s": 13.0, "holding_id": 8, "acquired": True}],
                "engagements": [{"kind": "detonation", "entity": 12, "target": 8, "time_s": 18.5},
                                {"kind": "damage", "entity": 12, "target": 8, "time_s": 18.5}]}
        plan = perf.hit_capture_plan(data)
        self.assertEqual(len(plan), 10)
        self.assertEqual([frame["phase"] for frame in plan[:3]], ["release", "flight", "approach"])
        self.assertEqual([frame["time_s"] for frame in plan[:3]], [13.1, 14.0, 18.25])
        self.assertEqual(plan[0]["camera"][:2], ["--follow", "2"])
        self.assertEqual(plan[1]["camera"][:2], ["--follow", "6"])
        self.assertEqual(plan[2]["camera"][:2], ["--follow", "6"])
        self.assertTrue(all(frame["time_s"] < 18.5 for frame in plan[:3]))
        self.assertTrue(all(frame["camera"][:2] == ["--follow", "5"] for frame in plan[3:]))
        data["weapons"][1]["terminal_kind"] = "expired"
        with self.assertRaisesRegex(RuntimeError, "native.*hit"):
            perf.hit_capture_plan(data)

    def test_hit_capture_plan_rejects_nonchronological_short_flight(self):
        perf = _tool("viewer_perf_test", "tools/viewer_perf/viewer_perf.py")
        data = {"carrier_slot": 2, "target_id": 8, "target_slot": 5, "duration_s": 40.0,
                "weapons": [{"id": 12, "weapon_slot": 6, "terminal_kind": "detonated", "end_time_s": 14.1,
                             "launch_time_s": 13.0, "holding_id": 8, "acquired": True}],
                "engagements": [{"kind": "detonation", "entity": 12, "target": 8, "time_s": 14.1},
                                {"kind": "damage", "entity": 12, "target": 8, "time_s": 14.1}]}
        for end in (14.1, 14.25):
            with self.subTest(end=end):
                data["weapons"][0]["end_time_s"] = end
                for event in data["engagements"]:
                    event["time_s"] = end
                with self.assertRaisesRegex(RuntimeError, "too short"):
                    perf.hit_capture_plan(data)

    def test_plume_frame_uses_replay_clock_and_native_weapon_slot(self):
        plume = _tool("plume_check_test", "tools/plume_check/check.py")
        def capture(command, **kwargs):
            plume.Image.new("RGB", (4, 4)).save(command[command.index("--screenshot") + 1])
            return types.SimpleNamespace(returncode=0)
        with tempfile.TemporaryDirectory() as folder, mock.patch.object(plume.subprocess, "run", side_effect=capture) as renderer:
            plume.render("native.fsrec", os.path.join(folder, "frame.png"), 17.25, True, slot=6)
        command = renderer.call_args.args[0]
        self.assertEqual(command[command.index("--follow") + 1], "6")
        self.assertEqual(command[command.index("--screenshot-clock") + 1], "replay")
        self.assertEqual(command[command.index("--screenshot-after") + 1], "17.25")

    def test_plume_does_not_reuse_stale_png_when_capture_writes_nothing(self):
        plume = _tool("plume_check_test", "tools/plume_check/check.py")
        with tempfile.TemporaryDirectory() as folder:
            out = pathlib.Path(folder) / "old-scripted-frame.png"
            plume.Image.new("RGB", (4, 4)).save(out)
            with mock.patch.object(plume.subprocess, "run", return_value=types.SimpleNamespace(returncode=0)):
                with self.assertRaisesRegex(RuntimeError, "screenshot"):
                    plume.render("native.fsrec", out, 17.25, True, slot=6)
            self.assertFalse(out.exists(), "old mechanical capture must not survive as native evidence")

    def test_plume_rejects_undecodable_png(self):
        plume = _tool("plume_check_test", "tools/plume_check/check.py")
        with tempfile.TemporaryDirectory() as folder:
            out = pathlib.Path(folder) / "broken.png"
            def capture(command, **kwargs):
                out.write_bytes(b"not a PNG")
                return types.SimpleNamespace(returncode=0)
            with mock.patch.object(plume.subprocess, "run", side_effect=capture):
                with self.assertRaisesRegex(RuntimeError, "screenshot"):
                    plume.render("native.fsrec", out, 17.25, True, slot=6)

    def test_plume_rejects_failed_viewer_process(self):
        plume = _tool("plume_check_test", "tools/plume_check/check.py")
        with tempfile.TemporaryDirectory() as folder:
            out = pathlib.Path(folder) / "frame.png"
            with mock.patch.object(plume.subprocess, "run", return_value=types.SimpleNamespace(returncode=7)):
                with self.assertRaisesRegex(RuntimeError, "viewer.*failed"):
                    plume.render("native.fsrec", out, 17.25, True, slot=6)


class Lattice(unittest.TestCase):
    def test_grid_fins(self):
        from hangar.shape import lattice
        a = Aircraft.load(_design("r77m"))
        fins = [s for s in a.surfaces if s.lattice]
        self.assertEqual(len(fins), 2)  # the upper and lower pairs of its X, each mirrored
        self.assertEqual(a.cell, 0.0015)  # (its [model] cell: walls a few cells thick)
        scene = lattice.lattice(fins[0], 0)
        frame, grid = scene["children"]
        self.assertEqual(frame["op"], "subtract")  # a box less its inside
        outer, hole = frame["a"], frame["b"]
        self.assertAlmostEqual(outer["half"][2], 0.5 * fins[0].lattice["width"])
        self.assertAlmostEqual(outer["half"][2] - hole["half"][2], lattice.FRAME)
        walls = grid["children"][0]["children"]
        self.assertGreaterEqual(len(walls), 2 * 3)  # two families, at least a cell's worth each way
        # the walls at 45 deg across the frame's face, along the flow as deep as the fin's chord
        s_axis, u_axis = np.array(outer["axes"][1]), np.array(outer["axes"][2])
        along = np.array(walls[0]["axes"][1])
        self.assertAlmostEqual(abs(float(along @ s_axis)), np.sqrt(0.5), places=6)
        self.assertAlmostEqual(abs(float(along @ u_axis)), np.sqrt(0.5), places=6)
        self.assertAlmostEqual(walls[0]["half"][0], outer["half"][0])
        self.assertTrue(all(w["mirror"] for w in walls))  # the defined half's, mirrored as the surface is
        self.assertFalse(lattice.lattice(fins[0], 0, mirror=False)["children"][0]["a"]["mirror"])


if __name__ == "__main__":
    unittest.main()
