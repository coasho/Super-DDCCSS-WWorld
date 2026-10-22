"""The air-to-air missiles as hangar designs (hangar/missiles.py, ADR-30 MS-7e1): each design is what the table writes,
holds its card's published figures, and carries its motor to the viewer. A native encounter selects a real carrier
station and records the SDK release."""
import json
import copy
import importlib.util
import io
import math
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


def _synthetic_motor():
    return {"exit": [1.0, 0.0, 0.0], "exit_diameter": 0.1,
            "phase": [{"duration_s": 1.0, "thrust_kn": 0.001}]}


def _invalid_synthetic_motors():
    yield "not mapping", None
    yield "missing fields", {}
    for value in ([1.0, 0.0], [1.0, 0.0, 0.0, 0.0], "100", {0: 1.0, 1: 0.0, 2: 0.0}):
        yield "exit shape", _synthetic_motor() | {"exit": value}
    for value in (float("nan"), float("inf"), float("-inf"), True, "1.0", None, complex(1.0, 0.0)):
        yield "exit coordinate", _synthetic_motor() | {"exit": [value, 0.0, 0.0]}
        for field in ("exit_diameter", "throat_diameter", "cone_length"):
            yield field, _synthetic_motor() | {field: value}
        for field in ("duration_s", "thrust_kn"):
            phase = {"duration_s": 1.0, "thrust_kn": 0.001} | {field: value}
            yield field, _synthetic_motor() | {"phase": [phase]}
    for field, value in (("exit_diameter", 0.0), ("exit_diameter", -1.0),
                         ("throat_diameter", 0.0), ("throat_diameter", 0.1),
                         ("throat_diameter", 0.2), ("cone_length", 0.0), ("cone_length", -1.0),
                         ("smoke", "unrecognised"), ("smoke", []), ("phase", None),
                         ("phase", {}), ("phase", "one"), ("phase", [None]),
                         ("phase", [{}]), ("phase", [{"duration_s": 1.0}]),
                         ("phase", [{"thrust_kn": 0.001}]),
                         ("phase", [{"duration_s": 0.0, "thrust_kn": 0.001}]),
                         ("phase", [{"duration_s": -1.0, "thrust_kn": 0.001}]),
                         ("phase", [{"duration_s": 1.0, "thrust_kn": -0.001}])):
        yield field, _synthetic_motor() | {field: value}


class Missiles(unittest.TestCase):
    def test_pulse_motor_uses_ignition_time_without_changing_other_generated_files(self):
        import xml.etree.ElementTree as ET
        from hangar import storebodies
        for name, card_id in (("pl15", "pl-15"), ("r37m", "r-37m"), ("r77m", "r-77m")):
            with self.subTest(model=name):
                root = pathlib.Path(pipeline.repo_root())
                card = json.loads((root / "assets/stores" / (card_id + ".json")).read_text(encoding="utf-8"))
                generated = storebodies.files(card)
                engine_name = "Engines/%s_motor.xml" % name
                table = ET.fromstring(generated[engine_name]).find("thrust_table")
                self.assertEqual(table.get("name"), "fsim/ignition-time-thrust-v1")
                rows = [tuple(map(float, line.split())) for line in table.findtext("tableData").splitlines() if line.strip()]
                self.assertEqual(rows[-1][1], 0.0)
                for path, text in generated.items():
                    if path != engine_name:
                        self.assertEqual(text, (root / "aircraft/stores" / name / path).read_text(encoding="utf-8"))

    def test_pulse_time_table_keeps_the_coast_and_conserves_assumed_impulse(self):
        import bisect
        from hangar.storebodies import BUILDUP, LBF, _ignition_time_thrust_rows
        phases = [{"duration_s": 4.0, "thrust_kn": 25.0},
                  {"duration_s": 6.0, "thrust_kn": 0.0},
                  {"duration_s": 4.0, "thrust_kn": 12.0}]
        original = copy.deepcopy(phases)
        rows = _ignition_time_thrust_rows(phases)
        deficit = BUILDUP * (1 - 2 / math.pi)
        self.assertEqual(rows[0][0], 0.0)
        self.assertEqual(rows[-1], (14.0 + deficit, 0.0))
        self.assertEqual(phases, original)
        coordinates = [x for x, _ in rows]
        emitted = [float("%.17g" % x) for x in coordinates]
        self.assertTrue(all(a < b for a, b in zip(emitted, emitted[1:])))
        for time, newtons in ((1.0, 25000.0), (4.0 + deficit, 2.0), (5.0, 2.0),
                              (9.0, 2.0), (10.0 + deficit, 12000.0), (11.0, 12000.0), (15.0, 0.0)):
            index = max(0, min(len(rows) - 2, bisect.bisect_right(coordinates, time) - 1))
            (x0, y0), (x1, y1) = rows[index:index + 2]
            actual = y0 + max(0.0, min(1.0, (time - x0) / (x1 - x0))) * (y1 - y0)
            self.assertAlmostEqual(actual * LBF, newtons, delta=1e-9)
        # The native sine ramp removes exactly this first-phase impulse.
        area = sum((x1 - x0) * (y0 + y1) / 2 for (x0, y0), (x1, y1) in zip(rows, rows[1:])) * LBF
        self.assertAlmostEqual(area - 25000.0 * deficit, 148012.0, delta=1e-7)

    def test_pulse_time_table_rejects_unsupported_start_and_collapsed_time_knots(self):
        from hangar.storebodies import _ignition_time_thrust_rows
        cases = ([{"duration_s": 0.01, "thrust_kn": 25.0}, {"duration_s": 1.0, "thrust_kn": 0.0}],
                 [{"duration_s": 4.0, "thrust_kn": 25.0}, {"duration_s": 1e-18, "thrust_kn": 0.0}],
                 [{"duration_s": 1e15, "thrust_kn": 25.0}, {"duration_s": 0.001, "thrust_kn": 0.0}])
        for phases in cases:
            with self.subTest(phases=phases), self.assertRaisesRegex(ValueError, "motor:.*(buildup|time|precision)"):
                _ignition_time_thrust_rows(phases)

    def test_continuous_motor_generation_preserves_committed_bytes(self):
        from hangar import storebodies
        root = pathlib.Path(pipeline.repo_root())
        for name, card_id in (("aim120c", "aim-120c"), ("meteor", "meteor")):
            card = json.loads((root / "assets/stores" / (card_id + ".json")).read_text(encoding="utf-8"))
            for path, text in storebodies.files(card).items():
                with self.subTest(model=name, path=path):
                    self.assertEqual(text, (root / "aircraft/stores" / name / path).read_text(encoding="utf-8"))

    def test_motor_input_helper_preserves_valid_declarations(self):
        from hangar.motorinputs import validate_motor
        for smoke in Motor.SMOKE:
            spec = _synthetic_motor() | {"smoke": smoke, "note": "synthetic input only"}
            original = copy.deepcopy(spec)
            self.assertIsNone(validate_motor(spec))
            self.assertEqual(spec, original, "validation must not rewrite authored fields")

    def test_motor_input_helper_preserves_visual_only_and_zero_thrust(self):
        from hangar.motorinputs import validate_motor
        for phases in (None, [], [{"duration_s": 1.0, "thrust_kn": 0.0}]):
            spec = _synthetic_motor()
            if phases is None:
                del spec["phase"]
            else:
                spec["phase"] = phases
            original = copy.deepcopy(spec)
            validate_motor(spec)
            self.assertEqual(spec, original)

    def test_motor_input_helper_rejects_invalid_declarations(self):
        from hangar.motorinputs import validate_motor
        for label, spec in _invalid_synthetic_motors():
            with self.subTest(label=label, spec=spec), self.assertRaisesRegex(ValueError, "motor:"):
                validate_motor(spec)

    def test_motor_input_helper_rejects_unrepresentable_numbers(self):
        from hangar.motorinputs import validate_motor
        for field in ("exit_diameter", "throat_diameter", "cone_length"):
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, "motor:"):
                validate_motor(_synthetic_motor() | {field: 10 ** 400})

    def test_motor_constructor_rejects_invalid_declarations(self):
        for label, spec in _invalid_synthetic_motors():
            with self.subTest(label=label, spec=spec), self.assertRaisesRegex(ValueError, "motor:"):
                Motor(spec)

    def test_motor_generator_rejects_invalid_declarations(self):
        from hangar.storebodies import motor_of
        for label, spec in _invalid_synthetic_motors():
            with self.subTest(label=label, spec=spec), self.assertRaisesRegex(ValueError, "motor:"):
                motor_of({"motor": spec}, 1.0, 0.5, 0.1)

    def test_motor_valid_consumer_defaults_are_preserved(self):
        from hangar.storebodies import motor_of
        spec = _synthetic_motor()
        motor = Motor(spec)
        self.assertEqual(motor.smoke, "reduced")
        self.assertEqual(motor.throat_diameter, 0.4 * spec["exit_diameter"])
        self.assertEqual(motor.cone_length, 1.2 * spec["exit_diameter"])
        generated = motor_of({"motor": spec}, 1.0, 0.5, 0.1)
        self.assertTrue(generated)
        self.assertEqual(generated["burn_s"], 1.0)
        self.assertEqual(spec, _synthetic_motor(), "consumers must preserve the input mapping")

    def test_motor_propulsion_stage_preserves_visual_only_declaration(self):
        from hangar import storestages
        for phases in (None, []):
            with self.subTest(phases=phases):
                spec = _synthetic_motor()
                if phases is None:
                    del spec["phase"]
                else:
                    spec["phase"] = phases
                aircraft = types.SimpleNamespace(spec={"motor": spec, "mass": {"empty": 2.0, "empty_cg": [1.0, 0.0, 0.0]}},
                                                 bodies=[types.SimpleNamespace(kind="fuselage", w=[0.2], x=[0.0, 2.0])])
                d = types.SimpleNamespace(aircraft=aircraft, save=lambda stage, data: data)
                # Synthetic motor cases isolate the card-file boundary, not motor validation.
                with mock.patch.object(storestages, "card_of", return_value={}):
                    result = storestages.propulsion(d)
                self.assertNotIn("motor", result)
                self.assertEqual(result["checks"][0]["value"], "none")

    def test_motor_propulsion_stage_rejects_present_invalid_declaration(self):
        from hangar import storestages
        for label, spec in _invalid_synthetic_motors():
            with self.subTest(label=label):
                aircraft = types.SimpleNamespace(spec={"motor": spec, "mass": {"empty": 2.0, "empty_cg": [1.0, 0.0, 0.0]}},
                                                 bodies=[types.SimpleNamespace(kind="fuselage", w=[0.2], x=[0.0, 2.0])])
                d = types.SimpleNamespace(aircraft=aircraft, save=lambda stage, data: data)
                with mock.patch.object(storestages, "card_of", return_value={}), \
                        self.assertRaisesRegex(ValueError, "motor:"):
                    storestages.propulsion(d)

    def test_motor_burn_commands_ignition_before_stepping_and_closes_world(self):
        from hangar import storebodies, storestages
        aircraft = types.SimpleNamespace(spec={"mass": {"empty": 2.0, "empty_cg": [0.5, 0.0, 0.0]}},
                                         bodies=[types.SimpleNamespace(kind="fuselage", w=[0.2])])
        design = types.SimpleNamespace(name="synthetic-burn", aircraft=aircraft)
        motor = {"propellant_kg": 0.25, "burn_s": 0.1, "isp": 245.0}
        for refuse in (False, True):
            with self.subTest(ignition_refused=refuse):
                events = []

                def ignite(**command):
                    events.append(("ignite", command))
                    if refuse:
                        raise RuntimeError("synthetic ignition refusal")

                def step(count):
                    self.assertEqual(events[0], ("ignite", {"throttle": 1}))
                    events.append(("step", count))

                vehicle = types.SimpleNamespace(command_actuator=ignite, mass_kg=1.75,
                    state=types.SimpleNamespace(diverged=False, velocity_ned_ms=(450.0, 0.0, 0.0)))
                world = types.SimpleNamespace(create_vehicle=mock.Mock(return_value=vehicle), step_seconds=0.05,
                                              step=step, close=lambda: events.append(("close",)))
                with mock.patch.object(storestages, "_world", return_value=(None, world)), \
                        mock.patch.object(storestages, "card_of", return_value={"model": "synthetic-model"}), \
                        mock.patch.object(storebodies, "card_motor_of", return_value=motor) as selected_motor:
                    result = storestages._burn(design, {"model": "synthetic-model"})
                selected_motor.assert_called_once_with({"model": "synthetic-model"}, aircraft.spec, 2.0, 0.5, 0.1)
                self.assertEqual(events[0], ("ignite", {"throttle": 1}))
                self.assertEqual(events[-1], ("close",))
                self.assertEqual(sum(event[0] == "close" for event in events), 1)
                self.assertEqual(sum(event[0] == "ignite" for event in events), 1)
                if refuse:
                    self.assertFalse(any(event[0] == "step" for event in events))
                    self.assertEqual(result["checks"][-1]["status"], "fail")
                    self.assertIn("synthetic ignition refusal", result["checks"][-1]["note"])
                else:
                    self.assertTrue(any(event[0] == "step" for event in events))
                    self.assertTrue(result["burn"])
                    self.assertTrue(all(check["status"] == "pass" for check in result["checks"]))

    def test_motor_input_helper_rejects_derived_and_accumulated_overflow(self):
        from hangar.motorinputs import validate_motor
        cases = {
            "thrust conversion": [{"duration_s": 1.0, "thrust_kn": 1e308}],
            "phase impulse": [{"duration_s": 1e308, "thrust_kn": 1.0}],
            "total burn duration": [{"duration_s": 1e308, "thrust_kn": 0.0}] * 2,
            "total impulse": [{"duration_s": 1.0, "thrust_kn": 9e304}] * 2,
        }
        for label, phases in cases.items():
            with self.subTest(label=label), self.assertRaisesRegex(ValueError, "motor:.*(thrust|impulse|duration)"):
                validate_motor(_synthetic_motor() | {"phase": phases})

    def test_motor_generator_rejects_invalid_installation_quantities(self):
        from hangar.storebodies import motor_of
        cases = [("launch mass", value, 0.5, 0.1) for value in
                 (-1.0, 0.0, True, float("nan"), float("inf"), 10 ** 400)]
        cases += [("launch CG", 1.0, value, 0.1) for value in
                  (True, float("nan"), float("inf"), 10 ** 400)]
        cases += [("body radius", 1.0, 0.5, value) for value in
                  (-0.1, 0.0, True, float("nan"), float("inf"), 10 ** 400)]
        for label, mass, cg, radius in cases:
            with self.subTest(label=label, mass=mass, cg=cg, radius=radius), self.assertRaisesRegex(ValueError, "motor:"):
                motor_of({"motor": _synthetic_motor()}, mass, cg, radius)

    def test_motor_generator_rejects_unrepresentable_grain_and_exhaust(self):
        from hangar.storebodies import motor_of
        for radius in (1e-250, 1e-100, 1e200):
            with self.subTest(radius=radius), self.assertRaisesRegex(ValueError, "motor:.*grain"):
                motor_of({"motor": _synthetic_motor()}, 1.0, 0.5, radius)
        for diameter in (1e-200, 1e200):
            spec = _synthetic_motor() | {"exit_diameter": diameter, "throat_diameter": 0.4 * diameter,
                                         "cone_length": 0.1}
            with self.subTest(diameter=diameter), self.assertRaisesRegex(ValueError, "motor:.*exit area"):
                motor_of({"motor": spec}, 1.0, 0.5, 0.1)

    def test_motor_generator_rejects_propellant_without_positive_structure(self):
        from hangar.storebodies import G0, ISP_PLAIN, motor_of
        propellant = 2.0 / (ISP_PLAIN * G0)  # the existing 2 N minimum sustained for one second
        for mass in (0.5 * propellant, propellant):
            with self.subTest(mass=mass), self.assertRaisesRegex(ValueError, "motor:.*(propellant|structure)"):
                motor_of({"motor": _synthetic_motor()}, mass, 0.5, 0.1)
        spec = _synthetic_motor() | {"phase": [{"duration_s": 1e308, "thrust_kn": 0.0}]}
        with self.assertRaisesRegex(ValueError, "motor:.*impulse"):
            motor_of({"motor": spec}, 1e308, 0.5, 0.1)

    def test_motor_generator_rejects_impossible_structure_inertia_and_grain_fit(self):
        from hangar.storebodies import motor_of
        base = {"aircraft": {"category": "missile"}, "motor": _synthetic_motor(),
                "body": [{"stations": [{"x": 0.0}, {"x": 2.0}]}],
                "mass": {"gyration": [0.1, 0.3, 0.3]}}
        for gyration in ([0.0, 0.3, 0.3], [1e-8, 0.3, 0.3], [0.1, 1e-8, 0.3],
                         [0.1, 0.3, float("nan")], [1e308, 0.3, 0.3]):
            design = copy.deepcopy(base)
            design["mass"]["gyration"] = gyration
            with self.subTest(gyration=gyration), self.assertRaisesRegex(ValueError, "motor:.*(inertia|gyration)"):
                motor_of(design, 1.0, 0.5, 0.1)
        with self.assertRaisesRegex(ValueError, "motor:.*grain.*(fit|body)"):
            motor_of(base, 1.0, 0.5, 1e-5)
        with self.assertRaisesRegex(ValueError, "motor:.*(CG|moment|structure)"):
            motor_of(base, 1e308, 1e308, 0.1)

    def test_motor_invalid_physical_state_fails_before_generated_xml(self):
        from hangar import storeaero, storebodies
        with open(os.path.join(pipeline.repo_root(), "assets", "stores", "aim-120c.json"), encoding="utf-8") as f:
            card = json.load(f)
        geo = storeaero.geometry(card)
        geo["design"] = copy.deepcopy(geo["design"])
        geo["design"]["mass"]["gyration"][0] = 1e-12
        with mock.patch.object(storeaero, "geometry", return_value=geo), self.assertRaisesRegex(ValueError, "motor:.*inertia"):
            storebodies.files(card)

    def test_motor_generator_rejects_malformed_physical_body_dimensions(self):
        from hangar.storebodies import motor_of
        bodies = ([], [{"stations": []}], [{"stations": [{"x": 0.0}, {"x": 0.0}]}],
                  [{"stations": [{"x": 0.0}, {"x": float("nan")}]}])
        for body in bodies:
            design = {"aircraft": {"category": "missile"}, "motor": _synthetic_motor(),
                      "body": body, "mass": {"gyration": [0.1, 0.3, 0.3]}}
            with self.subTest(body=body), self.assertRaisesRegex(ValueError, "motor:.*body"):
                motor_of(design, 1.0, 0.5, 0.1)

    def test_motor_generator_preserves_zero_thrust_coast_semantics(self):
        from hangar.storebodies import G0, ISP_PLAIN, LBF, motor_of
        spec = _synthetic_motor() | {"phase": [{"duration_s": 0.5, "thrust_kn": 0.0},
                                              {"duration_s": 0.75, "thrust_kn": 0.004}]}
        original = copy.deepcopy(spec)
        motor = motor_of({"motor": spec}, 1.0, 0.5, 0.1)
        self.assertEqual(motor["burn_s"], 1.25)
        self.assertAlmostEqual(motor["propellant_kg"], 4.0 / (ISP_PLAIN * G0))
        self.assertEqual(motor["rows"][0][1], 2.0 / LBF)
        self.assertEqual(motor["rows"][-1][1], 4.0 / LBF)
        self.assertEqual(spec, original)

    def test_motor_generator_rejects_unrepresentable_consumed_mass_grid(self):
        from hangar.storebodies import motor_of
        cases = {
            "boundary offset exceeds next phase": [
                {"duration_s": 1.0, "thrust_kn": 0.004},
                {"duration_s": 1e-4, "thrust_kn": 0.004}],
            "first phase collapses in emitted precision": [
                {"duration_s": 1e-5, "thrust_kn": 0.004}],
            "large consumed mass loses boundary offset": [
                {"duration_s": 1e15, "thrust_kn": 0.004},
                {"duration_s": 1e15, "thrust_kn": 0.004}],
        }
        for label, phases in cases.items():
            spec = _synthetic_motor() | {"phase": phases}
            with self.subTest(label=label), self.assertRaisesRegex(ValueError, "motor:.*(table|grid|precision)"):
                motor_of({"motor": spec}, 1e14, 0.5, 1.0)

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
        perf = _tool("viewer_perf_test", "tools/validation/viewer_perf/viewer_perf.py")
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
        perf = _tool("viewer_perf_test", "tools/validation/viewer_perf/viewer_perf.py")
        with tempfile.TemporaryDirectory() as folder:
            cache = pathlib.Path(folder) / "shots"
            cache.mkdir()
            (cache / "hit.fsrec").write_bytes(b"obsolete mechanical recording")
            args = types.SimpleNamespace(out=folder, build=os.path.join(folder, "missing-build"))
            with self.assertRaisesRegex(RuntimeError, "native"):
                perf.recording("hit", args)

    def test_hit_reference_uses_native_target_and_fuze_time(self):
        perf = _tool("viewer_perf_test", "tools/validation/viewer_perf/viewer_perf.py")
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
        perf = _tool("viewer_perf_test", "tools/validation/viewer_perf/viewer_perf.py")
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
        perf = _tool("viewer_perf_test", "tools/validation/viewer_perf/viewer_perf.py")
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
        plume = _tool("plume_check_test", "tools/validation/plume_check/check.py")
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
        plume = _tool("plume_check_test", "tools/validation/plume_check/check.py")
        with tempfile.TemporaryDirectory() as folder:
            out = pathlib.Path(folder) / "old-scripted-frame.png"
            plume.Image.new("RGB", (4, 4)).save(out)
            with mock.patch.object(plume.subprocess, "run", return_value=types.SimpleNamespace(returncode=0)):
                with self.assertRaisesRegex(RuntimeError, "screenshot"):
                    plume.render("native.fsrec", out, 17.25, True, slot=6)
            self.assertFalse(out.exists(), "old mechanical capture must not survive as native evidence")

    def test_plume_rejects_undecodable_png(self):
        plume = _tool("plume_check_test", "tools/validation/plume_check/check.py")
        with tempfile.TemporaryDirectory() as folder:
            out = pathlib.Path(folder) / "broken.png"
            def capture(command, **kwargs):
                out.write_bytes(b"not a PNG")
                return types.SimpleNamespace(returncode=0)
            with mock.patch.object(plume.subprocess, "run", side_effect=capture):
                with self.assertRaisesRegex(RuntimeError, "screenshot"):
                    plume.render("native.fsrec", out, 17.25, True, slot=6)

    def test_plume_rejects_failed_viewer_process(self):
        plume = _tool("plume_check_test", "tools/validation/plume_check/check.py")
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
