"""Generated finite missile controls, independent geometry and force checks."""
import json
import copy
import math
import re
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
from unittest import mock

from hangar import pipeline, storeaero, storebodies


def card(name):
    return json.loads((Path(pipeline.repo_root()) / "assets" / "stores" / (name + ".json")).read_text(encoding="utf-8"))


class Expressions:
    """Evaluate emitted algebra for synthetic conditions, without JSBSim motion."""
    def __init__(self, model, properties):
        self.functions = {n.get("name"): n for n in model.findall(".//function[@name]")}
        self.fcs = {}
        for node in model.findall(".//fcs_function"):
            self.fcs[node.get("name")] = node
            if node.findtext("output"):
                self.fcs[node.findtext("output")] = node
        self.actuators = {n.findtext("output"): n for n in model.findall(".//actuator")}
        self.gains = {n.findtext("output"): n for n in model.findall(".//pure_gain")}
        self.properties = dict(properties)

    def property(self, name):
        if name in self.properties:
            return self.properties[name]
        if name in self.fcs:
            fcs = self.fcs[name]
            value = self.eval(fcs.find("function"))
            clip = fcs.find("clipto")
            if clip is not None:
                value = max(float(clip.findtext("min")), min(float(clip.findtext("max")), value))
            return value
        if name in self.actuators:
            # Steady actuator positions for algebra checks only. Native tests
            # separately exercise actual finite rate, lag, failure and reset.
            actuator = self.actuators[name]
            value = self.property(actuator.findtext("input"))
            return max(float(actuator.findtext("clipto/min")), min(float(actuator.findtext("clipto/max")), value))
        if name in self.gains:
            node = self.gains[name]
            return float(node.findtext("gain")) * self.property(node.findtext("input"))
        return self.eval(self.functions[name])

    def eval(self, n):
        if n.tag == "value":
            return float(n.text)
        if n.tag == "property":
            return self.property(n.text.strip())
        if n.tag == "table":
            x = self.property(n.findtext("independentVar"))
            rows = [list(map(float, r.split())) for r in n.findtext("tableData").splitlines() if r.strip()]
            if x <= rows[0][0]:
                return rows[0][1]
            for (x0, y0), (x1, y1) in zip(rows, rows[1:]):
                if x <= x1:
                    return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
            return rows[-1][1]
        values = [self.eval(c) for c in n if c.tag != "description"]
        if n.tag == "function":
            return values[0]
        if n.tag == "sum":
            return sum(values)
        if n.tag == "product":
            return math.prod(values)
        if n.tag == "difference":
            return values[0] - values[1]
        if n.tag == "sin":
            return math.sin(values[0])
        if n.tag == "cos":
            return math.cos(values[0])
        raise AssertionError("Unsupported emitted expression " + n.tag)


def evaluate(store, responses=None, alpha=0.0, beta=0.0, qbar=100.0, mach=0.6):
    c = card(store)
    name = c["model"].removeprefix("jsbsim:")
    model = ET.fromstring(storebodies.files(c)[name + ".xml"])
    props = {"aero/alpha-rad": alpha, "aero/beta-rad": beta, "aero/qbar-psf": qbar,
             "velocities/mach": mach, "fcs/elevator-response-rad": 0.0,
             "fcs/rudder-response-rad": 0.0, "fcs/aileron-response-rad": 0.0}
    props.update(responses or {})
    props["metrics/Sw-sqft"] = float(model.findtext("metrics/wingarea"))
    props["metrics/cbarw-ft"] = float(model.findtext("metrics/chord"))
    return model, Expressions(model, props)


def native_actuator_value(model, text):
    """Resolve the actual native numeric literal or published property input."""
    try:
        return float(text)
    except ValueError:
        node = next(n for n in model.findall("./flight_control/property") if n.text == text)
        return float(node.get("value"))


class StoreControlTest(unittest.TestCase):
    def test_card_selected_native_actuator_dynamics_reach_all_fcs_actuators(self):
        c = card("aim-120c")
        for name, value, unit in (("weapon_execution_version", 1, "1"),
                                  ("control_algorithm", 1, "1"), ("control_allocator", 1, "1"),
                                  ("control_actuator_lag_s", .08, "s"),
                                  ("control_actuator_rate_rad_s", 3.25, "rad/s")):
            c["parameters"][name] = {"value": value, "unit": unit}
        model = ET.fromstring(storebodies.files(c)[c["model"].removeprefix("jsbsim:") + ".xml"])
        metadata = {n.text: float(n.get("value")) for n in model.findall("./flight_control/property")}
        self.assertEqual(metadata.get("fsim/missile-control/actuator-version"), 1)
        self.assertEqual(metadata["fsim/missile-control/lag-coefficient-s-1"], 12.5)
        self.assertEqual(metadata["fsim/missile-control/channel-rate-rad_s"], 3.25)
        fins = 0
        for actuator in model.findall("./flight_control/channel/actuator"):
            name = actuator.get("name")
            rate_key = actuator.findtext("rate_limit")
            self.assertIn(rate_key, metadata, "the native FCS must consume the published rate rather than a stale literal")
            self.assertEqual(metadata[rate_key], 3.25)
            if name.startswith("fcs/fin-"):
                fins += 1
            else:
                self.assertEqual(actuator.findtext("lag"), "fsim/missile-control/lag-coefficient-s-1")
        self.assertGreater(fins, 0)

    def test_native_actuator_profile_fingerprint_and_explicit_refusals(self):
        from hangar import storecontrol
        c = {"id": "synthetic", "parameters": {name: {"value": value, "unit": unit}
             for name, value, unit in (("weapon_execution_version", 1, "1"),
                ("control_algorithm", 1, "1"), ("control_allocator", 1, "1"),
                ("control_actuator_lag_s", .08, "s"), ("control_actuator_rate_rad_s", 3.25, "rad/s"))}}
        first = storecontrol.actuator_profile(c)
        self.assertEqual(first, storecontrol.actuator_profile(copy.deepcopy(c)))
        other = copy.deepcopy(c)
        other["parameters"]["control_actuator_rate_rad_s"]["value"] = 2.5
        self.assertNotEqual(first["sha256"], storecontrol.actuator_profile(other)["sha256"])
        for defect in range(7):
            bad = copy.deepcopy(c)
            if defect == 0:
                del bad["parameters"]["weapon_execution_version"]
            elif defect == 1:
                bad["parameters"]["weapon_execution_version"]["value"] = 2
            elif defect == 2:
                del bad["parameters"]["control_actuator_lag_s"]
            elif defect == 3:
                bad["parameters"]["control_actuator_lag_s"]["unit"] = "ms"
            elif defect == 4:
                bad["parameters"]["control_actuator_rate_rad_s"]["value"] = 0
            elif defect == 5:
                bad["parameters"]["control_actuator_lag_s"]["value"] = float("nan")
            else:
                bad["parameters"]["control_allocator"]["value"] = True
            with self.subTest(defect=defect), self.assertRaises(ValueError):
                storecontrol.actuator_profile(bad)
        self.assertEqual(storecontrol.actuator_profile(), {"version": 0, "lag_coefficient": 25, "rate_rad_s": 4})

    def test_unversioned_generator_call_retains_legacy_actuator_literals(self):
        from hangar import storecontrol
        c = card("aim-120c")
        geo = storeaero.geometry(c)
        baseline = storeaero.coefficients(geo, c["id"])
        source = '<fdm_config><flight_control name="old"/></fdm_config>'
        model = ET.fromstring(storecontrol.apply(source, '<flight_control name="old"/>', geo, baseline, 0))
        metadata = {n.text: float(n.get("value")) for n in model.findall("./flight_control/property")}
        self.assertNotIn("fsim/missile-control/actuator-version", metadata)
        for actuator in model.findall("./flight_control/channel/actuator"):
            self.assertEqual(float(actuator.findtext("rate_limit")), 4)
            if not actuator.get("name").startswith("fcs/fin-"):
                self.assertEqual(float(actuator.findtext("lag")), 25)

    def test_signed_normalization_metadata_matches_each_native_axis_range(self):
        model, _ = evaluate("aim-120c")
        metadata = {n.text: float(n.get("value")) for n in model.findall("./flight_control/property")}
        self.assertEqual(metadata.get("fsim/missile-control/normalization-version"), 1)
        for node in model.findall("./flight_control/channel/aerosurface_scale"):
            channel = node.get("name").removeprefix("fcs/").removesuffix("-control")
            for bound in ("min", "max"):
                self.assertEqual(metadata.get("fsim/missile-control/%s-%s-rad" % (channel, bound)),
                                 float(node.findtext("range/" + bound)))

    def test_asymmetric_authored_stops_retain_distinct_signed_metadata(self):
        from hangar import storecontrol
        c = card("aim-120c")
        geo = storeaero.geometry(c)
        for surface in geo["design"]["surface"]:
            for control in surface.get("controls", []):
                control["limits"] = [-10, 30]
        baseline = storeaero.coefficients(geo, c["id"])
        source = '<fdm_config><metrics><wingarea unit="FT2">1</wingarea><wingspan unit="FT">1</wingspan><chord unit="FT">1</chord></metrics><flight_control name="old"/><aerodynamics/></fdm_config>'
        model = ET.fromstring(storecontrol.apply(source, '<flight_control name="old"/>', geo, baseline, 0))
        metadata = {n.text: float(n.get("value")) for n in model.findall("./flight_control/property")}
        different = False
        for node in model.findall("./flight_control/channel/aerosurface_scale"):
            channel = node.get("name").removeprefix("fcs/").removesuffix("-control")
            low, high = float(node.findtext("range/min")), float(node.findtext("range/max"))
            self.assertEqual(metadata["fsim/missile-control/%s-min-rad" % channel], low)
            self.assertEqual(metadata["fsim/missile-control/%s-max-rad" % channel], high)
            different |= low != -high
        self.assertTrue(different, "the synthetic authored design must actually exercise asymmetric travel")

    def test_distinct_authored_surfaces_cannot_share_one_physical_fin_identity(self):
        c = card("aim-120c")
        geo = storeaero.geometry(c)
        duplicate = copy.deepcopy(next(s for s in geo["design"]["surface"] if s.get("controls")))
        geo["design"]["surface"].append(duplicate)
        with self.assertRaisesRegex(ValueError, "identity|collision|duplicate"):
            from hangar import storecontrol
            storecontrol.build(geo, storeaero.coefficients(geo, c["id"]))

    def test_physical_fin_identity_is_stable_and_shared_across_aerodynamic_patches(self):
        from hangar import storecontrol
        c = card("aim-120c")
        geo = storeaero.geometry(c)
        baseline = storeaero.coefficients(geo, c["id"])
        ordinary = storecontrol.build(geo, baseline)
        ids = [fin.get("id") for fin in ordinary["fins"]]
        self.assertTrue(all(isinstance(key, int) and key > 0 for key in ids),
                        "physical fins need nonzero stable authored identities")
        self.assertEqual(len(ids), len(set(ids)))
        original = storecontrol.Surface
        def subdivided(spec):
            surface = original(spec)
            surface.eta_sections = sorted(set(surface.eta_sections) | {0.5})
            return surface
        with mock.patch.object(storecontrol, "Surface", side_effect=subdivided):
            split = storecontrol.build(geo, baseline)
        self.assertEqual(sorted(fin["id"] for fin in split["fins"]), sorted(ids),
                         "a numerical aero-patch split must not create another physical actuator")
        reordered = copy.deepcopy(geo)
        reordered["design"]["surface"].reverse()
        self.assertEqual(sorted(fin["id"] for fin in storecontrol.build(reordered, baseline)["fins"]), sorted(ids),
                         "reordering authored surfaces must preserve fin identity")
        model, _ = evaluate("aim-120c")
        metadata = {node.text: float(node.get("value")) for node in model.findall("./flight_control/property")}
        self.assertEqual(sorted(int(metadata["fsim/missile-control/fin-%d-id" % i]) for i in range(len(ids))), sorted(ids))

    def test_all_physical_fins_jammed_neutral_report_zero_standard_positions(self):
        model, expressions = evaluate("aim-120c")
        # Set actuator states, not command surrogates. This also exercises the
        # previous generator's defect: it exposed virtual channel states as the
        # standard position output even while the actual physical fins were jammed.
        for actuator in model.findall(".//actuator"):
            is_fin = actuator.get("name", "").startswith("fcs/fin-")
            expressions.properties[actuator.findtext("output")] = 0.0 if is_fin else 0.1
        for channel in ("elevator", "rudder", "left-aileron"):
            with self.subTest(channel=channel):
                self.assertEqual(expressions.property("fcs/%s-pos-rad" % channel), 0,
                                 "reported axis positions must come from physical fins rather than filtered demand")
        for axis in "xyz":
            self.assertEqual(expressions.property("aero/force/fsim-fin-%s-lbs" % axis), 0)

    def test_healthy_forward_reverse_actual_positions_reconstruct_supported_axes(self):
        from hangar import missiles
        for row in missiles.MISSILES.values():
            for sign in (-1, 1):
                responses = {"elevator": sign * 0.01, "rudder": -sign * 0.007, "aileron": sign * 0.003}
                model, expressions = evaluate(row["card"], {"fcs/%s-response-rad" % channel: value
                                                            for channel, value in responses.items()})
                metadata = {n.text: float(n.get("value")) for n in model.findall("./flight_control/property")}
                for channel, output in (("elevator", "elevator"), ("rudder", "rudder"), ("aileron", "left-aileron")):
                    expected = responses[channel] if metadata["fsim/missile-control/%s-limit-rad" % channel] > 0 else 0
                    with self.subTest(store=row["card"], sign=sign, channel=channel):
                        self.assertAlmostEqual(expressions.property("fcs/%s-pos-rad" % output), expected, delta=1e-12)
                self.assertEqual(expressions.property("fcs/aileron-pos-rad"),
                                 expressions.property("fcs/left-aileron-pos-rad"))

    def test_generated_property_segments_are_valid_jsbsim_identifiers(self):
        from hangar import missiles
        invalid = []
        checked = 0
        for row in missiles.MISSILES.values():
            c = card(row["card"])
            name = c["model"].removeprefix("jsbsim:")
            model = ET.fromstring(storebodies.files(c)[name + ".xml"])
            paths = [(n.text or "").strip().removeprefix("-") for n in model.iter()
                     if n.tag in ("property", "input", "output", "independentVar")]
            paths += [n.get("name") for n in model.iter()
                      if n.tag in ("function", "fcs_function", "actuator", "pure_gain", "aerosurface_scale")
                      and n.get("name")]
            for path in paths:
                for segment in path.split("/"):
                    checked += 1
                    if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_.-]*(?:\[[0-9]+\])?", segment):
                        invalid.append((name, path, segment))
        self.assertGreater(checked, 1000, "the syntax check must visit actual emitted property references")
        self.assertEqual(invalid, [], "JSBSim property names must begin with an alphabetic character or underscore: %r" % invalid[:6])

    def test_controlled_missile_emits_finite_actuators_and_actual_fin_forces(self):
        model = ET.fromstring(storebodies.files(card("aim-120c"))["aim120c.xml"])
        for channel, output in (("elevator", "elevator"), ("rudder", "rudder"), ("aileron", "left-aileron")):
            with self.subTest(channel=channel):
                actuator = model.find(".//actuator[@name='fcs/%s-actuator']" % channel)
                self.assertIsNotNone(actuator, "released controlled missiles must execute finite JSBSim actuators")
                self.assertGreater(native_actuator_value(model, actuator.findtext("lag")), 0)
                self.assertGreater(native_actuator_value(model, actuator.findtext("rate_limit")), 0)
                self.assertLess(float(actuator.findtext("clipto/min")), 0)
                self.assertGreater(float(actuator.findtext("clipto/max")), 0)
                self.assertEqual(actuator.findtext("output"), "fcs/%s-response-rad" % channel)
        for name in ("aero/force/fsim-fin-x-lbs", "aero/force/fsim-fin-y-lbs", "aero/force/fsim-fin-z-lbs",
                     "aero/moment/fsim-fin-l-lbsft", "aero/moment/fsim-fin-m-lbsft", "aero/moment/fsim-fin-n-lbsft"):
            self.assertIsNotNone(model.find(".//function[@name='%s']" % name), name)

    def test_motor_and_nozzle_outputs_remain_byte_identical(self):
        from hangar import missiles
        root = Path(pipeline.repo_root())
        for store in sorted(row["card"] for row in missiles.MISSILES.values()):
            c = card(store)
            name = c["model"].removeprefix("jsbsim:")
            for path, text in storebodies.files(c).items():
                if path.startswith("Engines/"):
                    with self.subTest(store=store, path=path):
                        self.assertEqual(text.encode("utf-8"), (root / "aircraft" / "stores" / name / path).read_bytes())

    def test_physical_fin_actuators_limit_after_mixing(self):
        model, _ = evaluate("aim-120c")
        fins = model.findall(".//actuator")
        fins = [a for a in fins if a.get("name", "").startswith("fcs/fin-")]
        self.assertEqual(len(fins), 4, "each authored physical mirrored fin requires its own finite actuator")
        for i, actuator in enumerate(fins):
            with self.subTest(fin=i):
                self.assertEqual(actuator.get("name"), "fcs/fin-%d-actuator" % i)
                self.assertEqual(actuator.findtext("input"), "fcs/fin-%d-demand-rad" % i)
                self.assertEqual(actuator.findtext("output"), "fcs/fin-%d-delta-rad" % i)
                self.assertEqual(native_actuator_value(model, actuator.findtext("rate_limit")), 4.0)
                self.assertIsNone(actuator.find("lag"), "channel response already owns the assumed lag")
                self.assertLess(float(actuator.findtext("clipto/min")), 0)
                self.assertGreater(float(actuator.findtext("clipto/max")), 0)

    def test_neutral_controls_do_not_replace_passive_aerodynamics(self):
        for store in ("aim-120c", "aim-9m", "r-73", "r-77m"):
            _, expressions = evaluate(store, alpha=0.2, beta=-0.15)
            for kind, axes, unit in (("force", "xyz", "lbs"), ("moment", "lmn", "lbsft")):
                for axis in axes:
                    with self.subTest(store=store, axis=axis):
                        self.assertEqual(expressions.property("aero/%s/fsim-fin-%s-%s" % (kind, axis, unit)), 0)

    def test_control_forces_and_moments_have_authored_signed_authority(self):
        for store, zsign, ysign in (("aim-120c", -1, 1), ("aim-9m", 1, -1)):
            _, pitch = evaluate(store, {"fcs/elevator-response-rad": 0.01})
            _, yaw = evaluate(store, {"fcs/rudder-response-rad": 0.01})
            self.assertGreater(zsign * pitch.property("aero/force/fsim-fin-z-lbs"), 0)
            self.assertLess(pitch.property("aero/moment/fsim-fin-m-lbsft"), 0)
            self.assertGreater(ysign * yaw.property("aero/force/fsim-fin-y-lbs"), 0)
            self.assertLess(yaw.property("aero/moment/fsim-fin-n-lbsft"), 0)
        _, roll = evaluate("aim-120c", {"fcs/aileron-response-rad": 0.01})
        self.assertGreater(roll.property("aero/moment/fsim-fin-l-lbsft"), 0)
        self.assertAlmostEqual(roll.property("aero/force/fsim-fin-y-lbs"), 0, delta=1e-12)
        self.assertAlmostEqual(roll.property("aero/force/fsim-fin-z-lbs"), 0, delta=1e-12)

    def test_small_signal_export_matches_actual_fin_force_and_moment(self):
        for store in ("aim-120c", "aim-9m", "aim-7m", "r-27r", "r-73", "r-77m"):
            for channel, output, force, moment in (("elevator", "elevator", "z", "m"),
                    ("rudder", "rudder", "y", "n"), ("aileron", "left-aileron", None, "l")):
                with self.subTest(store=store, channel=channel):
                    delta = 1e-7
                    _, e = evaluate(store, {"fcs/%s-response-rad" % channel: delta})
                    qS = e.properties["aero/qbar-psf"] * e.properties["metrics/Sw-sqft"]
                    reference = e.properties["metrics/cbarw-ft"]
                    if force:
                        actual = e.property("aero/force/fsim-fin-%s-lbs" % force) / (qS * delta)
                        self.assertAlmostEqual(actual, e.property("fsim/control/c%s-%s" % (force, channel)), delta=1e-8)
                    actual = e.property("aero/moment/fsim-fin-%s-lbsft" % moment) / (qS * reference * delta)
                    self.assertAlmostEqual(actual, e.property("fsim/control/c%s-%s" % (moment, channel)), delta=1e-8)

    def test_control_increment_cannot_add_aerodynamic_energy(self):
        for alpha, beta in ((0, 0), (0.3, -0.2), (-0.4, 0.15), (1.2, -0.6)):
            for delta in (-0.3, -0.05, 0.05, 0.3):
                with self.subTest(alpha=alpha, beta=beta, delta=delta):
                    _, e = evaluate("aim-120c", {"fcs/elevator-response-rad": delta,
                        "fcs/rudder-response-rad": -delta, "fcs/aileron-response-rad": delta}, alpha=alpha, beta=beta)
                    airflow = (math.cos(alpha) * math.cos(beta), math.sin(beta), math.sin(alpha) * math.cos(beta))
                    work = sum(e.property("aero/force/fsim-fin-%s-lbs" % axis) * component
                               for axis, component in zip("xyz", airflow))
                    self.assertLessEqual(work, 1e-10)

    def test_scheduled_stability_and_damping_preserve_selected_baseline(self):
        c = card("aim-120c")
        geo = storeaero.geometry(c)
        b = storeaero.coefficients(geo, c["id"])
        mapping = {"cz-alpha": ("cna", -1), "cy-beta": ("cyb", -1), "cm-alpha": ("cma", 1),
                   "cn-beta": ("cnb", -1), "cm-q": ("cmq", 1), "cn-r": ("cnr", 1), "cl-p": ("clp", 1)}
        for index, mach in enumerate(b["mach"]):
            _, e = evaluate("aim-120c", mach=mach)
            for output, (source, sign) in mapping.items():
                with self.subTest(mach=mach, output=output):
                    self.assertAlmostEqual(e.property("fsim/control/" + output), sign * b[source][index], delta=1e-12)

    def test_uncontrolled_bomb_output_remains_unchanged(self):
        root = Path(pipeline.repo_root())
        c = card("mk82")
        for path, text in storebodies.files(c).items():
            self.assertEqual(text, (root / "aircraft" / "stores" / "mk82" / path).read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
