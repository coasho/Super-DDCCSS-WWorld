"""Payload plant configuration contracts; synthetic values identify no weapon."""
import copy
import unittest

from hangar import components


def card():
    return {"id": "synthetic", "model": "jsbsim:synthetic", "parameters": {
        "lag": {"value": .08, "unit": "s"}, "rate": {"value": 3.25, "unit": "rad/s"}},
        "component_models": {"schema_version": 1, "components": {"actuator": {
            "availability": "available", "implementation_id": components.ACTUATOR,
            "configuration_version": 1, "parameter_refs": {"lag_s": "lag", "rate_rad_s": "rate"},
            "bindings": {"model": "jsbsim:synthetic"}, "basis": "assumed", "note": "Synthetic plant",
            "validity": "Engineering model only"}}}}


class ComponentsTest(unittest.TestCase):
    def test_representative_sensor_selection_does_not_change_generated_physical_assets(self):
        import json
        from pathlib import Path
        from hangar import pipeline, storemodels
        root = Path(pipeline.repo_root())
        for store in ("aim-9m", "aim-9x", "aim-7m", "aim-120c"):
            with self.subTest(store=store):
                selected = json.loads((root / "assets/stores" / (store + ".json")).read_text(encoding="utf-8"))
                legacy = copy.deepcopy(selected)
                rows = legacy["component_models"]["components"]
                rows["seeker"]["implementation_id"] = "fsim.seeker.encounter.v1"
                rows["seeker"]["bindings"] = {"mode": legacy["seeker"]}
                rows["guidance"]["implementation_id"] = "fsim.guidance.velocity_normal_pn.v1"
                rows["guidance"]["bindings"] = {}
                self.assertEqual(storemodels.files(selected), storemodels.files(legacy))

    def test_resolved_profile_is_immutable_and_consumes_nondefault_actuator(self):
        raw = card()
        selected = components.resolve(raw, "actuator")
        self.assertEqual(components.actuator(raw)["lag_coefficient"], 12.5)
        self.assertEqual(components.actuator(raw)["rate_rad_s"], 3.25)
        before = components.fingerprint(selected)
        raw["parameters"]["lag"]["value"] = .04
        self.assertEqual(components.number(selected, "lag_s", "s"), .08)
        self.assertEqual(components.fingerprint(selected), before)
        self.assertNotEqual(components.fingerprint(components.resolve(raw, "actuator")), before)

    def test_explicit_invalid_never_becomes_absent_legacy(self):
        for defect in range(8):
            raw = card()
            row = raw["component_models"]["components"]["actuator"]
            if defect == 0: raw["component_models"]["schema_version"] = 2
            if defect == 1: row["implementation_id"] = "unknown"
            if defect == 2: del raw["parameters"]["lag"]
            if defect == 3: raw["parameters"]["lag"]["unit"] = "ms"
            if defect == 4: raw["parameters"]["rate"]["value"] = 0
            if defect == 5: row["bindings"]["model"] = "jsbsim:other"
            if defect == 6: row["parameter_refs"]["extra"] = "lag"
            if defect == 7: row["configuration_version"] = True
            with self.subTest(defect=defect), self.assertRaises(ValueError):
                components.actuator(raw)
        self.assertIsNone(components.resolve({"parameters": {}}, "actuator"))

    def test_declared_unavailable_is_not_an_unpowered_store(self):
        raw = card()
        raw["component_models"]["components"]["actuator"] = {
            "availability": "unavailable", "reason": "Native plant not built"}
        with self.assertRaisesRegex(ValueError, "unavailable"):
            components.actuator(raw)

    def test_motor_consumes_selected_phase_and_grain_inputs_without_mutation(self):
        design = {"motor": {"exit": [1., 0., 0.], "exit_diameter": .1, "throat_diameter": .04,
                    "cone_length": .12, "smoke": "reduced", "phase": [{"duration_s": 1., "thrust_kn": .001}]}}
        raw = {"id": "synthetic", "model": "jsbsim:synthetic", "parameters": {}}
        proposal = components.migration(raw, design)
        raw["parameters"].update(proposal["parameters"])
        raw["component_models"] = {"schema_version": 1, "components": proposal["components"]}
        before = copy.deepcopy(design)
        selected, settings = components.motor(raw, design)
        self.assertEqual(selected, before)
        self.assertEqual(settings["isp_s"], 245)
        self.assertEqual(settings["coast_thrust_n"], 2)
        raw["parameters"]["plant_motor_phase_0_duration_s"]["value"] = 2.
        self.assertEqual(components.motor(raw, design)[0]["motor"]["phase"][0]["duration_s"], 2.)
        from hangar import storebodies
        generated = storebodies.card_motor_of(raw, design, 1., .5, .1)
        self.assertEqual(generated["propellant_kg"], 4. / (245. * 9.80665))
        self.assertEqual(design, before)

    def test_fingerprint_has_cross_language_stable_fixture(self):
        selected = components.resolve(card(), "actuator")
        self.assertIsInstance(components.fingerprint(selected), int)
        self.assertEqual(components.fingerprint(selected), 3348686125)

    def test_migrating_existing_inputs_preserves_motor_nozzle_and_physics_xml(self):
        import json
        from pathlib import Path
        import tomllib
        import xml.etree.ElementTree as ET
        from hangar import pipeline, storemodels
        root = Path(pipeline.repo_root())
        raw = json.loads((root / "assets/stores/aim-120c.json").read_text(encoding="utf-8"))
        raw.pop("component_models", None)
        design = tomllib.loads((root / "aircraft/aim120c/aim120c.toml").read_text(encoding="utf-8"))
        legacy = storemodels.files(raw)
        selected = copy.deepcopy(raw)
        proposal = components.migration(selected, design)
        selected["parameters"].update(proposal["parameters"])
        selected["component_models"] = {"schema_version": 1, "components": proposal["components"]}
        configured = storemodels.files(selected)
        for asset in ("Engines/aim120c_motor.xml", "Engines/aim120c_nozzle.xml"):
            self.assertEqual(legacy[asset], configured[asset])
        before, after = ET.fromstring(legacy["aim120c.xml"]), ET.fromstring(configured["aim120c.xml"])
        for node in ("metrics", "mass_balance", "propulsion", "aerodynamics", "ground_reactions"):
            self.assertEqual(ET.tostring(before.find(node)), ET.tostring(after.find(node)), node)
        metadata = {n.text: float(n.get("value")) for n in after.findall("./flight_control/property")}
        for kind in proposal["components"]:
            if proposal["components"][kind]["availability"] == "available":
                self.assertEqual(metadata["fsim/components/%s/profile-fingerprint" % kind],
                                 components.fingerprint(components.resolve(selected, kind)))

    def test_migration_declares_explicit_control_without_fabricating_execution_or_missing_guidance(self):
        raw = {"id": "synthetic", "class": "aam", "model": "jsbsim:synthetic", "parameters": {}}
        missing = components.migration(raw, None)
        self.assertEqual(missing["components"]["autopilot"]["availability"], "unavailable")
        selected = components.migration(raw, {"surface": [{"controls": [{}]}]})
        self.assertNotIn("weapon_execution_version", selected["parameters"])
        self.assertEqual(selected["components"]["autopilot"]["bindings"]["execution_mode"], "configured")
        self.assertEqual(len(selected["components"]["autopilot"]["parameter_refs"]), 5)

    def test_all_guided_cards_choose_explicit_control_and_descriptive_effect_models(self):
        import json
        from pathlib import Path
        from hangar import pipeline
        guided = []
        for path in sorted((Path(pipeline.repo_root()) / "assets/stores").glob("*.json")):
            raw = json.loads(path.read_text(encoding="utf-8"))
            rows = raw["component_models"]["components"]
            if rows["guidance"]["availability"] != "available":
                continue
            guided.append(raw["id"])
            with self.subTest(store=raw["id"]):
                self.assertEqual(rows["autopilot"]["bindings"]["execution_mode"], "configured")
                self.assertEqual(len(rows["autopilot"]["parameter_refs"]), 5)
                self.assertNotIn("legacy", rows["fuze"]["implementation_id"])
                self.assertNotIn("legacy", rows["warhead"]["implementation_id"])
        self.assertEqual(len(guided), 24)

    def test_all_store_profiles_preserve_existing_generated_numerics(self):
        import json
        from pathlib import Path
        import xml.etree.ElementTree as ET
        from hangar import pipeline, storemodels
        root = Path(pipeline.repo_root())
        paths = sorted((root / "assets/stores").glob("*.json"))
        self.assertGreater(len(paths), 0, "the equivalence gate must visit the actual store catalogue")

        def normalize(xml, old_properties):
            model = ET.fromstring(xml)
            properties = {n.text: n.get("value") for n in model.findall("./flight_control/property")}
            for control in model.findall("./flight_control"):
                for node in list(control):
                    if node.tag == "property" and (node.text or "").startswith("fsim/components/"):
                        control.remove(node)
                    elif node.tag == "property" and node.text == "fsim/missile-control/actuator-version" and node.text not in old_properties:
                        control.remove(node)
            for node in model.iter():
                if node.tag in ("lag", "rate_limit") and (node.text or "").strip() in properties:
                    node.text = properties[node.text.strip()]
            # Only indentation is insignificant. Preserve all attributes,
            # element order and the complete remaining text, including tables.
            def value(node):
                return (node.tag, tuple(sorted(node.attrib.items())), (node.text or "").strip(),
                        tuple(value(child) for child in node))
            return value(model)

        checked = 0
        for path in paths:
            with self.subTest(store=path.stem):
                selected = json.loads(path.read_text(encoding="utf-8"))
                self.assertIn("component_models", selected, "every maintained store must declare its providers")
                legacy = copy.deepcopy(selected)
                legacy.pop("component_models")
                before, after = storemodels.files(legacy), storemodels.files(selected)
                self.assertIsNotNone(before, "existing store must have a native generated model")
                self.assertIsNotNone(after, "explicit profile cannot lose a native generated model")
                self.assertEqual(set(before), set(after), "component migration cannot replace the native asset inventory")
                for asset in before:
                    if asset.startswith("Engines/"):
                        self.assertEqual(before[asset], after[asset], asset)
                    else:
                        old_model = ET.fromstring(before[asset])
                        properties = {n.text for n in old_model.findall("./flight_control/property")}
                        self.assertEqual(normalize(before[asset], properties), normalize(after[asset], properties), asset)
                checked += 1
        self.assertEqual(checked, len(paths), "every discovered store must complete the strict equivalence comparison")

    def test_nonfinned_stage_diagnostics_preserve_legacy_values_for_three_classes(self):
        import json
        from pathlib import Path
        from unittest import mock
        from hangar import pipeline, storestages
        root = Path(pipeline.repo_root())
        cards = [json.loads(p.read_text(encoding="utf-8")) for p in sorted((root / "assets/stores").glob("*.json"))]
        for kind in ("tank", "pod", "cargo"):
            with self.subTest(kind=kind):
                raw = next(c for c in cards if c["class"] == kind)
                name = raw["model"].removeprefix("jsbsim:")
                design = pipeline.Design(str(root / "aircraft" / name / (name + ".toml")), log=lambda *a, **k: None)
                design.save = lambda stage, data: data
                legacy = copy.deepcopy(raw); legacy.pop("component_models")
                with mock.patch.object(storestages, "card_of", return_value=legacy):
                    before = storestages.aero(design)
                with mock.patch.object(storestages, "card_of", return_value=raw):
                    after = storestages.aero(design)
                self.assertEqual(before, after, "profile selection cannot alter existing diagnostic values/checks")
                self.assertEqual(after["coefficient_authority"], "face_drag_diagnostic" if kind == "cargo" else "geometry_diagnostic")
                self.assertEqual(after["native_aerodynamics_source"], "bluff_cargo" if kind == "cargo" else "slender_store")

    def test_nonfinned_stage_validates_explicit_descriptor_before_early_return(self):
        import json
        from pathlib import Path
        from unittest import mock
        from hangar import pipeline, storestages
        root = Path(pipeline.repo_root())
        cards = [json.loads(p.read_text(encoding="utf-8")) for p in sorted((root / "assets/stores").glob("*.json"))]
        for kind in ("tank", "pod", "cargo"):
            raw = next(c for c in cards if c["class"] == kind)
            name = raw["model"].removeprefix("jsbsim:")
            design = pipeline.Design(str(root / "aircraft" / name / (name + ".toml")), log=lambda *a, **k: None)
            design.save = lambda stage, data: data
            for defect in ("source", "model", "design", "unavailable"):
                bad = copy.deepcopy(raw)
                row = bad["component_models"]["components"]["aerodynamics"]
                if defect == "source": row["bindings"]["source"] = "slender_store" if kind == "cargo" else "bluff_cargo"
                if defect == "model": row["bindings"]["model"] = "jsbsim:other"
                if defect == "design": row["bindings"]["design"] = "aircraft/other/other.toml"
                if defect == "unavailable":
                    bad["component_models"]["components"]["aerodynamics"] = {"availability": "unavailable", "reason": "Native model missing"}
                with self.subTest(kind=kind, defect=defect), mock.patch.object(storestages, "card_of", return_value=bad), self.assertRaises(ValueError):
                    storestages.aero(design)


if __name__ == "__main__":
    unittest.main()
