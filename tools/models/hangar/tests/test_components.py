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
    def test_migration_preserves_declared_new_mechanisms(self):
        import json,tomllib
        from pathlib import Path
        from hangar import pipeline
        root=Path(pipeline.repo_root())
        for name in ('aim-9x','meteor','pl-10'):
            raw=json.loads((root/'assets/stores'/(name+'.json')).read_text(encoding='utf-8'))
            model=raw['model'].removeprefix('jsbsim:')
            design=tomllib.loads((root/'aircraft'/model/(model+'.toml')).read_text(encoding='utf-8'))
            proposal=components.migration(raw,design)
            for role in ('autopilot','propulsion'):
                self.assertEqual(proposal['components'][role],raw['component_models']['components'][role])
        absent=components.migration({'id':'missing','model':'jsbsim:missing','parameters':{}},
            {'motor':{'kind':'ducted_rocket','phase':[{'duration_s':1.,'thrust_kn':1.}],
                      'exit':[1.,0.,0.],'exit_diameter':.1}})
        self.assertEqual(absent['components']['propulsion']['availability'],'unavailable')

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
            # A following FCS replacement may reindent the separator outside
            # this subtree. Every byte within the physical subtree still agrees.
            left, right = before.find(node), after.find(node)
            left.tail = right.tail = None
            self.assertEqual(ET.tostring(left), ET.tostring(right), node)
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
        required_guided=set();aam=set()
        for path in sorted((Path(pipeline.repo_root()) / "assets/stores").glob("*.json")):
            raw = json.loads(path.read_text(encoding="utf-8"))
            if raw.get('class') in ('aam','cruise_missile'):required_guided.add(raw['id'])
            if raw.get('class')=='aam':aam.add(raw['id'])
            rows = raw["component_models"]["components"]
            if rows["guidance"]["availability"] != "available":
                continue
            guided.append(raw["id"])
            with self.subTest(store=raw["id"]):
                self.assertEqual(rows["autopilot"]["bindings"]["execution_mode"], "configured")
                vector=bool(rows['autopilot'].get('bindings',{}).get('thrust_vector'))
                self.assertEqual(len(rows["autopilot"]["parameter_refs"]), 7 if vector else 5)
                if vector:self.assertEqual(rows['autopilot']['bindings']['thrust_vector'],'jsbsim_nozzle_v1')
                self.assertNotIn("legacy", rows["fuze"]["implementation_id"])
                self.assertNotIn("legacy", rows["warhead"]["implementation_id"])
        self.assertEqual(len(aam),24)
        self.assertEqual(set(guided),required_guided,'every authored guided class requires guidance, including air-breathing stores')

    def test_authored_airbreathing_hardware_cannot_migrate_to_not_applicable(self):
        raw={'id':'synthetic','class':'cruise_missile','model':'jsbsim:synthetic','parameters':{}}
        design={'engine':[{'type':'turbofan'}],'store_propulsion':{'version':1}}
        for row in (None,{'availability':'not_applicable','reason':'missing implementation'},
                    {'availability':'unavailable','reason':'missing selection'}):
            card=copy.deepcopy(raw)
            if row is not None:card['component_models']={'schema_version':1,'components':{'propulsion':row}}
            with self.subTest(row=row):
                self.assertEqual(components.migration(card,design)['components']['propulsion']['availability'],'unavailable')

    def test_typed_airbreathing_source_refuses_missing_or_incompatible_selection(self):
        from hangar import turbofanstore
        raw={'id':'synthetic','class':'cruise_missile','model':'jsbsim:synthetic','parameters':{}}
        design={'engine':[{'type':'turbofan'}],'store_propulsion':{'version':1}}
        available=dict(availability='available',implementation_id=components.PROPULSION,
            configuration_version=1,parameter_refs={},bindings={},basis='assumed',note='Fixture',validity='Fixture')
        for row in (None,{'availability':'not_applicable','reason':'not selected'},
                    {'availability':'unavailable','reason':'not implemented'},available):
            card=copy.deepcopy(raw)
            if row is not None:card['component_models']={'schema_version':1,'components':{'propulsion':row}}
            with self.subTest(row=row),self.assertRaises(ValueError):turbofanstore.inputs(card,design)
        for changed in ({'engine':[{'type':'turbofan'}],'store_propulsion':{'version':2}},
                        {'engine':[],'store_propulsion':{'version':1}},
                        {'engine':[{'type':'turbofan'},{'type':'turbofan'}],'store_propulsion':{'version':1}},
                        {'engine':[{'type':'turbofan'}]}):
            with self.subTest(design=changed),self.assertRaises(ValueError):turbofanstore.inputs(raw,changed)
        self.assertIsNone(turbofanstore.inputs(raw,{}),'legacy non-engine source behavior remains unchanged')
        mixed=copy.deepcopy(design);mixed['motor']={'phase':[{'duration_s':1.,'thrust_kn':1.}]}
        with self.assertRaisesRegex(ValueError,'mixed rocket'):turbofanstore.inputs(raw,mixed)

    def test_retained_legacy_store_profiles_preserve_existing_generated_numerics(self):
        import json
        from pathlib import Path
        import xml.etree.ElementTree as ET
        from hangar import pipeline, storemodels,storeaero,storestages
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
        legacy_names=set();typed_names=set()
        for path in paths:
            with self.subTest(store=path.stem):
                selected = json.loads(path.read_text(encoding="utf-8"))
                self.assertIn("component_models", selected, "every maintained store must declare its providers")
                # Compatibility gate for the retained original implementations.
                # New physical mechanisms have separate source/native tests;
                # removing their descriptors must never enable them implicitly.
                rows=selected['component_models']['components']
                autopilot=rows['autopilot']
                geometry=storeaero.geometry(selected) or {}
                design=geometry.get('design',{})
                if design.get('engine') or 'store_propulsion' in design:
                    # A newly typed physical source has no descriptor-free
                    # legacy representation. Refuse missing selection rather
                    # than silently generating an unpowered asset.
                    legacy=copy.deepcopy(selected);legacy.pop('component_models')
                    card_bytes=path.read_bytes()
                    with self.assertRaises(ValueError):storemodels.files(legacy)
                    after=storemodels.files(selected)
                    self.assertIsNotNone(after)
                    self.assertEqual(after,storemodels.files(selected),'typed native generation is deterministic')
                    self.assertEqual(path.read_bytes(),card_bytes,'text generation cannot mutate source cards')
                    model_name=selected['model'].removeprefix('jsbsim:')
                    model=ET.fromstring(after[model_name+'.xml'])
                    engines=model.findall('propulsion/engine');tanks=design['mass']['tank']
                    self.assertEqual(len(engines),len(design['engine']))
                    self.assertEqual(len(model.findall('propulsion/tank')),len(tanks))
                    for engine in engines:
                        engine_path='Engines/'+engine.get('file')+'.xml'
                        nozzle_path='Engines/'+engine.find('thruster').get('file')+'.xml'
                        self.assertIn(engine_path,after);self.assertIn(nozzle_path,after)
                        self.assertEqual(ET.fromstring(after[engine_path]).tag,'turbine_engine')
                        self.assertEqual(ET.fromstring(after[nozzle_path]).tag,'direct')
                        self.assertEqual({int(n.text) for n in engine.findall('feed')},set(range(len(tanks))))
                    governor=model.find("flight_control/property[.='fsim/cruise-governor-v1/version']")
                    self.assertIsNotNone(governor);self.assertEqual(governor.get('value'),'1')
                    empty,held,_,_=storestages._xml_mass(after[model_name+'.xml'])
                    fuel=sum(t['capacity']*t.get('fill',1.) for t in tanks)
                    self.assertAlmostEqual(held,fuel,delta=.02)
                    self.assertAlmostEqual(empty+held,selected['parameters']['mass_kg']['value'],delta=.02)
                    typed_names.add(path.stem);checked+=1
                    continue
                if autopilot.get('bindings',{}).get('thrust_vector'):
                    del autopilot['bindings']['thrust_vector']
                    for role in ('tvc_limit_rad','tvc_rate_rad_s'):del autopilot['parameter_refs'][role]
                if autopilot.get('implementation_id')==components.ACTUATOR_AWARE_AUTOPILOT:
                    autopilot['implementation_id']=components.AUTOPILOT
                if rows['propulsion'].get('implementation_id')==components.DUCTED_PROPULSION:
                    from hangar import ductedrocket
                    rows['propulsion']['implementation_id']=components.PROPULSION
                    for role in ductedrocket.ROLES:del rows['propulsion']['parameter_refs'][role]
                legacy = copy.deepcopy(selected)
                legacy.pop("component_models")
                card_bytes=path.read_bytes()
                if design.get('motor',{}).get('kind')=='ducted_rocket':
                    # The retained legacy fixture predates the explicitly typed
                    # ducted design as well as its component descriptor.
                    from unittest import mock
                    geometry=copy.deepcopy(geometry)
                    geometry['design']['motor'].pop('kind',None)
                    with mock.patch.object(storeaero,'geometry',return_value=geometry):
                        before,after=storemodels.files(legacy),storemodels.files(selected)
                else:before, after = storemodels.files(legacy), storemodels.files(selected)
                self.assertEqual(path.read_bytes(),card_bytes,"text generation must not publish or mutate source cards")
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
                legacy_names.add(path.stem)
        self.assertEqual(checked, len(paths), "every discovered store must complete legacy equivalence or the explicit typed-source contract")
        self.assertFalse(legacy_names & typed_names)
        self.assertEqual(legacy_names | typed_names,{p.stem for p in paths},'all discovered stores require legacy or typed-source contract coverage')

    def test_nonfinned_stage_uses_shared_vlm_for_three_classes(self):
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
                self.assertEqual(after["kind"], "airframe")
                self.assertEqual(after["method"], "vlm")

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
