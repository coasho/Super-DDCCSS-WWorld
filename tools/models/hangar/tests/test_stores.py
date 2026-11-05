"""Every store is a design the pipeline builds (hangar/storestages.py, docs/hangar.md, Stores): each card names one, a
design is what its writer writes, the card's figures are the design's, and its flight model and its model for the
viewer are what the stages make. (What the platform does with it - carried, released, followed to the ground - is the
`fly` stage's: `python -m hangar stores`.)"""
import json
import io
import math
import os
from pathlib import Path
import struct
import tempfile
import types
import unittest
import xml.etree.ElementTree as ET
from contextlib import contextmanager, redirect_stderr, redirect_stdout
from unittest import mock

from hangar import pipeline, storedesigns, storemodels, stores, storestages

ROOT = pipeline.repo_root()


def cards():
    out = {}
    d = os.path.join(ROOT, "assets", "stores")
    for f in sorted(os.listdir(d)):
        if f.endswith(".json"):
            with open(os.path.join(d, f), encoding="utf-8") as h:
                c = json.load(h)
            out[c["id"]] = c
    return out


def design(name):
    d = pipeline.Design(os.path.join(ROOT, "aircraft", name, name + ".toml"), log=lambda *a, **k: None)
    d.save = lambda stage, data: data  # (the checks, not the cache: nothing is written)
    return d


def glb_triangles(path):
    with open(path, "rb") as f:
        data = f.read()
    magic, version, length = struct.unpack("<III", data[:12])
    assert magic == 0x46546C67 and version == 2 and length == len(data), path
    n = struct.unpack_from("<I", data, 12)[0]
    j = json.loads(data[20:20 + n])
    return sum(j["accessors"][p["indices"]]["count"] // 3 for m in j["meshes"] for p in m["primitives"])


def evidence_fixture(folder):
    root = Path(folder)
    card = "cards/sample.json"
    inputs = [card, "design/sample.toml", "tools/writer.py"]
    artifacts = ["output/sample.xml", "output/companion.xml"]
    data = {"id": "sample", "model": "jsbsim:sample", "parameters": {
        "dimension": {"value": 1.0, "unit": "m", "basis": "published", "source": "fixture"}},
        "sources": {"fixture": {"title": "Synthetic metadata fixture", "publisher": "Test fixture",
                                "url": "https://example.invalid/reference"}}}
    for name in inputs + artifacts:
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data) if name == card else "synthetic fixture\n", encoding="utf-8")
    return root, card, inputs, artifacts








def test_catalogue_fixture(folder):
    """An explicitly linked synthetic catalogue, with no physical model claims."""
    root = Path(folder)
    files = {
        "aircraft/plane/plane.toml": '[aircraft]\nname = "plane"\ncategory = "special mission"\nstands_in_for = ["stock"]\n',
        "aircraft/object/object.toml": '[aircraft]\nname = "object"\ncategory = "store"\ncard = "sample"\n',
        "aircraft/plane/payloads.json": json.dumps({"aircraft": "plane", "payloads": [{"card": "rack"}],
            "loadouts": [{"name": "sample load", "stations": [{"station": 1, "store": "sample"}]}]}),
        "assets/payloads/rack.json": json.dumps({"id": "rack", "family": "store_station", "stations": [{"number": 1}]}),
        "assets/stores/sample.json": json.dumps({"id": "sample", "class": "cargo", "model": "jsbsim:object"}),
        "aircraft/stores/object/object.xml": '<fdm_config><propulsion><engine file="sample_motor"><thruster file="sample_nozzle"/></engine></propulsion></fdm_config>',
        "aircraft/stores/object/Engines/sample_motor.xml": '<rocket_engine/>',
        "aircraft/stores/object/Engines/sample_nozzle.xml": '<nozzle/>',
        "aircraft/models.txt": 'stock plane 0 0 0\n',
        "third_party/jsbsim/aircraft/stock/stock.xml": '<fdm_config/>',
    }
    for name, content in files.items():
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")
    return root


class StoresTest(unittest.TestCase):
    def test_generated_store_mass_preserves_authored_material(self):
        """Serialization cannot create mass when a real store leaves its station."""
        population = cards()
        self.assertEqual(len(population), 53)
        for cid, card in population.items():
            with self.subTest(model=cid):
                generated = storemodels.files(card)
                self.assertTrue(generated)
                model = card['model'].split(':', 1)[1]
                root = ET.fromstring(generated[model + '.xml'])
                def kg(node):
                    self.assertIsNotNone(node)
                    unit = node.get('unit', 'LBS')
                    self.assertIn(unit, ('LBS', 'KG'))
                    return float(node.text) * (0.45359237 if unit == 'LBS' else 1.)
                material = kg(root.find('mass_balance/emptywt'))
                material += sum(kg(node) for node in root.findall('mass_balance/pointmass/weight'))
                material += sum(kg(node) for node in root.findall('propulsion/tank/contents'))
                authored = card['parameters']['mass_kg']['value']
                # External tank fuel is loaded separately by World::flyReleases.
                self.assertLessEqual(abs(material-authored), 64*math.ulp(max(1., authored)))

    def test_all_nonmissile_designs_reproduce_the_authored_shared_inputs(self):
        for name in storedesigns.cards():
            text,_=storedesigns.design(name)
            self.assertEqual(text,(Path(ROOT)/'aircraft'/name/(name+'.toml')).read_text(encoding='utf-8'),name)

    def test_catalogue_complete_chain_is_deterministic_and_read_only(self):
        from hangar import catalogue
        with tempfile.TemporaryDirectory() as folder:
            root = test_catalogue_fixture(folder)
            with mock.patch.object(Path, "write_text", side_effect=AssertionError("audit writes")), \
                    mock.patch.object(pipeline, "Design", side_effect=AssertionError("audit constructs a design")):
                result = catalogue.audit(root)
                self.assertEqual(result, catalogue.audit(root))
            self.assertTrue(result["structural_passed"], result["issues"])
            self.assertEqual(result["schema"], "fsim.store-catalogue.v1")
            self.assertEqual(result["source_assessment"], "not_assessed")
            self.assertEqual(result["installation_assessment"], "not_assessed")
            self.assertEqual(result["counts"], {"authored_aircraft": 1, "store_cards": 1, "loadouts": 1,
                                              "referenced_stores": 1, "stock_definitions": 1})
            self.assertEqual(result["stores"][0]["design"], "aircraft/object/object.toml")
            self.assertEqual(result["stores"][0]["generated_references"], [
                "aircraft/stores/object/Engines/sample_motor.xml", "aircraft/stores/object/Engines/sample_nozzle.xml"])
            self.assertEqual(json.loads(json.dumps(result, allow_nan=False)), result)

    def test_catalogue_missing_links_fail_explicitly(self):
        from hangar import catalogue
        cases = [("aircraft/plane/payloads.json", "missing-fit"),
                 ("assets/payloads/rack.json", "missing-payload-card"),
                 ("assets/stores/sample.json", "missing-store-card"),
                 ("aircraft/object/object.toml", "missing-store-design"),
                 ("aircraft/stores/object/object.xml", "missing-generated-model"),
                 ("aircraft/stores/object/Engines/sample_motor.xml", "missing-generated-reference"),
                 ("aircraft/stores/object/Engines/sample_nozzle.xml", "missing-generated-reference")]
        for path, code in cases:
            with self.subTest(path=path), tempfile.TemporaryDirectory() as folder:
                root = test_catalogue_fixture(folder)
                (root / path).unlink()
                result = catalogue.audit(root)
                self.assertFalse(result["structural_passed"])
                self.assertIn(code, {issue["code"] for issue in result["issues"]})

    def test_catalogue_rejects_duplicate_and_mismatched_identities(self):
        from hangar import catalogue
        with tempfile.TemporaryDirectory() as folder:
            root = test_catalogue_fixture(folder)
            (root / "assets/stores/copy.json").write_text(
                '{"id":"sample","model":"jsbsim:object"}', encoding="utf-8")
            (root / "aircraft/copy").mkdir()
            (root / "aircraft/copy/copy.toml").write_text(
                '[aircraft]\nname="copy"\ncategory="store"\ncard="sample"\n', encoding="utf-8")
            result = catalogue.audit(root)
            codes = {issue["code"] for issue in result["issues"]}
            self.assertFalse(result["structural_passed"])
            self.assertIn("card-identity-mismatch", codes)
            self.assertIn("duplicate-card-id", codes)
            self.assertIn("duplicate-store-design", codes)

    def test_catalogue_rejects_unsafe_references_before_reading(self):
        from hangar import catalogue
        for identity in ("../outside", ".codex", "bad/name", "bad\\name", "C:escape", None, True):
            with self.subTest(identity=identity), tempfile.TemporaryDirectory() as folder:
                root = test_catalogue_fixture(folder)
                path = root / "aircraft/plane/payloads.json"
                fit = json.loads(path.read_text(encoding="utf-8"))
                fit["loadouts"][0]["stations"][0]["store"] = identity
                path.write_text(json.dumps(fit), encoding="utf-8")
                result = catalogue.audit(root)
                self.assertFalse(result["structural_passed"])
                self.assertIn("invalid-identity", {issue["code"] for issue in result["issues"]})

    def test_catalogue_rejects_malformed_shapes_and_duplicate_keys(self):
        from hangar import catalogue
        cases = [("assets/stores/sample.json", '[]'),
                 ("assets/stores/sample.json", '{"id":"sample","id":"sample","model":"jsbsim:object"}'),
                 ("assets/stores/sample.json", '{"id":"sample","model":"jsbsim:../outside"}'),
                 ("aircraft/plane/payloads.json", '{"aircraft":"plane","payloads":{},"loadouts":[]}'),
                 ("aircraft/plane/payloads.json", '{"aircraft":"plane","payloads":[],"loadouts":[false]}')]
        for path, text in cases:
            with self.subTest(path=path, text=text), tempfile.TemporaryDirectory() as folder:
                root = test_catalogue_fixture(folder)
                (root / path).write_text(text, encoding="utf-8")
                result = catalogue.audit(root)
                self.assertFalse(result["structural_passed"], result)

    def test_catalogue_validates_station_numbers_and_assignments(self):
        from hangar import catalogue
        for station in (True, 0, "1", 2):
            with self.subTest(station=station), tempfile.TemporaryDirectory() as folder:
                root = test_catalogue_fixture(folder)
                path = root / "aircraft/plane/payloads.json"
                fit = json.loads(path.read_text(encoding="utf-8"))
                fit["loadouts"][0]["stations"][0]["station"] = station
                path.write_text(json.dumps(fit), encoding="utf-8")
                self.assertFalse(catalogue.audit(root)["structural_passed"])
        with tempfile.TemporaryDirectory() as folder:
            root = test_catalogue_fixture(folder)
            path = root / "aircraft/plane/payloads.json"
            fit = json.loads(path.read_text(encoding="utf-8"))
            fit["loadouts"][0]["stations"] *= 2
            path.write_text(json.dumps(fit), encoding="utf-8")
            self.assertIn("duplicate-loadout-station", {i["code"] for i in catalogue.audit(root)["issues"]})

    def test_catalogue_preserves_pending_without_inheriting_alias_fit(self):
        from hangar import catalogue
        with tempfile.TemporaryDirectory() as folder:
            root = test_catalogue_fixture(folder)
            fit_path = root / "aircraft/plane/payloads.json"
            fit = json.loads(fit_path.read_text(encoding="utf-8"))
            fit["loadouts"] = []
            fit_path.write_text(json.dumps(fit), encoding="utf-8")
            (root / "aircraft/models.txt").write_text("", encoding="utf-8")
            result = catalogue.audit(root)
            self.assertTrue(result["structural_passed"], result["issues"])
            self.assertIsNone(result["stock_definitions"][0]["fit"])
            self.assertEqual(result["stock_definitions"][0]["visual_alias"], "plane")
            self.assertTrue({"unused-store-card", "no-loadouts", "stock-applicability-pending",
                             "visual-alias-unregistered"}.issubset({p["code"] for p in result["pending"]}))
            (root / "aircraft/stock").mkdir()
            fit["aircraft"] = "stock"
            (root / "aircraft/stock/payloads.json").write_text(json.dumps(fit), encoding="utf-8")
            result = catalogue.audit(root)
            self.assertEqual(result["stock_definitions"][0]["fit"], "aircraft/stock/payloads.json")
            self.assertEqual(result["counts"]["authored_aircraft"], 1)

    def test_catalogue_rejects_dangling_visual_target(self):
        from hangar import catalogue
        with tempfile.TemporaryDirectory() as folder:
            root = test_catalogue_fixture(folder)
            with (root / "aircraft/models.txt").open("a", encoding="utf-8") as stream:
                stream.write("extra missing 0 0 0\n")
            result = catalogue.audit(root)
            self.assertFalse(result["structural_passed"])
            self.assertIn("missing-visual-target", {item["code"] for item in result["issues"]})

    def test_catalogue_prunes_excluded_directories_before_descent(self):
        from hangar import catalogue
        with tempfile.TemporaryDirectory() as folder:
            root = test_catalogue_fixture(folder)
            original_scandir, original_lstat, original_read = os.scandir, Path.lstat, Path.read_text
            parents = {root / "aircraft", root / "third_party/jsbsim/aircraft"}

            def permitted(path):
                self.assertNotIn(".codex", str(path).casefold().replace("\\", "/").split("/"))

            @contextmanager
            def test_catalogue_scandir(path):
                permitted(path)
                with original_scandir(path) as entries:
                    listed = list(entries)
                    if Path(path) in parents:
                        listed.append(types.SimpleNamespace(name=".CoDeX"))
                    yield iter(listed)

            def test_catalogue_lstat(path, *args, **kwargs):
                permitted(path)
                return original_lstat(path, *args, **kwargs)

            def test_catalogue_read(path, *args, **kwargs):
                permitted(path)
                return original_read(path, *args, **kwargs)

            with mock.patch.object(os, "scandir", side_effect=test_catalogue_scandir), \
                    mock.patch.object(Path, "lstat", test_catalogue_lstat), \
                    mock.patch.object(Path, "read_text", test_catalogue_read):
                result = catalogue.audit(root)
            self.assertTrue(result["structural_passed"], result["issues"])
            self.assertEqual(result["counts"]["stock_definitions"], 1)

    def test_catalogue_empty_denominators_fail(self):
        from hangar import catalogue
        for relative, code in (("aircraft/plane/plane.toml", "empty-aircraft-roster"),
                               ("assets/stores/sample.json", "empty-store-catalogue")):
            with self.subTest(relative=relative), tempfile.TemporaryDirectory() as folder:
                root = test_catalogue_fixture(folder)
                (root / relative).unlink()
                result = catalogue.audit(root)
                self.assertFalse(result["structural_passed"])
                self.assertIn(code, {i["code"] for i in result["issues"]})

    def test_catalogue_rejects_unsafe_generated_references(self):
        from hangar import catalogue
        for name in ("../escape", ".codex", "Engines/file", "C:escape"):
            with self.subTest(name=name), tempfile.TemporaryDirectory() as folder:
                root = test_catalogue_fixture(folder)
                (root / "aircraft/stores/object/object.xml").write_text(
                    '<fdm_config><propulsion><engine file="%s"/></propulsion></fdm_config>' % name, encoding="utf-8")
                self.assertFalse(catalogue.audit(root)["structural_passed"])

    def test_catalogue_preserves_legacy_stock_definitions_separately(self):
        from hangar import catalogue
        with tempfile.TemporaryDirectory() as folder:
            root = test_catalogue_fixture(folder)
            (root / "third_party/jsbsim/aircraft/template.xml").write_text('<FDM_CONFIG/>', encoding="utf-8")
            result = catalogue.audit(root)
            self.assertEqual(result["counts"]["stock_definitions"], 1)
            self.assertEqual(result["legacy_stock_definitions"][0]["path"], "third_party/jsbsim/aircraft/template.xml")
            self.assertIn("legacy-stock-applicability-pending", {i["code"] for i in result["pending"]})

    def test_catalogue_cli_reports_pending_and_fails_structural_errors(self):
        from hangar import catalogue
        with tempfile.TemporaryDirectory() as folder:
            root = test_catalogue_fixture(folder)
            stream = io.StringIO()
            with redirect_stdout(stream):
                self.assertEqual(catalogue.main(["--root", str(root)]), 0)
            self.assertIn("pending equipment coverage", stream.getvalue())
            stream = io.StringIO()
            with redirect_stdout(stream):
                self.assertEqual(catalogue.main(["--root", str(root), "--json"]), 0)
            self.assertTrue(json.loads(stream.getvalue())["structural_passed"])
            (root / "aircraft/plane/payloads.json").unlink()
            with redirect_stdout(io.StringIO()):
                self.assertEqual(catalogue.main(["--root", str(root)]), 3)

    def test_catalogue_stock_auxiliary_xml_is_not_an_aircraft_identity(self):
        from hangar import catalogue
        with tempfile.TemporaryDirectory() as folder:
            root = test_catalogue_fixture(folder)
            path = root / "third_party/jsbsim/aircraft/stock/Systems/Conventional Controls.xml"
            path.parent.mkdir()
            path.write_text('<system/>', encoding="utf-8")
            result = catalogue.audit(root)
            self.assertTrue(result["structural_passed"], result["issues"])
            self.assertEqual(result["counts"]["stock_definitions"], 1)

    def test_catalogue_duplicate_stock_identity_is_not_merged(self):
        from hangar import catalogue
        with tempfile.TemporaryDirectory() as folder:
            root = test_catalogue_fixture(folder)
            path = root / "third_party/jsbsim/aircraft/variant/stock.xml"
            path.parent.mkdir()
            path.write_text('<fdm_config/>', encoding="utf-8")
            result = catalogue.audit(root)
            self.assertFalse(result["structural_passed"])
            self.assertIn("duplicate-stock-id", {i["code"] for i in result["issues"]})
            self.assertEqual(len(result["stock_definitions"]), 2)

    def test_catalogue_excluded_root_is_rejected_before_io(self):
        from hangar import catalogue
        with mock.patch.object(Path, "lstat", side_effect=AssertionError("excluded root reached IO")):
            result = catalogue.audit("some/.CODEX/inputs")
        self.assertFalse(result["structural_passed"])
        self.assertEqual(result["issues"][0]["code"], "invalid-root")

    def test_catalogue_rejects_malformed_visual_offsets(self):
        from hangar import catalogue
        for offset in ("unknown", "NaN", "inf", "-Infinity", "1e9999"):
            with self.subTest(offset=offset), tempfile.TemporaryDirectory() as folder:
                root = test_catalogue_fixture(folder)
                (root / "aircraft/models.txt").write_text("stock plane %s 0 0\n" % offset, encoding="utf-8")
                result = catalogue.audit(root)
                self.assertFalse(result["structural_passed"])
                self.assertIn("malformed-visual-registration", {i["code"] for i in result["issues"]})

    def test_catalogue_rejects_wrong_generated_reference_root(self):
        from hangar import catalogue
        for kind, tag in (("motor", "fdm_config"), ("motor", "nozzle"),
                          ("nozzle", "fdm_config"), ("nozzle", "rocket_engine")):
            with self.subTest(kind=kind, tag=tag), tempfile.TemporaryDirectory() as folder:
                root = test_catalogue_fixture(folder)
                (root / ("aircraft/stores/object/Engines/sample_%s.xml" % kind)).write_text(
                    "<%s/>" % tag, encoding="utf-8")
                result = catalogue.audit(root)
                self.assertFalse(result["structural_passed"])
                self.assertIn("invalid-generated-reference-root", {i["code"] for i in result["issues"]})

    def test_catalogue_accepts_supported_generated_reference_roots(self):
        from hangar import catalogue
        for kind, tags in (("motor", ("piston_engine", "turbine_engine", "turboprop_engine", "rocket_engine",
                                     "electric_engine", "brushless_dc_motor")),
                           ("nozzle", ("propeller", "nozzle", "rotor", "direct"))):
            for tag in tags:
                with self.subTest(kind=kind, tag=tag), tempfile.TemporaryDirectory() as folder:
                    root = test_catalogue_fixture(folder)
                    (root / ("aircraft/stores/object/Engines/sample_%s.xml" % kind)).write_text(
                        "<%s/>" % tag, encoding="utf-8")
                    result = catalogue.audit(root)
                    self.assertTrue(result["structural_passed"], result["issues"])
























    def test_provenance_reports_source_categories_without_admitting_claims(self):
        from hangar import provenance
        with tempfile.TemporaryDirectory() as folder:
            root, card, inputs, artifacts = evidence_fixture(folder)
            data = json.loads((root / card).read_text())
            data["sources"]["fixture"]["evidence_kind"] = "game_reference"
            data["sources"]["unused"] = {"evidence_kind": "government_publication"}
            (root / card).write_text(json.dumps(data), encoding="utf-8")
            captured = provenance.capture(root, inputs, artifacts, card=card)
            self.assertEqual(captured["source_assessment"], "not_assessed")
            self.assertEqual(captured["card"]["source_kinds"], {
                "fixture": "game_reference", "unused": "government_publication"})
            self.assertEqual(captured["card"]["parameters"]["dimension"], {
                "basis": "published", "source": "fixture", "source_kind": "game_reference"})

    def test_provenance_source_categories_default_and_reject_unknown_spellings(self):
        from hangar import provenance
        with tempfile.TemporaryDirectory() as folder:
            root, card, inputs, artifacts = evidence_fixture(folder)
            captured = provenance.capture(root, inputs, artifacts, card=card)
            self.assertEqual(captured["card"]["parameters"]["dimension"]["source_kind"], "unclassified")
            data = json.loads((root / card).read_text())
            for kind in ("verified_real", None, [], {}):
                data["sources"]["fixture"]["evidence_kind"] = kind
                (root / card).write_text(json.dumps(data), encoding="utf-8")
                with self.subTest(kind=kind), self.assertRaisesRegex(ValueError, "source category"):
                    provenance.capture(root, inputs, artifacts, card=card)

    def test_missile_catalogue_has_explicit_source_categories(self):
        from hangar import provenance
        cards = Path(pipeline.repo_root()) / "assets" / "stores"
        count = 0
        for path in sorted(cards.glob("*.json")):
            data = json.loads(path.read_text(encoding="utf-8"))
            if data.get("class") != "aam" and data.get("id") != "cj-10a":
                continue
            count += 1
            for name, source in data["sources"].items():
                with self.subTest(card=data["id"], source=name):
                    self.assertIn(source.get("evidence_kind"), provenance.SOURCE_KINDS)
                    if name == "wt-datamine":
                        self.assertEqual(source["evidence_kind"], "game_reference")
        self.assertGreater(count, 0, "the catalogue must not silently become empty")

    def test_provenance_capture_is_deterministic_and_does_not_admit_physics(self):
        from hangar import provenance
        with tempfile.TemporaryDirectory() as folder:
            root, card, inputs, artifacts = evidence_fixture(folder)
            captured = provenance.capture(root, inputs, artifacts, card=card)
            self.assertEqual(captured, provenance.capture(root, inputs[::-1], artifacts[::-1], card=card))
            self.assertEqual(captured["source_assessment"], "not_assessed")
            self.assertEqual(captured["card"]["parameters"]["dimension"]["basis"], "published")
            with mock.patch.object(Path, "write_text", side_effect=AssertionError("verify must be read-only")):
                provenance.verify(root, captured, inputs, artifacts, card=card)

    def test_provenance_detects_changed_and_missing_inputs_and_artifacts(self):
        from hangar import provenance
        for name in ("cards/sample.json", "tools/writer.py", "output/sample.xml", "output/companion.xml"):
            for change in ("modify", "remove"):
                with self.subTest(name=name, change=change), tempfile.TemporaryDirectory() as folder:
                    root, card, inputs, artifacts = evidence_fixture(folder)
                    captured = provenance.capture(root, inputs, artifacts, card=card)
                    path = root / name
                    if change == "remove":
                        path.unlink()
                    else:
                        path.write_bytes(path.read_bytes() + b"changed")
                    with self.assertRaises(ValueError):
                        provenance.verify(root, captured, inputs, artifacts, card=card)

    def test_provenance_requires_the_independent_expected_sets(self):
        from hangar import provenance
        with tempfile.TemporaryDirectory() as folder:
            root, card, inputs, artifacts = evidence_fixture(folder)
            for supplied_inputs, supplied_artifacts in ((inputs[:-1], artifacts), (inputs, artifacts[:-1])):
                captured = provenance.capture(root, supplied_inputs, supplied_artifacts, card=card)
                with self.assertRaisesRegex(ValueError, "expected.*set"):
                    provenance.verify(root, captured, inputs, artifacts, card=card)

    def test_provenance_rejects_empty_expected_groups(self):
        from hangar import provenance
        with tempfile.TemporaryDirectory() as folder:
            root, card, inputs, artifacts = evidence_fixture(folder)
            for supplied_inputs, supplied_artifacts in (([], artifacts), (inputs, [])):
                with self.subTest(inputs=supplied_inputs, artifacts=supplied_artifacts), self.assertRaises(ValueError):
                    provenance.capture(root, supplied_inputs, supplied_artifacts)

    def test_provenance_cli_rejects_excluded_manifest_before_io(self):
        from hangar import provenance
        for operation in ("capture", "verify"):
            for manifest in (".codex/blocked.json", "some/.CODEX/blocked.json"):
                with self.subTest(operation=operation, manifest=manifest), \
                        mock.patch.object(Path, "resolve", side_effect=AssertionError("excluded path reached resolve")), \
                        mock.patch.object(provenance, "_load", side_effect=AssertionError("excluded path reached read")), \
                        mock.patch.object(Path, "write_text", side_effect=AssertionError("excluded path reached write")), \
                        redirect_stderr(io.StringIO()):
                    self.assertEqual(provenance.main([operation, "--root", ".", "--manifest", manifest,
                                                     "--input", "fixture.txt", "--artifact", "output.txt"]), 3)

    def test_provenance_rejects_unresolved_citations_and_card_identity(self):
        from hangar import provenance
        for key, value in (("id", "another-card"), ("source", "missing-citation"), ("basis", "unlabelled")):
            with self.subTest(key=key), tempfile.TemporaryDirectory() as folder:
                root, card, inputs, artifacts = evidence_fixture(folder)
                data = json.loads((root / card).read_text())
                if key == "id":
                    data[key] = value
                else:
                    data["parameters"]["dimension"][key] = value
                (root / card).write_text(json.dumps(data), encoding="utf-8")
                with self.assertRaises(ValueError):
                    provenance.capture(root, inputs, artifacts, card=card)

    def test_provenance_rejects_ambiguous_and_out_of_scope_paths(self):
        from hangar import provenance
        for extra in ("../outside", ".codex/forbidden", "tools/../writer.py", "C:/outside", "tools\\writer.py",
                      "TOOLS/writer.py", "tools/writer.py"):
            with self.subTest(extra=extra), tempfile.TemporaryDirectory() as folder:
                root, card, inputs, artifacts = evidence_fixture(folder)
                with self.assertRaises(ValueError):
                    provenance.capture(root, inputs + [extra], artifacts, card=card)

    def test_provenance_rejects_modified_or_malformed_manifests(self):
        from hangar import provenance
        for change in ("digest", "duplicate", "unknown-field", "schema"):
            with self.subTest(change=change), tempfile.TemporaryDirectory() as folder:
                root, card, inputs, artifacts = evidence_fixture(folder)
                captured = provenance.capture(root, inputs, artifacts, card=card)
                if change == "digest":
                    captured["snapshot_sha256"] = "0" * 64
                elif change == "duplicate":
                    captured["inputs"].append(captured["inputs"][0])
                elif change == "unknown-field":
                    captured["unreviewed"] = True
                else:
                    captured["schema"] = "future-schema"
                with self.assertRaises(ValueError):
                    provenance.verify(root, captured, inputs, artifacts, card=card)

    def test_provenance_cli_capture_verify_and_failure(self):
        from hangar import provenance
        with tempfile.TemporaryDirectory() as folder:
            root, card, inputs, artifacts = evidence_fixture(folder)
            manifest = root / "snapshot.json"
            options = ["--root", str(root), "--manifest", str(manifest), "--card", card]
            for name in inputs:
                options += ["--input", name]
            for name in artifacts:
                options += ["--artifact", name]
            transcript = io.StringIO()
            with redirect_stdout(transcript), redirect_stderr(transcript):
                self.assertEqual(provenance.main(["capture"] + options), 0)
                self.assertEqual(provenance.main(["verify"] + options), 0)
                (root / artifacts[0]).write_text("modified output", encoding="utf-8")
                self.assertEqual(provenance.main(["verify"] + options), 3)
            self.assertIn("not model acceptance", transcript.getvalue())

    def test_batch_nonzero_child_cannot_pass_with_old_passing_reports(self):
        for code in (0, 1, 7):
            with self.subTest(code=code), tempfile.TemporaryDirectory() as folder:
                root, card, _, _ = evidence_fixture(folder)
                design_dir = root / "aircraft/sample"
                (design_dir / "out").mkdir(parents=True)
                (design_dir / "sample.toml").write_text('[aircraft]\ncard = "sample"\n', encoding="utf-8")
                (design_dir / "out/build.json").write_text('{"checks": [{"status": "pass", "name": "old"}]}')
                target = root / "assets/stores/sample.json"
                target.parent.mkdir(parents=True)
                target.write_text((root / card).read_text(), encoding="utf-8")
                child = types.SimpleNamespace(returncode=code, poll=lambda: code, wait=lambda: code)
                report = root / "record.md"
                with mock.patch.object(storestages, "designs", return_value={"sample": str(design_dir / "sample.toml")}), \
                        mock.patch.object(storestages.pipeline, "repo_root", return_value=str(root)), \
                        mock.patch.object(storestages.subprocess, "Popen", return_value=child) as spawn, \
                        mock.patch("platform.platform", return_value="test-platform"), \
                        mock.patch.object(storestages, "record", return_value=[]), redirect_stdout(io.StringIO()):
                    result = storestages.main(["build", "--jobs", "1", "--record", str(report)])
                spawn.assert_called_once()
                self.assertEqual(spawn.call_args.args[0][1:], ["-m", "hangar", "sample", "build"])
                self.assertEqual(result, 0 if code == 0 else 1)
                recorded = json.loads(report.with_suffix(".json").read_text())
                self.assertEqual(recorded["child_exit_codes"], {"sample": code})
                self.assertEqual(recorded["batch_passed"], code == 0)
                self.assertIn("test-platform", report.read_text())
                self.assertIn("Batch result: " + ("PASS" if code == 0 else "FAIL"), report.read_text())

    def test_every_card_has_its_design(self):
        designs = storestages.designs()
        named = {}
        for name in designs:
            named[storestages.card_of(design(name))["id"]] = name
        for cid, card in cards().items():
            with self.subTest(cid):
                self.assertIn(cid, named, "no design in aircraft/ names the card %s ([aircraft] card)" % cid)
                # the name the platform makes it by, the viewer draws it by and its design is filed under: one
                self.assertEqual(card["model"], "jsbsim:" + named[cid])
        self.assertEqual(len(named), len(designs), "two designs name one card")
        self.assertEqual(sorted(named), sorted(cards()), "a design names a card that is not in assets/stores")

    def test_written_as_committed(self):
        # a tank's, a pod's, a bomb's and a cargo store's design is its class's rule: the committed file is what it writes
        for name in storedesigns.cards():
            toml, paint = storedesigns.design(name)
            for fn, text in ((name + ".toml", toml), ("paint.toml", paint)):
                path = os.path.join(ROOT, "aircraft", name, fn)
                with open(path, encoding="utf-8") as f:
                    self.assertEqual(f.read(), text, "%s is not what `python -m hangar storedesigns %s` writes" % (path, name))

    def test_card_figures_are_the_designs(self):
        # its size, its mass and what its aircraft feels while it carries it (its drag area, its Munk volume, its fins'
        # side force): the card's are the design's - the stages' own checks, none failed
        for name in storestages.designs():
            d = design(name)
            card = storestages.card_of(d)
            par = {k: v["value"] for k, v in card["parameters"].items() if isinstance(v, dict) and "value" in v}
            with self.subTest(name):
                a = d.aircraft
                length = max(float(b.x[-1]) for b in a.bodies) - min(float(b.x[0]) for b in a.bodies)
                self.assertAlmostEqual(length, par["length_m"], delta=0.005 * par["length_m"])
                across = max(float(w) for b in a.bodies if b.kind == "fuselage" for w in b.w)
                self.assertAlmostEqual(across, par.get("width_m" if "width_m" in par else "diameter_m"), delta=0.005 * across)
                for stage in (storestages.mass, storestages.aero):
                    failed = [c["name"] for c in stage(d)["checks"] if c["status"] == "fail"]
                    self.assertEqual(failed, [], "%s: %s" % (name, stage.__name__))

    def test_flight_models(self):
        # every store's flight model is what its design's build stage writes - a tank's, a pod's and a cargo store's
        # from its card, a missile's and a bomb's from its design, a missile's motor and nozzle with it - and nothing
        # else is in its folder
        for name in storestages.designs():
            d = design(name)
            card = storestages.card_of(d)
            path, written = storestages.flight_model(card)
            self.assertTrue(written)
            out = storemodels.files(card)
            with self.subTest(name):
                self.assertIsNotNone(out, "%s: nothing to write its flight model from" % name)
                folder = os.path.dirname(path)
                for rel, text in out.items():
                    with open(os.path.join(folder, *rel.split("/")), encoding="utf-8", newline="") as f:
                        self.assertEqual(f.read(), text, "%s/%s is not what `python -m hangar %s build` writes" % (folder, rel, name))
                kept = {os.path.relpath(os.path.join(r, f), folder).replace(os.sep, "/") for r, _, fs in os.walk(folder) for f in fs}
                self.assertEqual(kept, set(out) | {card["sources"]["hangar-vlm"]["url"]})
                empty, held, _, _ = storestages._xml_mass(out[name + ".xml"])
                self.assertAlmostEqual(empty + held, card["parameters"]["mass_kg"]["value"], delta=0.02)
                source=d.aircraft.spec
                if source.get('engine'):
                    # Air-breathing fuel is an authored, fed tank inventory;
                    # engine presence alone does not imply nonzero fuel fill.
                    native=ET.fromstring(out[name+'.xml'])
                    feeds={int(feed.text) for feed in native.findall('propulsion/engine/feed')}
                    tanks=source.get('mass',{}).get('tank',[])
                    self.assertEqual(feeds,set(range(len(tanks))))
                    fuel=sum(t['capacity']*t.get('fill',1.) for i,t in enumerate(tanks) if i in feeds)
                    self.assertAlmostEqual(held,fuel,delta=.02)
                    self.assertEqual(held>0.,fuel>0.)
                else:self.assertEqual(held > 0.0, "motor" in source)  # rocket propellant, or an unpowered body

    def test_a_store_before_a_stock_aircraft(self):
        # the Mk 82 a card names is hangar's, built from its design: the platform finds it before JSBSim's own mk82
        self.assertTrue(os.path.isfile(os.path.join(ROOT, "aircraft", "stores", "mk82", "mk82.xml")))
        with open(os.path.join(ROOT, "aircraft", "stores", "mk82", "mk82.xml"), encoding="utf-8") as f:
            self.assertIn("Written by hangar", f.read(300))

    def test_models(self):
        superseded = os.path.join(ROOT, "assets", "models")
        for name in storestages.designs():
            d = design(name)
            card = storestages.card_of(d)
            glb = os.path.join(d.dir, name + ".glb")
            with self.subTest(name):
                self.assertTrue(os.path.isfile(glb) and os.path.isfile(glb + ".manifest"), "%s: python -m hangar %s model" % (glb, name))
                triangles = glb_triangles(glb)
                most = storedesigns.TRIANGLES.get(card["class"])
                if most:
                    # drawn on every station that carries one: no aircraft's budget
                    self.assertLessEqual(triangles, most)
                    self.assertEqual(d.aircraft.triangles, most)
                # its picture is its flight model's: its origin its CG, its length and its width its card's - on the
                # file's own vertices (the viewer hangs its origin where the simulation has the store)
                failed = [c["name"] + ": " + c["note"] for c in storestages.picture_checks(d, card, glb) if c["status"] == "fail"]
                self.assertEqual(failed, [], name)
                # the viewer looks in assets/models before a design's folder: nothing there may stand in for its model
                self.assertFalse(os.path.exists(os.path.join(superseded, name + ".glb")), "assets/models/%s.glb shadows its design's" % name)

    def test_cargo_drawn_as_committed(self):
        # a cargo store's picture is stores.py's box, the same bytes each time it is written
        for name, card in stores.cargo_cards().items():
            with tempfile.TemporaryDirectory() as tmp:
                with open(stores.write_cargo(name, card, os.path.join(tmp, name + ".glb")), "rb") as f:
                    data = f.read()
            committed = os.path.join(ROOT, "aircraft", name, name + ".glb")
            with open(committed, "rb") as f:
                self.assertEqual(f.read(), data, "%s is not what `python -m hangar %s model` writes" % (committed, name))


if __name__ == "__main__":
    unittest.main()
