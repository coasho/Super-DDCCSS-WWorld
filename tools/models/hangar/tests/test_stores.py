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


def offline_fixture(folder):
    from hangar import storetables
    root = Path(folder)
    spec = {"aircraft": {"name": "fixture", "category": "missile", "card": "fixture"},
            "mass": {"empty": 2.0, "empty_cg": [1.0, 0.0, 0.0], "gyration": [0.1, 0.3, 0.3]},
            "body": [{"kind": "fuselage", "stations": [
                {"x": 0.0, "w": 0.2, "top": 0.1, "bottom": -0.1},
                {"x": 2.0, "w": 0.2, "top": 0.1, "bottom": -0.1}]}],
            "store": {"aero_table": "aero.json"}}
    geo = {"x": [0.0, 2.0], "r": [0.1, 0.1], "fins": [], "cg": 1.0,
           "length": 2.0, "diameter": 0.2, "design": spec,
           "design_path": str(root / "fixture.toml"), "source": "its design"}
    card = {"id": "fixture", "model": "jsbsim:fixture", "class": "aam", "parameters": {
        name: {"value": value} for name, value in (("mass_kg", 2.0), ("length_m", 2.0),
                                                  ("diameter_m", 0.2), ("drag_area_m2", 0.01))}}
    dataset = {"schema": "fsim.store-aero-derivatives.v1", "card": "fixture",
               "geometry_sha256": storetables.geometry_digest(geo),
               "reference": {"area_m2": math.pi * 0.2 ** 2 / 4.0, "diameter_m": 0.2,
                             "moment_x_m": 1.0, "frame": "body-from-first-station"},
               "producer": {"name": "synthetic-fixture", "version": "1", "case": "integration-only"},
               "mach": [0.0, 0.6, 0.6001, 2.0],
               "coefficients": {name: [sign * v for v in (1.0, 2.0, 3.0, 4.0)] for name, sign in (
                   ("ca", 0.1), ("cna", 1.0), ("cma", -1.0), ("cyb", 0.5),
                   ("cnb", -0.5), ("cmq", -2.0), ("cnr", -2.0), ("clp", -1.0))},
               "crossflow": {name: 0.0 for name in ("normal", "pitch", "side", "yaw", "turn_m5")}}
    (root / "aero.json").write_text(json.dumps(dataset), encoding="utf-8")
    return geo, card, dataset


def xml_coefficients(text, name):
    node = ET.fromstring(text).find(".//function[@name='aero/coefficient/%s']//tableData" % name)
    return [[float(value) for value in row.split()] for row in node.text.strip().splitlines()]


def offline_design_fixture(folder):
    root = Path(folder)
    design_dir = root / "aircraft" / "fixture"
    design_dir.mkdir(parents=True)
    _, card, dataset = offline_fixture(design_dir)
    path = design_dir / "fixture.toml"
    path.write_text('''[aircraft]
name = "fixture"
category = "missile"
card = "fixture"
[reference]
area = 0.031415926535897934
span = 0.2
chord = 0.2
aero_point = [1.0, 0.0, 0.0]
[mass]
empty = 2.0
empty_cg = [1.0, 0.0, 0.0]
gyration = [0.1, 0.3, 0.3]
[store]
aero_table = "aero.json"
[[body]]
name = "body"
kind = "fuselage"
stations = [{x=0.0, w=0.2, top=0.1, bottom=-0.1}, {x=2.0, w=0.2, top=0.1, bottom=-0.1}]
''', encoding="utf-8")
    asset = root / "assets" / "stores" / "fixture.json"
    asset.parent.mkdir(parents=True)
    asset.write_text(json.dumps(card), encoding="utf-8")
    return root, path, dataset


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

    def test_offline_aero_changes_generated_coefficients_and_digest(self):
        from hangar import storeaero
        with tempfile.TemporaryDirectory() as folder:
            geo, card, dataset = offline_fixture(folder)
            with mock.patch.object(storeaero, "geometry", return_value=geo):
                first = storemodels.files(card)["fixture.xml"]
                self.assertEqual(xml_coefficients(first, "CN"), list(map(list, zip(dataset["mach"], dataset["coefficients"]["cna"]))))
                self.assertEqual(xml_coefficients(first, "CA"), list(map(list, zip(dataset["mach"], dataset["coefficients"]["ca"]))))
                dataset["coefficients"]["cna"][1] = 7.0
                (Path(folder) / "aero.json").write_text(json.dumps(dataset), encoding="utf-8")
                second = storemodels.files(card)["fixture.xml"]
            self.assertNotEqual(first, second)
            self.assertEqual(xml_coefficients(second, "CN")[1], [0.6, 7.0])
            self.assertIn("Offline derivative-table SHA-256:", first)

    def test_offline_aero_stage_uses_generated_dataset(self):
        from hangar import storeaero, storedrag
        with tempfile.TemporaryDirectory() as folder:
            geo, card, dataset = offline_fixture(folder)
            d = types.SimpleNamespace(name="fixture", aircraft=types.SimpleNamespace(spec=geo["design"]),
                                      save=lambda stage, data: data)
            parts = dict(body=0.01, base=0.0, fins=0.0, frontal=0.03, munk=0.0, slope=0.0, arm=0.0)
            with mock.patch.object(storeaero, "geometry", return_value=geo), \
                    mock.patch.object(storestages, "card_of", return_value=card), \
                    mock.patch.object(storedrag, "build_up", return_value=(0.01, parts)):
                result = storestages.aero(d)
            self.assertEqual(result["mach"], dataset["mach"])
            self.assertEqual(result["cna"], dataset["coefficients"]["cna"])
            self.assertEqual(result["ca"], dataset["coefficients"]["ca"])
            self.assertEqual(result["input"]["producer"], dataset["producer"])

    def test_offline_aero_rejects_bad_dataset_before_writing(self):
        from hangar import storeaero
        changes = [("shape", lambda d: d["coefficients"]["cna"].pop()),
                   ("missing-channel", lambda d: d["coefficients"].pop("cmq")),
                   ("duplicate-axis", lambda d: d["mach"].__setitem__(2, 0.6)),
                   ("nonfinite", lambda d: d["coefficients"]["ca"].__setitem__(0, float("nan"))),
                   ("boolean", lambda d: d["mach"].__setitem__(0, False)),
                   ("reference", lambda d: d["reference"].__setitem__("area_m2", 1.0)),
                   ("geometry", lambda d: d.__setitem__("geometry_sha256", "0" * 64)),
                   ("identity", lambda d: d.__setitem__("card", "another")),
                   ("unknown-field", lambda d: d.__setitem__("typo", 1))]
        for label, change in changes:
            with self.subTest(label=label), tempfile.TemporaryDirectory() as folder:
                geo, card, dataset = offline_fixture(folder)
                change(dataset)
                (Path(folder) / "aero.json").write_text(json.dumps(dataset), encoding="utf-8")
                d = types.SimpleNamespace(name="fixture", aircraft=types.SimpleNamespace(spec=geo["design"]))
                with mock.patch.object(storeaero, "geometry", return_value=geo), \
                        mock.patch.object(storestages, "card_of", return_value=card), \
                        mock.patch.object(storestages, "flight_model", return_value=(str(Path(folder) / "fixture.xml"), True)), \
                        mock.patch.object(pipeline, "write_text") as write:
                    with self.assertRaises(ValueError):
                        storestages.build(d)
                    write.assert_not_called()

    def test_offline_aero_rejects_excluded_or_nonlocal_paths(self):
        from hangar import storetables
        with tempfile.TemporaryDirectory() as folder:
            geo, card, _ = offline_fixture(folder)
            for path in ("../outside.json", ".codex/blocked.json", "some/.CODEX/blocked.json", "C:/outside.json", "a\\b.json"):
                with self.subTest(path=path), mock.patch.object(Path, "resolve", side_effect=AssertionError("excluded path resolved")):
                    with self.assertRaises(ValueError):
                        storetables.load(geo, card["id"], path)

    def test_offline_aero_table_preserves_close_coordinates_and_rejects_truncation(self):
        from hangar import storebodies
        text = storebodies._table([0.6, 0.6001], [1.0, 2.0], 0, precise=True)
        self.assertEqual([float(row.split()[0]) for row in text.splitlines()], [0.6, 0.6001])
        for axis, values in (([0.0, 1.0], [1.0]), ([0.0, 0.0], [1.0, 2.0]), ([0.0, float("inf")], [1.0, 2.0])):
            with self.subTest(axis=axis), self.assertRaises(ValueError):
                storebodies._table(axis, values, 0)

    def test_offline_aero_checks_complete_generated_grid(self):
        template = '<fdm_config><function name="aero/coefficient/CN"><table><independentVar lookup="row">velocities/mach</independentVar><tableData>%s</tableData></table></function></fdm_config>'
        self.assertEqual(storestages._table_error(template % '0 1\n6e-1 2\n', "CN", [0.0, 0.6], [1.0, 2.0]), 0.0)
        for rows in ('0 1\n', '0 1\n0 2\n', '0 1\n0.6 2\n1 3\n'):
            with self.subTest(rows=rows), self.assertRaises(ValueError):
                storestages._table_error(template % rows, "CN", [0.0, 0.6], [1.0, 2.0])

    def test_offline_aero_keeps_unselected_build_up(self):
        from hangar import storeaero
        with tempfile.TemporaryDirectory() as folder:
            geo, card, _ = offline_fixture(folder)
            del geo["design"]["store"]["aero_table"]
            self.assertEqual(storeaero.coefficients(geo, card["id"]), storeaero.build_up(geo))

    def test_offline_aero_build_stage_uses_real_files_and_rejects_stale_report(self):
        with tempfile.TemporaryDirectory() as folder:
            root, path, dataset = offline_design_fixture(folder)
            design_dir = path.parent
            with mock.patch.object(pipeline, "repo_root", return_value=str(root)):
                d = pipeline.Design(str(path), log=lambda *args, **kwargs: None)
                storestages.aero(d)
                built = storestages.build(d)
                self.assertFalse([c for c in built["checks"] if c["status"] == "fail"])
                generated = root / "aircraft" / "stores" / "fixture" / "fixture.xml"
                self.assertEqual(xml_coefficients(generated.read_text(), "CN")[1], [0.6, 2.0])
                dataset["coefficients"]["cna"][1] = 9.0
                (design_dir / "aero.json").write_text(json.dumps(dataset), encoding="utf-8")
                stale = storestages.build(d)
                self.assertTrue(any(c["status"] == "fail" and "input" in c["name"] for c in stale["checks"]))
                storestages.aero(d)
                refreshed = storestages.build(d)
                self.assertFalse([c for c in refreshed["checks"] if c["status"] == "fail"])
                self.assertEqual(xml_coefficients(generated.read_text(), "CN")[1], [0.6, 9.0])

    def test_offline_aero_build_requires_supported_report(self):
        for report in ("missing", "truncated", None, {}, [], True, "unsupported", {"kind": "box"}, {"kind": "unknown"}):
            with self.subTest(report=report), tempfile.TemporaryDirectory() as folder:
                root, path, _ = offline_design_fixture(folder)
                with mock.patch.object(pipeline, "repo_root", return_value=str(root)):
                    d = pipeline.Design(str(path), log=lambda *args, **kwargs: None)
                    if report != "missing":
                        (Path(d.out) / "aero.json").write_text("{" if report == "truncated" else json.dumps(report), encoding="utf-8")
                    built = storestages.build(d)
                failures = [c for c in built["checks"] if c["status"] == "fail"]
                self.assertTrue(failures)
                self.assertTrue(any("aero" in c["note"].lower() for c in failures))

    def test_offline_aero_build_rejects_malformed_input_evidence(self):
        for metadata in (None, [], "wrong", {}, {"sha256": False}):
            with self.subTest(metadata=metadata), tempfile.TemporaryDirectory() as folder:
                root, path, _ = offline_design_fixture(folder)
                with mock.patch.object(pipeline, "repo_root", return_value=str(root)):
                    d = pipeline.Design(str(path), log=lambda *args, **kwargs: None)
                    report = storestages.aero(d)
                    report["input"] = metadata
                    (Path(d.out) / "aero.json").write_text(json.dumps(report), encoding="utf-8")
                    built = storestages.build(d)
                self.assertTrue(any(c["status"] == "fail" and "input" in c["name"] for c in built["checks"]))

    def test_offline_aero_expected_grid_and_values_require_finite_numbers(self):
        text = ('<fdm_config><function name="aero/coefficient/CN"><table>'
                '<independentVar lookup="row">velocities/mach</independentVar>'
                '<tableData>0 1\n1 2</tableData></table></function></fdm_config>')
        for axis, values in (([0.0, 1.0], [1.0, float("nan")]),
                             ([0.0, 1.0], [1.0, float("inf")]),
                             ([0.0, 1.0], [True, 2.0]),
                             ([False, 1.0], [1.0, 2.0]),
                             ([0.0, 1.0], [1.0, 10 ** 400])):
            with self.subTest(axis=axis, values=values), self.assertRaises(ValueError):
                storestages._table_error(text, "CN", axis, values)

    def test_offline_aero_build_rejects_nonfinite_and_boolean_cached_values(self):
        for field, index, value in (("cna", 1, float("nan")), ("cma", 1, float("inf")),
                                    ("cna", 0, True), ("mach", 0, False)):
            with self.subTest(field=field, value=value), tempfile.TemporaryDirectory() as folder:
                root, path, _ = offline_design_fixture(folder)
                with mock.patch.object(pipeline, "repo_root", return_value=str(root)):
                    d = pipeline.Design(str(path), log=lambda *args, **kwargs: None)
                    report = storestages.aero(d)
                    report[field][index] = value
                    (Path(d.out) / "aero.json").write_text(json.dumps(report), encoding="utf-8")
                    built = storestages.build(d)
                self.assertTrue(any(c["status"] == "fail" for c in built["checks"]))

    def test_offline_aero_build_requires_complete_input_metadata(self):
        for field, value in (("schema", "unknown"), ("producer", None), ("path", ""), ("path", "../outside.json")):
            with self.subTest(field=field, value=value), tempfile.TemporaryDirectory() as folder:
                root, path, _ = offline_design_fixture(folder)
                with mock.patch.object(pipeline, "repo_root", return_value=str(root)):
                    d = pipeline.Design(str(path), log=lambda *args, **kwargs: None)
                    report = storestages.aero(d)
                    report["input"][field] = value
                    (Path(d.out) / "aero.json").write_text(json.dumps(report), encoding="utf-8")
                    built = storestages.build(d)
                self.assertTrue(any(c["status"] == "fail" and "input" in c["name"] for c in built["checks"]))

    def test_offline_aero_build_rejects_invalid_cached_scalar_data(self):
        for field, value in (("S", 0.0), ("d", -1.0), ("ref", True), ("cg_m", None),
                             ("cross_y", float("nan")), ("turn", float("inf")),
                             ("S", 0.06), ("d", 0.4), ("ref", 1.5), ("cg_m", 1.5),
                             ("cross_n", 0.5), ("turn", 0.001)):
            with self.subTest(field=field, value=value), tempfile.TemporaryDirectory() as folder:
                root, path, _ = offline_design_fixture(folder)
                with mock.patch.object(pipeline, "repo_root", return_value=str(root)):
                    d = pipeline.Design(str(path), log=lambda *args, **kwargs: None)
                    report = storestages.aero(d)
                    report[field] = value
                    (Path(d.out) / "aero.json").write_text(json.dumps(report), encoding="utf-8")
                    built = storestages.build(d)
                self.assertTrue(any(c["status"] == "fail" for c in built["checks"]))

    def test_offline_aero_legacy_build_does_not_require_import_report(self):
        with tempfile.TemporaryDirectory() as folder:
            root, path, _ = offline_design_fixture(folder)
            path.write_text(path.read_text(encoding="utf-8").replace('aero_table = "aero.json"\n', ''), encoding="utf-8")
            with mock.patch.object(pipeline, "repo_root", return_value=str(root)):
                d = pipeline.Design(str(path), log=lambda *args, **kwargs: None)
                built = storestages.build(d)
            self.assertFalse([c for c in built["checks"] if c["status"] == "fail"])

    def test_offline_aero_payload_digest_ignores_bookkeeping_and_normalizes_numbers(self):
        from hangar import storetables
        with tempfile.TemporaryDirectory() as folder:
            geo, card, _ = offline_fixture(folder)
            selected = dict(storetables.load(geo, card["id"], "aero.json"), cg_m=1.0)
            before = json.dumps(selected, sort_keys=True)
            digest = storetables.payload_digest(selected)
            report = json.loads(before)
            report.update(kind="body", stage="aero", time="synthetic bookkeeping")
            report["cna"][0] = 1
            self.assertEqual(storetables.payload_digest(report), digest)
            self.assertEqual(json.dumps(selected, sort_keys=True), before)
            report["input"]["producer"]["version"] = "changed"
            self.assertNotEqual(storetables.payload_digest(report), digest)

    def test_offline_aero_cli_refuses_missing_evidence_and_accepts_refreshed_report(self):
        from hangar.__main__ import main
        from hangar import storedrag
        with tempfile.TemporaryDirectory() as folder:
            root, path, _ = offline_design_fixture(folder)
            with mock.patch.object(pipeline, "repo_root", return_value=str(root)), \
                    mock.patch.object(storestages.subprocess, "Popen", side_effect=AssertionError("unexpected process launch")), \
                    redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                card_path = root / "assets/stores/fixture.json"
                card = json.loads(card_path.read_text(encoding="utf-8"))
                card["parameters"]["drag_area_m2"]["value"] = storedrag.build_up(card)[0]
                card_path.write_text(json.dumps(card), encoding="utf-8")
                self.assertEqual(main([str(path), "build"]), 1)
                self.assertEqual(main([str(path), "aero", "build"]), 0)

    def test_offline_aero_exports_selected_input_domain(self):
        from hangar import storeaero
        prefix = "fsim/model/imported-aero/"
        with tempfile.TemporaryDirectory() as folder:
            geo, card, dataset = offline_fixture(folder)
            dataset["mach"] = [0.25, 0.3, 0.6, 0.75]
            (Path(folder) / "aero.json").write_text(json.dumps(dataset), encoding="utf-8")
            with mock.patch.object(storeaero, "geometry", return_value=geo):
                root = ET.fromstring(storemodels.files(card)["fixture.xml"])
            for name, value in (("selected", 1.0), ("mach-min", 0.25), ("mach-max", 0.75)):
                node = root.find("aerodynamics/function[@name='%s%s']" % (prefix, name))
                self.assertIsNotNone(node, name)
                self.assertEqual(float(node.findtext("value")), value)
            flag = root.find("aerodynamics/function[@name='%smach-in-range']" % prefix)
            self.assertIsNotNone(flag)
            self.assertEqual(flag.get("type"), "pre")
            for comparison, endpoint in (("ge", "mach-min"), ("le", "mach-max")):
                node = flag.find("and/" + comparison)
                self.assertEqual([p.text for p in node.findall("property")], ["velocities/mach", prefix + endpoint])

    def test_offline_aero_rejects_duplicate_json_and_preserves_small_crossflow(self):
        from hangar import storeaero, storetables
        with tempfile.TemporaryDirectory() as folder:
            geo, card, dataset = offline_fixture(folder)
            path = Path(folder) / "aero.json"
            path.write_text('{"card":"duplicate",' + json.dumps(dataset)[1:], encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "duplicate"):
                storetables.load(geo, card["id"], "aero.json")
            dataset["crossflow"]["normal"] = 1.234567890123e-10
            path.write_text(json.dumps(dataset), encoding="utf-8")
            with mock.patch.object(storeaero, "geometry", return_value=geo):
                root = ET.fromstring(storemodels.files(card)["fixture.xml"])
            value = root.find(".//function[@name='aero/coefficient/CN']/product/sum/product[2]/value")
            self.assertEqual(float(value.text), dataset["crossflow"]["normal"])

    def test_offline_aero_diagnostic_does_not_extrapolate(self):
        from hangar import storeaero, storetables
        with tempfile.TemporaryDirectory() as folder:
            geo, card, dataset = offline_fixture(folder)
            dataset["mach"] = [1.0, 1.1, 1.1001, 2.0]
            (Path(folder) / "aero.json").write_text(json.dumps(dataset), encoding="utf-8")
            selected = storetables.load(geo, card["id"], "aero.json")
            summary = storeaero.summary(geo, selected)
            self.assertIsNone(summary["normal_area_m2"])
            self.assertIsNone(summary["normal_cp_m"])

    def test_offline_aero_rejects_nonpositive_reference_scales(self):
        from hangar import storetables
        for diameter, length, area_override in ((0.0, 2.0, None), (-0.2, 2.0, None),
                                                (0.2, 0.0, None), (1e-6, 2.0, 0.0)):
            with self.subTest(diameter=diameter, length=length, area=area_override), tempfile.TemporaryDirectory() as folder:
                geo, card, dataset = offline_fixture(folder)
                geo.update(diameter=diameter, length=length)
                dataset["geometry_sha256"] = storetables.geometry_digest(geo)
                dataset["reference"]["diameter_m"] = diameter
                dataset["reference"]["area_m2"] = math.pi * diameter ** 2 / 4.0 if area_override is None else area_override
                (Path(folder) / "aero.json").write_text(json.dumps(dataset), encoding="utf-8")
                with self.assertRaises(ValueError):
                    storetables.load(geo, card["id"], "aero.json")

    def test_offline_aero_preserves_imported_reference_precision(self):
        from hangar import storeaero, storetables
        from hangar.storemodels import FT, IN
        with tempfile.TemporaryDirectory() as folder:
            geo, card, dataset = offline_fixture(folder)
            diameter, reference = 1e-6, 1.234567891011
            geo.update(diameter=diameter, r=[0.5 * diameter] * 2, cg=reference)
            geo["design"]["mass"]["empty_cg"][0] = reference
            dataset["geometry_sha256"] = storetables.geometry_digest(geo)
            dataset["reference"].update(area_m2=math.pi * diameter ** 2 / 4.0, diameter_m=diameter, moment_x_m=reference)
            (Path(folder) / "aero.json").write_text(json.dumps(dataset), encoding="utf-8")
            with mock.patch.object(storeaero, "geometry", return_value=geo):
                root = ET.fromstring(storemodels.files(card)["fixture.xml"])
            area = float(root.findtext("metrics/wingarea")) * FT ** 2
            span = float(root.findtext("metrics/wingspan")) * FT
            chord = float(root.findtext("metrics/chord")) * FT
            point = float(root.findtext("metrics/location[@name='AERORP']/x")) * IN
            self.assertAlmostEqual(area / dataset["reference"]["area_m2"], 1.0, places=13)
            self.assertAlmostEqual(span / diameter, 1.0, places=13)
            self.assertAlmostEqual(chord / diameter, 1.0, places=13)
            self.assertAlmostEqual(point, reference, places=13)

    def test_offline_aero_rejects_unrepresentable_converted_scalars_before_writing(self):
        from hangar import storeaero, storetables
        for field in ("turn", "reference-point", "reference-area"):
            with self.subTest(field=field), tempfile.TemporaryDirectory() as folder:
                geo, card, dataset = offline_fixture(folder)
                if field == "turn":
                    dataset["crossflow"]["turn_m5"] = 1e308
                elif field == "reference-point":
                    geo["cg"] = 1e308
                    geo["design"]["mass"]["empty_cg"][0] = 1e308
                    dataset["reference"]["moment_x_m"] = 1e308
                else:
                    diameter = 6e153
                    geo.update(diameter=diameter, r=[0.5 * diameter] * 2)
                    dataset["reference"].update(diameter_m=diameter,
                                                area_m2=math.pi * diameter * diameter / 4.0)
                dataset["geometry_sha256"] = storetables.geometry_digest(geo)
                (Path(folder) / "aero.json").write_text(json.dumps(dataset), encoding="utf-8")
                with mock.patch.object(storeaero, "geometry", return_value=geo):
                    with self.assertRaisesRegex(ValueError, "finite.*conversion"):
                        storemodels.files(card)
                d = types.SimpleNamespace(name="fixture", aircraft=types.SimpleNamespace(spec=geo["design"]))
                with mock.patch.object(storeaero, "geometry", return_value=geo), \
                        mock.patch.object(storestages, "card_of", return_value=card), \
                        mock.patch.object(storestages, "flight_model", return_value=(str(Path(folder) / "fixture.xml"), True)), \
                        mock.patch.object(pipeline, "write_text") as write:
                    with self.assertRaisesRegex(ValueError, "finite.*conversion"):
                        storestages.build(d)
                    write.assert_not_called()

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
                self.assertEqual(kept, set(out))
                empty, held, _, _ = storestages._xml_mass(out[name + ".xml"])
                self.assertAlmostEqual(empty + held, card["parameters"]["mass_kg"]["value"], delta=0.02)
                self.assertEqual(held > 0.0, "motor" in d.aircraft.spec)  # (a missile's propellant; nothing else burns)

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
