"""Configuration-report regressions; no flight, sensor or damage model execution."""
import copy
import importlib.util
from pathlib import Path
import tempfile
import unittest

SPEC = importlib.util.spec_from_file_location("payload_report", Path(__file__).with_name("report.py"))
report = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(report)


def card(identity="one"):
    rows = {role: {"availability": "not_applicable", "reason": "Synthetic inert store"}
            for role in report.ROLES}
    rows["actuator"] = {
        "availability": "available", "implementation_id": "fsim.actuator.jsbsim_lag_rate_stops",
        "configuration_version": 1, "parameter_refs": {"lag_s": "lag"}, "bindings": {},
        "basis": "assumed", "note": "Synthetic configuration only", "validity": "No physical acceptance"}
    return {"id": identity, "designation": "Synthetic " + identity, "model": "jsbsim:" + identity,
            "parameters": {"lag": {"value": .1, "unit": "s", "basis": "published",
                                     "uncertainty": .01, "source": "synthetic"}},
            "sources": {"synthetic": {"title": "Synthetic source", "url": "https://example.invalid/source"}},
            "component_models": {"schema_version": 1, "components": rows}}


class ReportTest(unittest.TestCase):
    def test_selected_finite_zoned_v2_keeps_its_concrete_class_and_calibration_boundary(self):
        import json
        actual = json.loads((report.ROOT / "assets/stores/aim-120c.json").read_text(encoding="utf-8"))
        row = report.describe(actual)["components"]["warhead"]
        self.assertEqual(row["concrete_class"], "FiniteZonedCaseWarheadV2")
        self.assertEqual(row["basis"], "assumed")
        self.assertEqual(row["runtime_evidence"], "not_assessed")
        self.assertEqual(row["physical_acceptance"], "not_assessed")

    def test_shared_class_is_not_counted_as_variant_specific_implementation(self):
        first, second = card(), card("two")
        second["parameters"]["lag"]["value"] = .2
        raw = copy.deepcopy([first, second])
        result = report.summarize([report.describe(first), report.describe(second)])
        role = result["roles"]["actuator"]
        self.assertEqual(role["available"], 2)
        self.assertEqual(role["concrete_class_count"], 1)
        self.assertEqual(role["parameter_configuration_count"], 2)
        self.assertEqual([first, second], raw)

    def test_published_inputs_do_not_promote_assumed_component_or_claim_runtime_acceptance(self):
        result = report.describe(card())
        component = result["components"]["actuator"]
        self.assertEqual(component["basis"], "assumed")
        self.assertEqual(component["parameter_basis_counts"], {"published": 1})
        self.assertEqual(component["runtime_evidence"], "not_assessed")
        self.assertEqual(component["physical_acceptance"], "not_assessed")
        self.assertEqual(component["parameters"]["lag_s"]["source_record"]["title"], "Synthetic source")

    def test_unavailable_and_not_applicable_remain_distinct_and_have_no_class(self):
        raw = card()
        raw["component_models"]["components"]["fuze"] = {
            "availability": "unavailable", "reason": "Synthetic missing model"}
        result = report.describe(raw)
        self.assertEqual(result["components"]["fuze"]["availability"], "unavailable")
        self.assertEqual(result["components"]["warhead"]["availability"], "not_applicable")
        for role in ("fuze", "warhead"):
            self.assertIsNone(result["components"][role]["concrete_class"])

    def test_unknown_provider_does_not_acquire_a_known_class_or_fallback(self):
        raw = card()
        raw["component_models"]["components"]["actuator"]["implementation_id"] = "thirdparty.unknown"
        result = report.describe(raw)["components"]["actuator"]
        self.assertIsNone(result["concrete_class"])
        self.assertEqual(result["mapping_status"], "unknown")
        self.assertEqual(result["implementation_id"], "thirdparty.unknown")

    def test_invalid_explicit_configuration_is_not_reported_as_available(self):
        for defect in ("missing_role", "extra_role", "source", "nonfinite", "missing_parameter"):
            raw = card()
            if defect == "missing_role": del raw["component_models"]["components"]["fuze"]
            if defect == "extra_role": raw["component_models"]["components"]["extra"] = {}
            if defect == "source": raw["parameters"]["lag"]["source"] = "missing"
            if defect == "nonfinite": raw["parameters"]["lag"]["value"] = float("nan")
            if defect == "missing_parameter": del raw["parameters"]["lag"]
            with self.subTest(defect=defect), self.assertRaises(ValueError):
                report.describe(raw)

    def test_legacy_absence_does_not_invent_nine_available_components(self):
        raw = card()
        del raw["component_models"]
        result = report.describe(raw)
        self.assertEqual({row["availability"] for row in result["components"].values()}, {"legacy_unknown"})
        self.assertEqual(report.summarize([result])["roles"]["actuator"]["available"], 0)

    def test_aliases_share_a_class_and_registry_versions_are_not_maturity(self):
        raw = card()
        rows = raw["component_models"]["components"]
        rows["fuze"] = copy.deepcopy(rows["actuator"])
        rows["fuze"]["implementation_id"] = "native.fuze.legacy.v1"
        first = report.describe(raw)["components"]["fuze"]
        rows["fuze"]["implementation_id"] = "native.fuze.swept_path.v1"
        second = report.describe(raw)["components"]["fuze"]
        self.assertEqual(first["concrete_class"], "SweptPathFuzeV1")
        self.assertEqual(first["concrete_class"], second["concrete_class"])
        self.assertEqual(first["physical_acceptance"], "not_assessed")

    def test_report_is_deterministic_and_escapes_card_text(self):
        raw = card()
        raw["designation"] = '<script>alert("x")</script>'
        document = {"schema": report.SCHEMA, "cards": [report.describe(raw)], "input_manifest": {},
                    "summary": report.summarize([report.describe(raw)]), "head": "synthetic"}
        html = report.render_html(document)
        self.assertEqual(html, report.render_html(document))
        self.assertNotIn(raw["designation"], html)
        self.assertIn("&lt;script&gt;", html)
        self.assertIn("not_assessed", html)

    def test_duplicate_identity_fails_and_catalogue_inputs_are_content_bound(self):
        import json
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            stores = root / "assets/stores"
            stores.mkdir(parents=True)
            first = stores / "one.json"
            first.write_text(json.dumps(card()), encoding="utf-8")
            before = report.collect(root, head="synthetic")
            changed = card()
            changed["parameters"]["lag"]["value"] = .2
            first.write_text(json.dumps(changed), encoding="utf-8")
            after = report.collect(root, head="synthetic")
            self.assertNotEqual(before["input_manifest"], after["input_manifest"])
            (stores / "two.json").write_text(first.read_text(), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "duplicate"):
                report.collect(root, head="synthetic")

    def test_imported_test_failure_stays_failed_without_promoting_any_model(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "results.xml"
            path.write_text('<testsuite><testcase name="config" time="1"/>'
                            '<testcase name="scene" time="2"><failure message="zero hits"/></testcase>'
                            '<testcase name="other"><skipped/></testcase></testsuite>')
            result = report.read_test_results(path)
            self.assertEqual([row["status"] for row in result["cases"]], ["passed", "failed", "skipped"])
            self.assertEqual(result["failed"], 1)
            self.assertEqual(result["passed"], 1)
            self.assertEqual(result["skipped"], 1)
            self.assertEqual(result["sha256"], report.digest(path.read_bytes()))
            self.assertEqual(report.describe(card())["components"]["actuator"]["physical_acceptance"], "not_assessed")

    def test_empty_test_result_is_not_a_passing_gate(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "results.xml"
            path.write_text('<testsuite tests="0"/>')
            with self.assertRaisesRegex(ValueError, "no test cases"):
                report.read_test_results(path)


if __name__ == "__main__":
    unittest.main()
