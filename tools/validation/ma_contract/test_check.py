"""check.py's own checks: what it must refuse (python -B tools/validation/ma_contract/test_check.py)."""
import base64
import copy
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("ma_contract_check", HERE / "check.py")
check = importlib.util.module_from_spec(spec)
spec.loader.exec_module(check)
REPO = HERE.parents[2]
DOCUMENT = check.parse((REPO / check.SCHEMA).read_bytes())
SCHEMA = check.Schema(DOCUMENT)
FIXTURES = REPO / check.FIXTURES


def fixture(name):
    return check.parse((FIXTURES / name).read_bytes())


def sub(definition):
    return {"$ref": "#/$defs/" + definition}


class StrictJson(unittest.TestCase):
    def test_refuses_what_is_not_strict_json(self):
        for text in (b'{"a": NaN}', b'{"a": Infinity}', b'{"a": -Infinity}', b'{"a": 1e999}', b'{"a": 1, "a": 2}',
                     b'\xef\xbb\xbf{}', b'{"a": "\xff"}', b'{"a": 1,}', b'{"a": 1} // note', b"{'a': 1}", b""):
            with self.assertRaises(check.Refused, msg=text):
                check.parse(text)

    def test_keeps_identities_exact(self):
        text = b'{"id": "18446744073709551615", "n": 9007199254740993}'
        self.assertEqual(check.parse(text)["id"], "18446744073709551615")
        self.assertEqual(check.parse(check.dump(check.parse(text)))["n"], 9007199254740993)

    def test_dump_never_writes_nan(self):
        with self.assertRaises(ValueError):
            check.dump({"a": float("nan")})


class Identities(unittest.TestCase):
    def test_bounds(self):
        for definition, bits in (("u16", 16), ("u32", 32), ("u64", 64)):
            top = 2 ** bits - 1
            for value in (0, 1, 9, 10, top - 1, top):
                self.assertEqual(SCHEMA.validate(str(value), sub(definition)), [], (definition, value))
            for text in (str(top + 1), str(top * 10), "00", "01", "-1", "+1", "", " 1", "1 ", "1\n", "1.0", "1e3", "0x10"):
                self.assertTrue(SCHEMA.validate(text, sub(definition)), (definition, text))
            self.assertTrue(SCHEMA.validate(top, sub(definition)), "a number is not an identity")

    def test_above_2_53_survives(self):
        for value in (2 ** 53, 2 ** 53 + 1, 2 ** 63, 2 ** 64 - 1):
            self.assertEqual(SCHEMA.validate(str(value), sub("u64")), [])
            self.assertNotEqual(float(value + 1), value + 1 if value == 2 ** 53 else None)

    def test_nonzero(self):
        self.assertTrue(SCHEMA.validate("0", sub("u64_nonzero")))
        self.assertEqual(SCHEMA.validate("1", sub("u64_nonzero")), [])


class Unknowns(unittest.TestCase):
    def test_null_is_the_only_unknown(self):
        self.assertEqual(SCHEMA.validate(None, sub("number_or_null")), [])
        self.assertEqual(SCHEMA.validate(0.0, sub("number_or_null")), [])
        for value in ("NaN", "unknown", False, [], {}):
            self.assertTrue(SCHEMA.validate(value, sub("number_or_null")), value)

    def test_a_report_keeps_its_validity(self):
        observation = fixture("observation.wing-ma.json")
        stale = observation["reports"]["link_tracks"]["items"][1]
        self.assertEqual(stale["meta"]["validity"], "stale")
        self.assertIsNone(stale["height_m"])
        del stale["meta"]["validity"]
        self.assertTrue(SCHEMA.validate(observation))


class Bytes(unittest.TestCase):
    def errors(self, parameters):
        found = []
        check.message_bytes(parameters, "$", found)
        return found

    def refused(self, size, limits=None):
        actions = fixture("actions.wing-ma.json")
        actions["requests"][10]["parameters"].update({"byte_length": size, "data_b64": base64.b64encode(bytes(size)).decode()})
        self.assertEqual(SCHEMA.validate(actions), [])
        self.assertEqual(check.rules(actions, limits), ([], []), "a message over a configured limit is not a malformed document")
        return check.refusals(actions, limits)

    def test_round_trip_of_bytes_that_are_not_text(self):
        raw = bytes(range(256))
        holder = {"byte_length": len(raw), "data_b64": base64.b64encode(raw).decode()}
        self.assertEqual(SCHEMA.validate(holder["data_b64"], sub("base64")), [])
        self.assertEqual(self.errors(holder), [])
        with self.assertRaises(UnicodeDecodeError):
            raw.decode("utf-8")

    def test_refused_encodings(self):
        for text in ("QQ", "QQ=", "Q===", "QUJD\n", "QUJD ", "QU-D", "QU_D", "====", "QQ==QQ=="):
            self.assertTrue(SCHEMA.validate(text, sub("base64")), text)
        self.assertTrue(self.errors({"byte_length": 1, "data_b64": "QR=="}), "unused bits set")
        self.assertTrue(self.errors({"byte_length": 2, "data_b64": "QQ=="}), "length disagrees")

    def test_bound_is_on_decoded_bytes(self):
        self.assertEqual(self.refused(1024), {})
        self.assertEqual(self.refused(1025), {"9007199254741008": "message_too_large"})
        self.assertEqual(self.refused(1025, {"max_message_bytes": 2048}), {})
        self.assertEqual(self.refused(0), {})
        self.assertEqual(self.errors({"byte_length": 0, "data_b64": ""}), [])

    def test_a_malformed_message_is_not_a_refused_request(self):
        actions = fixture("actions.wing-ma.json")
        actions["requests"][10]["parameters"]["byte_length"] += 1
        errors, _ = check.rules(actions)
        self.assertTrue(any("bytes decoded" in e for e in errors), "its document is refused whole")
        self.assertEqual(check.refusals(actions), {})
        observation = fixture("observation.wing-ma.json")
        observation["messages"]["items"][0].update({"byte_length": 2000, "data_b64": base64.b64encode(bytes(2000)).decode()})
        self.assertEqual(check.rules(observation), ([], []), "a delivered message is as large as its net let it be")


class Dialect(unittest.TestCase):
    def test_the_schema_is_in_its_dialect(self):
        self.assertEqual(SCHEMA.errors, [])
        self.assertEqual(DOCUMENT["$schema"], "https://json-schema.org/draft/2020-12/schema")

    def test_refuses_keywords_outside_it(self):
        for extra in ({"patternProperties": {}}, {"format": "uuid"}, {"uniqueItems": True}, {"dependentRequired": {}},
                      {"$ref": "#/$defs/missing"}, {"$ref": "other.json#/a"}, {"pattern": "^($"}, {"pattern": "[0-9]+"}, {"type": "float"}, {"then": {}}):
            self.assertTrue(check.Schema({"$defs": {"x": extra}}).errors, extra)

    def test_keywords(self):
        s = check.Schema({})
        cases = (
            ({"type": "integer"}, 1.0, 1.5), ({"type": "number"}, 1, True), ({"const": True}, True, 1), ({"enum": [None, "a"]}, None, 0),
            ({"minimum": 1}, 1, 0), ({"maximum": 1}, 1, 2), ({"exclusiveMinimum": 0}, 0.1, 0), ({"minLength": 1}, "a", ""),
            ({"maxLength": 1}, "a", "ab"), ({"minItems": 1}, [1], []), ({"maxItems": 1}, [1], [1, 2]), ({"items": {"type": "string"}}, ["a"], [1]),
            ({"required": ["a"]}, {"a": 1}, {}), ({"properties": {"a": False}}, {}, {"a": 1}),
            ({"properties": {"a": {}}, "additionalProperties": False}, {"a": 1}, {"b": 1}),
            ({"additionalProperties": {"type": "number"}}, {"b": 1}, {"b": "x"}), ({"propertyNames": {"pattern": "^a$"}}, {"a": 1}, {"b": 1}), ({"pattern": "^a$"}, "a", "a\n"),
            ({"maxProperties": 1}, {"a": 1}, {"a": 1, "b": 2}), ({"allOf": [{"minimum": 1}, {"maximum": 2}]}, 2, 3),
            ({"anyOf": [{"type": "string"}, {"type": "null"}]}, None, 1), ({"oneOf": [{"minimum": 1}, {"maximum": 2}]}, 3, 2),
            ({"not": {"const": "0"}}, "1", "0"), ({"if": {"required": ["a"]}, "then": {"required": ["b"]}}, {"c": 1}, {"a": 1}),
        )
        for schema, good, bad in cases:
            self.assertEqual(s.validate(good, schema), [], (schema, good))
            self.assertTrue(s.validate(bad, schema), (schema, bad))


class Rules(unittest.TestCase):
    def refused(self, document, text, limits=None):
        self.assertEqual(SCHEMA.validate(document), [], "the case must be schema-valid")
        errors, _ = check.rules(document, limits)
        self.assertTrue(any(text in e for e in errors), errors)

    def test_valid_fixtures_pass(self):
        for name in ("config.two-ship-c2.json", "observation.wing-ma.json", "observation.crc-c2.json", "actions.wing-ma.json",
                     "actions.crc-c2.json", "receipts.wing-ma.json", "outcome.running.json", "outcome.truncated.json", "truth.json"):
            document = fixture(name)
            self.assertEqual(SCHEMA.validate(document), [], name)
            self.assertEqual(check.rules(document), ([], []), name)

    def test_actions(self):
        actions = fixture("actions.crc-c2.json")
        same = copy.deepcopy(actions)
        same["requests"][1]["request_id"] = same["requests"][0]["request_id"]
        self.refused(same, "do not strictly increase")
        self.refused(actions, "over the 4", {"max_requests_per_document": 4})
        extension = fixture("actions.extension-kind.json")
        self.assertEqual(check.rules(extension), ([], ["strike.new"]))
        self.assertEqual(check.refusals(extension), {"31": "unsupported_kind"})

    def test_a_request_cannot_name_its_sender(self):
        actions = fixture("actions.wing-ma.json")
        for field in ("controller_id", "source_system_id", "sender", "service_id"):
            changed = copy.deepcopy(actions)
            changed["requests"][10]["parameters"][field] = "1"
            self.assertTrue(SCHEMA.validate(changed), field)

    def test_config(self):
        config = fixture("config.two-ship-c2.json")
        both = copy.deepcopy(config)
        both["services"][1]["command_capabilities"] = ["*"]
        self.refused(both, "every command capability to one service and some to another")
        mixed = copy.deepcopy(config)
        mixed["services"][0]["command_capabilities"] = ["*", "fsim.guidance.hsa"]
        self.refused(mixed, "stands alone")
        counted = copy.deepcopy(config)
        counted["scenario"]["vehicles"][1].update({"name": "wing", "count": 2})
        self.refused(counted, "wing is no platform")
        counted["services"][3]["platform"] = counted["systems"][1]["platform"] = "wing-2"
        counted["nets"][0]["members"][1] = "wing-1"
        counted["dissemination"][1]["platform"] = "wing-1"
        for criterion in counted["task"]["criteria"][1:]:
            criterion["platform"] = "wing-2"
        self.assertEqual(check.rules(counted), ([], []))

    def test_observation(self):
        observation = fixture("observation.wing-ma.json")
        absent = copy.deepcopy(observation)
        absent["reports"]["navigation"]["own_ship"] = None
        self.refused(absent, "own_ship is given exactly when it is available")
        stale = copy.deepcopy(observation)
        stale["reports"]["peers"]["items"][1]["meta"]["validity"] = "stale"
        self.refused(stale, "stale without an age")
        behaviour = fixture("actions.wing-ma.json")
        behaviour["requests"][1]["operation"] = "update"
        behaviour["requests"][1]["parameters"] = {"mode": "behavior", "behavior": "pursuit", "activity_id": "8589934601", "params": {"gain": 1.0}}
        self.assertEqual(SCHEMA.validate(behaviour), [], "a behaviour's UPDATE carries its parameters")
        behaviour["requests"][1]["parameters"]["target_id"] = "2147483649"
        self.assertTrue(SCHEMA.validate(behaviour), "a behaviour's UPDATE names no target")

    def test_receipts(self):
        receipts = fixture("receipts.wing-ma.json")
        success = copy.deepcopy(receipts)
        success["receipts"][3]["state"] = "completed"
        self.refused(success, "without the native answer it stands for")
        refusal = copy.deepcopy(receipts)
        refusal["receipts"][4]["state"] = "failed"
        self.refused(refusal, "a gateway refusal is rejected")
        self.assertTrue(SCHEMA.validate(dict(receipts, reward=1.0)), "no reward in an actor's receipts")

    def test_gateway_close_receipts(self):
        """gateway.close makes no simulation call: its receipt is queued, rejected by the gateway, or accepted and
        closed - each with no native answer; nothing else, and the native-answer rule still holds for every other kind."""
        receipts = fixture("receipts.wing-ma.json")

        def close(state, closed, reason=None, native=None, operation="close"):
            document = copy.deepcopy(receipts)
            document["receipts"][3].update({"kind": "gateway", "operation": operation, "state": state, "closed": closed,
                                            "gateway_reason": reason, "native": native})
            return document
        for document in (close("accepted", True), close("queued", False), close("rejected", True, "unknown_request")):
            self.assertEqual(SCHEMA.validate(document), [])
            self.assertEqual(check.rules(document), ([], []), document["receipts"][3])
        for state, closed in (("pending", False), ("executing", False), ("completed", True), ("failed", True), ("canceled", True),
                              ("accepted", False)):
            self.refused(close(state, closed), "gateway.close")
        native = {"domain": "dms", "status": "done", "reason": None, "reason_code": None, "ids": {}}
        self.refused(close("accepted", True, native=native), "gateway.close")
        self.refused(close("accepted", True, operation="open"), "gateway.close")
        ordinary = copy.deepcopy(receipts)
        ordinary["receipts"][3].update({"kind": "flight", "operation": "new", "state": "accepted", "closed": True})
        self.refused(ordinary, "without the native answer it stands for")

    def test_outcome(self):
        outcome = fixture("outcome.truncated.json")
        outcome["roles"][1]["bootstrap"] = False
        self.refused(outcome, "bootstrap exactly when truncated")
        both = fixture("outcome.truncated.json")
        both["roles"][3]["truncated"] = True
        self.refused(both, "terminated and truncated at once")

    def test_c2(self):
        actions = fixture("actions.crc-c2.json")
        self.assertEqual(SCHEMA.validate(actions), [])
        del actions["requests"][6]["parameters"]["value"]
        self.assertTrue(SCHEMA.validate(actions), "an override that does not cancel needs its value")
        statuses = fixture("observation.crc-c2.json")["reports"]["command_statuses"]["items"]
        self.assertEqual(statuses[0]["command_id"], "0", "a role declaration's answer carries command id 0")


class Pages(unittest.TestCase):
    def test_anchors(self):
        text = "# One two\n\n## `ma.config.v1`: the configuration\n\n```\n# not a heading\n```\n## One two\n### M5 core, [linked](x.md)\n"
        self.assertEqual(check.anchors(text), {"one-two", "maconfigv1-the-configuration", "one-two-1", "m5-core-linked"})

    def test_a_broken_link_and_a_stray_character_are_found(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp)
            for relative in check.PAGES + check.LINKING:
                (repo / relative).parent.mkdir(parents=True, exist_ok=True)
                (repo / relative).write_text("# Page\n", encoding="utf-8")
            (repo / check.CONTRACT).write_text("# Page\n\n[gone](missing.md) [far](ma-composition.md#nowhere) [ok](ma-composition.md#page)\n"
                                               "[web](https://example.invalid/x)\ncafé\n", encoding="utf-8")
            errors, links = check.check_pages(repo, DOCUMENT)
            self.assertEqual(links, 3)
            self.assertEqual(sum("does not exist" in e for e in errors), 1)
            self.assertEqual(sum("has no heading #nowhere" in e for e in errors), 1)
            self.assertEqual(sum("not printable ASCII" in e for e in errors), 1)
            self.assertTrue(any("request operation flight.new is in the schema only" in e for e in errors))
            self.assertTrue(any("gateway reason late_tick is in the schema only" in e for e in errors))


class Manifest(unittest.TestCase):
    def test_expectations_are_held(self):
        entry = {"file": "x", "expect": "valid"}
        for name, expect in (("invalid.actions.id-as-number.json", "schema"), ("invalid.actions.base64-length.json", "rule"),
                             ("refused.observation.nan.json", "refused as JSON")):
            problems, _ = check.check_fixture(SCHEMA, FIXTURES / name, entry)
            self.assertTrue(problems and problems[0].startswith(expect), (name, problems))
        problems, _ = check.check_fixture(SCHEMA, FIXTURES / "actions.wait.json", {"file": "x", "expect": "schema_invalid"})
        self.assertEqual(problems, ["schema-valid, expected schema_invalid"])
        problems, _ = check.check_fixture(SCHEMA, FIXTURES / "actions.extension-kind.json", entry)
        self.assertTrue(any("request refusals" in p for p in problems))
        for name in ("actions.extension-kind.json", "actions.message-too-large.json"):
            entry = next(e for e in check.parse((FIXTURES / "manifest.json").read_bytes())["fixtures"] if e["file"] == name)
            self.assertEqual(check.check_fixture(SCHEMA, FIXTURES / name, entry)[0], [], name)

    def test_required_coverage(self):
        found = set()
        for name in ("observation.wing-ma.json", "actions.wing-ma.json"):
            check.coverage(fixture(name), found)
        self.assertEqual(found, {"identity_above_2_53", "unknown_numeric_report", "non_utf8_message_bytes"})
        nothing = set()
        check.coverage(fixture("actions.wait.json"), nothing)
        self.assertEqual(nothing, {"identity_above_2_53"})  # (its env_id alone)


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])
