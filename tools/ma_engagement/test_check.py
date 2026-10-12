"""Security and compatibility cases for contract documents, not SDK simulation tests."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import check


def strike():
    return {"request_id": "9007199254740993", "kind": "strike", "operation": "new",
            "parameters": {"payload": "2", "target_id": "1048577", "store": "fixture_store", "quantity": 1}}


def profile():
    return {"schema": "training.authorization.v1", "profiles": [
        {"platform": "lead", "policy_id": "9007199254740993", "revision": "1", "initially_active": False,
         "strike_policy": "requires_approval", "request_validity_s": 8.0,
         "authorizer_role_ids": ["2"], "activation_role_ids": ["1", "2"]}]}


def fixture(name):
    directory = Path(__file__).resolve().parents[2] / "tests/fixtures/mission_training/engagement"
    return json.loads((directory / (name + ".json")).read_text(encoding="utf-8"))


def generic_fixture(name):
    directory = Path(__file__).resolve().parents[2] / "tests/fixtures/mission_training/contract"
    return json.loads((directory / (name + ".json")).read_text(encoding="utf-8"))


class ContractGuards(unittest.TestCase):
    def test_documented_request_table_matches_registration(self):
        self.assertFalse(check.check_page(check.contract())[0])

    def test_non_string_request_kind_is_refused(self):
        document = strike()
        document["kind"] = []
        self.assertTrue(check.validate_request(document))

    def test_request_accepts_large_decimal_id(self):
        self.assertFalse(check.validate_request(strike()))

    def test_actor_cannot_supply_release_consent(self):
        request = strike()
        request["parameters"]["consent"] = True
        self.assertTrue(check.validate_request(request))

    def test_actor_cannot_forge_controller_or_sender(self):
        for field in ("controller_id", "service_id", "source", "sender_system_id", "platform"):
            with self.subTest(field=field):
                request = strike()
                request["parameters"][field] = "1"
                self.assertTrue(check.validate_request(request))

    def test_actor_id_is_not_a_json_number(self):
        request = strike()
        request["request_id"] = 9007199254740993
        self.assertTrue(check.validate_request(request))

    def test_u64_overflow_is_refused(self):
        request = strike()
        request["request_id"] = "18446744073709551616"
        self.assertTrue(check.validate_request(request))

    def test_duplicate_explicit_store_is_refused(self):
        request = strike()
        request["parameters"]["stores"] = ["1", "1"]
        self.assertTrue(check.validate_request(request))

    def test_dispense_is_single_native_operation(self):
        request = {"request_id": "3", "kind": "countermeasure", "operation": "dispense",
                   "parameters": {"kind": "flare"}}
        self.assertFalse(check.validate_request(request))
        request["parameters"]["salvos"] = 10
        self.assertTrue(check.validate_request(request))

    def test_frozen_generic_c2_enum_is_not_extended(self):
        request = {"request_id": "3", "kind": "c2", "operation": "activate_policy",
                   "parameters": {"ma_system_id": "2", "policy_id": "1", "revision": "1"}}
        self.assertTrue(check.validate_request(request))

    def test_policy_activation_uses_independent_kind(self):
        request = {"request_id": "3", "kind": "engagement_policy", "operation": "activate",
                   "parameters": {"ma_system_id": "2", "policy_id": "1", "revision": "1"}}
        self.assertFalse(check.validate_request(request))

    def test_consent_answer_uses_received_native_request_id(self):
        request = {"request_id": "3", "kind": "consent", "operation": "answer",
                   "parameters": {"ma_system_id": "2", "consent_request_id": "4", "granted": True}}
        self.assertFalse(check.validate_request(request))
        request["parameters"]["activity_id"] = "9"
        self.assertTrue(check.validate_request(request))

    def test_profile_is_range_configuration_only(self):
        self.assertFalse(check.validate_extension("training.authorization.v1", profile(), "config"))
        self.assertTrue(check.validate_extension("training.authorization.v1", profile(), "observation"))

    def test_profile_validity_is_positive_finite(self):
        for value in (0, -1, float("nan"), float("inf")):
            with self.subTest(value=value):
                document = profile()
                document["profiles"][0]["request_validity_s"] = value
                self.assertTrue(check.validate_extension("training.authorization.v1", document, "config"))

    def test_duplicate_policy_binding_is_refused(self):
        document = profile()
        document["profiles"].append(deepcopy(document["profiles"][0]))
        self.assertTrue(check.validate_extension("training.authorization.v1", document, "config"))

    def test_duplicate_authorizer_role_is_refused(self):
        document = profile()
        document["profiles"][0]["authorizer_role_ids"] = ["2", "2"]
        self.assertTrue(check.validate_extension("training.authorization.v1", document, "config"))

    def test_profile_selector_uses_generic_platform_name_syntax(self):
        document = profile()
        document["profiles"][0]["platform"] = "lead with spaces"
        self.assertTrue(check.validate_extension(document["schema"], document, "config"))

    def test_unknown_extension_is_refused(self):
        self.assertTrue(check.validate_extension("engagement.unregistered.v1", {}, "observation"))

    def test_referee_truth_cannot_be_an_actor_extension(self):
        document = {"schema": "engagement.truth.v1", "events": [], "weapons": []}
        self.assertFalse(check.validate_extension("engagement.truth.v1", document, "truth"))
        self.assertTrue(check.validate_extension("engagement.truth.v1", document, "observation"))

    def test_launch_zone_is_an_assumption_not_hit_probability(self):
        document = {"schema": "engagement.launch_zone.v1", "target_id": "1048577", "store": "fixture_store",
                    "basis": "assumed", "availability": "available", "maximum_m": 4000.0,
                    "no_escape_m": 2000.0, "minimum_m": 900.0}
        self.assertFalse(check.validate_extension("engagement.launch_zone.v1", document, "receipt"))
        document["hit_probability"] = 1.0
        self.assertTrue(check.validate_extension("engagement.launch_zone.v1", document, "receipt"))

    def test_unknown_zone_uses_null_not_zero(self):
        document = {"schema": "engagement.launch_zone.v1", "target_id": "1048577", "store": "fixture_store",
                    "basis": "assumed", "availability": "unavailable", "maximum_m": None,
                    "no_escape_m": None, "minimum_m": None}
        self.assertFalse(check.validate_extension("engagement.launch_zone.v1", document, "receipt"))
        document["maximum_m"] = 0
        self.assertTrue(check.validate_extension("engagement.launch_zone.v1", document, "receipt"))

    def test_consent_number_requires_its_ma_scope(self):
        document = fixture("correlation")
        self.assertFalse(check.validate_extension(document["schema"], document, "receipt"))
        document["ma_system_id"] = None
        self.assertTrue(check.validate_extension(document["schema"], document, "receipt"))

    def test_policy_identity_and_revision_are_known_together(self):
        document = fixture("correlation")
        document["policy_id"] = "1"
        self.assertTrue(check.validate_extension(document["schema"], document, "receipt"))

    def test_profiles_cannot_select_two_initial_policies(self):
        document = profile()
        document["profiles"][0]["initially_active"] = True
        other = deepcopy(document["profiles"][0])
        other["policy_id"] = "2"
        document["profiles"].append(other)
        self.assertTrue(check.validate_extension(document["schema"], document, "config"))

    def test_unavailable_inventory_cannot_carry_default_counts(self):
        document = fixture("reports")
        document["countermeasures"]["availability"] = "unavailable"
        self.assertTrue(check.validate_extension(document["schema"], document, "observation"))

    def test_report_count_and_availability_are_consistent(self):
        document = fixture("reports")
        self.assertFalse(check.validate_extension(document["schema"], document, "observation"))
        document["stores"]["count"] = 2
        self.assertTrue(check.validate_extension(document["schema"], document, "observation"))
        document["stores"]["count"] = 1
        document["stores"]["availability"] = "not_owner"
        self.assertTrue(check.validate_extension(document["schema"], document, "observation"))

    def test_consent_report_uses_delivered_provenance(self):
        document = fixture("reports")
        document["consents"]["meta"]["source"] = "own_platform"
        self.assertTrue(check.validate_extension(document["schema"], document, "observation"))

    def test_metadata_cannot_claim_stale_without_sample_age(self):
        document = fixture("reports")
        document["stores"]["meta"].update(validity="stale", sample_time_s=None, age_s=None)
        self.assertTrue(check.validate_extension(document["schema"], document, "observation"))

    def test_crew_loss_cannot_be_proved_by_detonation(self):
        document = fixture("outcome")
        self.assertFalse(check.validate_extension(document["schema"], document, "outcome"))
        document["events"][0].update(kind="detonation", crew_lost=False)
        self.assertTrue(check.validate_extension(document["schema"], document, "outcome"))

    def test_crew_loss_event_must_be_for_the_same_target(self):
        document = fixture("outcome")
        document["signals"][0]["subject_id"] = "9"
        self.assertTrue(check.validate_extension(document["schema"], document, "outcome"))

    def test_unavailable_signal_is_not_false(self):
        document = fixture("outcome")
        document["signals"][1]["value"] = False
        self.assertTrue(check.validate_extension(document["schema"], document, "outcome"))

    def test_in_flight_weapon_has_no_terminal_time(self):
        document = fixture("truth")
        document["weapons"][0]["end"] = "in_flight"
        self.assertTrue(check.validate_extension(document["schema"], document, "truth"))

    def test_weapon_end_criterion_requires_terminal_selector(self):
        document = fixture("training")
        document["criteria"][0]["kind"] = "weapon_end"
        self.assertTrue(check.validate_extension(document["schema"], document, "config"))
        document["criteria"][0]["weapon_end"] = "detonated"
        self.assertFalse(check.validate_extension(document["schema"], document, "config"))

    def test_reward_names_are_bounded(self):
        document = fixture("training")
        document["rewards"]["guaranteed_kill"] = 1.0
        self.assertTrue(check.validate_extension(document["schema"], document, "config"))

    def test_complete_action_envelope_enforces_request_parameters(self):
        document = generic_fixture("actions.wait")
        document["requests"] = [strike()]
        self.assertFalse(check.validate_document(document))
        document["requests"][0]["parameters"]["consent"] = True
        self.assertTrue(check.validate_document(document))

    def test_complete_envelope_preserves_generic_id_order_and_limits(self):
        document = generic_fixture("actions.wait")
        document["requests"] = [strike(), dict(strike(), request_id="2")]
        self.assertTrue(check.validate_document(document))
        document["requests"] = [strike()]
        self.assertTrue(check.validate_document(document, {"max_document_bytes": 1}))
        self.assertTrue(check.validate_document(document, {"max_requests_per_document": 0}))

    def test_configuration_extension_resolves_platform_names(self):
        document = generic_fixture("config.two-ship-c2")
        document["extensions"] = {"training.authorization.v1": profile()}
        self.assertFalse(check.validate_document(document))
        document["extensions"]["training.authorization.v1"]["profiles"][0]["platform"] = "missing"
        self.assertTrue(check.validate_document(document))

    def test_malformed_configuration_extension_is_refused_without_crashing(self):
        document = generic_fixture("config.two-ship-c2")
        document["extensions"] = {"training.authorization.v1": {"profiles": "invalid"}}
        self.assertTrue(check.validate_document(document))

    def test_observation_does_not_admit_truth_or_unknown_extensions(self):
        document = generic_fixture("observation.wing-ma")
        for reference, extension in (("engagement.truth.v1", fixture("truth")), ("engagement.future.v1", {})):
            with self.subTest(reference=reference):
                document["extensions"] = {reference: extension}
                self.assertTrue(check.validate_document(document))

    def test_equipment_age_uses_enclosing_observation_time(self):
        document = generic_fixture("observation.wing-ma")
        reports = fixture("reports")
        for name in ("stores", "strikes", "consents", "countermeasures"):
            reports[name]["meta"].update(sample_time_s=document["time_s"], receive_time_s=document["time_s"])
        document["extensions"] = {reports["schema"]: reports}
        self.assertFalse(check.validate_document(document))
        reports["stores"]["meta"]["age_s"] = 1.0
        self.assertTrue(check.validate_document(document))

    def test_combat_evidence_does_not_override_frozen_generic_termination(self):
        document = generic_fixture("outcome.running")
        document["extensions"] = {"engagement.outcome.v1": fixture("outcome")}
        self.assertFalse(check.validate_document(document))
        document.update(reason="success", episode_done=True)
        self.assertTrue(check.validate_document(document))


if __name__ == "__main__":
    unittest.main()
