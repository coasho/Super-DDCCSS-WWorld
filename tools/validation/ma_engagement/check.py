"""Validate proposed extension documents, not installed SDK handlers or simulation.

Uses the frozen generic parser and JSON Schema 2020-12 dialect with the standard
library alone. No SDK binary is loaded.
"""
import argparse
from pathlib import Path
import re
import sys

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO))
from tools.validation.ma_contract import check as generic

SCHEMA = "docs/sdk/ma-engagement-contract.schema.json"
REGISTRY = "docs/sdk/ma-engagement-registry.json"
PAGE = "docs/sdk/ma-engagement-contract.md"
FIXTURES = "tests/training/fixtures/mission_training/engagement"
COMMON = ("u64", "u64_nonzero", "u32", "u32_nonzero", "u16", "component", "meta",
          "availability", "native_name", "number_or_null", "name")
CONTAINERS = {"ma.config.v1": "config", "ma.observation.v1": "observation",
              "ma.outcome.v1": "outcome", "ma.truth.v1": "truth"}


def duplicates(values, where, errors):
    if len(values) != len(set(values)):
        errors.append(where + ": duplicate identity")


def extension_rules(document):
    errors = []
    name = document["schema"]
    if name == "training.authorization.v1":
        profiles = document["profiles"]
        duplicates([(p["platform"], p["policy_id"], p["revision"]) for p in profiles], "$.profiles", errors)
        duplicates([p["platform"] for p in profiles if p["initially_active"]], "$.profiles.initially_active", errors)
        for i, profile in enumerate(profiles):
            for field in ("authorizer_role_ids", "activation_role_ids"):
                duplicates(profile[field], "$.profiles[%d].%s" % (i, field), errors)
    elif name == "training.combat.v1":
        duplicates([c["id"] for c in document["criteria"]], "$.criteria", errors)
        for i, criterion in enumerate(document["criteria"]):
            if (criterion["kind"] == "weapon_end") != ("weapon_end" in criterion):
                errors.append("$.criteria[%d]: terminal selector exactly for weapon_end" % i)
    elif name == "engagement.reports.v1":
        for key in ("stores", "strikes", "consents"):
            report = document[key]
            if report["count"] != len(report["items"]):
                errors.append("$.%s: count does not match items" % key)
            if report["availability"] != "available" and report["items"]:
                errors.append("$.%s: unavailable report carries items" % key)
        inventory = document["countermeasures"]
        if (inventory["availability"] == "available") != (inventory["inventory"] is not None):
            errors.append("$.countermeasures: inventory exactly when available")
        for key in ("stores", "strikes", "consents", "countermeasures"):
            meta = document[key]["meta"]
            source = "dms" if key == "consents" else "own_platform"
            if meta["source"] != source:
                errors.append("$.%s.meta: expected %s provenance" % (key, source))
            if ((meta["sample_time_s"] is None) != (meta["age_s"] is None)
                    or (meta["validity"] == "stale" and meta["age_s"] is None)):
                errors.append("$.%s.meta: invalid sample/age availability" % key)
        duplicates([s["index"] for s in document["stores"]["items"]], "$.stores", errors)
        duplicates([s["activity_id"] for s in document["strikes"]["items"]], "$.strikes", errors)
        duplicates([(c["from_system_id"], c["consent_request_id"]) for c in document["consents"]["items"]], "$.consents", errors)
        for i, strike in enumerate(document["strikes"]["items"]):
            duplicates(strike["stores"], "$.strikes.items[%d].stores" % i, errors)
            duplicates(strike["weapon_ids"], "$.strikes.items[%d].weapon_ids" % i, errors)
        for i, consent in enumerate(document["consents"]["items"]):
            if consent["receive_time_s"] < consent["request_time_s"]:
                errors.append("$.consents.items[%d]: received before requested" % i)
    elif name == "engagement.correlation.v1":
        if (document["policy_id"] is None) != (document["policy_revision"] is None):
            errors.append("$: policy identity and revision are known together")
        if document["consent_request_id"] is not None and document["ma_system_id"] is None:
            errors.append("$: consent request requires its MA system scope")
    elif name == "engagement.launch_zone.v1":
        for key in ("maximum_m", "no_escape_m", "minimum_m"):
            if (document["availability"] == "available") != (document[key] is not None):
                errors.append("$.%s: distance exactly when available" % key)
    elif name in ("engagement.outcome.v1", "engagement.truth.v1"):
        duplicates([e["serial"] for e in document["events"]], "$.events", errors)
        events = {e["serial"]: e for e in document["events"]}
        if name == "engagement.outcome.v1":
            duplicates([(s["kind"], s["subject_id"]) for s in document["signals"]], "$.signals", errors)
            for i, signal in enumerate(document["signals"]):
                where = "$.signals[%d]" % i
                if (signal["availability"] == "available") != (signal["value"] is not None):
                    errors.append(where + ": value exactly when available")
                serial = signal["event_serial"]
                if serial is not None and serial not in events:
                    errors.append(where + ": referenced event is absent")
                if signal["kind"] == "crew_loss" and signal["value"] is True and serial in events:
                    event = events[serial]
                    if not (event["kind"] == "damage" and event["crew_lost"] and event["target_id"] == signal["subject_id"]):
                        errors.append(where + ": crew loss requires matching native damage evidence")
        else:
            duplicates([w["id"] for w in document["weapons"]], "$.weapons", errors)
            for i, weapon in enumerate(document["weapons"]):
                if (weapon["end"] == "in_flight") != (weapon["end_time_s"] is None):
                    errors.append("$.weapons[%d]: terminal time exactly when ended" % i)
                if weapon["end_time_s"] is not None and weapon["end_time_s"] < 0:
                    errors.append("$.weapons[%d]: negative terminal time" % i)
    return errors


class Contract:
    def __init__(self, repo=REPO):
        self.repo = Path(repo)
        self.schema = generic.Schema(generic.parse((self.repo / SCHEMA).read_bytes()))
        self.registry = generic.parse((self.repo / REGISTRY).read_bytes())
        self.base = generic.Schema(generic.parse((self.repo / generic.SCHEMA).read_bytes()))

    def registration_errors(self):
        errors = list(self.schema.errors) + list(self.base.errors)
        if self.schema.root.get("$schema") != "https://json-schema.org/draft/2020-12/schema":
            errors.append("schema: expected JSON Schema 2020-12")
        if self.registry.get("schema") != "ma.engagement.registry.v1" or self.registry.get("runtime_installed") is not False:
            errors.append("registry: expected proposed v1 registrations with runtime_installed=false")
        if self.registry.get("common_definitions") != list(COMMON):
            errors.append("registry: common definitions must be the explicit frozen list")
        for name in COMMON:
            if self.schema.root["$defs"].get(name) != self.base.root["$defs"].get(name):
                errors.append("schema: frozen common definition changed: " + name)
        for reference, entry in self.registry["extensions"].items():
            definition = self.schema.root["$defs"].get(entry["definition"], {})
            if definition.get("properties", {}).get("schema", {}).get("const") != reference:
                errors.append("registry: extension definition mismatch: " + reference)
            if entry["container"] not in set(CONTAINERS.values()) | {"receipt"}:
                errors.append("registry: unknown container: " + reference)
        for name, entry in self.registry["requests"].items():
            properties = self.schema.root["$defs"].get(entry["definition"], {}).get("properties", {})
            if name in generic.GENERIC_KINDS or properties.get("kind", {}).get("const") != name:
                errors.append("registry: request kind mismatch or generic collision: " + name)
            if set(properties.get("operation", {}).get("enum", [])) != set(entry["operations"]):
                errors.append("registry: request operation mismatch: " + name)
        return errors

    def shape(self, document, definition):
        try:
            generic.parse(generic.dump(document))
        except (ValueError, TypeError, OverflowError) as exc:
            return ["$: not strict finite JSON: " + str(exc)]
        return self.schema.validate(document, self.schema.resolve("#/$defs/" + definition))

    def request(self, document):
        if not isinstance(document, dict) or not isinstance(document.get("kind"), str):
            return ["$: request is an object with a string kind"]
        entry = self.registry["requests"].get(document.get("kind"))
        if entry is None:
            return ["$: unregistered extension request kind"]
        errors = self.shape(document, entry["definition"])
        if not errors:
            duplicates(document["parameters"].get("stores", []), "$.parameters.stores", errors)
        return errors

    def extension(self, reference, document, container):
        entry = self.registry["extensions"].get(reference)
        if entry is None:
            return ["$: unregistered extension reference"]
        if entry["container"] != container:
            return ["$: extension is not allowed in " + container]
        errors = self.shape(document, entry["definition"])
        return errors or extension_rules(document)

    def document(self, document, limits=None):
        """Static generic-envelope validation with proposed extension registrations."""
        try:
            encoded = generic.dump(document).encode("utf-8")
            generic.parse(encoded)
        except (ValueError, TypeError, OverflowError) as exc:
            return ["$: not strict finite JSON: " + str(exc)]
        limits = dict(generic.DEFAULT_LIMITS, **(limits or {}))
        errors = self.base.validate(document)
        if errors:
            return errors
        if len(encoded) > limits["max_document_bytes"]:
            errors.append("$: exceeds max_document_bytes")
        errors += generic.rules(document, limits)[0]
        if document["schema"] == "ma.actions.v1":
            for i, request in enumerate(document["requests"]):
                if request["kind"] not in generic.GENERIC_KINDS:
                    errors += ["$.requests[%d]: %s" % (i, e) for e in self.request(request)]
        container = CONTAINERS.get(document["schema"])
        if container:
            for reference, extension in document.get("extensions", {}).items():
                errors += self.extension(reference, extension, container)
        for receipt in document.get("receipts", []):
            for reference, extension in receipt.get("extensions", {}).items():
                errors += self.extension(reference, extension, "receipt")
        if document["schema"] == "ma.config.v1":
            names = generic.platforms(document)
            for reference, extension in document.get("extensions", {}).items():
                if reference not in ("training.authorization.v1", "training.combat.v1") or self.extension(reference, extension, "config"):
                    continue
                for entry in extension.get("profiles", extension.get("criteria", [])):
                    for field in ("platform", "launcher"):
                        if field in entry and entry[field] not in names:
                            errors.append("$: extension selects an unknown configured platform")
        if document["schema"] == "ma.observation.v1":
            reports = document.get("extensions", {}).get("engagement.reports.v1")
            if reports and not self.extension("engagement.reports.v1", reports, "observation"):
                for name in ("stores", "strikes", "consents", "countermeasures"):
                    meta = reports[name]["meta"]
                    if meta["sample_time_s"] is not None and abs(document["time_s"] - meta["sample_time_s"] - meta["age_s"]) > 1e-6:
                        errors.append("$: extension age does not match observation time")
        return errors


_default = None


def contract():
    global _default
    if _default is None:
        _default = Contract()
    return _default


def validate_request(request):
    return contract().request(request)


def validate_extension(reference, document, container):
    return contract().extension(reference, document, container)


def validate_document(document, limits=None):
    return contract().document(document, limits)


def check_fixtures(validator):
    directory = validator.repo / FIXTURES
    manifest = generic.parse((directory / "manifest.json").read_bytes())
    errors, counts, listed, requests, references = [], {}, set(), set(), set()
    if manifest.get("schema") != "ma.engagement.fixtures.v1":
        errors.append("fixtures: unknown manifest version")
    for entry in manifest["files"]:
        name = entry["file"]
        if Path(name).name != name or name in listed:
            errors.append("fixtures: duplicate or nonlocal path: " + name)
            continue
        listed.add(name)
        try:
            document = generic.parse((directory / name).read_bytes())
            if entry["kind"] == "request":
                problems = validator.request(document)
            elif entry["kind"] == "extension":
                problems = validator.extension(entry["reference"], document, entry["container"])
            elif entry["kind"] == "document":
                problems = validator.document(document)
            else:
                errors.append(name + ": unknown fixture kind")
                continue
            if not generic.same(document, generic.parse(generic.dump(document))):
                errors.append(name + ": canonical JSON did not round-trip")
        except (ValueError, OSError) as exc:
            document, problems = None, [str(exc)]
        expected = entry["expect"]
        counts[expected] = counts.get(expected, 0) + 1
        if expected not in ("valid", "invalid") or bool(problems) != (expected == "invalid"):
            errors.append("%s: expected %s, found %s" % (name, expected, problems or "valid"))
        if expected == "invalid" and (not entry.get("error_contains") or not any(entry["error_contains"] in p for p in problems)):
            errors.append(name + ": refusal did not match its expected reason")
        if expected == "valid" and not problems:
            if entry["kind"] == "request":
                requests.add((document["kind"], document["operation"]))
            elif entry["kind"] == "extension":
                references.add(entry["reference"])
    actual = {p.name for p in directory.glob("*.json") if p.name != "manifest.json"}
    if actual != listed:
        errors.append("fixtures: manifest and file inventory differ")
    needed = {(name, op) for name, entry in validator.registry["requests"].items() for op in entry["operations"]}
    if needed != requests or set(validator.registry["extensions"]) != references:
        errors.append("fixtures: missing valid operation or extension coverage")
    return errors, counts


def check_page(validator):
    path = validator.repo / PAGE
    page = path.read_text(encoding="utf-8")
    errors, links = [], 0
    for i, line in enumerate(page.splitlines(), 1):
        if any(not (" " <= c <= "~" or c == "\t") for c in line):
            errors.append("page:%d: not printable ASCII" % i)
        for target in generic.LINK.findall(line):
            if re.match(r"[a-z][a-z0-9+.-]*:", target):
                continue
            links += 1
            file, _, anchor = target.partition("#")
            linked = (path.parent / file).resolve() if file else path
            if not linked.exists():
                errors.append("page:%d: missing link %s" % (i, target))
            elif anchor and linked.suffix == ".md" and anchor not in generic.anchors(linked.read_text(encoding="utf-8")):
                errors.append("page:%d: missing anchor %s" % (i, target))
    tabulated = set(re.findall(r"^\| `([a-z_]+)` \| `([a-z_]+)` \|", page, re.M))
    tabulated.discard(("kind", "operation"))
    registered = {(name, op) for name, entry in validator.registry["requests"].items() for op in entry["operations"]}
    if tabulated != registered:
        errors.append("page: typed request table differs from registry")
    for reference in validator.registry["extensions"]:
        if "`" + reference + "`" not in page:
            errors.append("page: missing registered extension: " + reference)
    return errors, links


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", default=str(REPO))
    validator = Contract(Path(parser.parse_args().repo).resolve())
    errors = validator.registration_errors()
    fixture_errors, counts = check_fixtures(validator)
    page_errors, links = check_page(validator)
    errors += fixture_errors + page_errors
    for error in errors:
        print(error)
    print("%d definitions; %d fixtures (%s); %d local links; runtime_installed=false; %d errors" % (
        len(validator.schema.root["$defs"]), sum(counts.values()),
        ", ".join("%d %s" % (n, k) for k, n in sorted(counts.items())), links, len(errors)))
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
