"""Read-only payload configuration/source report, not model acceptance evidence."""
import argparse
from collections import Counter
import hashlib
from html import escape
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools/models/hangar"))
from hangar import components  # noqa: E402; reuse the maintained card resolver

SCHEMA = "fsim.payload-model-evidence.v1"
ROLES = ("seeker", "guidance", "autopilot", "allocator", "actuator", "fuze",
         "warhead", "propulsion", "aerodynamics")
BASE = "src/simulation/mission_systems/components/"
# A maintained source index, not RTTI or evidence of an instantiated provider.
PROVIDERS = {
    ("seeker", "fsim.seeker.encounter.v1"): "EncounterSeekerV1",
    ("seeker", "fsim.seeker.nonimaging_ir.v2"): "NonImagingIrSeekerV2",
    ("seeker", "fsim.seeker.imaging_ir.v2"): "ImagingIrSeekerV2",
    ("seeker", "fsim.seeker.semi_active_radar.v2"): "SemiActiveRadarSeekerV2",
    ("seeker", "fsim.seeker.active_radar.v2"): "ActiveRadarSeekerV2",
    ("guidance", "fsim.guidance.velocity_normal_pn.v1"): "VelocityNormalPnGuidanceV1",
    ("guidance", "fsim.guidance.measured_pn.v2"): "MeasuredPnGuidanceV2",
    ("autopilot", components.AUTOPILOT): "LoadFeedbackCascadeAutopilotV1",
    ("allocator", components.ALLOCATOR): "JsbsimAuthoredFinMixerV1",
    ("actuator", components.ACTUATOR): "JsbsimLagRateStopsActuatorV1",
    ("propulsion", components.PROPULSION): "JsbsimSolidRocketPropulsionV1",
    ("aerodynamics", components.AERODYNAMICS): "JsbsimGeneratedStoreAerodynamicsV1",
    ("fuze", "native.fuze.legacy.v1"): "SweptPathFuzeV1",
    ("fuze", "native.fuze.swept_path.v1"): "SweptPathFuzeV1",
    ("fuze", "native.fuze.turning_cpa.v1"): "TurningCpaFuzeV1",
    ("warhead", "native.warhead.legacy.v1"): "ZonedCaseWarheadV1",
    ("warhead", "native.warhead.zoned_case.v1"): "ZonedCaseWarheadV1",
    ("warhead", "native.warhead.side_case.v1"): "SideCaseWarheadV1",
    ("warhead", "native.warhead.finite_zoned_case.v2"): "FiniteZonedCaseWarheadV2",
}


def digest(data):
    return hashlib.sha256(data).hexdigest()


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def source_record(card, entry):
    source = entry.get("source", "")
    record = card.get("sources", {}).get(source)
    if entry.get("basis") in ("published", "derived") and not isinstance(record, dict):
        raise ValueError("unresolved provenance source: " + str(source))
    return record


def describe(card):
    if not isinstance(card.get("id"), str) or not card["id"]:
        raise ValueError("card needs a nonempty identity")
    block = card.get("component_models")
    if "component_models" in card and (not isinstance(block, dict) or
            not isinstance(block.get("components"), dict) or set(block["components"]) != set(ROLES)):
        raise ValueError("explicit profile must declare exactly the nine component roles")
    result = {"id": card["id"], "designation": card.get("designation", card["id"]),
              "native_model": card.get("model"), "components": {}}
    for role in ROLES:
        resolved = components.resolve(card, role)
        row = {"availability": "legacy_unknown" if resolved is None else resolved["availability"],
               "concrete_class": None, "runtime_evidence": "not_assessed", "physical_acceptance": "not_assessed"}
        result["components"][role] = row
        if resolved is None:
            row["reason"] = "No explicit component profile; this report does not infer legacy execution."
            continue
        if resolved["availability"] != "available":
            row["reason"] = resolved["reason"]
            continue
        row.update({key: resolved.get(key) for key in
                    ("implementation_id", "configuration_version", "basis", "source", "note", "validity", "bindings")})
        row["source_record"] = source_record(card, resolved)
        selected = PROVIDERS.get((role, row["implementation_id"]))
        row["mapping_status"] = "indexed_source" if selected else "unknown"
        if selected:
            row["concrete_class"] = selected
            row["class_header"] = BASE + "implementations/" + role + "/" + selected + ".h"
            row["class_source"] = row["class_header"][:-1] + "cpp"
            row["registration_source"] = BASE + "registration/" + role.capitalize() + "ProvidersV1.cpp"
        row["parameters"] = {}
        for slot, parameter in sorted(resolved["parameters"].items()):
            if parameter.get("basis") not in ("published", "derived", "assumed"):
                raise ValueError("referenced parameter needs an explicit provenance basis")
            row["parameters"][slot] = dict(parameter, card_parameter=resolved["parameter_refs"][slot],
                                            source_record=source_record(card, parameter))
        row["parameter_basis_counts"] = dict(sorted(Counter(p["basis"] for p in row["parameters"].values()).items()))
        # Exclude paths/identities and provenance: a renamed asset or a different
        # citation does not establish a different numerical configuration.
        values = {slot: {key: parameter[key] for key in ("value", "unit")}
                  for slot, parameter in row["parameters"].items()}
        row["parameter_fingerprint"] = digest(canonical(values).encode())
    return result


def summarize(cards):
    roles = {}
    for role in ROLES:
        rows = [card["components"][role] for card in cards]
        counts = Counter(row["availability"] for row in rows)
        available = [row for row in rows if row["availability"] == "available"]
        roles[role] = {state: counts[state] for state in ("available", "unavailable", "not_applicable", "legacy_unknown")}
        roles[role].update(
            concrete_class_count=len({row["concrete_class"] for row in available if row["concrete_class"]}),
            implementation_id_count=len({row["implementation_id"] for row in available}),
            parameter_configuration_count=len({row["parameter_fingerprint"] for row in available}),
            unmapped_count=sum(row["concrete_class"] is None for row in available))
    return {"card_count": len(cards), "roles": roles}


def collect(root, head):
    paths = sorted((root / "assets/stores").glob("*.json"))
    if not paths:
        raise ValueError("no store cards found")
    cards, manifest, identities = [], {}, set()
    for path in paths:
        if path.is_symlink():
            raise ValueError("store card must be a regular repository input: " + path.name)
        content = path.read_bytes()
        card = describe(json.loads(content))
        if card["id"] in identities:
            raise ValueError("duplicate store identity: " + card["id"])
        identities.add(card["id"])
        card["card_path"] = path.relative_to(root).as_posix()
        cards.append(card)
        manifest[card["card_path"]] = digest(content)
    return {"schema": SCHEMA, "head": head, "evidence_scope": "configuration_and_indexed_source_only",
            "acceptance_note": "Native execution and physical acceptance are not assessed by this report.",
            "cards": cards, "summary": summarize(cards), "input_manifest": manifest}


def bind_source_inputs(document, root):
    paths = {"tools/validation/payload_models/report.py", "tools/models/hangar/hangar/components.py",
             "src/simulation/mission_systems/weapon/FuzeAdapter.cpp",
             "src/simulation/mission_systems/damage/WarheadAdapter.cpp"}
    for card in document["cards"]:
        for row in card["components"].values():
            if row.get("mapping_status") != "indexed_source":
                continue
            paths.update(row[key] for key in ("class_header", "class_source", "registration_source"))
            header = (root / row["class_header"]).read_text(encoding="utf-8")
            if "class " + row["concrete_class"] not in header:
                raise ValueError("missing indexed class declaration: " + row["class_header"])
            registration = (root / row["registration_source"]).read_text(encoding="utf-8")
            if '"' + row["implementation_id"] + '"' not in registration:
                raise ValueError("missing indexed registration: " + row["implementation_id"])
    paths.add(BASE + "registration/EffectFactoriesV1.cpp")
    for path in sorted(paths):
        document["input_manifest"][path] = digest((root / path).read_bytes())
    document["source_index_note"] = "Maintained ID-to-class index with declaration/registration checks; not runtime dispatch verification."


def read_test_results(path):
    content = path.read_bytes()
    if b"<!DOCTYPE" in content.upper():
        raise ValueError("JUnit document types are unsupported")
    tree = ET.fromstring(content)
    cases = []
    for case in tree.iter("testcase"):
        status = "passed"
        if case.find("failure") is not None or case.find("error") is not None:
            status = "failed"
        elif case.find("skipped") is not None or case.get("status") in ("disabled", "notrun"):
            status = "skipped"
        cases.append({"name": case.get("name", ""), "status": status,
                      "time_s": case.get("time"), "output": case.findtext("system-out", "")})
    if not cases:
        raise ValueError("JUnit artifact contains no test cases")
    counts = Counter(case["status"] for case in cases)
    return {"sha256": digest(content), "cases": cases,
            "passed": counts["passed"], "failed": counts["failed"], "skipped": counts["skipped"],
            "scope": "Imported test outcomes only; per-model runtime/physical acceptance and binary provenance are not inferred."}


def render_html(document):
    def e(value):
        return escape(str(value), quote=True)
    sections = []
    for card in document["cards"]:
        rows = []
        for role, row in card["components"].items():
            evidence = e(json.dumps(row, sort_keys=True, indent=2))
            rows.append('<tr><td>' + e(role) + '</td><td>' + e(row["availability"]) + '</td><td>' +
                        e(row.get("concrete_class") or "Unmapped / no executing class declared") + '</td><td>' +
                        e(row.get("basis", "n/a")) + '</td><td><details><summary>Configuration and sources</summary><pre>' +
                        evidence + '</pre></details></td></tr>')
        sections.append('<section><h2>' + e(card["designation"]) + ' <small>' + e(card["id"]) + '</small></h2><p>' +
                        e(card.get("card_path", "")) + '</p><table><thead><tr><th>Role</th><th>Availability</th>' +
                        '<th>Indexed concrete class</th><th>Component basis</th><th>Provenance / limitations</th></tr></thead><tbody>' +
                        ''.join(rows) + '</tbody></table></section>')
    summary = ''.join('<tr><td>' + e(role) + '</td>' + ''.join('<td>' + e(row[key]) + '</td>' for key in
                      ("available", "unavailable", "not_applicable", "legacy_unknown", "concrete_class_count", "unmapped_count")) + '</tr>'
                      for role, row in document["summary"]["roles"].items())
    tests = ''.join('<tr><td>' + e(case["name"]) + '</td><td>' + e(case["status"]) + '</td></tr>'
                    for artifact in document.get("test_artifacts", []) for case in artifact["cases"])
    test_section = ('<h2>Imported test results</h2><p>Failures remain failures. These results do not establish per-model physical acceptance or binary provenance.</p>'
                    '<table><thead><tr><th>Test</th><th>Outcome</th></tr></thead><tbody>' + tests + '</tbody></table>') if tests else ''
    return '''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Payload component evidence</title><style>
body{font:16px system-ui;background:#111923;color:#e2e9ef;max-width:1500px;margin:auto;padding:32px}
h1,h2{color:#fff}small{color:#b0c4d8}p{line-height:1.6}table{width:100%;border-collapse:collapse;margin:20px 0}
th,td{text-align:left;vertical-align:top;border:1px solid #45576a;padding:10px}th{background:#223248}
section{border-top:2px solid #52687e;margin-top:36px}pre{white-space:pre-wrap;overflow-wrap:anywhere;font-size:13px;max-width:640px}
input{font:inherit;padding:12px;width:min(90%,700px)}summary{cursor:pointer}details{min-width:180px}
</style><h1>Payload component evidence</h1><p>Configuration and indexed source only. Runtime evidence and physical acceptance: <strong>not_assessed</strong>.
Available means configured, not calibrated. A provider version is not a maturity level. Shared classes remain shared across variants.
Parameter configuration counts exclude identity/path changes; they do not count complete behavioral or asset differences.</p><p>HEAD: ''' + e(document["head"]) + ' | Cards: ' + e(len(document["cards"])) + '</p>' + test_section + '''
<table><thead><tr><th>Role</th><th>Available</th><th>Unavailable</th><th>Not applicable</th><th>Legacy unknown</th><th>Indexed classes</th><th>Unmapped</th></tr></thead><tbody>''' + summary + '''</tbody></table>
<label>Filter cards, components, classes or source text <input id="filter" type="search" placeholder="Type a card ID, class, role or source"></label>
''' + ''.join(sections) + '''<script>
const sections=[...document.querySelectorAll('section')];
document.getElementById('filter').addEventListener('input',event=>{
 const query=event.target.value.toLowerCase().trim();
 for(const section of sections)section.hidden=!section.textContent.toLowerCase().includes(query);
});
</script></html>'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--native-junit", type=Path, action="append", default=[])
    args = parser.parse_args()
    for path in [args.output_dir, *args.native_junit]:
        if any(part.casefold() == ".codex" for part in path.parts):
            parser.error("excluded path")
    destination = args.output_dir.resolve()
    if any(part.casefold() == ".codex" for part in destination.parts):
        parser.error("excluded output path")
    head = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    document = collect(ROOT, head)
    bind_source_inputs(document, ROOT)
    document["test_artifacts"] = []
    for path in args.native_junit:
        artifact = read_test_results(path)
        artifact["path"] = path.as_posix()
        document["test_artifacts"].append(artifact)
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "report.json").write_text(json.dumps(document, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
    (destination / "report.html").write_text(render_html(document), encoding="utf-8")
    print(json.dumps(document["summary"], sort_keys=True))
    return int(any(row["unmapped_count"] for row in document["summary"]["roles"].values()) or
               any(artifact["failed"] or artifact["skipped"] for artifact in document["test_artifacts"]))


if __name__ == "__main__":
    raise SystemExit(main())
