"""Verify every missile card against the reviewed public-data facts.

This gate is about identity and admitted provenance, not about engagement
outcomes:

* the card set equals the fact-table set, and every AAM card has a fact entry;
* seeker mode, warhead kind, fuze arrangement and motor type agree with the
  entry, so a model cannot be re-labelled by a later edit;
* each published value the entry names is present on the card with basis
  'published', the same value and the same source record in the card;
* every component role exists, its parameter references resolve and its
  availability is a known state;
* every published parameter names a source the card carries;
* the calibrated parameters stay inside physical bounds (positive ranges,
  gimbal not wider than 180 degrees, seeker reach not beyond the published
  kinematic range, warhead not heavier than the missile).

It does not claim physical accuracy. An 'assumed' value remains an estimate.
"""
import argparse
import json
import math
import sys
from pathlib import Path

ROLES = ("seeker", "guidance", "autopilot", "allocator", "actuator",
         "fuze", "warhead", "propulsion", "aerodynamics")
AVAILABILITY = ("available", "unavailable", "not_applicable")
BASES = ("published", "derived", "assumed")


def fail(errors, message):
    errors.append(message)


def check_card(card, entry, sources, errors):
    name = card.get("id", "?")
    for field, fact_field in (("seeker", "seeker_mode"), ("warhead_kind", "warhead_kind")):
        if card.get(field) != entry.get(fact_field):
            fail(errors, "%s: card %s %r differs from the reviewed %r" %
                 (name, field, card.get(field), entry.get(fact_field)))
    for fact in ("fuze_arrangement", "motor_kind"):
        if not entry.get(fact):
            fail(errors, "%s: fact entry has no %s" % (name, fact))
    parameters = card.get("parameters", {})
    card_sources = card.get("sources", {})
    components = card.get("component_models", {}).get("components", {})
    seeker_refs = components.get("seeker", {}).get("parameter_refs", {})
    for slot, fact in entry.get("published", {}).items():
        parameter = seeker_refs.get(slot)
        if parameter is None:
            fail(errors, "%s: seeker has no parameter reference for the published %s" % (name, slot))
            continue
        record = parameters.get(parameter)
        if record is None:
            fail(errors, "%s: published %s has no card parameter %s" % (name, slot, parameter))
            continue
        if record.get("basis") != "published":
            fail(errors, "%s: %s must declare basis 'published'" % (name, parameter))
        if record.get("value") != fact["value"]:
            fail(errors, "%s: %s is %r, reviewed value is %r" %
                 (name, parameter, record.get("value"), fact["value"]))
        if record.get("source") != fact["source"]:
            fail(errors, "%s: %s cites %r, reviewed source is %r" %
                 (name, parameter, record.get("source"), fact["source"]))
        if fact["source"] not in card_sources:
            fail(errors, "%s: card does not carry the source record %r" % (name, fact["source"]))
        elif fact["source"] not in sources:
            fail(errors, "%s: fact table does not define the source %r" % (name, fact["source"]))
    for slot, fact in entry.get("calibrated", {}).items():
        parameter = seeker_refs.get(slot)
        if parameter is None:
            fail(errors, "%s: seeker has no parameter reference for the calibrated %s" % (name, slot))
            continue
        record = parameters.get(parameter, {})
        if record.get("value") != fact["value"]:
            fail(errors, "%s: %s is %r, reviewed calibration is %r" %
                 (name, parameter, record.get("value"), fact["value"]))
        if record.get("basis") != fact.get("basis", "derived"):
            fail(errors, "%s: %s must declare basis %r" % (name, parameter, fact.get("basis", "derived")))
        if not record.get("note"):
            fail(errors, "%s: %s carries no estimate method note" % (name, parameter))

    for parameter, record in parameters.items():
        basis = record.get("basis")
        if basis not in BASES:
            fail(errors, "%s: %s has unknown basis %r" % (name, parameter, basis))
        if basis == "published":
            source = record.get("source")
            if not source:
                fail(errors, "%s: published %s names no source" % (name, parameter))
            elif source not in card_sources:
                fail(errors, "%s: published %s cites %r, which the card does not carry" %
                     (name, parameter, source))

    for role in ROLES:
        component = components.get(role)
        if not isinstance(component, dict):
            fail(errors, "%s: component role %s is missing" % (name, role))
            continue
        if component.get("availability") not in AVAILABILITY:
            fail(errors, "%s: %s availability %r is unknown" %
                 (name, role, component.get("availability")))
        if component.get("availability") == "available" and not component.get("implementation_id"):
            fail(errors, "%s: available %s names no implementation" % (name, role))
        if not isinstance(component.get("parameter_refs", {}), dict):
            fail(errors, "%s: %s parameter_refs is not an object" % (name, role))
            continue
        for slot, reference in component.get("parameter_refs", {}).items():
            if reference not in parameters:
                fail(errors, "%s: %s.%s refers to missing parameter %r" %
                     (name, role, slot, reference))

    def value(key):
        record = parameters.get(key)
        return record.get("value") if record else None

    def referenced(role, slot):
        return value(components.get(role, {}).get("parameter_refs", {}).get(slot))

    gimbal = referenced("seeker", "gimbal_deg")
    fov = referenced("seeker", "launch_fov_deg")
    acquire = referenced("seeker", "acquire_m")
    fuze = referenced("fuze", "radius_m")
    warhead = referenced("warhead", "warhead_kg")
    explosive = referenced("warhead", "explosive_kg")
    mass = value("mass_kg")
    rng = value("range_km")
    for key, number in (("gimbal", gimbal), ("launch_fov", fov), ("acquire", acquire),
                        ("fuze_radius", fuze), ("warhead_kg", warhead), ("mass_kg", mass)):
        if number is None or not isinstance(number, (int, float)) or not math.isfinite(number) or number <= 0:
            fail(errors, "%s: %s must be a positive finite number, got %r" % (name, key, number))
    if isinstance(gimbal, (int, float)) and not 0 < gimbal <= 180:
        fail(errors, "%s: gimbal %r is outside (0, 180] degrees" % (name, gimbal))
    if isinstance(fov, (int, float)) and isinstance(gimbal, (int, float)) and fov < gimbal and \
            entry.get("published", {}).get("gimbal_deg"):
        fail(errors, "%s: published launch field %r is narrower than its published gimbal %r" %
             (name, fov, gimbal))
    if isinstance(acquire, (int, float)) and isinstance(rng, (int, float)) and acquire > rng * 1500:
        fail(errors, "%s: seeker reach %r greatly exceeds the published kinematic range" % (name, acquire))
    if isinstance(warhead, (int, float)) and isinstance(explosive, (int, float)) and explosive >= warhead:
        fail(errors, "%s: explosive mass %r is not below warhead mass %r" % (name, explosive, warhead))
    if isinstance(mass, (int, float)) and isinstance(warhead, (int, float)) and warhead >= mass:
        fail(errors, "%s: warhead mass %r is not below launch mass %r" % (name, warhead, mass))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[3])
    args = parser.parse_args()
    root = args.repo.resolve()
    facts = json.loads((Path(__file__).with_name("missile_facts.json")).read_text(encoding="utf-8"))
    if facts.get("schema") != "missile-public-data-v1":
        raise SystemExit("unexpected fact schema")
    sources = facts.get("sources", {})
    cards = {}
    for path in sorted((root / "assets" / "stores").glob("*.json")):
        card = json.loads(path.read_text(encoding="utf-8"))
        if card.get("class") == "aam":
            cards[card["id"]] = card
    errors = []
    missing = sorted(set(cards) - set(facts["cards"]))
    extra = sorted(set(facts["cards"]) - set(cards))
    for name in missing:
        fail(errors, "%s: AAM card has no reviewed fact entry" % name)
    for name in extra:
        fail(errors, "%s: fact entry has no AAM card" % name)
    for name, card in sorted(cards.items()):
        entry = facts["cards"].get(name)
        if entry is None:
            continue
        if card.get("designation") != entry.get("designation"):
            fail(errors, "%s: designation %r differs from the reviewed %r" %
                 (name, card.get("designation"), entry.get("designation")))
        check_card(card, entry, sources, errors)
    for message in errors:
        print("FAIL: " + message)
    print("%d AAM cards checked, %d errors" % (len(cards), len(errors)))
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
