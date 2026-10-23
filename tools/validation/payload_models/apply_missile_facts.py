"""Apply the reviewed public-data facts to missile store cards.

The fact table (missile_facts.json) is the reviewed oracle. This tool writes
only the published values it names, into the card parameters that the component
model consumes, and copies the source record into the card. It never touches a
card that has no published fact, and it is idempotent: a card already carrying
the value, basis and source is left byte-identical.

No simulation outcome is read. Nothing here calibrates a missile to a scene.
"""
import argparse
import json
import re
import sys
from pathlib import Path

INDENT = " " * 4
FIELD = " " * 6
PARAMETER_UNITS = {"gimbal_deg": "deg", "launch_fov_deg": "deg", "acquire_m": "m"}


def _block(key, entry, unit):
    lines = ['%s"%s": {' % (INDENT, key),
             '%s"value": %s,' % (FIELD, json.dumps(entry["value"])),
             '%s"unit": "%s",' % (FIELD, unit),
             '%s"uncertainty": %s,' % (FIELD, json.dumps(entry["uncertainty"])),
             '%s"basis": "%s",' % (FIELD, entry.get("basis", "published"))]
    if entry.get("source"):
        lines.append('%s"source": "%s",' % (FIELD, entry["source"]))
    lines += ['%s"note": %s' % (FIELD, json.dumps(entry["note"], ensure_ascii=False)),
              '%s}' % INDENT]
    return "\n".join(lines)


def _replace_parameter(text, key, entry, unit):
    pattern = re.compile(r'%s"%s": \{\n(?:.*?\n)*?%s\}' % (re.escape(INDENT), re.escape(key), re.escape(INDENT)))
    matches = pattern.findall(text)
    if len(matches) != 1:
        raise ValueError("card does not have exactly one %s parameter" % key)
    return pattern.sub(lambda _: _block(key, entry, unit), text, count=1)


def _source_block(name, record):
    lines = ['%s"%s": {' % (INDENT, name)]
    for field in ("evidence_kind", "title", "publisher", "url", "date", "accessed"):
        if field in record:
            lines.append('%s"%s": %s,' % (FIELD, field, json.dumps(record[field], ensure_ascii=False)))
    lines.append('%s"note": %s' % (FIELD, json.dumps(record.get("note", ""), ensure_ascii=False)))
    lines.append('%s},' % INDENT)
    return "\n".join(lines)


def _ensure_source(text, name, record):
    if re.search(r'%s"%s": \{' % (re.escape(INDENT), re.escape(name)), text):
        return text
    anchor = '"sources": {\n'
    if text.count(anchor) != 1:
        raise ValueError("card has no single sources object")
    return text.replace(anchor, anchor + _source_block(name, record) + "\n", 1)


def apply_card(path, entry, sources):
    raw = path.read_text(encoding="utf-8")
    text = raw
    card = json.loads(raw)
    refs = card.get("component_models", {}).get("components", {}).get("seeker", {}).get("parameter_refs", {})
    for slot, fact in sorted(entry.get("published", {}).items()):
        parameter = refs.get(slot)
        if parameter is None:
            raise ValueError("%s: seeker has no parameter reference for %s" % (path.name, slot))
        record = sources.get(fact["source"])
        if record is None:
            raise ValueError("%s: fact source %s is not defined" % (path.name, fact["source"]))
        text = _replace_parameter(text, parameter, fact, PARAMETER_UNITS[slot])
        text = _ensure_source(text, fact["source"], record)
    for slot, fact in sorted(entry.get("calibrated", {}).items()):
        parameter = refs.get(slot)
        if parameter is None:
            raise ValueError("%s: seeker has no parameter reference for %s" % (path.name, slot))
        text = _replace_parameter(text, parameter, fact, PARAMETER_UNITS[slot])
        if fact.get("source"):
            record = sources.get(fact["source"]) or card.get("sources", {}).get(fact["source"])
            if record is None:
                raise ValueError("%s: calibrated source %s is not defined" % (path.name, fact["source"]))
            text = _ensure_source(text, fact["source"], record)
    if text != raw:
        path.write_text(text, encoding="utf-8", newline="")
    return text != raw


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[3])
    parser.add_argument("--check", action="store_true", help="report drift without writing")
    args = parser.parse_args()
    root = args.repo.resolve()
    facts = json.loads((Path(__file__).with_name("missile_facts.json")).read_text(encoding="utf-8"))
    sources = facts["sources"]
    drift = 0
    for card_id, entry in sorted(facts["cards"].items()):
        path = root / "assets" / "stores" / (card_id + ".json")
        if not path.is_file():
            raise SystemExit("missing card: " + str(path))
        if args.check:
            before = path.read_text(encoding="utf-8")
            # The check path re-derives what apply would write without touching disk.
            import tempfile
            with tempfile.TemporaryDirectory() as tmp:
                copy = Path(tmp) / path.name
                copy.write_text(before, encoding="utf-8", newline="")
                changed = apply_card(copy, entry, sources)
                if changed:
                    drift += 1
                    print("drift: " + card_id)
        else:
            if apply_card(path, entry, sources):
                print("updated: " + card_id)
    if args.check and drift:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
