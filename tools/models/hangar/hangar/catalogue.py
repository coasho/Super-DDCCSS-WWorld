"""Read-only store catalogue integrity, without accepting installations or physics.

Run ``python -m hangar.catalogue --root ROOT [--json]``. Only the explicit
aircraft, store-card and bundled stock-aircraft trees are scanned. Visual aliases
are reported separately and never provide payload-fit inheritance.
"""
import argparse
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import stat
import sys
import tomllib
import xml.etree.ElementTree as ET


SCHEMA = "fsim.store-catalogue.v1"
_IDENTITY = re.compile(r"[A-Za-z0-9][A-Za-z0-9_-]*\Z")
_RESERVED = {"con", "prn", "aux", "nul"} | {"%s%d" % (p, n) for p in ("com", "lpt") for n in range(1, 10)}
# The element families loaded by JSBSim FGPropulsion and FGEngine.
_ENGINE_ROOTS = frozenset(("piston_engine", "turbine_engine", "turboprop_engine", "rocket_engine",
                           "electric_engine", "brushless_dc_motor"))
_THRUSTER_ROOTS = frozenset(("propeller", "nozzle", "rotor", "direct"))


def _pairs(items):
    result = {}
    for key, value in items:
        if key in result:
            raise ValueError("duplicate JSON key: %s" % key)
        result[key] = value
    return result


def _constant(value):
    raise ValueError("nonfinite JSON constant: %s" % value)


class _Audit:
    def __init__(self, root):
        self.result = {"schema": SCHEMA, "counts": {}, "aircraft": [], "stores": [],
                       "stock_definitions": [], "legacy_stock_definitions": [],
                       "structural_passed": False, "issues": [], "pending": [],
                       "source_assessment": "not_assessed", "installation_assessment": "not_assessed"}
        raw = Path(root)
        if any(p.casefold() == ".codex" for p in raw.parts):
            raise ValueError("excluded root")
        self.root = Path(os.path.abspath(raw))
        if any(p.casefold() == ".codex" for p in self.root.parts):
            raise ValueError("excluded root")
        # Inspect link metadata before following any user-supplied root component.
        for path in reversed((self.root,) + tuple(self.root.parents)):
            self._plain(path)
        if not self.root.is_dir():
            raise ValueError("root must be a permitted directory")
        self.cards, self.designs, self.fits, self.aliases = {}, {}, {}, {}
        self.references = set()

    @staticmethod
    def _plain(path):
        mode = path.lstat()
        if stat.S_ISLNK(mode.st_mode) or getattr(mode, "st_file_attributes", 0) & getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0):
            raise ValueError("links and junctions are not catalogue inputs")

    def note(self, code, path, detail, *, pending=False):
        self.result["pending" if pending else "issues"].append({"code": code, "path": path, "detail": detail})

    def identity(self, value, path, label="identity"):
        if not isinstance(value, str) or not _IDENTITY.fullmatch(value) or value.casefold() in _RESERVED:
            self.note("invalid-identity", path, "%s must be one safe catalogue identifier" % label)
            return False
        return True

    def path(self, relative):
        posix = PurePosixPath(relative)
        if (not relative or posix.is_absolute() or "\\" in relative or ":" in relative or
                posix.as_posix() != relative or
                any(p in ("", ".", "..") or p.casefold() == ".codex" for p in relative.split("/"))):
            raise ValueError("noncanonical or excluded catalogue path")
        path = self.root
        for part in posix.parts:
            path = path / part
            try:
                self._plain(path)
            except FileNotFoundError:
                return None
        return path

    def exists(self, relative):
        try:
            return self.path(relative) is not None
        except (OSError, ValueError) as error:
            self.note("unsafe-path", relative, str(error))
            return False

    def entries(self, relative, *, required=True):
        try:
            path = self.path(relative)
            if path is None:
                if required:
                    self.note("missing-directory", relative, "expected catalogue directory is absent")
                return []
            names = {}
            with os.scandir(path) as entries:
                for entry in entries:
                    if entry.name.casefold() == ".codex":
                        continue  # prune before any descent or file inspection
                    names.setdefault(entry.name.casefold(), []).append(entry.name)
            result = []
            for folded in sorted(names):
                if len(names[folded]) != 1:
                    self.note("case-ambiguous-path", relative, "directory has case-ambiguous entries")
                else:
                    result.append(names[folded][0])
            return sorted(result)
        except (OSError, ValueError) as error:
            self.note("unreadable-directory", relative, str(error))
            return []

    def text(self, relative, missing="missing-file", *, optional=False):
        try:
            path = self.path(relative)
            if path is None:
                if not optional:
                    self.note(missing, relative, "expected referenced file is absent")
                return None
            if not path.is_file():
                raise ValueError("expected a regular file")
            return path.read_text(encoding="utf-8-sig")
        except (OSError, UnicodeError, ValueError) as error:
            self.note("unreadable-file", relative, str(error))
            return None

    def json(self, relative, missing):
        text = self.text(relative, missing)
        if text is None:
            return None
        try:
            value = json.loads(text, object_pairs_hook=_pairs, parse_constant=_constant)
            if not isinstance(value, dict):
                raise ValueError("expected a JSON object")
            return value
        except ValueError as error:
            self.note("malformed-json", relative, str(error))
            return None

    def sequence(self, value, path, label):
        if not isinstance(value, list) or any(not isinstance(item, dict) for item in value):
            self.note("malformed-list", path, "%s must be a list of objects" % label)
            return []
        return value

    def xml(self, relative, missing):
        text = self.text(relative, missing)
        if text is None:
            return None
        try:
            if "<!DOCTYPE" in text.upper() or "<!ENTITY" in text.upper():
                raise ValueError("XML declarations with entities are unsupported")
            return ET.fromstring(text)
        except (ET.ParseError, ValueError) as error:
            self.note("malformed-xml", relative, str(error))
            return None

    def scan_cards(self):
        seen_ids, seen_models = {}, {}
        for filename in self.entries("assets/stores"):
            if not filename.lower().endswith(".json"):
                continue
            relative, cid = "assets/stores/" + filename, filename[:-5]
            if not filename.endswith(".json"):
                self.note("noncanonical-path", relative, "store-card extension must be .json")
            if not self.identity(cid, relative, "card filename"):
                continue
            data = self.json(relative, "missing-store-card")
            if data is None:
                continue
            actual = data.get("id")
            if self.identity(actual, relative, "card id"):
                folded = actual.casefold()
                if folded in seen_ids:
                    self.note("duplicate-card-id", relative, "card id is also declared by %s" % seen_ids[folded])
                seen_ids[folded] = relative
                if actual != cid:
                    self.note("card-identity-mismatch", relative, "card id must match its exact filename")
            model = data.get("model")
            model_id = model[7:] if isinstance(model, str) and model.startswith("jsbsim:") else None
            if not self.identity(model_id, relative, "jsbsim model id"):
                model_id = None
            elif model_id.casefold() in seen_models:
                self.note("duplicate-model-id", relative, "model is also declared by %s" % seen_models[model_id.casefold()])
            else:
                seen_models[model_id.casefold()] = relative
            self.cards[cid] = {"id": cid, "card": relative, "class": data.get("class") if isinstance(data.get("class"), str) else None,
                               "model": "jsbsim:" + model_id if model_id else None,
                               "design": None, "generated_model": None, "generated_references": [], "used_by": []}
        if not self.cards:
            self.note("empty-store-catalogue", "assets/stores", "store-card denominator cannot be empty")

    def scan_designs(self):
        for name in self.entries("aircraft"):
            try:
                directory = self.path("aircraft/" + name)
                if directory is None or not directory.is_dir():
                    continue
            except (OSError, ValueError) as error:
                self.note("unsafe-design-path", "aircraft/" + name, str(error))
                continue
            if not self.identity(name, "aircraft/" + name, "design directory"):
                continue
            relative = "aircraft/%s/%s.toml" % (name, name)
            if not self.exists(relative):
                continue
            text = self.text(relative)
            if text is None:
                continue
            try:
                data = tomllib.loads(text).get("aircraft")
                if not isinstance(data, dict):
                    raise ValueError("expected [aircraft] table")
            except (ValueError, tomllib.TOMLDecodeError) as error:
                self.note("malformed-design", relative, str(error))
                continue
            if not self.identity(data.get("name"), relative, "aircraft name") or data.get("name") != name:
                self.note("design-identity-mismatch", relative, "aircraft name must match its exact directory and TOML filename")
            if not isinstance(data.get("category"), str) or not data["category"].strip():
                self.note("malformed-design", relative, "aircraft category must be a nonempty string")
                continue
            if "card" in data or data.get("category") in ("missile", "store"):
                cid = data.get("card")
                if self.identity(cid, relative, "design card"):
                    self.designs.setdefault(cid, []).append((name, relative))
                continue
            row = {"id": name, "design": relative, "fit": "aircraft/%s/payloads.json" % name,
                   "loadouts": [], "visual_aliases": []}
            aliases = data.get("stands_in_for", [])
            if not isinstance(aliases, list):
                self.note("malformed-aliases", relative, "stands_in_for must be a list of identifiers")
                aliases = []
            for alias in aliases:
                if not self.identity(alias, relative, "visual alias"):
                    continue
                if any(alias.casefold() == existing.casefold() for existing in self.aliases):
                    self.note("duplicate-visual-alias", relative, "visual alias is declared more than once")
                    continue
                self.aliases[alias] = name
                row["visual_aliases"].append(alias)
            row["visual_aliases"].sort()
            self.result["aircraft"].append(row)
        if not self.result["aircraft"]:
            self.note("empty-aircraft-roster", "aircraft", "authored-aircraft denominator cannot be empty")

    def resolve_stores(self):
        for cid, paths in sorted(self.designs.items()):
            if len(paths) != 1:
                self.note("duplicate-store-design", paths[0][1], "multiple designs declare card %s" % cid)
            if cid not in self.cards:
                self.note("missing-store-card", paths[0][1], "design names absent store card %s" % cid)
        for cid, row in sorted(self.cards.items()):
            paths = self.designs.get(cid, [])
            if not paths:
                self.note("missing-store-design", row["card"], "no authored store design declares this card")
            elif len(paths) == 1:
                name, row["design"] = paths[0]
                if row["model"] != "jsbsim:" + name:
                    self.note("model-design-mismatch", row["card"], "card model must identify its one declared design")
            if not row["model"]:
                continue
            model = row["model"][7:]
            relative = "aircraft/stores/%s/%s.xml" % (model, model)
            row["generated_model"] = relative
            xml = self.xml(relative, "missing-generated-model")
            if xml is None:
                continue
            if xml.tag != "fdm_config":
                self.note("invalid-generated-model", relative, "expected fdm_config root element")
            refs = set()
            for engine in xml.findall("./propulsion/engine"):
                for node in [engine] + list(engine.findall("thruster")):
                    token = node.get("file")
                    if self.identity(token, relative, "generated engine/thruster reference"):
                        ref = "aircraft/stores/%s/Engines/%s.xml" % (model, token)
                        refs.add(ref)
                        document = self.xml(ref, "missing-generated-reference")
                        supported = _ENGINE_ROOTS if node.tag == "engine" else _THRUSTER_ROOTS
                        if document is not None and document.tag not in supported:
                            self.note("invalid-generated-reference-root", ref,
                                      "expected a supported JSBSim %s root" % node.tag)
            row["generated_references"] = sorted(refs)
        self.result["stores"] = [self.cards[cid] for cid in sorted(self.cards)]

    def fit(self, identity, relative):
        if identity in self.fits:
            return self.fits[identity]
        row = {"path": relative, "loadouts": []}
        self.fits[identity] = row
        data = self.json(relative, "missing-fit")
        if data is None:
            return row
        if data.get("aircraft") != identity:
            self.note("fit-identity-mismatch", relative, "fit aircraft must match its exact aircraft identifier")
        stations = set()
        for payload in self.sequence(data.get("payloads", []), relative, "payloads"):
            cid = payload.get("card")
            if not self.identity(cid, relative, "payload card"):
                continue
            path = "assets/payloads/%s.json" % cid
            card = self.json(path, "missing-payload-card")
            if card is None:
                continue
            if card.get("id") != cid:
                self.note("payload-card-identity-mismatch", path, "payload card id must match its exact filename")
            if card.get("family") != "store_station":
                continue
            for station in self.sequence(card.get("stations", []), path, "stations"):
                number = station.get("number")
                if type(number) is not int or number <= 0:
                    self.note("invalid-station-number", path, "station number must be a positive integer")
                elif number in stations:
                    self.note("duplicate-station-number", relative, "station number is declared more than once")
                else:
                    stations.add(number)
        names = set()
        for loadout in self.sequence(data.get("loadouts", []), relative, "loadouts"):
            name = loadout.get("name")
            if not isinstance(name, str) or not name.strip():
                self.note("invalid-loadout-name", relative, "loadout needs a nonempty name")
                continue
            if name.casefold() in names:
                self.note("duplicate-loadout-name", relative, "loadout names must be unique")
            names.add(name.casefold())
            out, used = {"name": name, "stations": []}, set()
            for item in self.sequence(loadout.get("stations", []), relative, "loadout stations"):
                number, cid = item.get("station"), item.get("store")
                if type(number) is not int or number <= 0:
                    self.note("invalid-station-number", relative, "loadout station must be a positive integer")
                elif number in used:
                    self.note("duplicate-loadout-station", relative, "one loadout assigns a station more than once")
                else:
                    used.add(number)
                    if number not in stations:
                        self.note("unresolved-loadout-station", relative, "loadout references undeclared station %d" % number)
                if not self.identity(cid, relative, "loadout store"):
                    continue
                self.references.add(cid)
                if cid not in self.cards:
                    self.note("missing-store-card", relative, "loadout names absent store card %s" % cid)
                else:
                    self.cards[cid]["used_by"].append({"aircraft": identity, "loadout": name,
                                                       "station": number if type(number) is int else None})
                out["stations"].append({"station": number if type(number) is int else None, "store": cid})
            row["loadouts"].append(out)
        if not row["loadouts"]:
            self.note("no-loadouts", relative, "no explicit store loadout; not a not-fitted decision", pending=True)
        return row

    def registrations(self):
        registered, folded = {}, set()
        targets = {row["id"] for row in self.result["aircraft"]}
        targets.update(name for paths in self.designs.values() for name, _ in paths)
        relative = "aircraft/models.txt"
        text = self.text(relative, optional=True)
        for line in (text or "").splitlines():
            tokens = line.split("#", 1)[0].split()
            if not tokens:
                continue
            if len(tokens) != 5:
                self.note("malformed-visual-registration", relative, "expected alias, design and three offsets")
                continue
            try:
                if not all(math.isfinite(float(token)) for token in tokens[2:]):
                    raise ValueError("nonfinite offset")
            except ValueError:
                self.note("malformed-visual-registration", relative, "visual offsets must be finite numbers")
                continue
            alias, target = tokens[:2]
            if not self.identity(alias, relative, "registered alias") or not self.identity(target, relative, "registered design"):
                continue
            if target not in targets:
                self.note("missing-visual-target", relative, "%s names absent authored design %s" % (alias, target))
            if alias.casefold() in folded:
                self.note("duplicate-visual-registration", relative, "visual alias is registered more than once")
                continue
            folded.add(alias.casefold())
            registered[alias] = target
        for alias, target in sorted(self.aliases.items()):
            if alias not in registered:
                self.note("visual-alias-unregistered", relative, "%s -> %s is declared but not registered" % (alias, target), pending=True)
            elif registered[alias] != target:
                self.note("visual-alias-mismatch", relative, "%s registration disagrees with its design declaration" % alias)
        for alias in sorted(set(registered) - set(self.aliases)):
            self.note("undeclared-visual-registration", relative, "%s has no matching design declaration" % alias, pending=True)

    def stock(self, relative="third_party/jsbsim/aircraft"):
        for filename in self.entries(relative):
            path = relative + "/" + filename
            try:
                real = self.path(path)
                if real is None:
                    continue
                if real.is_dir():
                    self.stock(path)
                    continue
            except (OSError, ValueError) as error:
                self.note("unsafe-stock-path", path, str(error))
                continue
            if not filename.lower().endswith(".xml"):
                continue
            xml = self.xml(path, "missing-stock-definition")
            if xml is None or xml.tag not in ("fdm_config", "FDM_CONFIG"):
                continue
            # Ancillary system/engine XML filenames are not aircraft identities.
            # Their enumerated paths were checked before reading; only fdm roots
            # may now supply an identifier used to look up another file.
            identity = filename[:-4]
            if not self.identity(identity, path, "stock definition filename"):
                continue
            fit_path = "aircraft/%s/payloads.json" % identity
            fit = fit_path if self.exists(fit_path) else None
            if fit:
                self.fit(identity, fit)
            legacy = xml.tag == "FDM_CONFIG"
            row = {"id": identity, "path": path, "fit": fit, "visual_alias": self.aliases.get(identity)}
            self.result["legacy_stock_definitions" if legacy else "stock_definitions"].append(row)
            self.note("legacy-stock-applicability-pending" if legacy else "stock-applicability-pending", path,
                      "SDK applicability and equipment installation are unassessed; visual aliases do not supply a fit", pending=True)

    def run(self):
        self.scan_cards()
        self.scan_designs()
        self.resolve_stores()
        for row in self.result["aircraft"]:
            row["loadouts"] = self.fit(row["id"], row["fit"])["loadouts"]
            self.note("equipment-coverage-not-assessed", row["fit"],
                      "fit presence does not establish actual equipment installation or complete equipment coverage", pending=True)
        self.registrations()
        self.stock()
        for cid, row in sorted(self.cards.items()):
            row["used_by"].sort(key=lambda value: (value["aircraft"], value["loadout"], value["station"] or 0))
            if not row["used_by"]:
                self.note("unused-store-card", row["card"], "no explicit fit loadout references this card", pending=True)
        self.result["stock_definitions"].sort(key=lambda value: value["path"])
        self.result["legacy_stock_definitions"].sort(key=lambda value: value["path"])
        stock_ids = {}
        for row in self.result["stock_definitions"]:
            folded = row["id"].casefold()
            if folded in stock_ids:
                self.note("duplicate-stock-id", row["path"], "stock identity is also declared by %s" % stock_ids[folded])
            stock_ids[folded] = row["path"]
        self.result["counts"] = {"authored_aircraft": len(self.result["aircraft"]), "store_cards": len(self.cards),
                                 "loadouts": sum(len(fit["loadouts"]) for fit in self.fits.values()),
                                 "referenced_stores": len(self.references), "stock_definitions": len(self.result["stock_definitions"])}
        for key in ("issues", "pending"):
            self.result[key] = sorted(self.result[key], key=lambda value: (value["path"], value["code"], value["detail"]))
        self.result["structural_passed"] = not self.result["issues"]
        return self.result


def audit(root):
    """Return deterministic catalogue links and pending scope; never run or write models."""
    try:
        checker = _Audit(root)
    except (OSError, ValueError, TypeError) as error:
        return {"schema": SCHEMA, "counts": {"authored_aircraft": 0, "store_cards": 0, "loadouts": 0,
                                             "referenced_stores": 0, "stock_definitions": 0},
                "aircraft": [], "stores": [], "stock_definitions": [], "legacy_stock_definitions": [],
                "structural_passed": False, "issues": [{"code": "invalid-root", "path": "", "detail": str(error)}],
                "pending": [], "source_assessment": "not_assessed", "installation_assessment": "not_assessed"}
    return checker.run()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--root", required=True)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)
    result = audit(args.root)
    if args.json:
        print(json.dumps(result, sort_keys=True, indent=2, allow_nan=False))
    else:
        print("Catalogue structural integrity: %s; pending equipment coverage: %d" % (
            "passed" if result["structural_passed"] else "failed", len(result["pending"])))
        print("Source and installation assessments: not_assessed; no physical or functioning-model acceptance")
        print(json.dumps(result["counts"], sort_keys=True))
        for issue in result["issues"]:
            print("%s: %s: %s" % (issue["code"], issue["path"], issue["detail"]))
        for pending in result["pending"]:
            print("pending %s: %s: %s" % (pending["code"], pending["path"], pending["detail"]))
    return 0 if result["structural_passed"] else 3


if __name__ == "__main__":
    sys.exit(main())
