"""Mass accounting (docs/hangar.md, Mass accounting): every kilogram of every design in one ledger - its empty weight,
its fixed load, its tanks' fuel, the stores on its stations - and none in two. Each [[mass.payload]] has its kind; the
flight model built from a design holds what the design says; nothing a station can carry is also a fixed load, and a
store is fixed load; every store has a flight model; every loadout's stores fit the stations they hang on."""
import glob
import json
import os
import re
import tomllib
import unittest

from hangar import mass, pipeline

LB = 0.45359237
#: A fixed load named like a store: none may be.
STORE_NAMES = re.compile(r"missile|bomb|rocket|weapon|\btanks?\b|\bpods?\b", re.I)


def designs():
    """(name, design, directory) of every fixed-wing design (a rotorcraft's weight is one figure: hangar/rotorcraft)."""
    root = os.path.join(pipeline.repo_root(), "aircraft")
    for path in sorted(glob.glob(os.path.join(root, "*", "*.toml"))):
        name = os.path.splitext(os.path.basename(path))[0]
        if os.path.basename(os.path.dirname(path)) != name:
            continue
        with open(path, "rb") as f:
            spec = tomllib.load(f)
        if "empty" in spec.get("mass", {}):
            yield name, spec, os.path.dirname(path)


def kilograms(value, unit):
    return float(value) * (1.0 if (unit or "LBS").upper() == "KG" else LB)


def flight_model(path):
    """A JSBSim aircraft's empty weight (kg), its point masses [(name, kg)] and its fuel tanks' capacities (kg)."""
    with open(path, encoding="utf-8") as f:
        text = f.read()
    empty = re.search(r"<emptywt(?:\s+unit\s*=\s*\"(\w+)\")?\s*>\s*([-+0-9.eE]+)", text)
    points = []
    for m in re.finditer(r"<pointmass([^>]*)>(.*?)</pointmass>", text, re.S):
        name = re.search(r"name\s*=\s*\"([^\"]*)\"", m.group(1))
        weight = re.search(r"<weight(?:\s+unit\s*=\s*\"(\w+)\")?\s*>\s*([-+0-9.eE]+)", m.group(2))
        points.append((name.group(1) if name else "", kilograms(weight.group(2), weight.group(1))))
    tanks = []
    for m in re.finditer(r"<tank([^>]*)>(.*?)</tank>", text, re.S):
        capacity = re.search(r"<capacity(?:\s+unit\s*=\s*\"(\w+)\")?\s*>\s*([-+0-9.eE]+)", m.group(2))
        if "FUEL" in m.group(1).upper() and capacity:
            tanks.append(kilograms(capacity.group(2), capacity.group(1)))
    return kilograms(empty.group(2), empty.group(1)), points, tanks


def fit_of(directory):
    """A design's fit: (its payloads, its stations by number, its loadouts); empty without one."""
    path = os.path.join(directory, "payloads.json")
    if not os.path.isfile(path):
        return [], {}, []
    with open(path, encoding="utf-8") as f:
        fit = json.load(f)
    library = os.path.join(pipeline.repo_root(), "assets", "payloads")
    stations = {}
    for p in fit.get("payloads", []):
        with open(os.path.join(library, p["card"] + ".json"), encoding="utf-8") as f:
            card = json.load(f)
        if card.get("family") == "store_station":
            stations.update({s["number"]: s for s in card.get("stations", [])})
    return fit.get("payloads", []), stations, fit.get("loadouts", [])


def store_cards():
    out = {}
    for path in glob.glob(os.path.join(pipeline.repo_root(), "assets", "stores", "*.json")):
        with open(path, encoding="utf-8") as f:
            card = json.load(f)
        parameters = card.get("parameters", {})
        out[card["id"]] = (card.get("class"), (parameters.get("mass_kg") or {}).get("value", 0.0) + (parameters.get("fuel_kg") or {}).get("value", 0.0))
    return out


class Ledgers(unittest.TestCase):
    def test_every_fixed_load_has_its_kind(self):
        # its people, its built-in equipment, its small items; nothing that is a store (mass.check_payload raises)
        for name, spec, _ in designs():
            for p in spec["mass"].get("payload", []):
                with self.subTest(design=name, payload=p.get("name")):
                    mass.check_payload(p)
                    self.assertIn(p["kind"], mass.PAYLOAD_KINDS)
                    if p["kind"] not in ("crew", "passenger"):  # (a weapons officer is crew)
                        self.assertIsNone(STORE_NAMES.search(p["name"]), "%s: %r is named as a store: it is weighed on its station, "
                                          "as the store it is - not as fixed load" % (name, p["name"]))
        for kind in mass.STORE_KINDS:
            with self.assertRaises(ValueError):
                mass.check_payload({"name": "x", "kind": kind, "mass": 100.0})

    def test_every_store_has_a_flight_model_and_a_model_to_draw(self):
        # what a station carries can leave it: its card names a JSBSim model (hangar's, in aircraft/stores or a design of
        # its own; the Mk 82's is JSBSim's) and the viewer has a model of it
        root = pipeline.repo_root()
        for path in sorted(glob.glob(os.path.join(root, "assets", "stores", "*.json"))):
            with open(path, encoding="utf-8") as f:
                card = json.load(f)
            with self.subTest(store=card["id"]):
                self.assertTrue(card.get("model", "").startswith("jsbsim:"), "its card names no flight model")
                name = card["model"][7:]
                flown = [os.path.join(root, "aircraft", "stores", name, name + ".xml"), os.path.join(root, "aircraft", name, name + ".xml"),
                         os.path.join(root, "third_party", "jsbsim", "aircraft", name, name + ".xml")]
                self.assertTrue(any(os.path.isfile(x) for x in flown), "no flight model %s" % name)
                drawn = [os.path.join(root, "assets", "models", name + ".glb"), os.path.join(root, "aircraft", name, name + ".glb")]
                self.assertTrue(any(os.path.isfile(x) for x in drawn), "no model for the viewer")

    def test_the_flight_model_holds_what_the_design_says(self):
        for name, spec, directory in designs():
            xml = os.path.join(directory, name + ".xml")
            if not os.path.isfile(xml):
                continue
            with self.subTest(design=name):
                m = spec["mass"]
                empty, points, tanks = flight_model(xml)
                self.assertAlmostEqual(empty, m["empty"], delta=0.5)
                fixed = [(n, kg) for n, kg in points if not n.startswith("station ")]
                self.assertEqual([n for n, _ in fixed], [p["name"] for p in m.get("payload", [])])
                for (n, kg), p in zip(fixed, m.get("payload", [])):
                    self.assertAlmostEqual(kg, p["mass"], delta=0.05, msg=n)
                self.assertAlmostEqual(sum(tanks), sum(t["capacity"] for t in m.get("tank", [])), delta=1.0)
                # its stations' point masses: one a station of its fit, each empty until the store management loads it
                _, stations, _ = fit_of(directory)
                on = [(n, kg) for n, kg in points if n.startswith("station ")]
                self.assertEqual(sorted(n for n, _ in on), sorted("station %d" % k for k in stations))
                self.assertTrue(all(kg == 0.0 for _, kg in on))

    def test_nothing_is_weighed_twice(self):
        for name, spec, directory in designs():
            payloads, stations, loadouts = fit_of(directory)
            with self.subTest(design=name):
                # no payload of a fit keeps a station to itself, and no fixed load hangs on one: what a station carries is
                # a store, loaded there - weighed once, by the store management
                self.assertFalse([p["name"] for p in payloads if p.get("stations")])
                self.assertFalse([p["name"] for p in spec["mass"].get("payload", []) if p.get("stations") is not None])

    def test_every_loadout_fits_its_stations(self):
        cards = store_cards()
        for name, _, directory in designs():
            _, stations, loadouts = fit_of(directory)
            for lo in loadouts:
                for item in lo["stations"]:
                    with self.subTest(design=name, loadout=lo["name"], station=item["station"]):
                        self.assertIn(item["store"], cards)
                        self.assertIn(item["station"], stations)
                        kind, kg = cards[item["store"]]
                        station = stations[item["station"]]
                        self.assertIn(kind, station["takes"])
                        self.assertLessEqual(int(item.get("count", 1)) * kg, station["max_kg"] + 0.5)


if __name__ == "__main__":
    unittest.main()
