"""Mass accounting in the simulator (docs/hangar.md, Mass accounting): every aircraft weighs its empty weight, its fixed
load and its tanks' fuel as it spawns; each loadout adds what its stores weigh, no more, and unloaded it weighs what it
did; every store is a vehicle of its own once it leaves - none is fixed load; an external tank's fuel is the tank's until it is drawn, and drawn
it is neither lost nor made."""
import glob
import json
import os
import tomllib
import unittest

import fsim

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))


def make_world():
    return fsim.World("py-mass", publish=False, workers=1, pin_workers=False, seed=5, terrain=False)


def designs():
    for path in sorted(glob.glob(os.path.join(ROOT, "aircraft", "*", "*.toml"))):
        name = os.path.splitext(os.path.basename(path))[0]
        directory = os.path.dirname(path)
        if os.path.basename(directory) != name or not os.path.isfile(os.path.join(directory, name + ".xml")):
            continue
        with open(path, "rb") as f:
            spec = tomllib.load(f)
        if spec.get("aircraft", {}).get("category") == "missile" or "empty" not in spec.get("mass", {}):
            continue
        fit = {}
        if os.path.isfile(os.path.join(directory, "payloads.json")):
            with open(os.path.join(directory, "payloads.json"), encoding="utf-8") as f:
                fit = json.load(f)
        if fit or spec["mass"].get("tank") or spec["mass"].get("payload"):
            yield name, spec["mass"], fit


class MassAccounting(unittest.TestCase):
    def test_every_aircraft_weighs_its_ledgers(self):
        w = make_world()
        for name, mass, fit in designs():
            if not fit and not mass.get("payload"):
                continue  # (a store's own model: nothing aboard but itself)
            with self.subTest(aircraft=name):
                v = w.create_vehicle(name, "jsbsim:" + name, latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=3000.0, airspeed_ms=150.0)
                w.step()
                fixed = sum(p["mass"] for p in mass.get("payload", []))
                fuel = sum(t["capacity"] * t.get("fill", 1.0) for t in mass.get("tank", []))
                spawned = v.mass_kg
                self.assertAlmostEqual(spawned, mass["empty"] + fixed + fuel, delta=max(1.0, 1e-4 * spawned))
                self.assertAlmostEqual(v.state.fuel_kg, fuel, delta=max(1.0, 1e-4 * fuel))
                self.assertEqual(v.stores, [])  # (its stations empty: what a station carries, a loadout puts there)
                heavy = []
                for lo in fit.get("loadouts", []):
                    held = v.stores
                    try:
                        v.load_loadout(lo["name"])
                    except fsim.MissionRejected as refused:
                        # past its maximum take-off weight with its tanks as its design fills them: none of it loaded
                        self.assertTrue(str(refused).startswith("over_max_weight"), refused)
                        self.assertIn(name, ("a10c", "c17a"))
                        self.assertEqual(v.stores, held)
                        heavy.append(lo["name"])
                        continue
                    w.step(2)
                    stores = v.stores
                    self.assertTrue(all(s.status == "loaded" for s in stores))
                    # the flight model gained what its stores weigh (a step's fuel burnt aside)
                    self.assertAlmostEqual(v.mass_kg - spawned, sum(s.mass_kg for s in stores), delta=2.0, msg=lo["name"])
                for station in sorted({s.station for s in v.stores}):
                    v.load_store(station, None)
                w.step(2)
                self.assertAlmostEqual(v.mass_kg, spawned, delta=2.0)
                v.remove()
                if heavy:
                    # with a quarter of its fuel it takes them: each tank that fraction of what it holds
                    v = w.create_vehicle(name + "-light", "jsbsim:" + name, latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=3000.0,
                                         airspeed_ms=150.0, fuel_fraction=0.25)
                    w.step()
                    capacity = sum(t["capacity"] for t in mass.get("tank", []))
                    light = v.mass_kg
                    self.assertAlmostEqual(light, mass["empty"] + fixed + 0.25 * capacity, delta=max(1.0, 1e-4 * light))
                    self.assertAlmostEqual(v.state.fuel_kg, 0.25 * capacity, delta=max(1.0, 1e-4 * capacity))
                    for lo in heavy:
                        v.load_loadout(lo)
                    w.step(2)
                    self.assertAlmostEqual(v.mass_kg - light, sum(s.mass_kg for s in v.stores), delta=2.0)
                    self.assertLessEqual(v.mass_kg, mass.get("max_takeoff", mass.get("mtow")) + 1.0)
                    v.remove()

    def test_the_growler_s_pods_are_stores(self):
        # its three ALQ-99 pods were part of its airframe and a fixed load; they are stores now, on their stations
        w = make_world()
        v = w.create_vehicle("growler", "jsbsim:ea18g", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=3000.0, airspeed_ms=180.0)
        w.step()
        spawned = v.mass_kg
        v.load_loadout("jamming pods")
        v.load_loadout("external tanks")
        w.step(2)
        self.assertEqual([(s.station, s.store) for s in v.stores], [(3, "an-alq-99"), (4, "fpu-11"), (6, "an-alq-99"), (8, "fpu-11"), (9, "an-alq-99")])
        self.assertAlmostEqual(v.mass_kg - spawned, sum(s.mass_kg for s in v.stores), delta=1.0)
        self.assertAlmostEqual(sum(s.mass_kg for s in v.stores if s.store == "an-alq-99"), 3 * 435.0, places=6)

    def test_every_store_is_a_vehicle_of_its_own(self):
        # every store card names a JSBSim model that flies on its own and weighs what the card says (a tank empty: its
        # fuel is what its station holds); none is fixed load that could not leave its aircraft
        w = make_world()
        cards = sorted(glob.glob(os.path.join(ROOT, "assets", "stores", "*.json")))
        self.assertGreaterEqual(len(cards), 52)
        classes = set()
        for path in cards:
            with open(path, encoding="utf-8") as f:
                card = json.load(f)
            with self.subTest(store=card["id"]):
                self.assertTrue(card.get("model", "").startswith("jsbsim:"), "no flight model")
                b = w.create_vehicle(card["id"], card["model"], latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=5000.0, airspeed_ms=150.0)
                # (as it is made, before a step: a missile made on its own, not jettisoned, has its motor lit)
                self.assertAlmostEqual(b.mass_kg, card["parameters"]["mass_kg"]["value"], delta=0.01)
                w.step(60)
                self.assertFalse(b.state.diverged)
                if card["class"] != "aam":
                    self.assertLess(b.state.altitude_msl_m, 5000.0)  # (it falls)
                classes.add(card["class"])
                b.remove()
        self.assertTrue({"aam", "bomb", "tank", "pod", "cargo"} <= classes)

    def test_cargo_leaves_over_the_ramp(self):
        w = make_world()
        v = w.create_vehicle("herk", "jsbsim:c130j", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=3000.0, airspeed_ms=110.0, heading_deg=90.0)
        w.step()
        spawned = v.mass_kg
        v.load_loadout("pallets")
        w.step(2)
        self.assertEqual([(s.station, s.store) for s in v.stores], [(n, "463l-pallet") for n in range(1, 6)])
        self.assertAlmostEqual(v.mass_kg - spawned, 5 * 2268.0, delta=1.0)
        for station in range(1, 6):
            v.load_store(station, None)
        v.load_loadout("airdrop")
        w.step(2)
        self.assertAlmostEqual(v.mass_kg - spawned, 6 * 998.0, delta=1.0)

    def test_a_tank_s_fuel_is_weighed_once(self):
        # an F-16C with its three tanks: their fuel is the tanks' (the stores' mass) until drawn into its own tanks as they
        # have room; what it weighs falls by what its engine burns, whichever tank held it
        w = make_world()
        v = w.create_vehicle("viper", "jsbsim:f16c", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=5000.0, airspeed_ms=220.0, heading_deg=90.0)
        w.step()
        v.load_loadout("external tanks")
        w.step(2)
        v.submit_hsa(speed=220.0, speed_reference="true_airspeed")
        internal = v.state.fuel_kg
        mass0 = v.mass_kg
        external0 = sum(s.mass_kg for s in v.stores)
        burnt = 0.0
        for _ in range(600):
            w.step(int(round(1.0 / w.step_seconds)))
            burnt += v.navigation_report().consumption
        external = sum(s.mass_kg for s in v.stores)
        self.assertGreater(burnt, 100.0)
        self.assertLess(external, external0 - 0.5 * burnt)  # (drawn from the tanks first)
        self.assertAlmostEqual(v.state.fuel_kg, internal, delta=30.0)  # (its own kept full meanwhile)
        self.assertAlmostEqual(mass0 - v.mass_kg, burnt, delta=0.05 * burnt)
        self.assertAlmostEqual((internal - v.state.fuel_kg) + (external0 - external), burnt, delta=0.05 * burnt)


if __name__ == "__main__":
    unittest.main()
