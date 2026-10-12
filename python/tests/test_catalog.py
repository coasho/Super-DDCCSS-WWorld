"""The catalog through Python (docs/sdk/python.md, "The catalog"): what the platform carries, read without a world -
and that it is what a World then flies."""
import os
import unittest

import fsim
import fsim.catalog


class CatalogTest(unittest.TestCase):
    def test_aircraft_fits_and_loadouts(self):
        names = fsim.catalog.aircraft()
        self.assertEqual(names, sorted(names))
        for name in ("f16c", "su27s", "e3g"):
            self.assertIn(name, names)
        self.assertNotIn("aim120c", names)                      # (a store's model has no fit)
        fit = fsim.catalog.fit("f16c")
        self.assertEqual([p["name"] for p in fit["payloads"]], ["radar", "rwr", "cmds", "gun", "stations", "ews"])
        loadouts = fsim.catalog.loadouts("f16c")
        self.assertEqual(loadouts[0].name, "air superiority")
        self.assertEqual(sorted(store for _, store in loadouts[0].stores), ["aim-120c"] * 4 + ["aim-9x"] * 2)
        self.assertEqual({station for station, _ in loadouts[0].stores}, {1, 2, 3, 7, 8, 9})
        with self.assertRaises(KeyError):
            fsim.catalog.fit("no-such-type")
        for bad in ("", "..", "../f16c", "a/b", ".hidden"):
            with self.assertRaises(ValueError):
                fsim.catalog.fit(bad)

    def test_cards(self):
        card = fsim.catalog.store("aim-120c")
        self.assertEqual(card["class"], "aam")
        self.assertEqual(card["parameters"]["mass_kg"]["value"], 161.5)
        self.assertIn("aim-120c", fsim.catalog.stores())
        self.assertEqual(fsim.catalog.payload("an-apg-68v9")["designation"], "AN/APG-68(V)9")
        self.assertIn("an-alr-56m", fsim.catalog.payloads())
        with self.assertRaises(KeyError):
            fsim.catalog.store("no-such-store")
        card["class"] = "changed"                               # (the caller's own copy)
        self.assertEqual(fsim.catalog.store("aim-120c")["class"], "aam")

    def test_models_and_manifests(self):
        path = fsim.catalog.model("f16c")
        self.assertTrue(path and os.path.isfile(path) and path.endswith("f16c.glb"))
        self.assertIsNone(fsim.catalog.model("no-such-type"))
        m = fsim.catalog.manifest("f16c")
        self.assertEqual((m.forward, m.up, m.scale), ("-y", "+z", 1.0))
        self.assertEqual(sorted(m.stations), list(range(1, 10)))
        self.assertEqual(m.stations[9], (-1.758, 4.75, -0.0, 0))
        self.assertEqual(len(m.engines), 1)
        self.assertTrue(any(kind == "crew" for kind, *_ in m.components))
        self.assertTrue(fsim.catalog.manifest("aim120c").rockets)
        self.assertIsNone(fsim.catalog.manifest("no-such-type"))
        self.assertTrue(os.path.isdir(fsim.catalog.share_dir()))

    def test_it_is_what_a_world_flies(self):
        w = fsim.World("py-catalog", publish=False, workers=1, pin_workers=False, seed=3, terrain=False)
        v = w.create_vehicle("viper", "jsbsim:f16c", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=3000.0, airspeed_ms=180.0)
        fit = fsim.catalog.fit("f16c")
        self.assertEqual([p.name for p in v.payloads], [p["name"] for p in fit["payloads"]])
        self.assertEqual([p.card for p in v.payloads], [p["card"] for p in fit["payloads"]])
        loadout = fsim.catalog.loadouts("f16c")[0]
        v.load_loadout(loadout.name)
        self.assertEqual(sorted((s.station, s.store) for s in v.stores), sorted(loadout.stores))
        for s in v.stores:
            self.assertEqual(s.mnemonic, fsim.catalog.store(s.store)["designation"])


if __name__ == "__main__":
    unittest.main()
