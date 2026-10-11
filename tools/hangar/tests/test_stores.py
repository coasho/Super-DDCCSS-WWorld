"""Every store is a design the pipeline builds (hangar/storestages.py, docs/hangar.md, Stores): each card names one, a
design is what its writer writes, the card's figures are the design's, and its flight model and its model for the
viewer are what the stages make. (What the platform does with it - carried, released, followed to the ground - is the
`fly` stage's: `python -m hangar stores`.)"""
import json
import os
import struct
import tempfile
import unittest

from hangar import pipeline, storedesigns, storemodels, stores, storestages

ROOT = pipeline.repo_root()


def cards():
    out = {}
    d = os.path.join(ROOT, "assets", "stores")
    for f in sorted(os.listdir(d)):
        if f.endswith(".json"):
            with open(os.path.join(d, f), encoding="utf-8") as h:
                c = json.load(h)
            out[c["id"]] = c
    return out


def design(name):
    d = pipeline.Design(os.path.join(ROOT, "aircraft", name, name + ".toml"), log=lambda *a, **k: None)
    d.save = lambda stage, data: data  # (the checks, not the cache: nothing is written)
    return d


def glb_triangles(path):
    with open(path, "rb") as f:
        data = f.read()
    magic, version, length = struct.unpack("<III", data[:12])
    assert magic == 0x46546C67 and version == 2 and length == len(data), path
    n = struct.unpack_from("<I", data, 12)[0]
    j = json.loads(data[20:20 + n])
    return sum(j["accessors"][p["indices"]]["count"] // 3 for m in j["meshes"] for p in m["primitives"])


class StoresTest(unittest.TestCase):
    def test_every_card_has_its_design(self):
        designs = storestages.designs()
        named = {}
        for name in designs:
            named[storestages.card_of(design(name))["id"]] = name
        for cid, card in cards().items():
            with self.subTest(cid):
                self.assertIn(cid, named, "no design in aircraft/ names the card %s ([aircraft] card)" % cid)
                # the name the platform makes it by, the viewer draws it by and its design is filed under: one
                self.assertEqual(card["model"], "jsbsim:" + named[cid])
        self.assertEqual(len(named), len(designs), "two designs name one card")
        self.assertEqual(sorted(named), sorted(cards()), "a design names a card that is not in assets/stores")

    def test_written_as_committed(self):
        # a tank's, a pod's, a bomb's and a cargo store's design is its class's rule: the committed file is what it writes
        for name in storedesigns.cards():
            toml, paint = storedesigns.design(name)
            for fn, text in ((name + ".toml", toml), ("paint.toml", paint)):
                path = os.path.join(ROOT, "aircraft", name, fn)
                with open(path, encoding="utf-8") as f:
                    self.assertEqual(f.read(), text, "%s is not what `python -m hangar storedesigns %s` writes" % (path, name))

    def test_card_figures_are_the_designs(self):
        # its size, its mass and what its aircraft feels while it carries it (its drag area, its Munk volume, its fins'
        # side force): the card's are the design's - the stages' own checks, none failed
        for name in storestages.designs():
            d = design(name)
            card = storestages.card_of(d)
            par = {k: v["value"] for k, v in card["parameters"].items() if isinstance(v, dict) and "value" in v}
            with self.subTest(name):
                a = d.aircraft
                length = max(float(b.x[-1]) for b in a.bodies) - min(float(b.x[0]) for b in a.bodies)
                self.assertAlmostEqual(length, par["length_m"], delta=0.005 * par["length_m"])
                across = max(float(w) for b in a.bodies if b.kind == "fuselage" for w in b.w)
                self.assertAlmostEqual(across, par.get("width_m" if "width_m" in par else "diameter_m"), delta=0.005 * across)
                for stage in (storestages.mass, storestages.aero):
                    failed = [c["name"] for c in stage(d)["checks"] if c["status"] == "fail"]
                    self.assertEqual(failed, [], "%s: %s" % (name, stage.__name__))

    def test_flight_models(self):
        # every store's flight model is what its design's build stage writes - a tank's, a pod's and a cargo store's
        # from its card, a missile's and a bomb's from its design, a missile's motor and nozzle with it - and nothing
        # else is in its folder
        for name in storestages.designs():
            d = design(name)
            card = storestages.card_of(d)
            path, written = storestages.flight_model(card)
            self.assertTrue(written)
            out = storemodels.files(card)
            with self.subTest(name):
                self.assertIsNotNone(out, "%s: nothing to write its flight model from" % name)
                folder = os.path.dirname(path)
                for rel, text in out.items():
                    with open(os.path.join(folder, *rel.split("/")), encoding="utf-8", newline="") as f:
                        self.assertEqual(f.read(), text, "%s/%s is not what `python -m hangar %s build` writes" % (folder, rel, name))
                kept = {os.path.relpath(os.path.join(r, f), folder).replace(os.sep, "/") for r, _, fs in os.walk(folder) for f in fs}
                self.assertEqual(kept, set(out))
                empty, held, _, _ = storestages._xml_mass(out[name + ".xml"])
                self.assertAlmostEqual(empty + held, card["parameters"]["mass_kg"]["value"], delta=0.02)
                self.assertEqual(held > 0.0, "motor" in d.aircraft.spec)  # (a missile's propellant; nothing else burns)

    def test_a_store_before_a_stock_aircraft(self):
        # the Mk 82 a card names is hangar's, built from its design: the platform finds it before JSBSim's own mk82
        self.assertTrue(os.path.isfile(os.path.join(ROOT, "aircraft", "stores", "mk82", "mk82.xml")))
        with open(os.path.join(ROOT, "aircraft", "stores", "mk82", "mk82.xml"), encoding="utf-8") as f:
            self.assertIn("Written by hangar", f.read(300))

    def test_models(self):
        superseded = os.path.join(ROOT, "assets", "models")
        for name in storestages.designs():
            d = design(name)
            card = storestages.card_of(d)
            glb = os.path.join(d.dir, name + ".glb")
            with self.subTest(name):
                self.assertTrue(os.path.isfile(glb) and os.path.isfile(glb + ".manifest"), "%s: python -m hangar %s model" % (glb, name))
                triangles = glb_triangles(glb)
                most = storedesigns.TRIANGLES.get(card["class"])
                if most:
                    # drawn on every station that carries one: no aircraft's budget
                    self.assertLessEqual(triangles, most)
                    self.assertEqual(d.aircraft.triangles, most)
                # its picture is its flight model's: its origin its CG, its length and its width its card's - on the
                # file's own vertices (the viewer hangs its origin where the simulation has the store)
                failed = [c["name"] + ": " + c["note"] for c in storestages.picture_checks(d, card, glb) if c["status"] == "fail"]
                self.assertEqual(failed, [], name)
                # the viewer looks in assets/models before a design's folder: nothing there may stand in for its model
                self.assertFalse(os.path.exists(os.path.join(superseded, name + ".glb")), "assets/models/%s.glb shadows its design's" % name)

    def test_cargo_drawn_as_committed(self):
        # a cargo store's picture is stores.py's box, the same bytes each time it is written
        for name, card in stores.cargo_cards().items():
            with tempfile.TemporaryDirectory() as tmp:
                with open(stores.write_cargo(name, card, os.path.join(tmp, name + ".glb")), "rb") as f:
                    data = f.read()
            committed = os.path.join(ROOT, "aircraft", name, name + ".glb")
            with open(committed, "rb") as f:
                self.assertEqual(f.read(), data, "%s is not what `python -m hangar %s model` writes" % (committed, name))


if __name__ == "__main__":
    unittest.main()
