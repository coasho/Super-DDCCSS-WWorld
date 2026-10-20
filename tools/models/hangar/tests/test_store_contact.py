"""Authored point-contact metadata; no fuze geometry is inferred from length."""
import copy
import json
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from hangar import pipeline, storebodies


class StoreContactTest(unittest.TestCase):
    def test_controlled_missile_retains_authored_nose_tip(self):
        root = Path(pipeline.repo_root())
        card = json.loads((root / "assets/stores/aim-120c.json").read_text(encoding="utf-8"))
        xml = ET.fromstring(storebodies.files(card)["aim120c.xml"])
        properties = {p.text.strip(): float(p.get("value")) for p in xml.findall("./flight_control/property")}
        self.assertEqual(properties.get("fsim/model/nose-version"), 1)
        self.assertEqual(properties["fsim/model/nose-x-in"], 0)
        self.assertEqual(properties["fsim/model/nose-y-in"], 0)
        self.assertEqual(properties["fsim/model/nose-z-in"], 0)
        self.assertGreater(properties["fsim/missile-control/version"], 0)

    def test_point_tip_uses_body_origin_and_station_offsets(self):
        design = {"body": [{"kind": "fuselage", "origin": [2, 3, 4], "stations": [
            {"x": .25, "y": -.5, "w": 0, "top": .75, "bottom": .75},
            {"x": 2, "w": .5, "top": 1, "bottom": 0}]}]}
        self.assertEqual(storebodies.point_contact_nose(design), (2.25, 2.5, 4.75))
        for mutation in ("width", "height", "ambiguous", "mirror", "nonfinite"):
            with self.subTest(mutation=mutation):
                invalid = copy.deepcopy(design)
                body = invalid["body"][0]
                if mutation == "width": body["stations"][0]["w"] = .01
                if mutation == "height": body["stations"][0]["top"] += .01
                if mutation == "ambiguous": invalid["body"].append(copy.deepcopy(body))
                if mutation == "mirror": body["mirror"] = True
                if mutation == "nonfinite": body["origin"][0] = float("inf")
                self.assertIsNone(storebodies.point_contact_nose(invalid))

    def test_infrared_seeker_dome_is_the_authored_foremost_tip(self):
        root = Path(pipeline.repo_root())
        card = json.loads((root / "assets/stores/aim-9x.json").read_text(encoding="utf-8"))
        xml = ET.fromstring(storebodies.files(card)["aim9x.xml"])
        properties = {p.text.strip(): float(p.get("value")) for p in xml.findall("./flight_control/property")}
        self.assertEqual(properties.get("fsim/model/nose-version"), 1)
        self.assertEqual(tuple(properties["fsim/model/nose-%s-in" % axis] for axis in "xyz"), (0, 0, 0))


if __name__ == "__main__":
    unittest.main()
