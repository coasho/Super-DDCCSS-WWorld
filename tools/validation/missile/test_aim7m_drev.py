"""Offline integrity checks for the public AIM-7M DREV dataset."""
import json
import math
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[3]
DATA = Path(__file__).with_name("data") / "aim7m_drev_tm1999_050.json"


class Aim7mDrevTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.data = json.loads(DATA.read_text(encoding="utf-8"))

    def test_identity_and_source(self):
        self.assertEqual(self.data["schema"], "fsim.missile-wind-tunnel.v1")
        self.assertEqual(self.data["model"], "aim7m")
        self.assertEqual(self.data["source"]["report"], "DREV-TM-1999-050")
        self.assertIn("micro_IA40243311_1824", self.data["source"]["url"])
        self.assertEqual(self.data["source"]["classification"],
                         "UNCLASSIFIED / Unlimited Distribution")

    def test_table_coverage(self):
        tables = self.data["tables"]
        self.assertEqual(len(tables), 34)
        for roll in (0.0, 45.0):
            for wing in (0.0, 10.0, 20.0):
                rows = [row for row in tables
                        if row["roll_deg"] == roll and row["wing_deflection_deg"] == wing]
                self.assertTrue(rows, (roll, wing))

    def test_known_values(self):
        def row(mach, roll=0.0, wing=0.0):
            return next(item for item in self.data["tables"]
                        if item["mach"] == mach and item["roll_deg"] == roll
                        and item["wing_deflection_deg"] == wing)

        self.assertAlmostEqual(row(2.0)["cn_alpha_per_rad"], 33.264)
        self.assertAlmostEqual(row(2.0)["cm_alpha_per_rad"], -46.155)
        self.assertAlmostEqual(row(2.0)["xcp_cal"], 1.3875)
        self.assertAlmostEqual(row(3.5)["cn_alpha_per_rad"], 21.841)
        self.assertAlmostEqual(row(3.5)["cm_alpha_per_rad"], -23.047)
        self.assertAlmostEqual(row(3.5)["xcp_cal"], 1.0552)

    def test_missing_values_are_explicit(self):
        for item in self.data["tables"]:
            values = (item["cn_alpha_per_rad"], item["cm_alpha_per_rad"], item["xcp_cal"])
            if any(value is None for value in values):
                self.assertTrue(item.get("note"), item)
            for value in values:
                if value is not None:
                    self.assertTrue(math.isfinite(value), item)

    def test_project_catalog_links_the_dataset(self):
        catalog = json.loads((ROOT / "tools/models/hangar/hangar/public_data.json")
                             .read_text(encoding="utf-8"))
        entries = catalog["models"]["aim7m"]["wind_tunnel_datasets"]
        entry = next(item for item in entries if item["source"] == "DREV-TM-1999-050")
        self.assertEqual(entry["path"], "tools/validation/missile/data/aim7m_drev_tm1999_050.json")
        self.assertTrue((ROOT / entry["validator"]).is_file())


if __name__ == "__main__":
    unittest.main()
