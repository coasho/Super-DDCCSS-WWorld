"""Every design's model gives the viewer what its effects are drawn from (hangar/effects.py): an exhaust for each
engine that burns, a jet's kind and ratings, a rocket's line, a propeller that turns, its lifting surfaces - nothing a
design has is left for the viewer to guess."""
import unittest

from hangar import effects


class EffectsTest(unittest.TestCase):
    def test_nothing_missing(self):
        rows = effects.audit()
        self.assertGreater(len(rows), 80)
        for r in rows:
            with self.subTest(r["name"]):
                self.assertEqual(r["missing"], [], "%s: python -m hangar %s model" % (r["name"], r["name"]))

    def test_jets_carry_their_ratings(self):
        # each `engine` line has its engine's thrust and fuel, the design's: what a reheat's flame is drawn from
        import os
        from hangar import pipeline
        root = os.path.join(pipeline.repo_root(), "aircraft")
        seen = 0
        for r in effects.audit():
            if not r["manifest"].get("engine"):
                continue
            with open(os.path.join(root, r["name"], r["name"] + ".glb.manifest"), encoding="utf-8") as f:
                for line in f:
                    if line.startswith("engine "):
                        fields = line.split()
                        seen += 1
                        self.assertEqual(len(fields), 8, "%s: %s" % (r["name"], line.strip()))
                        self.assertGreater(float(fields[4]), 0.0)                      # (its dry thrust)
                        self.assertEqual(float(fields[5]) > 0.0, fields[3] == "1")     # (reheat thrust where it has one)
        self.assertGreater(seen, 30)


if __name__ == "__main__":
    unittest.main()
