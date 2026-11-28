"""Retained V2 generation must preserve the original compatibility behavior."""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location('weapon_v2_assets', Path(__file__).with_name('weapon_v2_assets.py'))
GENERATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GENERATOR)

# Canonical original fixture bytes, excluding descriptive source/calibration
# metadata. New production behavior requires a separate compatibility decision;
# regenerating a fixture must not silently change these retained inputs.
ORIGINAL_BEHAVIOR = {
    'aim-9m': 'dfe065d11379a0dbfd0bce9b22b9b5a3f9a2c90bc3394549b02fd9f228973cdb',
    'aim-9x': 'bfcb826e32cdc317361c3e83cb916daee4939180251d34912d14877eefce4c80',
    'aim-7m': '887e30249052c383a322830201f1c0346fbb40db331b035f97c83a4c750d9f7d',
    'aim-120c': '2b67c30db176d4bb438f592d7b4a44b4fd89d2d79c1285cdd2b9717751af5399',
}


def behavior_hash(card):
    retained = {key: value for key, value in card.items() if key not in ('sources', 'simulation_model')}
    return hashlib.sha256(json.dumps(retained, sort_keys=True, separators=(',', ':'),
                                    ensure_ascii=False, allow_nan=False).encode('utf-8')).hexdigest()


class RetainedV2Assets(unittest.TestCase):
    def test_current_authored_cards_reproduce_original_v2_behavior(self):
        source = ROOT/'assets'
        inputs = {name: (source/'stores'/(name+'.json')).read_bytes() for name in ORIGINAL_BEHAVIOR}
        with tempfile.TemporaryDirectory(prefix='m03-v2-fixture-test-', dir='D:/ENV/tmp') as temporary:
            destination = Path(temporary)/'weapon-v2-fixture/share/flightsim'
            GENERATOR.stage(source, destination)
            for name, expected in ORIGINAL_BEHAVIOR.items():
                with self.subTest(model=name):
                    card = json.loads((destination/'stores'/(name+'.json')).read_text(encoding='utf-8'))
                    components = card['component_models']['components']
                    self.assertTrue(components['seeker']['implementation_id'].endswith('.v2'))
                    self.assertNotIn('reticle_pattern', components['seeker']['bindings'])
                    self.assertNotIn('reticle_design', components['seeker']['bindings'])
                    self.assertNotIn('target_detection_v1', components['fuze']['bindings'])
                    self.assertEqual(behavior_hash(card), expected)
        for name, original in inputs.items():
            self.assertEqual((source/'stores'/(name+'.json')).read_bytes(), original)


if __name__ == '__main__':
    unittest.main(verbosity=2)
