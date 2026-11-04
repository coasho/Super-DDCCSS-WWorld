"""Equipment identity and unchanged numerical-schema regression checks."""
import copy
import json
from pathlib import Path
import unittest

from hangar import components
from hangar import equipment_models as equipment

ROOT = Path(__file__).resolve().parents[4]


class EquipmentModelIdentity(unittest.TestCase):
    def test_all_current_models_and_component_roles_have_exact_names(self):
        rows = equipment.catalogue()
        cards = {p.stem: json.loads(p.read_text(encoding='utf-8'))
                 for p in (ROOT/'assets/stores').glob('*.json')}
        self.assertEqual(len(rows), 53)
        self.assertEqual({row['store_id'] for row in rows}, set(cards))
        self.assertEqual(equipment.definition('aim-120c')['model_id'], 'aim-120c-7')
        self.assertEqual(equipment.definition('aim-9x')['designation'], 'AIM-9X Block I')
        selections = []
        for row in rows:
            self.assertEqual(set(row['components']), set(equipment.ROLES))
            card = cards[row['store_id']]
            self.assertEqual(row['native_model'], card['model'])
            for kind, part in row['components'].items():
                self.assertEqual(part['availability'], card['component_models']['components'][kind]['availability'])
                if part['availability'] != 'available':
                    continue
                selections.append(part['selector'])
                self.assertTrue(part['name'].startswith(row['designation']))
                self.assertEqual(equipment.normalize(card, kind, part['selector']), part['mechanism_id'])
                self.assertIsNone(part['hardware_model_name'])
                self.assertTrue(part['hardware_identity'])
        self.assertEqual(len(selections), len(set(selections)))

    def test_wrong_equipment_role_model_and_invented_selection_refuse(self):
        card = json.loads((ROOT/'assets/stores/aim-120d.json').read_text(encoding='utf-8'))
        for kind, selected in [('seeker', 'equipment.aim-120c-7.seeker'),
                               ('warhead', 'equipment.aim-120d.seeker'),
                               ('seeker', 'equipment.aim-120d-3.seeker')]:
            with self.assertRaises(ValueError):
                equipment.normalize(card, kind, selected)
        for key in ('id', 'model', 'designation'):
            wrong = copy.deepcopy(card)
            wrong[key] = 'other'
            with self.assertRaises(ValueError):
                equipment.normalize(wrong, 'seeker', 'equipment.aim-120d.seeker')

    def test_named_and_legacy_selections_keep_the_same_physical_fingerprint(self):
        for model in equipment.catalogue():
            card = json.loads((ROOT/model['card_path']).read_text(encoding='utf-8'))
            for kind, part in model['components'].items():
                if part['availability'] != 'available':
                    continue
                legacy = copy.deepcopy(card)
                named = copy.deepcopy(card)
                legacy['component_models']['components'][kind]['implementation_id'] = part['mechanism_id']
                named['component_models']['components'][kind]['implementation_id'] = part['selector']
                before = components.resolve(legacy, kind)
                after = components.resolve(named, kind)
                self.assertEqual(after['implementation_id'], part['mechanism_id'])
                self.assertEqual(after['selection_id'], part['selector'])
                self.assertEqual(components.fingerprint(before), components.fingerprint(after))

    def test_legacy_and_custom_provider_ids_retain_their_meaning(self):
        self.assertEqual(equipment.normalize({}, 'seeker', 'custom.receiver'), 'custom.receiver')
        self.assertEqual(equipment.normalize({}, 'seeker', 'fsim.seeker.active_radar.v2'), 'fsim.seeker.active_radar.v2')
        owned = equipment.catalogue()
        owned[0]['model_id'] = 'modified'
        self.assertNotEqual(equipment.catalogue()[0]['model_id'], 'modified')


if __name__ == '__main__':
    unittest.main()
