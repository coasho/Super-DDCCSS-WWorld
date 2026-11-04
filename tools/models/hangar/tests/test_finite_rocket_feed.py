"""Normal-generator finite feed policy; no publication or numerical retuning."""
import copy
import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from hangar import pipeline,storebodies


class FiniteRocketFeedGenerationTests(unittest.TestCase):
    def test_every_actual_explicit_aam_motor_opts_in_once(self):
        root=Path(pipeline.repo_root())
        catalogue=json.loads((root/'tools/models/hangar/hangar/data/equipment_models.json').read_text(encoding='utf-8'))
        cards=[]
        for entry in catalogue['models']:
            card=json.loads((root/entry['card_path']).read_text(encoding='utf-8'))
            if card.get('class')=='aam':cards.append(card)
        self.assertEqual(len(cards),24)
        self.assertEqual(len({c['id'] for c in cards}),24)
        for card in cards:
            with self.subTest(card=card['id']):
                name=card['model'].removeprefix('jsbsim:')
                products=storebodies.files(card)
                xml=ET.fromstring(products[name+'.xml'])
                policy=xml.findall('./flight_control/property[.="fsim/rocket-feed-v1/engine[0]/version"]')
                self.assertEqual(len(policy),1)
                self.assertEqual(float(policy[0].get('value')),1.)
                self.assertFalse(xml.findall('./flight_control/property[.="fsim/rocket-feed-v1/engine[1]/version"]'))
                engine=ET.fromstring(products[f'Engines/{name}_motor.xml'])
                self.assertIsNotNone(engine.find('thrust_table'))
                # Native force, Isp, phase table and nozzle remain authored.
                self.assertGreater(float(engine.findtext('isp')),0)
                self.assertTrue(engine.findtext('./thrust_table/tableData').strip())

    def test_absent_explicit_component_profile_keeps_legacy_no_policy(self):
        root=Path(pipeline.repo_root())
        card=json.loads((root/'assets/stores/aim-120c.json').read_text(encoding='utf-8'))
        card=copy.deepcopy(card);card.pop('component_models')
        xml=ET.fromstring(storebodies.files(card)['aim120c.xml'])
        self.assertFalse(any((node.text or '').startswith('fsim/rocket-feed-v1/') for node in xml.findall('./flight_control/property')))


if __name__=='__main__':unittest.main(verbosity=2)
