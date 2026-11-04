"""Independent native-axis work oracle for emitted unpowered detached pieces."""
import json
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
from hangar import pieces


def evaluate(node, properties):
    if node.tag == 'property':
        return properties[node.text.strip()]
    if node.tag == 'value':
        return float(node.text)
    if node.tag == 'function':
        expressions = [child for child in node if child.tag != 'description']
        if len(expressions) != 1:
            raise ValueError('Generated native function needs exactly one expression')
        return evaluate(expressions[0], properties)
    if node.tag in ('cos', 'sin'):
        value = evaluate(list(node)[0], properties)
        return math.cos(value) if node.tag == 'cos' else math.sin(value)
    if node.tag == 'product':
        return math.prod(evaluate(child, properties) for child in node)
    if node.tag == 'table':
        x = properties[node.find('independentVar').text.strip()]
        rows = [tuple(map(float, line.split())) for line in node.find('tableData').text.splitlines() if line.strip()]
        if x <= rows[0][0]:
            return rows[0][1]
        for (a, u), (b, v) in zip(rows, rows[1:]):
            if x <= b:
                return u + (v-u)*(x-a)/(b-a)
        return rows[-1][1]
    raise ValueError('Unexpected generated force operator: ' + node.tag)


class PiecePassivity(unittest.TestCase):
    def test_every_emitted_piece_force_opposes_air_velocity_over_full_angle_domain(self):
        root = Path(__file__).resolve().parents[4]
        data = json.loads((root/'aircraft/f16c/f16c.damage.json').read_text())
        seen = 0
        for section in data['sections']:
            if section['name'] not in pieces.PARTS:
                continue
            xml = ET.fromstring(pieces.piece_xml(data, section['name']))
            worst = (-math.inf, 0, 0)
            for alpha in range(-180, 181):
                for beta in range(-80, 81, 20):
                    a = math.radians(alpha)
                    b = math.radians(beta)
                    velocity = (math.cos(a)*math.cos(b), math.sin(b), math.sin(a)*math.cos(b))
                    values = {'aero/qbar-psf': 1., 'metrics/Sw-sqft': 1., 'aero/alpha-deg': alpha,
                              'aero/alpha-rad': a, 'aero/beta-rad': b}
                    forces = {name: sum(evaluate(fn, values) for fn in xml.findall("aerodynamics/axis[@name='"+name+"']/function"))
                              for name in ('AXIAL', 'SIDE', 'NORMAL')}
                    # JSBSim body axial/normal signs: Fx=-CA,Fy=CY,Fz=-CN.
                    power = -forces['AXIAL']*velocity[0]+forces['SIDE']*velocity[1]-forces['NORMAL']*velocity[2]
                    worst = max(worst, (power, alpha, beta))
            self.assertLessEqual(worst[0], 1e-12, (section['name'], worst))
            seen += 1
        self.assertEqual(seen, 4)


if __name__ == '__main__':
    unittest.main(verbosity=2)
