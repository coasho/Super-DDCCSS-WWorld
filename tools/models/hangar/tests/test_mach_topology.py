"""Mach corrections must follow physical wing/tail topology, not list order."""
from pathlib import Path
import json
import unittest
import xml.etree.ElementTree as ET

from hangar.aero import mach
from hangar.geometry import Aircraft

ROOT = Path(__file__).resolve().parents[4]


class MachTopology(unittest.TestCase):
    def test_all24_published_mach_multipliers_preserve_lift_and_damping_sign(self):
        count=0
        for path in sorted((ROOT/'assets/stores').glob('*.json')):
            card=json.loads(path.read_text(encoding='utf-8'))
            if card.get('class')!='aam':continue
            name=card['model'].removeprefix('jsbsim:')
            xml=ET.parse(ROOT/'aircraft/stores'/name/(name+'.xml'))
            for key in ('mach-lift','mach-side'):
                text=xml.find(".//function[@name='aero/function/"+key+"']/table/tableData").text
                for line in text.splitlines():
                    if not line.strip():continue
                    mach_number,factor=map(float,line.split())
                    with self.subTest(model=name,table=key,mach=mach_number):
                        self.assertGreater(factor,0.,'Mach scaling cannot reverse the base lift or rate damping')
            count+=1
        self.assertEqual(count,24)

    def test_wingless_fins_do_not_invent_a_main_wing_wake(self):
        # These six authored configurations exposed negative lift/damping
        # multipliers. The VLM base already includes their mutual interaction.
        for name in ('micaem', 'micair', 'pl10', 'r77m', 'irist', 'r37m'):
            aircraft = Aircraft.load(str(ROOT/'aircraft'/name/(name+'.toml')))
            self.assertFalse(any(s.kind == 'wing' for s in aircraft.surfaces))
            for surface in aircraft.surfaces:
                with self.subTest(model=name, surface=surface.name):
                    self.assertEqual(mach._tail_downwash(aircraft, surface), 0.)

    def test_surfaces_ahead_of_or_abreast_of_wing_are_not_in_its_wake(self):
        def surface(name, kind, x):
            return dict(name=name, kind=kind, airfoil='plate', mirror=True,
                        sections=[dict(le=[x, 0., 0.], chord=1.),
                                  dict(le=[x, 2., 0.], chord=1.)])
        aircraft = Aircraft(dict(aircraft=dict(name='topology-fixture'), surface=[
            surface('wing', 'wing', 0.), surface('forward fin', 'fin', -2.),
            surface('abreast fin', 'fin', 0.), surface('aft tail', 'htail', 3.)]))
        for s in aircraft.surfaces[:3]:
            with self.subTest(surface=s.name):
                self.assertEqual(mach._tail_downwash(aircraft, s), 0.)
        # Preserve the existing estimate for a real downstream aircraft tail.
        self.assertGreater(mach._tail_downwash(aircraft, aircraft.surfaces[3]), 0.)


if __name__ == '__main__':
    unittest.main()
