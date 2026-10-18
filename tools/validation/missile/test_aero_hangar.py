"""Generated static-model comparison guards; no SDK or optional solver imports."""
import math
from pathlib import Path
import tempfile
import unittest

from aero_hangar import assess_initial


def fixture():
    return '''<fdm_config><metrics><wingarea unit="FT2">1</wingarea>
<chord unit="FT">2</chord><location name="AERORP" unit="IN"><x>12</x><y>0</y><z>0</z></location></metrics>
<aerodynamics>
<axis name="AXIAL"><function name="aero/coefficient/CA"><product>
<property>aero/qbar-psf</property><property>metrics/Sw-sqft</property><value>.3</value>
<table><independentVar lookup="row">velocities/mach</independentVar><tableData>0 1
1 2</tableData></table></product></function></axis>
<axis name="NORMAL"><function name="aero/coefficient/CN"><product>
<property>aero/qbar-psf</property><property>metrics/Sw-sqft</property><sum>
<product><sin><property>aero/alpha-rad</property></sin><cos><property>aero/alpha-rad</property></cos>
<table><independentVar lookup="row">velocities/mach</independentVar><tableData>0 2
1 4</tableData></table></product>
<product><value>5</value><sin><property>aero/alpha-rad</property></sin>
<abs><sin><property>aero/alpha-rad</property></sin></abs></product></sum></product></function></axis>
<axis name="PITCH"><function name="aero/coefficient/Cm"><product>
<property>aero/qbar-psf</property><property>metrics/Sw-sqft</property><property>metrics/cbarw-ft</property>
<product><value>-3</value><sin><property>aero/alpha-rad</property></sin></product>
</product></function></axis></aerodynamics></fdm_config>'''


class InitialHangarTests(unittest.TestCase):
    def assess(self, text=None, rows=None, **kwargs):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'model.xml'
            path.write_text(fixture() if text is None else text)
            reference = dict(area_m2=.3048**2, length_m=.6096, moment_x_m=.3048)
            reference.update(kwargs)
            return assess_initial(path, rows or [dict(mach=.6, alpha_deg=2, coefficients=dict(ca=.1, cn=.2, cm=-.3))], reference)

    def test_static_expression_uses_emitted_precision_and_exact_conditions(self):
        result = self.assess()
        row = result['cases'][0]
        angle = math.radians(2)
        expected = dict(ca=.48, cn=3.2*math.sin(angle)*math.cos(angle)+5*math.sin(angle)**2,
                        cm=-3*math.sin(angle))
        for key in expected:
            self.assertAlmostEqual(row['initial_full_static'][key], expected[key])
        self.assertAlmostEqual(row['difference']['ca']['absolute'], .38)
        self.assertFalse(result['model_accepted'])
        self.assertFalse(result['like_for_like_inviscid'])
        self.assertIn('crossflow', result['basis'])
        self.assertEqual(len(result['raw_file']['sha256']), 64)

    def test_zero_and_signed_incidence_preserve_static_symmetry(self):
        rows = [dict(mach=.6, alpha_deg=a, coefficients=dict(ca=.48, cn=0, cm=0)) for a in (0, 2, -2)]
        cases = self.assess(rows=rows)['cases']
        self.assertEqual(cases[0]['initial_full_static'], dict(ca=.48, cn=0, cm=0))
        for key in ('cn', 'cm'):
            self.assertEqual(cases[1]['initial_full_static'][key], -cases[2]['initial_full_static'][key])

    def test_no_silent_extrapolation_or_nonzero_sideslip(self):
        for field, value in (('mach', 2), ('alpha_deg', float('nan')), ('beta_deg', .1)):
            row = dict(mach=.6, alpha_deg=2, coefficients=dict(ca=.1, cn=.2, cm=-.3))
            row[field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                self.assess(rows=[row])

    def test_generated_reference_mismatch_is_rejected(self):
        for changes in (dict(area_m2=.2), dict(length_m=1), dict(moment_x_m=.4)):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.assess(**changes)

    def test_emitted_reference_rounding_is_rescaled_to_solver_references(self):
        area, length = .3048**2 + 1e-8, .6096 + 1e-7
        row = self.assess(area_m2=area, length_m=length)['cases'][0]['initial_full_static']
        self.assertAlmostEqual(row['ca'], .48*.3048**2/area, places=14)
        self.assertAlmostEqual(row['cm'], -3*math.sin(math.radians(2))*.3048**2/area*.6096/length, places=14)

    def test_rounded_origin_transport_keeps_generated_coefficients_and_signed_torque(self):
        area, length = .3048**2 + 1e-8, .6096 + 1e-7
        target_x = .3048 + 1e-5
        rows = [dict(mach=.6, alpha_deg=a, coefficients=dict(ca=0, cn=0, cm=0)) for a in (0, 2, -2)]
        for z_in in (-.0004, .0004):
            text = fixture().replace('<z>0</z>', '<z>%.4f</z>' % z_in)
            result = self.assess(text, rows, area_m2=area, length_m=length, moment_x_m=target_x)
            generated_xyz = [12*.0254, 0, z_in*.0254]
            self.assertEqual(result['generated_moment_reference_xyz_m'], generated_xyz)
            self.assertEqual(result['requested_moment_reference_xyz_m'], [target_x, 0, 0])
            delta = [target_x-generated_xyz[0], 0, -generated_xyz[2]]
            self.assertEqual(result['moment_transport']['delta_requested_minus_generated_m'], delta)
            self.assertEqual(result['moment_transport']['reference_length_m'], length)
            for case in result['cases']:
                angle = math.radians(case['alpha_deg'])
                expected = dict(ca=.48*.3048**2/area,
                    cn=(3.2*math.sin(angle)*math.cos(angle)+5*math.sin(angle)*abs(math.sin(angle)))*.3048**2/area,
                    cm=-3*math.sin(angle)*.3048**2/area*.6096/length)
                for key in expected:
                    self.assertAlmostEqual(case['initial_at_generated_origin'][key], expected[key], places=14)
                correction = (delta[0]*expected['cn']-delta[2]*expected['ca'])/length
                self.assertAlmostEqual(case['moment_transport_cm'], correction, places=14)
                self.assertAlmostEqual(case['initial_full_static']['cm'], expected['cm']+correction, places=14)
                self.assertEqual(case['initial_full_static']['ca'], case['initial_at_generated_origin']['ca'])
                self.assertEqual(case['initial_full_static']['cn'], case['initial_at_generated_origin']['cn'])

    def test_difference_arithmetic_rejects_finite_input_overflow(self):
        for factor, selected in (('1e308', -1e308), ('1e-300', 1e308)):
            text = fixture().replace('<value>.3</value>', '<value>%s</value>' % factor)
            row = dict(mach=.6, alpha_deg=2, coefficients=dict(ca=selected, cn=.2, cm=-.3))
            with self.subTest(factor=factor), self.assertRaises(ValueError):
                self.assess(text, [row])

    def test_moment_transport_rejects_finite_input_overflow(self):
        text = fixture().replace('<chord unit="FT">2</chord>', '<chord unit="FT">1e-300</chord>')
        text = text.replace('0 2\n1 4', '0 1e307\n1 1e307')
        with self.assertRaises(ValueError):
            self.assess(text, length_m=1e-300*.3048, moment_x_m=.3048+1e-5)

    def test_only_utf8_xml_is_supported_and_encoded_declarations_are_rejected(self):
        reference = dict(area_m2=.3048**2, length_m=.6096, moment_x_m=.3048)
        rows = [dict(mach=.6, alpha_deg=2, coefficients=dict(ca=.1, cn=.2, cm=-.3))]
        declaration = '<!DOCTYPE fdm_config [<!ENTITY coefficient ".3">]>\n'
        entity_xml = declaration + fixture().replace('<value>.3</value>', '<value>&coefficient;</value>')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'model.xml'
            for encoding in ('utf-16', 'utf-16-le', 'utf-16-be', 'utf-32', 'utf-32-le', 'utf-32-be'):
                path.write_bytes(entity_xml.encode(encoding))
                with self.subTest(encoding=encoding), self.assertRaises(ValueError):
                    assess_initial(path, rows, reference)
            path.write_bytes(fixture().encode('utf-8-sig'))
            self.assertAlmostEqual(assess_initial(path, rows, reference)['cases'][0]['initial_full_static']['ca'], .48)

    def test_missing_conditions_and_force_normalizers_are_rejected(self):
        for missing in ('mach', 'alpha_deg'):
            row = dict(mach=.6, alpha_deg=2, coefficients=dict(ca=.1, cn=.2, cm=-.3))
            del row[missing]
            with self.subTest(missing=missing), self.assertRaises(ValueError):
                self.assess(rows=[row])
        with self.assertRaises(ValueError):
            self.assess(fixture().replace('<property>metrics/Sw-sqft</property>', '<value>2</value>'))

    def test_unknown_expression_property_or_duplicate_function_is_rejected(self):
        text = fixture()
        for changed in (text.replace('<abs>', '<sqrt>').replace('</abs>', '</sqrt>'),
                        text.replace('aero/alpha-rad', 'velocities/q-rad_sec'),
                        text.replace('<aerodynamics>', '<aerodynamics><axis name="AXIAL"><function name="aero/coefficient/CA"><value>1</value></function></axis>'),
                        text.replace('name="PITCH"', 'name="YAW"')):
            with self.subTest(changed=changed[-100:]), self.assertRaises(ValueError):
                self.assess(changed)

    def test_malformed_finite_table_and_numeric_outputs_are_rejected(self):
        for changed in (fixture().replace('1 2</tableData>', '0 2</tableData>'),
                        fixture().replace('1 2</tableData>', '1 nan</tableData>'),
                        fixture().replace('<value>.3</value>', '<value>1e309</value>'),
                        fixture().replace('<chord unit="FT">', '<chord unit="M">')):
            with self.subTest(), self.assertRaises(ValueError):
                self.assess(changed)

    def test_missing_table_coordinate_or_selected_coefficients_fail_explicitly(self):
        with self.assertRaises(ValueError):
            self.assess(fixture().replace('>velocities/mach</independentVar>', '></independentVar>'))
        for coefficients in (None, {}, {'ca': .1, 'cn': .2}):
            with self.subTest(coefficients=coefficients), self.assertRaises(ValueError):
                self.assess(rows=[dict(mach=.6, alpha_deg=2, coefficients=coefficients)])


if __name__ == '__main__':
    unittest.main()
