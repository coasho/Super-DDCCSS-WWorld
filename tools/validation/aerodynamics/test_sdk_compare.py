"""Safeguards against invalid or misleading SDK comparison measurements."""
import unittest
import xml.etree.ElementTree as ET

from sdk_compare import marker, reference_coefficients, reference_error, relative_change


class SdkComparisonSafeguards(unittest.TestCase):
    def test_difference_is_not_named_or_reported_as_accuracy(self):
        self.assertEqual(relative_change([3.,4.],[6.,8.]),100.)
        self.assertEqual(relative_change([3.,4.],[3.,4.]),0.)
        self.assertIsNone(relative_change([0.,0.],[1.,2.]))

    def test_incomplete_and_nonfinite_pairs_are_rejected(self):
        for old,new in (([],[]),([1.],[1.,2.]),([1.],[float('nan')]),([float('inf')],[1.])):
            with self.subTest(old=old,new=new),self.assertRaises(ValueError):
                relative_change(old,new)

    def test_neutral_protocol_keeps_custom_fin_output_nodes_available(self):
        source = '''<fdm_config><flight_control name="control">
          <property value="12">source/limit-deg</property>
          <channel><pure_gain><input>fcs/cmd</input><gain>3</gain>
            <output>fcs/custom-fin-rad</output></pure_gain></channel>
        </flight_control><aerodynamics><axis name="PITCH">
          <function name="aero/custom"><property>fcs/custom-fin-rad</property></function>
        </axis></aerodynamics></fdm_config>'''
        measured,name=marker(source,'a'*64)
        root=ET.fromstring(measured)
        control=root.find('flight_control')
        self.assertIsNone(control.find('channel'))
        values={node.text:node.get('value') for node in control.findall('property')}
        self.assertEqual(values['fcs/custom-fin-rad'],'0')
        self.assertEqual(values['source/limit-deg'],'12')
        self.assertEqual(values[name],'1')
        self.assertIsNotNone(root.find("aerodynamics/axis/function[@name='aero/custom']"))

    def test_reference_moments_do_not_change_when_the_cg_moves(self):
        base=dict(force_body_N=[6.,10.,12.],moment_body_Nm=[2.,4.,6.],
                  cg_m=[0.,0.,0.],aerodynamic_reference_m=[0.,0.,0.],qbar_psf=100.)
        shifted=dict(base,cg_m=[1.,2.,3.],moment_body_Nm=[-52.,10.,28.])
        for a,b in zip(reference_coefficients(base),reference_coefficients(shifted)):
            self.assertAlmostEqual(a,b)

    def test_reference_errors_preserve_worsening_and_zero_reference(self):
        self.assertEqual(reference_error([2.,2.],[1.,1.])['rmse'],1.)
        self.assertEqual(reference_error([3.,3.],[1.,1.])['rmse'],2.)
        self.assertIsNone(reference_error([1.],[0.])['normalized_rmse_percent'])
        with self.assertRaises(ValueError):
            reference_error([1.],[float('nan')])


if __name__ == '__main__':
    unittest.main()
