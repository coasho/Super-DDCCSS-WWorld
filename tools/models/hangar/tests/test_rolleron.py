"""Passive patch admission, physical geometry and exclusion from active authority."""
import copy
import tomllib
import unittest
import numpy as np
import xml.etree.ElementTree as ET
from pathlib import Path
from hangar import pipeline
from hangar import missiles,rolleron
from hangar.geometry.aircraft import Aircraft

class Rollerons(unittest.TestCase):
    def test_actual_authored_design_and_preset_select_same_geometry_derived_hinge(self):
        actual=Aircraft.load(str(Path(pipeline.repo_root())/'aircraft/aim9m/aim9m.toml'))
        preset=Aircraft(tomllib.loads(missiles.design('aim9m')[0]))
        a,b=rolleron.geometry(actual),rolleron.geometry(preset)
        self.assertEqual(set(a),set(b));self.assertEqual(len(a),4)
        for key,row in a.items():
            self.assertEqual(row['hinge-aerodynamic-method'],1)
            self.assertAlmostEqual(row['hinge-section-stiffness-m3'],.00048861811637,delta=1e-10)
            self.assertGreater(row['hinge-aerodynamic-stiffness-m3'],1e-4)
            self.assertLess(row['hinge-aerodynamic-stiffness-m3'],2e-4)
            self.assertEqual(row['hinge-aerodynamic-stiffness-m3'],b[key]['hinge-aerodynamic-stiffness-m3'])
        changed=copy.deepcopy(actual.spec)
        for surface in changed['surface']:
            for section in surface['sections']:
                section['le']=[2*v for v in section['le']];section['chord']*=2
        twice=rolleron.geometry(Aircraft(changed))
        for key in a:self.assertAlmostEqual(twice[key]['hinge-aerodynamic-stiffness-m3']/a[key]['hinge-aerodynamic-stiffness-m3'],8.,places=12)

    def test_actual_aim9m_has_four_finite_hinged_tip_patches(self):
        source,_=missiles.design('aim9m')
        aircraft=Aircraft(tomllib.loads(source))
        selected=rolleron.geometry(aircraft)
        self.assertEqual(len(selected),4)
        self.assertEqual(len(set(selected)),4)
        for row in selected.values():
            self.assertGreater(row['wheel-inertia-kg-m2'],0)
            self.assertGreater(row['hinge-inertia-kg-m2'],0)
            self.assertAlmostEqual(np.linalg.norm(row['hinge-axis-body']),1)
            self.assertGreater(row['wheel-drag-coefficient'],row['wheel-drive-coefficient']*row['free-spin-ratio']/4)
        self.assertFalse(rolleron.geometry(Aircraft(tomllib.loads(missiles.design('aim9x')[0]))))

    def test_passive_bindings_are_not_electrical_command_actuators(self):
        aircraft=Aircraft(tomllib.loads(missiles.design('aim9m')[0]))
        geometry=rolleron.geometry(aircraft)
        physical=[dict(id=key,gains={'aileron':1},lo=-.2,hi=.2) for key in geometry]
        tables={'native_controls':physical}
        retained=copy.deepcopy(tables)
        prepared=rolleron.without_authority(aircraft,tables)
        self.assertEqual(tables,retained)
        self.assertTrue(all(not any(p['gains'].values()) for p in prepared['native_controls']))
        fcs=ET.Element('flight_control');channel=ET.SubElement(fcs,'channel')
        rows=rolleron.install(fcs,channel,aircraft,physical)
        self.assertEqual(len(rows),4)
        self.assertEqual(len(channel.findall('fcs_function')),4)
        self.assertFalse(channel.findall('actuator'))
        for node in channel.findall('fcs_function'):
            self.assertTrue(node.findtext('function/property').startswith('fsim/passive-rolleron-v1/fin-'))
            self.assertTrue(node.findtext('output').endswith('-delta-rad'))

if __name__=='__main__':unittest.main()
