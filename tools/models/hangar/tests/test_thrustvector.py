"""Equivalent-vector signs, native finite actuation and strict opt-in."""
import copy
import json
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from hangar import components,storebodies,thrustvector

ROOT=Path(__file__).resolve().parents[4]


def fixture(store):
    card=json.loads((ROOT/'assets/stores'/(store+'.json')).read_text(encoding='utf-8'))
    control=card['component_models']['components']['autopilot']
    control['bindings']['thrust_vector']=thrustvector.BINDING
    for key,value,unit in (('tvc_limit_rad',math.radians(10),'rad'),('tvc_rate_rad_s',4.,'rad/s')):
        control['parameter_refs'][key]=key
        card['parameters'][key]=dict(value=value,unit=unit,basis='assumed',uncertainty=abs(value)*.5,
            note='Synthetic equivalent-vector fixture; not hardware calibration.')
    return card


class ThrustVector(unittest.TestCase):
    def test_all_four_supported_airframes_emit_finite_native_nozzle_actuators(self):
        for store in ('aim-9x','iris-t','mica-em','mica-ir'):
            with self.subTest(store=store):
                card=fixture(store);retained=copy.deepcopy(card);name=card['model'].removeprefix('jsbsim:')
                model=ET.fromstring(storebodies.files(card)[name+'.xml'])
                self.assertEqual(card,retained)
                metadata={n.text:float(n.get('value')) for n in model.findall('flight_control/property')}
                self.assertEqual(metadata['fsim/thrust-vector-control/version'],1)
                for axis,coefficient in (('pitch','cm-elevator'),('yaw','cn-rudder')):
                    gain=metadata['fsim/thrust-vector-control/'+axis+'_gain']
                    slope=float(model.find(".//function[@name='fsim/control/"+coefficient+"']/table/tableData").text.split()[1])
                    # At an aft axial nozzle positive pitch/yaw produces a
                    # negative moment; vector and authored fins must cooperate.
                    self.assertGreater((-gain)*slope,0)
                    actuator=next(a for a in model.findall('.//actuator')
                        if a.findtext('output')=='propulsion/engine/'+axis+'-angle-rad')
                    self.assertEqual(metadata[actuator.findtext('rate_limit')],4.)
                    self.assertIsNone(actuator.find('lag'),'the finite virtual-channel lag applies once')
                    self.assertAlmostEqual(float(actuator.findtext('clipto/max')),math.radians(10),places=15)
                    self.assertAlmostEqual(float(actuator.findtext('clipto/min')),-math.radians(10),places=15)

    def test_missing_angle_wrong_units_or_unselected_binding_refuses(self):
        for defect in ('missing','unit','zero','binding'):
            card=fixture('aim-9x');row=card['component_models']['components']['autopilot']
            if defect=='missing':del row['parameter_refs']['tvc_limit_rad']
            if defect=='unit':card['parameters']['tvc_limit_rad']['unit']='deg'
            if defect=='zero':card['parameters']['tvc_rate_rad_s']['value']=0
            if defect=='binding':row['bindings']['thrust_vector']='unsupported'
            with self.subTest(defect=defect),self.assertRaises(ValueError):components.control(card)


if __name__=='__main__':unittest.main()
