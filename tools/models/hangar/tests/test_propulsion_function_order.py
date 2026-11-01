"""Preserve authored JSBSim pre-function dependencies during axis conversion."""
import unittest
import xml.etree.ElementTree as ET

from hangar import airframe


class PropulsionFunctionOrder(unittest.TestCase):
    def fixture(self):
        aero=ET.fromstring('<aerodynamics>'+''.join('<axis name="'+name+'"/>' for name in ('DRAG','SIDE','LIFT','ROLL','PITCH','YAW'))+'</aerodynamics>')
        old=ET.fromstring('''<aerodynamics>
          <function name="helper/provider"><value>2</value></function>
          <function name="helper/consumer"><product><property>helper/provider</property><value>3</value></product></function>
          <axis name="X"><function name="rotor/drag-x"><property>helper/consumer</property></function></axis>
          <axis name="Y"><function name="rotor/drag-y"><value>0</value></function></axis>
          <axis name="Z"><function name="rotor/drag-z"><value>0</value></function></axis>
        </aerodynamics>''')
        return aero,old

    def test_provider_precedes_consumer_and_converted_body_force(self):
        aero,old=self.fixture();airframe._propulsion_loads(aero,old)
        names=[n.get('name') for n in aero if n.tag=='function']
        self.assertLess(names.index('helper/provider'),names.index('helper/consumer'))
        self.assertLess(names.index('helper/consumer'),names.index('rotor/drag-x'))

    def test_conversion_preserves_the_authored_helper_sequence(self):
        aero,old=self.fixture();airframe._propulsion_loads(aero,old)
        self.assertEqual([n.get('name') for n in aero if n.tag=='function'],
            ['helper/provider','helper/consumer','rotor/drag-x','rotor/drag-y','rotor/drag-z'])

    def test_conversion_does_not_mutate_original_physical_functions(self):
        aero,old=self.fixture();before=ET.tostring(old)
        airframe._propulsion_loads(aero,old)
        self.assertEqual(before,ET.tostring(old))
        self.assertEqual(aero.find("function[@name='rotor/drag-x']").get('type'),'pre')


if __name__=='__main__':unittest.main()
