"""Potential recovery must preserve the finite-body momentum integral."""
import unittest
import numpy as np
from hangar.aero.bodies import BodyAero
from hangar.carriage import _attached_recovery_design_model


class ControlDesignIdealizations(unittest.TestCase):
    def body(self, n, blunt=False):
        x=np.linspace(0.,2.,n)
        area=x*(2.-x)+(0.2*x if blunt else 0.)
        body=BodyAero.__new__(BodyAero)
        body.inlets=[]
        body.parts=[dict(x=x,pts=np.column_stack([x,np.zeros(n),np.zeros(n)]),
            inside=np.zeros(n,dtype=bool),s_v=area,s_h=area,
            potential=np.zeros(n),w=np.ones(n),h=np.ones(n),
            eta=0.,cdc=np.zeros(n),friction=0.)]
        return body,x,area

    def test_closed_body_munk_integral_without_mutating_nominal(self):
        for n in (65,257):
            body,x,area=self.body(n)
            alternative=_attached_recovery_design_model(body)
            force,moment=alternative.forces(np.array([2.,.01,0.]),np.zeros(3),np.zeros(3))
            expected=-2.*.01*np.trapezoid(area,x)
            self.assertAlmostEqual(moment[2],expected,places=12)
            self.assertAlmostEqual(force[1],0.,places=12)
            np.testing.assert_array_equal(body.parts[0]['potential'],0.)
            original=body.forces(np.array([2.,.01,0.]),np.zeros(3),np.zeros(3))
            np.testing.assert_array_equal(original,np.zeros((2,3)))

    def test_blunt_endpoint_force_and_reference_transfer_are_retained(self):
        body,x,area=self.body(257,blunt=True)
        alternative=_attached_recovery_design_model(body)
        velocity=np.array([2.,.01,0.]);zero=np.zeros(3)
        force,moment=alternative.forces(velocity,zero,zero)
        expected_force=2.*.01*(area[-1]-area[0])
        expected_moment=2.*.01*(x[-1]*area[-1]-np.trapezoid(area,x))
        self.assertAlmostEqual(force[1],expected_force,places=12)
        self.assertLess(abs(moment[2]-expected_moment),1.e-5)
        shift=np.array([.7,.2,-.1])
        shifted_force,shifted_moment=alternative.forces(velocity,zero,shift)
        np.testing.assert_allclose(shifted_force,force,rtol=1.e-13,atol=1.e-14)
        np.testing.assert_allclose(shifted_moment,moment-np.cross(shift,force),rtol=1.e-13,atol=1.e-14)


if __name__=='__main__':
    unittest.main()
