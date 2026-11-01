"""A source-symmetric section must preserve aerodynamic reflection symmetry."""
import unittest
import numpy as np
from hangar.aero.section import SectionPolar
from hangar.geometry.airfoil import biconvex,naca4


class PolarReflection(unittest.TestCase):
    def test_symmetric_source_has_symmetric_default_lift_limits(self):
        for foil in (biconvex(.04),naca4('0012')):
            with self.subTest(foil=foil.name):
                p=SectionPolar(foil)
                self.assertEqual(p.clmax,-p.clmin)

    def test_full_angle_neutral_lift_and_moment_are_odd_drag_is_even(self):
        alpha=np.radians(np.linspace(-179.,179.,717))
        for foil in (biconvex(.04),naca4('0012')):
            with self.subTest(foil=foil.name):
                p=SectionPolar(foil)
                values=p.evaluate(alpha);reflected=p.evaluate(-alpha)
                for index,sign in ((0,-1),(1,1),(2,-1)):
                    np.testing.assert_allclose(values[index],sign*reflected[index],rtol=0,atol=2e-12)
                self.assertEqual(float(p.evaluate(0.)[0]),0.)

    def test_flap_reflection_and_authored_asymmetric_limits(self):
        alpha=np.radians(np.linspace(-35.,35.,141))
        p=SectionPolar(biconvex(.04),flap_chord=.3)
        for delta in np.radians([2.,10.,20.]):
            a=p.evaluate(alpha,delta);b=p.evaluate(-alpha,-delta)
            for index,sign in ((0,-1),(1,1),(2,-1)):
                np.testing.assert_allclose(a[index],sign*b[index],rtol=0,atol=2e-12)
        authored=SectionPolar(biconvex(.04),overrides=dict(clmax=1.2,clmin=-.8))
        self.assertEqual(authored.clmax,1.2)
        self.assertEqual(authored.clmin,-.8)


if __name__=='__main__':unittest.main()
