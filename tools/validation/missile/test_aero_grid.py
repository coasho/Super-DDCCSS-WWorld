"""Independent published and manufactured grid-error arithmetic controls."""
import math
import unittest

from aero_grid import richardson


class GridUncertainty(unittest.TestCase):
    def test_nasa_pressure_recovery_example(self):
        row = richardson([1, 2, 4], [.97050, .96854, .96178], normalizer=.97050)
        self.assertAlmostEqual(row["observed_order"], 1.78617, places=5)
        self.assertAlmostEqual(row["extrapolated"], .9713003, places=7)
        self.assertAlmostEqual(row["normalized_uncertainty"], .0010308, places=7)

    def test_unequal_refinement_manufactured_quadratic(self):
        h = [1, 1.7, 2.3]
        for sign in (-1, 1):
            row = richardson(h, [sign*(3+2*x*x) for x in h], normalizer=3)
            self.assertAlmostEqual(row["observed_order"], 2, places=10)
            self.assertAlmostEqual(row["extrapolated"], sign*3, places=10)
            self.assertAlmostEqual(row["absolute_uncertainty"], 2.5, places=10)

    def test_near_zero_uses_explicit_scale_without_clamping(self):
        row = richardson([1, 2, 4], [-.001, -.004, -.016], normalizer=.1)
        self.assertAlmostEqual(row["extrapolated"], 0)
        self.assertAlmostEqual(row["absolute_uncertainty"], .00125)
        self.assertAlmostEqual(row["normalized_uncertainty"], .0125)

    def test_invalid_or_nonasymptotic_trends_cannot_produce_uncertainty(self):
        for h, f, scale in (([1, 2, 4], [1, 2, 1], 1), ([1, 2, 4], [1, 1, 1], 1),
                            ([1, 1, 4], [1, 2, 3], 1), ([1, 2, 4], [1, 2, math.nan], 1),
                            ([1, 2, 4], [1, 2, 3], 0), ([1, 2, 4], [1, 2, 2.1], 1)):
            with self.subTest(h=h, f=f, scale=scale), self.assertRaises(ValueError):
                richardson(h, f, normalizer=scale)


if __name__ == "__main__":
    unittest.main()
