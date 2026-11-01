"""Retain the three UH-1H fuel-order failures at their original conditions."""
import unittest
from pathlib import Path

from hangar import rotorcraft
from hangar.rotorcraft import performance as P


class RotorPower(unittest.TestCase):
    def test_original_three_uh1h_fuel_order_columns(self):
        import fsim
        fsim.set_log_level('off')
        root = Path(__file__).resolve().parents[4]
        r = rotorcraft.Rotorcraft(str(root/'aircraft/uh1h/uh1h.toml'))
        # Frozen targets from the failing original performance campaign. Do
        # not move the targets with a newly computed top-speed boundary.
        cases = [(100., [58.78132657039039,60.9831293624786,61.30047010874466]),
                 (1000., [56.455842549567414,58.74993822195422,58.570492563147255]),
                 (1000., [60.6221058228794,63.09778092581143,62.89172183611171])]
        runs = P._Runs(r)
        try:
            for h, speeds in cases:
                with self.subTest(altitude_m=h, speeds=speeds):
                    observations = [runs.run(h,v,f) for v,f in zip(speeds,P.FUEL_FRACTIONS)]
                    for v,row in zip(speeds,observations):
                        self.assertTrue(runs.within(row,v),row)
                    flows = [row['fuel_kg_s'] for row in observations]
                    self.assertTrue(all(flows[i] <= 1.01*flows[i+1] for i in range(2)), flows)
        finally:
            runs.close()


if __name__ == '__main__':
    unittest.main()
