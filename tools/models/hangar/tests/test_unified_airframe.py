"""The aircraft solver must handle actual store/body geometries as well."""
from pathlib import Path
import unittest

import numpy as np

from hangar.geometry import Aircraft
from hangar.aero.model import AeroModel
from hangar.aero import tables

ROOT = Path(__file__).resolve().parents[4]


class UnifiedAirframe(unittest.TestCase):
    def test_real_finned_store_uses_the_aircraft_lattice_and_controls(self):
        aircraft = Aircraft.load(str(ROOT/'aircraft/aim120c/aim120c.toml'))
        model = AeroModel(aircraft)
        self.assertGreater(model.lat.n_panels, 0)
        neutral = model.evaluate(.05)
        controlled = model.evaluate(.05, controls={'elevator': .05})
        self.assertTrue(all(np.isfinite(neutral[key]) for key in tables.COEFFS))
        self.assertNotAlmostEqual(neutral['Cm'], controlled['Cm'])

    def test_body_without_fins_does_not_get_a_manufactured_wing(self):
        aircraft = Aircraft.load(str(ROOT/'aircraft/463lpallet/463lpallet.toml'))
        model = AeroModel(aircraft)
        self.assertIsNone(model.lat)
        self.assertEqual(aircraft.surfaces, [])
        for alpha, beta in ((0., 0.), (.3, -.2), (1.5, .5), (3., -.8)):
            result = model.evaluate(alpha, beta, p=.01, q=.01, r=.01)
            self.assertTrue(all(np.isfinite(result[key]) for key in tables.COEFFS))

    def test_body_only_tables_include_the_same_six_axes_and_mach_response(self):
        aircraft = Aircraft.load(str(ROOT/'aircraft/463lpallet/463lpallet.toml'))
        model = AeroModel(aircraft)
        result = tables.build(model, alpha=[-10., -2., 0., 2., 8., 10.], beta=[-5., 0., 5.])
        self.assertEqual(set(result['base']), set(tables.COEFFS)|{'CDi'})
        for values in result['base'].values():
            self.assertTrue(np.isfinite(values).all())
        self.assertTrue(np.isfinite(result['mach']['dCD0']).all())

    def test_single_physical_fin_uses_the_same_solver_without_moving_others(self):
        aircraft = Aircraft.load(str(ROOT/'aircraft/aim120c/aim120c.toml'))
        model = AeroModel(aircraft)
        zero = model.evaluate(.02)
        changed = model.evaluate(.02, pieces={0: .05})
        self.assertTrue(all(np.isfinite(changed[key]) for key in tables.COEFFS))
        self.assertGreater(sum(abs(changed[k]-zero[k]) for k in tables.COEFFS), 0.)

    def test_mach_drag_preserves_separated_drag_and_ignores_previous_frame_lift(self):
        from hangar import airframe,pipeline
        from hangar.verify import TableModel
        for name,alpha,mach in (('aim9m',32.,5.),('irist',55.,4.),('mk82',42.,1.2)):
            with self.subTest(model=name):
                d=airframe.design(pipeline.find_design(name))
                model=TableModel(d.calibrated(d.tables()))
                current=model.evaluate(alpha,0.,mach=mach)
                stale=model.evaluate(alpha,0.,mach=mach,cl2=1.e12)
                self.assertGreaterEqual(current['CD'],0.)
                self.assertEqual(current,stale)
