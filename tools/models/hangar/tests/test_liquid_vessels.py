"""Independent inferred-vessel controls; no authored flight input is changed."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import manifold3d as mf
import numpy as np

from hangar import liquid, liquid_vessels as vessels


def declaration():
    return dict(schema='fsim.inferred-liquid-vessels.v1', design='rq4b',
                basis='theoretical_inference', geometry='original_source_wing_root_strip_v1',
                attachment='sealed_whole_native_tank_wing_module_v1',
                tanks=[dict(index=0, section='left wing', surface='wing'),
                       dict(index=1, section='right wing', surface='wing')])


def tank(i, volume, side):
    return dict(index=i, capacity_kg=800*volume, density_kg_m3=800.,
                anchor_body_m=[.5, side*1.5, .5], section='left wing' if side < 0 else 'right wing')


class Partition:
    y_root = 1.
    origin = np.zeros(3)

    def possible(self, lo, hi):
        if hi[1] <= -1.: return frozenset({'left wing'})
        if lo[1] >= 1.: return frozenset({'right wing'})
        return frozenset({'centre'}) if lo[1] >= -1. and hi[1] <= 1. else frozenset({'centre', 'left wing', 'right wing'})

    def candidate_region(self, owner, lo, hi):
        lo, hi = np.array(lo).copy(), np.array(hi).copy()
        if owner == 'left wing': hi[1] = min(hi[1], -1.)
        else: lo[1] = max(lo[1], 1.)
        return liquid._box(lo, hi)


class VesselTests(unittest.TestCase):
    def test_declaration_is_exact_bounded_and_only_two_inboard_native_tanks(self):
        doc = declaration(); self.assertEqual(vessels.validate_declaration(doc, 'rq4b'), doc)
        mutations = [lambda d: d.update(design='f16'), lambda d: d.update(capacity=5000),
                     lambda d: d.update(geometry='any_fuselage'), lambda d: d.update(basis='OEM'),
                     lambda d: d['tanks'][0].update(index=2), lambda d: d['tanks'].reverse(),
                     lambda d: d['tanks'][0].update(section='right wing'),
                     lambda d: d['tanks'][0].update(surface='tail'),
                     lambda d: d['tanks'][0].update(index=True)]
        for mutate in mutations:
            bad = copy.deepcopy(doc); mutate(bad)
            with self.subTest(bad=bad), self.assertRaises(liquid.LiquidError):
                vessels.validate_declaration(bad, 'rq4b')

    def test_optional_file_does_not_allow_duplicate_keys_symlinks_or_malformed_downgrade(self):
        with tempfile.TemporaryDirectory(dir='D:/ENV/tmp') as temporary:
            source = Path(temporary)/'rq4b.toml'; source.write_text('', encoding='utf-8')
            self.assertIsNone(vessels.read_declaration(source, 'rq4b'))
            path = source.with_suffix('.liquid-geometry-v1.json')
            for raw in ('{"schema":0,"schema":1}', '[1]', '{"x":NaN}', ' ' * 16385):
                path.write_text(raw, encoding='utf-8')
                with self.subTest(raw=raw[:40]), self.assertRaises(liquid.LiquidError):
                    vessels.read_declaration(source, 'rq4b')
            path.write_text(json.dumps(declaration()), encoding='utf-8')
            self.assertEqual(vessels.read_declaration(source, 'rq4b')[0], declaration())

    def policy(self):
        # Exact boxes are independent geometric oracles; source wing fixture is
        # narrower than the whole hull, so a whole-hull-only shortcut is caught.
        root = mf.Manifold.cube([1., 2., 1.]).translate([0., -1., 0.])
        tanks = [tank(0, 1.25, -1), tank(1, 1.25, 1), tank(2, .5, -1), tank(3, .5, 1)]
        return vessels.Policy(declaration(), Partition(), root, tanks), tanks

    def test_per_tank_and_final_slab_certificates_do_not_expand_outboard_or_cross_centre(self):
        policy, _ = self.policy()
        left = (np.array([.1, -.9, .1]), np.array([.9, -.1, .9]))
        self.assertEqual(policy.certificate(0, *left), 1)
        self.assertEqual(policy.certificate(2, *left), -1)
        self.assertEqual(policy.certificate(1, *left), -1)
        self.assertEqual(policy.certificate(0, np.array([.9, -.8, .1]), np.array([1.1, -.2, .9])), 0)
        self.assertEqual(policy.certificate(0, np.array([1.1, -.8, .1]), np.array([1.3, -.2, .9])), -1)
        self.assertNotEqual(policy.certificate(0, np.array([.1, -.1, .1]), np.array([.9, .1, .9])), 1)
        self.assertEqual(policy.assignment_order([0, 1, 2, 3]), [2, 3, 0, 1])

    def test_restricted_first_real_pack_closes_without_borrowing_root_for_outboard(self):
        policy, tanks = self.policy()
        hull = mf.Manifold.cube([1., 4., 1.]).translate([0., -2., 0.])
        sections = {'left wing': None, 'right wing': None}
        # Each side requires 1.75m3; ordinary wing has 1m3. Allocating the
        # flexible tank first near its native anchor would starve TK2/TK3.
        result = liquid._pack_owned('rq4b', hull, sections, tanks, None,
            dict(max_seconds=30.), aircraft=SimpleNamespace(), origin=np.zeros(3), vessel_policy=policy)
        by_tank = {t['index']: [] for t in tanks}
        for cell in result['cells']:
            by_tank[cell['tank']].append(cell)
            self.assertEqual(policy.certificate(cell['tank'], np.array(cell['low_m']), np.array(cell['high_m'])), 1)
            if cell['tank'] in (2, 3): self.assertEqual(cell['eligibility_v1'], 'native_owner_interval_v1')
        for t in tanks:
            self.assertAlmostEqual(sum(c['volume_m3'] for c in by_tank[t['index']]), t['capacity_kg']/800., places=10)
        receipt = result['inferred_vessels_v1']
        self.assertFalse(result['ledger']['native_owner_interval_certified'])
        self.assertTrue(result['ledger']['per_tank_vessel_eligibility_certified'])
        for row in receipt['tanks']:
            self.assertGreater(row['root_m3'], 0.)
            self.assertAlmostEqual(row['ordinary_m3']+row['root_m3'], row['full']['volume_m3'])

    def test_declared_vessels_retain_real_hull_equipment_and_global_nonoverlap_checks(self):
        policy, tanks = self.policy()
        hull = mf.Manifold.cube([1., 4., 1.]).translate([0., -2., 0.])
        excluded = mf.Manifold.cube([1., .2, 1.]).translate([0., -.9, 0.])
        exclusions = [dict(low_m=[0., -.9, 0.], high_m=[1., -.7, 1.])]
        result = liquid._pack_owned('rq4b', hull, {'left wing': None, 'right wing': None}, tanks, excluded,
            dict(max_seconds=30.), aircraft=SimpleNamespace(), origin=np.zeros(3),
            exclusion_boxes=exclusions, vessel_policy=policy)
        cells = result['cells']
        for i, cell in enumerate(cells):
            lo, hi = np.array(cell['low_m']), np.array(cell['high_m'])
            self.assertTrue(np.any(np.minimum(hi, exclusions[0]['high_m']) <= np.maximum(lo, exclusions[0]['low_m'])))
            self.assertTrue(liquid._valid(liquid._box(lo, hi)-hull, 'independent test hull').is_empty())
            for other in cells[:i]:
                self.assertTrue(np.any(np.minimum(hi, other['high_m']) <= np.maximum(lo, other['low_m'])))
        small_hull = mf.Manifold.cube([.75, 4., 1.]).translate([0., -2., 0.])
        with self.assertRaises(liquid.LiquidError):
            liquid._pack_owned('rq4b', small_hull, {'left wing': None, 'right wing': None}, tanks, None,
                dict(max_seconds=30.), aircraft=SimpleNamespace(), origin=np.zeros(3), vessel_policy=policy)

    def test_centroid_and_half_volume_use_actual_spatial_slices_not_half_density(self):
        cells = [dict(low_m=[0., 0., 0.], high_m=[2., 1., 1.], volume_m3=2.),
                 dict(low_m=[2., 0., 1.], high_m=[3., 1., 3.], volume_m3=2.)]
        full = vessels.spatial_summary(cells, [0., 0., 0.], 1.)
        half = vessels.spatial_summary(cells, [0., 0., 0.], .5)
        np.testing.assert_allclose(full['centroid_body_m'], [1.75, .5, 1.25], atol=1e-12)
        self.assertAlmostEqual(half['volume_m3'], 2., places=12)
        np.testing.assert_allclose(half['centroid_body_m'], [2.5, .5, 2.], atol=1e-12)
        self.assertAlmostEqual(half['surface_body_z_m'], 1., places=12)
        self.assertEqual(half['low_m'], [2., 0., 1.])
        with self.assertRaises(liquid.LiquidError): vessels.spatial_summary(cells, [0, 0, 0], float('nan'))

    def test_root_source_scene_binds_real_foil_bytes_and_refuses_stale_loaded_tables(self):
        from hangar.geometry.surface import Surface
        from hangar.shape import meshkit
        with tempfile.TemporaryDirectory(dir='D:/ENV/tmp') as temporary:
            directory = Path(temporary); source = directory/'rq4b.toml'; source.write_text('', encoding='utf-8')
            foil = directory/'lrn1015.dat'
            foil.write_bytes((Path(__file__).resolve().parents[4]/'aircraft/rq4b/lrn1015.dat').read_bytes())
            path = source.with_suffix('.liquid-geometry-v1.json'); path.write_text(json.dumps(declaration()), encoding='utf-8')
            surface = Surface(dict(name='wing', kind='wing', mirror=True, airfoil='lrn1015.dat', sections=[
                dict(le=[0., 0., 0.], chord=1., twist=3.), dict(le=[1., 5., .2], chord=.5, twist=0.)]), str(directory))
            body = SimpleNamespace(kind='fuselage', x=np.array([-1., 2.]), w=np.array([1.9, 1.9]))
            aircraft = SimpleNamespace(name='rq4b', path=source, surfaces=[surface], bodies=[body],
                                       extent=lambda: (np.array([-1., -5., -1.]), np.array([2., 5., 1.])))
            mesh = mf.Manifold.cube([1., 2., 1.]).translate([0., -1., 0.]).to_mesh64()
            numerical_backend = directory/'test-mesher.dll'; numerical_backend.write_bytes(b'explicit method test double')
            result = dict(positions=np.asarray(mesh.vert_properties), triangles=np.asarray(mesh.tri_verts),
                          boundary_edges=0, nonmanifold_edges=0, misoriented_edges=0, degenerate_triangles=0)
            with patch.object(meshkit, 'build', return_value=result) as build, patch.object(meshkit, 'library', return_value=SimpleNamespace(_name=str(numerical_backend))):
                _, sources = vessels.create(aircraft, np.zeros(3), self.policy()[1], (declaration(), path))
                refs = {r['role']: r for r in sources}
                self.assertEqual(refs['inferred_vessel_airfoil_coordinates']['sha256'], hashlib.sha256(foil.read_bytes()).hexdigest())
                self.assertIn('inferred_vessel_airfoil_method', refs); self.assertIn('inferred_vessel_section_method', refs)
                scene = build.call_args.args[0]; original = scene['root']['children'][0]
                self.assertEqual(original['inflate'], 0.)
                np.testing.assert_array_equal(original['le'], [s.le for s in surface.sections])
                np.testing.assert_array_equal(original['chord'], [s.chord for s in surface.sections])
                # Changing the current file while preserving the already loaded
                # aircraft cannot produce a fresh-looking mixed source receipt.
                surface.sections[0].airfoil._yc = surface.sections[0].airfoil._yc+.01
                build.reset_mock()
                with self.assertRaisesRegex(liquid.LiquidError, 'Loaded.*airfoil'):
                    vessels.create(aircraft, np.zeros(3), self.policy()[1], (declaration(), path))
                build.assert_not_called()
                foil.unlink()
                with self.assertRaisesRegex(liquid.LiquidError, 'coordinates'):
                    vessels.create(aircraft, np.zeros(3), self.policy()[1], (declaration(), path))


class ExactDyadicBandTests(unittest.TestCase):
    @staticmethod
    def roof(scale=1., rational=False):
        points=np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0],
                         [0,0,1],[1,0,1],[1,1,1],[0,1,1]],dtype=float)
        triangles=np.array([[0,4,7],[0,7,3],[1,2,6],[1,6,5],
                            [0,1,5],[0,5,4],[3,7,6],[3,6,2],
                            [0,3,2],[0,2,1],[4,5,6],[4,6,7]],dtype=np.uint64)
        if rational:
            points[:,2]+=points[:,0]+points[:,1]
            points[:,:2]*=3
        else: points[:,2]+=.25*points[:,0]+.125*points[:,1]
        return SimpleNamespace(vert_properties=points*scale,tri_verts=triangles)

    def test_band_matches_independent_dyadic_roof_at_extreme_finite_scales(self):
        from fractions import Fraction
        for scale in (1.,2.**-1000,2.**900):
            with self.subTest(scale=scale):
                lo=np.array([.25,.25,-1])*scale;hi=np.array([.75,.75,2])*scale
                result=vessels._inscribed_vertical_band(self.roof(scale),lo,hi)
                self.assertEqual(result['lower'],(9/32)*scale)
                self.assertEqual(result['upper'],(35/32)*scale)
                self.assertFalse(result['vertical_conflict'])
                self.assertEqual(result['sensitivity'],(Fraction(1,8)*Fraction.from_float(scale),
                                                        Fraction(1,16)*Fraction.from_float(scale)))

    def test_band_rounds_independent_rational_roof_only_inward(self):
        from fractions import Fraction
        import math
        lo=np.array([.5,.125,-1]);hi=np.array([1.,.25,3])
        result=vessels._inscribed_vertical_band(self.roof(rational=True),lo,hi)
        # z=x/3+y/3 and z=1+x/3+y/3 over this XY rectangle.
        lower,upper=Fraction(5,12),Fraction(29,24)
        a,b=float(lower),float(upper)
        if Fraction.from_float(a)<lower:a=math.nextafter(a,math.inf)
        if Fraction.from_float(b)>upper:b=math.nextafter(b,-math.inf)
        self.assertEqual(result['lower'],a);self.assertEqual(result['upper'],b)
        self.assertGreaterEqual(Fraction.from_float(a),lower)
        self.assertLessEqual(Fraction.from_float(b),upper)
        self.assertEqual(result['sensitivity'],(Fraction(1,6),Fraction(1,24)))

    def test_band_keeps_degenerate_refusal_and_original_deadline_exception(self):
        lo=np.zeros(3);hi=np.ones(3)
        mesh=SimpleNamespace(vert_properties=np.zeros((3,3)),tri_verts=np.array([[0,1,2]],dtype=np.uint64))
        self.assertIsNone(vessels._inscribed_vertical_band(mesh,lo,hi))
        refused=RuntimeError('original bounded deadline sentinel')
        def check():raise refused
        with self.assertRaises(RuntimeError) as caught:
            vessels._inscribed_vertical_band(self.roof(),lo,hi,check)
        self.assertIs(caught.exception,refused)


if __name__ == '__main__': unittest.main(verbosity=2)
