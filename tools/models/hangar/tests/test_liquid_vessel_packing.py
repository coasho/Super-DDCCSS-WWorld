"""Independent nested-eligibility and final-audit controls for inferred RQ packing."""
import copy
from fractions import Fraction
import math
from collections import Counter
import unittest
from unittest.mock import patch
from types import SimpleNamespace

import manifold3d as mf
import numpy as np

from hangar import liquid, liquid_vessels as vessels
from test_liquid_vessels import Partition, declaration, tank


# Exact original prior AABB endpoints from the retained 134-cell native witness.
PRIOR_CONTACT_BOUNDS = (
    ((0.2421875, -1.625, 0.25), (0.484375, -1.5, 0.5)),
    ((0.2421875, -1.625, 0.5), (0.484375, -1.5, 0.75)),
    ((0.2421875, -1.5, 0.25), (0.484375, -1.375, 0.5)),
    ((0.2421875, -1.5, 0.5), (0.484375, -1.375, 0.75)),
    ((0.515625, -1.625, 0.25), (0.7578125, -1.5, 0.5)),
    ((0.515625, -1.625, 0.5), (0.7578125, -1.5, 0.75)),
    ((0.515625, -1.5, 0.25), (0.7578125, -1.375, 0.5)),
    ((0.515625, -1.5, 0.5), (0.7578125, -1.375, 0.75)),
    ((0.2421875, -1.75, 0.25), (0.484375, -1.625, 0.5)),
    ((0.2421875, -1.75, 0.5), (0.484375, -1.625, 0.75)),
    ((0.2421875, -1.375, 0.25), (0.484375, -1.25, 0.5)),
    ((0.2421875, -1.375, 0.5), (0.484375, -1.25, 0.75)),
    ((0.515625, -1.75, 0.25), (0.7578125, -1.625, 0.5)),
    ((0.515625, -1.75, 0.5), (0.7578125, -1.625, 0.75)),
    ((0.515625, -1.375, 0.25), (0.7578125, -1.25, 0.5)),
    ((0.515625, -1.375, 0.5), (0.7578125, -1.25, 0.75)),
    ((0.2421875, -1.875, 0.25), (0.484375, -1.75, 0.5)),
    ((0.2421875, -1.875, 0.5), (0.484375, -1.75, 0.75)),
    ((0.2421875, -1.25, 0.25), (0.484375, -1.125, 0.5)),
    ((0.2421875, -1.25, 0.5), (0.484375, -1.125, 0.75)),
    ((0.515625, -1.875, 0.25), (0.7578125, -1.75, 0.5)),
    ((0.515625, -1.875, 0.5), (0.7578125, -1.75, 0.75)),
    ((0.515625, -1.25, 0.25), (0.7578125, -1.125, 0.5)),
    ((0.515625, -1.25, 0.5), (0.7578125, -1.125, 0.75)),
    ((0.0, -1.625, 0.25), (0.2421875, -1.5, 0.5)),
    ((0.0, -1.625, 0.5), (0.2421875, -1.5, 0.75)),
    ((0.0, -1.5, 0.25), (0.2421875, -1.375, 0.5)),
    ((0.0, -1.5, 0.5), (0.2421875, -1.375, 0.75)),
    ((0.7578125, -1.625, 0.25), (1.0, -1.5, 0.5)),
    ((0.7578125, -1.625, 0.5), (1.0, -1.5, 0.75)),
    ((0.7578125, -1.5, 0.25), (1.0, -1.375, 0.5)),
    ((0.7578125, -1.5, 0.5), (1.0, -1.375, 0.75)),
    ((0.2421875, -1.625, 0.0), (0.484375, -1.5, 0.25)),
    ((0.2421875, -1.625, 0.75), (0.484375, -1.5, 1.0)),
    ((0.2421875, -1.5, 0.0), (0.484375, -1.375, 0.25)),
    ((0.2421875, -1.5, 0.75), (0.484375, -1.375, 1.0)),
    ((0.515625, -1.625, 0.0), (0.7578125, -1.5, 0.25)),
    ((0.515625, -1.625, 0.75), (0.7578125, -1.5, 1.0)),
    ((0.515625, -1.5, 0.0), (0.7578125, -1.375, 0.25)),
    ((0.515625, -1.5, 0.75), (0.7578125, -1.375, 1.0)),
    ((0.0, -1.75, 0.25), (0.2421875, -1.625, 0.5)),
    ((0.0, -1.75, 0.5), (0.2421875, -1.625, 0.75)),
    ((0.0, -1.375, 0.25), (0.2421875, -1.25, 0.5)),
    ((0.0, -1.375, 0.5), (0.2421875, -1.25, 0.75)),
    ((0.7578125, -1.75, 0.25), (1.0, -1.625, 0.5)),
    ((0.7578125, -1.75, 0.5), (1.0, -1.625, 0.75)),
    ((0.7578125, -1.375, 0.25), (1.0, -1.25, 0.5)),
    ((0.7578125, -1.375, 0.5), (1.0, -1.25, 0.75)),
    ((0.2421875, -1.75, 0.0), (0.484375, -1.625, 0.25)),
    ((0.2421875, -1.75, 0.75), (0.484375, -1.625, 1.0)),
    ((0.2421875, -1.375, 0.0), (0.484375, -1.25, 0.25)),
    ((0.2421875, -1.375, 0.75), (0.484375, -1.25, 1.0)),
    ((0.515625, -1.75, 0.0), (0.7578125, -1.625, 0.25)),
    ((0.515625, -1.75, 0.75), (0.7578125, -1.625, 1.0)),
    ((0.515625, -1.375, 0.0), (0.7578125, -1.25, 0.25)),
    ((0.515625, -1.375, 0.75), (0.7578125, -1.25, 1.0)),
    ((0.2421875, -2.0, 0.25), (0.484375, -1.875, 0.5)),
    ((0.2421875, -2.0, 0.5), (0.484375, -1.875, 0.75)),
    ((0.2421875, -1.125, 0.25), (0.484375, -1.0, 0.5)),
    ((0.2421875, -1.125, 0.5), (0.484375, -1.0, 0.75)),
    ((0.515625, -2.0, 0.25), (0.7578125, -1.875, 0.5)),
    ((0.515625, -2.0, 0.5), (0.7578125, -1.875, 0.75)),
    ((0.515625, -1.125, 0.25), (0.7578125, -1.0, 0.5)),
    ((0.515625, -1.125, 0.5), (0.7578125, -1.0, 0.75)),
    ((0.0, -1.875, 0.25), (0.2421875, -1.75, 0.5)),
    ((0.0, -1.875, 0.5), (0.2421875, -1.75, 0.75)),
    ((0.0, -1.25, 0.4838709677419355), (0.2421875, -1.125, 0.5)),
    ((0.2421875, 1.375, 0.25), (0.484375, 1.5, 0.5)),
    ((0.2421875, 1.375, 0.5), (0.484375, 1.5, 0.75)),
    ((0.2421875, 1.5, 0.25), (0.484375, 1.625, 0.5)),
    ((0.2421875, 1.5, 0.5), (0.484375, 1.625, 0.75)),
    ((0.515625, 1.375, 0.25), (0.7578125, 1.5, 0.5)),
    ((0.515625, 1.375, 0.5), (0.7578125, 1.5, 0.75)),
    ((0.515625, 1.5, 0.25), (0.7578125, 1.625, 0.5)),
    ((0.515625, 1.5, 0.5), (0.7578125, 1.625, 0.75)),
    ((0.2421875, 1.25, 0.25), (0.484375, 1.375, 0.5)),
    ((0.2421875, 1.25, 0.5), (0.484375, 1.375, 0.75)),
    ((0.2421875, 1.625, 0.25), (0.484375, 1.75, 0.5)),
    ((0.2421875, 1.625, 0.5), (0.484375, 1.75, 0.75)),
    ((0.515625, 1.25, 0.25), (0.7578125, 1.375, 0.5)),
    ((0.515625, 1.25, 0.5), (0.7578125, 1.375, 0.75)),
    ((0.515625, 1.625, 0.25), (0.7578125, 1.75, 0.5)),
    ((0.515625, 1.625, 0.5), (0.7578125, 1.75, 0.75)),
    ((0.2421875, 1.125, 0.25), (0.484375, 1.25, 0.5)),
    ((0.2421875, 1.125, 0.5), (0.484375, 1.25, 0.75)),
    ((0.2421875, 1.75, 0.25), (0.484375, 1.875, 0.5)),
    ((0.2421875, 1.75, 0.5), (0.484375, 1.875, 0.75)),
    ((0.515625, 1.125, 0.25), (0.7578125, 1.25, 0.5)),
    ((0.515625, 1.125, 0.5), (0.7578125, 1.25, 0.75)),
    ((0.515625, 1.75, 0.25), (0.7578125, 1.875, 0.5)),
    ((0.515625, 1.75, 0.5), (0.7578125, 1.875, 0.75)),
    ((0.0, 1.375, 0.25), (0.2421875, 1.5, 0.5)),
    ((0.0, 1.375, 0.5), (0.2421875, 1.5, 0.75)),
    ((0.0, 1.5, 0.25), (0.2421875, 1.625, 0.5)),
    ((0.0, 1.5, 0.5), (0.2421875, 1.625, 0.75)),
    ((0.7578125, 1.375, 0.25), (1.0, 1.5, 0.5)),
    ((0.7578125, 1.375, 0.5), (1.0, 1.5, 0.75)),
    ((0.7578125, 1.5, 0.25), (1.0, 1.625, 0.5)),
    ((0.7578125, 1.5, 0.5), (1.0, 1.625, 0.75)),
    ((0.2421875, 1.375, 0.0), (0.484375, 1.5, 0.25)),
    ((0.2421875, 1.375, 0.75), (0.484375, 1.5, 1.0)),
    ((0.2421875, 1.5, 0.0), (0.484375, 1.625, 0.25)),
    ((0.2421875, 1.5, 0.75), (0.484375, 1.625, 1.0)),
    ((0.515625, 1.375, 0.0), (0.7578125, 1.5, 0.25)),
    ((0.515625, 1.375, 0.75), (0.7578125, 1.5, 1.0)),
    ((0.515625, 1.5, 0.0), (0.7578125, 1.625, 0.25)),
    ((0.515625, 1.5, 0.75), (0.7578125, 1.625, 1.0)),
    ((0.0, 1.25, 0.25), (0.2421875, 1.375, 0.5)),
    ((0.0, 1.25, 0.5), (0.2421875, 1.375, 0.75)),
    ((0.0, 1.625, 0.25), (0.2421875, 1.75, 0.5)),
    ((0.0, 1.625, 0.5), (0.2421875, 1.75, 0.75)),
    ((0.7578125, 1.25, 0.25), (1.0, 1.375, 0.5)),
    ((0.7578125, 1.25, 0.5), (1.0, 1.375, 0.75)),
    ((0.7578125, 1.625, 0.25), (1.0, 1.75, 0.5)),
    ((0.7578125, 1.625, 0.5), (1.0, 1.75, 0.75)),
    ((0.2421875, 1.25, 0.0), (0.484375, 1.375, 0.25)),
    ((0.2421875, 1.25, 0.75), (0.484375, 1.375, 1.0)),
    ((0.2421875, 1.625, 0.0), (0.484375, 1.75, 0.25)),
    ((0.2421875, 1.625, 0.75), (0.484375, 1.75, 1.0)),
    ((0.515625, 1.25, 0.0), (0.7578125, 1.375, 0.25)),
    ((0.515625, 1.25, 0.75), (0.7578125, 1.375, 1.0)),
    ((0.515625, 1.625, 0.0), (0.7578125, 1.75, 0.25)),
    ((0.515625, 1.625, 0.75), (0.7578125, 1.75, 1.0)),
    ((0.2421875, 1.0, 0.25), (0.484375, 1.125, 0.5)),
    ((0.2421875, 1.0, 0.5), (0.484375, 1.125, 0.75)),
    ((0.2421875, 1.875, 0.25), (0.484375, 2.0, 0.5)),
    ((0.2421875, 1.875, 0.5), (0.484375, 2.0, 0.75)),
    ((0.515625, 1.0, 0.25), (0.7578125, 1.125, 0.5)),
    ((0.515625, 1.0, 0.5), (0.7578125, 1.125, 0.75)),
    ((0.515625, 1.875, 0.25), (0.7578125, 2.0, 0.5)),
    ((0.515625, 1.875, 0.5), (0.7578125, 2.0, 0.75)),
    ((0.0, 1.125, 0.25), (0.2421875, 1.25, 0.5)),
    ((0.0, 1.125, 0.5), (0.2421875, 1.25, 0.75)),
    ((0.0, 1.75, 0.4838709677419355), (0.2421875, 1.875, 0.5)),
)

def original_certificate(self, index, lo, hi):
    """Frozen pre-extraction predicate; source method was 2554778a/77bc4c78."""
    possible = self.classifier.possible(lo, hi); owner = self.tanks[index]['section']
    ordinary = 1 if possible == frozenset({owner}) else 0 if owner in possible else -1
    if ordinary == 1 or index not in self.selected: return ordinary
    origin_y = self.classifier.origin[1]; cut = self.classifier.y_root
    lower, upper = (-cut-origin_y, -origin_y) if index == 0 else (-origin_y, cut-origin_y)
    if (hi[1] <= lower or lo[1] >= upper or
            np.any(hi <= self.bounds[:3]) or np.any(lo >= self.bounds[3:])): return ordinary
    if lo[1] < lower or hi[1] > upper: return max(ordinary, 0)
    box = liquid._box(lo, hi)
    if liquid._valid(box-self.root, 'Inferred root cell/source wing difference').is_empty(): return 1
    if liquid._valid(box ^ self.root, 'Inferred root cell/source wing intersection').is_empty(): return ordinary
    return max(ordinary, 0)


class SharedVesselPackingTests(unittest.TestCase):
    def test_exact_box_mesh_certificate_preserves_provided_seam_topology(self):
        lo, hi = np.array([-3., 7., 11.]), np.array([-1., 10., 15.])
        native = mf.Manifold.cube(hi-lo).translate(lo).to_mesh64()
        self.assertTrue(vessels._represented_box(native, lo, hi))
        positions = np.asarray(native.vert_properties, float)
        triangles = np.asarray(native.tri_verts, np.uint64)
        duplicated = positions[triangles.reshape(-1)]
        first = {}; source = []; target = []
        for index, original in enumerate(triangles.reshape(-1).tolist()):
            if original in first: source.append(index); target.append(first[original])
            else: first[original] = index
        seam = SimpleNamespace(vert_properties=duplicated, tri_verts=np.arange(36, dtype=np.uint64).reshape((-1, 3)),
            merge_from_vert=np.array(source, np.uint64), merge_to_vert=np.array(target, np.uint64))
        self.assertTrue(vessels._represented_box(seam, lo, hi))
        # Equal coordinates do not authorize welding distinct geometric fans.
        seam.merge_from_vert = seam.merge_from_vert[:-1]; seam.merge_to_vert = seam.merge_to_vert[:-1]
        self.assertFalse(vessels._represented_box(seam, lo, hi))

    def test_box_certificate_rejects_duplicate_shell_cavity_and_ulp_notch(self):
        lo, hi = np.zeros(3), np.ones(3); native = mf.Manifold.cube([1., 1., 1.]).to_mesh64()
        positions = np.asarray(native.vert_properties, float); triangles = np.asarray(native.tri_verts, np.uint64)
        def mesh(p, t): return SimpleNamespace(vert_properties=p, tri_verts=t, merge_from_vert=[], merge_to_vert=[])
        doubled = mesh(np.concatenate((positions, positions)), np.concatenate((triangles, triangles+len(positions))))
        self.assertFalse(vessels._represented_box(doubled, lo, hi))
        cavity = (mf.Manifold.cube([1., 1., 1.])-mf.Manifold.cube([.5, .5, .5]).translate([.25]*3)).to_mesh64()
        self.assertFalse(vessels._represented_box(cavity, lo, hi))
        notch = positions.copy(); index = next(i for i, p in enumerate(notch) if np.all(p == 1.))
        notch[index, 2] = np.nextafter(1., 0.)
        # Feed represented double vertices directly: native ingest may simplify
        # a sub-epsilon notch before ANY later certificate can observe it.
        self.assertFalse(vessels._represented_box(mesh(notch, triangles), lo, hi))
        for change in ('inward', 'degenerate', 'missing', 'duplicate'):
            bad = triangles.copy()
            if change == 'inward': bad[0] = bad[0][::-1]
            elif change == 'degenerate': bad[0, 1] = bad[0, 0]
            elif change == 'missing': bad = bad[:-1]
            else: bad[0] = bad[1]
            with self.subTest(change=change): self.assertFalse(vessels._represented_box(mesh(positions, bad), lo, hi))

    def test_box_certificate_agrees_with_valid_difference_and_rejects_partial_intersection(self):
        outer = mf.Manifold.cube([2., 2., 2.]); lo, hi = np.array([.25]*3), np.array([.75]*3)
        box = liquid._box(lo, hi); intersection = liquid._valid(box ^ outer, 'analytic containment intersection', nonempty=True)
        self.assertTrue(vessels._represented_box(intersection.to_mesh64(), lo, hi))
        self.assertTrue(liquid._valid(box-outer, 'analytic reference difference').is_empty())
        clipped = liquid._valid(box ^ mf.Manifold.cube([.5, 2., 2.]), 'analytic partial intersection', nonempty=True)
        self.assertFalse(vessels._represented_box(clipped.to_mesh64(), lo, hi))
        self.assertFalse(liquid._valid(box-mf.Manifold.cube([.5, 2., 2.]), 'analytic partial difference').is_empty())

    def wedge(self):
        mesh = mf.Manifold.cube([1., 4., 1.]).to_mesh64(); p = np.asarray(mesh.vert_properties).copy()
        p[:, 2] = p[:, 0]/8+p[:, 2]/128
        return mf.Manifold(mf.Mesh64(np.array(p, dtype=np.float64, order='C', copy=True),
                                     np.array(mesh.tri_verts, dtype=np.uint64, order='C', copy=True)))

    def test_all_facet_band_and_sensitive_axis_preserve_long_flat_direction(self):
        solid = self.wedge(); lo, hi = np.array([0., 0., 0.]), np.array([1., 4., 17/128])
        proposal = vessels._inscribed_vertical_band(solid.to_mesh64(), lo, hi)
        self.assertGreaterEqual(proposal['lower'], proposal['upper'])
        self.assertEqual(vessels._column_residual_axis(proposal, lo, hi, .5), 1)
        narrow = np.array([1/32, .5, 5/128])
        piece = liquid._valid(liquid._box(lo, narrow) ^ solid, 'analytic narrow wedge', nonempty=True)
        proposal = vessels._inscribed_vertical_band(piece.to_mesh64(), lo, narrow)
        self.assertEqual(proposal['lower'], 1/256); self.assertEqual(proposal['upper'], 1/128)
        self.assertEqual(vessels._column_residual_axis(proposal, lo, narrow, .5), 0)
        core_lo, core_hi = lo.copy(), narrow.copy(); core_lo[2], core_hi[2] = proposal['lower'], proposal['upper']
        self.assertTrue(liquid._valid(liquid._box(core_lo, core_hi)-piece, 'analytic core difference').is_empty())

    def test_band_partition_is_exact_and_nonconvex_failure_is_not_empty_material(self):
        lo, hi = np.array([0., 0., 0.]), np.array([1., 4., 1.])
        parts = vessels._column_partition(lo, hi, .25, .75)
        total = Fraction()
        for a, b, kind in parts:
            self.assertTrue(np.all(a >= lo) and np.all(b <= hi) and np.all(b > a))
            total += math.prod(Fraction.from_float(float(y))-Fraction.from_float(float(x)) for x, y in zip(a, b))
        self.assertEqual(total, Fraction(4)); self.assertEqual({p[2] for p in parts}, {'core', 'lower', 'upper'})
        vessels._audit_disjoint([dict(low_m=a.tolist(), high_m=b.tolist()) for a, b, _ in parts], lambda: None)
        cavity = mf.Manifold.cube([1., 1., 1.])-mf.Manifold.cube([.5, .5, .5]).translate([.25]*3)
        self.assertGreater(cavity.volume(), 0.)
        result = vessels._inscribed_vertical_band(cavity.to_mesh64(), np.zeros(3), np.ones(3))
        self.assertTrue(result is None or result['lower'] >= result['upper'] or result['vertical_conflict'])

    def test_production_shared_search_closes_exact_thin_wedge_without_resource_relaxation(self):
        self.assertTrue(callable(vessels._inscribed_vertical_band))
        hull = self.wedge(); volume = (15/512)/4
        tanks = [dict(index=i, capacity_kg=800*volume, density_kg_m3=800.,
                      anchor_body_m=[.5, 1. if i in (0, 2) else 3., .0625],
                      section='left wing' if i in (0, 2) else 'right wing') for i in range(4)]
        class WedgePolicy:
            selected = {0, 1}
            def __init__(self): self.tanks = {t['index']: t for t in tanks}
            def assignment_order(self, ids): return [2, 3, 0, 1]
            def _certificates(self, a, b):
                if b[1] <= 2.: return (1, -1, 1, -1)
                if a[1] >= 2.: return (-1, 1, -1, 1)
                return (0, 0, 0, 0)
            def certificate(self, i, a, b): return self._certificates(a, b)[i]
            def region(self, i, a, b): return liquid._box(a, b)
            def eligibility(self, i, a, b):
                if self.certificate(i, a, b) != 1: raise liquid.LiquidError('analytic owner refusal')
                return vessels.ORDINARY
            def receipt(self, cells): return {'scope': 'synthetic exact dyadic wedge; not native RQ geometry'}
        result = vessels._pack_shared('rq4b', hull, tanks, None, dict(max_seconds=60.), WedgePolicy())
        self.assertEqual(result['limits']['max_cells'], 65536)
        self.assertLessEqual(result['ledger']['screened_cells'], 1048576)
        self.assertLessEqual(len(result['cells']), 65536)
        self.assertAlmostEqual(math.fsum(c['volume_m3'] for c in result['cells']), 15/512, places=12)
        self.assertTrue(all(max(b-a for a, b in zip(c['low_m'], c['high_m'])) <= .5*(1+1e-12) for c in result['cells']))
        self.assertTrue(result['ledger']['global_aabb_nonoverlap_verified'])
    def test_actual_134_prior_box_contact_witness_has_exact_fraction_proof_and_ulp_refusal(self):
        # Captured before correction, SHA09325ac7...e2dc. The general CSG
        # returned NoError/-8.673617379884035e-19 for these represented boxes.
        # This self-contained oracle does not depend on that backend bug persisting.
        cells = [dict(low_m=list(lo), high_m=list(hi)) for lo, hi in PRIOR_CONTACT_BOUNDS]
        lo, hi = np.array([.7578125, -1.25, .25]), np.array([1., -1.125, .5])
        qlo, qhi = list(map(Fraction.from_float, lo)), list(map(Fraction.from_float, hi))
        positive = 0; contacts = 0; face = None
        for low, high in PRIOR_CONTACT_BOUNDS:
            a, b = list(map(Fraction.from_float, low)), list(map(Fraction.from_float, high))
            lengths = [min(qhi[i], b[i])-max(qlo[i], a[i]) for i in range(3)]
            positive += all(v > 0 for v in lengths)
            contacts += all(v >= 0 for v in lengths) and any(v == 0 for v in lengths)
            if sum(v == 0 for v in lengths) == 1 and all(v >= 0 for v in lengths): face = (a, b, lengths)
        self.assertEqual(len(cells), 134); self.assertEqual(positive, 0); self.assertEqual(contacts, 9)
        bounds = vessels._prior_endpoints(cells)
        self.assertTrue(vessels._prior_clear(lo, hi, bounds))
        self.assertIsNotNone(face)
        a, b, lengths = face; axis = next(i for i, v in enumerate(lengths) if v == 0)
        moved_lo, moved_hi = lo.copy(), hi.copy()
        if qhi[axis] == a[axis]: moved_hi[axis] = np.nextafter(hi[axis], math.inf)
        else: moved_lo[axis] = np.nextafter(lo[axis], -math.inf)
        self.assertFalse(vessels._prior_clear(moved_lo, moved_hi, bounds))
        exact_positive = any(all(min(Fraction.from_float(float(moved_hi[i])), Fraction.from_float(high[i])) >
                                 max(Fraction.from_float(float(moved_lo[i])), Fraction.from_float(low[i])) for i in range(3))
                             for low, high in PRIOR_CONTACT_BOUNDS)
        self.assertTrue(exact_positive)

    def test_malformed_or_nonpositive_prior_material_cannot_enter_endpoint_proof(self):
        for cell in ({'low_m': [0., 0.], 'high_m': [1., 1., 1.]},
                     {'low_m': [math.nan, 0., 0.], 'high_m': [1., 1., 1.]},
                     {'low_m': [0., 0., 0.], 'high_m': [0., 1., 1.]},
                     {'low_m': [2., 0., 0.], 'high_m': [1., 1., 1.]},
                     {'high_m': [1., 1., 1.]}):
            with self.subTest(cell=cell), self.assertRaises(liquid.LiquidError): vessels._prior_endpoints([cell])

    def test_material_overlap_bbox_is_lossless_without_moving_exact_solid_faces(self):
        lo, hi = np.array([0., -2., 0.]), np.array([1., 2., 1.])
        solid = mf.Manifold.cube([.25, 4., .125]).translate([.375, -2., .4375])
        gap = mf.Manifold.cube([.03125, 4., .125]).translate([.484375, -2., .4375])
        overlap = liquid._valid(solid-gap, 'independent disconnected overlap')
        lower, upper = vessels._contract_overlap(overlap, lo, hi)
        self.assertTrue(np.all(lower >= lo) and np.all(upper <= hi))
        self.assertEqual(lower[0], .375); self.assertEqual(upper[0], .625)
        self.assertEqual(lower[1], -2.); self.assertEqual(upper[1], 2.)
        self.assertTrue(liquid._valid(overlap-liquid._box(lower, upper), 'independent overlap preservation').is_empty())
        discarded = liquid._box(lo, hi)-liquid._box(lower, upper)
        self.assertTrue(liquid._valid(overlap ^ discarded, 'independent discarded empty material').is_empty())
        for raw in ([math.nan, -1., .1, .8, 1., .9], [-.001, -1., .1, .8, 1., .9],
                    [.1, -1., .1, 1.001, 1., .9], [.8, -1., .1, .1, 1., .9],
                    [.5, -1., .1, .5, 1., .9]):
            with self.subTest(raw=raw):
                a, b = vessels._contract_overlap(SimpleNamespace(bounding_box=lambda: raw), lo, hi)
                np.testing.assert_array_equal(a, lo); np.testing.assert_array_equal(b, hi)
        raw = [.5, -1., .1, np.nextafter(.5, math.inf), 1., .9]
        a, b = vessels._contract_overlap(SimpleNamespace(bounding_box=lambda: raw), lo, hi)
        self.assertEqual(a[0], raw[0]); self.assertEqual(b[0], raw[3])
        # Actual retained failure: outward padding -.875 to the next greater
        # double entered a native exclusion. Raw extrema preserve this face.
        raw = [0., -1., 0., 1., -.875, 1.]
        a, b = vessels._contract_overlap(SimpleNamespace(bounding_box=lambda: raw), lo, hi)
        self.assertEqual(b[1], -.875)

    def test_batch_certificates_match_the_unchanged_public_predicate_with_one_classifier_call(self):
        _, _, policy = self.fixture()
        bounds = [([.1, y, .1], [.9, z, .9]) for y, z in
                  ((-2., -1.), (-1., 0.), (0., 1.), (1., 2.), (-1.1, -.9), (-.1, .1),
                   (np.nextafter(-1., -math.inf), -1.), (-1., np.nextafter(-1., math.inf)))]
        for low, high in bounds:
            lo, hi = np.array(low), np.array(high)
            try: expected = tuple(original_certificate(policy, i, lo, hi) for i in range(4))
            except liquid.LiquidError as refused:
                # Adjacent-double geometry may refuse in the unchanged native
                # Boolean predicate. Both wrappers must preserve that refusal.
                for evaluate in (lambda: tuple(policy.certificate(i, lo, hi) for i in range(4)),
                                 lambda: policy._certificates(lo, hi)):
                    with self.assertRaises(liquid.LiquidError) as actual: evaluate()
                    self.assertEqual(str(actual.exception), str(refused))
                continue
            self.assertEqual(tuple(policy.certificate(i, lo, hi) for i in range(4)), expected)
            with patch.object(policy.classifier, 'possible', wraps=policy.classifier.possible) as possible:
                actual = policy._certificates(lo, hi)
                self.assertEqual(possible.call_count, 1)
            self.assertEqual(actual, expected)

    def test_all_definite_certificates_are_inherited_only_by_exact_subsets(self):
        hull, tanks, policy = self.fixture(); definite = []; calls = 0
        original = policy._certificates
        def observed(lo, hi):
            nonlocal calls
            calls += 1
            self.assertFalse(any(np.all(lo >= a) and np.all(hi <= b) and
                                 (np.any(lo > a) or np.any(hi < b)) for a, b in definite))
            result = original(lo, hi)
            if 0 not in result: definite.append((lo.copy(), hi.copy()))
            return result
        with patch.object(policy, '_certificates', side_effect=observed):
            result = vessels._pack_shared('rq4b', hull, tanks, None, dict(max_seconds=20.), policy)
        self.assertGreater(calls, 0); self.assertGreater(len(definite), 0)
        self.assertLess(calls, result['ledger']['screened_cells'])

    def fixture(self, *, restricted=.5, flexible=1.25):
        hull = mf.Manifold.cube([1., 4., 1.]).translate([0., -2., 0.])
        root = mf.Manifold.cube([1., 2., 1.]).translate([0., -1., 0.])
        tanks = [tank(0, flexible, -1), tank(1, flexible, 1),
                 tank(2, restricted, -1), tank(3, restricted, 1)]
        return hull, tanks, vessels.Policy(declaration(), Partition(), root, tanks)

    def run_pack(self, *, limits=None, **options):
        hull, tanks, policy = self.fixture(**options)
        return vessels._pack_shared('rq4b', hull, tanks, None,
                                    dict({'max_seconds': 20.}, **(limits or {})), policy)

    def test_shared_pool_closes_all_nested_demands_with_canonical_restricted_first_slabs(self):
        result = self.run_pack()
        self.assertEqual(result['inferred_vessels_v1']['packing_v1']['method'],
                         'nested_eligible_shared_pool_v1')
        self.assertEqual(result['inferred_vessels_v1']['packing_v1']['assignment_order'], [2, 3, 0, 1])
        self.assertEqual(list(dict.fromkeys(c['tank'] for c in result['cells'])), [2, 3, 0, 1])
        for index, expected in ((0, 1.25), (1, 1.25), (2, .5), (3, .5)):
            cells = [c for c in result['cells'] if c['tank'] == index]
            self.assertAlmostEqual(math.fsum(c['volume_m3'] for c in cells), expected, places=12)
            if index in (2, 3):
                self.assertTrue(all(c['eligibility_v1'] == vessels.ORDINARY for c in cells))
        self.assertFalse(result['ledger']['native_owner_interval_certified'])
        self.assertTrue(result['ledger']['global_aabb_nonoverlap_verified'])

    def test_nested_feasibility_requires_restricted_capacity_even_with_ample_total(self):
        # Each side contains 1 ordinary + 1 root m3. Total demand is only
        # 1.75, but restricted demand 1.25 cannot borrow the 1m3 root.
        with self.assertRaises(liquid.LiquidError):
            self.run_pack(restricted=1.25, flexible=.5,
                          limits=dict(max_screened_cells=6000, max_depth=20))

    def test_nested_feasibility_requires_total_capacity_even_when_restricted_fits(self):
        with self.assertRaises(liquid.LiquidError):
            self.run_pack(restricted=.5, flexible=1.6)

    def test_original_single_tank_resolution_is_not_replaced_by_combined_quota(self):
        result = self.run_pack()
        packing = result['inferred_vessels_v1']['packing_v1']
        self.assertEqual(packing['target_volume_m3_by_tank'],
                         [{'tank': 0, 'target_m3': 1.25/64}, {'tank': 1, 'target_m3': 1.25/64},
                          {'tank': 2, 'target_m3': .5/64}, {'tank': 3, 'target_m3': .5/64}])
        # Every ordinary piece is contained in a pool box that was refined to
        # the smaller restricted target, including pieces assigned to TK0/1.
        self.assertTrue(all(c['volume_m3'] <= (.5 if c['eligibility_v1'] == vessels.ORDINARY else 1.25)/64*(1+1e-12)
                            for c in result['cells']))

    def test_unknown_restricted_membership_cannot_be_admitted_as_flexible_only(self):
        hull, tanks, policy = self.fixture()
        original = policy._certificate
        seen = []
        def certificate(index, lo, hi, possible):
            value = original(index, lo, hi, possible)
            # A narrow ordinary strip is deliberately undecidable for the
            # restricted tank. A flexible true certificate is not a waiver.
            if index == 2 and lo[1] < -1.5 < hi[1]:
                seen.append((lo.copy(), hi.copy()))
                return 0
            return value
        policy._certificate = certificate
        result = vessels._pack_shared('rq4b', hull, tanks, None, dict(max_seconds=20.), policy)
        self.assertTrue(seen)
        for c in result['cells']:
            self.assertFalse(c['low_m'][1] < -1.5 < c['high_m'][1])

    def test_every_emitted_slab_retains_fresh_hull_and_honest_exact_prior_checks(self):
        hull, tanks, policy = self.fixture()
        counts = Counter(); original = liquid._valid
        def audited(solid, context, **options):
            counts[context] += 1
            return original(solid, context, **options)
        with patch.object(liquid, '_valid', side_effect=audited), patch.object(vessels, '_prior_clear', wraps=vessels._prior_clear) as prior:
            result = vessels._pack_shared('rq4b', hull, tanks, None, dict(max_seconds=20.), policy)
        cells = result['cells']
        self.assertEqual(counts['Emitted slab/hull difference'], len(cells))
        self.assertEqual(counts['Emitted slab/prior liquid intersection'], 0)
        self.assertEqual(prior.call_count, sum(c['tank'] != 2 for c in cells))
        for i, cell in enumerate(cells):
            lo, hi = np.array(cell['low_m']), np.array(cell['high_m'])
            self.assertEqual(policy.certificate(cell['tank'], lo, hi), 1)
            for other in cells[:i]:
                self.assertTrue(np.any(np.minimum(hi, other['high_m']) <= np.maximum(lo, other['low_m'])))

    def test_equipment_clearance_and_original_hull_are_not_replaced_by_policy_bounds(self):
        hull, tanks, policy = self.fixture()
        exclusion = dict(low_m=[0., -.875, 0.], high_m=[1., -.75, 1.])
        excluded = liquid._box(np.array(exclusion['low_m']), np.array(exclusion['high_m']))
        result = vessels._pack_shared('rq4b', hull, tanks, excluded, dict(max_seconds=20.), policy,
                                     exclusion_boxes=[exclusion])
        for c in result['cells']:
            self.assertTrue(liquid._clear_of_exclusion_boxes(np.array(c['low_m']), np.array(c['high_m']), [exclusion]))
        smaller = mf.Manifold.cube([.75, 4., 1.]).translate([0., -2., 0.])
        with self.assertRaises(liquid.LiquidError):
            vessels._pack_shared('rq4b', smaller, tanks, None, dict(max_seconds=20.), policy)

    def test_fresh_final_hull_eligibility_and_prior_failures_are_fatal(self):
        for failed in ('hull', 'eligibility', 'prior'):
            with self.subTest(failed=failed):
                hull, tanks, policy = self.fixture()
                original = liquid._valid
                def invalid(solid, context, **options):
                    if ((failed == 'hull' and context == 'Emitted slab/hull difference') or
                            (failed == 'prior' and context == 'Emitted slab/prior liquid intersection')):
                        return original(mf.Manifold.cube([1., 1., 1.]), context)
                    return original(solid, context, **options)
                if failed == 'eligibility':
                    policy.eligibility = lambda *args: (_ for _ in ()).throw(liquid.LiquidError('final eligibility refusal'))
                # Native prior boxes use the complete exact endpoint proof;
                # inject that final audit's refusal directly. Hull Boolean
                # status/volume validation remains independently mandatory.
                guard = patch.object(vessels, '_prior_clear', return_value=False) if failed == 'prior' else patch.object(liquid, '_valid', side_effect=invalid)
                with guard, self.assertRaises(liquid.LiquidError):
                    vessels._pack_shared('rq4b', hull, tanks, None, dict(max_seconds=20.), policy)

    def test_global_count_cell_edge_depth_and_clock_limits_refuse(self):
        for limits in (dict(max_screened_cells=1), dict(max_cells=1),
                       dict(max_assignment_edges=1), dict(max_depth=1), dict(max_seconds=1e-12)):
            with self.subTest(limits=limits), self.assertRaises(liquid.LiquidError):
                self.run_pack(limits=limits)

    def test_storage_order_does_not_change_canonical_payload(self):
        hull, tanks, policy = self.fixture()
        first = vessels._pack_shared('rq4b', hull, tanks, None, dict(max_seconds=20.), policy)
        other = copy.deepcopy(tanks); other.reverse()
        second = vessels._pack_shared('rq4b', hull, other, None, dict(max_seconds=20.), policy)
        self.assertEqual(first['cells'], second['cells'])
        self.assertEqual(first['ledger'], second['ledger'])

    def test_exact_boundary_audit_accepts_faces_but_rejects_one_ulp_positive_overlap(self):
        def box(lo, hi): return dict(low_m=lo, high_m=hi)
        first = box([0., 0., 0.], [1., 1., 1.])
        for second in (box([1., 0., 0.], [2., 1., 1.]),
                       box([1., 1., 0.], [2., 2., 1.]),
                       box([np.nextafter(1., math.inf), 0., 0.], [2., 1., 1.])):
            vessels._audit_disjoint([first, second], lambda: None)
            vessels._audit_disjoint([second, first], lambda: None)
        penetrates = box([np.nextafter(1., -math.inf), 0., 0.], [2., 1., 1.])
        for cells in ([first, penetrates], [penetrates, first]):
            with self.assertRaisesRegex(liquid.LiquidError, 'overlapping interiors'):
                vessels._audit_disjoint(cells, lambda: None)

    def test_disconnected_thin_void_and_non_aabb_exclusion_retain_fresh_material_checks(self):
        hull, tanks, policy = self.fixture()
        gap = mf.Manifold.cube([.03125, 4., 1.]).translate([.484375, -2., 0.])
        hollow = hull-gap
        result = vessels._pack_shared('rq4b', hollow, tanks, None, dict(max_seconds=20.), policy)
        for c in result['cells']:
            slab = liquid._box(np.array(c['low_m']), np.array(c['high_m']))
            self.assertTrue(liquid._valid(slab-hollow, 'independent disconnected hull').is_empty())
        cylinder = mf.Manifold.cylinder(1., .0625, circular_segments=12).translate([.5, -1.5, 0.])
        result = vessels._pack_shared('rq4b', hull, tanks, cylinder, dict(max_seconds=20.), policy)
        for c in result['cells']:
            slab = liquid._box(np.array(c['low_m']), np.array(c['high_m']))
            self.assertTrue(liquid._valid(slab ^ cylinder, 'independent curved exclusion').is_empty())

    def test_fresh_equipment_check_and_global_independent_audit_cannot_be_skipped(self):
        hull, tanks, policy = self.fixture()
        boxes = [dict(low_m=[2., 0., 0.], high_m=[3., 1., 1.])]
        with patch.object(liquid, '_clear_of_exclusion_boxes', return_value=False), self.assertRaises(liquid.LiquidError):
            vessels._pack_shared('rq4b', hull, tanks, None, dict(max_seconds=20.), policy, exclusion_boxes=boxes)
        with patch.object(vessels, '_audit_disjoint', side_effect=liquid.LiquidError('independent audit refusal')) as audit:
            with self.assertRaisesRegex(liquid.LiquidError, 'independent audit refusal'):
                vessels._pack_shared('rq4b', hull, tanks, None, dict(max_seconds=20.), policy)
            audit.assert_called_once()

    def test_bounded_partial_parent_cache_falls_back_without_geometry_or_capacity_change(self):
        hull, tanks, policy = self.fixture()
        gap = mf.Manifold.cube([.03125, 4., 1.]).translate([.484375, -2., 0.])
        hull = hull-gap
        first = vessels._pack_shared('rq4b', hull, tanks, None, dict(max_seconds=20.), policy)
        # These are cache limits in pack; no GLB is loaded in this fixture.
        # Every nonempty partial solid exceeds them, forcing original-domain
        # fallback while preserving the same strict final audits.
        second = vessels._pack_shared('rq4b', hull, tanks, None,
                                      dict(max_seconds=20., max_vertices=1, max_triangles=1), policy)
        self.assertEqual(first['cells'], second['cells'])
        self.assertEqual(first['ledger'], second['ledger'])

    def test_prior_endpoint_arrays_are_immutable_complete_and_reused_per_native_tank(self):
        hull, tanks, policy = self.fixture(); queries = []
        original = vessels._prior_clear
        def observed(lo, hi, bounds):
            self.assertFalse(bounds[0].flags.writeable); self.assertFalse(bounds[1].flags.writeable)
            queries.append((id(bounds[0]), bounds[0].copy(), bounds[1].copy()))
            return original(lo, hi, bounds)
        with patch.object(vessels, '_prior_clear', side_effect=observed):
            result = vessels._pack_shared('rq4b', hull, tanks, None, dict(max_seconds=20.), policy)
        at = 0
        for index, prior_ids in ((3, {2}), (0, {2, 3}), (1, {0, 2, 3})):
            prior = [c for c in result['cells'] if c['tank'] in prior_ids]
            count = sum(c['tank'] == index for c in result['cells'])
            group = queries[at:at+count]; at += count
            self.assertEqual(len(group), count); self.assertEqual(len({row[0] for row in group}), 1)
            for _, lows, highs in group:
                np.testing.assert_array_equal(lows, [c['low_m'] for c in prior])
                np.testing.assert_array_equal(highs, [c['high_m'] for c in prior])
        self.assertEqual(at, len(queries))
        for lower in (-1., 0.):
            bounds = (np.full((1, 3), lower), np.full((1, 3), lower+1.))
            self.assertTrue(original(np.array([lower+1., lower, lower]),
                                     np.array([lower+2., lower+1., lower+1.]), bounds))
        bounds = (np.zeros((1, 3)), np.ones((1, 3)))
        self.assertFalse(original(np.array([np.nextafter(1., -math.inf), 0., 0.]), np.array([2., 1., 1.]), bounds))


if __name__ == '__main__': unittest.main(verbosity=2)
