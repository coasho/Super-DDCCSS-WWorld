"""Actual source-bound RQ Boolean witness and strict partial-proof controls.

Mesh64 export cannot preserve internal CSG history. Root must report whether
the native re-ingestion reproduces the captured failure, rather than assuming.
"""
from fractions import Fraction
import math
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import manifold3d as mf
import numpy as np
from hangar import liquid, liquid_vessels as vessels

# Captured NPZ SHA256 bc89d8ef77cc1200d8bfe5275ce769bf310bd7d3df8e1abb2e72c74fa5f2ca2a.
ACTUAL_CANDIDATE = {'vert_properties': [[-1.3478693196226277, 15.897557809948921, -0.28123281903043285],
                     [-1.3478693196226277, 15.897557809948921, -0.26861846803128575],
                     [-1.3478693196226277, 15.98475695071175, -0.27165987322496954],
                     [-1.3478693196226277, 16.209278762340542, -0.28123281903043273],
                     [-1.3478693196226277, 16.209278762340546, -0.2809034354140212],
                     [-1.3478693196226277, 16.209278762340546, -0.2795340030499943],
                     [-1.3443297041895341, 15.897557809948921, -0.28123281903043285],
                     [-1.3443297041895341, 15.897557809948921, -0.2684251834220449],
                     [-1.3443297041895341, 15.961155741816324, -0.2706434057416337],
                     [-1.3443297041895341, 16.209278762340542, -0.28123281903043273],
                     [-1.3443297041895341, 16.209278762340546, -0.28057405179760964],
                     [-1.3443297041895341, 16.209278762340546, -0.2793452457188345]],
 'tri_verts': [[1, 7, 2],
               [2, 8, 5],
               [2, 7, 8],
               [8, 11, 5],
               [3, 9, 0],
               [6, 0, 9],
               [3, 4, 9],
               [4, 10, 9],
               [2, 3, 0],
               [0, 1, 2],
               [0, 7, 1],
               [2, 4, 3],
               [2, 5, 4],
               [5, 11, 4],
               [0, 6, 7],
               [8, 6, 9],
               [6, 8, 7],
               [4, 11, 10],
               [8, 9, 10],
               [8, 10, 11]],
 'merge_from_vert': [],
 'merge_to_vert': [],
 'run_index': [0,
               12,
               18,
               24,
               60,
               60,
               60,
               60,
               60,
               60,
               60,
               60,
               60,
               60,
               60,
               60,
               60,
               60,
               60,
               60,
               60,
               60,
               60,
               60,
               60,
               60,
               60,
               60,
               60,
               60,
               60,
               60,
               60,
               60,
               60],
 'run_original_id': [1,
                     100935,
                     211604,
                     4661529,
                     2632049,
                     1473087,
                     800949,
                     424955,
                     39945,
                     15669,
                     4727,
                     2991,
                     1347,
                     679,
                     359,
                     206,
                     123,
                     84,
                     2,
                     4,
                     5,
                     6,
                     7,
                     8,
                     9,
                     3,
                     35,
                     25,
                     36,
                     50,
                     42,
                     25,
                     43,
                     49],
 'run_transform': [[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]],
                   [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]],
                   [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]],
                   [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]],
                   [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]],
                   [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]],
                   [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]],
                   [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]],
                   [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]],
                   [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]],
                   [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]],
                   [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]],
                   [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]],
                   [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]],
                   [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]],
                   [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]],
                   [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]],
                   [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]],
                   [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]],
                   [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]],
                   [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]],
                   [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]],
                   [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]],
                   [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]],
                   [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]],
                   [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]],
                   [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]],
                   [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]],
                   [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]],
                   [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]],
                   [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]],
                   [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]],
                   [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]],
                   [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 0.0, 0.0]]],
 'run_flags': [0,
               0,
               0,
               0,
               0,
               0,
               0,
               0,
               0,
               0,
               0,
               0,
               0,
               0,
               0,
               0,
               0,
               0,
               1,
               1,
               1,
               1,
               1,
               1,
               1,
               1,
               0,
               0,
               0,
               0,
               0,
               0,
               0,
               0],
 'face_id': [70300, 70630, 70300, 70630, 0, 0, 8, 8, 10, 10, 4, 10, 10, 8, 4, 6, 6, 8, 6, 6],
 'halfedge_tangent': []}
ACTUAL_SHAPES = {'vert_properties': [12, 3], 'tri_verts': [20, 3], 'merge_from_vert': [0], 'merge_to_vert': [0], 'run_index': [35], 'run_original_id': [34], 'run_transform': [34, 4, 3], 'run_flags': [34], 'face_id': [20], 'halfedge_tangent': [0, 3, 4]}
ACTUAL_LO = [-1.3478693196226277, 15.897557809948921, -0.28123281903043285]
ACTUAL_HI = [-1.3443297041895341, 16.209278762340546, -0.28057405179760964]


def actual_candidate():
    types = dict(vert_properties=np.float64, tri_verts=np.uint64,
                 merge_from_vert=np.uint64, merge_to_vert=np.uint64,
                 run_index=np.uint64, run_original_id=np.uint32,
                 run_transform=np.float64, run_flags=np.uint8, face_id=np.uint64,
                 halfedge_tangent=np.float64)
    arrays = {k: np.ascontiguousarray(np.asarray(v, dtype=types[k]).reshape(ACTUAL_SHAPES[k]))
              for k,v in ACTUAL_CANDIDATE.items()}
    return liquid._valid(mf.Manifold(mf.Mesh64(**arrays)), 'Captured original candidate', nonempty=True)


class PartialSupportTests(unittest.TestCase):
    def test_actual_native_rq_partial_cell_reaches_refinement_without_invalid_difference(self):
        candidate = actual_candidate(); lo, hi = np.array(ACTUAL_LO), np.array(ACTUAL_HI)
        box = liquid._box(lo, hi)
        overlap = liquid._valid(box ^ candidate, 'Captured cell/interior intersection', nonempty=True)
        # Existing native positive/intersection admission remains authoritative.
        # On the original producer this call must retain its original strict
        # negative-volume error if the exported re-ingestion reproduces it.
        contained, mesh = vessels._cell_contained(box, candidate, overlap, lo, hi)
        self.assertFalse(contained)
        self.assertGreater(overlap.volume(), 0.)

    def test_actual_exact_support_proof_precedes_difference_and_retains_intersection(self):
        candidate = actual_candidate(); lo, hi = np.array(ACTUAL_LO), np.array(ACTUAL_HI)
        box = liquid._box(lo, hi); overlap = liquid._valid(box ^ candidate, 'Witness overlap', nonempty=True)
        self.assertTrue(vessels._strict_partial_box(candidate, lo, hi))
        original = liquid._valid
        def strict(solid, context, **kwargs):
            if context == 'Cell/interior difference': raise AssertionError('Already-proven partial cell repeated invalid Boolean')
            return original(solid, context, **kwargs)
        before = overlap.to_mesh64().vert_properties.copy()
        with patch.object(liquid, '_valid', side_effect=strict):
            self.assertFalse(vessels._cell_contained(box, candidate, overlap, lo, hi)[0])
        np.testing.assert_array_equal(overlap.to_mesh64().vert_properties, before)

    def test_equal_boundary_contact_and_contained_boxes_are_never_partial(self):
        lo, hi = np.zeros(3), np.ones(3); box = liquid._box(lo, hi)
        self.assertFalse(vessels._strict_partial_box(box, lo, hi))
        self.assertTrue(vessels._cell_contained(box, box, box, lo, hi)[0])
        outer = liquid._box(-np.ones(3), np.full(3, 2.))
        self.assertFalse(vessels._strict_partial_box(outer, lo, hi))
        self.assertTrue(vessels._cell_contained(box, outer, liquid._valid(box ^ outer, 'Contained control'), lo, hi)[0])

    def test_one_ulp_partial_plane_is_not_erased_by_float_epsilon(self):
        lo, hi = np.zeros(3), np.ones(3)
        candidate = liquid._box(lo, np.array([math.nextafter(1., -math.inf), 1., 1.]))
        self.assertTrue(vessels._strict_partial_box(candidate, lo, hi))

    def test_float_ranking_cannot_certify_a_nonsupporting_plane(self):
        # An interior triangle can have outside box corners, but the outer
        # cube vertices on both sides prevent a global supporting-plane proof.
        p = np.array([[x,y,z] for x in (-1.,2.) for y in (-1.,2.) for z in (-1.,2.)]
                     + [[.5,0.,0.],[.5,1.,0.],[.5,0.,1.]])
        mesh = SimpleNamespace(vert_properties=p,tri_verts=np.array([[8,9,10]],dtype=np.uint64))
        solid = SimpleNamespace(num_vert=lambda:len(p),num_tri=lambda:1,to_mesh64=lambda:mesh)
        self.assertFalse(vessels._strict_partial_box(solid,np.zeros(3),np.ones(3)))

    def test_unknown_proof_keeps_original_hard_negative_difference_refusal(self):
        candidate = actual_candidate(); lo, hi = np.array(ACTUAL_LO), np.array(ACTUAL_HI)
        box = liquid._box(lo, hi); overlap = liquid._valid(box ^ candidate, 'Unknown-proof overlap', nonempty=True)
        original = liquid._valid
        def strict(solid, context, **kwargs):
            if context == 'Cell/interior difference': raise liquid.LiquidError('retained strict negative refusal')
            return original(solid, context, **kwargs)
        with patch.object(vessels,'_strict_partial_box',return_value=False), patch.object(liquid,'_valid',side_effect=strict):
            with self.assertRaisesRegex(liquid.LiquidError,'retained strict negative refusal'):
                vessels._cell_contained(box,candidate,overlap,lo,hi)

    def test_deadline_and_work_caps_do_not_become_geometric_admission(self):
        def expired(): raise liquid.LiquidError('unchanged deadline')
        large = SimpleNamespace(num_vert=lambda:65,num_tri=lambda:20,
                                to_mesh64=lambda: self.fail('Work cap exported an oversized mesh'))
        self.assertFalse(vessels._strict_partial_box(large,np.zeros(3),np.ones(3)))
        with self.assertRaisesRegex(liquid.LiquidError,'unchanged deadline'):
            vessels._strict_partial_box(large,np.zeros(3),np.ones(3),expired)
        candidate=actual_candidate(); checks=[]
        self.assertTrue(vessels._strict_partial_box(candidate,np.array(ACTUAL_LO),np.array(ACTUAL_HI),lambda:checks.append(1)))
        self.assertGreater(len(checks),1)

    def test_invalid_or_degenerate_mesh_cannot_certify_partial(self):
        for p,t in [(np.array([[float('nan'),0,0]]*3),np.array([[0,1,2]],dtype=np.uint64)),
                    (np.zeros((3,3)),np.array([[0,1,2]],dtype=np.uint64)),
                    (np.zeros((3,3)),np.array([[0,1,9]],dtype=np.uint64))]:
            mesh=SimpleNamespace(vert_properties=p,tri_verts=t)
            solid=SimpleNamespace(num_vert=lambda:3,num_tri=lambda:1,to_mesh64=lambda:mesh)
            self.assertFalse(vessels._strict_partial_box(solid,np.zeros(3),np.ones(3)))




# Current helper-call capture: NPZ SHA 4d9a98ba1bd347b51510462b264d6d00c0a393e4d2e5238a34a7033457b827f1.
# This is a captured-array routing regression, not kernel history re-ingestion.
CURRENT_HELPER_SOLIDS = {'candidate': {'vert_properties': [[-1.3584881659219088, 15.897557809948925, -0.28123281903043285],
                                   [-1.3584881659219088, 15.897557809948921, -0.2785141216514379],
                                   [-1.3584881659219088, 15.897557809948921, -0.26919832185900827],
                                   [-1.3584881659219088, 16.053418286144733, -0.2812328190304328],
                                   [-1.3584881659219088, 16.053418286144733, -0.27463455503338424],
                                   [-1.3581668736009966, 16.053418286144733, -0.2746170105114232],
                                   [-1.3549485504888152, 15.897557809948923, -0.28123281903043285],
                                   [-1.3549485504888152, 15.897557809948921, -0.27987347034093535],
                                   [-1.3549485504888152, 15.897557809948921, -0.26900503724976743],
                                   [-1.3549485504888152, 15.975488048046827, -0.28123281903043285],
                                   [-1.3549485504888152, 16.053418286144733, -0.2812328190304328],
                                   [-1.3549485504888152, 16.031959368502605, -0.2736928081916413],
                                   [-1.3549485504888152, 16.053418286144733, -0.27444538675936647]],
               'tri_verts': [[4, 2, 8],
                             [4, 11, 5],
                             [4, 8, 11],
                             [12, 5, 11],
                             [3, 9, 0],
                             [3, 10, 9],
                             [1, 0, 7],
                             [7, 0, 6],
                             [1, 3, 0],
                             [1, 7, 2],
                             [1, 2, 4],
                             [1, 4, 3],
                             [3, 4, 5],
                             [3, 5, 12],
                             [0, 9, 6],
                             [7, 6, 9],
                             [7, 9, 8],
                             [7, 8, 2],
                             [3, 12, 10],
                             [9, 10, 8],
                             [10, 12, 11],
                             [10, 11, 8]]},
 'intersection': {'vert_properties': [[-1.3584881659219088, 15.897557809948925, -0.28123281903043285],
                                      [-1.3584881659219088, 15.897557809948921, -0.27851412165143785],
                                      [-1.3584881659219088, 16.05341828614473, -0.2812328190304328],
                                      [-1.3584881659219088, 16.053418286144733, -0.27851412165143785],
                                      [-1.3549485504888152, 15.897557809948923, -0.28123281903043285],
                                      [-1.3549485504888152, 15.897557809948921, -0.27987347034093535],
                                      [-1.3549485504888152, 15.897557809948921, -0.27851412165143785],
                                      [-1.3561284222998464, 16.001464794079464, -0.28123281903043285],
                                      [-1.3549485504888152, 15.975488048046827, -0.28123281903043285],
                                      [-1.3549485504888152, 16.053418286144733, -0.2812328190304328],
                                      [-1.3549485504888152, 16.053418286144733, -0.27851412165143785]],
                  'tri_verts': [[2, 7, 0],
                                [2, 9, 7],
                                [8, 7, 9],
                                [1, 0, 5],
                                [0, 4, 5],
                                [1, 2, 0],
                                [3, 9, 2],
                                [1, 3, 2],
                                [1, 10, 3],
                                [0, 8, 4],
                                [0, 7, 8],
                                [1, 5, 6],
                                [1, 6, 10],
                                [5, 4, 10],
                                [6, 5, 10],
                                [10, 4, 8],
                                [10, 8, 9],
                                [3, 10, 9]]}}
CURRENT_HELPER_LOW = [-1.3584881659219088, 15.897557809948921, -0.28123281903043285]
CURRENT_HELPER_HIGH = [-1.3549485504888152, 16.053418286144733, -0.27851412165143785]

class CurrentOverlapPartialTests(unittest.TestCase):
    class Solid:
        def __init__(self, name): self.name=name
        def num_vert(self): return len(CURRENT_HELPER_SOLIDS[self.name]['vert_properties'])
        def num_tri(self): return len(CURRENT_HELPER_SOLIDS[self.name]['tri_verts'])
        def volume(self): return 5.1929869485188e-6 if self.name=='candidate' else 1.4998676817462426e-6
        def to_mesh64(self):
            return SimpleNamespace(vert_properties=np.array(CURRENT_HELPER_SOLIDS[self.name]['vert_properties'],float),
                                   tri_verts=np.array(CURRENT_HELPER_SOLIDS[self.name]['tri_verts'],np.uint64))
    class Box:
        def __init__(self): self.subtractions=0
        def __sub__(self, other):
            self.subtractions+=1
            # Replay the ACTUAL already-materialized difference's status/volume.
            # No kernel Boolean is performed by this unit adapter.
            return SimpleNamespace(status=lambda:mf.Error.NoError,volume=lambda:-2.**-69)
    def inputs(self):
        return self.Box(),self.Solid('candidate'),self.Solid('intersection'),np.array(CURRENT_HELPER_LOW),np.array(CURRENT_HELPER_HIGH)
    def test_current_overlap_proves_partial_without_unnecessary_failed_difference(self):
        box,candidate,overlap,lo,hi=self.inputs()
        self.assertFalse(vessels._strict_partial_box(candidate,lo,hi))
        self.assertTrue(vessels._strict_partial_box(overlap,lo,hi))
        contained,mesh=vessels._cell_contained(box,candidate,overlap,lo,hi)
        self.assertFalse(contained)
        self.assertIsNone(mesh)
        self.assertEqual(box.subtractions,0)
    def test_unproved_cells_keep_the_original_negative_volume_refusal(self):
        box,candidate,overlap,lo,hi=self.inputs()
        with patch.object(vessels,'_strict_partial_box',return_value=False):
            with self.assertRaisesRegex(liquid.LiquidError,'Cell/interior difference: nonfinite or negative solid volume'):
                vessels._cell_contained(box,candidate,overlap,lo,hi)
        self.assertEqual(box.subtractions,1)

    def test_invalid_overlap_mesh_cannot_bypass_strict_difference_validation(self):
        box,candidate,overlap,lo,hi=self.inputs()
        overlap.to_mesh64=lambda: SimpleNamespace(vert_properties=np.full((3,3),float('nan')),
                                                 tri_verts=np.array([[0,1,2]],np.uint64))
        with self.assertRaisesRegex(liquid.LiquidError,'Cell/interior difference: nonfinite or negative solid volume'):
            vessels._cell_contained(box,candidate,overlap,lo,hi)
        self.assertEqual(box.subtractions,1)


if __name__=='__main__': unittest.main(verbosity=2)
