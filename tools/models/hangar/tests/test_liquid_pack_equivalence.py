"""Search equivalence to the frozen pre-optimization pack, plus geometry oracles.

The frozen function is a search reference under the same unchanged geometry
helpers. It is not an alternative production packer or a performance tier.
"""
from collections import Counter
import copy
import hashlib
import json
import math
import sys
import time
import unittest
from unittest.mock import patch
from types import SimpleNamespace

import manifold3d as mf
import numpy as np
from hangar import liquid

_FROZEN_ORIGIN = 'c4ef2d09d3c9b3e5ff3a758b0c08d079d20c4ee35cf3d79da915080c33d11601'
_FROZEN_PACK = r'''def pack(design, hull, tanks, *, excluded=None, limits=None, authority=None, occupied=None,
         owner_certificate=None, candidate_bounds=None, necessary_region=None, exclusion_boxes=None, occupied_boxes=None):
    """Produce capacity-conserving disjoint cells; raise on incomplete geometry.

    Box partitions prove nonoverlap by construction. Every accepted full box and
    every emitted clipped slab additionally receives a Boolean containment check.
    Nearest-anchor assignment is an explicitly estimated internal distribution.
    """
    limits = _limits(limits); mf = _mf(); deadline = time.monotonic()+limits['max_seconds']
    _valid(hull, 'Structural hull', nonempty=True)
    tanks = [dict(t, anchor_body_m=_vec(t['anchor_body_m'], 'Tank anchor').tolist()) for t in tanks]
    seen = set()
    for t in tanks:
        if type(t['index']) is not int or t['index'] < 0 or t['index'] in seen:
            raise LiquidError('Invalid or duplicate native tank index')
        seen.add(t['index'])
        if any(isinstance(t[k], bool) or not isinstance(t[k], numbers.Real) or
               not math.isfinite(t[k]) or t[k] <= 0 for k in ('capacity_kg', 'density_kg_m3')):
            raise LiquidError('Finite positive capacity and physical density required')
    tanks.sort(key=lambda t: t['index'])
    if not tanks: raise LiquidError('pack requires liquid tanks; no-liquid admission uses produce')
    volumes = np.array([t['capacity_kg']/t['density_kg_m3'] for t in tanks])
    if not np.isfinite(volumes).all() or np.any(volumes <= 0): raise LiquidError('Invalid physical liquid volume')
    required = math.fsum(volumes)
    available = _valid(hull ^ authority, 'Owner/original exterior clip') if authority is not None else hull
    if excluded is not None: available = _valid(available-excluded, 'Available interior excluding equipment')
    if occupied is not None: available = _valid(available-occupied, 'Available interior excluding prior actual liquid cells')
    if available.volume()+_tolerance(required, 1) < required:
        raise LiquidError('insufficient available interior volume: %.12g < %.12g m3' % (available.volume(), required))
    bounds = np.array(available.bounding_box()); low = bounds[:3]; high = bounds[3:]
    anchors = np.array([t['anchor_body_m'] for t in tanks])
    target_volume = required/max(64, 16*len(tanks)); max_edge = max(high-low)/8
    if necessary_region is not None:
        # A union of valid analytic necessary boxes, never inferred cap solids.
        # Save the original grid criterion above; final whole-hull containment
        # and singleton owner certificates remain independently mandatory.
        available = _valid(available ^ necessary_region(low, high), 'Original hull/necessary classifier region clip')
        if available.volume()+_tolerance(required, 1) < required:
            raise LiquidError('insufficient physical volume in necessary native-owner region')
        bounds = np.array(available.bounding_box()); low, high = bounds[:3], bounds[3:]
    if candidate_bounds is not None:
        low, high = candidate_bounds(low, high)
        if np.any(low >= high): raise LiquidError('insufficient possible native-owner bounding volume')
    pending = []; sequence = 0

    def push(lo, hi, depth):
        nonlocal sequence
        distance = float(np.min(np.sum(np.maximum(np.maximum(lo-anchors, anchors-hi), 0.)**2, axis=1)))
        ownership = owner_certificate(lo, hi) if owner_certificate is not None else 1
        # Breadth first is mandatory: certified but hull-straddling cells must
        # not refine forever ahead of a coarser indeterminate owner region.
        # Within one depth, prefer proven owners and cheap impossible discards.
        heapq.heappush(pending, (depth, int(ownership == 0), distance, sequence,
                               tuple(lo), tuple(hi), ownership)); sequence += 1

    push(low, high, 0); pool = []; accepted_volume = 0.; screened = 0
    initial_upper_volume = math.prod(high-low); upper_volume = initial_upper_volume
    if initial_upper_volume+_tolerance(initial_upper_volume, 1) < required:
        raise LiquidError('insufficient native-owner candidate volume upper bound')
    def discard_volume(volume):
        nonlocal upper_volume
        upper_volume = math.fsum((upper_volume, -volume))
        # This is only an impossibility bound, never a capacity admission.
        if upper_volume+_tolerance(initial_upper_volume, screened) < required:
            raise LiquidError('insufficient native-owner certified volume upper bound')
    def split(lo, hi, depth):
        axis = int(np.argmax(hi-lo)); midpoint = float((lo[axis]+hi[axis])*.5)
        if depth >= limits['max_depth'] or hi[axis]-lo[axis] <= limits['min_extent_m'] or midpoint in (lo[axis], hi[axis]):
            raise LiquidError('Liquid spatial refinement budget exhausted before capacity closure')
        left = hi.copy(); left[axis] = midpoint; right = lo.copy(); right[axis] = midpoint
        push(lo, left, depth+1); push(right, hi, depth+1)
    while pending and accepted_volume < required:
        if screened >= limits['max_screened_cells'] or time.monotonic() > deadline:
            raise LiquidError('Liquid geometry screening budget exceeded')
        depth, _, _, _, lo, hi, ownership = heapq.heappop(pending); lo = np.array(lo); hi = np.array(hi)
        volume = math.prod(hi-lo)
        if ownership == -1: screened += 1; discard_volume(volume); continue
        if ownership == 0:
            # It cannot be emitted yet. Refining its exact partition enclosure
            # needs no expensive physical hull Boolean; its whole box remains
            # in the conservative unexplored-volume upper bound.
            screened += 1; split(lo, hi, depth); continue
        box = _box(lo, hi); screened += 1
        overlap = _valid(box ^ available, 'Cell/interior intersection')
        if overlap.is_empty(): discard_volume(volume); continue
        contained = _valid(box-available, 'Cell/interior difference').is_empty()
        if contained and ownership == 1 and volume <= target_volume*(1+1e-12) and max(hi-lo) <= max_edge*(1+1e-12):
            pool.append([lo, hi]); accepted_volume = math.fsum([accepted_volume, volume])
            if len(pool) > limits['max_cells']: raise LiquidError('Liquid cell budget exceeded')
        else:
            split(lo, hi, depth)
    if accepted_volume+_tolerance(required, len(pool)) < required:
        raise LiquidError('insufficient certified cell volume')
    if len(pool)*len(tanks) > limits['max_assignment_edges']:
        raise LiquidError('Liquid nearest-anchor assignment budget exceeded')
    centres = np.array([(lo+hi)*.5 for lo, hi in pool])
    orders = [np.argsort(np.sum((centres-anchor)**2, axis=1), kind='stable') for anchor in anchors]
    positions = [0]*len(tanks); remaining = volumes.copy(); assignment = []; allocated_volumes = [[] for _ in tanks]

    def candidate(k):
        while positions[k] < len(pool):
            i = int(orders[k][positions[k]]); positions[k] += 1
            lo, hi = pool[i]
            if np.all(hi > lo):
                heapq.heappush(assignment, (float(np.sum((centres[i]-anchors[k])**2)), k, i)); return
        if remaining[k] > _tolerance(volumes[k], len(pool)):
            raise LiquidError('insufficient unowned cells for native tank')

    for k in range(len(tanks)): candidate(k)
    cells = []
    while assignment:
        if time.monotonic() > deadline: raise LiquidError('Liquid assignment time budget exceeded')
        _, k, i = heapq.heappop(assignment); lo, hi = pool[i]
        if remaining[k] <= 0: continue
        if np.any(hi <= lo): candidate(k); continue
        lo = lo.copy(); hi = hi.copy(); free = math.prod(hi-lo)
        take = min(float(remaining[k]), free); allocated_low = lo.copy(); allocated_high = hi.copy()
        if take < free:
            axis = int(np.argmax(hi-lo)); cross = math.prod(np.delete(hi-lo, axis)); width = take/cross
            if anchors[k, axis] <= centres[i, axis]:
                allocated_high[axis] = lo[axis]+width; pool[i][0] = allocated_high.copy()
                pool[i][0][np.arange(3) != axis] = lo[np.arange(3) != axis]
            else:
                allocated_low[axis] = hi[axis]-width; pool[i][1] = allocated_low.copy()
                pool[i][1][np.arange(3) != axis] = hi[np.arange(3) != axis]
        else: pool[i][0] = hi.copy()
        slab = _box(allocated_low, allocated_high)
        if owner_certificate is not None and owner_certificate(allocated_low, allocated_high) != 1:
            raise LiquidError('Emitted liquid slab has uncertified native owner membership')
        if not _valid(slab-hull, 'Emitted slab/hull difference').is_empty():
            raise LiquidError('Emitted liquid slab escapes structural boundary')
        if authority is not None and not _valid(slab-authority, 'Emitted slab/original exterior difference').is_empty():
            raise LiquidError('Emitted liquid slab escapes original authoritative exterior')
        if occupied is not None:
            intersection = _valid(slab ^ occupied, 'Emitted slab/prior liquid intersection')
            # Manifold can retain an exactly coplanar zero-volume contact as
            # nonempty geometry. For the per-TK box path, original endpoints
            # give a stronger exact positive-interior test, without tolerance.
            disjoint = (_clear_of_exclusion_boxes(allocated_low, allocated_high, occupied_boxes)
                        if occupied_boxes is not None else intersection.is_empty())
            if not disjoint: raise LiquidError('Emitted liquid slab overlaps prior actual liquid cells')
        if exclusion_boxes is not None:
            if not _clear_of_exclusion_boxes(allocated_low, allocated_high, exclusion_boxes):
                raise LiquidError('Emitted liquid slab has positive interior overlap with occupied equipment box')
        elif excluded is not None and not _valid(slab ^ excluded, 'Emitted slab/exclusion intersection').is_empty():
            raise LiquidError('Emitted liquid slab intersects occupied equipment')
        actual = math.prod(allocated_high-allocated_low)
        cells.append(dict(id=len(cells), tank=tanks[k]['index'], section=tanks[k].get('section', 'airframe'), low_m=allocated_low.tolist(),
                          high_m=allocated_high.tolist(), volume_m3=actual))
        if len(cells) > limits['max_cells']: raise LiquidError('Emitted liquid cell budget exceeded')
        allocated_volumes[k].append(actual)
        residual = volumes[k]-math.fsum(allocated_volumes[k])
        # Use the already-declared final ledger's binary64 roundoff budget;
        # never construct an unrepresentable zero-width residual slab.
        remaining[k] = 0. if abs(residual) <= _tolerance(volumes[k], len(allocated_volumes[k])) else max(0., residual)
        if remaining[k] > 0: candidate(k)
    checks = []
    for t, expected in zip(tanks, volumes):
        owned = [c for c in cells if c['tank'] == t['index']]
        actual = math.fsum(c['volume_m3'] for c in owned); tolerance = _tolerance(expected, len(owned))
        if not owned or abs(actual-expected) > tolerance:
            raise LiquidError('Liquid tank capacity ledger failed for native tank '+str(t['index']))
        checks.append(dict(tank=t['index'], required_m3=float(expected), actual_m3=actual,
                           error_m3=actual-expected, tolerance_m3=tolerance))
    return dict(schema=SCHEMA, design=design, frame=FRAME, sources=[], tanks=tanks, cells=cells,
                fluid_policy_v1=dict(FLUID_POLICY_V1),
                ledger=dict(capacity_conserved=True, containment_certified=True, exclusions_certified=True,
                            disjoint_certified=True, disjoint_basis='recursive box partition and consumed clipped slabs',
                            screened_cells=screened, certified_pool_cells=len(pool),
                            assignment_edges=len(pool)*len(tanks),
                            available_interior_m3=available.volume(), tanks=checks),
                uncertainty=dict(distribution='estimated capacity-constrained nearest-anchor occupancy; not OEM tank layout',
                                 boundary='delivered neutral polygonal GLB, not exact smooth CAD',
                                 maximum_cell_diagonal_m=max(float(np.linalg.norm(np.array(c['high_m'])-c['low_m'])) for c in cells),
                                 dynamics='full-capacity geometry; runtime contents/free surface and slosh are separate'),
                limits=limits)


'''
_reference_namespace = dict(liquid.__dict__)
exec(compile(_FROZEN_PACK, '<frozen-pre-optimization-pack>', 'exec'), _reference_namespace)
reference_pack = _reference_namespace['pack']


def tank(index, volume, anchor):
    return dict(index=index, capacity_kg=800.*volume, density_kg_m3=800., anchor_body_m=list(anchor))


def cube(size, low=(0., 0., 0.)):
    return mf.Manifold.cube(size).translate(low)


def endpoint_box(low, high):
    # Independent direct endpoints avoid translated-cube lo+(hi-lo) rounding.
    unit = mf.Manifold.cube().to_mesh64()
    vertices = np.where(np.asarray(unit.vert_properties)[:, :3] == 0, low, high)
    return mf.Manifold(mf.Mesh64(np.array(vertices, dtype=np.float64, order='C', copy=True),
                                np.array(unit.tri_verts, dtype=np.uint64, order='C', copy=True)))


def observed(function, design, hull, tanks, *, certificate=None, reject_final=False, clock=None, **kwargs):
    counts = Counter(); trace = []; original = liquid._valid
    def valid(solid, context, **options):
        counts[context] += 1
        if reject_final and context == 'Emitted slab/hull difference':
            return cube((1., 1., 1.))
        return original(solid, context, **options)
    def owner(lo, hi):
        result = certificate(lo, hi) if certificate else 1
        trace.append(([float(x).hex() for x in lo], [float(x).hex() for x in hi], result))
        return result
    started = time.perf_counter(); cpu = time.process_time()
    result = None; refusal = None
    replacements = {'_valid': valid}
    if clock is not None: replacements['time'] = SimpleNamespace(monotonic=clock)
    private = {}; previous_profile = sys.getprofile()
    def profile(frame, event, value):
        if event == 'return' and frame.f_code is function.__code__:
            private.update(copy.deepcopy(frame.f_locals.get('_cache_metrics', {})))
    with patch.dict(liquid.__dict__, replacements), patch.dict(function.__globals__, replacements):
        sys.setprofile(profile)
        try:
            result = function(design, hull, copy.deepcopy(tanks), owner_certificate=owner, **kwargs)
        except liquid.LiquidError as error:
            refusal = str(error)
        finally:
            sys.setprofile(previous_profile)
    return dict(result=result, refusal=refusal, counts=counts, owner_trace=trace,
                cache_metrics=private, elapsed_s=time.perf_counter()-started, cpu_s=time.process_time()-cpu)


class LiquidPackEquivalenceTests(unittest.TestCase):
    def compare(self, name, hull, tanks, *, certificate=None, reject_final=False, **options):
        old = observed(reference_pack, name, hull, tanks, certificate=certificate, reject_final=reject_final, **options)
        new = observed(liquid.pack, name, hull, tanks, certificate=certificate, reject_final=reject_final, **options)
        if old['refusal'] == 'Liquid geometry screening budget exceeded':
            self.assertTrue(new['refusal'].startswith(old['refusal']))
        else:
            self.assertEqual(old['refusal'], new['refusal'])
        self.assertEqual(json.dumps(old['result'], sort_keys=True, separators=(',', ':'), allow_nan=False),
                         json.dumps(new['result'], sort_keys=True, separators=(',', ':'), allow_nan=False))
        self.assertEqual(old['owner_trace'], new['owner_trace'])
        self.assertEqual({k:v for k,v in old['counts'].items() if k.startswith('Emitted ')},
                         {k:v for k,v in new['counts'].items() if k.startswith('Emitted ')})
        print(json.dumps(dict(case=name, reference_origin=_FROZEN_ORIGIN,
            reference_function_sha256=hashlib.sha256(_FROZEN_PACK.encode()).hexdigest(),
            old_counts=old['counts'], new_counts=new['counts'],
            old_elapsed_s=old['elapsed_s'], new_elapsed_s=new['elapsed_s'],
            old_cpu_s=old['cpu_s'], new_cpu_s=new['cpu_s'], refusal=new['refusal'],
            cache_metrics=new['cache_metrics'],
            cell_count=len(new['result']['cells']) if new['result'] else 0), sort_keys=True), flush=True)
        if new['result']:
            cells = new['result']['cells']; bounds = []
            self.assertEqual(new['counts']['Emitted slab/hull difference'], len(cells))
            for cell in cells:
                low, high = np.array(cell['low_m']), np.array(cell['high_m'])
                box = endpoint_box(low, high)
                difference = box-hull
                self.assertEqual(difference.status(), mf.Error.NoError); self.assertTrue(difference.is_empty())
                for key in ('excluded', 'occupied'):
                    if options.get(key) is not None:
                        overlap = box ^ options[key]
                        self.assertEqual(overlap.status(), mf.Error.NoError); self.assertTrue(overlap.is_empty())
                if options.get('authority') is not None:
                    self.assertTrue((box-options['authority']).is_empty())
                for a, b in bounds:
                    self.assertTrue(np.any(np.minimum(high, b) <= np.maximum(low, a)))
                bounds.append((low, high))
            for row in tanks:
                volume = math.fsum(c['volume_m3'] for c in cells if c['tank'] == row['index'])
                self.assertAlmostEqual(volume, row['capacity_kg']/row['density_kg_m3'], places=11)
        return old, new

    def test_contained_boxes_need_no_redundant_intersection(self):
        old, new = self.compare('contained-intersection', cube((1.,1.,1.)), [tank(0,.5,(.25,.5,.5))])
        self.assertLess(new['counts']['Cell/interior intersection'], old['counts']['Cell/interior intersection'])

    def test_contained_parent_children_reuse_only_physical_membership(self):
        old, new = self.compare('contained-parent', cube((1.,1.,1.)), [tank(0,.5,(.25,.5,.5))])
        self.assertLess(new['counts']['Cell/interior difference'], old['counts']['Cell/interior difference'])

    def test_cavity_exclusions_authority_prior_occupancy_and_partial_slabs(self):
        hull = cube((3.,3.,3.))
        excluded = cube((.4,.4,3.), (1.3,1.3,0.))
        occupied = cube((.25,3.,3.), (0.,0.,0.))
        self.compare('cavity-authority-occupied', hull,
            [tank(0,1.31,(.7,.5,.5)), tank(1,2.17,(2.8,2.5,2.5))],
            authority=hull, excluded=excluded, occupied=occupied)

    def test_disconnected_real_gap_and_shared_boundary(self):
        for gap in (0., 2.**-26, .5):
            with self.subTest(gap=gap):
                hull = cube((1.,1.,1.))+cube((1.,1.,1.), (1.+gap,0.,0.))
                _, new = self.compare('gap-'+float(gap).hex(), hull, [tank(0,1.2,(.2,.5,.5))])
                if gap:
                    for c in new['result']['cells']:
                        self.assertTrue(c['high_m'][0] <= 1. or c['low_m'][0] >= 1.+gap)

    def test_thin_and_tilted_source_preserves_exact_outputs(self):
        angle=.31; c,s=math.cos(angle),math.sin(angle)
        for tilted in (False,True):
            with self.subTest(tilted=tilted):
                hull=cube((5.,.8,.06))
                if tilted: hull=hull.transform([[c,-s,0.,0.],[s,c,0.,0.],[0.,0.,1.,0.]])
                self.compare('thin-'+str(tilted), hull, [tank(0,.08,(2.,.8,.03))])

    def test_mixed_unknown_owner_keeps_every_certificate_call_and_search_order(self):
        def certificate(lo,hi):
            return -1 if hi[0] <= .3 else 1 if lo[0] >= .3 else 0
        self.compare('mixed-owner', cube((1.,1.,1.)), [tank(0,.3,(.8,.5,.5))], certificate=certificate)

    def test_partial_parent_domains_are_shared_reused_and_reclaimed(self):
        hull = cube((2.,2.,2.))-cube((.4,.4,2.),(.8,.8,0.))
        _, new = self.compare('partial-domain-reuse', hull, [tank(0,2.,(.4,.4,.4))])
        metrics = new['cache_metrics']
        self.assertGreater(metrics.get('admitted',0),0)
        self.assertGreater(metrics.get('reused',0),0)
        self.assertGreater(metrics.get('reclaimed',0),0)
        self.assertEqual(metrics.get('vertices'),0)
        self.assertEqual(metrics.get('triangles'),0)
        self.assertEqual(metrics.get('records'),0)
        self.assertLessEqual(metrics['peak_vertices'],liquid.DEFAULT_LIMITS['max_vertices'])
        self.assertLessEqual(metrics['peak_triangles'],liquid.DEFAULT_LIMITS['max_triangles'])
        self.assertLessEqual(metrics['peak_records'],liquid.DEFAULT_LIMITS['max_screened_cells'])

    def test_tiny_cache_falls_back_to_original_available_deterministically(self):
        hull = cube((2.,2.,2.))-cube((.4,.4,2.),(.8,.8,0.))
        for limits in ({'max_vertices':1},{'max_triangles':1}):
            with self.subTest(limits=limits):
                _, new = self.compare('tiny-cache-'+next(iter(limits)), hull,
                    [tank(0,2.,(.4,.4,.4))], limits=limits)
                metrics = new['cache_metrics']
                self.assertGreater(metrics.get('fallback',0),0)
                self.assertEqual(metrics.get('admitted'),0)
                self.assertEqual(metrics.get('peak_records'),0)
                again = observed(liquid.pack,'tiny-cache-'+next(iter(limits)),hull,
                    [tank(0,2.,(.4,.4,.4))],limits=limits)
                self.assertEqual(new['result'],again['result'])
                self.assertEqual(metrics,again['cache_metrics'])

    def test_near_coplanar_adjacent_double_sliver_and_storage_order(self):
        for boundary in (math.nextafter(.5,0.),.5,math.nextafter(.5,1.)):
            for reverse in (False,True):
                with self.subTest(boundary=boundary,reverse=reverse):
                    # Adjacent represented cavity planes, a real thin arm and
                    # reversed Boolean input storage. No gap/plane tolerance.
                    pieces = [cube((1.,1.,1.)),cube((2.**-18,1.,1.),(1.,0.,0.))]
                    if reverse: pieces.reverse()
                    hull = mf.Manifold.batch_boolean(pieces,mf.OpType.Add)
                    excluded = endpoint_box(np.array([boundary,.4,0.]),np.array([.75,.6,1.]))
                    self.compare('adjacent-sliver-'+boundary.hex()+'-'+str(reverse),hull,
                        [tank(0,.31,(.2,.2,.2))],excluded=excluded)

    def test_cache_pressure_mixes_reuse_and_original_domain_fallback(self):
        hull=cube((2.,2.,2.))-cube((.4,.4,2.),(.8,.8,0.))
        limits={'max_vertices':64,'max_triangles':96}
        _,new=self.compare('mixed-cache-pressure',hull,[tank(0,2.,(.4,.4,.4))],limits=limits)
        metrics=new['cache_metrics']
        self.assertGreater(metrics['admitted'],0)
        self.assertGreater(metrics['reused'],0)
        self.assertGreater(metrics['fallback'],0)
        self.assertEqual(metrics['admitted'],metrics['reclaimed'])
        self.assertEqual(metrics['records'],0)
        self.assertLessEqual(metrics['peak_vertices'],limits['max_vertices'])
        self.assertLessEqual(metrics['peak_triangles'],limits['max_triangles'])
        again=observed(liquid.pack,'mixed-cache-pressure',hull,[tank(0,2.,(.4,.4,.4))],limits=limits)
        self.assertEqual(new['result'],again['result'])
        self.assertEqual(metrics,again['cache_metrics'])

    def test_partial_ancestor_domain_survives_unknown_owner_refinement(self):
        hull = cube((2.,2.,2.))-cube((.4,.4,2.),(.8,.8,0.))
        def certificate(lo,hi):
            # Conservative uncertainty in intermediate-sized boxes is valid:
            # every eventual positive certificate belongs to the same owner.
            width=max(hi-lo)
            return 0 if .5 < width <= 1. else 1
        _, new = self.compare('partial-domain-unknown-owner',hull,
            [tank(0,2.,(.4,.4,.4))],certificate=certificate)
        self.assertGreater(new['cache_metrics'].get('unknown_reused',0),0)

    def test_small_cache_refusal_keeps_original_count_and_certificate_trace(self):
        hull=cube((2.,2.,2.))-cube((.4,.4,2.),(.8,.8,0.))
        _,new=self.compare('domain-cache-count-refusal',hull,[tank(0,2.,(.4,.4,.4))],
            limits={'max_screened_cells':8,'max_vertices':1,'max_triangles':1})
        self.assertIn('reason=count',new['refusal'])
        self.assertGreater(new['cache_metrics'].get('fallback',0),0)

    def test_invalid_and_exhausted_inputs_refuse_identically(self):
        rows = [dict(limits={'max_screened_cells':1}), dict(limits={'max_cells':1}),
                dict(limits={'max_depth':1}), dict(limits={'unknown':1}),
                dict(certificate=lambda lo,hi:0), dict(certificate=lambda lo,hi:-1)]
        for index, options in enumerate(rows):
            with self.subTest(index=index):
                if index==4: options['limits']={'max_screened_cells':8}
                old,new=self.compare('refusal-'+str(index),cube((1.,1.,1.)),[tank(0,.5,(.5,.5,.5))],**options)
                self.assertIsNotNone(old['refusal']);self.assertIsNotNone(new['refusal'])
        for row in (tank(0,2.,(.5,.5,.5)), dict(tank(0,.5,(.5,.5,.5)),density_kg_m3=0.)):
            old,new=self.compare('invalid-capacity-density',cube((1.,1.,1.)),[row])
            self.assertIsNotNone(old['refusal']);self.assertIsNotNone(new['refusal'])

    def test_final_actual_slab_audit_cannot_be_inherited_or_bypassed(self):
        old,new=self.compare('fresh-final-audit',cube((1.,1.,1.)),[tank(0,.5,(.5,.5,.5))],reject_final=True)
        self.assertEqual(new['refusal'],'Emitted liquid slab escapes structural boundary')
        self.assertEqual(old['counts']['Emitted slab/hull difference'],1)
        self.assertEqual(new['counts']['Emitted slab/hull difference'],1)

    def test_count_diagnostic_keeps_native_tank_and_short_circuits_clock(self):
        class Clock:
            calls=0
            def __call__(self): self.calls+=1;return 10.
        old_clock,new_clock=Clock(),Clock()
        old=observed(reference_pack,'count-clock',cube((1.,1.,1.)),[tank(2,.5,(.5,.5,.5))],
                     clock=old_clock,limits={'max_screened_cells':1})
        new=observed(liquid.pack,'count-clock',cube((1.,1.,1.)),[tank(2,.5,(.5,.5,.5))],
                     clock=new_clock,limits={'max_screened_cells':1})
        self.assertEqual(old_clock.calls,2);self.assertEqual(new_clock.calls,old_clock.calls)
        self.assertEqual(old['refusal'],'Liquid geometry screening budget exceeded')
        self.assertIn('reason=count',new['refusal']);self.assertIn('tanks=2',new['refusal'])
        self.assertIn('clock=not-sampled-count-short-circuit',new['refusal'])

    def test_deadline_diagnostic_keeps_the_original_strict_clock_condition(self):
        class Clock:
            calls=0
            def __call__(self): self.calls+=1;return 10. if self.calls==1 else 12.
        old_clock,new_clock=Clock(),Clock()
        old=observed(reference_pack,'deadline-clock',cube((1.,1.,1.)),[tank(0,.5,(.5,.5,.5))],
                     clock=old_clock,limits={'max_seconds':1.})
        new=observed(liquid.pack,'deadline-clock',cube((1.,1.,1.)),[tank(0,.5,(.5,.5,.5))],
                     clock=new_clock,limits={'max_seconds':1.})
        self.assertEqual(old_clock.calls,2);self.assertEqual(new_clock.calls,old_clock.calls)
        self.assertEqual(old['refusal'],'Liquid geometry screening budget exceeded')
        self.assertIn('reason=time',new['refusal']);self.assertIn('tanks=0',new['refusal'])
        self.assertIn('remaining_clock_s=-1',new['refusal'])
        class BoundaryClock:
            calls=0
            def __call__(self): self.calls+=1;return 10. if self.calls==1 else 11.
        old_clock,new_clock=BoundaryClock(),BoundaryClock()
        old=observed(reference_pack,'exact-deadline',cube((1.,1.,1.)),[tank(0,.5,(.5,.5,.5))],
                     clock=old_clock,limits={'max_seconds':1.,'max_screened_cells':1})
        new=observed(liquid.pack,'exact-deadline',cube((1.,1.,1.)),[tank(0,.5,(.5,.5,.5))],
                     clock=new_clock,limits={'max_seconds':1.,'max_screened_cells':1})
        self.assertEqual(old_clock.calls,2);self.assertEqual(new_clock.calls,old_clock.calls)
        self.assertEqual(old['refusal'],'Liquid geometry screening budget exceeded')
        self.assertIn('reason=count',new['refusal'])  # Equality did not expire time.


if __name__=='__main__': unittest.main()
