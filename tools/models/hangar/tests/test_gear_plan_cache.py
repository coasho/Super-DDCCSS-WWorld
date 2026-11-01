"""Manufactured gear plans exercise reuse without native geometry or CFD."""
import contextlib
from contextvars import copy_context
import copy
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from hangar.geometry.aircraft import Gear
from hangar.shape import gear, meshkit
from tests.paths import TEMP_ROOT


class GearPlanCache(unittest.TestCase):
    def setUp(self):
        self.probes = self.closed = self.fits = self.bays = 0
        self.dependencies = {'source': b'fixed source', 'threads': 1}
        fixture = self

        class Probe:
            def __init__(self, scene):
                fixture.probes += 1

            def __enter__(self):
                return self

            def __exit__(self, *args):
                fixture.closed += 1

        def fit(leg, probe):
            fixture.fits += 1
            leg.set_stow([0., 1., .1], 84., 12.)
            leg.fitted = True

        def bay(leg, probe, solid, foils):
            fixture.bays += 1
            return dict(leg=leg, cut={'solid': solid, 'foils': foils},
                        doors=[dict(hinge=leg.hinge, axis=leg.swing_axis,
                                    scene={'root': solid, 'foils': foils})],
                        cover=None, protrusion=.01, allowed=.0)

        for target, value in ((meshkit, Probe),):
            self.enterContext(patch.object(target, 'Probe', value))
        self.enterContext(patch.object(gear.Leg, 'fit', fit))
        self.enterContext(patch.object(gear, '_bay', bay))
        self.enterContext(patch.object(gear, '_cache_dependencies',
                                     side_effect=lambda: self.dependencies, create=True))

    @staticmethod
    def aircraft(name='fixture', mirror=True):
        spec = dict(name='Main', position=[0., 1., 0.], attach=[0., 1., 1.2],
                    retractable=True, mirror=mirror, metadata={'nested': [1., 2.]})
        return SimpleNamespace(name=name, gear=[Gear(spec)])

    @staticmethod
    def scene():
        return {'prim': 'ellipsoid', 'r': np.array([1., 2., 3.])}, [{'xy': np.array([[0., 0.], [1., .1]])}]

    def reuse(self, **kwargs):
        # The old implementation still runs the manufactured cold calculation;
        # RED must demonstrate repeated work, rather than fail at a missing API.
        return getattr(gear, '_bay_plan_reuse', lambda **kw: contextlib.nullcontext())(**kwargs)

    def call(self, aircraft=None, scene=None):
        solid, foils = self.scene() if scene is None else scene
        return gear.bays(self.aircraft() if aircraft is None else aircraft, solid, foils)

    def test_equivalent_content_reuses_one_complete_native_plan(self):
        with self.reuse():
            first = self.call(self.aircraft('one'))
            second = self.call(self.aircraft('another model identity'))
        self.assertEqual((self.probes, self.closed, self.fits, self.bays), (1, 1, 1, 2))
        self.assertEqual([p['leg'].name for p in first], ['Left Main', 'Right Main'])
        for a, b in zip(first, second):
            self.assertTrue(a['leg'].fitted and b['leg'].fitted)
            np.testing.assert_array_equal(a['leg'].R, b['leg'].R)
        np.testing.assert_array_equal(first[0]['leg'].swing_axis,
                                      -np.diag([1., -1., 1.]) @ first[1]['leg'].swing_axis)

    def test_cold_and_warm_results_are_detached_with_internal_aliases(self):
        aircraft = self.aircraft()
        scene = self.scene()
        with self.reuse():
            first = self.call(aircraft, scene)
            baseline = copy.deepcopy(first)
            self.assertIs(first[0]['doors'][0]['hinge'], first[0]['leg'].hinge)
            first[0]['leg'].hinge[:] = 900.
            first[0]['leg'].R[:] = 700.
            first[0]['leg'].spec['metadata']['nested'][0] = 600.
            first[0]['cut']['solid']['r'][:] = 500.
            first[0]['cut']['foils'][0]['xy'][:] = 400.
            self.assertEqual(aircraft.gear[0].spec['metadata']['nested'][0], 1.)
            np.testing.assert_array_equal(scene[0]['r'], [1., 2., 3.])
            second = self.call(aircraft, scene)
            self.assertIs(second[0]['doors'][0]['hinge'], second[0]['leg'].hinge)
            self.assertIs(second[0]['cut']['solid'], second[1]['cut']['solid'])
            np.testing.assert_array_equal(second[0]['leg'].R, baseline[0]['leg'].R)
            np.testing.assert_array_equal(second[0]['cut']['solid']['r'], [1., 2., 3.])
            second[0]['leg'].hinge[:] = 300.
            third = self.call(aircraft, scene)
            np.testing.assert_array_equal(third[0]['leg'].hinge, baseline[0]['leg'].hinge)
        self.assertEqual(self.probes, 1)

    def test_each_complete_input_and_dependency_change_misses(self):
        with self.reuse():
            self.call()
            solid, foils = self.scene()
            solid['r'][0] = np.nextafter(solid['r'][0], 2.)
            self.call(scene=(solid, foils))
            solid, foils = self.scene()
            foils[0]['xy'][1, 1] = np.nextafter(.1, 1.)
            self.call(scene=(solid, foils))
            aircraft = self.aircraft()
            aircraft.gear[0].spec['metadata']['nested'][0] = 3.
            self.call(aircraft)
            aircraft = self.aircraft()
            aircraft.gear[0].wheel_width += .01
            self.call(aircraft)
            self.dependencies['source'] = b'changed source'
            self.call()
            self.dependencies['threads'] = 2
            self.call()
        self.assertEqual(self.probes, 7)

    def test_order_alias_structure_and_exact_array_dtype_are_content(self):
        first = self.aircraft(mirror=False)
        second = self.aircraft(mirror=False)
        second.gear[0].name = 'Other'
        both = SimpleNamespace(gear=first.gear+second.gear)
        with self.reuse():
            self.call(both)
            self.call(SimpleNamespace(gear=list(reversed(both.gear))))
            solid, foils = self.scene()
            solid['r'] = solid['r'].astype(np.float32)
            self.call(scene=(solid, foils))
            solid, foils = self.scene()
            solid['alias'] = solid['r']
            self.call(scene=(solid, foils))
            solid['alias'] = solid['r'].copy()
            self.call(scene=(solid, foils))
        self.assertEqual(self.probes, 5)

    def test_ordinary_callers_remain_uncached_and_request_cleanup_is_finally(self):
        self.call(); self.call()
        with self.assertRaisesRegex(RuntimeError, 'request failed'):
            with self.reuse():
                self.call(); self.call()
                raise RuntimeError('request failed')
        with self.reuse():
            self.call(); self.call()
        self.call()
        self.assertEqual(self.probes, 5)

    def test_nested_context_restores_outer_cache(self):
        with self.reuse():
            self.call()
            with self.reuse():
                self.call(); self.call()
            self.call()
        self.assertEqual(self.probes, 2)

    def test_failures_and_partial_plans_are_not_retained(self):
        with self.reuse():
            with patch.object(gear, '_bay', side_effect=RuntimeError('manufactured bay failure')):
                with self.assertRaisesRegex(RuntimeError, 'manufactured bay failure'):
                    self.call()
            self.call(); self.call()
            self.dependencies['source'] = b'partial branch'
            with patch.object(gear, '_bay', return_value={'leg': None}):
                self.assertEqual(self.call(), [{'leg': None}, {'leg': None}])
                self.assertEqual(self.call(), [{'leg': None}, {'leg': None}])
            self.call(); self.call()
        self.assertEqual(self.probes, 5)
        self.assertEqual(self.closed, self.probes)

    def test_entry_limit_lru_and_oversized_success_are_uncached(self):
        with self.reuse(max_entries=2) as cache:
            for value in (1., 2., 1., 3., 2.):
                solid, foils = self.scene(); solid['extra'] = value
                self.call(scene=(solid, foils))
            if cache is not None:
                self.assertLessEqual(len(cache.entries), 2)
                self.assertLessEqual(cache.retained_bytes, 64*1024*1024)
        self.assertEqual(self.probes, 4)
        before = self.probes
        with self.reuse(max_bytes=1):
            self.call(); self.call()
        self.assertEqual(self.probes-before, 2)

    def test_unsupported_and_cyclic_graphs_bypass_without_copying_or_native_changes(self):
        cases = [object(), np.array([object()], dtype=object), float('nan')]
        cycle = []; cycle.append(cycle); cases.append(cycle)
        for unsupported in cases:
            with self.subTest(kind=type(unsupported).__name__), self.reuse():
                solid, foils = self.scene(); solid['unsupported'] = unsupported
                before = self.probes
                self.call(scene=(solid, foils)); self.call(scene=(solid, foils))
                self.assertEqual(self.probes-before, 2)

    def test_empty_or_fixed_gear_never_constructs_probe(self):
        aircraft = self.aircraft(); aircraft.gear[0].retractable = False
        with self.reuse():
            self.assertEqual(self.call(aircraft), [])
            self.assertEqual(self.call(SimpleNamespace(gear=[])), [])
        self.assertEqual(self.probes, 0)

    def test_cleanup_releases_all_retained_entries_after_error(self):
        with self.assertRaisesRegex(RuntimeError, 'request failed'):
            with self.reuse() as cache:
                self.call()
                self.assertEqual(len(cache.entries), 1)
                self.assertGreater(cache.retained_bytes, 0)
                raise RuntimeError('request failed')
        self.assertEqual(len(cache.entries), 0)
        self.assertEqual(cache.retained_bytes, 0)

    def test_copied_context_cannot_repopulate_a_finished_request(self):
        with self.reuse() as cache:
            self.call()
            stale = copy_context()
        stale.run(self.call)
        self.assertTrue(cache.closed)
        self.assertFalse(cache.entries)
        self.assertEqual(cache.retained_bytes, 0)
        self.assertEqual(self.probes, 2)

    def test_limits_cannot_expand_production_retention(self):
        for kwargs in (dict(max_entries=5), dict(max_entries=True), dict(max_entries=0),
                       dict(max_bytes=64*1024*1024+1), dict(max_bytes=0), dict(max_bytes=True)):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                with self.reuse(**kwargs):
                    pass

    def test_exact_bit_keys_keep_signed_zero_and_array_shapes_distinct(self):
        with self.reuse():
            for value in (0., -0.):
                solid, foils = self.scene(); solid['signed'] = value
                self.call(scene=(solid, foils))
        self.assertEqual(self.probes, 2)
        for _ in range(8):
            self.assertNotEqual(gear._plan_bytes([np.arange(6).reshape(2, 3), np.arange(6).reshape(3, 2)]),
                                gear._plan_bytes([np.arange(6).reshape(2, 3), np.arange(6).reshape(2, 3)]))

    def test_overlapping_distinct_array_views_safely_bypass(self):
        solid, foils = self.scene()
        solid['view'] = solid['r'][1:]
        with self.reuse():
            self.call(scene=(solid, foils)); self.call(scene=(solid, foils))
        self.assertEqual(self.probes, 2)

    def test_input_changed_during_cold_calculation_cannot_seed_old_key(self):
        original = gear._bay
        solid, foils = self.scene()

        def mutate(*args):
            row = original(*args)
            solid['r'][0] += .125
            return row

        with self.reuse() as cache:
            with patch.object(gear, '_bay', mutate):
                self.call(scene=(solid, foils))
            self.assertFalse(cache.entries)
            self.call(scene=(solid, foils)); self.call(scene=(solid, foils))
        self.assertEqual(self.probes, 2)





if __name__ == '__main__':
    unittest.main()
