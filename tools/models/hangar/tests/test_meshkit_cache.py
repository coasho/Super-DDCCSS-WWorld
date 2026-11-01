"""Exact immutable-scene query reuse, including eviction and output isolation."""
import unittest
import threading
from unittest import mock

import numpy as np

from hangar.shape import meshkit


class _Library:
    def __init__(self):self.points=0;self.closed=0
    def mk_scene_new(self,*args):return 1
    def mk_scene_free(self,*args):self.closed+=1
    def mk_scene_eval(self,handle,pointer,n,distance,material):
        self.points+=n
        p=np.ctypeslib.as_array(pointer,shape=(n*3,)).reshape(n,3)
        # Keep signed-zero behavior observable as well as material IDs.
        np.ctypeslib.as_array(distance,shape=(n,))[:]=p[:,0]
        np.ctypeslib.as_array(material,shape=(n,))[:]=np.signbit(p[:,0]).astype(np.int32)


class ExactSceneQueries(unittest.TestCase):
    def probe(self,lib,capacity):
        with mock.patch.object(meshkit,'library',return_value=lib):
            return meshkit.Probe({},cache_capacity=capacity)

    def test_repeated_and_reordered_points_preserve_every_output_bit(self):
        lib=_Library();plain=_Library()
        points=np.array([[0.,1.,2.],[-0.,1.,2.],[3.,4.,5.],[3.,4.,5.]])
        with self.probe(lib,16) as cached,self.probe(plain,0) as baseline:
            for order in ([0,1,2,3],[3,1,0,2],[1,1,0,3]):
                a=cached(points[order]);b=baseline(points[order])
                for x,y in zip(a,b):self.assertEqual(x.tobytes(),y.tobytes())
            self.assertEqual(lib.points,3)
            self.assertEqual(plain.points,12)
            self.assertEqual(cached._stats['cache_hits'],9)

    def test_eviction_is_bounded_and_never_changes_results(self):
        lib=_Library()
        with self.probe(lib,2) as probe:
            for index in range(12):
                x=np.array([[float(index),0.,0.],[float(index+1),0.,0.]])
                np.testing.assert_array_equal(probe(x)[0],x[:,0])
                self.assertLessEqual(len(probe._cache),2)
        self.assertFalse(probe._cache)
        self.assertEqual(lib.closed,1)

    def test_mutating_inputs_or_outputs_cannot_poison_a_later_query(self):
        lib=_Library();points=np.array([[2.,3.,4.]])
        with self.probe(lib,2) as probe:
            distance,material=probe(points);distance[:]=99;material[:]=99
            self.assertEqual(probe(points)[0][0],2.)
            points[0,0]=3.
            self.assertEqual(probe(points)[0][0],3.)
            self.assertEqual(lib.points,2)

    def test_large_scans_bypass_and_scenes_do_not_share_cached_values(self):
        lib=_Library();points=np.zeros((4097,3))
        with self.probe(lib,2) as first,self.probe(lib,2) as second:
            first(points);self.assertFalse(first._cache)
            first(points[:1]);second(points[:1])
            self.assertEqual(lib.points,4099)

    def test_capacity_is_explicit_and_finite(self):
        for value in (-1,65537,True,1.5):
            with self.subTest(value=value),self.assertRaises(ValueError):meshkit.Probe({},cache_capacity=value)

    def test_close_waits_for_native_evaluation_and_is_idempotent(self):
        entered=threading.Event();release=threading.Event();closed=threading.Event();errors=[]
        class Blocking(_Library):
            def mk_scene_eval(self,*args):
                entered.set()
                if not release.wait(2):raise RuntimeError('Test native callback was not released')
                super().mk_scene_eval(*args)
        lib=Blocking();probe=self.probe(lib,2)
        def query():
            try:probe([[1.,2.,3.]])
            except Exception as error:errors.append(error)
        def close():probe.close();closed.set()
        worker=threading.Thread(target=query);closer=threading.Thread(target=close)
        try:
            worker.start();self.assertTrue(entered.wait(2));closer.start()
            self.assertFalse(closed.wait(.02));self.assertEqual(lib.closed,0)
        finally:
            release.set();worker.join(2)
            if closer.ident is not None:closer.join(2)
            probe.close()
        self.assertFalse(worker.is_alive());self.assertFalse(closer.is_alive())
        self.assertFalse(errors);self.assertEqual(lib.closed,1)
        with self.assertRaises(meshkit.MeshError):probe([[1.,2.,3.]])

    def test_native_scene_reordering_is_bit_exact_with_and_without_reuse(self):
        self.assertIsNotNone(meshkit.library(),'Native geometry backend is required for the engineering gate')
        scene={'root':{'op':'union','k':.1,'children':[
            {'prim':'box','centre':[0.,0.,0.],'half':[1.,2.,3.],'material':2},
            {'prim':'box','centre':[1.,0.,0.],'half':[1.,1.,1.],'material':3}]}}
        # Use the public shape constructor's operator schema.
        from hangar.shape import airframe as sh
        scene['root']=sh.union(scene['root']['children'],.1)
        points=np.random.default_rng(811).normal(size=(100,3))
        points=np.vstack([points,points[::3],np.zeros((4,3))])
        with meshkit.Probe(scene,cache_capacity=256) as cached,meshkit.Probe(scene,cache_capacity=0) as plain:
            for order in (np.arange(len(points)),np.arange(len(points))[::-1]):
                for actual,expected in zip(cached(points[order]),plain(points[order])):
                    self.assertEqual(actual.tobytes(),expected.tobytes())
