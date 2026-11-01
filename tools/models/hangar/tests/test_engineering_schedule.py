"""Scheduling may change cost, never cohort authority or the declared job set."""
import unittest

from hangar import engineering


class GenerationScheduling(unittest.TestCase):
    def test_cost_order_preserves_priority_and_all_jobs(self):
        records=[dict(name='slow',category='fixed-wing'),dict(name='fast',category='fixed-wing'),
                 dict(name='last',category='rotorcraft')]
        profile=dict(schema='fsim.sim-m02.scheduling-profile.v1',generation_wall_seconds={'slow':10.,'fast':1.,'last':100.})
        result=engineering.generation_order(records,profile)
        self.assertEqual([r['name'] for r in result],['slow','fast','last'])
        self.assertEqual({id(r) for r in result},{id(r) for r in records})
        self.assertEqual([r['name'] for r in engineering.generation_order(records)],['fast','slow','last'])

    def test_real_roster_keeps_every_design_and_the_aam_priority(self):
        records=engineering.roster()
        profile=dict(schema='fsim.sim-m02.scheduling-profile.v1',generation_wall_seconds={r['name']:i+1 for i,r in enumerate(records)})
        result=engineering.generation_order(records,profile)
        self.assertEqual(len(result),88)
        self.assertEqual({r['name'] for r in result},{r['name'] for r in records})
        order={'aam':0,'fixed-wing':1,'stores':2,'rotorcraft':3}
        priority=[order[engineering.cohort(r)] for r in result]
        self.assertEqual(priority,sorted(priority))

    def test_missing_extra_nonfinite_or_nonpositive_costs_are_rejected(self):
        records=[dict(name='item',category='fixed-wing')]
        for costs in ({},{'item':1,'extra':2},{'item':0},{'item':-1},{'item':True},
                      {'item':float('inf')},{'item':float('nan')}):
            with self.subTest(costs=costs),self.assertRaises(ValueError):
                engineering.generation_order(records,dict(schema='fsim.sim-m02.scheduling-profile.v1',generation_wall_seconds=costs))
