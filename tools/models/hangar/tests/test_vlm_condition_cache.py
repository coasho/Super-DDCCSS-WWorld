"""Wake/condition caches must not change a solver result with traversal order."""
import itertools
import math
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

from hangar.aero.vlm import VLM


def operators():
    model=VLM.__new__(VLM)
    model.wake=np.array([-1.,1.]);model._current=None
    model._sets=[]
    for value in (1.,3.):
        block={k:np.full((3,2,2),value) for k in ('dec_v','dec_w','S_dec_v','S_dec_w')}
        block.update({k:np.array([value]) for k in ('g_v','g_w','W_strip','x_ac')})
        block['g_ctrl']=[];model._sets.append(block)
    model.select(0.)
    return model


class ConditionCache(unittest.TestCase):
    def test_distinct_nearby_wakes_never_alias(self):
        model=operators();model.select(5e-10)
        self.assertEqual(model._current,5e-10)
        self.assertFalse(np.array_equal(model.dec_v,model._sets[0]['dec_v']*.5+model._sets[1]['dec_v']*.5))

    def test_cached_response_is_exact_and_invalidates_flow_rate_and_wake(self):
        model=operators()
        for flow,rate,wake in [([1.,0,0],[0.,0,0],0.),([1.,.1,0],[0.,0,0],0.),
                               ([1.,.1,0],[0.,.2,0],0.),([1.,.1,0],[0.,.2,0],5e-10)]:
            v,w=np.array(flow),np.array(rate);model.select(wake)
            expected_strip=np.tensordot(v,model.S_dec_v,1)+np.tensordot(w,model.S_dec_w,1)
            expected_panel=np.tensordot(v,model.dec_v,1)+np.tensordot(w,model.dec_w,1)
            with patch('hangar.aero.vlm.np.tensordot',wraps=np.tensordot) as contracted:
                np.testing.assert_array_equal(model.decamber_response(v,w),expected_strip)
                np.testing.assert_array_equal(model.decamber_panels(v,w),expected_panel)
                np.testing.assert_array_equal(model.decamber_response(v,w),expected_strip)
                np.testing.assert_array_equal(model.decamber_panels(v,w),expected_panel)
                self.assertEqual(contracted.call_count,4)

    def test_nonlinear_six_axis_results_are_exact_under_reversed_traversal(self):
        from hangar import pipeline
        from hangar.aero.model import AeroModel
        root=Path(__file__).resolve().parents[4]
        aircraft=pipeline.Design(str(root/'aircraft/c172/c172.toml'),log=lambda *args:None).aircraft
        model=AeroModel(aircraft,density=.2,wake_deg=(-10.,0.,10.))
        cases=list(itertools.product((-math.pi,-.1,0.,.2,math.pi/2,math.pi),(-math.pi/2,-.1,0.,.1,math.pi/2)))
        keys=('CD','CY','CL','Cl','Cm','Cn','CDi')
        first={case:np.array([value[k] for k in keys]) for case in cases for value in [model.evaluate(*case,controls={'elevator':.1})]}
        for case in reversed(cases):
            value=model.evaluate(*case,controls={'elevator':.1})
            np.testing.assert_array_equal([value[k] for k in keys],first[case],err_msg=str(case))

    def test_original_wake_formula_is_preserved_outside_principal_angles(self):
        from hangar import pipeline
        from hangar.aero.model import AeroModel, wind_axes
        root=Path(__file__).resolve().parents[4]
        aircraft=pipeline.Design(str(root/'aircraft/c172/c172.toml'),log=lambda *args:None).aircraft
        model=AeroModel(aircraft,density=.2,wake_deg=(-10.,0.,10.))
        for alpha,beta in [(2*math.pi,0.),(math.pi,math.pi)]:
            model.evaluate(alpha,beta)
            direction=wind_axes(alpha,beta)[0]
            expected=float(np.clip(math.atan2(direction[2],max(direction[0],1e-6)),model.vlm.wake[0],model.vlm.wake[-1]))
            self.assertEqual(model.vlm._current,expected)


if __name__=='__main__':unittest.main()
