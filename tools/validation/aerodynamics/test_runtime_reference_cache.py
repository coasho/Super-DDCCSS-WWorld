"""Exact-storage/reference optimization checks, not physical-model tests."""
import importlib.util
import json
import math
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
spec=importlib.util.spec_from_file_location('runtime_reference',ROOT/'tools/models/hangar/hangar/engineering_runtime.py')
runtime=importlib.util.module_from_spec(spec);spec.loader.exec_module(runtime)

def fixture(native):
    coefficients=(*runtime.COEFFS,'CDi')
    matrix=lambda k:[[.013*(k+1)+.017*i-.009*j+.003*i*j for j in range(3)] for i in range(3)]
    control=lambda ch,offset:dict(deflection=[-.3,0.,.3],channel=ch,lo=-.3,hi=.3,values={key:matrix(k+offset) for k,key in enumerate(coefficients)})
    controls={ch:dict(deflection=[-20.,0.,20.],**{key:matrix(k+2) for k,key in enumerate(coefficients)}) for ch in ('elevator','aileron')}
    mach={key:[.8,1.,1.3] for key in ('K_L','K_Y','dCm_dCL','K_elevator','K_aileron','dCD0','induced_factor')};mach['mach']=[0.,.5,2.]
    table=dict(alpha=[-10.,0.,20.],beta=[-30.,0.,30.],base={key:matrix(k) for k,key in enumerate(coefficients)},mach=mach,controls=controls,native_controls=[control('aileron',1),control('elevator',2)] if native else [],rates={rate:{key:[.001*(k+1),.002*(k+1),.003*(k+1)] for k,key in enumerate(coefficients)} for rate in ('p','q','r')},alphadot={key:.004*(k+1) for k,key in enumerate(runtime.COEFFS)},ground_effect=dict(h_b=[0.,.5,2.],lift=[1.3,1.1,1.],drag=[.6,.9,1.]))
    state=dict(alpha=math.radians(7),beta=math.radians(-12),mach=1.1,hb=.35,p_hat=.12,q_hat=-.07,r_hat=.05,adot_hat=-.02)
    return table,state,dict(elevator=4.5,aileron=-.7),{0:-.12,1:.21}

class RuntimeReferenceCache(unittest.TestCase):
    def test_bilinear_boundaries_preserve_clamping_and_mixed_term(self):
        a=[0.,2.];b=[0.,4.];data=[[1.,5.],[7.,19.]]
        self.assertEqual(runtime.interp2(a,b,data,1.,2.),8.)
        self.assertEqual(runtime.interp2(a,b,data,-3.,8.),5.)
        self.assertEqual(runtime.interp2(a,b,data,8.,-2.),7.)
        self.assertEqual(runtime.interp2(a,b,data,2.,4.),19.)
    def test_preparation_is_exact_read_only_and_does_not_mutate_input(self):
        source=dict(grid=[-0.,0.,1.,-2.],matrix=[[1.,2.],[3.,4.]],meta=[{'name':'one','flag':True}],identities=[2**53+1])
        before=json.dumps(source);prepared=runtime.prepare_reference_tables(source)
        self.assertEqual(json.dumps(source),before)
        self.assertEqual(prepared['grid'].tobytes(),np.asarray(source['grid'],dtype=np.float64).tobytes())
        self.assertFalse(prepared['grid'].flags.writeable);self.assertFalse(prepared['matrix'].flags.writeable)
        self.assertEqual(prepared['meta'],source['meta'])
        self.assertEqual(prepared['identities'],source['identities'])
        self.assertIs(runtime.prepare_reference_tables(prepared)['grid'],prepared['grid'])
    def test_reference_retains_frozen_pre_optimization_values(self):
        for native,expected in GOLDEN.items():
            t,state,controls,pieces=fixture(native=='native')
            for tables in (t,runtime.prepare_reference_tables(t)):
                got=runtime.reference(tables,state,controls,pieces)
                self.assertEqual({k:float(v).hex() for k,v in got.items()},expected)
    def test_mach_factors_are_reused_only_within_the_current_state(self):
        table,state,controls,pieces=fixture(True);t=runtime.prepare_reference_tables(table)
        original=runtime.np.interp
        for mach in (1.1,.2):
            state['mach']=mach;calls=[]
            def interp(x,xp,fp):
                if xp is t['mach']['mach']:calls.append(id(fp))
                return original(x,xp,fp)
            with patch.object(runtime.np,'interp',side_effect=interp):runtime.reference(t,state,controls,pieces)
            self.assertEqual(len(calls),7);self.assertEqual(len(set(calls)),7)

# Frozen from pre-optimization source SHA256 c896c544903d84e37426877b6d36f1292e1f9c83f277a44f42ca57c0d6d6ceec.
GOLDEN={'legacy': {'CD': '0x1.3dc8cf1e8df5fp+0', 'CY': '0x1.a925b36dd61c5p-3', 'CL': '0x1.0c4d77126c4f7p-2', 'Cl': '0x1.2e62c44569c1ap-2', 'Cm': '0x1.947e16b147605p-2', 'Cn': '0x1.8832aed3e8750p-2'}, 'native': {'CD': '0x1.3a42e5bffa36ep+0', 'CY': '0x1.8a1455f85a195p-3', 'CL': '0x1.f98990af5c9c0p-3', 'Cl': '0x1.1eda158aabc02p-2', 'Cm': '0x1.84f567f6895edp-2', 'Cn': '0x1.78aa00192a738p-2'}}

if __name__=='__main__':unittest.main()
