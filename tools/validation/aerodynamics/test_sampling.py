"""Sampling must execute the generated native numerical configuration."""
import importlib.util
import math
from pathlib import Path
import unittest

SOURCE=Path(__file__).resolve().parent/'engineering_sdk.py'
spec=importlib.util.spec_from_file_location('sampling_integration_contract',SOURCE)
sampling=importlib.util.module_from_spec(spec);spec.loader.exec_module(sampling)


class Model:
    def __init__(self,dt):self.values={'simulation/integrator/rate/rotational':1.,'simulation/dt':dt}
    def get_property(self,key):return self.values[key]
    def set_property(self,key,value):self.values[key]=value


class SamplingIntegration(unittest.TestCase):
    def setUp(self):self.receipt={'integration':{'rotational_rate_method':4,'maximum_internal_dt_s':1/8160}}

    def test_raw_reset_euler_is_restored_before_sampling_without_changing_timestep(self):
        model=Model(1/8160)
        observed=sampling.restore_sampling_integration(model,self.receipt)
        self.assertEqual(observed['raw_reset_rotational_rate_method'],1.)
        self.assertEqual(observed['before_step']['rotational_rate_method'],4.)
        self.assertEqual(observed['before_step']['internal_dt_s'],1/8160)
        self.assertEqual(sampling.observe_integration(model,self.receipt),observed['before_step'])

    def test_missing_or_nonfinite_configuration_and_native_drift_are_rejected(self):
        for method,dt in ((True,1/8160),(3,1/8160),(4,0.),(4,float('nan')),(4,True)):
            with self.subTest(method=method,dt=dt),self.assertRaises(ValueError):
                sampling.declared_integration({'integration':{'rotational_rate_method':method,'maximum_internal_dt_s':dt}})
        model=Model(1/8160);sampling.restore_sampling_integration(model,self.receipt)
        for key,value in (('simulation/integrator/rate/rotational',1.),('simulation/dt',1/120),('simulation/dt',math.inf),('simulation/dt',0.)):
            model=Model(1/8160);sampling.restore_sampling_integration(model,self.receipt);model.values[key]=value
            with self.subTest(key=key,value=value),self.assertRaises(ValueError):sampling.observe_integration(model,self.receipt)

    def test_legacy_declared_euler_retains_its_method(self):
        receipt={'integration':{'maximum_internal_dt_s':1/120}}
        model=Model(1/120)
        self.assertEqual(sampling.restore_sampling_integration(model,receipt)['before_step']['rotational_rate_method'],1.)

    def test_independent_reload_rejects_old_sampling_or_native_configuration_drift(self):
        import copy
        import engineering_delivery as delivery
        model=Model(1/8160);value=sampling.restore_sampling_integration(model,self.receipt)
        value['after_step']=sampling.observe_integration(model,self.receipt)
        sample={'native_integration':value};declared=self.receipt['integration']
        self.assertTrue(delivery.sampling_integration_current(sample,declared))
        self.assertFalse(delivery.sampling_integration_current({},declared))
        for phase,key,value in (('before_step','rotational_rate_method',1.),('after_step','internal_dt_s',1/120),('after_step','internal_dt_s',0.)):
            broken=copy.deepcopy(sample);broken['native_integration'][phase][key]=value
            with self.subTest(phase=phase,key=key):self.assertFalse(delivery.sampling_integration_current(broken,declared))


if __name__=='__main__':unittest.main()
