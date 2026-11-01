"""Explicit test-only population scope; ordinary runs retain every family."""
import functools
import os


def nonrotor_closure():
    value=os.environ.get('FSIM_TEST_FUNCTIONAL_SCOPE','')
    if value not in ('','sim-m02-nonrotor-v1'):
        raise ValueError('Unknown test functional population scope')
    return value=='sim-m02-nonrotor-v1'


def deferred_rotor(method):
    @functools.wraps(method)
    def invoke(self,*args,**kwargs):
        if nonrotor_closure():
            print('FSIM_SCOPE_DEFERRED_METHOD: '+self.id(),flush=True)
            self.skipTest('User-deferred rotor functional scenario; not passed')
        return method(self,*args,**kwargs)
    return invoke
