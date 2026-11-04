"""Pure fixture-contract controls; no SDK or simulation execution."""
import math
import unittest
from types import SimpleNamespace
from explosives import C2_ENDPOINT,create_approver_endpoint,retire_coordinate,Clock
from acceptance import WorldHealthV1,reconcile_world_health


class Site:
    def __init__(self,height):self.height=height;self.calls=[]
    def terrain(self,lat,lon):self.calls.append(('terrain',lat,lon));return self.height
    def create_c2_node(self,name,lat,lon,**values):
        self.calls.append(('c2',name,lat,lon,values));return 42


class ExplosiveFixtureTests(unittest.TestCase):
    def test_clock_observes_final_world_state_after_all_weapons_are_terminal(self):
        state=SimpleNamespace(diverged=False,airspeed_true_ms=200.,position_ecef=[0.,0.,0.],
            velocity_body_ms=[200.,0.,0.],angular_rate_body_rad_s=[0.,0.,0.],alpha_rad=0.,beta_rad=0.)
        vehicle=SimpleNamespace(id=1,name='carrier',state=state,mass_kg=1000.)
        class World:
            time=0.;weapons=[SimpleNamespace(end='detonated')]
            def vehicles(self):return [vehicle]
            def step(self):
                self.time+=1
                if self.time==2:state.diverged=True
        world=World();clock=Clock(world,False);health=WorldHealthV1();clock.health=health;health.observe(world)
        clock.step();clock.step()
        self.assertEqual([r[0] for r in health.rows],[0.,1.,2.]);self.assertEqual(health.invalid,{1})
        record=dict(complete_world_finite=False,observed_world_states=3,launch_time_s=0.,duration_s=2.,frame_skip=120)
        with self.assertRaises(ValueError):reconcile_world_health(record,dict(world_health=health.rows,invalid_entities=[1]))

    def test_antenna_absolute_height_uses_native_site_clearance(self):
        site=Site(812.5);node,receipt=create_approver_endpoint(site)
        self.assertEqual(node,42)
        self.assertEqual(site.calls[0],('terrain',math.radians(36.20),math.radians(-115.03)))
        self.assertEqual(site.calls[1],('c2','approver',36.20,-115.03,{'altitude_m':1012.5}))
        self.assertEqual(receipt['native_ground_m'],812.5)
        self.assertEqual(receipt['altitude_m']-receipt['native_ground_m'],C2_ENDPOINT['antenna_clearance_m'])

    def test_missing_or_nonfinite_native_ground_never_creates_endpoint(self):
        for height in (None,math.nan,math.inf,-math.inf):
            with self.subTest(height=height):
                site=Site(height)
                with self.assertRaisesRegex(ValueError,'native terrain unavailable'):create_approver_endpoint(site)
                self.assertEqual(len(site.calls),1)

    def test_void_undesignate_is_verified_by_owner_scoped_state(self):
        class Agent:
            valid=True
            def designations(self):return [(123,0,99,self.valid)]
            def undesignate(self,target):self.valid=False
        receipt=retire_coordinate(Agent(),123)
        self.assertTrue(receipt['previously_valid']);self.assertFalse(receipt['valid_after'])

    def test_designation_that_remains_valid_is_not_claimed_retired(self):
        class Agent:
            def designations(self):return [(123,0,99,True)]
            def undesignate(self,target):return None
        with self.assertRaisesRegex(ValueError,'remained valid'):retire_coordinate(Agent(),123)


if __name__=='__main__':unittest.main(verbosity=2)
