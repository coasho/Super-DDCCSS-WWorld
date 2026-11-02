"""Native V1 fusion through the public SDK, with real sensor and link delivery."""
import math
import unittest

import fsim


class FusionTest(unittest.TestCase):
    def test_local_and_delivered_knowledge(self):
        w=fsim.World('py-fusion-v1',publish=False,workers=1,pin_workers=False,terrain=False,seed=3)
        def aircraft(name,lat,lon,heading):
            return w.create_vehicle(name,'jsbsim:f16c',latitude_deg=lat,longitude_deg=lon,
                altitude_msl_m=5000,airspeed_ms=200,heading_deg=heading)
        own=aircraft('lead',40,0,90)
        other=aircraft('target',40,8e3/(6371e3*math.cos(math.radians(40)))*180/math.pi,270)
        receiver=aircraft('receiver',39.97,0,90)
        net=w.create_net('fusion',latency_s=.15)
        for vehicle in (own,receiver):w.join_net(vehicle,net)
        lead=w.create_agent(own,'lead');wing=w.create_agent(receiver,'wing')
        lead.set_payload_state('radar','operate');w.step()
        lead.amti_search(azimuth_half_width_rad=math.radians(3),bars=2)
        for _ in range(600):
            w.step()
            if lead.fused_tracks_v1():break
        local=lead.fused_tracks_v1();self.assertTrue(local)
        self.assertTrue(all(not t.identity_known and not hasattr(t,'truth') for t in local))
        self.assertTrue(any(s.kind==fsim.FusionSourceKindV1.LOCAL_RADAR for t in local for s in t.sources))
        self.assertEqual(lead.fused_tracks_v1(),local)
        self.assertEqual(lead.fusion_status_v1().rejected_inputs,0)
        with self.assertRaises(fsim.Error):lead._world._h.fused_tracks_v1(other.id)
        lead.publish_tracks(destinations=[wing.system])
        topic=fsim.TOPIC_FREE_FORM+51;wing.subscribe(topic)
        lead.send(topic,b'fusion-preserves-inbox',destinations=[wing.system])
        w.step(60)
        remote=wing.fused_tracks_v1();self.assertTrue(remote)
        self.assertEqual(wing.fused_tracks_v1(),remote)
        self.assertTrue(any(s.kind==fsim.FusionSourceKindV1.DELIVERED_LINK and s.system==lead.system and
            s.diagonal_link_covariance and s.received_time_s>s.sample_time_s for t in remote for s in t.sources))
        for t in remote:
            self.assertTrue(all(math.isfinite(v) for v in (*t.state_ecef,*t.covariance_ecef)))
            self.assertTrue(all(t.covariance_ecef[i*6+i]>0 for i in range(6)))
        self.assertTrue(any(m.data==b'fusion-preserves-inbox' for m in wing.inbox()))
        self.assertEqual(wing.fusion_status_v1().rejected_inputs,0)


if __name__=='__main__':unittest.main()
