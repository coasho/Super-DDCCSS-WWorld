"""Public opt-in weapon links: real release, delivered C2 consent and return telemetry."""
import math
import unittest
from pathlib import Path
import fsim
from fsim import isolation
from fsim.weapon_link_process import WeaponLinkProcessV1

def require(ok,why):
    if not ok: raise AssertionError(why)

def world(name):
    return fsim.World(name,publish=False,terrain=False,workers=1,pin_workers=False,seed=19)

def release(world, tick, store="aim-120d"):
    carrier = world.create_vehicle("launcher", "jsbsim:f16c", latitude_deg=40.0, longitude_deg=0.0,
                                   altitude_msl_m=6000.0, heading_deg=90.0, airspeed_ms=250.0)

    launcher = world.create_agent(carrier, "launcher")
    launcher.submit(fsim.Level.VELOCITY, airspeed_ms=250.0, vertical_speed_ms=0.0, heading_rad=math.pi / 2)
    c2 = world.create_c2_node("approver", 39.9, 0.0, altitude_m=200.0)
    net = world.create_net("consent")
    require(world.join_net(carrier, net) and world.join_net(c2, net), "cannot join consent net")
    approver = world.create_agent(c2, "approver")
    require(approver.declare_role(fsim.ROLE_QB, "APPROVER"), "C2 role declaration refused")
    stations = next(p.index for p in launcher.payloads if p.family == "store_station")
    radar = next(p.index for p in launcher.payloads if p.family == "air_radar")
    carrier.load_store(1,store,payload=stations)
    for payload in (stations, radar):
        launcher.set_payload_state(payload, "operate")
    for _ in range(1200):
        tick()
        if sum(p.state == fsim.PayloadState.OPERATE for p in launcher.payloads if p.index in (stations, radar)) == 2:
            break
    else:
        raise RuntimeError("native radar/station startup did not finish")
    own = carrier.state
    target = world.create_vehicle("opponent", "jsbsim:f16c", latitude_deg=math.degrees(own.latitude_rad) + .01,
                                  longitude_deg=math.degrees(own.longitude_rad) + .15,
                                  altitude_msl_m=6000.0, heading_deg=270.0, airspeed_ms=250.0)
    target.submit(fsim.Level.VELOCITY, airspeed_ms=250.0, vertical_speed_ms=0.0, heading_rad=3 * math.pi / 2)
    launcher.amti_search(radar, bars=1, azimuth_half_width_rad=math.radians(15), range_scale_m=60000)
    track = None
    for _ in range(600):
        tick()
        track = next((t for t in launcher.amti_tracks(radar) if t.updates >= 3), None)
        if track is not None:
            break
    require(track is not None, "radar did not confirm a measured track")
    links=fsim.WeaponLinksV1(launcher)
    fused=launcher.fused_tracks_v1()
    require(bool(fused),"Accepted platform fusion missing")
    designation=links.designate_fused(fused[0].id)
    require(designation, "own radar designation refused")
    launcher.amti_track(track.id, radar, range_scale_m=60000)
    activity = launcher.strike(store, target=designation, payload=stations)
    require(not world.weapons, "strike bypassed consent")
    answered = False
    for _ in range(1200):
        tick()
        for report in approver.report_consents().items:
            request = report.value
            if request.strike == activity:
                require(report.meta.source == "dms" and not world.weapons, "consent bypassed native delivery")
                require(approver.answer_consent(request.source, request.request_id), "consent answer refused")
                answered = True
                break
        if answered:
            break
    require(answered, "C2 received no release-consent request")
    for _ in range(600):
        tick()
        if world.weapons:
            break
    require(len(world.weapons) == 1, "delivered consent did not release one armed missile")
    return launcher,approver,links,world.weapons[0].id


class ActorPolicy:
    def __init__(self,links):self.links=links
    def assess(self,weapon,request_id):return self.links.assess(weapon,request_id)
    def read(self,weapon,assessment):return self.links.feedback(weapon,assessment)
    def probe(self):
        forbidden=False
        try:self.links._channel.request({'op':'agent','m':'state'})
        except AttributeError:forbidden=True
        return (forbidden,hasattr(self.links,'world'),hasattr(self.links,'_world'),hasattr(self.links,'vehicles'))

class WeaponLinkTest(unittest.TestCase):
    def test_two_way_delivery_ownership_and_cancel(self):
        with world('py-weapon-link-v1') as w:
            agent,approver,links,weapon=release(w,w.step)
            cap=links.capability(weapon)
            self.assertEqual(cap.direction,fsim.WeaponLinkDirectionV1.TWO_WAY)
            receipt=links.assess(weapon,101)
            self.assertTrue(receipt.accepted)
            self.assertEqual(links.assess(weapon,101),receipt)
            self.assertFalse(links.feedback(weapon,receipt.assessment))
            reports=[]
            for _ in range(120):
                w.step()
                reports=links.feedback(weapon,receipt.assessment)
                if reports:break
            self.assertTrue(reports)
            self.assertEqual(reports,links.feedback(weapon,receipt.assessment))
            for r in reports:
                self.assertGreater(r.received_time_s,r.sent_time_s)
                self.assertLessEqual(r.navigation_time_s,r.sample_time_s)
                self.assertTrue(r.navigation_valid)
                self.assertGreater(r.navigation_covariance[0],0)
                self.assertFalse(hasattr(r,'holding'))
                self.assertFalse(hasattr(r,'target'))
                self.assertFalse(hasattr(r,'damage'))
            self.assertFalse(fsim.WeaponLinksV1(approver).assess(weapon,101).accepted)
            self.assertEqual(links.assess(weapon,101,duration_s=29).reason,fsim.WeaponLinkReasonV1.REQUEST_CONFLICT)
            self.assertEqual(links.assess(weapon,102,instance=links.instance+1).reason,fsim.WeaponLinkReasonV1.CLOSED)
            self.assertTrue(links.cancel(weapon,receipt.assessment))
            before=links.feedback(weapon,receipt.assessment)
            w.step(30)
            self.assertEqual(links.feedback(weapon,receipt.assessment),before)
            self.assertEqual(links.status(weapon,receipt.assessment).state,fsim.WeaponAssessmentStateV1.CANCELED)

    def test_explicit_remote_proxy(self):
        with world('py-weapon-link-remote') as w:
            agent,approver,links,weapon=release(w,w.step)
            with WeaponLinkProcessV1(w,agent,'test_weapon_links:ActorPolicy',path=[str(Path(__file__).parent)]) as actor:
                receipt=actor.call('assess',weapon,201)
                self.assertTrue(receipt.accepted)
                self.assertFalse(actor.call('read',weapon,receipt.assessment))
                for _ in range(120):
                    w.step()
                    reports=actor.call('read',weapon,receipt.assessment)
                    if reports:break
                self.assertTrue(reports)
                self.assertEqual(reports,links.feedback(weapon,receipt.assessment))
                self.assertEqual(actor.call('probe'),(True,False,False,False))

    def test_receive_only_fit_refuses_return(self):
        with world('py-weapon-link-one-way') as w:
            agent,approver,links,weapon=release(w,w.step,'aim-120c')
            self.assertEqual(links.capability(weapon).direction,fsim.WeaponLinkDirectionV1.UPLINK)
            receipt=links.assess(weapon,1)
            self.assertFalse(receipt.accepted)
            self.assertEqual(receipt.reason,fsim.WeaponLinkReasonV1.UNSUPPORTED)

    def test_default_actor_methods_unchanged(self):
        calls=isolation.agent_calls()
        self.assertNotIn('weapon_links_v1',calls)
        self.assertNotIn('assess',calls)
        self.assertNotIn('feedback',calls)
        self.assertEqual(set(isolation.mission_endpoint_calls()),{'observation_json','submit_json','receipts_json'})

if __name__=='__main__':unittest.main()
