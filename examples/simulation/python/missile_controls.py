"""Actual native fin motion after a legal radar/C2-guided release.

Run with the installed fsim Python package. --realtime gives the Viewer time
to attach to --world; --record writes the same fin states for replay. This is
a Range diagnostic: actual weapon telemetry is never an actor observation.
"""
import argparse
import math
import time

import fsim


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def release(world, tick):
    carrier = world.create_vehicle("launcher", "jsbsim:f16c", latitude_deg=40.0, longitude_deg=0.0,
                                   altitude_msl_m=6000.0, heading_deg=90.0, airspeed_ms=250.0)
    carrier.load_loadout("air superiority")
    launcher = world.create_agent(carrier, "launcher")
    launcher.submit(fsim.Level.VELOCITY, airspeed_ms=250.0, vertical_speed_ms=0.0, heading_rad=math.pi / 2)
    c2 = world.create_c2_node("approver", 39.9, 0.0, altitude_m=200.0)
    net = world.create_net("consent")
    require(world.join_net(carrier, net) and world.join_net(c2, net), "cannot join consent net")
    approver = world.create_agent(c2, "approver")
    require(approver.declare_role(fsim.ROLE_QB, "APPROVER"), "C2 role declaration refused")
    stations = next(p.index for p in launcher.payloads if p.family == "store_station")
    radar = next(p.index for p in launcher.payloads if p.family == "air_radar")
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
    designation = launcher.designate(radar, track.id)
    require(designation, "own radar designation refused")
    launcher.amti_track(track.id, radar, range_scale_m=60000)
    activity = launcher.strike("aim-120c", target=designation, payload=stations)
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
    return world.weapons[0].id


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="missile-controls")
    parser.add_argument("--record", default="missile-controls.fsrec")
    parser.add_argument("--realtime", action="store_true")
    parser.add_argument("--flight-s", type=float, default=6.0)
    parser.add_argument("--jam-after-s", type=float, default=2.0)
    args = parser.parse_args()
    with fsim.World(args.world, publish=True, terrain=False, workers=1, pin_workers=False, seed=19,
                    record_path=args.record, record_interval_s=1 / 60) as world:
        def tick():
            world.step()
            if args.realtime:
                time.sleep(world.step_seconds)
        missile = release(world, tick)
        world.focus_viewer(missile)
        start = world.time
        jammed = None
        moved = False
        controller_seen = False
        unavailable_decisions = 0
        while world.time - start < args.flight_s:
            tick()
            sample = world.weapon_controls(missile)
            if sample is None:
                break
            require(sample.available, "native actual-fin metadata unavailable")
            controller_seen |= sample.controller_available
            if not sample.controller_available:
                unavailable_decisions += 1
                require(all(command == 0 for command in sample.commands),
                        "unavailable controller decision did not return neutral commands")
            require(sample.sample_time_s == world.time and sample.controller_time_s <= sample.sample_time_s,
                    "control sample is not aligned to native time")
            for fin in sample.fins:
                require(fin.valid and fin.minimum_rad <= fin.actual_rad <= fin.maximum_rad, "invalid actual fin state")
                moved |= abs(fin.actual_rad) > 1e-8
            fin = sample.fins[0]
            if jammed is None and world.time - start >= args.jam_after_s:
                world.vehicle(missile).set_property("fcs/fin-%d-actuator/malfunction/fail_stuck" % fin.native_index, 1)
                jammed = (fin.id, fin.actual_rad)
                print("native fin jam: id=%s actual_rad=%.9f" % jammed)
            elif jammed is not None:
                failed = next(f for f in sample.fins if f.id == jammed[0])
                require(failed.failed_stuck and failed.actual_rad == jammed[1], "actual jam state was lost")
        require(moved, "armed guidance never moved a physical fin")
        require(controller_seen, "native controller never produced an available decision")
        print("native armed release=%d actual_fin_motion=1 unavailable_decisions=%d record=%s" %
              (missile, unavailable_decisions, args.record))


if __name__ == "__main__":
    main()
