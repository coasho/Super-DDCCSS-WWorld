"""Legal reports with their provenance (docs/sdk/agents.md, "Legal reports"; docs/sdk/mission-training.md, section 10).

    fsim python examples/training/python/ma_reports.py

A lead searches with its radar and shares its tracks over a net; its wingman keeps its radar off. The script prints
what each agent holds and where each report came from: the lead's own radar tracks, sampled at their last update; the
wingman's link tracks, sampled when the lead reported them and received when the net delivered them; the same link
tracks a second later with nothing delivered in between - unchanged but older, and stale under a threshold.

It uses the public Python SDK alone, and publishes nothing: there is no flight to watch.
"""
import math

import fsim

EAST_KM_DEG = math.degrees(1e3 / (6371000.0 * math.cos(math.radians(40.0))))


def aircraft(w, name, north_km, east_km, altitude_m, heading_deg):
    return w.create_vehicle(name, "jsbsim:f16c", latitude_deg=40.0 + north_km / 111.2, longitude_deg=east_km * EAST_KM_DEG,
                            altitude_msl_m=altitude_m, heading_deg=heading_deg, airspeed_ms=230.0)


def time(x):
    return "unknown" if x is None else "%.2f s" % x


def show(title, collection):
    print("%s: %s, %d item(s)" % (title, collection.availability, len(collection.items)))
    for meta, value in collection.items:
        origin = meta.source + ("" if meta.system is None else " from system %#x over net %d" % (meta.system, meta.net))
        print("  %-36s %-7s sampled %s, received %s, age %s" % (origin, meta.validity, time(meta.sample_time_s),
                                                             time(meta.receive_time_s), time(meta.age_s)))


def main():
    w = fsim.World("ma-reports-demo", publish=False, workers=1, pin_workers=False, seed=3, terrain=False)
    lead = aircraft(w, "lead", 0.0, 0.0, 6000.0, 90.0)
    aircraft(w, "bandit", 3.0, 55.0, 7000.0, 270.0)
    wing = aircraft(w, "wing", -10.0, 0.0, 6000.0, 90.0)
    blue1, blue2 = w.create_agent(lead, "blue-1"), w.create_agent(wing, "blue-2")
    net = w.create_net("l16")
    w.assign_system(lead, 0x4001, "VIPER11")
    w.join_net(lead, net)
    w.join_net(wing, net)
    w.set_dissemination(lead, position_period_s=1.0, tracks_period_s=2.0)
    blue1.set_payload_state("radar", "operate")
    w.step()
    blue1.amti_search("radar", elevation_centre_rad=math.radians(-1.0), bars=4)
    w.step(int(round(45.0 / w.step_seconds)))

    nav = blue1.report_navigation()
    print("time %.2f s; the lead's navigation: %s, %s, sampled %s" % (w.time, nav.meta.source, nav.meta.validity, time(nav.meta.sample_time_s)))
    show("the lead's own radar tracks", blue1.report_tracks())
    show("the wingman's own radar tracks (its radar is off)", blue2.report_tracks())
    show("the wingman's link tracks", blue2.report_link_tracks())
    show("the wingman's peers", blue2.report_peers())
    w.step(int(round(1.0 / w.step_seconds)))
    print("time %.2f s" % w.time)
    show("the wingman's link tracks, stale after 1.5 s", blue2.report_link_tracks(stale_after_s=1.5))


if __name__ == "__main__":
    main()
