"""Air search radars through the SDK (ADR-30 MS-4, docs/sdk/mission-systems.md), for the viewer: an F-16C over the
Pacific searching ahead with its AN/APG-68(V)9 in range-while-search, an E-3G behind it sweeping 360 deg with its
AN/APY-2, and four aircraft for them to find - another F-16C head on, a B-52H crossing high, a C-130J and an A-10C
low over the sea. Each radar is powered to operate and commanded to search through the SDK; what it detects is
their tracks printed here, and the viewer draws the same radars' displays (key p shows or hides them; tab switches aircraft).
Ten minutes.

    fsim python examples\\simulation\\python\\radar.py
    fsim viewer --world radar --camera chase           (a second terminal)

--focus sentry starts the viewer on the E-3G; --fast runs as fast as it goes; --signal-level searches the F-16Cs' radars
at the signal level (MS-6: their samples and processing, the sea's clutter among them), the viewer then drawing their
range-Doppler map too.
"""
import datetime
import math
import sys
import time

import fsim

real_time = "--fast" not in sys.argv
signal_level = "--signal-level" in sys.argv
focus = sys.argv[sys.argv.index("--focus") + 1] if "--focus" in sys.argv else "viper"

LAT, LON = 37.2, -123.4  # over the Pacific, west of San Francisco
KM_LAT = 1.0 / 111.2
KM_LON = 1.0 / (111.2 * math.cos(math.radians(LAT)))


def at(north_km, east_km):
    return dict(latitude_deg=LAT + north_km * KM_LAT, longitude_deg=LON + east_km * KM_LON)


world = fsim.World("radar")
world.set_environment(epoch_utc_seconds=datetime.datetime(2026, 6, 21, 20, 0, tzinfo=datetime.timezone.utc).timestamp())  # midday
flights = {  # name: (type, north km, east km, altitude m, heading deg, speed m/s)
    "viper": ("f16c", 0, 0, 6000, 270, 230),
    "bandit": ("f16c", 4, -60, 7000, 90, 230),
    "bomber": ("b52h", 25, -75, 10500, 180, 220),
    "lifter": ("c130j", -18, -45, 4000, 0, 140),
    "hog": ("a10c", 6, -32, 2500, 30, 150),
    "sentry": ("e3g", 0, 90, 9000, 0, 170),
}
vehicles = {}
for name, (kind, north, east, alt, hdg, tas) in flights.items():
    vehicles[name] = world.create_vehicle(name, "jsbsim:" + kind, altitude_msl_m=alt, heading_deg=hdg, airspeed_ms=tas,
                                          **at(north, east))
speeds = {name: f[5] for name, f in flights.items()}

# each radar: powered to operate, then a search (its card's warm-up, where it gives one, first)
radars = {"viper": dict(mode="range_while_search", azimuth_half_width_rad=math.radians(60), elevation_centre_rad=math.radians(-3),
                       bars=4, range_scale_m=150e3),  # (4 bars, a little down: the A-10C is low)
          "bandit": dict(mode="range_while_search", azimuth_half_width_rad=math.radians(30), bars=2, range_scale_m=150e3),
          "sentry": dict(mode="surveillance", bars=2, range_scale_m=400e3)}
if signal_level:
    for name in ("viper", "bandit"):
        radars[name]["signal_level"] = True
for name in radars:
    vehicles[name].set_payload_state("radar", "operate")
world.focus_viewer(vehicles[focus])

searching = set()
names = {v.id: n for n, v in vehicles.items()}
t0 = time.perf_counter()
last_report = -1
while world.time < 600.0:
    for name, v in vehicles.items():
        # level, on its heading (a banked AWACS tilts its rotodome's beam: the sentry flies straight)
        v.command_velocity(airspeed_ms=speeds[name], vertical_speed_ms=0.0, heading_rad=math.radians(flights[name][4]))
    for name, search in radars.items():
        v = vehicles[name]
        if name not in searching and v.payload("radar").state == fsim.PayloadState.OPERATE:
            v.amti_search("radar", **search)
            searching.add(name)
            print("t=%5.1f s  %s: %s searching (%s%s)" % (world.time, name, v.payload("radar").designation, search["mode"],
                                                       ", signal level" if search.get("signal_level") else ""))
    world.step()
    if int(world.time // 10) != last_report:
        last_report = int(world.time // 10)
        for name in radars:
            # its tracks (MS-5: the radar's own track processing, false alarms and all); `truth` names who it really is
            status = vehicles[name].radar_status("radar")
            tracks = vehicles[name].amti_tracks("radar")
            print("t=%5.1f s  %s: %d tracks (%d tentative, %d false alarms so far): " % (world.time, name, len(tracks), status.tentative_tracks,
                                                                                    status.false_alarms) + ", ".join(
                "#%d %s %.0f km %+.0f deg %.0f m/s +-%.0f m" % (t.id, names.get(t.truth, "false"), t.range_m / 1e3, math.degrees(t.azimuth_rad),
                                                            math.hypot(*t.velocity_ned_ms[:2]), t.position_sigma_m)
                for t in tracks))
    if real_time:
        ahead = world.time - (time.perf_counter() - t0)
        if ahead > 0:
            time.sleep(ahead)
