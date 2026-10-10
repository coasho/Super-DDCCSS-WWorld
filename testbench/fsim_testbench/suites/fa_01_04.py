"""FA-1 to FA-4: discovery and support, the command envelope and activities, performance and energy, the terrain, the
altimeter, the frames and the magnetic model."""
import math

import fsim

from ..registry import aircraft, choice, number, scenario
from ._fa import ALL, FIGHTERS, R, ROTORS, WINGS, deg, ended, heading_of, hold_level, offset_rad, rad, reason, wrap_deg

G1 = "FA-1 Discovery and support"
G2 = "FA-2 Commands and activities"
G3 = "FA-3 Performance and energy"
G4 = "FA-4 Terrain, altimeter, frames, magnetic"


# --- FA-1 ----------------------------------------------------------------------------------------------------------
@scenario("fa1.support", group=G1, title="What an aircraft can do", refs="ADR-29 4.1-4.7",
          summary="The support table of the chosen aircraft - every public feature supported, partial, not implemented or not "
                  "supported, with the rule that excludes it - and its capabilities. Then a feature it cannot fly is commanded "
                  "and refused with that reason, and one it can is flown: a wing a route out and back, a rotorcraft a hover "
                  "over a point ahead.",
          options=[aircraft("f16c", ALL)])
def support(t):
    kind = t.opt("aircraft")
    v = t.spawn(kind, -3000.0, 4000.0, agl_m=800.0, heading_deg=0.0)
    hold_level(v) if not v.performance.hovers else v.submit(fsim.Level.VELOCITY, north_ms=0.0, east_ms=0.0, vertical_speed_ms=0.0)
    table = v.support_table()
    counts = {}
    for s in table:
        counts[s.support.name] = counts.get(s.support.name, 0) + 1
        why = ""
        if s.support != fsim.Support.SUPPORTED:
            why = " - " + ", ".join(str(r) for r in s.rules) if s.rules else (" - stage " + str(s.stage) if s.stage else "")
        t.log("  %-48s %s%s" % (s.feature, s.support.name, why))
    t.log("support: " + ", ".join("%s %d" % kv for kv in sorted(counts.items())))
    caps = v.capabilities()
    t.log("capabilities: " + ", ".join(c.id for c in caps))
    t.check("the table lists every public feature", len(table) == len(fsim.SUPPORT_FEATURES), "%d features" % len(table))
    hovers = v.performance.hovers
    hover = v.support("fsim.guidance.hover")
    t.check("a hover is %s" % ("supported (it hovers)" if hovers else "not supported (a wing)"),
            (hover.support == fsim.Support.SUPPORTED) == bool(hovers), hover.support.name)
    try:
        v.submit_pattern(pattern="hover", duration_s=20.0) if not hovers else v.submit_pattern(pattern="hold", radius_m=2000.0)
        t.check("what it cannot fly is refused", False, "accepted")
    except fsim.Rejected as ex:
        t.check("what it cannot fly is refused, saying why", ex.reason in ("not_supported", "not_implemented", "invalid_parameter"), ex.reason)
    S = t.fit(kind)
    s0 = v.state
    if hovers:  # a rotorcraft: to a point 300 m ahead and hover there
        lat, lon = offset_rad(s0.latitude_rad, s0.longitude_rad, 300.0, 0.0)
        a = t.watch(v.submit_pattern(pattern="hover", latitude_rad=lat, longitude_rad=lon, altitude_m=s0.altitude_msl_m, duration_s=30.0),
                    "hover over a point ahead")
    else:  # a wing: a route out 4 km, across and back
        lat, lon = s0.latitude_rad, s0.longitude_rad
        pts = [fsim.Waypoint(*offset_rad(lat, lon, n * S, e * S), s0.altitude_msl_m) for n, e in ((4000.0, 0.0), (4000.0, 3000.0), (0.0, 3000.0))]
        a = t.watch(v.submit_route(pts), "a route out and back")
    ok = yield t.until_done(a, 900.0)
    t.check("what it can fly is flown", ended(a), a.state.name)


# --- FA-2 ----------------------------------------------------------------------------------------------------------
@scenario("fa2.hsa", group=G2, title="Heading, speed and altitude (HSA)", refs="ADR-29 4.8; VI 4.4",
          summary="An HSA turns the aircraft 90 degrees, climbs it 300 m and changes its speed; UPDATEs change one field at a "
                  "time (use the buttons while it flies). Checks: each target captured.",
          options=[aircraft("c172", WINGS + ROTORS), number("turn", "Turn", 90.0, -180.0, 180.0, 15.0, "deg"),
                   number("climb", "Climb", 300.0, -300.0, 1500.0, 50.0, "m")])
def hsa(t):
    kind = t.opt("aircraft")
    v = t.spawn(kind, 0.0, -4000.0, agl_m=700.0)
    s = v.state
    target_hdg = (heading_of(s) + t.opt("turn")) % 360.0
    target_alt = s.altitude_msl_m + t.opt("climb")
    speed = max(v.performance.min_cas_ms * 1.3, s.airspeed_true_ms) if not v.performance.hovers else 15.0
    a = t.watch(v.submit_hsa(heading_rad=rad(target_hdg), altitude_m=target_alt, speed=speed, speed_reference="true_airspeed"), "hsa")
    t.metric("heading error, deg", lambda: wrap_deg(heading_of(v.state) - target_hdg))
    t.metric("altitude error, m", lambda: v.state.altitude_msl_m - target_alt)
    goal = {"hdg": target_hdg, "alt": target_alt}

    def turn(d):
        def go():
            goal["hdg"] = (goal["hdg"] + d) % 360.0
            a.update(heading_rad=rad(goal["hdg"]))
        return go

    def climb(d):
        def go():
            goal["alt"] += d
            a.update(altitude_m=goal["alt"])
        return go
    t.action("Turn left 90", turn(-90.0))
    t.action("Turn right 90", turn(90.0))
    t.action("Climb 300 m", climb(300.0))
    t.action("Descend 300 m", climb(-300.0))
    ok = yield t.until(lambda: abs(wrap_deg(heading_of(v.state) - target_hdg)) < 3.0, 180.0, "the heading")
    t.check("heading captured within 3 deg", ok, "%.1f deg off" % wrap_deg(heading_of(v.state) - target_hdg))
    ok = yield t.until(lambda: abs(v.state.altitude_msl_m - target_alt) < 20.0, 300.0, "the altitude")
    t.check("altitude captured within 20 m", ok, "%.0f m off" % (v.state.altitude_msl_m - target_alt))
    yield t.wait(20.0)
    t.check("the hsa still flies", a.live, a.state.name)


@scenario("fa2.schedule", group=G2, title="Ranks, a start window and a queued command", refs="ADR-29 4.9, 4.10",
          summary="An HSA flies; a second, outranked, waits for it; a third waits for its start window 20 s away. Cancel the "
                  "first and the waiting one starts; disable and enable an activity with the buttons.",
          options=[aircraft("c172", WINGS)])
def schedule(t):
    v = t.spawn(t.opt("aircraft"), 0.0, -4000.0, agl_m=700.0)
    h = heading_of(v.state)
    first = t.watch(v.submit(fsim.Level.VELOCITY, airspeed_ms=v.state.airspeed_true_ms, heading_rad=rad(h), rank=(1, 0)), "velocity, rank 1")
    waits = t.watch(v.submit_hsa(heading_rad=rad(h + 60.0), rank=fsim.Rank(5, 2)), "hsa, rank 5: outranked")
    t.check("the outranked hsa waits", waits.deferred and waits.state == fsim.ActivityState.PENDING, waits.state.name)
    later = t.watch(v.submit_hsa(heading_rad=rad(h - 60.0), window=fsim.TimeWindow(start_not_before=t.world.time + 20.0, criticality="start")),
                    "hsa, starts in 20 s")
    t.action("Cancel the velocity", lambda: first.cancel() if first.live else None)
    t.action("Disable the 20 s hsa", lambda: later.disable() if later.live else None)
    t.action("Enable the 20 s hsa", lambda: later.enable() if later.live else None)
    yield t.wait(5.0)
    t.check("the windowed hsa has not started yet", later.state == fsim.ActivityState.PENDING, later.state.name)
    ok = yield t.until(lambda: later.state == fsim.ActivityState.ACTIVE, 40.0, "its window")
    t.check("it starts in its window", ok, "%s at %.0f s" % (later.state.name, t.world.time))
    ok = yield t.until(lambda: abs(wrap_deg(heading_of(v.state) - (h - 60.0))) < 5.0, 120.0, "its heading")
    t.check("and turns to its heading", ok)


@scenario("fa2.tasks", group=G2, title="A stored task, flown twice", refs="ADR-29 4.11",
          summary="An aileron roll stored as a task with two attempts, then commanded: the task runs twice and completes. A "
                  "speed beyond the aircraft's range is refused with a suggested task in its place, which is flown.",
          options=[aircraft("f16c", FIGHTERS)])
def tasks(t):
    v = t.spawn(t.opt("aircraft"), 0.0, -5000.0, agl_m=3000.0, speed_ms=200.0)
    hold_level(v)
    yield t.wait(3.0)
    roll = fsim.BatchCommand("submit_behavior", "aerobatics", manoeuvre=0.0)
    v.store_task(7, roll, attempts=2, interval_s=1.0)
    t.check("stored, awaiting execution", v.task_status(7).state == fsim.TaskState.AWAITING_EXECUTION)
    t.metric("task 7", lambda: "%s, run %d of %d, %.0f %%" % (v.task_status(7).state.name, v.task_status(7).run, v.task_status(7).runs,
                                                                v.task_status(7).percent))
    a = t.watch(v.command_task(7, command_id=4), "task 7: a roll")
    ok = yield t.until(lambda: v.task_status(7).state == fsim.TaskState.COMPLETED, 90.0, "both runs")
    s = v.task_status(7)
    t.check("the task completes after its two runs", ok and (s.run, s.runs) == (2, 2), "%s, run %d of %d" % (s.state.name, s.run, s.runs))
    try:
        v.submit_hsa(heading_rad=1.5, speed=600.0, speed_reference="true_airspeed", range=fsim.RangePolicy.REJECT)
        t.check("600 m/s refused", False, "accepted")
        return
    except fsim.Rejected as ex:
        t.check("600 m/s refused, with a suggestion", bool(ex.suggestion), "%s, suggestion %#x" % (ex.reason, ex.suggestion))
        suggestion = ex.suggestion
    b = t.watch(v.command_task(suggestion, range=fsim.RangePolicy.REJECT), "the suggested task")
    yield t.wait(20.0)
    t.check("the suggestion is flown", b.state in (fsim.ActivityState.ACTIVE, fsim.ActivityState.COMPLETED), b.state.name)


# --- FA-3 ----------------------------------------------------------------------------------------------------------
@scenario("fa3.navigation", group=G3, title="Fuel, endurance and playtime", refs="ADR-29 4.14",
          summary="The navigation report while the aircraft flies away from its recovery point (the runway): fuel, endurance, "
                  "and the playtime left before it must turn back, shrinking as it goes.",
          options=[aircraft("c172", WINGS + ROTORS)])
def navigation(t):
    v = t.spawn(t.opt("aircraft"), 0.0, 3000.0, agl_m=900.0, heading_deg=90.0)
    hold_level(v)
    lat, lon = t.place(0.0, 0.0)
    v.set_recovery(lat, lon, t.field_m + 300.0)
    t.metric("fuel, kg", lambda: v.navigation_report().fuel_kg)
    t.metric("percent left", lambda: v.navigation_report().percent)
    t.metric("endurance, min", lambda: v.navigation_report().endurance_s / 60.0)
    t.metric("playtime, min", lambda: v.navigation_report().playtime_s / 60.0)
    t.metric("return distance, km", lambda: v.navigation_report().return_distance_m / 1000.0)
    t.metric("contingency", lambda: v.navigation_report().contingency.name)
    # (settled first: the playtime is reckoned at the fuel flow now, which a jet's hold is still trimming for a while)
    yield t.wait(30.0)
    r0 = v.navigation_report()
    t.check("a playtime to the recovery point, less than the endurance", 0.0 < r0.playtime_s < r0.endurance_s,
            "%.0f of %.0f min" % (r0.playtime_s / 60.0, r0.endurance_s / 60.0))
    yield t.wait(90.0)
    r1 = v.navigation_report()
    t.check("flying away, the playtime shrinks", r1.playtime_s < r0.playtime_s, "%.1f -> %.1f min" % (r0.playtime_s / 60.0, r1.playtime_s / 60.0))
    t.check("and the return grows", r1.return_distance_m > r0.return_distance_m,
            "%.1f -> %.1f km" % (r0.return_distance_m / 1000.0, r1.return_distance_m / 1000.0))


@scenario("fa3.speed", group=G3, title="Long-range cruise and maximum endurance", refs="ADR-29 4.13, 4.17",
          summary="An HSA flown at the performance tables' long-range cruise speed, then at their maximum-endurance speed: "
                  "the second is slower and burns less fuel per second.",
          options=[aircraft("c130j", WINGS)])
def speed_opt(t):
    v = t.spawn(t.opt("aircraft"), 0.0, -6000.0, agl_m=2500.0)
    a = t.watch(v.submit_hsa(heading_rad=v.state.euler_rad[2], speed_optimization="long_range_cruise"), "hsa: long-range cruise")
    t.metric("commanded speed, m/s", lambda: a.progress.speed_ms if a.live else float("nan"))
    t.metric("fuel flow, kg/s", lambda: v.navigation_report().consumption)
    yield t.wait(90.0)
    lrc, flow1 = v.state.airspeed_true_ms, v.navigation_report().consumption
    t.log("long-range cruise: %.1f m/s TAS, %.3f kg/s" % (lrc, flow1))
    a.update(speed_optimization="max_endurance")
    yield t.wait(120.0)
    mxe, flow2 = v.state.airspeed_true_ms, v.navigation_report().consumption
    t.log("maximum endurance: %.1f m/s TAS, %.3f kg/s" % (mxe, flow2))
    t.check("maximum endurance flies slower", mxe < lrc - 2.0, "%.1f against %.1f m/s" % (mxe, lrc))
    t.check("and burns less", flow2 < flow1, "%.3f against %.3f kg/s" % (flow2, flow1))


@scenario("fa3.endurance", group=G3, title="A route beyond the fuel", refs="ADR-29 4.18",
          summary="A route 3,000 km long is refused insufficient_endurance, saying what it needs against what is left; "
                  "validated, the same; overridden, it is flown and marked so. A short one is flown without a word.",
          options=[aircraft("c172", WINGS)], terrain=False)  # (a route across the Pacific: no tiles along it)
def endurance(t):
    v = t.spawn(t.opt("aircraft"), 0.0, 0.0, agl_m=1200.0, heading_deg=270.0)
    hold_level(v)
    yield t.wait(3.0)
    s = v.state
    # beyond its fuel: half again its range, as its navigation report gives it (its endurance at its consumption now)
    nav = v.navigation_report()
    reach = nav.endurance_s * max(s.airspeed_true_ms, 30.0) if math.isfinite(nav.endurance_s) else 3.0e6
    far_m = min(1.5 * reach, 1.5e7)
    t.log("its range now about %.0f km: the far route %.0f km west" % (reach / 1000.0, far_m / 1000.0))
    flat, flon = offset_rad(s.latitude_rad, s.longitude_rad, 0.0, -far_m)
    far = [fsim.Waypoint(flat, math.remainder(flon, 2.0 * math.pi), altitude_m=s.altitude_msl_m)]
    near = [fsim.Waypoint(*offset_rad(s.latitude_rad, s.longitude_rad, 0.0, -2.0e4), altitude_m=s.altitude_msl_m)]
    try:
        v.submit_route(far)
        t.check("the far route is refused", False, "accepted")
    except fsim.Rejected as ex:
        e = ex.endurance
        t.check("the far route is refused insufficient_endurance", ex.reason == "insufficient_endurance" and e is not None,
                "%s: needs %.0f, has %.0f above its reserve" % (ex.reason, e.required if e else 0, e.remaining if e else 0))
    check = v.submit_route(far, validate_only=True)
    t.check("validated: the same answer", not check.valid and check.reason == "insufficient_endurance", check.reason)
    flown = t.watch(v.submit_route(far, override_rejection=True), "the far route, overridden")
    t.check("overridden, it is flown and marked", flown.overridden)
    yield t.wait(30.0)
    near_a = t.watch(v.submit_route(near), "the near route")
    t.check("the near route needs no override", not near_a.overridden and near_a.endurance is None)
    ok = yield t.until_done(near_a, 600.0)
    t.check("the near route is flown to its end", ended(near_a), near_a.state.name)


@scenario("fa3.profile", group=G3, title="The performance profile", refs="ADR-29 4.13, 4.15",
          summary="The aircraft's performance summary and its HSA profile at its condition now - speeds, ceiling, climb, "
                  "turn and roll limits - logged, and an HSA asked beyond them refused with the limit it breaks. Then flown: "
                  "a 600 m climb at its tables' best rate, and a reversal within its bank limit.",
          options=[aircraft("f16c", WINGS)])
def profile(t):
    v = t.spawn(t.opt("aircraft"), 0.0, -3000.0, agl_m=2000.0)
    hold_level(v)
    p = v.performance
    for name in ("min_cas_ms", "max_cas_ms", "max_mach", "cruise_tas_ms", "ceiling_m", "max_bank_rad", "max_load_factor", "max_climb_ms",
                 "max_descent_ms", "max_roll_rate_rad_s"):
        x = getattr(p, name)
        t.log("  %-22s %s" % (name, "%.3f" % x if isinstance(x, float) else x))
    try:
        prof = v.performance_profile("hsa_csa")
        t.log("hsa_csa profile: " + ", ".join("%s=%s" % (k, ("%.2f" % x) if isinstance(x, float) else x)
                                             for k, x in prof._asdict().items() if not isinstance(x, list)))
        t.check("an hsa profile at its condition now", prof is not None)
    except fsim.Rejected as ex:
        t.check("an hsa profile", False, ex.reason)
    try:
        v.submit_hsa(speed=5.0 * p.max_cas_ms if math.isfinite(p.max_cas_ms) else 2000.0, speed_reference="calibrated_airspeed",
                     range=fsim.RangePolicy.REJECT)
        t.check("a speed beyond its envelope is refused", False, "accepted")
    except fsim.Rejected as ex:
        t.check("a speed beyond its envelope is refused, naming the limit", ex.reason == "invalid_parameter" or ex.constraint != "none",
                "%s, %s" % (ex.reason, ex.constraint))
    # flown: a best-rate climb of 600 m by its performance tables, then a reversal turn within its bank limit
    s0 = v.state
    lat0, lon0, alt0, hdg0 = s0.latitude_rad, s0.longitude_rad, s0.altitude_msl_m, s0.euler_rad[2]  # (the state is a live view: values now)
    S = t.fit(t.opt("aircraft"))
    ahead = offset_rad(lat0, lon0, 12000.0 * S * math.cos(hdg0), 12000.0 * S * math.sin(hdg0))
    c = t.watch(v.submit_route([fsim.Waypoint(ahead[0], ahead[1], alt0 + 600.0, climb_optimization="best_rate")]), "best-rate climb, +600 m")
    t.metric("climb rate, m/s", lambda: -v.state.velocity_ned_ms[2])
    ok = yield t.until(lambda: v.state.altitude_msl_m > alt0 + 580.0 or not c.live, 900.0, "600 m higher")
    t.check("climbs 600 m at its best rate", ok and v.state.altitude_msl_m > alt0 + 580.0, "%.0f m up" % (v.state.altitude_msl_m - alt0))
    bank = [0.0]

    def banked():
        bank[0] = max(bank[0], abs(deg(v.state.euler_rad[0])))
        return bank[0]
    t.every(banked)
    t.metric("most bank, deg", lambda: bank[0])
    back = (deg(v.state.euler_rad[2]) + 180.0) % 360.0
    r = t.watch(v.submit_hsa(heading_rad=rad(back), altitude_m=v.state.altitude_msl_m), "a reversal: heading +180")
    ok = yield t.until(lambda: abs(wrap_deg(heading_of(v.state) - back)) < 5.0, 300.0, "the reversal")
    t.check("turns round, banking within its limit", ok and 10.0 < bank[0] <= deg(p.max_bank_rad) + 5.0,
            "%.0f deg at the most (its limit %.0f)" % (bank[0], deg(p.max_bank_rad)))


# --- FA-4 ----------------------------------------------------------------------------------------------------------
# San Bruno Mountain, 400 m, 11 km north-west of the runway
SAN_BRUNO = (37.6871, -122.4340)


@scenario("fa4.terrain", group=G4, title="A route into a mountain", refs="ADR-29 4.19",
          summary="A route at 250 m across San Bruno Mountain (400 m) is refused terrain_conflict, naming where it would meet "
                  "the ground and when. The same route at 700 m is accepted and flown over the summit.",
          options=[aircraft("c172", WINGS)])
def terrain(t):
    lat, lon = map(rad, SAN_BRUNO)
    start = offset_rad(lat, lon, 0.0, 8000.0)  # (over the bay, east of it)
    # (450 m: the route at 700 m is to be flown over the mountain - a jet, fast, climbs there on a shallower path
    # than a C172, and from 250 m would meet the slope on its way up)
    v = t.spawn(t.opt("aircraft"), at=(deg(start[0]), deg(start[1])), alt_m=450.0, heading_deg=270.0)
    hold_level(v)
    yield t.wait(2.0)
    beyond = offset_rad(lat, lon, 0.0, -6000.0)
    low = [fsim.Waypoint(lat, lon, altitude_m=250.0), fsim.Waypoint(beyond[0], beyond[1], altitude_m=250.0)]
    try:
        v.submit_route(low)
        t.check("the route at 250 m is refused", False, "accepted")
    except fsim.Rejected as ex:
        tp = ex.terrain
        t.check("refused terrain_conflict, with the place it meets the ground", ex.reason == "terrain_conflict" and tp is not None,
                "%s at %.4f, %.4f, ground %.0f m, %.0f s ahead" % (ex.reason, deg(tp.latitude_rad), deg(tp.longitude_rad), tp.ground_m, tp.time_s)
                if tp else ex.reason)
    high = [fsim.Waypoint(lat, lon, altitude_m=700.0), fsim.Waypoint(beyond[0], beyond[1], altitude_m=700.0)]
    a = t.watch(v.submit_route(high), "the route at 700 m")
    lowest = [1e9]

    def track():
        lowest[0] = min(lowest[0], v.state.altitude_agl_m)
        return lowest[0]
    t.every(track)
    t.metric("lowest above the ground, m", lambda: lowest[0])
    ok = yield t.until_done(a, 900.0)
    t.check("the route at 700 m is flown over the mountain", ended(a), a.state.name)
    t.check("never under 150 m above the ground", lowest[0] > 150.0, "%.0f m at the least" % lowest[0])


@scenario("fa4.altimeter", group=G4, title="The barometric altimeter and its setting", refs="ADR-29 4.20",
          summary="An HSA holds 1,500 m on the altimeter (barometric); the QNH is changed by 10 hPa with the buttons and the "
                  "aircraft follows its isobar: the indicated altitude stays, the true altitude moves about 85 m.",
          options=[aircraft("c172", WINGS)])
def altimeter(t):
    v = t.spawn(t.opt("aircraft"), 0.0, -4000.0, alt_m=1500.0)
    v.set_qnh(101325.0)
    a = t.watch(v.submit_hsa(heading_rad=v.state.euler_rad[2], altitude_m=1500.0, altitude_reference="barometric"), "hsa at 1,500 m barometric")
    t.metric("QNH, hPa", lambda: v.qnh / 100.0)
    t.metric("indicated altitude, m", lambda: v.state_data().indicated_altitude_m)
    t.metric("true altitude, m", lambda: v.state.altitude_msl_m)
    t.action("QNH +10 hPa", lambda: v.set_qnh(v.qnh + 1000.0))
    t.action("QNH -10 hPa", lambda: v.set_qnh(v.qnh - 1000.0))
    yield t.wait(60.0)
    alt1 = v.state.altitude_msl_m
    t.check("holds 1,500 m indicated", abs(v.state_data().indicated_altitude_m - 1500.0) < 15.0, "%.0f m" % v.state_data().indicated_altitude_m)
    v.set_qnh(102325.0)
    t.log("QNH set to 1023.25 hPa")
    yield t.wait(90.0)
    alt2 = v.state.altitude_msl_m
    t.check("on the new setting, still 1,500 m indicated", abs(v.state_data().indicated_altitude_m - 1500.0) < 15.0,
            "%.0f m" % v.state_data().indicated_altitude_m)
    t.check("its true altitude moved down with the isobar", alt1 - alt2 > 50.0, "%.0f m lower" % (alt1 - alt2))


@scenario("fa4.magnetic", group=G4, title="A magnetic heading", refs="ADR-29 4.22",
          summary="An HSA flown on a magnetic heading: the aircraft's true heading differs by the declination at SFO "
                  "(about 13 degrees east), as the World Magnetic Model gives it.",
          options=[aircraft("c172", WINGS), number("heading", "Magnetic heading", 360.0, 0.0, 360.0, 10.0, "deg")])
def magnetic(t):
    v = t.spawn(t.opt("aircraft"), 0.0, -3000.0, agl_m=800.0)
    a = t.watch(v.submit_hsa(heading_rad=rad(t.opt("heading") % 360.0), direction_reference=fsim.DirectionReference.MAGNETIC_NORTH),
                "hsa: magnetic heading")
    t.metric("magnetic heading, deg", lambda: deg(v.state_data().magnetic_heading_rad) % 360.0)
    t.metric("true heading, deg", lambda: heading_of(v.state))
    t.metric("declination, deg", lambda: deg(v.state_data().declination_rad))
    yield t.wait(90.0)
    s, d = v.state, v.state_data()
    expected = fsim.magnetic_field(s.latitude_rad, s.longitude_rad, s.altitude_msl_m, t.world.magnetic_year).declination_rad
    t.check("the declination is the model's", abs(d.declination_rad - expected) < 1e-6, "%.2f deg" % deg(expected))
    t.check("it flies the magnetic heading", abs(wrap_deg(deg(d.magnetic_heading_rad) - t.opt("heading"))) < 3.0,
            "%.1f deg magnetic" % (deg(d.magnetic_heading_rad) % 360.0))
    t.check("its true heading is the declination off it", abs(wrap_deg(heading_of(s) - t.opt("heading") - deg(expected))) < 3.0,
            "%.1f deg true" % heading_of(s))


@scenario("fa4.frames", group=G4, title="A route in a moving frame", refs="ADR-29 4.21, 4.29",
          summary="A frame moving north at 10 m/s, as a ship would; a route of four points in it, a box 2 km across, flown "
                  "twice: each point placed where the frame is as the aircraft flies to it.",
          options=[aircraft("c172", WINGS)])
def frames(t):
    v = t.spawn(t.opt("aircraft"), 0.0, -3000.0, agl_m=700.0, heading_deg=0.0)
    hold_level(v)
    lat, lon = t.place_rad(2000.0, -3000.0)
    ship = t.world.create_frame(fsim.FrameOrigin.MOVING, latitude_rad=lat, longitude_rad=lon, north_ms=10.0)
    box = [(0.0, 0.0), (2000.0, 0.0), (2000.0, 2000.0), (0.0, 2000.0)]
    a = t.watch(v.submit_route([fsim.Waypoint(0.0, 0.0, frame=ship, frame_x_m=x, frame_y_m=y) for x, y in box], repeat=1),
                "a box in the frame, repeated")
    t.metric("the frame has moved north, m", lambda: (t.world.frame_point(ship)[0] - lat) * R)
    ok = yield t.until(lambda: a.progress is not None and a.progress.laps >= 1, 900.0, "a lap")
    t.check("a lap of the moving box", ok, a.state.name)
    t.check("still flying it", a.live, a.state.name)
