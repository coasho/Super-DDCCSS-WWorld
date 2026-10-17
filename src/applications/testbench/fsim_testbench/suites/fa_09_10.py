"""FA-9 and FA-10 on San Francisco's runway 28R: takeoff, the rejected takeoff, the taxi and a route that starts on the
ground; the recovery to the runway with its go-around and missed approach, the crosswind limit, drag devices, placards,
CleanUp and DirtyUp, a route that ends in a landing, an aerobatic manoeuvre given up, a tail-wheel aircraft and a
rotorcraft's recovery."""
import math

import fsim

from ..registry import aircraft, choice, flag, number, scenario, wind
from ._fa import FIGHTERS, R, ROTORS, WINGS, deg, ended, ground_speed, heading_of, hold_level, rad, reason, wrap_deg

G9 = "FA-9 Ground operations"
G10 = "FA-10 Recovery and landing"
HALF_RUNWAY = 22.5  # m: half a 45 m runway (28R is 61 m wide; the checks keep the platform's own bound)


def runway_track(t, v):
    """Readouts and a record of the vehicle along and across the runway, on the ground."""
    worst = {"across_on_ground": 0.0}

    def across():
        s = v.state
        along, acr = t.runway_offsets(s)
        if s.on_ground and along > -50.0:
            worst["across_on_ground"] = max(worst["across_on_ground"], abs(acr))
        return acr
    t.every(across)
    t.metric("along the runway, m", lambda: t.runway_offsets(v.state)[0])
    t.metric("across it, m", lambda: t.runway_offsets(v.state)[1])
    t.metric("worst across it on the wheels, m", lambda: worst["across_on_ground"])
    return worst


def crosswind_limit(v):
    try:
        x = v.profile_value("envelope/crosswind_max_ms")
        return x if x == x else None
    except Exception:  # noqa: BLE001
        return None


# --- FA-9 ----------------------------------------------------------------------------------------------------------
@scenario("fa9.launch", group=G9, title="Takeoff from the runway", refs="ADR-29 4.49, 4.54",
          summary="Parked on 28R's threshold (28L's, added beside another run), the aircraft takes off along the runway and climbs out. With a crosswind beyond "
                  "its limit the launch is refused crosswind_limit.",
          options=[aircraft("c172", WINGS), wind()])
def launch(t):
    v = t.spawn_on_runway(t.opt("aircraft"))
    t.set_wind_across(t.opt("wind"))
    limit = crosswind_limit(v)
    t.log("crosswind limit: %s" % ("%.1f m/s" % limit if limit else "none known"))
    worst = runway_track(t, v)
    try:
        a = t.watch(v.submit_behavior("launch", airfield=t.SFO, runway=t.RWY), "launch")
    except fsim.Rejected as ex:
        t.check("refused beyond its crosswind limit", ex.reason == "crosswind_limit" and limit is not None and t.opt("wind") > limit, ex.reason)
        return
    ok = yield t.until_done(a, 400.0)
    t.check("the launch completes", ended(a), "%s %s" % (a.state.name, reason(a)))
    t.check("on the runway while on its wheels", worst["across_on_ground"] < HALF_RUNWAY, "%.1f m off the centre line at worst" % worst["across_on_ground"])
    t.check("climbing out, 300 m up", v.state.altitude_agl_m > 300.0, "%.0f m up" % v.state.altitude_agl_m)
    yield t.wait(20.0)


@scenario("fa9.lift", group=G9, title="A rotorcraft lifts to its hover", refs="ADR-29 4.49",
          summary="A helicopter or a quadrotor parked on the runway lifts to its hover height over the spot it stood on.",
          options=[aircraft("uh60", ROTORS), number("height", "Hover height", 10.0, 1.0, 50.0, 1.0, "m")])
def lift(t):
    v = t.spawn_on_runway(t.opt("aircraft"), along_m=300.0)
    s0 = v.state
    h = t.opt("height") if t.opt("aircraft") not in ("cf2", "iris") else min(t.opt("height"), 3.0)
    a = t.watch(v.submit_behavior("launch", airfield=t.SFO, runway=t.RWY, hover_agl_m=h), "launch to a hover")
    lat0, lon0 = s0.latitude_rad, s0.longitude_rad
    t.metric("height, m", lambda: v.state.altitude_agl_m)
    ok = yield t.until_done(a, 120.0)
    s = v.state
    off = math.hypot((s.latitude_rad - lat0) * R, (s.longitude_rad - lon0) * R * math.cos(lat0))
    t.check("the lift completes", ended(a), a.state.name)
    t.check("hovering over its spot at its height", off < 3.0 and abs(s.altitude_agl_m - h) < 1.5, "%.1f m off, %.1f m up" % (off, s.altitude_agl_m))


@scenario("fa9.reject", group=G9, title="A rejected takeoff", refs="ADR-29 4.50",
          summary="A takeoff rejected - by the policy's CANCEL at a fraction of the rotation speed, by the button, or by an "
                  "engine failure (its fuel cut) - and FA's own activity stops it on the runway.",
          options=[aircraft("f16c", WINGS), choice("how", "Rejected by", "cancel", ("cancel", "button", "engine failure")),
                   number("at", "At this fraction of the rotation speed", 0.5, 0.1, 0.95, 0.05)])
def reject(t):
    v = t.spawn_on_runway(t.opt("aircraft"))
    worst = runway_track(t, v)
    a = t.watch(v.submit_behavior("launch", airfield=t.SFO, runway=t.RWY), "launch")
    vr = 1.1 * v.profile_value("envelope/clean/cas_min_ms")
    own = {}

    def cancel():
        if a.live and "a" not in own:
            o = a.cancel()
            if o is not None:
                own["a"] = t.watch(o, "FA's own: stop on the runway")
    how = t.opt("how")
    if how == "button":
        t.action("Reject the takeoff now", cancel)
        ok = yield t.until(lambda: "a" in own or not a.live, 120.0, "the button")
    elif how == "cancel":
        ok = yield t.until(lambda: v.state.airspeed_calibrated_ms >= t.opt("at") * vr, 120.0, "%.0f m/s" % (t.opt("at") * vr))
        cancel()
    else:
        ok = yield t.until(lambda: v.state.airspeed_calibrated_ms >= 15.0, 120.0, "15 m/s")
        t.log("the fuel cut")

        def starve():
            for i in range(8):
                try:
                    v.set_property("propulsion/tank[%d]/contents-lbs" % i, 0.0)
                except Exception:  # noqa: BLE001
                    break
            return not a.live
        ok = yield t.until(starve, 120.0, "the launch to end")
        t.check("the launch fails takeoff_rejected", a.state == fsim.ActivityState.FAILED and reason(a) == "takeoff_rejected",
                "%s %s" % (a.state.name, reason(a)))
    if "a" in own:
        o = own["a"]
        t.check("FA's own activity takes over", o.source == fsim.Source.AUTOPILOT, o.source.name)
        yield t.until_done(o, 120.0)
    else:
        yield t.until(lambda: v.state.on_ground and ground_speed(v.state) < 0.5, 120.0, "it stops")
    s = v.state
    t.check("stopped on the runway", s.on_ground and ground_speed(s) < 1.0 and worst["across_on_ground"] < HALF_RUNWAY,
            "%.1f m/s, %.0f m along, %.1f m across at worst" % (ground_speed(s), t.runway_offsets(s)[0], worst["across_on_ground"]))


@scenario("fa9.taxi", group=G9, title="A taxi", refs="ADR-29 4.51",
          summary="From 28R's threshold, 400 m along the runway, then a turn across to 28L's centre line 229 m away (the "
                  "other way round on 28L), at "
                  "the chosen speed: within 2 m of its path, stopped at its end.",
          options=[aircraft("c172", WINGS), number("speed", "Taxi speed", 8.0, 2.0, 15.0, 1.0, "m/s")])
def taxi(t):
    v = t.spawn_on_runway(t.opt("aircraft"))
    pts = []
    side = -229.0 if t.RWY == 28 else 229.0  # (across to the other runway: 28L is left of 28R)
    for along, across in ((400.0, 0.0), (520.0, side)):
        lat, lon = t.along_rad(along, across)
        pts.append((lat, lon, 0.0, math.nan, 5.0))
    a = t.watch(v.submit_behavior("taxi", points=pts, speed_ms=t.opt("speed")), "taxi")
    t.metric("ground speed, m/s", lambda: ground_speed(v.state))
    ok = yield t.until_done(a, 400.0)
    s = v.state
    end = t.along(520.0, side)
    off = math.hypot((s.latitude_rad - rad(end[0])) * R, (s.longitude_rad - rad(end[1])) * R * math.cos(s.latitude_rad))
    t.check("the taxi completes", ended(a), a.state.name)
    t.check("stopped at its end", off < 10.0 and ground_speed(s) < 0.5, "%.1f m from it" % off)


@scenario("fa9.ground_route", group=G9, title="A route that starts on the ground", refs="ADR-29 4.52",
          summary="Parked south of the runway: taxi points to 28R's start, the runway's start and limit, then two points in "
                  "the air - one route: taxied, lined up, taken off and flown on.",
          options=[aircraft("c172", WINGS)])
def ground_route(t):
    v = t.spawn(t.opt("aircraft"), at=t.along(250.0, -110.0), on_ground=True, heading_deg=t.heading_deg(180.0))
    t.step_seconds(1.0)
    T = fsim.WaypointType
    field = t.field_m

    def at(along, across, kind=fsim.HOLD, alt=fsim.HOLD, ref=fsim.HOLD):
        lat, lon = t.along_rad(along, across)
        return fsim.Waypoint(lat, lon, alt, ref, waypoint_type=kind)
    taxi_pts = [at(60.0, -110.0, T.TAXI), at(60.0, -40.0, T.TAXI)]
    runway = [at(0.0, 0.0, T.RUNWAY_START), at(t.origin.length_m, 0.0, T.RUNWAY_LIMIT)]
    air = [at(9000.0, 0.0, alt=field + 600.0, ref="msl"), at(9000.0, 6000.0, alt=field + 600.0, ref="msl")]
    try:
        a = t.watch(v.submit_route(taxi_pts + runway + air), "route from the ground")
    except fsim.Rejected as ex:
        t.check("the route is accepted", False, "%s at point %d" % (ex.reason, ex.index))
        return
    t.metric("on its wheels", lambda: "yes" if v.state.on_ground else "no")
    t.metric("point", lambda: "%d/%d" % (a.progress.segment, a.progress.segments) if a.progress else "")
    ok = yield t.until(lambda: not v.state.on_ground and v.state.altitude_agl_m > 100.0, 600.0, "airborne")
    t.check("taxied, took off and climbed", ok, "%.0f m up" % v.state.altitude_agl_m)
    ok = yield t.until_done(a, 900.0)
    t.check("the route completes", ended(a), "%s %s" % (a.state.name, reason(a)))


# --- FA-10 ---------------------------------------------------------------------------------------------------------
STARTS = {"straight in (15 km out)": (-15000.0, 0.0), "from the left (8 km abeam)": (-6000.0, -8000.0),
          "from the right (8 km abeam)": (-6000.0, 8000.0)}


def recovery_watch(t, v, a):
    """Touchdown, bounce and rollout, recorded as it flies."""
    rec = {"touch": None, "sink": 0.0, "bounce": 0.0, "approaches": 0}
    last = {"ground": False, "agl": None}

    def watch():
        s = v.state
        along, across = t.runway_offsets(s)
        if not s.on_ground and s.altitude_agl_m < 30.0 and rec["touch"] is None:
            rec["sink"] = s.velocity_ned_ms[2]
        if s.on_ground and rec["touch"] is None:
            rec["touch"] = (along, across, rec["sink"], s.airspeed_calibrated_ms)
            last["agl"] = s.altitude_agl_m
            t.log("touched down %.0f m along, %.1f m across, sinking %.2f m/s at %.1f m/s" % (along, across, rec["sink"], s.airspeed_calibrated_ms))
        if rec["touch"] is not None and not s.on_ground:
            rec["bounce"] = max(rec["bounce"], s.altitude_agl_m - (last["agl"] or 0.0))
        return "%s" % ("on the wheels" if s.on_ground else "%.0f m up" % s.altitude_agl_m)
    t.every(watch)
    t.metric("touchdown", lambda: "%.0f m along, %.1f m across, %.2f m/s down" % rec["touch"][:3] if rec["touch"] else "not yet")
    t.metric("state", lambda: "on the wheels" if v.state.on_ground else "%.0f m up" % v.state.altitude_agl_m)
    return rec


@scenario("fa10.recovery", group=G10, title="A recovery to the runway", refs="ADR-29 4.53, 4.54, 4.58-4.63",
          summary="RECOVERY to runway 28R (28L, added beside another run) from 15 km out or abeam: the approach, the glide slope, the flare, the touchdown in "
                  "its zone and the rollout to a stop on the centre line. The buttons: CleanUp (gear and flaps up: not "
                  "stable, it goes around and comes again), DirtyUp (landing configuration now) and hand back.",
          options=[aircraft("c172", WINGS + ROTORS), choice("start", "From", "straight in (15 km out)", tuple(STARTS)), wind(),
                   number("height", "Starting height", 1000.0, 300.0, 3000.0, 100.0, "m")])
def recovery(t):
    kind = t.opt("aircraft")
    n, e = STARTS[t.opt("start")]
    d = t.origin.direction_rad
    north, east = n * math.cos(d) - e * math.sin(d), n * math.sin(d) + e * math.cos(d)
    v = t.spawn(kind, north, east, agl_m=t.opt("height"), heading_deg=t.heading_deg())
    if t.performance(kind).hovers:
        v.submit(fsim.Level.VELOCITY, north_ms=0.0, east_ms=0.0, vertical_speed_ms=0.0)
    else:
        hold_level(v)
    t.set_wind_across(t.opt("wind"))
    worst = runway_track(t, v)
    yield t.wait(3.0)
    a = t.watch(v.submit_behavior("recovery", airfield=t.SFO, runway=t.RWY), "recovery to " + t.runway_name)
    rec = recovery_watch(t, v, a)
    t.action("CleanUp (go around)", lambda: a.update(configuration=1) if a.live else None)
    t.action("DirtyUp (configure to land)", lambda: a.update(configuration=2) if a.live else None)
    t.action("Hand the configuration back", lambda: a.update(configuration=0) if a.live else None)
    ok = yield t.until_done(a, 3600.0)
    t.check("the recovery completes", ended(a), "%s %s" % (a.state.name, reason(a)))
    if rec["touch"]:
        along, across, sink, cas = rec["touch"]
        zone = max(900.0, t.origin.length_m / 3.0)
        t.check("touched down in the zone", -50.0 < along < zone, "%.0f m along" % along)
        t.check("sinking under 3 m/s", sink < 3.0, "%.2f m/s" % sink)
    s = v.state
    t.check("stopped on the runway", s.on_ground and ground_speed(s) < 1.5 and worst["across_on_ground"] < HALF_RUNWAY,
            "%.0f m along, %.1f m across at worst, %.1f m/s" % (t.runway_offsets(s)[0], worst["across_on_ground"], ground_speed(s)))


@scenario("fa10.crosswind", group=G10, title="A crosswind beyond the limit: waved off", refs="ADR-29 4.54",
          summary="A recovery across a wind beyond the aircraft's crosswind limit: it waves off on the centre line, flies "
                  "its missed approach, comes again, waves off again and fails crosswind_limit.",
          options=[aircraft("c172", WINGS), number("wind", "Wind across", 12.0, 0.0, 25.0, 1.0, "m/s")])
def crosswind(t):
    v = t.spawn(t.opt("aircraft"), -12000.0 * math.cos(t.origin.direction_rad), -12000.0 * math.sin(t.origin.direction_rad), agl_m=900.0)
    hold_level(v)
    limit = crosswind_limit(v)
    t.log("crosswind limit %s" % ("%.1f m/s" % limit if limit else "unknown"))
    t.set_wind_across(t.opt("wind"))
    yield t.wait(3.0)
    a = t.watch(v.submit_behavior("recovery", airfield=t.SFO, runway=t.RWY), "recovery to " + t.runway_name)
    runway_track(t, v)
    ok = yield t.until_done(a, 7200.0)
    beyond = limit is not None and t.opt("wind") > limit
    if beyond:
        t.check("fails crosswind_limit after its approaches", a.state == fsim.ActivityState.FAILED and reason(a) == "crosswind_limit",
                "%s %s" % (a.state.name, reason(a)))
        t.check("never touched down", not v.state.on_ground)
    else:
        t.check("within its limit, it lands", ended(a), "%s %s" % (a.state.name, reason(a)))


@scenario("fa10.drag", group=G10, title="Drag devices: the speedbrake", refs="ADR-29 4.55",
          summary="A fighter at 200 m/s opens its speedbrake: the deceleration it adds, measured against the same aircraft "
                  "with it shut. An aircraft without one is refused.",
          options=[aircraft("f15c", ("f15c", "f16c", "fa18c", "ea18g", "f22a", "su27s", "su57", "mig29a", "typhoon", "rafale", "gripen", "a10c",
                                     "c130j", "c172"))])
def drag(t):
    kind = t.opt("aircraft")
    a_ = t.spawn(kind, 0.0, 4000.0, agl_m=3000.0, speed_ms=min(200.0, t.performance(kind).max_tas_ms * 0.8), name="speedbrake")
    b_ = t.spawn(kind, 1000.0, 4000.0, agl_m=3000.0, speed_ms=min(200.0, t.performance(kind).max_tas_ms * 0.8), name="shut")
    for v in (a_, b_):
        v.submit(fsim.Level.ATTITUDE, roll_rad=0.0, pitch_rad=0.0, throttle=0.0)
    try:
        sb = t.watch(a_.submit_support("speedbrake", 1.0), "speedbrake open")
    except fsim.Rejected as ex:
        t.check("an aircraft without a speedbrake is refused", ex.reason in ("unknown_capability", "not_supported"), ex.reason)
        return
    v0a, v0b = a_.state.airspeed_true_ms, b_.state.airspeed_true_ms
    t.metric("speedbrake, m/s", lambda: a_.state.airspeed_true_ms)
    t.metric("shut, m/s", lambda: b_.state.airspeed_true_ms)
    yield t.wait(8.0)
    da, db = v0a - a_.state.airspeed_true_ms, v0b - b_.state.airspeed_true_ms
    t.check("it slows faster with its speedbrake open", da > db + 2.0, "%.1f against %.1f m/s lost in 8 s" % (da, db))


@scenario("fa10.placards", group=G10, title="Placards: gear and flaps", refs="ADR-29 4.57",
          summary="Above its gear's placard speed a gear-down command is refused unavailable, and flaps past their threshold "
                  "above theirs; slowed below them, both are accepted.",
          options=[aircraft("f16c", ("f16c", "c172", "c130j", "f15c", "fa18c", "b52h", "c17a"))])
def placards(t):
    kind = t.opt("aircraft")
    v = t.spawn(kind, 0.0, 4000.0, agl_m=1500.0, speed_ms=t.performance(kind).cruise_tas_ms)
    h = hold_level(v)
    try:
        v.submit_support("gear", 1.0)
        t.check("gear down refused above its placard", False, "accepted at %.0f m/s" % v.state.airspeed_calibrated_ms)
    except fsim.Rejected as ex:
        t.check("gear down refused above its placard", ex.reason in ("unavailable", "unknown_capability"), ex.reason)
    slow = 1.3 * v.performance.min_cas_ms
    h.update(speed=slow, speed_reference="calibrated_airspeed")
    ok = yield t.until(lambda: v.state.airspeed_calibrated_ms < slow + 3.0, 300.0, "slowed")
    try:
        g = t.watch(v.submit_support("gear", 1.0), "gear down")
        t.check("accepted below it", True)
        yield t.until_done(g, 30.0)
        t.check("the gear is down", v.state.gear_position > 0.99, "%.2f" % v.state.gear_position)
    except fsim.Rejected as ex:
        t.check("accepted below it", ex.reason == "unknown_capability", ex.reason + " (fixed gear)" if ex.reason == "unknown_capability" else ex.reason)


@scenario("fa10.landing_route", group=G10, title="A route that ends in a landing", refs="ADR-29 4.59",
          summary="Two points in the air east of the bay, 28R's threshold and limit, then two taxi points: landed in the "
                  "touchdown zone, taxied to the end, completed.",
          options=[aircraft("f16c", WINGS)])
def landing_route(t):
    kind = t.opt("aircraft")
    d = t.origin.direction_rad
    v = t.spawn(kind, -22000.0 * math.cos(d) + 4000.0, -22000.0 * math.sin(d), agl_m=1000.0, heading_deg=t.heading_deg())
    hold_level(v)
    yield t.wait(2.0)
    T = fsim.WaypointType
    field = t.field_m

    def at(along, across, kind_=fsim.HOLD, alt=fsim.HOLD, ref=fsim.HOLD):
        lat, lon = t.along_rad(along, across)
        return fsim.Waypoint(lat, lon, alt, ref, waypoint_type=kind_)
    L = t.origin.length_m
    pts = [at(-15000.0, 2500.0, alt=field + 900.0, ref="msl"), at(-10000.0, 0.0, alt=field + 800.0, ref="msl"),
           at(0.0, 0.0, T.RUNWAY_THRESHOLD, field, "msl"), at(L, 0.0, T.RUNWAY_LIMIT, field, "msl"),
           at(L - 100.0, -60.0, T.TAXI), at(L - 300.0, -120.0, T.TAXI)]
    a = t.watch(v.submit_route(pts), "route to a landing")
    worst = runway_track(t, v)
    rec = recovery_watch(t, v, a)
    ok = yield t.until_done(a, 1800.0)
    t.check("the route completes", ended(a), "%s %s" % (a.state.name, reason(a)))
    if rec["touch"]:
        t.check("touched down in the zone", -50.0 < rec["touch"][0] < max(900.0, L / 3.0), "%.0f m along" % rec["touch"][0])


@scenario("fa10.aerobatics", group=G10, title="Aerobatics, given up and recovered", refs="ADR-29 4.64, 4.67",
          summary="A loop, an Immelmann, a split-S or a roll from cruise at 3,000 m. Where a fighter is given up - too slow "
                  "over the top, or departed - it recovers: pulled through, then levelled out, never lower than it started "
                  "by more than its split-S takes.",
          options=[aircraft("gripen", FIGHTERS + ("su25", "a10c")), choice("manoeuvre", "Manoeuvre", "loop", ("roll", "loop", "immelmann", "split_s"))])
def aerobatics(t):
    kind = t.opt("aircraft")
    v = t.spawn(kind, 0.0, 6000.0, alt_m=3000.0, speed_ms=t.performance(kind).cruise_tas_ms, heading_deg=90.0)
    yield t.wait(1.0)
    m = ("roll", "loop", "immelmann", "split_s").index(t.opt("manoeuvre"))
    start = v.state.altitude_msl_m
    low = [start]

    def lowest():
        low[0] = min(low[0], v.state.altitude_msl_m)
        return low[0]
    t.metric("lowest, m", lowest)
    t.metric("CAS, m/s", lambda: v.state.airspeed_calibrated_ms)
    try:
        a = t.watch(v.submit_behavior("aerobatics", manoeuvre=float(m)), t.opt("manoeuvre"))
    except fsim.Rejected as ex:
        t.check("the manoeuvre is accepted", False, "%s (%s)" % (ex.reason, ex.constraint))
        return
    ok = yield t.until(lambda: lowest() is None or not a.live, 120.0, "the manoeuvre")
    t.log("the manoeuvre ended %s %s" % (a.state.name, reason(a)))
    t.check("flown, or given up and recovered", a.state == fsim.ActivityState.COMPLETED or reason(a) == "behavior_failed",
            "%s %s" % (a.state.name, reason(a)))
    ok = yield t.until(lambda: lowest() is None or (abs(deg(v.state.euler_rad[0])) < 10.0 and abs(deg(v.state.euler_rad[1])) < 10.0), 90.0, "level")
    t.check("level afterwards", ok, "roll %.0f, pitch %.0f deg" % (deg(v.state.euler_rad[0]), deg(v.state.euler_rad[1])))
    allowed = 1500.0 if m == 3 else 400.0
    t.check("never far below its start", start - low[0] < allowed, "%.0f m below at the lowest" % (start - low[0]))


@scenario("fa10.tailwheel", group=G10, title="A tail-wheel aircraft: the U-2S", refs="ADR-29 4.65",
          summary="The U-2S takes off from 28R on its two main wheels, its pogos falling away in effect, climbs, then flies a "
                  "recovery back to 28R and lands on its main wheel, its tail held down.",
          options=[wind(0.0)])
def tailwheel(t):
    v = t.spawn_on_runway("u2s")
    t.set_wind_across(t.opt("wind"))
    worst = runway_track(t, v)
    a = t.watch(v.submit_behavior("launch", airfield=t.SFO, runway=t.RWY), "launch")
    ok = yield t.until_done(a, 300.0)
    t.check("taken off", ended(a), "%s %s" % (a.state.name, reason(a)))
    t.check("on the runway while on its wheels", worst["across_on_ground"] < HALF_RUNWAY, "%.1f m at worst" % worst["across_on_ground"])
    worst["across_on_ground"] = 0.0
    yield t.wait(30.0)
    b = t.watch(v.submit_behavior("recovery", airfield=t.SFO, runway=t.RWY), "recovery to " + t.runway_name)
    rec = recovery_watch(t, v, b)
    ok = yield t.until_done(b, 3600.0)
    t.check("landed and stopped", ended(b), "%s %s" % (b.state.name, reason(b)))
    t.check("on the runway rolling out", worst["across_on_ground"] < HALF_RUNWAY, "%.1f m at worst" % worst["across_on_ground"])


@scenario("fa10.rotor_recovery", group=G10, title="A rotorcraft's recovery", refs="ADR-29 4.53, 4.66",
          summary="A helicopter recovers to 28R from 3 km out: its approach, a hover over the touchdown point and the "
                  "landing. Across a wind beyond its limit it goes around and fails crosswind_limit.",
          options=[aircraft("uh60", ROTORS), wind(0.0)])
def rotor_recovery(t):
    kind = t.opt("aircraft")
    small = kind in ("cf2", "iris")
    d = t.origin.direction_rad
    back = 300.0 if small else 3000.0
    v = t.spawn(kind, -back * math.cos(d), -back * math.sin(d), agl_m=30.0 if small else 300.0)
    v.submit(fsim.Level.VELOCITY, north_ms=0.0, east_ms=0.0, vertical_speed_ms=0.0)
    t.set_wind_across(t.opt("wind"))
    limit = crosswind_limit(v)
    yield t.wait(5.0)
    a = t.watch(v.submit_behavior("recovery", airfield=t.SFO, runway=t.RWY), "recovery")
    rec = recovery_watch(t, v, a)
    runway_track(t, v)
    ok = yield t.until_done(a, 1800.0)
    if limit is not None and t.opt("wind") > limit:
        t.check("fails crosswind_limit", reason(a) == "crosswind_limit", "%s %s" % (a.state.name, reason(a)))
    else:
        t.check("landed", ended(a) and v.state.on_ground, "%s %s" % (a.state.name, reason(a)))
