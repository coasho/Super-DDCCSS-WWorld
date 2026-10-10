"""FA-5 and FA-6: patterns (orbit, racetrack, figure eight, hold, hover, in a frame), curves (Bezier and NURBS), and
routes with everything A-GRA's waypoint carries: turn points, loiters, segment performance, times of arrival, planned
states, RNP, paths, branches and civil path terminators. They fly over the bay east of the runway."""
import math

import fsim

from ..registry import aircraft, choice, number, scenario
from ._fa import R, ROTORS, WINGS, deg, distance_m, ended, heading_of, hold_level, offset_rad, rad, track_of, wrap_deg

G5 = "FA-5 Patterns and curves"
G6 = "FA-6 Routes"
UP = 1200.0  # m above the ground: clear of the peninsula's hills


def start(t, kind, heading=0.0, east=4000.0, north=-3000.0, up=UP):
    """The aircraft in the air over the bay, holding its heading; the scenario's geometry scaled to it (t.scale)."""
    S = t.fit(kind)
    v = t.spawn(kind, north * S, east, agl_m=up, heading_deg=heading)
    hold_level(v)
    return v


def here(v):
    s = v.state
    return s.latitude_rad, s.longitude_rad


def east_of(lat, m):
    return m / (R * math.cos(lat))


# --- FA-5 ----------------------------------------------------------------------------------------------------------
@scenario("fa5.orbit", group=G5, title="An orbit: laps and an exit point", refs="ADR-29 4.23",
          summary="An orbit of the chosen radius 2 km east of the aircraft, flown the chosen number of laps, then left at its "
                  "exit point (its east) out along its course. Change the radius while it flies.",
          options=[aircraft("c172", WINGS + ROTORS), number("radius", "Radius", 800.0, 300.0, 5000.0, 100.0, "m"),
                   number("laps", "Laps", 1.0, 1.0, 5.0, 1.0), choice("turn", "Turns", "right", ("right", "left"))])
def orbit(t):
    v = start(t, t.opt("aircraft"))
    yield t.wait(2.0)
    lat, lon = here(v)
    S = t.scale
    r = max(t.opt("radius"), 1.15 * t.turn_radius_m(t.opt("aircraft")))
    if r > t.opt("radius"):
        t.log("radius %.0f m: the %s turns in %.0f m" % (r, t.opt("aircraft"), t.turn_radius_m(t.opt("aircraft"))))
    centre = (lat, lon + east_of(lat, 2000.0 * S))
    exit_point = (centre[0], centre[1] + east_of(lat, r))
    a = t.watch(v.submit_pattern(latitude_rad=centre[0], longitude_rad=centre[1], radius_m=r, orbits=int(t.opt("laps")),
                                 clockwise=t.opt("turn") == "right", exit_latitude_rad=exit_point[0], exit_longitude_rad=exit_point[1]),
                "orbit")
    t.metric("distance from the centre, m", lambda: distance_m(centre[0], centre[1], *here(v)))
    t.metric("laps", lambda: a.progress.laps if a.progress else float("nan"))
    t.action("Radius +300 m", lambda: a.update(radius_m=a.setpoint().kwargs["radius_m"] + 300.0) if a.live else None)
    ok = yield t.until_done(a, 300.0 + 600.0 * t.opt("laps") * max(1.0, r / 800.0))
    t.check("the orbit completes its laps", ended(a) and a.progress.laps == int(t.opt("laps")), "%s, %s laps" % (a.state.name, a.progress.laps))
    off = distance_m(exit_point[0], exit_point[1], *here(v))
    t.check("it leaves at its exit point", off < 200.0, "%.0f m from it" % off)


@scenario("fa5.racetrack", group=G5, title="A racetrack and a figure eight", refs="ADR-29 4.23",
          summary="A racetrack between two circles of different radii, then a figure eight on the same circles: the shape "
                  "taken from its circles.",
          options=[aircraft("c172", WINGS), choice("shape", "Shape", "racetrack", ("racetrack", "figure_eight"))])
def racetrack(t):
    v = start(t, t.opt("aircraft"))
    yield t.wait(2.0)
    lat, lon = here(v)
    S = t.scale
    c1 = (lat, lon + east_of(lat, 3000.0 * S))
    c2 = (lat + 4000.0 * S / R, lon + east_of(lat, 3000.0 * S))
    a = t.watch(v.submit_pattern(pattern=t.opt("shape"), latitude_rad=c1[0], longitude_rad=c1[1], radius_m=900.0 * S, latitude2_rad=c2[0],
                                 longitude2_rad=c2[1], radius2_m=1200.0 * S, orbits=2), t.opt("shape"))
    t.metric("laps", lambda: a.progress.laps if a.progress else float("nan"))
    t.metric("leg", lambda: "%d/%d" % (a.progress.segment, a.progress.segments) if a.progress else "")
    # (two laps of a C172's racetrack take some 10 minutes; one drawn S times as big, flown at its own speed, longer)
    ok = yield t.until_done(a, 1800.0 * max(1.0, S * 50.0 / max(50.0, v.state.airspeed_true_ms)))
    t.check("two laps of the %s" % t.opt("shape").replace("_", " "), ended(a), a.state.name)


@scenario("fa5.hold", group=G5, title="A hold: entry, legs and turns", refs="ADR-29 4.24",
          summary="A hold at a fix 5 km east, entered as chosen (or as ATC's rules say), its inbound course north, its legs a "
                  "minute long, flown for 6 minutes.",
          options=[aircraft("c172", WINGS), choice("entry", "Entry", "atc", ("atc", "direct", "parallel", "teardrop")),
                   choice("turn", "Turns", "right", ("right", "left"))])
def hold(t):
    v = start(t, t.opt("aircraft"), heading=90.0)
    yield t.wait(2.0)
    lat, lon = here(v)
    fix = (lat, lon + east_of(lat, 5000.0 * t.scale))
    fields = dict(pattern="hold", latitude_rad=fix[0], longitude_rad=fix[1], course_rad=0.0, clockwise=t.opt("turn") == "right",
                  duration_s=360.0)
    if t.opt("entry") == "atc":
        fields["hold_context"] = "atc"
    else:
        fields["hold_entry"] = t.opt("entry")
    a = t.watch(v.submit_pattern(**fields), "hold")
    t.metric("distance from the fix, m", lambda: distance_m(fix[0], fix[1], *here(v)))
    t.metric("leg", lambda: "%d/%d" % (a.progress.segment, a.progress.segments) if a.progress else "")
    yield t.wait(5.0)
    kw = a.setpoint().kwargs
    t.log("hold: radius %.0f m, legs %.0f m, entry %s" % (kw["radius_m"], kw["leg_m"], kw.get("hold_entry")))
    ok = yield t.until_done(a, 900.0)
    t.check("the hold is flown for its 6 minutes", ended(a), a.state.name)


@scenario("fa5.hover", group=G5, title="A rotorcraft's hover over a point", refs="ADR-29 4.25",
          summary="A helicopter or a quadrotor flies to a point 300 m ahead and 20 m higher and hovers over it for a minute.",
          options=[aircraft("uh60", ROTORS)])
def hover(t):
    kind = t.opt("aircraft")
    small = kind in ("cf2", "iris")
    v = t.spawn(kind, 0.0, 4000.0, agl_m=30.0 if small else 150.0, heading_deg=0.0)
    still = v.submit(fsim.Level.VELOCITY, north_ms=0.0, east_ms=0.0, vertical_speed_ms=0.0)
    yield t.wait(10.0)
    lat, lon = here(v)
    target = (lat + (40.0 if small else 300.0) / R, lon)
    alt = v.state.altitude_msl_m + (3.0 if small else 20.0)
    a = t.watch(v.submit_pattern(pattern="hover", latitude_rad=target[0], longitude_rad=target[1], altitude_m=alt, duration_s=60.0), "hover")
    t.metric("from the point, m", lambda: distance_m(target[0], target[1], *here(v)))
    t.metric("height error, m", lambda: v.state.altitude_msl_m - alt)
    ok = yield t.until_done(a, 240.0)
    off = distance_m(target[0], target[1], *here(v))
    t.check("the hover completes", ended(a), a.state.name)
    t.check("over its point", off < (2.0 if small else 15.0), "%.1f m off, %.1f m high" % (off, v.state.altitude_msl_m - alt))


@scenario("fa5.frame", group=G5, title="An orbit round a moving ship", refs="ADR-29 4.21, 4.25",
          summary="A frame moving north-east at 8.5 m/s, as a ship would, turned with its heading; an orbit of 1 km round a "
                  "point 500 m ahead of it, carried as it moves.",
          options=[aircraft("c172", WINGS)])
def frame(t):
    v = start(t, t.opt("aircraft"))
    yield t.wait(2.0)
    lat, lon = here(v)
    S = t.scale
    ship = t.world.create_frame("moving", latitude_rad=lat, longitude_rad=lon + east_of(lat, 3000.0 * S), yaw_rad=math.pi / 4.0, north_ms=6.0,
                                east_ms=6.0, time_s=t.world.time)
    a = t.watch(v.submit_pattern(radius_m=1000.0 * S, frame=ship, frame_rotation="yaw", frame_offsets="cartesian", frame_x_m=500.0), "orbit in the frame")

    def from_centre():
        clat, clon, _ = t.world.frame_point(ship, 500.0, rotation="yaw")
        return distance_m(clat, clon, *here(v))
    t.metric("from the moving centre, m", from_centre)
    yield t.wait(240.0)
    t.check("still orbiting the moving point", a.live and abs(from_centre() - 1000.0 * S) < 250.0 * S, "%.0f m from it" % from_centre())


def s_curve(n0, length, swing):
    """A quintic Bezier S north from n0 (metres from the curve's reference)."""
    return fsim.BezierSegment(north=[n0 + length * k / 5.0 for k in range(6)], east=[0.0, 0.0, swing, -swing, 0.0, 0.0], down=[0.0] * 6)


def nurbs_s(n0, length, swing):
    L = length
    return fsim.NurbsSegment(north=[n0, n0 + 0.15 * L, n0 + 0.3 * L, n0 + 0.5 * L, n0 + 0.7 * L, n0 + 0.85 * L, n0 + L],
                             east=[0.0, 0.0, swing, swing, -swing, 0.0, 0.0], down=[0.0] * 7,
                             knots=[0.0, 0.0, 0.0, 0.0, 0.2, 0.45, 0.7, 1.0, 1.0, 1.0, 1.0], weights=[1.0, 1.4, 0.8, 1.0, 1.25, 0.9, 1.0])


@scenario("fa5.curve", group=G5, title="A curve: Bezier or NURBS segments", refs="ADR-29 4.26-4.28; VI 4.7",
          summary="Two S-shaped segments north of the aircraft, as quintic Beziers or as A-GRA's rational B-splines, flown "
                  "in a speed band; at the end, a circular loiter or straight on. Append a segment while it flies.",
          options=[aircraft("c172", WINGS + ROTORS), choice("form", "Segments", "bezier", ("bezier", "nurbs")),
                   choice("end", "At its end", "loiter", ("loiter", "continue")), number("swing", "Swing", 600.0, 0.0, 1500.0, 100.0, "m")])
def curve(t):
    v = start(t, t.opt("aircraft"), heading=0.0)
    yield t.wait(2.0)
    make = s_curve if t.opt("form") == "bezier" else nurbs_s
    S = t.scale
    w = t.opt("swing") * S
    segs = [make(0.0, 6000.0 * S, w), make(6000.0 * S, 6000.0 * S, -w)]
    a = t.watch(v.submit_curve(segs, end=t.opt("end")), "curve")
    t.metric("segment", lambda: "%d/%d" % (a.progress.segment, a.progress.segments) if a.progress else "")
    t.metric("cross-track, m", lambda: a.progress.cross_track_m if a.progress else float("nan"))
    t.action("Append a segment", lambda: a.append([make(12000.0 * S, 6000.0 * S, w)]) if a.live else None)
    worst = [0.0]

    def track():
        p = a.progress
        if p is not None and a.state == fsim.ActivityState.ACTIVE and p.segment >= 1 and 5.0 < (p.percent or 0.0) < 95.0:
            worst[0] = max(worst[0], abs(p.cross_track_m))
        return False
    ok = yield t.until(lambda: track() or not a.live or (a.progress and a.progress.percent >= 99.9), 900.0, "its end")
    t.check("the curve is flown to its end", ok and a.state in (fsim.ActivityState.ACTIVE, fsim.ActivityState.COMPLETED), a.state.name)
    t.check("within %.0f m of it" % (150.0 * S), worst[0] < 150.0 * S, "%.0f m at the worst" % worst[0])
    yield t.wait(60.0)


# --- FA-6 ----------------------------------------------------------------------------------------------------------
@scenario("fa6.route", group=G6, title="A route: fly-by and fly-over points", refs="ADR-29 4.29; VI 4.5",
          summary="Five waypoints round a box over the bay, passed fly-by (turning early) or fly-over, optionally repeated. "
                  "The progress names the point flown to.",
          options=[aircraft("c172", WINGS + ROTORS), choice("turn", "Turns", "fly_by", ("fly_by", "fly_over")),
                   choice("repeat", "Repeat", "no", ("no", "yes")), number("size", "Box size", 4000.0, 1500.0, 15000.0, 500.0, "m")])
def route(t):
    v = start(t, t.opt("aircraft"), heading=90.0)
    yield t.wait(2.0)
    lat, lon = here(v)
    d = t.opt("size") * t.scale
    box = [(0, d), (d, d), (d, 0), (0.5 * d, -0.3 * d), (0, 0)]
    pts = [fsim.Waypoint(*offset_rad(lat, lon, n, e), turn=t.opt("turn"), id=10 + i) for i, (n, e) in enumerate(box)]
    a = t.watch(v.submit_route(pts, repeat=t.opt("repeat") == "yes"), "route")
    t.metric("to point id", lambda: a.progress.segment_id if a.progress else float("nan"))
    if t.opt("repeat") == "yes":
        ok = yield t.until(lambda: a.progress and a.progress.laps >= 1, 2400.0, "a lap")
        t.check("a lap of the repeated route", ok)
    else:
        ok = yield t.until_done(a, 2400.0)
        t.check("the route is flown to its last point", ended(a), a.state.name)


@scenario("fa6.turnpoints", group=G6, title="Turn points: an arc and a course capture", refs="ADR-29 4.30",
          summary="A start-turn and end-turn pair make a quarter circle of the given radius; then a point captured on its "
                  "outbound course.",
          options=[aircraft("c172", WINGS), number("radius", "Arc radius", 1500.0, 500.0, 5000.0, 100.0, "m")])
def turnpoints(t):
    v = start(t, t.opt("aircraft"), heading=0.0)
    yield t.wait(2.0)
    lat, lon = here(v)
    S = t.scale
    r = max(t.opt("radius"), 1.15 * t.turn_radius_m(t.opt("aircraft")))
    e = lambda m: east_of(lat, m)  # noqa: E731
    pts = [fsim.Waypoint(lat + 3000.0 * S / R, lon, turn="start_turn", turn_radius_m=r),
           fsim.Waypoint(lat + (3000.0 * S + r) / R, lon + e(r), turn=fsim.TurnType.END_TURN, course_rad=0.5 * math.pi),
           fsim.Waypoint(lat + (3000.0 * S + r) / R, lon + e(r + 3000.0 * S), turn="capture_outbound_course", course_rad=0.5 * math.pi),
           fsim.Waypoint(lat + (3000.0 * S + r) / R, lon + e(r + 7000.0 * S))]
    a = t.watch(v.submit_route(pts), "route with an arc")
    centre = (lat + 3000.0 * S / R, lon + e(r))
    worst = [0.0]

    def arc_error():
        p = a.progress
        if p and p.segment == 1 and 10.0 < p.segment_percent < 90.0:
            worst[0] = max(worst[0], abs(distance_m(centre[0], centre[1], *here(v)) - r))
        return worst[0]
    t.metric("worst off the arc, m", arc_error)
    ok = yield t.until(lambda: arc_error() is None or not a.live, 1200.0, "its end")
    t.check("the route completes", ended(a), a.state.name)
    t.check("the arc flown within 100 m of its radius", worst[0] < 100.0, "%.0f m" % worst[0])


@scenario("fa6.loiters", group=G6, title="A loiter point inside a route", refs="ADR-29 4.31",
          summary="North 3 km, then twice round an orbit at the second point, then 3 km east: the progress tells the loiter's "
                  "time to go while it orbits.",
          options=[aircraft("c172", WINGS), choice("pattern", "Loiter", "orbit", ("orbit", "racetrack", "figure_eight", "hold")),
                   number("laps", "Laps", 2.0, 1.0, 4.0, 1.0)])
def loiters(t):
    v = start(t, t.opt("aircraft"), heading=0.0)
    yield t.wait(2.0)
    lat, lon = here(v)
    S = t.scale
    pts = [fsim.Waypoint(lat + 3000.0 * S / R, lon), fsim.Waypoint(lat + 8000.0 * S / R, lon, kind="loiter_point"),
           fsim.Waypoint(lat + 8000.0 * S / R, lon + east_of(lat, 3000.0 * S))]
    loiter = fsim.RouteLoiter(1, pattern=t.opt("pattern"), orbits=int(t.opt("laps")))
    if t.opt("pattern") == "hold":
        loiter = fsim.RouteLoiter(1, pattern="hold", duration_s=240.0)
    a = t.watch(v.submit_route(pts, loiters=[loiter]), "route with a loiter")
    t.metric("time to go, s", lambda: a.progress.time_to_go_s if a.progress else float("nan"))
    t.metric("segment", lambda: "%d/%d %.0f %%" % (a.progress.segment, a.progress.segments, a.progress.segment_percent) if a.progress else "")
    ok = yield t.until_done(a, 2400.0)
    t.check("the route with its loiter completes", ended(a), a.state.name)


@scenario("fa6.segments", group=G6, title="Segment performance: best range speed and a climb", refs="ADR-29 4.32",
          summary="East 3 km at 50 m/s, then 13 km at the performance tables' best range speed reached at 0.1 m/s^2, climbing "
                  "300 m at the best rate.",
          options=[aircraft("c172", ("c172", "c130j", "f16c", "rq4b", "b52h", "kc135r"))])
def segments(t):
    v = start(t, t.opt("aircraft"), heading=90.0)
    yield t.wait(2.0)
    lat, lon = here(v)
    alt = v.state.altitude_msl_m
    first_speed = max(50.0, t.cruise_ms(t.opt("aircraft")))
    S = t.scale
    pts = [fsim.Waypoint(lat, lon + east_of(lat, 3000.0 * S), speed=first_speed),
           fsim.Waypoint(lat, lon + east_of(lat, 16000.0 * S), altitude_m=alt + 300.0, speed_optimization="long_range_cruise",
                         acceleration_ms2=0.1, climb_optimization="best_rate")]
    a = t.watch(v.submit_route(pts), "route")
    best = a.setpoint().args[0][1].speed
    t.log("the best range speed now: %.1f m/s" % best)
    # (0.1 m/s2 takes a C172 to its best range speed well within the segment; a jet's is far faster than it starts at:
    # flown again at an acceleration that gets it there in the first half of the segment)
    need = (best * best - first_speed * first_speed) / (2.0 * 0.5 * 13000.0 * S)
    if need > 0.1:
        t.log("from %.0f m/s it needs %.2f m/s2 to reach it in the segment" % (first_speed, need))
        a.cancel()
        pts[1] = fsim.Waypoint(lat, lon + east_of(lat, 16000.0 * S), altitude_m=alt + 300.0, speed_optimization="long_range_cruise",
                               acceleration_ms2=need, climb_optimization="best_rate")
        a = t.watch(v.submit_route(pts), "route, accelerating at %.2f m/s2" % need)
    t.metric("speed it asks for, m/s", lambda: a.progress.speed_ms if a.progress else float("nan"))
    t.metric("true airspeed, m/s", lambda: v.state.airspeed_true_ms)
    ok = yield t.until_done(a, 900.0)
    t.check("the route completes", ended(a), a.state.name)
    t.check("at the best range speed", abs(v.state.airspeed_true_ms - best) < 4.0, "%.1f against %.1f m/s" % (v.state.airspeed_true_ms, best))
    t.check("and 300 m higher", abs(v.state.altitude_msl_m - alt - 300.0) < 30.0, "%.0f m" % (v.state.altitude_msl_m - alt))


@scenario("fa6.arrival", group=G6, title="A required time of arrival", refs="ADR-29 4.33",
          summary="A point 8 km east to be reached between 200 and 210 s from now: the speed scheduled over the ground to "
                  "arrive in its window; its estimate shown as it flies.",
          options=[aircraft("c172", ("c172", "c130j", "f16c", "rq4b")), number("time", "Arrive after", 200.0, 120.0, 400.0, 10.0, "s")])
def arrival(t):
    v = start(t, t.opt("aircraft"), heading=90.0)
    yield t.wait(2.0)
    lat, lon = here(v)
    t0 = t.world.time
    S = t.scale
    # the time asked for, as a C172 at 50 m/s would make it: for a faster aircraft over its longer legs, as its own speed would
    when = t.opt("time") * (S * 50.0 / max(50.0, v.state.airspeed_true_ms))
    slowest = 1.3 * v.performance.min_cas_ms  # (the average it would need, no slower than it flies comfortably)
    if 8000.0 * S / when < slowest:
        when = 8000.0 * S / slowest
        t.log("the window moved to %.0f s: sooner would need it slower than %.0f m/s" % (when, slowest))
    pts = [fsim.Waypoint(lat, lon + east_of(lat, 3000.0 * S)),
           fsim.Waypoint(lat, lon + east_of(lat, 8000.0 * S), arrival_begin_s=t0 + when, arrival_end_s=t0 + when + 10.0)]
    a = t.watch(v.submit_route(pts), "route with a time")
    t.metric("estimated arrival, s from the start", lambda: a.progress.arrival_s - t0 if a.progress else float("nan"))
    t.metric("against its window, s", lambda: a.progress.arrival_delta_s if a.progress else float("nan"))
    ok = yield t.until_done(a, when + 200.0)
    arrived = t.world.time - t0
    t.check("arrives in its window", ended(a) and when - 3.0 <= arrived <= when + 13.0, "at %.0f s (window %.0f to %.0f s)" % (arrived, when, when + 10.0))


@scenario("fa6.states", group=G6, title="Planned states: through a point at a time", refs="ADR-29 4.34",
          summary="A route east whose second segment must pass 5.5 km east at 1,550 m at 140 s: the aircraft slows and climbs "
                  "to meet it.",
          options=[aircraft("c172", ("c172", "c130j", "f16c"))])
def states(t):
    v = start(t, t.opt("aircraft"), heading=90.0)
    yield t.wait(2.0)
    lat, lon = here(v)
    alt = v.state.altitude_msl_m
    t0 = t.world.time
    S = t.scale
    speed = max(50.0, v.state.airspeed_true_ms)
    due = 140.0 * S * 50.0 / speed  # (a C172's 140 s at 50 m/s, for this aircraft at its own speed)
    slowest = 1.3 * v.performance.min_cas_ms
    if 5500.0 * S / due < slowest:
        due = 5500.0 * S / slowest
        t.log("its state's time moved to %.0f s: later would need it slower than %.0f m/s" % (due, slowest))
    pts = [fsim.Waypoint(lat, lon + east_of(lat, 3000.0 * S), alt, speed=speed), fsim.Waypoint(lat, lon + east_of(lat, 8000.0 * S), alt)]
    sts = [fsim.RouteState(1, lat, lon + east_of(lat, 5500.0 * S), alt + 50.0, "msl", time_s=t0 + due)]
    a = t.watch(v.submit_route(pts, states=sts), "route with a state")
    passed = {}

    def past():
        if "t" not in passed and (here(v)[1] - lon) * R * math.cos(lat) >= 5500.0 * S:
            passed["t"], passed["alt"] = t.world.time - t0, v.state.altitude_msl_m
        return "t" in passed
    ok = yield t.until(past, 3.0 * due, "its state's place")
    t.check("passed its state's place within 5 s of %.0f s" % due, ok and abs(passed["t"] - due) < 5.0, "at %.1f s" % passed.get("t", float("nan")))
    t.check("at its altitude within 30 m", ok and abs(passed["alt"] - alt - 50.0) < 30.0, "%.0f m" % (passed.get("alt", float("nan")) - alt))
    yield t.until_done(a, 300.0)


@scenario("fa6.rnp", group=G6, title="Required navigation performance", refs="ADR-29 4.35",
          summary="North 3 km, then 3 km on, within the given RNP of the path: a tight RNP flags the turn onto the first leg, "
                  "a loose one does not.",
          options=[aircraft("c172", WINGS), number("rnp", "RNP", 20.0, 5.0, 2000.0, 5.0, "m")])
def rnp(t):
    v = start(t, t.opt("aircraft"), heading=90.0)
    yield t.wait(2.0)
    lat, lon = here(v)
    S = t.scale
    pts = [fsim.Waypoint(lat + 3000.0 * S / R, lon, rnp_m=t.opt("rnp")), fsim.Waypoint(lat + 6000.0 * S / R, lon, rnp_m=t.opt("rnp"))]
    a = t.watch(v.submit_route(pts), "route with an RNP")
    t.metric("cross-track, m", lambda: a.progress.cross_track_m if a.progress else float("nan"))
    t.metric("off its RNP now", lambda: "yes" if a.info and a.info.constraints & int(fsim.ActivityFlag.NAVIGATION_PERFORMANCE) else "no")
    ok = yield t.until_done(a, 600.0)
    seen = bool(a.info.constraints_seen & int(fsim.ActivityFlag.NAVIGATION_PERFORMANCE))
    t.check("the route completes", ended(a), a.state.name)
    t.check("off its RNP %s" % ("in the turn (a tight RNP)" if t.opt("rnp") < 200.0 else "never (a loose RNP)"),
            seen == (t.opt("rnp") < 200.0), "flagged" if seen else "never flagged")


@scenario("fa6.paths", group=G6, title="Paths and branches", refs="ADR-29 4.36, 4.37",
          summary="Path A (two points) leads into path B, four points flown round and round; at B's last point a branch takes "
                  "it out along path C after two laps - or when you press the button (the operator's input).",
          options=[aircraft("c172", WINGS)])
def paths(t):
    v = start(t, t.opt("aircraft"), heading=90.0)
    yield t.wait(2.0)
    lat, lon = here(v)
    at = lambda n, e: offset_rad(lat, lon, n * 3000.0 * t.scale, e * 3000.0 * t.scale)  # noqa: E731
    ne = [(0, 1), (0, 2), (1, 3), (2, 3), (2, 4), (1, 4), (-1, 4), (-1, 5)]
    pts = [fsim.Waypoint(*at(n, e), id=100 + i) for i, (n, e) in enumerate(ne)]
    pts[1] = pts[1]._replace(next=2)
    pts[5] = pts[5]._replace(next=2)
    paths_ = [fsim.RoutePath(1, "primary", 0, 2), fsim.RoutePath(2, "primary", 2, 4), fsim.RoutePath(3, "egress", 6, 2)]
    out = fsim.RouteBranch(5, 6, captures=2, captures_comparison="greater_equal")
    asked = fsim.RouteBranch(3, 6, operator_input=1)
    a = t.watch(v.submit_route(pts, paths=paths_, branches=[out, asked]), "route with paths and branches")
    flown = []

    def follow():
        p = a.progress
        if p and p.segments and (not flown or flown[-1] != p.segment):
            flown.append(p.segment)
        return " ".join(str(x) for x in flown[-14:])
    t.metric("points flown to", follow)
    t.action("Take the branch out (operator input)", lambda: a.command_branch(1) if a.live else None)
    ok = yield t.until(lambda: follow() is None or not a.live, 3000.0, "its end")
    t.check("the route ends along path C", ended(a) and flown and flown[-1] == 7, "flew " + " ".join(map(str, flown)))
    t.check("round path B before leaving", flown.count(2) >= 1, "%d times to its first point" % flown.count(2))


@scenario("fa6.terminators", group=G6, title="Civil path terminators: a radius to fix", refs="ADR-29 4.38",
          summary="East 3 km, a quarter circle of 1,500 m round to the right as an RF leg (its centre given), then south 3 km "
                  "as a TF leg.",
          options=[aircraft("c172", WINGS), number("radius", "Arc radius", 1500.0, 800.0, 4000.0, 100.0, "m")])
def terminators(t):
    v = start(t, t.opt("aircraft"), heading=90.0, north=3000.0)
    yield t.wait(2.0)
    lat, lon = here(v)
    S = t.scale
    r = max(t.opt("radius"), 1.15 * t.turn_radius_m(t.opt("aircraft")))
    at = lambda n, e: offset_rad(lat, lon, n, e)  # noqa: E731
    pts = [fsim.Waypoint(*at(0, 3000 * S)), fsim.Waypoint(*at(-r, 3000 * S + r), terminator="rf"), fsim.Waypoint(*at(-r - 3000 * S, 3000 * S + r))]
    centre = at(-r, 3000 * S)
    arc = fsim.RouteTerminator(1, center_latitude_rad=centre[0], center_longitude_rad=centre[1], radius_m=r, clockwise=True)
    a = t.watch(v.submit_route(pts, terminators=[arc]), "route with an RF leg")
    worst = [0.0]

    def arc_error():
        p = a.progress
        if p and a.state == fsim.ActivityState.ACTIVE and p.segment == 1 and 5.0 < p.segment_percent < 95.0:
            worst[0] = max(worst[0], abs(distance_m(centre[0], centre[1], *here(v)) - r))
        return worst[0]
    t.metric("worst off the arc, m", arc_error)
    ok = yield t.until(lambda: arc_error() is None or not a.live, 900.0, "its end")
    t.check("the route completes", ended(a), a.state.name)
    t.check("the RF leg flown within 60 m of its arc", worst[0] < 60.0, "%.0f m" % worst[0])
