"""FA-7 and FA-8: route plans (uploaded, activated, validated in a wind), and the must flies - a point, another vehicle, a
zone, a corridor, a volume - the altitude-stacked marshall and the route intercept."""
import math

import fsim

from ..registry import aircraft, choice, number, scenario
from ._fa import R, WINGS, deg, distance_m, ended, hold_level, offset_rad, rad

G7 = "FA-7 Route plans"
G8 = "FA-8 Must fly, marshall, intercept"
UP = 1200.0


def start(t, kind, heading=90.0, north=-3000.0, east=4000.0, name=None):
    """The aircraft in the air over the bay, holding its heading; the scenario's geometry scaled to it (t.scale)."""
    S = t.fit(kind)
    v = t.spawn(kind, north * S, east, agl_m=UP, heading_deg=heading, name=name)
    hold_level(v)
    return v


def here(v):
    s = v.state
    return s.latitude_rad, s.longitude_rad


# --- FA-7 ----------------------------------------------------------------------------------------------------------
@scenario("fa7.plan", group=G7, title="A route plan: uploaded, activated, superseded", refs="ADR-29 4.39",
          summary="A plan of three points prepared for upload, published, uploaded and read back; prepared for activation and "
                  "activated (it flies); an HSA supersedes it; the plan is activated again and flown to its end.",
          options=[aircraft("c172", WINGS)])
def plan(t):
    v = start(t, t.opt("aircraft"))
    yield t.wait(2.0)
    lat, lon = here(v)
    alt = v.state.altitude_msl_m
    S = t.scale
    pts = [fsim.Waypoint(*offset_rad(lat, lon, n * S, e * S), alt) for n, e in ((0.0, 3000.0), (3000.0, 6000.0), (6000.0, 3000.0))]
    p = fsim.RoutePlan(21, fsim.BatchCommand("submit_route", pts), version=1, remarks_name="testbench")
    t.metric("plan 21", lambda: "%s, %s, %.0f %%" % (v.plan_status(21).state.name, v.plan_status(21).execution.name, v.plan_status(21).percent))
    t.check("prepared for upload", v.plan_command(21, "prepare_for_upload").completed)
    v.publish_plan(p)
    t.check("uploaded", v.plan_command(21, fsim.PlanCommand.UPLOAD).completed)
    t.check("read back as given", v.plan(21).remarks_name == "testbench")
    t.check("prepared for activation", v.plan_command(21, "prepare_for_activation").completed)
    r = v.plan_command(21, "activate", command_id=44)
    t.check("activated", r.completed and v.plan_status(21).state == fsim.PlanState.ACTIVATED, v.plan_status(21).state.name)
    yield t.wait(40.0)
    hsa = t.watch(v.submit_hsa(heading_rad=0.0), "hsa: supersedes the plan")
    yield t.wait(3.0)
    t.check("superseded by the hsa", v.plan_status(21).execution == fsim.PlanExecution.SUPERSEDED, v.plan_status(21).execution.name)
    yield t.wait(20.0)
    v.plan_command(21, "prepare_for_activation")
    r = v.plan_command(21, "activate")
    t.check("activated again", r.completed, v.plan_status(21).state.name)
    ok = yield t.until(lambda: v.plan_status(21).execution not in (fsim.PlanExecution.PENDING, fsim.PlanExecution.EXECUTING), 900.0, "its end")
    t.check("flown to its end", v.plan_status(21).execution == fsim.PlanExecution.COMPLETE, v.plan_status(21).execution.name)


@scenario("fa7.validate", group=G7, title="A plan validated in a wind", refs="ADR-29 4.41",
          summary="A plan with two tight corners validated without flying it: valid in calm air, invalid with 20 m/s from the "
                  "west behind the turns; modified to validate, valid. Then the wind blows, and the route is flown in it, its "
                  "turns held to what the aircraft can do.",
          options=[aircraft("c172", WINGS), number("wind", "Wind", 20.0, 0.0, 30.0, 1.0, "m/s")])
def validate(t):
    v = start(t, t.opt("aircraft"))
    yield t.wait(2.0)
    lat, lon = here(v)
    alt = v.state.altitude_msl_m
    # its corners just wide enough for it in calm air - found by validating, tighter to wider (nothing is flown) - so a
    # wind behind its turns is what they cannot take
    wind = t.opt("wind") * max(1.0, v.state.airspeed_true_ms / 50.0)  # (a C172's 20 m/s against its 50: the same share of the speed)

    def corners(S):
        at = lambda n, e: fsim.Waypoint(lat + n * S / R, lon + e * S / (R * math.cos(lat)), alt)  # noqa: E731
        return [at(0.0, 3000.0), at(1200.0, 3000.0), at(1200.0, 4200.0), at(1200.0, 7000.0)]
    S = 0.25
    while S < 40.0 and not v.validate_plan(fsim.RoutePlan(91, fsim.BatchCommand("submit_route", corners(S)))).valid:
        S *= 1.15
    t.log("corners x%.2f: the tightest it flies in calm air" % S)
    pts = corners(S)
    p = fsim.RoutePlan(91, fsim.BatchCommand("submit_route", pts))
    calm = v.validate_plan(p)
    t.check("valid in calm air", calm.valid, calm.validation.reason)
    windy = v.validate_plan(p, wind=(0.0, wind))
    t.check("invalid with %.0f m/s behind its turns" % wind, not windy.valid or t.opt("wind") < 10.0,
            "%s at point %s" % (windy.validation.reason, windy.index))
    fixed = v.validate_plan(p, wind=(0.0, wind), modify=True)
    t.check("modified to validate, valid", fixed.valid, "%d points adjusted" % len(fixed.validation.adjustments))
    # flown: the wind blowing, the route given with its values held to the aircraft's limits (as modified to validate)
    if wind > 0.0:
        t.world.set_wind(270.0, wind)
        t.log("wind %.0f m/s from the west" % wind)
    a = t.watch(v.submit_route(pts, range=fsim.RangePolicy.CLAMP), "the route, flown in the wind")
    t.metric("cross-track, m", lambda: a.progress.cross_track_m if a.progress else float("nan"))
    ok = yield t.until_done(a, 900.0)
    t.check("flown to its end in the wind", ended(a), a.state.name)


# --- FA-8 ----------------------------------------------------------------------------------------------------------
@scenario("fa8.point", group=G8, title="Must fly: a point, from a window of bearings", refs="ADR-29 4.42",
          summary="An operational point 4 km east kept by the world, to be passed over at 1,600 m approached from the south "
                  "(bearings 170 to 190 from it): the aircraft lays a route round to come in from there, flies over, and flies on.",
          options=[aircraft("c172", WINGS), number("ingress", "Approach from (bearing)", 180.0, 0.0, 360.0, 10.0, "deg")])
def mf_point(t):
    v = start(t, t.opt("aircraft"))
    yield t.wait(2.0)
    # (4 km east for a C172; drawn bigger for a big jet, east would be the East Bay hills: west, over the sea)
    lat, lon = offset_rad(*here(v), 1000.0 * t.scale, 4000.0 * t.scale * (1.0 if t.scale <= 2.0 else -1.0))
    b = t.opt("ingress")
    wrap = lambda x: math.remainder(rad(x), 2.0 * math.pi)  # noqa: E731 (bearings within +-180 deg)
    t.world.set_op_point(fsim.OpPoint(7, lat, lon, t.field_m + 1000.0, ingress_min_rad=wrap(b - 10.0), ingress_max_rad=wrap(b + 10.0)))
    a = t.watch(v.submit_must_fly(location=fsim.MustFlyLocation.OP_POINT, target=7), "must fly over op point 7")
    closest = [1e9, 0.0]

    def near():
        d = distance_m(lat, lon, *here(v))
        if d < closest[0]:
            s = v.state
            closest[:] = [d, math.degrees(math.atan2(-(s.velocity_ned_ms[1]), -(s.velocity_ned_ms[0]))) % 360.0]
        return d
    t.metric("from the point, m", near)
    ok = yield t.until(lambda: near() is None or not a.live, 900.0, "over it")
    t.check("passed over it", ended(a) and closest[0] < 150.0, "%.0f m at the closest" % closest[0])
    t.check("from its window of bearings", abs((closest[1] - b + 180.0) % 360.0 - 180.0) < 25.0, "came from %.0f deg" % closest[1])


@scenario("fa8.entity", group=G8, title="Must fly: over another aircraft", refs="ADR-29 4.42",
          summary="A slower aircraft flies north 3 km south-east; the chosen aircraft is commanded to fly over it, catching it "
                  "and passing over it at least 500 ft above.",
          options=[aircraft("f16c", WINGS)])
def mf_entity(t):
    v = start(t, t.opt("aircraft"))
    s = v.state
    slow = t.spawn("c172", -6000.0 * t.scale, 10000.0 * t.scale, alt_m=s.altitude_msl_m - 100.0, heading_deg=0.0, speed_ms=45.0, name="target")
    slow.submit_hsa(course_rad=0.0, speed=45.0, altitude_m=s.altitude_msl_m - 100.0)
    yield t.wait(2.0)
    a = t.watch(v.submit_must_fly(location="entity", target=slow), "must fly over the target")
    best = [1e9]

    def near():
        best[0] = min(best[0], distance_m(*here(slow), *here(v)))
        return best[0]
    t.metric("closest over the ground, m", near)
    ok = yield t.until(lambda: near() is None or not a.live, 900.0, "over it")
    t.check("flew over it", ended(a) and best[0] < 150.0, "%.0f m at the closest" % best[0])


@scenario("fa8.zone", group=G8, title="Must fly: into a zone", refs="ADR-29 4.43",
          summary="A zone 5 km east - an ellipse, a rectangle or a polygon with a hole - to be entered: the activity completes "
                  "as the aircraft is inside it.",
          options=[aircraft("c172", WINGS), choice("shape", "Zone", "ellipse", ("ellipse", "rectangle", "polygon"))])
def mf_zone(t):
    v = start(t, t.opt("aircraft"))
    yield t.wait(2.0)
    S = t.scale
    lat, lon = offset_rad(*here(v), 2000.0 * S, 5000.0 * S)
    shape = t.opt("shape")
    if shape == "ellipse":
        zone = fsim.OpZone(0, "ellipse", latitude_rad=lat, longitude_rad=lon, semi_major_m=800.0 * S, semi_minor_m=500.0 * S)
    elif shape == "rectangle":
        zone = fsim.OpZone(0, "rectangle", latitude_rad=lat, longitude_rad=lon, width_m=1000.0 * S, height_m=1500.0 * S, orientation_rad=0.5)
    else:
        sq = [offset_rad(lat, lon, n * S, e * S) for n, e in ((-800, -800), (-800, 800), (800, 800), (800, -800))]
        hole = [offset_rad(lat, lon, n * S, e * S) for n, e in ((-200, -200), (-200, 200), (200, 200), (200, -200))]
        zone = fsim.OpZone(0, fsim.ZoneShape.POLYGON, sq, [hole])
    a = t.watch(v.submit_must_fly(zone=zone), "must fly into the " + shape)
    t.metric("from its centre, m", lambda: distance_m(lat, lon, *here(v)))
    ok = yield t.until_done(a, 600.0)
    t.check("in the zone: completed", ended(a), a.state.name)
    t.check("near its centre as it completed", distance_m(lat, lon, *here(v)) < 1000.0 * S, "%.0f m" % distance_m(lat, lon, *here(v)))


@scenario("fa8.corridor", group=G8, title="Must fly: through a corridor", refs="ADR-29 4.44",
          summary="A corridor 1 km wide with a right-angle bend, flown through from its first vertex to its last, each turn "
                  "inside its widths - widened, for an aircraft that turns wider than they allow, to what its turn needs.",
          options=[aircraft("c172", WINGS), number("width", "Half width", 500.0, 100.0, 2000.0, 50.0, "m")])
def mf_corridor(t):
    v = start(t, t.opt("aircraft"))
    yield t.wait(2.0)
    lat, lon = here(v)
    S = t.scale
    corners = [offset_rad(lat, lon, 0.0, 2000.0 * S), offset_rad(lat, lon, 0.0, 5000.0 * S), offset_rad(lat, lon, 3000.0 * S, 5000.0 * S)]
    # a fly-by right angle cuts its corner by about 0.41 of the turn's radius: the half width it needs
    need = 0.6 * t.turn_radius_m(t.opt("aircraft"))
    w = max(t.opt("width"), need)
    if w > t.opt("width"):
        t.log("half width %.0f m: the %s turns in %.0f m" % (w, t.opt("aircraft"), t.turn_radius_m(t.opt("aircraft"))))
    a = t.watch(v.submit_must_fly(line=fsim.OpLine(0, corners, left_width_m=w, right_width_m=w)), "must fly the corridor")
    ok = yield t.until_done(a, 600.0)
    t.check("through the corridor: completed", ended(a), a.state.name)


@scenario("fa8.volume", group=G8, title="Must fly: into a volume", refs="ADR-29 4.45",
          summary="A sphere of 500 m round a point 4 km east and 300 m up, or a vertical cylinder: entered, the activity "
                  "completes.",
          options=[aircraft("c172", WINGS), choice("shape", "Volume", "sphere", ("sphere", "cylinder", "dome"))])
def mf_volume(t):
    v = start(t, t.opt("aircraft"))
    yield t.wait(2.0)
    S = t.scale
    lat, lon = offset_rad(*here(v), 0.0, 4000.0 * S)
    alt = v.state.altitude_msl_m
    shape = t.opt("shape")
    if shape == "sphere":
        vol = fsim.OpVolume(0, "sphere", lat, lon, altitude_m=alt + 300.0, radius_m=500.0 * S)
    elif shape == "cylinder":
        vol = fsim.OpVolume(0, fsim.VolumeShape.CYLINDER, lat, lon, altitude_m=0.0, radius_m=700.0 * S, length_m=5000.0, pitch_rad=math.pi / 2)
    else:
        vol = fsim.OpVolume(0, "dome", lat, lon, altitude_m=alt - 800.0, radius_m=1500.0 * S)
    a = t.watch(v.submit_must_fly(volume=vol), "must fly into the " + shape)
    t.metric("from its centre over the ground, m", lambda: distance_m(lat, lon, *here(v)))
    ok = yield t.until_done(a, 600.0)
    t.check("in the volume: completed", ended(a), a.state.name)


@scenario("fa8.marshall", group=G8, title="An altitude-stacked marshall", refs="ADR-29 4.46",
          summary="Three aircraft marshal round one point, each given the lowest free slot 300 m apart from 1,500 m; a fourth "
                  "is refused a full stack.",
          options=[aircraft("c172", WINGS)])
def marshall(t):
    kind = t.opt("aircraft")
    S = t.fit(kind)
    vs = [t.spawn(kind, (-3000.0 + 1500.0 * i) * S, 4000.0, alt_m=1500.0 + 300.0 * i, heading_deg=90.0, name="m%d" % (i + 1)) for i in range(4)]
    for v in vs:
        hold_level(v)
    yield t.wait(2.0)
    lat, lon = offset_rad(*here(vs[0]), 3000.0 * S, 3000.0 * S)
    stack = dict(latitude_rad=lat, longitude_rad=lon, radius_m=1500.0 * S, altitude_min_m=1500.0, altitude_max_m=2100.0, separation_m=300.0)
    acts = []
    for i, v in enumerate(vs[:3]):
        a = t.watch(v.submit_marshall(**stack), "m%d marshal" % (i + 1))
        acts.append(a)
        t.log("m%d given %.0f m" % (i + 1, a.setpoint().kwargs["altitude_m"]))
    try:
        vs[3].submit_marshall(**stack)
        t.check("a fourth refused: the stack is full", False, "accepted")
    except fsim.Rejected as ex:
        t.check("a fourth refused: the stack is full", ex.reason == "stack_full", ex.reason)
    slots = sorted(a.setpoint().kwargs["altitude_m"] for a in acts)
    t.check("three slots 300 m apart from 1,500 m", [round(x) for x in slots] == [1500, 1800, 2100], ", ".join("%.0f" % x for x in slots))
    for i, v in enumerate(vs[:3]):
        t.metric("m%d altitude, m" % (i + 1), lambda v=v: v.state.altitude_msl_m)
    yield t.wait(240.0)
    t.check("each holds its slot", all(abs(v.state.altitude_msl_m - a.setpoint().kwargs["altitude_m"]) < 40.0 for v, a in zip(vs, acts)))


@scenario("fa8.intercept", group=G8, title="A route intercept", refs="ADR-29 4.47",
          summary="A kept plan of four points north, 2 km west of the aircraft flying north; it joins the plan by the chosen "
                  "method - its beginning, the nearest point, the shortest distance or the soonest - and flies it to its end.",
          options=[aircraft("c172", WINGS), choice("method", "Join", "soonest", ("beginning", "discrete", "shortest_distance", "soonest"))])
def intercept(t):
    v = start(t, t.opt("aircraft"), heading=0.0)
    yield t.wait(2.0)
    lat, lon = here(v)
    S = t.scale
    west = lon - 2000.0 * S / (R * math.cos(lat))
    alt = v.state.altitude_msl_m
    pts = [fsim.Waypoint(lat + (k - 2.4) * 3000.0 * S / R, west, alt, id=100 + k) for k in range(4)]
    v.plan_command(44, "prepare_for_upload")
    v.publish_plan(fsim.RoutePlan(44, fsim.BatchCommand("submit_route", pts)))
    v.plan_command(44, "upload")
    m = t.opt("method")
    a = t.watch(v.submit_intercept(plan=44, **({} if m == "beginning" else {"method": m})), "intercept plan 44")
    s = a.intercept_status()
    t.log("joins at point %s (laid %s)" % (s.joined, s.laid))
    t.metric("intercept", lambda: "%s, joined %s, now %s" % (a.intercept_status().execution.name if hasattr(a.intercept_status().execution, "name")
                                                               else a.intercept_status().execution, a.intercept_status().joined,
                                                               a.intercept_status().current) if a.live else a.state.name)
    ok = yield t.until_done(a, 900.0)
    t.check("the plan is flown to its end", ended(a), a.state.name)
