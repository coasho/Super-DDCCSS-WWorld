"""The platform beneath the suites: the terrain strike, a start inside the terrain, and a sandbox to fly by hand."""
import math

import fsim

from ..registry import aircraft, choice, number, scenario, suite
from ._fa import ALL, ROTORS, WINGS, deg, ended, heading_of, hold_level, offset_rad, rad, reason, wrap_deg

suite("Platform", "What every suite stands on: the terrain, the world, flying by hand")
GP = "The terrain and the world"
SAN_BRUNO = (37.6871, -122.4340)  # 400 m, 11 km north-west of the runway


@scenario("platform.strike", group=GP, title="Flown into a mountain", refs="ADR-29 4.68", suite="Platform",
          summary="Level at 250 m towards San Bruno Mountain (400 m) on an attitude command, which no terrain check stops: "
                  "the aircraft strikes the slope, is stopped where it struck - on the surface, not in it - and its activity "
                  "fails 'crashed'. Watch it in the viewer: it never passes through the drawn hillside.",
          options=[aircraft("c172", ALL), number("height", "Height", 250.0, 50.0, 380.0, 10.0, "m")])
def strike(t):
    lat, lon = map(rad, SAN_BRUNO)
    s0 = offset_rad(lat, lon, 0.0, 6000.0)  # (over the bay, east of it)
    kind = t.opt("aircraft")
    v = t.spawn(kind, at=(deg(s0[0]), deg(s0[1])), alt_m=t.opt("height"), heading_deg=270.0,
                speed_ms=max(30.0, t.cruise_ms(kind)) if not t.performance(kind).hovers else 0.0)
    if t.performance(kind).hovers:
        a = t.watch(v.submit(fsim.Level.VELOCITY, north_ms=0.0, east_ms=-25.0, vertical_speed_ms=0.0), "velocity west")
    else:
        # level at its own trim (a C172's 3 deg; a jet, faster, less: held at 3 deg it climbs over the mountain) -
        # found under a velocity command, which looks no further ahead than the attitude one
        v.submit(fsim.Level.VELOCITY, airspeed_ms=max(30.0, t.cruise_ms(kind)), vertical_speed_ms=0.0, heading_rad=rad(270.0))
        yield t.wait(8.0)
        trim = v.state.euler_rad[1]
        a = t.watch(v.submit(fsim.Level.ATTITUDE, roll_rad=0.0, pitch_rad=trim, heading_rad=rad(270.0),
                             airspeed_ms=max(30.0, t.cruise_ms(kind))), "attitude: level, west")
    t.metric("above the ground, m", lambda: v.state.altitude_agl_m)
    t.metric("crashed", lambda: "yes" if v.state.crashed else "no")
    ok = yield t.until(lambda: v.state.crashed or v.state.diverged, 600.0, "the strike")
    s = v.state
    t.check("it struck the terrain", s.crashed, "%.0f m up, %.4f, %.4f" % (s.altitude_msl_m, deg(s.latitude_rad), deg(s.longitude_rad)))
    # (its centre a few metres up, more for a big aircraft resting against the slope on its nose or a wing: a C-17A's
    # is some 16 m up)
    t.check("stopped on the surface, not in it", -0.5 < s.altitude_agl_m < 25.0 and ground_speed(s) == 0.0, "%.1f m above it" % s.altitude_agl_m)
    yield t.wait(2.0)
    t.check("its activity failed 'crashed'", a.state == fsim.ActivityState.FAILED and reason(a) == "crashed", "%s %s" % (a.state.name, reason(a)))
    t.check("not diverged", not v.state.diverged)
    yield t.wait(5.0)


def ground_speed(s):
    return math.hypot(s.velocity_ned_ms[0], s.velocity_ned_ms[1])


@scenario("platform.inside", group=GP, title="Started inside a mountain", refs="ADR-29 4.68", suite="Platform",
          summary="An aircraft created in the air 150 m under San Bruno Mountain's summit: it is stood on the surface as it "
                  "starts, crashed, instead of being thrown out of the rock. A reset above the mountain flies again.",
          options=[aircraft("c172", ALL)])
def inside(t):
    kind = t.opt("aircraft")
    v = t.spawn(kind, at=SAN_BRUNO, alt_m=250.0, heading_deg=270.0)
    s = v.state
    t.check("crashed as it starts", s.crashed, "%.1f m above the ground" % s.altitude_agl_m)
    t.check("on the surface", -0.5 < s.altitude_agl_m < 3.0, "%.1f m" % s.altitude_agl_m)
    yield t.wait(5.0)
    t.check("still there, not diverged", v.state.crashed and not v.state.diverged)
    v.reset(latitude_deg=SAN_BRUNO[0], longitude_deg=SAN_BRUNO[1], altitude_msl_m=900.0, heading_deg=270.0,
            airspeed_ms=t.cruise_ms(kind) if not t.performance(kind).hovers else 0.0)
    hold_level(v) if not t.performance(kind).hovers else v.submit(fsim.Level.VELOCITY, north_ms=0.0, east_ms=0.0, vertical_speed_ms=0.0)
    yield t.wait(10.0)
    t.check("reset above it, it flies", not v.state.crashed and v.state.altitude_agl_m > 300.0, "%.0f m up" % v.state.altitude_agl_m)


@scenario("platform.sandbox", group="Fly by hand", title="Sandbox", refs="", suite="Platform", interactive=True,
          summary="An aircraft over the bay holding what it flies; the buttons turn, climb, change speed, orbit, loop and "
                  "send it home to land on 28R - try anything and watch it in the viewer. No checks: it runs until stopped.",
          options=[aircraft("f16c", ALL), number("height", "Height", 1500.0, 100.0, 12000.0, 100.0, "m")])
def sandbox(t):
    kind = t.opt("aircraft")
    hovers = t.performance(kind).hovers
    v = t.spawn(kind, -4000.0, 6000.0, agl_m=t.opt("height"), heading_deg=t.heading_deg(180.0))
    cur = {"a": None}

    def hsa(**fields):
        if cur["a"] is not None and cur["a"].live and cur["a"].level == "hsa":
            cur["a"].update(**fields)
        else:
            s = v.state
            base = dict(heading_rad=s.euler_rad[2], altitude_m=s.altitude_msl_m)
            base.update(fields)
            cur["a"] = t.watch(v.submit_hsa(**base), "hsa")

    def turn(dd):
        return lambda: hsa(heading_rad=rad((heading_of(v.state) + dd) % 360.0))

    def climb(dh):
        return lambda: hsa(altitude_m=v.state.altitude_msl_m + dh)

    def speed(dv):
        return lambda: hsa(speed=max(5.0, v.state.airspeed_true_ms + dv), speed_reference="true_airspeed")
    hsa()
    t.action("Left 90", turn(-90.0))
    t.action("Right 90", turn(90.0))
    t.action("Climb 500 m", climb(500.0))
    t.action("Descend 500 m", climb(-500.0))
    t.action("Faster", speed(10.0))
    t.action("Slower", speed(-10.0))

    def orbit():
        cur["a"] = t.watch(v.submit_pattern(radius_m=1500.0 if not hovers else 200.0), "orbit")
    t.action("Orbit here", orbit)

    def manoeuvre(m):
        def go():
            try:
                cur["a"] = t.watch(v.submit_behavior("aerobatics", manoeuvre=float(m)), ("roll", "loop", "immelmann", "split-S")[m])
            except fsim.Rejected as ex:
                t.log("refused: %s %s" % (ex.reason, ex.constraint))
        return go
    if not hovers:
        t.action("Roll", manoeuvre(0))
        t.action("Loop", manoeuvre(1))

    def home():
        try:
            cur["a"] = t.watch(v.submit_behavior("recovery", airfield=t.SFO, runway=t.RWY), "recovery to " + t.runway_name)
        except fsim.Rejected as ex:
            t.log("refused: %s" % ex.reason)
    t.action("Land on " + t.runway_name, home)
    yield t.until(lambda: False, 36000.0, "")


GV = "The viewer"


def deg_to_rad(x):
    return math.radians(x)


# The viewer's vapour (world::WingVapour, a port of aeronautic's wing vapour): the air round the aircraft expanded
# on the moist adiabat, cloud wherever it passes its dew point. Here its two strongest numbers, each over what
# condenses (1 or more: cloud), for aeronautic's own fighter (14 m span, chords 9.2 and 1.6 m, 55 degrees of sweep):
# the testbench does not know each model's planform as the viewer does.
_SPAN, _ROOT, _TIP, _SWEEP = 14.0, 9.2, 1.6, math.radians(55.0)


def _air(altitude_m, sea_level_k=288.15):
    """The standard atmosphere's temperature (K), pressure (Pa) and density there, as humid as the viewer draws it."""
    h = min(max(altitude_m, 0.0), 11000.0)
    t_std = 288.15 - 0.0065 * h
    p = 101325.0 * (t_std / 288.15) ** 5.25588
    t = t_std + (sea_level_k - 288.15)
    rh = 0.85  # (world::WingVapour::defaultHumidity: aeronautic's default day)
    es = lambda k: 610.94 * math.exp(17.625 * (k - 273.15) / (k - 273.15 + 243.04))  # noqa: E731
    e = rh * es(t)
    return t, p, (p - 0.378 * e) / (287.05 * t), 0.622 * e / (p - e) * min(1.06, 0.995 / rh), es


def _needed(t, p, w, es):
    """The least pressure deficit (1 - p / p0) that condenses anything in the moistest air of the day."""
    lo, hi = 0.3, 1.0
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        tk, pk = t * mid ** (287.05 / 1004.6), p * mid
        if w > 0.622 * es(tk) / (pk - es(tk)):
            lo = mid
        else:
            hi = mid
    return 1.0 - lo


def _lift(alpha_rad, mach):
    """DATCOM's lift slope and Polhamus's vortex lift: the attached flow's coefficient, and the whole."""
    area = _SPAN * (_ROOT + _TIP) / 2.0
    ar, taper = _SPAN * _SPAN / area, _TIP / _ROOT
    half = math.tan(_SWEEP) - 2.0 * (1.0 - taper) / (ar * (1.0 + taper))
    m = min(max(mach, 0.0), 0.9)
    b = math.sqrt(1.0 - m * m)
    kp = 2.0 * math.pi * ar / (2.0 + math.sqrt(ar * ar * b * b / 0.9025 * (1.0 + half * half / (b * b)) + 4.0))
    kv = max(kp - kp * kp / (math.pi * ar), 0.0) / math.cos(_SWEEP)
    sweep_deg = math.degrees(_SWEEP)
    te = math.radians(max(1.3 * (sweep_deg - 47.0), 3.0))
    whole = min(max((te + math.radians(15.0) - abs(alpha_rad)) / math.radians(15.0), 0.0), 1.0)
    sn, cs = math.sin(alpha_rad), math.cos(alpha_rad)
    potential = kp * sn * cs * cs
    return potential, potential + kv * sn * abs(sn) * cs * (0.4 + 0.6 * whole), area


def tip_vortex(alpha_rad, tas_ms, altitude_m, sea_level_k=288.15):
    """The tip vortex's core: its pressure deficit (a Scully vortex holding 35 % of the wing's circulation in a core 2 %
    of the span across) over what condenses."""
    t, p, rho, w, es = _air(altitude_m, sea_level_k)
    _, cl, area = _lift(alpha_rad, tas_ms / math.sqrt(1.4 * 287.05 * t))
    lift = 0.5 * rho * tas_ms * tas_ms * area * cl
    circulation = 0.35 * abs(lift) / (rho * max(tas_ms, 1.0) * math.pi / 4.0 * _SPAN)
    core = 0.02 * _SPAN
    deficit = rho / (8.0 * math.pi * math.pi * p) * circulation * circulation / (core * core)
    return deficit / _needed(t, p, w, es)


def wing_vapour(alpha_rad, tas_ms, altitude_m, sea_level_k=288.15):
    """The wing's suction peak (thin-airfoil theory a tenth of the chord back, Karman-Tsien on the flow normal to the
    sweep, held to an expansion to Mach 1.4) as a deficit over what condenses."""
    t, p, _, w, es = _air(altitude_m, sea_level_k)
    mach = tas_ms / math.sqrt(1.4 * 287.05 * t)
    potential, _, _ = _lift(alpha_rad, mach)
    taper, ar = _TIP / _ROOT, _SPAN * _SPAN / (_SPAN * (_ROOT + _TIP) / 2.0)
    cos_eff = math.sqrt(math.cos(math.atan(math.tan(_SWEEP) - 2.0 * (1.0 - taper) / (ar * (1.0 + taper)))))
    separation = min(max((abs(alpha_rad) - math.radians(3.0)) / math.radians(5.0), 0.0), 1.0)
    separation = separation * separation * (3.0 - 2.0 * separation)
    shape = math.sqrt(0.9 / 0.13) * (1.0 - separation) + math.pi * 0.9 * separation
    incompressible = (-(max(potential, 0.0) / math.pi) * shape - 1.3 * 0.06 * math.sin(0.1 * math.pi)) / cos_eff
    mn = mach * math.sqrt(cos_eff)
    beta = max(math.sqrt(abs(1.0 - mn * mn)), 0.15)
    cp = incompressible / max(beta + mn * mn / (1.0 + beta) * incompressible / 2.0, beta * 0.5)
    cap = 1.0 - ((1.0 + 0.2 * mn * mn) / (1.0 + 0.2 * 1.96)) ** 3.5
    deficit = -0.7 * mn * mn * cp
    deficit = cap * (1.0 - math.exp(-deficit / cap)) if deficit > 0.0 else deficit
    return deficit / _needed(t, p, w, es)


@scenario("platform.parts", group=GV, title="Moving parts follow the state", refs="viewer.md", suite="Platform",
          summary="An aircraft works each of its moving parts in turn from commands alone - ailerons, elevator, rudder, "
                  "flaps, gear down and up, the throttle to full (afterburner where it has one), the speedbrake open and "
                  "shut - and each check reads the state field the viewer draws that part by. Then on the runway: the "
                  "throttle up, the nose wheel steered with the rudder, the wheels rolling on compressed struts, and the "
                  "wheel brakes stopping it. Watch the parts move in the viewer: nothing but the simulation drives them.",
          options=[aircraft("f16c", WINGS)])
def parts(t):
    kind = t.opt("aircraft")
    v = t.spawn(kind, -3000.0, 4000.0, agl_m=1500.0, heading_deg=t.heading_deg(180.0))
    peak = {}

    def record():
        s = v.state
        for name in ("aileron_rad", "elevator_rad", "rudder_rad", "flaps_rad", "speedbrake_position"):
            peak[name] = max(peak.get(name, 0.0), abs(getattr(s, name)))
        peak["afterburner"] = max(peak.get("afterburner", 0.0), max(s.afterburner[:max(s.engine_count, 1)]))
        peak["rpm"] = max(peak.get("rpm", 0.0), max(s.engine_rpm[:max(s.engine_count, 1)]))
        n = s.wheel_count
        if n:
            peak["steer"] = max(peak.get("steer", 0.0), max(abs(x) for x in s.wheel_steer_rad[:n]))
            peak["roll"] = max(peak.get("roll", 0.0), max(abs(x) for x in s.wheel_speed_ms[:n]))
            peak["squash"] = max(peak.get("squash", 0.0), max(s.wheel_compression_m[:n]))
    t.every(record)

    def has(kind_, value):
        r = v.submit_support(kind_, value, validate_only=True)
        if not r.valid:
            t.log("no %s to work: %s" % (kind_, r.reason))
        return r.valid
    p = t.performance(kind)
    slow = 1.45 * p.min_cas_ms  # (true airspeed, 1,500 m up: about 1.35 times its least calibrated)
    stick = dict(aileron=0.0, elevator=0.0, rudder=0.0, throttle=0.7, flaps=0.0)

    def actuate(**change):
        stick.update(change)
        v.command_actuator(**stick)

    t.metric("aileron, deg", lambda: deg(v.state.aileron_rad))
    t.metric("elevator, deg", lambda: deg(v.state.elevator_rad))
    t.metric("rudder, deg", lambda: deg(v.state.rudder_rad))
    t.metric("flaps, deg", lambda: deg(v.state.flaps_rad))
    t.metric("gear (1 down)", lambda: v.state.gear_position)
    t.metric("speedbrake (1 open)", lambda: v.state.speedbrake_position)
    t.metric("thrust, kN", lambda: sum(v.state.thrust_n[:max(v.state.engine_count, 1)]) / 1000.0)
    # the surfaces by the stick, briefly (nothing else flies it meanwhile)
    actuate()
    yield t.wait(1.0)
    for name, field, value in (("aileron", "aileron_rad", 0.4), ("elevator", "elevator_rad", -0.25), ("rudder", "rudder_rad", 0.5)):
        for sign in (1.0, -1.0):
            actuate(**{name: sign * value})
            yield t.wait(1.0)
        actuate(**{name: 0.0})
        yield t.wait(1.0)
        t.check("the %s moves both ways" % name, peak[field] > math.radians(3.0), "%.0f deg at the most" % deg(peak[field]))
    # the rest beside a hold of its heading and height, slowed for its flaps and gear
    t.watch(hold_level(v, slow), "hsa: level at %.0f m/s" % slow)
    ok = yield t.until(lambda: v.state.airspeed_true_ms < slow * 1.05, 120.0, "slowed")
    if has("flaps", 1.0):
        v.submit_support("flaps", 1.0)
        yield t.wait(12.0)
        t.check("the flaps come out", v.state.flaps_rad > math.radians(5.0), "%.0f deg" % deg(v.state.flaps_rad))
        v.submit_support("flaps", 0.0)
        yield t.wait(10.0)
    if has("gear", 1.0):
        v.submit_support("gear", 1.0)
        ok = yield t.until(lambda: v.state.gear_position > 0.99, 30.0, "the gear down")
        t.check("the gear comes down", ok, "%.2f" % v.state.gear_position)
        yield t.wait(3.0)
        v.submit_support("gear", 0.0)
        ok = yield t.until(lambda: v.state.gear_position < 0.01, 30.0, "the gear up")
        t.check("and goes up", ok, "%.2f" % v.state.gear_position)
    if has("speedbrake", 1.0):
        t.watch(v.submit_support("speedbrake", 1.0), "speedbrake open")
        ok = yield t.until(lambda: v.state.speedbrake_position > 0.98, 15.0, "the speedbrake open")
        t.check("the speedbrake opens", ok, "%.2f" % v.state.speedbrake_position)
        yield t.wait(3.0)
        v.submit_support("speedbrake", 0.0)
        ok = yield t.until(lambda: v.state.speedbrake_position < 0.02, 15.0, "the speedbrake shut")
        t.check("and shuts", ok, "%.2f" % v.state.speedbrake_position)
    # the engines: from the slow hold to near its top speed
    thrust = lambda: sum(v.state.thrust_n[:max(v.state.engine_count, 1)])  # noqa: E731
    t0 = thrust()
    fast = 0.9 * min(p.max_tas_ms, 300.0)
    t.watch(hold_level(v, fast), "hsa: level at %.0f m/s" % fast)
    yield t.wait(12.0)
    t.check("the engines answer the throttle", peak["rpm"] > 0.0 or thrust() > 1.3 * t0 or peak["afterburner"] > 0.0,
            "thrust %.1f -> %.1f kN at the most, afterburner %.2f, rpm %.0f" % (t0 / 1000.0, thrust() / 1000.0, peak["afterburner"], peak["rpm"]))
    # on the runway: rolling, steering, braking
    g = t.spawn_on_runway(kind, along_m=300.0, name=kind + " on the runway")
    v = g
    peak.clear()
    stick.update(aileron=0.0, elevator=0.0, rudder=0.0, throttle=0.35, flaps=0.0)
    actuate()
    t.metric("ground speed, m/s", lambda: math.hypot(g.state.velocity_ned_ms[0], g.state.velocity_ned_ms[1]))
    ok = yield t.until(lambda: math.hypot(g.state.velocity_ned_ms[0], g.state.velocity_ned_ms[1]) > 8.0, 60.0, "rolling at 8 m/s")
    for sign in (1.0, -1.0):
        actuate(rudder=0.4 * sign, throttle=0.1)
        yield t.wait(2.0)
    actuate(rudder=0.0)
    t.check("it rolls on its wheels", peak.get("roll", 0.0) > 5.0 and peak.get("squash", 0.0) > 0.0,
            "%.1f m/s at the rim, struts %.2f m in" % (peak.get("roll", 0.0), peak.get("squash", 0.0)))
    t.check("the nose wheel steers with the rudder", peak.get("steer", 0.0) > math.radians(1.0), "%.1f deg" % deg(peak.get("steer", 0.0)))
    actuate(throttle=0.0, brake_left=1.0, brake_right=1.0)
    ok = yield t.until(lambda: math.hypot(g.state.velocity_ned_ms[0], g.state.velocity_ned_ms[1]) < 0.3, 60.0, "stopped")
    t.check("the wheel brakes stop it", ok)
    yield t.wait(3.0)


@scenario("platform.airflow", group=GV, title="The air around it", refs="viewer.md#effects", suite="Platform",
          summary="A fighter low over the bay and out to sea, flown into each condition the viewer's effects are drawn from: idle "
                  "power (the hot gas bending the scene behind it - too cool at idle for the pipe to glow), a hard pull at 250 m/s "
                  "(condensation over the wings and in the leading-edge extensions' vortices, wing tip vortices), and "
                  "Mach 0.95 down low (the vapour cone: a collar round the wing roots ending at the shock; full dry power, "
                  "the pipe a dull orange from astern). Each check "
                  "reads the state numbers the effect is drawn by. Watch it in the viewer with the effects on (key e).",
          options=[aircraft("f16c", WINGS)])
def airflow(t):
    kind = t.opt("aircraft")
    p = t.performance(kind)
    v = t.spawn(kind, -3000.0, 4000.0, agl_m=600.0, heading_deg=t.heading_deg(0.0))  # (west: over the hills of the peninsula, 400 m, and out to sea)
    peak = {"g": 0.0, "alpha": 0.0, "mach": 0.0, "tas": 0.0}

    def record():
        s = v.state
        peak["g"] = max(peak["g"], s.load_factor)
        peak["vortex"] = max(peak.get("vortex", 0.0), tip_vortex(s.alpha_rad, s.airspeed_true_ms, s.altitude_msl_m))
        peak["alpha"] = max(peak["alpha"], deg(s.alpha_rad))
        peak["mach"] = max(peak["mach"], s.mach)
        peak["tas"] = max(peak["tas"], s.airspeed_true_ms)
    t.every(record)
    t.metric("true airspeed, m/s", lambda: v.state.airspeed_true_ms)
    t.metric("Mach", lambda: v.state.mach)
    t.metric("load factor, g", lambda: v.state.load_factor)
    t.metric("angle of attack, deg", lambda: deg(v.state.alpha_rad))
    t.metric("core speed (N2), %", lambda: v.state.engine_n2[0])
    alt = v.state.altitude_msl_m
    # idle: as slow as it comfortably flies, the throttle back
    a = t.watch(v.submit_hsa(heading_rad=v.state.euler_rad[2], altitude_m=alt, speed=1.25 * p.min_cas_ms,
                             speed_reference="calibrated_airspeed"), "hsa: slow")
    yield t.until(lambda: v.state.airspeed_calibrated_ms < 1.35 * p.min_cas_ms, 90.0, "slowed")
    n2 = v.state.engine_n2[0]
    t.check("the engine runs at low power (its heat haze)", n2 > 20.0 or v.state.engine_rpm[0] > 0.0,
            "core speed %.0f %%" % n2)
    # fast: the air streaks
    fast = min(250.0, 0.85 * p.max_tas_ms)
    a.update(speed=fast, speed_reference="true_airspeed")
    ok = yield t.until(lambda: v.state.airspeed_true_ms > fast - 10.0, 120.0, "%.0f m/s" % fast)
    yield t.wait(5.0)
    # a hard pull: vapour and vortices
    peak["g"], peak["alpha"], peak["vortex"] = 0.0, 0.0, 0.0
    v.command_acceleration(load_factor_g=min(6.5, max(2.0, p.max_load_factor)), roll_rate_rad_s=0.0, throttle=1.0)
    yield t.wait(4.0)
    if p.max_load_factor >= 4.0:
        # (the viewer's model: the vortex core's pressure deficit from the lift and the speed, past what condenses)
        t.check("the pull is hard and fast enough for wing tip vortices", peak["vortex"] > 1.0,
                "%.1f deg, %.1f g at the most; the core's deficit %.1f times what condenses" % (peak["alpha"], peak["g"], peak["vortex"]))
        # (the viewer's model: the suction peak over the wing from the angle of attack and the Mach, past what condenses)
        over = wing_vapour(deg_to_rad(peak["alpha"]), peak["tas"], alt)
        t.check("fast and loaded enough for condensation over the wings", over > 1.0,
                "%.1f deg at Mach %.2f: %.1f times what condenses" % (peak["alpha"], peak["mach"], over))
    else:
        t.log("the %s pulls %.1f g at most: no vortices or vapour to see" % (kind, p.max_load_factor))
    t.log("vapour over the wings wants 10 deg and 3.5 g or more: %.1f deg, %.1f g" % (peak["alpha"], peak["g"]))
    a = t.watch(v.submit_hsa(heading_rad=v.state.euler_rad[2], altitude_m=alt, speed=fast, speed_reference="true_airspeed"),
                "hsa: back level")
    yield t.until(lambda: abs(v.state.altitude_msl_m - alt) < 100.0 and abs(v.state.euler_rad[1]) < 0.1, 120.0, "level again")
    # Mach 0.97 low down: the vapour cone
    top = p.max_mach if math.isfinite(p.max_mach) else p.max_tas_ms / 340.0  # (its summary gives one or the other)
    if top >= 0.95:
        target = min(0.97, top - 0.005) * 340.0
        a.update(speed=target, speed_reference="true_airspeed")
        ok = yield t.until(lambda: v.state.mach > 0.95, 150.0, "Mach 0.95")
        t.check("Mach 0.94 to 1.03 below 3 km: the vapour cone", ok and v.state.altitude_msl_m < 3000.0,
                "Mach %.2f at %.0f m" % (v.state.mach, v.state.altitude_msl_m))
        t.log("over the wings at %.1f deg: %s" % (deg(v.state.alpha_rad), "a veil too" if wing_vapour(v.state.alpha_rad, v.state.airspeed_true_ms, alt) > 1.0
                                                   else "too little lift for a veil; the collar only"))
        yield t.wait(8.0)
    else:
        t.log("the %s does not fly Mach 0.95 (its limit %.2f): no vapour cone" % (kind, top))
