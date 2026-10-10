"""Tactics: aircraft against each other. A beyond-visual-range engagement between two fifth-generation fighters, each
flown by decision rules standing in for OMS (fsim_testbench.tactical - TEMPORARY, until OMS exists): sensors (radar
against the other's stealth, IRST, warning receivers), notional missiles (boost and coast, datalink, seeker, notch),
and the decisions a pilot makes on them - commit on the controller's picture, shoot inside the launch zone, crank to
support the missile, notch or drag when warned, abort short of the other's no-escape range, egress when out of
missiles. Nobody is shot down: a kill is scored and the set ends, and the next begins, for as long as it runs. The
aircraft are JSBSim's; the sensors, the missiles and the decisions are computed here."""
import math

from ..registry import aircraft, number, scenario, suite
from ..tactical.agent import Agent, Engagement
from ..tactical.data import FIGHTERS
from ..tactical.frame import Frame, atmosphere
from ..tactical.missile import MISSILES
from ._fa import R, deg, offset_rad, rad

suite("Tactics", "Aircraft against each other, flown by decision rules standing in for OMS")
GB = "BVR: beyond visual range"
FIFTH = ("f22a", "f35a", "j20a", "su57")
ARENA = (37.45, -123.45)  # over the Pacific, some 90 km west-south-west of SFO
VISUAL_M = 15000.0        # closer than this the fight is within visual range
TICK_S = 0.25
STEADY_DEG = 15.0         # a roll through this bank one way and then the other is a reversal


@scenario("tactics.bvr", group=GB, title="A BVR engagement: two fifth-generation fighters", suite="Tactics",
          interactive=True,
          summary="Two fifth-generation fighters (F-22A, F-35A, J-20A, Su-57) fight beyond visual range over the Pacific, "
                  "set after set, each flown by decision rules standing in for OMS: it commits on the controller's picture, "
                  "finds the other on its own radar (at a range the other's stealth cuts to the fourth root of its RCS) or "
                  "IRST, shoots its notional AIM-120D, PL-15 or R-77M inside its reach on the other as it flies (from the "
                  "missile's own flyouts: Rmax head-on down to the no-escape range at a target running away), cranks to keep "
                  "the datalink, notches or drags when warned of a missile (at 6 g, keeping its energy), aborts short of the "
                  "other's no-escape range with no shot of its own, and leaves the fight when out of missiles, fuel or ways in. "
                  "Nobody is shot down: as on a training range a hit scores the kill and the controller calls \"knock it off\"; "
                  "both turn cold, separate to the start range, rearm and refuel, and turn in for the next set. It runs until "
                  "stopped (or for the duration given); the decisions and the score are in the log.",
          options=[aircraft("f22a", FIFTH, label="Blue", name="blue"), aircraft("su57", FIFTH, label="Red", name="red"),
                   number("range", "Start range", 160.0, 80.0, 250.0, 10.0, "km"),
                   number("altitude", "Fight altitude", 11000.0, 7000.0, 13000.0, 500.0, "m"),
                   number("duration", "Duration", 0.0, 0.0, 600.0, 5.0, "min", help="0: until stopped"),
                   number("seed", "Seed", 1, 1, 999, 1, "", help="The picture's errors, the notch, the warheads")])
def bvr(t):
    fight = t.opt("altitude")
    half = 500.0 * t.opt("range")
    duration = 60.0 * t.opt("duration")
    lat0, lon0 = map(rad, ARENA)
    lat0 += t.lane_north_m / R  # (each run its own lane of the air)
    frame = Frame(lat0, lon0)
    agents = []
    for name, east, heading in (("blue", -half, 90.0), ("red", half, 270.0)):
        lat, lon = offset_rad(lat0, lon0, 0.0, east)
        kind = t.opt(name)
        v = t.spawn(kind, at=(deg(lat), deg(lon)), alt_m=fight - 1000.0, heading_deg=heading,
                    speed_ms=1.1 * atmosphere(fight - 1000.0)[1], name=name)
        agents.append(Agent(name, kind, v, fight, t.watch, t.log))
    for a in agents:
        d = FIGHTERS[a.kind]
        t.log("%s: %s - RCS %.4f m^2, radar %.0f km on 1 m^2, IRST %s, %d x %s" % (
            a.name, a.kind, d["rcs_m2"], d["radar_m"] / 1000.0, "%.0f km" % (d["irst_m"] / 1000.0) if d["irst_m"] else "none",
            d["missiles"], MISSILES[d["missile"]]["name"]))
    eng = Engagement(frame, agents, int(t.opt("seed")), t.log, 2.0 * half)
    t.metric("set", lambda: "%d, %s" % (len(eng.sets), "fight's on" if eng.phase == "fight" else "knocked off, separating"))
    t.metric("score", lambda: ", ".join("%s %d" % (a.name, a.scored) for a in agents))
    t.metric("range, km", lambda: "%.1f" % (eng.closest_now / 1000.0) if hasattr(eng, "closest_now") else "-")
    for a in agents:
        t.metric(a.name, lambda a=a: "%s, Mach %.2f, %.0f m, bank %.0f, %d missiles%s%s" % (
            a.mode, a.v.state.mach, a.v.state.altitude_msl_m, math.degrees(a.v.state.euler_rad[0]), a.missiles,
            ", track" if a.sensors.track else ", irst" if a.sensors.irst else "", ", spiked" if a.spiked else ""))
    t.metric("missiles in flight", lambda: ", ".join("%s's %s %s%.1f km" % (
        m.shooter.name, MISSILES[m.kind]["name"], "active " if m.active else "",
        math.dist(m.pos, frame.of(m.target.v.state)[0]) / 1000.0) for m in eng.flying()) or "none")

    # steadiness: wing rocking is a roll one way through 15 deg of bank and back the other way within 6 s, while
    # cruising, committing or cranking (a defensive turn reverses on purpose)
    steady = {a: dict(side=0, t=-1e9, reversals=0, seconds=0.0) for a in agents}

    def sample():
        for a in agents:
            if a.mode not in ("commit", "crank", "reset", "egress"):
                continue
            s = steady[a]
            s["seconds"] += 0.1
            r = math.degrees(a.v.state.euler_rad[0])
            side = 1 if r > STEADY_DEG else -1 if r < -STEADY_DEG else 0
            if side:
                if s["side"] and side != s["side"] and t.world.time - s["t"] < 6.0:
                    s["reversals"] += 1
                s["side"], s["t"] = side, t.world.time
    t.every(sample)

    # what it shows, kept up to date set by set
    checks = dict(
        found=t.expect("someone finds the other on its own sensors"),
        shots=t.expect("every shot is taken on a track (radar, or IRST well inside), inside the shooter's reach on the bandit"),
        bvr=t.expect("every set's first shot is taken beyond visual range"),
        ends=t.expect("the sets end - a scored kill, or a fighter out of the fight - and the next begins"),
        nobody=t.expect("nobody is shot down: both fly on through every set"))
    for a in agents:
        checks["steady " + a.name] = t.expect("%s flies steadily: no wing rocking while committing, cranking or separating" % a.name)
        checks["flying " + a.name] = t.expect("%s keeps flying: never below Mach 0.5 or 1.5 km" % a.name)

    def judge():
        done = [s for s in eng.sets if s["end"] is not None]
        checks["found"].set(any(a.sensors.first_track or a.sensors.first_irst for a in agents), "; ".join(
            "%s %s" % (a.name, "radar %.0f km" % (a.sensors.first_track[1] / 1000.0) if a.sensors.first_track else
                       "IRST %.0f km" % (a.sensors.first_irst[1] / 1000.0) if a.sensors.first_irst else "nothing") for a in agents))
        shots = eng.shots
        checks["shots"].set(bool(shots) and all((s["track"] or s["irst"]) and s["range"] <= s["reach"] for s in shots),
                            "%d shots; the longest %.0f km of a reach of %.0f" % (
                                len(shots), max(s["range"] for s in shots) / 1000.0, max(s["reach"] for s in shots) / 1000.0)
                            if shots else "no shots yet")
        firsts = []
        for s in eng.sets:
            inset = [x for x in shots if x["t"] >= s["start"] and (s["end"] is None or x["t"] <= s["end"])]
            if inset:
                firsts.append(inset[0]["range"])
        checks["bvr"].set(bool(firsts) and min(firsts) > VISUAL_M, "first shots at %s km" % ", ".join("%.0f" % (r / 1000.0) for r in firsts))
        checks["ends"].set(len(eng.sets) >= 2, "; ".join("set %d: %s" % (s["n"], s["why"]) for s in done) or "the first set still on")
        checks["nobody"].set(all(not a.v.state.crashed for a in agents),
                             "score %s" % ", ".join("%s %d" % (a.name, a.scored) for a in agents))
        for a in agents:
            s = steady[a]
            per_min = s["reversals"] / max(s["seconds"] / 60.0, 1e-9)
            checks["steady " + a.name].set(per_min <= 0.5, "%d reversals in %.0f s (%.2f a minute)" % (s["reversals"], s["seconds"], per_min))
            checks["flying " + a.name].set(a.lowest_mach >= 0.5 and a.lowest_alt >= 1500.0,
                                           "Mach %.2f, %.0f m at the least" % (a.lowest_mach, a.lowest_alt))

    t0 = t.world.time
    now, last, judged = 0.0, 0.0, 1  # (judged as each set ends)
    while duration <= 0.0 or now < duration:
        now = t.world.time - t0
        # (the simulation's own time since the last tick: the missiles keep pace with the aircraft)
        eng.closest_now = eng.tick(now, max(now - last, 0.0) if now > 0.0 else TICK_S)
        last = now
        if len(eng.sets) != judged:
            judged = len(eng.sets)
            judge()
        yield t.wait(TICK_S)
    judge()
    for a in agents:
        t.log("%s: scored %d, scored against %d" % (a.name, a.scored, a.lost))
