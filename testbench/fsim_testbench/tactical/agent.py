"""TEMPORARY (an OMS stand-in, see __init__): the decision rules a fighter flies a beyond-visual-range engagement by -
and the engagement that steps the sensors and the missiles between two of them.

Each tick a fighter knows what its sensors give it (its own radar track; an infra-red track's bearing; else the
ground controller's picture, a few kilometres off and a few seconds old), what warns it (a radar tracking it, a
launch while it is, a missile's seeker active on it, a missile close) and its own missiles and fuel of fight. It
picks, in this order:

- defend   a missile on its way: notch it (beam it, diving into the clutter below) while it is far, drag (turn
           cold, dive, run) when it is close; held until the warning has gone
- support  its own missile still on the datalink: crank (the bandit at the edge of its radar's gimbal, slowing the
           closure while the track holds); once the missile's seeker has it, turn away (skate) if inside the
           other's reach
- shoot    a radar track and the bandit inside 85 % of its reach on it as it flies - Rmax at a bandit coming at it,
           down to Rne at one running away (a longer shot cannot arrive); with no radar track, an infra-red one inside
           60 % of it (the range from the picture: a stealthy bandit's only shot)
- abort    no shot to take and closer than the other's no-escape range against it (and a margin): turn cold
           until it is well outside, then come back
- egress   no missiles left, nine minutes of fight (its fuel), or three defences and aborts without a shot of its own
           between them (no way in): out of the fight once none of its own missiles still flies
- commit   otherwise: an intercept on what it knows of the bandit, climbing to the fight's height and accelerating

Nobody is shot down: as on a training range, a missile that would have killed scores the kill, and the controller
calls "knock it off" - so it does when either fighter is out of the fight. Both turn cold and separate; at the set's
start range they turn in again, rearmed and refuelled, for the next set, for as long as the engagement runs."""
import math
import random

from .data import FIGHTERS, PICTURE_ERROR_M, PICTURE_PERIOD_S
from .frame import Frame, G, add, atmosphere, bearing, norm, off_nose, sub, unit, wrap
from .missile import MISSILES, Missile, launch_zone
from .pilot import Pilot
from .sensors import MAWS_M, Sensors, radar_range


class Agent:
    def __init__(self, name, kind, vehicle, fight_alt, watch, log):
        self.name, self.kind, self.v = name, kind, vehicle
        self.d = FIGHTERS[kind]
        self.fight_alt = fight_alt
        self.log = log
        self.pilot = Pilot(vehicle, watch, "%s: pilot (OMS stand-in)" % name)
        self.sensors = Sensors(kind)
        self.missiles = self.d["missiles"]
        self.mode, self.since = "commit", 0.0
        self.alive = True
        self.picture, self.picture_t = None, -1e9
        self.warnings = {}       # missile -> (time, bearing, range, what)
        self.spiked = False
        self.side = 1.0
        self.events = []
        self.support = []        # (time, bandit within the radar's gimbal) while supporting
        self.defend_lag = []     # (warned at, defending at)
        self.warned_at = None
        self.lowest_mach, self.lowest_alt = 9.0, 1e9
        self.setbacks = 0        # defences and aborts since its last shot: three, and it gives up the fight
        self.committed_at = None
        self.scored, self.lost = 0, 0  # kills it has scored, and been scored against it
        # (its fuel at the start: each set starts full again - a tanker on the range's edge)
        # (tank by tank until they hold all its fuel: asking for a tank past the last is logged as an error)
        self.tanks = []
        total_lbs, held = vehicle.state.fuel_kg / 0.45359237, 0.0
        for i in range(16):
            if held >= total_lbs - 1.0:
                break
            path = "propulsion/tank[%d]/contents-lbs" % i
            self.tanks.append((path, vehicle.get_property(path)))
            held += self.tanks[-1][1]

    def rearm(self, now):
        """A new set: full of missiles and fuel, its doctrine's counts reset, committing on the picture."""
        self.missiles = self.d["missiles"]
        for path, lbs in self.tanks:
            self.v.set_property(path, lbs)
        self.setbacks, self.committed_at = 0, None
        self.warnings.clear()
        self.warned_at = None
        self.picture_t = -1e9
        self.mode, self.since = "commit", now

    def say(self, now, text):
        self.events.append((now, self.mode, text))
        self.log("%5.0f s  %s (%s): %s" % (now, self.name, self.kind, text))

    def go(self, now, mode, text):
        if mode != self.mode:
            self.mode, self.since = mode, now
            self.say(now, text)

    def think(self, eng, now, me, bandit, rng_true, bpos, bvel):
        """This tick's decision: (heading, height, speed, g limit) for the pilot, and whether it shoots."""
        p, v = eng.frame.of(me)
        heading_now = math.atan2(v[1], v[0])
        mach_s = atmosphere(p[2])[1]
        # knock it off: cold, away from the other, to the next set's range
        if eng.phase == "reset":
            self.go(now, "reset", "knock it off: turning cold to separate for the next set")
            return (bearing(p, bpos) + math.pi, self.fight_alt, 0.95 * mach_s, 3.0), False
        # what it knows of the bandit: its own track, else an infra-red bearing on the picture's range, else the picture
        if now - self.picture_t >= PICTURE_PERIOD_S:
            r = eng.rng
            self.picture = (add(bpos, (r.gauss(0.0, PICTURE_ERROR_M), r.gauss(0.0, PICTURE_ERROR_M), r.gauss(0.0, 300.0))), bvel)
            self.picture_t = now
        if self.sensors.track:
            known, kvel = bpos, bvel
        elif self.sensors.irst and self.picture is not None:
            known = add(p, unit(sub(bpos, p)), norm(sub(self.picture[0], p)))
            kvel = self.picture[1]
        else:
            known, kvel = self.picture
        rng = norm(sub(known, p))
        to_bandit = bearing(p, known)
        bstate = bandit.v.state
        rmax, rne = launch_zone(self.d["missile"], p[2], norm(v), known[2], norm(kvel))
        # its reach on the bandit as it flies now: Rmax at one coming at it, down to Rne at one running away
        hot = sum(a * b for a, b in zip(unit(kvel), unit(sub(p, known))))
        reach = rne + (rmax - rne) * 0.5 * (1.0 + hot)
        e_rmax, e_rne = launch_zone(bandit.d["missile"], known[2], norm(kvel), p[2], norm(v))
        mine = [m for m in eng.missiles if m.shooter is self and m.state == "flying"]
        guiding = [m for m in mine if not m.active and not m.lost]
        shoot = False
        # the warnings in force: a seeker on it or a missile close (their bearing, their range); a launch it heard,
        # its missile reckoned on its way at some 1,100 m/s
        # (a seeker's or a close missile's warning lasts while it is renewed; a launch's for the flight it is reckoned)
        live = {m: w for m, w in self.warnings.items()
                if m.state == "flying" and now - w[0] < (w[2] / 900.0 if w[3] == "launch" else 3.0)}
        threat = None
        for m, w in live.items():
            est = w[2] if w[3] != "launch" else max(w[2] - 1100.0 * (now - m.t0), 0.0)
            if w[3] != "launch" or est < 30000.0:
                if threat is None or est < threat[1]:
                    threat = (w[1], est)
        if threat is not None:
            if self.warned_at is None:
                self.warned_at = now
            tgo = threat[1] / 1100.0
            if tgo > 12.0:
                beam = threat[0] + self.side * math.pi / 2.0
                if abs(wrap(beam - heading_now)) > math.pi / 2.0:
                    self.side = -self.side
                    beam = threat[0] + self.side * math.pi / 2.0
                self.go(now, "notch", "missile warning, %.1f km off at %03.0f: notch it, diving" % (threat[1] / 1000.0, math.degrees(threat[0]) % 360))
                cmd = (beam, max(p[2] - 4000.0, 2500.0), 1.3 * mach_s, min(self.d["max_g"], 6.0))
            else:
                self.go(now, "drag", "missile %.1f km off: drag cold, diving" % (threat[1] / 1000.0))
                cmd = (threat[0] + math.pi, max(p[2] - 5000.0, 2000.0), 1.6 * mach_s, min(self.d["max_g"], 6.0))
            if self.mode in ("notch", "drag") and self.warned_at is not None and self.warned_at >= 0.0:
                self.setbacks += 1
                self.defend_lag.append((self.warned_at, now))
                self.warned_at = -1.0  # (counted: until this warning has gone)
            return cmd, False
        self.warned_at = None
        if self.mode in ("notch", "drag") and now - self.since < 6.0:
            # (the warning has gone: a few seconds more of it before trusting that)
            return (heading_now, max(p[2] - 1000.0, 2500.0), 1.4 * mach_s, self.d["max_g"]), False
        # its own missile on the datalink: crank, holding the track at the edge of the gimbal
        if guiding and self.sensors.track:
            off = off_nose(heading_now, p, v, bpos)
            self.support.append((now, off <= math.radians(self.d["gimbal_deg"])))
            crank = math.radians(self.d["gimbal_deg"] - 12.0)
            if self.mode != "crank":
                self.side = 1.0 if wrap(heading_now - to_bandit) >= 0.0 else -1.0
            self.go(now, "crank", "supporting its %s, %.1f km to the bandit: crank" % (MISSILES[self.d["missile"]]["name"], rng / 1000.0))
            return (to_bandit + self.side * crank, self.fight_alt, 1.25 * mach_s, 4.0), False
        if mine and not guiding and rng < 1.1 * e_rmax and self.mode == "crank":
            self.go(now, "skate", "its missile's seeker has the bandit; inside the bandit's reach: turn away")
        if self.mode == "skate" and mine:
            return (to_bandit + math.pi, self.fight_alt - 1500.0, 1.5 * mach_s, 5.0), False
        # out of missiles - or of the fight's fuel (nine minutes of it), or of ways in (three defences or aborts without
        # a shot) - home
        if self.committed_at is None:
            self.committed_at = now
        if (self.missiles == 0 or now - self.committed_at > 540.0 or self.setbacks >= 3) and not mine:
            self.go(now, "egress", "winchester: egress" if self.missiles == 0 else
                    "bingo: egress" if now - self.committed_at > 540.0 else "no way in after %d tries: egress" % self.setbacks)
            return (to_bandit + math.pi, self.fight_alt, 1.4 * mach_s, 4.0), False
        # a shot: its own radar track inside 85 % of its reach on the bandit as it flies (at one running away, its
        # no-escape range: a longer shot cannot arrive); or, with no radar track, an infra-red one well inside it
        # (the range from the picture: a stealthy bandit's only chance)
        irst_shot = not self.sensors.track and self.sensors.irst and rng <= 0.6 * reach
        if (self.sensors.track and rng <= 0.85 * reach or irst_shot) and self.missiles > 0 and not mine:
            self.missiles -= 1
            shoot = True
            self.side = 1.0 if wrap(heading_now - to_bandit) >= 0.0 else -1.0
            self.say(now, "fox 3%s at %.1f km (its reach %.0f km: Rmax %.0f, Rne %.0f; Mach %.2f, %.0f m), %d left"
                     % (" on the IRST" if irst_shot else "", rng / 1000.0, reach / 1000.0, rmax / 1000.0, rne / 1000.0, me.mach, p[2],
                        self.missiles))
            eng.shots.append(dict(t=now, shooter=self.name, range=rng, rmax=rmax, rne=rne, reach=reach,
                                  track=self.sensors.track, irst=irst_shot))
            self.setbacks = 0
            self.mode, self.since = "crank", now
            return (to_bandit, self.fight_alt, 1.3 * mach_s, 3.0), True
        # too close to the bandit's no-escape range with no shot: turn cold, come back from outside it
        mar = e_rne + 8000.0
        if self.mode == "abort":
            if rng > mar + 15000.0:
                self.go(now, "commit", "%.1f km, outside the bandit's reach again: re-commit" % (rng / 1000.0))
            else:
                return (to_bandit + math.pi, self.fight_alt, 1.4 * mach_s, 5.0), False
        elif rng < mar and not (self.sensors.track and self.missiles > 0):
            self.setbacks += 1
            self.go(now, "abort", "%.1f km, inside the bandit's no-escape range + margin (%.0f km) with no shot: abort"
                    % (rng / 1000.0, mar / 1000.0))
            return (to_bandit + math.pi, self.fight_alt, 1.4 * mach_s, 5.0), False
        if self.mode in ("skate", "crank", "notch", "drag") or self.mode == "commit" and now == self.since:
            self.go(now, "commit", "%.1f km: commit" % (rng / 1000.0))
        # commit: an intercept on what it knows - lead the bandit by the time to close
        closing = max(norm(v) + norm(kvel) * 0.5, 200.0)
        lead = add(known, kvel, min(rng / closing, 60.0) * 0.5)
        speed = (1.5 if self.kind == "f22a" else 1.4) * mach_s
        return (bearing(p, lead), self.fight_alt, speed, 3.0), False


class Engagement:
    """Two agents, the missiles between them, the outcome."""

    def __init__(self, frame, agents, seed, log, start_range_m):
        self.frame, self.agents, self.log = frame, agents, log
        self.rng = random.Random(seed)
        self.missiles, self.shots, self.kills = [], [], []
        self.closest = 1e12
        self.first = {}
        self.start_range_m = start_range_m
        self.phase = "fight"   # or "reset": knocked off, separating for the next set
        self.sets = [dict(n=1, start=0.0, end=None, why=None, shots=0, kills=[])]

    def knock_it_off(self, now, why):
        """The set ends: what is still flying is terminated, and both separate for the next."""
        if self.phase != "fight":
            return
        self.phase = "reset"
        s = self.sets[-1]
        s["end"], s["why"] = now, why
        for m in self.flying():
            m.state, m.why = "terminated", "knock it off"
        self.log("%5.0f s  set %d: knock it off - %s" % (now, s["n"], why))

    def new_set(self, now, rng):
        self.phase = "fight"
        self.sets.append(dict(n=len(self.sets) + 1, start=now, end=None, why=None, shots=0, kills=[]))
        for a in self.agents:
            a.rearm(now)
        self.log("%5.0f s  set %d: fight's on - %.0f km, both rearmed and refuelled" % (now, self.sets[-1]["n"], rng / 1000.0))

    def tick(self, now, dt):
        a, b = self.agents
        st = {x: x.v.state for x in self.agents}
        pv = {x: self.frame.of(st[x]) for x in self.agents}
        rng = norm(sub(pv[a][0], pv[b][0]))
        self.closest = min(self.closest, rng)
        for me, other in ((a, b), (b, a)):
            if me.alive:
                me.sensors.update(dt, now, st[me], other.kind, pv[me][0], pv[me][1], pv[other][0], pv[other][1], st[other])
                me.lowest_mach = min(me.lowest_mach, st[me].mach)
                me.lowest_alt = min(me.lowest_alt, st[me].altitude_msl_m)
        for me, other in ((a, b), (b, a)):
            their = radar_range(other.kind, me.kind, pv[me][0], pv[me][1], pv[other][0])
            spiked = other.alive and me.sensors.hears(other.sensors, rng, their)
            if spiked != me.spiked and me.alive:
                me.spiked = spiked
                me.say(now, "spiked: the %s's radar tracks it" % other.kind if spiked else "the spike has gone")
        # the missiles: in 0.05 s steps, 0.01 s in their last kilometres
        for m in self.missiles:
            if m.state != "flying":
                continue
            tgt = m.target
            tp, tv = pv[tgt]
            # the datalink: the shooter's radar track - or its infra-red one, the bearing true and the range the
            # picture's - else nothing
            told = None
            sh = m.shooter
            if sh.alive and sh.sensors.track:
                told = (tp, tv)
            elif sh.alive and sh.sensors.irst and sh.picture is not None:
                sp = pv[sh][0]
                told = (add(sp, unit(sub(tp, sp)), norm(sub(sh.picture[0], sp))), sh.picture[1])
            t, end = 0.0, dt
            while t < end and m.state == "flying":
                h = 0.01 if norm(sub(tp, m.pos)) < 1500.0 else 0.05
                m.step(h, add(tp, tv, t), tv, None if told is None else (add(told[0], told[1], t), told[1]), self.rng)
                t += h
            was_active = m.active
            if m.state != "flying":
                self.log("%5.0f s  %s's %s at %s: %s - %s" % (now, m.shooter.name, MISSILES[m.kind]["name"], tgt.name, m.state, m.why))
                if m.state == "hit" and self.phase == "fight":
                    # (scored, not shot down: the controller calls it, and the set ends)
                    self.kills.append((now, m.shooter.name, tgt.name))
                    self.sets[-1]["kills"].append((now, m.shooter.name, tgt.name))
                    m.shooter.scored += 1
                    tgt.lost += 1
                    tgt.say(now, "killed by %s's %s (scored - no shootdown)" % (m.shooter.name, MISSILES[m.kind]["name"]))
                    self.knock_it_off(now, "%s's %s kills %s" % (m.shooter.name, MISSILES[m.kind]["name"], tgt.name))
                continue
            # what the target is warned of
            d = norm(sub(m.pos, tp))
            what = "seeker" if m.active else "maws" if d < MAWS_M else None
            if m.active and not m.__dict__.get("told"):
                m.told = True
                self.log("%5.0f s  %s's %s is active on %s, %.1f km out" % (now, m.shooter.name, MISSILES[m.kind]["name"], tgt.name, d / 1000.0))
            if what:
                tgt.warnings[m] = (now, bearing(tp, m.pos), d, what)
        # the decisions
        for me, other in ((a, b), (b, a)):
            if not me.alive:
                continue
            (heading, alt, speed, g), shoot = me.think(self, now, st[me], other, rng, pv[other][0], pv[other][1])
            me.pilot.fly(heading, alt, speed, g)
            if shoot:
                self.sets[-1]["shots"] += 1
                p, v = pv[me]
                m = Missile(me.d["missile"], me, other, p, v, now, FIGHTERS[other.kind]["rcs_m2"])
                self.missiles.append(m)
                # (it shot while its radar held the other - which the other's warning receiver hears: a launch)
                if other.spiked or (other.alive and other.sensors.hears(me.sensors, rng,
                                                                         radar_range(me.kind, other.kind, pv[other][0], pv[other][1], p))):
                    other.warnings[m] = (now, bearing(pv[other][0], p), rng, "launch")
                    other.say(now, "launch warning from the %s, %.1f km" % (me.kind, rng / 1000.0))
        # the set's end: a fighter out of the fight (and nothing of its own still flying) - and the next set's start,
        # once they are back out at its range
        if self.phase == "fight":
            for me in self.agents:
                if me.mode == "egress" and not [m for m in self.flying() if m.shooter is me]:
                    self.knock_it_off(now, "%s out of the fight (%s)" % (me.name, me.events[-1][2] if me.events else "egress"))
                    break
        elif rng >= self.start_range_m:
            self.new_set(now, rng)
        return rng

    def flying(self):
        return [m for m in self.missiles if m.state == "flying"]
