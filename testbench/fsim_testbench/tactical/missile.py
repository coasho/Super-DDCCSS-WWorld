"""TEMPORARY (an OMS stand-in, see __init__): a notional long-range air-to-air missile - a point mass with a motor (one
pulse, or two for the PL-15), drag rising through the sound barrier and with every g it pulls, a turn limit from the
dynamic pressure; proportional navigation, lofted at long range; the target from the shooter over a datalink - its
radar track, or an infra-red bearing on the picture's range - while it has one, else where it was going; its own seeker active near the end, at a range that shrinks with
the target's stealth; a target beaming it from below can sit in its Doppler notch and break the lock; a proximity
fuse. And from it, the launch zone: the longest range it reaches a target flying on (Rmax) and the longest it
catches one that turns away at the shot (Rne). Numbers are notional, rounded from public estimates."""
import math

from .frame import G, add, atmosphere, dot, norm, sub, unit

# mass (kg), diameter (m), motor pulses ((start s, burn s, thrust N, propellant kg), ...), seeker range on a 5 m^2
# target (m), turn limit (g), lift per dynamic pressure (the body and fins' CL at the limit)
MISSILES = {
    "aim120d": dict(name="AIM-120D", mass=157.0, diameter=0.178, pulses=((0.0, 8.0, 17500.0, 52.0),), seeker_m=18000.0,
                    max_g=40.0, cl=18.0),
    "pl15": dict(name="PL-15", mass=210.0, diameter=0.203, pulses=((0.0, 6.0, 21000.0, 48.0), (22.0, 6.0, 12000.0, 28.0)),
                 seeker_m=22000.0, max_g=38.0, cl=18.0),
    "r77m": dict(name="R-77M", mass=190.0, diameter=0.200, pulses=((0.0, 7.0, 19000.0, 50.0), (20.0, 5.0, 10000.0, 20.0)),
                 seeker_m=20000.0, max_g=38.0, cl=18.0),
}
LETHAL_M = 12.0        # the warhead's reach
PK = 0.9               # its chance of a kill inside it
SEEKER_FOV = math.radians(35.0)
NOTCH_MS = 45.0        # a target whose speed along the missile's line of sight is under this sits in the clutter
NOTCH_BREAK_S = 1.2    # ...for this long, and the seeker lets go


def drag_cd(mach):
    """Zero-lift drag coefficient: low subsonic, a peak just past Mach 1, falling as the speed rises."""
    if mach < 0.8:
        return 0.30
    if mach < 1.1:
        return 0.30 + 0.38 * (mach - 0.8) / 0.3
    return 0.68 * (1.1 / mach) ** 0.75


class Missile:
    """One missile in flight, in the arena's frame (north, east, up)."""

    def __init__(self, kind, shooter, target, pos, vel, now, target_rcs):
        self.k = MISSILES[kind]
        self.kind, self.shooter, self.target = kind, shooter, target
        self.pos, self.vel = pos, add(vel, unit(vel), 25.0)  # (off the rail, a little faster than the aircraft)
        self.mass = self.k["mass"]
        self.t0, self.t = now, 0.0
        self.active = False         # its own seeker has the target
        self.lost = False           # its seeker let go (the notch), or the datalink gave out with no seeker
        self.notch = 0.0
        self.est_pos, self.est_vel = None, None  # where it thinks the target is
        self.seeker_m = self.k["seeker_m"] * (max(target_rcs, 1e-6) / 5.0) ** 0.25
        self.state = "flying"       # flying, hit, missed (and why)
        self.why = ""
        self.closest = 1e12

    def area(self):
        return math.pi * 0.25 * self.k["diameter"] ** 2

    def thrust(self):
        f = 0.0
        for start, burn, thrust, prop in self.k["pulses"]:
            if start <= self.t < start + burn:
                f += thrust
        return f

    def burning(self):
        return self.thrust() > 0.0

    def step(self, dt, target_pos, target_vel, told, rng):
        """Fly dt seconds: target_pos/vel the truth (its seeker's, its fuse's); told what the shooter's datalink says
        of the target now - (pos, vel), or None while it says nothing."""
        if self.state != "flying":
            return
        rho, a = atmosphere(self.pos[2])
        V = max(norm(self.vel), 1.0)
        mach = V / a
        # what it guides on: the truth once its seeker has it, the shooter's track over the datalink, else where
        # the target was going
        to = sub(target_pos, self.pos)
        rng_true = norm(to)
        if not self.active and not self.lost:
            if told is not None:
                self.est_pos, self.est_vel = told
            elif self.est_pos is None:
                self.est_pos, self.est_vel = target_pos, target_vel
            else:
                self.est_pos = add(self.est_pos, self.est_vel, dt)
            looking = sub(self.est_pos, self.pos)
            if norm(looking) <= self.seeker_m and rng_true <= self.seeker_m and \
                    math.acos(max(-1.0, min(1.0, dot(unit(to), unit(self.vel))))) < SEEKER_FOV:
                self.active = True
            elif told is None and norm(looking) < 0.5 * self.seeker_m:
                # (at the point the shooter last saw it, and nothing there)
                self.lost, self.why = True, "lost: no datalink, nothing in the seeker"
        if self.active and not self.lost:
            self.est_pos, self.est_vel = target_pos, target_vel
            # the Doppler notch: the target's speed along the line of sight, looking down into the clutter
            radial = abs(dot(target_vel, unit(to)))
            if radial < NOTCH_MS and target_pos[2] < self.pos[2]:
                self.notch += dt
                if self.notch > NOTCH_BREAK_S and rng.random() < 0.65 * dt / 0.5:
                    self.lost, self.why = True, "lost in the notch"
            else:
                self.notch = 0.0
        if self.lost:
            self.est_pos = add(self.est_pos, self.est_vel, dt)
        # proportional navigation (N 4) on the aim point - lofted while far
        aim = self.est_pos
        rng_est = norm(sub(aim, self.pos))
        if rng_est > 25000.0 and not self.active:
            aim = add(aim, (0.0, 0.0, min(0.3 * (rng_est - 20000.0), 9000.0)))
        los = sub(aim, self.pos)
        r = max(norm(los), 1.0)
        rel_v = sub(self.est_vel, self.vel)
        omega = tuple(c / (r * r) for c in (los[1] * rel_v[2] - los[2] * rel_v[1], los[2] * rel_v[0] - los[0] * rel_v[2],
                                            los[0] * rel_v[1] - los[1] * rel_v[0]))
        closing = -dot(rel_v, unit(los))
        vhat = unit(self.vel)
        acc = (4.0 * closing * (omega[1] * vhat[2] - omega[2] * vhat[1]), 4.0 * closing * (omega[2] * vhat[0] - omega[0] * vhat[2]),
               4.0 * closing * (omega[0] * vhat[1] - omega[1] * vhat[0]))
        acc = add(acc, (0.0, 0.0, G))  # (holding itself up)
        q = 0.5 * rho * V * V
        limit = min(self.k["max_g"] * G, q * self.area() * self.k["cl"] / self.mass)
        lateral = sub(acc, tuple(dot(acc, vhat) * c for c in vhat))
        n_lat = norm(lateral)
        if n_lat > limit:
            lateral = tuple(c * limit / n_lat for c in lateral)
            n_lat = limit
        # along the path: thrust, drag (and the drag of every g it pulls), gravity
        drag = q * self.area() * drag_cd(mach) + 0.08 * (self.mass * n_lat) ** 2 / max(q * self.area(), 1.0)
        thrust = self.thrust()
        along = (thrust - drag) / self.mass
        accel = add(add(lateral, vhat, along), (0.0, 0.0, -G))
        self.vel = add(self.vel, accel, dt)
        self.pos = add(self.pos, self.vel, dt)
        for start, burn, f, prop in self.k["pulses"]:
            if start <= self.t < start + burn:
                self.mass -= prop / burn * dt
        self.t += dt
        # the end: the fuse at the closest approach, or out of energy, or the ground
        d = norm(sub(target_pos, self.pos))
        if d < self.closest:
            self.closest = d
        elif self.closest < 400.0:
            self.state = "hit" if self.closest <= LETHAL_M and rng.random() < PK else "missed"
            self.why = "fused %.0f m from it" % self.closest
            return
        if not self.burning() and self.t > 4.0 and (norm(self.vel) < 1.05 * a or self.t > 150.0):
            self.state, self.why = "missed", "out of energy (%.0f m/s)" % norm(self.vel)
        elif self.pos[2] < 50.0:
            self.state, self.why = "missed", "into the ground"


def flyout(kind, shooter_alt, shooter_speed, target_alt, target_speed, rng_m, target_turns_away, dt=0.1):
    """Whether it reaches a target from this range (head-on at the shot; the target either flies on at its speed or
    turns away and runs at it) - straight line, perfect datalink, no notch."""
    import random
    m = Missile(kind, None, None, (0.0, 0.0, shooter_alt), (shooter_speed, 0.0, 0.0), 0.0, 5.0)
    m.seeker_m = 1e9  # (guided to the end: the zone is the missile's reach, not its seeker's)
    tp = (rng_m, 0.0, target_alt)
    tv = (target_speed if target_turns_away else -target_speed, 0.0, 0.0)
    r = random.Random(1)
    while m.state == "flying" and m.t < 160.0:
        m.step(dt, tp, tv, (tp, tv), r)
        tp = add(tp, tv, dt)
        if m.closest < 60.0:
            return True
    return m.state == "hit" or m.closest < 60.0


_zones = {}


def launch_zone(kind, shooter_alt, shooter_speed, target_alt, target_speed):
    """(Rmax, Rne) in m for a head-on shot at these heights and speeds - from the flyouts, cached on a coarse grid."""
    key = (kind, round(shooter_alt / 1500.0), round(shooter_speed / 60.0), round(target_alt / 1500.0), round(target_speed / 60.0))
    z = _zones.get(key)
    if z is None:
        ha, va, ht, vt = key[1] * 1500.0, key[2] * 60.0, key[3] * 1500.0, key[4] * 60.0

        def longest(away):
            lo, hi = 3000.0, 200000.0
            for _ in range(11):
                mid = 0.5 * (lo + hi)
                if flyout(kind, max(ha, 1000.0), max(va, 150.0), max(ht, 500.0), max(vt, 150.0), mid, away):
                    lo = mid
                else:
                    hi = mid
            return lo
        z = _zones[key] = (longest(False), longest(True))
    return z
