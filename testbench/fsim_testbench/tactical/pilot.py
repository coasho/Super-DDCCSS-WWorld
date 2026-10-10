"""TEMPORARY (an OMS stand-in, see __init__): a pilot on the acceleration level - load factor, roll rate, throttle - that
flies a heading, a height and a speed at a g limit, so a crank pulls 3 g and a defensive turn 7, where the velocity
level's autopilot banks 60 deg at most (2 g)."""
import math

import fsim

from .frame import G, atmosphere, wrap


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class Pilot:
    def __init__(self, vehicle, watch=None, label="pilot"):
        self.v = vehicle
        self.watch, self.label = watch, label
        self.activity = None

    def fly(self, heading, alt, speed, g_max):
        """Towards a heading (rad, true) and a height (m), at a speed (m/s true), pulling no more than g_max."""
        s = self.v.state
        vn, ve, vd = s.velocity_ned_ms
        V = max(math.sqrt(vn * vn + ve * ve + vd * vd), 50.0)
        track = math.atan2(ve, vn)
        vs = -vd
        gamma = math.asin(clamp(vs / V, -1.0, 1.0))
        # the turn: as fast as the limit allows towards the heading, easing in over the last 20 deg or so - the limit
        # what the speed leaves (all of it from Mach 1, 1.5 g at Mach 0.7: high up a sustained 7 g turn bleeds an
        # aircraft to a stall; a pilot keeps his energy and trades height for the turn instead)
        a_s = atmosphere(s.altitude_msl_m)[1]
        n_max = max(1.0 + (g_max - 1.0) * clamp((V - 0.7 * a_s) / (0.3 * a_s), 0.0, 1.0), 1.5)
        omega_max = G * math.sqrt(n_max * n_max - 1.0) / V
        # the turn rate asked for: the heading error closed over some 3 s (more gently still for a few degrees - a
        # pilot corrects 2 deg with 10 deg of bank, not 60), so the roll-out begins before the heading is reached
        # and nothing overshoots it; a turn rate of 1.2 /s per radian of error banked 60 deg for 2 deg and rocked
        # the wings for 8 s after every correction
        err = wrap(heading - track)
        tau = 3.0 + 5.0 * math.exp(-abs(err) / 0.08)
        a_h = V * clamp(err / tau, -omega_max, omega_max)
        # the climb or the dive: a vertical speed towards the height, the flight path following it
        vs_target = clamp(0.06 * (alt - s.altitude_msl_m), -0.3 * V, 0.25 * V)
        # (never pushing below 0.2 g: with the lift gone the bank would be anyone's)
        a_v = max(G * math.cos(gamma) + 0.8 * (vs_target - vs), 0.2 * G)
        # what the wing can give: the limit taken from the turn first, the height kept
        if math.hypot(a_h, a_v) > n_max * G:
            a_h = math.copysign(math.sqrt(max((n_max * G) ** 2 - a_v * a_v, 0.0)), a_h)
        n = clamp(math.hypot(a_h, a_v) / G, -1.0, n_max)
        bank = math.atan2(a_h, a_v)
        # the roll towards that bank, damped by the roll rate already under way
        p = clamp(2.5 * wrap(bank - s.euler_rad[0]) - 0.3 * s.angular_rate_body_rad_s[0], -math.radians(200.0), math.radians(200.0))
        throttle = clamp(0.8 + 0.03 * (speed - s.airspeed_true_ms), 0.0, 1.0)
        fields = dict(load_factor_g=n, roll_rate_rad_s=p, throttle=throttle)
        if self.activity is not None:
            try:
                self.activity.update(**fields)
                return
            except fsim.Rejected:
                self.activity = None  # (it ended - another command took the axes, or the vehicle refused it): afresh
        a = self.v.submit(fsim.Level.ACCELERATION, **fields)
        self.activity = self.watch(a, self.label) if self.watch else a
