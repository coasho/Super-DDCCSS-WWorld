"""TEMPORARY (an OMS stand-in, see __init__): what each fighter senses of the other.

- radar: detects at radar_m * RCS^(1/4) (the radar equation), the RCS the target shows from where the radar sees it
  (a stealth fighter is stealthiest head-on), within its gimbal off the nose; a track after 2 s of detections, lost
  after 3 s without. Low-probability-of-intercept radars (the F-22's, the F-35's) are heard by a warning receiver at
  half the range they track at, the others at all of it.
- IRST: angles only, at irst_m head-on - further with the target's afterburner, its speed (a hot skin) and from behind
  (its exhaust) - within 70 deg off the nose.
- the radar warning receiver: spiked while the other's radar tracks it (and hears it); a launch warning when the
  other shoots while it is spiked.
- missile warning: a missile's seeker gone active on it, or (the ultraviolet/infra-red warner) a missile closer than
  15 km - with a bearing and roughly how far."""
import math

from .data import FIGHTERS
from .frame import norm, off_nose, sub, unit

LPI = {"f22a": 0.5, "f35a": 0.5}
MAWS_M = 15000.0


def aspect(target_pos, target_vel, viewer_pos):
    """0 head-on (the target flying at the viewer) .. pi tail-on."""
    to_viewer = unit(sub(viewer_pos, target_pos))
    v = unit(target_vel)
    return math.acos(max(-1.0, min(1.0, to_viewer[0] * v[0] + to_viewer[1] * v[1] + to_viewer[2] * v[2])))


def rcs_seen(kind, asp):
    """The RCS the radar sees: the frontal one head-on, larger abeam (the flat sides and fins), between from behind."""
    front = FIGHTERS[kind]["rcs_m2"]
    return front * (1.0 + 9.0 * math.sin(asp) ** 4 + 2.0 * max(-math.cos(asp), 0.0))


def radar_range(viewer, target, tpos, tvel, vpos):
    return FIGHTERS[viewer]["radar_m"] * rcs_seen(target, aspect(tpos, tvel, vpos)) ** 0.25


def irst_range(viewer, ts, tpos, tvel, vpos):
    base = FIGHTERS[viewer]["irst_m"]
    if base <= 0.0:
        return 0.0
    hot = 1.0 + 0.8 * max(ts.afterburner[0], 0.0) + 0.6 * max(ts.mach - 1.0, 0.0)
    tail = 1.0 + 0.8 * max(-math.cos(aspect(tpos, tvel, vpos)), 0.0)
    return base * hot * tail


class Sensors:
    """One fighter's view of its bandit, kept from tick to tick."""

    def __init__(self, kind):
        self.kind = kind
        self.seen_s = 0.0        # how long its radar has seen the bandit without a break
        self.unseen_s = 99.0     # ...and how long since it last did
        self.track = False       # a radar track (range, velocity: enough to shoot on)
        self.irst = False        # an infra-red track (angles)
        self.first_track = None  # (time, range) of its first radar track
        self.first_irst = None

    def update(self, dt, now, me, bandit_kind, mpos, mvel, bpos, bvel, bstate):
        rng = norm(sub(bpos, mpos))
        heading = math.atan2(mvel[1], mvel[0])
        off = off_nose(heading, mpos, mvel, bpos)
        seen = rng <= radar_range(self.kind, bandit_kind, bpos, bvel, mpos) and off <= math.radians(FIGHTERS[self.kind]["gimbal_deg"])
        if seen:
            self.seen_s += dt
            self.unseen_s = 0.0
        else:
            self.seen_s = 0.0
            self.unseen_s += dt
        if not self.track and self.seen_s >= 2.0:
            self.track = True
            if self.first_track is None:
                self.first_track = (now, rng)
        elif self.track and self.unseen_s >= 3.0:
            self.track = False
        self.irst = rng <= irst_range(self.kind, bstate, bpos, bvel, mpos) and off <= math.radians(70.0)
        if self.irst and self.first_irst is None:
            self.first_irst = (now, rng)
        return rng

    def hears(self, other, rng, their_range_m):
        """Whether its warning receiver hears the other's radar tracking it: the other holds a track and is within
        the share of the range it tracks at (their_range_m) a warning receiver hears its kind of radar at."""
        return other.track and rng <= LPI.get(other.kind, 1.0) * their_range_m
