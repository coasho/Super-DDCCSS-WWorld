"""TEMPORARY (an OMS stand-in, see __init__): a flat local frame over the arena and the standard atmosphere."""
import math

R = 6371008.8
G = 9.80665


class Frame:
    """North, east, up (m) about an origin (lat, lon in radians) - flat over the 200 km of an arena."""

    def __init__(self, lat0, lon0):
        self.lat0, self.lon0 = lat0, lon0
        self.k = math.cos(lat0)

    def of(self, s):
        """A vehicle state's position and velocity: ((n, e, u), (vn, ve, vu))."""
        p = ((s.latitude_rad - self.lat0) * R, (s.longitude_rad - self.lon0) * R * self.k, s.altitude_msl_m)
        v = (s.velocity_ned_ms[0], s.velocity_ned_ms[1], -s.velocity_ned_ms[2])
        return p, v


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def add(a, b, k=1.0):
    return (a[0] + k * b[0], a[1] + k * b[1], a[2] + k * b[2])


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def norm(a):
    return math.sqrt(dot(a, a))


def unit(a):
    n = norm(a)
    return (a[0] / n, a[1] / n, a[2] / n) if n > 1e-9 else (1.0, 0.0, 0.0)


def bearing(a, b):
    """From position a to b, true (rad)."""
    return math.atan2(b[1] - a[1], b[0] - a[0])


def wrap(x):
    return (x + math.pi) % (2.0 * math.pi) - math.pi


def atmosphere(h):
    """(density kg/m^3, speed of sound m/s) in the standard atmosphere."""
    h = max(h, 0.0)
    if h < 11000.0:
        t = 288.15 - 0.0065 * h
        rho = 1.225 * (t / 288.15) ** 4.2559
    else:
        t = 216.65
        rho = 0.36392 * math.exp(-(h - 11000.0) / 6341.6)
    return rho, math.sqrt(1.4 * 287.05 * t)


def off_nose(heading, pos, vel, target):
    """How far (rad) the target is off the velocity vector, horizontally - the angle a radar's gimbal sees it at."""
    return abs(wrap(bearing(pos, target) - (math.atan2(vel[1], vel[0]) if math.hypot(vel[0], vel[1]) > 1.0 else heading)))
