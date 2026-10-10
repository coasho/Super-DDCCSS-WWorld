"""What the Flight Autonomy suites share: the aircraft lists and small helpers."""
import math

import fsim

from ..registry import suite

suite("FA", "Flight Autonomy (ADR-29): what a vehicle can do on its own, FA-1 to FA-10, and the platform beneath it")

R = 6371008.8
WINGS = ("c172", "skua", "c130j", "ec130h", "f16c", "f15c", "fa18c", "ea18g", "f22a", "f35a", "gripen", "rafale", "typhoon",
         "mirage2000", "j10a", "j20a", "su27s", "su57", "mig29a", "su25", "a10c", "b52h", "c17a", "e3g", "e7a", "kc135r", "kc46a",
         "rc135w", "rq4b", "u2s", "h6k")
FIGHTERS = ("f16c", "f15c", "fa18c", "ea18g", "f22a", "f35a", "gripen", "rafale", "typhoon", "mirage2000", "j10a", "j20a", "su27s",
            "su57", "mig29a")
ROTORS = ("uh60", "uh1h", "iris", "cf2")
ALL = WINGS + ROTORS
DONE = (fsim.ActivityState.COMPLETED,)


def rad(deg):
    return math.radians(deg)


def deg(r):
    return math.degrees(r)


def wrap_deg(x):
    return (x + 180.0) % 360.0 - 180.0


def heading_of(s):
    return deg(s.euler_rad[2]) % 360.0


def track_of(s):
    return deg(math.atan2(s.velocity_ned_ms[1], s.velocity_ned_ms[0])) % 360.0


def ground_speed(s):
    return math.hypot(s.velocity_ned_ms[0], s.velocity_ned_ms[1])


def distance_m(lat1, lon1, lat2, lon2):
    """Between two places in radians, flat over a few tens of kilometres."""
    return math.hypot((lat2 - lat1) * R, (lon2 - lon1) * R * math.cos(0.5 * (lat1 + lat2)))


def offset_rad(lat, lon, north_m, east_m):
    return lat + north_m / R, lon + east_m / (R * math.cos(lat))


def hold_level(v, speed_ms=None):
    """An hsa holding what it flies now (a scenario's start: the aircraft otherwise flies its vehicle default)."""
    s = v.state
    fields = dict(heading_rad=s.euler_rad[2], altitude_m=s.altitude_msl_m)
    if speed_ms:
        fields.update(speed=speed_ms, speed_reference="true_airspeed")
    return v.submit_hsa(**fields)


def state_name(a):
    info = a.info
    return "forgotten" if info is None else info.state.name


def ended(a, *states):
    """It ended in one of these states (COMPLETED if none given)."""
    return a.state in (states or DONE)


def reason(a):
    info = a.info
    return "" if info is None else getattr(info.reason, "name", str(info.reason))
