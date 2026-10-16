"""A pure planning function for the composition example: no state, no randomness - the same inputs give the same plan.
A planner component calls it; an upper-layer runtime would put its own planner here."""
import math


def plan_route(latitude_rad, longitude_rad, goal_latitude_rad, goal_longitude_rad, altitude_m, legs, speed_ms=230.0):
    """`legs` evenly spaced points from the own position to the goal (the last is the goal), each at `altitude_m`:
    `ma.actions.v1` route waypoints. Nothing for a position that is not known."""
    if latitude_rad is None or longitude_rad is None or legs < 1:
        return []
    points = []
    for k in range(1, legs + 1):
        f = k / legs
        points.append({"latitude_rad": round(latitude_rad + f * (goal_latitude_rad - latitude_rad), 9),
                       "longitude_rad": round(longitude_rad + f * (goal_longitude_rad - longitude_rad), 9),
                       "altitude_m": float(altitude_m), "speed": float(speed_ms)})
    return points


def bearing_rad(latitude_rad, longitude_rad, goal_latitude_rad, goal_longitude_rad):
    """The initial great-circle bearing to the goal, 0 .. 2 pi."""
    d_lon = goal_longitude_rad - longitude_rad
    y = math.sin(d_lon) * math.cos(goal_latitude_rad)
    x = math.cos(latitude_rad) * math.sin(goal_latitude_rad) - math.sin(latitude_rad) * math.cos(goal_latitude_rad) * math.cos(d_lon)
    b = math.atan2(y, x)
    if b < 0.0:
        b += 2.0 * math.pi
    return b + 0.0 if b < 2.0 * math.pi - 1e-9 else 0.0  # (due north is 0, never 2 pi or a negative zero)
