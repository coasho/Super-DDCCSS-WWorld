"""Control-channel deflection signs in source geometry and JSBSim frames."""
import numpy as np


def channel_gain(channel, ctrl, u, where):
    """Local deflection of a control per unit deflection of a channel.

    JSBSim's conventions: elevator and flaps positive trailing edge down;
    aileron positive = left aileron trailing edge down (right roll); rudder
    positive = trailing edge left. A local deflection is positive when the
    trailing edge moves towards -u."""
    if ctrl is None or channel not in ctrl.channels:
        return 0.0
    g = ctrl.channels[channel]
    if channel in ("elevator", "flap"):
        return g * np.sign(u[2]) if abs(u[2]) > 1e-6 else 0.0
    if channel == "aileron":
        side = -1.0 if where[1] > 0 else 1.0  # left (y < 0) down for a positive command
        return g * side * np.sign(u[2]) if abs(u[2]) > 1e-6 else 0.0
    if channel == "rudder":
        return g * np.sign(u[1]) if abs(u[1]) > 1e-6 else 0.0
    return 0.0
