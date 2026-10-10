"""MAs for test_isolation.py: run in processes of their own (fsim.isolation), each holding only its agent's channel."""
import math

import fsim


class Prober:
    """Tries what an isolated MA can and cannot reach, and reports."""

    def __init__(self, agent, label):
        self.agent, self.label = agent, label

    def identity(self):
        a = self.agent
        return (self.label, a.vehicle_id, a.agent_name, a.controller)

    def reach(self):
        """Every way out it might try: none answers but its agent's calls."""
        a, out = self.agent, {}
        for name in ("_world", "_vehicle", "world", "referee", "step", "create_vehicle", "vehicle", "network"):
            try:
                getattr(a, name)
                out[name] = "reached"
            except AttributeError:
                out[name] = "refused"
        return out

    def fly(self, heading_deg):
        """An HSA, its activity held here and used again."""
        act = self.agent.submit_hsa(heading_rad=math.radians(heading_deg), speed=230.0, altitude_m=6000.0)
        self.activity = act
        return act.id, act.live, act.level

    def update(self, heading_deg):
        self.activity.update(heading_rad=math.radians(heading_deg))
        return self.activity.setpoint()

    def own_ship(self):
        return self.agent.ownship()

    def refused(self):
        """An agent call the platform refuses: the same exception as in one process."""
        try:
            self.agent.submit_hsa(heading_rad=0.0, speed=230.0, altitude_m=6000.0, source=fsim.Source.OVERRIDE)
        except fsim.Error as e:
            return type(e).__name__, str(e)
        return None

    def fail(self):
        raise ValueError("the MA's own mistake")

    def bytes_round_trip(self, data):
        return data, list(data), (1.5, float("nan"), -0.0)
