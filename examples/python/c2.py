"""C2 authority (ADR-31, AB-5a; docs/sdk/agents.md, "C2 authority"): what a C2 node may do to a platform, as A-GRA has
it - only through the platform's MA, by the role it declared.

    fsim python examples/python/c2.py           (flightsim-viewer shows the world "c2-demo" while it runs)

The range builds an F-16C, a control and reporting centre (CRC) and an observer post, both C2 nodes, puts them on one net,
and loads the platform's RBAC configuration (A-GRA's default roles). Then:

- the MA flies east at 6,000 m on its own - its intent, re-commanded every second;
- the observer post declares the Observer role and tries to turn the jet: unauthorized, nothing happens;
- the CRC declares the QB role and sends a direct HSA command - south, climb to 7,000 m. The MA core flies it as C2's:
  the MA's own commands are refused (authority_held) while it flies;
- after 40 s the CRC cancels it, and the MA flies its own intent again - under an altitude override the CRC sets
  (7,000 m for 25 s): the MA's own commands fly it in place of their 6,000 m.

The referee reports the true heading and altitude, and who holds the flight.
"""
import math
import time

import fsim


class EastboundMa:
    """Its own intent: east at 6,000 m - unless C2 holds the flight."""

    def __init__(self, agent):
        self.agent = agent

    def step(self):
        a = self.agent
        c2 = a.authority().commands
        if c2:
            (activity, system, command), = c2
            return "C2 %#x flies its command %d: holding" % (system, command)
        try:
            a.submit_hsa(heading_rad=math.radians(90.0), speed=230.0, altitude_m=6000.0)
            overridden = [o for o in a.overrides() if o.kind == fsim.OVERRIDE_ALTITUDE]
            return "flying its own: east, %s" % ("%.0f m (C2's override)" % overridden[0].value if overridden else "6,000 m")
        except fsim.Rejected as refused:
            return "refused: %s" % refused.reason


class Crc:
    """A QB: takes the jet south for 40 s, then gives it back."""

    def __init__(self, agent, ma):
        self.agent, self.ma, self.order = agent, ma, None

    def step(self, t):
        c = self.agent
        if t == 0:
            c.declare_role(fsim.ROLE_QB, "CRC LEAD")
        if t == 15:
            self.order = c.command_hsa(self.ma, heading_rad=math.radians(180.0), speed=230.0, altitude_m=7000.0)
        if t == 55:
            c.cancel_command(self.ma, self.order)
            c.set_override(self.ma, fsim.OVERRIDE_ALTITUDE, 7000.0, expiry_s=c.time + 25.0)
        states = {fsim.C2_ACCEPTED: "accepted", fsim.C2_REJECTED: "rejected", fsim.C2_UNAUTHORIZED: "unauthorized",
                  fsim.C2_COMPLETED: "completed", fsim.C2_FAILED: "failed", fsim.C2_CANCELED: "canceled"}
        last = [s for s in c.command_statuses() if s.command_id == self.order and s.topic == fsim.TOPIC_FLIGHT_COMMAND]
        return "its command: %s" % (states[last[-1].state] if last else "-")


def main():
    # --- the range ------------------------------------------------------------------------------------------------------
    w = fsim.World("c2-demo", workers=2)
    viper = w.create_vehicle("viper", "jsbsim:f16c", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=6000.0,
                             airspeed_ms=230.0, heading_deg=90.0)
    crc = w.create_c2_node("crc", 39.5, 0.2, 300.0)
    post = w.create_c2_node("observer-post", 39.8, -0.3, 150.0)
    net = w.create_net("c2-net")
    for e in (viper, crc, post):
        w.join_net(e, net)
    w.set_rbac(viper, w.rbac(viper))  # (mission planning: A-GRA's default roles)
    ma = EastboundMa(w.create_agent(viper, "blue-1"))
    boss = Crc(w.create_agent(crc, "crc"), ma.agent.system)
    watcher = w.create_agent(post, "observer")
    ref = w.referee()

    # --- the run ----------------------------------------------------------------------------------------------------------
    steps_per_s = round(1.0 / w.step_seconds)
    start, peek = time.perf_counter(), None
    for t in range(110):
        if t == 0:
            watcher.declare_role(fsim.ROLE_OBSERVER, "WATCH")
        if t == 5:
            peek = watcher.command_hsa(ma.agent.system, heading_rad=math.radians(0.0), speed=230.0, altitude_m=6000.0)
        ma_report, c2_report = ma.step(), boss.step(t)
        w.step(steps_per_s)
        if t % 5 == 0:
            s = ref.state(viper.id)
            print("t=%2d s  MA: %-48s CRC: %-22s  referee: heading %3.0f deg, %5.0f m" %
                  (t, ma_report, c2_report, math.degrees(s.euler_rad[2]) % 360, s.altitude_msl_m))
    seen = [s for s in watcher.command_statuses() if s.command_id == peek]
    print("the observer's command:", "unauthorized" if seen and seen[-1].state == fsim.C2_UNAUTHORIZED else seen)
    print("done in %.1f s of wall time" % (time.perf_counter() - start))


if __name__ == "__main__":
    main()
