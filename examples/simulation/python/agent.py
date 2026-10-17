"""An agent and a referee (ADR-31; docs/sdk/agents.md): the range builds the world and judges it, the MA holds only its
agent.

    fsim python examples/simulation/python/agent.py        (flightsim-viewer shows the world "agent-demo" while it runs)

The MA - `SimpleMa` below - sees what its F-16C's own radar and reports show: it searches, designates the nearest
track as its target, and pursues it - flown on the track's estimate, never the bandit's true position, which it cannot
read. The range steps the world; the referee reports the true range to the bandit, and which entity the MA's target is,
from its own knowledge.
"""
import math
import time

import fsim


def bearing(lat1, lon1, lat2, lon2):
    """The initial bearing from one point to another (rad)."""
    y = math.sin(lon2 - lon1) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(lon2 - lon1)
    return math.atan2(y, x)


class SimpleMa:
    """A scripted MA: search, designate the nearest track, pursue it. It holds its agent, nothing else."""

    def __init__(self, agent):
        self.agent = agent
        self.searching = False
        self.target = 0
        self.pursuit = None

    def step(self):
        a = self.agent
        if not self.searching:
            if a.payload("radar").state != fsim.PayloadState.OPERATE:
                a.set_payload_state("radar", "operate")
                return "radar warming up"
            a.amti_search("radar", elevation_centre_rad=math.radians(-1.0), bars=4)
            self.searching = True
            return "searching"
        tracks = a.amti_tracks("radar")
        if not tracks:
            return "no track yet"
        track = min(tracks, key=lambda t: t.range_m)
        live = {d[0] for d in a.designations() if d[3]}
        if self.target not in live:  # (none yet, or its track was dropped: designate afresh)
            self.target = a.designate("radar", track.id)
            self.pursuit = a.submit_behavior("pursuit", target=self.target)
        own = a.ownship()  # (its own position: its navigation's report)
        heading = bearing(own.latitude_rad, own.longitude_rad, track.latitude_rad, track.longitude_rad)
        return "pursuing target %#x: track %d at %.1f km, bearing %.0f deg" % (self.target, track.id, track.range_m / 1e3,
                                                                                math.degrees(heading) % 360)


def main():
    # --- the range: the world, its aircraft, the agent it hands to the MA, the referee it keeps ---------------------
    w = fsim.World("agent-demo", workers=2)
    viper = w.create_vehicle("viper", "jsbsim:f16c", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=6000.0,
                             airspeed_ms=230.0, heading_deg=90.0)
    bandit = w.create_vehicle("bandit", "jsbsim:mig29a", latitude_deg=40.05, longitude_deg=0.65, altitude_msl_m=7000.0,
                              airspeed_ms=230.0, heading_deg=300.0)
    bandit.submit_hsa(heading_rad=math.radians(300.0), speed=230.0, altitude_m=7000.0)  # (the range's own script)
    ma = SimpleMa(w.create_agent(viper, "blue-1"))
    ref = w.referee()
    print(ma.agent)

    # --- the run: the MA decides, the range steps, the referee judges --------------------------------------------------
    steps_per_s = round(1.0 / w.step_seconds)
    start = time.perf_counter()
    for second in range(150):
        report = ma.step()
        w.step(steps_per_s)
        if second % 10 == 0:
            own, them = ref.state(viper.id), ref.state(bandit.id)
            true_km = math.dist(own.position_ecef, them.position_ecef) / 1e3
            whose = ref.designation_truth(ma.target) if ma.target else 0
            print("t=%3d s  MA: %-62s  referee: true range %.1f km, target %s" %
                  (second, report, true_km, "is the bandit" if whose == bandit.id else "-"))
    print("done in %.1f s of wall time" % (time.perf_counter() - start))


if __name__ == "__main__":
    main()
