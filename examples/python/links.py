"""Links between agents (ADR-31, AB-4; docs/sdk/agents.md, "Links"): three MAs that know of each other only what their
links carry.

    fsim python examples/python/links.py             (flightsim-viewer shows the world "links-demo" while it runs)
    fsim python examples/python/links.py --isolated  (each MA in a process of its own - ADR-31, AB-6 - the same flight
                                                      to the bit; the world unpublished)

The range builds a two-ship of F-16Cs, a C2 node on the ground and a bandit it scripts itself, puts the blue systems on
one Link 16 net and gives each a Data Dissemination Plan: the lead's MA core publishes its position each second and its
radar's tracks every two; the wingman's, its position. Then it hands each MA its agent and keeps the referee.

- The lead's MA searches with its radar - its tracks reach the others as entity reports, by its MA core.
- The C2 node's MA watches the picture the net builds for it - both fighters' position reports, the lead's tracks - and
  once a track has been held for a while sends the wingman a free-form "commit" naming it (by its source SystemID and
  track number: never an entity), reliably, to the wingman's SystemID.
- The wingman's MA, its own radar off, waits for that order, designates the received track (a network track) and
  pursues it - flown on the lead's last report carried forward, never the bandit's true position, which no MA can read.

The referee reports, from its own knowledge, the true range to the bandit, which entity the wingman's target is, and the
net's traffic.
"""
import math
import os
import struct
import sys
import time

import fsim
import fsim.isolation

COMMIT = fsim.TOPIC_FREE_FORM + 1  # (the MAs' own topic: "commit on this track")


class LeadMa:
    """Search; its MA core shares the tracks (its plan)."""

    def __init__(self, agent):
        self.agent, self.searching = agent, False

    def step(self):
        a = self.agent
        if not self.searching:
            if a.payload("radar").state != fsim.PayloadState.OPERATE:
                a.set_payload_state("radar", "operate")
                return "radar warming up"
            a.amti_search("radar", elevation_centre_rad=math.radians(-1.0), bars=4)
            self.searching = True
        return "searching: %d tracks" % len(a.amti_tracks("radar"))


class C2Ma:
    """Watch the picture; commit the wingman on a track held for ten seconds."""

    def __init__(self, agent):
        self.agent, self.first_seen, self.order, self.wingman = agent, {}, None, None

    def step(self):
        a = self.agent
        peers = {p.system: p for p in a.peers() if p.has_position}
        tracks = a.link_tracks()
        for t in tracks:
            self.first_seen.setdefault((t.source, t.payload, t.track), a.time)
        if self.order is None and tracks and len(peers) == 2:
            # (the wingman: the peer that shares no tracks)
            sources = {t.source for t in tracks}
            self.wingman = next(s for s in peers if s not in sources)
            held = [t for t in tracks if a.time - self.first_seen[(t.source, t.payload, t.track)] >= 10.0]
            if held:
                t = max(held, key=lambda t: t.updates)
                self.order = a.send(COMMIT, struct.pack("<IIQ", t.source, t.payload, t.track), destinations=[self.wingman])
                return "commit sent to %#x: track %d of %#x" % (self.wingman, t.track, t.source)
        if self.order is not None:
            ((_, delivery),) = a.transmission(self.order).destinations
            return "commit %s" % ("delivered" if delivery == fsim.DELIVERY_DELIVERED else "pending")
        return "picture: %d peers, %d tracks" % (len(peers), len(tracks))


class WingMa:
    """Wait for C2's commit; designate the received track and pursue it."""

    def __init__(self, agent):
        self.agent, self.target, self.order = agent, 0, None

    def step(self):
        a = self.agent
        for m in a.receive():  # (what arrived since it last looked)
            if m.topic == COMMIT:
                self.order = struct.unpack("<IIQ", m.data)
        if self.order is None:
            return "holding: no order"
        live = {d[0] for d in a.designations() if d[3]}
        if self.target not in live:
            self.target = a.designate_link(*self.order)
            if not self.target:
                return "ordered, the track not in its picture"
            a.submit_behavior("pursuit", target=self.target)
        held = [t for t in a.link_tracks() if (t.source, t.payload, t.track) == self.order]
        age = a.time - held[0].time_s if held else float("nan")
        return "pursuing target %#x (network track, report %.1f s old)" % (self.target, age)

    def designated(self):
        """Its target's id (for the range's report)."""
        return self.target


class _InProcess:
    """An MA in the range's process (tiers 1 and 2): called directly."""

    def __init__(self, ma):
        self.ma = ma

    def step(self):
        return self.ma.step()

    def call(self, method, *args):
        return getattr(self.ma, method)(*args)

    def close(self):
        pass


def run(isolated=False, seconds=150, quiet=False):
    """The engagement: each MA in the range's process, or (isolated) each in a process of its own (ADR-31, AB-6) - the
    same calls in the same order, so the same flight to the bit. Its record: what the referee knows at the end."""
    # --- the range: aircraft, a C2 node, a net, the plans, the agents ------------------------------------------------
    w = fsim.World("links-demo", workers=2, publish=not isolated)  # (isolated: no shared-memory segment to read truth from)
    lead = w.create_vehicle("viper-1", "jsbsim:f16c", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=6000.0,
                            airspeed_ms=230.0, heading_deg=90.0)
    wing = w.create_vehicle("viper-2", "jsbsim:f16c", latitude_deg=39.93, longitude_deg=-0.03, altitude_msl_m=5800.0,
                            airspeed_ms=230.0, heading_deg=90.0)
    bandit = w.create_vehicle("bandit", "jsbsim:mig29a", latitude_deg=40.05, longitude_deg=0.65, altitude_msl_m=7000.0,
                              airspeed_ms=230.0, heading_deg=300.0)
    bandit.submit_hsa(heading_rad=math.radians(300.0), speed=230.0, altitude_m=7000.0)  # (the range's own script)
    c2 = w.create_c2_node("crc-1", 39.6, -0.5, 300.0)
    net = w.create_net("l16-blue")
    for entity, system, tail in ((lead, 0x5101, "VIPER11"), (wing, 0x5102, "VIPER12"), (c2, 0x5C01, "CRC1")):
        w.assign_system(entity, system, tail)
        w.join_net(entity, net)
    w.set_dissemination(lead, position_period_s=1.0, status_period_s=10.0, tracks_period_s=2.0)
    w.set_dissemination(wing, position_period_s=1.0, status_period_s=10.0)
    here = os.path.dirname(os.path.abspath(__file__))
    mas = []
    for name, entity, agent_name, cls in (("lead", lead, "blue-1", "LeadMa"), ("C2", c2, "crc-1", "C2Ma"),
                                          ("wing", wing, "blue-2", "WingMa")):
        agent = w.create_agent(entity, agent_name)
        mas.append((name, fsim.isolation.AgentProcess(w, agent, "links:" + cls, path=[here]) if isolated
                    else _InProcess(globals()[cls](agent))))
    ref = w.referee()

    # --- the run: the MAs decide, the range steps, the referee judges ------------------------------------------------
    steps_per_s = round(1.0 / w.step_seconds)
    start = time.perf_counter()
    try:
        for second in range(seconds):
            reports = {name: ma.step() for name, ma in mas}
            w.step(steps_per_s)
            if second % 10 == 0 and not quiet:
                own, them = ref.state(wing.id), ref.state(bandit.id)
                true_km = math.dist(own.position_ecef, them.position_ecef) / 1e3
                target = mas[2][1].call("designated")
                whose = ref.designation_truth(target) if target else 0
                print("t=%3d s" % second)
                for name, report in reports.items():
                    print("    %-4s MA: %s" % (name, report))
                print("    referee: wingman to bandit %.1f km, its target %s" % (true_km, "is the bandit" if whose == bandit.id else "-"))
        s = ref.net_stats(net)
        record = {"states": {v: tuple(ref.state(v).position_ecef) + tuple(ref.state(v).velocity_ned_ms) for v in (lead.id, wing.id, bandit.id)},
                  "net": tuple(s), "audit": ref.audit(), "target": mas[2][1].call("designated")}
    finally:
        for _, ma in mas:
            ma.close()
    if not quiet:
        print("net: %d transmissions, %d delivered, %d out of range, %d masked, %d lost, %d dropped by queue" %
              (s.transmissions, s.delivered, s.out_of_range, s.masked, s.lost, s.queue_dropped))
        print("done in %.1f s of wall time%s" % (time.perf_counter() - start, " (each MA in its own process)" if isolated else ""))
    return record


def main():
    run(isolated="--isolated" in sys.argv)


if __name__ == "__main__":
    main()
