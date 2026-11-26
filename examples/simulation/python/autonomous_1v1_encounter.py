"""SIM-M09 (Mission Systems M1): Minimal autonomous 1v1 engagement loop via public Python SDK.

Both fighters operate strictly behind their fsim.Agent handles (zero World/Referee truth access):
- Power on radar, RWR, countermeasures, and store stations;
- Search in Track-While-Scan, acquire and designate an own-radar track;
- Gate employment on flyout-derived Agent.launch_zone_v2("aim-120c", designation);
- Request strike release consent over modeled RF DMS from a Blue C2 node (QB role);
- Defend upon RWR warning with a beam/crank maneuver and native chaff dispensing.

Run headless:
    python examples/simulation/python/autonomous_1v1_encounter.py --headless
"""
from __future__ import annotations

import argparse
import json
import math
import time

import fsim


def _find_payload(agent: fsim.Agent, family: str):
    for p in agent.payloads:
        if p.family == family:
            return p.index
    return None


class AutonomousAgentPilot:
    """Minimal scripted agent holding ONLY an fsim.Agent handle (ADR-31 B6)."""

    def __init__(self, agent: fsim.Agent, store: str, heading_deg: float, alt_m: float, speed_ms: float):
        self.agent = agent
        self.store = store
        self.heading_deg = heading_deg
        self.alt_m = alt_m
        self.speed_ms = speed_ms
        self.radar = _find_payload(agent, "air_radar")
        self.rwr = _find_payload(agent, "esm")
        self.stations = _find_payload(agent, "store_station")
        self.cmds = _find_payload(agent, "countermeasures")
        self.powered = False
        self.searching = False
        self.fired = False
        self.defending = False
        self.designation = 0
        self.chaff_count = 0
        self.last_chaff_s = -10.0

    def step(self, now_s: float) -> None:
        a = self.agent
        if not self.powered:
            self.powered = True
            a.submit_hsa(heading_rad=math.radians(self.heading_deg), altitude_m=self.alt_m, speed=self.speed_ms)
            for idx in (self.radar, self.rwr, self.stations, self.cmds):
                if idx is not None:
                    a.set_payload_state(idx, fsim.PayloadState.OPERATE)
            return

        payloads = {p.index: p for p in a.payloads}
        if not self.searching and self.radar is not None and payloads[self.radar].state == fsim.PayloadState.OPERATE:
            a.amti_search(self.radar, mode="track_while_scan", azimuth_half_width_rad=math.radians(15.0),
                          bars=1, range_scale_m=100e3)
            self.searching = True

        tracks = a.amti_tracks(self.radar) if self.radar is not None else []
        rwr_contacts = a.contacts(self.rwr) if self.rwr is not None else []

        if (not self.fired and tracks and self.stations is not None and
                payloads[self.stations].state == fsim.PayloadState.OPERATE):
            if self.designation == 0:
                self.designation = a.designate(self.radar, tracks[0].id)
            if self.designation != 0:
                dlz = a.launch_zone_v2(self.store, self.designation)
                if dlz is not None and dlz.valid and dlz.in_range:
                    a.strike(store=self.store, quantity=1, target=self.designation, consent=False, payload=self.stations)
                    self.fired = True

        threat = any(c.warning or c.mode in (fsim.EmitterMode.SEEKER, fsim.EmitterMode.LOCK) for c in rwr_contacts)
        if threat and not self.defending:
            self.defending = True
            a.submit_hsa(heading_rad=math.radians((self.heading_deg + 75.0) % 360.0),
                         altitude_m=self.alt_m - 800.0, speed=self.speed_ms + 25.0)
        if threat and self.cmds is not None and now_s - self.last_chaff_s >= 1.5 and self.chaff_count < 4:
            try:
                a.dispense("chaff")
                self.chaff_count += 1
                self.last_chaff_s = now_s
            except fsim.Rejected:
                pass


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="sim-m09-1v1")
    parser.add_argument("--seed", type=int, default=20261126)
    parser.add_argument("--seconds", type=float, default=45.0)
    parser.add_argument("--headless", action="store_true", help="Disable viewer publication and run full speed")
    args = parser.parse_args()

    world = fsim.World(args.world, publish=not args.headless, seed=args.seed, workers=2, pin_workers=False)
    blue = world.create_vehicle("blue_hornet", "jsbsim:fa18c", latitude_deg=35.0, longitude_deg=-118.0,
                                altitude_msl_m=8000.0, heading_deg=0.0, airspeed_ms=270.0)
    red = world.create_vehicle("red_viper", "jsbsim:f16c", latitude_deg=35.25, longitude_deg=-118.0,
                               altitude_msl_m=8000.0, heading_deg=180.0, airspeed_ms=260.0)
    blue.load_store(2, "aim-120c", payload="stations")
    blue.load_store(4, "aim-7m", payload="stations")
    red.load_store(1, "aim-120c", payload="stations")
    red.load_store(9, "aim-120c", payload="stations")

    c2 = world.create_c2_node("blue_awacs", 34.7, -118.0, 9000.0)
    net = world.create_net("blue_c2_net")
    world.join_net(blue, net)
    world.join_net(c2, net)
    world.set_rbac(blue, world.rbac(blue))

    blue_pilot = AutonomousAgentPilot(world.create_agent(blue, "blue_ma"), "aim-120c", 0.0, 8000.0, 270.0)
    red_pilot = AutonomousAgentPilot(world.create_agent(red, "red_ma"), "aim-120c", 180.0, 8000.0, 260.0)
    c2_agent = world.create_agent(c2, "blue_c2")
    ref = world.referee()
    c2_agent.declare_role(fsim.ROLE_QB, "MAGIC-01")

    t0 = time.perf_counter()
    c2_consented = False
    ticks = int(args.seconds / 0.5)
    for _ in range(ticks):
        now_s = world.time
        blue_pilot.step(now_s)
        red_pilot.step(now_s)
        for req in c2_agent.consent_requests():
            if not req.answered:
                if c2_agent.answer_consent(req.source, req.request_id, True):
                    c2_consented = True
        world.step(15)
        if not args.headless:
            ahead = world.time - (time.perf_counter() - t0)
            if ahead > 0:
                time.sleep(ahead)

    wall_s = time.perf_counter() - t0
    weapons = ref.weapons()
    summary = {
        "schema": "fsim.sim-m09.autonomous-1v1.v1",
        "sim_time_s": round(world.time, 3),
        "wall_time_s": round(wall_s, 3),
        "real_time_factor": round(world.time / wall_s, 2) if wall_s > 0 else 0.0,
        "blue_fired": blue_pilot.fired,
        "c2_consented": c2_consented,
        "red_defending": red_pilot.defending,
        "red_chaff_dispensed": red_pilot.chaff_count,
        "weapons_released": len(weapons),
        "weapon_ends": [str(w.end) for w in weapons],
    }
    print(json.dumps(summary))
    return 0 if (blue_pilot.fired and c2_consented and len(weapons) >= 1) else 1


if __name__ == "__main__":
    raise SystemExit(main())
