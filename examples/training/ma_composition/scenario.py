"""The example's range: a two-ship scenario with a C2 node (the contract fixture's), the MA system behind the lead's
flight service, and a scripted C2 that declares a role and sends one patrol task over the modelled link.

The range holds the episode and the referee; the MA system holds one endpoint and nothing else. The C2 script is the
range's stand-in for another system's MA - it too uses only its own endpoint."""
import json

from . import contracts
from .components import CoordinationPolicy, NavigationPolicy, RoutePlanner, TaskExecutive
from .system import ReferenceSystem

LEAD_FLIGHT, CRC = 1, 4
LEAD_SYSTEM, CRC_SYSTEM = "1392508929", "1392508931"
BASE_SEED = 18446744073709551615
GOAL = (0.6370451769779303, -2.007128639793479)  # (the patrol point)

CONFIG = {
    "schema": "ma.config.v1", "env_id": "9007199254740993", "base_seed": str(BASE_SEED),
    "timing": {"dt_s": 0.008333333333333333, "frame_skip": 4, "decision_steps": 30},
    "scenario": {
        "world": {"name": "ma-composition", "capacity": 8, "terrain": False},
        "vehicles": [
            {"name": "lead", "type": "jsbsim:f16c", "initial": {"lat_deg": 36.0, "lon_deg": -115.0, "alt_msl_m": 7000, "heading_deg": 0, "airspeed_ms": 230}},
            {"name": "wing", "type": "jsbsim:f16c", "initial": {"lat_deg": 35.99, "lon_deg": -115.01, "alt_msl_m": 7000, "heading_deg": 0, "airspeed_ms": 230}}]},
    "c2_nodes": [{"name": "crc", "latitude_rad": 0.6283185307179586, "longitude_rad": -2.007128639793479, "altitude_msl_m": 600.0}],
    "systems": [{"platform": "lead", "system_id": LEAD_SYSTEM, "tail_number": "AF-01"},
                {"platform": "wing", "system_id": "1392508930", "tail_number": "AF-02"},
                {"platform": "crc", "system_id": CRC_SYSTEM, "tail_number": "CRC-1"}],
    "nets": [{"name": "l16-a", "members": ["lead", "wing", "crc"], "spec": {"latency_s": 0.1, "max_payload_bytes": 1024}}],
    "dissemination": [{"platform": "lead", "position_period_s": 1.0}, {"platform": "wing", "position_period_s": 1.0},
                      {"platform": "crc", "status_period_s": 10.0}],
    "services": [
        {"service_id": str(LEAD_FLIGHT), "name": "lead-flight", "platform": "lead",
         "command_capabilities": ["fsim.guidance.hsa", "fsim.guidance.route"], "owns_dms_ingress": True, "owns_dms_egress": True},
        {"service_id": "2", "name": "lead-sensors", "platform": "lead", "command_capabilities": ["fsim.ms.status:radar", "fsim.ms.amti:radar"],
         "owns_dms_ingress": False, "owns_dms_egress": False},
        {"service_id": str(CRC), "name": "crc-c2", "platform": "crc", "command_capabilities": [], "owns_dms_ingress": True, "owns_dms_egress": True},
        {"service_id": "9007199254740995", "name": "wing-ma", "platform": "wing", "command_capabilities": ["*"],
         "owns_dms_ingress": True, "owns_dms_egress": True}],
    "task": {"criteria": [{"id": "time", "kind": "tick_limit", "outcome": "truncated", "max_ticks": 600}]},
}


def config_text(jsbsim_root=None):
    document = json.loads(json.dumps(CONFIG))
    if jsbsim_root:
        document["scenario"]["world"]["jsbsim_root"] = jsbsim_root
    return json.dumps(document)


def make_system():
    """The lead flight service's MA system: four components, one configured resource owner (its C2 reports' topic)."""
    components = [NavigationPolicy(GOAL), CoordinationPolicy(CRC_SYSTEM), TaskExecutive(), RoutePlanner(GOAL)]
    return ReferenceSystem(LEAD_FLIGHT, components, owners={"dms:1001": CoordinationPolicy.component_id})


class C2Script:
    """The range's stand-in C2: declares its role, then tasks the lead with a patrol - through its own endpoint."""

    def __init__(self, endpoint):
        self.endpoint, self.request = endpoint, 0

    def tick(self):
        snapshot = json.loads(self.endpoint.observation_json())
        tick, requests = int(snapshot["tick"]), []
        if tick == 0:
            requests.append(("c2", "declare_role", {"role_id": "2", "callsign": "CRC LEAD"}))
        elif tick == 1:
            requests.append(("c2", "send_task", {"ma_system_id": LEAD_SYSTEM, "type": "patrol", "priority": 1,
                                                 "latitude_rad": GOAL[0], "longitude_rad": GOAL[1], "radius_m": 3000.0,
                                                 "text": "patrol north point"}))
        documents = []
        for kind, operation, parameters in requests:
            self.request += 1
            documents.append({"request_id": str(self.request), "kind": kind, "operation": operation, "parameters": parameters})
        self.endpoint.submit_json(json.dumps({"schema": "ma.actions.v1", "episode": snapshot["episode"],
                                              "service_id": str(CRC), "tick": snapshot["tick"], "requests": documents}))


def run_episode(episode, system, local_episode_index, ticks):
    """One generation: reset, begin the system's episode, run its ticks beside the C2 script, end it with the role's
    permitted feedback. The system's trace."""
    episode.reset(local_episode_index)
    lead, c2 = episode.endpoint(LEAD_FLIGHT), C2Script(episode.endpoint(CRC))
    key = json.loads(lead.observation_json())["episode"]
    system.begin_episode(contracts.context(BASE_SEED, key, LEAD_FLIGHT, [c.component_id for c in system.components]))
    for _ in range(ticks):
        c2.tick()
        system.tick(lead)
        episode.step()
    outcome = json.loads(episode.referee().outcome_json())
    system.end_episode({"episode_done": outcome["episode_done"], "reason": outcome["reason"]})  # (the role's feedback alone)
    return system.trace()
