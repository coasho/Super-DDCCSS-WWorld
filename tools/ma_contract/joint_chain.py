"""The joint chains of M8 (docs/sdk/ma-training-validation.md): fixed request scripts every public consumer replays.

    python tools/ma_contract/joint_chain.py author --python-path build/ucrt64-headless/python \
        [--out tests/fixtures/mission_training/joint]
    python tools/ma_contract/joint_chain.py replay --python-path <package> --chain <chain.txt> --trace <out.trace>

`author` runs each joint scenario once through the public Python endpoints, deciding as it goes - what a C2 node sends
once the lead holds a track, which task id the wing answers, which activity C2's command flies - and records every
submission it made as `<tick> <service id> <requests>` lines. The simulation is deterministic, so the recorded ids are
the ones every later replay meets: `ma_joint` (C++) checks what each case means on the replay's trace, and the installed
C++, C99 and Python consumers replay the same lines and must write the same trace, byte for byte.

A chain file: `config <file>` (beside it; its world's `jsbsim_root` the token @JSBSIM_ROOT@), `ticks <n>`, then the
submission lines in tick order and, within a tick, ascending service id. The trace: for each tick from 0 to n, each
configured service's snapshot (`obs <tick> <service> <document>`, ascending service id), then - before the tick closes
- each submission's acknowledgment (`ack <tick> <service> <document>`), every document as the SDK wrote it.

Standard library only; the package under test comes from --python-path."""
import argparse
import base64
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
CONTRACT = os.path.join(REPO, "tests", "fixtures", "mission_training", "contract", "config.two-ship-c2.json")
TOKEN = "@JSBSIM_ROOT@"

LEAD, SENSORS, CRC, OBS, WING = 1, 2, 4, 5, 9007199254740995
LEAD_SYS, WING_SYS, CRC_SYS, OBS_SYS = "1392508929", "1392508930", "1392508931", "1392508932"


def joint_config(lossy):
    """The joint scenario: the contract's two ships and C2 node, a bogey ahead of the lead, and - in the main chain - a
    second C2 node whose role (LRE: flight commands alone) may not task, control or form packages. The lossy chain
    has the contract's three platforms alone, on a net that loses 60% of attempts."""
    with open(CONTRACT, "rb") as f:
        c = json.loads(f.read().decode("utf-8"))
    c["env_id"] = "8001" if not lossy else "8002"
    c["scenario"]["world"]["name"] = "joint-lossy" if lossy else "joint"
    c["scenario"]["world"]["jsbsim_root"] = TOKEN
    c["task"]["criteria"] = [{"id": "time", "kind": "tick_limit", "outcome": "truncated", "max_ticks": 600}]
    if lossy:
        c["nets"][0]["spec"]["loss_probability"] = 0.6
        c["dissemination"] = []
        return c
    c["scenario"]["vehicles"].append({"name": "bogey", "type": "jsbsim:f16c",
                                      "initial": {"lat_deg": 36.25, "lon_deg": -115.0, "alt_msl_m": 7000, "heading_deg": 180, "airspeed_ms": 200}})
    c["c2_nodes"].append({"name": "obs", "latitude_rad": 0.629, "longitude_rad": -2.0071, "altitude_msl_m": 600.0})
    c["systems"].append({"platform": "obs", "system_id": OBS_SYS, "tail_number": "OBS-1"})
    c["nets"][0]["members"].append("obs")
    # the lead shares its tracks only when its MA says so; C2 is heard only when it speaks; the wing keeps a link
    # track 20 s after its last report
    c["dissemination"] = [{"platform": "lead", "position_period_s": 1.0},
                          {"platform": "wing", "position_period_s": 1.0, "track_timeout_s": 20.0}]
    c["services"].append({"service_id": str(OBS), "name": "obs-c2", "platform": "obs", "command_capabilities": [],
                          "owns_dms_ingress": True, "owns_dms_egress": True})
    return c


class Recorder:
    """An episode driven through its endpoints, every submission recorded."""

    def __init__(self, mission, config):
        self.episode = mission.MissionEpisode(json.dumps(config).replace(TOKEN, ""))
        self.episode.reset(0)
        self.services = sorted(int(s["service_id"]) for s in config["services"])
        self.lines, self.ids, self.tick = [], {}, 0
        self.pending = {}
        self.heard = {s: [] for s in self.services}  # every message each service's snapshots held: (tick, topic)

    def snap(self, service):
        return json.loads(self.episode.endpoint(service).observation_json())

    def ask(self, service, kind, operation, parameters, component=None):
        """A request for this tick; its id."""
        self.ids[service] = self.ids.get(service, 0) + 1
        r = {"request_id": str(self.ids[service]), "kind": kind, "operation": operation, "parameters": parameters}
        if component:
            r["component"] = {"component_id": str(component)}
        self.pending.setdefault(service, []).append(r)
        return self.ids[service]

    def step(self):
        for service in sorted(self.pending):
            snapshot = self.snap(service)
            requests = self.pending[service]
            self.episode.endpoint(service).submit_json(json.dumps({"schema": "ma.actions.v1", "episode": snapshot["episode"],
                                                                    "service_id": str(service), "tick": snapshot["tick"], "requests": requests}))
            self.lines.append("%d %d %s" % (self.tick, service, json.dumps(requests, separators=(",", ":"))))
        self.pending = {}
        self.episode.step()
        self.tick += 1
        for service in self.services:  # (each message is in exactly one snapshot: kept as it passes)
            self.heard[service] += [(self.tick, m["topic"]) for m in self.snap(service)["messages"]["items"]]

    def until(self, what, condition, limit=60):
        """Step until condition() holds - at most `limit` ticks, else the scenario is not what it was written for."""
        for _ in range(limit):
            if condition():
                return
            self.step()
        raise RuntimeError("joint chain: %s did not happen within %d ticks (tick %d)" % (what, limit, self.tick))

    def receipt(self, service, request_id):
        for r in self.snap(service)["receipts"]:
            if r["request_id"] == str(request_id):
                return r
        return None


def message(text):
    return base64.b64encode(text.encode()).decode(), len(text.encode())


def author_main(mission):
    """J2, J4, J6 and J7, one after another, on one episode."""
    r = Recorder(mission, joint_config(False))
    # roles first: C2's own declaration precedes anything it sends
    r.ask(CRC, "c2", "declare_role", {"role_id": "3", "callsign": "CRC AVC"})
    r.ask(OBS, "c2", "declare_role", {"role_id": "4", "callsign": "OBS LRE"})
    r.ask(SENSORS, "sensor", "set_state", {"payload": "0", "state": "operate"})
    r.step()
    r.ask(SENSORS, "sensor", "search", {"payload": "0", "mode": "range_while_search"})
    r.step()
    # J4: a typed task from the authorised role, and one from the role that may not task
    task_request = r.ask(CRC, "c2", "send_task", {"ma_system_id": WING_SYS, "type": "patrol", "priority": 1, "radius_m": 3000.0, "text": "north"})
    r.ask(OBS, "c2", "send_task", {"ma_system_id": WING_SYS, "type": "patrol", "priority": 1, "radius_m": 3000.0, "text": "obs"})
    r.step()
    # J3: C2's task request again, unchanged - answered `duplicate`: no second task
    r.ids[CRC] = task_request - 1
    r.ask(CRC, "c2", "send_task", {"ma_system_id": WING_SYS, "type": "patrol", "priority": 1, "radius_m": 3000.0, "text": "north"})
    r.ids[CRC] = task_request
    r.step()
    r.until("the task's arrival", lambda: r.snap(WING)["reports"]["tasks"]["items"])
    task = r.snap(WING)["reports"]["tasks"]["items"][0]["task_id"]
    r.ask(WING, "task", "accept", {"from_system_id": CRC_SYS, "task_id": task})
    r.step()
    r.step()
    r.ask(WING, "task", "report", {"from_system_id": CRC_SYS, "task_id": task, "state": "accepted", "percent": 50.0})
    r.step()
    # J2: the lead shares its own track once it holds one; the wing, its radar off, designates it once delivered
    r.until("the lead's track", lambda: r.snap(SENSORS)["reports"]["tracks"]["items"])
    track = r.snap(SENSORS)["reports"]["tracks"]["items"][0]["track_id"]
    r.ask(LEAD, "dms", "publish_native", {"report": "tracks", "payload": "0", "destination_system_ids": [WING_SYS]})
    r.step()
    r.until("the track's delivery", lambda: r.snap(WING)["reports"]["link_tracks"]["items"])
    r.ask(WING, "sensor", "designate_link", {"source_system_id": LEAD_SYS, "payload": "0", "track_id": track})
    r.step()
    # J6: C2's direct command holds the wing's flight axes for every component; only C2 cancels it
    hsa = r.ask(CRC, "c2", "command_hsa", {"ma_system_id": WING_SYS, "heading_rad": 1.0, "altitude_m": 7000.0, "speed": 230.0})
    r.step()
    r.until("C2's command", lambda: r.snap(WING)["reports"]["authority"]["commands"])
    c2_activity = r.snap(WING)["reports"]["authority"]["commands"][0]["activity_id"]
    r.ask(WING, "flight", "new", {"mode": "hsa", "heading_rad": 0.2}, component=1)
    r.ask(WING, "flight", "new", {"mode": "hsa", "heading_rad": 0.4}, component=2)
    r.ask(WING, "flight", "cancel", {"activity_id": c2_activity}, component=1)
    r.ask(OBS, "c2", "designate_control", {"ma_system_id": WING_SYS, "modes_mask": 1})
    r.step()
    r.step()
    command = r.receipt(CRC, hsa)["native"]["ids"]["command_id"]
    r.ask(CRC, "c2", "cancel_command", {"ma_system_id": WING_SYS, "command_id": command})
    r.step()
    r.until("C2's command's end", lambda: not r.snap(WING)["reports"]["authority"]["commands"])
    r.ask(WING, "flight", "new", {"mode": "hsa", "heading_rad": 0.3}, component=1)
    r.step()
    r.ask(WING, "task", "report", {"from_system_id": CRC_SYS, "task_id": task, "state": "completed"})
    r.step()
    # J7: lost link and its restoration in simulated time; a leader proposed over the link; the package C2 forms
    r.ask(CRC, "c2", "set_lost_comms", {"ma_system_id": WING_SYS, "threshold_s": 4.0, "plan_id": "0"})
    r.step()
    r.until("the lost link", lambda: r.snap(WING)["reports"]["authority"]["lost_comms"]["lost"])
    r.step()
    r.ask(CRC, "dms", "publish_native", {"report": "status", "destination_system_ids": [WING_SYS]})
    r.step()
    data, size = message("leader:" + LEAD_SYS)
    r.ask(WING, "dms", "send", {"topic": "1001", "destination_system_ids": [CRC_SYS], "byte_length": size, "data_b64": data})
    r.step()
    r.until("the proposal's delivery", lambda: [m for m in r.heard[CRC] if m[1] == "1001"])
    r.ask(OBS, "c2", "form_package", {"package_id": "8", "leader_system_id": WING_SYS, "member_system_ids": [LEAD_SYS, WING_SYS]})
    r.ask(CRC, "c2", "form_package", {"package_id": "7", "leader_system_id": LEAD_SYS, "member_system_ids": [LEAD_SYS, WING_SYS]})
    r.step()
    for _ in range(4):
        r.step()
    # the shared track ages on: stale past 10 s, dropped 20 s after its report
    r.until("the shared track's drop", lambda: not r.snap(WING)["reports"]["link_tracks"]["items"])
    r.step()
    return r, {"task_request": task_request}


def author_lossy(mission):
    """J3's message part: a proposal and its acknowledgment over a link that loses most attempts - each retransmitted
    until delivered, each delivered once - and a resubmitted request, which sends nothing again."""
    r = Recorder(mission, joint_config(True))
    data, size = message("propose:sector-north")
    proposal = {"topic": "1001", "destination_system_ids": [CRC_SYS], "byte_length": size, "data_b64": data}
    r.ask(WING, "dms", "send", proposal)
    r.step()
    r.until("the proposal's delivery", lambda: [m for m in r.heard[CRC] if m[1] == "1001"])
    data, size = message("ack:sector-north")
    r.ask(CRC, "dms", "send", {"topic": "1002", "destination_system_ids": [WING_SYS], "byte_length": size, "data_b64": data})
    r.step()
    # the wing's request again, as it was: answered `duplicate`, never sent again; and its id with another body: refused
    r.ids[WING] = 0
    r.ask(WING, "dms", "send", proposal)
    r.step()
    r.ids[WING] = 0
    r.ask(WING, "dms", "send", dict(proposal, topic="1003"))
    r.step()
    r.until("the acknowledgment's delivery", lambda: [m for m in r.heard[WING] if m[1] == "1002"])
    for _ in range(12):  # (retries settle: nothing arrives twice)
        r.step()
    return r, {}


def write_chain(path, config_name, r):
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write("# Recorded by tools/ma_contract/joint_chain.py author; replayed by ma_joint and the installed consumers.\n")
        f.write("config %s\n" % config_name)
        f.write("ticks %d\n" % r.tick)
        for line in r.lines:
            f.write(line + "\n")


def author(a):
    sys.path.insert(0, os.path.abspath(a.python_path))
    import fsim
    from fsim import mission
    fsim.set_log_level("off")
    os.makedirs(a.out, exist_ok=True)
    for name, lossy, run in (("joint", False, author_main), ("lossy", True, author_lossy)):
        config = joint_config(lossy)
        with open(os.path.join(a.out, "config.%s.json" % name), "w", encoding="utf-8", newline="\n") as f:
            f.write(json.dumps(config, indent=1) + "\n")
        r, _ = run(mission)
        write_chain(os.path.join(a.out, "chain.%s.txt" % name), "config.%s.json" % name, r)
        print("%s: %d ticks, %d submission lines" % (name, r.tick, len(r.lines)), flush=True)
    return 0


def read_chain(path):
    """(config text with @JSBSIM_ROOT@, ticks, [(tick, service, requests text)])."""
    base = os.path.dirname(os.path.abspath(path))
    config, ticks, lines = None, None, []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.rstrip("\n")
            if not line or line.startswith("#"):
                continue
            if line.startswith("config "):
                with open(os.path.join(base, line.split(" ", 1)[1]), encoding="utf-8") as c:
                    config = c.read()
            elif line.startswith("ticks "):
                ticks = int(line.split(" ", 1)[1])
            else:
                tick, service, requests = line.split(" ", 2)
                lines.append((int(tick), int(service), requests))
    return config, ticks, lines


def replay(mission, config, ticks, lines, jsbsim_root=""):
    """The trace of one replay (a list of lines), in-process."""
    episode = mission.MissionEpisode(config.replace(TOKEN, jsbsim_root.replace("\\", "/")))
    episode.reset(0)
    services = sorted(int(s["service_id"]) for s in json.loads(config.replace(TOKEN, ""))["services"])
    trace, k = [], 0
    for tick in range(ticks + 1):
        snapshots = {}
        for service in services:
            snapshots[service] = episode.endpoint(service).observation_json()
            trace.append("obs %d %d %s" % (tick, service, snapshots[service]))
        while k < len(lines) and lines[k][0] == tick:
            _, service, requests = lines[k]
            key = json.dumps(json.loads(snapshots[service])["episode"], separators=(",", ":"))
            ack = episode.endpoint(service).submit_json('{"schema":"ma.actions.v1","episode":%s,"service_id":"%d","tick":"%d","requests":%s}'
                                                        % (key, service, tick, requests))
            trace.append("ack %d %d %s" % (tick, service, ack))
            k += 1
        if tick < ticks:
            episode.step()
    episode.close()
    return trace


def replay_main(a):
    sys.path.insert(0, os.path.abspath(a.python_path))
    import fsim
    from fsim import mission
    fsim.set_log_level("off")
    config, ticks, lines = read_chain(a.chain)
    trace = replay(mission, config, ticks, lines, a.jsbsim_root or "")
    with open(a.trace, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(trace) + "\n")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("action", choices=("author", "replay"))
    ap.add_argument("--python-path", required=True)
    ap.add_argument("--out", default=os.path.join(REPO, "tests", "fixtures", "mission_training", "joint"))
    ap.add_argument("--chain")
    ap.add_argument("--trace")
    ap.add_argument("--jsbsim-root", default=None)
    a = ap.parse_args()
    return author(a) if a.action == "author" else replay_main(a)


if __name__ == "__main__":
    sys.exit(main())
