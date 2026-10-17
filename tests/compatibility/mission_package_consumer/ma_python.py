"""MA-service binding, legal reports and mission episodes from an installed Python package: `fsim` as installed, its own
library and data, nothing else (docs/sdk/mission-training.md, sections 3 to 16). Usage: ma_python.py <installed python
prefix> <runtime-data inventory> [<joint chain> <trace> <isolated actor trace>]..., or, the gate's installed-data probes,
ma_python.py <installed python prefix> <runtime-data inventory> --probe|--probe-native-read <joint chain>."""
import ctypes
import hashlib
import json
import math
import os
import sys

import fsim

HSA, ROUTE = "fsim.guidance.hsa", "fsim.guidance.route"


def require(condition, message):
    if not condition:
        raise SystemExit("installed MA consumer (Python) failed: " + message)


def inside(path, prefix):
    path, prefix = os.path.realpath(path), os.path.realpath(prefix)
    return os.path.commonpath([os.path.normcase(path), os.path.normcase(prefix)]) == os.path.normcase(prefix)


def loaded_library():
    """The path of the libfsim.dll this process loaded."""
    kernel = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel.GetModuleHandleW.restype = ctypes.c_void_p
    kernel.GetModuleHandleW.argtypes = [ctypes.c_wchar_p]
    kernel.GetModuleFileNameW.argtypes = [ctypes.c_void_p, ctypes.c_wchar_p, ctypes.c_uint32]
    module = kernel.GetModuleHandleW("libfsim.dll")
    require(module, "libfsim.dll is not loaded")
    buffer = ctypes.create_unicode_buffer(32768)
    require(kernel.GetModuleFileNameW(module, buffer, 32768), "cannot inspect the loaded libfsim.dll")
    return buffer.value


def make_world(name):
    return fsim.World(name, publish=False, workers=1, pin_workers=False, terrain=False)


def aircraft(w, name, north_deg=0.0):
    return w.create_vehicle(name, "jsbsim:f16c", latitude_deg=40.0 + north_deg, longitude_deg=0.0, altitude_msl_m=6000.0, airspeed_ms=230.0)


def ownership(flight_first):
    """Who owns what with two services on one platform, the agents made in one order or the other."""
    w = make_world("installed-py-order-%d" % flight_first)
    viper, wing = aircraft(w, "viper"), aircraft(w, "wing", 0.02)
    if flight_first:
        flight, comms = w.create_agent(viper, "flight"), w.create_agent(viper, "comms")
    else:
        comms, flight = w.create_agent(viper, "comms"), w.create_agent(viper, "flight")
    other = w.create_agent(wing, "other")
    # before anything is bound, the other service holds the grant and flies: the binding, not that, decides
    comms.request_control(HSA)
    before = comms.submit_hsa(heading_rad=1.0, speed=230.0, altitude_m=6000.0)
    bindings = [(flight, 1, (HSA, ROUTE), False, False), (comms, 2, (), True, True)]
    for agent, service, capabilities, ingress, egress in (bindings if flight_first else reversed(bindings)):
        w.bind_ma_service(agent, service, capabilities, owns_dms_ingress=ingress, owns_dms_egress=egress)

    class Lookalike:  # (the numbers of an agent, and no agent: only an fsim.Agent of this World binds)
        vehicle_id, controller = viper.id, other.controller

    for agent, service, capabilities in ((comms, 3, (HSA,)), (Lookalike, 4, ("*",)), (w, 4, ("*",))):
        try:
            w.bind_ma_service(agent, service, capabilities, owns_dms_ingress=False, owns_dms_egress=False)
        except fsim.Error as refused:
            require("MA service" in str(refused), "a refused binding gave no reason")
        else:
            require(False, "service %d was bound" % service)
    services = w.ma_services()
    require([s.service_id for s in services] == [1, 2], "a refused binding changed the bindings")
    # what stood before the bindings gave way to them
    live = before.live
    require(not (live() if callable(live) else live), "another service's flight outlived the binding")
    require(viper.control_status(HSA).holder == flight.controller, "a grant held before the binding is not its owner's after it")
    for refused in (lambda: comms.request_control(HSA), lambda: viper.request_control(HSA), lambda: viper.set_control_mode("open")):
        try:
            refused()
        except fsim.Error:
            pass
        else:
            require(False, "a bound platform's ownership was got around")
    mode = viper.control_mode
    require((mode() if callable(mode) else mode) == fsim.ControlMode.GRANTED, "a bound platform was opened")
    flight.submit_hsa(heading_rad=1.0, speed=230.0, altitude_m=6000.0)
    for refused, why in ((lambda: comms.submit_hsa(heading_rad=2.0, speed=230.0, altitude_m=6000.0), ""),
                         (lambda: flight.send(fsim.TOPIC_FREE_FORM + 1, b"\x01"), "DMS egress")):
        try:
            refused()
        except fsim.Error as error:
            require(why in str(error), "refused for another reason: %s" % error)
        else:
            require(False, "a service acted on what another owns")
    by_controller = {s.controller: s.service_id for s in services}
    return (by_controller.get(w.ma_capability_owner(viper, HSA), 0), by_controller.get(w.ma_capability_owner(viper, ROUTE), 0),
            next(s.service_id for s in services if s.owns_dms_ingress), next(s.service_id for s in services if s.owns_dms_egress))


def worlds():
    """Two Worlds mint the same numbers: an agent binds only in the World that made it."""
    one, two = make_world("installed-py-world-one"), make_world("installed-py-world-two")
    a1, a2 = aircraft(one, "viper"), aircraft(two, "viper")
    mine, theirs = one.create_agent(a1, "mine"), two.create_agent(a2, "theirs")
    require((a1.id, mine.controller) == (a2.id, theirs.controller), "the two worlds minted different numbers")
    try:
        one.bind_ma_service(theirs, 1, (HSA,))
    except fsim.Error as refused:
        require("MA service 1" in str(refused), "a refused binding gave no reason")
    else:
        require(False, "another world's agent was bound")
    require(one.ma_services() == [] and two.ma_services() == [] and not a1.control_status(HSA).granted, "a refused binding changed a world")
    one.bind_ma_service(mine, 1, (HSA,))
    require([(s.platform, s.controller) for s in one.ma_services()] == [(a1.id, mine.controller)], "its own agent was not bound")


def reports():
    w = make_world("installed-py-reports")
    viper = aircraft(w, "viper")
    crc = w.create_c2_node("crc", 39.64, 0.0, 200.0)
    net = w.create_net("l16")
    require(w.join_net(viper, net) and w.join_net(crc, net), "the net was not joined")
    blue, boss = w.create_agent(viper, "blue-1"), w.create_agent(crc, "crc")
    boss.declare_role(2, "CRC LEAD")  # (a role that may task the platform)
    w.step(60)
    nav = blue.report_navigation()
    require(isinstance(nav, fsim.Report) and isinstance(nav.meta, fsim.ReportMeta), "the report types are not the package's")
    require((nav.availability, nav.meta.source, nav.meta.validity) == ("available", "own_navigation", "valid"), "navigation: not its own, valid")
    require(nav.meta.sample_time_s == nav.value[0].time_s and nav.meta.receive_time_s == w.time, "navigation: not its own ship's times")
    require(blue.report_navigation(stale_after_s=-1.0).meta.validity == "stale", "a report older than the threshold is not stale")
    empty = fsim.ReportCollection("available", [])
    require(blue.report_tracks() == empty and blue.report_contacts() == empty and blue.report_link_tracks() == empty,
            "a platform whose sensors are off holds something")
    require(blue.contacts("rwr") == [], "contacts: not an agent's read")
    sent = w.time
    task = boss.send_task(blue.system, fsim.TASK_PATROL, latitude_rad=math.radians(40.3), longitude_rad=0.0)
    require(task > 0 and blue.report_tasks() == empty, "a task was there before its delivery")
    w.step(120)
    items = blue.report_tasks().items
    require(len(items) == 1 and items[0].value.task_id == task, "the delivered task is not reported")
    meta = items[0].meta
    require((meta.source, meta.system) == ("link", boss.system) and meta.receive_time_s == items[0].value.received_s > sent,
            "the task's provenance is not its delivery's")
    require(meta.sample_time_s is None and meta.age_s is None and math.isnan(items[0].value.altitude_m), "an unknown was given a value")
    require(len(blue.report_tasks().items) == 1 and len(blue.c2_tasks()) == 1, "reading a report consumed it")
    peers, beats = blue.report_peers().items, blue.report_peer_heartbeats().items
    require(len(beats) == len(peers) and all(b.meta.source == "link" and b.meta.system == p.value.system for b, p in zip(beats, peers)),
            "the heartbeats are not beside the peers")
    require(blue.report_authority().meta.source == "ma_core" and blue.operator_callsigns() == ["CRC LEAD"],
            "authority: not what its MA core keeps")
    require(boss.report_navigation() == fsim.Report("not_applicable", None, None) and boss.report_peers().availability == "available",
            "a C2 node's reports are not what it has")
    viper.remove()
    require(blue.report_navigation() == fsim.Report("unavailable", None, None), "a removed platform still reports")


MISSION_CONFIG = """{"schema":"ma.config.v1","env_id":"7","base_seed":"18446744073709551615",
 "timing":{"dt_s":0.008333333333333333,"frame_skip":4,"decision_steps":30},
 "scenario":{"world":{"name":"installed-mission","capacity":4},
  "vehicles":[{"name":"lead","type":"jsbsim:f16c","initial":{"lat_deg":36.0,"lon_deg":-115.0,"alt_msl_m":7000,"heading_deg":0,"airspeed_ms":230}}]},
 "c2_nodes":[{"name":"crc","latitude_rad":0.6283185307179586,"longitude_rad":-2.007128639793479,"altitude_msl_m":600.0}],
 "systems":[{"platform":"lead","system_id":"1392508929"},{"platform":"crc","system_id":"1392508931"}],
 "nets":[{"name":"l16","members":["lead","crc"]}],
 "services":[{"service_id":"9007199254740995","name":"lead-ma","platform":"lead","command_capabilities":["*"],"owns_dms_ingress":true,"owns_dms_egress":true},
  {"service_id":"4","name":"crc-c2","platform":"crc","command_capabilities":[],"owns_dms_ingress":true,"owns_dms_egress":true}],
 "task":{"criteria":[{"id":"time","kind":"tick_limit","outcome":"truncated","max_ticks":3}]}}"""


class Actor:
    """An actor in its own process (fsim.isolation.MissionEndpointProcess): its endpoint's three calls, nothing else."""

    def __init__(self, endpoint):
        self.endpoint = endpoint

    def observe(self):
        return self.endpoint.observation_json()

    def submit(self, text):
        return self.endpoint.submit_json(text)

    def echo(self, value):
        """What reached this process: ordinary data comes back as it was sent."""
        self.received = getattr(self, "received", 0) + 1
        return value

    def received_count(self):
        return getattr(self, "received", 0)


def mission_batch():
    """A batch of two slots through fsim.mission and the training adapter, from the installed package (M7 core)."""
    other = MISSION_CONFIG.replace('"env_id":"7"', '"env_id":"8"')
    require(other != MISSION_CONFIG, "the second slot needs its own env_id")
    batch = fsim.mission.MissionBatch([MISSION_CONFIG, other], autoreset="same_step", workers=2)
    adapter = fsim.training.TrainingAdapter(batch)

    class Waiting:
        def begin_episode(self, context):
            pass

        def tick(self, endpoint):
            snapshot = json.loads(endpoint.observation_json())
            endpoint.submit_json(json.dumps({"schema": "ma.actions.v1", "episode": snapshot["episode"],
                                             "service_id": snapshot["service"]["service_id"], "tick": snapshot["tick"], "requests": []}))

        def end_episode(self, feedback):
            pass
    for slot in (0, 1):
        adapter.register(slot, 9007199254740995, lambda slot, service: Waiting())
    adapter.reset()
    for _ in range(3):
        adapter.step()
    status = batch.status(0)
    require(status.ended and status.reset and status.generation == 2, "the slot did not end and reset in the same step: %r" % (status,))
    require(any(r.get("final_observation") for r in adapter.actor_stream()), "no final observation in the actor stream")
    require(all(set(r) >= {"outcome", "truth", "costs"} for r in adapter.critic_stream()), "the critic stream is incomplete")
    require(not any("ma.costs" in (r.get("observation") or "") for r in adapter.actor_stream()), "costs reached an actor")
    adapter.close()


def mission_episode():
    """A mission episode through fsim.mission, and an actor of it in its own process, from the installed package."""
    episode = fsim.mission.MissionEpisode(MISSION_CONFIG)
    episode.reset(41)
    lead, referee = episode.endpoint(9007199254740995), episode.referee()
    first = lead.observation_json()
    require(first == lead.observation_json(), "a snapshot read twice differs")
    snapshot = json.loads(first)
    require(snapshot["service"]["service_id"] == "9007199254740995", "an identity above 2**53 is not exact")
    ack = json.loads(lead.submit_json(json.dumps({"schema": "ma.actions.v1", "episode": snapshot["episode"], "service_id": "9007199254740995",
                                                  "tick": "0", "requests": [{"request_id": "1", "kind": "flight", "operation": "new",
                                                                             "parameters": {"mode": "hsa", "heading_rad": 0.5}}]})))
    require(ack["submission"]["requests"][0]["status"] == "queued", "the request was not queued")
    episode.step()
    states = {r["request_id"]: r["state"] for r in json.loads(lead.observation_json())["receipts"]}
    require(states.get("1") == "executing", "the request did not execute natively: %r" % states)
    require(json.loads(referee.truth_json())["schema"] == "ma.truth.v1", "no truth document")
    import fsim.isolation as isolation
    here = os.path.dirname(os.path.abspath(__file__))
    with isolation.MissionEndpointProcess(episode, lead, "ma_python:Actor", path=[here]) as actor:
        require(actor.call("observe") == lead.observation_json(), "the isolated actor read another snapshot")
        # ordinary data crosses: the refusals below are the isolation check's, not a call that could not be made
        ordinary = {"nested": [1, "two", {"three": 3.5}]}
        require(actor.call("echo", ordinary) == ordinary and actor.call("received_count") == 1, "ordinary data did not cross")
        other = fsim.mission.MissionEpisode(MISSION_CONFIG)  # (the same configuration: equal keys and service ids)
        other.reset(41)
        try:
            actor.rebind(other.endpoint(9007199254740995))
            require(False, "an isolated actor was re-bound to another episode's endpoint")
        except ValueError as e:
            require("not one this episode opened" in str(e), "another episode's endpoint was refused for another reason: %s" % e)
        costs = json.loads(referee.costs_json())
        require(costs["schema"] == "ma.costs.v1", "no costs document")
        for document in (json.loads(referee.truth_json()), {"nested": [json.loads(referee.outcome_json())]}, referee.truth_json(),
                         costs, {"nested": [costs]}, referee.costs_json()):
            try:
                actor.call("echo", document)
                require(False, "referee data reached the isolated actor")
            except TypeError as e:
                require("referee document" in str(e), "referee data was refused for another reason: %s" % e)
        require(actor.call("received_count") == 1, "referee data reached the actor's process before it was refused")
        require(actor.call("observe") == lead.observation_json(), "the actor's endpoint changed after the refusals")
        other.close()
    episode.reset(41)
    try:
        lead.observation_json()
        require(False, "an earlier generation's endpoint still answered")
    except fsim.Error:
        pass
    episode.close()
    return snapshot["episode"]


WING = 9007199254740995


def read_chain(path):
    """A joint chain (M8; tools/validation/ma_contract/joint_chain.py): its configuration - the installed package's own JSBSim data
    in place of the token - its ticks and its submission lines."""
    config, ticks, lines = None, None, []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.rstrip("\r\n")
            if not line or line.startswith("#"):
                continue
            if line.startswith("config "):
                with open(os.path.join(os.path.dirname(os.path.abspath(path)), line[7:]), encoding="utf-8") as c:
                    config = c.read().replace("@JSBSIM_ROOT@", "")
            elif line.startswith("ticks "):
                ticks = int(line[6:])
            else:
                tick, service, requests = line.split(" ", 2)
                lines.append((int(tick), int(service), requests))
    return config, ticks, lines


def replay_chain(path, isolated=None):
    """The chain's trace, as every installed consumer writes it; with `isolated` a service whose actor runs in a process
    of its own (fsim.isolation.MissionEndpointProcess), reading and submitting through its proxy alone."""
    import fsim.isolation as isolation
    config, ticks, lines = read_chain(path)
    services = sorted(int(s["service_id"]) for s in json.loads(config)["services"])
    episode = fsim.mission.MissionEpisode(config)
    episode.reset(0)
    here = os.path.dirname(os.path.abspath(__file__))
    actor = isolation.MissionEndpointProcess(episode, episode.endpoint(isolated), "ma_python:Actor", path=[here]) if isolated else None
    trace, k = [], 0
    try:
        for tick in range(ticks + 1):
            snapshots = {}
            for s in services:
                snapshots[s] = actor.call("observe") if s == isolated else episode.endpoint(s).observation_json()
                trace.append("obs %d %d %s" % (tick, s, snapshots[s]))
            while k < len(lines) and lines[k][0] == tick:
                _, s, requests = lines[k]
                key = json.dumps(json.loads(snapshots[s])["episode"], separators=(",", ":"))
                document = '{"schema":"ma.actions.v1","episode":%s,"service_id":"%d","tick":"%d","requests":%s}' % (key, s, tick, requests)
                trace.append("ack %d %d %s" % (tick, s, actor.call("submit", document) if s == isolated else episode.endpoint(s).submit_json(document)))
                k += 1
            if tick < ticks:
                episode.step()
    finally:
        if actor:
            actor.close()
        episode.close()
    return "\n".join(trace) + "\n"


def joint_chains(triplets):
    """Each chain replayed in-process and with the wing's actor isolated: the same trace, written for the gate to compare
    with the C++ and C99 consumers'."""
    for chain, trace, isolated_trace in triplets:
        in_process, alone = replay_chain(chain), replay_chain(chain, isolated=WING)
        require(in_process == alone, "%s: the isolated actor's trace differs from the in-process one" % chain)
        for path, text in ((trace, in_process), (isolated_trace, alone)):
            with open(path, "w", encoding="utf-8", newline="\n") as f:
                f.write(text)
    return len(triplets)


def probe(chain):
    """Whether the chain's episode can be built and reset from the data this process resolves: exit 0 if it can, 3 if
    the SDK refuses it. The gate runs it after the preflight's presence check alone (--probe-native-read) with the
    installed F-16C definition corrupted, and requires the SDK's refusal: the installed copy is the one read."""
    try:
        config, _, _ = read_chain(chain)
        with fsim.mission.MissionEpisode(config) as episode:
            episode.reset(0)
    except Exception as e:  # (the refusal the gate expects, as the SDK words it)
        print("refused: %s: %s" % (type(e).__name__, str(e).splitlines()[0] if str(e) else ""))
        return 3
    print("built and reset from %s" % os.environ.get("FSIM_AIRCRAFT_PATH"))
    return 0


def aircraft_paths(prefix):
    """FSIM_AIRCRAFT_PATH: the installed package's own aircraft and stores, and nothing else - else a development
    library's source aircraft directory would come first."""
    paths = [p for p in os.environ.get("FSIM_AIRCRAFT_PATH", "").split(";") if p]
    require(paths, "FSIM_AIRCRAFT_PATH is not set to the installed package's aircraft")
    for p in paths:
        require(inside(p, prefix) and os.path.isdir(p), "an aircraft directory outside the installed package: " + p)
    return paths


def preflight(prefix, inventory, hashes=True):
    """The runtime data the chains fly, at its installed paths, before any World is made: every file of the expected
    inventory (the staging rules applied to the source tree; the gate writes it as `<sha256>  <path>` lines) present
    under the installed package's aircraft directory and - with `hashes` - byte for byte. A missing or different file
    is reported with its installed path and fails the run (exit 4), although a development library would find a
    source copy after the installed one. Returns how many files it verified, else None."""
    root = prefix.replace("\\", "/").rstrip("/") + "/fsim/share/flightsim/aircraft"
    entries = []
    with open(inventory, encoding="utf-8") as f:
        for line in f:
            if line.strip():
                digest, relative = line.strip().split("  ", 1)
                entries.append((digest, relative))
    require(entries, "the runtime-data inventory is empty: " + inventory)
    problems = []
    for digest, relative in entries:
        path = root + "/" + relative
        if not os.path.isfile(path):
            problems.append("missing: " + path)
        elif hashes:
            with open(path, "rb") as f:
                if hashlib.sha256(f.read()).hexdigest() != digest:
                    problems.append("differs from the expected inventory: " + path)
    for problem in problems:
        print("installed runtime data " + problem)
    return None if problems else len(entries)


def provenance(prefix):
    """The package, its native module and library, and its aircraft paths: the installed package's own."""
    require(inside(fsim.__file__, prefix), "fsim was imported from outside the installed package: " + fsim.__file__)
    require(inside(fsim._native.__file__, prefix), "the native module is not the installed package's")
    library = loaded_library()
    require(inside(library, prefix), "libfsim.dll was loaded from outside the installed package: " + library)
    return library


USAGE = ("usage: ma_python.py <installed python prefix> <runtime-data inventory> [<joint chain> <trace> <isolated actor trace>]... "
         "| ... --probe|--probe-native-read <joint chain>")


def main(argv):
    require(len(argv) >= 3, USAGE)
    prefix, inventory, rest = argv[1], argv[2], argv[3:]
    library = provenance(prefix)
    paths = aircraft_paths(prefix)
    if rest[:1] in (["--probe"], ["--probe-native-read"]):
        require(len(rest) == 2, USAGE)
        if preflight(prefix, inventory, hashes=rest[0] == "--probe") is None:
            return 4
        return probe(rest[1])
    require(len(rest) % 3 == 0, USAGE)
    verified = preflight(prefix, inventory)
    if verified is None:
        return 4
    print("installed package: %s" % os.path.dirname(os.path.realpath(fsim.__file__)))
    print("installed DLL: %s" % library)
    print("installed aircraft: %s" % ";".join(paths))
    print("installed runtime data: %d files of the expected inventory, byte for byte" % verified)
    a, b = ownership(True), ownership(False)
    require(a == b, "ownership depends on the order the agents were made in: %r, %r" % (a, b))
    require(a == (1, 1, 2, 2), "ownership is not the bindings': %r" % (a,))
    worlds()
    reports()
    key = mission_episode()
    mission_batch()
    chains = joint_chains([rest[i:i + 3] for i in range(0, len(rest), 3)])
    print("fsim %s installed Python consumer: ownership %r in either creation order; legal reports with provenance; "
          "mission episode %s with an isolated actor; a mission batch under the training adapter; %d joint chain(s) replayed, "
          "in-process and isolated alike" % (fsim.__version__, a, json.dumps(key), chains))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
