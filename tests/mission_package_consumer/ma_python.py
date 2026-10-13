"""MA-service binding and legal reports from an installed Python package: `fsim` as installed, its own library and data,
nothing else (docs/sdk/mission-training.md, sections 9 and 10). Usage: ma_python.py <installed python prefix>."""
import ctypes
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


def main(argv):
    require(len(argv) == 2, "usage: ma_python.py <installed python prefix>")
    prefix = argv[1]
    require(inside(fsim.__file__, prefix), "fsim was imported from outside the installed package: " + fsim.__file__)
    require(inside(fsim._native.__file__, prefix), "the native module is not the installed package's")
    library = loaded_library()
    require(inside(library, prefix), "libfsim.dll was loaded from outside the installed package: " + library)
    print("installed package: %s" % os.path.dirname(os.path.realpath(fsim.__file__)))
    print("installed DLL: %s" % library)
    a, b = ownership(True), ownership(False)
    require(a == b, "ownership depends on the order the agents were made in: %r, %r" % (a, b))
    require(a == (1, 1, 2, 2), "ownership is not the bindings': %r" % (a,))
    worlds()
    reports()
    print("fsim %s installed Python consumer: ownership %r in either creation order; legal reports with provenance" % (fsim.__version__, a))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
