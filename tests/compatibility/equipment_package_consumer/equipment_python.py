"""Own-platform equipment APIs from an installed package, including wrapper, module, DLL and runtime-data provenance."""
import ctypes
import importlib
import math
import os
from pathlib import Path
import sys

import fsim


def require(condition, message):
    if not condition:
        raise RuntimeError("installed Python equipment consumer: " + message)


def inside(path, prefix):
    return Path(path).resolve().is_relative_to(Path(prefix).resolve())


def loaded_library(name):
    kernel = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel.GetModuleHandleW.restype = ctypes.c_void_p
    kernel.GetModuleHandleW.argtypes = [ctypes.c_wchar_p]
    kernel.GetModuleFileNameW.argtypes = [ctypes.c_void_p, ctypes.c_wchar_p, ctypes.c_uint32]
    module = kernel.GetModuleHandleW(name)
    require(module, name + " was not loaded")
    buffer = ctypes.create_unicode_buffer(32768)
    size = kernel.GetModuleFileNameW(module, buffer, len(buffer))
    require(0 < size < len(buffer), "cannot inspect loaded " + name)
    return Path(buffer.value).resolve()


def make_world(data):
    return fsim.World("installed-equipment-python", publish=False, workers=1, pin_workers=False, terrain=False, seed=19,
                      jsbsim_root=str(data / "jsbsim"))


def aircraft(world):
    return world.create_vehicle("viper", "jsbsim:f16c", latitude_deg=40.0, longitude_deg=0.0,
                                altitude_msl_m=6000.0, heading_deg=90.0, airspeed_ms=230.0)


def own_meta(meta, now):
    require((meta.source, meta.validity) == ("own_platform", "valid"), "report provenance is not own-platform")
    require((meta.sample_time_s, meta.receive_time_s, meta.age_s) == (now, now, 0.0), "report times differ from its native read")


def smoke(world):
    viper = aircraft(world)
    viper.load_loadout("mk82 x4")
    agent = world.create_agent(viper, "equipment")
    dispensers = [payload for payload in agent.payloads if payload.family == "countermeasures"]
    require(len(dispensers) == 1, "installed aircraft has no unique countermeasure payload")
    dispenser = dispensers[0].index
    agent.set_payload_state(dispenser, "operate")
    operating = False
    for _ in range(1200):
        world.step()
        operating = any(payload.index == dispenser and payload.state == fsim.PayloadState.OPERATE for payload in agent.payloads)
        if operating:
            break
    require(operating, "native countermeasure warmup did not complete")
    require(agent.countermeasures() == fsim.CountermeasureInventory(30, 15), "initial native inventory differs")
    before = agent.report_countermeasures()
    require(isinstance(before, fsim.Report) and before.availability == "available", "countermeasure report shape differs")
    require(before.value == agent.countermeasures(), "reported inventory differs from native inventory")
    own_meta(before.meta, world.time)
    agent.dispense("chaff")
    require(agent.countermeasures() == fsim.CountermeasureInventory(29, 15), "release did not consume exactly one chaff")
    try:
        agent.dispense("flare")
    except fsim.Error:
        pass
    else:
        require(False, "shared native cooldown allowed an immediate flare")
    require(agent.countermeasures() == fsim.CountermeasureInventory(29, 15), "refused release consumed inventory")
    require(agent.report_countermeasures(stale_after_s=-1.0).meta.validity == "stale", "strict age threshold was lost")
    stores = agent.report_stores()
    require(isinstance(stores, fsim.ReportCollection) and stores.availability == "available" and len(stores.items) == 4,
            "installed loadout stores were not reported")
    for item in stores.items:
        own_meta(item.meta, world.time)
        require(item.value.store == "mk82" and item.value.weapon == 0, "inventory contains another store or premature release")
    require(agent.report_stores() == stores, "repeated read consumed or changed stores")
    strikes = agent.report_strikes()
    require(strikes.availability == "available" and strikes.items == [], "idle platform reports a strike")
    consents = agent.report_consents()
    require(consents.items == [], "undelivered consent request was reported")
    try:
        agent.launch_zone("mk82", 0)
    except fsim.Error:
        pass
    else:
        require(False, "zero designation produced an available launch zone")
    print("installed Python equipment: chaff=29 flare=15 stores=4 strikes=0 consents=0")


def release(world, tick):
    carrier = world.create_vehicle("launcher", "jsbsim:f16c", latitude_deg=40.0, longitude_deg=0.0,
                                   altitude_msl_m=6000.0, heading_deg=90.0, airspeed_ms=250.0)
    carrier.load_loadout("air superiority")
    launcher = world.create_agent(carrier, "launcher")
    launcher.submit(fsim.Level.VELOCITY, airspeed_ms=250.0, vertical_speed_ms=0.0, heading_rad=math.pi / 2)
    c2 = world.create_c2_node("approver", 39.9, 0.0, altitude_m=200.0)
    net = world.create_net("consent")
    require(world.join_net(carrier, net) and world.join_net(c2, net), "cannot join consent net")
    approver = world.create_agent(c2, "approver")
    require(approver.declare_role(fsim.ROLE_QB, "APPROVER"), "C2 role declaration refused")
    stations = next(p.index for p in launcher.payloads if p.family == "store_station")
    radar = next(p.index for p in launcher.payloads if p.family == "air_radar")
    for payload in (stations, radar):
        launcher.set_payload_state(payload, "operate")
    for _ in range(1200):
        tick()
        if sum(p.state == fsim.PayloadState.OPERATE for p in launcher.payloads if p.index in (stations, radar)) == 2:
            break
    else:
        raise RuntimeError("native radar/station startup did not finish")
    own = carrier.state
    target = world.create_vehicle("opponent", "jsbsim:f16c", latitude_deg=math.degrees(own.latitude_rad) + .01,
                                  longitude_deg=math.degrees(own.longitude_rad) + .15,
                                  altitude_msl_m=6000.0, heading_deg=270.0, airspeed_ms=250.0)
    target.submit(fsim.Level.VELOCITY, airspeed_ms=250.0, vertical_speed_ms=0.0, heading_rad=3 * math.pi / 2)
    launcher.amti_search(radar, bars=1, azimuth_half_width_rad=math.radians(15), range_scale_m=60000)
    track = None
    for _ in range(600):
        tick()
        track = next((t for t in launcher.amti_tracks(radar) if t.updates >= 3), None)
        if track is not None:
            break
    require(track is not None, "radar did not confirm a measured track")
    designation = launcher.designate(radar, track.id)
    require(designation, "own radar designation refused")
    launcher.amti_track(track.id, radar, range_scale_m=60000)
    activity = launcher.strike("aim-120c", target=designation, payload=stations)
    require(not world.weapons, "strike bypassed consent")
    answered = False
    for _ in range(1200):
        tick()
        for report in approver.report_consents().items:
            request = report.value
            if request.strike == activity:
                require(report.meta.source == "dms" and not world.weapons, "consent bypassed native delivery")
                require(approver.answer_consent(request.source, request.request_id), "consent answer refused")
                answered = True
                break
        if answered:
            break
    require(answered, "C2 received no release-consent request")
    for _ in range(600):
        tick()
        if world.weapons:
            break
    require(len(world.weapons) == 1, "delivered consent did not release one armed missile")
    return world.weapons[0].id


def guided_controls(data):
    with make_world(data) as world:
        missile = release(world, world.step)
        moved = False
        for _ in range(20):
            world.step()
            sample = world.weapon_controls(missile)
            require(sample is not None and sample.available and sample.controller_available and len(sample.fins) == 4,
                    "installed DLL did not expose actual native fins/controller")
            require(sample.sample_time_s == world.time and sample.controller_time_s <= sample.sample_time_s,
                    "installed control timing differs from native time")
            require(len({f.id for f in sample.fins}) == 4, "physical fin IDs are duplicated")
            for fin in sample.fins:
                require(fin.valid and fin.minimum_rad <= fin.actual_rad <= fin.maximum_rad, "invalid actual native fin")
                moved |= abs(fin.actual_rad) > 1e-8
        require(moved, "armed guidance never moved an actual fin")
        fin = sample.fins[0]
        world.vehicle(missile).set_property("fcs/fin-%d-actuator/malfunction/fail_stuck" % fin.native_index, 1)
        world.step(5)
        failed = next(f for f in world.weapon_controls(missile).fins if f.id == fin.id)
        require(failed.failed_stuck and failed.actual_rad == fin.actual_rad, "installed query hid a native jam")
        print("installed Python guided controls: armed=1 delivered_consent=1 fins=4 jam=1")

def main(argv):
    require(len(argv) == 3, "usage: equipment_python.py <installed python prefix> <smoke|data_probe>")
    prefix = Path(argv[1]).resolve()
    package = prefix / "fsim"
    for name in ("fsim", "fsim.agent", "fsim.world", "fsim._native"):
        module = importlib.import_module(name)
        require(inside(module.__file__, package), name + " came from outside the installed package")
        print("installed module: %s" % Path(module.__file__).resolve())
    for name in ("libfsim.dll", "libJSBSim.dll"):
        actual = loaded_library(name)
        require(actual.samefile(package / "bin" / name), name + " came from outside the installed package")
        print("installed DLL: %s" % actual)
    data = package / "share" / "flightsim"
    require((data / "aircraft" / "f16c" / "f16c.xml").is_file(), "installed aircraft data is missing")
    print("installed runtime data: %s" % data)
    world = make_world(data)
    try:
        if argv[2] == "data_probe":
            try:
                aircraft(world)
            except fsim.Error as error:
                print("installed aircraft data rejected as expected: %s" % error)
                return 42
            require(False, "corrupt installed aircraft data was bypassed")
        require(argv[2] == "smoke", "unknown installed equipment consumer mode")
        smoke(world)
        guided_controls(data)
        return 0
    finally:
        world.close()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
