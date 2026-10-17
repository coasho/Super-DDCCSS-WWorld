"""Own-platform equipment APIs from an installed package, including wrapper, module, DLL and runtime-data provenance."""
import ctypes
import importlib
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
        return 0
    finally:
        world.close()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
