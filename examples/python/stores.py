"""Stores and weapon employment through the SDK (ADR-30 MS-7a, docs/sdk/mission-systems.md), for the viewer: an F-16C
over the Pacific takes four Mk 82s (its fit's "mk82 x4" loadout), arms its stations (the master arm: the station
payload operating), strikes two - the strike asks for release consent, given a few seconds later - and they leave a
half second apart, falling away as weapons; then it jettisons the other two in an emergency. Its stores' states are in
the viewer's mission systems window (key p). Two minutes.

    fsim python examples\\python\\stores.py
    fsim viewer --world stores --camera chase           (a second terminal)

--fast runs as fast as it goes.
"""
import math
import sys
import time

import fsim

real_time = "--fast" not in sys.argv

world = fsim.World("stores")
viper = world.create_vehicle("viper", "jsbsim:f16c", latitude_deg=37.2, longitude_deg=-123.4, altitude_msl_m=3000.0, heading_deg=270.0,
                             airspeed_ms=220.0)
viper.load_loadout("mk82 x4")
world.focus_viewer(viper)
print("loaded: " + ", ".join("station %d %s" % (s.station, s.mnemonic) for s in viper.stores))

strike = None
asked = False
jettisoned = False
t0 = time.perf_counter()
while world.time < 120.0:
    viper.command_velocity(airspeed_ms=220.0, vertical_speed_ms=0.0, heading_rad=math.radians(270.0))
    if world.time > 5.0 and viper.payload("stations").state == fsim.PayloadState.OFF:
        viper.set_payload_state("stations", "operate")
        print("t=%5.1f s  master arm on" % world.time)
    if strike is None and world.time > 10.0:
        strike = viper.strike("mk82", 2, ripple_s=0.5)
        print("t=%5.1f s  strike: two Mk 82s" % world.time)
    if strike is not None and not asked and viper.strike_status(strike).phase == "awaiting_consent":
        if world.time > 15.0:  # (a few seconds asking: the consent comes from whoever authorises the release)
            viper.strike_consent(strike)
            asked = True
            print("t=%5.1f s  release consent given" % world.time)
    if not jettisoned and world.time > 40.0:
        viper.jettison(emergency=True)
        jettisoned = True
        print("t=%5.1f s  emergency jettison" % world.time)
    world.step()
    if int(world.time * 2) % 20 == 0 and abs(world.time * 2 - round(world.time * 2)) < 1e-6:
        gone = [w for w in world.weapons if w.launcher == viper.id]
        print("t=%5.1f s  %s; weapons %d (%d ended)  mass %.0f kg" % (
            world.time, ", ".join("%d %s" % (s.station, s.status) for s in viper.stores), len(gone),
            sum(1 for w in gone if w.end != "in_flight"), viper.mass_kg))
    if real_time:
        ahead = world.time - (time.perf_counter() - t0)
        if ahead > 0:
            time.sleep(ahead)
