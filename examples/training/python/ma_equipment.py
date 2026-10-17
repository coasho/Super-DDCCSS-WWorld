"""Own-platform equipment access and reports through the public Python SDK.

    fsim python examples/training/python/ma_equipment.py

One configured service reads its platform's stores and dispenser inventory, powers
the native dispenser and releases one cartridge. It does not read weapon truth or
implement an automatic defensive policy.
"""
import fsim


def main():
    world = fsim.World("equipment-reports", publish=False, terrain=False, workers=1,
                       pin_workers=False, seed=17)
    aircraft = world.create_vehicle("blue", "jsbsim:f16c", latitude_deg=40.0, longitude_deg=0.0,
                                    altitude_msl_m=6000.0, airspeed_ms=230.0)
    aircraft.load_loadout("mk82 x4")
    agent = world.create_agent(aircraft, "equipment")
    world.bind_ma_service(agent, 1, ("*",), owns_dms_ingress=True, owns_dms_egress=True)

    stores = agent.report_stores()
    print("stores:", stores.availability, len(stores.items))
    for item in stores.items:
        print(" ", item.value, item.meta.source, item.meta.sample_time_s)

    inventory = agent.report_countermeasures()
    print("inventory:", inventory.availability, inventory.value)
    if inventory.availability != "available":
        return
    dispenser = next(payload for payload in agent.payloads if payload.family == "countermeasures")
    agent.set_payload_state(dispenser.index, "operate")
    world.step()
    before = agent.countermeasures()
    agent.dispense("chaff")
    after = agent.report_countermeasures()
    assert after.value.chaff == before.chaff - 1
    assert after.value.flare == before.flare
    print("after one native chaff release:", after.value, after.meta.source)
    print("strike reports:", len(agent.report_strikes().items))
    print("delivered consent reports:", len(agent.report_consents().items))


if __name__ == "__main__":
    main()
