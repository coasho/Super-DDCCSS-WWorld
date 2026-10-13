"""MA services on one platform (docs/sdk/agents.md, "MA services"; docs/sdk/mission-training.md, section 9).

    fsim python examples/python/ma_services.py

One F-16C carries two MA services: one flies, one works the radar and owns the inbox. The range binds each to what it
owns - by service id, to the controller the world minted for its agent - and from then on ownership is the bindings':
the same whichever agent was made first. The script builds the platform twice, the agents made in either order, and
prints what each service owns and what the library refuses the other.

It uses the public Python SDK alone, and publishes nothing: there is no flight to watch.
"""
import fsim

HSA, ROUTE = "fsim.guidance.hsa", "fsim.guidance.route"


def refused(call, *args, **kwargs):
    """What a refused call says, or None for one that went through."""
    try:
        call(*args, **kwargs)
    except fsim.Error as e:
        return str(e)
    return None


def run(flight_first):
    w = fsim.World("ma-services-demo", publish=False, workers=1, pin_workers=False, terrain=False)
    viper = w.create_vehicle("viper", "jsbsim:f16c", latitude_deg=40.0, longitude_deg=0.0, altitude_msl_m=6000.0, airspeed_ms=230.0)
    crc = w.create_c2_node("crc", 39.6, 0.0, 200.0)
    net = w.create_net("l16")
    w.join_net(viper, net)
    w.join_net(crc, net)
    if flight_first:
        flight, sensors = w.create_agent(viper, "flight"), w.create_agent(viper, "sensors")
    else:
        sensors, flight = w.create_agent(viper, "sensors"), w.create_agent(viper, "flight")
    boss = w.create_agent(crc, "crc")
    radar = next(p.index for p in viper.payloads if p.family == "air_radar")
    search = next(c.id for c in viper.mission_capabilities if c.payload == radar and c.type == "fsim.ms.amti")

    # The range's alone: service 1 flies and sends; service 2 searches and receives.
    w.bind_ma_service(flight, 1, (HSA, ROUTE), owns_dms_ingress=False, owns_dms_egress=True)
    w.bind_ma_service(sensors, 2, (search,), owns_dms_ingress=True, owns_dms_egress=False)
    service = {s.controller: s.service_id for s in w.ma_services()}

    # C2 designates every flight mode: the grants go to the capabilities' owners, not to the agent made last.
    boss.declare_role(fsim.ROLE_AVC, "AVC-1")
    w.step(10)
    boss.designate_control(flight.system, 0xFFFFFFFF)
    w.step(60)
    holder = viper.control_status(HSA).holder

    flight.submit_hsa(heading_rad=1.0, speed=230.0, altitude_m=6000.0)
    flight.send(fsim.TOPIC_FREE_FORM + 1, b"on station", [boss.system])
    boss.send(fsim.TOPIC_FREE_FORM + 2, b"copy", [flight.system])
    w.step(90)
    result = {
        "made first": "flight" if flight_first else "sensors",
        "owner of hsa": service.get(w.ma_capability_owner(viper, HSA)),
        "holds hsa after C2's designation": service.get(holder),
        "owner of the radar's search": service.get(w.ma_capability_owner(viper, search)),
        "sensors flies": refused(sensors.submit_hsa, heading_rad=2.0, speed=230.0, altitude_m=6000.0),
        "sensors asks for hsa": refused(sensors.request_control, HSA),
        "sensors sends": refused(sensors.send, fsim.TOPIC_FREE_FORM + 1, b"x", [boss.system]),
        "flight's inbox": len(flight.receive()),
        "sensors' inbox": len(sensors.receive()),
        "both read the same picture": len(flight.peers()) == len(sensors.peers()),
    }
    w.close()
    return result


def main():
    a, b = run(True), run(False)
    for key in a:
        print("%-36s %-44s %s" % (key, a[key], b[key]))
    same = {k: v for k, v in a.items() if k != "made first"} == {k: v for k, v in b.items() if k != "made first"}
    print("\nthe same ownership whichever agent was made first:", same)
    return 0 if same else 1


if __name__ == "__main__":
    raise SystemExit(main())
