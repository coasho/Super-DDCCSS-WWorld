"""A finned store's own flight model from its design (ADR-30, MS-7e1 and MS-7g): a missile's, a bomb's - the body that
flies on once it has left its station, aircraft/stores/<model>/<model>.xml (its card's model, "jsbsim:<model>"), written
by its design's build stage (storestages.py) as a tank's and a pod's are (storemodels.py):

- its mass its card's; its CG its design's (`[mass] empty_cg`); its inertia its design's radii of gyration - a
  missile's over its length (missiles.py: a slender rod's), a store's the mass model's (storedesigns.py);
- its aerodynamics the component build-up of its design (storeaero.py: slender body, Barrowman, Jorgensen) in body
  axes, tabled over Mach - its normal and side force and their moments about its launch CG (the AERORP: JSBSim takes
  the forces' moment about the CG from there, so the moment follows its CG as its motor burns), its damping, its fins'
  roll damping; along it its card's drag area (storedrag.py's build-up) with the transonic rise a carried store has;
- a missile's motor (`[motor]`): its design's phases - assumed, none is published - as a solid grain in the body, a
  cylinder with a bore, of the propellant the phases' impulse takes at its specific impulse; its nozzle's exit the design's. The world
  lights it or not (a jettisoned missile's stays unlit: InitialConditions::enginesRunning); nothing here guides it -
  it has no controls (`flight_control name="none"`): a body, not a weapon (docs/backlog.md, B-36). Its file says where
  its model's origin is (`fsim/model/origin-x-in`: its launch CG), since its empty CG - the platform's origin for
  every other model - is its structure's, burnt out, some decimetres ahead;
- six structure contacts round each end, so it lies on its side where it lands.

Estimates throughout, said so in each file written.
"""
import math
import os

from . import storeaero
from .storemodels import FT, IN, LB, MACH_RISE, RING, SLUGFT2, _value, model_name

G0 = 9.80665
LBF = 4.4482216152605
# s: its propellant's delivered specific impulse by its smoke (the design's `smoke`; assumed) - an aluminised
# composite's, and a reduced- or minimum-smoke one's without the aluminium
ISP = {"aluminised": 255.0}
ISP_PLAIN = 245.0
COAST_N = 2.0           # N: a pulse motor between its pulses (a phase of no thrust), so that its burn goes on to the next
BUILDUP = 0.05          # s: its thrust's rise
GRAIN_DENSITY = 1750.0  # kg/m^3 (AP/HTPB)
GRAIN_RADIUS = 0.9      # the grain's radius, of the body's
GRAIN_PORT = 0.15       # its bore's share of its section
GRAIN_AFT = 0.03        # m: between the grain's aft end and the nozzle's cone
# its axial drag's rise over Mach: a carried store's (storemodels.MACH_RISE), on to Mach 4 (a missile's speeds: assumed)
RISE = MACH_RISE + [(3.0, 1.4), (4.0, 1.2)]
CLASSES = ("aam", "bomb")


def _inertia(design, m, length):
    """(ixx, iyy) kg m^2 of the whole store about its CG: a missile's radii of gyration are over its length
    (missiles.py); a store's design is weighed by the mass model (storedesigns.py)."""
    if design["aircraft"].get("category") == "missile":
        g = design["mass"]["gyration"]
        return m * (g[0] * length) ** 2, m * (g[1] * length) ** 2
    from . import pipeline
    from .geometry.aircraft import Aircraft
    from .mass import MassModel
    name = design["aircraft"]["name"]
    e = MassModel(Aircraft.load(os.path.join(pipeline.repo_root(), "aircraft", name, name + ".toml"))).empty()
    return e["ixx"], e["iyy"]


def motor_of(design, m, cg, radius):
    """Its motor as its flight model has it, from its design's phases: {propellant_kg, burn_s, rows [(lb burnt, lbf)],
    grain: radius, bore, length, x (m), exit_x, exit_area}; None without one."""
    motor = design.get("motor")
    phases = (motor or {}).get("phase", [])
    if not phases:
        return None
    isp = ISP.get(motor.get("smoke"), ISP_PLAIN)
    rows, burnt = [], 0.0
    for i, p in enumerate(phases):
        newtons = max(1000.0 * p["thrust_kn"], COAST_N)
        rows.append(((burnt + (1e-6 if i else 0.0)) / LB, newtons / LBF))
        burnt += newtons * p["duration_s"] / (isp * G0)
        rows.append((burnt / LB, newtons / LBF))
    R = GRAIN_RADIUS * radius
    length = burnt / (GRAIN_DENSITY * math.pi * R * R * (1.0 - GRAIN_PORT))
    bore = 2.0 * R * math.sqrt(GRAIN_PORT)
    x = motor["exit"][0] - motor.get("cone_length", 0.0) - GRAIN_AFT - 0.5 * length
    return dict(isp=isp, propellant_kg=burnt, burn_s=sum(p["duration_s"] for p in phases), rows=rows, radius=R, bore=bore, length=length, x=x,
                exit_x=motor["exit"][0], exit_area=0.25 * math.pi * motor["exit_diameter"] ** 2)


def _table(mach, values, indent):
    pad = " " * indent
    return "\n".join("%s%.2f %.5f" % (pad, m, v) for m, v in zip(mach, values))


def _slope(sign, angle, mach, values, cross):
    """A force's or a moment's sum: its slope over Mach times sin cos of the angle, and its crossflow's times sin |sin|."""
    return """                    <sum>
                        <product>
                            <value> %.1f </value>
                            <sin><property>aero/%s-rad</property></sin>
                            <cos><property>aero/%s-rad</property></cos>
                            <table>
                            <independentVar lookup="row">velocities/mach</independentVar>
                            <tableData>
%s
                            </tableData>
                        </table>
                        </product>
                        <product>
                            <value> %.5f </value>
                            <sin><property>aero/%s-rad</property></sin>
                            <abs><sin><property>aero/%s-rad</property></sin></abs>
                        </product>
                    </sum>""" % (sign, angle, angle, _table(mach, values, 32), sign * cross, angle, angle)


def _damping(name, description, ref, rate, mach, values):
    return """            <function name="aero/coefficient/%s">%s
                <product>
                    <property>aero/qbar-psf</property>
                    <property>metrics/Sw-sqft</property>
                    <property>metrics/%s-ft</property>
                    <property>aero/%si2vel</property>
                    <property>velocities/%s-aero-rad_sec</property>
                    <table>
                        <independentVar lookup="row">velocities/mach</independentVar>
                        <tableData>
%s
                        </tableData>
                    </table>
                </product>
            </function>""" % (name, "\n                <description>%s</description>" % description if description else "", ref, ref[0], rate,
                              _table(mach, values, 28))


def _turning(name, description, value, rate):
    return """            <function name="aero/coefficient/%s">%s
                <product>
                    <value> %.6f </value>
                    <property>atmosphere/rho-slugs_ft3</property>
                    <property>velocities/%s-aero-rad_sec</property>
                    <abs><property>velocities/%s-aero-rad_sec</property></abs>
                </product>
            </function>""" % (name, "\n                <description>%s</description>" % description if description else "", value, rate, rate)


def files(card):
    """The store's flight model as its files, {path under aircraft/stores/<model>/: text} - its definition and, with a
    motor, its engine's and its nozzle's (None: not a missile or a bomb, or it has no design)."""
    name = model_name(card)
    if card.get("class") not in CLASSES or not name:
        return None
    geo = storeaero.geometry(card)
    if geo is None or geo.get("design") is None:
        return None
    design = geo["design"]
    b = storeaero.build_up(geo)
    S, d = b["S"], b["d"]
    r = 0.5 * d
    m = _value(card, "mass_kg")
    cg = float(design["mass"]["empty_cg"][0])            # (the design's frame: x aft from its nose's tip)
    x0 = cg - geo["cg"]                                  # where its body starts, behind a seeker's dome
    nose = min(body["stations"][0]["x"] for body in design["body"])
    tail = max(body["stations"][-1]["x"] for body in design["body"])
    ixx, iyy = _inertia(design, m, tail - nose)
    motor = motor_of(design, m, cg, r)
    empty, empty_cg = m, cg
    if motor:
        # its structure, burnt out: what JSBSim adds its grain to - the whole its design's, its motor full
        mp, R, rb, lg, xg = motor["propellant_kg"], motor["radius"], 0.5 * motor["bore"], motor["length"], motor["x"]
        empty = m - mp
        empty_cg = (m * cg - mp * xg) / empty
        ixx -= 0.5 * mp * (R * R + rb * rb)
        iyy -= mp * (3.0 * (R * R + rb * rb) + lg * lg) / 12.0 + mp * (xg - cg) ** 2 + empty * (empty_cg - cg) ** 2
    k = 1.0 / IN

    def loc(what, x):
        return '        <location name="%s" unit="IN"> <x> %.3f </x> <y> 0 </y> <z> 0 </z> </location>' % (what, x * k)

    spring, damping = 20.0 * m / LB, 2.0 * m / LB  # (its weight on 1.5 cm; about critically damped)

    def contacts(what, x):
        out = []
        for i in range(RING):
            a = 2.0 * math.pi * (i + 0.5) / RING
            out.append("""        <contact type="STRUCTURE" name="%s %d">
            <location unit="IN"> <x> %.3f </x> <y> %.3f </y> <z> %.3f </z> </location>
            <static_friction> 0.8 </static_friction>
            <dynamic_friction> 0.5 </dynamic_friction>
            <spring_coeff unit="LBS/FT"> %.0f </spring_coeff>
            <damping_coeff unit="LBS/FT/SEC"> %.0f </damping_coeff>
        </contact>""" % (what, i + 1, x * k, r * math.cos(a) * k, r * math.sin(a) * k, spring / RING, damping / RING))
        return "\n".join(out)

    if motor:
        propulsion = """    <propulsion>
        <engine file="%s_motor">
            <feed> 0 </feed>
            <thruster file="%s_nozzle">
                <location unit="IN"> <x> %.3f </x> <y> 0 </y> <z> 0 </z> </location>
                <orient unit="DEG"> <roll> 0 </roll> <pitch> 0 </pitch> <yaw> 0 </yaw> </orient>
            </thruster>
        </engine>
        <tank type="FUEL">
            <location unit="IN"> <x> %.3f </x> <y> 0 </y> <z> 0 </z> </location>
            <radius unit="IN"> %.4f </radius>
            <grain_config type="CYLINDRICAL">
                <length unit="IN"> %.4f </length>
                <bore_diameter unit="IN"> %.4f </bore_diameter>
            </grain_config>
            <capacity unit="LBS"> %.5f </capacity>
            <contents unit="LBS"> %.5f </contents>
        </tank>
    </propulsion>""" % (name, name, motor["exit_x"] * k, motor["x"] * k, motor["radius"] * k, motor["length"] * k, motor["bore"] * k,
                        motor["propellant_kg"] / LB, motor["propellant_kg"] / LB)
        # its model's origin for the platform (sim::JsbsimModel: fsim/model/origin-*-in): its CG as it is launched -
        # what its picture is drawn about and what its station holds - where the empty CG above is its structure's
        controls = """    <flight_control name="none">
        <!-- no controls. Its model's origin in this frame (in): its CG as it is launched, its motor full - its picture's
             origin and what a station holds; the CG above is its structure's, burnt out -->
        <property value="%.3f">fsim/model/origin-x-in</property>
        <property value="0">fsim/model/origin-y-in</property>
        <property value="0">fsim/model/origin-z-in</property>
    </flight_control>""" % (cg * k)
    else:
        propulsion = "    <propulsion/>"
        controls = '    <flight_control name="none"/>'
    mach = b["mach"]
    ca = (_value(card, "drag_area_m2") or 0.15 * S) / S
    turn = -b["turn"] / FT ** 5
    qS = "                    <property>aero/qbar-psf</property>\n                    <property>metrics/Sw-sqft</property>"
    qSc = qS + "\n                    <property>metrics/cbarw-ft</property>"
    xml = """<?xml version="1.0"?>
<!-- Written by hangar (tools/hangar/hangar/storemodels.py) from the store card %s and its design: the store flying on its
     own once it is released or jettisoned (MS-7e1, MS-7g). Its aerodynamics a component build-up (storeaero.py) -
     estimates, said so there. Do not edit. -->
<fdm_config name="%s" version="2.0" release="ALPHA">
    <fileheader>
        <author> hangar </author>
        <description> %s, released </description>
    </fileheader>
    <metrics>
        <wingarea unit="FT2"> %.5f </wingarea>
        <wingspan unit="FT"> %.5f </wingspan>
        <chord unit="FT"> %.5f </chord>
        <htailarea unit="FT2"> 0 </htailarea>
        <htailarm unit="FT"> 0 </htailarm>
        <vtailarea unit="FT2"> 0 </vtailarea>
        <vtailarm unit="FT"> 0 </vtailarm>
%s
%s
%s
    </metrics>
    <mass_balance negated_crossproduct_inertia="false">
        <ixx unit="SLUG*FT2"> %.5f </ixx>
        <iyy unit="SLUG*FT2"> %.5f </iyy>
        <izz unit="SLUG*FT2"> %.5f </izz>
        <emptywt unit="LBS"> %.4f </emptywt>
%s
    </mass_balance>
    <ground_reactions>
%s
%s
    </ground_reactions>
%s
%s
    <aerodynamics>
        <axis name="AXIAL">
            <function name="aero/coefficient/CA">
                <description>along it: its card's drag area on its frontal area, its transonic rise</description>
                <product>
%s
                    <value> %.5f </value>
                    <table>
                        <independentVar lookup="row">velocities/mach</independentVar>
                        <tableData>
%s
                        </tableData>
                    </table>
                </product>
            </function>
        </axis>
        <axis name="NORMAL">
            <function name="aero/coefficient/CN">
                <description>its body's and fins' lift (slender body, Barrowman) and its crossflow (Jorgensen)</description>
                <product>
%s
%s
                </product>
            </function>
        </axis>
        <axis name="SIDE">
            <function name="aero/coefficient/CY">
                <description>the same sideways</description>
                <product>
%s
%s
                </product>
            </function>
        </axis>
        <axis name="PITCH">
            <function name="aero/coefficient/Cm">
                <description>their moment about the launch CG (the AERORP)</description>
                <product>
%s
%s
                </product>
            </function>
%s
%s
        </axis>
        <axis name="YAW">
            <function name="aero/coefficient/Cn">
                <product>
%s
%s
                </product>
            </function>
%s
%s
        </axis>
        <axis name="ROLL">
%s
        </axis>
    </aerodynamics>
</fdm_config>
""" % (card["id"], name, card.get("designation", card["id"]), S / FT ** 2, d / FT, d / FT,
       loc("AERORP", cg), loc("EYEPOINT", cg), loc("VRP", cg),
       ixx / SLUGFT2, iyy / SLUGFT2, iyy / SLUGFT2, empty / LB, loc("CG", empty_cg),
       contacts("nose", x0 + 0.1 * geo["length"]), contacts("tail", x0 + 0.9 * geo["length"]),
       propulsion, controls,
       qS, ca, _table([p[0] for p in RISE], [p[1] for p in RISE], 28),
       qS, _slope(1.0, "alpha", mach, b["cna"], b["cross_n"]),
       qS, _slope(-1.0, "beta", mach, b["cyb"], b["cross_y"]),
       qSc, _slope(1.0, "alpha", mach, b["cma"], b["cross_m"]),
       _damping("Cmq", "each lifting part's slope times its arm squared", "cbarw", "q", mach, b["cmq"]),
       _turning("Cmq2", "the body's crossflow as it turns", turn, "q"),
       qSc, _slope(-1.0, "beta", mach, b["cnb"], b["cross_ym"]),
       _damping("Cnr", "", "cbarw", "r", mach, b["cnr"]),
       _turning("Cnr2", "", turn, "r"),
       _damping("Clp", "", "bw", "p", mach, b["clp"]))
    out = {name + ".xml": xml}
    if motor:
        out["Engines/%s_motor.xml" % name] = """<?xml version="1.0"?>
<!-- Written by hangar (storemodels.py) from the design of %s: its motor's phases there (assumed: none is published),
     %.1f kg of propellant at %.0f s (assumed), burning %.1f s. Do not edit. -->
<rocket_engine name="%s motor">
    <isp> %.1f </isp>
    <builduptime> %.2f </builduptime>
    <thrust_table name="propulsion/thrust_prop_remain" type="internal">
        <tableData>
%s
        </tableData>
    </thrust_table>
</rocket_engine>
""" % (name, motor["propellant_kg"], motor["isp"], motor["burn_s"], name, motor["isp"], BUILDUP, "\n".join("            %.7f %.3f" % row for row in motor["rows"]))
        out["Engines/%s_nozzle.xml" % name] = """<?xml version="1.0"?>
<!-- Written by hangar (storemodels.py): the nozzle's exit of %s's design. Do not edit. -->
<nozzle name="%s nozzle">
    <area unit="FT2"> %.6f </area>
</nozzle>
""" % (name, name, motor["exit_area"] / FT ** 2)
    return out
