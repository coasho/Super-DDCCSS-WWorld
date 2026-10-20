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
  lights it or not (a jettisoned missile's stays unlit: InitialConditions::enginesRunning). Authored missile controls
  use finite JSBSim actuators and the same fin geometry (storecontrol.py); other stores remain uncontrolled. Its file says where
  its model's origin is (`fsim/model/origin-x-in`: its launch CG), since its empty CG - the platform's origin for
  every other model - is its structure's, burnt out, some decimetres ahead;
- six structure contacts round each end, so it lies on its side where it lands.

Estimates throughout, said so in each file written.
"""
import math
import os

from . import storeaero
from .motorinputs import validate_motor
from .storemodels import FT, IN, LB, MACH_RISE, RING, SLUGFT2, _value, model_name

G0 = 9.80665
LBF = 4.4482216152605
# s: its propellant's delivered specific impulse by its smoke (the design's `smoke`; assumed) - an aluminised
# composite's, and a reduced- or minimum-smoke one's without the aluminium
ISP = {"aluminised": 255.0}
ISP_PLAIN = 245.0
COAST_N = 2.0           # N: a pulse motor between its pulses (a phase of no thrust), so that its burn goes on to the next
BUILDUP = 0.05          # s: its thrust's rise
_IGNITION_TIME_THRUST = "fsim/ignition-time-thrust-v1"
GRAIN_DENSITY = 1750.0  # kg/m^3 (AP/HTPB)
GRAIN_RADIUS = 0.9      # the grain's radius, of the body's
GRAIN_PORT = 0.15       # its bore's share of its section
GRAIN_AFT = 0.03        # m: between the grain's aft end and the nozzle's cone
# its axial drag's rise over Mach: a carried store's (storemodels.MACH_RISE), on to Mach 4 (a missile's speeds: assumed)
RISE = MACH_RISE + [(3.0, 1.4), (4.0, 1.2)]
CLASSES = ("aam", "bomb")


def point_contact_nose(design):
    """An authored point tip in design structural metres, or unavailable.

    A finite-width/height tip or ambiguous/mirrored foremost body does not define
    this point sensor. Do not infer a tip from card length or launch CG.
    """
    def number(value):
        return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
    candidates = []
    for body in design.get("body", []):
        stations = body.get("stations", [])
        origin = body.get("origin", [0, 0, 0])
        if not stations or not isinstance(origin, (list, tuple)) or len(origin) != 3:
            return None
        first = stations[0]
        values = [first.get(key) for key in ("x", "w", "top", "bottom")] + [first.get("y", 0)] + list(origin)
        if not all(number(value) for value in values):
            return None
        tip = (first["x"] + origin[0], first.get("y", 0) + origin[1], first["top"] + origin[2])
        if not all(number(value) for value in tip):
            return None
        candidates.append((tip, first, body.get("mirror", False)))
    if not candidates:
        return None
    # A separately authored IR dome can precede the blunt main fuselage.
    foremost = min(row[0][0] for row in candidates)
    candidates = [row for row in candidates if row[0][0] == foremost]
    if len(candidates) != 1:
        return None
    tip, first, mirrored = candidates[0]
    return tip if not mirrored and first["w"] == 0 and first["top"] == first["bottom"] else None


def _ignition_time_thrust_rows(phases):
    """A validated pulsed motor's vacuum thrust (lbf) against native ignition time (s).

    A consumed-mass axis can skip a low-thrust coast in one integration step. The
    versioned native time table advances through it regardless of its tiny mass.
    Preserve the existing 2 N coast and its grain/impulse. The first native sine
    ramp delays every subsequent boundary by its lost effective burn time.
    """
    if phases[0]["duration_s"] < BUILDUP:
        raise ValueError("motor: ignition-time table requires the first phase to cover buildup")
    forces = [max(1000.0 * p["thrust_kn"], COAST_N) / LBF for p in phases]
    deficit = BUILDUP * (1.0 - 2.0 / math.pi)
    rows, elapsed = [(0.0, forces[0])], 0.0
    for index, phase in enumerate(phases):
        elapsed += phase["duration_s"]
        boundary = elapsed + deficit
        before = math.nextafter(boundary, -math.inf)
        # Adjacent double coordinates describe an abrupt transition without a
        # unit-dependent epsilon. Seventeen digits preserve both in the XML.
        if not math.isfinite(boundary) or before <= rows[-1][0]:
            raise ValueError("motor: ignition-time table knots collapse at double precision")
        rows.append((before, forces[index]))
        rows.append((boundary, forces[index + 1] if index + 1 < len(forces) else 0.0))
    return rows


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
    if "motor" not in design:
        return None
    motor = design["motor"]
    validate_motor(motor)
    from .motorinputs import _number

    def quantity(value, label, positive=False):
        value = _number(value, label)
        if positive and value <= 0.0:
            raise ValueError("motor: %s must be positive" % label)
        return value

    m = quantity(m, "launch mass", True)
    cg = quantity(cg, "launch CG")
    radius = quantity(radius, "body radius", True)
    phases = (motor or {}).get("phase", [])
    if not phases:
        return None
    isp = ISP.get(motor.get("smoke"), ISP_PLAIN)
    rows, burnt, impulse, burn_s = [], 0.0, 0.0, 0.0
    for i, p in enumerate(phases):
        newtons = quantity(max(1000.0 * p["thrust_kn"], COAST_N), "simulated phase thrust", True)
        phase_impulse = quantity(newtons * p["duration_s"], "simulated phase impulse", True)
        impulse = quantity(impulse + phase_impulse, "total simulated impulse", True)
        burn_s = quantity(burn_s + p["duration_s"], "total simulated burn duration", True)
        rows.append((quantity((burnt + (1e-6 if i else 0.0)) / LB, "thrust-table consumed mass"),
                     quantity(newtons / LBF, "thrust-table force", True)))
        burnt = quantity(burnt + phase_impulse / (isp * G0), "propellant mass", True)
        rows.append((quantity(burnt / LB, "thrust-table consumed mass", True), newtons / LBF))
    previous = emitted_previous = -math.inf
    for consumed, force in rows:
        emitted_consumed = float("%.7f" % consumed)
        emitted_force = float("%.3f" % force)
        if consumed <= previous or emitted_consumed <= emitted_previous:
            raise ValueError("motor: consumed-mass thrust table must increase strictly at the emitted precision")
        if not math.isfinite(emitted_consumed) or not math.isfinite(emitted_force) or emitted_force <= 0.0:
            raise ValueError("motor: thrust table must remain finite with positive force at the emitted precision")
        previous, emitted_previous = consumed, emitted_consumed
    if burnt >= m:
        raise ValueError("motor: propellant mass must be less than launch mass")
    empty = quantity(m - burnt, "remaining structure mass", True)
    R = quantity(GRAIN_RADIUS * radius, "grain radius", True)
    volume_per_length = quantity(GRAIN_DENSITY * math.pi * R * R * (1.0 - GRAIN_PORT), "grain mass per length", True)
    length = quantity(burnt / volume_per_length, "grain length", True)
    bore = quantity(2.0 * R * math.sqrt(GRAIN_PORT), "grain bore", True)
    quantity(length * length, "grain squared length", True)
    x = quantity(motor["exit"][0] - motor.get("cone_length", 0.0) - GRAIN_AFT - 0.5 * length, "grain centre")
    try:
        exit_area = quantity(0.25 * math.pi * motor["exit_diameter"] ** 2, "nozzle exit area", True)
    except OverflowError as error:
        raise ValueError("motor: nozzle exit area is not finite") from error
    empty_cg = quantity((quantity(m * cg, "launch mass moment") - quantity(burnt * x, "grain mass moment")) / empty,
                        "remaining structure CG")
    if design.get("aircraft", {}).get("category") == "missile":
        try:
            nose = min(quantity(body["stations"][0]["x"], "body nose") for body in design["body"])
            tail = max(quantity(body["stations"][-1]["x"], "body tail") for body in design["body"])
            body_length = quantity(tail - nose, "body length", True)
            gyration = design["mass"]["gyration"]
            if len(gyration) != 3:
                raise ValueError("motor: gyration must have three positive coordinates")
            for value in gyration:
                quantity(value, "gyration", True)
            if x - 0.5 * length < nose or x + 0.5 * length > tail:
                raise ValueError("motor: derived grain does not fit inside the body")
            ixx, iyy = _inertia(design, m, body_length)
            quantity(ixx, "launch roll inertia", True)
            quantity(iyy, "launch pitch inertia", True)
            rb = 0.5 * bore
            quantity(ixx - 0.5 * burnt * (R * R + rb * rb), "remaining structure roll inertia", True)
            quantity(iyy - burnt * (3.0 * (R * R + rb * rb) + length * length) / 12.0 - burnt * (x - cg) ** 2 -
                     empty * (empty_cg - cg) ** 2, "remaining structure pitch inertia", True)
        except (KeyError, IndexError, TypeError, OverflowError, ValueError) as error:
            if isinstance(error, ValueError) and str(error).startswith("motor:"):
                raise
            raise ValueError("motor: invalid body geometry or structure inertia") from error
    return dict(isp=isp, propellant_kg=burnt, burn_s=burn_s, rows=rows, radius=R, bore=bore, length=length, x=x,
                exit_x=motor["exit"][0], exit_area=exit_area)


def _table(mach, values, indent, precise=False):
    try:
        valid = (len(mach) == len(values) and len(mach) >= 2 and
                 all(not isinstance(v, bool) and math.isfinite(v) for v in list(mach) + list(values)) and
                 all(m >= 0 and (i == 0 or m > mach[i - 1]) for i, m in enumerate(mach)))
    except (TypeError, ValueError, OverflowError):
        valid = False
    if not valid:
        raise ValueError("coefficient table needs matching finite values and a strictly increasing Mach grid")
    pad = " " * indent
    if precise:
        return "\n".join("%s%.17g %.17g" % (pad, m, v) for m, v in zip(mach, values))
    return "\n".join("%s%.2f %.5f" % (pad, m, v) for m, v in zip(mach, values))


def _finite_conversion(value, label):
    if not math.isfinite(value):
        raise ValueError("%s must remain finite after unit conversion" % label)
    return value


def _slope(sign, angle, mach, values, cross, precise=False):
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
                            <value> %s </value>
                            <sin><property>aero/%s-rad</property></sin>
                            <abs><sin><property>aero/%s-rad</property></sin></abs>
                        </product>
                    </sum>""" % (sign, angle, angle, _table(mach, values, 32, precise),
                                  ("%.17g" if precise else "%.5f") % (sign * cross), angle, angle)


def _damping(name, description, ref, rate, mach, values, precise=False):
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
                              _table(mach, values, 28, precise))


def _turning(name, description, value, rate, precise=False):
    return """            <function name="aero/coefficient/%s">%s
                <product>
                    <value> %s </value>
                    <property>atmosphere/rho-slugs_ft3</property>
                    <property>velocities/%s-aero-rad_sec</property>
                    <abs><property>velocities/%s-aero-rad_sec</property></abs>
                </product>
            </function>""" % (name, "\n                <description>%s</description>" % description if description else "",
                              ("%.17g" if precise else "%.6f") % value, rate, rate)


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
    b = storeaero.coefficients(geo, card["id"])
    precise = "input" in b
    S, d = b["S"], b["d"]
    area_ft2, diameter_ft = S / FT ** 2, d / FT
    if precise:
        _finite_conversion(area_ft2, "reference area")
        _finite_conversion(diameter_ft, "reference diameter")
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
        if precise:
            return '        <location name="%s" unit="IN"> <x> %.17g </x> <y> 0 </y> <z> 0 </z> </location>' % (what, _finite_conversion(x * k, what))
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
    ca_mach, ca_values = [p[0] for p in RISE], [p[1] for p in RISE]
    if precise:
        ca, ca_mach, ca_values = 1.0, mach, b["ca"]
    turn = _finite_conversion(-b["turn"] / FT ** 5, "turning coefficient")
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
        <wingarea unit="FT2"> %s </wingarea>
        <wingspan unit="FT"> %s </wingspan>
        <chord unit="FT"> %s </chord>
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
""" % (card["id"], name, card.get("designation", card["id"]),
       ("%.17g" if precise else "%.5f") % area_ft2,
       ("%.17g" if precise else "%.5f") % diameter_ft, ("%.17g" if precise else "%.5f") % diameter_ft,
       loc("AERORP", cg), loc("EYEPOINT", cg), loc("VRP", cg),
       ixx / SLUGFT2, iyy / SLUGFT2, iyy / SLUGFT2, empty / LB, loc("CG", empty_cg),
       contacts("nose", x0 + 0.1 * geo["length"]), contacts("tail", x0 + 0.9 * geo["length"]),
       propulsion, controls,
       qS, ca, _table(ca_mach, ca_values, 28, precise),
       qS, _slope(1.0, "alpha", mach, b["cna"], b["cross_n"], precise),
       qS, _slope(-1.0, "beta", mach, b["cyb"], b["cross_y"], precise),
       qSc, _slope(1.0, "alpha", mach, b["cma"], b["cross_m"], precise),
       _damping("Cmq", "each lifting part's slope times its arm squared", "cbarw", "q", mach, b["cmq"], precise),
       _turning("Cmq2", "the body's crossflow as it turns", turn, "q", precise),
       qSc, _slope(-1.0, "beta", mach, b["cnb"], b["cross_ym"], precise),
       _damping("Cnr", "", "cbarw", "r", mach, b["cnr"], precise),
       _turning("Cnr2", "", turn, "r", precise),
       _damping("Clp", "", "bw", "p", mach, b["clp"], precise))
    if precise:
        from .storetables import payload_digest
        xml = xml.replace("Its aerodynamics a component build-up (storeaero.py) -\n     estimates, said so there.",
                          "Its aerodynamics the selected offline derivative table -\n     imported inputs, not an accuracy or calibration acceptance.")
        xml = xml.replace("along it: its card's drag area on its frontal area, its transonic rise", "selected offline axial coefficient")
        xml = xml.replace("its body's and fins' lift (slender body, Barrowman) and its crossflow (Jorgensen)", "selected offline derivatives and crossflow")
        xml = xml.replace("each lifting part's slope times its arm squared", "selected offline damping derivative")
        domain = """
        <function name="fsim/model/imported-aero/selected" type="pre"><value>1</value></function>
        <function name="fsim/model/imported-aero/mach-min" type="pre"><value>%.17g</value></function>
        <function name="fsim/model/imported-aero/mach-max" type="pre"><value>%.17g</value></function>
        <function name="fsim/model/imported-aero/mach-in-range" type="pre">
            <and>
                <ge><property>velocities/mach</property><property>fsim/model/imported-aero/mach-min</property></ge>
                <le><property>velocities/mach</property><property>fsim/model/imported-aero/mach-max</property></le>
            </and>
        </function>""" % (mach[0], mach[-1])
        xml = xml.replace("    <aerodynamics>", "    <!-- Offline derivative-table SHA-256: %s -->\n"
                          "    <!-- Offline derivative-payload SHA-256: %s -->\n    <aerodynamics>%s" %
                          (b["input"]["sha256"], payload_digest(dict(b, cg_m=cg)), domain))
    from . import storecontrol
    xml = storecontrol.apply(xml, controls, geo, b, cg * k)
    # apply replaces the FCS of a controlled missile. Publish geometry after
    # that transformation so native contact sensing retains the authored tip.
    tip = point_contact_nose(design)
    metadata = '\n        <property value="%d">fsim/model/nose-version</property>' % (1 if tip else 0)
    if tip is not None:
        for axis, coordinate in zip("xyz", tip):
            metadata += '\n        <property value="%.17g">fsim/model/nose-%s-in</property>' % (
                _finite_conversion(coordinate * k, "contact nose"), axis)
    xml = xml.replace("</flight_control>", metadata + "\n    </flight_control>", 1)
    out = {name + ".xml": xml}
    if motor:
        table_name = "propulsion/thrust_prop_remain"
        table_text = "\n".join("            %.7f %.3f" % row for row in motor["rows"])
        if any(phase["thrust_kn"] == 0.0 for phase in design["motor"]["phase"]):
            table_name = _IGNITION_TIME_THRUST
            table_text = "\n".join("            %.17g %.17g" % row
                                   for row in _ignition_time_thrust_rows(design["motor"]["phase"]))
        out["Engines/%s_motor.xml" % name] = """<?xml version="1.0"?>
<!-- Written by hangar (storemodels.py) from the design of %s: its motor's phases there (assumed: none is published),
     %.1f kg of propellant at %.0f s (assumed), burning %.1f s. Do not edit. -->
<rocket_engine name="%s motor">
    <isp> %.1f </isp>
    <builduptime> %.2f </builduptime>
    <thrust_table name="%s" type="internal">
        <tableData>
%s
        </tableData>
    </thrust_table>
</rocket_engine>
""" % (name, motor["propellant_kg"], motor["isp"], motor["burn_s"], name, motor["isp"], BUILDUP, table_name, table_text)
        out["Engines/%s_nozzle.xml" % name] = """<?xml version="1.0"?>
<!-- Written by hangar (storemodels.py): the nozzle's exit of %s's design. Do not edit. -->
<nozzle name="%s nozzle">
    <area unit="FT2"> %.6f </area>
</nozzle>
""" % (name, name, motor["exit_area"] / FT ** 2)
    return out
