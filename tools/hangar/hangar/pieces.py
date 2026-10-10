"""The pieces an airframe breaks into, each a flight model of its own (ADR-30, MS-7g's damage part, G6): when a
breakup section's root fails in the world, the section flies on as a JSBSim vehicle - the same integrator, air, wind
and ground as every other - from this file, one per section that can part (the nose, each wing, the tail):

aircraft/<type>/pieces/<type>-<section>/<type>-<section>.xml, in its aircraft's structural frame (the section's centre
where it was on it), from the aircraft's damage data (damage.py):
- its mass and inertia the section's structure's (its products of inertia JSBSim's plain sums in the structural
  frame, negated_crossproduct_inertia="false", as the aircraft's); point masses at no weight that the world sets as
  it parts - the fuel left in the section's tanks (at their middle), then one a store station on it (at its place,
  "station <n>"), its stores;
- its aerodynamics a tumbling piece's, in body axes (estimates - no public source gives a piece's: Hoerner's flat
  plate and bluff-body figures):
  - a wing (or a tail) a flat plate: its normal force its lift slope's (2 pi A / (A + 2), its own aspect ratio) at
    small angles, stalled from 20 degrees, then 1.2 sin(alpha) (Hoerner's flat plate normal to the flow); its centre
    of pressure from a quarter of its chord at small angles to its half broadside, behind its centre of mass at 0.4 of
    its chord at small angles and ahead of it never - so it pitches as a falling plate does, fluttering and tumbling;
    edge on, a plate's friction and its thickness's pressure drag;
  - the nose a bluff body: crossflow drag 1.2 sin(alpha) on its side, 0.5 along it (its open break);
  - every piece's rotation damped (C_lp, C_mq, C_nr -0.5, -2, -0.5 on its own span and chord);
- two structure contacts at its ends, so it lies where it lands.
Not the model's shape; written with the damage data, outside the model's key."""
import math
import os

import numpy as np

LB, FT, IN = 0.45359237, 0.3048, 0.0254
SLUGFT2 = 1.35581795  # kg m^2 per slug ft^2
PARTS = ("nose", "left wing", "right wing", "tail")
STALL_DEG = 20.0
PLATE_CN = 1.2         # Hoerner: a flat plate broadside to the flow
BLUFF_AXIAL = 0.5
CG_CHORD = 0.4         # of a wing piece's chord, from its leading edge: its spars and tank (an estimate)
DAMPING = {"Clp": -0.5, "Cmq": -2.0, "Cnr": -0.5}


def slug(section):
    return section.replace(" ", "-")


def piece_name(aircraft_name, section):
    return "%s-%s" % (aircraft_name, slug(section))


def _structural(p_body, origin):
    """Body axes from the model's origin (x fwd, y right, z down) to the structural frame (x aft, y right, z up; m)."""
    p = np.asarray(p_body, float)
    return np.array([origin[0] - p[0], origin[1] + p[1], origin[2] - p[2]])


def _extent(data, s):
    """The section's panels' corners and centres (body axes): its reach for its size and its ends."""
    pts = []
    for p in data["panels"]:
        if p["section"] != s:
            continue
        if p["kind"] == "surface":
            pts.extend(p["corners"])
        else:
            c, h = np.asarray(p["centre"]), np.asarray(p["half"])
            pts.extend([c - h, c + h])
    return np.asarray(pts, float)


def _normal_table(plate, aspect):
    """C_N against alpha (deg, -180 .. 180)."""
    alphas = [0, 5, 10, 15, 20, 30, 45, 60, 75, 90]
    out = []
    slope = 2.0 * math.pi * aspect / (aspect + 2.0)  # per radian
    for a in alphas:
        s = math.sin(math.radians(a))
        if plate:
            cn = slope * math.radians(a) if a <= STALL_DEG else PLATE_CN * s
            if a == STALL_DEG:
                cn = min(cn, max(PLATE_CN * s, 0.8 * slope * math.radians(STALL_DEG)))
        else:
            cn = PLATE_CN * s * s  # (a bluff body's crossflow, as sin^2)
        out.append((a, cn))
    full = {}
    for a, cn in out:
        for x, v in ((a, cn), (-a, -cn), (180 - a, -cn), (a - 180, cn)):
            full[round(x, 6)] = v
    return sorted(full.items())


def _cp_table(plate):
    """The centre of pressure behind the centre of mass, in chords, against alpha (deg)."""
    out = {}
    for a in (0, 10, 20, 30, 45, 60, 75, 90):
        s = abs(math.sin(math.radians(a)))
        x = (0.25 + 0.25 * s - CG_CHORD) if plate else 0.1 * s
        for v in (a, -a, 180 - a, a - 180):
            out[round(v, 6)] = x
    return sorted(out.items())


def _table(var, rows, indent):
    pad = " " * indent
    body = "\n".join("%s    %10.4f %10.5f" % (pad, a, v) for a, v in rows)
    return ("%s<table>\n%s  <independentVar>%s</independentVar>\n%s  <tableData>\n%s\n%s  </tableData>\n%s</table>"
            % (pad, pad, var, pad, body, pad, pad))


def piece_xml(data, section):
    """The JSBSim definition of one section flying on its own (None: it does not part)."""
    s = next(x for x in data["sections"] if x["name"] == section)
    if s["root"] is None or not (s["mass_kg"] > 0.0):
        return None
    origin = np.asarray(data["origin_design_m"], float)
    pts = _extent(data, section)
    if len(pts) < 2:
        return None
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    size = hi - lo
    plate = section in ("left wing", "right wing", "tail")
    if plate:
        area = sum(0.5 * p["area_m2"] for p in data["panels"] if p["section"] == section and p["kind"] == "surface")
        span = max(size[1], 0.3) if section != "tail" else max(size[1], size[2], 0.3)
        area = max(area, 0.05)
        chord = max(area / span, 0.2)
    else:
        area = max(0.25 * s["skin_area_m2"], 0.05)  # (Cauchy's mean presented area: a quarter of its surface)
        span = max(size[1], 0.3)
        chord = max(size[0], 0.3)
    aspect = span * span / area
    cg = _structural(s["cg"], origin)
    ixx, iyy, izz, pxy, pxz, pyz = (max(v, 0.0) / SLUGFT2 if k < 3 else v / SLUGFT2 for k, v in enumerate(s["inertia"]))
    # (the damage data's products are body axes' plain sums - x forward, z down; the structural frame's, x aft and z
    # up, flip x and z: sum(x y) and sum(y z) change sign, sum(x z) does not)
    ixy, ixz, iyz = -pxy, pxz, -pyz
    tanks = s["tanks"]
    fuel_at = _structural(np.mean([t["centre"] for t in tanks], axis=0), origin) if tanks else cg
    ends = [_structural(lo, origin), _structural(hi, origin)]

    def loc(name, p, unit="IN"):
        k = 1.0 / IN
        return ('        <location name="%s" unit="IN"> <x> %.3f </x> <y> %.3f </y> <z> %.3f </z> </location>'
                % (name, p[0] * k, p[1] * k, p[2] * k))

    def pointmass(name, p):
        k = 1.0 / IN
        return ("""        <pointmass name="%s">
            <weight unit="LBS"> 0 </weight>
            <location unit="IN"> <x> %.3f </x> <y> %.3f </y> <z> %.3f </z> </location>
        </pointmass>""" % (name, p[0] * k, p[1] * k, p[2] * k))

    def contact(name, p):
        k = 1.0 / IN
        return ("""        <contact type="STRUCTURE" name="%s">
            <location unit="IN"> <x> %.3f </x> <y> %.3f </y> <z> %.3f </z> </location>
            <static_friction> 0.8 </static_friction>
            <dynamic_friction> 0.5 </dynamic_friction>
            <spring_coeff unit="LBS/FT"> %.0f </spring_coeff>
            <damping_coeff unit="LBS/FT/SEC"> %.0f </damping_coeff>
        </contact>""" % (name, p[0] * k, p[1] * k, p[2] * k, 20.0 * s["mass_kg"] / LB, 2.0 * s["mass_kg"] / LB))

    qS = "<property>aero/qbar-psf</property>\n                    <property>metrics/Sw-sqft</property>"
    alpha = "aero/alpha-deg"
    edge = 0.02 + (0.1 if plate else BLUFF_AXIAL)  # (a plate's friction and its edge's pressure; a bluff break's)
    name = piece_name(data["aircraft"], section)
    aero = """    <aerodynamics>
        <axis name="NORMAL">
            <function name="aero/coefficient/CN">
                <description>normal force: a tumbling piece's (an estimate)</description>
                <product>
                    %s
%s
                </product>
            </function>
        </axis>
        <axis name="AXIAL">
            <function name="aero/coefficient/CA">
                <description>along it: friction and its edge's or break's pressure, against its way (an estimate)</description>
                <product>
                    %s
                    <cos><property>aero/alpha-rad</property></cos>
                    <value> %.4f </value>
                </product>
            </function>
        </axis>
        <axis name="SIDE">
            <function name="aero/coefficient/CY">
                <description>sideways: edge on, little; a bluff body's crossflow (an estimate)</description>
                <product>
                    %s
                    <sin><property>aero/beta-rad</property></sin>
                    <value> %.4f </value>
                </product>
            </function>
        </axis>
        <axis name="PITCH">
            <function name="aero/coefficient/Cm">
                <description>its normal force behind its centre of mass (the centre of pressure walking aft with alpha)</description>
                <product>
                    %s
                    <property>metrics/cbarw-ft</property>
                    <value> -1.0 </value>
%s
%s
                </product>
            </function>
            <function name="aero/coefficient/Cmq">
                <product>
                    %s
                    <property>metrics/cbarw-ft</property>
                    <property>aero/ci2vel</property>
                    <property>velocities/q-aero-rad_sec</property>
                    <value> %.3f </value>
                </product>
            </function>
        </axis>
        <axis name="ROLL">
            <function name="aero/coefficient/Clp">
                <product>
                    %s
                    <property>metrics/bw-ft</property>
                    <property>aero/bi2vel</property>
                    <property>velocities/p-aero-rad_sec</property>
                    <value> %.3f </value>
                </product>
            </function>
        </axis>
        <axis name="YAW">
            <function name="aero/coefficient/Cnr">
                <product>
                    %s
                    <property>metrics/bw-ft</property>
                    <property>aero/bi2vel</property>
                    <property>velocities/r-aero-rad_sec</property>
                    <value> %.3f </value>
                </product>
            </function>
        </axis>
    </aerodynamics>""" % (qS, _table(alpha, _normal_table(plate, aspect), 20), qS, edge, qS, -(0.1 if plate else PLATE_CN), qS,
                          _table(alpha, _normal_table(plate, aspect), 20), _table(alpha, _cp_table(plate), 20),
                          qS, DAMPING["Cmq"], qS, DAMPING["Clp"], qS, DAMPING["Cnr"])
    return """<?xml version="1.0"?>
<!-- Written by hangar (tools/hangar/hangar/pieces.py) from %s's damage data: its %s flying on its own once it parts
     (MS-7g). Its aerodynamics a tumbling piece's - estimates, said so there. Do not edit: hangar's model stage of %s writes it. -->
<fdm_config name="%s" version="2.0" release="ALPHA">
    <fileheader>
        <author> hangar </author>
        <description> %s's %s, parted </description>
    </fileheader>
    <metrics>
        <wingarea unit="FT2"> %.4f </wingarea>
        <wingspan unit="FT"> %.4f </wingspan>
        <chord unit="FT"> %.4f </chord>
        <htailarea unit="FT2"> 0 </htailarea>
        <htailarm unit="FT"> 0 </htailarm>
        <vtailarea unit="FT2"> 0 </vtailarea>
        <vtailarm unit="FT"> 0 </vtailarm>
%s
%s
%s
    </metrics>
    <mass_balance negated_crossproduct_inertia="false">
        <ixx unit="SLUG*FT2"> %.4f </ixx>
        <iyy unit="SLUG*FT2"> %.4f </iyy>
        <izz unit="SLUG*FT2"> %.4f </izz>
        <ixy unit="SLUG*FT2"> %.4f </ixy>
        <ixz unit="SLUG*FT2"> %.4f </ixz>
        <iyz unit="SLUG*FT2"> %.4f </iyz>
        <emptywt unit="LBS"> %.3f </emptywt>
%s
%s
%s
    </mass_balance>
    <ground_reactions>
%s
%s
    </ground_reactions>
    <propulsion/>
    <flight_control name="none"/>
%s
</fdm_config>
""" % (data["aircraft"], section, data["aircraft"], name, data["aircraft"], section, area / FT ** 2, span / FT, chord / FT,
       loc("AERORP", cg), loc("EYEPOINT", cg), loc("VRP", cg), ixx, iyy, izz, ixy, ixz, iyz, s["mass_kg"] / LB, loc("CG", cg),
       pointmass("fuel", fuel_at), "\n".join(pointmass("station %d" % st["number"], _structural(st["at"], origin))
                                               for st in s.get("stations", [])) or "<!-- no store station -->",
       contact("end 1", ends[0]), contact("end 2", ends[1]), aero)


def write(data, aircraft_dir):
    """Each parting section's piece beside the aircraft (aircraft/<type>/pieces/...); none for a design without a
    wing. The paths written."""
    root = os.path.join(aircraft_dir, "pieces")
    written = []
    if data is None:
        return written
    for section in PARTS:
        text = piece_xml(data, section)
        if text is None:
            continue
        name = piece_name(data["aircraft"], section)
        path = os.path.join(root, name, name + ".xml")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        if os.path.isfile(path):
            with open(path, encoding="utf-8") as f:
                if f.read() == text:
                    written.append(path)
                    continue
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
        written.append(path)
    return written
