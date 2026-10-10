"""A released store's own flight model for the stores whose cards have none built elsewhere (ADR-30, MS-7g): an
external fuel tank or a pod, jettisoned, flies on as a JSBSim vehicle - the same integrator, air, wind and ground as
every other - from aircraft/stores/<model>/<model>.xml (its card's model, "jsbsim:<model>": the card's id without its hyphens,
the name the viewer draws it by), written from its store card (assets/stores/<id>.json):

    python -m hangar storemodels        writes every tank's and pod's

- its mass the card's empty mass (its dry structure), a shell `length_m` long and `diameter_m` across; a point mass
  "fuel" at no weight that the world sets as it leaves - the fuel left in it (a tank's), a solid cylinder filling it;
- its aerodynamics a slender body's in body axes (estimates - no public source gives a jettisoned tank's tables:
  Hoerner's figures): its axial drag its card's drag_area_m2 (storedrag.py's build-up - the drag its aircraft feels
  while it is carried, less the carriage's) on its frontal area, CA0 without one, with the transonic rise the world
  gives a carried store (ms::Host::storeDragMachFactor); a crossflow normal force 1.2 sin(alpha) |sin(alpha)| on its
  planform (a cylinder's broadside drag), acting at its planform's centre (the AERORP: JSBSim takes the forces' moment
  about the CG from there); its body's unstable moment, its card's munk_volume_m3 times q sin(2 alpha) (Munk's couple:
  what turns an unfinned tank broadside and tumbles it), and its fins' and base's force across it, its card's
  side_slope_m2 and side_arm_m (a finned store's steadies it) - the figures the aircraft carrying it feels in a
  sideslip, in pitch and in yaw alike once it is free; its rotation damped as a slender body's;
- six structure contacts round each end, so it lies on its side where it lands.

A cargo store (its card's class "cargo": a loaded pallet, an airdrop bundle - what a transport's hold carries) is a
box, `length_m` by `width_m` by `height_m`, of its card's mass_kg throughout: a uniform block's inertia; a bluff
body's drag, its card's drag_cd on each face broadside to the flow (Hoerner: a short block face-on, 1.05 to 1.2), so
the force along each of its axes is that face's, by the square of the flow's share along it; nothing that steadies or
turns it but its rotation's damping (it has no parachute: it falls as the box it is); a structure contact at each
corner, so it stands where it lands.
"""
import json
import math
import os

LB, FT, IN = 0.45359237, 0.3048, 0.0254
SLUGFT2 = 1.35581795
CA0 = 0.15            # its axial drag on its frontal area below Mach 0.8 without a drag_area_m2 (Hoerner: 0.1 - 0.2)
# its transonic rise over its drag below Mach 0.8: ms::Host::storeDragMachFactor's, the same for a store carried or free
MACH_RISE = [(0.0, 1.0), (0.8, 1.0), (0.9, 1.02), (0.95, 1.07), (1.0, 1.28), (1.05, 2.12), (1.1, 2.37), (1.2, 2.43), (1.5, 2.2),
             (2.0, 1.8)]  # (the Mk 82's measured rise, DSTO-TR-0554)
CROSSFLOW = 1.2       # a cylinder's broadside drag coefficient
CMQ = -6.0            # its pitch and yaw damping on its length (a slender body's, an estimate)
CLP = -0.5
RING = 6              # contacts round each end


def _card(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def _value(card, name, fallback=None):
    p = card.get("parameters", {}).get(name)
    return p["value"] if p else fallback


def model_name(card):
    """The flight model's name: its card's model without "jsbsim:" (None: it has none)."""
    m = card.get("model", "")
    return m[7:] if m.startswith("jsbsim:") else None


def _planform(card, length, diameter):
    """Its planform's area (m^2) and its centre from its nose (m): from its drawn profile (a tank's, a pod's)."""
    if card.get("class") in ("tank", "pod"):
        from . import stores
        profile = stores.tank_spec(card)["profile"]  # (x m, radius / its largest)
        area = moment = 0.0
        for (x0, r0), (x1, r1) in zip(profile, profile[1:]):
            a = 0.5 * (r0 + r1) * (x1 - x0)
            area += a
            moment += a * (x0 + (x1 - x0) * (r0 + 2.0 * r1) / (3.0 * (r0 + r1)) if r0 + r1 > 0.0 else x0)
        return area * diameter, moment / area
    return length * diameter, 0.5 * length


def _moment(name, what, value_ft, angle, extra=""):
    """A moment function: q times `value_ft` (ft^3) times sin of `angle` (a property, or twice it)."""
    return """            <function name="aero/moment/%s">
                <description>%s</description>
                <product>
                    <property>aero/qbar-psf</property>
                    <value> %.5f </value>
                    %s
                </product>
            </function>
""" % (name, what, value_ft, angle + extra)


def store_xml(card):
    """The JSBSim definition of the store flying on its own (None: not a tank or a pod, or its card lacks its size)."""
    if card.get("class") not in ("tank", "pod") or not model_name(card):
        return None
    m = _value(card, "mass_kg")
    length, diameter = _value(card, "length_m"), _value(card, "diameter_m")
    if not (m and length and diameter):
        return None
    fuel_kg = _value(card, "fuel_kg", 0.0) or 0.0
    r = 0.5 * diameter
    frontal = math.pi * r * r
    ca0 = (_value(card, "drag_area_m2") or CA0 * frontal) / frontal
    planform, cp = _planform(card, length, diameter)
    # its structure a thin shell: about its axis m r^2, across m (r^2 / 2 + L^2 / 12)
    ixx = m * r * r
    iyy = m * (0.5 * r * r + length * length / 12.0)
    cg = 0.5 * length  # (its middle: x aft from its nose, the structural frame)
    munk = _value(card, "munk_volume_m3", 0.0) or 0.0           # m^3
    slope = _value(card, "side_slope_m2", 0.0) or 0.0           # m^2 / rad
    arm = (_value(card, "side_arm_m", 0.0) or 0.0) - (cp - cg)  # m: its fins' and base's force behind the AERORP
    k = 1.0 / IN

    def loc(name, x):
        return '        <location name="%s" unit="IN"> <x> %.3f </x> <y> 0 </y> <z> 0 </z> </location>' % (name, x * k)

    full = m + fuel_kg
    spring, damping = 20.0 * full / LB, 2.0 * full / LB  # (its weight on 1.5 cm when full; about critically damped)

    def contacts(name, x):
        out = []
        for i in range(RING):
            a = 2.0 * math.pi * (i + 0.5) / RING
            out.append("""        <contact type="STRUCTURE" name="%s %d">
            <location unit="IN"> <x> %.3f </x> <y> %.3f </y> <z> %.3f </z> </location>
            <static_friction> 0.8 </static_friction>
            <dynamic_friction> 0.5 </dynamic_friction>
            <spring_coeff unit="LBS/FT"> %.0f </spring_coeff>
            <damping_coeff unit="LBS/FT/SEC"> %.0f </damping_coeff>
        </contact>""" % (name, i + 1, x * k, r * math.cos(a) * k, r * math.sin(a) * k, spring / RING, damping / RING))
        return "\n".join(out)

    qS = "<property>aero/qbar-psf</property>\n                    <property>metrics/Sw-sqft</property>"
    # the reference area its frontal area; the crossflow's on its planform, so scaled by planform / frontal
    scale = planform / frontal
    normal = """<sin><property>aero/alpha-rad</property></sin>
                    <abs><sin><property>aero/alpha-rad</property></sin></abs>
                    <value> %.4f </value>""" % (CROSSFLOW * scale)
    side = """<sin><property>aero/beta-rad</property></sin>
                    <abs><sin><property>aero/beta-rad</property></sin></abs>
                    <value> %.4f </value>""" % (-CROSSFLOW * scale)
    rise = "\n".join("                            %.2f %.2f" % m for m in MACH_RISE)
    mach = """<table>
                        <independentVar>velocities/mach</independentVar>
                        <tableData>
%s
                        </tableData>
                    </table>""" % rise
    sin = lambda a: "<sin><property>aero/%s-rad</property></sin>" % a
    sin2 = lambda a: "<sin><product><value> 2 </value><property>aero/%s-rad</property></product></sin>" % a
    force = lambda name, what, a, sign: """            <function name="aero/force/%s">
                <description>%s</description>
                <product>
                    <property>aero/qbar-psf</property>
                    <value> %.5f </value>
                    %s
                </product>
            </function>
""" % (name, what, sign * slope / FT ** 2, sin(a))
    extra = {
        "NORMAL": force("fins-normal", "its fins' and base's force across it (its card's side_slope_m2)", "alpha", 1.0),
        "SIDE": force("fins-side", "its fins' and base's force across it, sideways", "beta", -1.0),
        "PITCH": _moment("munk-pitch", "its body's unstable moment (Munk's couple: its card's munk_volume_m3)", munk / FT ** 3, sin2("alpha"))
                 + _moment("fins-pitch", "its fins' and base's force, behind the reference point", -slope * arm / FT ** 3, sin("alpha")),
        "YAW": _moment("munk-yaw", "its body's unstable moment, in yaw", -munk / FT ** 3, sin2("beta"))
               + _moment("fins-yaw", "its fins' and base's side force, behind the reference point", slope * arm / FT ** 3, sin("beta")),
    }
    xml = _template(card, locals())
    for axis, functions in extra.items():
        xml = xml.replace('        <axis name="%s">\n' % axis, '        <axis name="%s">\n%s' % (axis, functions), 1)
    return xml


def _template(card, v):
    frontal, diameter, length, cp, cg, ixx, iyy, m, r, k = (v[n] for n in ("frontal", "diameter", "length", "cp", "cg", "ixx", "iyy", "m", "r", "k"))
    loc, contacts, qS, ca0, mach, normal, side = (v[n] for n in ("loc", "contacts", "qS", "ca0", "mach", "normal", "side"))
    return """<?xml version="1.0"?>
<!-- Written by hangar (tools/hangar/hangar/storemodels.py) from the store card %s: the store flying on its own once it
     is released or jettisoned (MS-7g). Its aerodynamics a slender body's - estimates, said so there. Do not edit. -->
<fdm_config name="%s" version="2.0" release="ALPHA">
    <fileheader>
        <author> hangar </author>
        <description> %s, released </description>
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
        <emptywt unit="LBS"> %.3f </emptywt>
%s
        <pointmass name="fuel">
            <form shape="cylinder">
                <radius unit="IN"> %.3f </radius>
                <length unit="IN"> %.3f </length>
            </form>
            <weight unit="LBS"> 0 </weight>
%s
        </pointmass>
    </mass_balance>
    <ground_reactions>
%s
%s
    </ground_reactions>
    <propulsion/>
    <flight_control name="none"/>
    <aerodynamics>
        <axis name="AXIAL">
            <function name="aero/coefficient/CA">
                <description>along it: a streamlined body's drag on its frontal area, its transonic rise (an estimate)</description>
                <product>
                    %s
                    <cos><property>aero/alpha-rad</property></cos>
                    <value> %.4f </value>
                    %s
                </product>
            </function>
        </axis>
        <axis name="NORMAL">
            <function name="aero/coefficient/CN">
                <description>its crossflow: a cylinder's broadside drag on its planform, at its centre of pressure %.3f m behind its CG (an estimate)</description>
                <product>
                    %s
                    %s
                </product>
            </function>
        </axis>
        <axis name="SIDE">
            <function name="aero/coefficient/CY">
                <description>its crossflow sideways</description>
                <product>
                    %s
                    %s
                </product>
            </function>
        </axis>
        <axis name="PITCH">
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
        <axis name="YAW">
            <function name="aero/coefficient/Cnr">
                <product>
                    %s
                    <property>metrics/cbarw-ft</property>
                    <property>aero/ci2vel</property>
                    <property>velocities/r-aero-rad_sec</property>
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
    </aerodynamics>
</fdm_config>
""" % (card["id"], model_name(card), card.get("designation", card["id"]), frontal / FT ** 2, diameter / FT, length / FT,
       loc("AERORP", cp), loc("EYEPOINT", cg), loc("VRP", cg),
       ixx / SLUGFT2, iyy / SLUGFT2, iyy / SLUGFT2, m / LB, loc("CG", cg),
       r * 0.95 * k, length * 0.8 * k,
       '            <location unit="IN"> <x> %.3f </x> <y> 0 </y> <z> 0 </z> </location>' % (cg * k),
       contacts("nose", 0.1 * length), contacts("tail", 0.9 * length),
       qS, ca0, mach,
       cp - cg, qS, normal, qS, side,
       qS, CMQ,
       qS, CMQ, qS, CLP)


CARGO_CD = 1.1        # a box face-on, without its card's drag_cd (Hoerner: 1.05 - 1.2)
CARGO_DAMPING = -2.0  # its rotation's damping on its length (a bluff body's, an estimate)


def cargo_xml(card):
    """The JSBSim definition of a cargo store on its own (None: not one, or its card lacks its size): a uniform box."""
    if card.get("class") != "cargo" or not model_name(card):
        return None
    m = _value(card, "mass_kg")
    L, W, H = _value(card, "length_m"), _value(card, "width_m"), _value(card, "height_m")
    if not (m and L and W and H):
        return None
    cd = _value(card, "drag_cd", CARGO_CD)
    k = 1.0 / IN
    ixx, iyy, izz = m * (W * W + H * H) / 12.0, m * (L * L + H * H) / 12.0, m * (L * L + W * W) / 12.0
    cg = 0.5 * L  # (its middle: x aft from its front face, the structural frame; y right, z up)
    front, side, top = W * H, L * H, L * W
    spring, damping = 20.0 * m / LB, 2.0 * m / LB  # (its weight on 1.5 cm; about critically damped), shared by its corners

    def loc(name):
        return '        <location name="%s" unit="IN"> <x> %.3f </x> <y> 0 </y> <z> 0 </z> </location>' % (name, cg * k)

    corners = []
    for i, x in enumerate((0.0, L)):
        for j, y in enumerate((-0.5 * W, 0.5 * W)):
            for n, z in enumerate((-0.5 * H, 0.5 * H)):
                corners.append("""        <contact type="STRUCTURE" name="corner %d">
            <location unit="IN"> <x> %.3f </x> <y> %.3f </y> <z> %.3f </z> </location>
            <static_friction> 0.8 </static_friction>
            <dynamic_friction> 0.6 </dynamic_friction>
            <spring_coeff unit="LBS/FT"> %.0f </spring_coeff>
            <damping_coeff unit="LBS/FT/SEC"> %.0f </damping_coeff>
        </contact>""" % (4 * i + 2 * j + n + 1, x * k, y * k, z * k, spring / 4.0, damping / 4.0))

    def face(name, what, area, along):
        # the force along one body axis: q, the face's drag area, and the square of the flow's share along that axis
        # (its sign the flow's) - cos a cos b along x, sin b along y, sin a cos b along z
        share = {"x": "<cos><property>aero/alpha-rad</property></cos>\n                    <cos><property>aero/beta-rad</property></cos>",
                 "y": "<sin><property>aero/beta-rad</property></sin>",
                 "z": "<sin><property>aero/alpha-rad</property></sin>\n                    <cos><property>aero/beta-rad</property></cos>"}[along]
        return """            <function name="aero/force/%s">
                <description>%s</description>
                <product>
                    <property>aero/qbar-psf</property>
                    <value> %.5f </value>
                    %s
                    <abs>%s</abs>
                </product>
            </function>
""" % (name, what, cd * area / FT ** 2, share, share if along == "y" else "<product>" + share + "</product>")

    def damp(name, rate, arm):
        return """            <function name="aero/moment/%s">
                <description>its rotation's damping (an estimate)</description>
                <product>
                    <property>aero/qbar-psf</property>
                    <property>metrics/Sw-sqft</property>
                    <value> %.4f </value>
                    <property>aero/ci2vel</property>
                    <property>velocities/%s-aero-rad_sec</property>
                    <value> %.3f </value>
                </product>
            </function>
""" % (name, arm / FT, rate, CARGO_DAMPING)

    return """<?xml version="1.0"?>
<!-- Written by hangar (tools/hangar/hangar/storemodels.py) from the store card %s: the cargo on its own once it has
     left its aircraft's hold (docs/hangar.md, Mass accounting). A uniform box, a bluff body's drag - estimates, said
     so there. Do not edit. -->
<fdm_config name="%s" version="2.0" release="ALPHA">
    <fileheader>
        <author> hangar </author>
        <description> %s, released </description>
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
        <emptywt unit="LBS"> %.3f </emptywt>
%s
    </mass_balance>
    <ground_reactions>
%s
    </ground_reactions>
    <propulsion/>
    <flight_control name="none"/>
    <aerodynamics>
        <axis name="AXIAL">
%s        </axis>
        <axis name="SIDE">
%s        </axis>
        <axis name="NORMAL">
%s        </axis>
        <axis name="ROLL">
%s        </axis>
        <axis name="PITCH">
%s        </axis>
        <axis name="YAW">
%s        </axis>
    </aerodynamics>
</fdm_config>
""" % (card["id"], model_name(card), card.get("designation", card["id"]), front / FT ** 2, W / FT, L / FT,
       loc("AERORP"), loc("EYEPOINT"), loc("VRP"),
       ixx / SLUGFT2, iyy / SLUGFT2, izz / SLUGFT2, m / LB, loc("CG"),
       "\n".join(corners),
       face("front", "along it: its front face's drag", front, "x"),
       face("side", "across it: its side face's drag", -side, "y"),
       face("top", "through it: its top face's drag", top, "z"),
       damp("roll-damping", "p", W), damp("pitch-damping", "q", L), damp("yaw-damping", "r", L))


def write(cards_dir, aircraft_dir):
    """Each tank's, pod's and cargo store's model (<aircraft_dir>/<model>/<model>.xml); the paths written."""
    written = []
    for name in sorted(os.listdir(cards_dir)):
        if not name.endswith(".json"):
            continue
        card = _card(os.path.join(cards_dir, name))
        text = store_xml(card) or cargo_xml(card)
        if text is None:
            continue
        model = model_name(card)
        path = os.path.join(aircraft_dir, model, model + ".xml")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        from . import pipeline
        pipeline.write_text(path, text)
        written.append(path)
    return written


def main():
    from . import pipeline
    root = pipeline.repo_root()
    for p in write(os.path.join(root, "assets", "stores"), os.path.join(root, "aircraft", "stores")):
        print(p)


if __name__ == "__main__":
    main()
