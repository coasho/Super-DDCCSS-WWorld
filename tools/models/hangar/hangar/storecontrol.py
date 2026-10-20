"""Finite, geometry-derived missile controls; explicitly uncalibrated estimates.

The authored controls, their mixed channels, mirrored geometry and stops are
shared with Hangar's visual/VLM conventions. The existing Barrowman/Diederich
fin slope and Mach-dependent CP supply incremental control lift. A partial
trailing-edge control uses thin-airfoil effectiveness, NASA/TP-2015-218751,
section 7, "Trailing-Edge Flaps" (method only, not equipment parameters):
https://rotorcraft.arc.nasa.gov/Publications/files/Johnson%20TP-2015-218751.pdf

Each control increment acts at that fin's CP. Lift is perpendicular to the
airflow; positive additional drag opposes it. Moments are r cross the SAME force
about the launch AERORP; JSBSim supplies the current-CG transfer. Body/fin passive
stability remains in storeaero; zero deflection adds zero force or moment.

Channel command response uses a class-wide assumed 25/s lag and 4 rad/s rate
limit. Each physical fin AFTER mixing has its own 4 rad/s actuator and authored
stops; it adds no second lag. This is a
reduced-order engineering model, not calibrated fin, hinge or servo behavior.

Standard channel position reports are the fixed authored mix's least-squares
equivalent axes reconstructed from ACTUAL physical fin positions. Individual
jam/saturation residuals cannot fit the frozen three-axis VehicleState layout;
the per-fin properties retain that diagnostic information without inventing an
unactuated axis or reporting command response as physical position.
"""
import math
import re
import numpy as np
from types import SimpleNamespace
import xml.etree.ElementTree as ET

from . import storeaero
from .aero.vlm import channel_gain
from .geometry.surface import Surface
from . import finregistry

CHANNELS = ("elevator", "rudder", "aileron")
OUTPUTS = {"elevator": "elevator", "rudder": "rudder", "aileron": "left-aileron"}
LAG = 25.0                 # 1/s, assumed: 0.04 s first-order response
RATE = 4.0                 # rad/s, assumed channel-effector rate
FT = 0.3048


def _number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("missile control: %s must be finite numeric" % label)
    try:
        result = float(value)
    except (ValueError, OverflowError) as error:
        raise ValueError("missile control: %s must be finite numeric" % label) from error
    if not math.isfinite(result):
        raise ValueError("missile control: %s must be finite" % label)
    return result


def _effectiveness(fraction):
    if not 0 < fraction <= 1:
        raise ValueError("missile control: chord fraction must be within (0, 1]")
    theta = math.acos(2 * fraction - 1)
    return 1 - (theta - math.sin(theta)) / math.pi


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def build(geo, baseline):
    """Finite-control description, without changing the selected passive table.

    Coefficients are dimensionless BODY-axis derivatives per radian, moments
    referenced to the launch AERORP on S and d. Rate derivatives retain d/(2V).
    A design without authored missile controls has no substitute control model.
    """
    design = geo.get("design") or {}
    if design.get("aircraft", {}).get("category") != "missile":
        return None
    S, d, mach = baseline["S"], baseline["d"], baseline["mach"]
    # geometry() locates its first body station at x=0; authored surfaces keep
    # absolute design coordinates, including a dome ahead of that first station.
    body = next(b for b in design["body"] if b.get("kind") == "fuselage")
    origin = float(body["stations"][0]["x"])
    fins, controls = [], []
    physical = {}
    for spec in design.get("surface", []):
        if not spec.get("controls"):
            continue
        for raw in spec["controls"]:
            for field in ("gain", "pivot"):
                if field in raw:
                    _number(raw[field], field)
            for field in ("limits", "span"):
                if field not in raw or not isinstance(raw[field], (list, tuple)) or len(raw[field]) != 2:
                    raise ValueError("missile control: %s needs two bounds" % field)
                for value in raw[field]:
                    _number(value, field)
            fractions = raw.get("chord_fraction", 0.25)
            for value in fractions if isinstance(fractions, (list, tuple)) else [fractions]:
                _number(value, "chord fraction")
            for value in raw.get("mix", {}).values():
                _number(value, "mixer gain")
        surface = Surface(spec)
        names = [control.name for control in surface.controls]
        if len(names) != len(set(names)):
            raise ValueError("missile control: authored control names must be unique within a surface")
        full_root, full_cr, _, _ = surface.station(0)
        full_tip, full_ct, _, _ = surface.station(1)
        full_s = surface.half_arc
        full_area = surface.area / (2 if surface.mirror else 1)
        full = dict(x=float(full_root[0]) - origin, cr=full_cr, ct=full_ct,
                    xt=float(full_tip[0] - full_root[0]), s=full_s,
                    r=math.hypot(float(full_root[1]), float(full_root[2])),
                    lattice=bool(surface.lattice))
        for control in surface.controls:
            if not control.min_deg <= 0 <= control.max_deg or not control.min_deg < control.max_deg:
                raise ValueError("missile control: stops must contain neutral and have positive travel")
            if any(ch not in CHANNELS and gain != 0 for ch, gain in control.channels.items()):
                raise ValueError("missile control: unsupported missile channel")
            controls.append(control)
            edges = sorted({control.eta0, control.eta1} |
                           {float(e) for e in surface.eta_sections if control.eta0 < e < control.eta1})
            for eta0, eta1 in zip(edges, edges[1:]):
                a, cr, _, _ = surface.station(eta0)
                b, ct, _, _ = surface.station(eta1)
                mid = .5 * (eta0 + eta1)
                _, normal = surface.frame(mid)
                run = (eta1 - eta0) * full_s
                patch = dict(x=float(a[0]) - origin, cr=cr, ct=ct, xt=float(b[0] - a[0]), s=run,
                             r=math.hypot(float(a[1]), float(a[2])), lattice=bool(surface.lattice))
                area = .5 * run * (cr + ct)
                effectiveness = _effectiveness(control.chord_fraction(mid))
                slope = [storeaero._fin_slope(full, S, m) * (1 + full["r"] / (full_s + full["r"]))
                         * area / full_area * effectiveness for m in mach]
                for side in ((1, -1) if surface.mirror else (1,)):
                    u = (float(normal[0]), side * float(normal[1]), float(normal[2]))
                    center = ((float(a[0]) + float(b[0])) / 2,
                              side * (float(a[1]) + float(b[1])) / 2, (float(a[2]) + float(b[2])) / 2)
                    gains = {ch: float(channel_gain(ch, control, u, center)) for ch in CHANNELS}
                    if not any(gains.values()):
                        continue
                    orient = float(channel_gain(control.channel,
                        SimpleNamespace(channels={control.channel: 1.0}), u, center))
                    lo, hi = sorted((math.radians(control.min_deg) * orient,
                                     math.radians(control.max_deg) * orient))
                    if not all(math.isfinite(v) for v in (*u, *gains.values(), lo, hi, *slope)):
                        raise ValueError("missile control: invalid geometry or derived coefficient")
                    # The existing CP rule supplies chordwise and spanwise location;
                    # interpolate the authored (possibly mirrored) y/z span line.
                    cps = []
                    for m in mach:
                        xcp, radial = storeaero._fin_cp(patch, m)
                        fraction = (radial - patch["r"]) / run
                        ycp = side * float(a[1] + fraction * (b[1] - a[1]))
                        zcp = float(a[2] + fraction * (b[2] - a[2]))
                        cps.append((geo["cg"] - xcp, ycp, -zcp))
                    key = finregistry.identity(surface, control, side)
                    authored = (surface.name, control.name, side)
                    if key in physical and (physical[key][0] != authored or physical[key][2] is not surface):
                        raise ValueError("missile fin identity collision or duplicate authored surface")
                    if key not in physical:
                        fin = dict(id=key, gains=gains, lo=lo, hi=hi, patches=[])
                        physical[key] = (authored, fin, surface)
                        fins.append(fin)
                    physical[key][1]["patches"].append(dict(slope=slope,
                        normal=(-u[0], u[1], -u[2]), cp=cps))
    if not fins:
        return None
    limits = {}
    for channel in CHANNELS:
        own = [c for c in controls if c.channel == channel and abs(c.channels[channel]) > 1e-12]
        mixed = [c for c in controls if channel in c.mix and abs(c.channels[channel]) > 1e-12]
        bounds = [sorted((math.radians(c.min_deg) / c.channels[channel],
                          math.radians(c.max_deg) / c.channels[channel])) for c in own or mixed]
        limits[channel] = (min(p[0] for p in bounds), max(p[1] for p in bounds)) if bounds else (0.0, 0.0)
    derivatives = {name: [sign * v for v in baseline[key]] for name, key, sign in (
        ("cz-alpha", "cna", -1), ("cy-beta", "cyb", -1), ("cm-alpha", "cma", 1),
        ("cn-beta", "cnb", -1), ("cm-q", "cmq", 1), ("cn-r", "cnr", 1), ("cl-p", "clp", 1))}
    for name, channel, kind, component in (("cz-elevator", "elevator", "force", 2),
            ("cy-rudder", "rudder", "force", 1), ("cm-elevator", "elevator", "moment", 1),
            ("cn-rudder", "rudder", "moment", 2), ("cl-aileron", "aileron", "moment", 0)):
        derivatives[name] = [sum(patch["slope"][i] * fin["gains"][channel] *
            (patch["normal"][component] if kind == "force" else _cross(patch["cp"][i], patch["normal"])[component] / d)
            for fin in fins for patch in fin["patches"]) for i in range(len(mach))]
    return dict(fins=fins, channels=limits, mach=list(mach), derivatives=derivatives)


def _value(x):
    node = ET.Element("value")
    node.text = "%.17g" % x
    return node


def _property(name):
    node = ET.Element("property")
    node.text = name
    return node


def _op(tag, *children):
    if tag == "sum" and len(children) < 2:
        return children[0] if children else _value(0)
    node = ET.Element(tag)
    node.extend(children)
    return node


def _table(mach, values):
    node = ET.Element("table")
    ET.SubElement(node, "independentVar", lookup="row").text = "velocities/mach"
    ET.SubElement(node, "tableData").text = "\n" + "\n".join("%.17g %.17g" % pair for pair in zip(mach, values)) + "\n"
    return node


def _function(name, expression):
    node = ET.Element("function", name=name, type="pre")
    node.append(expression)
    return node


def apply(xml, old_controls, geo, baseline, origin_inches):
    """Extend only an authored controlled missile's XML. Motor files are untouched."""
    control = build(geo, baseline)
    if control is None:
        return xml
    # Controller derivatives and generated force/moment normalization must use
    # the same area and diameter, including on the older build-up path whose
    # passive-only XML rounded these metrics to five decimals.
    for tag, number in (("wingarea", baseline["S"] / FT ** 2),
                        ("wingspan", baseline["d"] / FT), ("chord", baseline["d"] / FT)):
        xml = re.sub(r'(<%s unit="[A-Z0-9]+">)[^<]*(</%s>)' % (tag, tag),
                     lambda match: match[1] + " %.17g " % number + match[2], xml, count=1)
    fcs = ET.Element("flight_control", name="Finite missile controls (engineering estimates)")
    metadata = {"version": 1, "surface-limit-rad": max(max(abs(v) for v in limits) for limits in control["channels"].values()),
                "surface-rate-rad_s": RATE, "channel-rate-rad_s": RATE, "lag-coefficient-s-1": LAG,
                "fin-count": len(control["fins"])}
    metadata.update({ch + "-limit-rad": max(abs(v) for v in bounds) for ch, bounds in control["channels"].items()})
    for i, fin in enumerate(control["fins"]):
        metadata["fin-%d-id" % i] = fin["id"]
        metadata["fin-%d-min-rad" % i] = fin["lo"]
        metadata["fin-%d-max-rad" % i] = fin["hi"]
        metadata["fin-%d-rate-rad_s" % i] = RATE
    for name, number in metadata.items():
        ET.SubElement(fcs, "property", value="%.17g" % number).text = "fsim/missile-control/" + name
    for axis, number in zip("xyz", (origin_inches, 0, 0)):
        ET.SubElement(fcs, "property", value="%.17g" % number).text = "fsim/model/origin-%s-in" % axis
    channels = ET.SubElement(fcs, "channel", name="Finite channel actuators and authored fin mixers")
    for channel in CHANNELS:
        low, high = control["channels"][channel]
        scale = ET.SubElement(channels, "aerosurface_scale", name="fcs/%s-control" % channel)
        ET.SubElement(scale, "input").text = "fcs/%s-cmd-norm" % channel
        limits = ET.SubElement(scale, "range")
        ET.SubElement(limits, "min").text = "%.17g" % low
        ET.SubElement(limits, "max").text = "%.17g" % high
        actuator = ET.SubElement(channels, "actuator", name="fcs/%s-actuator" % channel)
        ET.SubElement(actuator, "input").text = "fcs/%s-control" % channel
        ET.SubElement(actuator, "lag").text = "%.17g" % LAG
        ET.SubElement(actuator, "rate_limit").text = "%.17g" % RATE
        stops = ET.SubElement(actuator, "clipto")
        ET.SubElement(stops, "min").text = "%.17g" % low
        ET.SubElement(stops, "max").text = "%.17g" % high
        ET.SubElement(actuator, "output").text = "fcs/%s-response-rad" % channel
    for i, fin in enumerate(control["fins"]):
        mixer = ET.SubElement(channels, "fcs_function", name="fcs/fin-%d-demand-rad" % i)
        expression = ET.SubElement(mixer, "function")
        expression.append(_op("sum", *(_op("product", _value(gain), _property("fcs/%s-response-rad" % ch))
            for ch, gain in fin["gains"].items() if gain)))
        stops = ET.SubElement(mixer, "clipto")
        ET.SubElement(stops, "min").text = "%.17g" % fin["lo"]
        ET.SubElement(stops, "max").text = "%.17g" % fin["hi"]
        actuator = ET.SubElement(channels, "actuator", name="fcs/fin-%d-actuator" % i)
        ET.SubElement(actuator, "input").text = "fcs/fin-%d-demand-rad" % i
        ET.SubElement(actuator, "rate_limit").text = "%.17g" % RATE
        stops = ET.SubElement(actuator, "clipto")
        ET.SubElement(stops, "min").text = "%.17g" % fin["lo"]
        ET.SubElement(stops, "max").text = "%.17g" % fin["hi"]
        ET.SubElement(actuator, "output").text = "fcs/fin-%d-delta-rad" % i
    mixing = np.asarray([[fin["gains"][channel] for channel in CHANNELS] for fin in control["fins"]], dtype=float)
    equivalent_axes = np.linalg.pinv(mixing)
    equivalent_axes[np.abs(equivalent_axes) < 1e-12] = 0
    for channel, row in zip(CHANNELS, equivalent_axes):
        report = ET.SubElement(channels, "fcs_function", name="fcs/%s-position" % channel)
        expression = ET.SubElement(report, "function")
        expression.append(_op("sum", *(_op("product", _value(float(gain)),
            _property("fcs/fin-%d-delta-rad" % i)) for i, gain in enumerate(row) if gain)))
        ET.SubElement(report, "output").text = "fcs/%s-pos-rad" % OUTPUTS[channel]
    alias = ET.SubElement(channels, "pure_gain", name="fcs/aileron-position-alias")
    ET.SubElement(alias, "input").text = "fcs/left-aileron-pos-rad"
    ET.SubElement(alias, "gain").text = "1"
    ET.SubElement(alias, "output").text = "fcs/aileron-pos-rad"
    ET.indent(fcs, space="    ")
    xml = xml.replace(old_controls, ET.tostring(fcs, encoding="unicode"), 1)
    pre = []
    for name, values in control["derivatives"].items():
        pre.append(_function("fsim/control/" + name, _table(control["mach"], values)))
    airflow = (_op("product", _op("cos", _property("aero/alpha-rad")), _op("cos", _property("aero/beta-rad"))),
               _op("sin", _property("aero/beta-rad")),
               _op("product", _op("sin", _property("aero/alpha-rad")), _op("cos", _property("aero/beta-rad"))))
    for axis, expression in zip("xyz", airflow):
        pre.append(_function("fsim/fin-control/air-" + axis, expression))
    for i, physical_fin in enumerate(control["fins"]):
        patch_prefixes = []
        for j, fin in enumerate(physical_fin["patches"]):
            p = "fsim/fin-control/fin-%d/" % i
            if len(physical_fin["patches"]) > 1:
                p += "patch-%d/" % j
            patch_prefixes.append(p)
            delta = "fcs/fin-%d-delta-rad" % i
            pre.append(_function(p + "slope", _table(control["mach"], fin["slope"])))
            pre.append(_function(p + "normal-flow", _op("sum", *(_op("product", _value(n),
                _property("fsim/fin-control/air-" + axis)) for axis, n in zip("xyz", fin["normal"])))))
            for kind, trig in (("lift", "cos"), ("drag", "sin")):
                pre.append(_function(p + kind, _op("product", _property("aero/qbar-psf"),
                    _property("metrics/Sw-sqft"), _property(p + "slope"),
                    _op("sin", _property(delta)), _op(trig, _property(delta)))))
            for axis, normal in zip("xyz", fin["normal"]):
                pre.append(_function(p + "force-" + axis, _op("difference",
                    _op("product", _property(p + "lift"), _op("difference", _value(normal),
                        _op("product", _property(p + "normal-flow"), _property("fsim/fin-control/air-" + axis)))),
                    _op("product", _property(p + "drag"), _property("fsim/fin-control/air-" + axis)))))
            for k, axis in enumerate("xyz"):
                pre.append(_function(p + "arm-" + axis, _table(control["mach"], [cp[k] / FT for cp in fin["cp"]])))
            for axis, a, b in (("l", "yz", "zy"), ("m", "zx", "xz"), ("n", "xy", "yx")):
                pre.append(_function(p + "moment-" + axis, _op("difference",
                    _op("product", _property(p + "arm-" + a[0]), _property(p + "force-" + a[1])),
                    _op("product", _property(p + "arm-" + b[0]), _property(p + "force-" + b[1])))))
        if len(patch_prefixes) > 1:
            for kind, axes in (("force", "xyz"), ("moment", "lmn")):
                for axis in axes:
                    pre.append(_function("fsim/fin-control/fin-%d/%s-%s" % (i, kind, axis),
                        _op("sum", *(_property(prefix + kind + "-" + axis) for prefix in patch_prefixes))))
    for kind, axes, unit in (("force", "xyz", "lbs"), ("moment", "lmn", "lbsft")):
        for axis in axes:
            pre.append(_function("aero/%s/fsim-fin-%s-%s" % (kind, axis, unit), _op("sum",
                *(_property("fsim/fin-control/fin-%d/%s-%s" % (i, kind, axis)) for i in range(len(control["fins"]))))))
    description = "    <!-- Fin control derivatives/lag/rate and flap/drag corrections are engineering estimates, not equipment calibration. -->\n"
    xml = xml.replace("    <aerodynamics>", "    <aerodynamics>\n" + description +
        "\n".join(ET.tostring(node, encoding="unicode") for node in pre), 1)
    for axis, kind, component, sign, unit in (("AXIAL", "force", "x", -1, "lbs"),
            ("SIDE", "force", "y", 1, "lbs"), ("NORMAL", "force", "z", -1, "lbs"),
            ("ROLL", "moment", "l", 1, "lbsft"), ("PITCH", "moment", "m", 1, "lbsft"),
            ("YAW", "moment", "n", 1, "lbsft")):
        contribution = ET.Element("function", name="aero/coefficient/fsim-control-" + axis.lower())
        contribution.append(_op("product", _value(sign), _property("aero/%s/fsim-fin-%s-%s" % (kind, component, unit))))
        xml = xml.replace('        <axis name="%s">' % axis,
            '        <axis name="%s">\n%s' % (axis, ET.tostring(contribution, encoding="unicode")), 1)
    return xml
