"""Derive current game-plant directional-control theory from explicit PRE captures.

This tool is an offline mathematical reference. It does not execute a vehicle,
change a card, tune a controller, or admit current native/full-chain acceptance.
"""
from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import numpy as np

from acceptance import allowed

MATRIX_PATH = "docs/research/sim-m03/component-methods/per-model-six-components.json"
FAMILY_PATH = "docs/research/sim-m03/component-methods/autopilot.json"
MARKDOWN_PATH = "docs/research/sim-m03/component-methods/autopilot.md"
CASCADE = "fsim.control.load_feedback_cascade"
AWARE = "fsim.control.actuator_aware_cascade.v1"
OPERATING = "fsim.control.operating_point_bounded.v1"


def axis_indices(name):
    if name == "pitch":
        return 1, 2, 1, "alpha_rad"
    if name == "yaw":
        return -1, 1, 2, "beta_rad"
    raise ValueError("unknown control axis")


def residuals(i, name):
    _, faxis, maxis, angle = axis_indices(name)
    a, measured = i[name], i["measured"]
    w, inertia = i["rates_rad_s"], i["inertia_kg_m2"]
    gyro = ((inertia[2]-inertia[1])*w[1]*w[2],
            (inertia[0]-inertia[2])*w[0]*w[2],
            (inertia[1]-inertia[0])*w[0]*w[1])
    return {"force": measured["force_body_n"][faxis] - a["force_angle_n_rad"]*i[angle] -
            a["force_control_n_rad"]*measured["control_rad"][maxis],
            "moment": measured["moment_body_nm"][maxis] - a["moment_angle_nm_rad"]*i[angle] -
            a["moment_rate_nm_s_rad"]*w[maxis] - a["moment_control_nm_rad"]*measured["control_rad"][maxis],
            "gyro": gyro[maxis]}


def directional_trim(i, name):
    sign, faxis, _, _ = axis_indices(name)
    a, r = i[name], residuals(i, name)
    m, speed = i["mass_kg"], i["speed_m_s"]
    d, g = i["demand_body_m_s2"], i["gravity_body_m_s2"]
    fa, fc = a["force_angle_n_rad"], a["force_control_n_rad"]
    ma, mc, mr = a["moment_angle_nm_rad"], a["moment_control_nm_rad"], a["moment_rate_nm_s_rad"]
    if not math.isfinite(mc) or abs(mc) <= 1e-9 or m <= 0 or speed < 1:
        raise ValueError("invalid/singular native axis")
    effective = fa-fc*ma/mc
    determinant = effective-i["measured"]["force_body_n"][0]+m*d[0]-sign*fc*mr*(d[0]+g[0])/(mc*speed)
    if not math.isfinite(determinant) or abs(determinant) < 1e-6*max(abs(fa), 1.):
        raise ValueError("directional trim determinant is singular")
    path = -sign*(d[faxis]+g[faxis])/speed
    moment = r["moment"]-r["gyro"]
    angle = (m*d[faxis]-r["force"]+fc*(mr*path+moment)/mc)/determinant
    path += sign*(d[0]+g[0])*angle/speed
    surface = -(ma*angle+mr*path+moment)/mc
    return {"angle_rad": angle, "path_rate_rad_s": path, "surface_rad": surface,
            "effective_aerodynamic_force_n_rad": effective, "determinant_n_rad": determinant}


def response_times(packet):
    c, i = packet["config"], packet["input"]
    rate = max(c["rate_time_min_s"], c["rate_lag_multiplier"]*i["actuator_lag_s"])
    return rate, max(c["incidence_time_min_s"], c["incidence_rate_multiplier"]*rate)


def directional_state_matrix(i, name):
    sign, _, maxis, _ = axis_indices(name)
    a, mv, inertia = i[name], i["mass_kg"]*i["speed_m_s"], i["inertia_kg_m2"][maxis]
    # This is the CURRENT reduced small-angle directional plant, including
    # the native total nongravity Fx and body gravity_x. It is not Fa/(mV).
    aa = (a["force_angle_n_rad"]-i["measured"]["force_body_n"][0]-i["mass_kg"]*i["gravity_body_m_s2"][0])/mv
    return [[aa, sign, a["force_control_n_rad"]/mv],
            [a["moment_angle_nm_rad"]/inertia, a["moment_rate_nm_s_rad"]/inertia, a["moment_control_nm_rad"]/inertia],
            [0., 0., -1/i["actuator_lag_s"]]]


def feedback_gains(packet, name):
    i = packet["input"]
    sign, _, maxis, _ = axis_indices(name)
    a, inertia = i[name], i["inertia_kg_m2"][maxis]
    rate, incidence = response_times(packet)
    if packet["mechanism"] != AWARE:
        return [(inertia*sign/(rate*incidence)+a["moment_angle_nm_rad"])/a["moment_control_nm_rad"],
                (inertia/rate+a["moment_rate_nm_s_rad"])/a["moment_control_nm_rad"], 0.]
    A = directional_state_matrix(i, name)
    aa, b, c = A[0]
    d, e, f = A[1]
    lag, fast, slow = 1/i["actuator_lag_s"], 1/rate, 1/incidence
    p2, p1, p0 = lag+fast+slow, lag*fast+lag*slow+fast*slow, lag*fast*slow
    det, kd = aa*e-b*d, (p2+aa+e)/lag-1
    u, v, y, z = c, f, b*f-c*e, c*d-aa*f
    determinant = u*z-v*y
    if not math.isfinite(determinant) or abs(determinant) < 1e-12*max(abs(u*z), abs(v*y), 1.):
        raise ValueError("actuator-aware pole assignment determinant is singular")
    rhs1, rhs0 = (p1-det+lag*(1+kd)*(aa+e))/lag, (p0-lag*(1+kd)*det)/lag
    return [(rhs1*z-v*rhs0)/determinant, (u*rhs0-rhs1*y)/determinant, kd]


def bounded_trims(packet):
    i = packet["input"]
    result = {axis: directional_trim(i, axis) for axis in ("pitch", "yaw")}
    incidence = math.hypot(*(result[axis]["angle_rad"] for axis in ("pitch", "yaw")))
    limit = packet["config"]["max_incidence_rad"]
    if incidence <= limit:
        return result
    for name in result:
        sign, faxis, _, _ = axis_indices(name)
        a, r, trim = i[name], residuals(i, name), result[name]
        angle = trim["angle_rad"]*limit/incidence
        fc, mc, mr = a["force_control_n_rad"], a["moment_control_nm_rad"], a["moment_rate_nm_s_rad"]
        denominator = i["mass_kg"]*i["speed_m_s"]-sign*fc*mr/mc
        if abs(denominator) < 1e-9:
            raise ValueError("constrained path denominator is singular")
        force = trim["effective_aerodynamic_force_n_rad"]*angle+r["force"]-fc*(r["moment"]-r["gyro"])/mc
        force -= (i["measured"]["force_body_n"][0]+i["mass_kg"]*i["gravity_body_m_s2"][0])*angle
        path = -sign*(force+i["mass_kg"]*i["gravity_body_m_s2"][faxis])/denominator
        trim.update(angle_rad=angle, path_rate_rad_s=path,
                    surface_rad=-(a["moment_angle_nm_rad"]*angle+mr*path+r["moment"]-r["gyro"])/mc)
    return result


def normalize(surface, axis, stops):
    extent = -stops["lower_rad"][axis] if surface < 0 else stops["upper_rad"][axis]
    return max(-1., min(1., surface/extent)) if extent > 0 else 0.


def commands(packet):
    if packet["mechanism"] not in (CASCADE, AWARE):
        raise ValueError("bounded operating-point search is not a cascade")
    i, stops = packet["input"], packet["stops"]
    try:
        trims = bounded_trims(packet)
        rate_time, incidence_time = response_times(packet)
        out = {"available": True}
        for name, output in (("pitch", "elevator"), ("yaw", "rudder")):
            sign, _, maxis, angle_name = axis_indices(name)
            a, residual, trim = i[name], residuals(i, name), trims[name]
            angle, rate = i[angle_name], i["rates_rad_s"][maxis]
            if packet["mechanism"] == AWARE:
                gains = feedback_gains(packet, name)
                surface = (trim["surface_rad"]-gains[0]*(angle-trim["angle_rad"])-
                           gains[1]*(rate-trim["path_rate_rad_s"])-
                           gains[2]*(i["measured"]["control_rad"][maxis]-trim["surface_rad"]))
            else:
                desired = trim["path_rate_rad_s"]+sign*(trim["angle_rad"]-angle)/incidence_time
                surface = (i["inertia_kg_m2"][maxis]*(desired-rate)/rate_time+residual["gyro"]-
                           a["moment_angle_nm_rad"]*angle-a["moment_rate_nm_s_rad"]*rate-residual["moment"])/a["moment_control_nm_rad"]
            out[output] = normalize(surface, maxis, stops)
        w, inertia, measured = i["rates_rad_s"], i["inertia_kg_m2"], i["measured"]
        residual = measured["moment_body_nm"][0]-i["roll_damping"]*w[0]-i["roll_moment_per_rad"]*measured["control_rad"][0]
        gyro = (inertia[2]-inertia[1])*w[1]*w[2]
        surface = ((-inertia[0]*w[0]/rate_time+gyro-i["roll_damping"]*w[0]-residual)/i["roll_moment_per_rad"]
                   if i["limits_rad"][0] > 0 and abs(i["roll_moment_per_rad"]) > 1e-9 else 0.)
        out["aileron"] = normalize(surface, 0, stops)
        if not all(math.isfinite(v) for k, v in out.items() if k != "available"):
            raise ValueError("nonfinite reference output")
        return out
    except (ValueError, ZeroDivisionError):
        return dict(available=False, aileron=0., elevator=0., rudder=0.)


def operating_prediction(packet, command):
    """Finite captured FCS/fin-curve reference; not a new control optimizer."""
    point, stops, i = packet["operating_point"], packet["stops"], packet["input"]
    if point is None or point["actuator_substeps"] < 1 or point["dt_s"] <= 0:
        raise ValueError("an actual finite operating-point capture is required")
    dt = point["dt_s"]/point["actuator_substeps"]
    k = dt/i["actuator_lag_s"]
    ca, cb = k/(2+k), (2-k)/(2+k)
    lag, channel, previous = (list(point[key]) for key in ("channel_lag_rad", "channel_actual_rad", "channel_input_rad"))
    demand = [command[a]*(-stops["lower_rad"][a] if command[a] < 0 else stops["upper_rad"][a]) for a in range(3)]
    fins = [fin["actual_rad"] for fin in point["fins"]]

    def clip(value, low, high):
        return max(low, min(high, value))

    def failure(value, state, low, high):
        value = 0. if state["zero"] else value
        return (low if value < 0 else high) if state["hardover"] else value

    for _ in range(point["actuator_substeps"]):
        for axis in range(3):
            lag[axis] = (demand[axis]+previous[axis])*ca+lag[axis]*cb
            previous[axis] = demand[axis]
            state = point["channel_failures"][axis]
            if not state["stuck"]:
                value = failure(lag[axis], state, stops["lower_rad"][axis], stops["upper_rad"][axis])
                channel[axis] = clip(value, channel[axis]-stops["channel_rate_rad_s"]*dt,
                                     channel[axis]+stops["channel_rate_rad_s"]*dt)
                channel[axis] = clip(channel[axis], stops["lower_rad"][axis], stops["upper_rad"][axis])
        for j, fin in enumerate(point["fins"]):
            if fin["failure"]["stuck"]:
                continue
            mix = clip(sum(a*b for a, b in zip(channel, fin["mixer"])), fin["minimum_rad"], fin["maximum_rad"])
            mix = failure(mix, fin["failure"], fin["minimum_rad"], fin["maximum_rad"])
            fins[j] = clip(mix, fins[j]-fin["rate_rad_s"]*dt, fins[j]+fin["rate_rad_s"]*dt)
            fins[j] = clip(fins[j], fin["minimum_rad"], fin["maximum_rad"])
    moment = list(i["measured"]["moment_body_nm"])
    for j, fin in enumerate(point["fins"]):
        values = np.asarray(fin["moments_nm"])
        for axis in range(3):
            moment[axis] += float(np.interp(fins[j], fin["angles_rad"], values[:, axis])-
                                  np.interp(fin["actual_rad"], fin["angles_rad"], values[:, axis]))
    return dict(moment_nm=moment, channel_lag_rad=lag, channel_actual_rad=channel, fin_angles_rad=fins)


def operating_reference(packet):
    nominal = copy.deepcopy(packet)
    nominal["mechanism"] = CASCADE
    requested = commands(nominal)
    if not requested["available"]:
        raise ValueError("operating-point seed is unavailable")
    names = ("aileron", "elevator", "rudder")
    wanted = [requested[name] for name in names]
    virtual = operating_prediction(packet, wanted)["channel_actual_rad"]
    i = packet["input"]
    gain = [i["roll_moment_per_rad"], i["pitch"]["moment_control_nm_rad"], i["yaw"]["moment_control_nm_rad"]]
    target = [i["measured"]["moment_body_nm"][a]+gain[a]*(virtual[a]-i["measured"]["control_rad"][a]) for a in range(3)]

    def objective(u):
        moment = operating_prediction(packet, u)["moment_nm"]
        return sum(((moment[a]-target[a])/i["inertia_kg_m2"][a])**2 for a in range(3))

    captured = [packet["commands"][name] for name in names]
    alternatives = [[0., 0., 0.], *itertools.product((-1., 1.), repeat=3)]
    return {"requested_moment_nm": target,
            "captured_objective_s_4": objective(captured),
            "best_neutral_or_corner_objective_s_4": min(objective(u) for u in alternatives),
            "predicted_fin_angles_rad": operating_prediction(packet, captured)["fin_angles_rad"],
            "scope": "Independent finite FCS/curve evaluation of captured command against nine feasible controls; not global optimality or a native replay."}


def replace_autopilot_cells(text, replacements):
    """Replace only the exact JSON object spans of the 24 autopilot cells."""
    decoder = json.JSONDecoder()
    key = text.index('"cells"')
    position = text.index("[", key)+1
    edits = []
    while True:
        while text[position].isspace() or text[position] == ",":
            position += 1
        if text[position] == "]":
            break
        row, end = decoder.raw_decode(text, position)
        if row.get("component") == "autopilot":
            dumped = json.dumps(replacements[row["model"]], indent=2, ensure_ascii=True)
            edits.append((position, end, dumped.replace("\n", "\n    ")))
        position = end
    for start, end, replacement in reversed(edits):
        text = text[:start]+replacement+text[end:]
    return text


EQUATIONS = [
    "m*(v_dot_body + omega cross v_body)=F_nongravity_body+m*g_body; J*omega_dot=M-omega cross(J*omega)",
    "pitch s=+1, theta=alpha, rate=q, transverse=z; yaw s=-1, theta=beta, rate=r, transverse=y",
    "theta_dot=s*rate+(F_theta+m*g_theta-(F_x+m*g_x)*theta)/(m*V) in the disclosed small-angle reduction",
    "R_F=F_measured_theta-F_angle*theta-F_control*delta_actual; R_M=M_measured_axis-M_angle*theta-M_rate*rate-M_control*delta_actual; G=(omega cross J*omega)_axis",
    "rate_ref=-s*(demand_theta+g_theta)/V+s*(demand_x+g_x)*theta_ref/V",
    "(F_angle-F_x+m*demand_x)*theta_ref+F_control*delta_trim+R_F=m*demand_theta; M_angle*theta_ref+M_rate*rate_ref+M_control*delta_trim+R_M-G=0",
    "D=F_angle-F_control*M_angle/M_control-F_x+m*demand_x-s*F_control*M_rate*(demand_x+g_x)/(M_control*V)",
    "theta_ref=[m*demand_theta-R_F+(F_control/M_control)*(M_rate*(-s*(demand_theta+g_theta)/V)+R_M-G)]/D",
    "if hypot(alpha_ref,beta_ref)>incidence_limit: retain angle direction and solve rate_ref=-s*[(F_eff-F_x-m*g_x)*theta_ref+R_F-(F_control/M_control)*(R_M-G)+m*g_theta]/[m*V-s*F_control*M_rate/M_control]",
    "x=[theta,rate,actual_virtual_surface]; A=[[(F_angle-F_x-m*g_x)/(m*V),s,F_control/(m*V)],[M_angle/J_axis,M_rate/J_axis,M_control/J_axis],[0,0,-1/tau]]; B=[0,0,1/tau]^T",
    "cascade K=[(J_axis*s/(T_rate*T_incidence)+M_angle)/M_control,(J_axis/T_rate+M_rate)/M_control,0]; closed_A=A-B*K only in frozen unsaturated residual-constant domain",
    "actuator-aware K assigns eig(A-B*K)=[-1/tau,-1/T_rate,-1/T_incidence] using CURRENT axial/gravity A, subject to its finite controllability determinant",
    "roll_surface=[-J_x*p/T_rate+(J_z-J_y)*q*r-L_damping*p-R_L]/L_control; normalize each signed channel with its actual positive/negative stop",
    "finite TVC: theta_nozzle_demand=clip(G_pitch*delta_virtual_pitch,+/-limit); analogous yaw; actual nozzle angle is native rate-limited and stopped, never an instantaneous substituted force",
    "native TVC derivatives: dFz/ddelta=-T*cos(theta_nozzle)*G_pitch; dFy/ddelta=T*cos(theta_nozzle)*cos(psi_nozzle)*G_yaw; moments use actual nozzle-to-CG arm; measured Fx is total nongravity force counted once",
    "operating-point search uses the directional cascade as a seed/target, advances captured channel-input/lag/rate/stops and physical-fin states, interpolates current finite moment curves, then minimizes sum_axis((M_pred-M_requested)/J_axis)^2 over normalized channel bounds; finite search is not global optimality",
]
DOMAINS = [
    "SI body quantities and the same native PRE epoch; F_x is total nongravity axial force including propulsion, while transverse/moment residual ownership follows native aero/TVC composition.",
    "Small incidence, forward velocity cos(alpha)*cos(beta)>0, speed>=1m/s, positive mass/diagonal inertia, finite derivatives; the nonlinear native plant remains authority outside the frozen reduction.",
    "Current directional trim requires finite nonzero M_control and its own D determinant; aerodynamic F_eff=0 alone does not imply absence of powered directional authority.",
    "Finite stops, incidence projection, rate limits and actuator metadata agreement remain active. An eigenvalue calculation excludes those nonsmooth branches and is not a coupled stability/encounter proof.",
    "Every displayed numerical point is an explicitly supplied captured PRE input. No force, thrust, gravity, CG, inertia or motor age is inferred from a generic 500m/s operating point.",
    "The retained old 500m/s/6000m launch/half/depleted aerodynamic matrices are historical projections only; they omit current axial/gravity directional terms and cannot count as current poles or current acceptance.",
    "TVC combined local control derivatives apply only while the actual native nozzle follows its finite demand without rate/stop limiting; separate nozzle states/rate events invalidate a single unsaturated three-state pole interpretation.",
    "Signed controls have distinct force/moment polarity. Roll-inertial coupling and finite native FCS states are retained; no invented integral state, missing authority fallback or ideal-force execution is introduced.",
]
REFERENCES = [
    dict(title="Jackson, Overview of Missile Flight Control Systems, JHU/APL29(1),2010,pp9-24",
         url="https://www.jhuapl.edu/sites/default/files/2024-09/29-01-Jackson.pdf",
         scope="Physical feedback/actuator/airframe/TVC separation; the displayed directional algebra is the project's disclosed reduction, not an OEM parameter claim."),
    dict(title="SciPy place_poles primary documentation",
         url="https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.place_poles.html",
         scope="Independent executed pole-assignment comparison for actuator-aware current A-BK, not cascade stability admission."),
    dict(title="SciPy cont2discrete primary documentation",
         url="https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.cont2discrete.html",
         scope="Independent bilinear-transform check of the captured finite operating-point virtual lag."),
]


def sha(path):
    return hashlib.sha256(guarded(path).read_bytes()).hexdigest()


def guarded(path, seen=None):
    """Check lexical paths before filesystem access, including parent links.

    Do not call resolve() on an unexamined input: it can traverse an excluded
    ancestor before its canonical spelling is checked. Walk permitted parents
    from the drive down, checking a link target before descending through it.
    """
    path = allowed(allowed(path).absolute())
    seen = set() if seen is None else seen
    for parent in (*reversed(path.parents), path):
        allowed(parent)
        if parent.is_symlink() or (hasattr(parent, "is_junction") and parent.is_junction()):
            key = os.path.normcase(str(parent))
            if key in seen:
                raise ValueError("Cyclic input path link")
            target = allowed(parent.readlink())
            if not target.is_absolute():
                target = allowed(parent.parent/target)
            guarded(target, seen | {key})
    return path


def capture_root_from_environment(root):
    name = "FSIM_DIRECTIONAL_CONTROL_CAPTURE_ROOT"
    if name in os.environ:
        if not os.environ[name].strip():
            raise ValueError(f"{name} must name a complete actual24 capture population")
        return guarded(Path(os.environ[name]))
    return guarded(guarded(root)/"build/ucrt64-release/validation/aam_full_chain")


def display_path(root, path):
    try:
        return guarded(path).relative_to(guarded(root)).as_posix()
    except ValueError:
        return str(guarded(path))


def validate_packet(packet):
    contract = packet.get("axial_force_contract")
    if not contract or not contract.get("required") or not contract.get("matches"):
        raise ValueError("explicit same-PRE native total axial-force contract is required")
    actual = packet["input"]["measured"]["force_body_n"][0]
    if not math.isfinite(actual) or actual != contract.get("native_total_nongravity_fx_n") or actual != contract.get("passed_fx_n"):
        raise ValueError("captured total axial-force ownership mismatch")
    if packet["config"]["version"] != 1 or packet["mechanism"] not in (CASCADE, AWARE, OPERATING):
        raise ValueError("unsupported current directional PRE contract")
    if not math.isfinite(packet["time_s"]) or packet["dt_s"] <= 0:
        raise ValueError("invalid native PRE epoch")


def load_first_pre(root, capture_root, model):
    root, capture_root = guarded(root), guarded(capture_root)
    path = guarded(capture_root/model/"trace.csv")
    csv.field_size_limit(32*1024*1024)
    with path.open(newline="", encoding="utf-8-sig") as stream:
        for row in csv.DictReader(stream):
            if row.get("control_pre_json"):
                packets = json.loads(row["control_pre_json"])
                if packets:
                    break
        else:
            raise ValueError(f"{model}: no actual PRE packet; partial population is not admitted")
    capture = guarded(capture_root/model/"capture.json")
    manifest = json.loads(capture.read_text(encoding="utf-8"))
    if manifest.get("model") != model or not manifest.get("complete"):
        raise ValueError(f"{model}: complete identity-matched capture manifest is required")
    for packet in packets:
        validate_packet(packet)
    metadata = dict(path=display_path(root, path), sha256=sha(path),
                    capture_manifest_path=display_path(root, capture), capture_manifest_sha256=sha(capture),
                    input_epoch_sha256=manifest.get("input_epoch_sha256"),
                    scope="Source-bound supplied capture illustration; never automatically current native acceptance")
    return packets, metadata


def axis_projection(packet, name):
    A = np.asarray(directional_state_matrix(packet["input"], name))
    gains = np.asarray(feedback_gains(packet, name))
    B = np.array([0., 0., 1/packet["input"]["actuator_lag_s"]])
    closed = A-np.outer(B, gains)
    poles = np.linalg.eigvals(closed)
    return dict(trim=directional_trim(packet["input"], name), A=A.tolist(), B=B.tolist(),
                feedback_gains=gains.tolist(), closed_A=closed.tolist(),
                closed_poles_per_s=[[float(v.real), float(v.imag)] for v in poles],
                maximum_real_pole=float(max(v.real for v in poles)),
                scope=("Directional cascade initialization only; not operating-point nonlinear closed-loop poles"
                       if packet["mechanism"] == OPERATING else "Frozen unsaturated residual-constant directional reduction at this captured PRE input; not measured native poles"))


def independent_trim_error(i, name):
    sign, faxis, _, _ = axis_indices(name)
    a, r, m, speed = i[name], residuals(i, name), i["mass_kg"], i["speed_m_s"]
    d, g = i["demand_body_m_s2"], i["gravity_body_m_s2"]
    matrix = [[a["force_angle_n_rad"]-i["measured"]["force_body_n"][0]+m*d[0], 0., a["force_control_n_rad"]],
              [a["moment_angle_nm_rad"], a["moment_rate_nm_s_rad"], a["moment_control_nm_rad"]],
              [-sign*(d[0]+g[0])/speed, 1., 0.]]
    rhs = [m*d[faxis]-r["force"], -r["moment"]+r["gyro"], -sign*(d[faxis]+g[faxis])/speed]
    reference = np.linalg.solve(matrix, rhs)
    fitted = directional_trim(i, name)
    value = np.array([fitted["angle_rad"], fitted["path_rate_rad_s"], fitted["surface_rad"]])
    return float(np.max(np.abs(value-reference)))


def numerical_sensitivity(packet):
    result = []
    for parameter in ("measured_total_axial_force", "control_force_and_moment_effectiveness"):
        cases = {}
        for label, factor in (("minus10percent", .9), ("baseline", 1.), ("plus10percent", 1.1)):
            changed = copy.deepcopy(packet)
            if parameter == "measured_total_axial_force":
                changed["input"]["measured"]["force_body_n"][0] *= factor
            else:
                for name in ("pitch", "yaw"):
                    changed["input"][name]["force_control_n_rad"] *= factor
                    changed["input"][name]["moment_control_nm_rad"] *= factor
            value = {}
            for name in ("pitch", "yaw"):
                try:
                    projection = axis_projection(changed, name)
                    value[name] = {"directional_determinant_n_rad": projection["trim"]["determinant_n_rad"],
                                   "trim_angle_rad": projection["trim"]["angle_rad"],
                                   "trim_surface_rad": projection["trim"]["surface_rad"],
                                   "maximum_real_pole_per_s": projection["maximum_real_pole"]}
                except ValueError as error:
                    value[name] = dict(available=False, refusal=str(error))
            selected = changed
            if packet["mechanism"] == OPERATING:
                selected = copy.deepcopy(changed)
                selected["mechanism"] = CASCADE
            output = commands(selected)
            value["normalized_commands"] = output
            value["minimum_normalized_channel_headroom"] = min(1-abs(output[axis]) for axis in ("aileron", "elevator", "rudder")) if output["available"] else None
            value["output_scope"] = ("Directional cascade SEED only; the captured R77M finite search is not re-optimized by this perturbation"
                                     if packet["mechanism"] == OPERATING else
                                     "Offline selected-law demand with perturbed inputs; not a recorded native output or a new trajectory")
            cases[label] = value
        result.append(dict(parameter=parameter, relative_perturbation=.1,
                           expected_invariance=("Joint Fc/Mc scaling preserves their ratio, so D and recomputed closed-loop poles can be invariant while trim surface/normalized effort changes. Report the actual response; nonzero input does not guarantee nonzero output."
                                                if parameter == "control_force_and_moment_effectiveness" else
                                                "Only captured total Fx is perturbed; no prescribed nonzero output is assumed."),
                           scope="Offline explicit input perturbation with current law recomputed; other captured inputs frozen. Not a card change, burn trajectory, hardware confidence interval or tuning.", **cases))
    return result


def tvc_metadata(root, native_model):
    path = guarded(guarded(root)/f"aircraft/stores/{native_model}/{native_model}.xml")
    values = {element.text.strip().split("/")[-1]: float(element.attrib["value"])
              for element in ET.parse(path).iter("property")
              if element.text and element.text.startswith("fsim/thrust-vector-control/") and "value" in element.attrib}
    return dict(selected=bool(values), native_xml_path=display_path(root, path), native_xml_sha256=sha(path), properties=values,
                state="Actual native nozzle pitch/yaw angles with per-axis rate/stop constraints; capture input already includes composed nozzle force/moment derivatives.",
                force_ownership="Use captured total nongravity Fx once. Do not add native propulsion force again when the combined control derivative is evaluated.")


def build_model(root, old, plant, packets, metadata, capture_label):
    expected = old["mechanism_id"]
    if any(packet["mechanism"] != expected or not packet["dispatched"] for packet in packets):
        raise ValueError(f"{old['model']}: actual PRE mechanism/dispatch mismatch")
    selected = max(packets, key=lambda p: p["input"]["measured"]["force_body_n"][0])
    if not selected["commands"]["available"]:
        raise ValueError(f"{old['model']}: no admitted selected illustration")
    illustration = {name: axis_projection(selected, name) for name in ("pitch", "yaw")}
    trim_errors = [independent_trim_error(packet["input"], name) for packet in packets for name in ("pitch", "yaw")]
    comparison = dict(scope="Independent source-derived math vs actually captured native PRE commands; not a new native execution",
                      actual_packet_count=len(packets), newton_euler_trim_comparisons=len(trim_errors),
                      maximum_independent_trim_absolute_error=max(trim_errors), native_acceptance=False)
    if expected in (CASCADE, AWARE):
        errors = []
        for packet in packets:
            predicted = commands(packet)
            if predicted["available"] != packet["commands"]["available"]:
                raise ValueError(f"{old['model']}: captured command availability mismatch")
            errors.extend(abs(predicted[key]-packet["commands"][key]) for key in ("aileron", "elevator", "rudder"))
        comparison.update(command_comparisons=len(errors), maximum_normalized_command_absolute_error=max(errors))
        illustration["captured_commands"] = selected["commands"]
        illustration["reference_commands"] = commands(selected)
        comparison["math_reference_passed"] = max(trim_errors) < 1e-9 and max(errors) < 2e-9
    else:
        operating = [operating_reference(packet) for packet in packets]
        comparison.update(command_comparisons=0,
                          operating_point_finite_objective_comparisons=len(operating),
                          operating_point_references=operating,
                          math_reference_passed=max(trim_errors) < 1e-9 and all(
                              item["captured_objective_s_4"] <= item["best_neutral_or_corner_objective_s_4"]+1e-9 for item in operating))
        illustration["captured_commands"] = selected["commands"]
        illustration["finite_operating_point_reference"] = operating_reference(selected)
    if not comparison["math_reference_passed"]:
        raise ValueError(f"{old['model']}: actual command or independent mathematical reference mismatch")
    native = plant["native_model"]
    if native.startswith("jsbsim:"):
        native = native[7:]
    tvc = tvc_metadata(root, native)
    history = old.get("historical_aerodynamic_projection")
    if history is None:
        history = dict(scope="Retired aerodynamic-only fixed500m/s6000m projection. Not current directional equations or current acceptance.",
                       equations=old["theory"]["equations"], model_specific_outputs=old["theory"]["model_specific_outputs"],
                       sensitivity=old["theory"]["sensitivity"])
    row = copy.deepcopy(old)
    card_path = guarded(guarded(root)/f"assets/stores/{old['model']}.json")
    card = json.loads(card_path.read_text(encoding="utf-8"))
    descriptor = card["component_models"]["components"]["autopilot"]
    if descriptor["implementation_id"] != old["implementation_id"]:
        raise ValueError(f"{old['model']}: current card changed selected autopilot identity")
    resolved = {role:card["parameters"][ref] for role, ref in descriptor["parameter_refs"].items()}
    if any(resolved[role]["value"] != old["parameter_records"][role]["value"] for role in resolved):
        raise ValueError(f"{old['model']}: current controller parameters differ from the supplied historical projection; choose a fresh capture and rebind deliberately")
    row["historical_identity"] = old.get("historical_identity", {
        key:old[key] for key in ("card_sha256", "parameter_sha256", "plant_sha256", "plant_ref")})
    row.update(card_sha256=sha(card_path), descriptor=descriptor, parameter_records=resolved,
               parameter_sha256=hashlib.sha256(json.dumps(resolved, sort_keys=True, separators=(",", ":")).encode()).hexdigest(),
               identity_scope="Card/parameter hashes bind current analysis inputs only; historical actual producer identity is not inferred from them.",
               parameter_hash_scope="SHA256 of sorted compact JSON of current resolved role records, UTF-8",
               plant_ref_scope="Retained historical generated-plant projection; current actual co-epoch directional coefficients are in theory.model_specific_outputs.selected_pre.input")
    row["historical_aerodynamic_projection"] = history
    meta = dict(metadata, capture_label=capture_label)
    state = selected["input"]
    row["theory"] = dict(
        equations=EQUATIONS,
        primary_sources=REFERENCES,
        state_and_io=dict(plant=["alpha", "beta", "body p/q/r", "native mass/CG/inertia", "actual virtual surface positions", "channel input/lag/rate/stop states", "each physical fin angle", "native TVC pitch/yaw angles when selected"],
                          controller="Cascade and actuator-aware references are algebraic same-PRE decisions; operating-point execution additionally predicts actual finite FCS/fin states. No hidden integral.",
                          inputs="Actual captured demand/gravity/own state/current-CG derivatives/total axial force and correctly owned measured loads.",
                          outputs="Normalized native FCS channel demands only; no force is executed by this tool."),
        limits=DOMAINS,
        delta=[f"Selected {expected}; actual native model jsbsim:{native}; {len(selected['stops']['fin_rates_rad_s'])} controlled physical-fin rate entries.",
               f"Captured PRE t={selected['time_s']:.17g}s: mass={state['mass_kg']:.12g}kg, speed={state['speed_m_s']:.12g}m/s, Jyy={state['inertia_kg_m2'][1]:.12g}kgm2, total Fx={state['measured']['force_body_n'][0]:.12g}N.",
               f"Actual pitch D={illustration['pitch']['trim']['determinant_n_rad']:.12g}N/rad vs aerodynamic-only F_eff={illustration['pitch']['trim']['effective_aerodynamic_force_n_rad']:.12g}; yaw is independently signed/recomputed.",
               "Finite TVC selected with actual XML gain/arm/rate/stop metadata." if tvc["selected"] else "No TVC selected; axial thrust/drag still enters the directional determinant and dynamics."],
        model_specific_outputs=dict(mechanism=expected, capture=meta,
                                    selected_pre={k:selected[k] for k in ("sequence", "time_s", "dt_s", "mechanism", "input", "config", "stops", "axial_force_contract")},
                                    current_directional_illustration=illustration, tvc=tvc,
                                    original_three_mass_states_scope="Retained in historical_aerodynamic_projection only. Current native PRE illustration uses its complete actual co-epoch input; no mixed250/500m/s force substitution."),
        sensitivity=numerical_sensitivity(selected),
        failure_modes=["Aero-only F_eff precheck can reject real powered authority or misstate trim; use actual directional D.",
                       "Singular directional trim/pole-assignment system refuses; finite Fx cannot manufacture control moment.",
                       "Incidence/rate/stop saturation invalidates the displayed unsaturated poles; recompute attainable curvature instead of retaining a rejected request.",
                       "Wrong axial-force ownership, wrong gravity sign, double-counted propulsion, stale measured actual fins, and mismatched PRE epochs invalidate the reference.",
                       "Bounded operating-point cost may remain nonzero with finite search or absent directional authority; it must not be called exact pole assignment/global optimum."],
        uncertainty_policy="Existing game-plant/card inputs are unchanged. Declared +/-10% input perturbations report actual numerical responses or invariances; they are not new equipment parameters, accuracy bounds or calibration.")
    row.update(theory_projection_complete=True, passed=False, current_native_acceptance=False,
               method=dict(passed=False, reference=comparison, native_executed_by_generator=False,
                           case="Current-law math reference against supplied actual PRE inputs; coupled native acceptance is separately owned"))
    row["sources"] = []
    for item in old["sources"]:
        if item.get("scope") == "Explicit supplied-capture illustration, not current producer-source proof":
            continue
        updated = dict(item)
        if updated["path"].startswith(("src/", "assets/")):
            updated.update(sha256=sha(root/updated["path"]), scope="Current analysis-equation source, not historical capture producer proof")
        row["sources"].append(updated)
    row["sources"].append(dict(path=metadata["path"], sha256=metadata["sha256"], scope="Explicit supplied-capture illustration, not current producer-source proof"))
    row["shared_equivalence"] = dict(id=expected, mechanism_id=expected,
                                     scope="Shared current state equations only; each model retains actual co-epoch input, signed derivatives, mass/inertia, Fx, finite actuator/TVC composition, independent comparison and sensitivity. No representative model admits another.",
                                     limits=DOMAINS)
    return row


def render_markdown(models, metadata):
    lines = ["# Current directional autopilot theory for the 24 game plants", "",
             "This package derives the selected 22 load-feedback cascade laws, PL-10 actuator-aware law and R-77M bounded operating-point law from current software. Inputs are existing game-plant PRE captures. No card, force, gain, actuator limit or outcome is changed. Numerical references are distinct from current native acceptance.", "",
             "## Equations and physical ownership", ""]
    lines.extend(f"- `{equation}`" for equation in EQUATIONS)
    lines.extend(["", "The sign convention is x forward, y right, z down. Body acceleration obeys Newton/Euler equations; directional acceleration is the projection normal to velocity, so total axial force and axial gravity enter incidence kinematics. A positive thrust with zero aerodynamic derivatives can retain `D=-thrust`, provided its finite nozzle has actual moment authority. The old aerodynamic-only `F_eff` and `A00=F_angle/(mV)` are not the current measured-axial law, including in a coast with nonzero drag.", "",
                  "## State and domain", ""])
    lines.extend(f"- {domain}" for domain in DOMAINS)
    lines.extend(["", "Controller output is a normalized native channel request. Native FCS owns channel lag, signed stops, mixing, each physical fin's rate/position/malfunction, and nozzle rate/stops. Cascade has no invented integral state; the three-state A includes an unsaturated continuous virtual actuator only. The operating-point mechanism additionally retains previous channel INPUT, lag, actual position and every physical fin. It evaluates actual piecewise native moment curves with the correct body/current-CG transform.", "",
                  "## Executed mathematical references", "",
                  "The maintained Python tests independently solve the three coupled force/moment/incidence equations with NumPy, differentiate body-velocity Newton equations, compare the applicable 23 selected laws to captured native commands, compare PL-10 gains with SciPy `place_poles`, and compare the R-77M virtual lag to SciPy's bilinear transform. R-77M's captured command is independently evaluated against neutral/eight-corner feasible controls with the captured finite FCS and fin curves; this is not a claim of global optimizer superiority. Powered zero-aero authority and true singular refusal are separate controls.", "",
                  f"Capture root: `{metadata['capture_root']}`. Label: {metadata['capture_label']}. Every JSON row records its actual input epoch and selected PRE input. No generator invocation upgrades it to current native acceptance. All 24 `passed` and `current_native_acceptance` fields remain false in this theory projection.", "",
                  "## Actual model differences", "",
                  "| Model | Selected law | PRE time s | Mass kg | Fx N | Pitch F_eff N/rad | Pitch D N/rad | Current directional local max pole 1/s | TVC |",
                  "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |"])
    for row in models:
        out = row["theory"]["model_specific_outputs"]
        selected, projection = out["selected_pre"], out["current_directional_illustration"]["pitch"]
        state, trim = selected["input"], projection["trim"]
        law = {CASCADE:"cascade", AWARE:"actuator-aware", OPERATING:"operating-point (seed only)"}[row["mechanism_id"]]
        lines.append(f"| {row['model']} | {law} | {selected['time_s']:.6f} | {state['mass_kg']:.6g} | {state['measured']['force_body_n'][0]:.7g} | {trim['effective_aerodynamic_force_n_rad']:.7g} | {trim['determinant_n_rad']:.7g} | {projection['maximum_real_pole']:.7g} | {'finite' if out['tvc']['selected'] else 'none'} |")
    lines.extend(["", "Each pole is a frozen unsaturated mathematical illustration at that PRE input, not measured coupled damping. The R-77M entry is only its directional cascade initialization; it does not describe the bounded nonlinear search's closed-loop poles. Old 500m/s/6000m three-mass-state numbers remain explicitly historical in JSON and are not silently relabeled.", ""])
    for row in models:
        theory = row["theory"]
        lines.extend([f"### {row['model']}", "", *theory["delta"], "",
                      "The full JSON row contains independently signed pitch/yaw D, reference incidence/path rate/trim surface, axial/gravity A, feedback gains/poles, captured commands and comparison errors. Its state and domain are the governing method above, instantiated with this model's actual mass/inertia, derivative polarity, actuator/fin metadata and finite TVC selection.", "",
                      "Input perturbation responses (offline, no authored changes; zero or invariant responses are retained):", ""])
        for item in theory["sensitivity"]:
            minus, plus = item["minus10percent"]["pitch"], item["plus10percent"]["pitch"]
            if "directional_determinant_n_rad" in minus and "directional_determinant_n_rad" in plus:
                output_minus = item['minus10percent']['normalized_commands']
                output_plus = item['plus10percent']['normalized_commands']
                lines.append(f"- {item['parameter']} +/-10%: pitch D `{minus['directional_determinant_n_rad']:.8g} -> {plus['directional_determinant_n_rad']:.8g} N/rad`; local maximum real pole `{minus['maximum_real_pole_per_s']:.8g} -> {plus['maximum_real_pole_per_s']:.8g} 1/s`; trim surface `{minus['trim_surface_rad']:.8g} -> {plus['trim_surface_rad']:.8g} rad`; normalized elevator `{output_minus['elevator']:.8g} -> {output_plus['elevator']:.8g}`. Other captured inputs are frozen.")
                lines.append(f"  {item['expected_invariance']} {item['minus10percent']['output_scope']}.")
            else:
                lines.append(f"- {item['parameter']}: the explicit perturbation reaches a documented singular refusal; JSON retains each case.")
        lines.extend(["", "Failure/domain: directional singularity, incidence/rate/stop limiting, stale or double-counted load ownership and unsynchronized actual controls invalidate a precise frozen-pole interpretation. These are software/game-plant method limits, not new hardware claims.", ""])
    lines.extend(["## Primary references", ""])
    lines.extend(f"- [{item['title']}]({item['url']}): {item['scope']}" for item in REFERENCES)
    lines.extend(["", "## Reproduction", "",
                  "These are captured-data integration math tests. First produce the complete actual24 population through the normal UCRT64 Release full-chain native gate. A clean checkout with no capture is expected to fail explicitly, not skip or use synthetic aliases. The tests use `FSIM_DIRECTIONAL_CONTROL_CAPTURE_ROOT`; if absent, their only fallback is the normal preset output `build/ucrt64-release/validation/aam_full_chain`.", "", "```powershell",
                  "$env:FSIM_DIRECTIONAL_CONTROL_CAPTURE_ROOT = Join-Path (Get-Location) 'build/ucrt64-release/validation/aam_full_chain'",
                  "python -B tools/validation/weapon_quality/test_directional_control_theory.py -v",
                  "python -B tools/validation/weapon_quality/directional_control_theory.py --capture-root $env:FSIM_DIRECTIONAL_CONTROL_CAPTURE_ROOT --capture-label 'Explicit fresh native PRE source epoch; acceptance remains separately owned' --output build/validation/directional-control-theory.json",
                  "# Add --write only in the authorized documentation window; it replaces only the 24 autopilot object spans.", "```", "",
                  "To reproduce this document's exact supplied capture epoch, explicitly select it (an ignored capture directory is not a clean-checkout prerequisite). The capture label above determines whether these are historical illustrations:", "", "```powershell",
                  f"$env:FSIM_DIRECTIONAL_CONTROL_CAPTURE_ROOT = Join-Path (Get-Location) '{metadata['capture_root']}'",
                  "python -B tools/validation/weapon_quality/test_directional_control_theory.py -v",
                  f"python -B tools/validation/weapon_quality/directional_control_theory.py --capture-root $env:FSIM_DIRECTIONAL_CONTROL_CAPTURE_ROOT --capture-label '{metadata['capture_label']}' --output build/validation/directional-control-theory-captured-epoch.json",
                  "```", "",
                  "The generator always requires an explicit capture root and never assigns native acceptance. Full24 encounters, the other component families and overall SIM-M03 acceptance remain separate gates.", ""])
    return "\n".join(lines)


def generate(root, capture_root, capture_label):
    root, capture_root = guarded(root), guarded(capture_root)
    source = guarded(root/MATRIX_PATH)
    text = source.read_text(encoding="utf-8")
    ledger = json.loads(text)
    cells = [row for row in ledger["cells"] if row["component"] == "autopilot"]
    if len(cells) != 24 or len({row["model"] for row in cells}) != 24:
        raise ValueError("complete unique24 autopilot population is required")
    models = []
    for old in cells:
        packets, metadata = load_first_pre(root, capture_root, old["model"])
        plant = next(model for model in ledger["models"] if model["model"] == old["model"])
        models.append(build_model(root, old, plant, packets, metadata, capture_label))
    metadata = dict(script="tools/validation/weapon_quality/directional_control_theory.py", script_sha256=sha(Path(__file__)),
                    main_head_at_analysis=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip(),
                    capture_root=display_path(root, capture_root), capture_label=capture_label,
                    capture_scope="Explicit supplied source-epoch illustration; no automatic native acceptance",
                    producer_scope="Current source hashes describe the analysis equations, not proof that historical capture producers used those bytes")
    artifact = dict(schema="fsim.sim-m03.directional-autopilot-theory.v1", component="autopilot", passed=False,
                    theory_projection_complete=True, numeric_reference_passed=True, native_coupling_passed=False,
                    model_count=24, generation=metadata, sources=REFERENCES, models=models,
                    disposition="Current-law derivation plus executed mathematical comparisons; full24/native acceptance is not admitted")
    replacements = {row["model"]:row for row in models}
    rewritten = replace_autopilot_cells(text, replacements)
    check = json.loads(rewritten)
    if any(a != b for a, b in zip(ledger["cells"], check["cells"]) if a["component"] != "autopilot"):
        raise AssertionError("a non-autopilot family changed")
    for key in ledger:
        if key != "cells" and ledger[key] != check[key]:
            raise AssertionError("top-level/model projection changed")
    return artifact, rewritten, render_markdown(models, metadata)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).absolute().parents[3])
    parser.add_argument("--capture-root", required=True, type=Path)
    parser.add_argument("--capture-label", default="Explicit supplied source-epoch illustration; not current native acceptance")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--write", action="store_true", help="Write only the authorized autopilot documents/24cell object spans")
    args = parser.parse_args()
    root, capture_root, output = guarded(args.root), guarded(args.capture_root), guarded(args.output)
    artifact, matrix, markdown = generate(root, capture_root, args.capture_label)
    guarded(output.parent).mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(artifact, indent=2)+"\n", encoding="utf-8")
    if args.write:
        guarded(root/FAMILY_PATH).write_text(json.dumps(artifact, indent=2)+"\n", encoding="utf-8")
        guarded(root/MARKDOWN_PATH).write_text(markdown, encoding="utf-8")
        # Preserve every byte outside replacement object spans, including CRLF.
        target = guarded(root/MATRIX_PATH)
        before = target.read_bytes()
        newline = "\r\n" if b"\r\n" in before else "\n"
        target.write_bytes(matrix.replace("\n", newline).encode("utf-8"))
    comparisons = [row["method"]["reference"] for row in artifact["models"]]
    print(json.dumps(dict(models=24, math_reference_passed=True, native_acceptance=False,
                         captured_pre_packets=sum(row["actual_packet_count"] for row in comparisons),
                         command_comparisons=sum(row["command_comparisons"] for row in comparisons),
                         trim_reference_comparisons=sum(row["newton_euler_trim_comparisons"] for row in comparisons))))


if __name__ == "__main__":
    main()
