"""Offline inviscid static experiment on authored game geometry.

This is a comparison runner, never a production coefficient-table generator.
Static loads cannot establish damping, crossflow, viscous drag or equipment
calibration. Optional Gmsh/OpenVSP imports occur only in explicitly run workers.
"""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import shutil
import subprocess
import sys
import time
import types
from aero_history import parse_history
from aero_mesh import mesh, orient_closed_surface
from aero_panels import read_topology
from aero_vsp_frame import paired_conditions, roll_component_frames, rotate_vector
from aero_vsp_case import assess_case
from aero_vsp_adb import assess_adb
from aero_hangar import assess_initial


ZERO_ALPHA_TOLERANCES = {"cn": 0.02, "cm": 0.02}
SU2_LOAD_WINDOW = 100
SU2_LOAD_SPAN_LIMITS = {"absolute": 1e-4, "relative": 1e-4}
SU2_LOAD_DRIFT_LIMITS = {"absolute": 1e-5, "relative": 1e-5}
SU2_CAUCHY_MAXIMUM = 1e-10


class NotConverged(ValueError):
    """Valid numerical output which may be continued from its own checkpoint."""


def finite(value, label):
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(label + " must be finite") from error
    if isinstance(value, bool) or not math.isfinite(number):
        raise ValueError(label + " must be finite")
    return number


def body_coefficients(cd, cl, cm, alpha_deg):
    cd, cl, cm, alpha_deg = (finite(v, k) for v, k in
                              ((cd, "CD"), (cl, "CL"), (cm, "CMY"), (alpha_deg, "alpha")))
    angle = math.radians(alpha_deg)
    return {"ca": finite(cd * math.cos(angle) - cl * math.sin(angle), "body axial coefficient"),
            "cn": finite(cl * math.cos(angle) + cd * math.sin(angle), "body normal coefficient"), "cm": cm}


def parse_su2(path, alpha_deg, residual_limit=-8.0):
    with Path(path).open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream, skipinitialspace=True)
        if not reader.fieldnames or len({key.strip() for key in reader.fieldnames}) != len(reader.fieldnames):
            raise ValueError("missing or duplicate SU2 history columns")
        rows = list(reader)
    if not rows:
        raise ValueError("empty SU2 history")
    values, previous = [], -1
    for raw in rows:
        if None in raw or any(value is None for value in raw.values()):
            raise ValueError("SU2 history row does not match its header")
        row = {key.strip(): finite(value, "SU2 " + key.strip()) for key, value in raw.items()}
        try:
            iteration = row["Inner_Iter"]
            if iteration < 0 or not iteration.is_integer() or iteration <= previous:
                raise ValueError("SU2 iteration must increase strictly")
            previous = iteration
            rho = row["rms[Rho]"]
            coefficients = body_coefficients(row["CD"], row["CL"], row.get("CMy", row.get("CMY")), alpha_deg)
        except KeyError as error:
            raise ValueError("missing SU2 iteration, coefficient or convergence column") from error
        values.append(coefficients)
    # The complete required schema is checked before any continuable classification:
    # malformed output is terminal and never authorizes checkpoint continuation.
    try:
        cauchy = {key: row[key] for key in ("Cauchy[CD]", "Cauchy[CL]", "Cauchy[CMy]")}
    except KeyError as error:
        raise ValueError("missing SU2 native Cauchy convergence columns") from error
    if rho > finite(residual_limit, "residual limit"):
        raise NotConverged("SU2 density residual did not reach the frozen convergence limit: %g" % rho)
    if any(value < 0 or value > SU2_CAUCHY_MAXIMUM for value in cauchy.values()):
        raise NotConverged("SU2 native Cauchy targets were not met: " + json.dumps(cauchy, sort_keys=True))
    if len(values) < 2 * SU2_LOAD_WINDOW:
        raise NotConverged("SU2 load stability requires two complete 100-sample windows")
    stability = {}
    for key in ("ca", "cn", "cm"):
        older = [v[key] for v in values[-2*SU2_LOAD_WINDOW:-SU2_LOAD_WINDOW]]
        recent = [v[key] for v in values[-SU2_LOAD_WINDOW:]]
        old_mean, mean = math.fsum(older)/SU2_LOAD_WINDOW, math.fsum(recent)/SU2_LOAD_WINDOW
        span, drift = max(recent)-min(recent), abs(mean-old_mean)
        span_limit = SU2_LOAD_SPAN_LIMITS["absolute"] + SU2_LOAD_SPAN_LIMITS["relative"] * abs(mean)
        drift_limit = SU2_LOAD_DRIFT_LIMITS["absolute"] + SU2_LOAD_DRIFT_LIMITS["relative"] * abs(mean)
        stability[key] = {"span": span, "span_limit": span_limit, "mean_drift": drift,
                          "mean_drift_limit": drift_limit, "stable": span <= span_limit and drift <= drift_limit}
    if not all(v["stable"] for v in stability.values()):
        raise NotConverged("SU2 force/moment histories remain unsettled: " + json.dumps(stability, sort_keys=True))
    return {"status": "converged", "residual_log10": rho, "coefficients": coefficients,
            "wind_cd": row["CD"], "iterations": len(rows), "alpha_deg": alpha_deg,
            "load_window_samples": SU2_LOAD_WINDOW, "load_stability": stability, "native_cauchy": cauchy}


def parse_vspaero(path):
    lines = Path(path).read_text(encoding="utf-8").splitlines()
    header = None
    result = []
    for line in lines:
        fields = line.split()
        if "Mach" in fields and "AoA" in fields:
            if len(set(fields)) != len(fields):
                raise ValueError("duplicate VSPAERO polar columns")
            header = fields
            continue
        if not header or not fields:
            continue
        if len(fields) != len(header):
            raise ValueError("VSPAERO polar row does not match its header")
        row = dict(zip(header, fields))
        try:
            alpha = finite(row["AoA"], "VSPAERO alpha")
            mach = finite(row["Mach"], "VSPAERO Mach")
            if any(key in row for key in ("CLi", "CDi", "CMiy")):
                # VSPAERO 7.2: the i terms are near-surface inviscid loads; o terms
                # are viscous estimates. Euler comparison excludes o terms.
                values = body_coefficients(row["CDi"], row["CLi"], row["CMiy"], alpha)
                cd = finite(row["CDi"], "VSPAERO inviscid drag")
                basis = "near_surface_inviscid"
            else:
                values = body_coefficients(row["CDtot"], row["CL"], row["CMm"], alpha)
                cd = finite(row["CDtot"], "fixture drag")
                basis = "declared_static_fixture"
        except KeyError as error:
            raise ValueError("missing VSPAERO coefficient column") from error
        result.append({"status": "computed", "convergence_assessed": False, "basis": basis,
                       "mach": mach, "alpha_deg": alpha, "wind_cd": cd, "coefficients": values})
    if not result:
        raise ValueError("empty or unsupported VSPAERO polar")
    return result


def static_comparison(reference, candidate):
    if set(reference) != {"ca", "cn", "cm"} or set(candidate) != set(reference):
        raise ValueError("comparison requires exactly the three static coefficients")
    result = {}
    for key in sorted(reference):
        expected, actual = finite(reference[key], key), finite(candidate[key], key)
        error = finite(actual - expected, key + " residual")
        relative = finite(abs(error) / abs(expected), key + " relative residual") if expected else None
        result[key] = {"absolute": error, "relative": relative}
    return result


def physical_gates(rows):
    """Frozen Mach/angle/symmetry checks and nonnegative wind-axis drag.

    Negative body-axis axial force alone is not rejected: lift's coordinate
    conversion can produce it. Both normal force and pitch moment must vanish
    at zero alpha for this frozen symmetric design, at any axial moment
    reference. These are numerical rejection tolerances, not calibration.
    Residual convergence is checked separately.
    """
    conditions = len(rows) == 2 and sorted(finite(row["alpha_deg"], "alpha") for row in rows) == [0.0, 2.0]
    drag = bool(rows) and all(finite(row["wind_cd"], "wind drag") >= 0 for row in rows)
    zero = [row for row in rows if row["alpha_deg"] == 0.0]
    symmetry = {key: len(zero) == 1 and abs(finite(zero[0]["coefficients"][key], key + " coefficient")) <= limit
                for key, limit in ZERO_ALPHA_TOLERANCES.items()}
    mach = all(abs(finite(row["mach"], "Mach") - 0.6) <= 1e-10 for row in rows if "mach" in row)
    return {"conditions_match": conditions and mach, "nonnegative_wind_drag": drag,
            "zero_alpha_symmetry": symmetry["cn"], "zero_alpha_pitch_symmetry": symmetry["cm"]}


def vsp_convergence(path, wake_limit=5):
    """Check each actual case/wake against BOTH native printed GMRES targets.

    Outer convergence can stop a case before its configured wake limit. Case
    banners identify boundaries even when both cases stop at wake one. The
    nonlinear history remains a separate mandatory acceptance gate.
    """
    log = Path(path).read_text(encoding="utf-8", errors="replace")
    kutta = [int(value) for value in re.findall(r"NumberOfKuttaNodes after trimming:\s*(\d+)", log)]
    wakes = [int(value) for value in re.findall(r"NumberOfWakeLoops:\s*(\d+)", log)]
    number = r"([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)"
    pattern = (r"Wake Iter:\s*(\d+)\s*/\s*(\d+).*?Red:\s*" + number + r"\s*/\s*" + number +
               r".*?Max:\s*" + number + r"\s*/\s*" + number)
    banner = r"Solving\.\.\.\s*Mach:\s*" + number + r"\s*\.\.\.\s*Alpha:\s*" + number + r"\s*\.\.\.\s*Beta:\s*" + number
    starts = list(re.finditer(banner, log))
    groups, valid = [], len(starts) == 2 and isinstance(wake_limit, int) and wake_limit > 0
    for index, start in enumerate(starts):
        condition = tuple(float(start[i]) for i in (1, 2, 3))
        valid = valid and index < 2 and all(abs(a-b) <= 5e-7 for a, b in zip(condition, (.6, 2*index, 0)))
        segment = log[start.end():starts[index+1].start() if index+1 < len(starts) else len(log)]
        current, previous = {}, 0
        for line in segment.splitlines():
            if "Wake Iter:" not in line:
                continue
            match = re.search(pattern, line)
            if not match:
                valid = False
                continue
            wake, total = int(match[1]), int(match[2])
            if wake not in (previous, previous+1) or wake <= 0:
                valid = False
            current[wake] = {"wake": wake, "total": total, "reduction": float(match[3]), "reduction_target": float(match[4]),
                             "maximum": float(match[5]), "maximum_target": float(match[6])}
            previous = wake
        groups.append(current)
    complete = bool(valid and groups and all(group and set(group) == set(range(1, max(group)+1)) and
                   all(row["total"] == wake_limit and row["wake"] <= wake_limit and row["reduction"] <= row["reduction_target"] and
                       row["maximum"] <= row["maximum_target"] for row in group.values()) for group in groups))
    return {"lifting_wakes_present": bool(kutta) and min(kutta) > 0 and bool(wakes) and min(wakes) > 0,
            "gmres_targets_met_every_wake": complete, "kutta_nodes": kutta, "wake_loops": wakes,
            "final_wake_targets": [group[max(group)] for group in groups if group]}


def cart3d_without_tags(source, destination):
    """Keep all coordinates/connectivity, omitting optional component tags.

    OpenVSP 3.54's packaged importer crashes on the tested tagged mesh; its
    untagged path imports the same triangles. Reject malformed input first.
    """
    lines = Path(source).read_text(encoding="ascii").splitlines()
    try:
        nodes, triangles = map(int, lines[0].split())
        if nodes < 4 or triangles < 4 or len(lines) not in (1 + nodes + triangles, 1 + nodes + 2 * triangles):
            raise ValueError("invalid Cart3D counts")
        for line in lines[1:1 + nodes]:
            values = line.split()
            if len(values) != 3:
                raise ValueError("invalid Cart3D coordinate shape")
            for value in values:
                finite(value, "Cart3D coordinate")
        for line in lines[1 + nodes:1 + nodes + triangles]:
            values = [int(value) for value in line.split()]
            if len(values) != 3 or len(set(values)) != 3 or any(v < 1 or v > nodes for v in values):
                raise ValueError("invalid Cart3D triangle indices")
    except (IndexError, TypeError) as error:
        raise ValueError("invalid Cart3D document") from error
    Path(destination).write_text("\n".join(lines[:1 + nodes + triangles]) + "\n", encoding="ascii")


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def bounded(command, folder, log, timeout=120):
    env = os.environ.copy()
    for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
        env[name] = "2"
    for name in ("TMP", "TEMP", "TORCH_EXTENSIONS_DIR"):
        env[name] = "D:/ENV/tmp"
    started = time.monotonic()
    with Path(log).open("w", encoding="utf-8") as stream:
        options = {"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP | subprocess.CREATE_NO_WINDOW} if os.name == "nt" else {"start_new_session": True}
        process = subprocess.Popen(command, cwd=folder, stdout=stream, stderr=subprocess.STDOUT, env=env, **options)
        try:
            process.wait(timeout=timeout)
            result = {"exit_code": process.returncode, "status": "executed" if process.returncode == 0 else "failed"}
        except subprocess.TimeoutExpired:
            if os.name == "nt":
                subprocess.run(["taskkill", "/PID", str(process.pid), "/T", "/F"], stdout=stream,
                               stderr=subprocess.STDOUT, timeout=10, check=False)
            else:
                os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=10)
            result = {"exit_code": None, "status": "timeout"}
    return dict(result, command=[str(v) for v in command], seconds=time.monotonic() - started, log=str(log))


def su2_config(reference, alpha, iterations=2200, first_order=False, restart=False):
    return "\n".join(("SOLVER= EULER", "MATH_PROBLEM= DIRECT", "RESTART_SOL= " + ("YES" if restart else "NO"), "SYSTEM_MEASUREMENTS= SI",
        "MACH_NUMBER= 0.6", "AOA= %.17g" % alpha, "SIDESLIP_ANGLE= 0", "FREESTREAM_PRESSURE= 101325",
        "FREESTREAM_TEMPERATURE= 288.15", "REF_DIMENSIONALIZATION= DIMENSIONAL",
        "REF_AREA= %.17g" % reference["area_m2"], "REF_LENGTH= %.17g" % reference["length_m"],
        "REF_ORIGIN_MOMENT_X= %.17g" % reference["moment_x_m"], "REF_ORIGIN_MOMENT_Y= 0", "REF_ORIGIN_MOMENT_Z= 0",
        "MARKER_EULER= ( missile )", "MARKER_FAR= ( farfield )", "MARKER_MONITORING= ( missile )",
        "NUM_METHOD_GRAD= WEIGHTED_LEAST_SQUARES", "CFL_NUMBER= 1.0", "CFL_ADAPT= NO",
        "ITER= %d" % iterations, "LINEAR_SOLVER= FGMRES", "LINEAR_SOLVER_PREC= ILU", "LINEAR_SOLVER_ERROR= 1E-6",
        "LINEAR_SOLVER_ITER= 10", "CONV_NUM_METHOD_FLOW= ROE", "MUSCL_FLOW= " + ("NO" if first_order else "YES"), "SLOPE_LIMITER_FLOW= VENKATAKRISHNAN",
        "TIME_DISCRE_FLOW= EULER_IMPLICIT", "CONV_FIELD= ( RMS_DENSITY, DRAG, LIFT, MOMENT_Y )", "CONV_RESIDUAL_MINVAL= -8",
        "CONV_CAUCHY_ELEMS= 100", "CONV_CAUCHY_EPS= 1E-10", "CONV_STARTITER= 500",
        "MESH_FILENAME= mesh.su2", "MESH_FORMAT= SU2", "TABULAR_FORMAT= CSV",
        "CONV_FILENAME= history", "SOLUTION_FILENAME= restart.dat", "RESTART_FILENAME= restart-out",
        "OUTPUT_FILES= ( RESTART )", "HISTORY_OUTPUT= ( ITER, RMS_RES, AERO_COEFF, CAUCHY )",
        "SCREEN_OUTPUT= ( INNER_ITER, RMS_DENSITY, LIFT, DRAG )", "OUTPUT_WRT_FREQ= 1000", ""))


def su2_case(args, output, reference, alpha):
    """Continue valid unconverged fields without changing mesh or numerics.

    Each bounded pass has separate input/output checkpoint names and retained
    raw history/configuration. A timeout, failed process or malformed history
    never qualifies for continuation. Only a completed final pass may pass.
    """
    case = {"status": "unavailable", "passes": [], "seconds": 0.0}
    checkpoint = None
    for index in range(args.su2_passes):
        folder = output / ("su2-alpha-%g" % alpha) / ("pass-%d" % index)
        folder.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(output / "mesh.su2", folder / "mesh.su2")
        incoming, outgoing = folder / "restart.dat", folder / "restart-out.dat"
        incoming.unlink(missing_ok=True)
        outgoing.unlink(missing_ok=True)
        if checkpoint is not None:
            shutil.copyfile(checkpoint, incoming)
        cfg = folder / ("alpha-%g.cfg" % alpha)
        cfg.write_text(su2_config(reference, alpha, args.iterations, restart=index > 0), encoding="ascii")
        history = folder / "history.csv"
        history.unlink(missing_ok=True)
        run = bounded([str(args.su2), "-t", "2", str(cfg)], folder, folder / "solver.log", args.solver_timeout)
        run.update(pass_index=index, config_sha256=sha256(cfg), mesh_sha256=sha256(folder / "mesh.su2"),
                   input_restart_sha256=sha256(incoming) if incoming.exists() else None)
        case["seconds"] += run.get("seconds", 0)
        case["passes"].append(run)
        case["exit_code"] = run["exit_code"]
        if outgoing.exists():
            run["output_restart_sha256"] = sha256(outgoing)
        if history.exists():
            run["history_sha256"] = sha256(history)
            shutil.copyfile(history, output / ("history-alpha-%g.csv" % alpha))
        try:
            if run["exit_code"] != 0:
                raise ValueError("bounded SU2 pass did not complete successfully")
            result = parse_su2(history, alpha)
        except NotConverged as error:
            run.update(status="not_converged", reason=str(error))
            case["reason"] = str(error)
            if not outgoing.exists() or outgoing.stat().st_size == 0:
                case["reason"] += "; no fresh native output checkpoint"
                break
            checkpoint = outgoing
            continue
        except (ValueError, OSError) as error:
            run.update(status="unavailable", reason=str(error))
            case["reason"] = str(error)
            break
        else:
            run["status"] = "converged"
            # This runner's authored config fixes MACH_NUMBER=0.6 for every pass.
            result["mach"] = 0.6
            case.update(status="converged", result=result)
            case.pop("reason", None)
            break
    return case


def parametric_vsp(vsp, design, folder):
    """Build the authored body stations and each original lifting fin in VSP.

    Section values and fin LE/chords match the source. VSP's interpolated
    fuselage surface differs between stations; sampling records that difference.
    Actual Hangar foil coordinates are imported instead of a similar NACA foil.
    """
    import numpy as np
    root = Path(__file__).resolve().parents[3]
    sys.path.insert(0, str(root / "tools/models/hangar"))
    from hangar.geometry.aircraft import Aircraft
    aircraft = Aircraft.load(str(design))
    body = next(b for b in aircraft.bodies if b.kind == "fuselage")
    if float(body.x[0]) != 0.0:
        raise ValueError("representative VSP body must begin at the authored x=0 origin")
    sections = np.unique(np.concatenate((body.x, np.linspace(body.x[0], body.x[-1], 35))))
    widths, tops, bottoms, ys, _ = body.section(sections)
    nose_width, nose_height = float(widths[0]), float(tops[0]-bottoms[0])
    if (nose_width == 0.0) != (nose_height == 0.0):
        raise ValueError("VSP nose cannot have only one collapsed dimension")
    point_nose = nose_width == 0.0 and nose_height == 0.0
    length = float(body.x[-1] - body.x[0])
    fuselage = vsp.AddGeom("FUSELAGE", "")
    frames = [{"id": fuselage, "parent_id": "", "origin_xyz": [0, 0, 0], "rotation_xyz": [0, 0, 0]}]
    vsp.SetGeomName(fuselage, "Authored AIM120C body")
    vsp.SetParmVal(fuselage, "Length", "Design", length)
    vsp.SetParmVal(fuselage, "Tess_W", "Shape", 17)
    surface = vsp.GetXSecSurf(fuselage, 0)
    while vsp.GetNumXSec(surface) > 2:
        vsp.CutXSec(fuselage, 1)
    for _ in sections[1:-1]:
        vsp.InsertXSec(fuselage, 0, vsp.XS_ELLIPSE)
    for index in reversed(range(len(sections))):
        vsp.ChangeXSecShape(surface, index, vsp.XS_POINT if index == 0 and point_nose else vsp.XS_ELLIPSE)
        section = vsp.GetXSec(surface, index)
        for key, value in (("XLocPercent", float((sections[index] - body.x[0]) / length)),
                           ("YLocPercent", float(ys[index] / length)),
                           ("ZLocPercent", float((tops[index] + bottoms[index]) / (2 * length))), ("SectTess_U", 2)):
            vsp.SetParmVal(vsp.GetXSecParm(section, key), value)
        if index != 0 or not point_nose:
            vsp.SetXSecWidthHeight(section, max(2e-5, float(widths[index])), max(2e-5, float(tops[index] - bottoms[index])))
        vsp.SetXSecContinuity(section, 0)
        vsp.SetXSecTanStrengths(section, vsp.XSEC_BOTH_SIDES, 0, 0, 0, 0)
    for parm in vsp.GetGeomParmIDs(fuselage):
        if vsp.GetParmName(parm) in ("CapUMinOption", "CapUMaxOption"):
            vsp.SetParmVal(parm, vsp.FLAT_END_CAP)
    fins = []
    for index, authored in enumerate(aircraft.surfaces):
        root_le, root_chord, _, foil = authored.station(0)
        tip_le, tip_chord, _, _ = authored.station(1)
        xx = 0.5 * (1 - np.cos(np.linspace(0, math.pi, 201)))
        foil_path = folder / ("foil-%d.dat" % index)
        with foil_path.open("w", encoding="ascii") as stream:
            stream.write("Hangar authored %s\n" % authored.name)
            for x, y in zip(xx[::-1], foil.upper(xx)[::-1]):
                stream.write("%.17g %.17g\n" % (x, y))
            for x, y in zip(xx[1:], foil.lower(xx)[1:]):
                stream.write("%.17g %.17g\n" % (x, y))
        for mirrored in ((False, True) if authored.mirror else (False,)):
            le, tip = root_le.copy(), tip_le.copy()
            if mirrored:
                le[1], tip[1] = -le[1], -tip[1]
            delta = tip - le
            span = float(np.hypot(delta[1], delta[2]))
            rotation = math.degrees(math.atan2(float(delta[2]), float(delta[1])))
            wing = vsp.AddGeom("WING", "")
            frames.append({"id": wing, "parent_id": "", "origin_xyz": le.tolist(), "rotation_xyz": [rotation, 0, 0]})
            vsp.SetGeomName(wing, authored.name + (" mirrored" if mirrored else ""))
            vsp.SetParmVal(wing, "Sym_Planar_Flag", "Sym", 0)
            vsp.SetDriverGroup(wing, 1, vsp.SPAN_WSECT_DRIVER, vsp.ROOTC_WSECT_DRIVER, vsp.TIPC_WSECT_DRIVER)
            for key, value in (("Span", span), ("Root_Chord", root_chord), ("Tip_Chord", tip_chord),
                               ("Sweep_Location", 0), ("Sweep", math.degrees(math.atan2(float(delta[0]), span))),
                               ("Dihedral", 0), ("Twist", 0), ("SectTess_U", 6)):
                vsp.SetParmVal(wing, key, "XSec_1", float(value))
            for key, value in (("X_Rel_Location", le[0]), ("Y_Rel_Location", le[1]), ("Z_Rel_Location", le[2]),
                               ("X_Rel_Rotation", rotation)):
                vsp.SetParmVal(wing, key, "XForm", float(value))
            vsp.SetParmVal(wing, "Tess_W", "Shape", 17)
            wing_surface = vsp.GetXSecSurf(wing, 0)
            for station in (0, 1):
                vsp.ChangeXSecShape(wing_surface, station, vsp.XS_FILE_AIRFOIL)
                vsp.ReadFileAirfoil(vsp.GetXSec(wing_surface, station), str(foil_path))
            fins.append({"name": authored.name, "mirrored": mirrored, "root_le_m": le.tolist(), "tip_le_m": tip.tolist(),
                         "root_chord_m": float(root_chord), "tip_chord_m": float(tip_chord), "rotation_x_deg": rotation,
                         "foil_sha256": sha256(foil_path)})
    vsp.Update()
    differences = []
    for u in np.linspace(0, 1, 401):
        for w in (0, .25, .5, .75):
            p = vsp.CompPnt01(fuselage, 0, float(u), w)
            if p.x() <= float(body.x[0]) + 1e-8 or p.x() >= float(body.x[-1]) - 1e-8:
                continue  # flat end caps have radii from zero to the section's radius
            bw, bt, bb, by, _ = body.section(np.array([p.x()]))
            radial = math.hypot(p.y() - float(by[0]), p.z() - float((bt[0] + bb[0]) / 2))
            differences.append(abs(radial - float((bw[0] + bt[0] - bb[0]) / 4)))
    save(folder / "vsp-geometry-record.json", {"version": vsp.GetVSPVersion(), "design_sha256": sha256(design),
         "body_section_count": len(sections), "body_surface_max_sampled_radius_difference_m": max(differences),
         "nose_radius_m": 0.0 if point_nose else (nose_width+nose_height)/4,
         "nose_representation": "source point" if point_nose else "authored finite section with flat cap",
         "fins": fins, "foil_basis": "authored Hangar coordinates, 201 cosine points per side",
         "limits": "Source-matched sections with zero-strength C0 tangent controls; measured interpolation difference remains explicit. Elliptical sections retain authored width/height while CFD uses their mean radius; mesh convergence still required"})
    return frames


def assess_wake(folder, basename):
    # Optional NumPy is needed only for explicit native aerodynamic campaigns.
    from aero_wake_containment import assess
    return assess(folder, basename)


def assess_single_vsp_case(folder, pair, reference, *, basename="missile", log_name="solver.log"):
    """Freshly assess raw loads and the actual single-case native geometry."""
    folder = Path(folder)
    config = folder / (basename + ".vspaero")
    try:
        setup = config.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as error:
        raise ValueError("missing or unreadable native single-case config") from error
    values = re.findall(r"(?m)^[ \t]*NumWakeNodes[ \t]*=[ \t]*([^\r\n]*)", setup)
    if len(values) != 1 or not re.fullmatch(r"[+-]?\d+", values[0].strip()):
        raise ValueError("one exact native NumWakeNodes integer is required")
    wake_nodes = int(values[0])
    cg = rotate_vector([reference["moment_x_m"], 0, 0], pair["roll_deg"])
    row = assess_case(folder / (basename + ".polar"), folder / (basename + ".history"), folder / log_name, pair,
                      bref=reference["length_m"], cref=reference["length_m"],
                      reference_area=reference["area_m2"], native_cg=cg)
    actual = assess_adb(folder / (basename + ".adb"), folder / (basename + ".vspgeom"), pair,
                        bref=reference["length_m"], cref=reference["length_m"],
                        reference_area=reference["area_m2"], native_cg=cg, wake_nodes=wake_nodes)
    if actual.get("passed") is not True:
        raise ValueError("VSPAERO actual solved geometry failed: " + json.dumps(actual.get("checks", {}), sort_keys=True))
    containment = assess_wake(folder, basename)
    if containment.get("passed") is not True:
        raise ValueError("VSPAERO active wake containment failed: " + json.dumps(containment.get("failures", []), sort_keys=True))
    row.update(actual_geometry=actual, wake_containment=containment, native_config_sha256=sha256(config))
    return row


def paired_vsp_flows(vsp, folder, reference, frame_roll):
    """Execute two native single cases; retain each raw file in its own path.

    SetVSP3FileName selects native output names. VSPAEROSweep creates its own
    setup file and RedirectFile retains its native process output; inputs are
    never synthesized here. The already validated closed geometry is copied
    verbatim because ComputeSolver consumes, rather than rebuilds, that mesh.
    """
    geometry, key = folder / "missile.vspgeom", folder / "missile.vkey"
    digest = sha256(geometry)
    report = {"status": "running", "model_accepted": False, "geometry_sha256": digest, "result": []}
    target = folder / "vsp-frame-cases.json"
    save(target, report)
    cg = rotate_vector([reference["moment_x_m"], 0, 0], frame_roll)
    try:
        for pair in paired_conditions(frame_roll):
            case = folder / ("vsp-frame-alpha-%g" % pair["physical"]["alpha_deg"])
            case.mkdir(exist_ok=True)
            # Remove only known prior outputs before the native execution.
            for name in ("missile.polar", "missile.history", "missile.vspaero", "missile.adb", "solver.log", "case-report.json"):
                (case / name).unlink(missing_ok=True)
            shutil.copyfile(geometry, case / "missile.vspgeom")
            shutil.copyfile(key, case / "missile.vkey")
            if sha256(case / "missile.vspgeom") != digest:
                raise ValueError("paired case geometry differs from the validated closed mesh")
            vsp.SetVSP3FileName(str(case / "missile.vsp3"))
            analysis = "VSPAEROSweep"
            vsp.SetAnalysisInputDefaults(analysis)
            for name, value in (("NCPU", 2), ("GeomSet", vsp.SET_ALL), ("ThinGeomSet", vsp.SET_NONE),
                                ("AlphaNpts", 1), ("BetaNpts", 1), ("MachNpts", 1), ("WakeNumIter", 5)):
                vsp.SetIntAnalysisInput(analysis, name, [value])
            native = pair["native"]
            for name, value in (("Sref", reference["area_m2"]), ("bref", reference["length_m"]),
                                ("cref", reference["length_m"]), ("Xcg", cg[0]), ("Ycg", cg[1]), ("Zcg", cg[2]),
                                ("AlphaStart", native["alpha_deg"]), ("AlphaEnd", native["alpha_deg"]),
                                ("BetaStart", native["beta_deg"]), ("BetaEnd", native["beta_deg"]),
                                ("MachStart", native["mach"]), ("MachEnd", native["mach"]),
                                ("Vinf", 204.176), ("NonLinearConvergenceFactor", 1.0)):
                vsp.SetDoubleAnalysisInput(analysis, name, [value])
            vsp.SetStringAnalysisInput(analysis, "RedirectFile", [str(case / "solver.log")])
            result_id = vsp.ExecAnalysis(analysis)
            errors = []
            manager = vsp.ErrorMgrSingleton.getInstance()
            while manager.GetNumTotalErrors():
                errors.append(manager.PopLastError().GetErrorString())
            if errors or not result_id:
                raise ValueError("OpenVSP paired case failed: " + "; ".join(errors or ["missing native result"]))
            if sha256(case / "missile.vspgeom") != digest or sha256(geometry) != digest:
                raise ValueError("native paired case changed the validated closed geometry")
            row = assess_single_vsp_case(case, pair, reference)
            row.update(geometry_sha256=digest, native_result_id=result_id)
            save(case / "case-report.json", row)
            report["result"].append(row)
            save(target, report)
    except Exception as error:
        report.update(status="failed", reason="%s: %s" % (type(error).__name__, error))
        save(target, report)
        raise
    report["status"] = "computed"
    save(target, report)
    return report


def select_vsp_solver(vsp, executable, folder):
    """Select and record one explicit native solver through the supported API."""
    target = Path(folder) / "vsp-solver-record.json"
    target.unlink(missing_ok=True)
    binary = Path(executable).resolve()
    expected_name = "vspaero.exe" if os.name == "nt" else "vspaero"
    actual_name = binary.name.casefold() if os.name == "nt" else binary.name
    if actual_name != expected_name or not binary.is_file():
        raise ValueError("explicit panel solver must be the existing native platform command " + expected_name)
    directory = binary.parent
    if not vsp.SetVSPAEROPath(str(directory)) or Path(vsp.GetVSPAEROPath()).resolve() != directory:
        raise ValueError("OpenVSP did not select the requested native panel solver directory")
    record = {"path": str(binary), "sha256": sha256(binary), "bytes": binary.stat().st_size,
              "api_directory": str(directory), "scope": "Explicit solver selection; execution and convergence are separate gates"}
    save(target, record)
    return record


def vsp_worker(openvsp, folder, reference, design=None, geometry_only=False, frame_roll=0, panels_only=False,
               vspaero=None):
    if frame_roll and design is None:
        raise ValueError("rigid frame mode requires source-created component frames")
    config = types.ModuleType("openvsp_config")
    config.LOAD_GRAPHICS = config.LOAD_FACADE = config.LOAD_MULTI_FACADE = False
    config._IGNORE_IMPORTS, config.FACADE_PORT = True, -1
    sys.modules["openvsp_config"] = config
    sys.path.insert(0, str(Path(openvsp) / "python/openvsp"))
    import openvsp as vsp
    vsp.VSPCheckSetup()
    vsp.ClearVSPModel()
    if vspaero is not None:
        select_vsp_solver(vsp, vspaero, folder)
    print("OpenVSP ready:", vsp.GetVSPVersion(), flush=True)
    if design is None:
        cart3d_without_tags(folder / "missile.tri", folder / "missile-vspaero.tri")
        vsp.ImportFile(str(folder / "missile-vspaero.tri"), vsp.IMPORT_CART3D_TRI, "")
        print("Imported Cart3D mesh", flush=True)
    else:
        frames = parametric_vsp(vsp, design, folder)
        if frame_roll:
            transformed = roll_component_frames(frames, frame_roll)
            for component in transformed:
                for axis, value in zip(("X", "Y", "Z"), component["origin_xyz"]):
                    vsp.SetParmVal(component["id"], axis+"_Rel_Location", "XForm", value)
                for axis, value in zip(("X", "Y", "Z"), component["rotation_xyz"]):
                    vsp.SetParmVal(component["id"], axis+"_Rel_Rotation", "XForm", value)
            vsp.Update()
            save(folder / "vsp-frame-record.json", {"roll_deg": frame_roll, "source_components": frames,
                 "native_components": transformed, "paired_conditions": paired_conditions(frame_roll),
                 "source_cg_m": [reference["moment_x_m"], 0, 0],
                 "native_cg_m": rotate_vector([reference["moment_x_m"], 0, 0], frame_roll),
                 "limits": "Rigid coordinate change only; native winds and all six loads require matching transformation before any physical comparison"})
        print("Constructed authored fuselage and lifting fins", flush=True)
    vsp.WriteVSPFile(str(folder / "missile.vsp3"), vsp.SET_ALL)
    error_manager = vsp.ErrorMgrSingleton.getInstance()
    if geometry_only:
        errors = []
        while error_manager.GetNumTotalErrors():
            errors.append(error_manager.PopLastError().GetErrorString())
        if errors:
            raise ValueError("OpenVSP geometry errors: " + "; ".join(errors))
        return
    analysis = "VSPAEROComputeGeometry"
    vsp.SetAnalysisInputDefaults(analysis)
    vsp.SetIntAnalysisInput(analysis, "GeomSet", [vsp.SET_ALL])
    vsp.SetIntAnalysisInput(analysis, "ThinGeomSet", [vsp.SET_NONE])
    (folder / "missile.vspgeom").unlink(missing_ok=True)
    (folder / "missile.vkey").unlink(missing_ok=True)
    vsp.ExecAnalysis(analysis)
    panels = read_topology(folder / "missile.vspgeom")
    save(folder / "vsp-panel-record.json", panels)
    if not panels["closed_oriented"]:
        raise ValueError("VSPAERO solid panels are not one closed outward oriented shell: " +
                         json.dumps(panels["edge_use_counts"], sort_keys=True))
    if panels_only:
        errors = []
        while error_manager.GetNumTotalErrors():
            errors.append(error_manager.PopLastError().GetErrorString())
        if errors:
            raise ValueError("OpenVSP panel geometry errors: " + "; ".join(errors))
        return
    errors = []
    while error_manager.GetNumTotalErrors():
        errors.append(error_manager.PopLastError().GetErrorString())
    if errors:
        raise ValueError("OpenVSP panel geometry errors: " + "; ".join(errors))
    paired_vsp_flows(vsp, folder, reference, frame_roll)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--design", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--gmsh", type=Path)
    parser.add_argument("--openvsp", type=Path)
    parser.add_argument("--su2", type=Path)
    parser.add_argument("--vspaero", type=Path, help="Explicit native vspaero executable selected through OpenVSP")
    parser.add_argument("--initial-model", type=Path, help="Explicit generated Hangar XML for a separate full-static diagnostic comparison")
    parser.add_argument("--mesh-size", type=float, default=0.1)
    parser.add_argument("--iterations", type=int, default=2200)
    parser.add_argument("--su2-passes", type=int, default=4)
    parser.add_argument("--solver-timeout", type=float, default=300)
    parser.add_argument("--vsp-worker", action="store_true")
    parser.add_argument("--vsp-parametric-worker", action="store_true")
    parser.add_argument("--geometry-only", action="store_true")
    parser.add_argument("--panel-geometry-only", action="store_true")
    parser.add_argument("--frame-roll-deg", type=float, default=0)
    parser.add_argument("--mesh-worker", action="store_true")
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if args.vsp_worker or args.vsp_parametric_worker:
        vsp_worker(args.openvsp, output, json.loads((output / "mesh-record.json").read_text())["reference"],
                   args.design.resolve() if args.vsp_parametric_worker else None, args.geometry_only,
                   finite(args.frame_roll_deg, "frame roll"), args.panel_geometry_only, args.vspaero)
        return 0
    if args.mesh_worker:
        record = mesh(args.design.resolve(), output, args.gmsh, finite(args.mesh_size, "mesh size"))
        save(output / "mesh-record.json", record)
        return 0
    report = {"schema": "fsim.static-aero-experiment.v1", "equipment_calibration": False,
              "model_accepted": False, "zero_alpha_absolute_tolerances": dict(ZERO_ALPHA_TOLERANCES),
              "missing_channels": ["cmq", "cnr", "clp", "crossflow", "viscous drag"], "runs": {}}
    report["status"] = "running"
    save(output / "report.json", report)
    try:
        args.frame_roll_deg = finite(args.frame_roll_deg, "frame roll")
        if args.iterations <= 0 or args.su2_passes <= 0 or finite(args.solver_timeout, "solver timeout") <= 0:
            raise ValueError("iteration/pass budgets and solver timeout must be positive")
        report["su2_convergence_policy"] = {"density_log10_maximum": -8, "window_samples": SU2_LOAD_WINDOW,
            "native_cauchy_maximum": SU2_CAUCHY_MAXIMUM, "maximum_passes": args.su2_passes,
            "iterations_per_pass": args.iterations, "process_timeout_s": args.solver_timeout,
            "span_limits": dict(SU2_LOAD_SPAN_LIMITS), "adjacent_window_mean_drift_limits": dict(SU2_LOAD_DRIFT_LIMITS)}
        return campaign(args, output, report)
    except Exception as error:
        report.update(status="failed", model_accepted=False, reason="%s: %s" % (type(error).__name__, error))
        save(output / "report.json", report)
        return 1


def campaign(args, output, report):
    (output / "mesh-record.json").unlink(missing_ok=True)
    mesh_run = bounded([sys.executable, "-B", str(Path(__file__).resolve()), "--design", str(args.design.resolve()),
                        "--output", str(output), "--gmsh", str(args.gmsh), "--mesh-size", str(args.mesh_size),
                        "--mesh-worker"], output, output / "mesh.log")
    report["mesh_run"] = mesh_run
    try:
        if mesh_run["exit_code"] != 0:
            raise ValueError("bounded geometry/meshing worker did not complete successfully")
        record = json.loads((output / "mesh-record.json").read_text(encoding="utf-8"))
        report["mesh"] = record
    except Exception as error:
        report["status"], report["reason"] = "mesh_failed", str(error)
        save(output / "report.json", report)
        return 1
    for alpha in (0.0, 2.0):
        run = su2_case(args, output, record["reference"], alpha)
        report["runs"]["su2-alpha-%g" % alpha] = run
        save(output / "report.json", report)
    polar = output / "missile.polar"
    polar.unlink(missing_ok=True)
    nonlinear_history = output / "missile.history"
    nonlinear_history.unlink(missing_ok=True)
    panel_path = output / "missile.vspgeom"
    panel_path.unlink(missing_ok=True)
    (output / "vsp-panel-record.json").unlink(missing_ok=True)
    frame_report = output / "vsp-frame-cases.json"
    frame_report.unlink(missing_ok=True)
    solver_record = output / "vsp-solver-record.json"
    solver_record.unlink(missing_ok=True)
    command = [sys.executable, "-B", str(Path(__file__).resolve()), "--output", str(output),
                   "--openvsp", str(args.openvsp), "--design", str(args.design.resolve()),
                   "--vsp-parametric-worker"]
    if args.frame_roll_deg:
        command.extend(("--frame-roll-deg", str(args.frame_roll_deg)))
    if args.vspaero is not None:
        command.extend(("--vspaero", str(args.vspaero.resolve())))
    run = bounded(command, output, output / "vspaero.log", args.solver_timeout)
    if panel_path.exists():
        run["panels"] = read_topology(panel_path)
    try:
        if run["exit_code"] != 0:
            raise ValueError("OpenVSP/VSPAERO did not exit successfully")
        if args.vspaero is not None:
            selection = json.loads(solver_record.read_text(encoding="utf-8"))
            if selection["path"] != str(args.vspaero.resolve()) or selection["sha256"] != sha256(args.vspaero):
                raise ValueError("native panel solver selection no longer matches the requested executable")
            run["solver_selection"] = selection
        cases = json.loads(frame_report.read_text(encoding="utf-8"))
        rows = cases["result"]
        if (cases["status"] != "computed" or len(rows) != 2 or
                [row["paired_condition"] for row in rows] != paired_conditions(args.frame_roll_deg) or
                any(row["geometry_sha256"] != sha256(panel_path) for row in rows)):
            raise ValueError("incomplete, stale or unpaired native frame-case report")
        # Every normal case is single-condition, including zero roll. Re-read
        # raw files and reject changes after the worker assessment.
        run["result"] = []
        for pair, saved in zip(paired_conditions(args.frame_roll_deg), rows):
            case = output / ("vsp-frame-alpha-%g" % pair["physical"]["alpha_deg"])
            if sha256(case / "missile.vspgeom") != sha256(panel_path):
                raise ValueError("current paired case geometry differs from validated closed mesh")
            current = assess_single_vsp_case(case, pair, record["reference"])
            if (current["native_config_sha256"] != saved.get("native_config_sha256") or
                    current["raw_files"] != saved.get("raw_files") or
                    current["actual_geometry"]["raw_files"] != saved.get("actual_geometry", {}).get("raw_files") or
                    current["wake_containment"] != saved.get("wake_containment")):
                raise ValueError("current raw native case files changed after worker assessment")
            run["result"].append(current)
        rows = run["result"]
        run["convergence"] = {"lifting_wakes_present": all(row["linear"]["kutta_nodes"] > 0 and row["linear"]["wake_loops"] > 0 for row in rows),
                              "gmres_targets_met_every_wake": all(row["gmres_targets_met_every_wake"] for row in rows)}
        run["nonlinear"] = {"nonlinear_converged": all(row["nonlinear_converged"] for row in rows),
                            "cases": [row["history"] for row in rows]}
        run["frame_cases"] = cases
    except (ValueError, OSError) as error:
        run["status"], run["reason"] = "unavailable", str(error)
    report["runs"]["vspaero"] = run
    report["gates"] = {"su2_density_converged": all("result" in report["runs"]["su2-alpha-%g" % a] for a in (0, 2))}
    report["gates"]["su2_force_moment_stable"] = report["gates"]["su2_density_converged"]
    su2_rows = [report["runs"]["su2-alpha-%g" % a]["result"] for a in (0, 2)
                if "result" in report["runs"]["su2-alpha-%g" % a]]
    report["gates"].update({"su2_" + key: value for key, value in physical_gates(su2_rows).items()})
    report["gates"].update({"vsp_" + key: value for key, value in physical_gates(run.get("result", [])).items()})
    report["gates"].update({key: bool(run.get("convergence", {}).get(key)) for key in
                            ("lifting_wakes_present", "gmres_targets_met_every_wake")})
    report["gates"]["nonlinear_converged"] = bool(run.get("nonlinear", {}).get("nonlinear_converged"))
    report["gates"]["vsp_panels_closed"] = bool(run.get("panels", {}).get("closed_oriented"))
    report["gates"]["vsp_actual_geometry"] = (len(run.get("result", [])) == 2 and
        all(row.get("actual_geometry", {}).get("passed") is True for row in run["result"]) and run.get("status") != "unavailable")
    if args.initial_model is not None:
        # Empirical full-static differences do not modify inviscid agreement or
        # physical gates. Explicitly requested missing/invalid comparisons fail.
        diagnostic = {"status": "unavailable", "model_accepted": False}
        try:
            if len(su2_rows) != 2 or len(run.get("result", [])) != 2 or run.get("status") == "unavailable":
                raise ValueError("initial comparison requires both complete freshly assessed solver pairs")
            before = sha256(args.initial_model)
            comparisons = {label: assess_initial(args.initial_model, rows, record["reference"])
                           for label, rows in (("su2", su2_rows), ("vspaero", run["result"]))}
            if (sha256(args.initial_model) != before or
                    any(row["raw_file"]["sha256"] != before for row in comparisons.values())):
                raise ValueError("initial model changed during the comparison")
            diagnostic.update(status="computed", solvers=comparisons)
        except (OSError, ValueError) as error:
            diagnostic["reason"] = str(error)
        report["initial_model_comparison"] = diagnostic
        report["gates"]["initial_model_comparison_complete"] = diagnostic["status"] == "computed"
    report["gates"]["solver_static_residuals"] = False
    if (len(su2_rows) == 2 and len(run.get("result", [])) == 2 and
            report["gates"]["su2_conditions_match"] and report["gates"]["vsp_conditions_match"]):
        vsp_two = next(row for row in run["result"] if row["alpha_deg"] == 2)
        su2_two = next(row for row in su2_rows if row["alpha_deg"] == 2)
        report["comparison_at_two_degrees"] = static_comparison(su2_two["coefficients"], vsp_two["coefficients"])
        report["gates"]["solver_static_residuals"] = all(row["relative"] is not None and row["relative"] <= .25
                                                         for row in report["comparison_at_two_degrees"].values())
    report["status"] = "static_comparison_passed" if all(report["gates"].values()) else "failed_static_comparison"
    report["model_accepted"] = False
    report["mesh_sensitivity"] = "Not established by this single-grid run"
    save(output / "report.json", report)
    return 0 if report["status"] == "static_comparison_passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
