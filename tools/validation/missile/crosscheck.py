"""Compare generated native axial flight with RocketPy using explicit game inputs.

Run through the configured SDK launcher, for example::

    fsim python tools/validation/missile/crosscheck.py --model aim120c \
        --output build/architecture-reviews/sim-m02/flight.json \
        --native-bin build/ucrt64-headless/bin

The temporary model adds declared atmosphere overrides and, when selected, the
reference planet.
Production assets are read and checked, never edited. Passing means numerical
agreement in this axial case, not admission of real missile performance.
"""
import argparse
import ctypes
import hashlib
import importlib.metadata
import json
import math
import os
from pathlib import Path
import re
import sys
import tempfile

from reference import PLANET_MU_M3_S2, PLANET_RADIUS_M, axial_drag, compare, phase_curve, radial_gravity, verify_environment, verify_generated


ROOT = Path(__file__).resolve().parents[3]
LIMITS = {"mass_kg": 0.10, "speed_ms": 2.0, "height_m": 10.0}
FT, LB, LBF, SLUGFT2 = 0.3048, 0.45359237, 4.4482216152605, 1.3558179483314
ATMOSPHERES = {"vacuum": {"pressure_pa": 1e-9, "temperature_k": 236.15, "density_kg_m3": 0.0},
               "constant_air": {"pressure_pa": 35599.8, "temperature_k": 236.15,
                                "density_kg_m3": 35599.8 / (287.05287 * 236.15)}}


def permitted(path):
    path = Path(path)
    if any(p.casefold() == ".codex" for p in path.parts):
        raise ValueError("excluded path")
    if any(p.is_symlink() or getattr(p, "is_junction", lambda: False)() for p in (path, *path.parents)):
        raise ValueError("redirected reference path")
    return path.resolve()


def digest(path):
    path = permitted(path)
    return {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def runtime(native_bin):
    """Record loaded DLLs and require the explicitly expected build's bytes."""
    import fsim
    if sys.platform != "win32":
        raise ValueError("this UCRT64 reference requires Windows runtime provenance")
    kernel = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel.GetModuleHandleW.argtypes = [ctypes.c_wchar_p]
    kernel.GetModuleHandleW.restype = ctypes.c_void_p
    kernel.GetModuleFileNameW.argtypes = [ctypes.c_void_p, ctypes.c_wchar_p, ctypes.c_uint]
    kernel.GetModuleFileNameW.restype = ctypes.c_uint
    records = []
    for name in ("libfsim.dll", "libJSBSim.dll"):
        handle = kernel.GetModuleHandleW(name)
        buffer = ctypes.create_unicode_buffer(32768)
        if not handle or not kernel.GetModuleFileNameW(handle, buffer, len(buffer)):
            raise ValueError("native runtime was not loaded: " + name)
        actual, expected = digest(buffer.value), digest(native_bin / name)
        if actual["sha256"] != expected["sha256"]:
            raise ValueError("loaded native runtime differs from expected build: " + name)
        records.append(dict(actual, expected_path=expected["path"]))
    import inspect
    from rocketpy import Environment, Flight, Function, Rocket, SolidMotor
    numerical_sources = {inspect.getfile(cls) for cls in (Environment, Flight, Function, Rocket, SolidMotor)}
    numerical_sources.update((__file__, str(Path(__file__).with_name("reference.py"))))
    numerical_sources.update(str(ROOT / "tools/models/hangar/hangar" / (name + ".py")) for name in
                             ("storeaero", "storebodies", "motorinputs", "storemodels", "storedrag", "storetables"))
    return {"python_sdk": digest(fsim.__file__), "python_executable": digest(sys.executable), "libraries": records,
            "rocketpy_version": importlib.metadata.version("rocketpy"),
            "numpy_version": importlib.metadata.version("numpy"), "scipy_version": importlib.metadata.version("scipy"),
            "numerical_sources": [digest(p) for p in sorted(numerical_sources)],
            "provenance_scope": "explicit listed numerical sources and loaded native DLLs; not a full dependency closure"}


def inputs(model):
    if not re.fullmatch(r"[a-z0-9][a-z0-9-]*", model):
        raise ValueError("model must be a local design identifier")
    sys.path.insert(0, str(ROOT / "tools/models/hangar"))
    from hangar import storeaero, storebodies
    import tomllib
    source = ROOT / "aircraft" / model / (model + ".toml")
    design = tomllib.loads(permitted(source).read_text(encoding="utf-8"))
    card_id = design["aircraft"]["card"]
    if not re.fullmatch(r"[a-z0-9][a-z0-9-]*", card_id):
        raise ValueError("invalid card identity")
    card_path = ROOT / "assets/stores" / (card_id + ".json")
    card = json.loads(permitted(card_path).read_text(encoding="utf-8"))
    if card["model"] != "jsbsim:" + model or card["class"] != "aam":
        raise ValueError("reference requires an exact authored AAM model/card pair")
    generated = storebodies.files(card)
    records = verify_generated(generated, ROOT / "aircraft/stores" / model)
    geo = storeaero.geometry(card)
    coefficients = storeaero.coefficients(geo, card_id)
    mass, cg = design["mass"]["empty"], design["mass"]["empty_cg"][0]
    motor = storebodies.motor_of(design, mass, cg, geo["diameter"] / 2)
    if not motor:
        raise ValueError("a simulated solid motor is required")
    # Read the emitted axial table: its rounding is part of the generated model.
    import xml.etree.ElementTree as ET
    xml = ET.fromstring(generated[model + ".xml"])
    drag = axial_drag(generated[model + ".xml"])
    structure = xml.find("mass_balance")
    dry_mass = float(structure.findtext("emptywt")) * LB
    dry_cg = float(structure.find("location[@name='CG']/x").text) * 0.0254
    dry_inertia = [float(structure.findtext(a)) * SLUGFT2 for a in ("iyy", "izz", "ixx")]
    return {"model": model, "card": card_id, "design": design, "motor": motor,
            "mass": mass, "cg": cg, "dry_mass": dry_mass, "dry_cg": dry_cg, "dry_inertia": dry_inertia,
            "radius": math.sqrt(coefficients["S"] / math.pi), "drag": drag, "generated": generated,
            "records": records + [digest(source), digest(card_path)]}


def native(data, dt, atmosphere, times, folder, *, spherical=False):
    import fsim
    # FGModelFunctions loads these properties before initialization. The public
    # setter deliberately cannot create new arbitrary JSBSim properties.
    props = {"pressure": atmosphere["pressure_pa"] / (LBF / FT**2),
             "temperature": atmosphere["temperature_k"] * 1.8,
             "density": atmosphere["density_kg_m3"] / (14.5939029372 / FT**3)}
    injected = "\n".join('        <property value="%.17g">atmosphere/override/%s</property>' % (v, k)
                          for k, v in props.items())
    model_dir = folder / data["model"]
    for name, text in data["generated"].items():
        target = model_dir / name
        target.parent.mkdir(parents=True, exist_ok=True)
        if name == data["model"] + ".xml":
            if text.count("    <aerodynamics>") != 1:
                raise ValueError("cannot instrument the generated atmosphere properties")
            text = text.replace("    <aerodynamics>", "    <aerodynamics>\n" + injected)
            if spherical:
                planet = ('    <planet name="fsim-axial-reference">'
                          '<semimajor_axis unit="FT">%.17g</semimajor_axis>'
                          '<semiminor_axis unit="FT">%.17g</semiminor_axis>'
                          '<rotation_rate unit="RAD/SEC">0</rotation_rate>'
                          '<GM unit="FT3/SEC2">%.17g</GM><J2>0</J2></planet>\n') % (
                              PLANET_RADIUS_M / FT, PLANET_RADIUS_M / FT, PLANET_MU_M3_S2 / FT**3)
                if text.count("    <metrics>") != 1:
                    raise ValueError("cannot instrument the reference planet")
                text = text.replace("    <metrics>", planet + "    <metrics>")
        target.write_text(text, encoding="utf-8", newline="\n")
    previous = os.environ.get("FSIM_AIRCRAFT_PATH")
    os.environ["FSIM_AIRCRAFT_PATH"] = str(folder)
    world = None
    try:
        world = fsim.World("missile-reference", publish=False, workers=1, pin_workers=False,
                           terrain=False, dt=dt, frame_skip=1, seed=3, jsbsim_root=str(ROOT / "third_party/jsbsim"))
        vehicle = world.create_vehicle("reference", "jsbsim:" + data["model"], latitude_deg=0,
                                       longitude_deg=0, altitude_msl_m=8000, airspeed_ms=250,
                                       pitch_deg=90, heading_deg=0, roll_deg=0)
        vehicle.command_actuator(throttle=1)
        initial_environment = {key: vehicle.get_property(path) for key, path in
                               {"gravity_ms2": "accelerations/gravity-ft_sec2",
                                "sound_ms": "atmosphere/a-fps"}.items()}
        initial_environment = {k: v * FT for k, v in initial_environment.items()}
        initial_environment.update(pressure_pa=vehicle.get_property("atmosphere/P-psf") * LBF / FT**2,
                                   density_kg_m3=vehicle.get_property("atmosphere/rho-slugs_ft3") * 14.5939029372 / FT**3,
                                   temperature_k=vehicle.get_property("atmosphere/T-R") / 1.8)
        verify_environment(initial_environment, atmosphere, spherical=spherical)
        rows = []
        for time in times:
            steps = round(time / dt) - round(world.time / dt)
            if steps:
                world.step(steps)
            state = vehicle.state
            if state.diverged or abs(world.time - time) > 1e-8:
                raise ValueError("native flight diverged or missed a sample: expected %.9g, got %.9g, step %.9g, diverged %s" %
                                 (time, world.time, world.step_seconds, state.diverged))
            rows.append([time, vehicle.mass_kg, -state.velocity_ned_ms[2], state.altitude_msl_m])
        vacuum_impulse = vehicle.get_property("propulsion/engine/total-vac-impulse") * LBF
        fixture_records = [digest(model_dir / p) for p in data["generated"]]
        return rows, vacuum_impulse, initial_environment, fixture_records
    finally:
        if world is not None:
            world.close()
        if previous is None:
            os.environ.pop("FSIM_AIRCRAFT_PATH", None)
        else:
            os.environ["FSIM_AIRCRAFT_PATH"] = previous


def rocketpy(data, atmosphere, times, initial_environment, *, max_step, spherical=False):
    from rocketpy import Environment, Flight, Function, Rocket, SolidMotor
    from hangar.storebodies import BUILDUP, COAST_N, GRAIN_DENSITY
    # Preserve the current model's explicitly documented 2 N between pulses.
    phases = [dict(p, thrust_kn=max(1000 * p["thrust_kn"], COAST_N) / 1000)
              for p in data["design"]["motor"]["phase"]]
    curve = phase_curve(phases, buildup_s=BUILDUP)
    motor_data = data["motor"]
    motor = SolidMotor(thrust_source=curve, burn_time=(0, curve[-1][0]), dry_mass=0,
                       dry_inertia=(0, 0, 0), nozzle_radius=math.sqrt(motor_data["exit_area"] / math.pi),
                       grain_number=1, grain_density=GRAIN_DENSITY, grain_outer_radius=motor_data["radius"],
                       grain_initial_inner_radius=motor_data["bore"] / 2, grain_initial_height=motor_data["length"],
                       grain_separation=0, grains_center_of_mass_position=-motor_data["x"],
                       center_of_dry_mass_position=0, nozzle_position=-motor_data["exit_x"],
                       reference_pressure=0, only_radial_burn=True)
    # RocketPy otherwise wraps raw arrays with Function's default spline. The
    # emitted JSBSim table is piecewise linear with endpoint clamping.
    drag = Function(data["drag"], interpolation="linear", extrapolation="constant")
    rocket = Rocket(radius=data["radius"], mass=data["dry_mass"], inertia=data["dry_inertia"],
                    power_off_drag=drag, power_on_drag=drag,
                    center_of_mass_without_motor=-data["dry_cg"])
    rocket.add_motor(motor, position=0)
    env = Environment(gravity=radial_gravity if spherical else initial_environment["gravity_ms2"], latitude=0, longitude=0)
    if spherical:
        env.earth_rotation_vector = [0.0, 0.0, 0.0]
    env.pressure = Function(atmosphere["pressure_pa"])
    env.temperature = Function(atmosphere["temperature_k"])
    env.density = Function(atmosphere["density_kg_m3"])
    env.speed_of_sound = Function(initial_environment["sound_ms"])
    flight = Flight(rocket=rocket, environment=env, rail_length=1, max_time=times[-1],
                    initial_solution=[0, 0, 0, 8000, 0, 0, 250, 1, 0, 0, 0, 0, 0, 0],
                    max_time_step=max_step, rtol=1e-9, atol=1e-9, time_overshoot=False,
                    simulation_mode="6 DOF")
    if flight.solution[-1][0] < times[-1] - 1e-8:
        raise ValueError("RocketPy reference terminated before the complete sample interval")
    rows = [[t, float(rocket.total_mass(t)), float(flight.vz(t)), float(flight.z(t))] for t in times]
    return rows, float(motor.total_impulse), curve[-1][0]


def campaign(models, native_bin, *, spherical=False, rates=(240, 480)):
    if not isinstance(models, list) or not models or len(set(models)) != len(models):
        raise ValueError("a nonempty list of distinct models is required")
    if len(rates) != 2 or rates[0] >= rates[1] or any(type(r) is not int or r < 20 or r > 1920 or r % 20 for r in rates):
        raise ValueError("two increasing native rates divisible by 20, between 20 and 1920 Hz, are required")
    provenance = runtime(native_bin)
    results = []
    for name in models:
        data = inputs(name)
        duration = data["motor"]["burn_s"] + 0.05 * (1 - 2 / math.pi) + 0.25
        times = [i / 20 for i in range(math.ceil(duration * 20) + 1)]
        for case, atmosphere in ATMOSPHERES.items():
            runs = []
            for divisor in rates:
                with tempfile.TemporaryDirectory(prefix="fsim-reference-") as temporary:
                    actual, impulse, initial, fixtures = native(data, 1 / divisor, atmosphere, times, Path(temporary), spherical=spherical)
                    expected, expected_impulse, burnout = rocketpy(data, atmosphere, times, initial, max_step=1 / 960, spherical=spherical)
                comparison = compare(actual, expected, LIMITS)
                impulse_error = abs(impulse - expected_impulse) / expected_impulse
                comparison["passed"] &= impulse_error <= 0.005
                runs.append({"native_dt_s": 1 / divisor, "comparison": comparison,
                             "vacuum_impulse_ns": impulse, "reference_impulse_ns": expected_impulse,
                             "relative_impulse_error": impulse_error, "reference_burnout_s": burnout,
                             "native": actual, "reference": expected, "instrumented_models": fixtures,
                             "initial_environment": initial})
            convergence = runs[1]["comparison"]["max_abs"]["speed_ms"] <= runs[0]["comparison"]["max_abs"]["speed_ms"] + 0.1
            results.append({"model": name, "case": case, "atmosphere": atmosphere,
                            "passed": all(r["comparison"]["passed"] for r in runs) and convergence,
                            "step_sensitivity_passed": convergence, "inputs": data["records"], "runs": runs})
            print(name, case, "PASS" if results[-1]["passed"] else "FAIL",
                  [r["comparison"]["max_abs"] for r in runs], flush=True)
    return {"schema": "fsim.missile-flight-reference.v1", "passed": all(r["passed"] for r in results),
            "source_assessment": "not_assessed", "equipment_validation": "pending",
            "scope": "axial unguided flight under declared game inputs; lateral/control behavior excluded",
            "planet": {"kind": "nonrotating_spherical" if spherical else "native_wgs84_with_flat_reference",
                       "radius_m": PLANET_RADIUS_M if spherical else None,
                       "mu_m3_s2": PLANET_MU_M3_S2 if spherical else None},
            "provenance": provenance, "cases": results}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--model", action="append", required=True, dest="models")
    parser.add_argument("--native-bin", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--spherical-reference", action="store_true",
                        help="match a declared nonrotating central gravity field in both engines")
    parser.add_argument("--native-hz", type=int, nargs=2, default=(240, 480), metavar=("COARSE", "FINE"))
    args = parser.parse_args(argv)
    output = permitted(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    pending = {"schema": "fsim.missile-flight-reference.v1", "passed": False, "status": "incomplete",
               "source_assessment": "not_assessed", "equipment_validation": "pending"}
    # Invalidate an earlier success before native work, including interruption.
    output.write_text(json.dumps(pending) + "\n", encoding="utf-8")
    try:
        result = campaign(args.models, permitted(args.native_bin), spherical=args.spherical_reference, rates=args.native_hz)
        serialized = json.dumps(result, indent=2, allow_nan=False)
    except Exception as error:  # SDK and external numerical failures must invalidate the report.
        result = dict(pending, status="failed", error=type(error).__name__ + ": " + str(error))
        serialized = json.dumps(result, indent=2, allow_nan=False)
    output.write_text(serialized + "\n", encoding="utf-8")
    if "error" in result:
        print(result["error"], file=sys.stderr)
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
