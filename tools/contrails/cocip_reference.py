# cocip_reference.py <pycontrails clone>: reference values from pycontrails' own functions (a clone of
# github.com/contrailcirrus/pycontrails; its package __init__, xarray, scipy and the
# Flight/Met classes stubbed - the physics functions run as they are), for the C++ port's parity test: the
# Schmidt-Appleman criterion, the wake vortex and first contrail (cocip.py's sequence), and time steps of the plume
# (calc_timestep_contrail_evolution's sequence, calc_contrail_properties after each).
import dataclasses
import importlib.util
import json
import pathlib
import sys
import types

import numpy as np

ROOT = pathlib.Path(sys.argv[1]) / "pycontrails"


def module(name, path=None, **attrs):
    m = types.ModuleType(name)
    if path is not None:
        m.__path__ = [str(path)]
    for k, v in attrs.items():
        setattr(m, k, v)
    sys.modules[name] = m
    return m


class DataArray:  # (xarray: only isinstance checks reach it)
    pass


def newton(func, x0, fprime=None, maxiter=50, disp=True, tol=1.48e-8):  # scipy.optimize.newton with fprime
    x = np.asarray(x0, dtype=float).copy()
    for _ in range(maxiter):
        f = func(x)
        d = fprime(x)
        step = f / d
        x_new = x - step
        if np.all(np.abs(x_new - x) < tol):
            return x_new
        x = x_new
    return x


module("xarray", DataArray=DataArray)
scipy_mod = module("scipy")
scipy_mod.optimize = module("scipy.optimize", newton=newton)
pkg = module("pycontrails", ROOT, __version__="ref")
module("pycontrails.physics", ROOT / "physics")
module("pycontrails.models", ROOT / "models")
module("pycontrails.models.cocip", ROOT / "models" / "cocip")
module("pycontrails.core", ROOT / "core")
module("pycontrails.utils", ROOT / "utils")


@dataclasses.dataclass
class ModelParams:
    pass


class Model:
    pass


module("pycontrails.core.flight", Flight=object)
module("pycontrails.core.met", MetDataset=object)
module("pycontrails.core.met_var", AirTemperature=object, SpecificHumidity=object)
module("pycontrails.core.models", Model=Model, ModelParams=ModelParams)
module("pycontrails.core.vector", GeoVectorDataset=object)
module("pycontrails.models.humidity_scaling", HumidityScaling=object)
module("pycontrails.utils.types", apply_nan_mask_to_arraylike=lambda a, mask: a)


def load(name, file):
    spec = importlib.util.spec_from_file_location(name, file)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


constants = load("pycontrails.physics.constants", ROOT / "physics" / "constants.py")
units = load("pycontrails.physics.units", ROOT / "physics" / "units.py")
thermo = load("pycontrails.physics.thermo", ROOT / "physics" / "thermo.py")
fuel = load("pycontrails.core.fuel", ROOT / "core" / "fuel.py")
sac = load("pycontrails.models.sac", ROOT / "models" / "sac.py")
radiative_heating = load("pycontrails.models.cocip.radiative_heating", ROOT / "models" / "cocip" / "radiative_heating.py")
wind_shear = load("pycontrails.models.cocip.wind_shear", ROOT / "models" / "cocip" / "wind_shear.py")
wake_vortex = load("pycontrails.models.cocip.wake_vortex", ROOT / "models" / "cocip" / "wake_vortex.py")
cp = load("pycontrails.models.cocip.contrail_properties", ROOT / "models" / "cocip" / "contrail_properties.py")

G0, RD = constants.g, constants.R_d


def isa(h, T0=288.15, p0=101325.0):
    """The standard atmosphere from sea-level T and p (as the C++ test's)."""
    L = 0.0065
    if h <= 11000.0:
        T = T0 - L * h
        p = p0 * (T / T0) ** (G0 / (RD * L))
    else:
        T11 = T0 - L * 11000.0
        p11 = p0 * (T11 / T0) ** (G0 / (RD * L))
        T = T11
        p = p11 * np.exp(-G0 * (h - 11000.0) / (RD * T11))
    return T, p


def air(h, rh_liquid):
    T, p = isa(h)
    q = rh_liquid * constants.epsilon * thermo.e_sat_liquid(T) / p
    return T, p, q


A = lambda v: np.array([float(v)])


def dtheta_dz(h, rh):
    T, p, _ = air(h, rh)
    Tb, pb, _ = air(h - 200.0, rh)
    return thermo.T_potential_gradient(A(T), A(p), A(Tb), A(pb), 200.0)[0]


def properties(st, h, rh, ds, dsn):
    T, p, q = air(h, rh)
    rho = thermo.rho_d(T, p)
    enh = wind_shear.wind_shear_enhancement_factor(A(st["depth"]), 2000.0, 0.5)
    ds_dz = A(ds) * enh
    dsn_dz = A(dsn) * enh
    area = cp.plume_effective_cross_sectional_area(A(st["width"]), A(st["depth"]), A(st["sigma_yz"]))
    depth_eff = cp.plume_effective_depth(A(st["width"]), area)
    n_vol = cp.ice_particle_number_per_volume_of_plume(A(st["n"]), area)
    n_kg = cp.ice_particle_number_per_mass_of_air(n_vol, A(rho))
    mass = cp.plume_mass_per_distance(area, A(rho))
    r = cp.ice_particle_volume_mean_radius(A(st["iwc"]), n_kg)
    tau = cp.contrail_optical_depth(r, A(st["n"]), A(st["width"]))
    vt = cp.ice_particle_terminal_fall_speed(A(p), A(T), r)
    dh = cp.horizontal_diffusivity(ds_dz, A(st["depth"]), None)
    dv = cp.vertical_diffusivity(A(p), A(T), A(dtheta_dz(h, rh)), depth_eff, vt, 0.1, 0.5, None, None)
    agg = cp.particle_losses_aggregation(r, vt, area)
    turb = cp.particle_losses_turbulence(A(st["width"]), A(st["depth"]), depth_eff, dh, dv)
    st.update(dsn_dz=dsn_dz[0], area=area[0], mass=mass[0], r=r[0], tau=tau[0], vt=vt[0], dh=dh[0], dv=dv[0], agg=agg[0],
              turb=turb[0], n_vol=n_vol[0])


def case(name, h, rh, V, m, b, ff, ds, dsn, steps):
    T, p, q = air(h, rh)
    G = sac.slope_mixing_line(A(q), A(p), 0.3, 1.23, 43.13e6)
    T_LM = sac.T_sat_liquid(G)
    rh_crit = sac.rh_critical_sac(A(T), T_LM, G)
    rh_ = thermo.rh(A(q), A(T), A(p))
    forms = bool(sac.sac(rh_, rh_crit)[0] > 0.5)
    T_crit = sac.T_critical_sac(T_LM, rh_, G)
    out = {"name": name, "h": h, "rh": rh, "V": V, "m": m, "b": b, "ff": ff, "ds": ds, "dsn": dsn,
           "G": G[0], "T_LM": T_LM[0], "rh_crit": float(rh_crit[0]), "forms": forms, "T_crit": T_crit[0], "steps": []}
    if not forms:
        return out
    dT = dtheta_dz(h, rh)
    dz_max = wake_vortex.max_downward_displacement(A(b), A(V), A(m), A(T), A(dT), A(ds), A(p), 2000.0, 0.5, 0.1)[0]
    width = wake_vortex.initial_contrail_width(A(b), A(dz_max))[0]
    depth = wake_vortex.initial_contrail_depth(A(dz_max), 0.5)[0]
    fuel_dist = ff / V
    iwc = cp.initial_iwc(A(T), A(q), A(p), A(fuel_dist), A(width), A(depth), 1.23)[0]
    h1 = h - 0.5 * depth
    T1, p1, q1 = air(h1, rh)
    iwc_ad = cp.iwc_adiabatic_heating(A(T), A(p), A(p1))[0]
    iwc_1 = cp.iwc_post_wake_vortex(A(iwc), A(iwc_ad))[0]
    f_act = cp.ice_particle_activation_rate(A(T), A(T_crit[0]))[0]
    n0 = cp.initial_ice_particle_number(A(1e15 * f_act), A(fuel_dist), 1e13)[0]
    f_surv = cp.ice_particle_survival_fraction(A(iwc), A(iwc_1))[0]
    rhi_1 = thermo.rhi(q1, T1, p1)
    persistent = bool(cp.initial_persistent(A(iwc_1), A(rhi_1))[0] > 0.5)
    out.update(dz_max=dz_max, width=width, depth=depth, iwc0=iwc, iwc1=iwc_1, n0=n0, f_surv=f_surv, persistent=persistent)
    st = dict(width=width, depth=depth, sigma_yz=0.0, iwc=iwc_1, n=n0 * f_surv, h=h1, age=0.0, alive=persistent)
    properties(st, h1, rh, ds, dsn)
    out["steps"].append({k: float(v) for k, v in st.items()})
    for dt in steps:
        if not st["alive"]:
            break
        T1, p1, q1 = air(st["h"], rh)
        dt64 = np.timedelta64(int(round(dt * 1000)), "ms")
        syy, szz, syz = cp.plume_temporal_evolution(A(st["width"]), A(st["depth"]), A(st["sigma_yz"]), A(st["dsn_dz"]),
                                                    A(st["dh"]), A(st["dv"]), 1.0, dt64, 1500.0)
        w2, d2 = cp.new_contrail_dimensions(syy, szz)
        h2 = st["h"] - st["vt"] * dt
        T2, p2, q2 = air(h2, rh)
        rho2 = thermo.rho_d(T2, p2)
        area2 = cp.new_effective_area_from_sigma(syy, szz, syz)
        mass2 = cp.plume_mass_per_distance(area2, A(rho2))
        iwc2 = cp.new_ice_water_content(A(st["iwc"]), A(q1), A(q2), A(thermo.q_sat_ice(T1, p1)), A(thermo.q_sat_ice(T2, p2)),
                                        A(st["mass"]), mass2)[0]
        n2 = cp.new_ice_particle_number(A(st["n"]), A(st["agg"]), A(st["turb"]), 1.0, dt64)[0]
        st.update(width=w2[0], depth=d2[0], sigma_yz=syz[0], h=h2, iwc=iwc2, n=n2, age=st["age"] + dt)
        properties(st, h2, rh, ds, dsn)
        st["alive"] = (st["age"] < 72000.0 and 1e-6 <= st["tau"] <= 1e10 and 1e3 <= st["n_vol"] <= 1e20 and 6000.0 <= h2 <= 13000.0)
        out["steps"].append({k: float(v) for k, v in st.items()})
    return out


steps = [1.0] * 10 + [60.0] * 10
cases = [
    case("f16_issr", 10500.0, 0.75, 250.0, 12000.0, 9.45, 0.8, 0.004, 0.0025, steps),
    case("f16_dry", 10500.0, 0.35, 250.0, 12000.0, 9.45, 0.8, 0.004, 0.0025, steps),
    case("airliner_issr", 11500.0, 0.8, 230.0, 65000.0, 34.0, 0.7, 0.003, 0.003, steps),
    case("low_warm", 3000.0, 0.8, 200.0, 12000.0, 9.45, 0.8, 0.004, 0.0025, steps),
]
out = pathlib.Path(__file__).parent / "cocip_ref.json" # (make_cocip_test.py turns it into tests/test_cocip.cpp)
out.write_text(json.dumps(cases, indent=1))
for c in cases:
    print(c["name"], "forms", c["forms"], "persistent", c.get("persistent"), "dz_max", round(c.get("dz_max", 0), 2),
          "n steps", len(c["steps"]))
    if c["steps"]:
        s = c["steps"][-1]
        print("   last: age", s["age"], "w", round(s["width"], 1), "d", round(s["depth"], 1), "tau", s["tau"], "alive", s["alive"])
