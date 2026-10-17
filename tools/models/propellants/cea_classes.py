# cea_classes.py: the solid motors' propellant classes through NASA's chemical equilibrium code (CEA, McBride and
# Gordon, NASA RP-1311), for what the viewer draws of their exhaust (world/Exhaust.cpp rocketOf, world/Airflow.cpp).
#
#   python tools/models/propellants/cea_classes.py [out.json]      (default tools/models/propellants/classes.json)
#
# Needs RocketCEA (pip install rocketcea - CEA's Fortran compiled for Python; GPL-3, installed apart from this
# repository, which keeps only the numbers it gives). One composition a class, the same for every nation's missiles:
#   aluminised     AP 68 %, Al 18 %, HTPB 14 % (a tactical motor's usual loading)
#   reduced smoke  AP 86 %, HTPB 14 % (no metal; the aluminised's binder share)
# HTPB as CEA's users write it (C 7.075 H 10.65 O 0.223 N 0.063 per 100 g, +1,200 cal: cured with its isocyanate).
# The chamber at 10 MPa (a tactical motor's 7 - 14), the nozzle's area ratio the hangar's designs' (exit over throat
# diameter 0.1056 / 0.0475: 4.94), shifting equilibrium through it (CEA's rocket problem). For each: the chamber's and
# the exit's temperature, the exit's Mach number, the expansion's effective gamma (the exit's temperature against the
# chamber's over the pressure ratio - the two phases' together for an aluminised motor), the gas's molar mass and the
# specific impulse at the exit's pressure; the exit's composition by mass, and what follows for the plume: its water once
# its hydrogen has burnt in the air, the heat that afterburning releases (H2 at 120.0 MJ/kg, CO at 10.1 MJ/kg, lower
# heating values), its HCl and its alumina. And how much each moves with the chamber's pressure (7 and 14 MPa) and,
# for the reduced class, its AP share (84 and 88 %).
# A minimum-smoke class (a nitramine double base) is not computed: no design uses one, and its ingredients
# (nitrocellulose, nitroglycerine, HMX) are not in CEA's library - its numbers stay an estimate (said so where used).
import json
import math
import pathlib
import sys

from rocketcea.cea_obj import CEA_Obj, add_new_propellant

HTPB = "fuel HTPB  C 7.075 H 10.65 O 0.223 N 0.063  wt%={w}  h,cal=1200.0  t(k)=298.15\n"
AP = "oxid NH4CLO4(I) wt%={w} t(k)=298.15\n"
AL = "fuel AL(cr) wt%={w} t(k)=298.15\n"
EPS = 4.94
PSIA_PER_MPA = 145.0377
LHV = {"H2": 119.96e6, "CO": 10.10e6}
M_H2O, M_H2 = 18.0153, 2.01588


def compose(ap, al, htpb):
    return AP.format(w=ap) + (AL.format(w=al) if al > 0 else "") + HTPB.format(w=htpb)


def solve(name, card, pc_mpa):
    add_new_propellant(name, card)
    c = CEA_Obj(propName=name)
    pc = pc_mpa * PSIA_PER_MPA
    tc, tt, te = c.get_Temperatures(Pc=pc, eps=EPS)                       # (degR)
    k = 5.0 / 9.0
    mach = c.get_MachNumber(Pc=pc, eps=EPS)
    pratio = c.get_PcOvPe(Pc=pc, eps=EPS)
    mw, _ = c.get_exit_MolWt_gamma(Pc=pc, eps=EPS)
    isp_vac = c.get_Isp(Pc=pc, eps=EPS)                                     # (s)
    isp_amb = c.estimate_Ambient_Isp(Pc=pc, eps=EPS, Pamb=pc / pratio)[0]   # at the exit's pressure
    _, mass = c.get_SpeciesMassFractions(Pc=pc, eps=EPS, frozen=0, min_fraction=1e-6)
    exitf = {sp.lstrip("*"): v[3] for sp, v in mass.items()}               # [injector, chamber, throat, exit] (CEA stars some)
    w = lambda *names: sum(exitf.get(n, 0.0) for n in names)
    tcK, teK = tc * k, te * k
    gamma = 1.0 / (1.0 - math.log(tcK / teK) / math.log(pratio))            # T ratio = p ratio^((g-1)/g)
    h2o, h2, co = w("H2O"), w("H2"), w("CO")
    return {
        "chamber_mpa": pc_mpa, "chamber_k": round(tcK, 1), "throat_k": round(tt * k, 1), "exit_k": round(teK, 1),
        "exit_mach": round(mach, 3), "chamber_over_exit_pressure": round(pratio, 3),
        "exit_over_sea_level_pressure": round(pc_mpa * 1e6 / pratio / 101325.0, 3), "effective_gamma": round(gamma, 4),
        "exit_gas_molar_mass": round(mw, 3), "isp_vacuum_s": round(isp_vac, 1), "isp_exit_pressure_s": round(isp_amb, 1),
        "exit_mass_fractions": {sp: round(v, 5) for sp, v in sorted(exitf.items(), key=lambda kv: -kv[1]) if v >= 1e-4},
        "water_after_afterburning": round(h2o + h2 * M_H2O / M_H2, 4),
        "afterburning_heat_j_per_kg": round(h2 * LHV["H2"] + co * LHV["CO"], -3),
        "hcl": round(w("HCL"), 4), "alumina": round(w("AL2O3(a)", "AL2O3(L)", "AL2O3(cr)"), 4),
    }


def main(argv):
    out = pathlib.Path(argv[0]) if argv else pathlib.Path(__file__).with_name("classes.json")
    classes = {
        "aluminised": {"composition_wt_pct": {"AP": 68.0, "Al": 18.0, "HTPB": 14.0}, "card": compose(68.0, 18.0, 14.0)},
        "reduced": {"composition_wt_pct": {"AP": 86.0, "HTPB": 14.0}, "card": compose(86.0, 0.0, 14.0)},
    }
    result = {"method": "NASA CEA (McBride and Gordon, NASA RP-1311) through RocketCEA: rocket problem, shifting equilibrium, "
                        "infinite-area combustor", "area_ratio": EPS, "classes": {}}
    for name, c in classes.items():
        base = solve(name + "10", c["card"], 10.0)
        lo, hi = solve(name + "7", c["card"], 7.0), solve(name + "14", c["card"], 14.0)
        sens = {"chamber_7_14_mpa": {key: [lo[key], hi[key]] for key in ("chamber_k", "exit_k", "exit_mach", "effective_gamma",
                                                                         "water_after_afterburning", "hcl", "alumina")}}
        if name == "reduced":
            a, b = solve("red84", compose(84.0, 0.0, 16.0), 10.0), solve("red88", compose(88.0, 0.0, 12.0), 10.0)
            sens["ap_84_88_pct"] = {key: [a[key], b[key]] for key in ("chamber_k", "exit_k", "water_after_afterburning",
                                                                      "afterburning_heat_j_per_kg", "hcl")}
        result["classes"][name] = {"composition_wt_pct": c["composition_wt_pct"], **base, "sensitivity": sens}
        print("%-10s chamber %.0f K, exit %.0f K at Mach %.2f (gamma %.3f, %.2f g/mol); water after afterburning %.3f, "
              "afterburning %.2f MJ/kg, HCl %.3f, alumina %.3f; Isp %.0f s (vacuum %.0f)"
              % (name, base["chamber_k"], base["exit_k"], base["exit_mach"], base["effective_gamma"], base["exit_gas_molar_mass"],
                 base["water_after_afterburning"], base["afterburning_heat_j_per_kg"] / 1e6, base["hcl"], base["alumina"],
                 base["isp_exit_pressure_s"], base["isp_vacuum_s"]))
    out.write_text(json.dumps(result, indent=1) + "\n", encoding="utf-8", newline="\n")
    print(out)


if __name__ == "__main__":
    main(sys.argv[1:])
