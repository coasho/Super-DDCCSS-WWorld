# make_cocip_test.py: tests/test_cocip.cpp from cocip_reference.py's cocip_ref.json.
import json, pathlib

here = pathlib.Path(__file__).parent
cases = json.loads((here / "cocip_ref.json").read_text())
L = []
w = L.append
w('''// world::cocip against pycontrails itself: the reference values were made by pycontrails' own functions (its
// sac.py, wake_vortex.py, contrail_properties.py, wind_shear.py, thermo.py; cocip.py's sequence of them) for the same
// air - a standard atmosphere with a constant relative humidity over water - by tools/contrails/cocip_reference.py.
#include "visualization/vfx/effects/Cocip.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

using namespace fsim::world;

namespace {

constexpr double kG = 9.80665, kRd = 287.05, kEps = 287.05 / 461.51;

cocip::Air standard(double h, double rhLiquid) {
    const double T0 = 288.15, p0 = 101325.0, L = 0.0065;
    double T, p;
    if (h <= 11000.0) {
        T = T0 - L * h;
        p = p0 * std::pow(T / T0, kG / (kRd * L));
    } else {
        const double T11 = T0 - L * 11000.0, p11 = p0 * std::pow(T11 / T0, kG / (kRd * L));
        T = T11;
        p = p11 * std::exp(-kG * (h - 11000.0) / (kRd * T11));
    }
    return {T, p, rhLiquid * kEps * cocip::eSatLiquid(T) / p};
}

bool near(double a, double b, double rel = 1e-6) { return std::abs(a - b) <= rel * std::max({std::abs(a), std::abs(b), 1e-30}); }

struct Step {
    int index;
    double age, width, depth, sigmaYz, iwc, n, h, tau, fallSpeed;
    bool alive;
};

void check(const char* name, double h, double rh, double V, double m, double b, double ff, double ds, double dsn, double G, double tLM,
           double tCrit, bool forms, double dzMax, bool persistent, const std::vector<Step>& steps) {
    INFO(name);
    const cocip::AirAt air = [rh](double z) { return standard(z, rh); };
    const auto f = cocip::formation(air(h));
    REQUIRE(near(f.G, G));
    REQUIRE(near(f.tLM, tLM));
    REQUIRE(f.forms == forms);
    if (!forms) return;
    REQUIRE(near(f.tCritical, tCrit, 1e-9));
    cocip::Aircraft ac{b, m, V, ff};
    const cocip::Shear shear{ds, dsn};
    const auto d = cocip::downwash(ac, h, air, shear);
    REQUIRE(near(d.dzMax, dzMax));
    REQUIRE(d.persistent == persistent);
    cocip::Plume p = d.plume;
    const double dts[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60};
    int at = 0;
    for (const auto& s : steps) {
        while (at < s.index) {
            cocip::step(p, air, shear, dts[at]);
            ++at;
        }
        INFO("step " << s.index);
        REQUIRE(near(p.age, s.age));
        REQUIRE(near(p.width, s.width));
        REQUIRE(near(p.depth, s.depth));
        REQUIRE(near(p.sigmaYz, s.sigmaYz));
        REQUIRE(near(p.iwc, s.iwc));
        REQUIRE(near(p.nIcePerM, s.n));
        REQUIRE(near(p.altitude, s.h));
        REQUIRE(near(p.tau, s.tau));
        REQUIRE(near(p.fallSpeed, s.fallSpeed));
        REQUIRE(p.alive == s.alive);
    }
}

} // namespace

TEST_CASE("the contrail model is pycontrails' CoCiP: formation, wake vortex, the plume's evolution", "[cocip]") {''')
for c in cases:
    picks = []
    n = len(c["steps"])
    for k in sorted(set([0, 1, 5, 10, 15, n - 1])):
        if 0 <= k < n:
            s = c["steps"][k]
            picks.append("{%d, %.17g, %.17g, %.17g, %.17g, %.17g, %.17g, %.17g, %.17g, %.17g, %s}" % (
                k, s["age"], s["width"], s["depth"], s["sigma_yz"], s["iwc"], s["n"], s["h"], s["tau"], s["vt"],
                "true" if s["alive"] > 0.5 else "false"))
    w('    check("%s", %r, %r, %r, %r, %r, %r, %r, %r, %.17g, %.17g, %.17g, %s, %.17g, %s,\n          {%s});' % (
        c["name"], c["h"], c["rh"], c["V"], c["m"], c["b"], c["ff"], c["ds"], c["dsn"], c["G"], c["T_LM"], c["T_crit"],
        "true" if c["forms"] else "false", c.get("dz_max", 0.0), "true" if c.get("persistent") else "false",
        ",\n           ".join(picks)))
w("}")
(here.parents[1] / "tests" / "test_cocip.cpp").write_text("\n".join(L) + "\n", encoding="utf-8", newline="\n")
print("written")
