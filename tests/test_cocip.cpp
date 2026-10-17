// world::cocip against pycontrails itself: the reference values were made by pycontrails' own functions (its
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

TEST_CASE("the contrail model is pycontrails' CoCiP: formation, wake vortex, the plume's evolution", "[cocip]") {
    check("f16_issr", 10500.0, 0.75, 250.0, 12000.0, 9.45, 0.8, 0.004, 0.0025, 1.6096095215364905, 231.00385690099, 225.46077584310555, true, 54.012651473830346, true,
          {{0, 0, 7.4220126441058856, 27.006325736915173, 0, 7.9084925734332606e-05, 3160731048431.8623, 10486.496837131543, 1.7930291350858749, 0.00046717848939790345, true},
           {1, 1, 8.809685429704933, 27.252656835234763, 1.1046832093371597, 6.8649087357829032e-05, 3159134114722.7168, 10486.496369953053, 1.5239884431496293, 0.00047485287752540624, true},
           {5, 5, 13.153503151672103, 28.216805961780601, 5.6778223047610172, 5.0177112614659234e-05, 3153242308310.7056, 10486.49443232874, 1.0419415651572139, 0.00049880214144606527, true},
           {10, 10, 17.441947262653827, 29.378110234107666, 11.737564997067146, 4.0903673642036275e-05, 3146455093841.8306, 10486.491889167462, 0.79487943524281668, 0.0005225638958102814, true},
           {15, 310, 186.62587764783126, 70.139219874322961, 992.33196381304981, 1.6886396877850178e-05, 2951036839777.9053, 10486.232959673829, 0.2792916275737633, 0.0013207680516088298, true},
           {20, 610, 373.71416655311964, 95.357695519044796, 2944.6264088587986, 1.6130870619705584e-05, 2878379219940.4092, 10485.759302047385, 0.24287655733692154, 0.001946014314644143, true}});
    check("f16_dry", 10500.0, 0.35, 250.0, 12000.0, 9.45, 0.8, 0.004, 0.0025, 1.609548699580148, 231.00346353455853, 222.97464015024215, true, 54.012651473830346, true,
          {{0, 0, 7.4220126441058856, 27.006325736915173, 0, 3.5273595027287268e-05, 3018192287230.207, 10486.496837131543, 1.1886759173211618, 0.00033933560565519756, true},
           {1, 1, 8.8096853705377018, 27.252258832316578, 1.1046669299731511, 2.4777682862848268e-05, 3016892447502.0415, 10486.496497795937, 0.88397975226870806, 0.00031555768001899359, true},
           {5, 5, 13.153256613242526, 28.213898660506601, 5.6773877938089878, 6.2014828032132518e-06, 3012163365685.9424, 10486.49538200341, 0.24758781038498562, 0.00021129577758796514, true},
           {8, 8, 15.815241919476026, 28.913220851548079, 9.2669555454258372, 0, 3008887129906.8726, 10486.494867198262, 0, 6.2719864807226068e-09, false}});
    check("airliner_issr", 11500.0, 0.8, 230.0, 65000.0, 34.0, 0.7, 0.003, 0.003, 1.3755803464819889, 229.38276377075422, 224.40849301351361, true, 123.19114184410518, true,
          {{0, 0, 26.703537555513243, 61.595570922052588, 0, 2.4613550188392204e-05, 2836417948717.3857, 11469.202214538973, 0.48467984383676199, 0.00070703715584953097, true},
           {1, 1, 27.8290110159816, 61.659666442759672, 4.7699349630753298, 2.4329030950847981e-05, 2835963512047.3237, 11469.201507501817, 0.46860511035883001, 0.0007161854097593078, true},
           {5, 5, 32.07223607545891, 61.91566678012537, 23.927778635854168, 2.3454679259063882e-05, 2834156705839.813, 11469.198592200624, 0.42237825054086892, 0.00074868712514939123, true},
           {10, 10, 36.958190031458393, 62.234691030468184, 48.050794947911371, 2.2716768881687916e-05, 2831914759399.4673, 11469.194778126515, 0.38922179642493365, 0.00078285866924028937, true},
           {15, 310, 253.00795038814201, 79.239209963827577, 1860.9573066711532, 1.8555343513074486e-05, 2744261407536.0986, 11468.853138290388, 0.19371014291537536, 0.0015378675108222468, true},
           {20, 610, 471.23603016910192, 93.532063262522925, 4327.6591944442607, 1.8127413423352313e-05, 2688467833397.6436, 11468.326755620825, 0.19345173417614595, 0.0020687153202857147, true}});
    check("low_warm", 3000.0, 0.8, 200.0, 12000.0, 9.45, 0.8, 0.004, 0.0025, 4.6229276249019993, 242.68124819865031, 236.97900880645511, false, 0, false,
          {});
}
