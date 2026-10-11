// world::Airflow's trails against what has been observed of real contrails (Schumann et al. 2017, "Properties of
// individual contrails: a compilation of observations and some comparisons", ACP 17, 403-438, and the observations
// it compiles): what is drawn of a sample as it ages - its extinction per metre (the observations' total extinction
// EA, the optical depth integrated over the width), how it is shared between the primary and the secondary wake, how
// wide and deep it is - for the aircraft and air of the observations.
#include "world/Airflow.h"
#include "world/FlowFires.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <vector>

using namespace fsim::world;

namespace {

constexpr double kG = 9.80665, kRd = 287.05;

// Air round a flight level (p0, T0) with a lapse rate, holding the specific humidity of RHi there.
cocip::AirAt airAround(double h0, double p0, double T0, double lapse, double rhIce) {
    const double q = rhIce * cocip::qSatIce(T0, p0);
    return [=](double h) {
        const double T = T0 - lapse * (h - h0);
        const double p = p0 * std::pow(T / T0, kG / (kRd * lapse));
        return cocip::Air{T, p, q};
    };
}

// The algebraic model of EA against age the compilation fits to the remote-sensing data (its Eq. 7, Fig. 9, after
// Jessberger et al. 2013): the ice of the exhaust's water and of the supersaturation the plume takes in, over the
// particles the fuel's soot makes; its constants as given there (C 0.7, PEI 1e15 /kg, f_s 0.7, Q_ext 2, dilution
// N_dil = 20 (t / 1 s)^1.7).
double eaModel(double ageS, double fuelPerM, double T, double p, double rhIce) {
    const double rhoIce = 917.0, C = 0.7, Q = 2.0, fs = 0.7, PEI = 1e15, EIH2O = 1.23;
    const double nDil = 20.0 * std::pow(std::max(ageS, 1e-3), 1.7);
    const double water = EIH2O + (rhIce - 1.0) * nDil * 0.622 * cocip::eSatIce(T) / p;
    return std::cbrt(9.0 * 3.14159265358979 / (16.0 * rhoIce * rhoIce)) * C * Q * fuelPerM * std::cbrt(fs * PEI) *
           std::pow(std::max(water, 0.0), 2.0 / 3.0);
}

struct Case {
    const char* name;
    Airflow::Emitter emitter;
    cocip::AirAt air;
};

Airflow::Sample layAt(const Case& c) { return Airflow::lay(c.emitter, c.air, vsg::dvec3(0.0, 0.0, 0.0), 0.0); }

Airflow::Drawn agedTo(const Case& c, Airflow::Sample& s, double& now, double age) {
    while (now < age) {
        now = std::min(now + 0.05, age);
        Airflow::age(s, c.air, now);
    }
    return Airflow::drawn(s, c.air, now);
}

struct Row {
    double age;
    Airflow::Drawn w;
    double W() const { return 2.0 * w.halfSpacing + 4.0 * w.tube; } // the wake's width: the tubes' outer 2 sigma
    double D() const { return w.descent + 2.0 * w.tube; }            // its depth: the flight level to the tubes' lower 2 sigma
};

std::vector<Row> history(const Case& c, std::initializer_list<double> ages) {
    auto s = layAt(c);
    double now = 0.0;
    std::vector<Row> rows;
    for (double age : ages) rows.push_back({age, agedTo(c, s, now, age)});
    if (std::getenv("FSIM_CONTRAIL_TABLE")) {
        const auto a0 = c.air(c.emitter.altitude);
        std::printf("%s: b0 %.1f m, w0 %.2f m/s, dzMax %.0f m, link %.1f s, fuel %.2f g/m\n", c.name, s.separation, s.descentSpeed, s.dzMax, s.linkTime,
                    1e3 * s.fuelDist);
        std::printf("   age  primary secondary  plume  EA-model  tau(prim)  W(m)  D(m)  plume W  D\n");
        for (const auto& r : rows)
            std::printf("%7.2f %7.2f %8.2f %8.2f %8.2f %9.2f %6.0f %5.0f %7.0f %4.0f\n", r.age, r.w.primaryE, r.w.secondaryE, r.w.plumeE,
                        eaModel(r.age, s.fuelDist, a0.temperature, a0.pressure, 1.01), r.W() > 0.0 ? r.w.primaryE / r.W() : 0.0, r.W(), r.D(),
                        std::sqrt(8.0 * r.w.plumeSyy), std::sqrt(8.0 * r.w.plumeSzz));
    }
    return rows;
}

const Row& at(const std::vector<Row>& rows, double age) {
    return *std::find_if(rows.begin(), rows.end(), [age](const Row& r) { return r.age == age; });
}

} // namespace

TEST_CASE("the contrails as drawn against observed contrails", "[contrail-observed]") {
    const std::initializer_list<double> ages{0.05, 0.1, 0.125, 0.15, 0.175, 0.2, 0.3, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 150.0, 200.0, 600.0, 1800.0};

    // Sussmann and Gierens (1999), in Schumann et al. (2017), 3.2.4: a B747-400's contrail at 236.7 hPa, -52.3 C,
    // slightly unstable (taken neutral, the dry adiabatic lapse), RHi about 101 %; outboard engines 21 m either side.
    // Observed at 50 s: the primary and the secondary wake, 4500 m^2 each, the contrail 160 m deep and 90 m wide, the
    // optical depth 1.18 and 0.24; in another, the primary wake sublimated at 170 s and the secondary wake persisted.
    const Case b747{"B747-400 (Sussmann and Gierens 1999)",
                    {cocip::Aircraft{64.4, 300000.0, 250.0, 3.0}, 10800.0, 21.0, 1.2, cocip::Shear{0.004, 0.0025}},
                    airAround(10800.0, 23670.0, 273.15 - 52.3, 0.0098, 1.01)};
    {
        const auto rows = history(b747, ages);
        const auto& r50 = at(rows, 50.0);
        CHECK(r50.W() > 90.0 / 1.5);
        CHECK(r50.W() < 90.0 * 1.5);
        CHECK(r50.D() > 160.0 / 1.5);
        CHECK(r50.D() < 160.0 * 1.5);
        // the share of the ice: the secondary wake's against the whole, as the optical depths over equal sections have it
        CHECK(std::abs(r50.w.secondaryE / (r50.w.primaryE + r50.w.secondaryE) - 0.24 / 1.42) < 0.05);
        // the total extinction: between the compilation's EA model for this fuel and air and what the lidar saw
        const double lidar = (1.18 + 0.24) * 90.0;
        const auto a0 = b747.air(b747.emitter.altitude);
        const double model = eaModel(50.0, 3.0 / 250.0, a0.temperature, a0.pressure, 1.01);
        CHECK(r50.w.total() > model / 2.0);
        CHECK(r50.w.total() < lidar * 1.5);
        // the primary wake sublimates in its descent; the secondary wake outlasts it
        CHECK(at(rows, 200.0).w.primaryE < 0.1 * r50.w.primaryE);
        CHECK(at(rows, 200.0).w.secondaryE > 0.5 * r50.w.secondaryE);
    }

    // Busen and Schumann (1995), in Schumann et al. (2017), 3.1.5: ATTAS (VFW 614) at threshold conditions, its
    // contrail seen 25-35 m behind the engines (200 m/s), each jet's about 2 m across
    const Case attas{"ATTAS (Busen and Schumann 1995)",
                     {cocip::Aircraft{21.5, 18000.0, 200.0, 0.5}, 9000.0, 5.6, 0.5, cocip::Shear{0.004, 0.0025}},
                     airAround(9000.0, 30800.0, 273.15 - 50.0, 0.007, 1.0)};
    {
        const auto rows = history(attas, ages);
        const double full = at(rows, 1.0).w.total();
        CHECK(at(rows, 25.0 / 200.0).w.total() < 0.5 * full);
        CHECK(at(rows, 35.0 / 200.0).w.total() > 0.5 * full);
        CHECK(at(rows, 0.15).w.tube * 4.0 < 2.0 * 2.0); // the jets still apart, each about 2 m (4 sigma) across
    }

    // young contrails' optical depth (CONCERT, Voigt et al. 2011: median 0.13, mean 0.27; the B747's primary wake 1.18):
    // an F-16C's in supersaturated air, from its first seconds to the vortex phase's end
    const Case f16{"F-16C",
                   {cocip::Aircraft{9.45, 12000.0, 250.0, 0.3}, 10500.0, 0.0, 0.5, cocip::Shear{0.004, 0.0025}},
                   airAround(10500.0, 24500.0, 273.15 - 53.0, 0.0065, 1.05)};
    {
        const auto rows = history(f16, ages);
        for (double age : {1.0, 5.0, 10.0}) {
            const auto& r = at(rows, age);
            INFO("F-16C at " << age << " s");
            CHECK(r.w.primaryE / r.W() > 0.05);
            CHECK(r.w.primaryE / r.W() < 2.0);
        }
    }
}

TEST_CASE("a world with no humidity of its own: short contrails, none persisting", "[contrail-observed]") {
    const Airflow::Air world; // (no humidity given)
    const cocip::AirAt air = [world](double h) { return Airflow::airAt(world, h); };
    // the profile: short of ice saturation all the way up, the stratosphere dry
    for (double h : {0.0, 3000.0, 6000.0, 9000.0, 10500.0, 12000.0}) {
        INFO("at " << h << " m");
        CHECK(Airflow::typicalHumidityOverIce(h) < 1.0);
    }
    CHECK(Airflow::typicalHumidityOverIce(12500.0) < 0.35);
    const Airflow::Emitter f16{cocip::Aircraft{9.45, 12000.0, 250.0, 0.3}, 10500.0, 0.0, 0.5, cocip::Shear{0.004, 0.0025}};
    // at cruise it forms (the Schmidt-Appleman criterion holds at -53 C) and does not persist
    CHECK(cocip::formation(air(10500.0), 0.3).forms);
    auto s = Airflow::lay(f16, air, vsg::dvec3(0.0, 0.0, 0.0), 0.0);
    CHECK_FALSE(s.plume.alive);
    double now = 0.0;
    Airflow::Drawn w;
    while (now < 120.0) {
        now += 0.25;
        Airflow::age(s, air, now);
        w = Airflow::drawn(s, air, now);
    }
    CHECK(w.total() == 0.0);
    // at 6 km it is too warm to form one
    CHECK_FALSE(cocip::formation(air(6000.0), 0.3).forms);
}

TEST_CASE("a missile's smoke trail: dense from an aluminised motor, faint from a reduced-smoke one in dry air, thick in humid", "[contrail-observed]") {
    const Airflow::Air dry; // a typical day's humidity
    const cocip::AirAt typical = [dry](double h) { return Airflow::airAt(dry, h); };
    Airflow::Air wet;
    wet.humidity = 0.9; // over water
    const cocip::AirAt humid = [wet](double h) { return Airflow::airAt(wet, h); };
    const double h = 6000.0, V = 400.0, exitRadius = 0.053;
    const auto across = [](const Airflow::Sample& s, double E) { return E / s.plume.width; }; // the trail's optical depth through it
    // an AIM-7M's boost (aluminised), an AIM-120C's (reduced smoke), a minimum-smoke motor's, each 25 kN
    const Airflow::Smoke aluminised{2, 25000.0, V, exitRadius}, reduced{1, 25000.0, V, exitRadius}, minimum{0, 25000.0, V, exitRadius};
    const cocip::Shear shear{Airflow::kShear, Airflow::kShear * 0.64};
    const auto al = Airflow::laySmoke(aluminised, h, shear, typical, vsg::dvec3(), 0.0);
    const auto rs = Airflow::laySmoke(reduced, h, shear, typical, vsg::dvec3(), 0.0);
    const auto ms = Airflow::laySmoke(minimum, h, shear, typical, vsg::dvec3(), 0.0);
    INFO("aluminised " << across(al, al.smokePrimaryE) << ", reduced " << across(rs, rs.smokePrimaryE) << ", minimum "
                       << across(ms, ms.smokePrimaryE) << " across a " << al.plume.width << " m trail");
    // alumina: a dense white trail behind it (the trail's optical depth across it about one)
    CHECK(across(al, al.smokePrimaryE) > 0.3);
    CHECK(across(al, al.smokePrimaryE) < 3.0);
    // a reduced-smoke motor's own particles: a haze (under 0.15, most of the light through); a minimum-smoke one's next
    // to nothing (under 0.02, the contrails' threshold of being seen)
    CHECK(across(rs, rs.smokePrimaryE) < 0.15);
    CHECK(across(rs, rs.smokePrimaryE) > across(ms, ms.smokePrimaryE));
    CHECK(across(ms, ms.smokePrimaryE) < 0.02);
    // a typical day's air at 6 km is too dry for the HCl's droplets; humid air makes them, and then the HCl's smoke
    // outweighs a reduced-smoke motor's own; a minimum-smoke motor has no perchlorate, so none
    CHECK(rs.smokeSecondaryE == 0.0);
    const auto rw = Airflow::laySmoke(reduced, h, shear, humid, vsg::dvec3(), 0.0);
    const auto mw = Airflow::laySmoke(minimum, h, shear, humid, vsg::dvec3(), 0.0);
    INFO("reduced smoke in humid air: " << across(rw, rw.smokeSecondaryE));
    CHECK(rw.smokeSecondaryE > 5.0 * rw.smokePrimaryE);
    CHECK(mw.smokeSecondaryE == 0.0);
    // the droplets form more readily cold than warm, and hold more water the more humid the air
    CHECK(Airflow::secondarySmokeOnset(233.15) < Airflow::secondarySmokeOnset(273.15));
    CHECK(Airflow::acidMassFraction(0.5) > Airflow::acidMassFraction(0.9));
    // nothing laid with no thrust (burnt out) or standing still
    CHECK(Airflow::smokeExtinction(Airflow::Smoke{2, 0.0, V, exitRadius}, typical(h)).primary == 0.0);
    CHECK(Airflow::smokeExtinction(Airflow::Smoke{2, 25000.0, 0.0, exitRadius}, typical(h)).primary == 0.0);

    // it spreads with the air's turbulence and shear, its smoke kept, until it can no longer be seen
    auto s = al;
    double now = 0.0, width = s.plume.width;
    const double E = Airflow::drawn(s, typical, 1.0).plumeE;
    CHECK(E == al.smokePrimaryE);
    double visible = 0.0;
    while (now < 1800.0) {
        now += 0.5;
        Airflow::age(s, typical, now);
        CHECK(s.plume.width >= width);
        width = s.plume.width;
        if (across(s, E) >= 0.02) visible = now;
    }
    INFO("seen for " << visible << " s, " << width << " m wide after 30 minutes");
    CHECK(Airflow::drawn(s, typical, now).plumeE == E);
    CHECK(visible > 20.0);    // a trail that hangs in the air
    CHECK(visible < 1800.0);  // and fades
}

TEST_CASE("a missile's ice contrail: by its exhaust's water and heat, short in dry air, persisting where the air is ice-supersaturated",
          "[contrail-observed]") {
    const double V = 700.0, exitRadius = 0.053;
    const cocip::Shear shear{Airflow::kShear, Airflow::kShear * 0.64};
    // an AIM-120C's sustain (reduced smoke, some 10 kN) and an AIM-7M's boost (aluminised, 25 kN)
    const Airflow::Smoke amraam{1, 10000.0, V, exitRadius}, sparrow{2, 25000.0, V, exitRadius};
    const Airflow::Air world; // a typical day's humidity
    const cocip::AirAt typical = [world](double h) { return Airflow::airAt(world, h); };

    // the mixing line: the exhaust's water against the heat it brings (world/Propellants.h, NASA CEA). A reduced-smoke
    // motor's 0.33 kg of water for some 5.9 MJ per kg is more water for its heat than jet fuel's 1.23 kg for its 30 MJ:
    // it forms in air a few kelvin warmer than a jet's contrail does. An aluminised motor's 0.29 kg for some 10.8 MJ (its
    // exit at 2,327 K, its afterburning 5.8 MJ/kg) is less: it needs colder air - on a typical day at the tropopause
    // (11.5 km, -56.5 C), not at 10.5 km. Neither forms at 6 km
    const auto jet = cocip::formation(typical(10500.0), 0.3), rocket = Airflow::iceFormation(amraam, typical(10500.0));
    const auto alumina = Airflow::iceFormation(sparrow, typical(10500.0));
    INFO("G: reduced smoke " << rocket.G << " Pa/K, aluminised " << alumina.G << ", jet " << jet.G << "; T_LM " << rocket.tLM - 273.15
                             << ", " << alumina.tLM - 273.15 << ", " << jet.tLM - 273.15 << " C; the plumes' heat "
                             << Airflow::rocketHeat(1, V, 220.0) / 1e6 << ", " << Airflow::rocketHeat(2, V, 220.0) / 1e6 << " MJ/kg");
    CHECK(rocket.G > jet.G);
    CHECK(rocket.tLM > jet.tLM);
    CHECK(rocket.tLM - jet.tLM < 6.0);
    CHECK(rocket.forms);
    CHECK(alumina.G < jet.G);
    CHECK(alumina.tLM < jet.tLM);
    CHECK_FALSE(alumina.forms);
    CHECK(Airflow::iceFormation(sparrow, typical(11500.0)).forms);
    CHECK_FALSE(Airflow::iceFormation(amraam, typical(6000.0)).forms);
    CHECK_FALSE(Airflow::laySmoke(amraam, 6000.0, shear, typical, vsg::dvec3(), 0.0).ice.alive);
    // nothing with no thrust
    CHECK_FALSE(Airflow::iceFormation(Airflow::Smoke{1, 0.0, V, exitRadius}, typical(10500.0)).forms);

    // its trail: a reduced-smoke motor's haze turned into a white trail by its ice at 10.5 km, the alumina's at 11.5
    for (const auto* m : {&amraam, &sparrow}) {
        auto s = Airflow::laySmoke(*m, m == &amraam ? 10500.0 : 11500.0, shear, typical, vsg::dvec3(), 0.0);
        REQUIRE(s.ice.alive);
        double now = 0.0, seen = 0.0;
        const double smokeE = Airflow::drawn(s, typical, 1.0).plumeE - std::max(cocip::extinctionPerMetre(s.ice), 0.0);
        const auto first = Airflow::drawn(s, typical, 1.0);
        INFO((m == &amraam ? "AIM-120C" : "AIM-7M") << ": the trail's optical depth at 1 s " << first.plumeE / s.plume.width
                                                   << ", its smoke's " << smokeE / s.plume.width);
        CHECK(first.plumeE / s.plume.width > 0.3); // white
        CHECK(first.plumeE > 2.0 * smokeE);
        // on a typical day (short of ice saturation) its ice sublimates as the trail takes in the air, as a jet's
        // short contrail does, and leaves the smoke
        while (now < 600.0 && s.ice.alive) {
            now += 0.5;
            Airflow::age(s, typical, now);
            if (s.ice.alive && s.ice.tau >= 0.02) seen = now;
        }
        INFO("its ice seen for " << seen << " s, gone at " << now << " s");
        CHECK(seen > 0.5);
        CHECK(now < 600.0);
        CHECK_FALSE(s.ice.alive);
        CHECK(Airflow::drawn(s, typical, now).plumeE == s.smokePrimaryE + s.smokeSecondaryE);
    }

    // in ice-supersaturated air (110 % over ice) it persists and spreads, as a persistent contrail does
    const auto wet = airAround(10500.0, 24500.0, 273.15 - 53.0, 0.0065, 1.1);
    auto s = Airflow::laySmoke(amraam, 10500.0, shear, wet, vsg::dvec3(), 0.0);
    REQUIRE(s.ice.alive);
    double now = 0.0;
    while (now < 900.0) {
        now += 0.5;
        Airflow::age(s, wet, now);
    }
    const auto w = Airflow::drawn(s, wet, now);
    INFO("after 15 minutes: " << s.ice.width << " m wide, optical depth " << s.ice.tau << ", fallen " << w.plumeDrop << " m");
    CHECK(s.ice.alive);
    CHECK(s.ice.tau > 0.02);
    CHECK(s.ice.width > 100.0);
    CHECK(w.plumeDrop > 0.0); // its ice settles
}

TEST_CASE("a detonation's and a fuel fire's fireball against measured explosions", "[contrail-observed][detonation]") {
    namespace flowfires = fsim::world::flowfires;
    const auto near = [](double a, double b, double tol) { return std::abs(a - b) <= tol; };
    // the fireball's greatest diameter: 5.89 m for 1 kg of TNT and 12.05 m for 10 kg (high-speed photography, sea
    // level, fired at the ground: hemispheres) - in free air a sphere holding as much, 2^(-1/3) as wide; larger where
    // the air is thinner (the products expand to its pressure)
    CHECK(near(flowfires::detonationDiameter(1.0, 101325.0) * std::cbrt(2.0), 5.89, 0.05));
    CHECK(near(flowfires::detonationDiameter(10.0, 101325.0) * std::cbrt(2.0), 12.05, 0.25));
    const double p6km = flowfires::pressureAt(flowfires::Air{}, 6000.0);
    CHECK(near(p6km, 47181.0, 470.0)); // (the standard atmosphere's)
    CHECK(near(flowfires::detonationDiameter(8.0, p6km) / flowfires::detonationDiameter(8.0, 101325.0), std::cbrt(101325.0 / p6km), 1e-9));
    CHECK(flowfires::detonationDiameter(0.0, p6km) == 0.0); // (no explosive, nothing)
    // a fuel fireball (CCPS; Roberts 1981): 5.8 M^(1/3) m across and 0.45 M^(1/3) s long for M kg of fuel, larger in
    // thinner air as the detonation's
    CHECK(near(flowfires::ccpsDiameter(1.0, 101325.0), 5.8, 1e-9));
    CHECK(near(flowfires::ccpsDiameter(20.0, 101325.0), 5.8 * std::cbrt(20.0), 1e-9));
    CHECK(near(flowfires::ccpsDiameter(20.0, p6km) / flowfires::ccpsDiameter(20.0, 101325.0), std::cbrt(101325.0 / p6km), 1e-9));
    CHECK(near(flowfires::ccpsDuration(20.0), 0.45 * std::cbrt(20.0), 1e-9));
    CHECK(flowfires::ccpsDiameter(0.0, p6km) == 0.0);
}

TEST_CASE("a hit aircraft's fire: the flame and the soot it streams", "[contrail-observed][detonation]") {
    const auto near = [](double a, double b, double tol) { return std::abs(a - b) <= tol; };
    // the trail: what burns each second over the metres flown in it - 2 kg/s at 220 m/s, 2.7 m^2 of soot's absorption
    // per metre of trail, from a section 1.5 m across: near opaque where it leaves (as the T-33's)
    const cocip::Shear shear{0.002, 0.0013};
    const auto air = [](double) { return cocip::Air{}; };
    const auto s = Airflow::layFire(2.0, 220.0, 6000.0, shear, air, vsg::dvec3(), 0.0);
    CHECK(s.fire);
    CHECK(near(s.smokePrimaryE, 2.0 * Airflow::kFireSootYield * Airflow::kSootAbsorption / 220.0, 1e-9));
    CHECK(near(s.albedo, Airflow::kSootAlbedo, 1e-12));
    const double sigma = std::sqrt(0.125) * Airflow::kFireWidth;
    CHECK(s.smokePrimaryE / (std::sqrt(2.0 * 3.141592653589793) * sigma) > 1.5); // (its optical depth across the middle)
    // the flame: its young part, at a jet fire's temperature, cooling to nothing a few hundredths of a second later
    CHECK(Airflow::flameAt(0.0) == Airflow::kFlameK);
    CHECK(Airflow::flameAt(Airflow::kFlameS * 0.99) == Airflow::kFlameK);
    CHECK(Airflow::flameAt(Airflow::kFlameS + 0.2) == 0.0);
    CHECK(Airflow::flameAt(-1.0) == 0.0);

    // drawn flickering about its mean, its mean sigma T^4 a black body's at the radiating temperature (Gauss-Hermite)
    const double mean = SunlitBlackbody::meanOfFlicker(1500.0, kFlicker);
    const double nodes[3] = {-std::sqrt(3.0), 0.0, std::sqrt(3.0)}, weights[3] = {1.0 / 6.0, 4.0 / 6.0, 1.0 / 6.0};
    double t4 = 0.0;
    for (int i = 0; i < 3; ++i) t4 += weights[i] * std::pow(mean * (1.0 + kFlicker * nodes[i]), 4.0);
    CHECK(near(std::pow(t4, 0.25), 1500.0, 1e-6));
    // the table the shaders read: the sun's own temperature as bright as the sun, white
    const SunlitBlackbody table;
    const vsg::dvec4 sun = table.at(5778.0);
    CHECK(near(sun.w, 1.0, 0.01));
    CHECK(table.at(400.0).w == 0.0);
    CHECK(table.at(1500.0).x > table.at(1500.0).z); // (a flame's red over its blue)
}
