// The radio path (ADR-30, MS-2; src/simulation/mission_systems/rf): NTIA's P.528 and ITM against
// their own published example values, P.838-3 against its table 5, antenna
// patterns against the closed forms of their apertures, P.526's knife edge,
// and the path's assembly - terrain masking, the terrain's excess, rain.
#include "fsim/GroundProvider.h"
#include "simulation/mission_systems/rf/Antenna.h"
#include "simulation/mission_systems/rf/Ntia.h"
#include "simulation/mission_systems/rf/Path.h"
#include "simulation/mission_systems/rf/Rain.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <chrono>
#include <random>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace fsim::ms;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
namespace fs = std::filesystem;

namespace {

const fs::path kSource = fs::path(FSIM_TEST_JSBSIM_ROOT).parent_path().parent_path();
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

std::vector<std::vector<double>> readCsv(const fs::path& file, bool header) {
    std::ifstream in(file);
    std::vector<std::vector<double>> rows;
    std::string line;
    if (header) std::getline(in, line);
    while (std::getline(in, line)) {
        std::vector<double> row;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) if (!cell.empty() && cell != "\r") row.push_back(std::stod(cell));
        if (!row.empty()) rows.push_back(row);
    }
    return rows;
}

/// A Gaussian ridge across the path, on sea-level ground.
class Ridge final : public fsim::sim::GroundProvider {
public:
    Ridge(double latRad, double heightM, double halfWidthRad) : lat_(latRad), h_(heightM), w_(halfWidthRad) {}
    double heightAboveEllipsoidM(double lat, double) const override {
        const double x = (lat - lat_) / w_;
        return h_ * std::exp(-x * x);
    }
    double resolutionM() const noexcept override { return 30.0; }

private:
    double lat_, h_, w_;
};

} // namespace

TEST_CASE("P.528: NTIA's reference implementation reproduces its published example values", "[ms][rf]") {
    // (third_party/ntia/p528/README.md, "Example Values")
    struct Case { double dKm, h1, h2, fMhz; int pol; double time, lossDb; };
    const Case cases[] = {{15, 10, 1000, 500, 0, 50, 110.0}, {100, 100, 15000, 3600, 0, 90, 151.6},
                          {1500, 15, 10000, 5700, 0, 10, 293.4}, {30, 8, 20000, 22000, 1, 50, 151.1}};
    for (const auto& c : cases) {
        ntia::P528Result r;
        const int rc = ntia::p528(c.dKm, c.h1, c.h2, c.fMhz, c.pol, c.time, r);
        CAPTURE(c.dKm, c.fMhz, r.lossDb);
        CHECK((rc == 0 || rc == 11));
        CHECK_THAT(r.lossDb, WithinAbs(c.lossDb, 0.05));
    }
}

TEST_CASE("ITM: NTIA's reference implementation reproduces its point-to-point examples", "[ms][rf]") {
    const auto cases = readCsv(kSource / "third_party/ntia/itm/p2p.csv", true);
    const auto profiles = readCsv(kSource / "third_party/ntia/itm/pfls.csv", false);
    REQUIRE(cases.size() == 5);
    REQUIRE(profiles.size() == cases.size());
    for (std::size_t i = 0; i < cases.size(); ++i) {
        const auto& c = cases[i];
        ntia::ItmResult r;
        const int rc = ntia::itmP2p(c[0], c[1], profiles[i].data(), static_cast<int>(c[7]), c[4], c[5], static_cast<int>(c[6]), c[2], c[3],
                                        static_cast<int>(c[11]), c[8], c[9], c[10], r);
        CAPTURE(i, r.lossDb);
        CHECK(rc <= 1);
        CHECK_THAT(r.lossDb, WithinAbs(c[12], 0.01));
    }
}

TEST_CASE("P.838-3: the coefficients' regressions reproduce its table 5", "[ms][rf]") {
    // (Recommendation ITU-R P.838-3, table 5: f GHz, kH, alphaH, kV, alphaV)
    const double table[][5] = {{1, 0.0000259, 0.9691, 0.0000308, 0.8592},   {10, 0.01217, 1.2571, 0.01129, 1.2156},
                               {12, 0.02386, 1.1825, 0.02455, 1.1216},      {20, 0.09164, 1.0568, 0.09611, 0.9847},
                               {30, 0.2403, 0.9485, 0.2291, 0.9129},        {40, 0.4431, 0.8673, 0.4274, 0.8421}};
    for (const auto& t : table) {
        double kh, ah, kv, av;
        rf::rainCoefficientsHV(t[0], kh, ah, kv, av);
        CAPTURE(t[0], kh, ah, kv, av);
        CHECK_THAT(kh, WithinRel(t[1], 0.005));
        CHECK_THAT(ah, WithinAbs(t[2], 0.0006));
        CHECK_THAT(kv, WithinRel(t[3], 0.005));
        CHECK_THAT(av, WithinAbs(t[4], 0.0006));
    }
    // (4), (5): a horizontal path, horizontally polarised, is k_H and alpha_H; circular is their mean for k
    double kh, ah, kv, av;
    rf::rainCoefficientsHV(10, kh, ah, kv, av);
    CHECK_THAT(rf::rainCoefficients(10, 0, 0).k, WithinRel(kh, 1e-12));
    CHECK_THAT(rf::rainCoefficients(10, 0, kPi / 2).alpha, WithinRel(av, 1e-12));
    CHECK_THAT(rf::rainCoefficients(10, 0, kPi / 4).k, WithinRel((kh + kv) / 2, 1e-12));
    CHECK_THAT(rf::rainSpecificDbPerKm(10, 25, 0, 0), WithinRel(kh * std::pow(25.0, ah), 1e-12));
}

TEST_CASE("Antenna patterns: the closed forms of their apertures", "[ms][rf]") {
    const double f = 10e9, lambda = 299792458.0 / f;
    rf::ApertureSpec s;
    s.widthM = 0.7, s.heightM = 0.5;
    s.floorDbi = -100.0;
    {
        const rf::Pattern p(s, f); // uniform: sinc
        CHECK_THAT(p.firstSidelobeDb(), WithinAbs(-13.26, 0.03));
        CHECK_THAT(p.beamwidthRad(), WithinRel(2.0 * std::asin(0.8859 / 2.0 * lambda / s.widthM), 0.003));
        CHECK_THAT(p.beamwidthRad(true), WithinRel(2.0 * std::asin(0.8859 / 2.0 * lambda / s.heightM), 0.003));
        CHECK_THAT(p.taperEfficiency(), WithinAbs(1.0, 1e-9));
        CHECK_THAT(p.peakGainDbi(), WithinAbs(10.0 * std::log10(4.0 * kPi * s.widthM * s.heightM / (lambda * lambda)), 1e-9));
        const double ahead[3] = {1, 0, 0};
        CHECK_THAT(p.gainDbi(ahead), WithinAbs(p.peakGainDbi(), 1e-6));
    }
    {
        s.taper = rf::Taper::Cosine; // cos(pi x): -23.0 dB, 1.189 lambda/L, 8/pi^2 a plane
        const rf::Pattern p(s, f);
        CHECK_THAT(p.firstSidelobeDb(), WithinAbs(-23.0, 0.05));
        CHECK_THAT(p.beamwidthRad(), WithinRel(2.0 * std::asin(1.189 / 2.0 * lambda / s.widthM), 0.003));
        CHECK_THAT(p.taperEfficiency(), WithinRel(std::pow(8.0 / (kPi * kPi), 2.0), 1e-6));
    }
    {
        s.taper = rf::Taper::Taylor; // designed for 30 dB side lobes, n-bar 5
        s.taylorSidelobeDb = 30.0, s.taylorNbar = 5;
        const rf::Pattern p(s, f);
        // (its near side lobes at the design level, the first a fraction of a dB below it)
        CHECK(p.firstSidelobeDb() <= -29.95);
        CHECK(p.firstSidelobeDb() >= -30.5);
    }
    {
        rf::ApertureSpec c; // a uniform circular aperture: the Airy pattern, -17.57 dB, 1.029 lambda/D
        c.shape = rf::ApertureShape::Circular, c.widthM = 0.9, c.floorDbi = -100.0;
        const rf::Pattern p(c, f);
        CHECK_THAT(p.firstSidelobeDb(), WithinAbs(-17.57, 0.03));
        CHECK_THAT(p.beamwidthRad(), WithinRel(2.0 * std::asin(1.029 / 2.0 * lambda / c.widthM), 0.003));
    }
    {
        // An electronically scanned array: steered 60 deg, its gain falls by cos^1.5 and its beam broadens by 1/cos
        rf::ApertureSpec e = s;
        e.taper = rf::Taper::Uniform, e.electronic = true, e.scanExponent = 1.5;
        const rf::Pattern p(e, f);
        const double sy = std::sin(60 * kDeg);
        const double steered[3] = {std::cos(60 * kDeg), sy, 0};
        CHECK_THAT(p.gainDbi(steered, sy, 0.0), WithinAbs(p.peakGainDbi() + 15.0 * std::log10(0.5), 1e-6));
        // half power in sine space: u = 0.443 lambda/L either side
        const double du = 0.8859 / 2.0 * lambda / e.widthM;
        const double edge = std::asin(sy + du) - 60 * kDeg;
        const double atEdge[3] = {std::cos(60 * kDeg + edge), std::sin(60 * kDeg + edge), 0};
        CHECK_THAT(p.gainDbi(atEdge, sy, 0.0) - p.gainDbi(steered, sy, 0.0), WithinAbs(-3.01 + 15.0 * std::log10(std::cos(60 * kDeg + edge) / 0.5), 0.02));
        CHECK(edge > 1.9 * std::asin(du)); // (the broadside half-beam, near doubled at 60 deg)
        const double behind[3] = {-1, 0, 0};
        CHECK(p.gainDbi(behind) == e.backDbi);
    }
}

TEST_CASE("P.526: the knife edge and Bullington's construction", "[ms][rf]") {
    CHECK(rf::knifeEdgeLossDb(-0.78) == 0.0);
    CHECK_THAT(rf::knifeEdgeLossDb(0.0), WithinAbs(6.03, 0.01));  // (the grazing edge: 6 dB)
    CHECK_THAT(rf::knifeEdgeLossDb(1.0), WithinAbs(13.93, 0.01));
    CHECK_THAT(rf::knifeEdgeLossDb(10.0), WithinAbs(6.9 + 20 * std::log10(std::sqrt(9.9 * 9.9 + 1) + 9.9), 1e-9));
    // One edge, grazing the straight ray on a flat earth (an infinite radius): v = 0
    const double h[] = {0, 0, 100, 0, 0};
    const double luc = rf::knifeEdgeLossDb(0.0);
    CHECK_THAT(rf::bullingtonDb(h, 5, 1000.0, 100.0, 100.0, 0.03, 1e30), WithinAbs(luc + (1 - std::exp(-luc / 6)) * (10 + 0.02 * 4), 1e-9));
}

TEST_CASE("Bullington in one pass, and over the smooth earth in closed form: the reference's answers", "[ms][rf]") {
    // random ridged profiles and terminals - clear, grazing and beyond the horizon - at the radars' wavelengths
    std::mt19937_64 rng(3);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::vector<double> h, flat;
    int clear = 0, obstructed = 0;
    for (int k = 0; k < 3000; ++k) {
        const int n = 3 + static_cast<int>(unit(rng) * 2000);
        const double spacing = 30.0 + unit(rng) * 200.0, lambda = 0.02 + unit(rng) * 0.2;
        const double relief = std::pow(10.0, unit(rng) * 3.5);
        h.assign(static_cast<std::size_t>(n), 0.0), flat.assign(static_cast<std::size_t>(n), 0.0);
        double level = relief * unit(rng);
        for (auto& x : h) x = level = std::max(0.0, level + relief * 0.05 * (unit(rng) - 0.5));
        const double hts = h.front() + std::pow(10.0, unit(rng) * 4.0), hrs = h.back() + std::pow(10.0, unit(rng) * 4.0);
        const double ref = rf::bullingtonDb(h.data(), n, spacing, hts, hrs, lambda, 8.5e6);
        const double fast = rf::bullingtonFastDb(h.data(), n, spacing, hts, hrs, lambda, 8.5e6);
        (ref > 0.0 ? obstructed : clear) += 1;
        INFO("n " << n << " spacing " << spacing << " hts " << hts << " hrs " << hrs);
        REQUIRE_THAT(fast, WithinAbs(ref, 1e-9));
        const double flatRef = rf::bullingtonDb(flat.data(), n, spacing, hts - h.front(), hrs - h.back(), lambda, 8.5e6);
        REQUIRE_THAT(rf::bullingtonFlatDb(n, spacing, hts - h.front(), hrs - h.back(), lambda, 8.5e6), WithinAbs(flatRef, 1e-9));
    }
    CHECK(clear > 300);
    CHECK(obstructed > 300);
}

TEST_CASE("The path: free space and gases high up, terrain masking, the terrain's excess, rain", "[ms][rf]") {
    rf::PathOptions o;
    o.frequencyHz = 10e9;
    {
        // Two fighters 30 km apart at 9 km: line of sight, the loss free space plus a little gas
        const rf::Terminal a{40.0 * kDeg, 0.0, 9000.0}, b{40.0 * kDeg, 30e3 / 6371e3 / std::cos(40.0 * kDeg), 9000.0};
        const auto r = rf::computePath(a, b, o);
        CAPTURE(r.freeSpaceDb, r.basicDb, r.gasDb, r.code);
        CHECK(r.mode == rf::PathMode::LineOfSight);
        CHECK_THAT(r.slantRangeM, WithinRel(30e3, 0.006)); // (30 km of the 6371 km sphere's longitude, on the WGS-84 ellipsoid 9 km up)
        CHECK(r.gasDb > 0.0);
        CHECK(r.gasDb < 0.5);
        CHECK_THAT(r.basicDb, WithinAbs(r.freeSpaceDb + r.gasDb, 1.0));
        CHECK(r.terrainDb == 0.0);
        CHECK_THAT(r.elevationRad[0], WithinAbs(-30e3 / (2 * 6371e3), 0.0005)); // (the other below the local level)
        CHECK_THAT(r.azimuthRad[0], WithinAbs(90 * kDeg, 0.01));
        // reciprocal
        const auto back = rf::computePath(b, a, o);
        CHECK_THAT(back.totalDb, WithinAbs(r.totalDb, 1e-9));
    }
    {
        // Past the radio horizon (143 km for two at 300 m): 150 km apart - diffraction, above free space
        const rf::Terminal a{40.0 * kDeg, 0.0, 300.0}, b{40.0 * kDeg + 150e3 / 6371e3, 0.0, 300.0};
        const auto r = rf::computePath(a, b, o);
        CHECK(r.mode != rf::PathMode::LineOfSight);
        CAPTURE(r.basicDb, r.freeSpaceDb);
        CHECK(r.basicDb > r.freeSpaceDb + 10.0);
    }
    {
        // A 1500 m ridge between two helicopters at 500 m, 40 km apart: masked, tens of dB more
        const double lat0 = 40.0 * kDeg, dLat = 40e3 / 6371e3;
        const Ridge ridge(lat0 + dLat / 2, 1500.0, 1500.0 / 6371e3);
        o.ground = &ridge;
        const rf::Terminal a{lat0, 0.0, 500.0}, b{lat0 + dLat, 0.0, 500.0};
        const auto masked = rf::computePath(a, b, o);
        CAPTURE(masked.clearanceM, masked.terrainDb);
        CHECK(masked.terrainMasked);
        CHECK(masked.terrainDb > 25.0);
        const Ridge low(lat0 + dLat / 2, 50.0, 1500.0 / 6371e3);
        o.ground = &low;
        const auto clear = rf::computePath(a, b, o);
        CHECK_FALSE(clear.terrainMasked);
        CHECK(clear.terrainDb < 1.0);
        // The ITM over a 500 m ridge, asked for: at 300 MHz near the knife edge's answer (it rounds the hill, which costs
        // more as the frequency rises: 29 dB over the edge's at 10 GHz)
        const Ridge mid(lat0 + dLat / 2, 500.0, 1500.0 / 6371e3);
        o.ground = &mid;
        o.frequencyHz = 300e6;
        const auto edge = rf::computePath(a, b, o);
        o.model = rf::PathModel::Itm;
        const auto itm = rf::computePath(a, b, o);
        CAPTURE(itm.basicDb, edge.totalDb, itm.code);
        CHECK(itm.viaItm);
        CHECK_FALSE(edge.viaItm);
        CHECK_THAT(itm.totalDb, WithinAbs(edge.totalDb, 3.0));
        o.model = rf::PathModel::Auto;
        o.frequencyHz = 10e9;
        o.ground = nullptr;
    }
    {
        // 25 mm/h of rain along a level 20 km path below the rain height: k R^alpha a kilometre
        const rf::Terminal a{0.0, 0.0, 1000.0}, b{20e3 / 6371e3, 0.0, 1000.0};
        o.rainRateMmH = 25.0;
        const auto r = rf::computePath(a, b, o);
        double kh, ah, kv, av;
        rf::rainCoefficientsHV(10, kh, ah, kv, av);
        CHECK_THAT(r.rainDb, WithinRel(kh * std::pow(25.0, ah) * r.slantRangeM / 1000.0, 0.01));
        // above the rain height, none
        const rf::Terminal c{0.0, 0.0, 6000.0}, e{20e3 / 6371e3, 0.0, 6000.0};
        CHECK(rf::computePath(c, e, o).rainDb == 0.0);
        o.rainRateMmH = 0.0;
    }
}

TEST_CASE("P.676's tabulated layers and grazing traces: P.528 as the reference implementation computes it, within its own jitter", "[ms][rf]") {
    struct Case { double dKm, h1, h2, fMhz; };
    const Case cases[] = {{15, 10, 1000, 500}, {100, 100, 15000, 3600}, {1500, 15, 10000, 5700}, {30, 8, 20000, 22000},
                          {40, 3000, 9000, 9400}, {250, 1500, 12000, 16000}, {5, 2, 300, 60000 / 2.0}};
    for (const auto& c : cases) {
        ntia::P528Result tab, exact;
        ntia::tabulateLayers(true);
        ntia::p528(c.dKm, c.h1, c.h2, c.fMhz, 0, 50, tab);
        ntia::tabulateLayers(false);
        ntia::p528(c.dKm, c.h1, c.h2, c.fMhz, 0, 50, exact);
        CAPTURE(c.dKm, c.fMhz, tab.lossDb, exact.lossDb, tab.gasDb, exact.gasDb);
        CHECK_THAT(tab.lossDb, WithinAbs(exact.lossDb, 0.002));
        CHECK_THAT(tab.gasDb, WithinAbs(exact.gasDb, 0.002));
    }
    // and over the radars' geometries at random: the layers and the terminals' grazing traces both read from their
    // tables, against the exact computation - and against the exact computation's own answer to its terminals moved a
    // centimetre (it lays its layers afresh for each height, and its searches stop within a metre: a jitter no table
    // smooth in the height can follow, and the yardstick for one)
    std::mt19937_64 rng(5);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    const double bands[] = {1300, 3100, 5600, 9400, 9860, 16000};
    double worst = 0.0, worstGas = 0.0, jitter = 0.0;
    for (int k = 0; k < 300; ++k) {
        const double ha = 1.5 * std::pow(20000.0 / 1.5, unit(rng)), hb = 1.5 * std::pow(20000.0 / 1.5, unit(rng));
        const double h1 = std::min(ha, hb), h2 = std::max(ha, hb);
        const double dKm = 2.0 * std::pow(600.0 / 2.0, unit(rng)), fMhz = bands[k % 6];
        ntia::P528Result tab, exact, moved;
        ntia::tabulateLayers(true);
        ntia::p528(dKm, h1, h2, fMhz, 0, 50, tab);
        ntia::tabulateLayers(false);
        ntia::p528(dKm, h1, h2, fMhz, 0, 50, exact);
        ntia::p528(dKm, h1 + 0.01, h2 + 0.01, fMhz, 0, 50, moved);
        worst = std::max(worst, std::fabs(tab.lossDb - exact.lossDb)), worstGas = std::max(worstGas, std::fabs(tab.gasDb - exact.gasDb));
        jitter = std::max(jitter, std::fabs(moved.lossDb - exact.lossDb));
    }
    ntia::tabulateLayers(true);
    WARN("P.528 tabulated against exact over 300 random paths: loss within " << worst << " dB, gas within " << worstGas
                                                                             << " dB; exact against itself 1 cm up: " << jitter << " dB");
    CHECK(worst < 0.02);
    CHECK(worst < 0.25 * jitter);
    CHECK(worstGas < 0.002);
}

TEST_CASE("P.528: what a path costs", "[ms][rf][.bench]") {
    const auto t0 = std::chrono::steady_clock::now();
    ntia::P528Result r;
    double sum = 0;
    for (int i = 0; i < 200; ++i) {
        ntia::p528(10.0 + i, 3000, 9000, 10000, 0, 50, r);
        sum += r.lossDb;
    }
    const double us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count() / 200;
    WARN("P.528: " << us << " us a path (" << sum << ")");
}
