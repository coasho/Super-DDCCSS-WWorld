// Radar cross sections at run time (ADR-30, MS-3): the tables' format and
// lookup - the scattering matrix's moments to any pair of linear
// polarisations, the bins, the bands between - a target's RCS through the
// World as another aircraft's radar sees it (its aspect and polarisation in
// the target's axes), and every aircraft's table against the published
// figures its signature card lists.
#include "mode_flights.h"

#include "core/Json.h"
#include "ms/rcs/Table.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

using namespace fsim;
using namespace fsim::modes;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
namespace fs = std::filesystem;

namespace {

const fs::path kSource = fs::path(FSIM_TEST_JSBSIM_ROOT).parent_path().parent_path();

double db(double x) { return 10.0 * std::log10(x); }

/// The direction of an aspect (deg) in the body axes.
void direction(double azDeg, double elDeg, double d[3]) {
    const double a = azDeg * kDeg, e = elDeg * kDeg;
    d[0] = std::cos(e) * std::cos(a), d[1] = std::cos(e) * std::sin(a), d[2] = -std::sin(e);
}

} // namespace

TEST_CASE("rcs tables: moments to polarisations, bins, bands between, and the file", "[ms][rcs]") {
    ms::rcs::Moments m;
    m.tt = 4.0f, m.pp = 1.0f, m.tp = 0.25f, m.ttppRe = 1.5f, m.fluctuation = 2.0f;
    CHECK_THAT(ms::rcs::sigmaOf(m, 0, 0), WithinAbs(4.0, 1e-6));             // theta in, theta out
    CHECK_THAT(ms::rcs::sigmaOf(m, kPi / 2, kPi / 2), WithinAbs(1.0, 1e-6)); // phi
    CHECK_THAT(ms::rcs::sigmaOf(m, 0, kPi / 2), WithinAbs(0.25, 1e-6));      // crossed
    // at 45 deg both ways: (S_tt + 2 S_tp + S_pp) / 2, its mean |.|^2 from the moments
    CHECK_THAT(ms::rcs::sigmaOf(m, kPi / 4, kPi / 4), WithinAbs(0.25 * (4.0 + 1.0 + 4 * 0.25 + 2 * 1.5), 1e-6));

    ms::rcs::Table t;
    t.aircraft = "test";
    t.binDeg = 3.0, t.azimuthBins = 120, t.elevationBins = 61;
    for (double f : {3e9, 12e9}) {
        ms::rcs::Band b;
        b.frequencyHz = f;
        b.bins.resize(120 * 61);
        for (int e = 0; e < 61; ++e)
            for (int a = 0; a < 120; ++a) {
                auto& x = b.bins[static_cast<std::size_t>(e * 120 + a)];
                x.tt = x.pp = static_cast<float>((f < 5e9 ? 1.0 : 100.0) * (1 + a)); // (each azimuth its own)
                x.fluctuation = 1.0f;
            }
        t.bands.push_back(std::move(b));
    }
    t.provenance = R"({"method":"a test"})";
    double d[3];
    direction(30.0, 0.0, d);
    int az, el;
    t.bin(d, az, el);
    CHECK(az == 10);
    CHECK(el == 30);
    direction(-3.0, 45.0, d); // (azimuths wrap; elevations up are above)
    t.bin(d, az, el);
    CHECK(az == 119);
    CHECK(el == 45);
    direction(30.0, 0.0, d);
    CHECK_THAT(t.sigma(d, 3e9, 0, 0), WithinRel(11.0, 1e-6));
    CHECK_THAT(t.sigma(d, 12e9, 0, 0), WithinRel(1100.0, 1e-6));
    CHECK_THAT(t.sigma(d, 6e9, 0, 0), WithinRel(110.0, 1e-6)); // (halfway in log frequency: halfway in log sigma)
    CHECK_THAT(t.sigma(d, 1e9, 0, 0), WithinRel(11.0, 1e-6));  // (beyond the bands: the nearest)

    const fs::path file = fs::temp_directory_path() / "fsim-rcs-test.bin";
    std::string error;
    REQUIRE(ms::rcs::writeTable(t, file, &error));
    const auto back = ms::rcs::readTable(file, &error);
    REQUIRE(back != nullptr);
    CHECK(back->aircraft == "test");
    CHECK(back->bands.size() == 2);
    CHECK(back->provenance == R"({"method":"a test"})");
    CHECK_THAT(back->sigma(d, 6e9, 0, 0), WithinRel(110.0, 1e-6));
    fs::remove(file);
    CHECK(ms::rcs::readTable(kSource / "README.md", &error) == nullptr);
}

TEST_CASE("rcs through the World: an F-16C as another's radar sees it, head on and rolled", "[ms][rcs]") {
    session::World w(options("rcs-world"));
    const auto target = wing(w, "f16c", 5000.0, 200.0);           // east at 5 km
    session::VehicleSpec s;
    s.name = "radar";
    s.type = "jsbsim:f16c";
    s.initial.altitudeMslM = 5000.0;
    s.initial.headingDeg = 270.0;
    s.initial.airspeedTrueMs = 200.0;
    s.initial.longitudeDeg += 20e3 / (kEarthM * std::cos(s.initial.latitudeDeg * kDeg)) / kDeg; // 20 km ahead of it
    const auto radar = w.createVehicle(s);
    REQUIRE(radar != 0);
    const auto table = w.rcsTable("f16c");
    REQUIRE(table != nullptr);
    const auto head = w.radarCrossSection(radar, target, ms::RcsQuery{});
    REQUIRE(head.has_value());
    CHECK(head->modelled);
    CHECK(std::fabs(head->aspectAzimuthRad) < 2 * kDeg);
    CHECK(std::fabs(head->aspectElevationRad) < 3 * kDeg); // (its trim's angle of attack: the radar a little below its nose line)
    double d[3];
    direction(head->aspectAzimuthRad / kDeg, head->aspectElevationRad / kDeg, d);
    // a vertically polarised radar on a level target: the table's theta-theta; horizontal: phi-phi
    CHECK_THAT(head->sigmaM2, WithinRel(table->sigma(d, 9.5e9, 0, 0), 0.02));
    ms::RcsQuery h;
    h.transmitPolarizationRad = kPi / 2;
    CHECK_THAT(w.radarCrossSection(radar, target, h)->sigmaM2, WithinRel(table->sigma(d, 9.5e9, kPi / 2, kPi / 2), 0.02));
    // nose-on the table's frontal mean is a few square metres
    CHECK(head->sigmaM2 > 0.3);
    CHECK(head->sigmaM2 < 30.0);
    // unknown: none
    CHECK_FALSE(w.radarCrossSection(radar, 999, ms::RcsQuery{}).has_value());
    // a type with no table: unmodelled
    const auto c172 = wing(w, "c172x", 1500.0, 55.0, 2);
    const auto none = w.radarCrossSection(radar, c172, ms::RcsQuery{});
    REQUIRE(none.has_value());
    CHECK(none->modelled == (w.rcsTable("c172x") != nullptr)); // (the C172's design draws the stock one: its table if built)
}

TEST_CASE("rcs signature cards: features from evidence, the same treatments for all, the published figures compared", "[ms][rcs]") {
    // The treatments' values, one set for every aircraft, each with its basis and source
    std::ifstream tin(kSource / "assets/signature/treatments.json");
    REQUIRE(tin);
    std::stringstream tss;
    tss << tin.rdbuf();
    const auto treatments = core::Json::parse(tss.str(), "treatments.json");
    for (const char* key : {"ram_full_loss_db", "ram_partial_loss_db", "ram_low_frequency", "intake_concealed_loss_db"}) {
        const core::Json& p = treatments.child("parameters").child(key);
        CAPTURE(key);
        CHECK(p.has("value"));
        CHECK(treatments.child("sources").has(p.string("source", "")));
        CHECK(!p.string("basis", "").empty());
    }
    int compared = 0;
    for (const auto& dir : fs::directory_iterator(kSource / "aircraft")) {
        const fs::path card = dir.path() / "signature.json";
        if (!fs::exists(card)) continue;
        std::ifstream in(card);
        std::stringstream ss;
        ss << in.rdbuf();
        const auto doc = core::Json::parse(ss.str(), card.string());
        const std::string name = dir.path().filename().string();
        CAPTURE(name);
        CHECK(!doc.has("parameters")); // (no per-aircraft treatment: nothing fitted to a claim)
        // every feature present is stated by its source, quoted; a feature not stated is absent
        for (const auto& [key, f] : doc.child("features").asObject()) {
            CAPTURE(key);
            const bool present = f.find("value") && (f.child("value").isBool() ? f.child("value").asBool() : f.child("value").asString() != "none");
            if (!present) continue;
            CHECK(doc.child("sources").has(f.string("source", "")));
            CHECK(!f.string("evidence", "").empty());
        }
        // the figures reported for it: who reported them and how, against its table (none fitted)
        const fs::path table = dir.path() / "rcs.bin";
        if (!fs::exists(table)) continue;
        std::string error;
        const auto t = ms::rcs::readTable(table, &error);
        REQUIRE(t != nullptr);
        for (const core::Json& v : doc.child("reported").asArray()) {
            CHECK(!v.string("origin", "").empty());
            const double az0 = v.number("azimuth_deg", 0), el0 = v.number("elevation_deg", 0), half = std::max(v.number("window_deg", 20) / 2, 1.5);
            const double f = v.number("frequency_hz", 9.5e9);
            const double lo = v.child("sigma_m2").asArray()[0].asNumber(), hi = v.child("sigma_m2").asArray()[1].asNumber();
            double sum = 0;
            int n = 0;
            for (double el = -90; el <= 90; el += t->binDeg)
                for (double az = 0; az < 360; az += t->binDeg) {
                    if (std::fabs(std::remainder(az - az0, 360.0)) > half + 1e-9 || std::fabs(el - el0) > half + 1e-9) continue;
                    double d[3];
                    direction(az, el, d);
                    sum += 0.5 * (t->sigma(d, f, 0, 0) + t->sigma(d, f, kPi / 2, kPi / 2)), ++n;
                }
            REQUIRE(n > 0);
            const double mean = sum / n, mid = std::sqrt(lo * hi);
            WARN(name << " " << v.string("aspect", "") << ": table " << mean << " m^2 (" << db(mean) << " dBsm); " << v.string("origin", "") << " "
                      << lo << "-" << hi << " m^2 (" << v.string("source", "") << "): " << db(mean) - db(mid) << " dB from its middle");
            ++compared;
        }
    }
    WARN(compared << " published figures compared");
}

TEST_CASE("rcs tables: an untreated fighter near its published figure", "[ms][rcs]") {
    // the bare-metal Su-27S, no treatment of any kind: frontal 10-15 m^2 as reported, within the model's own uncertainty (6 dB)
    const fs::path table = kSource / "aircraft/su27s/rcs.bin";
    if (!fs::exists(table)) SKIP("no Su-27S table");
    std::string error;
    const auto t = ms::rcs::readTable(table, &error);
    REQUIRE(t != nullptr);
    double sum = 0;
    int n = 0;
    for (double el = -9; el <= 9; el += 3)
        for (double az = -9; az <= 9; az += 3) {
            double d[3];
            direction(az, el, d);
            sum += 0.5 * (t->sigma(d, 9.5e9, 0, 0) + t->sigma(d, 9.5e9, kPi / 2, kPi / 2)), ++n;
        }
    const double mean = sum / n;
    CAPTURE(mean);
    CHECK(db(mean) > db(10.0) - 6.0);
    CHECK(db(mean) < db(15.0) + 6.0);
}
