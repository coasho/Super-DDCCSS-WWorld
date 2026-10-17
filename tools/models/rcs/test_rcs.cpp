// The RCS engine (ADR-30, MS-3; tools/models/rcs) against the closed forms of
// canonical targets (Knott, Shaeffer and Tuley, "Radar Cross Section", table
// 6.1): a flat plate broadside and at its first side lobe, a sphere, a
// cylinder broadside, a dihedral (two bounces, its polarisations opposed) and
// a trihedral (three). On a GPU with ray queries; skipped without one.
#include "Engine.h"
#include "Model.h"
#include "Shapes.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <memory>
#include <string>

using namespace fsim::rcs;
using Catch::Matchers::WithinAbs;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr double kF = 10e9;
constexpr double kLambda = 299792458.0 / kF;

double db(double x) { return 10.0 * std::log10(std::max(x, 1e-30)); }

Engine* engine() {
    static std::unique_ptr<Engine> e = [] {
        std::string error;
        auto made = Engine::create(&error);
        if (!made) WARN("no RCS engine: " << error);
        return made;
    }();
    return e.get();
}

Scattering at(const Mesh& m, double azDeg, double elDeg, Settings s = {}) {
    std::string error;
    REQUIRE(engine()->load(m, &error));
    return engine()->evaluate({Aspect{azDeg * kDeg, elDeg * kDeg}}, kF, s)[0];
}

} // namespace

TEST_CASE("rcs: a bay fighter's model as the radar sees it - its bays shut under their covers, no doors, no breakup caps", "[rcs]") {
    // (the doors' gaps are the viewer's, for them to turn: seen through them the bay was a cavity returning more than a
    // stealth airframe's front; and a breakup section's cap over its break lies inside the airframe, no surface of it -
    // read as a cavity's wall it raised an F-16C's diffuse return 12 %)
    Mesh mesh;
    ModelReport report;
    std::string error;
    REQUIRE(readModel(std::string(FSIM_SOURCE_DIR) + "/aircraft/f22a/f22a.glb", mesh, &report, &error));
    int covers = 0;
    for (const ModelPart& p : report.parts) {
        CAPTURE(p.name);
        CHECK(p.name.rfind("fsim:bay:", 0) != 0);
        if (p.name.rfind("fsim:cover:", 0) == 0 && p.triangles > 0) ++covers;
    }
    CHECK(covers == 3); // the main bay and the two side bays
    CHECK(report.skippedTriangles > 1000); // its caps (and its doors)
}

TEST_CASE("rcs: a flat plate broadside, 4 pi A^2 / lambda^2, and its first side lobe 13.26 dB down", "[rcs]") {
    if (!engine()) SKIP("no GPU with ray queries");
    const double a = 0.5;
    const double sigma0 = 4 * kPi * a * a * a * a / (kLambda * kLambda);
    const Mesh plate = shapes::plate(a);
    const Scattering s = at(plate, 0, 0);
    CAPTURE(db(std::norm(s.tt)), db(std::norm(s.pp)), db(sigma0));
    CHECK_THAT(db(std::norm(s.tt)), WithinAbs(db(sigma0), 0.1));
    CHECK_THAT(db(std::norm(s.pp)), WithinAbs(db(sigma0), 0.1));
    CHECK(std::norm(s.tp) < sigma0 * 1e-4);
    // PO in the plane of the azimuth: sigma0 sinc^2(k a sin theta) cos^2 theta - its first side lobe
    const double k = 2 * kPi / kLambda, theta = std::asin(4.4934 / (k * a));
    const Scattering side = at(plate, theta / kDeg, 0);
    const double expected = sigma0 * std::pow(std::sin(4.4934) / 4.4934, 2) * std::pow(std::cos(theta), 2);
    CAPTURE(db(std::norm(side.tt)), db(expected));
    CHECK_THAT(db(std::norm(side.tt)), WithinAbs(db(expected), 0.25));
}

TEST_CASE("rcs: a sphere, pi a^2 from any side", "[rcs]") {
    if (!engine()) SKIP("no GPU with ray queries");
    const double a = 0.5;
    const Mesh sphere = shapes::sphere(a);
    for (const auto& [az, el] : {std::pair{0.0, 0.0}, {37.0, 21.0}, {130.0, -60.0}}) {
        const Scattering s = at(sphere, az, el);
        CAPTURE(az, el, db(std::norm(s.tt)), db(std::norm(s.pp)), db(kPi * a * a));
        CHECK_THAT(db(std::norm(s.tt)), WithinAbs(db(kPi * a * a), 0.3));
        CHECK_THAT(db(std::norm(s.pp)), WithinAbs(db(kPi * a * a), 0.3));
    }
}

TEST_CASE("rcs: a cylinder broadside, 2 pi a l^2 / lambda", "[rcs]") {
    if (!engine()) SKIP("no GPU with ray queries");
    const double a = 0.25, l = 1.0;
    const Scattering s = at(shapes::cylinder(a, l), 0, 0);
    const double expected = 2 * kPi * a * l * l / kLambda;
    CAPTURE(db(std::norm(s.tt)), db(std::norm(s.pp)), db(expected));
    CHECK_THAT(db(std::norm(s.tt)), WithinAbs(db(expected), 0.25));
    CHECK_THAT(db(std::norm(s.pp)), WithinAbs(db(expected), 0.25));
}

TEST_CASE("rcs: a dihedral, 8 pi a^4 / lambda^2 by two bounces, its polarisations returned opposed", "[rcs]") {
    if (!engine()) SKIP("no GPU with ray queries");
    const double a = 0.3;
    const double expected = 8 * kPi * a * a * a * a / (kLambda * kLambda);
    const Scattering s = at(shapes::dihedral(a), 0, 0);
    CAPTURE(db(std::norm(s.tt)), db(std::norm(s.pp)), db(expected));
    CHECK_THAT(db(std::norm(s.tt)), WithinAbs(db(expected), 0.25));
    CHECK_THAT(db(std::norm(s.pp)), WithinAbs(db(expected), 0.25));
    CHECK((s.tt * std::conj(s.pp)).real() < 0.0); // (the field across the seam turns over once more than along it)
    // one bounce alone: nearly nothing comes back
    Settings one;
    one.maxBounces = 1;
    CHECK(db(std::norm(at(shapes::dihedral(a), 0, 0, one).tt)) < db(expected) - 15.0);
}

TEST_CASE("rcs: a trihedral, 12 pi a^4 / lambda^2 by three bounces", "[rcs]") {
    if (!engine()) SKIP("no GPU with ray queries");
    const double a = 0.2;
    const double expected = 12 * kPi * a * a * a * a / (kLambda * kLambda);
    const Scattering s = at(shapes::trihedral(a), 0, 0);
    CAPTURE(db(std::norm(s.tt)), db(std::norm(s.pp)), db(expected));
    CHECK_THAT(db(std::norm(s.tt)), WithinAbs(db(expected), 0.25));
    CHECK_THAT(db(std::norm(s.pp)), WithinAbs(db(expected), 0.25));
}
