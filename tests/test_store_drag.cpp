// Carried stores' drag on their aircraft (MS-7g): each store's own (its card's drag_area_m2, hangar storedrag.py) times
// its carriage's interference, the carriage itself (pylon and rack, launcher rail) from the first store on, each against
// the airflow at its station - so it slows the aircraft by its drag over its mass, and one wing's alone yaws it.
#include "mode_flights.h"
#include "ms/Host.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

using namespace fsim;
using namespace fsim::modes;

namespace {

std::uint16_t stationPayload(session::World& w, std::uint32_t id) {
    for (const auto& pl : w.missionSystems(id)->payloads())
        if (pl.family == ms::Family::StoreStation) return pl.index;
    return 0;
}

} // namespace

TEST_CASE("an F-16's three tanks: drag points at their stations - each tank's own with its lugs, and its pylon; "
          "jettisoned, the pylons stay",
          "[stores][drag]") {
    session::World w(options("drag-points"));
    const auto id = wing(w, "f16c", 5000.0, 220.0);
    std::vector<ms::Host::DragPoint> points;
    w.missionSystems(id)->carriageDrag(points);
    CHECK(points.empty()); // (clean: no carriage on it yet)
    REQUIRE(w.loadLoadout(id, "external tanks") == ms::Reason::None);
    w.missionSystems(id)->carriageDrag(points);
    REQUIRE(points.size() == 3);
    double loaded = 0.0;
    for (const auto& p : points) loaded += p.areaM2;
    // the cards' 0.02361 (370 US gal) twice and 0.02122 (300 US gal), each tank's lugs 0.007 and its pylon's 0.024
    CHECK(std::abs(loaded - (2 * 0.02361 + 0.02122 + 3 * (0.007 + 0.024))) < 1e-4);
    ms::JettisonCommand all;
    all.payload = stationPayload(w, id);
    all.emergency = true;
    REQUIRE(w.submit(id, all).accepted());
    w.step(stepsFor(w, 0.5));
    w.missionSystems(id)->carriageDrag(points);
    REQUIRE(points.size() == 3);
    for (const auto& p : points) CHECK(std::abs(p.areaM2 - 0.024) < 1e-9);
}

TEST_CASE("the tanks' drag slows an F-16 by their drag over its mass, and the right wing's tank alone yaws it to the right",
          "[stores][drag][world]") {
    // two worlds alike but for the stores' drag: the difference is the drag's alone
    auto fly = [](bool drag, bool asymmetric, double& speed, double& yawRate, double& predicted) {
        auto o = options(drag ? "drag-on" : "drag-off");
        o.storeDrag = drag;
        session::World w(o);
        const auto id = wing(w, "f16c", 5000.0, 220.0);
        REQUIRE(w.loadLoadout(id, "external tanks") == ms::Reason::None);
        w.step(1);
        if (asymmetric) { // (the left wing's tank and the centreline's off: the right's alone)
            ms::JettisonCommand some;
            some.payload = stationPayload(w, id);
            some.emergency = true;
            for (const auto& s : w.stores(id))
                if (s.station != 6) some.stores.push_back(s.index);
            REQUIRE(w.submit(id, some).accepted());
            w.step(1);
        }
        std::vector<ms::Host::DragPoint> points;
        w.missionSystems(id)->carriageDrag(points);
        double area = 0.0;
        for (const auto& p : points) area += p.areaM2;
        const double q = w.model(id)->property("aero/qbar-psf").get() * 47.880258888889;
        const double kg = w.model(id)->energy().massKg;
        const double v0 = w.vehicleState(id)->airspeedTrueMs;
        w.step(stepsFor(w, 1.0));
        speed = w.vehicleState(id)->airspeedTrueMs - v0;
        yawRate = w.vehicleState(id)->angularRateBodyRadS[2];
        predicted = -q * area / kg * 1.0; // (its deceleration over the second)
    };
    double on = 0.0, off = 0.0, rOn = 0.0, rOff = 0.0, predicted = 0.0, ignored = 0.0;
    fly(true, false, on, rOn, predicted);
    fly(false, false, off, rOff, ignored);
    if (std::getenv("FSIM_DRAG_SHOW")) std::printf("speed change over 1 s: %.4f m/s with drag, %.4f without; predicted %.4f\n", on, off, predicted);
    CHECK(on - off < 0.0);
    CHECK(std::abs((on - off) - predicted) < 0.15 * std::abs(predicted));
    // the right wing's tank and the three pylons: the drag at the right yaws the nose right (r > 0) against the world without
    fly(true, true, on, rOn, predicted);
    fly(false, true, off, rOff, ignored);
    if (std::getenv("FSIM_DRAG_SHOW")) std::printf("yaw rate after 1 s with the right tank alone: %.5f rad/s with drag, %.5f without\n", rOn, rOff);
    CHECK(rOn - rOff > 0.0);
}

TEST_CASE("the stores' drag held to the published drag indices (RAND MR-1216-AF, appendix A): compared, not fitted - within 30 %",
          "[stores][drag]") {
    // a drag index: the stores' added CD x 10^4 on the aircraft's wing area, at Mach 0.8 (RAND's tables do not define it:
    // this as an F-16 engineer gives it); each case the stores' drag points' sum against it
    auto index = [](session::World& w, std::uint32_t id, double wingFt2) {
        std::vector<ms::Host::DragPoint> points;
        w.missionSystems(id)->carriageDrag(points);
        double area = 0.0;
        for (const auto& p : points) area += p.areaM2;
        return area / (wingFt2 * 0.09290304) * 1e4;
    };
    session::World w(options("drag-index"));
    struct Case {
        const char* aircraft;
        double wingFt2;
        std::vector<std::pair<int, const char*>> stores;
        double published;
        const char* what;
    };
    const std::vector<Case> cases = {
        {"f16c", 300.0, {{4, "tank-370gal"}, {6, "tank-370gal"}}, 35.0, "F-16C, 2 x 370 US gal tanks (with their pylons)"},
        {"f15c", 608.0, {{2, "tank-610gal"}, {6, "tank-610gal"}, {10, "tank-610gal"}}, 33.1, "F-15C, 3 x 610 US gal tanks"},
        {"f15c", 608.0, {{4, "aim-120c"}, {5, "aim-120c"}, {7, "aim-120c"}, {8, "aim-120c"}}, 5.2, "F-15C, 4 x AIM-120 (fuselage wells)"},
        {"f16c", 300.0, {{5, "an-alq-184"}}, 29.0, "F-16C, ALQ-184 on its centreline pylon (18 + 11)"},
    };
    int k = 0;
    for (const auto& c : cases) {
        INFO(c.what);
        const auto id = wing(w, c.aircraft, 5000.0, 220.0, k++);
        const std::uint16_t payload = stationPayload(w, id);
        for (const auto& s : c.stores) REQUIRE(w.loadStore(id, payload, s.first, s.second) == ms::Reason::None);
        const double ours = index(w, id, c.wingFt2);
        if (std::getenv("FSIM_DRAG_SHOW")) std::printf("%-44s ours %5.1f, published %5.1f: %.2f\n", c.what, ours, c.published, ours / c.published);
        CHECK(std::abs(ours / c.published - 1.0) < 0.3);
    }
}

TEST_CASE("in a sideslip an F-16's tanks turn its nose from the wind: their bodies' unstable moment at their stations; "
          "a finned Mk 82's fins push back behind its station, at half their free strength",
          "[stores][drag][stability]") {
    session::World w(options("drag-sideslip"));
    const auto id = wing(w, "f16c", 5000.0, 220.0);
    REQUIRE(w.loadLoadout(id, "external tanks") == ms::Reason::None);
    std::vector<ms::Host::DragPoint> points;
    w.missionSystems(id)->carriageDrag(points);
    REQUIRE(points.size() == 3);
    double munk = 0.0, side = 0.0;
    for (const auto& p : points) munk += p.munkM3, side += p.sideSlopeM2;
    // the cards': 1.3645 m^3 for each 370 US gal tank, 1.147 for the 300; their bases' side force next to nothing
    CHECK(std::abs(munk - (2 * 1.3645 + 1.147)) < 1e-3);
    CHECK(side < 0.1);
    // what that is of its directional stability: dCn_beta = -2 x munk / (S b), on its 27.87 m^2 and 9.45 m
    const double dCnBeta = -2.0 * munk / (27.87 * 9.45);
    if (std::getenv("FSIM_DRAG_SHOW")) std::printf("F-16C, three tanks: Cn_beta changes by %.4f per radian (%.5f per degree)\n", dCnBeta, dCnBeta * kDeg);
    CHECK(dCnBeta < -0.02);
    CHECK(dCnBeta > -0.05);
    // a finned bomb: its fins' force behind its station at half its card's (carried fins measure half the isolated
    // theory: NASA TN D-3582) - free, they outweigh its body's moment (its card's 0.2469 x 1.152 against 2 x 0.0889)
    const auto bomber = wing(w, "f16c", 5000.0, 220.0, 1);
    REQUIRE(w.loadLoadout(bomber, "mk82 x4") == ms::Reason::None);
    w.missionSystems(bomber)->carriageDrag(points);
    REQUIRE(points.size() == 4);
    for (const auto& p : points) {
        CHECK(std::abs(p.sideSlopeM2 - 0.5 * 0.24693) < 1e-4);
        CHECK(std::abs(p.sideArmM - 1.1523) < 1e-3);
        CHECK(std::abs(p.munkM3 - 0.0889) < 1e-4);
        CHECK(2.0 * p.sideSlopeM2 * p.sideArmM > 2.0 * p.munkM3); // (free: steady)
    }
}

TEST_CASE("jettisoned, a bare tank swings broadside to the air and tumbles; a finned Mk 82 flies on nose first", "[stores][drag][stability][world]") {
    // the largest angle between its axis and the air over 6 s of its fall
    auto swing = [](const char* loadout) {
        session::World w(options("drag-tumble"));
        const auto id = wing(w, "f16c", 5000.0, 220.0);
        REQUIRE(w.loadLoadout(id, loadout) == ms::Reason::None);
        w.step(stepsFor(w, 1.0));
        ms::JettisonCommand all;
        all.payload = stationPayload(w, id);
        all.emergency = true;
        REQUIRE(w.submit(id, all).accepted());
        w.step(stepsFor(w, 0.2));
        std::uint32_t store = 0;
        for (const auto& s : w.stores(id))
            if (s.weapon) store = s.weapon;
        REQUIRE(store != 0);
        double worst = 0.0;
        for (int k = 0; k < 180; ++k) {
            w.step(stepsFor(w, 1.0 / 30.0));
            const sim::VehicleState* s = w.vehicleState(store);
            REQUIRE(s != nullptr);
            // (the angle between its axis and its way through the air: cos = cos(alpha) cos(beta))
            worst = std::max(worst, std::acos(std::clamp(std::cos(s->alphaRad) * std::cos(s->betaRad), -1.0, 1.0)));
        }
        return worst / kDeg;
    };
    const double tank = swing("external tanks"), bomb = swing("mk82 x4");
    if (std::getenv("FSIM_DRAG_SHOW")) std::printf("the largest angle to the air in 6 s: a tank %.0f deg, a Mk 82 %.0f deg\n", tank, bomb);
    CHECK(tank > 45.0);
    CHECK(bomb < 20.0);
}
