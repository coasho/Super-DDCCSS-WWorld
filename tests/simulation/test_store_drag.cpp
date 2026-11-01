// Shared aircraft VLM carriage tables, mounting hardware and released-body behavior.
#include "mode_flights.h"
#include "simulation/mission_systems/Host.h"
#include "simulation/runtime/stores/StoreAerodynamicsV1.h"
#include "foundation/io/AssetResolver.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <stdexcept>

using namespace fsim;
using namespace fsim::modes;

namespace {

std::shared_ptr<const session::StoreAerodynamicsV1> storeTable(const ms::Host::CarriedAirframeV1& store) {
    const auto& card=*store.card;
    const auto marker=card.parameters.find("airframe_aerodynamics_version");
    if(marker==card.parameters.end() || marker->second.value!=1.) throw std::runtime_error("Missing VLM binding");
    const auto relative=card.sources.at(marker->second.source).url;
    io::AssetResolver resolver;
    const auto name=card.model.substr(7);
    const auto directory=resolver.findAircraft(name,{});
    const auto path=directory/name/relative;
    if(!directory.empty() && std::filesystem::is_regular_file(path))return session::StoreAerodynamicsV1::load(path);
    throw std::runtime_error("Missing VLM product");
}

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
    // Hardware and authored airframe tables are separate contributions.
    CHECK(std::abs(loaded - 3 * (0.007 + 0.024)) < 1e-9);
    std::vector<ms::Host::CarriedAirframeV1> airframes;
    w.missionSystems(id)->carriedAirframesV1(airframes);
    REQUIRE(airframes.size()==3);
    for(const auto& store:airframes)CHECK(storeTable(store)->area>0.);
    ms::JettisonCommand all;
    all.payload = stationPayload(w, id);
    all.emergency = true;
    REQUIRE(w.submit(id, all).accepted());
    w.step(stepsFor(w, 0.5));
    w.missionSystems(id)->carriageDrag(points);
    REQUIRE(points.size() == 3);
    for (const auto& p : points) CHECK(std::abs(p.areaM2 - 0.024) < 1e-9);
    w.missionSystems(id)->carriedAirframesV1(airframes);
    CHECK(airframes.empty());
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
        std::vector<ms::Host::CarriedAirframeV1> airframes;
        w.missionSystems(id)->carriedAirframesV1(airframes);
        const double zero[3]{};
        for(const auto& store:airframes) {
            const auto table=storeTable(store);
            area+=table->area*table->coefficients(w.vehicleState(id)->mach,0.,0.,220.,zero)[0];
        }
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

TEST_CASE("the stores' configured drag indices retain the RAND MR-1216-AF comparison within 30 %",
          "[stores][drag]") {
    // a drag index: the stores' added CD x 10^4 on the aircraft's wing area, at Mach 0.8 (RAND's tables do not define it:
    // this as an F-16 engineer gives it); each case the stores' drag points' sum against it
    auto index = [](session::World& w, std::uint32_t id, double wingFt2) {
        std::vector<ms::Host::DragPoint> points;
        w.missionSystems(id)->carriageDrag(points);
        double area = 0.0;
        for (const auto& p : points) area += p.areaM2;
        std::vector<ms::Host::CarriedAirframeV1> airframes;
        w.missionSystems(id)->carriedAirframesV1(airframes);
        const double zero[3]{};
        for(const auto& store:airframes) {
            const auto table=storeTable(store);
            area+=table->area*table->coefficients(.8,0.,0.,250.,zero)[0];
        }
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

TEST_CASE("carried tanks and finned bombs select complete shared six-axis VLM products", "[stores][drag][stability]") {
    session::World w(options("shared-carriage"));
    for(const auto* loadout : {"external tanks","mk82 x4"}) {
        const auto id=wing(w,"f16c",5000.,220.,loadout[0]=='e'?0:1);
        REQUIRE(w.loadLoadout(id,loadout)==ms::Reason::None);
        std::vector<ms::Host::CarriedAirframeV1> stores;
        w.missionSystems(id)->carriedAirframesV1(stores);
        REQUIRE(stores.size()==(loadout[0]=='e'?3u:4u));
        for(const auto& store:stores) {
            CHECK(store.card->parameters.count("munk_volume_m3")==0);
            CHECK(store.card->parameters.count("side_slope_m2")==0);
            const auto table=storeTable(store);
            const double rates[]={.01,-.02,.015};
            const auto neutral=table->coefficients(.8,0.,0.,220.,rates);
            const auto sideslip=table->coefficients(.8,.05,.05,220.,rates);
            for(const auto value:sideslip)CHECK(std::isfinite(value));
            CHECK(std::abs(neutral[1]-sideslip[1])>1.e-6);
            CHECK(std::abs(neutral[5]-sideslip[5])>1.e-6);
        }
    }
}

TEST_CASE("in a sideslip an F-16's tanks retain the original directional-stability bounds; a free finned Mk 82 is stable",
          "[stores][drag][stability]") {
    // Preserve the original physical magnitude/sign gates on the current
    // common table loads. Removed scalar Munk/half-fin fields are no longer
    // physical inputs; finite/nonzero routing alone cannot replace stability.
    session::World w(options("drag-sideslip"));
    const auto id=wing(w,"f16c",5000.,220.);
    REQUIRE(w.loadLoadout(id,"external tanks")==ms::Reason::None);
    std::vector<ms::Host::CarriedAirframeV1> stores;
    w.missionSystems(id)->carriedAirframesV1(stores);
    REQUIRE(stores.size()==3);
    double cg[3]{};REQUIRE(w.model(id)->cgFromOriginBodyM(cg));
    constexpr double h=1.e-4;
    const auto gradient=[](const session::StoreAerodynamicsV1& table,const double arm[3]) {
        const double rates[3]{};
        const auto moment=[&](double beta) {
            const auto c=table.coefficients(.8,0.,beta,220.,rates);
            const double fx=table.area*(-c[0]*std::cos(beta)-c[1]*std::sin(beta));
            const double fy=table.area*(-c[0]*std::sin(beta)+c[1]*std::cos(beta));
            return table.area*table.span*c[5]+arm[0]*fy-arm[1]*fx;
        };
        return (moment(h)-moment(-h))/(2*h);
    };
    double yaw=0.;
    for(const auto& store:stores) {
        const auto table=storeTable(store);
        double arm[3];for(unsigned k=0;k<3;++k)arm[k]=store.positionM[k]+table->aeroPoint[k]-cg[k];
        yaw+=gradient(*table,arm);
    }
    const double dCnBeta=yaw/(27.87*9.45);
    INFO("shared-table tank dCn_beta "<<dCnBeta);
    CHECK(dCnBeta < -0.02);
    CHECK(dCnBeta > -0.05);
    const auto bomber=wing(w,"f16c",5000.,220.,1);
    REQUIRE(w.loadLoadout(bomber,"mk82 x4")==ms::Reason::None);
    w.missionSystems(bomber)->carriedAirframesV1(stores);
    REQUIRE(stores.size()==4);
    for(const auto& store:stores) {
        const auto table=storeTable(store);
        const double arm[]={table->aeroPoint[0],table->aeroPoint[1],table->aeroPoint[2]};
        const double freeGradient=gradient(*table,arm);
        INFO("shared-table free store yaw derivative "<<freeGradient);
        CHECK(freeGradient > 0.); // fins overcome the body's unstable couple
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
