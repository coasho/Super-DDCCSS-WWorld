// A vehicle that strikes the terrain (docs/flight-autonomy.md, 4.68): JSBSim's ground pushes up only, under each point
// it is asked about, so a wing flown into a hillside went into the rock and flew on inside it. It is stopped where it
// struck instead, on the surface, `crashed` set and its activity Failed "crashed", until it is reset; one started inside
// the terrain is stopped on its surface as it starts. The ground: a wall 400 m high from 3 km east of the start.
#include "mode_flights.h"

#include "fsim/Capability.h"
#include "fsim/GroundProvider.h"
#include "fsim/InitialConditions.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <memory>

using namespace fsim;
using namespace fsim::control;
using namespace fsim::modes;

namespace {

constexpr double kR = 6371008.8; // the platform's mean radius (core/Geodesy.h)
const sim::InitialConditions kStart;
const double kLat0 = kStart.latitudeDeg * kDeg, kLon0 = kStart.longitudeDeg * kDeg;

double eastOf(double lon) { return (lon - kLon0) * kR * std::cos(kLat0); }

class Wall final : public sim::GroundProvider {
public:
    double heightAboveEllipsoidM(double, double lon) const override { return eastOf(lon) >= 3000.0 ? 400.0 : 0.0; }
};

session::WorldOptions walled(const char* name) {
    auto o = options(name);
    o.ground = std::make_shared<Wall>();
    return o;
}

std::uint32_t spawn(session::World& w, double eastM, double altitudeM) {
    session::VehicleSpec s;
    s.name = "c172x";
    s.type = "jsbsim:c172x";
    s.initial.longitudeDeg += eastM / (kR * std::cos(kLat0)) / kDeg;
    s.initial.altitudeMslM = altitudeM;
    s.initial.headingDeg = 90.0;
    s.initial.airspeedTrueMs = 55.0;
    const auto id = w.createVehicle(s);
    REQUIRE(id != 0);
    return id;
}

} // namespace

TEST_CASE("terrain strike: flown into a wall, a wing stops on its surface, crashed, its activity Failed; a reset clears it",
          "[terrain_strike]") {
    session::World w(walled("terrain-strike"));
    const auto v = spawn(w, 0.0, 200.0);
    // (an attitude: a path into the wall - a velocity, a route - is refused terrain_conflict before it is flown, 4.19)
    AttitudeCommand level;
    level.pitchRad = 2.0 * kDeg, level.headingRad = 90.0 * kDeg, level.airspeedMs = 55.0;
    const CommandResult r = w.submit(v, level);
    INFO("reason " << static_cast<int>(r.reason));
    REQUIRE(r.accepted());

    double furthest = -1e9;
    for (unsigned k = 0; k < stepsFor(w, 120.0) && !w.vehicleState(v)->crashed; ++k) {
        w.step();
        furthest = std::max(furthest, eastOf(w.vehicleState(v)->longitudeRad));
    }
    const sim::VehicleState s = *w.vehicleState(v);
    INFO("stopped " << eastOf(s.longitudeRad) << " m east at " << s.altitudeMslM << " m, " << s.altitudeAglM << " m above the ground");
    REQUIRE(s.crashed);
    CHECK_FALSE(s.diverged);
    CHECK(s.onGround);
    CHECK(furthest < 3000.0 + 60.0);                                       // struck at the wall's face, not inside it
    CHECK((s.altitudeMslM > 400.0 - 1.0 && s.altitudeMslM < 400.0 + 10.0)); // on its top, as it was lifted out
    CHECK(std::hypot(s.velocityNedMs[0], s.velocityNedMs[1], s.velocityNedMs[2]) == 0.0);
    CHECK(s.airspeedTrueMs == 0.0);

    // its activity ended Failed, crashed; and it stays where it struck
    w.step();
    const ActivityRecord* a = w.activity(r.activity);
    REQUIRE(a != nullptr);
    CHECK(a->state == ActivityState::Failed);
    CHECK(a->reason == Reason::Crashed);
    w.step(stepsFor(w, 10.0));
    CHECK(w.vehicleState(v)->positionEcef[0] == s.positionEcef[0]);
    CHECK(w.vehicleState(v)->altitudeMslM == s.altitudeMslM);
    CHECK_FALSE(w.submit(v, level).accepted()); // nothing more until it is reset

    // a reset clears it: flying again where it starts
    REQUIRE(w.resetVehicle(v));
    CHECK_FALSE(w.vehicleState(v)->crashed);
    w.step(stepsFor(w, 2.0));
    CHECK_FALSE(w.vehicleState(v)->crashed);
    CHECK(std::abs(w.vehicleState(v)->altitudeMslM - 200.0) < 30.0);
}

TEST_CASE("terrain strike: a wing started inside the terrain is stopped on its surface as it starts, not blown out of it",
          "[terrain_strike]") {
    session::World w(walled("terrain-strike-spawn"));
    const auto v = spawn(w, 3500.0, 200.0); // 200 m under the wall's top
    const sim::VehicleState s = *w.vehicleState(v);
    INFO("started at " << s.altitudeMslM << " m, " << s.altitudeAglM << " m above the ground");
    REQUIRE(s.crashed);
    CHECK(std::abs(s.altitudeMslM - 401.5) < 0.1); // a gear's height above it, as a start on the ground begins
    w.step(stepsFor(w, 2.0));
    CHECK_FALSE(w.vehicleState(v)->diverged);
    CHECK(w.vehicleState(v)->altitudeMslM == s.altitudeMslM);

    // started over the wall instead: flies
    sim::InitialConditions above = kStart;
    above.longitudeDeg += 3500.0 / (kR * std::cos(kLat0)) / kDeg;
    above.altitudeMslM = 700.0, above.headingDeg = 90.0, above.airspeedTrueMs = 55.0;
    REQUIRE(w.resetVehicle(v, &above));
    w.step(stepsFor(w, 2.0));
    CHECK_FALSE(w.vehicleState(v)->crashed);
    CHECK(w.vehicleState(v)->altitudeMslM > 600.0);
}
