// Weapons in flight as entities (ADR-30, MS-2): a store released by an
// aircraft becomes a player of the world - at its station, with its launcher's
// velocity and the ejection's push, flown by its own model and by no flight
// capability - until it strikes the ground, when its record is kept; the
// same release twice is the same flight to the bit; and the radio path
// between two entities.
#include "mode_flights.h"

#include "foundation/core/Geodesy.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <string>
#include <vector>

using namespace fsim;
using namespace fsim::modes;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

ms::WeaponLaunch mk82() {
    ms::WeaponLaunch l;
    l.type = "jsbsim:mk82";
    l.offsetBodyM[0] = 0.5, l.offsetBodyM[1] = 2.5, l.offsetBodyM[2] = 0.8; // under the right wing
    l.ejectionBodyMs[2] = 3.0;                                             // an ejector rack's push, down
    return l;
}

struct Drop {
    double timeS = 0.0, latitudeRad = 0.0, longitudeRad = 0.0, speedMs = 0.0;
};

Drop dropOnce(const char* name) {
    session::World w(options(name));
    const auto f16 = wing(w, "f16c", 3000.0, 200.0);
    w.step(stepsFor(w, 1.0));
    const auto bomb = w.launchWeapon(f16, mk82());
    REQUIRE(bomb != 0);
    for (unsigned i = 0; i < stepsFor(w, 60.0) && w.alive(bomb); ++i) w.step();
    const auto records = w.weapons();
    REQUIRE(records.size() == 1);
    return Drop{records[0].endTimeS, records[0].endLatitudeRad, records[0].endLongitudeRad, records[0].endSpeedMs};
}

} // namespace

TEST_CASE("weapons: a Mk 82 released by an F-16C falls as a player of the world and ends where it strikes", "[ms][weapons]") {
    session::World w(options("weapons-drop"));
    const auto f16 = wing(w, "f16c", 3000.0, 200.0);
    w.step(stepsFor(w, 1.0));
    const sim::VehicleState launcher = *w.vehicleState(f16);

    const auto bomb = w.launchWeapon(f16, mk82());
    REQUIRE(bomb != 0);
    const session::VehicleInfo* info = w.info(bomb);
    REQUIRE(info != nullptr);
    CHECK(info->role == ms::Role::Weapon);
    CHECK(info->launcher == f16);
    CHECK(info->type == "jsbsim:mk82");
    CHECK(w.info(f16)->role == ms::Role::Aircraft);

    // At its station, with the launcher's velocity and the push
    const sim::VehicleState& s0 = *w.vehicleState(bomb);
    const double dx = s0.positionEcef[0] - launcher.positionEcef[0], dy = s0.positionEcef[1] - launcher.positionEcef[1],
                 dz = s0.positionEcef[2] - launcher.positionEcef[2];
    CHECK_THAT(std::sqrt(dx * dx + dy * dy + dz * dz), WithinAbs(std::sqrt(0.5 * 0.5 + 2.5 * 2.5 + 0.8 * 0.8), 0.05));
    CHECK_THAT(s0.velocityNedMs[0], WithinAbs(launcher.velocityNedMs[0], 0.2));
    CHECK_THAT(s0.velocityNedMs[1], WithinAbs(launcher.velocityNedMs[1], 0.2));
    CHECK(s0.velocityNedMs[2] > launcher.velocityNedMs[2] + 2.0); // (pushed down)

    // No flight capability takes it: each restricted
    const auto status = w.capabilityStatus(bomb, "fsim.flight.attitude");
    CHECK(status.availability == control::Availability::Unavailable);
    CHECK(status.reason == control::Reason::Restricted);
    control::CommandOptions opts;
    CHECK_FALSE(w.submit(bomb, control::Command(control::AttitudeCommand{}), opts).accepted());

    // A player: in the world's list, the radio path to it from its launcher
    const auto ids = w.vehicleIds();
    CHECK(std::find(ids.begin(), ids.end(), bomb) != ids.end());
    w.step(stepsFor(w, 2.0));
    const auto path = w.radioPath(f16, bomb, ms::RadioPathOptions{});
    REQUIRE(path.has_value());
    {
        const sim::VehicleState& a = *w.vehicleState(f16);
        const sim::VehicleState& b = *w.vehicleState(bomb);
        const double ex = a.positionEcef[0] - b.positionEcef[0], ey = a.positionEcef[1] - b.positionEcef[1],
                     ez = a.positionEcef[2] - b.positionEcef[2];
        CHECK_THAT(path->slantRangeM, WithinAbs(std::sqrt(ex * ex + ey * ey + ez * ez), 0.001)); // (WGS-84 against JSBSim's ECEF)
        CHECK(path->elevationRad[0] < 0.0); // the bomb below the jet
        CHECK(path->totalDb > path->freeSpaceDb - 7.0);
    }
    {
        const auto live = w.weapons();
        REQUIRE(live.size() == 1);
        CHECK(live[0].end == ms::WeaponEnd::InFlight);
        CHECK(std::isnan(live[0].endTimeS));
    }

    // Falls to the ground (sea level here): from 3000 m at 200 m/s, ~25 s in a vacuum, a little more with its drag
    double launchTime = w.simTime();
    for (unsigned i = 0; i < stepsFor(w, 60.0) && w.alive(bomb); ++i) w.step();
    CHECK_FALSE(w.alive(bomb));
    const auto records = w.weapons();
    REQUIRE(records.size() == 1);
    const ms::WeaponRecord& r = records[0];
    CHECK(r.end == ms::WeaponEnd::Impact);
    CHECK(r.id == bomb);
    CHECK(r.launcher == f16);
    const double fall = r.endTimeS - launchTime + 2.0;
    CAPTURE(fall, r.endSpeedMs, r.endHeightM);
    CHECK(fall > 24.0);
    CHECK(fall < 32.0);
    CHECK(std::fabs(r.endHeightM) < 5.0);
    CHECK(r.endSpeedMs > 200.0);
    // its range ahead of where it was released: the 200 m/s it carried, less its drag
    const double range = geo::distanceM(launcher.latitudeRad, launcher.longitudeRad, r.endLatitudeRad, r.endLongitudeRad);
    CHECK(range > 3500.0);
    CHECK(range < 200.0 * fall);
    CHECK(w.alive(f16)); // (its launcher flies on)
}

TEST_CASE("weapons: the same release is the same flight to the bit; a weapon taken out is recorded removed", "[ms][weapons]") {
    const Drop a = dropOnce("weapons-a"), b = dropOnce("weapons-b");
    CHECK(a.timeS == b.timeS);
    CHECK(a.latitudeRad == b.latitudeRad);
    CHECK(a.longitudeRad == b.longitudeRad);
    CHECK(a.speedMs == b.speedMs);

    session::World w(options("weapons-removed"));
    const auto f16 = wing(w, "f16c", 3000.0, 200.0);
    const auto bomb = w.launchWeapon(f16, mk82());
    REQUIRE(bomb != 0);
    w.step(stepsFor(w, 1.0));
    CHECK(w.removeVehicle(bomb));
    const auto records = w.weapons();
    REQUIRE(records.size() == 1);
    CHECK(records[0].end == ms::WeaponEnd::Removed);
    CHECK(w.launchWeapon(999, mk82()) == 0); // (no such launcher)
}
