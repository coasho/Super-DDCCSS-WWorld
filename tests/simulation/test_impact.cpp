// A vehicle striking the ground (ms/damage/Impact.h, session::World::impacted): what the strike does follows how it
// came in - its angle to the surface, its speed across and along it, the forces stopping it takes.
#include "mode_flights.h"
#include "simulation/mission_systems/damage/Impact.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using namespace fsim;
using namespace fsim::modes;
namespace dmg = fsim::ms::damage;

namespace {


dmg::StrikeOutcome strikeAt(double speed, double angleDeg, double massKg = 12000.0, double fuelKg = 3000.0) {
    dmg::GroundStrike s;
    s.velocity = {speed * std::cos(angleDeg * kDeg), 0.0, -speed * std::sin(angleDeg * kDeg)}; // (x along the ground, z up)
    s.normal = {0.0, 0.0, 1.0};
    s.massKg = massKg, s.fuelKg = fuelKg;
    return dmg::groundStrike(s);
}

struct Crash {
    ipc::EngagementEvent impact{}, fire{};
    int impacts = 0, fires = 0, separated = 0;
    std::vector<double> along, across; ///< where each piece lies from where it struck: down its track, off it (m)
    double wreckAlong = 0.0;
};

/// An F-16C flown into level ground at this speed and angle below the horizon, heading east; followed `followS` on.
Crash crash(const char* name, double speed, double angleDeg, double followS) {
    session::World w(options(name));
    session::VehicleSpec s;
    s.name = "viper";
    s.type = "jsbsim:f16c";
    s.initial.altitudeMslM = 40.0 + 2.0 * speed * std::sin(angleDeg * kDeg); // (two seconds from the ground)
    s.initial.headingDeg = 90.0;
    s.initial.pitchDeg = -angleDeg;
    s.initial.airspeedTrueMs = speed;
    const auto id = w.createVehicle(s);
    REQUIRE(id != 0);
    Crash c;
    sim::VehicleState struck{};
    bool down = false;
    auto take = [&] {
        for (const auto& ev : w.takeEngagementEvents()) {
            const auto kind = static_cast<ipc::EngagementKind>(ev.kind);
            if (kind == ipc::EngagementKind::Impact && ev.slot == 0) c.impact = ev, ++c.impacts;
            if (kind == ipc::EngagementKind::Fire && ev.slot == 0 && ev.amountKg > 0.0) c.fire = ev, ++c.fires;
            if (kind == ipc::EngagementKind::Separated) ++c.separated;
        }
    };
    for (unsigned k = 0; k < stepsFor(w, 12.0) && !down; ++k) {
        w.step(1);
        take();
        if (w.vehicleState(id)->crashed) down = true, struck = *w.vehicleState(id);
    }
    REQUIRE(down);
    w.step(stepsFor(w, followS));
    take();
    // each piece from where the aircraft struck, in metres east (down its track) and north (off it)
    const double R = 6371000.0, cl = std::cos(struck.latitudeRad);
    for (const auto v : w.vehicleIds()) {
        const sim::VehicleState& st = *w.vehicleState(v);
        const double east = (st.longitudeRad - struck.longitudeRad) * R * cl, north = (st.latitudeRad - struck.latitudeRad) * R;
        if (w.info(v)->role == ms::Role::Debris) c.along.push_back(east), c.across.push_back(north);
    }
    return c;
}
} // namespace

TEST_CASE("a ground strike: its angle splits its speed across and along the surface, and each does its own work", "[damage][impact]") {
    const auto dive = strikeAt(200.0, 70.0), shallow = strikeAt(200.0, 5.0);
    // the same speed and energy; the dive's is nearly all across the surface, the shallow strike's along it
    CHECK(std::abs(dive.energyJ - shallow.energyJ) < 1.0);
    CHECK(dive.energyJ == 0.5 * 12000.0 * 200.0 * 200.0);
    CHECK(std::abs(dive.angleRad - 70.0 * kDeg) < 1e-9);
    CHECK(std::abs(shallow.angleRad - 5.0 * kDeg) < 1e-9);
    CHECK(dive.normalEnergyJ > 0.85 * dive.energyJ);
    CHECK(shallow.normalEnergyJ < 0.01 * shallow.energyJ);
    // stopping it across the surface: v_n^2 / 2 d - thousands of g in the dive, some tens in the shallow strike
    const double d = dmg::kCrushShare * std::cbrt(12000.0 / dmg::kBulkDensityKgM3) + dmg::kGroundGiveM;
    CHECK(std::abs(dive.crushM - d) < 1e-12);
    CHECK(std::abs(dive.decelMs2 - dive.normalMs * dive.normalMs / (2.0 * d)) < 1e-6);
    CHECK(dive.decelMs2 / 9.80665 > 1000.0);
    CHECK(shallow.decelMs2 / 9.80665 > 5.0);
    CHECK(shallow.decelMs2 / 9.80665 < 30.0);
    // what leaves the strike: the dive's wreckage has no way on (the ground's friction took all it had along it), the
    // shallow strike's goes on down its track at nine tenths of its speed; each rebounds a tenth of what it came in at
    CHECK(std::hypot(dive.after.x, dive.after.y) < 1e-9);
    CHECK(std::abs(dive.after.z - dmg::kRestitution * dive.normalMs) < 1e-9);
    CHECK(shallow.after.x > 0.9 * 200.0);
    CHECK(shallow.after.x < shallow.tangentMs);
    CHECK(std::abs(shallow.tangentMs - shallow.after.x - dmg::kGroundFriction * (1.0 + dmg::kRestitution) * shallow.normalMs) < 1e-9);
    CHECK(std::abs(shallow.after.z - dmg::kRestitution * shallow.normalMs) < 1e-9);
    // the accelerometer's reading through the crush: up the normal, and against its motion along the ground
    CHECK(dive.specificForce.z > 0.0);
    CHECK(shallow.specificForce.x < 0.0);
    CHECK(std::abs(shallow.specificForce.x) < shallow.specificForce.z);
    // no energy from nowhere
    for (const auto& o : {dive, shallow}) CHECK(dmg::dot(o.after, o.after) < o.speedMs * o.speedMs);
}

TEST_CASE("a ground strike's fuel: held through a landing's sink, burst and burning at once in a hard strike", "[damage][impact]") {
    // a wheels-up landing: 70 m/s along the runway, 2 m/s into it - inside what a crashworthy tank is built for
    const auto belly = strikeAt(70.0, std::atan2(2.0, 70.0) / kDeg);
    CHECK(belly.severityMs < dmg::kTankSafeMs);
    CHECK(belly.spiltShare == 0.0);
    CHECK(belly.fireballKg == 0.0);
    CHECK(belly.poolKg == 0.0);
    // harder strikes spill more, and more of what they spill burns at once; never more than there was
    double spilt = 0.0, fireball = 0.0;
    for (const double angle : {4.0, 8.0, 15.0, 30.0, 60.0, 90.0}) {
        const auto o = strikeAt(180.0, angle);
        INFO(angle << " deg: severity " << o.severityMs << " m/s, spilt " << o.spiltShare << ", fireball " << o.fireballKg << " kg");
        CHECK(o.spiltShare >= spilt);
        CHECK(o.fireballKg >= fireball);
        CHECK(o.fireballKg + o.poolKg <= 3000.0 + 1e-9);
        CHECK(std::abs(o.fireballKg + o.poolKg - 3000.0 * o.spiltShare) < 1e-9);
        spilt = o.spiltShare, fireball = o.fireballKg;
    }
    CHECK(spilt == 1.0);                       // (straight in at 180 m/s: every tank)
    CHECK(fireball == dmg::kMistMost * 3000.0); // (and the most of it at once)
    // a touch, a bounce: leaving the ground is no strike
    dmg::GroundStrike up;
    up.velocity = {50.0, 0.0, 3.0}, up.normal = {0.0, 0.0, 1.0}, up.massKg = 1000.0, up.fuelKg = 100.0;
    const auto none = dmg::groundStrike(up);
    CHECK(none.decelMs2 == 0.0);
    CHECK(none.fireballKg == 0.0);
    // a slope turns a level flight path into a strike: the same velocity against a 30 deg hillside
    dmg::GroundStrike hill;
    hill.velocity = {150.0, 0.0, 0.0}, hill.normal = {-0.5, 0.0, std::sqrt(0.75)}, hill.massKg = 12000.0, hill.fuelKg = 3000.0;
    const auto side = dmg::groundStrike(hill);
    CHECK(std::abs(side.angleRad - 30.0 * kDeg) < 1e-9);
    CHECK(std::abs(side.normalMs - 75.0) < 1e-9);
    CHECK(side.after.z > 0.0); // (what leaves it runs up the slope)
}

TEST_CASE("an F-16C flown into the ground: a dive's wreckage lies where it struck under its fireball, a shallow strike's goes on down its track",
          "[damage][impact][world]") {
    const Crash dive = crash("impact-dive", 220.0, 60.0, 25.0);
    const Crash shallow = crash("impact-shallow", 200.0, 6.0, 25.0);
    for (const Crash* c : {&dive, &shallow}) {
        CHECK(c->impacts == 1);
        CHECK(c->impact.massKg > 9000.0);
        CHECK(std::string(c->impact.name) == "ground");
        // the ground's normal there: up (level ground)
        const double* p = c->impact.positionEcef;
        const double r = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
        const double up = (c->impact.normalEcef[0] * p[0] + c->impact.normalEcef[1] * p[1] + c->impact.normalEcef[2] * p[2]) / r;
        CHECK(up > 0.999);
        CHECK(c->impact.energyJ > 0.4 * c->impact.massKg * 190.0 * 190.0);
    }
    if (std::getenv("FSIM_IMPACT_SHOW"))
        for (const Crash* c : {&dive, &shallow}) {
            std::printf("angle %.1f deg, %.0f g, %u sections, fireball %.0f kg moving %.0f m/s, burning on %.1f kg/s; pieces:",
                        c->impact.angleRad / kDeg, c->impact.loadG, c->impact.index, c->fire.amountKg,
                        std::sqrt(c->fire.velocityEcef[0] * c->fire.velocityEcef[0] + c->fire.velocityEcef[1] * c->fire.velocityEcef[1] +
                                  c->fire.velocityEcef[2] * c->fire.velocityEcef[2]),
                        c->impact.rateKgS);
            for (std::size_t k = 0; k < c->along.size(); ++k) std::printf(" (%.0f, %.0f)", c->along[k], c->across[k]);
            std::printf("\n");
        }
    // their angles as flown, and the load each took across the ground: the dive's a hundred times the shallow strike's
    CHECK(dive.impact.angleRad > 50.0 * kDeg);
    CHECK(shallow.impact.angleRad < 12.0 * kDeg);
    CHECK(dive.impact.loadG > 20.0 * shallow.impact.loadG);
    // both break up (an F-16C's wings, nose and tail), and both burn: the dive burns more of its fuel at once
    CHECK(dive.impact.index >= 3);
    CHECK(dive.separated == static_cast<int>(dive.impact.index));
    CHECK(shallow.separated == static_cast<int>(shallow.impact.index));
    CHECK(dive.fires == 1);
    CHECK(dive.fire.amountKg > 0.0);
    CHECK(dive.fire.amountKg >= shallow.fire.amountKg);
    CHECK(dive.impact.rateKgS > 0.0);
    // the dive's fireball rises where it struck: what left the strike has no way on along the ground
    auto speed = [](const double v[3]) { return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); };
    CHECK(speed(dive.fire.velocityEcef) < 0.15 * speed(dive.impact.velocityEcef));
    REQUIRE(!dive.along.empty());
    for (std::size_t k = 0; k < dive.along.size(); ++k) CHECK(std::hypot(dive.along[k], dive.across[k]) < 150.0);
    // the shallow strike's goes on: its fire moves down its track at most of its speed, and its pieces lie far along it
    if (shallow.fires > 0) CHECK(speed(shallow.fire.velocityEcef) > 0.8 * speed(shallow.impact.velocityEcef));
    if (!shallow.along.empty()) {
        const double furthest = *std::max_element(shallow.along.begin(), shallow.along.end());
        CHECK(furthest > 300.0);
        for (std::size_t k = 0; k < shallow.along.size(); ++k) {
            CHECK(shallow.along[k] > -20.0);                                        // (none thrown back)
            CHECK(std::abs(shallow.across[k]) < 0.35 * std::max(shallow.along[k], 60.0)); // (down its track, not across it)
        }
    }
}

TEST_CASE("a jettisoned tank that strikes the ground bursts and burns its fuel; a bomb's and a missile's body have none to burn",
          "[damage][impact][stores][world]") {
    // an F-16C at 400 m with its three tanks full and two Mk 82s lets them all go: each is a vehicle of its own until it
    // strikes the ground, where it ended without a trace - its strike is told now, and a tank's fuel left is a fire
    session::World w(options("impact-stores"));
    const auto id = wing(w, "f16c", 400.0, 200.0);
    REQUIRE(w.loadLoadout(id, "mk82 x4") == ms::Reason::None);
    REQUIRE(w.loadLoadout(id, "external tanks") == ms::Reason::None);
    w.step(stepsFor(w, 0.5));
    std::uint16_t payload = 0;
    for (const auto& pl : w.missionSystems(id)->payloads())
        if (pl.family == ms::Family::StoreStation) payload = pl.index;
    ms::JettisonCommand all;
    all.payload = payload;
    all.emergency = true;
    REQUIRE(w.submit(id, all).accepted());
    int strikes = 0, fires = 0;
    double burnt = 0.0, steepest = 0.0;
    for (unsigned k = 0; k < stepsFor(w, 25.0); ++k) {
        w.step(1);
        for (const auto& ev : w.takeEngagementEvents()) {
            const auto kind = static_cast<ipc::EngagementKind>(ev.kind);
            if (kind == ipc::EngagementKind::Impact && ev.slot != 0) {
                ++strikes;
                steepest = std::max(steepest, ev.angleRad);
                CHECK(ev.rateKgS == 0.0); // (it ends there: nothing of it burns on)
                CHECK(ev.energyJ > 0.0);
            }
            if (kind == ipc::EngagementKind::Fire && ev.slot != 0 && ev.amountKg > 0.0) ++fires, burnt += ev.amountKg;
        }
    }
    if (std::getenv("FSIM_IMPACT_SHOW")) std::printf("stores struck: %d, fires %d, %.0f kg burnt at once, steepest %.0f deg\n", strikes, fires, burnt, steepest / kDeg);
    CHECK(strikes == 5); // (three tanks, two bombs)
    CHECK(fires == 3);   // (the tanks': a bomb's body has no fuel)
    CHECK(burnt > 300.0);
    CHECK(burnt < 3.0 * 1200.0); // (no more than they held)
}
