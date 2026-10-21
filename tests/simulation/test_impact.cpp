// A vehicle striking the ground (ms/damage/Impact.h, session::World::impacted): what the strike does follows how it
// came in - its angle to the surface, its speed across and along it, the forces stopping it takes.
#include "mode_flights.h"
#include "simulation/mission_systems/damage/Impact.h"
#include "fsim/GroundProvider.h"

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

namespace contact_tests {
using namespace dmg;
ContactBody body(double mass, double ix=1, double iy=1, double iz=1) {
    ContactBody out; out.massKg=mass; out.inertia[0]=ix; out.inertia[4]=iy; out.inertia[8]=iz; return out;
}
bool close(double a,double b) { return std::abs(a-b)<1e-10; }

TEST_CASE("Central body contact conserves momentum and loses exactly the analytic energy", "[contact-energy]") {
    auto a=body(2), b=body(3); a.velocity={-5,0,0};
    const auto out=inelasticBodyContact(a,b,{1,0,0});
    REQUIRE(out.valid); REQUIRE(out.approaching);
    CHECK(close(out.effectiveMassKg,1.2)); CHECK(close(out.impulseA.x,6));
    CHECK(out.impulseB.x==-out.impulseA.x); CHECK(close(out.dissipatedJ,15));
    const auto va=a.velocity+out.impulseA*(1/a.massKg), vb=b.velocity+out.impulseB*(1/b.massKg);
    CHECK(close(va.x,-2)); CHECK(close(vb.x,-2));
    CHECK(close(.5*a.massKg*dot(va,va)+.5*b.massKg*dot(vb,vb),10));
}

TEST_CASE("Off-centre contact includes both rotational inertias and angular momentum", "[contact-energy]") {
    auto a=body(2,2,3,4), b=body(3,5,6,7);
    a.velocity={-5,0,0}; a.arm={0,1,0}; b.arm={0,0,1};
    const auto out=inelasticBodyContact(a,b,{1,0,0});
    REQUIRE(out.valid); REQUIRE(out.approaching);
    CHECK(close(out.effectiveMassKg,.8)); CHECK(close(out.impulseA.x,4));
    const auto va=a.velocity+out.impulseA*(1/a.massKg), vb=b.velocity+out.impulseB*(1/b.massKg);
    const V3 wa{out.angularImpulseA.x/2,out.angularImpulseA.y/3,out.angularImpulseA.z/4};
    const V3 wb{out.angularImpulseB.x/5,out.angularImpulseB.y/6,out.angularImpulseB.z/7};
    CHECK(close((va+cross(wa,a.arm)-vb-cross(wb,b.arm)).x,0));
    const double after=.5*a.massKg*dot(va,va)+.5*b.massKg*dot(vb,vb)+
        .5*dot(wa,{2*wa.x,3*wa.y,4*wa.z})+.5*dot(wb,{5*wb.x,6*wb.y,7*wb.z});
    CHECK(close(25-after,out.dissipatedJ));
    const auto beforeL=cross(a.arm*(-1),a.velocity*a.massKg);
    const auto afterL=cross(a.arm*(-1),va*a.massKg)+cross(b.arm*(-1),vb*b.massKg)+
        V3{2*wa.x,3*wa.y,4*wa.z}+V3{5*wb.x,6*wb.y,7*wb.z};
    CHECK(length(afterL-beforeL)<1e-10);
    CHECK(length(va*a.massKg+vb*b.massKg-a.velocity*a.massKg)<1e-10);
}

TEST_CASE("Body contact retains inertia products and Galilean invariance", "[contact-energy]") {
    auto a=body(2,2,2,4), b=body(3); a.inertia[1]=a.inertia[3]=.5;
    a.arm={0,0,1}; a.velocity={-5,0,0};
    const auto first=inelasticBodyContact(a,b,{1,0,0});
    REQUIRE(first.valid); CHECK(close(first.impulseA.x,150.0/41));
    a.velocity+={120,-70,30}; b.velocity+={120,-70,30};
    const auto boosted=inelasticBodyContact(a,b,{7,0,0});
    REQUIRE(boosted.valid); CHECK(length(boosted.impulseA-first.impulseA)<1e-10);
    CHECK(close(boosted.dissipatedJ,first.dissipatedJ));
}

TEST_CASE("Separating tangent and invalid bodies cannot acquire contact impulse", "[contact-energy]") {
    auto a=body(2),b=body(3); a.velocity={2,3,0};
    auto out=inelasticBodyContact(a,b,{1,0,0});
    REQUIRE(out.valid); CHECK_FALSE(out.approaching); CHECK(length(out.impulseA)==0);
    a.velocity={0,3,0}; out=inelasticBodyContact(a,b,{1,0,0});
    REQUIRE(out.valid); CHECK_FALSE(out.approaching);
    a.massKg=0; CHECK_FALSE(inelasticBodyContact(a,b,{1,0,0}).valid);
    a.massKg=2; a.inertia[4]=-1; CHECK_FALSE(inelasticBodyContact(a,b,{1,0,0}).valid);
    a=body(2); CHECK_FALSE(inelasticBodyContact(a,b,{}).valid);
}
} // namespace contact_tests

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

TEST_CASE("ground strike energy uses the incoming structural contact before ground reactions dissipate it",
          "[damage][impact][world][contact_energy]") {
    auto o = options("impact-contact-energy");
    o.frameSkip = 1;
    session::World w(o);
    session::VehicleSpec spec;
    spec.name = "falling-tank";
    spec.type = "jsbsim:tank370gal";
    spec.aircraftDir = std::filesystem::path(FSIM_TEST_AIRCRAFT_DIR) / "stores";
    spec.initial.altitudeMslM = 30.0;
    spec.initial.pitchDeg = -50.0;
    spec.initial.airspeedTrueMs = 70.0;
    const auto id = w.createVehicle(spec);
    REQUIRE(id != 0);
    auto* model = w.model(id);
    REQUIRE(model != nullptr);
    const auto* contactSource = dynamic_cast<const sim::TerrainStrikeSourceV1*>(model);
    REQUIRE(contactSource != nullptr);
    sim::VehicleState retained{};
    CHECK_FALSE(contactSource->terrainStrikeStateV1(retained));
    REQUIRE(model->beginMassChange());
    model->property("inertia/pointmass-weight-lbs").set(1120.0 / 0.45359237);
    REQUIRE(model->endMassChange(nullptr, nullptr));
    const auto units = model->property("gear/num-units");
    REQUIRE(units.valid());
    std::vector<PropertyHandle> compression;
    for (int k = 0; k < static_cast<int>(units.get()); ++k) {
        compression.push_back(model->property("contact/unit[" + std::to_string(k) + "]/compression-ft"));
        REQUIRE(compression.back().valid());
    }
    sim::VehicleState incoming{};
    bool touched = false, struck = false;
    ipc::EngagementEvent impact{};
    for (unsigned k = 0; k < stepsFor(w, 5.0) && !struck; ++k) {
        const auto before = *w.vehicleState(id);
        w.step(1);
        const bool contact = std::any_of(compression.begin(), compression.end(), [](const auto& p) { return p.get() > 0.0; });
        if (!touched && contact) incoming = before, touched = true;
        for (const auto& event : w.takeEngagementEvents())
            if (static_cast<ipc::EngagementKind>(event.kind) == ipc::EngagementKind::Impact && event.slot == 0)
                impact = event, struck = true;
    }
    REQUIRE(touched);
    REQUIRE(struck);
    const double speed2 = dmg::dot(dmg::V3{incoming.velocityNedMs[0], incoming.velocityNedMs[1], incoming.velocityNedMs[2]},
                                  dmg::V3{incoming.velocityNedMs[0], incoming.velocityNedMs[1], incoming.velocityNedMs[2]});
    const double incomingEnergy = 0.5 * impact.massKg * speed2;
    CAPTURE(incoming.simTime, impact.energyJ, incomingEnergy);
    CHECK(std::abs(impact.energyJ / incomingEnergy - 1.0) < 1e-10);
    REQUIRE(contactSource->terrainStrikeStateV1(retained));
    CHECK(retained.simTime == incoming.simTime);
    const double sl = std::sin(incoming.latitudeRad), cl = std::cos(incoming.latitudeRad);
    const double so = std::sin(incoming.longitudeRad), co = std::cos(incoming.longitudeRad);
    const auto& v = incoming.velocityNedMs;
    const double ecef[3] = {-sl * co * v[0] - so * v[1] - cl * co * v[2],
                            -sl * so * v[0] + co * v[1] - cl * so * v[2], cl * v[0] - sl * v[2]};
    for (int k = 0; k < 3; ++k) CHECK(std::abs(impact.velocityEcef[k] - ecef[k]) < 1e-10);
    REQUIRE(w.resetVehicle(id));
    CHECK_FALSE(contactSource->terrainStrikeStateV1(retained));

    // Batching four native steps must retain the same incoming sample, even
    // when initial contact falls inside the batch rather than at its boundary.
    o.name = "impact-contact-energy-batched";
    o.frameSkip = 4;
    session::World batched(o);
    const auto batchedId = batched.createVehicle(spec);
    REQUIRE(batchedId != 0);
    auto* batchedModel = batched.model(batchedId);
    REQUIRE(batchedModel->beginMassChange());
    batchedModel->property("inertia/pointmass-weight-lbs").set(1120.0 / 0.45359237);
    REQUIRE(batchedModel->endMassChange(nullptr, nullptr));
    bool batchedStruck = false;
    for (unsigned k = 0; k < stepsFor(batched, 5.0) && !batchedStruck; ++k) {
        batched.step(1);
        for (const auto& event : batched.takeEngagementEvents())
            if (static_cast<ipc::EngagementKind>(event.kind) == ipc::EngagementKind::Impact && event.slot == 0) {
                batchedStruck = true;
                CHECK(event.energyJ == impact.energyJ);
                for (int axis = 0; axis < 3; ++axis) CHECK(event.velocityEcef[axis] == impact.velocityEcef[axis]);
            }
    }
    REQUIRE(batchedStruck);
}

TEST_CASE("ground strike contact history clears after disengagement and excludes landing gear", "[impact][contact_energy][world]") {
    // A controlled synthetic terrain discontinuity exercises contact-episode
    // lifetime without altering spring constants or a normal crash fixture.
    struct Plane final : sim::GroundProvider {
        double height = 0;
        double heightAboveEllipsoidM(double, double) const override { return height; }
    };
    auto plane = std::make_shared<Plane>();
    auto o = options("impact-contact-disengagement");
    o.ground = plane;
    o.frameSkip = 1;
    session::World w(o);
    session::VehicleSpec spec;
    spec.type = "jsbsim:tank370gal";
    spec.aircraftDir = std::filesystem::path(FSIM_TEST_AIRCRAFT_DIR) / "stores";
    spec.initial.altitudeMslM = 100;
    spec.initial.airspeedTrueMs = 20;
    const auto id = w.createVehicle(spec);
    REQUIRE(id != 0);
    w.step(3);
    auto* model = w.model(id);
    const int units = static_cast<int>(model->property("gear/num-units").get());
    double lowest = std::numeric_limits<double>::infinity();
    std::vector<PropertyHandle> compression;
    for (int k = 0; k < units; ++k) {
        const std::string prefix = "contact/unit[" + std::to_string(k) + "]/";
        lowest = std::min(lowest, model->property(prefix + "AGL-ft").get() * 0.3048);
        compression.push_back(model->property(prefix + "compression-ft"));
    }
    REQUIRE(std::isfinite(lowest));
    plane->height = lowest + .05;
    w.step(1);
    REQUIRE_FALSE(w.vehicleState(id)->crashed);
    REQUIRE(std::any_of(compression.begin(), compression.end(), [](const auto& p) { return p.get() > 0; }));
    plane->height = -1000;
    w.step(5);
    REQUIRE(std::all_of(compression.begin(), compression.end(), [](const auto& p) { return p.get() == 0; }));
    const auto incoming = *w.vehicleState(id);
    plane->height = incoming.altitudeMslM + 5;
    w.step(1);
    REQUIRE(w.vehicleState(id)->crashed);
    const auto* source = dynamic_cast<const sim::TerrainStrikeSourceV1*>(model);
    REQUIRE(source != nullptr);
    sim::VehicleState retained{};
    REQUIRE(source->terrainStrikeStateV1(retained));
    CHECK(retained.simTime == incoming.simTime);
    for (int k = 0; k < 3; ++k) CHECK(retained.velocityNedMs[k] == incoming.velocityNedMs[k]);

    session::World landed(options("impact-gear-only"));
    session::VehicleSpec landing;
    landing.type = "jsbsim:c172x";
    landing.initial.onGround = true;
    landing.initial.airspeedTrueMs = 0;
    const auto parked = landed.createVehicle(landing);
    REQUIRE(parked != 0);
    landed.step(stepsFor(landed, 1));
    CHECK_FALSE(landed.vehicleState(parked)->crashed);
    const auto* landingSource = dynamic_cast<const sim::TerrainStrikeSourceV1*>(landed.model(parked));
    REQUIRE(landingSource != nullptr);
    CHECK_FALSE(landingSource->terrainStrikeStateV1(retained));
}

TEST_CASE("ground strike energy retains incoming mass when contact mass changes",
          "[impact][contact_energy_mass][world]") {
    auto o = options("impact-contact-changing-mass");
    o.frameSkip = 1;
    session::World w(o);
    session::VehicleSpec spec;
    spec.name = "changing-mass-tank";
    spec.type = "jsbsim:tank370gal";
    spec.aircraftDir = std::filesystem::path(FSIM_TEST_AIRCRAFT_DIR) / "stores";
    spec.initial.altitudeMslM = 30;
    spec.initial.pitchDeg = -50;
    spec.initial.airspeedTrueMs = 70;
    const auto id = w.createVehicle(spec);
    REQUIRE(id != 0);
    auto* model = w.model(id);
    auto fuel = model->property("inertia/pointmass-weight-lbs");
    REQUIRE(model->beginMassChange());
    fuel.set(1120.0 / 0.45359237);
    REQUIRE(model->endMassChange(nullptr, nullptr));
    std::vector<PropertyHandle> compression;
    const int units = static_cast<int>(model->property("gear/num-units").get());
    for (int k = 0; k < units; ++k)
        compression.push_back(model->property("contact/unit[" + std::to_string(k) + "]/compression-ft"));
    bool touched = false, struck = false;
    double incomingMass = 0, incomingSpeed2 = 0;
    ipc::EngagementEvent impact{};
    for (unsigned k = 0; k < stepsFor(w, 5) && !struck; ++k) {
        const auto before = *w.vehicleState(id);
        const double mass = model->energy().massKg;
        w.step(1);
        if (!touched && std::any_of(compression.begin(), compression.end(), [](const auto& p) { return p.get() > 0; })) {
            REQUIRE_FALSE(w.vehicleState(id)->crashed);
            touched = true;
            incomingMass = mass;
            for (double v : before.velocityNedMs) incomingSpeed2 += v * v;
            // Represent a later leak/separation without changing contact,
            // force or fire coefficients: it cannot rewrite prior energy.
            REQUIRE(model->beginMassChange());
            fuel.set(560.0 / 0.45359237);
            REQUIRE(model->endMassChange(nullptr, nullptr));
        }
        for (const auto& event : w.takeEngagementEvents())
            if (static_cast<ipc::EngagementKind>(event.kind) == ipc::EngagementKind::Impact && event.slot == 0)
                impact = event, struck = true;
    }
    REQUIRE(touched);
    REQUIRE(struck);
    CHECK(impact.massKg == incomingMass);
    CHECK(std::abs(impact.energyJ / (0.5 * incomingMass * incomingSpeed2) - 1) < 1e-10);
}
