// The limits that follow the load (docs/vehicle-interface.md, 7.3): a wing's least airspeed by what it weighs, a turn's
// bank by its airspeed's margin over that, the speed floor where the airspeed asked sinks it - and what its activities
// say of it.
#include "mode_flights.h"

#include "control/Atmosphere.h"
#include "control/Protection.h"
#include "ipc/WorldMirror.h"
#include "fsim/GuidanceModes.h"
#include "fsim/PerformanceProfile.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

using namespace fsim;
using namespace fsim::control;
using namespace fsim::modes;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

double code(SpeedReference r) { return static_cast<double>(r); }

/// A wing in the air at a calibrated airspeed, level.
sim::VehicleState flying(double casMs, double bankRad = 0.0, double climbMs = 0.0) {
    sim::VehicleState s;
    s.airspeedCalibratedMs = casMs, s.airspeedTrueMs = casMs;
    s.eulerRad[0] = bankRad;
    s.velocityNedMs[2] = -climbMs;
    s.loadFactor = 1.0;
    return s;
}

Protection wingProtection(double casMinMs) {
    Protection p;
    p.mode = ProtectionMode::Limit;
    p.clean.casMinMs = casMinMs;
    p.flaps = p.clean;
    return p;
}

} // namespace

TEST_CASE("load limits: a turn's bank by its airspeed's margin over its least", "[load]") {
    // the least airspeed's load factor over 1.3, 15 deg at least: as a route's turns are flown
    CHECK(turnBankRadFor(100.0, std::numeric_limits<double>::quiet_NaN()) == std::numeric_limits<double>::infinity());
    CHECK_THAT(turnBankRadFor(60.0, 60.0) / kDeg, WithinAbs(15.0, 1e-9));  // at its least: the floor
    CHECK_THAT(turnBankRadFor(69.0, 60.0) / kDeg, WithinAbs(15.0, 1e-9));  // 1.15 times: still the floor
    CHECK_THAT(turnBankRadFor(78.0, 60.0) / kDeg, WithinAbs(39.7, 0.1));   // 1.3 times
    CHECK_THAT(turnBankRadFor(96.0, 60.0) / kDeg, WithinAbs(59.5, 0.1));   // 1.6 times
    double before = 0.0;
    for (double cas = 40.0; cas < 200.0; cas += 1.0) { // (never steeper slower)
        const double bank = turnBankRadFor(cas, 60.0);
        CHECK(bank >= before);
        before = bank;
    }

    const Protection p = wingProtection(60.0);
    // slow: a heading's bank and a turn rate held to it, and said so only while the turn is flown at it
    {
        const sim::VehicleState s = flying(78.0);
        const LimitState here = limitState(s, p.clean, p);
        CHECK_THAT(here.bankCapRad / kDeg, WithinAbs(39.7, 0.1));
        Command heading = AttitudeCommand{kHold, 0.05, 1.0, 60.0 * kDeg, kHold, 80.0};
        CHECK(limitSetpoint(heading, p.clean, p, here) == 0); // (wings level: a bank it does not ask for is not held back)
        CHECK_THAT(std::get<AttitudeCommand>(heading).maxBankRad / kDeg, WithinAbs(39.7, 0.1));
        const sim::VehicleState turning = flying(78.0, 39.5 * kDeg);
        Command again = AttitudeCommand{kHold, 0.05, 1.0, 60.0 * kDeg, kHold, 80.0};
        CHECK((limitSetpoint(again, p.clean, p, limitState(turning, p.clean, p)) & limitBit(Limit::Bank)) != 0);
        Command roll = AttitudeCommand{60.0 * kDeg, 0.05, kHold, kHold, kHold, 80.0};
        CHECK((limitSetpoint(roll, p.clean, p, here) & limitBit(Limit::Bank)) != 0);
        CHECK_THAT(std::get<AttitudeCommand>(roll).rollRad / kDeg, WithinAbs(39.7, 0.1));
        Command turn = VelocityCommand{80.0, 0.0, kHold, 0.5};
        CHECK((limitSetpoint(turn, p.clean, p, here) & limitBit(Limit::Bank)) != 0);
        CHECK_THAT(std::get<VelocityCommand>(turn).turnRateRadS, WithinRel(9.80665 * std::tan(here.bankCapRad) / 78.0, 1e-9));
    }
    // fast, on its wheels, or a rotorcraft: nothing bound
    {
        const sim::VehicleState s = flying(200.0);
        CHECK(limitState(s, p.clean, p).bankCapRad == std::numeric_limits<double>::infinity());
        Command heading = AttitudeCommand{kHold, 0.05, 1.0, 60.0 * kDeg, kHold, 200.0};
        CHECK(limitSetpoint(heading, p.clean, p, s) == 0);
        CHECK(std::get<AttitudeCommand>(heading).maxBankRad == 60.0 * kDeg);
        sim::VehicleState rolling = flying(70.0);
        rolling.onGround = true;
        CHECK(limitState(rolling, p.clean, p).bankCapRad == std::numeric_limits<double>::infinity());
        Protection rotor = p;
        rotor.wingborne = false;
        CHECK(limitState(flying(70.0), rotor.clean, rotor).bankCapRad == std::numeric_limits<double>::infinity());
    }
}

TEST_CASE("load limits: the speed floor raises an airspeed the aircraft sinks at, and keeps it until more is asked", "[load]") {
    const Protection p = wingProtection(60.0);
    SpeedFloor floor;
    auto update = [&](const sim::VehicleState& s, double askedMs, double climbAskedMs, bool clean = true) {
        LimitState here = limitState(s, p.clean, p);
        floor.active = floor.seen = false;
        here.floor = &floor, here.dt = 0.1, here.clean = clean;
        Command c = VelocityCommand{askedMs, climbAskedMs, 0.0, kHold};
        limitSetpoint(c, p.clean, p, here);
        return std::get<VelocityCommand>(c).airspeedMs;
    };
    // level and holding: nothing; sinking, asked level, slow: nothing for three seconds, then the floor from the speed it has
    for (int i = 0; i < 100; ++i) CHECK(update(flying(66.0), 66.0, 0.0) == 66.0);
    CHECK(floor.casMs == 0.0);
    for (int i = 0; i < 29; ++i) CHECK(update(flying(66.0, 0.0, -3.0), 66.0, 0.0) == 66.0);
    double flown = 66.0;
    for (int i = 0; i < 40; ++i) flown = update(flying(66.0 + 0.1 * i, 0.0, -3.0), 66.0, 0.0);
    CHECK(floor.active);
    CHECK(flown > 69.0);
    CHECK(floor.casMs <= kFloorMost * 60.0);
    // it sinks no more: the floor stays where it is, the airspeed asked still raised to it
    const double kept = floor.casMs;
    for (int i = 0; i < 100; ++i) CHECK(update(flying(kept), 66.0, 0.0) == kept);
    CHECK(floor.active);
    // a slight floor - a turn's sink: within 2 m/s of the airspeed asked - is let go once it has flown level at it, wings
    // level, for 15 s; not while it turns, and not one well above the airspeed asked (the 100 updates above)
    {
        SpeedFloor& slight = floor;
        const double was = slight.casMs;
        for (int i = 0; i < 200; ++i) CHECK(update(flying(was, 30.0 * kDeg), was - 1.0, 0.0) == was); // (banked: kept)
        for (int i = 0; i < 140; ++i) CHECK(update(flying(was), was - 1.0, 0.0) == was);
        for (int i = 0; i < 20; ++i) update(flying(was), was - 1.0, 0.0);
        CHECK(slight.casMs == 0.0); // (level 15 s: gone)
        CHECK(update(flying(was), was - 1.0, 0.0) == was - 1.0); // (the command flies its own)
        for (int i = 0; i < 40; ++i) update(flying(was - 1.0, 0.0, -3.0), was - 1.0, 0.0); // (sinking again: raised again)
        CHECK(slight.casMs > was - 1.0);
        slight.casMs = was, slight.steadyS = 0.0; // (as it was, for what follows)
    }
    // a command asks at least as much: the floor is gone, and the command flies its own
    CHECK(update(flying(kept), kept + 5.0, 0.0) == kept + 5.0);
    CHECK(floor.casMs == 0.0);
    CHECK_FALSE(floor.active);
    // asked to descend, fast, or being landed (flaps or gear out): a sink is no reason
    for (int i = 0; i < 100; ++i) update(flying(66.0, 0.0, -3.0), 66.0, -3.0);
    CHECK(floor.casMs == 0.0);
    for (int i = 0; i < 100; ++i) update(flying(90.0, 0.0, -3.0), 90.0, 0.0);
    CHECK(floor.casMs == 0.0);
    for (int i = 0; i < 100; ++i) update(flying(66.0, 0.0, -3.0), 66.0, 0.0, false);
    CHECK(floor.casMs == 0.0);
}

TEST_CASE("load limits: an F-16C's least airspeed follows what it carries", "[load]") {
    session::World w(options("load-f16c"));
    const auto id = wing(w, "f16c", 3000.0, 165.0);
    w.step();
    const Performance spawned = *w.performance(id);
    const double clean = w.massKg(id);
    CHECK(spawned.loadSpeedRatio == 1.0);
    CHECK(spawned.loadKg == 0.0);
    CHECK(spawned.minCasMs == w.profile(id)->envelope.clean.casMinMs); // (as it spawns: its profile's, to the bit)
    const std::uint32_t revision = w.controlRevision(id);

    REQUIRE(w.loadLoadout(id, "air superiority") == ms::Reason::None);
    REQUIRE(w.loadLoadout(id, "external tanks") == ms::Reason::None);
    w.step(2);
    const double loaded = w.massKg(id);
    REQUIRE(loaded > clean + 4000.0);
    const Performance heavy = *w.performance(id);
    const double ratio = std::sqrt(loaded / clean);
    CHECK_THAT(heavy.loadSpeedRatio, WithinAbs(ratio, 0.0026)); // (in steps of half a percent)
    CHECK(heavy.minCasMs == spawned.minCasMs * heavy.loadSpeedRatio);
    CHECK_THAT(heavy.loadKg, WithinAbs(loaded - clean, 13.0));  // (in steps of 25 kg)
    CHECK(heavy.revision > spawned.revision);
    CHECK(w.controlRevision(id) > revision);

    // the least a command may ask keeps its margin over that: a speed below is held to it - clamped, and under Reject
    // refused naming the limit; one at it is flown as asked
    CHECK(heavy.leastCasMs() == Performance::kLeastSpeedMargin * heavy.minCasMs);
    CommandOptions strict;
    strict.range = RangePolicy::Reject;
    for (const double below : {spawned.minCasMs, heavy.minCasMs, heavy.leastCasMs() - 0.5}) {
        HsaCommand slow;
        slow.speed = below, slow.speedReference = code(SpeedReference::CalibratedAirspeed);
        const auto clamped = w.submit(id, slow);
        REQUIRE(clamped.accepted());
        CHECK((clamped.flags & kClamped) != 0);
        const auto refused = w.submit(id, slow, strict);
        CHECK(refused.reason == Reason::PerformanceLimit);
        CHECK(refused.constraint == Constraint::MinAirspeed);
    }
    HsaCommand least;
    least.speed = heavy.leastCasMs() + 0.01, least.speedReference = code(SpeedReference::CalibratedAirspeed);
    const auto flown = w.submit(id, least, strict);
    REQUIRE(flown.accepted());
    CHECK((flown.flags & kClamped) == 0);

    // its tables are read at what it weighs: a profile at the weight now, where there was none beyond their heaviest row
    PerformanceProfile profile;
    REQUIRE(w.performanceProfile(id, FlightMode::HsaCsa, profile) == Reason::None);
    CHECK_THAT(profile.weightKg, WithinAbs(loaded, 5.0));
    REQUIRE_FALSE(profile.minAirspeed.empty());
    CHECK_FALSE(profile.maxAirspeed.empty());
    for (const auto& point : profile.minAirspeed) // (no less than a command may ask there)
        CHECK(point.value >= isa::trueFromCalibrated(heavy.leastCasMs(), point.altitudeMslM) - 1e-6);

    // unloaded again: as it spawned
    for (const auto& s : w.stores(id)) REQUIRE(w.loadStore(id, s.payload, s.station, "") == ms::Reason::None);
    w.step(2);
    CHECK(w.performance(id)->loadSpeedRatio == 1.0);
    CHECK_THAT(w.performance(id)->minCasMs, WithinRel(spawned.minCasMs, 1e-12));
}

TEST_CASE("load limits: commanded the least airspeed it may be at full load, a Mirage 2000 keeps its height - on its speed floor, saying so",
          "[load]") {
    // (tanks and missiles on a delta, on the back of its drag curve there: where it sinks, the floor takes it faster)
    session::World w(options("load-floor"));
    const auto id = wing(w, "mirage2000", 3000.0, 165.0);
    REQUIRE(w.loadLoadout(id, "air defence") == ms::Reason::None);
    REQUIRE(w.loadLoadout(id, "external tanks") == ms::Reason::None);
    w.step(2);
    HsaCommand h;
    h.headingRad = 0.5 * kPi, h.altitudeM = 3000.0;
    h.speed = w.performance(id)->leastCasMs(), h.speedReference = code(SpeedReference::CalibratedAirspeed);
    const auto r = w.submit(id, h);
    REQUIRE(r.accepted());
    double lowest = 3000.0;
    std::uint16_t seen = 0;
    for (int second = 0; second < 300; ++second) {
        w.step(stepsFor(w, 1.0));
        const sim::VehicleState& s = *w.vehicleState(id);
        REQUIRE_FALSE(s.crashed);
        lowest = std::min(lowest, s.altitudeMslM);
        seen = static_cast<std::uint16_t>(seen | w.activity(r.activity)->constraints);
    }
    const sim::VehicleState& s = *w.vehicleState(id);
    INFO("lowest " << lowest << " m; at the end " << s.altitudeMslM << " m, " << s.airspeedCalibratedMs << " m/s calibrated, asked " << h.speed);
    CHECK(lowest > 3000.0 - 60.0);
    CHECK(std::abs(s.altitudeMslM - 3000.0) < 60.0); // (back up past it as its thrust comes in, and slow to settle from there)
    CHECK(std::abs(s.velocityNedMs[2]) < 1.0);
    CHECK(w.activity(r.activity)->state == ActivityState::Active);
    if (s.airspeedCalibratedMs > h.speed + 2.0) { // (faster than asked: on its speed floor - and said)
        CHECK((seen & kActivityLowEnergy) != 0);
        CHECK((w.activity(r.activity)->constraints & kActivityLowEnergy) != 0);
    } else {
        CHECK((w.activity(r.activity)->constraints & kActivityLowEnergy) == 0);
    }
    CHECK((seen & kActivityUpset) == 0);
}

TEST_CASE("load limits: a turn at 1.3 times its least airspeed is flown within the bank that speed leaves", "[load]") {
    for (const char* type : {"f16c", "ea18g", "b52h"}) {
        INFO(type);
        session::World w(options("load-turn"));
        const auto id = wing(w, type, 3000.0, w.performance(wing(w, type, 3000.0, 150.0, 1))->cruiseTasMs);
        w.step();
        const double least = w.performance(id)->minCasMs;
        HsaCommand h;
        h.headingRad = 0.5 * kPi, h.altitudeM = 3000.0, h.speed = 1.3 * least, h.speedReference = code(SpeedReference::CalibratedAirspeed);
        const auto r = w.submit(id, h);
        REQUIRE(r.accepted());
        w.step(stepsFor(w, 150.0));
        REQUIRE(std::abs(w.vehicleState(id)->airspeedCalibratedMs - 1.3 * least) < 3.0);
        HsaCommand turn;
        turn.headingRad = 0.5 * kPi + 60.0 * kDeg;
        REQUIRE(w.update(r.activity, Command(turn)).accepted());
        double bank = 0.0, lowest = 3000.0, alpha = 0.0;
        for (int i = 0; i < 600; ++i) {
            w.step(stepsFor(w, 0.1));
            const sim::VehicleState& s = *w.vehicleState(id);
            bank = std::max(bank, std::abs(s.eulerRad[0]));
            lowest = std::min(lowest, s.altitudeMslM);
            alpha = std::max(alpha, s.alphaRad);
        }
        const sim::VehicleState& s = *w.vehicleState(id);
        INFO("bank " << bank / kDeg << " deg, lowest " << lowest << " m, alpha " << alpha / kDeg << " deg");
        CHECK(bank < 47.0 * kDeg); // (the 39.7 deg that speed leaves, and what its roll overshoots: it would stall at 53.7)
        CHECK(lowest > 3000.0 - 60.0);
        CHECK(std::abs(degreesApart(s.eulerRad[2], 0.5 * kPi + 60.0 * kDeg)) < 3.0);
        CHECK((w.activity(r.activity)->constraintsSeen & kActivityUpset) == 0);
    }
}

TEST_CASE("load limits: the viewer is given each vehicle's flight status, and its stores station by station", "[load]") {
    // what the viewer's mirror reads of a published world (ipc/WorldLayout.h, Derived): as the world has it
    session::WorldOptions o = options("load-mirror");
    o.publish = true;
    session::World w(o);
    const auto viper = wing(w, "f16c", 3000.0, 165.0);
    const auto growler = wing(w, "ea18g", 3000.0, 180.0, 1);
    w.step();
    REQUIRE(w.loadLoadout(viper, "air superiority") == ms::Reason::None);
    REQUIRE(w.loadLoadout(viper, "external tanks") == ms::Reason::None);
    REQUIRE(w.loadLoadout(growler, "jamming pods") == ms::Reason::None);
    REQUIRE(w.loadLoadout(growler, "external tanks") == ms::Reason::None);
    w.step(2);
    w.publishNow();
    ipc::WorldMirror mirror;
    REQUIRE(mirror.open("load-mirror"));
    sim::SnapshotBatch batch;
    REQUIRE(mirror.pollSnapshot(batch));
    REQUIRE(mirror.status().size() >= 2);
    const Performance& p = *w.performance(viper);
    const double* d = mirror.status()[0].derived;
    CHECK_THAT(d[ipc::kDerivedMassKg], WithinAbs(w.massKg(viper), 1.0));
    CHECK(d[ipc::kDerivedStallCasMs] == p.minCasMs);
    CHECK(d[ipc::kDerivedLeastCasMs] == p.leastCasMs());
    CHECK(d[ipc::kDerivedLoadRatio] == p.loadSpeedRatio);
    CHECK(p.loadSpeedRatio > 1.1);
    CHECK(d[ipc::kDerivedFloorCasMs] == 0.0);
    CHECK(d[ipc::kDerivedConstraints] == 0.0);
    CHECK(d[ipc::kDerivedBankCapRad] > 1.0); // (at 165 m/s: more than its bank limit)
    // slow, the bank its airspeed leaves it; an activity's flags as its record has them
    HsaCommand h;
    h.headingRad = 0.5 * kPi, h.altitudeM = 3000.0, h.speed = 1.2 * p.minCasMs, h.speedReference = code(SpeedReference::CalibratedAirspeed);
    const auto r = w.submit(viper, h);
    REQUIRE(r.accepted());
    w.step(stepsFor(w, 150.0));
    w.publishNow();
    REQUIRE(mirror.pollSnapshot(batch));
    d = mirror.status()[0].derived;
    const sim::VehicleState& s = *w.vehicleState(viper);
    CHECK_THAT(d[ipc::kDerivedBankCapRad], WithinAbs(turnBankRadFor(s.airspeedCalibratedMs, w.performance(viper)->minCasMs), 0.02));
    CHECK(d[ipc::kDerivedBankCapRad] < 40.0 * kDeg);
    CHECK(static_cast<std::uint16_t>(d[ipc::kDerivedConstraints]) == w.activity(r.activity)->constraints);
    // the Growler's three jamming pods and two tanks: each a store with its card, for the viewer to hang on its station
    ipc::MissionRecord m;
    REQUIRE(mirror.mission(1, m));
    std::string stations;
    for (std::uint32_t i = 0; i < m.storeCount; ++i) {
        const ipc::MissionStore& st = m.stores[i];
        stations += std::to_string(st.station) + ":" + st.store + " ";
        CHECK(st.status == static_cast<std::uint8_t>(ms::StoreStatus::Loaded));
    }
    CHECK(stations == "3:an-alq-99 4:fpu-11 6:an-alq-99 8:fpu-11 9:an-alq-99 ");
}
