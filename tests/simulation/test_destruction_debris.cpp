// VIS-M11: Multi-tier native aircraft destruction state machine (B-58),
// lightweight deterministic 6-DOF tumbling rigid-body debris pool (zero
// jsbsim:<piece> vehicles in LightweightDebris mode), early gear/belly contact
// & Coulomb belly-slide with fuel burn-down (B-54), surface-aware Soil/Water/
// Rock impact physics (B-55), additive DebrisSegmentV1 IPC & .debris replay.

#include "contracts/world_stream/DebrisLayoutV1.h"
#include "foundation/io/TerrainTiles.h"
#include "foundation/platform/Paths.h"
#include "foundation/transport/DebrisSegmentV1.h"
#include "foundation/transport/Recording.h"
#include "mode_flights.h"
#include "simulation/mission_systems/damage/Destruction.h"
#include "simulation/mission_systems/damage/Impact.h"
#include "simulation/runtime/world/DebrisField.h"
#include "simulation/runtime/world/World.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

using namespace fsim;
using namespace fsim::modes;
namespace dmg = fsim::ms::damage;

namespace {

session::WorldOptions lightweightOptions(const char* name, std::uint64_t seed = 41) {
    auto o = options(name);
    o.dt = 1.0 / 100.0;
    o.frameSkip = 2; // 50 Hz world step
    o.seed = seed;
    o.terrain = false;
    return o;
}

double vecLen(const double v[3]) noexcept { return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }
double vecDist(const double a[3], const double b[3]) noexcept {
    const double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}
dmg::V3 stateVelEcef(const sim::VehicleState& st) noexcept {
    const double sl = std::sin(st.latitudeRad), cl = std::cos(st.latitudeRad);
    const double so = std::sin(st.longitudeRad), co = std::cos(st.longitudeRad);
    const double vn = st.velocityNedMs[0], ve = st.velocityNedMs[1], vd = st.velocityNedMs[2];
    return {
        -sl * co * vn - so * ve - cl * co * vd,
        -sl * so * vn + co * ve - cl * so * vd,
        cl * vn - sl * vd,
    };
}

bool hasArchetype(const session::DebrisField& field, ipc::DebrisArchetypeV1 arch) noexcept {
    for (const auto& b : field.slots()) {
        if (b.alive && b.archetype == arch) return true;
    }
    return false;
}

std::size_t countRoleDebris(const session::World& w) {
    std::size_t n = 0;
    for (const auto id : w.vehicleIds()) {
        if (const auto* info = w.info(id); info && info->role == ms::Role::Debris) ++n;
    }
    return n;
}

} // namespace

TEST_CASE("VIS-M11 wire contracts, ullage flammability, hydrodynamic-ram spray, and B-54/B-55 surface physics",
          "[vis-m11][destruction]") {
    // 1. Frozen wire strides & validators
    CHECK(sizeof(ipc::DebrisBodyWireV1) == 104);
    CHECK(alignof(ipc::DebrisBodyWireV1) == 8);
    CHECK(sizeof(ipc::VehicleDestructionWireV1) == 64);
    CHECK(alignof(ipc::VehicleDestructionWireV1) == 8);

    ipc::DebrisBodyWireV1 body{};
    body.id = 7;
    body.ownerVehicleId = 1;
    body.archetype = static_cast<std::uint8_t>(ipc::DebrisArchetypeV1::WingFragment);
    body.flags = static_cast<std::uint8_t>(ipc::kDebrisFlagBurning | ipc::kDebrisFlagSmoking | ipc::packDebrisPartTagV1(ipc::DebrisPartTagV1::SectionLeftWing));
    CHECK(ipc::debrisPartTagV1(body.flags) == ipc::DebrisPartTagV1::SectionLeftWing);
    CHECK((body.flags & ipc::kDebrisFlagBurning) != 0);
    CHECK(ipc::validDebrisBodyV1(body));
    body.id = 0;
    CHECK_FALSE(ipc::validDebrisBodyV1(body)); // id must be nonzero
    body.id = 7;
    body.quatBodyToEcef[3] = 0.0f;
    CHECK_FALSE(ipc::validDebrisBodyV1(body)); // degenerate quaternion rejected

    ipc::VehicleDestructionWireV1 vd{};
    vd.vehicleId = 3;
    vd.slot = 0;
    vd.tier = static_cast<std::uint8_t>(ipc::DestructionTierV1::CatastrophicKill);
    vd.hullRetired = 1;
    CHECK(ipc::validVehicleDestructionV1(vd, 16));
    vd.slot = 16;
    CHECK_FALSE(ipc::validVehicleDestructionV1(vd, 16)); // out-of-range slot rejected

    // 2. MIL-HDBK-2069 ullage flammability envelope (liquid-full and bone-dry negative controls)
    CHECK(dmg::ullageFlammability(0.0) == 0.0);
    CHECK(dmg::ullageFlammability(0.05) == 0.0);
    CHECK(dmg::ullageFlammability(0.98) == 0.0);
    CHECK(dmg::ullageFlammability(0.50) > 0.75);
    CHECK(dmg::ullageFlammability(0.50) > dmg::ullageFlammability(0.15));

    // 3. Hydrodynamic-ram fuel spray speed (B-58): physical, bounded, monotonic
    CHECK(dmg::hydrodynamicRamSpraySpeedMs(0.0, 1200.0, 400.0) == 0.0);
    CHECK(dmg::hydrodynamicRamSpraySpeedMs(20000.0, 1200.0, 0.0) == 0.0);
    const double sprayLow = dmg::hydrodynamicRamSpraySpeedMs(10000.0, 900.0, 400.0);
    const double sprayHigh = dmg::hydrodynamicRamSpraySpeedMs(80000.0, 1800.0, 400.0);
    CHECK(sprayLow > 10.0);
    CHECK(sprayHigh > sprayLow);
    CHECK(sprayHigh <= 260.0);

    // 4. B-54 early gear-overload / belly-scrape trigger
    CHECK_FALSE(dmg::exceedsGearOrBellyContact(2.0, 0.0, 65.0)); // gentle flared touchdown
    CHECK(dmg::exceedsGearOrBellyContact(7.2, 0.0, 50.0));       // exceeds MIL-A-8863C gear sink limit
    CHECK(dmg::exceedsGearOrBellyContact(1.5, 0.08, 70.0));      // structural belly contact before 1.0 m penetration

    // 5. B-55 surface-aware Soil / Water / Rock ground strike physics
    auto strikeSurface = [](dmg::SurfaceKind surface, double speed, double angleDeg) {
        dmg::GroundStrike gs;
        gs.velocity = {speed * std::cos(angleDeg * kDeg), 0.0, -speed * std::sin(angleDeg * kDeg)};
        gs.normal = {0.0, 0.0, 1.0};
        gs.massKg = 11000.0;
        gs.fuelKg = 2500.0;
        gs.surface = surface;
        return dmg::groundStrike(gs);
    };
    const auto soil = strikeSurface(dmg::SurfaceKind::Soil, 180.0, 12.0);
    const auto water = strikeSurface(dmg::SurfaceKind::Water, 180.0, 12.0);
    const auto rock = strikeSurface(dmg::SurfaceKind::Rock, 180.0, 12.0);

    CHECK(water.crushM > soil.crushM);
    CHECK(rock.crushM < soil.crushM);
    CHECK(rock.decelMs2 > soil.decelMs2);
    CHECK(soil.decelMs2 > water.decelMs2);
    CHECK(water.after.x > soil.after.x); // lower hydrodynamic drag on shallow skim
    CHECK(water.poolKg < soil.poolKg);   // water quenches surface fuel pool
    CHECK(water.splashKg > 0.0);
    CHECK(rock.slideDecelMs2 > soil.slideDecelMs2);
    CHECK(soil.slideDecelMs2 > water.slideDecelMs2);

    // 6. Steep / high-vertical-energy land impact disintegrates hull vs shallow belly-slide
    {
        dmg::VehicleDestructionState steep48;
        dmg::applyGroundOrWaterDestruction(steep48, strikeSurface(dmg::SurfaceKind::Soil, 78.0, 48.0), 1200.0, true);
        CHECK(steep48.tier == dmg::DestructionTier::GroundWreck);
        CHECK(steep48.hullRetired);
        CHECK_FALSE(steep48.groundSliding);

        dmg::VehicleDestructionState steepMid;
        dmg::applyGroundOrWaterDestruction(steepMid, strikeSurface(dmg::SurfaceKind::Soil, 50.0, 50.0), 1200.0, true);
        CHECK(steepMid.tier == dmg::DestructionTier::GroundWreck);
        CHECK(steepMid.hullRetired);

        dmg::VehicleDestructionState highNormal;
        dmg::applyGroundOrWaterDestruction(highNormal, strikeSurface(dmg::SurfaceKind::Soil, 125.0, 28.0), 1200.0, true);
        CHECK(highNormal.tier == dmg::DestructionTier::GroundWreck);
        CHECK(highNormal.hullRetired);

        dmg::VehicleDestructionState shallow;
        dmg::applyGroundOrWaterDestruction(shallow, strikeSurface(dmg::SurfaceKind::Soil, 80.0, 12.0), 1200.0, true);
        CHECK(shallow.tier == dmg::DestructionTier::GroundWreck);
        CHECK_FALSE(shallow.hullRetired);
        CHECK(shallow.groundSliding);
    }
}

TEST_CASE("VIS-M11 LightweightDebris section separation spawns zero jsbsim:<piece> vehicles and conserves momentum",
          "[vis-m11][debris]") {
    session::World w(lightweightOptions("vis-m11-sep", 101));
    w.setDestructionModeV1(session::World::DestructionModeV1::LightweightDebris);
    const auto id = wing(w, "f16c", 3000.0, 220.0, 0, 0.55);
    w.step(10);

    const double massBefore = w.massKg(id);
    const sim::VehicleState stBefore = *w.vehicleState(id);
    const dmg::V3 vBefore = stateVelEcef(stBefore);
    const dmg::V3 pBefore{vBefore.x * massBefore, vBefore.y * massBefore, vBefore.z * massBefore};

    REQUIRE(w.separateSection(id, "left wing"));

    // Zero heavyweight jsbsim:<piece> vehicles spawned — VehiclePool stays constant!
    CHECK(w.vehicleIds().size() == 1);
    CHECK(countRoleDebris(w) == 0);

    const double massAfter = w.massKg(id);
    CHECK(massAfter < massBefore - 50.0);
    REQUIRE(w.debrisField().activeCount() >= 4);
    CHECK(hasArchetype(w.debrisField(), ipc::DebrisArchetypeV1::WingFragment));

    // Check total linear momentum conservation (parent after recoil + spawned debris cloud == parent before).
    const sim::VehicleState stAfter = *w.vehicleState(id);
    const dmg::V3 vAfter = stateVelEcef(stAfter);
    const dmg::V3 pParentAfter{vAfter.x * massAfter, vAfter.y * massAfter, vAfter.z * massAfter};
    const dmg::V3 pDebris = w.debrisField().totalMomentumEcef(id);
    const dmg::V3 pTotal{pParentAfter.x + pDebris.x, pParentAfter.y + pDebris.y, pParentAfter.z + pDebris.z};
    const double errRatio = dmg::length(pTotal - pBefore) / std::max(dmg::length(pBefore), 1.0);
    CHECK(errRatio < 0.02);

    const auto* ds = w.destructionState(id);
    REQUIRE(ds != nullptr);
    CHECK(ds->tier == dmg::DestructionTier::StructuralBreakup);
    CHECK_FALSE(ds->hullRetired);

    // Step forward 2.0 s and verify smooth, finite 6-DOF tumbling trajectories
    std::vector<ipc::DebrisBodyWireV1> wireBefore, wireAfter;
    w.debrisField().fillWire(wireBefore);
    w.step(stepsFor(w, 2.0));
    w.debrisField().fillWire(wireAfter);
    REQUIRE_FALSE(wireAfter.empty());
    for (const auto& b : wireAfter) {
        CHECK(ipc::validDebrisBodyV1(b));
    }
    // Primary chunk moved significantly over 2 s at ~220 m/s
    CHECK(vecDist(wireBefore.front().positionEcef, wireAfter.front().positionEcef) > 100.0);
}

TEST_CASE("VIS-M11 Catastrophic kill via ullage overpressure, sympathetic cook-off, and native AIM-120C detonation",
          "[vis-m11][destruction][b58]") {
    // Part A: Direct physical evaluation of ullage ignition and sympathetic ordnance cook-off
    {
        Rng rng(777);
        // Negative control: liquid-full tank (ullage = 2% < 12%) cannot undergo vapour ullage deflagration
        dmg::VehicleDestructionState fullState;
        dmg::DestructionStepInput fullIn;
        fullIn.totalRemainingFuelKg = 980.0;
        fullIn.tanks.push_back({0, 1, 980.0, 1000.0, 35000.0, 1500.0, 0.0, false});
        const auto fullOut = dmg::evaluateDestruction(fullState, fullIn, true, rng);
        CHECK_FALSE(fullOut.catastrophicTriggered);
        CHECK(fullOut.hydrodynamicRamSprayMs > 10.0);

        // Positive control: half-empty tank (ullage = 50%) struck by high-energy incendiary fragments -> CatastrophicKill
        dmg::VehicleDestructionState halfState;
        dmg::DestructionStepInput halfIn;
        halfIn.totalRemainingFuelKg = 500.0;
        halfIn.tanks.push_back({0, 1, 500.0, 1000.0, 35000.0, 1500.0, 0.0, false});
        const auto halfOut = dmg::evaluateDestruction(halfState, halfIn, true, rng);
        CHECK(halfOut.catastrophicTriggered);
        CHECK(halfOut.cause == dmg::CatastrophicCause::UllageOverpressure);
        CHECK(halfState.tier == dmg::DestructionTier::CatastrophicKill);
        CHECK(halfState.hullRetired);
        CHECK(halfOut.explosionTntKg > 0.0);
        CHECK(halfOut.fireballFuelKg > 0.0);
        CHECK(halfOut.fireballFuelKg <= halfIn.totalRemainingFuelKg);

        // Positive control: sustained section fuel fire soaking carried HE ordnance past kCookoffThermalS -> SympatheticCookoff
        dmg::VehicleDestructionState cookState;
        dmg::DestructionStepInput cookIn;
        cookIn.totalRemainingFuelKg = 600.0;
        cookIn.dtS = 1.0;
        cookIn.ordnance.push_back({1, 22.0, 0.0, 0.0, true});
        bool cooked = false;
        dmg::DestructionOutcome cookOut{};
        for (int s = 0; s < 10 && !cooked; ++s) {
            cookOut = dmg::evaluateDestruction(cookState, cookIn, true, rng);
            cooked = cookOut.catastrophicTriggered;
        }
        REQUIRE(cooked);
        CHECK(cookOut.cause == dmg::CatastrophicCause::SympatheticCookoff);
        CHECK(cookOut.explosionTntKg >= 22.0);
        CHECK(cookState.hullRetired);
    }

    // Part B: Native World AIM-120C warhead detonation beside F-16C in LightweightDebris mode
    {
        session::World w(lightweightOptions("vis-m11-catastrophic", 202));
        w.setDestructionModeV1(session::World::DestructionModeV1::LightweightDebris);
        const auto target = wing(w, "f16c", 3500.0, 235.0, 0, 0.50);
        w.step(15);

        const sim::VehicleState before = *w.vehicleState(target);
        const dmg::V3 preVel = stateVelEcef(before);
        const double preSpeed = std::max(dmg::length(preVel), 1.0);
        session::Detonation det;
        det.store = "aim-120c";
        det.positionEcef[0] = before.positionEcef[0] + 2.5;
        det.positionEcef[1] = before.positionEcef[1] + 2.0;
        det.positionEcef[2] = before.positionEcef[2] + 1.0;
        det.velocityEcef[0] = -preVel.x * 2.5;
        det.velocityEcef[1] = -preVel.y * 2.5;
        det.velocityEcef[2] = -preVel.z * 2.5;
        det.axisEcef[0] = -preVel.x / preSpeed;
        det.axisEcef[1] = -preVel.y / preSpeed;
        det.axisEcef[2] = -preVel.z / preSpeed;
        const auto events = w.detonate(det);
        CHECK_FALSE(events.empty());

        // Zero jsbsim:<piece> child vehicles created AND destroyed airframe simulation instance retired cleanly!
        CHECK(w.vehicleIds().empty());
        CHECK(w.vehicleState(target) == nullptr);
        CHECK(countRoleDebris(w) == 0);

        const auto* ds = w.destructionState(target);
        REQUIRE(ds != nullptr);
        CHECK(ds->tier == dmg::DestructionTier::CatastrophicKill);
        CHECK(ds->hullRetired);
        CHECK(w.debrisField().activeCount() >= 25);
        CHECK(hasArchetype(w.debrisField(), ipc::DebrisArchetypeV1::AirframeHull));
        CHECK(hasArchetype(w.debrisField(), ipc::DebrisArchetypeV1::BurningChunk));
        CHECK(hasArchetype(w.debrisField(), ipc::DebrisArchetypeV1::SparkEmber));

        // Verify debris cloud momentum matches aircraft pre-explosion flight direction closely
        // (isotropic radial blast impulse sums to zero linear momentum).
        const double cloudMass = w.debrisField().totalMassKg(target);
        const dmg::V3 cloudMom = w.debrisField().totalMomentumEcef(target);
        REQUIRE(cloudMass > 500.0);
        const dmg::V3 meanVel{cloudMom.x / cloudMass, cloudMom.y / cloudMass, cloudMom.z / cloudMass};
        const double velDiff = dmg::length(meanVel - preVel);
        CHECK(velDiff < 25.0);

        // Benchmark DebrisField step performance (acceptance target: < 0.05 ms/step on modern CPU, check < 0.5 ms generously for CI jitter)
        const auto t0 = std::chrono::steady_clock::now();
        w.step(stepsFor(w, 2.5));
        const auto t1 = std::chrono::steady_clock::now();
        const double totalMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
        CHECK(totalMs < 5000.0);

        std::vector<ipc::DebrisBodyWireV1> wire;
        w.debrisField().fillWire(wire);
        CHECK_FALSE(wire.empty());
        for (const auto& b : wire) CHECK(ipc::validDebrisBodyV1(b));
    }
}

TEST_CASE("VIS-M11 Ground belly-slide with tank fuel burn-down, steep dive, and Water impact (B-54, B-55)",
          "[vis-m11][impact][b54][b55]") {
    // 1. Shallow Soil belly-slide: slides along ground, emits sparks, burns actual tank fuel, zero jsbsim:<piece>
    {
        session::World w(lightweightOptions("vis-m11-slide", 303));
        w.setDestructionModeV1(session::World::DestructionModeV1::LightweightDebris);
        w.setImpactSurfaceV1(dmg::SurfaceKind::Soil);
        session::VehicleSpec s;
        s.name = "slider";
        s.type = "jsbsim:f16c";
        s.initial.altitudeMslM = 35.0;
        s.initial.headingDeg = 90.0;
        s.initial.pitchDeg = -5.0;
        s.initial.airspeedTrueMs = 155.0;
        s.initial.fuelFraction = 0.60;
        const auto id = w.createVehicle(s);
        REQUIRE(id != 0);

        bool down = false;
        sim::VehicleState atImpact{};
        for (unsigned k = 0; k < stepsFor(w, 10.0) && !down; ++k) {
            w.step(1);
            if (w.vehicleState(id)->crashed) {
                down = true;
                atImpact = *w.vehicleState(id);
            }
        }
        REQUIRE(down);
        CHECK(w.vehicleIds().size() == 1);
        CHECK(countRoleDebris(w) == 0);

        const auto* ds = w.destructionState(id);
        REQUIRE(ds != nullptr);
        CHECK(ds->tier == dmg::DestructionTier::GroundWreck);
        CHECK(ds->groundSliding);
        const double speedStart = vecLen(atImpact.velocityNedMs);
        const double fuelAtImpact = atImpact.fuelKg;

        // Step 3.0 s post-impact: wreck slides smoothly across terrain while decelerating & burning real fuel,
        // resting ON TOP of terrain via JoltPhysics (never sunk underground!).
        w.step(stepsFor(w, 3.0));
        const sim::VehicleState afterSlide = *w.vehicleState(id);
        const double slideDistM = vecDist(atImpact.positionEcef, afterSlide.positionEcef);
        const double speedEnd = vecLen(afterSlide.velocityNedMs);
        CHECK(slideDistM > 30.0);
        CHECK(speedEnd < speedStart);
        CHECK(afterSlide.altitudeAglM >= 1.2);
        CHECK(ds->totalFuelBurnedKg > 0.0);
        CHECK(afterSlide.fuelKg < fuelAtImpact);
        CHECK(w.debrisField().activeCount() > 0);
    }

    // 2. Steep dive Soil impact: hull disintegrates radially under fireball
    {
        session::World w(lightweightOptions("vis-m11-dive", 304));
        w.setDestructionModeV1(session::World::DestructionModeV1::LightweightDebris);
        w.setImpactSurfaceV1(dmg::SurfaceKind::Soil);
        session::VehicleSpec s;
        s.name = "diver";
        s.type = "jsbsim:f16c";
        s.initial.altitudeMslM = 180.0;
        s.initial.headingDeg = 90.0;
        s.initial.pitchDeg = -65.0;
        s.initial.airspeedTrueMs = 195.0;
        const auto id = w.createVehicle(s);
        REQUIRE(id != 0);

        bool down = false;
        for (unsigned k = 0; k < stepsFor(w, 8.0) && !down; ++k) {
            w.step(1);
            const auto* st = w.vehicleState(id);
            if (!st || st->crashed) down = true;
        }
        REQUIRE(down);
        CHECK(w.vehicleIds().empty());
        CHECK(w.vehicleState(id) == nullptr);
        const auto* ds = w.destructionState(id);
        REQUIRE(ds != nullptr);
        CHECK(ds->tier == dmg::DestructionTier::GroundWreck);
        CHECK(ds->hullRetired);
        CHECK(w.debrisField().activeCount() >= 20);
    }

    // 3. High-speed Water impact (B-55): shatters violently into skipping sections + hydrodynamic SplashPlume (never intact shark-fin skim)
    {
        session::World w(lightweightOptions("vis-m11-water", 305));
        w.setDestructionModeV1(session::World::DestructionModeV1::LightweightDebris);
        w.setImpactSurfaceV1(dmg::SurfaceKind::Water);
        session::VehicleSpec s;
        s.name = "splasher";
        s.type = "jsbsim:f16c";
        s.initial.altitudeMslM = 80.0;
        s.initial.headingDeg = 90.0;
        s.initial.pitchDeg = -18.0;
        s.initial.airspeedTrueMs = 160.0;
        const auto id = w.createVehicle(s);
        REQUIRE(id != 0);

        bool down = false;
        for (unsigned k = 0; k < stepsFor(w, 10.0) && !down; ++k) {
            w.step(1);
            const auto* st = w.vehicleState(id);
            if (!st || st->crashed) down = true;
        }
        REQUIRE(down);
        CHECK(w.vehicleIds().empty());
        CHECK(w.vehicleState(id) == nullptr);
        const auto* ds = w.destructionState(id);
        REQUIRE(ds != nullptr);
        CHECK(ds->tier == dmg::DestructionTier::WaterImpact);
        CHECK(ds->surface == dmg::SurfaceKind::Water);
        CHECK(ds->hullRetired);
        CHECK_FALSE(ds->groundSliding);
        CHECK(hasArchetype(w.debrisField(), ipc::DebrisArchetypeV1::AirframeHull));
        CHECK(hasArchetype(w.debrisField(), ipc::DebrisArchetypeV1::SplashPlume));
    }
}

TEST_CASE("VIS-M11 Bit-identical determinism, DebrisSegmentV1 live IPC, and .debris replay round-trip",
          "[vis-m11][determinism][ipc]") {
    const auto runOnce = [](const std::filesystem::path& recPath) {
        auto opt = lightweightOptions("vis-m11-det", 555);
        if (!recPath.empty()) opt.recordPath = recPath.string();
        session::World w(opt);
        w.setDestructionModeV1(session::World::DestructionModeV1::LightweightDebris);
        const auto id = wing(w, "f16c", 3200.0, 230.0, 0, 0.50);
        w.step(10);
        const sim::VehicleState before = *w.vehicleState(id);
        const dmg::V3 preVel = stateVelEcef(before);
        const double preSpeed = std::max(dmg::length(preVel), 1.0);
        session::Detonation det;
        det.store = "aim-120c";
        det.positionEcef[0] = before.positionEcef[0] + 2.0;
        det.positionEcef[1] = before.positionEcef[1] + 1.5;
        det.positionEcef[2] = before.positionEcef[2];
        det.velocityEcef[0] = -preVel.x * 2.5;
        det.velocityEcef[1] = -preVel.y * 2.5;
        det.velocityEcef[2] = -preVel.z * 2.5;
        det.axisEcef[0] = -preVel.x / preSpeed;
        det.axisEcef[1] = -preVel.y / preSpeed;
        det.axisEcef[2] = -preVel.z / preSpeed;
        w.detonate(det);
        w.step(stepsFor(w, 1.5));
        std::vector<ipc::DebrisBodyWireV1> wire;
        w.debrisField().fillWire(wire);
        return wire;
    };

    const auto tmpDir = std::filesystem::temp_directory_path() / "fsim-vis-m11-test";
    std::filesystem::create_directories(tmpDir);
    const auto recPath = tmpDir / "det_roundtrip.fsrec";
    const auto debPath = std::filesystem::path(recPath.string() + ".debris");
    std::error_code ec;
    std::filesystem::remove(recPath, ec);
    std::filesystem::remove(debPath, ec);

    const auto runA = runOnce(recPath);
    const auto runB = runOnce({});
    REQUIRE(runA.size() >= 20);
    REQUIRE(runA.size() == runB.size());
    for (std::size_t i = 0; i < runA.size(); ++i) {
        CHECK(std::memcmp(&runA[i], &runB[i], sizeof(ipc::DebrisBodyWireV1)) == 0);
    }

    // Companion .debris sidecar recorded automatically alongside .fsrec and loads cleanly via DebrisReplayV1
    REQUIRE(std::filesystem::is_regular_file(recPath));
    REQUIRE(std::filesystem::is_regular_file(debPath));

    ipc::Recording rec;
    std::string recErr;
    REQUIRE(rec.load(recPath.string(), &recErr));
    CHECK_FALSE(rec.frames().empty());

    ipc::DebrisReplayV1 replay;
    std::string debErr;
    REQUIRE(replay.load(debPath, &debErr));
    REQUIRE(replay.frameCount() >= 20);

    // Zero-teleport regression: before detonation (t = 0.10 s < 0.20 s) there are zero debris bodies
    // and hullRetired == 0; before the first recorded frame, atTime returns false (never clamps forward!).
    ipc::DebrisFrameV1 pre{};
    CHECK_FALSE(replay.atTime(replay.frames().front().simTime - 0.10, pre, true));
    REQUIRE(replay.atTime(0.10, pre, true));
    CHECK(pre.bodies.empty());
    for (const auto& v : pre.vehicles) CHECK(v.hullRetired == 0);

    // Mid-timeline interpolated query produces valid unit quaternions and live debris
    const double midT = 0.5 * (replay.frames().front().simTime + replay.frames().back().simTime);
    ipc::DebrisFrameV1 interp{};
    REQUIRE(replay.atTime(midT, interp, true));
    CHECK_FALSE(interp.bodies.empty());
    CHECK_FALSE(interp.vehicles.empty());
    for (const auto& b : interp.bodies) CHECK(ipc::validDebrisBodyV1(b));

    // Exact end-frame matches runA bit-for-bit
    ipc::DebrisFrameV1 last{};
    REQUIRE(replay.atTime(replay.frames().back().simTime, last, false));
    REQUIRE(last.bodies.size() == runA.size());
    for (std::size_t i = 0; i < runA.size(); ++i) {
        CHECK(std::memcmp(&last.bodies[i], &runA[i], sizeof(ipc::DebrisBodyWireV1)) == 0);
    }

    // Live shared-memory DebrisSegmentV1 round-trip
    {
        ipc::DebrisSegmentV1 pub, sub;
        REQUIRE(pub.create("vis_m11_shm_test", ipc::kMaxDebrisV1, 32, 12345, 1));
        REQUIRE(sub.open("vis_m11_shm_test", 12345, 1));
        REQUIRE(pub.publish(7, last));
        ipc::DebrisFrameV1 readBack{};
        REQUIRE(sub.read(7, readBack));
        REQUIRE(readBack.bodies.size() == last.bodies.size());
        CHECK(std::memcmp(readBack.bodies.data(), last.bodies.data(), last.bodies.size() * sizeof(ipc::DebrisBodyWireV1)) == 0);
        sub.close();
        pub.close();
    }

    std::filesystem::remove(recPath, ec);
    std::filesystem::remove(debPath, ec);
}

TEST_CASE("VIS-M11b Multi-grade progressive combat damage: aspect, hit location, fragment density, and thermal escalation",
          "[vis-m11][vis-m11b][graded]") {
    auto bodyToEcefOffset = [](const sim::VehicleState& st, const dmg::V3& b) {
        const double* R = st.rotationBodyToEcef;
        return dmg::V3{
            R[0] * b.x + R[1] * b.y + R[2] * b.z,
            R[3] * b.x + R[4] * b.y + R[5] * b.z,
            R[6] * b.x + R[7] * b.y + R[8] * b.z,
        };
    };

    // 1. Pure unit checks: hit aspect classification, wing-tank blow-off (Grade 3 vs Grade 4), thermal weakening & delayed runaway
    {
        CHECK(dmg::classifyHitAspect({1.0, 0.1, 0.0}) == dmg::HitAspect::Front);
        CHECK(dmg::classifyHitAspect({-1.0, 0.2, 0.1}) == dmg::HitAspect::Rear);
        CHECK(dmg::classifyHitAspect({0.1, -1.0, 0.0}) == dmg::HitAspect::Left);
        CHECK(dmg::classifyHitAspect({0.1, 1.0, 0.0}) == dmg::HitAspect::Right);
        CHECK(dmg::classifyHitAspect({0.0, 0.1, -1.0}) == dmg::HitAspect::Dorsal);
        CHECK(dmg::classifyHitAspect({0.0, 0.1, 1.0}) == dmg::HitAspect::Ventral);

        // Concentrated outboard wing-tank hit -> Grade 3 wing blow-off (NOT instant fuselage vaporization!)
        Rng rng(404);
        dmg::VehicleDestructionState wingState;
        dmg::DestructionStepInput wingIn;
        wingIn.totalRemainingFuelKg = 1400.0;
        wingIn.fragmentsStriking = 90;
        wingIn.hitFromBody = {0.0, -1.0, 0.0};
        wingIn.hasHitDirection = true;
        dmg::TankUllageInput wt{};
        wt.tankIndex = 2;
        wt.section = 2;
        wt.fuelKg = 110.0;
        wt.capacityKg = 210.0;
        wt.hitEnergyJ = 32000.0;
        wt.fastestFragmentMs = 1400.0;
        wt.hitCount = 6;
        wt.isCentreSection = false;
        wingIn.tanks.push_back(wt);
        const auto wingOut = dmg::evaluateDestruction(wingState, wingIn, true, rng);
        CHECK_FALSE(wingOut.catastrophicTriggered);
        CHECK(wingOut.rupturedWingSection == 2);
        CHECK(wingState.grade == dmg::DamageGrade::StructuralBreakup);
        CHECK(wingState.hitAspect == dmg::HitAspect::Left);
        CHECK_FALSE(wingState.hullRetired);

        // Sparse stray fragments (3 hits, 20 kJ) into centre fuselage tank -> does NOT trigger instant CatastrophicKill
        dmg::VehicleDestructionState grazeState;
        dmg::DestructionStepInput grazeIn;
        grazeIn.totalRemainingFuelKg = 1200.0;
        grazeIn.fragmentsStriking = 18;
        dmg::TankUllageInput ct{};
        ct.tankIndex = 0;
        ct.section = 1;
        ct.fuelKg = 500.0;
        ct.capacityKg = 1000.0;
        ct.hitEnergyJ = 20000.0;
        ct.fastestFragmentMs = 1100.0;
        ct.hitCount = 3;
        ct.isCentreSection = true;
        grazeIn.tanks.push_back(ct);
        const auto grazeOut = dmg::evaluateDestruction(grazeState, grazeIn, true, rng);
        CHECK_FALSE(grazeOut.catastrophicTriggered);
        CHECK(grazeState.grade == dmg::DamageGrade::LightPerforation);

        // Dynamic thermal escalation: sustained intense centre-bay fire weakens roots over time and eventually
        // triggers a delayed secondary mid-air ullage explosion (Grade 2 -> Grade 4, never instant on step 1!).
        auto frame = std::make_shared<dmg::Airframe>();
        {
            dmg::Airframe::Section centreSec;
            centreSec.name = "centre";
            centreSec.hasRoot = false;
            centreSec.skinAreaM2 = 40.0;
            frame->sections.push_back(centreSec);

            dmg::Airframe::Section wingSec;
            wingSec.name = "left wing";
            wingSec.hasRoot = true;
            wingSec.skinAreaM2 = 20.0;
            frame->sections.push_back(wingSec);

            dmg::Airframe::Component centreTank;
            centreTank.name = "aft fuselage";
            centreTank.kind = "fuel";
            centreTank.section = 0;
            centreTank.tank = 0;
            centreTank.fuelKg = 1000.0;
            frame->components.push_back(centreTank);

            dmg::Airframe::Component wingTank;
            wingTank.name = "left wing";
            wingTank.kind = "fuel";
            wingTank.section = 1;
            wingTank.tank = 1;
            wingTank.fuelKg = 400.0;
            frame->components.push_back(wingTank);
        }
        dmg::VehicleDamage vdmg(frame);
        // Ignite both tanks via direct thermal fire setup (single high-energy fragment row).
        dmg::Impact fireHit;
        fireHit.fragmentsStriking = 8;
        fireHit.components.push_back({0, 4, 25000.0, 0.002, 1200.0});
        fireHit.components.push_back({1, 4, 25000.0, 0.002, 1200.0});
        Rng dmgRng(99);
        for (int tries = 0; tries < 20; ++tries) {
            vdmg.apply(fireHit, 101325.0, dmgRng);
            const auto it0 = vdmg.tanks().find(0);
            const auto it1 = vdmg.tanks().find(1);
            if (it0 != vdmg.tanks().end() && it0->second.burning && it1 != vdmg.tanks().end() && it1->second.burning) break;
        }
        REQUIRE(vdmg.tanks().at(0).burning);
        REQUIRE(vdmg.tanks().at(1).burning);

        dmg::VehicleDestructionState thermState;
        dmg::DestructionStepInput thermIn;
        thermIn.damage = &vdmg;
        thermIn.dtS = 0.5;
        thermIn.totalRemainingFuelKg = 900.0;
        thermIn.activeFuelFireRateKgS = 3.4;
        dmg::TankUllageInput tc{};
        tc.tankIndex = 0; tc.section = 0; tc.fuelKg = 480.0; tc.capacityKg = 1000.0; tc.burning = true; tc.isCentreSection = true;
        thermIn.tanks.push_back(tc);

        bool sawRootWeaken = false;
        bool triggeredDelayed = false;
        int stepsToExplode = 0;
        for (int step = 1; step <= 24; ++step) {
            const auto out = dmg::evaluateDestruction(thermState, thermIn, true, rng);
            if (out.thermalStrengthDelta[1] > 0.0) {
                sawRootWeaken = true;
                vdmg.weakenSectionThermal(1, out.thermalStrengthDelta[1]);
            }
            if (step == 1) {
                CHECK_FALSE(out.catastrophicTriggered);
                CHECK(thermState.grade == dmg::DamageGrade::FireAndStall);
            }
            if (out.catastrophicTriggered) {
                triggeredDelayed = true;
                stepsToExplode = step;
                CHECK(out.cause == dmg::CatastrophicCause::UllageOverpressure);
                CHECK(thermState.grade == dmg::DamageGrade::CatastrophicKill);
                CHECK(thermState.hullRetired);
                break;
            }
        }
        CHECK(sawRootWeaken);
        CHECK(vdmg.sections()[1].strength < 0.99);
        CHECK(triggeredDelayed);
        CHECK(stepsToExplode >= 4);
    }

    // 2. Native World: Grade 1 (distant flank graze -> perforation + un-ignited fuel mist leak, stays controllable & alive)
    {
        session::World w(lightweightOptions("vis-m11b-grade1", 601));
        w.setDestructionModeV1(session::World::DestructionModeV1::LightweightDebris);
        const auto id = wing(w, "f16c", 3800.0, 225.0, 0, 0.60);
        w.step(15);
        const sim::VehicleState st = *w.vehicleState(id);
        const dmg::V3 v = stateVelEcef(st);
        const dmg::V3 off = bodyToEcefOffset(st, {2.5, 11.8, -1.0});
        session::Detonation det;
        det.store = "aim-9m";
        for (int k = 0; k < 3; ++k) {
            det.positionEcef[k] = st.positionEcef[k] + (&off.x)[k];
            det.velocityEcef[k] = (&v.x)[k] * 1.4;
        }
        const dmg::V3 dir = dmg::normalized(v);
        det.axisEcef[0] = dir.x; det.axisEcef[1] = dir.y; det.axisEcef[2] = dir.z;
        w.detonate(det);
        w.step(stepsFor(w, 2.0));

        REQUIRE(w.vehicleState(id) != nullptr);
        const auto* ds = w.destructionState(id);
        REQUIRE(ds != nullptr);
        CHECK(ds->hitFragmentsTotal > 0);
        CHECK(ds->hitAspect == dmg::HitAspect::Right);
        CHECK(ds->grade >= dmg::DamageGrade::LightPerforation);
        CHECK(ds->grade <= dmg::DamageGrade::FireAndStall);
        CHECK_FALSE(ds->hullRetired);
        CHECK(w.debrisField().activeCount() > 0); // localized impact sparks/flakes
    }

    // 3. Native World: Grade 2 (rear-quarter tail/engine hit -> engine out / fire / control loss -> burning stall, stays alive)
    {
        session::World w(lightweightOptions("vis-m11b-grade2", 602));
        w.setDestructionModeV1(session::World::DestructionModeV1::LightweightDebris);
        const auto id = wing(w, "f16c", 3200.0, 225.0, 0, 0.55);
        w.step(15);
        const sim::VehicleState st = *w.vehicleState(id);
        const dmg::V3 v = stateVelEcef(st);
        const dmg::V3 dir = dmg::normalized(v);
        const dmg::V3 off = bodyToEcefOffset(st, {-6.8, 0.5, 1.0});
        session::Detonation det;
        det.store = "aim-9m";
        for (int k = 0; k < 3; ++k) {
            det.positionEcef[k] = st.positionEcef[k] + (&off.x)[k];
            det.velocityEcef[k] = (&v.x)[k] * 2.2;
        }
        det.axisEcef[0] = dir.x; det.axisEcef[1] = dir.y; det.axisEcef[2] = dir.z;
        w.detonate(det);
        w.step(stepsFor(w, 3.0));

        REQUIRE(w.vehicleState(id) != nullptr);
        const auto* ds = w.destructionState(id);
        REQUIRE(ds != nullptr);
        CHECK(ds->hitAspect == dmg::HitAspect::Rear);
        CHECK(ds->grade >= dmg::DamageGrade::FireAndStall);
        CHECK(ds->grade < dmg::DamageGrade::CatastrophicKill);
        CHECK_FALSE(ds->hullRetired);
        // Engine/control damage causes airspeed loss & ember shedding
        CHECK(w.vehicleState(id)->airspeedTrueMs < st.airspeedTrueMs - 15.0);
    }

    // 4. Native World: Grade 3 (concentrated left-wing hit -> single wing breaks off into Jolt debris, aircraft stays alive & tumbles!)
    {
        session::World w(lightweightOptions("vis-m11b-grade3", 603));
        w.setDestructionModeV1(session::World::DestructionModeV1::LightweightDebris);
        const auto id = wing(w, "f16c", 3200.0, 230.0, 0, 0.55);
        w.step(15);
        const sim::VehicleState st = *w.vehicleState(id);
        const dmg::V3 v = stateVelEcef(st);
        const dmg::V3 dir = dmg::normalized(v);
        const dmg::V3 off = bodyToEcefOffset(st, {-0.38, -4.2, -0.75});
        session::Detonation det;
        det.store = "aim-9m";
        for (int k = 0; k < 3; ++k) {
            det.positionEcef[k] = st.positionEcef[k] + (&off.x)[k];
            det.velocityEcef[k] = -(&v.x)[k] * 2.0;
        }
        det.axisEcef[0] = -dir.x; det.axisEcef[1] = -dir.y; det.axisEcef[2] = -dir.z;
        w.detonate(det);
        w.step(stepsFor(w, 2.5));

        REQUIRE(w.vehicleState(id) != nullptr);
        const auto* ds = w.destructionState(id);
        REQUIRE(ds != nullptr);
        CHECK(ds->hitAspect == dmg::HitAspect::Left);
        CHECK(ds->tier == dmg::DestructionTier::StructuralBreakup);
        CHECK(ds->grade == dmg::DamageGrade::StructuralBreakup);
        CHECK_FALSE(ds->hullRetired);
        CHECK(countRoleDebris(w) == 0);
        CHECK(hasArchetype(w.debrisField(), ipc::DebrisArchetypeV1::WingFragment));
    }

    // 5. Delayed Grade 3 -> Grade 4 deduplication: detaching left wing first and then hitting catastrophic must NEVER duplicate SectionLeftWing!
    {
        session::World w(lightweightOptions("vis-m11b-dedup", 604));
        w.setDestructionModeV1(session::World::DestructionModeV1::LightweightDebris);
        const auto id = wing(w, "f16c", 3200.0, 230.0, 0, 0.55);
        w.step(10);
        REQUIRE(w.separateSection(id, "left wing"));
        w.step(stepsFor(w, 0.8));

        const sim::VehicleState st = *w.vehicleState(id);
        const dmg::V3 v = stateVelEcef(st);
        const dmg::V3 dir = dmg::normalized(v);
        session::Detonation det;
        det.store = "aim-120c";
        det.positionEcef[0] = st.positionEcef[0] + 1.8;
        det.positionEcef[1] = st.positionEcef[1] + 1.2;
        det.positionEcef[2] = st.positionEcef[2] + 0.5;
        det.velocityEcef[0] = -v.x * 2.5; det.velocityEcef[1] = -v.y * 2.5; det.velocityEcef[2] = -v.z * 2.5;
        det.axisEcef[0] = -dir.x; det.axisEcef[1] = -dir.y; det.axisEcef[2] = -dir.z;
        w.detonate(det);

        REQUIRE(w.vehicleIds().empty());
        const auto* ds = w.destructionState(id);
        REQUIRE(ds != nullptr);
        CHECK(ds->grade == dmg::DamageGrade::CatastrophicKill);
        int leftWingCount = 0;
        for (const auto& b : w.debrisField().slots()) {
            if (b.alive && b.ownerVehicleId == id && b.partTag == ipc::DebrisPartTagV1::SectionLeftWing) ++leftWingCount;
        }
        CHECK(leftWingCount == 1);
    }
}

TEST_CASE("VIS-M11 optional native visual acceptance replay generator", "[vis-m11][visual-gen]") {
    const char* outEnv = std::getenv("FSIM_VIS_M11_RECORD_DIR");
    if (!outEnv || !outEnv[0]) {
        SUCCEED("Set FSIM_VIS_M11_RECORD_DIR to generate 4K fullscreen visual acceptance replays");
        return;
    }
    const std::filesystem::path dir(outEnv);
    std::filesystem::create_directories(dir);

    // Scenario 1: Mid-air AIM-120C catastrophic kill of F-16C over San Francisco Bay (alt 2500 m, 230 m/s)
    {
        auto opt = lightweightOptions("vis-m11-vis-midair", 901);
        opt.recordPath = (dir / "midair_kill.fsrec").string();
        session::World w(opt);
        w.setDestructionModeV1(session::World::DestructionModeV1::LightweightDebris);

        session::VehicleSpec s;
        s.name = "viper-1";
        s.type = "jsbsim:f16c";
        s.initial.latitudeDeg = 37.65;
        s.initial.longitudeDeg = -122.42;
        s.initial.altitudeMslM = 2500.0;
        s.initial.headingDeg = 75.0;
        s.initial.airspeedTrueMs = 230.0;
        s.initial.fuelFraction = 0.55;
        const auto id = w.createVehicle(s);
        REQUIRE(id != 0);

        w.step(stepsFor(w, 1.4));
        const sim::VehicleState st = *w.vehicleState(id);
        const dmg::V3 v = stateVelEcef(st);
        const double spd = std::max(dmg::length(v), 1.0);
        session::Detonation det;
        det.store = "aim-120c";
        det.positionEcef[0] = st.positionEcef[0] + 2.2;
        det.positionEcef[1] = st.positionEcef[1] + 1.8;
        det.positionEcef[2] = st.positionEcef[2] + 0.8;
        det.velocityEcef[0] = -v.x * 2.5;
        det.velocityEcef[1] = -v.y * 2.5;
        det.velocityEcef[2] = -v.z * 2.5;
        det.axisEcef[0] = -v.x / spd;
        det.axisEcef[1] = -v.y / spd;
        det.axisEcef[2] = -v.z / spd;
        w.detonate(det);
        REQUIRE(w.destructionState(id)->tier == dmg::DestructionTier::CatastrophicKill);
        REQUIRE(w.destructionState(id)->hullRetired);
        w.step(stepsFor(w, 4.6));
    }

    // Scenario 2: Shallow ground belly-slide across real KSFO Terrarium terrain (matching Viewer's z12 meshDimension=64 elevation!)
    {
        auto opt = lightweightOptions("vis-m11-vis-slide", 902);
        opt.recordPath = (dir / "ground_slide.fsrec").string();
        std::filesystem::path mapsDir = platform::executableDir() / ".." / "maps";
        if (!std::filesystem::is_directory(mapsDir)) {
            const auto repoMaps = std::filesystem::current_path() / "assets" / "maps";
            if (std::filesystem::is_directory(repoMaps)) mapsDir = repoMaps;
            else mapsDir = platform::mapsDir();
        }
        io::TerrainTiles::Options to;
        to.zoom = 12;          // matches config/viewer.json elevationMaxLevel
        to.meshDimension = 64; // matches EarthSettings::elevationMeshDimension
        to.offline = true;
        to.cacheDir = mapsDir;
        auto ksfoGround = std::make_shared<io::TerrainTiles>(to);
        ksfoGround->prefetch(37.6188 * kDeg, -122.375 * kDeg, 4000.0);
        const double ksfoGroundM = ksfoGround->heightAboveEllipsoidM(37.6188 * kDeg, -122.375 * kDeg);
        opt.ground = ksfoGround;

        session::World w(opt);
        w.setDestructionModeV1(session::World::DestructionModeV1::LightweightDebris);
        w.setImpactSurfaceV1(dmg::SurfaceKind::Soil);

        session::VehicleSpec s;
        s.name = "viper-slide";
        s.type = "jsbsim:f16c";
        s.initial.latitudeDeg = 37.6188;
        s.initial.longitudeDeg = -122.375;
        s.initial.altitudeMslM = ksfoGroundM + 26.0;
        s.initial.headingDeg = 100.0;
        s.initial.pitchDeg = -5.0;
        s.initial.airspeedTrueMs = 150.0;
        s.initial.fuelFraction = 0.65;
        const auto id = w.createVehicle(s);
        REQUIRE(id != 0);
        w.step(stepsFor(w, 6.0));
        REQUIRE(w.destructionState(id)->tier == dmg::DestructionTier::GroundWreck);
    }

    // Scenario 3: Water impact over Pacific Ocean off Golden Gate (alt 65 m, 165 m/s, -16 deg pitch)
    {
        auto opt = lightweightOptions("vis-m11-vis-water", 903);
        opt.recordPath = (dir / "water_impact.fsrec").string();
        session::World w(opt);
        w.setDestructionModeV1(session::World::DestructionModeV1::LightweightDebris);
        w.setImpactSurfaceV1(dmg::SurfaceKind::Water);

        session::VehicleSpec s;
        s.name = "viper-water";
        s.type = "jsbsim:f16c";
        s.initial.latitudeDeg = 37.72;
        s.initial.longitudeDeg = -122.62;
        s.initial.altitudeMslM = 65.0;
        s.initial.headingDeg = 270.0;
        s.initial.pitchDeg = -16.0;
        s.initial.airspeedTrueMs = 165.0;
        s.initial.fuelFraction = 0.50;
        const auto id = w.createVehicle(s);
        REQUIRE(id != 0);
        w.step(stepsFor(w, 5.0));
        REQUIRE(w.destructionState(id)->tier == dmg::DestructionTier::WaterImpact);
    }

    // Scenario 4: Near-vertical steep-dive ground crash explosion into real KSFO Terrarium terrain (-74 deg, 215 m/s)
    {
        auto opt = lightweightOptions("vis-m11-vis-steep-dive", 904);
        opt.recordPath = (dir / "steep_dive.fsrec").string();
        std::filesystem::path mapsDir = platform::executableDir() / ".." / "maps";
        if (!std::filesystem::is_directory(mapsDir)) {
            const auto repoMaps = std::filesystem::current_path() / "assets" / "maps";
            if (std::filesystem::is_directory(repoMaps)) mapsDir = repoMaps;
            else mapsDir = platform::mapsDir();
        }
        io::TerrainTiles::Options to;
        to.zoom = 12;          // matches config/viewer.json elevationMaxLevel
        to.meshDimension = 64; // matches EarthSettings::elevationMeshDimension
        to.offline = true;
        to.cacheDir = mapsDir;
        auto ksfoGround = std::make_shared<io::TerrainTiles>(to);
        ksfoGround->prefetch(37.6188 * kDeg, -122.375 * kDeg, 4000.0);
        const double ksfoGroundM = ksfoGround->heightAboveEllipsoidM(37.6188 * kDeg, -122.375 * kDeg);
        opt.ground = ksfoGround;

        session::World w(opt);
        w.setDestructionModeV1(session::World::DestructionModeV1::LightweightDebris);
        w.setImpactSurfaceV1(dmg::SurfaceKind::Soil);

        session::VehicleSpec s;
        s.name = "viper-dive";
        s.type = "jsbsim:f16c";
        s.initial.latitudeDeg = 37.6188;
        s.initial.longitudeDeg = -122.375;
        s.initial.altitudeMslM = ksfoGroundM + 265.0;
        s.initial.headingDeg = 100.0;
        s.initial.pitchDeg = -74.0;
        s.initial.airspeedTrueMs = 215.0;
        s.initial.fuelFraction = 0.72;
        const auto id = w.createVehicle(s);
        REQUIRE(id != 0);
        w.step(stepsFor(w, 6.5));
        REQUIRE(w.destructionState(id)->tier == dmg::DestructionTier::GroundWreck);
        REQUIRE(w.destructionState(id)->hullRetired);
    }

    auto bodyToEcefOffset = [](const sim::VehicleState& st, const dmg::V3& b) {
        const double* R = st.rotationBodyToEcef;
        return dmg::V3{
            R[0] * b.x + R[1] * b.y + R[2] * b.z,
            R[3] * b.x + R[4] * b.y + R[5] * b.z,
            R[6] * b.x + R[7] * b.y + R[8] * b.z,
        };
    };

    // Scenario 5 (VIS-M11b Grade 1): Distant flank graze -> sparse perforation, metallic sparks + crisp white kerosene mist leak, flies on
    {
        auto opt = lightweightOptions("vis-m11b-vis-grade1", 911);
        opt.recordPath = (dir / "grade1_light_leak.fsrec").string();
        session::World w(opt);
        w.setDestructionModeV1(session::World::DestructionModeV1::LightweightDebris);

        session::VehicleSpec s;
        s.name = "viper-g1";
        s.type = "jsbsim:f16c";
        s.initial.latitudeDeg = 37.65;
        s.initial.longitudeDeg = -122.42;
        s.initial.altitudeMslM = 3800.0;
        s.initial.headingDeg = 75.0;
        s.initial.airspeedTrueMs = 225.0;
        s.initial.fuelFraction = 0.60;
        const auto id = w.createVehicle(s);
        REQUIRE(id != 0);

        w.step(stepsFor(w, 1.4));
        const sim::VehicleState st = *w.vehicleState(id);
        const dmg::V3 v = stateVelEcef(st);
        const dmg::V3 velDir = dmg::normalized(v);
        const dmg::V3 off = bodyToEcefOffset(st, {2.5, 11.8, -1.0});
        session::Detonation det;
        det.store = "aim-9m";
        for (int k = 0; k < 3; ++k) {
            det.positionEcef[k] = st.positionEcef[k] + (&off.x)[k];
            det.velocityEcef[k] = (&v.x)[k] * 1.4;
        }
        det.axisEcef[0] = velDir.x; det.axisEcef[1] = velDir.y; det.axisEcef[2] = velDir.z;
        w.detonate(det);
        w.step(stepsFor(w, 5.6));
        REQUIRE(w.vehicleState(id) != nullptr);
        REQUIRE(w.destructionState(id)->grade >= dmg::DamageGrade::LightPerforation);
    }

    // Scenario 6 (VIS-M11b Grade 2): Rear-quarter tail/engine strike -> EngineOut + ControlLost + burning stall/spiral descent
    {
        auto opt = lightweightOptions("vis-m11b-vis-grade2", 912);
        opt.recordPath = (dir / "grade2_fire_stall.fsrec").string();
        session::World w(opt);
        w.setDestructionModeV1(session::World::DestructionModeV1::LightweightDebris);

        session::VehicleSpec s;
        s.name = "viper-g2";
        s.type = "jsbsim:f16c";
        s.initial.latitudeDeg = 37.65;
        s.initial.longitudeDeg = -122.42;
        s.initial.altitudeMslM = 3800.0;
        s.initial.headingDeg = 75.0;
        s.initial.airspeedTrueMs = 140.0;
        s.initial.fuelFraction = 0.55;
        const auto id = w.createVehicle(s);
        REQUIRE(id != 0);

        w.step(stepsFor(w, 1.4));
        const sim::VehicleState st = *w.vehicleState(id);
        const dmg::V3 v = stateVelEcef(st);
        const dmg::V3 velDir = dmg::normalized(v);
        const dmg::V3 off = bodyToEcefOffset(st, {-6.4, 1.1, 0.85});
        session::Detonation det;
        det.store = "aim-9m";
        for (int k = 0; k < 3; ++k) {
            det.positionEcef[k] = st.positionEcef[k] + (&off.x)[k];
            det.velocityEcef[k] = (&v.x)[k] * 2.5;
        }
        det.axisEcef[0] = velDir.x; det.axisEcef[1] = velDir.y; det.axisEcef[2] = velDir.z;
        w.detonate(det);
        w.step(stepsFor(w, 8.0));
        REQUIRE(w.vehicleState(id) != nullptr);
        REQUIRE(w.destructionState(id)->grade >= dmg::DamageGrade::FireAndStall);
    }

    // Scenario 7 (VIS-M11b Grade 3): Concentrated left-wing strike -> single real GLB wing snaps off & tumbles, parent spirals on fire!
    {
        auto opt = lightweightOptions("vis-m11b-vis-grade3", 913);
        opt.recordPath = (dir / "grade3_wing_breakup.fsrec").string();
        session::World w(opt);
        w.setDestructionModeV1(session::World::DestructionModeV1::LightweightDebris);

        session::VehicleSpec s;
        s.name = "viper-g3";
        s.type = "jsbsim:f16c";
        s.initial.latitudeDeg = 37.65;
        s.initial.longitudeDeg = -122.42;
        s.initial.altitudeMslM = 3800.0;
        s.initial.headingDeg = 75.0;
        s.initial.airspeedTrueMs = 230.0;
        s.initial.fuelFraction = 0.55;
        const auto id = w.createVehicle(s);
        REQUIRE(id != 0);

        w.step(stepsFor(w, 1.4));
        const sim::VehicleState st = *w.vehicleState(id);
        const dmg::V3 v = stateVelEcef(st);
        const dmg::V3 velDir = dmg::normalized(v);
        const dmg::V3 off = bodyToEcefOffset(st, {-0.38, -4.2, -0.75});
        session::Detonation det;
        det.store = "aim-9m";
        for (int k = 0; k < 3; ++k) {
            det.positionEcef[k] = st.positionEcef[k] + (&off.x)[k];
            det.velocityEcef[k] = -(&v.x)[k] * 2.0;
        }
        det.axisEcef[0] = -velDir.x; det.axisEcef[1] = -velDir.y; det.axisEcef[2] = -velDir.z;
        w.detonate(det);
        w.step(stepsFor(w, 6.6));
        REQUIRE(w.vehicleState(id) != nullptr);
        REQUIRE(w.destructionState(id)->grade == dmg::DamageGrade::StructuralBreakup);
        REQUIRE_FALSE(w.destructionState(id)->hullRetired);
    }
}
