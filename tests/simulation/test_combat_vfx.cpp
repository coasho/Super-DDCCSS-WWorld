// VIS-M13: Unified Compute GPU Particle Engine & Combat VFX transport/authority tests.
// Validates CombatVfxLayoutV1 wire contracts, CombatVfxSegmentV1 live seqlock IPC,
// CombatVfxRecorderV1 / CombatVfxReplayV1 interpolation & event deduplication, native
// World M61A1 gun tracer/muzzle/impact publishing, flare/chaff decoy streaming, and
// generates native 4K visual showcase replays when FSIM_VIS_M13_RECORD_DIR is set.

#include "contracts/world_stream/CombatVfxLayoutV1.h"
#include "foundation/transport/CombatVfxSegmentV1.h"
#include "mode_flights.h"
#include "simulation/runtime/world/World.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <limits>
#include <string>
#include <unordered_set>
#include <vector>

namespace {

using namespace fsim;
using namespace fsim::modes;

session::WorldOptions vfxWorldOptions(const char* name, std::uint64_t seed) {
    auto o = options(name);
    o.dt = 1.0 / 100.0;
    o.frameSkip = 2; // 50 Hz world step
    o.workers = 1;
    o.pinWorkers = false;
    o.publish = false;
    o.terrain = false;
    o.seed = seed;
    return o;
}

std::uint16_t findPayloadByFamily(const session::World& world, std::uint32_t id, ms::Family fam) {
    const auto* host = world.missionSystems(id);
    REQUIRE(host != nullptr);
    for (const auto& p : host->payloads()) {
        if (p.family == fam) return p.index;
    }
    FAIL("Payload family not found");
    return 0;
}

void powerPayloadToOperate(session::World& world, std::uint32_t id, std::uint16_t payload) {
    ms::StateCommand cmd;
    cmd.payload = payload;
    cmd.state = ms::PayloadState::Operate;
    const auto res = world.submit(id, cmd);
    REQUIRE(res.accepted());
    for (int i = 0; i < 400; ++i) {
        const auto* host = world.missionSystems(id);
        REQUIRE(host != nullptr);
        if (host->payloads()[payload].state == ms::PayloadState::Operate) return;
        world.step(1);
    }
    FAIL("Payload did not reach Operate state");
}

void stateVelEcef(const sim::VehicleState& st, double out[3]) noexcept {
    const double sl = std::sin(st.latitudeRad), cl = std::cos(st.latitudeRad);
    const double so = std::sin(st.longitudeRad), co = std::cos(st.longitudeRad);
    const double vn = st.velocityNedMs[0], ve = st.velocityNedMs[1], vd = st.velocityNedMs[2];
    out[0] = -sl * co * vn - so * ve - cl * co * vd;
    out[1] = -sl * so * vn + co * ve - cl * so * vd;
    out[2] = cl * vn - sl * vd;
}

} // namespace

TEST_CASE("VIS-M13 CombatVfxLayoutV1 wire contracts and validators", "[combat_vfx][vis-m13]") {
    using namespace fsim::ipc;

    static_assert(sizeof(ProjectileWireV1) == 80 && alignof(ProjectileWireV1) == 8);
    static_assert(sizeof(DecoyWireV1) == 80 && alignof(DecoyWireV1) == 8);
    static_assert(sizeof(CombatVfxEventWireV1) == 80 && alignof(CombatVfxEventWireV1) == 8);

    ProjectileWireV1 p{};
    CHECK_FALSE(validProjectileWireV1(p)); // id == 0 rejected
    p.id = 7;
    p.shooterVehicleId = 1;
    p.positionEcef[0] = 6378137.0;
    p.velocityEcef[0] = 1000.0;
    p.calibreM = 0.02f;
    p.maxAgeS = 8.0f;
    p.flags = kProjectileFlagTracer | kProjectileFlagHe;
    CHECK(validProjectileWireV1(p));
    p.reserved[0] = 1;
    CHECK_FALSE(validProjectileWireV1(p));
    p.reserved[0] = 0;
    p.positionEcef[1] = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(validProjectileWireV1(p));

    DecoyWireV1 d{};
    CHECK_FALSE(validDecoyWireV1(d));
    d.id = 11;
    d.ownerVehicleId = 2;
    d.kind = static_cast<std::uint8_t>(DecoyKindV1::Flare);
    d.intensityOrSigma = 6000.0f;
    d.bloomRadiusM = 0.55f;
    d.lifeS = 4.0f;
    CHECK(validDecoyWireV1(d));
    d.kind = 9;
    CHECK_FALSE(validDecoyWireV1(d));

    CombatVfxEventWireV1 e{};
    CHECK_FALSE(validCombatVfxEventV1(e));
    e.serial = 3;
    e.kind = static_cast<std::uint8_t>(CombatVfxEventKindV1::MuzzleFlash);
    e.dirEcef[0] = 1.0f;
    e.magnitude = 0.02f;
    CHECK(validCombatVfxEventV1(e));
    e.magnitude = -1.0f;
    CHECK_FALSE(validCombatVfxEventV1(e));
}

TEST_CASE("VIS-M13 CombatVfxSegmentV1 live IPC and CombatVfxReplayV1 interpolation", "[combat_vfx][vis-m13]") {
    using namespace fsim::ipc;

    CombatVfxFrameV1 f0;
    f0.simTime = 1.0;
    ProjectileWireV1 p0{};
    p0.id = 42;
    p0.shooterVehicleId = 1;
    p0.positionEcef[0] = 1000.0;
    p0.velocityEcef[0] = 500.0;
    p0.calibreM = 0.02f;
    p0.ageS = 0.1f;
    p0.maxAgeS = 8.0f;
    p0.flags = kProjectileFlagTracer;
    f0.projectiles.push_back(p0);

    DecoyWireV1 d0{};
    d0.id = 99;
    d0.ownerVehicleId = 2;
    d0.kind = static_cast<std::uint8_t>(DecoyKindV1::Flare);
    d0.positionEcef[2] = 2000.0;
    d0.velocityEcef[2] = -20.0;
    d0.ageS = 0.2f;
    d0.lifeS = 4.0f;
    d0.intensityOrSigma = 5400.0f;
    d0.bloomRadiusM = 0.55f;
    f0.decoys.push_back(d0);

    CombatVfxEventWireV1 ev0{};
    ev0.serial = 10;
    ev0.kind = static_cast<std::uint8_t>(CombatVfxEventKindV1::MuzzleFlash);
    ev0.sourceVehicleId = 1;
    ev0.dirEcef[0] = 1.0f;
    ev0.magnitude = 0.02f;
    f0.events.push_back(ev0);

    CombatVfxFrameV1 f1 = f0;
    f1.simTime = 1.1;
    f1.projectiles[0].positionEcef[0] = 1050.0;
    f1.projectiles[0].ageS = 0.2f;
    f1.decoys[0].positionEcef[2] = 1998.0;
    f1.decoys[0].ageS = 0.3f;
    f1.decoys[0].intensityOrSigma = 5000.0f;
    f1.events.clear();

    // 1. Live wait-free triple-buffered shared-memory roundtrip.
    const std::string segWorld = "vis_m13_vfx_unit_test";
    CombatVfxSegmentV1 writer, reader;
    REQUIRE(writer.create(segWorld, kMaxProjectilesV1, kMaxDecoysV1, kMaxCombatVfxEventsV1, 7777u, 1u));
    REQUIRE(reader.open(segWorld, 7777u, 1u));
    REQUIRE(writer.publish(5u, f0));
    CombatVfxFrameV1 got;
    REQUIRE(reader.read(5u, got));
    REQUIRE(got.projectiles.size() == 1);
    CHECK(got.projectiles[0].id == 42u);
    REQUIRE(got.decoys.size() == 1);
    CHECK(got.decoys[0].intensityOrSigma == Catch::Approx(5400.0f));
    REQUIRE(got.events.size() == 1);
    CHECK(got.events[0].serial == 10u);
    reader.close();
    writer.close();

    // 2. Companion .vfx recorder & midpoint interpolation + event lookback window.
    const auto tmpPath = std::filesystem::temp_directory_path() / "fsim_vis_m13_test.vfx";
    {
        CombatVfxRecorderV1 rec;
        REQUIRE(rec.open(tmpPath));
        REQUIRE(rec.write(f0));
        REQUIRE(rec.write(f1));
        rec.close();
    }
    CombatVfxReplayV1 rep;
    std::string err;
    REQUIRE(rep.load(tmpPath, &err));
    CHECK(rep.frameCount() == 2);

    CombatVfxFrameV1 mid;
    REQUIRE(rep.atTime(1.05, mid, true));
    REQUIRE(mid.projectiles.size() == 1);
    CHECK(mid.projectiles[0].positionEcef[0] == Catch::Approx(1025.0).margin(1e-6));
    CHECK(mid.projectiles[0].ageS == Catch::Approx(0.15f).margin(1e-5f));
    REQUIRE(mid.decoys.size() == 1);
    CHECK(mid.decoys[0].positionEcef[2] == Catch::Approx(1999.0).margin(1e-6));
    CHECK(mid.decoys[0].intensityOrSigma == Catch::Approx(5200.0f).margin(1e-3f));
    REQUIRE(mid.events.size() == 1); // event at t=1.0 still visible in lookback window at t=1.05
    CHECK(mid.events[0].serial == 10u);

    std::error_code ec;
    std::filesystem::remove(tmpPath, ec);
}

TEST_CASE("VIS-M13 native World gun burst and flare/chaff decoys publish live combat VFX frames", "[combat_vfx][vis-m13]") {
    auto runOnce = [&](std::size_t& maxProjectiles, std::size_t& tracerCount, std::size_t& muzzleEvents,
                       std::size_t& hitEvents, std::size_t& flareCount, std::size_t& chaffCount,
                       std::size_t& cmEjectEvents, double& firstTracerX) {
        session::World world(vfxWorldOptions("vis-m13-gun-cm", 20261126u));
        world.setDestructionModeV1(session::World::DestructionModeV1::LightweightDebris);

        session::VehicleSpec shooterSpec;
        shooterSpec.name = "shooter";
        shooterSpec.type = "jsbsim:f16c";
        shooterSpec.initial.latitudeDeg = 37.65;
        shooterSpec.initial.longitudeDeg = -122.42;
        shooterSpec.initial.altitudeMslM = 3800.0;
        shooterSpec.initial.headingDeg = 90.0;
        shooterSpec.initial.airspeedTrueMs = 230.0;

        session::VehicleSpec targetSpec = shooterSpec;
        targetSpec.name = "target";
        // Place target ~120 m dead ahead along East
        targetSpec.initial.longitudeDeg += 120.0 / (111320.0 * std::cos(37.65 * 3.141592653589793 / 180.0));

        const std::uint32_t shooter = world.createVehicle(shooterSpec);
        const std::uint32_t target = world.createVehicle(targetSpec);
        REQUIRE(shooter != 0u);
        REQUIRE(target != 0u);

        const std::uint16_t gunIdx = findPayloadByFamily(world, shooter, ms::Family::Gun);
        const std::uint16_t cmIdx = findPayloadByFamily(world, target, ms::Family::Countermeasures);
        powerPayloadToOperate(world, shooter, gunIdx);
        powerPayloadToOperate(world, target, cmIdx);

        // Fire a 60-round 20 mm M61A1 burst + dispense flare & chaff on the target.
        ms::PayloadTask fireTask;
        fireTask.payload = gunIdx;
        fireTask.kind = ms::TaskKind::Fire;
        fireTask.rounds = 60;
        const auto gunRes = world.submit(shooter, fireTask);
        REQUIRE(gunRes.accepted());
        REQUIRE(world.dispense(target, true) == ms::Reason::None);

        std::unordered_set<std::uint32_t> seenSerials;
        std::unordered_set<std::uint32_t> seenTracerIds;
        firstTracerX = 0.0;

        for (int step = 0; step < 60; ++step) {
            if (step == 22) REQUIRE(world.dispense(target, false) == ms::Reason::None);
            world.step(1);
            const auto frame = world.combatVfxFrame();
            maxProjectiles = std::max(maxProjectiles, frame.projectiles.size());
            for (const auto& p : frame.projectiles) {
                if (p.flags & ipc::kProjectileFlagTracer) {
                    if (seenTracerIds.insert(p.id).second && firstTracerX == 0.0) firstTracerX = p.positionEcef[0];
                }
            }
            for (const auto& d : frame.decoys) {
                if (d.kind == static_cast<std::uint8_t>(ipc::DecoyKindV1::Flare)) {
                    ++flareCount;
                    CHECK(d.intensityOrSigma > 0.0f);
                } else {
                    ++chaffCount;
                    CHECK(d.bloomRadiusM >= 0.6f);
                }
            }
            for (const auto& ev : frame.events) {
                if (!seenSerials.insert(ev.serial).second) continue;
                const auto k = static_cast<ipc::CombatVfxEventKindV1>(ev.kind);
                if (k == ipc::CombatVfxEventKindV1::MuzzleFlash) ++muzzleEvents;
                else if (k == ipc::CombatVfxEventKindV1::BulletAirframeHit) ++hitEvents;
                else if (k == ipc::CombatVfxEventKindV1::CountermeasureEject) ++cmEjectEvents;
            }
        }
        tracerCount = seenTracerIds.size();
    };

    std::size_t maxP1 = 0, tr1 = 0, mf1 = 0, hit1 = 0, fl1 = 0, ch1 = 0, cm1 = 0;
    double tx1 = 0.0;
    runOnce(maxP1, tr1, mf1, hit1, fl1, ch1, cm1, tx1);

    CHECK(maxP1 >= 4u);
    CHECK(tr1 >= 4u);
    CHECK(mf1 >= 1u);
    CHECK(hit1 >= 1u);
    CHECK(fl1 >= 10u);
    CHECK(ch1 >= 10u);
    CHECK(cm1 == 2u);

    // Deterministic seeded repeatability check.
    std::size_t maxP2 = 0, tr2 = 0, mf2 = 0, hit2 = 0, fl2 = 0, ch2 = 0, cm2 = 0;
    double tx2 = 0.0;
    runOnce(maxP2, tr2, mf2, hit2, fl2, ch2, cm2, tx2);
    CHECK(maxP1 == maxP2);
    CHECK(tr1 == tr2);
    CHECK(mf1 == mf2);
    CHECK(hit1 == hit2);
    CHECK(tx1 == tx2);
}

TEST_CASE("VIS-M13 visual showcase replay generator", "[combat_vfx][vis-m13][visual-gen]") {
    const char* outEnv = std::getenv("FSIM_VIS_M13_RECORD_DIR");
    if (!outEnv || !outEnv[0]) return;

    const std::filesystem::path outDir(outEnv);
    std::filesystem::create_directories(outDir);

    // --- Showcase 1: M61A1 20mm Tail-Chase Gun Burst -> Muzzle Flash, Tracers, Ricochet Sparks & Flares ---
    {
        auto opts = vfxWorldOptions("vis-m13-gun-chase", 20261301u);
        opts.recordPath = outDir / "gun_tail_chase.fsrec";
        opts.recordIntervalSeconds = 0.02;
        session::World world(opts);
        world.setDestructionModeV1(session::World::DestructionModeV1::LightweightDebris);

        session::VehicleSpec shooterSpec;
        shooterSpec.name = "viper-1";
        shooterSpec.type = "jsbsim:f16c";
        shooterSpec.initial.latitudeDeg = 37.65;
        shooterSpec.initial.longitudeDeg = -122.42;
        shooterSpec.initial.altitudeMslM = 3800.0;
        shooterSpec.initial.headingDeg = 75.0;
        shooterSpec.initial.airspeedTrueMs = 230.0;
        shooterSpec.initial.fuelFraction = 0.55;

        const double hdgRad = 75.0 * 3.141592653589793 / 180.0;
        const double distM = 95.0;
        session::VehicleSpec targetSpec = shooterSpec;
        targetSpec.name = "bandit-1";
        targetSpec.initial.latitudeDeg += (distM * std::cos(hdgRad)) / 111132.0;
        targetSpec.initial.longitudeDeg += (distM * std::sin(hdgRad)) / (111320.0 * std::cos(37.65 * 3.141592653589793 / 180.0));
        // Place target slightly above shooter line so natural gravity drop / nose elevation lines right through fuselage!
        targetSpec.initial.altitudeMslM = 3800.8;

        const std::uint32_t shooter = world.createVehicle(shooterSpec);
        const std::uint32_t target = world.createVehicle(targetSpec);
        REQUIRE(shooter != 0u);
        REQUIRE(target != 0u);

        const std::uint16_t gunIdx = findPayloadByFamily(world, shooter, ms::Family::Gun);
        const std::uint16_t cmIdx = findPayloadByFamily(world, target, ms::Family::Countermeasures);
        powerPayloadToOperate(world, shooter, gunIdx);
        powerPayloadToOperate(world, target, cmIdx);

        ms::PayloadTask fireTask;
        fireTask.payload = gunIdx;
        fireTask.kind = ms::TaskKind::Fire;
        fireTask.rounds = 120;

        for (int step = 0; step < 260; ++step) {
            if (step == 35) world.submit(shooter, fireTask);  // sustained 1.2 s 20mm burst!
            if (step == 115) world.submit(shooter, fireTask); // follow-up burst!
            if (step == 50 || step == 80 || step == 110) world.dispense(target, true); // defensive IR flares
            world.step(1);
        }
        std::cout << "[VIS-M13 visual-gen] wrote " << opts.recordPath.string() << " (+ .vfx + .debris)\n";
    }

    // --- Showcase 2: IR Flare Curtains + Chaff Dipole Clouds + AIM-120C Shockwave & Catastrophic Kill ---
    {
        auto opts = vfxWorldOptions("vis-m13-cm-kill", 20261302u);
        opts.recordPath = outDir / "cm_missile_kill.fsrec";
        opts.recordIntervalSeconds = 0.02;
        session::World world(opts);
        world.setDestructionModeV1(session::World::DestructionModeV1::LightweightDebris);

        session::VehicleSpec targetSpec;
        targetSpec.name = "viper-cm";
        targetSpec.type = "jsbsim:f16c";
        targetSpec.initial.latitudeDeg = 37.65;
        targetSpec.initial.longitudeDeg = -122.42;
        targetSpec.initial.altitudeMslM = 3800.0;
        targetSpec.initial.headingDeg = 75.0;
        targetSpec.initial.airspeedTrueMs = 230.0;
        targetSpec.initial.fuelFraction = 0.55;

        const std::uint32_t target = world.createVehicle(targetSpec);
        REQUIRE(target != 0u);
        const std::uint16_t cmIdx = findPayloadByFamily(world, target, ms::Family::Countermeasures);
        powerPayloadToOperate(world, target, cmIdx);

        for (int step = 0; step < 340; ++step) {
            // Dispenser cooldown is 0.35 s (~18 steps at 50 Hz): alternate flares and chaff!
            if (step == 20 || step == 58 || step == 96 || step == 134) world.dispense(target, true);  // IR Flares
            if (step == 39 || step == 77 || step == 115) world.dispense(target, false);              // Chaff clouds
            if (step == 155) {
                if (const auto* st = world.vehicleState(target)) {
                    double velEcef[3];
                    stateVelEcef(*st, velEcef);
                    const double spd = std::max(std::sqrt(velEcef[0] * velEcef[0] + velEcef[1] * velEcef[1] + velEcef[2] * velEcef[2]), 1.0);
                    const double* R = st->rotationBodyToEcef;
                    const double offBody[3] = {0.5, -2.5, -0.4};
                    session::Detonation det;
                    det.store = "aim-120c";
                    for (int k = 0; k < 3; ++k) {
                        det.positionEcef[k] = st->positionEcef[k] + R[k] * offBody[0] + R[3 + k] * offBody[1] + R[6 + k] * offBody[2];
                        det.velocityEcef[k] = -velEcef[k] * 2.2;
                        det.axisEcef[k] = -velEcef[k] / spd;
                    }
                    world.detonate(det);
                }
            }
            world.step(1);
        }
        std::cout << "[VIS-M13 visual-gen] wrote " << opts.recordPath.string() << " (+ .vfx + .debris)\n";
    }
}
