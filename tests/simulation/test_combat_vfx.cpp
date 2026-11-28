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
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
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
        double maxGunFireballKg = 0.0;

        for (int step = 0; step < 60; ++step) {
            if (step == 22) REQUIRE(world.dispense(target, false) == ms::Reason::None);
            world.step(1);
            for (const auto& eng : world.takeEngagementEvents()) {
                if (eng.kind == static_cast<std::uint8_t>(ipc::EngagementKind::Fire)) {
                    maxGunFireballKg = std::max(maxGunFireballKg, eng.amountKg);
                }
            }
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
        // Physical damage-grade mapping: 20 mm cannon hits ignite streaming fuel/engine fires, never fake 50 kg missile fireballs,
        // and never flood the air with dozens of half-meter 3-D metal chunks.
        CHECK(maxGunFireballKg == 0.0);
        std::size_t gunShards = 0;
        for (const auto& b : world.debrisField().slots()) {
            if (b.alive && (b.archetype == ipc::DebrisArchetypeV1::PanelShard || b.archetype == ipc::DebrisArchetypeV1::BurningChunk))
                ++gunShards;
        }
        CHECK(gunShards <= 4u);
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

    // --- Showcase 3: Full Live 6-DOF AIM-120C Launch -> Rocket Homing -> Flare/Chaff Defense -> Proximity-Fuze Destruction ---
    {
        auto opts = vfxWorldOptions("vis-m13-live-missile", 20261303u);
        opts.recordPath = outDir / "live_missile_intercept.fsrec";
        opts.recordIntervalSeconds = 0.02;
        session::World world(opts);
        world.setDestructionModeV1(session::World::DestructionModeV1::LightweightDebris);

        constexpr double kDeg = 3.14159265358979323846 / 180.0;
        const double lat0 = 37.65;
        const double lon0 = -122.42;
        const double hdgRad = 75.0 * kDeg;
        const double cosLat = std::cos(lat0 * kDeg);

        session::VehicleSpec shooterSpec;
        shooterSpec.name = "viper-blue";
        shooterSpec.type = "jsbsim:f16c";
        shooterSpec.initial.latitudeDeg = lat0;
        shooterSpec.initial.longitudeDeg = lon0;
        shooterSpec.initial.altitudeMslM = 4000.0;
        shooterSpec.initial.headingDeg = 75.0;
        shooterSpec.initial.airspeedTrueMs = 265.0;
        shooterSpec.initial.fuelFraction = 0.60;

        // Place target 10.5 km ahead flying head-on (255 deg, slight lateral offset 180 m) at 235 m/s:
        // High PD Doppler closure cleanly filters stationary chaff clouds while the live 6-DOF AIM-120C
        // homes through bandit-red's defensive flare & chaff volley to a real proximity-fuze hit!
        const double aheadM = 10500.0;
        const double rightM = 180.0;
        const double dNorth = aheadM * std::cos(hdgRad) - rightM * std::sin(hdgRad);
        const double dEast = aheadM * std::sin(hdgRad) + rightM * std::cos(hdgRad);
        session::VehicleSpec targetSpec = shooterSpec;
        targetSpec.name = "bandit-red";
        targetSpec.initial.latitudeDeg = lat0 + dNorth / 111132.0;
        targetSpec.initial.longitudeDeg = lon0 + dEast / (111320.0 * cosLat);
        targetSpec.initial.altitudeMslM = 4030.0;
        targetSpec.initial.headingDeg = 255.0;
        targetSpec.initial.airspeedTrueMs = 235.0;
        targetSpec.initial.fuelFraction = 0.55;

        const std::uint32_t shooter = world.createVehicle(shooterSpec);
        const std::uint32_t target = world.createVehicle(targetSpec);
        REQUIRE(shooter != 0u);
        REQUIRE(target != 0u);

        const std::uint16_t radarIdx = findPayloadByFamily(world, shooter, ms::Family::AirRadar);
        const std::uint16_t stationsIdx = findPayloadByFamily(world, shooter, ms::Family::StoreStation);
        const std::uint16_t cmIdx = findPayloadByFamily(world, target, ms::Family::Countermeasures);
        REQUIRE(world.loadStore(shooter, stationsIdx, 1, "aim-120c") == ms::Reason::None);
        REQUIRE(world.loadStore(shooter, stationsIdx, 9, "aim-120c") == ms::Reason::None);

        // Hold smooth level flight on both jets via native HSA autopilot so radar lock & launch are clean.
        auto setHsa = [&](std::uint32_t id, double hdgDeg, double altM, double spdMs) {
            control::HsaCommand hsa;
            hsa.headingRad = hdgDeg * kDeg;
            hsa.altitudeM = altM;
            hsa.speed = spdMs;
            (void)world.submit(id, hsa);
        };
        setHsa(shooter, 75.0, 4000.0, 265.0);
        setHsa(target, 255.0, 4030.0, 235.0);

        // Power up radar, store stations, and target countermeasures concurrently.
        (void)world.submit(shooter, ms::StateCommand{radarIdx, ms::PayloadState::Operate});
        (void)world.submit(shooter, ms::StateCommand{stationsIdx, ms::PayloadState::Operate});
        (void)world.submit(target, ms::StateCommand{cmIdx, ms::PayloadState::Operate});

        bool searching = false;
        bool launched = false;
        bool detonated = false;
        double launchTimeS = -1.0;
        double detTimeS = -1.0;
        double lastCmTimeS = -10.0;
        int cmShot = 0;
        int postDetSteps = 0;

        for (int step = 0; step < 1200; ++step) {
            world.step(1);
            const double nowS = world.simTime();
            const auto* shHost = world.missionSystems(shooter);
            if (!searching && shHost && shHost->payloads()[radarIdx].state == ms::PayloadState::Operate) {
                ms::AmtiCommand ac;
                ac.payload = radarIdx;
                ac.mode = ms::AmtiMode::TrackWhileScan;
                ac.task = ms::AmtiTask::Volume;
                ac.rangeScaleM = 40000.0;
                ac.azimuthHalfWidthRad = 25.0 * kDeg;
                ac.bars = 2;
                if (world.submit(shooter, ac).accepted()) searching = true;
            }
            if (!launched && searching && shHost && shHost->payloads()[stationsIdx].state == ms::PayloadState::Operate) {
                const auto tracks = world.amtiTracks(shooter, radarIdx);
                if (!tracks.empty()) {
                    const std::uint32_t desig = world.designate(shooter, radarIdx, tracks.front().id);
                    if (desig != 0u) {
                        ms::StrikeCommand sc;
                        sc.payload = stationsIdx;
                        sc.store = "aim-120c";
                        sc.quantity = 1;
                        sc.target = desig;
                        sc.consent = true;
                        if (world.submit(shooter, sc).accepted()) {
                            launched = true;
                            launchTimeS = nowS;
                        }
                    }
                }
            }
            // Watch live missile closing on target; dispense alternating aft-ventral flares & chaff!
            const auto wpns = world.weapons();
            if (!wpns.empty()) {
                const auto& w = wpns.front();
                if (!detonated && w.end == ms::WeaponEnd::Detonated) {
                    detonated = true;
                    detTimeS = nowS;
                }
                if (!detonated && w.end != ms::WeaponEnd::Detonated) {
                    const auto* wSt = world.vehicleState(w.id);
                    const auto* tSt = world.vehicleState(target);
                    if (wSt && tSt) {
                        const double dx = wSt->positionEcef[0] - tSt->positionEcef[0];
                        const double dy = wSt->positionEcef[1] - tSt->positionEcef[1];
                        const double dz = wSt->positionEcef[2] - tSt->positionEcef[2];
                        const double distM = std::sqrt(dx * dx + dy * dy + dz * dz);
                        if (distM < 4800.0 && nowS - lastCmTimeS >= 0.40 && cmShot < 8) {
                            const bool flare = ((cmShot % 2) == 0);
                            if (world.dispense(target, flare) == ms::Reason::None) {
                                lastCmTimeS = nowS;
                                ++cmShot;
                            }
                        }
                    }
                }
            }
            if (detonated) {
                if (++postDetSteps >= 230) break; // record 4.6 s of post-detonation shockwave, fireball & wreckage!
            }
        }
        const auto* ds = world.destructionState(target);
        std::cout << "[VIS-M13 visual-gen] wrote " << opts.recordPath.string()
                  << " | launch=" << launchTimeS << "s det=" << detTimeS << "s cm=" << cmShot
                  << " grade=" << (ds ? static_cast<int>(ds->grade) : -1)
                  << " tier=" << (ds ? static_cast<int>(ds->tier) : -1)
                  << " retired=" << (ds && ds->hullRetired ? 1 : 0)
                  << " debris=" << world.debrisField().activeCount() << "\n";
        CHECK(launched);
        CHECK(detonated);
    }
}

TEST_CASE("VIS-M13 live published 1x real-time radar and AIM-120C intercept showcase", "[combat_vfx][vis-m13][live-radar]") {
    const char* worldName = std::getenv("FSIM_VIS_M13_LIVE_WORLD");
    if (!worldName || !*worldName) return;

    double durationS = 90.0;
    if (const char* durEnv = std::getenv("FSIM_VIS_M13_LIVE_DURATION_S"); durEnv && *durEnv) {
        const double parsed = std::atof(durEnv);
        if (parsed >= 5.0 && parsed <= 600.0) durationS = parsed;
    }

    auto opts = vfxWorldOptions(worldName, 20261309u);
    opts.workers = 2;
    opts.publish = true;
    opts.publishIntervalSeconds = 1.0 / 60.0;
    session::World world(opts);
    world.setDestructionModeV1(session::World::DestructionModeV1::LightweightDebris);

    constexpr double kDeg = 3.14159265358979323846 / 180.0;
    const double lat0 = 37.65;
    const double lon0 = -122.42;
    const double hdgRad = 75.0 * kDeg;

    session::VehicleSpec shooterSpec;
    shooterSpec.name = "viper-blue";
    shooterSpec.type = "jsbsim:f16c";
    shooterSpec.initial.latitudeDeg = lat0;
    shooterSpec.initial.longitudeDeg = lon0;
    shooterSpec.initial.altitudeMslM = 4000.0;
    shooterSpec.initial.headingDeg = 75.0;
    shooterSpec.initial.airspeedTrueMs = 265.0;
    shooterSpec.initial.fuelFraction = 0.65;

    const std::uint32_t shooter = world.createVehicle(shooterSpec);
    REQUIRE(shooter != 0u);

    auto spawnBanditAhead = [&](const char* name, double aheadM, double rightM, double altM) -> std::uint32_t {
        const auto* sSt = world.vehicleState(shooter);
        const double baseLat = sSt ? sSt->latitudeRad / kDeg : lat0;
        const double baseLon = sSt ? sSt->longitudeRad / kDeg : lon0;
        const double baseCos = std::cos(baseLat * kDeg);
        const double dNorth = aheadM * std::cos(hdgRad) - rightM * std::sin(hdgRad);
        const double dEast = aheadM * std::sin(hdgRad) + rightM * std::cos(hdgRad);
        session::VehicleSpec spec = shooterSpec;
        spec.name = name;
        spec.initial.latitudeDeg = baseLat + dNorth / 111132.0;
        spec.initial.longitudeDeg = baseLon + dEast / (111320.0 * baseCos);
        spec.initial.altitudeMslM = altM;
        spec.initial.headingDeg = 255.0;
        spec.initial.airspeedTrueMs = 235.0;
        spec.initial.fuelFraction = 0.55;
        const std::uint32_t id = world.createVehicle(spec);
        if (id != 0u) {
            control::HsaCommand hsa;
            hsa.headingRad = 255.0 * kDeg;
            hsa.altitudeM = altM;
            hsa.speed = 235.0;
            (void)world.submit(id, hsa);
            const std::uint16_t bRadar = findPayloadByFamily(world, id, ms::Family::AirRadar);
            const std::uint16_t bCm = findPayloadByFamily(world, id, ms::Family::Countermeasures);
            (void)world.submit(id, ms::StateCommand{bRadar, ms::PayloadState::Operate});
            (void)world.submit(id, ms::StateCommand{bCm, ms::PayloadState::Operate});
            if (const auto* host = world.missionSystems(id)) {
                for (const auto& p : host->payloads()) {
                    if (p.family == ms::Family::Esm) {
                        (void)world.submit(id, ms::StateCommand{p.index, ms::PayloadState::Operate});
                    }
                }
            }
        }
        return id;
    };

    std::uint32_t target = spawnBanditAhead("bandit-red-1", 16500.0, 220.0, 4040.0);
    REQUIRE(target != 0u);

    const std::uint16_t radarIdx = findPayloadByFamily(world, shooter, ms::Family::AirRadar);
    const std::uint16_t stationsIdx = findPayloadByFamily(world, shooter, ms::Family::StoreStation);
    REQUIRE(world.loadStore(shooter, stationsIdx, 1, "aim-120c") == ms::Reason::None);
    REQUIRE(world.loadStore(shooter, stationsIdx, 9, "aim-120c") == ms::Reason::None);

    {
        control::HsaCommand hsa;
        hsa.headingRad = 75.0 * kDeg;
        hsa.altitudeM = 4000.0;
        hsa.speed = 265.0;
        (void)world.submit(shooter, hsa);
    }
    (void)world.submit(shooter, ms::StateCommand{radarIdx, ms::PayloadState::Operate});
    (void)world.submit(shooter, ms::StateCommand{stationsIdx, ms::PayloadState::Operate});
    if (const auto* shHost = world.missionSystems(shooter)) {
        for (const auto& p : shHost->payloads()) {
            if (p.family == ms::Family::Esm) {
                (void)world.submit(shooter, ms::StateCommand{p.index, ms::PayloadState::Operate});
            }
        }
    }

    auto startSignalTws = [&](std::uint32_t id, std::uint16_t rIdx) -> bool {
        ms::AmtiCommand ac;
        ac.payload = rIdx;
        ac.mode = ms::AmtiMode::TrackWhileScan;
        ac.task = ms::AmtiTask::Volume;
        ac.rangeScaleM = 40000.0;
        ac.azimuthHalfWidthRad = 25.0 * kDeg;
        ac.bars = 2;
        ac.signalLevel = true;
        return world.submit(id, ac).accepted();
    };

    bool shooterSearching = false;
    bool targetSearching = false;
    bool launchedWave1 = false;
    bool launchedWave2 = false;
    bool spawnedWave2 = false;
    double wave1DetTimeS = -1.0;
    double wave2SpawnTimeS = -1.0;
    double lastCmTimeS = -10.0;
    int cmShot = 0;

    std::cout << "[VIS-M13 live-radar] publishing world='" << worldName << "' duration=" << durationS
              << "s (1x real-time, signalLevel TWS B-Scope + Range-Doppler)\n"
              << std::flush;

    const double stepDtS = opts.dt * static_cast<double>(opts.frameSkip);
    const int totalSteps = static_cast<int>(std::ceil(durationS / stepDtS));
    const auto wallStart = std::chrono::steady_clock::now();

    for (int step = 0; step < totalSteps; ++step) {
        world.step(1);
        const double nowS = world.simTime();

        const auto* shHost = world.missionSystems(shooter);
        if (!shooterSearching && shHost && shHost->payloads()[radarIdx].state == ms::PayloadState::Operate) {
            if (startSignalTws(shooter, radarIdx)) {
                shooterSearching = true;
                std::cout << "[VIS-M13 live-radar] t=" << nowS << "s viper-blue AN/APG-68 TWS signalLevel ON\n" << std::flush;
            }
        }
        if (!targetSearching && target != 0u) {
            const auto* tHost = world.missionSystems(target);
            const std::uint16_t bRadar = findPayloadByFamily(world, target, ms::Family::AirRadar);
            if (tHost && tHost->payloads()[bRadar].state == ms::PayloadState::Operate) {
                if (startSignalTws(target, bRadar)) {
                    targetSearching = true;
                    std::cout << "[VIS-M13 live-radar] t=" << nowS << "s bandit AN/APG-68 TWS signalLevel ON\n" << std::flush;
                }
            }
        }

        // Wait until t >= 7.0 s for wave 1 so the user sees sweeping B-Scope, confirmed track & Range-Doppler map first.
        if (!launchedWave1 && nowS >= 7.0 && shooterSearching && shHost &&
            shHost->payloads()[stationsIdx].state == ms::PayloadState::Operate) {
            const auto tracks = world.amtiTracks(shooter, radarIdx);
            if (!tracks.empty()) {
                const std::uint32_t desig = world.designate(shooter, radarIdx, tracks.front().id);
                if (desig != 0u) {
                    ms::StrikeCommand sc;
                    sc.payload = stationsIdx;
                    sc.store = "aim-120c";
                    sc.quantity = 1;
                    sc.target = desig;
                    sc.consent = true;
                    if (world.submit(shooter, sc).accepted()) {
                        launchedWave1 = true;
                        std::cout << "[VIS-M13 live-radar] t=" << nowS << "s AIM-120C #1 launched at track "
                                  << tracks.front().id << "\n" << std::flush;
                    }
                }
            }
        }

        // Watch active missile(s) & trigger defensive flare/chaff volleys on current live bandit.
        const auto wpns = world.weapons();
        for (const auto& w : wpns) {
            if (wave1DetTimeS < 0.0 && w.end == ms::WeaponEnd::Detonated) {
                wave1DetTimeS = nowS;
                const auto* ds = world.destructionState(target);
                std::cout << "[VIS-M13 live-radar] t=" << nowS << "s AIM-120C detonated! grade="
                          << (ds ? static_cast<int>(ds->grade) : -1) << " retired="
                          << (ds && ds->hullRetired ? 1 : 0) << "\n" << std::flush;
            }
            if (w.end != ms::WeaponEnd::Detonated && target != 0u) {
                const auto* wSt = world.vehicleState(w.id);
                const auto* tSt = world.vehicleState(target);
                if (wSt && tSt) {
                    const double dx = wSt->positionEcef[0] - tSt->positionEcef[0];
                    const double dy = wSt->positionEcef[1] - tSt->positionEcef[1];
                    const double dz = wSt->positionEcef[2] - tSt->positionEcef[2];
                    const double distM = std::sqrt(dx * dx + dy * dy + dz * dz);
                    if (distM < 4800.0 && nowS - lastCmTimeS >= 0.40 && cmShot < 8) {
                        const bool flare = ((cmShot % 2) == 0);
                        if (world.dispense(target, flare) == ms::Reason::None) {
                            lastCmTimeS = nowS;
                            ++cmShot;
                        }
                    }
                }
            }
        }

        // 8 s after wave 1 detonation, spawn bandit-red-2 at 17 km ahead for a second live radar/missile engagement.
        if (!spawnedWave2 && wave1DetTimeS > 0.0 && nowS >= wave1DetTimeS + 8.0) {
            target = spawnBanditAhead("bandit-red-2", 17000.0, -260.0, 4120.0);
            targetSearching = false;
            cmShot = 0;
            lastCmTimeS = -10.0;
            spawnedWave2 = (target != 0u);
            wave2SpawnTimeS = nowS;
            std::cout << "[VIS-M13 live-radar] t=" << nowS << "s spawned bandit-red-2 at 17 km\n" << std::flush;
        }
        if (spawnedWave2 && !launchedWave2 && nowS >= wave2SpawnTimeS + 6.0 && shHost &&
            shHost->payloads()[stationsIdx].state == ms::PayloadState::Operate) {
            const auto tracks = world.amtiTracks(shooter, radarIdx);
            if (!tracks.empty()) {
                const std::uint32_t desig = world.designate(shooter, radarIdx, tracks.back().id);
                if (desig != 0u) {
                    ms::StrikeCommand sc;
                    sc.payload = stationsIdx;
                    sc.store = "aim-120c";
                    sc.quantity = 1;
                    sc.target = desig;
                    sc.consent = true;
                    if (world.submit(shooter, sc).accepted()) {
                        launchedWave2 = true;
                        std::cout << "[VIS-M13 live-radar] t=" << nowS << "s AIM-120C #2 launched at track "
                                  << tracks.back().id << "\n" << std::flush;
                    }
                }
            }
        }

        const auto nextWall = wallStart + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                              std::chrono::duration<double>((step + 1) * stepDtS));
        std::this_thread::sleep_until(nextWall);
    }
    std::cout << "[VIS-M13 live-radar] finished live showcase (" << durationS << " s)\n" << std::flush;
}

TEST_CASE("Countermeasure payload cards drive per-aircraft inventories and airborne AMTI chaff detection",
          "[combat_vfx][countermeasure]") {
    // 1. Per-aircraft card-driven magazine counts across distinct real dispenser systems.
    {
        session::World world(vfxWorldOptions("cm-card-inventories", 20261128u));
        auto spawn = [&](const char* name, const char* type, double lonOffsetDeg, double speedMs) {
            session::VehicleSpec spec;
            spec.name = name;
            spec.type = type;
            spec.initial.latitudeDeg = 37.65;
            spec.initial.longitudeDeg = -122.42 + lonOffsetDeg;
            spec.initial.altitudeMslM = 4000.0;
            spec.initial.headingDeg = 90.0;
            spec.initial.airspeedTrueMs = speedMs;
            const std::uint32_t id = world.createVehicle(spec);
            REQUIRE(id != 0u);
            return id;
        };

        const std::uint32_t f16 = spawn("viper", "jsbsim:f16c", 0.00, 230.0);
        const std::uint32_t f15 = spawn("eagle", "jsbsim:f15c", 0.05, 240.0);
        const std::uint32_t a10 = spawn("hog", "jsbsim:a10c", 0.10, 145.0);
        const std::uint32_t f22 = spawn("raptor", "jsbsim:f22a", 0.15, 250.0);
        const std::uint32_t b52 = spawn("buff", "jsbsim:b52h", 0.20, 185.0);

        const auto cm16 = world.countermeasures(f16);
        const auto cm15 = world.countermeasures(f15);
        const auto cm10 = world.countermeasures(a10);
        const auto cm22 = world.countermeasures(f22);
        const auto cm52 = world.countermeasures(b52);

        CHECK(cm16.chaff == 30u);
        CHECK(cm16.flare == 15u);
        CHECK(cm15.chaff == 60u);
        CHECK(cm15.flare == 30u);
        CHECK(cm10.chaff == 240u);
        CHECK(cm10.flare == 120u);
        CHECK(cm22.chaff == 0u);
        CHECK(cm22.flare == 30u);
        CHECK(cm52.chaff == 1120u);
        CHECK(cm52.flare == 192u);

        // F-22A internal stealth bays carry flares only: chaff release must reject cleanly without consuming flares.
        const std::uint16_t f22Cm = findPayloadByFamily(world, f22, ms::Family::Countermeasures);
        powerPayloadToOperate(world, f22, f22Cm);
        CHECK(world.dispense(f22, false) == ms::Reason::Expended);
        CHECK(world.dispense(f22, true) == ms::Reason::None);
        CHECK(world.countermeasures(f22).flare == 29u);
    }

    // 2. Airborne AN/APG-68 AMTI radar physically detects chaff dipole clouds dispensed by a target aircraft,
    //    while VFX publishes real bloomed cloud radii (>4 m) and calibrated 3-phase flare radiometry.
    {
        session::World world(vfxWorldOptions("cm-amti-chaff", 20261129u));
        constexpr double kDeg = 3.14159265358979323846 / 180.0;
        const double lat0 = 37.65;
        const double lon0 = -122.42;
        const double cosLat = std::cos(lat0 * kDeg);

        session::VehicleSpec blueSpec;
        blueSpec.name = "viper-radar";
        blueSpec.type = "jsbsim:f16c";
        blueSpec.initial.latitudeDeg = lat0;
        blueSpec.initial.longitudeDeg = lon0;
        blueSpec.initial.altitudeMslM = 5000.0;
        blueSpec.initial.headingDeg = 90.0;
        blueSpec.initial.airspeedTrueMs = 240.0;

        session::VehicleSpec redSpec = blueSpec;
        redSpec.name = "bandit-chaff";
        // 8 km dead ahead along East, flying West head-on so stationary chaff has strong Doppler closing rate (~240 m/s).
        redSpec.initial.longitudeDeg = lon0 + 8000.0 / (111320.0 * cosLat);
        redSpec.initial.headingDeg = 270.0;
        redSpec.initial.airspeedTrueMs = 230.0;

        const std::uint32_t blue = world.createVehicle(blueSpec);
        const std::uint32_t red = world.createVehicle(redSpec);
        REQUIRE(blue != 0u);
        REQUIRE(red != 0u);

        control::HsaCommand blueHsa;
        blueHsa.headingRad = 90.0 * kDeg;
        blueHsa.altitudeM = 5000.0;
        blueHsa.speed = 240.0;
        (void)world.submit(blue, blueHsa);

        control::HsaCommand redHsa;
        redHsa.headingRad = 270.0 * kDeg;
        redHsa.altitudeM = 5000.0;
        redHsa.speed = 230.0;
        (void)world.submit(red, redHsa);

        const std::uint16_t radarIdx = findPayloadByFamily(world, blue, ms::Family::AirRadar);
        const std::uint16_t cmIdx = findPayloadByFamily(world, red, ms::Family::Countermeasures);
        (void)world.submit(blue, ms::StateCommand{radarIdx, ms::PayloadState::Operate});
        (void)world.submit(red, ms::StateCommand{cmIdx, ms::PayloadState::Operate});

        bool radarStarted = false;
        bool dispensedChaff = false;
        bool dispensedFlare = false;
        bool sawChaffRadarDetection = false;
        float maxChaffBloomM = 0.0f;
        float maxChaffRcsM2 = 0.0f;
        float maxFlareIntensity = 0.0f;
        float maxFlareCoreM = 0.0f;
        int cmOperatingStep = -1;

        for (int step = 0; step < 320; ++step) {
            world.step(1);
            const auto* bHost = world.missionSystems(blue);
            const auto* rHost = world.missionSystems(red);
            if (!radarStarted && bHost && bHost->payloads()[radarIdx].state == ms::PayloadState::Operate) {
                ms::AmtiCommand ac;
                ac.payload = radarIdx;
                ac.mode = ms::AmtiMode::TrackWhileScan;
                ac.task = ms::AmtiTask::Volume;
                ac.rangeScaleM = 30000.0;
                ac.azimuthHalfWidthRad = 20.0 * kDeg;
                ac.bars = 2;
                if (world.submit(blue, ac).accepted()) radarStarted = true;
            }
            if (cmOperatingStep < 0 && rHost && rHost->payloads()[cmIdx].state == ms::PayloadState::Operate) {
                cmOperatingStep = step;
            }
            if (radarStarted && cmOperatingStep >= 0) {
                if (!dispensedChaff) {
                    REQUIRE(world.dispense(red, false) == ms::Reason::None);
                    dispensedChaff = true;
                } else if (!dispensedFlare && step >= cmOperatingStep + 20) {
                    REQUIRE(world.dispense(red, true) == ms::Reason::None);
                    dispensedFlare = true;
                } else if (dispensedChaff && (step - cmOperatingStep) == 45) {
                    (void)world.dispense(red, false);
                }
            }
            if (radarStarted) {
                for (const auto& det : world.amtiDetections(blue, radarIdx)) {
                    if (det.truth >= ms::kDecoyIds) {
                        sawChaffRadarDetection = true;
                    }
                }
            }
            const auto vfx = world.combatVfxFrame();
            for (const auto& d : vfx.decoys) {
                if (d.kind == static_cast<std::uint8_t>(ipc::DecoyKindV1::Chaff)) {
                    maxChaffBloomM = std::max(maxChaffBloomM, d.bloomRadiusM);
                    maxChaffRcsM2 = std::max(maxChaffRcsM2, d.intensityOrSigma);
                } else {
                    maxFlareIntensity = std::max(maxFlareIntensity, d.intensityOrSigma);
                    maxFlareCoreM = std::max(maxFlareCoreM, d.bloomRadiusM);
                }
            }
        }

        CHECK(radarStarted);
        CHECK(dispensedChaff);
        CHECK(dispensedFlare);
        CHECK(sawChaffRadarDetection);
        CHECK(maxChaffBloomM > 6.0f);
        CHECK(maxChaffRcsM2 > 12.0f);
        CHECK(maxFlareIntensity > 5000.0f);
        CHECK(maxFlareCoreM > 0.10f);
        CHECK(maxFlareCoreM < 0.25f);
    }
}
