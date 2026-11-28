#include "simulation/mission_systems/Cards.h"
#include "simulation/mission_systems/gun/Gun.h"
#include "simulation/mission_systems/rcs/StoreRcs.h"
#include "simulation/runtime/world/World.h"
#include "foundation/core/Geodesy.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

using namespace fsim;

namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;
constexpr double kEarthR = 6371008.8;

double metersToDegLat(double meters) { return (meters / kEarthR) / kDeg; }
double metersToDegLon(double meters, double latDeg) {
    return (meters / (kEarthR * std::cos(latDeg * kDeg))) / kDeg;
}

session::WorldOptions baseOptions(std::uint64_t seed) {
    session::WorldOptions opt;
    opt.jsbsimRoot = FSIM_TEST_JSBSIM_ROOT;
    opt.dt = 1.0 / 100.0;
    opt.frameSkip = 2; // 50 Hz world step
    opt.workers = 1;
    opt.pinWorkers = false;
    opt.publish = false;
    opt.terrain = false;
    opt.seed = seed;
    return opt;
}

double reportValue(const std::vector<ms::PayloadValue>& rep, std::string_view name) {
    for (const auto& kv : rep) {
        if (kv.name == name) return kv.value;
    }
    return std::numeric_limits<double>::quiet_NaN();
}

std::uint16_t findPayloadByName(const session::World& world, std::uint32_t id, std::string_view name) {
    const auto* host = world.missionSystems(id);
    REQUIRE(host != nullptr);
    for (const auto& p : host->payloads()) {
        if (p.name && std::string_view(p.name) == name) return p.index;
    }
    FAIL("Payload not found: " << name);
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

struct BallisticResult {
    double timeS = 0.0;
    double rangeNorthM = 0.0;
    double rangeEastM = 0.0;
    double impactSpeedMs = 0.0;
};

// Independent RK4 point-mass standard-atmosphere ballistic integrator (McCoy / NATO STANAG 4355)
// for B-50 cross-check against the live 6-DOF JSBSim Mk 82 drop.
BallisticResult integratePointMassRk4(double h0M, double vn0, double ve0, double vd0,
                                      double massKg, double dragAreaM2,
                                      double windNorthMs, double windEastMs) {
    constexpr double g = 9.80665;
    constexpr double rho0 = 1.225;
    constexpr double dt = 0.005;

    struct State {
        double n, e, h, vn, ve, vd; // h = altitude up (m), vd = down velocity (m/s)
    };
    auto deriv = [&](const State& s) -> State {
        const double rho = rho0 * ms::geom::densityRatio(std::max(0.0, s.h));
        const double vrN = s.vn - windNorthMs;
        const double vrE = s.ve - windEastMs;
        const double vrD = s.vd;
        const double vRel = std::sqrt(vrN * vrN + vrE * vrE + vrD * vrD);
        const double k = 0.5 * rho * dragAreaM2 / massKg;
        return {
            s.vn,
            s.ve,
            -s.vd,
            -k * vRel * vrN,
            -k * vRel * vrE,
            g - k * vRel * vrD
        };
    };
    auto add = [](const State& a, const State& b, double scale) -> State {
        return {
            a.n + b.n * scale, a.e + b.e * scale, a.h + b.h * scale,
            a.vn + b.vn * scale, a.ve + b.ve * scale, a.vd + b.vd * scale
        };
    };

    State s{0.0, 0.0, h0M, vn0, ve0, vd0};
    double t = 0.0;
    while (s.h > 0.0 && t < 180.0) {
        const State k1 = deriv(s);
        const State k2 = deriv(add(s, k1, 0.5 * dt));
        const State k3 = deriv(add(s, k2, 0.5 * dt));
        const State k4 = deriv(add(s, k3, dt));
        State next{
            s.n + (dt / 6.0) * (k1.n + 2.0 * k2.n + 2.0 * k3.n + k4.n),
            s.e + (dt / 6.0) * (k1.e + 2.0 * k2.e + 2.0 * k3.e + k4.e),
            s.h + (dt / 6.0) * (k1.h + 2.0 * k2.h + 2.0 * k3.h + k4.h),
            s.vn + (dt / 6.0) * (k1.vn + 2.0 * k2.vn + 2.0 * k3.vn + k4.vn),
            s.ve + (dt / 6.0) * (k1.ve + 2.0 * k2.ve + 2.0 * k3.ve + k4.ve),
            s.vd + (dt / 6.0) * (k1.vd + 2.0 * k2.vd + 2.0 * k3.vd + k4.vd)
        };
        if (next.h <= 0.0) {
            const double frac = s.h / std::max(s.h - next.h, 1e-9);
            const double tHit = t + frac * dt;
            const double nHit = s.n + frac * (next.n - s.n);
            const double eHit = s.e + frac * (next.e - s.e);
            const double vnHit = s.vn + frac * (next.vn - s.vn);
            const double veHit = s.ve + frac * (next.ve - s.ve);
            const double vdHit = s.vd + frac * (next.vd - s.vd);
            return {tHit, nHit, eHit, std::sqrt(vnHit * vnHit + veHit * veHit + vdHit * vdHit)};
        }
        s = next;
        t += dt;
    }
    return {t, s.n, s.e, std::sqrt(s.vn * s.vn + s.ve * s.ve + s.vd * s.vd)};
}

} // namespace

TEST_CASE("SIM-M10: Exterior ballistics, 11/11 Gun cards, and physical-optics StoreRcs", "[sim-m10][ms]") {
    // 1. McCoy sea-level quadratic drag retardation factor k = 0.5 * rho0 * Cd * A / m
    const double kM61 = ms::gun::dragRetardationPerM(0.020, 0.101, 0.30);
    // Expected: 0.5 * 1.225 * 0.30 * (pi * 0.01^2) / 0.101 ~= 5.716e-4 1/m
    REQUIRE(kM61 == Catch::Approx(5.7155e-4).margin(1e-5));
    const double kGau8 = ms::gun::dragRetardationPerM(0.030, 0.360, 0.28);
    REQUIRE(kGau8 < kM61);
    REQUIRE(kGau8 > 2.5e-4);

    // 2. All 11 maintained Family::Gun cards load cleanly and construct GunModel instances
    const std::filesystem::path payloadDir = std::filesystem::path(FSIM_TEST_PAYLOAD_DIR);
    const std::vector<std::string> gunCards = {
        "m61a1", "m61a2", "gau-22a", "gau-8a", "bk-27",
        "defa-554", "giat-30m791", "gsh-23", "gsh-30-1", "gsh-30-2", "m60d"
    };
    for (const auto& id : gunCards) {
        std::vector<std::string> errs;
        const auto card = ms::readCard(payloadDir / (id + ".json"), errs);
        INFO("card=" << id << " errs=" << errs.size());
        REQUIRE(card != nullptr);
        REQUIRE(card->family == ms::Family::Gun);
        const ms::gun::Spec spec = ms::gun::specOf(*card, 1);
        REQUIRE(spec.calibreM >= 0.007);
        REQUIRE(spec.rateOfFireRpm >= 500.0);
        REQUIRE(spec.muzzleMs >= 700.0);
        REQUIRE(spec.projectileKg > 0.005);
        REQUIRE(spec.shedMassPerRoundKg >= spec.projectileKg);
        REQUIRE(spec.capacity >= 100);

        auto model = ms::gun::makeGun(*card, 1);
        REQUIRE(model != nullptr);
        std::vector<ms::PayloadValue> rep;
        model->report(rep);
        REQUIRE(reportValue(rep, "rounds_remaining") == Catch::Approx(static_cast<double>(spec.capacity)));
        REQUIRE(reportValue(rep, "calibre_mm") == Catch::Approx(spec.calibreM * 1000.0));
        REQUIRE(reportValue(rep, "rate_of_fire_rpm") == Catch::Approx(spec.rateOfFireRpm));
    }

    // 3. Physical-optics StoreRcs (B-37): broadside specular > nose; tank/pod > missile
    const std::filesystem::path storeDir = std::filesystem::path(FSIM_TEST_STORE_DIR);
    std::vector<std::string> errs;
    const auto tankCard = ms::readStoreCard(storeDir / "tank-370gal.json", errs);
    REQUIRE(tankCard != nullptr);
    const auto aim120Card = ms::readStoreCard(storeDir / "aim-120c.json", errs);
    REQUIRE(aim120Card != nullptr);
    const double noseDir[3] = {1.0, 0.0, 0.0};
    const double broadsideDir[3] = {0.0, 1.0, 0.0};
    const double fX = 10.0e9;
    const double tankNose = ms::rcs::externalStoreSigmaM2(*tankCard, noseDir, fX);
    const double tankSide = ms::rcs::externalStoreSigmaM2(*tankCard, broadsideDir, fX);
    const double missileNose = ms::rcs::externalStoreSigmaM2(*aim120Card, noseDir, fX);
    const double missileSide = ms::rcs::externalStoreSigmaM2(*aim120Card, broadsideDir, fX);
    REQUIRE(tankNose > 0.05);
    REQUIRE(tankSide > tankNose * 3.0);
    REQUIRE(tankNose > missileNose);
    REQUIRE(tankSide > missileSide);
}

TEST_CASE("SIM-M10: Gun firing lifecycle, B-33 ammunition mass conservation, and authority gating", "[sim-m10][ms][b33]") {
    session::World world(baseOptions(101));
    session::VehicleSpec f16Spec;
    f16Spec.name = "viper";
    f16Spec.type = "jsbsim:f16c";
    f16Spec.initial.latitudeDeg = 35.0;
    f16Spec.initial.longitudeDeg = -117.0;
    f16Spec.initial.altitudeMslM = 5000.0;
    f16Spec.initial.headingDeg = 90.0;
    f16Spec.initial.airspeedTrueMs = 220.0;
    const std::uint32_t f16 = world.createVehicle(f16Spec);
    REQUIRE(f16 != 0);

    // Clean twin F-16C that does NOT fire its gun, isolating exact B-33 mass loss from fuel burn.
    session::VehicleSpec twinSpec = f16Spec;
    twinSpec.name = "twin";
    twinSpec.initial.latitudeDeg = 35.05;
    const std::uint32_t twin = world.createVehicle(twinSpec);
    REQUIRE(twin != 0);

    // Mirage 2000 dual DEFA-554 count=2 scaling (250 rounds, 3600 rpm).
    session::VehicleSpec m2kSpec = f16Spec;
    m2kSpec.name = "mirage";
    m2kSpec.type = "jsbsim:mirage2000";
    m2kSpec.initial.latitudeDeg = 35.10;
    const std::uint32_t m2k = world.createVehicle(m2kSpec);
    REQUIRE(m2k != 0);
    const std::uint16_t m2kGun = findPayloadByName(world, m2k, "guns");
    const auto m2kRep = world.payloadReport(m2k, m2kGun);
    REQUIRE(reportValue(m2kRep, "magazine_capacity") == Catch::Approx(250.0));
    REQUIRE(reportValue(m2kRep, "rate_of_fire_rpm") == Catch::Approx(3600.0));

    const std::uint16_t gunIdx = findPayloadByName(world, f16, "gun");
    const std::string strikeCap = "fsim.ms.strike:gun";

    // Firing while Off must be refused with PayloadOff.
    ms::PayloadTask fireTask;
    fireTask.payload = gunIdx;
    fireTask.kind = ms::TaskKind::Fire;
    fireTask.rounds = 50;
    const auto offRes = world.submit(f16, fireTask);
    REQUIRE(!offRes.accepted());
    REQUIRE(offRes.reason == ms::Reason::PayloadOff);

    // Power gun to Operate and verify ControlMode::Granted gating.
    powerPayloadToOperate(world, f16, gunIdx);
    REQUIRE(world.setControlMode(f16, control::ControlMode::Granted) == control::Reason::None);
    ms::CommandOptions policyOpts;
    policyOpts.source = control::Source::Policy;
    policyOpts.controller = 42;
    const auto ungranted = world.submit(f16, fireTask, policyOpts);
    REQUIRE(!ungranted.accepted());
    REQUIRE(ungranted.reason == ms::Reason::NotGranted);
    REQUIRE(world.requestControl(f16, strikeCap, 42) == control::Reason::None);

    // Step 5 ticks so both F-16Cs settle identically before firing.
    world.step(5);
    const double massBeforeF16 = world.model(f16)->energy().massKg;
    const double massBeforeTwin = world.model(twin)->energy().massKg;
    REQUIRE(massBeforeF16 == Catch::Approx(massBeforeTwin).margin(0.05));

    // Fire a 100-round burst (M61A1: 6000 rpm = 100 rds/s -> completes in ~1.0 s).
    fireTask.rounds = 100;
    const auto burstRes = world.submit(f16, fireTask, policyOpts);
    REQUIRE(burstRes.accepted());
    world.step(80); // 1.6 s at 50 Hz

    const auto* act = world.missionSystems(f16)->activity(burstRes.activity);
    REQUIRE(act != nullptr);
    REQUIRE(act->state == control::ActivityState::Completed);
    REQUIRE(act->reason == ms::Reason::GoalReached);

    const auto repAfter = world.payloadReport(f16, gunIdx);
    REQUIRE(reportValue(repAfter, "rounds_fired") == Catch::Approx(100.0));
    REQUIRE(reportValue(repAfter, "rounds_remaining") == Catch::Approx(400.0));

    // B-33: JSBSim mass of firing F-16C must be lower than twin by 100 * 0.155 kg = 15.5 kg.
    const double massAfterF16 = world.model(f16)->energy().massKg;
    const double massAfterTwin = world.model(twin)->energy().massKg;
    const double massDiffKg = massAfterTwin - massAfterF16;
    std::cout << "[SIM-M10 B-33] 100 rounds fired: delta_mass_vs_twin=" << massDiffKg
              << " kg (expected 15.5 kg)\n";
    REQUIRE(massDiffKg == Catch::Approx(15.5).margin(0.5));

    // Continuous fire until magazine exhaustion (remaining 400 rounds -> ~4.0 s).
    ms::PayloadTask emptyTask;
    emptyTask.payload = gunIdx;
    emptyTask.kind = ms::TaskKind::Fire;
    emptyTask.rounds = 0; // continuous
    const auto emptyRes = world.submit(f16, emptyTask, policyOpts);
    REQUIRE(emptyRes.accepted());
    world.step(260); // 5.2 s

    const auto* emptyAct = world.missionSystems(f16)->activity(emptyRes.activity);
    REQUIRE(emptyAct != nullptr);
    REQUIRE(emptyAct->state == control::ActivityState::Failed);
    REQUIRE(emptyAct->reason == ms::Reason::Expended);
    REQUIRE(reportValue(world.payloadReport(f16, gunIdx), "rounds_remaining") == Catch::Approx(0.0));

    // Total B-33 shed mass across all 500 rounds: 500 * 0.155 = 77.5 kg.
    const double totalMassDiffKg = world.model(twin)->energy().massKg - world.model(f16)->energy().massKg;
    std::cout << "[SIM-M10 B-33] 500 rounds expended: delta_mass_vs_twin=" << totalMassDiffKg
              << " kg (expected 77.5 kg)\n";
    REQUIRE(totalMassDiffKg == Catch::Approx(77.5).margin(1.5));

    // Firing again on an empty magazine must be refused with Expended.
    const auto afterEmpty = world.submit(f16, fireTask, policyOpts);
    REQUIRE(!afterEmpty.accepted());
    REQUIRE(afterEmpty.reason == ms::Reason::Expended);
}

TEST_CASE("SIM-M10: Live air-to-air gun engagement triggers native VehicleDamage deterministically", "[sim-m10][ms][native-engagement]") {
    auto runTailChase = [](std::uint64_t seed) {
        session::World world(baseOptions(seed));
        session::VehicleSpec shooterSpec;
        shooterSpec.name = "shooter";
        shooterSpec.type = "jsbsim:f16c";
        shooterSpec.initial.latitudeDeg = 35.0;
        shooterSpec.initial.longitudeDeg = -117.0;
        shooterSpec.initial.altitudeMslM = 5000.0;
        shooterSpec.initial.headingDeg = 90.0; // east
        shooterSpec.initial.airspeedTrueMs = 220.0;
        const std::uint32_t shooter = world.createVehicle(shooterSpec);
        REQUIRE(shooter != 0);

        // Place target 220 m dead ahead on the same heading and altitude (short gun-range tracking shot).
        session::VehicleSpec targetSpec = shooterSpec;
        targetSpec.name = "target";
        targetSpec.initial.longitudeDeg = shooterSpec.initial.longitudeDeg + metersToDegLon(220.0, 35.0);
        const std::uint32_t target = world.createVehicle(targetSpec);
        REQUIRE(target != 0);

        // Hold both level east so they stay aligned during the burst.
        control::HsaCommand hold;
        hold.headingRad = 90.0 * kDeg;
        hold.altitudeM = 5000.0;
        hold.speed = 220.0;
        REQUIRE(world.command(shooter, hold));
        REQUIRE(world.command(target, hold));

        const std::uint16_t gunIdx = findPayloadByName(world, shooter, "gun");
        powerPayloadToOperate(world, shooter, gunIdx);

        ms::PayloadTask fireTask;
        fireTask.payload = gunIdx;
        fireTask.kind = ms::TaskKind::Fire;
        fireTask.rounds = 120;
        REQUIRE(world.submit(shooter, fireTask).accepted());

        world.step(100); // 2.0 s: burst completes (~1.2 s) and rounds reach 220 m (~0.21 s flight)

        struct Summary {
            int gunDamageEvents = 0;
            int hitEvents = 0;
            double targetSpeedMs = 0.0;
            double roundsRemaining = 0.0;
        };
        Summary out;
        for (const auto& ev : world.engagementEvents()) {
            if (ev.kind == "damage" && ev.reason == "gun" && ev.target == target) ++out.gunDamageEvents;
        }
        for (const auto& ev : world.takeEngagementEvents()) {
            if (ev.kind == static_cast<std::uint8_t>(ipc::EngagementKind::Hit)) ++out.hitEvents;
        }
        out.targetSpeedMs = world.vehicleState(target)->airspeedTrueMs;
        out.roundsRemaining = reportValue(world.payloadReport(shooter, gunIdx), "rounds_remaining");
        return out;
    };

    const auto runA = runTailChase(777);
    const auto runB = runTailChase(777);
    std::cout << "[SIM-M10 Gun A2A] gunDamageEvents=" << runA.gunDamageEvents
              << " hitEvents=" << runA.hitEvents
              << " roundsRemaining=" << runA.roundsRemaining << "\n";

    REQUIRE(runA.roundsRemaining == Catch::Approx(380.0));
    REQUIRE(runA.gunDamageEvents >= 5);
    REQUIRE(runA.hitEvents >= 5);
    // Bit-identical determinism across two runs with the same seed.
    REQUIRE(runA.gunDamageEvents == runB.gunDamageEvents);
    REQUIRE(runA.hitEvents == runB.hitEvents);
    REQUIRE(runA.targetSpeedMs == runB.targetSpeedMs);
    REQUIRE(runA.roundsRemaining == runB.roundsRemaining);
}

TEST_CASE("SIM-M10 (B-37): Loaded external stores increase RCS while closed stealth bays add zero", "[sim-m10][ms][b37]") {
    session::World world(baseOptions(202));

    session::VehicleSpec obsNoseSpec;
    obsNoseSpec.name = "obs-nose";
    obsNoseSpec.type = "jsbsim:f16c";
    obsNoseSpec.initial.latitudeDeg = 35.0;
    obsNoseSpec.initial.longitudeDeg = -117.0;
    obsNoseSpec.initial.altitudeMslM = 6000.0;
    obsNoseSpec.initial.headingDeg = 90.0;
    obsNoseSpec.initial.airspeedTrueMs = 220.0;
    const std::uint32_t obsNose = world.createVehicle(obsNoseSpec);

    const double tgtLonDeg = -117.0 + metersToDegLon(30000.0, 35.0);
    session::VehicleSpec obsSideSpec = obsNoseSpec;
    obsSideSpec.name = "obs-side";
    obsSideSpec.initial.latitudeDeg = 35.0 - metersToDegLat(30000.0);
    obsSideSpec.initial.longitudeDeg = tgtLonDeg;
    obsSideSpec.initial.headingDeg = 0.0;
    const std::uint32_t obsSide = world.createVehicle(obsSideSpec);

    auto spawnTarget = [&](const std::string& name, const std::string& type) {
        session::VehicleSpec sp = obsNoseSpec;
        sp.name = name;
        sp.type = type;
        sp.initial.longitudeDeg = tgtLonDeg;
        sp.initial.headingDeg = 270.0; // nose pointing toward obsNose
        const std::uint32_t id = world.createVehicle(sp);
        REQUIRE(id != 0);
        return id;
    };

    const std::uint32_t f35Clean = spawnTarget("f35-clean", "jsbsim:f35a");
    const std::uint32_t f35Internal = spawnTarget("f35-internal", "jsbsim:f35a");
    REQUIRE(world.loadLoadout(f35Internal, "internal air-to-air") == ms::Reason::None);

    const std::uint32_t f16Clean = spawnTarget("f16-clean", "jsbsim:f16c");
    const std::uint32_t f16Bombs = spawnTarget("f16-mk82", "jsbsim:f16c");
    REQUIRE(world.loadLoadout(f16Bombs, "mk82 x4") == ms::Reason::None);
    const std::uint32_t f16Tanks = spawnTarget("f16-tanks", "jsbsim:f16c");
    REQUIRE(world.loadLoadout(f16Tanks, "external tanks") == ms::Reason::None);

    const std::uint32_t ea18Clean = spawnTarget("ea18g-clean", "jsbsim:ea18g");
    const std::uint32_t ea18Pods = spawnTarget("ea18g-pods", "jsbsim:ea18g");
    REQUIRE(world.loadLoadout(ea18Pods, "jamming pods") == ms::Reason::None);

    ms::RcsQuery q;
    q.frequencyHz = 10.0e9;

    // 1. Stealth internal weapon bay (doors closed) must preserve clean F-35A RCS bit-identically.
    const auto rcsF35CleanNose = world.radarCrossSection(obsNose, f35Clean, q);
    const auto rcsF35IntNose = world.radarCrossSection(obsNose, f35Internal, q);
    const auto rcsF35CleanSide = world.radarCrossSection(obsSide, f35Clean, q);
    const auto rcsF35IntSide = world.radarCrossSection(obsSide, f35Internal, q);
    REQUIRE(rcsF35CleanNose.has_value());
    REQUIRE(rcsF35IntNose.has_value());
    REQUIRE(rcsF35IntNose->sigmaM2 == rcsF35CleanNose->sigmaM2);
    REQUIRE(rcsF35IntSide->sigmaM2 == rcsF35CleanSide->sigmaM2);

    // 2. External Mk 82s (using aircraft/mk82/rcs.bin) and external tanks increase F-16C RCS.
    const auto rcsF16CleanNose = world.radarCrossSection(obsNose, f16Clean, q);
    const auto rcsF16BombsNose = world.radarCrossSection(obsNose, f16Bombs, q);
    const auto rcsF16TanksNose = world.radarCrossSection(obsNose, f16Tanks, q);
    const auto rcsF16CleanSide = world.radarCrossSection(obsSide, f16Clean, q);
    const auto rcsF16TanksSide = world.radarCrossSection(obsSide, f16Tanks, q);
    REQUIRE(rcsF16CleanNose.has_value());
    REQUIRE(rcsF16BombsNose->sigmaM2 > rcsF16CleanNose->sigmaM2);
    REQUIRE(rcsF16TanksNose->sigmaM2 > rcsF16CleanNose->sigmaM2);
    REQUIRE(rcsF16TanksSide->sigmaM2 > rcsF16CleanSide->sigmaM2);

    // 3. EA-18G ALQ-99 jamming pods increase both frontal and broadside RCS.
    const auto rcsEaCleanNose = world.radarCrossSection(obsNose, ea18Clean, q);
    const auto rcsEaPodsNose = world.radarCrossSection(obsNose, ea18Pods, q);
    const auto rcsEaCleanSide = world.radarCrossSection(obsSide, ea18Clean, q);
    const auto rcsEaPodsSide = world.radarCrossSection(obsSide, ea18Pods, q);
    REQUIRE(rcsEaPodsNose->sigmaM2 > rcsEaCleanNose->sigmaM2);
    REQUIRE(rcsEaPodsSide->sigmaM2 > rcsEaCleanSide->sigmaM2);

    std::cout << "[SIM-M10 B-37] F-35A nose clean=" << rcsF35CleanNose->sigmaM2
              << " internal=" << rcsF35IntNose->sigmaM2
              << " | F-16C nose clean=" << rcsF16CleanNose->sigmaM2
              << " mk82x4=" << rcsF16BombsNose->sigmaM2
              << " tanks=" << rcsF16TanksNose->sigmaM2
              << " | EA-18G nose clean=" << rcsEaCleanNose->sigmaM2
              << " pods=" << rcsEaPodsNose->sigmaM2 << " m^2\n";
}

TEST_CASE("SIM-M10 (B-50): Mk 82 free-fall bomb ballistic validation vs RK4 standard-atmosphere ODE", "[sim-m10][ms][b50]") {
    auto runDrop = [](double windNorthMs) {
        session::World world(baseOptions(303));
        if (std::abs(windNorthMs) > 0.0) {
            sim::EnvironmentState env = world.environment();
            // Meteorological direction FROM: 180 deg = wind from south blowing northward.
            env.windDirectionDeg = windNorthMs >= 0.0 ? 180.0 : 0.0;
            env.windSpeedMs = std::abs(windNorthMs);
            world.setEnvironment(env);
        }

        session::VehicleSpec f16Spec;
        f16Spec.name = "bomber";
        f16Spec.type = "jsbsim:f16c";
        f16Spec.initial.latitudeDeg = 35.0;
        f16Spec.initial.longitudeDeg = -117.0;
        f16Spec.initial.altitudeMslM = 3000.0;
        f16Spec.initial.headingDeg = 90.0; // east
        f16Spec.initial.airspeedTrueMs = 220.0;
        const std::uint32_t bomber = world.createVehicle(f16Spec);
        REQUIRE(bomber != 0);
        REQUIRE(world.loadLoadout(bomber, "mk82 x4") == ms::Reason::None);

        control::HsaCommand hold;
        hold.headingRad = 90.0 * kDeg;
        hold.altitudeM = 3000.0;
        hold.speed = 220.0;
        REQUIRE(world.command(bomber, hold));

        const std::uint16_t stationsIdx = findPayloadByName(world, bomber, "stations");
        powerPayloadToOperate(world, bomber, stationsIdx);

        // Let the aircraft stabilize briefly in level flight before releasing 1 Mk 82.
        world.step(50); // 1.0 s

        ms::StrikeCommand strike;
        strike.payload = stationsIdx;
        strike.store = "mk82";
        strike.quantity = 1;
        const auto res = world.submit(bomber, strike);
        REQUIRE(res.accepted());

        std::uint32_t bombId = 0;
        double releaseTimeS = 0.0;
        sim::VehicleState relState{};
        bool consentGiven = false;

        for (int i = 0; i < 400 && bombId == 0; ++i) {
            world.step(1);
            if (!consentGiven) {
                const auto st = world.strikeStatus(res.activity);
                if (st && st->phase == ms::StrikePhase::AwaitingConsent) {
                    REQUIRE(world.strikeConsent(res.activity, true) == ms::Reason::None);
                    consentGiven = true;
                }
            }
            for (const std::uint32_t id : world.vehicleIds()) {
                const auto* info = world.info(id);
                if (info && info->role == ms::Role::Weapon) {
                    bombId = id;
                    releaseTimeS = world.simTime();
                    relState = *world.vehicleState(id);
                    break;
                }
            }
        }
        REQUIRE(bombId != 0);

        // Step until the Mk 82 strikes the ground (WeaponEnd::Impact).
        sim::VehicleState lastBombState = relState;
        bool impacted = false;
        double impactTimeS = 0.0;
        for (int i = 0; i < 4000; ++i) {
            world.step(1);
            if (const auto* st = world.vehicleState(bombId)) {
                lastBombState = *st;
            } else {
                for (const auto& rec : world.weapons()) {
                    if (rec.id == bombId) {
                        impacted = (rec.end == ms::WeaponEnd::Impact || rec.end == ms::WeaponEnd::Detonated);
                        impactTimeS = rec.endTimeS;
                        break;
                    }
                }
                break;
            }
        }
        REQUIRE(impacted);

        double northM = 0.0, eastM = 0.0;
        geo::localNorthEastM(relState.latitudeRad, relState.longitudeRad,
                             lastBombState.latitudeRad, lastBombState.longitudeRad,
                             northM, eastM);
        const double impactSpeedMs = std::sqrt(
            lastBombState.velocityNedMs[0] * lastBombState.velocityNedMs[0] +
            lastBombState.velocityNedMs[1] * lastBombState.velocityNedMs[1] +
            lastBombState.velocityNedMs[2] * lastBombState.velocityNedMs[2]);
        struct DropSummary {
            sim::VehicleState release;
            BallisticResult sim;
        };
        return DropSummary{relState, {impactTimeS - releaseTimeS, northM, eastM, impactSpeedMs}};
    };

    const auto calm = runDrop(0.0);
    std::vector<std::string> errs;
    const auto mk82Card = ms::readStoreCard(std::filesystem::path(FSIM_TEST_STORE_DIR) / "mk82.json", errs);
    REQUIRE(mk82Card != nullptr);
    const double massKg = mk82Card->value("mass_kg", 226.8);
    const double dragAreaM2 = mk82Card->value("drag_area_m2", 0.00952846);

    const BallisticResult refCalm = integratePointMassRk4(
        calm.release.altitudeMslM,
        calm.release.velocityNedMs[0], calm.release.velocityNedMs[1], calm.release.velocityNedMs[2],
        massKg, dragAreaM2, 0.0, 0.0);

    const double tofErrPct = 100.0 * std::abs(calm.sim.timeS - refCalm.timeS) / refCalm.timeS;
    const double rangeErrPct = 100.0 * std::abs(calm.sim.rangeEastM - refCalm.rangeEastM) / refCalm.rangeEastM;
    const double speedErrPct = 100.0 * std::abs(calm.sim.impactSpeedMs - refCalm.impactSpeedMs) / refCalm.impactSpeedMs;

    std::cout << "[SIM-M10 B-50 Calm] h0=" << calm.release.altitudeMslM
              << " m | TOF sim=" << calm.sim.timeS << " s vs RK4=" << refCalm.timeS << " s (err " << tofErrPct << "%)"
              << " | RangeE sim=" << calm.sim.rangeEastM << " m vs RK4=" << refCalm.rangeEastM << " m (err " << rangeErrPct << "%)"
              << " | Vimp sim=" << calm.sim.impactSpeedMs << " m/s vs RK4=" << refCalm.impactSpeedMs << " m/s (err " << speedErrPct << "%)\n";

    REQUIRE(calm.sim.timeS > 20.0);
    REQUIRE(calm.sim.timeS < 35.0);
    REQUIRE(tofErrPct < 6.0);
    REQUIRE(rangeErrPct < 10.0);
    REQUIRE(speedErrPct < 10.0);

    // Crosswind run: 20 m/s wind blowing northward must drift the impact point distinctly north.
    const auto windy = runDrop(20.0);
    const double driftNorthM = windy.sim.rangeNorthM - calm.sim.rangeNorthM;
    std::cout << "[SIM-M10 B-50 Crosswind 20 m/s N] driftNorth=" << driftNorthM << " m\n";
    REQUIRE(driftNorthM > 15.0);
    REQUIRE(driftNorthM < 400.0);
}

TEST_CASE("Phase 3 Gun Ballistics: BRL/McCoy G7 Mach drag curve, per-round self-destruct and crosswind RK4 drift", "[sim-m10][ms][gun-ballistics]") {
    // 1) Published BRL / McCoy G7 drag function Cd(M): low subsonic -> sharp transonic wave-drag peak -> supersonic decay
    const double cdSub = ms::gun::g7DragCoefficient(0.50);
    const double cdTrans = ms::gun::g7DragCoefficient(1.05);
    const double cdSuper2 = ms::gun::g7DragCoefficient(2.00);
    const double cdSuper3 = ms::gun::g7DragCoefficient(3.00);
    CHECK(cdSub == Catch::Approx(0.1198).margin(1e-3));
    CHECK(cdTrans == Catch::Approx(0.4043).margin(1e-3));
    CHECK(cdTrans > cdSub * 3.0);
    CHECK(cdSuper2 < cdTrans);
    CHECK(cdSuper3 < cdSuper2);

    // 2) Per-round pyrotechnic self-destruct lifetimes loaded strictly from payload cards (no C++ card.id table)
    std::vector<std::string> cardErrs;
    const std::filesystem::path payloadDir(FSIM_TEST_PAYLOAD_DIR);
    const auto m61 = ms::readCard(payloadDir / "m61a1.json", cardErrs);
    const auto gau8 = ms::readCard(payloadDir / "gau-8a.json", cardErrs);
    REQUIRE(cardErrs.empty());
    REQUIRE(m61 != nullptr);
    REQUIRE(gau8 != nullptr);
    CHECK(ms::gun::specOf(*m61, 1).maxFlightTimeS == Catch::Approx(5.5));
    CHECK(ms::gun::specOf(*gau8, 1).maxFlightTimeS == Catch::Approx(9.5));

    // 3) Live RK4 air-relative wind drift & self-destruct verification
    // Apply crosswind immediately after muzzle exit so calm and windy bursts share
    // identical launch position and muzzle velocity (isolating exterior ballistics from aircraft weathercocking).
    auto fireAndMeasure = [](double windNorthMs) {
        session::World world(baseOptions(404));
        session::VehicleSpec f16Spec;
        f16Spec.name = "gunner";
        f16Spec.type = "jsbsim:f16c";
        f16Spec.initial.latitudeDeg = 35.0;
        f16Spec.initial.longitudeDeg = -117.0;
        f16Spec.initial.altitudeMslM = 1500.0;
        f16Spec.initial.headingDeg = 90.0; // east
        f16Spec.initial.airspeedTrueMs = 220.0;
        const std::uint32_t gunner = world.createVehicle(f16Spec);
        REQUIRE(gunner != 0);
        const std::uint16_t gunIdx = findPayloadByName(world, gunner, "gun");
        powerPayloadToOperate(world, gunner, gunIdx);

        ms::PayloadTask fire;
        fire.payload = gunIdx;
        fire.kind = ms::TaskKind::Fire;
        fire.rounds = 12;
        REQUIRE(world.submit(gunner, fire).accepted());

        // Step 8 ticks (0.16 s) so the 12-round burst (6000 rpm = 0.12 s burst) has exited the muzzle cleanly
        world.step(8);
        if (std::abs(windNorthMs) > 0.0) {
            sim::EnvironmentState env = world.environment();
            env.windDirectionDeg = 180.0; // blowing FROM south -> northward
            env.windSpeedMs = windNorthMs;
            world.setEnvironment(env);
        }

        // Advance 3.0 s (150 ticks at 50 Hz): rounds decelerate from Mach ~3.6 through transonic drag peak
        world.step(150);
        const auto frame = world.combatVfxFrame();
        REQUIRE(!frame.projectiles.empty());
        // Project average projectile displacement along local North unit vector at (35 N, -117 E)
        const double lat0 = 35.0 * kDeg, lon0 = -117.0 * kDeg;
        const double northEcef[3] = {
            -std::sin(lat0) * std::cos(lon0),
            -std::sin(lat0) * std::sin(lon0),
            std::cos(lat0)};
        double sumNorth = 0.0;
        for (const auto& p : frame.projectiles) {
            sumNorth += p.positionEcef[0] * northEcef[0] +
                        p.positionEcef[1] * northEcef[1] +
                        p.positionEcef[2] * northEcef[2];
        }
        const double meanNorth = sumNorth / static_cast<double>(frame.projectiles.size());
        // Step another 3.0 s (total > 6.1 s > M61A1 5.5 s pyrotechnic self-destruct): all rounds must expire
        world.step(150);
        const bool allExpired = world.combatVfxFrame().projectiles.empty();
        return std::pair<double, bool>{meanNorth, allExpired};
    };

    const auto [calmNorth, calmExpired] = fireAndMeasure(0.0);
    const auto [windyNorth, windyExpired] = fireAndMeasure(30.0);
    CHECK(calmExpired);
    CHECK(windyExpired);
    // By Didion's wind-drift law y = w * (t - x/v0), a 30 m/s northward crosswind over 3.0 s at 1500 m
    // produces ~40-60 m of pure aerodynamic lateral drift on 20 mm M56 projectiles.
    const double driftM = windyNorth - calmNorth;
    CHECK(driftM > 25.0);
    CHECK(driftM < 90.0);
}
