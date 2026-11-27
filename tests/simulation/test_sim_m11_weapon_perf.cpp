// SIM-M11 (HC-45): Parallel WeaponSensor, fast RF/clutter path, and on-rail/reticle IR acceleration.
// Verifies bit-identical cross-worker determinism (workers=1 vs workers=8) across multi-missile
// and on-rail IR loadouts, and enforces sustained >= 2.0x RTF on 4v4 + multi-missile + IR loadouts.

#include "fsim/Agent.h"
#include "fsim/World.h"
#include "simulation/mission_systems/components/implementations/seeker/RadioReceiverV1.h"
#include "simulation/mission_systems/components/implementations/seeker/ThermalReticleReceiverV1.h"
#include "simulation/mission_systems/radar/Signal.h"
#include "simulation/mission_systems/rf/Path.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace {

using namespace fsim;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

WorldOptions makeWorldOptions(std::uint64_t seed, unsigned workers) {
    WorldOptions opts;
    opts.jsbsimRoot = FSIM_TEST_JSBSIM_ROOT;
    opts.publish = false;
    opts.workers = workers;
    opts.pinWorkers = false;
    opts.seed = seed;
    return opts;
}

VehicleSpec makeAircraft(const char* type,
                         const char* name,
                         double latDeg,
                         double lonDeg,
                         double altM,
                         double headingDeg,
                         double speedMs) {
    VehicleSpec spec;
    spec.type = std::string("jsbsim:") + type;
    spec.name = name;
    spec.initial.latitudeDeg = latDeg;
    spec.initial.longitudeDeg = lonDeg;
    spec.initial.altitudeMslM = altM;
    spec.initial.headingDeg = headingDeg;
    spec.initial.airspeedTrueMs = speedMs;
    return spec;
}

std::optional<std::uint16_t> findPayload(const Agent& agent, ms::Family family) {
    for (const auto& p : agent.payloads()) {
        if (p.family == family) return p.index;
    }
    return std::nullopt;
}

void setHsa(Vehicle& v, double headingDeg, double altM, double speedMs) {
    control::HsaCommand hsa;
    hsa.headingRad = headingDeg * kDeg;
    hsa.altitudeM = altM;
    hsa.speed = speedMs;
    REQUIRE(v.submit(hsa).accepted());
}

std::uint64_t fnv1aUpdate(std::uint64_t h, const void* data, std::size_t n) noexcept {
    const auto* bytes = static_cast<const std::uint8_t*>(data);
    for (std::size_t i = 0; i < n; ++i) {
        h ^= bytes[i];
        h *= 1099511628211ull;
    }
    return h;
}

template <typename T>
std::uint64_t hashVal(std::uint64_t h, const T& v) noexcept {
    return fnv1aUpdate(h, &v, sizeof(T));
}

std::uint64_t digestWorld(const Referee& ref) {
    std::uint64_t h = 14695981039346656037ull;
    for (std::uint32_t id : ref.vehicles()) {
        h = hashVal(h, id);
        if (const VehicleState* st = ref.state(id)) {
            h = hashVal(h, st->positionEcef[0]);
            h = hashVal(h, st->positionEcef[1]);
            h = hashVal(h, st->positionEcef[2]);
            h = hashVal(h, st->velocityNedMs[0]);
            h = hashVal(h, st->velocityNedMs[1]);
            h = hashVal(h, st->velocityNedMs[2]);
            h = hashVal(h, st->eulerRad[0]);
            h = hashVal(h, st->eulerRad[1]);
            h = hashVal(h, st->eulerRad[2]);
        }
    }
    for (const auto& w : ref.weapons()) {
        h = hashVal(h, w.id);
        h = hashVal(h, static_cast<std::uint8_t>(w.end));
        h = hashVal(h, static_cast<std::uint8_t>(w.phase));
        h = hashVal(h, w.holding);
        h = hashVal(h, static_cast<std::uint8_t>(w.tracking ? 1 : 0));
    }
    return h;
}

struct MultiMissileResult {
    double wallS = 0.0;
    double rtf = 0.0;
    std::size_t weaponsReleased = 0;
    std::uint64_t digest = 0;
};

/// 4v4 air-to-air scenario with 2 concurrent live AIM-120C missiles + on-rail IR loadouts (AIM-9X & R-73)
/// and active radars/RWRs/IRSTs/chaff.
MultiMissileResult run4v4MultiMissileWithOnRailIr(std::uint64_t seed,
                                                 unsigned workers,
                                                 unsigned activeSteps,
                                                 bool alsoLaunchIrMissiles) {
    World world(makeWorldOptions(seed, workers));

    std::vector<Vehicle> blueVehs;
    std::vector<Vehicle> redVehs;
    // Start 28 km apart head-on so radars acquire quickly and AIM-120C / AIM-9X can engage.
    const double redLat = alsoLaunchIrMissiles ? 35.14 : 35.28;
    for (int i = 0; i < 4; ++i) {
        const char* bType = (i < 2) ? "f35a" : "f16c";
        const std::string bName = "blue_" + std::to_string(i + 1);
        auto bv = world.createVehicle(makeAircraft(bType, bName.c_str(), 35.0, -118.08 + 0.05 * i, 7500.0 + 150.0 * i, 0.0, 250.0));
        const auto bSt = *findPayload(world.createAgent(bv, "tmp_b_" + std::to_string(i)), ms::Family::StoreStation);
        if (i < 2) {
            REQUIRE(bv.loadStore(bSt, 4, "aim-120c") == ms::Reason::None);
            REQUIRE(bv.loadStore(bSt, 8, "aim-120c") == ms::Reason::None);
        } else {
            REQUIRE(bv.loadStore(bSt, 1, "aim-9x") == ms::Reason::None);
            REQUIRE(bv.loadStore(bSt, 9, "aim-9x") == ms::Reason::None);
            REQUIRE(bv.loadStore(bSt, 2, "aim-120c") == ms::Reason::None);
        }
        setHsa(bv, 0.0, 7500.0 + 150.0 * i, 250.0);
        blueVehs.push_back(bv);

        const std::string rName = "red_" + std::to_string(i + 1);
        auto rv = world.createVehicle(makeAircraft("su27s", rName.c_str(), redLat, -118.08 + 0.05 * i, 7500.0 + 150.0 * i, 180.0, 240.0));
        const auto rSt = *findPayload(world.createAgent(rv, "tmp_r_" + std::to_string(i)), ms::Family::StoreStation);
        REQUIRE(rv.loadStore(rSt, 4, "r-27r") == ms::Reason::None);
        REQUIRE(rv.loadStore(rSt, 1, "r-73") == ms::Reason::None);
        setHsa(rv, 180.0, 7500.0 + 150.0 * i, 240.0);
        redVehs.push_back(rv);
    }

    std::vector<Agent> blueAgents, redAgents;
    for (std::size_t i = 0; i < 4; ++i) {
        blueAgents.push_back(world.createAgent(blueVehs[i], "blue_ma_" + std::to_string(i + 1)));
        redAgents.push_back(world.createAgent(redVehs[i], "red_ma_" + std::to_string(i + 1)));
    }

    // Power on radars, RWRs, IRSTs, and store stations across all 8 jets (puts on-rail AIM-9X and R-73 into Ready!).
    for (auto& a : blueAgents) {
        for (const auto& p : a.payloads()) {
            if (p.family == ms::Family::AirRadar || p.family == ms::Family::Esm ||
                p.family == ms::Family::StoreStation || p.family == ms::Family::Countermeasures) {
                (void)a.submit(ms::StateCommand{p.index, ms::PayloadState::Operate});
            }
        }
    }
    for (auto& a : redAgents) {
        for (const auto& p : a.payloads()) {
            if (p.family == ms::Family::AirRadar || p.family == ms::Family::Esm ||
                p.family == ms::Family::PassiveOptical || p.family == ms::Family::StoreStation) {
                (void)a.submit(ms::StateCommand{p.index, ms::PayloadState::Operate});
            }
        }
    }

    // Warm up payloads (~3.5 s so StoreStation reaches Operate while keeping fighters cleanly separated).
    world.step(105u);
    for (auto& a : blueAgents) {
        if (auto r = findPayload(a, ms::Family::AirRadar)) {
            ms::AmtiCommand cmd;
            cmd.payload = *r;
            cmd.mode = ms::AmtiMode::TrackWhileScan;
            cmd.rangeScaleM = 100e3;
            (void)a.submit(cmd);
        }
    }
    for (auto& a : redAgents) {
        if (auto r = findPayload(a, ms::Family::AirRadar)) {
            ms::AmtiCommand cmd;
            cmd.payload = *r;
            cmd.mode = ms::AmtiMode::TrackWhileScan;
            cmd.rangeScaleM = 100e3;
            (void)a.submit(cmd);
        }
    }
    // Wait up to 8 s for Blue radars to hold confirmed tracks.
    for (int i = 0; i < 16; ++i) {
        world.step(15u);
        const auto r0 = findPayload(blueAgents[0], ms::Family::AirRadar);
        const auto r1 = findPayload(blueAgents[1], ms::Family::AirRadar);
        if (r0 && r1 && !blueAgents[0].amtiTracks(*r0).empty() && !blueAgents[1].amtiTracks(*r1).empty()) break;
    }

    // Release 2 concurrent live AIM-120C from blue_1 and blue_2.
    for (std::size_t idx : {std::size_t{0}, std::size_t{1}}) {
        if (auto r = findPayload(blueAgents[idx], ms::Family::AirRadar)) {
            if (auto st = findPayload(blueAgents[idx], ms::Family::StoreStation)) {
                const auto tr = blueAgents[idx].amtiTracks(*r);
                if (!tr.empty()) {
                    const auto desig = blueAgents[idx].designate(*r, tr[0].id);
                    if (desig != 0) {
                        ms::StrikeCommand sc;
                        sc.payload = *st;
                        sc.store = "aim-120c";
                        sc.quantity = 1;
                        sc.target = desig;
                        sc.consent = true;
                        (void)blueVehs[idx].submit(sc);
                    }
                }
            }
        }
    }
    if (alsoLaunchIrMissiles) {
        // Also release 2 concurrent live AIM-9X from blue_3 and blue_4 (total 4 live missiles!).
        for (std::size_t idx : {std::size_t{2}, std::size_t{3}}) {
            if (auto st = findPayload(blueAgents[idx], ms::Family::StoreStation)) {
                ms::StrikeCommand sc;
                sc.payload = *st;
                sc.store = "aim-9x";
                sc.quantity = 1;
                sc.target = redVehs[idx].id();
                sc.consent = true;
                (void)blueVehs[idx].submit(sc);
            }
        }
    }
    (void)blueAgents[2].dispense(false);
    (void)blueAgents[3].dispense(true);

    const auto t0 = std::chrono::steady_clock::now();
    world.step(activeSteps);
    const auto t1 = std::chrono::steady_clock::now();

    MultiMissileResult out;
    out.wallS = std::chrono::duration<double>(t1 - t0).count();
    const double simS = static_cast<double>(activeSteps) * (4.0 / 120.0);
    out.rtf = (out.wallS > 0.0) ? (simS / out.wallS) : 0.0;
    auto ref = world.referee();
    out.weaponsReleased = ref.weapons().size();
    out.digest = digestWorld(ref);
    return out;
}

} // namespace

TEST_CASE("SIM-M11 micro-profile: RF computePath, RadioReceiverV1, and ThermalReticleReceiverV1", "[sim-m11][profile]") {
    using Clock = std::chrono::steady_clock;
    using Ms = std::chrono::duration<double, std::milli>;

    // 1. Flat-earth computePath x 512 ground clutter patches (one active AIM-120C step).
    {
        ms::rf::Terminal a{35.0 * kDeg, -118.0 * kDeg, 7500.0};
        ms::rf::PathOptions opts;
        opts.frequencyHz = 10.0e9;
        opts.ground = nullptr;
        // Warm up the one-time per-frequency P.676 layer table (done once at World startup).
        (void)ms::rf::computePath(a, ms::rf::Terminal{a.latitudeRad + 0.01, a.longitudeRad, 0.0}, opts);
        const auto t0 = Clock::now();
        double sumLoss = 0.0;
        for (int band = 0; band < 64; ++band) {
            const double rangeM = 7500.0 + (band + 0.5) * (42500.0 / 64.0);
            const double dLat = (rangeM / 111132.0) * kDeg;
            for (int az = 0; az < 8; ++az) {
                const double phi = 2.0 * kPi * (az + 0.5) / 8.0;
                ms::rf::Terminal b{a.latitudeRad + dLat * std::cos(phi), a.longitudeRad + dLat * std::sin(phi), 0.0};
                const auto res = ms::rf::computePath(a, b, opts);
                sumLoss += res.totalDb;
            }
        }
        const double ms = Ms(Clock::now() - t0).count();
        std::cout << "[SIM-M11 profile] computePath(512 flat ground patches): " << ms << " ms (sum=" << sumLoss << ")\n";
        CHECK(std::isfinite(sumLoss));
    }

    // 2. RadioReceiverV1::expose with 512 ground patches (AIM-120C medium-PRF 3-interval waveform).
    {
        ms::components::RadioReceiverConfigV1 cfg;
        cfg.version = 1;
        cfg.frequencyHz = 10.0e9;
        cfg.temperatureK = 290.0;
        cfg.noiseFigureDb = 6.0;
        cfg.systemLossDb = 6.0;
        cfg.transmitAveragePowerW = 20.0;
        cfg.bandwidthHz = 1.0e6;
        cfg.monopulseSlope = 1.5;
        cfg.clutterRateSigmaMs = 2.0;
        cfg.associationGateRad = 0.1;
        cfg.temporalMemoryS = 0.12;
        cfg.observationWindowS = 0.012;
        cfg.aperture.shape = ms::rf::ApertureShape::Circular;
        cfg.aperture.taper = ms::rf::Taper::Uniform;
        cfg.aperture.widthM = 0.10668;
        cfg.aperture.heightM = 0.10668;
        cfg.aperture.efficiency = 0.6;
        cfg.aperture.pedestal = 0.0;
        cfg.aperture.peakGainDbi = 0.0;
        cfg.aperture.floorDbi = -30.0;
        cfg.aperture.backDbi = -40.0;
        cfg.aperture.scanExponent = 1.5;
        cfg.waveform = ms::radar::signal::mediumPrf(10.0e9, 0.012, 3, 3, 15.0, 0.05);
        cfg.processing.rangeScaleM = 100000.0;
        cfg.processing.minRateMs = -2000.0;
        cfg.processing.maxRateMs = 2000.0;
        cfg.processing.pfaInterval = 1e-6;
        cfg.processing.cfarCells = 16;
        cfg.processing.cfarGuard = 2;
        cfg.processing.coincidence = 1;
        cfg.processing.notchMs = 5.0;

        ms::components::RadioReceiverV1 rx;
        std::string err;
        REQUIRE(rx.configure(cfg, err));

        ms::components::RadioSceneFrameV1 scene;
        scene.timeS = 1.0;
        scene.frequencyHz = 10.0e9;
        scene.up = {0.0, 0.0, 1.0};
        scene.altitudeM = 7500.0;
        for (int i = 0; i < 512; ++i) {
            const double band = static_cast<double>(i / 8);
            const double az = 2.0 * kPi * ((i % 8) + 0.5) / 8.0;
            const double r = 8000.0 + band * 650.0;
            ms::components::RadioEchoSceneV1 g;
            const double down = 7500.0 / r;
            const double horiz = std::sqrt(std::max(0.0, 1.0 - down * down));
            g.direction = {horiz * std::cos(az), horiz * std::sin(az), -down};
            g.receiverRangeM = g.delayRangeM = r;
            g.dopplerHz = 2.0 * 700.0 * g.direction.x / 0.03;
            g.rcsM2 = 1500.0;
            scene.groundPatches.push_back(g);
        }
        std::array<ms::weapon::V3, 3> axes{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
        const auto t0 = Clock::now();
        const auto res = rx.expose(scene, {1, 0, 0}, axes, 42u, false);
        const double totalMs = Ms(Clock::now() - t0).count();
        REQUIRE(res.valid);
        std::cout << "[SIM-M11 profile] RadioReceiverV1::expose(512 ground patches): total=" << totalMs
                  << " ms (clutter=" << res.stats.clutterS * 1e3 << " ms, process=" << res.stats.processS * 1e3
                  << " ms, cells=" << res.stats.cells << ")\n";
    }

    // 3. ThermalReticleReceiverV1::expose on-rail AIM-9M level flight (horizon crossing FOV, empty sky).
    {
        ms::components::ThermalReticleConfigV1 cfg;
        cfg.version = 1;
        cfg.periods = 12;
        cfg.samplesPerScan = 128;
        cfg.spatialSamples = 4;
        cfg.scanRadiusRad = 0.024;
        cfg.stopRadiusRad = 0.048;
        cfg.frequencyHz = 132.62911924324612;
        auto& d = cfg.detector;
        d.version = 1;
        d.width = 64;
        d.height = 64;
        d.adcBits = 14;
        d.exposureSamples = 4;
        d.pixelPitchRad = 0.0015;
        d.psfSigmaRad = 0.00075;
        d.apertureAreaM2 = 0.0007917304360898403;
        d.opticalTransmission = 0.65;
        d.quantumEfficiency = 0.6;
        d.effectiveWavelengthM = 4e-6;
        d.integrationTimeS = 1e-5;
        d.pixelResponseTimeS = 1e-4;
        d.readNoiseElectrons = 12.0;
        d.darkCurrentElectronsS = 100.0;
        d.fullWellElectrons = 1e6;
        d.detectionSigma = 5.0;
        d.backgroundRadiusPixels = 5.0;
        d.minimumFeaturePixels = 1.0;
        d.associationGateRad = 0.035;

        const auto tc0 = Clock::now();
        ms::components::ThermalReticleReceiverV1 reticle;
        std::string err;
        REQUIRE(reticle.configure(cfg, err));
        const double cfgMs = Ms(Clock::now() - tc0).count();

        ms::components::ThermalSceneFrameV1 scene;
        scene.timeS = 0.0;
        scene.up = {0.0, 0.0, 1.0};
        scene.groundHorizonCosine = -0.048;
        scene.skyRadianceWm2Sr = ms::components::ThermalImageReceiverV1::bandRadiance(220.0, 1.0, 3.0, 5.0);
        scene.groundRadianceWm2Sr = ms::components::ThermalImageReceiverV1::bandRadiance(288.0, 1.0, 3.0, 5.0);
        (void)reticle.expose(scene, {1, 0, 0}, 77u);
        scene.timeS = 0.03333333333333333;
        const auto te0 = Clock::now();
        const auto r = reticle.expose(scene, {1, 0, 0}, 77u);
        const double exposeMs = Ms(Clock::now() - te0).count();
        REQUIRE(r.valid);
        REQUIRE(r.complete);
        // Sub-breakdown of expose(): incidentFieldV1 (single bg render) vs inferMeasuredV1 (fit + fitHorizon) vs unitGroundFieldV1.
        auto optical = cfg.detector;
        optical.exposureSamples = 1;
        optical.integrationTimeS = cfg.detector.integrationTimeS / cfg.detector.exposureSamples;
        const auto tb0 = Clock::now();
        const auto bgField = ms::components::ThermalImageReceiverV1::incidentFieldV1(scene, {1, 0, 0}, optical);
        const double bgMs = Ms(Clock::now() - tb0).count();
        REQUIRE(bgField.has_value());
        auto rInfer = r;
        const auto ti0 = Clock::now();
        reticle.inferMeasuredV1(rInfer, {1, 0, 0}, scene.up);
        const double inferMs = Ms(Clock::now() - ti0).count();
        auto rHoriz = r;
        const auto th0 = Clock::now();
        reticle.inferStaticHorizonForCalibrationV2(rHoriz, {1, 0, 0}, scene.up);
        const double horizMs = Ms(Clock::now() - th0).count();
        const auto tu0 = Clock::now();
        const auto ug = ms::components::ThermalReticleReceiverV1::unitGroundFieldV1(cfg.detector, {1, 0, 0}, scene.up, -0.048);
        const double ugMs = Ms(Clock::now() - tu0).count();
        REQUIRE(ug.has_value());
        std::cout << "[SIM-M11 profile] ThermalReticleReceiverV1(aim-9m level horizon empty sky): configure=" << cfgMs
                  << " ms, expose=" << exposeMs << " ms (bg1=" << bgMs << " ms, infer=" << inferMs
                  << " ms, staticHoriz=" << horizMs << " ms, unitGround1=" << ugMs << " ms), features=" << r.features.size()
                  << " horizonRefusal=" << r.horizonDiagnostic.refusal << "\n";
        // Also check mid-FOV horizon (groundHorizonCosine = 0.0, e.g. level/slightly nose-down scanning horizon).
        auto midScene = scene;
        midScene.groundHorizonCosine = 0.0;
        midScene.timeS = 0.06666666666666667;
        const auto tm0 = Clock::now();
        const auto rMid = reticle.expose(midScene, {1, 0, 0}, 77u);
        const double midMs = Ms(Clock::now() - tm0).count();
        const auto tmBg0 = Clock::now();
        const auto midBg = ms::components::ThermalImageReceiverV1::incidentFieldV1(midScene, {1, 0, 0}, optical);
        const double midBgMs = Ms(Clock::now() - tmBg0).count();
        auto rMidH = rMid;
        const auto tmH0 = Clock::now();
        reticle.inferStaticHorizonForCalibrationV2(rMidH, {1, 0, 0}, midScene.up);
        const double midHMs = Ms(Clock::now() - tmH0).count();
        REQUIRE(rMid.valid);
        REQUIRE(midBg.has_value());
        std::cout << "[SIM-M11 profile] ThermalReticleReceiverV1(aim-9m mid-FOV horizon): expose=" << midMs
                  << " ms (bg1=" << midBgMs << " ms, staticHoriz=" << midHMs << " ms), features=" << rMid.features.size()
                  << " horizonRefusal=" << rMid.horizonDiagnostic.refusal << "\n";

        // Also check clean sky (no horizon inside FOV, e.g. pitched +5 deg up) vs with a fighter target patch.
        auto skyScene = scene;
        skyScene.groundHorizonCosine = -0.25;
        skyScene.timeS = 0.1;
        const auto ts0 = Clock::now();
        const auto rSky = reticle.expose(skyScene, {1, 0, 0}, 77u);
        const double skyMs = Ms(Clock::now() - ts0).count();
        REQUIRE(rSky.valid);
        std::cout << "[SIM-M11 profile] ThermalReticleReceiverV1(aim-9m pure sky): expose=" << skyMs
                  << " ms, features=" << rSky.features.size() << " horizonRefusal=" << rSky.horizonDiagnostic.refusal << "\n";
    }

    // 4. Single F-16C carrying 2x on-rail AIM-9M (StoreStation Operate, 2.0 s = 60 steps).
    {
        World world(makeWorldOptions(5501ull, 4));
        auto jet = world.createVehicle(makeAircraft("f16c", "viper", 35.0, -118.0, 7500.0, 0.0, 250.0));
        const auto st = *findPayload(world.createAgent(jet, "tmp"), ms::Family::StoreStation);
        REQUIRE(jet.loadStore(st, 1, "aim-9m") == ms::Reason::None);
        REQUIRE(jet.loadStore(st, 9, "aim-9m") == ms::Reason::None);
        setHsa(jet, 0.0, 7500.0, 250.0);
        auto ag = world.createAgent(jet, "pilot");
        REQUIRE(ag.submit(ms::StateCommand{st, ms::PayloadState::Operate}).accepted());
        world.step(45u); // power up to Ready
        const auto t0 = Clock::now();
        double maxStepMs = 0.0;
        unsigned slowSteps = 0;
        for (unsigned k = 0; k < 60u; ++k) {
            const auto ts = Clock::now();
            world.step(1u);
            const double stepMs = Ms(Clock::now() - ts).count();
            if (stepMs > maxStepMs) maxStepMs = stepMs;
            if (stepMs > 100.0) {
                ++slowSteps;
                if (slowSteps <= 8u) {
                    std::cout << "  [slow step] k=" << k << " t=" << world.time() << " wallMs=" << stepMs << "\n";
                }
            }
        }
        const double wallS = std::chrono::duration<double>(Clock::now() - t0).count();
        const auto stJet = jet.state();
        const double upDotFwd = stJet.rotationBodyToEcef[0] * std::cos(stJet.latitudeRad) * std::cos(stJet.longitudeRad)
                              + stJet.rotationBodyToEcef[3] * std::cos(stJet.latitudeRad) * std::sin(stJet.longitudeRad)
                              + stJet.rotationBodyToEcef[6] * std::sin(stJet.latitudeRad);
        std::cout << "[SIM-M11 profile] F-16C + 2x on-rail AIM-9M Ready (2.0s sim): wall=" << wallS
                  << " s, RTF=" << (2.0 / wallS) << "x, maxStepMs=" << maxStepMs << " slowSteps=" << slowSteps
                  << " altAgl=" << stJet.altitudeAglM << " pitchDeg="
                  << (std::asin(std::clamp(upDotFwd, -1.0, 1.0)) * 180.0 / 3.14159265358979323846) << "\n";
    }
}

TEST_CASE("SIM-M11 HC-45: 4v4 multi-missile + on-rail IR determinism and >= 2.0x RTF", "[sim-m11][hc45][performance]") {
    // 1. Cross-worker bit-identical determinism on 4v4 + 2 live AIM-120C + on-rail AIM-9X/R-73 (2.0 s = 60 steps).
    const auto d1 = run4v4MultiMissileWithOnRailIr(20261111ull, 1u, 60u, false);
    const auto d8 = run4v4MultiMissileWithOnRailIr(20261111ull, 8u, 60u, false);
    std::cout << "[SIM-M11 det-2x120c] w1: wall=" << d1.wallS << "s RTF=" << d1.rtf << "x wpns=" << d1.weaponsReleased
              << " | w8: wall=" << d8.wallS << "s RTF=" << d8.rtf << "x wpns=" << d8.weaponsReleased << "\n";
    REQUIRE(d1.weaponsReleased >= 2u);
    CHECK(d1.weaponsReleased == d8.weaponsReleased);
    CHECK(d1.digest == d8.digest);

    // 2. Sustained 4v4 + 2 concurrent live AIM-120C + on-rail AIM-9X/R-73 (10.0 s = 300 steps, 8 workers).
    const auto b2 = run4v4MultiMissileWithOnRailIr(20261112ull, 8u, 300u, false);
    std::cout << "[SIM-M11 bench-4v4+2x120c+onrailIR] 10s sim: wall=" << b2.wallS << "s RTF=" << b2.rtf
              << "x wpns=" << b2.weaponsReleased << "\n";
    CHECK(b2.weaponsReleased >= 2u);
    CHECK(b2.rtf >= 2.0);

    // 3. 4v4 + 4 concurrent live missiles (2x AIM-120C + 2x AIM-9X) determinism (w1 vs w8) & RTF.
    const auto f1 = run4v4MultiMissileWithOnRailIr(20261113ull, 1u, 60u, true);
    const auto f8 = run4v4MultiMissileWithOnRailIr(20261113ull, 8u, 60u, true);
    std::cout << "[SIM-M11 bench-4v4+4live(2x120c+2x9x)] 2s sim w1=" << f1.wallS << "s (" << f1.rtf
              << "x) w8=" << f8.wallS << "s (" << f8.rtf << "x) wpns=" << f8.weaponsReleased << "\n";
    REQUIRE(f1.weaponsReleased >= 4u);
    CHECK(f1.weaponsReleased == f8.weaponsReleased);
    CHECK(f1.digest == f8.digest);
    CHECK(f8.rtf >= 2.0);
}
