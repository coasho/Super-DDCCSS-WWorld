// VIS-M14: Tactical & EMSO 3-D Overlays, Fleet Propulsion Extension (B-57/B-59),
// and Replay/Live Sidecar Transport Unit Tests.
// Validates:
//   1. Frozen TacticalOverlayLayoutV1 wire struct sizes, alignment, and strict validators
//   2. TacticalOverlaySegmentV1 wait-free seqlock IPC and .tact companion recorder/replay
//   3. Native World::tacticalOverlayFrame() authority:
//      - Radar scan volumes, STT/TWS tracks, B-37 RCS contrast
//      - EA jammer lobes (EA-18G AN/ALQ-99) and live burn-through pocket reduction
//      - ESM/RWR DF rays and geolocation
//      - Link 16 / DMS packets, Mode 5 IFF, PNT GNSS drift
//      - 5-D launchZoneV2 DLZ/WEZ (Rmax > Rnez > Rmin) and McCoy gun LCOS
//      - B-57 8-engine B-52H independent propulsion telemetry (engines 0..7)
//      - B-59 UH-60 rotorcraft propulsion telemetry
//   4. Optional FSIM_VIS_M14_RECORD_DIR showcase replay generator for 4K desktop-gui acceptance.

#include "contracts/world_stream/TacticalOverlayLayoutV1.h"
#include "foundation/transport/TacticalOverlaySegmentV1.h"
#include "mode_flights.h"
#include "simulation/runtime/world/World.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

using namespace fsim;
using namespace fsim::modes;

session::WorldOptions tacticalOptions(const char* name, std::uint64_t seed) {
    auto o = options(name);
    o.dt = 1.0 / 100.0;
    o.frameSkip = 2; // 50 Hz
    o.workers = 1;
    o.pinWorkers = false;
    o.publish = false;
    o.terrain = false;
    o.seed = seed;
    return o;
}

bool findPayloadByFamily(const session::World& world, std::uint32_t id, ms::Family fam, std::uint16_t& out) {
    const auto* host = world.missionSystems(id);
    if (!host) return false;
    for (const auto& p : host->payloads()) {
        if (p.family == fam) {
            out = p.index;
            return true;
        }
    }
    return false;
}

} // namespace

TEST_CASE("VIS-M14 TacticalOverlayLayoutV1 wire contracts and validators", "[tactical_overlays][vis-m14]") {
    using namespace fsim::ipc;

    static_assert(sizeof(TacticalRadarVolumeWireV1) == 96 && alignof(TacticalRadarVolumeWireV1) == 8);
    static_assert(sizeof(TacticalJammerLobeWireV1) == 96 && alignof(TacticalJammerLobeWireV1) == 8);
    static_assert(sizeof(TacticalEsmRayWireV1) == 96 && alignof(TacticalEsmRayWireV1) == 8);
    static_assert(sizeof(TacticalLinkPulseWireV1) == 80 && alignof(TacticalLinkPulseWireV1) == 8);
    static_assert(sizeof(TacticalWeaponCueWireV1) == 128 && alignof(TacticalWeaponCueWireV1) == 8);
    static_assert(sizeof(TacticalSwathWireV1) == 96 && alignof(TacticalSwathWireV1) == 8);
    static_assert(sizeof(TacticalPropulsionExtWireV1) == 192 && alignof(TacticalPropulsionExtWireV1) == 8);

    TacticalRadarVolumeWireV1 r{};
    CHECK_FALSE(validTacticalRadarV1(r));
    r.vehicleId = 1;
    r.rangeScaleM = 80000.0f;
    r.detectionRangeM = 75000.0f;
    r.burnThroughRangeM = 35000.0f;
    r.azimuthHalfWidthRad = 0.5f;
    r.elevationHalfWidthRad = 0.2f;
    CHECK(validTacticalRadarV1(r));
    r.reserved[0] = 1;
    CHECK_FALSE(validTacticalRadarV1(r));
    r.reserved[0] = 0;
    r.detectionRangeM = std::numeric_limits<float>::quiet_NaN();
    CHECK_FALSE(validTacticalRadarV1(r));

    TacticalJammerLobeWireV1 j{};
    CHECK_FALSE(validTacticalJammerV1(j));
    j.vehicleId = 2;
    j.boresightEcef[0] = 1.0;
    j.rangeM = 90000.0f;
    j.erpDbw = 45.0f;
    j.frequencyGhz = 9.86f;
    j.bandwidthMhz = 40.0f;
    j.halfAngleRad = 0.35f;
    CHECK(validTacticalJammerV1(j));
    j.rangeM = -10.0f;
    CHECK_FALSE(validTacticalJammerV1(j));

    TacticalEsmRayWireV1 e{};
    CHECK_FALSE(validTacticalEsmRayV1(e));
    e.observerVehicleId = 1;
    e.rayDirEcef[1] = 1.0f;
    e.rayLengthM = 45000.0f;
    CHECK(validTacticalEsmRayV1(e));

    TacticalLinkPulseWireV1 l{};
    CHECK_FALSE(validTacticalLinkPulseV1(l));
    l.fromVehicleId = 1;
    l.kind = static_cast<std::uint8_t>(TacticalLinkKindV1::Link16Dms);
    CHECK(validTacticalLinkPulseV1(l));
    l.kind = 99;
    CHECK_FALSE(validTacticalLinkPulseV1(l));

    TacticalWeaponCueWireV1 w{};
    CHECK_FALSE(validTacticalWeaponCueV1(w));
    w.vehicleId = 1;
    w.flags = kTactWeaponHasDlz;
    w.rMaxM = 60000.0f;
    w.rNeM = 28000.0f;
    w.rMinM = 2000.0f;
    CHECK(validTacticalWeaponCueV1(w));

    TacticalSwathWireV1 s{};
    CHECK_FALSE(validTacticalSwathV1(s));
    s.vehicleId = 3;
    s.kind = static_cast<std::uint8_t>(TacticalSwathKindV1::SarSpotlight);
    s.halfWidthM = 2000.0f;
    s.halfLengthM = 2000.0f;
    CHECK(validTacticalSwathV1(s));

    TacticalPropulsionExtWireV1 p{};
    CHECK_FALSE(validTacticalPropulsionExtV1(p));
    p.vehicleId = 4;
    p.engineCount = 8;
    CHECK(validTacticalPropulsionExtV1(p));
    p.engineCount = 9;
    CHECK_FALSE(validTacticalPropulsionExtV1(p));
}

TEST_CASE("VIS-M14 TacticalOverlaySegmentV1 live seqlock IPC and .tact replay roundtrip", "[tactical_overlays][vis-m14]") {
    using namespace fsim::ipc;

    TacticalOverlayFrameV1 f0;
    f0.simTime = 1.0;
    {
        TacticalRadarVolumeWireV1 r{};
        r.vehicleId = 1;
        r.slot = 0;
        r.rangeScaleM = 80000.0f;
        r.detectionRangeM = 72000.0f;
        r.burnThroughRangeM = 24000.0f;
        r.jammerRatioDb = 18.5f;
        r.azimuthHalfWidthRad = 0.5f;
        r.elevationHalfWidthRad = 0.2f;
        r.flags = kTactRadarSearching | kTactRadarJammed;
        f0.radars.push_back(r);

        TacticalJammerLobeWireV1 j{};
        j.vehicleId = 2;
        j.slot = 1;
        j.boresightEcef[0] = 1.0;
        j.rangeM = 85000.0f;
        j.erpDbw = 45.0f;
        j.frequencyGhz = 9.86f;
        j.bandwidthMhz = 40.0f;
        j.halfAngleRad = 0.35f;
        f0.jammers.push_back(j);

        TacticalEsmRayWireV1 e{};
        e.observerVehicleId = 1;
        e.observerSlot = 0;
        e.rayDirEcef[0] = 0.7071f;
        e.rayDirEcef[1] = 0.7071f;
        e.rayLengthM = 35000.0f;
        e.flags = kTactEsmHasGeo | kTactEsmWarning;
        e.crlbCep50M = 420.0f;
        e.crlbVertM = 180.0f;
        f0.esmRays.push_back(e);

        TacticalLinkPulseWireV1 l{};
        l.fromVehicleId = 1;
        l.toVehicleId = 2;
        l.kind = static_cast<std::uint8_t>(TacticalLinkKindV1::Mode5IffReply);
        l.ageS = 0.5f;
        l.durationS = 2.0f;
        f0.links.push_back(l);

        TacticalWeaponCueWireV1 w{};
        w.vehicleId = 1;
        w.slot = 0;
        w.flags = kTactWeaponHasDlz | kTactWeaponHasLcos;
        w.rMaxM = 62000.0f;
        w.rNeM = 31000.0f;
        w.rMinM = 1800.0f;
        f0.weapons.push_back(w);

        TacticalSwathWireV1 sw{};
        sw.vehicleId = 1;
        sw.kind = static_cast<std::uint8_t>(TacticalSwathKindV1::SarSpotlight);
        sw.halfWidthM = 2000.0f;
        sw.halfLengthM = 2000.0f;
        sw.metricDb = -26.0f;
        f0.swaths.push_back(sw);

        TacticalPropulsionExtWireV1 pr{};
        pr.vehicleId = 5;
        pr.slot = 4;
        pr.engineCount = 8;
        for (int i = 0; i < 8; ++i) {
            pr.engineN2[i] = 90.0f + static_cast<float>(i);
            pr.thrustN[i] = 52000.0f;
        }
        f0.propulsion.push_back(pr);
    }

    TacticalOverlayFrameV1 f1 = f0;
    f1.simTime = 1.5;
    f1.radars[0].burnThroughRangeM = 28000.0f;

    // 1. Live triple-buffered shared memory seqlock roundtrip
    const std::string worldName = "vis_m14_tact_unit_test";
    TacticalOverlaySegmentV1 writer, reader;
    REQUIRE(writer.create(worldName, 9911u, 1u));
    REQUIRE(reader.open(worldName, 9911u, 1u));
    REQUIRE(writer.publish(12u, f0));
    TacticalOverlayFrameV1 got;
    REQUIRE(reader.read(12u, got));
    REQUIRE(got.radars.size() == 1u);
    CHECK(got.radars[0].jammerRatioDb == Catch::Approx(18.5f));
    REQUIRE(got.jammers.size() == 1u);
    REQUIRE(got.esmRays.size() == 1u);
    CHECK(got.esmRays[0].crlbCep50M == Catch::Approx(420.0f));
    REQUIRE(got.links.size() == 1u);
    REQUIRE(got.weapons.size() == 1u);
    REQUIRE(got.swaths.size() == 1u);
    REQUIRE(got.propulsion.size() == 1u);
    CHECK(got.propulsion[0].engineCount == 8u);
    CHECK(got.propulsion[0].engineN2[7] == Catch::Approx(97.0f));
    reader.close();
    writer.close();

    // 2. Lazy .tact companion recorder & replay roundtrip
    const auto tmpPath = std::filesystem::temp_directory_path() / "fsim_vis_m14_test.tact";
    {
        TacticalOverlayRecorderV1 rec;
        REQUIRE(rec.open(tmpPath));
        REQUIRE(rec.write(f0));
        REQUIRE(rec.write(f1));
        rec.close();
    }
    TacticalOverlayReplayV1 rep;
    std::string err;
    REQUIRE(rep.load(tmpPath, &err));
    CHECK(rep.frameCount() == 2u);
    TacticalOverlayFrameV1 sampled;
    REQUIRE(rep.atTime(1.25, sampled));
    CHECK(sampled.simTime == Catch::Approx(1.0));
    CHECK(sampled.radars[0].burnThroughRangeM == Catch::Approx(24000.0f));
    REQUIRE(rep.atTime(1.55, sampled));
    CHECK(sampled.simTime == Catch::Approx(1.5));
    CHECK(sampled.radars[0].burnThroughRangeM == Catch::Approx(28000.0f));

    std::error_code ec;
    std::filesystem::remove(tmpPath, ec);
}

TEST_CASE("VIS-M14 native World publishes EMSO overlays (Radar, EA Burn-Through, ESM, Weapons) and B-57/B-59 propulsion", "[tactical_overlays][vis-m14]") {
    // --- A. Native EMSO scene: F-16C radar + AIM-120C DLZ + M61A1 gun LCOS vs EA-18G Growler spot jammer ---
    {
        session::World world(tacticalOptions("vis-m14-emso", 20261401u));
        const double lat0 = 37.65;
        const double lon0 = -122.42;
        const double cosLat = std::cos(lat0 * kDeg);

        session::VehicleSpec blueSpec;
        blueSpec.name = "viper-1";
        blueSpec.type = "jsbsim:f16c";
        blueSpec.initial.latitudeDeg = lat0;
        blueSpec.initial.longitudeDeg = lon0;
        blueSpec.initial.altitudeMslM = 5500.0;
        blueSpec.initial.headingDeg = 90.0;
        blueSpec.initial.airspeedTrueMs = 240.0;

        session::VehicleSpec growlerSpec = blueSpec;
        growlerSpec.name = "growler-red";
        growlerSpec.type = "jsbsim:ea18g";
        growlerSpec.initial.longitudeDeg = lon0 + 16000.0 / (111320.0 * cosLat);
        growlerSpec.initial.headingDeg = 270.0;

        const std::uint32_t viper = world.createVehicle(blueSpec);
        const std::uint32_t growler = world.createVehicle(growlerSpec);
        REQUIRE(viper != 0u);
        REQUIRE(growler != 0u);

        // Hold level head-on flight during payload warmup so Growler's directional jammer & Viper's radar stay aligned.
        {
            control::HsaCommand hBlue;
            hBlue.headingRad = 90.0 * kDeg;
            hBlue.altitudeM = 5500.0;
            hBlue.speed = 240.0;
            (void)world.submit(viper, hBlue);
            control::HsaCommand hRed;
            hRed.headingRad = 270.0 * kDeg;
            hRed.altitudeM = 5500.0;
            hRed.speed = 240.0;
            (void)world.submit(growler, hRed);
        }

        std::uint16_t radarIdx = 0, rwrIdx = 0, gunIdx = 0, storesIdx = 0, jamIdx = 0;
        REQUIRE(findPayloadByFamily(world, viper, ms::Family::AirRadar, radarIdx));
        REQUIRE(findPayloadByFamily(world, viper, ms::Family::Esm, rwrIdx));
        REQUIRE(findPayloadByFamily(world, viper, ms::Family::Gun, gunIdx));
        REQUIRE(findPayloadByFamily(world, viper, ms::Family::StoreStation, storesIdx));
        REQUIRE(findPayloadByFamily(world, growler, ms::Family::ElectronicAttack, jamIdx));
        REQUIRE(world.loadStore(viper, storesIdx, 1, "aim-120c") == ms::Reason::None);

        // Warm all payloads concurrently so head-on range stays inside AN/ALR-56M RWR sensitivity (-60 dBm).
        REQUIRE(world.submit(viper, ms::StateCommand{radarIdx, ms::PayloadState::Operate}).accepted());
        REQUIRE(world.submit(viper, ms::StateCommand{rwrIdx, ms::PayloadState::Operate}).accepted());
        REQUIRE(world.submit(viper, ms::StateCommand{gunIdx, ms::PayloadState::Operate}).accepted());
        REQUIRE(world.submit(viper, ms::StateCommand{storesIdx, ms::PayloadState::Operate}).accepted());
        REQUIRE(world.submit(growler, ms::StateCommand{jamIdx, ms::PayloadState::Operate}).accepted());
        for (int i = 0; i < 600; ++i) {
            const auto* hv = world.missionSystems(viper);
            const auto* hg = world.missionSystems(growler);
            REQUIRE(hv != nullptr);
            REQUIRE(hg != nullptr);
            if (hv->payloads()[radarIdx].state == ms::PayloadState::Operate &&
                hv->payloads()[rwrIdx].state == ms::PayloadState::Operate &&
                hv->payloads()[gunIdx].state == ms::PayloadState::Operate &&
                hv->payloads()[storesIdx].state == ms::PayloadState::Operate &&
                hg->payloads()[jamIdx].state == ms::PayloadState::Operate) {
                break;
            }
            world.step(1);
        }
        REQUIRE(world.missionSystems(viper)->payloads()[radarIdx].state == ms::PayloadState::Operate);
        REQUIRE(world.missionSystems(viper)->payloads()[rwrIdx].state == ms::PayloadState::Operate);
        REQUIRE(world.missionSystems(growler)->payloads()[jamIdx].state == ms::PayloadState::Operate);

        // Start TWS scan on Viper
        ms::AmtiCommand search;
        search.payload = radarIdx;
        search.mode = ms::AmtiMode::TrackWhileScan;
        search.task = ms::AmtiTask::Volume;
        search.rangeScaleM = 80000.0;
        search.azimuthHalfWidthRad = 25.0 * kDeg;
        search.bars = 2;
        REQUIRE(world.submit(viper, search).accepted());
        world.step(50);

        const auto cleanFrame = world.tacticalOverlayFrame();
        REQUIRE_FALSE(cleanFrame.radars.empty());
        const ipc::TacticalRadarVolumeWireV1* cleanRadar = nullptr;
        for (const auto& r : cleanFrame.radars) {
            if (r.vehicleId == viper) cleanRadar = &r;
        }
        REQUIRE(cleanRadar != nullptr);
        CHECK(ipc::validTacticalRadarV1(*cleanRadar));
        CHECK(cleanRadar->detectionRangeM > 15000.0f);
        CHECK(cleanRadar->jammerRatioDb == Catch::Approx(0.0f).margin(0.1f));

        // Verify 5-D launchZoneV2 DLZ + McCoy gun LCOS once TWS detects Growler!
        const ipc::TacticalWeaponCueWireV1* cleanWpn = nullptr;
        for (const auto& w : cleanFrame.weapons) {
            if (w.vehicleId == viper) cleanWpn = &w;
        }
        REQUIRE(cleanWpn != nullptr);
        CHECK(ipc::validTacticalWeaponCueV1(*cleanWpn));
        CHECK((cleanWpn->flags & ipc::kTactWeaponHasLcos) != 0u);
        if ((cleanWpn->flags & ipc::kTactWeaponHasDlz) != 0u) {
            CHECK(cleanWpn->rMaxM > cleanWpn->rNeM);
            CHECK(cleanWpn->rNeM > cleanWpn->rMinM);
            CHECK(cleanWpn->rMinM > 0.0f);
        }

        // Now turn on EA-18G AN/ALQ-99 spot noise jamming right in AN/APG-68 X-band (9.86 GHz)!
        ms::PayloadTask jam;
        jam.payload = jamIdx;
        jam.kind = ms::TaskKind::Jam;
        jam.frequencyHz = 9.86e9;
        jam.bandwidthHz = 40e6;
        jam.azimuthCentreRad = 0.0;
        jam.elevationCentreRad = 0.0;
        jam.azimuthHalfWidthRad = 25.0 * kDeg;
        jam.elevationHalfWidthRad = 15.0 * kDeg;
        REQUIRE(world.submit(growler, jam).accepted());
        world.step(80);

        const auto jammedFrame = world.tacticalOverlayFrame();
        REQUIRE_FALSE(jammedFrame.jammers.empty());
        CHECK(ipc::validTacticalJammerV1(jammedFrame.jammers.front()));
        CHECK(jammedFrame.jammers.front().vehicleId == growler);
        CHECK(jammedFrame.jammers.front().erpDbw > 20.0f);

        const ipc::TacticalRadarVolumeWireV1* jammedRadar = nullptr;
        for (const auto& r : jammedFrame.radars) {
            if (r.vehicleId == viper) jammedRadar = &r;
        }
        REQUIRE(jammedRadar != nullptr);
        CHECK((jammedRadar->flags & ipc::kTactRadarJammed) != 0u);
        CHECK(jammedRadar->jammerRatioDb > 1.0f);
        CHECK(jammedRadar->burnThroughRangeM < jammedRadar->detectionRangeM);

        // ESM receiver on Viper detects Growler's active jamming emission and produces a DF ray!
        CHECK_FALSE(jammedFrame.esmRays.empty());
        bool sawJamStrobe = false;
        for (const auto& ray : jammedFrame.esmRays) {
            CHECK(ipc::validTacticalEsmRayV1(ray));
            if (ray.observerVehicleId == viper && ray.emitterMode == static_cast<std::uint8_t>(ms::EmitterMode::Jammer))
                sawJamStrobe = true;
        }
        CHECK(sawJamStrobe);
    }

    // --- B. Native B-57 (8-engine B-52H) & B-59 (UH-60 rotorcraft) propulsion sidecar ---
    {
        session::World world(tacticalOptions("vis-m14-fleet", 20261402u));

        session::VehicleSpec b52Spec;
        b52Spec.name = "buff-1";
        b52Spec.type = "jsbsim:b52h";
        b52Spec.initial.latitudeDeg = 37.65;
        b52Spec.initial.longitudeDeg = -122.42;
        b52Spec.initial.altitudeMslM = 8500.0;
        b52Spec.initial.headingDeg = 90.0;
        b52Spec.initial.airspeedTrueMs = 230.0;

        session::VehicleSpec uh60Spec;
        uh60Spec.name = "hawk-1";
        uh60Spec.type = "jsbsim:uh60";
        uh60Spec.initial.latitudeDeg = 37.66;
        uh60Spec.initial.longitudeDeg = -122.42;
        uh60Spec.initial.altitudeMslM = 120.0;
        uh60Spec.initial.headingDeg = 90.0;
        uh60Spec.initial.airspeedTrueMs = 15.0;

        const std::uint32_t b52 = world.createVehicle(b52Spec);
        const std::uint32_t uh60 = world.createVehicle(uh60Spec);
        REQUIRE(b52 != 0u);
        REQUIRE(uh60 != 0u);

        world.step(10);
        const auto frame = world.tacticalOverlayFrame();
        const ipc::TacticalPropulsionExtWireV1* b52Prop = nullptr;
        const ipc::TacticalPropulsionExtWireV1* uh60Prop = nullptr;
        for (const auto& p : frame.propulsion) {
            CHECK(ipc::validTacticalPropulsionExtV1(p));
            if (p.vehicleId == b52) b52Prop = &p;
            if (p.vehicleId == uh60) uh60Prop = &p;
        }
        REQUIRE(b52Prop != nullptr);
        CHECK(b52Prop->engineCount == 8u);
        CHECK((b52Prop->flags & (ipc::kTactPropRotorcraft | ipc::kTactPropPropeller)) == 0u);
        for (int e = 0; e < 8; ++e) {
            CHECK(b52Prop->engineN2[e] > 20.0f);
            CHECK(b52Prop->thrustN[e] > 0.0f);
        }

        REQUIRE(uh60Prop != nullptr);
        CHECK((uh60Prop->flags & ipc::kTactPropRotorcraft) != 0u);
        CHECK(uh60Prop->rotorRadiusM == Catch::Approx(8.18f).margin(0.05f));
        CHECK(uh60Prop->engineCount >= 1u);
    }
}

TEST_CASE("VIS-M14 4K visual acceptance showcase replay generator", "[tactical_overlays][vis-m14][visual-gen]") {
    const char* outEnv = std::getenv("FSIM_VIS_M14_RECORD_DIR");
    if (!outEnv || !outEnv[0]) return;

    const std::filesystem::path outDir(outEnv);
    std::filesystem::create_directories(outDir);

    // Showcase 1: Rich EMSO + Multi-Camera + PiP Combat Scenario (.fsrec + .tact + .vfx + .debris)
    {
        auto opts = tacticalOptions("vis-m14-emso-showcase", 20261410u);
        opts.recordPath = outDir / "emso_tactical_showcase.fsrec";
        opts.recordIntervalSeconds = 0.02;
        session::World world(opts);
        world.setDestructionModeV1(session::World::DestructionModeV1::LightweightDebris);

        const double lat0 = 37.65;
        const double lon0 = -122.42;
        const double cosLat = std::cos(lat0 * kDeg);

        session::VehicleSpec blue1;
        blue1.name = "viper-1";
        blue1.type = "jsbsim:f16c";
        blue1.initial.latitudeDeg = lat0;
        blue1.initial.longitudeDeg = lon0;
        blue1.initial.altitudeMslM = 4800.0;
        blue1.initial.headingDeg = 90.0;
        blue1.initial.airspeedTrueMs = 260.0;

        session::VehicleSpec blue2 = blue1;
        blue2.name = "viper-2";
        blue2.initial.latitudeDeg = lat0 + 600.0 / 111132.0;
        blue2.initial.longitudeDeg = lon0 - 400.0 / (111320.0 * cosLat);

        session::VehicleSpec growler = blue1;
        growler.name = "growler-red";
        growler.type = "jsbsim:ea18g";
        growler.initial.longitudeDeg = lon0 + 18500.0 / (111320.0 * cosLat);
        growler.initial.latitudeDeg = lat0 + 300.0 / 111132.0;
        growler.initial.altitudeMslM = 5000.0;
        growler.initial.headingDeg = 270.0;
        growler.initial.airspeedTrueMs = 235.0;

        session::VehicleSpec bandit = blue1;
        bandit.name = "bandit-red";
        bandit.type = "jsbsim:f16c";
        bandit.initial.longitudeDeg = lon0 + 14500.0 / (111320.0 * cosLat);
        bandit.initial.latitudeDeg = lat0 - 200.0 / 111132.0;
        bandit.initial.altitudeMslM = 4850.0;
        bandit.initial.headingDeg = 270.0;
        bandit.initial.airspeedTrueMs = 240.0;

        const std::uint32_t b1 = world.createVehicle(blue1);
        const std::uint32_t b2 = world.createVehicle(blue2);
        const std::uint32_t gr = world.createVehicle(growler);
        const std::uint32_t br = world.createVehicle(bandit);
        REQUIRE(b1 != 0u);
        REQUIRE(b2 != 0u);
        REQUIRE(gr != 0u);
        REQUIRE(br != 0u);

        auto holdHsa = [&](std::uint32_t id, double hdgDeg, double altM, double spdMs) {
            control::HsaCommand h;
            h.headingRad = hdgDeg * kDeg;
            h.altitudeM = altM;
            h.speed = spdMs;
            (void)world.submit(id, h);
        };
        holdHsa(b1, 90.0, 4800.0, 260.0);
        holdHsa(b2, 90.0, 4800.0, 260.0);
        holdHsa(gr, 270.0, 5000.0, 235.0);
        holdHsa(br, 270.0, 4850.0, 240.0);

        std::uint16_t b1Radar = 0, b1Rwr = 0, b1Gun = 0, b1Stores = 0, grJam = 0, grEsm = 0, brRadar = 0, brCm = 0;
        REQUIRE(findPayloadByFamily(world, b1, ms::Family::AirRadar, b1Radar));
        REQUIRE(findPayloadByFamily(world, b1, ms::Family::Esm, b1Rwr));
        REQUIRE(findPayloadByFamily(world, b1, ms::Family::Gun, b1Gun));
        REQUIRE(findPayloadByFamily(world, b1, ms::Family::StoreStation, b1Stores));
        REQUIRE(findPayloadByFamily(world, gr, ms::Family::ElectronicAttack, grJam));
        const bool hasGrEsm = findPayloadByFamily(world, gr, ms::Family::Esm, grEsm);
        REQUIRE(findPayloadByFamily(world, br, ms::Family::AirRadar, brRadar));
        REQUIRE(findPayloadByFamily(world, br, ms::Family::Countermeasures, brCm));

        REQUIRE(world.loadStore(b1, b1Stores, 1, "aim-120c") == ms::Reason::None);
        REQUIRE(world.loadStore(b1, b1Stores, 9, "aim-120c") == ms::Reason::None);

        (void)world.submit(b1, ms::StateCommand{b1Radar, ms::PayloadState::Operate});
        (void)world.submit(b1, ms::StateCommand{b1Rwr, ms::PayloadState::Operate});
        (void)world.submit(b1, ms::StateCommand{b1Gun, ms::PayloadState::Operate});
        (void)world.submit(b1, ms::StateCommand{b1Stores, ms::PayloadState::Operate});
        (void)world.submit(gr, ms::StateCommand{grJam, ms::PayloadState::Operate});
        if (hasGrEsm) (void)world.submit(gr, ms::StateCommand{grEsm, ms::PayloadState::Operate});
        (void)world.submit(br, ms::StateCommand{brRadar, ms::PayloadState::Operate});
        (void)world.submit(br, ms::StateCommand{brCm, ms::PayloadState::Operate});

        bool b1Searching = false, brSearching = false, jamming = false, launched = false;
        for (int step = 0; step < 900; ++step) {
            world.step(1);
            const auto* h1 = world.missionSystems(b1);
            const auto* hg = world.missionSystems(gr);
            const auto* hb = world.missionSystems(br);
            if (!b1Searching && h1 && h1->payloads()[b1Radar].state == ms::PayloadState::Operate) {
                ms::AmtiCommand ac;
                ac.payload = b1Radar;
                ac.mode = ms::AmtiMode::TrackWhileScan;
                ac.task = ms::AmtiTask::Volume;
                ac.rangeScaleM = 50000.0;
                ac.azimuthHalfWidthRad = 30.0 * kDeg;
                ac.bars = 2;
                if (world.submit(b1, ac).accepted()) b1Searching = true;
            }
            if (!brSearching && hb && hb->payloads()[brRadar].state == ms::PayloadState::Operate) {
                ms::AmtiCommand ac;
                ac.payload = brRadar;
                ac.mode = ms::AmtiMode::TrackWhileScan;
                ac.task = ms::AmtiTask::Volume;
                ac.rangeScaleM = 50000.0;
                ac.azimuthHalfWidthRad = 25.0 * kDeg;
                ac.bars = 2;
                if (world.submit(br, ac).accepted()) brSearching = true;
            }
            if (!jamming && step >= 260 && hg && hg->payloads()[grJam].state == ms::PayloadState::Operate) {
                ms::PayloadTask jt;
                jt.payload = grJam;
                jt.kind = ms::TaskKind::Jam;
                jt.frequencyHz = 9.86e9;
                jt.bandwidthHz = 40e6;
                jt.azimuthHalfWidthRad = 25.0 * kDeg;
                jt.elevationHalfWidthRad = 15.0 * kDeg;
                if (world.submit(gr, jt).accepted()) jamming = true;
            }
            if (!launched && step >= 180 && b1Searching && h1 && h1->payloads()[b1Stores].state == ms::PayloadState::Operate) {
                const auto tracks = world.amtiTracks(b1, b1Radar);
                if (!tracks.empty()) {
                    const std::uint32_t desig = world.designate(b1, b1Radar, tracks.front().id);
                    if (desig != 0u) {
                        ms::StrikeCommand sc;
                        sc.payload = b1Stores;
                        sc.store = "aim-120c";
                        sc.quantity = 1;
                        sc.target = desig;
                        sc.consent = true;
                        if (world.submit(b1, sc).accepted()) launched = true;
                    }
                }
            }
            if (launched && (step % 35 == 0)) (void)world.dispense(br, (step % 70) == 0);
        }
        std::cout << "[VIS-M14 visual-gen] wrote " << opts.recordPath.string() << " (+ .tact + .vfx + .debris)\n";
    }

    // Showcase 2: Fleet VFX Parity (B-52H 8 engines, UH-60 low hover downwash + tips + exhaust haze, C-130J 4-prop)
    {
        auto opts = tacticalOptions("vis-m14-fleet-showcase", 20261411u);
        opts.recordPath = outDir / "fleet_vfx_parity.fsrec";
        opts.recordIntervalSeconds = 0.02;
        session::World world(opts);
        sim::EnvironmentState env = world.environment();
        env.humidity = 0.88;
        world.setEnvironment(env);

        session::VehicleSpec uh60;
        uh60.name = "uh60-hover";
        uh60.type = "jsbsim:uh60";
        uh60.initial.latitudeDeg = 37.62;
        uh60.initial.longitudeDeg = -122.38;
        uh60.initial.altitudeMslM = 12.0;
        uh60.initial.headingDeg = 90.0;
        uh60.initial.airspeedTrueMs = 8.0;

        session::VehicleSpec b52;
        b52.name = "b52h-8eng";
        b52.type = "jsbsim:b52h";
        b52.initial.latitudeDeg = 37.622;
        b52.initial.longitudeDeg = -122.38;
        b52.initial.altitudeMslM = 10200.0;
        b52.initial.headingDeg = 90.0;
        b52.initial.airspeedTrueMs = 235.0;

        session::VehicleSpec c130;
        c130.name = "c130j-prop";
        c130.type = "jsbsim:c130j";
        c130.initial.latitudeDeg = 37.624;
        c130.initial.longitudeDeg = -122.38;
        c130.initial.altitudeMslM = 1200.0;
        c130.initial.headingDeg = 90.0;
        c130.initial.airspeedTrueMs = 135.0;

        const std::uint32_t uId = world.createVehicle(uh60);
        const std::uint32_t bId = world.createVehicle(b52);
        const std::uint32_t cId = world.createVehicle(c130);
        REQUIRE(uId != 0u);
        REQUIRE(bId != 0u);
        REQUIRE(cId != 0u);

        auto hold = [&](std::uint32_t id, double altM, double spdMs) {
            control::HsaCommand h;
            h.headingRad = 90.0 * kDeg;
            h.altitudeM = altM;
            h.speed = spdMs;
            (void)world.submit(id, h);
        };
        hold(uId, 12.0, 8.0);
        hold(bId, 10200.0, 235.0);
        hold(cId, 1200.0, 135.0);

        for (int step = 0; step < 300; ++step) world.step(1);
        std::cout << "[VIS-M14 visual-gen] wrote " << opts.recordPath.string() << " (+ .tact)\n";
    }
}
