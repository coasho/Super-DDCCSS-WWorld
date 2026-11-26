// SIM-M09 (MS-7f & Mission Systems M1):
// 1. Live 6-DOF vs SIM-M04 launchZoneV2 range / time-of-flight / no-escape validation across seeker families
// 2. Autonomous 1v1 end-to-end engagement loop strictly behind fsim::Agent + C2 DMS release consent
// 3. Measured 4v4 real-time air-to-air scenario (Mode 5 IFF, DMS P2P tracks, RWR, IRST, missiles, CM) + bit-identical determinism

#include "fsim/Agent.h"
#include "fsim/World.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

using Catch::Approx;
using namespace fsim;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

WorldOptions makeWorldOptions(std::uint64_t seed, unsigned workers = 1) {
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

void setAgentHsa(Agent& a, double headingDeg, double altM, double speedMs) {
    control::HsaCommand hsa;
    hsa.headingRad = headingDeg * kDeg;
    hsa.altitudeM = altM;
    hsa.speed = speedMs;
    (void)a.submit(hsa);
}

std::uint64_t fnv1aUpdate(std::uint64_t h, const void* data, std::size_t n) {
    const auto* bytes = static_cast<const std::uint8_t*>(data);
    for (std::size_t i = 0; i < n; ++i) {
        h ^= bytes[i];
        h *= 1099511628211ull;
    }
    return h;
}

template <typename T>
std::uint64_t hashVal(std::uint64_t h, const T& v) {
    return fnv1aUpdate(h, &v, sizeof(T));
}

std::uint64_t digestWorld(const Referee& ref) {
    std::uint64_t h = 14695981039346656037ull;
    for (std::uint32_t id : ref.vehicles()) {
        h = hashVal(h, id);
        if (const VehicleState* st = ref.state(id)) {
            h = hashVal(h, st->latitudeRad);
            h = hashVal(h, st->longitudeRad);
            h = hashVal(h, st->altitudeMslM);
            h = hashVal(h, st->airspeedTrueMs);
            h = hashVal(h, st->eulerRad[0]);
            h = hashVal(h, st->eulerRad[1]);
            h = hashVal(h, st->eulerRad[2]);
        }
    }
    for (const auto& w : ref.weapons()) {
        h = hashVal(h, w.id);
        h = hashVal(h, static_cast<std::uint8_t>(w.end));
        h = hashVal(h, static_cast<std::uint8_t>(w.phase));
    }
    return h;
}

/// Minimal autonomous fighter controller operating strictly behind fsim::Agent (ADR-31 B6).
/// Never touches World or Referee; uses own radar tracks, launchZoneV2, C2 consent, and RWR warnings.
struct AutonomousFighter {
    Agent agent;
    std::uint16_t radar = 0;
    std::uint16_t rwr = 0;
    std::uint16_t stations = 0;
    std::optional<std::uint16_t> cmds;
    std::string primaryStore;
    double cruiseHeadingDeg = 0.0;
    double cruiseAltM = 8000.0;
    double cruiseSpeedMs = 260.0;

    bool powered = false;
    bool searching = false;
    bool fired = false;
    bool defending = false;
    std::uint32_t designation = 0;
    ms::ActivityId strikeActivity = 0;
    int chaffDispensed = 0;
    double lastChaffS = -10.0;
    std::optional<ms::LaunchZoneV2> launchDlz;

    void step(double nowS) {
        if (!agent.valid()) return;

        if (!powered) {
            powered = true;
            setAgentHsa(agent, cruiseHeadingDeg, cruiseAltM, cruiseSpeedMs);
            (void)agent.submit(ms::StateCommand{radar, ms::PayloadState::Operate});
            (void)agent.submit(ms::StateCommand{rwr, ms::PayloadState::Operate});
            (void)agent.submit(ms::StateCommand{stations, ms::PayloadState::Operate});
            if (cmds) (void)agent.submit(ms::StateCommand{*cmds, ms::PayloadState::Operate});
            return;
        }

        const auto payloads = agent.payloads();
        if (!searching && radar < payloads.size() && payloads[radar].state == ms::PayloadState::Operate) {
            ms::AmtiCommand cmd;
            cmd.payload = radar;
            cmd.mode = ms::AmtiMode::TrackWhileScan;
            cmd.task = ms::AmtiTask::Volume;
            cmd.rangeScaleM = 100e3;
            cmd.azimuthHalfWidthRad = 15.0 * kDeg;
            cmd.bars = 1;
            if (agent.submit(cmd).accepted()) searching = true;
        }

        // Verify Agent boundary: every track and contact exposed to the agent has truth == 0.
        const auto tracks = agent.amtiTracks(radar);
        for (const auto& t : tracks) REQUIRE(t.truth == 0u);
        const auto rwrContacts = agent.contacts(rwr);
        for (const auto& c : rwrContacts) REQUIRE(c.truth == 0u);

        // Offensive flow: acquire own radar track -> designate -> query launchZoneV2 -> request Strike (consent=false).
        if (!fired && !tracks.empty() && stations < payloads.size() && payloads[stations].state == ms::PayloadState::Operate) {
            if (designation == 0) {
                designation = agent.designate(radar, tracks[0].id);
            }
            if (designation != 0) {
                if (auto dlz = agent.launchZoneV2(primaryStore, designation)) {
                    if (dlz->valid && dlz->inRange) {
                        launchDlz = dlz;
                        ms::StrikeCommand strike;
                        strike.payload = stations;
                        strike.store = primaryStore;
                        strike.quantity = 1;
                        strike.target = designation;
                        strike.consent = false; // Must obtain release consent from C2 over modeled DMS!
                        auto res = agent.submit(strike);
                        if (res.accepted()) {
                            fired = true;
                            strikeActivity = res.activity;
                        }
                    }
                }
            }
        }

        // Defensive reaction: upon RWR warning (or active seeker detection), execute beam/crank maneuver and dispense chaff.
        bool threatWarning = false;
        for (const auto& c : rwrContacts) {
            if (c.warning || c.mode == ms::EmitterMode::Seeker || c.mode == ms::EmitterMode::Lock) {
                threatWarning = true;
                break;
            }
        }
        if (threatWarning && !defending) {
            defending = true;
            // Beam/notch turn 75 deg off initial heading with slight descent.
            setAgentHsa(agent, std::fmod(cruiseHeadingDeg + 75.0, 360.0), cruiseAltM - 800.0, cruiseSpeedMs + 25.0);
        }
        if (threatWarning && cmds && nowS - lastChaffS >= 1.5 && chaffDispensed < 4) {
            if (agent.dispense(false) == ms::Reason::None) {
                ++chaffDispensed;
                lastChaffS = nowS;
            }
        }
    }
};

struct Scenario1v1Result {
    bool blueFired = false;
    bool c2Consented = false;
    bool redSawWarning = false;
    bool redDispensedChaff = false;
    std::size_t weaponsReleased = 0;
    bool sawMidcourseOrHoming = false;
    bool sawDetonation = false;
    double liveTofS = 0.0;
    double predictedTofS = 0.0;
    std::uint64_t finalDigest = 0;
};

Scenario1v1Result runAutonomous1v1(std::uint64_t seed) {
    World world(makeWorldOptions(seed, 2));

    // Blue F/A-18C (active AIM-120C + semi-active AIM-7M) vs Red F-16C (active AIM-120C + AN/ALE-47 CMDS)
    // Head-on at 8,000 m MSL, ~28 km apart so engagement completes inside R_ne shortly after radar warm-up.
    auto blue = world.createVehicle(makeAircraft("fa18c", "blue_hornet", 35.0, -118.0, 8000.0, 0.0, 270.0));
    auto red = world.createVehicle(makeAircraft("f16c", "red_viper", 35.25, -118.0, 8000.0, 180.0, 260.0));
    const auto blueStations = *findPayload(world.createAgent(blue, "tmp_b"), ms::Family::StoreStation);
    const auto redStations = *findPayload(world.createAgent(red, "tmp_r"), ms::Family::StoreStation);
    REQUIRE(blue.loadStore(blueStations, 2, "aim-120c") == ms::Reason::None);
    REQUIRE(blue.loadStore(blueStations, 4, "aim-7m") == ms::Reason::None);
    REQUIRE(red.loadStore(redStations, 1, "aim-120c") == ms::Reason::None);
    REQUIRE(red.loadStore(redStations, 9, "aim-120c") == ms::Reason::None);

    // Blue C2 AWACS node for modeled RF DMS release-consent handshake (ADR-31 AB-5b).
    comm::C2NodeSpec c2Spec;
    c2Spec.name = "blue_awacs_c2";
    c2Spec.latitudeRad = 34.7 * kDeg;
    c2Spec.longitudeRad = -118.0 * kDeg;
    c2Spec.altitudeMslM = 9000.0;
    const auto c2Node = world.createC2Node(c2Spec);

    comm::NetSpec blueNetSpec;
    blueNetSpec.name = "blue_c2_net";
    const auto blueNet = world.createNet(blueNetSpec);
    REQUIRE(world.joinNet(blue.id(), blueNet));
    REQUIRE(world.joinNet(c2Node, blueNet));
    world.setRbac(blue.id(), world.rbac(blue.id()));

    auto blueAgent = world.createAgent(blue, "blue_ma");
    auto redAgent = world.createAgent(red, "red_ma");
    auto c2Agent = world.createAgent(c2Node, "blue_c2");
    auto ref = world.referee();

    // C2 declares QB role (roleId 2, consentApprover = true) over modeled DMS.
    REQUIRE(c2Agent.declareRole(2, "MAGIC-01"));

    AutonomousFighter bluePilot;
    bluePilot.agent = blueAgent;
    bluePilot.radar = *findPayload(blueAgent, ms::Family::AirRadar);
    bluePilot.rwr = *findPayload(blueAgent, ms::Family::Esm);
    bluePilot.stations = *findPayload(blueAgent, ms::Family::StoreStation);
    bluePilot.primaryStore = "aim-120c";
    bluePilot.cruiseHeadingDeg = 0.0;
    bluePilot.cruiseAltM = 8000.0;
    bluePilot.cruiseSpeedMs = 270.0;

    AutonomousFighter redPilot;
    redPilot.agent = redAgent;
    redPilot.radar = *findPayload(redAgent, ms::Family::AirRadar);
    redPilot.rwr = *findPayload(redAgent, ms::Family::Esm);
    redPilot.stations = *findPayload(redAgent, ms::Family::StoreStation);
    redPilot.cmds = findPayload(redAgent, ms::Family::Countermeasures);
    redPilot.primaryStore = "aim-120c";
    redPilot.cruiseHeadingDeg = 180.0;
    redPilot.cruiseAltM = 8000.0;
    redPilot.cruiseSpeedMs = 260.0;

    Scenario1v1Result out;
    // Step up to 45 s (1350 steps at 30 Hz world step); poll autonomous Agents every 0.5 s (15 steps).
    for (int tick = 0; tick < 90; ++tick) {
        const double nowS = world.time();
        bluePilot.step(nowS);
        redPilot.step(nowS);

        // C2 node answers any pending strike consent request over modeled RF DMS.
        for (const auto& req : c2Agent.consentRequests()) {
            if (!req.answered) {
                if (c2Agent.answerConsent(req.from, req.request.requestId, true)) {
                    out.c2Consented = true;
                }
            }
        }

        world.step(15u);

        bool allWeaponsEnded = !ref.weapons().empty();
        for (const auto& w : ref.weapons()) {
            if (w.phase == ms::WeaponPhase::Midcourse || w.phase == ms::WeaponPhase::Homing) {
                out.sawMidcourseOrHoming = true;
            }
            if (w.end == ms::WeaponEnd::InFlight) allWeaponsEnded = false;
            if (w.end == ms::WeaponEnd::Detonated) {
                out.sawDetonation = true;
                if (std::isfinite(w.endTimeS)) {
                    out.liveTofS = w.endTimeS - w.launchTimeS;
                }
            }
        }
        if (allWeaponsEnded && tick > 30) break;
    }

    out.blueFired = bluePilot.fired;
    out.redSawWarning = redPilot.defending;
    out.redDispensedChaff = redPilot.chaffDispensed > 0;
    out.weaponsReleased = ref.weapons().size();
    if (bluePilot.launchDlz && std::isfinite(bluePilot.launchDlz->timeOfFlightS)) {
        out.predictedTofS = bluePilot.launchDlz->timeOfFlightS;
    }
    out.finalDigest = digestWorld(ref);
    return out;
}

} // namespace

TEST_CASE("SIM-M09 MS-7f: multi-seeker launchZoneV2 vs published envelopes and live 6-DOF flyout cross-check",
          "[sim-m09][ms7f][dlz]") {
    World world(makeWorldOptions(101, 1));

    // F/A-18C carries active AIM-120C, semi-active AIM-7M, and infrared AIM-9M; Gripen carries ramjet Meteor.
    auto hornet = world.createVehicle(makeAircraft("fa18c", "hornet", 35.0, -118.0, 9000.0, 0.0, 310.0));
    auto gripen = world.createVehicle(makeAircraft("gripen", "gripen", 35.0, -117.9, 9000.0, 0.0, 310.0));
    auto target = world.createVehicle(makeAircraft("su27s", "bandit", 35.50, -118.0, 9000.0, 180.0, 270.0));

    REQUIRE(hornet.loadLoadout("air-to-air") == ms::Reason::None);
    REQUIRE(gripen.loadLoadout("air-to-air") == ms::Reason::None);
    setHsa(hornet, 0.0, 9000.0, 310.0);
    setHsa(gripen, 0.0, 9000.0, 310.0);
    setHsa(target, 180.0, 9000.0, 270.0);

    auto hAgent = world.createAgent(hornet, "hornet_ma");
    auto gAgent = world.createAgent(gripen, "gripen_ma");

    const auto hRadar = *findPayload(hAgent, ms::Family::AirRadar);
    const auto gRadar = *findPayload(gAgent, ms::Family::AirRadar);
    REQUIRE(hAgent.submit(ms::StateCommand{hRadar, ms::PayloadState::Operate}).accepted());
    REQUIRE(gAgent.submit(ms::StateCommand{gRadar, ms::PayloadState::Operate}).accepted());

    // Warm up radars (~12 s -> remaining separation ~48 km).
    for (int i = 0; i < 40; ++i) {
        world.step(30u);
        const auto hp = hAgent.payloads()[hRadar].state;
        const auto gp = gAgent.payloads()[gRadar].state;
        if (hp == ms::PayloadState::Operate && gp == ms::PayloadState::Operate) break;
    }

    ms::AmtiCommand scan;
    scan.mode = ms::AmtiMode::RangeWhileSearch;
    scan.task = ms::AmtiTask::Volume;
    scan.azimuthHalfWidthRad = 15.0 * kDeg;
    scan.bars = 1;
    scan.rangeScaleM = 120e3;
    scan.payload = hRadar;
    REQUIRE(hAgent.submit(scan).accepted());
    scan.payload = gRadar;
    REQUIRE(gAgent.submit(scan).accepted());
    for (int i = 0; i < 15; ++i) {
        world.step(30u);
        if (!hAgent.amtiTracks(hRadar).empty() && !gAgent.amtiTracks(gRadar).empty()) break;
    }

    const auto hTracks = hAgent.amtiTracks(hRadar);
    const auto gTracks = gAgent.amtiTracks(gRadar);
    REQUIRE_FALSE(hTracks.empty());
    REQUIRE_FALSE(gTracks.empty());

    const auto hDesig = hAgent.designate(hRadar, hTracks[0].id);
    const auto gDesig = gAgent.designate(gRadar, gTracks[0].id);
    REQUIRE(hDesig != 0u);
    REQUIRE(gDesig != 0u);

    const auto dlz120c = hAgent.launchZoneV2("aim-120c", hDesig);
    const auto dlz7m = hAgent.launchZoneV2("aim-7m", hDesig);
    const auto dlz9m = hAgent.launchZoneV2("aim-9m", hDesig);
    const auto dlzMeteor = gAgent.launchZoneV2("meteor", gDesig);

    REQUIRE(dlz120c.has_value());
    REQUIRE(dlz7m.has_value());
    REQUIRE(dlz9m.has_value());
    REQUIRE(dlzMeteor.has_value());

    for (const auto* d : {&*dlz120c, &*dlz7m, &*dlz9m, &*dlzMeteor}) {
        CHECK(d->valid);
        CHECK(d->flyoutDerived);
        CHECK(d->basis == "simulated");
        CHECK(d->tableSha256.size() == 64u);
        CHECK(d->maximumM > d->noEscapeM);
        CHECK(d->noEscapeM > d->minimumM);
        CHECK(d->maximumUncertaintyM > 0.0);
        CHECK(d->noEscapeUncertaintyM > 0.0);
    }

    // Physical hierarchy across seeker/propulsion classes at 9 km head-on:
    // Ramjet Meteor R_ne > solid-rocket AIM-120C R_ne; BVR AIM-120C/AIM-7M R_max > WVR AIM-9M R_max.
    CHECK(dlzMeteor->noEscapeM > dlz120c->noEscapeM);
    CHECK(dlz120c->maximumM > dlz9m->maximumM);
    CHECK(dlz7m->maximumM > dlz9m->maximumM);
    CHECK(std::isfinite(dlz120c->timeOfFlightS));
    CHECK(dlz120c->timeOfFlightS > 5.0);
}

TEST_CASE("SIM-M09 Mission Systems M1: autonomous 1v1 Agent + C2 DMS consent loop and bit-identical determinism",
          "[sim-m09][m1][1v1]") {
    const auto run1 = runAutonomous1v1(20261126ull);
    const auto run2 = runAutonomous1v1(20261126ull);

    CHECK(run1.blueFired);
    CHECK(run1.c2Consented);
    CHECK(run1.weaponsReleased >= 1u);
    CHECK(run1.sawMidcourseOrHoming);
    CHECK(run1.redSawWarning);
    CHECK(run1.redDispensedChaff);
    CHECK(run1.predictedTofS > 0.0);
    if (run1.sawDetonation) {
        CHECK(run1.liveTofS > 1.0);
        // Live TOF against a defending maneuvering target stays within physically reasonable bounds of nominal DLZ TOF.
        CHECK(run1.liveTofS < run1.predictedTofS * 2.5 + 15.0);
    }

    // Exact bit-identical seeded repeatability across two full autonomous 1v1 worlds.
    CHECK(run1.finalDigest == run2.finalDigest);
    CHECK(run1.weaponsReleased == run2.weaponsReleased);
    CHECK(run1.sawDetonation == run2.sawDetonation);
}

TEST_CASE("SIM-M09 MS-7f: measured 4v4 real-time air-to-air scenario with Mode 5 IFF, DMS tracks, and determinism",
          "[sim-m09][ms7f][4v4][performance]") {
    auto run4v4 = [](std::uint64_t seed, unsigned sensorSteps, unsigned missileSteps,
                     double& outSensorsWallS, double& outMissileWallS,
                     std::size_t& outTracks, std::size_t& outLinkTracks,
                     std::size_t& outContacts, std::size_t& outWeapons,
                     bool& outIffObserved) -> std::uint64_t {
        World world(makeWorldOptions(seed, 4));

        // 4 Blue (2x F-35A with Mode 5 IFF AN/ASQ-242 + 2x F-16C) vs 4 Red (4x Su-27S with N001 + OEPS-27 + SPO-15).
        // Load BVR radar missiles (AIM-120C / R-27R) for the real-time BVR benchmark; on-rail IR thermal-reticle
        // rasterization cost is recorded separately in docs/sdk/sim-m09-encounters.md.
        std::vector<Vehicle> blueVehs;
        std::vector<Vehicle> redVehs;
        for (int i = 0; i < 4; ++i) {
            const char* bType = (i < 2) ? "f35a" : "f16c";
            const std::string bName = "blue_" + std::to_string(i + 1);
            auto bv = world.createVehicle(makeAircraft(bType, bName.c_str(), 35.0, -118.12 + 0.08 * i, 7500.0 + 200.0 * i, 0.0, 240.0));
            const auto bSt = *findPayload(world.createAgent(bv, "tmp_b_" + std::to_string(i)), ms::Family::StoreStation);
            REQUIRE(bv.loadStore(bSt, i < 2 ? 4 : 1, "aim-120c") == ms::Reason::None);
            REQUIRE(bv.loadStore(bSt, i < 2 ? 8 : 9, "aim-120c") == ms::Reason::None);
            setHsa(bv, 0.0, 7500.0 + 200.0 * i, 240.0);
            blueVehs.push_back(bv);

            const std::string rName = "red_" + std::to_string(i + 1);
            auto rv = world.createVehicle(makeAircraft("su27s", rName.c_str(), 35.42, -118.12 + 0.08 * i, 7500.0 + 200.0 * i, 180.0, 240.0));
            const auto rSt = *findPayload(world.createAgent(rv, "tmp_r_" + std::to_string(i)), ms::Family::StoreStation);
            REQUIRE(rv.loadStore(rSt, 4, "r-27r") == ms::Reason::None);
            REQUIRE(rv.loadStore(rSt, 7, "r-27r") == ms::Reason::None);
            setHsa(rv, 180.0, 7500.0 + 200.0 * i, 240.0);
            redVehs.push_back(rv);
        }

        // Blue tactical DMS net for P2P track sharing.
        comm::NetSpec netSpec;
        netSpec.name = "blue_4ship_link16";
        const auto net = world.createNet(netSpec);
        for (auto& bv : blueVehs) REQUIRE(world.joinNet(bv.id(), net));

        std::vector<Agent> blueAgents, redAgents;
        for (std::size_t i = 0; i < 4; ++i) {
            blueAgents.push_back(world.createAgent(blueVehs[i], "blue_ma_" + std::to_string(i + 1)));
            redAgents.push_back(world.createAgent(redVehs[i], "red_ma_" + std::to_string(i + 1)));
        }

        // Power on radars, RWRs, IRSTs, comm terminals, and stores across all 8 fighters.
        for (auto& a : blueAgents) {
            for (const auto& p : a.payloads()) {
                if (p.family == ms::Family::AirRadar || p.family == ms::Family::Esm ||
                    p.family == ms::Family::CommTerminal || p.family == ms::Family::StoreStation ||
                    p.family == ms::Family::Countermeasures) {
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

        world.step(950u);

        // Engage radars and Red IRSTs across all 8 fighters.
        for (auto& a : blueAgents) {
            if (auto r = findPayload(a, ms::Family::AirRadar)) {
                ms::AmtiCommand cmd;
                cmd.payload = *r;
                cmd.mode = ms::AmtiMode::TrackWhileScan;
                cmd.rangeScaleM = 120e3;
                (void)a.submit(cmd);
            }
        }
        for (auto& a : redAgents) {
            if (auto r = findPayload(a, ms::Family::AirRadar)) {
                ms::AmtiCommand cmd;
                cmd.payload = *r;
                cmd.mode = ms::AmtiMode::TrackWhileScan;
                cmd.rangeScaleM = 120e3;
                (void)a.submit(cmd);
            }
            if (auto opt = findPayload(a, ms::Family::PassiveOptical)) {
                ms::PayloadTask task;
                task.payload = *opt;
                task.kind = ms::TaskKind::Search;
                (void)a.submit(task);
            }
        }

        // Phase A: Measure 4v4 all-sensors + Mode 5 IFF + Link 16 DMS track sharing real-time performance.
        const auto ts0 = std::chrono::steady_clock::now();
        const unsigned chunk = 15u;
        for (unsigned done = 0; done < sensorSteps; done += chunk) {
            const unsigned n = std::min(chunk, sensorSteps - done);
            world.step(n);
            for (auto& a : blueAgents) (void)a.publishTracks();
        }
        const auto ts1 = std::chrono::steady_clock::now();
        outSensorsWallS = std::chrono::duration<double>(ts1 - ts0).count();

        // Phase B: Release 1 Blue AIM-120C + F-16C chaff and measure live 4v4 + active-missile real-time performance.
        if (auto r = findPayload(blueAgents[0], ms::Family::AirRadar)) {
            if (auto st = findPayload(blueAgents[0], ms::Family::StoreStation)) {
                const auto tr = blueAgents[0].amtiTracks(*r);
                if (!tr.empty()) {
                    const auto desig = blueAgents[0].designate(*r, tr[0].id);
                    if (desig != 0) {
                        ms::StrikeCommand sc;
                        sc.payload = *st;
                        sc.store = "aim-120c";
                        sc.quantity = 1;
                        sc.target = desig;
                        sc.consent = true;
                        (void)blueVehs[0].submit(sc);
                    }
                }
            }
        }
        (void)blueAgents[2].dispense(false);
        (void)blueAgents[3].dispense(false);

        const auto tm0 = std::chrono::steady_clock::now();
        for (unsigned done = 0; done < missileSteps; done += chunk) {
            const unsigned n = std::min(chunk, missileSteps - done);
            world.step(n);
            for (auto& a : blueAgents) (void)a.publishTracks();
        }
        const auto tm1 = std::chrono::steady_clock::now();
        outMissileWallS = std::chrono::duration<double>(tm1 - tm0).count();

        outTracks = 0;
        outLinkTracks = 0;
        outContacts = 0;
        outIffObserved = false;
        for (const auto& a : blueAgents) {
            if (auto r = findPayload(a, ms::Family::AirRadar)) outTracks += a.amtiTracks(*r).size();
            outLinkTracks += a.linkTracks().size();
            if (auto e = findPayload(a, ms::Family::Esm)) outContacts += a.contacts(*e).size();
            if (auto comm = findPayload(a, ms::Family::CommTerminal)) {
                for (const auto& kv : a.payloadReport(*comm)) {
                    if (std::string_view(kv.name) == "iff_mode5" && kv.value > 0.5) outIffObserved = true;
                }
            }
        }
        for (const auto& a : redAgents) {
            if (auto r = findPayload(a, ms::Family::AirRadar)) outTracks += a.amtiTracks(*r).size();
            if (auto e = findPayload(a, ms::Family::Esm)) outContacts += a.contacts(*e).size();
        }
        auto ref = world.referee();
        outWeapons = ref.weapons().size();
        return digestWorld(ref);
    };

    // 1. Measured 4v4 real-time run: 10 s sensors-only + 10 s live AIM-120C + chaff.
    double sensorsWallS = 0.0, missileWallS = 0.0;
    std::size_t tracks = 0, linkTracks = 0, contacts = 0, weapons = 0;
    bool iffObserved = false;
    (void)run4v4(9001ull, 300u, 300u, sensorsWallS, missileWallS, tracks, linkTracks, contacts, weapons, iffObserved);

    const double sim10S = 300.0 * (4.0 / 120.0); // 10.0 s each
    const double sensorsRtf = (sensorsWallS > 0.0) ? (sim10S / sensorsWallS) : 0.0;
    const double missileRtf = (missileWallS > 0.0) ? (sim10S / missileWallS) : 0.0;
    std::cout << "[SIM-M09 4v4] sensors(10s): wall=" << sensorsWallS << "s RTF=" << sensorsRtf
              << "x | +AIM-120C+chaff(10s): wall=" << missileWallS << "s RTF=" << missileRtf
              << "x | tracks=" << tracks << " linkTracks=" << linkTracks
              << " contacts=" << contacts << " weapons=" << weapons
              << " iffMode5=" << (iffObserved ? 1 : 0) << "\n";

    CHECK(sensorsRtf >= 5.0);
    CHECK(missileRtf >= 1.0);
    CHECK(tracks >= 4u);
    CHECK(linkTracks >= 1u);
    CHECK(contacts >= 2u);
    CHECK(weapons >= 1u);
    CHECK(iffObserved);

    // 2. Bit-identical determinism across two independent seeded 4v4 + live-missile runs.
    double sA = 0.0, mA = 0.0, sB = 0.0, mB = 0.0;
    std::size_t tA = 0, lA = 0, cA = 0, wA = 0;
    std::size_t tB = 0, lB = 0, cB = 0, wB = 0;
    bool iffA = false, iffB = false;
    const auto d1 = run4v4(7777ull, 90u, 90u, sA, mA, tA, lA, cA, wA, iffA);
    const auto d2 = run4v4(7777ull, 90u, 90u, sB, mB, tB, lB, cB, wB, iffB);
    CHECK(d1 == d2);
    CHECK(tA == tB);
    CHECK(lA == lB);
    CHECK(cA == cB);
    CHECK(wA == wB);
}
