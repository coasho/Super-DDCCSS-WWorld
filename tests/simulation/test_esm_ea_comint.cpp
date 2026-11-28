// SIM-M05 (MS-9b): Electronic Attack (RF Noise Jamming), Enhanced ESM/DF Geolocation,
// COMINT Datalink Interception, Communications Jamming, and Programmed Countermeasure Salvos.

#include "fsim/Agent.h"
#include "fsim/World.h"
#include "simulation/mission_systems/Cards.h"
#include "simulation/mission_systems/ea/Dircm.h"
#include "simulation/mission_systems/ea/Jammer.h"
#include "simulation/mission_systems/esm/Receiver.h"
#include "simulation/mission_systems/components/implementations/seeker/RadioReceiverV1.h"
#include "simulation/mission_systems/radar/Signal.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr double kEarthM = 6371000.0;

fsim::WorldOptions makeOptions(const char* name, std::uint64_t seed = 42) {
    fsim::WorldOptions opt;
    opt.name = name;
    opt.jsbsimRoot = FSIM_TEST_JSBSIM_ROOT;
    opt.publish = false;
    opt.terrain = false;
    opt.dt = 1.0 / 120.0;
    opt.frameSkip = 4;
    opt.workers = 1;
    opt.seed = seed;
    return opt;
}

fsim::Vehicle createAircraft(fsim::World& world, const std::string& name, const std::string& type,
                             double eastKm, double northKm, double altM, double headingDeg, double speedMs) {
    fsim::VehicleSpec spec;
    spec.name = name;
    spec.type = "jsbsim:" + type;
    spec.initial.latitudeDeg = 35.0 + northKm * 1e3 / kEarthM / kDeg;
    spec.initial.longitudeDeg = -117.0 + eastKm * 1e3 / (kEarthM * std::cos(35.0 * kDeg)) / kDeg;
    spec.initial.altitudeMslM = altM;
    spec.initial.headingDeg = headingDeg;
    spec.initial.airspeedTrueMs = speedMs;
    return world.createVehicle(spec);
}

std::uint16_t findPayload(const fsim::Vehicle& v, const std::string& name) {
    for (const auto& p : v.payloads()) {
        if (name == p.name) return p.index;
    }
    FAIL("payload not found: " + name);
    return 0;
}

void powerToOperate(fsim::World& world, fsim::Vehicle& v, std::uint16_t payload) {
    const auto res = v.submit(fsim::ms::StateCommand{payload, fsim::ms::PayloadState::Operate});
    REQUIRE(res.accepted());
    for (int i = 0; i < 900; ++i) {
        if (v.payloads()[payload].state == fsim::ms::PayloadState::Operate) break;
        world.step(1);
    }
    REQUIRE(v.payloads()[payload].state == fsim::ms::PayloadState::Operate);
}

double reportValue(const std::vector<fsim::ms::PayloadValue>& values, const std::string& key) {
    for (const auto& kv : values) {
        if (kv.name && key == kv.name) return kv.value;
    }
    return std::numeric_limits<double>::quiet_NaN();
}

} // namespace

TEST_CASE("SIM-M05 EA unit physics: spectral overlap, J0 density and card family discrimination", "[sim-m05][ea]") {
    using namespace fsim::ms;

    // Spectral overlap bounds
    CHECK(ea::spectralOverlap(9.5e9, 40e6, 9.5e9, 2e6) == Catch::Approx(1.0).margin(1e-12));
    CHECK(ea::spectralOverlap(9.5e9, 10e6, 9.5e9, 20e6) == Catch::Approx(0.5).margin(1e-12));
    CHECK(ea::spectralOverlap(9.5e9, 20e6, 9.51e9, 20e6) == Catch::Approx(0.5).margin(1e-12));
    CHECK(ea::spectralOverlap(1.0e9, 20e6, 9.5e9, 2e6) == 0.0);
    CHECK(ea::spectralOverlap(9.5e9, 0.0, 9.5e9, 2e6) == 0.0);

    // Analytic J0 = (P * 10^((Gt + Gr - L)/10) / Bj) * overlap
    const double j0 = ea::receivedJammerDensityWHz(1000.0, 14.0, 30.0, 144.0, 40e6, 1.0);
    const double expected = 1000.0 * std::pow(10.0, (14.0 + 30.0 - 144.0) / 10.0) / 40e6;
    CHECK(j0 == Catch::Approx(expected).epsilon(1e-12));
    CHECK(ea::receivedJammerDensityWHz(1000.0, 14.0, 30.0, 144.0, 40e6, 0.0) == 0.0);

    // Optical/IR DIRCM must return null from makeJammer (driven strictly by an-aaq-24.json parameters)
    const std::filesystem::path payloadDir = std::filesystem::path(FSIM_TEST_AIRCRAFT_DIR).parent_path() / "assets" / "payloads";
    std::vector<std::string> cardErrors;
    const auto dircmCard = readCard(payloadDir / "an-aaq-24.json", cardErrors);
    const auto alq99Card = readCard(payloadDir / "an-alq-99.json", cardErrors);
    REQUIRE(cardErrors.empty());
    REQUIRE(dircmCard != nullptr);
    REQUIRE(alq99Card != nullptr);
    CHECK(ea::makeJammer(*dircmCard, 1) == nullptr);

    // RF jammer card succeeds, handles preemptive finite duration and rejects out-of-band frequency
    auto jammer = ea::makeJammer(*alq99Card, 3);
    REQUIRE(jammer != nullptr);

    PayloadTask badFreq;
    badFreq.kind = TaskKind::Jam;
    badFreq.frequencyHz = 50.0e9; // above 18 GHz band max
    CHECK(jammer->begin(badFreq, 0.0) == Reason::InvalidParameter);

    PayloadTask spot;
    spot.kind = TaskKind::Jam;
    spot.frequencyHz = 9.5e9;
    spot.bandwidthHz = 40e6;
    spot.durationS = 2.0;
    REQUIRE(jammer->begin(spot, 1.0) == Reason::None);

    Emission em;
    REQUIRE(jammer->emission(em));
    CHECK(em.mode == EmitterMode::Jammer);
    CHECK(em.frequencyHz == Catch::Approx(9.5e9));
    CHECK(em.bandwidthHz == Catch::Approx(40e6));
    CHECK(em.powerW == Catch::Approx(4500.0)); // 3 pods * 1500 W

    std::vector<PayloadValue> rep;
    jammer->report(rep);
    CHECK(reportValue(rep, "active") == 1.0);
    CHECK(reportValue(rep, "power") == Catch::Approx(4500.0));

    // Reactive contact-follow terminates cleanly with ContactLost when contact drops
    PayloadTask reactive;
    reactive.kind = TaskKind::Jam;
    reactive.contact = 12345;
    REQUIRE(jammer->begin(reactive, 2.0) == Reason::None);
    CHECK(jammer->follows() == 12345);
    Contact c;
    c.id = 12345;
    c.frequencyHz = 9.7e9;
    c.azimuthRad = 0.25;
    c.elevationRad = -0.05;
    jammer->followed(&c);
    REQUIRE(jammer->emission(em));
    CHECK(em.frequencyHz == Catch::Approx(9.7e9));
    CHECK(em.azimuthCentreRad == Catch::Approx(0.25));
    jammer->followed(nullptr);
    CHECK(jammer->finished() == Reason::ContactLost);
    CHECK(!jammer->emission(em));
}

TEST_CASE("SIM-M05 native radar jamming: mainlobe degradation, out-of-band immunity, burn-through and ESM strobe", "[sim-m05][ea][radar]") {
    auto runRadarScenario = [](double rangeKm, bool jamOn, double jamFreqHz, double jamBwHz, bool omni, bool signalLevel,
                               std::size_t& detCount, double& meanSnrDb, bool& rwrSawJammer) {
        fsim::World world(makeOptions("sim_m05_radar_jam", 77));
        auto victim = createAircraft(world, "f16", "f16c", 0.0, 0.0, 6000.0, 90.0, 220.0);
        auto growler = createAircraft(world, "ea18g", "ea18g", rangeKm, 0.0, 6000.0, 270.0, 220.0);
        const auto radarIdx = findPayload(victim, "radar");
        const auto rwrIdx = findPayload(victim, "rwr");
        const auto jamIdx = findPayload(growler, "jammers");

        powerToOperate(world, victim, radarIdx);
        powerToOperate(world, victim, rwrIdx);
        powerToOperate(world, growler, jamIdx);

        fsim::ms::AmtiCommand search;
        search.payload = radarIdx;
        search.mode = fsim::ms::AmtiMode::RangeWhileSearch;
        search.azimuthCentreRad = 0.0;
        search.azimuthHalfWidthRad = 15.0 * kDeg;
        search.elevationCentreRad = 0.0;
        search.bars = 2;
        search.rangeScaleM = 120e3;
        search.signalLevel = signalLevel;
        REQUIRE(victim.submit(search).accepted());

        if (jamOn) {
            fsim::ms::PayloadTask jam;
            jam.payload = jamIdx;
            jam.kind = fsim::ms::TaskKind::Jam;
            jam.frequencyHz = jamFreqHz;
            jam.bandwidthHz = jamBwHz;
            jam.azimuthCentreRad = 0.0;
            jam.elevationCentreRad = 0.0;
            jam.azimuthHalfWidthRad = omni ? kPi : (20.0 * kDeg);
            jam.elevationHalfWidthRad = omni ? (0.5 * kPi) : (15.0 * kDeg);
            const auto jr = growler.submit(jam);
            REQUIRE(jr.accepted());
            const auto rep = growler.payloadReport(jamIdx);
            CHECK(reportValue(rep, "active") == 1.0);
        }

        // Step 4 seconds of flight/sensing
        world.step(120);

        const auto dets = victim.amtiDetections(radarIdx);
        detCount = 0;
        double snrSum = 0.0;
        for (const auto& d : dets) {
            if (d.truth == growler.id()) {
                ++detCount;
                snrSum += d.snrDb;
            }
        }
        meanSnrDb = detCount > 0 ? (snrSum / static_cast<double>(detCount)) : -1e9;

        rwrSawJammer = false;
        for (const auto& c : victim.contacts(rwrIdx)) {
            if (c.truth == growler.id() && c.mode == fsim::ms::EmitterMode::Jammer) {
                rwrSawJammer = true;
            }
        }
    };

    // 1) Baseline 60 km unjammed vs in-band X-band (AN/APG-68(V)9 @ 9.86 GHz) mainlobe spot noise jamming
    std::size_t cleanDet = 0, jamDet = 0, oobDet = 0, barrageFarDet = 0, burnDet = 0;
    double cleanSnr = 0.0, jamSnr = 0.0, oobSnr = 0.0, barrageFarSnr = 0.0, burnSnr = 0.0;
    bool cleanRwr = false, jamRwr = false, oobRwr = false, barrageFarRwr = false, burnRwr = false;

    runRadarScenario(60.0, false, 9.86e9, 40e6, false, false, cleanDet, cleanSnr, cleanRwr);
    REQUIRE(cleanDet >= 2);
    CHECK(!cleanRwr);

    runRadarScenario(60.0, true, 9.86e9, 40e6, false, false, jamDet, jamSnr, jamRwr);
    CHECK((jamDet == 0 || jamSnr < cleanSnr - 10.0));

    // 2) Out-of-band jamming (1.0 GHz L-band) leaves X-band radar un-degraded
    runRadarScenario(60.0, true, 1.0e9, 40e6, false, false, oobDet, oobSnr, oobRwr);
    CHECK(oobDet == cleanDet);
    CHECK(oobSnr == Catch::Approx(cleanSnr).margin(0.5));

    // 3) Full-band omnidirectional barrage: suppressed at 60 km, burns through at short range (8 km) + RWR strobe
    const double qnan = std::numeric_limits<double>::quiet_NaN();
    runRadarScenario(60.0, true, qnan, qnan, true, false, barrageFarDet, barrageFarSnr, barrageFarRwr);
    CHECK((barrageFarDet == 0 || barrageFarSnr < cleanSnr - 10.0));
    runRadarScenario(8.0, true, qnan, qnan, true, false, burnDet, burnSnr, burnRwr);
    CHECK(burnDet >= 2);
    CHECK(burnRwr);

    // 4) Signal-level radar processing also sees jamming degradation
    std::size_t sigCleanDet = 0, sigJamDet = 0;
    double sigCleanSnr = 0.0, sigJamSnr = 0.0;
    bool sigCleanRwr = false, sigJamRwr = false;
    runRadarScenario(55.0, false, 9.86e9, 40e6, false, true, sigCleanDet, sigCleanSnr, sigCleanRwr);
    runRadarScenario(55.0, true, 9.86e9, 40e6, false, true, sigJamDet, sigJamSnr, sigJamRwr);
    REQUIRE(sigCleanDet >= 1);
    CHECK((sigJamDet == 0 || sigJamSnr < sigCleanSnr - 10.0));
}

TEST_CASE("SIM-M05 reactive ESM contact-following jamming and ContactLost termination", "[sim-m05][ea][esm]") {
    fsim::World world(makeOptions("sim_m05_reactive_jam", 88));
    auto growler = createAircraft(world, "ea18g", "ea18g", 0.0, 0.0, 6000.0, 90.0, 220.0);
    auto emitter = createAircraft(world, "f16", "f16c", 45.0, 0.0, 6000.0, 270.0, 220.0);

    const auto rxIdx = findPayload(growler, "receiver");
    const auto jamIdx = findPayload(growler, "jammers");
    const auto radarIdx = findPayload(emitter, "radar");

    powerToOperate(world, growler, rxIdx);
    powerToOperate(world, growler, jamIdx);
    powerToOperate(world, emitter, radarIdx);

    fsim::ms::AmtiCommand search;
    search.payload = radarIdx;
    search.azimuthHalfWidthRad = 20.0 * kDeg;
    search.rangeScaleM = 100e3;
    REQUIRE(emitter.submit(search).accepted());
    world.step(60); // 2 s

    const auto contacts = growler.contacts(rxIdx);
    REQUIRE(!contacts.empty());
    const auto targetContact = contacts.front();
    CHECK(targetContact.mode == fsim::ms::EmitterMode::Search);

    // Command reactive jamming following the ESM contact
    fsim::ms::PayloadTask reactive;
    reactive.payload = jamIdx;
    reactive.kind = fsim::ms::TaskKind::Jam;
    reactive.contact = targetContact.id;
    const auto res = growler.submit(reactive);
    REQUIRE(res.accepted());
    world.step(30); // 1 s

    const auto rep = growler.payloadReport(jamIdx);
    CHECK(reportValue(rep, "active") == 1.0);
    CHECK(reportValue(rep, "frequency") == Catch::Approx(targetContact.frequencyHz).margin(1e6));

    // Emitter turns radar off; once ESM contact ages out (> 5 s), reactive jammer ends with ContactLost
    REQUIRE(emitter.submit(fsim::ms::StateCommand{radarIdx, fsim::ms::PayloadState::Off}).accepted());
    world.step(240); // 8 s

    const auto act = growler.missionActivity(res.activity);
    REQUIRE(act.has_value());
    CHECK(act->state == fsim::control::ActivityState::Failed);
    CHECK(act->reason == fsim::ms::Reason::ContactLost);
    CHECK(reportValue(growler.payloadReport(jamIdx), "active") == 0.0);
}

TEST_CASE("SIM-M05 COMINT datalink interception and Compass Call comms jamming", "[sim-m05][comint][dms]") {
    fsim::World world(makeOptions("sim_m05_comint", 99));
    auto tx = createAircraft(world, "tx", "f16c", 0.0, 0.0, 6000.0, 90.0, 220.0);
    auto rx = createAircraft(world, "rx", "f16c", 45.0, 0.0, 6000.0, 270.0, 220.0);
    auto rivet = createAircraft(world, "rc135w", "rc135w", 20.0, 25.0, 8000.0, 180.0, 200.0);
    auto compass = createAircraft(world, "ec130h", "ec130h", 45.0, 35.0, 7000.0, 180.0, 140.0);

    const auto sigintIdx = findPayload(rivet, "sigint");
    const auto eaIdx = findPayload(compass, "ea");
    powerToOperate(world, rivet, sigintIdx);
    powerToOperate(world, compass, eaIdx);

    fsim::comm::NetSpec spec;
    spec.name = "link16";
    spec.frequencyHz = 1.0e9;
    spec.rangeM = 200e3;
    spec.latencyS = 0.05;
    spec.jitterS = 0.0;
    const auto net = world.createNet(spec);
    REQUIRE(world.joinNet(tx.id(), net));
    REQUIRE(world.joinNet(rx.id(), net));

    auto txAgent = world.createAgent(tx, "tx-agent");
    auto rxAgent = world.createAgent(rx, "rx-agent");
    auto ref = world.referee();
    REQUIRE(txAgent.valid());
    REQUIRE(rxAgent.valid());

    // Unjammed transmission: rx receives message, RC-135W intercepts Datalink carrier (no payload bytes)
    fsim::comm::DmsMessage msg;
    msg.topic = fsim::comm::topic::kFreeForm + 1;
    msg.bytes = {'S', 'E', 'C', 'R', 'E', 'T'};
    REQUIRE(txAgent.send(msg) != 0);
    world.step(15); // 0.5 s

    const auto received1 = rxAgent.receive();
    REQUIRE(received1.size() == 1);
    CHECK(received1[0].bytes == msg.bytes);

    bool sawDatalink = false;
    for (const auto& c : rivet.contacts(sigintIdx)) {
        if (c.mode == fsim::ms::EmitterMode::Datalink && c.truth == tx.id()) {
            sawDatalink = true;
            CHECK(c.frequencyHz == Catch::Approx(1.0e9).margin(1e6));
        }
    }
    CHECK(sawDatalink);

    // Compass Call jams Link-16 carrier (1.0 GHz, 20 MHz) pointed south toward rx
    fsim::ms::PayloadTask jamComms;
    jamComms.payload = eaIdx;
    jamComms.kind = fsim::ms::TaskKind::Jam;
    jamComms.frequencyHz = 1.0e9;
    jamComms.bandwidthHz = 20e6;
    jamComms.azimuthCentreRad = 0.0;
    jamComms.elevationCentreRad = 0.0;
    jamComms.azimuthHalfWidthRad = 35.0 * kDeg;
    jamComms.elevationHalfWidthRad = 20.0 * kDeg;
    const auto jamAct = compass.submit(jamComms);
    REQUIRE(jamAct.accepted());
    world.step(5);

    const auto lostBefore = ref.netStats(net).lost;
    REQUIRE(txAgent.send(msg) != 0);
    world.step(15);
    CHECK(rxAgent.receive().empty());
    CHECK(ref.netStats(net).lost > lostBefore);

    // Retune Compass Call out-of-band (3.5 GHz): Link-16 delivery resumes cleanly
    jamComms.frequencyHz = 3.5e9;
    REQUIRE(compass.submit(jamComms).accepted());
    world.step(5);
    REQUIRE(txAgent.send(msg) != 0);
    world.step(15);
    CHECK(rxAgent.receive().size() == 1);
}

TEST_CASE("SIM-M05 moving-platform passive bearing-only ESM triangulation geolocation", "[sim-m05][esm][geo]") {
    fsim::World world(makeOptions("sim_m05_esm_geo", 101));
    // RC-135W flies north-to-south at east=0, creating a baseline perpendicular to emitter at east=35 km
    auto rivet = createAircraft(world, "rc135w", "rc135w", 0.0, 10.0, 8000.0, 180.0, 210.0);
    auto emitter = createAircraft(world, "f16", "f16c", 35.0, 0.0, 6000.0, 270.0, 180.0);

    const auto sigintIdx = findPayload(rivet, "sigint");
    const auto radarIdx = findPayload(emitter, "radar");
    powerToOperate(world, rivet, sigintIdx);
    powerToOperate(world, emitter, radarIdx);

    fsim::ms::AmtiCommand search;
    search.payload = radarIdx;
    search.azimuthHalfWidthRad = 45.0 * kDeg;
    search.rangeScaleM = 120e3;
    REQUIRE(emitter.submit(search).accepted());

    // Initial look (~1 s): baseline too short -> range, lat/lon and CEP must remain NaN (no fabricated single-look range)
    world.step(30);
    auto initialContacts = rivet.contacts(sigintIdx);
    REQUIRE(!initialContacts.empty());
    CHECK(std::isnan(initialContacts.front().rangeM));
    CHECK(std::isnan(initialContacts.front().latitudeRad));
    CHECK(std::isnan(initialContacts.front().longitudeRad));
    CHECK(std::isnan(initialContacts.front().sizeM));

    // Fly ~30 s (~6.3 km cross-range baseline -> >10 deg parallax): geolocation converges with finite CRLB CEP50
    world.step(900);
    auto laterContacts = rivet.contacts(sigintIdx);
    REQUIRE(!laterContacts.empty());
    const auto& c = laterContacts.front();
    CHECK(std::isfinite(c.rangeM));
    CHECK(std::isfinite(c.latitudeRad));
    CHECK(std::isfinite(c.longitudeRad));
    CHECK(std::isfinite(c.sizeM));
    CHECK(c.sizeM > 10.0);
    CHECK(c.sizeM < 50000.0);
    CHECK(c.rangeM > 5000.0);
    CHECK(c.rangeM < 80000.0);
    const auto rep = rivet.payloadReport(sigintIdx);
    CHECK(reportValue(rep, "geolocated") >= 1.0);
    CHECK(std::isfinite(reportValue(rep, "geo_cep_min")));
}

TEST_CASE("SIM-M05 programmed countermeasure salvos and Agent permission boundaries", "[sim-m05][cm][agent]") {
    fsim::World world(makeOptions("sim_m05_salvo_agent", 105));
    auto f16 = createAircraft(world, "f16", "f16c", 0.0, 0.0, 5000.0, 90.0, 220.0);
    auto growler = createAircraft(world, "ea18g", "ea18g", 18.0, 0.0, 5000.0, 270.0, 220.0);

    const auto cmdsIdx = findPayload(f16, "cmds");
    const auto rwrIdx = findPayload(f16, "rwr");
    const auto jamIdx = findPayload(growler, "jammers");
    powerToOperate(world, f16, cmdsIdx);
    powerToOperate(world, f16, rwrIdx);
    powerToOperate(world, growler, jamIdx);

    auto f16Agent = world.createAgent(f16, "f16-agent");
    auto growlerAgent = world.createAgent(growler, "growler-agent");
    REQUIRE(f16Agent.valid());
    REQUIRE(growlerAgent.valid());

    // Agent starts jammer on own platform; another agent cannot cancel it
    fsim::ms::PayloadTask jam;
    jam.payload = jamIdx;
    jam.kind = fsim::ms::TaskKind::Jam;
    jam.frequencyHz = 9.5e9;
    jam.bandwidthHz = 40e6;
    const auto jamAct = growlerAgent.submit(jam);
    REQUIRE(jamAct.accepted());
    CHECK(!f16Agent.cancelMission(jamAct.activity).accepted());
    CHECK(reportValue(growlerAgent.payloadReport(jamIdx), "active") == 1.0);

    world.step(30);
    // Agent::contacts redacts truth to 0
    const auto agentContacts = f16Agent.contacts(rwrIdx);
    REQUIRE(!agentContacts.empty());
    CHECK(agentContacts.front().mode == fsim::ms::EmitterMode::Jammer);
    CHECK(agentContacts.front().truth == 0);

    // Programmed countermeasure salvo via Agent: 2 chaff + 1 flare x 2 salvos
    const auto beforeInv = f16Agent.countermeasures();
    fsim::ms::PayloadTask salvo;
    salvo.payload = cmdsIdx;
    salvo.kind = fsim::ms::TaskKind::Dispense;
    salvo.chaff = 2;
    salvo.flares = 1;
    salvo.salvos = 2;
    salvo.intervalS = 1.2;
    const auto salvoAct = f16Agent.submit(salvo);
    REQUIRE(salvoAct.accepted());

    world.step(150); // 5 s: plenty of time for 6 cartridges at 0.35 s spacing + 1.2 s salvo interval
    const auto afterInv = f16Agent.countermeasures();
    CHECK(beforeInv.chaff - afterInv.chaff == 4);
    CHECK(beforeInv.flare - afterInv.flare == 2);
    const auto act = f16Agent.missionActivity(salvoAct.activity);
    REQUIRE(act.has_value());
    CHECK(act->state == fsim::control::ActivityState::Completed);
    CHECK(act->reason == fsim::ms::Reason::GoalReached);
}

TEST_CASE("SIM-M05 open-source alignment: Skolnik/Adamy SSJ burn-through scaling vs native Radar::dwell", "[sim-m05][ea][alignment]") {
    using namespace fsim::ms;

    // 1) Closed-form Skolnik (Radar Handbook) / Adamy (EW 101) Self-Screening Jammer (SSJ) check:
    // J0 = P_j * G_jt * G_jr / (L_path * B_j) * overlap; SNR_deg = 10 log10(1 + J0/N0).
    constexpr double kLight = 299792458.0;
    constexpr double kBoltzmannT0 = 1.380649e-23 * 290.0; // W/Hz
    const double freqHz = 9.86e9;
    const double lambda = kLight / freqHz;
    const double pjW = 4500.0;
    const double gjtDbi = 3.0;
    const double gjrDbi = 30.0;
    const double bjHz = 17.936e9;
    const double nfLinear = std::pow(10.0, 4.0 / 10.0);
    const double n0 = kBoltzmannT0 * nfLinear;

    const double rFar = 20000.0;
    const double rNear = 10000.0;
    const double fsplFarDb = 20.0 * std::log10(4.0 * kPi * rFar / lambda);
    const double fsplNearDb = 20.0 * std::log10(4.0 * kPi * rNear / lambda);
    const double j0Far = ea::receivedJammerDensityWHz(pjW, gjtDbi, gjrDbi, fsplFarDb, bjHz, 1.0);
    const double j0Near = ea::receivedJammerDensityWHz(pjW, gjtDbi, gjrDbi, fsplNearDb, bjHz, 1.0);
    // One-way R^-2 jammer scaling: halving range quadruples J0 (+6.0206 dB).
    CHECK(10.0 * std::log10(j0Near / j0Far) == Catch::Approx(6.0205999).margin(1e-5));
    // Jam-dominated regime (J0 >> N0): SNR degradation increases by ~6.0 dB at half range,
    // while radar echo rises by +12.04 dB (R^-4), yielding exact +6.02 dB net S/J burn-through slope.
    const double degFarDb = 10.0 * std::log10(1.0 + j0Far / n0);
    const double degNearDb = 10.0 * std::log10(1.0 + j0Near / n0);
    CHECK(degFarDb > 20.0);
    CHECK((degNearDb - degFarDb) == Catch::Approx(6.02).margin(0.05));

    // 2) Live native World Radar::dwell check confirming R^-4 clean vs R^-2 SSJ burn-through slope:
    auto measureMeanSnr = [](double rangeKm, bool jamOn) -> double {
        fsim::World world(makeOptions("sim_m05_ssj_slope", 131));
        auto f16 = createAircraft(world, "f16", "f16c", 0.0, 0.0, 6000.0, 90.0, 220.0);
        auto growler = createAircraft(world, "ea18g", "ea18g", rangeKm, 0.0, 6000.0, 270.0, 220.0);
        const auto radarIdx = findPayload(f16, "radar");
        const auto jamIdx = findPayload(growler, "jammers");
        powerToOperate(world, f16, radarIdx);
        powerToOperate(world, growler, jamIdx);

        fsim::ms::AmtiCommand search;
        search.payload = radarIdx;
        search.mode = fsim::ms::AmtiMode::RangeWhileSearch;
        search.azimuthHalfWidthRad = 10.0 * kDeg;
        search.bars = 1;
        search.rangeScaleM = 80e3;
        REQUIRE(f16.submit(search).accepted());

        if (jamOn) {
            fsim::ms::PayloadTask jam;
            jam.payload = jamIdx;
            jam.kind = fsim::ms::TaskKind::Jam;
            jam.frequencyHz = std::numeric_limits<double>::quiet_NaN();
            jam.bandwidthHz = std::numeric_limits<double>::quiet_NaN();
            jam.azimuthHalfWidthRad = kPi;
            jam.elevationHalfWidthRad = 0.5 * kPi;
            REQUIRE(growler.submit(jam).accepted());
        }
        world.step(60); // 2 s
        double sum = 0.0;
        std::size_t n = 0;
        for (const auto& d : f16.amtiDetections(radarIdx)) {
            if (d.truth == growler.id()) {
                sum += d.snrDb;
                ++n;
            }
        }
        REQUIRE(n >= 1);
        return sum / static_cast<double>(n);
    };

    const double clean8 = measureMeanSnr(8.0, false);
    const double clean4 = measureMeanSnr(4.0, false);
    const double jam8 = measureMeanSnr(8.0, true);
    const double jam4 = measureMeanSnr(4.0, true);

    // Unjammed radar SNR rises by ~12 dB (R^-4); SSJ jammed SNR rises by ~6 dB (R^-2 burn-through).
    const double cleanGainDb = clean4 - clean8;
    const double jamGainDb = jam4 - jam8;
    CHECK(cleanGainDb > 9.0);
    CHECK(cleanGainDb < 16.0);
    CHECK(jamGainDb > 3.5);
    CHECK(jamGainDb < 9.5);
    CHECK((cleanGainDb - jamGainDb) == Catch::Approx(6.0).margin(2.0));
}

TEST_CASE("SIM-M05 open-source alignment: O'Donoughue Stansfield + Gauss-Newton ML triangulation and CRLB/CEP50", "[sim-m05][esm][geo][alignment]") {
    using namespace fsim::ms::esm;

    // Geometry on the equator (lat=0, lon=0, h=6 km):
    // Emitter at East=+40 km, North=0, Up=0 relative to equator point (R+6km, 0, 0).
    // Receiver samples along North-South baseline y = -6..+6 km (9 points, 1.5 km spacing).
    constexpr double kR = 6378137.0 + 6000.0;
    const double emitter[3] = {kR, 40000.0, 0.0}; // +Y is East at (lat=0, lon=0)

    auto buildSamples = [&](double sigmaRad, double rangeScale) {
        const double target[3] = {kR, 40000.0 * rangeScale, 0.0};
        std::vector<BearingSample> samples;
        for (int i = -4; i <= 4; ++i) {
            BearingSample s;
            s.originEcef[0] = kR;
            s.originEcef[1] = 0.0;
            s.originEcef[2] = static_cast<double>(i) * 1500.0 * rangeScale; // +Z is North at equator
            const double d[3] = {target[0] - s.originEcef[0],
                                 target[1] - s.originEcef[1],
                                 target[2] - s.originEcef[2]};
            const double r = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
            s.rayEcef[0] = d[0] / r;
            s.rayEcef[1] = d[1] / r;
            s.rayEcef[2] = d[2] / r;
            s.sigmaRad = sigmaRad;
            samples.push_back(s);
        }
        return samples;
    };

    const double sigma1Deg = 1.0 * kDeg;
    const auto s1 = buildSamples(sigma1Deg, 1.0);
    const auto r1 = triangulateBearings(s1.data(), s1.size(), s1.back().originEcef, s1.back().rayEcef);
    REQUIRE(r1.valid);
    const double posErrM = std::sqrt(
        std::pow(r1.positionEcef[0] - emitter[0], 2) +
        std::pow(r1.positionEcef[1] - emitter[1], 2) +
        std::pow(r1.positionEcef[2] - emitter[2], 2));
    // Exact noiseless recovery to sub-millimeter precision.
    CHECK(posErrM < 1e-3);
    REQUIRE(std::isfinite(r1.crlbRmsM));
    REQUIRE(std::isfinite(r1.crlbCep50M));
    CHECK(r1.crlbRmsM > 100.0);
    CHECK(r1.crlbCep50M > 100.0);

    // Exact CRLB proportionality to bearing noise sigma_theta (O'Donoughue Ch. 10):
    // doubling sigma_theta must double both CRLB RMS and CEP50 identically.
    const auto s2 = buildSamples(2.0 * sigma1Deg, 1.0);
    const auto r2 = triangulateBearings(s2.data(), s2.size(), s2.back().originEcef, s2.back().rayEcef);
    REQUIRE(r2.valid);
    CHECK(r2.crlbRmsM == Catch::Approx(2.0 * r1.crlbRmsM).epsilon(1e-6));
    CHECK(r2.crlbCep50M == Catch::Approx(2.0 * r1.crlbCep50M).epsilon(1e-6));

    // Exact GDOP range scaling at constant parallax angle:
    // doubling both range and baseline doubles CRLB RMS and CEP50.
    const auto sFar = buildSamples(sigma1Deg, 2.0);
    const auto rFar = triangulateBearings(sFar.data(), sFar.size(), sFar.back().originEcef, sFar.back().rayEcef);
    REQUIRE(rFar.valid);
    CHECK(rFar.crlbRmsM == Catch::Approx(2.0 * r1.crlbRmsM).epsilon(1e-4));
    CHECK(rFar.crlbCep50M == Catch::Approx(2.0 * r1.crlbCep50M).epsilon(1e-4));
}

TEST_CASE("Phase 1 EW: DRFM false targets and RGPO against airborne AMTI radar", "[sim-m05][ea][drfm]") {
    fsim::World world(makeOptions("phase1_drfm_radar", 141));
    auto victim = createAircraft(world, "f16", "f16c", 0.0, 0.0, 6000.0, 90.0, 220.0);
    auto growler = createAircraft(world, "ea18g", "ea18g", 35.0, 0.0, 6000.0, 270.0, 220.0);
    const auto radarIdx = findPayload(victim, "radar");
    const auto jamIdx = findPayload(growler, "jammers");
    powerToOperate(world, victim, radarIdx);
    powerToOperate(world, growler, jamIdx);

    fsim::ms::AmtiCommand search;
    search.payload = radarIdx;
    search.mode = fsim::ms::AmtiMode::RangeWhileSearch;
    search.azimuthHalfWidthRad = 15.0 * kDeg;
    search.bars = 1;
    search.rangeScaleM = 100e3;
    REQUIRE(victim.submit(search).accepted());

    // 1) Multi-false-target DRFM train at F-16C X-band (9.86 GHz)
    fsim::ms::EaDeceptionCommandV1 ft;
    ft.payload = jamIdx;
    ft.mode = fsim::ms::EaDeceptionModeV1::FalseTargets;
    ft.frequencyHz = 9.86e9;
    ft.bandwidthHz = 40e6;
    ft.falseTargetCount = 3;
    ft.rangeOffsetM = 2500.0;
    ft.azimuthHalfWidthRad = 25.0 * kDeg;
    ft.elevationHalfWidthRad = 20.0 * kDeg;
    REQUIRE(growler.submit(ft).accepted());
    CHECK(reportValue(growler.payloadReport(jamIdx), "deception_mode") == 4.0);
    CHECK(reportValue(growler.payloadReport(jamIdx), "false_targets") == 3.0);

    world.step(90); // 3 s
    std::vector<std::uint32_t> ghostIds;
    std::vector<double> ghostRanges;
    double realRangeSum = 0.0;
    std::size_t realCount = 0;
    for (const auto& d : victim.amtiDetections(radarIdx)) {
        if (d.truth == growler.id()) {
            realRangeSum += d.rangeM;
            ++realCount;
        } else if (d.truth >= fsim::ms::kDrfmFalseTargetIds) {
            ghostIds.push_back(d.truth);
            ghostRanges.push_back(d.rangeM);
        }
    }
    REQUIRE(realCount >= 1);
    REQUIRE(ghostIds.size() >= 3);
    std::sort(ghostIds.begin(), ghostIds.end());
    ghostIds.erase(std::unique(ghostIds.begin(), ghostIds.end()), ghostIds.end());
    CHECK(ghostIds.size() == 3u);
    const double meanRealRange = realRangeSum / static_cast<double>(realCount);
    const double maxGhostRange = *std::max_element(ghostRanges.begin(), ghostRanges.end());
    // Causal repeater delay: false targets appear strictly behind the jammer's real range.
    CHECK(maxGhostRange > meanRealRange + 4000.0);

    // 2) RGPO walks apparent range outward over time
    fsim::ms::EaDeceptionCommandV1 rgpo;
    rgpo.payload = jamIdx;
    rgpo.mode = fsim::ms::EaDeceptionModeV1::Rgpo;
    rgpo.frequencyHz = 9.86e9;
    rgpo.bandwidthHz = 40e6;
    rgpo.rangeOffsetM = 300.0;
    rgpo.rangePullRateMs = 500.0;
    rgpo.rangePullMaxM = 4000.0;
    rgpo.azimuthHalfWidthRad = 25.0 * kDeg;
    rgpo.elevationHalfWidthRad = 20.0 * kDeg;
    REQUIRE(growler.submit(rgpo).accepted());

    const auto serialStart = victim.radarStatus(radarIdx).detections;
    world.step(120); // 4 s (1 s hold + 3 s walk = +1500 m pull-off)
    double firstPullOff = -1.0, lastPullOff = -1.0;
    double lastRealRange = -1.0;
    for (const auto& d : victim.amtiDetections(radarIdx, serialStart)) {
        if (d.truth == growler.id()) {
            lastRealRange = d.rangeM;
        } else if (d.truth >= fsim::ms::kDrfmFalseTargetIds && lastRealRange > 0.0) {
            const double pull = d.rangeM - lastRealRange;
            if (firstPullOff < 0.0) firstPullOff = pull;
            lastPullOff = pull;
        }
    }
    CHECK(firstPullOff > 100.0);
    // RGPO walks the delayed repeater ghost progressively farther behind the real aircraft range.
    CHECK(lastPullOff > firstPullOff + 500.0);
}

TEST_CASE("Phase 1 EW: RadioReceiverV1 aperture-weighted jamming, passive monopulse HOJ and burn-through", "[sim-m05][ea][hoj]") {
    using namespace fsim::ms;
    using namespace fsim::ms::components;

    RadioReceiverConfigV1 cfg;
    cfg.version = 1;
    cfg.frequencyHz = 10.0e9;
    cfg.temperatureK = 290.0;
    cfg.noiseFigureDb = 6.0;
    cfg.systemLossDb = 6.0;
    cfg.transmitAveragePowerW = 1000.0;
    cfg.bandwidthHz = 1.0e6;
    cfg.monopulseSlope = 1.5;
    cfg.clutterRateSigmaMs = 2.0;
    cfg.associationGateRad = 0.1;
    cfg.temporalMemoryS = 0.12;
    cfg.observationWindowS = 0.012;
    cfg.aperture.shape = rf::ApertureShape::Circular;
    cfg.aperture.taper = rf::Taper::Uniform;
    cfg.aperture.widthM = 0.18;
    cfg.aperture.heightM = 0.18;
    cfg.aperture.efficiency = 0.6;
    cfg.aperture.peakGainDbi = 0.0;
    cfg.aperture.floorDbi = -30.0;
    cfg.aperture.backDbi = -40.0;
    cfg.processing.rangeScaleM = 20000.0;
    cfg.processing.minRateMs = -1000.0;
    cfg.processing.maxRateMs = 1000.0;
    cfg.processing.pfaInterval = 1e-6;
    cfg.processing.cfarCells = 16;
    cfg.processing.cfarGuard = 2;
    cfg.processing.coincidence = 1;
    cfg.processing.notchMs = 0.0;
    cfg.processing.knownNoiseThreshold = false;
    cfg.processing.reportAmbiguities = true;
    cfg.waveform = radar::signal::mediumPrf(cfg.frequencyHz, 0.012, 3, 3, 150.0, 0.05);

    RadioReceiverV1 rx;
    std::string err;
    REQUIRE(rx.configure(cfg, err));

    const std::array<weapon::V3, 3> axes{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    const weapon::V3 boresight{1, 0, 0};
    const weapon::V3 targetDir = measured::unit(weapon::V3{1.0, 0.015, -0.01});

    auto makeScene = [&](double timeS, double rangeM, double jamDensityWHz) {
        RadioSceneFrameV1 s;
        s.timeS = timeS;
        s.frequencyHz = cfg.frequencyHz;
        s.up = {0, 0, -1};
        RadioEchoSceneV1 echo;
        echo.direction = targetDir;
        echo.receiverRangeM = rangeM;
        echo.delayRangeM = rangeM;
        echo.dopplerHz = 2.0 * 450.0 / (299792458.0 / cfg.frequencyHz);
        echo.rcsM2 = 5.0;
        echo.fluctuation = 0;
        s.echoes.push_back(echo);
        s.refereeAttributionIndices.push_back(0u);
        if (jamDensityWHz > 0.0) {
            RadioJammerSceneV1 jam;
            jam.direction = targetDir;
            jam.rangeM = rangeM;
            jam.incidentDensityWHz = jamDensityWHz;
            jam.bandwidthHz = 40e6;
            jam.noiseJamming = true;
            s.jammers.push_back(jam);
            s.refereeJammerAttributionIndices.push_back(1u);
        }
        return s;
    };

    // 1) Clean unjammed exposure at 6 km -> coherent active detection with range & Doppler
    const auto clean = rx.expose(makeScene(0.1, 6000.0, 0.0), boresight, axes, 777, false);
    REQUIRE(clean.valid);
    CHECK(!clean.homeOnJam);
    CHECK(clean.totalInterferencePowerW == 0.0);
    REQUIRE(!clean.measurements.empty());
    CHECK(clean.measurements.front().rangeM.has_value());
    CHECK(clean.measurements.front().rangeRateMs.has_value());
    CHECK(!clean.measurements.front().homeOnJam);

    // 2) Strong mainlobe noise jammer at 6 km -> coherent echo drowned, passive monopulse HOJ engages
    const auto jammed = rx.expose(makeScene(0.2, 6000.0, 1e-12), boresight, axes, 777, false);
    REQUIRE(jammed.valid);
    CHECK(jammed.totalInterferencePowerW > 1e-6);
    CHECK(jammed.homeOnJam);
    REQUIRE(jammed.measurements.size() == 1u);
    CHECK(jammed.measurements.front().homeOnJam);
    CHECK(!jammed.measurements.front().rangeM.has_value());
    CHECK(!jammed.measurements.front().rangeRateMs.has_value());
    CHECK(jammed.measurements.front().refereeAttributionIndex == 1u);
    CHECK(measured::dot(jammed.measurements.front().direction, targetDir) > std::cos(2.0 * kDeg));

    // 3) Close range burn-through (1.8 km) against moderate jammer (J/N ~ +31 dB) -> active coherent detection resumes
    const auto burn = rx.expose(makeScene(0.3, 1800.0, 1e-19), boresight, axes, 777, false);
    REQUIRE(burn.valid);
    CHECK(burn.totalInterferencePowerW > 1e-12);
    CHECK(!burn.homeOnJam);
    REQUIRE(!burn.measurements.empty());
    CHECK(burn.measurements.front().rangeM.has_value());
    CHECK(!burn.measurements.front().homeOnJam);
}

TEST_CASE("Phase 1 EW: AN/AAQ-24 DIRCM laser turret physics and E-7A countermeasures card", "[sim-m05][ea][dircm]") {
    using namespace fsim::ms;

    // Preserve makeJammer(dircm) == nullptr while makeDircm / makeModel builds the optical laser model
    // strictly from an-aaq-24.json parameters (no C++ card.id lookup table).
    std::vector<std::string> dircmErrs;
    const auto dircmPath = std::filesystem::path(FSIM_TEST_AIRCRAFT_DIR).parent_path() / "assets" / "payloads" / "an-aaq-24.json";
    const auto dircm = readCard(dircmPath, dircmErrs);
    REQUIRE(dircmErrs.empty());
    REQUIRE(dircm != nullptr);
    CHECK(ea::makeJammer(*dircm, 1) == nullptr);
    auto laserModel = makeModel(*dircm, 2);
    REQUIRE(laserModel != nullptr);
    // DIRCM rejects explicit RF frequency commands.
    PayloadTask rfJam;
    rfJam.kind = TaskKind::Jam;
    rfJam.frequencyHz = 9.5e9;
    CHECK(laserModel->begin(rfJam, 0.0) == Reason::InvalidParameter);
    // DIRCM accepts optical sector/cued task and never produces RF emissions.
    PayloadTask sector;
    sector.kind = TaskKind::Jam;
    sector.azimuthCentreRad = 0.1;
    sector.elevationCentreRad = -0.1;
    REQUIRE(laserModel->begin(sector, 0.0) == Reason::None);
    Emission rfOut;
    CHECK(!laserModel->emission(rfOut));

    // Live World check: KC-46A with AN/AAQ-24 DIRCM tracks and lases an incoming IR missile,
    // and E-7A Wedgetail exposes calibrated 120 chaff / 60 flares from e7a-cm.json.
    fsim::World world(makeOptions("phase1_dircm_e7a", 151));
    auto tanker = createAircraft(world, "kc46", "kc46a", 0.0, 0.0, 7000.0, 90.0, 210.0);
    auto wedgetail = createAircraft(world, "e7a", "e7a", 15.0, 5.0, 8000.0, 270.0, 200.0);
    const auto cmInv = world.countermeasures(wedgetail.id());
    CHECK(cmInv.chaff == 120u);
    CHECK(cmInv.flare == 60u);

    std::uint16_t dircmIdx = 0;
    bool foundDircm = false;
    for (const auto& p : tanker.payloads()) {
        if (std::string(p.card) == "an-aaq-24") {
            dircmIdx = p.index;
            foundDircm = true;
            break;
        }
    }
    REQUIRE(foundDircm);
    powerToOperate(world, tanker, dircmIdx);
    world.step(10);
    CHECK(reportValue(tanker.payloadReport(dircmIdx), "active") == 1.0);
    CHECK(reportValue(tanker.payloadReport(dircmIdx), "lasing") == 0.0);

    // Command optical DIRCM sector and step past 0.35 s lock acquisition latency -> active MWIR laser
    PayloadTask dirTask;
    dirTask.payload = dircmIdx;
    dirTask.kind = TaskKind::Jam;
    dirTask.azimuthCentreRad = 0.0;
    dirTask.elevationCentreRad = -5.0 * kDeg;
    dirTask.durationS = 3.0;
    REQUIRE(tanker.submit(dirTask).accepted());
    world.step(30); // 1.0 s > 0.35 s lock time
    CHECK(reportValue(tanker.payloadReport(dircmIdx), "lasing") == 1.0);
    CHECK(reportValue(tanker.payloadReport(dircmIdx), "power") > 1.0);
}
