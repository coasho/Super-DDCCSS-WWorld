// SIM-M07 (MS-11): Remaining Mission Systems Families — Weather Radar, SAR,
// SMTI, PNT (INS & GNSS under RF Jamming), Comm Terminals + Mode 5 IFF, and Ground-Map Radar.

#include "fsim/Agent.h"
#include "fsim/World.h"
#include "simulation/mission_systems/Cards.h"
#include "simulation/mission_systems/Model.h"
#include "simulation/mission_systems/comm/CommTerminal.h"
#include "simulation/mission_systems/pnt/PntReceiver.h"
#include "simulation/mission_systems/sar/SarRadar.h"
#include "simulation/mission_systems/smti/SmtiRadar.h"
#include "simulation/mission_systems/weather/WeatherRadar.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <set>
#include <string>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr double kEarthM = 6371000.0;

fsim::WorldOptions makeOptions(const char* name, std::uint64_t seed = 81) {
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

TEST_CASE("SIM-M07 open-source literature alignment: Marshall-Palmer Z-R, Curlander-McDonough SAR NESZ, Skolnik MDV, Groves PNT, and ICAO IFF", "[sim-m07][alignment]") {
    using namespace fsim::ms;

    // 1) Marshall-Palmer (1948) Z = 200 * R^1.6 & ITU-R P.838 X-band rain attenuation
    CHECK(weather::marshallPalmerDbz(10.0) == Catch::Approx(10.0 * std::log10(200.0 * std::pow(10.0, 1.6))).margin(1e-6));
    CHECK(weather::marshallPalmerDbz(50.0) > 49.0);
    CHECK(std::isinf(weather::marshallPalmerDbz(0.0)));
    const double att10 = weather::rainSpecificAttenuationDbKm(10.0, 9.375e9);
    const double att50 = weather::rainSpecificAttenuationDbKm(50.0, 9.375e9);
    CHECK(att10 > 0.1);
    CHECK(att50 > att10 * 5.0);

    // 2) Curlander & McDonough SAR integration time and NESZ R^3 scaling + jamming degradation
    const double lambdaX = 299792458.0 / 9.6e9;
    const double t1 = sar::integrationTimeS(lambdaX, 50000.0, 200.0, 1.0, 0.5 * kPi);
    const double tHalf = sar::integrationTimeS(lambdaX, 50000.0, 200.0, 0.5, 0.5 * kPi);
    CHECK(t1 == Catch::Approx(lambdaX * 50000.0 / (2.0 * 200.0 * 1.0)).margin(1e-9));
    CHECK(tHalf == Catch::Approx(2.0 * t1).margin(1e-9));

    const double nesz50 = sar::neszDb(50000.0, 200.0, 60.0 * kDeg, 1500.0, 38.0, 9.6e9, 1.0, 4.0, 5.0, 0.0);
    const double nesz100 = sar::neszDb(100000.0, 200.0, 60.0 * kDeg, 1500.0, 38.0, 9.6e9, 1.0, 4.0, 5.0, 0.0);
    const double neszJam = sar::neszDb(50000.0, 200.0, 60.0 * kDeg, 1500.0, 38.0, 9.6e9, 1.0, 4.0, 5.0, 12.0);
    CHECK(nesz50 < -10.0);
    CHECK((nesz100 - nesz50) == Catch::Approx(30.0 * std::log10(2.0)).margin(1e-5)); // Exact R^3 (+9.03 dB)
    CHECK((neszJam - nesz50) == Catch::Approx(12.0).margin(1e-9));

    // 3) Skolnik/Richards SMTI MDV = (lambda / 2) * clutterSpreadHz
    CHECK(smti::minimumDetectableVelocityMs(9.5e9, 160.0) == Catch::Approx(0.5 * (299792458.0 / 9.5e9) * 160.0).margin(1e-9));

    // 4) Groves PNT: GNSS C/N0 under jamming & INS Schuler drift
    const double cn0Clean = pnt::effectiveGnssCn0DbHz(-160.0, 3.0, 0.0, 24.0);
    CHECK(cn0Clean == Catch::Approx(40.975).margin(0.05));
    const double cn0Jammed = pnt::effectiveGnssCn0DbHz(-160.0, 3.0, 1e-14, 24.0);
    CHECK(cn0Jammed < 10.0);
    CHECK(pnt::insHorizontalErrorM(1800.0, 0.412, 15.0) > pnt::insHorizontalErrorM(60.0, 0.412, 15.0));

    // 5) ICAO Annex 10 / STANAG 4193 Mode 5 IFF 1090 MHz link margin
    CHECK(comm_payload::iffLinkMarginDb(27.0, 130.0, 0.0, -80.0) == Catch::Approx(7.0).margin(1e-9));
}

TEST_CASE("SIM-M07 fleet coverage: all 18 MS-11 payload cards construct cleanly with honest IFF and GNSS discrimination", "[sim-m07][cards]") {
    std::vector<std::string> errors;
    std::vector<std::shared_ptr<const fsim::ms::Card>> cards;
    for (const auto& entry : std::filesystem::directory_iterator(std::filesystem::path(FSIM_TEST_PAYLOAD_DIR))) {
        if (!entry.is_regular_file() || entry.path().extension() != ".json") continue;
        if (auto c = fsim::ms::readCard(entry.path(), errors)) cards.push_back(std::move(c));
    }
    REQUIRE(errors.empty());

    const std::set<std::string> iffCards = {"mids-jtrs", "an-asq-242"};
    int ms11Count = 0;
    for (const auto& card : cards) {
        if (!card) continue;
        const auto fam = card->family;
        if (fam != fsim::ms::Family::WeatherRadar && fam != fsim::ms::Family::Sar &&
            fam != fsim::ms::Family::Smti && fam != fsim::ms::Family::Pnt &&
            fam != fsim::ms::Family::CommTerminal && fam != fsim::ms::Family::GroundMapRadar) {
            continue;
        }
        ++ms11Count;
        auto model = fsim::ms::makeModel(*card, 1);
        INFO("MS-11 card=" << card->id);
        REQUIRE(model != nullptr);
        std::vector<fsim::ms::PayloadValue> rep;
        model->report(rep);
        REQUIRE(!rep.empty());
        if (fam == fsim::ms::Family::CommTerminal) {
            const double expIff = iffCards.count(card->id) ? 1.0 : 0.0;
            CHECK(reportValue(rep, "iff_capable") == expIff);
            CHECK(reportValue(rep, "iff_mode5") == expIff);
        }
        if (fam == fsim::ms::Family::Pnt) {
            CHECK(reportValue(rep, "gnss_capable") == (card->id == "magr-2k" ? 1.0 : 0.0));
        }
    }
    CHECK(ms11Count == 18);
}

TEST_CASE("SIM-M07 weather radar: Marshall-Palmer precipitation detection, rain attenuation and X-band ESM visibility", "[sim-m07][weather]") {
    fsim::World world(makeOptions("sim_m07_weather", 83));
    auto c130 = createAircraft(world, "c130j", "c130j", 0.0, 0.0, 3000.0, 90.0, 140.0);
    auto f16 = createAircraft(world, "f16", "f16c", 35.0, 0.0, 3000.0, 270.0, 210.0);
    const auto wxIdx = findPayload(c130, "radar");
    const auto rwrIdx = findPayload(f16, "rwr");
    powerToOperate(world, c130, wxIdx);
    powerToOperate(world, f16, rwrIdx);

    fsim::ms::PayloadTask scan;
    scan.payload = wxIdx;
    scan.kind = fsim::ms::TaskKind::Search;
    scan.azimuthCentreRad = 0.0;
    scan.elevationCentreRad = 0.0;
    scan.azimuthHalfWidthRad = 45.0 * kDeg;
    scan.rangeScaleM = 120000.0;
    REQUIRE(c130.submit(scan).accepted());

    // Dry atmosphere -> zero weather contacts, yet X-band weather radar is detected by nearby RWR
    world.step(40);
    CHECK(c130.contacts(wxIdx).empty());
    bool rwrHeardWx = false;
    for (const auto& c : f16.contacts(rwrIdx)) {
        if (c.truth == c130.id()) rwrHeardWx = true;
    }
    CHECK(rwrHeardWx);

    // Heavy convective rain (precipitation = 0.90 -> ~40.5 mm/h) produces Weather contacts & attenuation
    fsim::EnvironmentState stormy = world.environment().state();
    stormy.cloudBaseM = 2000.0;
    stormy.cloudCover = 0.9;
    stormy.precipitation = 0.90;
    world.environment().set(stormy);
    world.step(45);

    const auto wxContacts = c130.contacts(wxIdx);
    REQUIRE(!wxContacts.empty());
    CHECK(wxContacts.front().kind == fsim::ms::ContactKind::Weather);
    CHECK(wxContacts.front().strengthDb > 30.0);
    CHECK(std::isfinite(wxContacts.front().rangeM));
    CHECK(wxContacts.front().sizeM >= 5000.0);
    const auto rep = c130.payloadReport(wxIdx);
    CHECK(reportValue(rep, "rain_rate") > 35.0);
    CHECK(reportValue(rep, "attenuation") > 1.0);
}

TEST_CASE("SIM-M07 SAR spotlight imaging: side-looking geometry enforcement, Curlander-McDonough NESZ and GoalReached image contact", "[sim-m07][sar]") {
    fsim::World world(makeOptions("sim_m07_sar", 85));
    // U-2S flying east at 18 km MSL; side-looking target 45 km south of track (90 deg broadside squint)
    auto u2 = createAircraft(world, "u2s", "u2s", 0.0, 0.0, 18000.0, 90.0, 200.0);
    const auto sarIdx = findPayload(u2, "sar");
    powerToOperate(world, u2, sarIdx);

    // 1) Reject dead-ahead target (squint ~ 0 deg) with OutOfReach on first collection step
    fsim::ms::PayloadTask ahead;
    ahead.payload = sarIdx;
    ahead.kind = fsim::ms::TaskKind::Image;
    ahead.latitudeRad = 35.0 * kDeg;
    ahead.longitudeRad = (-117.0 + 45.0 * 1e3 / (kEarthM * std::cos(35.0 * kDeg)) / kDeg) * kDeg;
    ahead.resolutionM = 1.0;
    const auto badAct = u2.submit(ahead);
    REQUIRE(badAct.accepted());
    world.step(5);
    const auto badStatus = u2.missionActivity(badAct.activity);
    REQUIRE(badStatus.has_value());
    CHECK(badStatus->state == fsim::control::ActivityState::Failed);
    CHECK(badStatus->reason == fsim::ms::Reason::OutOfReach);

    // 2) Valid broadside spotlight collection 45 km south at 1.0 m resolution (~3.7 s integration)
    fsim::ms::PayloadTask spot;
    spot.payload = sarIdx;
    spot.kind = fsim::ms::TaskKind::Image;
    spot.latitudeRad = (35.0 - 45.0 * 1e3 / kEarthM / kDeg) * kDeg;
    spot.longitudeRad = -117.0 * kDeg;
    spot.resolutionM = 1.0;
    const auto okAct = u2.submit(spot);
    REQUIRE(okAct.accepted());

    world.step(200); // ~6.7 s > ~3.7 s required aperture integration
    const auto okStatus = u2.missionActivity(okAct.activity);
    REQUIRE(okStatus.has_value());
    CHECK(okStatus->state == fsim::control::ActivityState::Completed);
    CHECK(okStatus->reason == fsim::ms::Reason::GoalReached);

    const auto imgs = u2.contacts(sarIdx);
    REQUIRE(imgs.size() == 1);
    CHECK(imgs.front().kind == fsim::ms::ContactKind::Image);
    CHECK(imgs.front().sizeM == Catch::Approx(1.0));
    CHECK(imgs.front().strengthDb < -5.0); // Sensible negative NESZ dB
    CHECK(imgs.front().rangeM > 40000.0);
    CHECK(reportValue(u2.payloadReport(sarIdx), "images") == 1.0);
}

TEST_CASE("SIM-M07 PNT (INS vs SAASM GPS under RF jamming), Comm/Mode-5 IFF, and Ground-Map Radar", "[sim-m07][pnt][comm][ground_map]") {
    fsim::World world(makeOptions("sim_m07_pnt_comm", 89));
    auto kc46 = createAircraft(world, "kc46", "kc46a", 0.0, 0.0, 7000.0, 90.0, 200.0);
    auto f15 = createAircraft(world, "f15", "f15c", 2.0, 0.0, 7000.0, 90.0, 220.0);
    auto growler = createAircraft(world, "ea18g", "ea18g", 10.0, 0.0, 7000.0, 270.0, 220.0);
    auto f35 = createAircraft(world, "f35", "f35a", 5.0, 5.0, 7000.0, 90.0, 220.0);
    auto b52 = createAircraft(world, "b52", "b52h", 8.0, -5.0, 7000.0, 90.0, 200.0);
    auto compass = createAircraft(world, "ec130", "ec130h", 12.0, -5.0, 7000.0, 270.0, 140.0);

    const auto gpsIdx = findPayload(kc46, "gps");
    const auto insIdx = findPayload(f15, "ins");
    const auto jamIdx = findPayload(growler, "jammers");
    const auto cniIdx = findPayload(f35, "cni");
    const auto uhfIdx = findPayload(b52, "radio");
    const auto gmapIdx = findPayload(b52, "radar");
    const auto ccIdx = findPayload(compass, "ea");

    powerToOperate(world, kc46, gpsIdx);
    powerToOperate(world, f15, insIdx);
    powerToOperate(world, growler, jamIdx);
    powerToOperate(world, f35, cniIdx);
    powerToOperate(world, b52, uhfIdx);
    powerToOperate(world, b52, gmapIdx);
    powerToOperate(world, compass, ccIdx);

    // 1) Unjammed baseline
    world.step(40);
    const auto gpsClean = kc46.payloadReport(gpsIdx);
    const auto insClean = f15.payloadReport(insIdx);
    CHECK(reportValue(gpsClean, "gnss_locked") == 1.0);
    CHECK(reportValue(gpsClean, "jammed") == 0.0);
    CHECK(reportValue(gpsClean, "pos_error_h") < 10.0);
    CHECK(reportValue(insClean, "gnss_capable") == 0.0);
    CHECK(reportValue(insClean, "jammed") == 0.0);

    // F-35A AN/ASQ-242 CNI reports Mode 5 IFF capable & detects in-range cooperative aircraft
    const auto cniRep = f35.payloadReport(cniIdx);
    CHECK(reportValue(cniRep, "link_available") == 1.0);
    CHECK(reportValue(cniRep, "iff_capable") == 1.0);
    CHECK(reportValue(cniRep, "iff_mode5") == 1.0);
    CHECK(reportValue(cniRep, "iff_in_range") >= 4.0);

    // B-52H AN/APQ-166 ground-map radar produces Terrain contacts & clearance report
    const auto gmapContacts = b52.contacts(gmapIdx);
    REQUIRE(!gmapContacts.empty());
    CHECK(gmapContacts.front().kind == fsim::ms::ContactKind::Terrain);
    CHECK(reportValue(b52.payloadReport(gmapIdx), "min_clearance") > 5000.0);

    // 2) Growler directionally jams GPS L1 + L2 (1.4 GHz center, 450 MHz bandwidth, +14 dBi beam);
    //    Compass Call jams B-52 UHF (300 MHz).
    fsim::ms::PayloadTask gpsJam;
    gpsJam.payload = jamIdx;
    gpsJam.kind = fsim::ms::TaskKind::Jam;
    gpsJam.frequencyHz = 1.40e9;
    gpsJam.bandwidthHz = 450.0e6;
    gpsJam.azimuthCentreRad = 0.0;
    gpsJam.elevationCentreRad = 0.0;
    gpsJam.azimuthHalfWidthRad = 35.0 * kDeg;
    gpsJam.elevationHalfWidthRad = 20.0 * kDeg;
    const auto gpsJamAct = growler.submit(gpsJam);
    REQUIRE(gpsJamAct.accepted());

    fsim::ms::PayloadTask uhfJam;
    uhfJam.payload = ccIdx;
    uhfJam.kind = fsim::ms::TaskKind::Jam;
    uhfJam.frequencyHz = 300.0e6;
    uhfJam.bandwidthHz = 40.0e6;
    uhfJam.azimuthHalfWidthRad = kPi;
    uhfJam.elevationHalfWidthRad = 0.5 * kPi;
    REQUIRE(compass.submit(uhfJam).accepted());

    world.step(120); // 4 s of jamming
    const auto gpsDenied = kc46.payloadReport(gpsIdx);
    const auto insDuring = f15.payloadReport(insIdx);
    CHECK(reportValue(gpsDenied, "jammed") == 1.0);
    CHECK(reportValue(gpsDenied, "gnss_locked") == 0.0);
    CHECK(reportValue(gpsDenied, "pos_error_h") > reportValue(gpsClean, "pos_error_h") + 1.0);
    // Pure INS remains completely immune to RF GPS jamming
    CHECK(reportValue(insDuring, "jammed") == 0.0);
    // B-52H AN/ARC-210 UHF radio is jammed, while F-35A CNI (969 MHz-15 GHz) stays unaffected by 300 MHz jammer
    CHECK(reportValue(b52.payloadReport(uhfIdx), "jammed") == 1.0);
    CHECK(reportValue(b52.payloadReport(uhfIdx), "link_available") == 0.0);
    CHECK(reportValue(f35.payloadReport(cniIdx), "jammed") == 0.0);

    // 3) Stop GPS jammer -> MAGR 2K cleanly re-acquires GPS lock
    REQUIRE(growler.cancelMission(gpsJamAct.activity).status == fsim::control::CommandStatus::Canceled);
    world.step(15);
    const auto gpsRecovered = kc46.payloadReport(gpsIdx);
    CHECK(reportValue(gpsRecovered, "jammed") == 0.0);
    CHECK(reportValue(gpsRecovered, "gnss_locked") == 1.0);
}

TEST_CASE("Phase 2 Comm/IFF: DMS CommTerminal processing-gain coupling, failed-radio dropout and two-way Mode 5 IFF jamming", "[sim-m07][comm][iff][dms]") {
    fsim::World world(makeOptions("phase2_comm_iff", 181));
    // Two F-35As with AN/ASQ-242 CNI (processing_gain = 24 dB, Mode 5 IFF), a third F-35A with CNI Off,
    // two B-52Hs with AN/ARC-210 UHF (processing_gain = 6 dB), and an EA-18G jammer.
    auto f35Tx = createAircraft(world, "f35_tx", "f35a", 0.0, 0.0, 7000.0, 90.0, 220.0);
    auto f35Rx = createAircraft(world, "f35_rx", "f35a", 25.0, 0.0, 7000.0, 270.0, 220.0);
    auto f35Silent = createAircraft(world, "f35_silent", "f35a", 12.0, 4.0, 7000.0, 90.0, 220.0);
    auto b52Tx = createAircraft(world, "b52_tx", "b52h", 0.0, -5.0, 7000.0, 90.0, 200.0);
    auto b52Rx = createAircraft(world, "b52_rx", "b52h", 25.0, -5.0, 7000.0, 270.0, 200.0);
    auto growler = createAircraft(world, "ea18g", "ea18g", 12.0, 40.0, 7000.0, 180.0, 220.0);

    const auto cniTxIdx = findPayload(f35Tx, "cni");
    const auto cniRxIdx = findPayload(f35Rx, "cni");
    const auto cniSilentIdx = findPayload(f35Silent, "cni");
    const auto uhfTxIdx = findPayload(b52Tx, "radio");
    const auto uhfRxIdx = findPayload(b52Rx, "radio");
    const auto jamIdx = findPayload(growler, "jammers");

    powerToOperate(world, f35Tx, cniTxIdx);
    powerToOperate(world, f35Rx, cniRxIdx);
    powerToOperate(world, b52Tx, uhfTxIdx);
    powerToOperate(world, b52Rx, uhfRxIdx);
    powerToOperate(world, growler, jamIdx);

    // 1) Cooperative Mode 5 IFF transponder gating: f35Silent (CNI Off) must NOT be counted until powered on.
    world.step(20);
    const double iffBeforePower = reportValue(f35Tx.payloadReport(cniTxIdx), "iff_in_range");
    // Expected: f35Rx (CNI Operate) + b52Tx + b52Rx + growler = 4 (NOT f35Silent whose IFF terminal is Off)
    CHECK(iffBeforePower == Catch::Approx(4.0));

    powerToOperate(world, f35Silent, cniSilentIdx);
    world.step(20);
    const double iffAfterPower = reportValue(f35Tx.payloadReport(cniTxIdx), "iff_in_range");
    CHECK(iffAfterPower == Catch::Approx(5.0));

    // 2) Two-way 1030/1090 MHz Mode 5 IFF jamming suppresses IFF replies
    fsim::ms::PayloadTask iffJam;
    iffJam.payload = jamIdx;
    iffJam.kind = fsim::ms::TaskKind::Jam;
    iffJam.frequencyHz = 1.060e9; // covers both 1030 MHz interrogation and 1090 MHz reply
    iffJam.bandwidthHz = 80.0e6;
    iffJam.azimuthCentreRad = 0.0;
    iffJam.elevationCentreRad = 0.0;
    iffJam.azimuthHalfWidthRad = 45.0 * kDeg;
    iffJam.elevationHalfWidthRad = 25.0 * kDeg;
    const auto iffJamAct = growler.submit(iffJam);
    REQUIRE(iffJamAct.accepted());
    world.step(25);
    CHECK(reportValue(f35Tx.payloadReport(cniTxIdx), "iff_jammed") == 1.0);
    CHECK(reportValue(f35Tx.payloadReport(cniTxIdx), "iff_in_range") < iffAfterPower);
    REQUIRE(growler.cancelMission(iffJamAct.activity).status == fsim::control::CommandStatus::Canceled);
    world.step(15);
    CHECK(reportValue(f35Tx.payloadReport(cniTxIdx), "iff_in_range") == Catch::Approx(5.0));

    // 3) Real CommTerminal processing-gain coupling on DMS:
    //    At 40 km jammer standoff (same geometry for both nets), AN/ASQ-242 (24 dB AJ) survives
    //    a wideband 1.0 GHz barrage that easily jams AN/ARC-210 (6 dB AJ) at 300 MHz.
    fsim::comm::NetSpec link16Spec;
    link16Spec.name = "link16_f35";
    link16Spec.frequencyHz = 1.0e9;
    link16Spec.rangeM = 200e3;
    link16Spec.latencyS = 0.05;
    link16Spec.jitterS = 0.0;
    const auto link16Net = world.createNet(link16Spec);
    REQUIRE(world.joinNet(f35Tx.id(), link16Net));
    REQUIRE(world.joinNet(f35Rx.id(), link16Net));

    fsim::comm::NetSpec uhfSpec;
    uhfSpec.name = "uhf_b52";
    uhfSpec.frequencyHz = 300.0e6;
    uhfSpec.rangeM = 200e3;
    uhfSpec.latencyS = 0.05;
    uhfSpec.jitterS = 0.0;
    const auto uhfNet = world.createNet(uhfSpec);
    REQUIRE(world.joinNet(b52Tx.id(), uhfNet));
    REQUIRE(world.joinNet(b52Rx.id(), uhfNet));

    auto f35TxAg = world.createAgent(f35Tx, "f35-tx");
    auto f35RxAg = world.createAgent(f35Rx, "f35-rx");
    auto b52TxAg = world.createAgent(b52Tx, "b52-tx");
    auto b52RxAg = world.createAgent(b52Rx, "b52-rx");
    fsim::comm::DmsMessage msg;
    msg.topic = fsim::comm::topic::kFreeForm + 1;
    msg.bytes = {'O', 'K'};

    // Wideband 1.0 GHz barrage (200 MHz BW) at 40 km -> J/S ~ +13 dB: below AN/ASQ-242's 16 dB margin -> delivers!
    fsim::ms::PayloadTask l16Barrage = iffJam;
    l16Barrage.frequencyHz = 1.0e9;
    l16Barrage.bandwidthHz = 200.0e6;
    const auto l16Act = growler.submit(l16Barrage);
    REQUIRE(l16Act.accepted());
    world.step(5);
    REQUIRE(f35TxAg.send(msg) != 0);
    world.step(15);
    CHECK(f35RxAg.receive().size() == 1u);

    // Narrowband 1.0 GHz spot jam (10 MHz BW) -> +13 dB higher PSD -> exceeds AN/ASQ-242 margin -> drops!
    l16Barrage.bandwidthHz = 10.0e6;
    REQUIRE(growler.submit(l16Barrage).accepted());
    world.step(5);
    REQUIRE(f35TxAg.send(msg) != 0);
    world.step(15);
    CHECK(f35RxAg.receive().empty());

    // Focused 300 MHz UHF sub-band jam (10 MHz BW) against B-52 AN/ARC-210 (6 dB AJ, 25 W, 25 kHz ch) -> jammed!
    l16Barrage.frequencyHz = 300.0e6;
    l16Barrage.bandwidthHz = 10.0e6;
    REQUIRE(growler.submit(l16Barrage).accepted());
    world.step(5);
    REQUIRE(b52TxAg.send(msg) != 0);
    world.step(15);
    CHECK(b52RxAg.receive().empty());

    // Stop jamming, then fail F-35 Rx CNI -> failed terminal blocks DMS link even with zero jamming
    REQUIRE(growler.submit(fsim::ms::StateCommand{jamIdx, fsim::ms::PayloadState::Off}).accepted());
    world.step(10);
    REQUIRE(f35TxAg.send(msg) != 0);
    world.step(15);
    CHECK(f35RxAg.receive().size() == 1u);

    REQUIRE(f35Rx.failPayload(cniRxIdx, true) == fsim::ms::Reason::None);
    world.step(5);
    REQUIRE(f35TxAg.send(msg) != 0);
    world.step(15);
    CHECK(f35RxAg.receive().empty());
}

TEST_CASE("Phase 2 PNT: PntReceiver GNSS denial/INS drift couple to sensedState and GPS jamming denies weapon strapdown GNSS", "[sim-m07][pnt][weapon-strapdown]") {
    fsim::World world(makeOptions("phase2_pnt_coupling", 183));
    auto kc46 = createAircraft(world, "kc46", "kc46a", 0.0, 0.0, 7000.0, 90.0, 200.0);
    auto f15 = createAircraft(world, "f15", "f15c", 2.0, 0.0, 7000.0, 90.0, 220.0);
    auto f16 = createAircraft(world, "f16", "f16c", 0.0, -2.0, 7000.0, 90.0, 260.0);
    auto growler = createAircraft(world, "ea18g", "ea18g", 10.0, 0.0, 7000.0, 270.0, 220.0);

    const auto gpsIdx = findPayload(kc46, "gps");
    const auto insIdx = findPayload(f15, "ins");
    const auto jamIdx = findPayload(growler, "jammers");
    powerToOperate(world, kc46, gpsIdx);
    powerToOperate(world, f15, insIdx);
    powerToOperate(world, growler, jamIdx);

    // 1) Clean baseline: MAGR 2K has valid GNSS and small nominal error; AN/ASN-109 has Schuler INS drift.
    world.step(30);
    CHECK(kc46.sensed().gnssValid);
    CHECK(kc46.sensed().positionErrorM > 1.0);
    CHECK(kc46.sensed().positionErrorM < 10.0);
    CHECK(f15.sensed().gnssValid);
    CHECK(f15.sensed().positionErrorM > 15.0);

    // Verify truth pool state is NOT mutated by sensed PNT drift.
    const double insDiffM = std::hypot(
        f15.sensed().state.positionEcef[0] - f15.state().positionEcef[0],
        f15.sensed().state.positionEcef[1] - f15.state().positionEcef[1],
        f15.sensed().state.positionEcef[2] - f15.state().positionEcef[2]);
    CHECK(insDiffM > 10.0);
    CHECK(std::fabs(f15.state().altitudeMslM - 7000.0) < 200.0);

    // 2) Launch a GNSS-aided AIM-120D from F-16C and verify live L1/L2 GPS jamming denies BOTH KC-46 and missile GNSS
    REQUIRE(f16.loadLoadout("air superiority AIM-120D") == fsim::ms::Reason::None);
    const auto stIdx = findPayload(f16, "stations");
    powerToOperate(world, f16, stIdx);
    fsim::ms::StrikeCommand strike;
    strike.payload = stIdx;
    strike.store = "aim-120d";
    strike.quantity = 1;
    strike.target = 0;
    strike.consent = true;
    const auto stAct = f16.submit(strike);
    REQUIRE(stAct.accepted());
    std::uint32_t missileId = 0;
    for (int i = 0; i < 300; ++i) {
        world.step(1);
        const auto status = f16.strikeStatus(stAct.activity);
        if (status && !status->weapons.empty()) {
            missileId = status->weapons.front();
            break;
        }
    }
    REQUIRE(missileId != 0);
    auto missile = world.vehicle(missileId);
    REQUIRE(missile.valid());
    CHECK(missile.sensed().gnssValid);

    // Growler jams GPS L1 + L2 (1.40 GHz, 450 MHz BW)
    fsim::ms::PayloadTask gpsJam;
    gpsJam.payload = jamIdx;
    gpsJam.kind = fsim::ms::TaskKind::Jam;
    gpsJam.frequencyHz = 1.40e9;
    gpsJam.bandwidthHz = 450.0e6;
    gpsJam.azimuthCentreRad = 0.0;
    gpsJam.elevationCentreRad = 0.0;
    gpsJam.azimuthHalfWidthRad = 45.0 * kDeg;
    gpsJam.elevationHalfWidthRad = 25.0 * kDeg;
    const auto gpsJamAct = growler.submit(gpsJam);
    REQUIRE(gpsJamAct.accepted());
    world.step(90); // 3 s of L1/L2 denial

    // KC-46 MAGR 2K loses GNSS lock -> sensed().gnssValid becomes false and positionErrorM grows
    CHECK(!kc46.sensed().gnssValid);
    CHECK(kc46.sensed().positionErrorM > 5.0);
    const double kcDiffM = std::hypot(
        kc46.sensed().state.positionEcef[0] - kc46.state().positionEcef[0],
        kc46.sensed().state.positionEcef[1] - kc46.state().positionEcef[1],
        kc46.sensed().state.positionEcef[2] - kc46.state().positionEcef[2]);
    CHECK(kcDiffM > 3.5);
    // Pure INS on F-15C remains completely immune to RF GPS jamming
    CHECK(f15.sensed().gnssValid);
    // In-flight AIM-120D loses GNSS aiding under live L1/L2 RF jamming
    REQUIRE(missile.valid());
    CHECK(!missile.sensed().gnssValid);

    // Stop GPS jammer -> both KC-46 and AIM-120D cleanly recover GNSS lock
    REQUIRE(growler.cancelMission(gpsJamAct.activity).status == fsim::control::CommandStatus::Canceled);
    world.step(15);
    CHECK(kc46.sensed().gnssValid);
    CHECK(missile.sensed().gnssValid);
}

TEST_CASE("Phase 3 Radars: GroundMap 2D/shadowing/jamming, SMTI RCS/noise/emission, and Weather 2D spatial cells", "[sim-m07][ground_map][smti][weather]") {
    using namespace fsim::ms;

    // 1) GroundMapRadar: 2D multi-azimuth scan, grazing-angle sigma0 (not constant -12 dB), ridge LOS shadowing & X-band jamming
    {
        struct RidgeGround final : fsim::GroundProvider {
            double heightAboveEllipsoidM(double /*latRad*/, double lonRad) const override {
                // Place a 2500 m steep ridge between 10 km and 14 km east of (35 deg N, -117 deg E)
                const double eastM = (lonRad - (-117.0 * kDeg)) * kEarthM * std::cos(35.0 * kDeg);
                if (eastM > 10000.0 && eastM < 14000.0) return 2500.0;
                return 200.0;
            }
            double pointHeightAboveEllipsoidM(double latRad, double lonRad) const override {
                return heightAboveEllipsoidM(latRad, lonRad);
            }
        };
        auto opt = makeOptions("phase3_gmap_ridge", 211);
        opt.ground = std::make_shared<RidgeGround>();
        fsim::World world(opt);
        auto b52 = createAircraft(world, "b52", "b52h", 0.0, 0.0, 2600.0, 90.0, 200.0);
        auto growler = createAircraft(world, "ea18g", "ea18g", 18.0, 0.0, 5000.0, 270.0, 220.0);
        const auto gmapIdx = findPayload(b52, "radar");
        const auto jamIdx = findPayload(growler, "jammers");
        powerToOperate(world, b52, gmapIdx);
        powerToOperate(world, growler, jamIdx);

        world.step(35);
        const auto cleanContacts = b52.contacts(gmapIdx);
        REQUIRE(cleanContacts.size() >= 10u);
        double minAz = 1e9, maxAz = -1e9;
        double minSig = 1e9, maxSig = -1e9;
        for (const auto& c : cleanContacts) {
            minAz = std::min(minAz, c.azimuthRad);
            maxAz = std::max(maxAz, c.azimuthRad);
            minSig = std::min(minSig, c.strengthDb);
            maxSig = std::max(maxSig, c.strengthDb);
        }
        // True 2D azimuth sweep (not a single 0-rad centerline) and varying grazing-angle sigma0 (not flat -12 dB)
        CHECK((maxAz - minAz) > 40.0 * kDeg);
        CHECK((maxSig - minSig) > 2.0);
        // Steep 2500 m ridge casts a genuine radar shadow behind it
        CHECK(reportValue(b52.payloadReport(gmapIdx), "shadowed_cells") >= 1.0);

        // X-band jamming degrades GroundMapRadar SNR and suppresses contacts
        PayloadTask xJam;
        xJam.payload = jamIdx;
        xJam.kind = TaskKind::Jam;
        xJam.frequencyHz = 9.4e9;
        xJam.bandwidthHz = 40.0e6;
        xJam.azimuthCentreRad = 0.0;
        xJam.elevationCentreRad = 0.0;
        xJam.azimuthHalfWidthRad = 30.0 * kDeg;
        xJam.elevationHalfWidthRad = 20.0 * kDeg;
        REQUIRE(growler.submit(xJam).accepted());
        world.step(40);
        CHECK(reportValue(b52.payloadReport(gmapIdx), "jam_degradation") > 15.0);
        CHECK(b52.contacts(gmapIdx).size() < cleanContacts.size());
    }

    // 2) SmtiRadar: real RCS/clutter/Swerling detection, noisy measurements (not exact truth), and RWR-detectable emission
    {
        struct SmtiScene final : Scene {
            fsim::sim::VehicleState mover{};
            std::vector<Emission> ems;
            fsim::sim::EnvironmentState env{};
            double sigmaVal = 25.0;
            std::size_t targets(std::uint32_t* ids, const fsim::sim::VehicleState** states, std::size_t max) const override {
                if (max == 0) return 0;
                ids[0] = 42;
                states[0] = &mover;
                return 1;
            }
            double sigma(std::uint32_t, double, double, double& fluct) const override {
                fluct = 1.0; // steady vehicle scatterer
                return sigmaVal;
            }
            double pathLossDb(std::uint32_t, double freqHz) const override {
                return 20.0 * std::log10(4.0 * kPi * 25000.0 * freqHz / 299792458.0);
            }
            double surfaceHeightM(double, double) const override { return 100.0; }
            const std::vector<Emission>& emissions() const override { return ems; }
            const fsim::sim::EnvironmentState& environment() const override { return env; }
            bool masked(std::uint32_t, double) const override { return false; }
        };

        Card card;
        card.id = "smti-test";
        card.family = Family::Smti;
        auto model = smti::makeSmtiRadar(card, 1);
        REQUIRE(model != nullptr);

        PayloadTask search;
        search.kind = TaskKind::Search;
        search.azimuthCentreRad = 0.0;
        search.elevationCentreRad = 0.0;
        search.azimuthHalfWidthRad = 45.0 * kDeg;
        search.elevationHalfWidthRad = 30.0 * kDeg;
        search.rangeScaleM = 120000.0;
        REQUIRE(model->begin(search, 0.0) == Reason::None);

        // Active SMTI must radiate X-band search emission visible to adversary RWRs
        Emission em{};
        REQUIRE(model->emission(em));
        CHECK(em.mode == EmitterMode::Search);
        CHECK(em.frequencyHz == Catch::Approx(9.5e9));
        CHECK(em.powerW > 100.0);

        fsim::sim::VehicleState own{};
        own.latitudeRad = 35.0 * kDeg;
        own.longitudeRad = -117.0 * kDeg;
        own.altitudeMslM = 8000.0;
        own.rotationBodyToEcef[0] = 1.0;
        own.rotationBodyToEcef[4] = 1.0;
        own.rotationBodyToEcef[8] = 1.0;
        own.positionEcef[0] = 6378137.0 + 8000.0;

        SmtiScene sc;
        sc.mover = own;
        sc.mover.onGround = true;
        sc.mover.altitudeMslM = 100.0;
        sc.mover.altitudeAglM = 0.0;
        sc.mover.positionEcef[0] = own.positionEcef[0] + 24000.0;
        sc.mover.positionEcef[2] = own.positionEcef[2] - 7900.0;
        sc.mover.velocityBodyMs[0] = 20.0; // > MDV (2.5 m/s)

        fsim::Rng rng(9001);
        model->step(1.0, own, sc, rng);
        std::vector<Contact> dets;
        model->contacts(dets);
        REQUIRE(dets.size() == 1u);
        // Physical SINR (NOT hardcoded 18.0 dB) and noisy range (NOT exact truth range)
        CHECK(dets.front().strengthDb != Catch::Approx(18.0).margin(1e-3));
        CHECK(dets.front().strengthDb > 10.0);
        const double trueR = std::hypot(24000.0, 7900.0);
        CHECK(std::fabs(dets.front().rangeM - trueR) > 1e-6);
        CHECK(std::fabs(dets.front().rangeM - trueR) < 100.0);
    }

    // 3) WeatherRadar: 2D multi-azimuth spatial convective cells and X-band RF jamming degradation
    {
        fsim::World world(makeOptions("phase3_weather_2d", 217));
        auto c130 = createAircraft(world, "c130j", "c130j", 0.0, 0.0, 3000.0, 90.0, 140.0);
        auto growler = createAircraft(world, "ea18g", "ea18g", 25.0, 0.0, 3000.0, 270.0, 220.0);
        const auto wxIdx = findPayload(c130, "radar");
        const auto jamIdx = findPayload(growler, "jammers");
        powerToOperate(world, c130, wxIdx);
        powerToOperate(world, growler, jamIdx);

        fsim::EnvironmentState stormy = world.environment().state();
        stormy.cloudBaseM = 2000.0;
        stormy.cloudCover = 0.9;
        stormy.precipitation = 0.85;
        world.environment().set(stormy);

        PayloadTask scan;
        scan.payload = wxIdx;
        scan.kind = TaskKind::Search;
        scan.azimuthCentreRad = 0.0;
        scan.elevationCentreRad = 0.0;
        scan.azimuthHalfWidthRad = 45.0 * kDeg;
        scan.rangeScaleM = 120000.0;
        REQUIRE(c130.submit(scan).accepted());
        world.step(40);

        const auto cleanWx = c130.contacts(wxIdx);
        REQUIRE(cleanWx.size() >= 8u);
        std::set<int> distinctAzDeg;
        double minDbz = 1e9, maxDbz = -1e9;
        for (const auto& c : cleanWx) {
            distinctAzDeg.insert(static_cast<int>(std::lround(c.azimuthRad / kDeg)));
            minDbz = std::min(minDbz, c.strengthDb);
            maxDbz = std::max(maxDbz, c.strengthDb);
        }
        // Multiple distinct azimuth rays across the 2D scan sector and heterogeneous convective cell reflectivity
        CHECK(distinctAzDeg.size() >= 3u);
        CHECK((maxDbz - minDbz) > 4.0);

        // X-band noise jammer raises weather radar MDS and suppresses weaker/distant rain cells
        PayloadTask xJam;
        xJam.payload = jamIdx;
        xJam.kind = TaskKind::Jam;
        xJam.frequencyHz = 9.375e9;
        xJam.bandwidthHz = 40.0e6;
        xJam.azimuthCentreRad = 0.0;
        xJam.elevationCentreRad = 0.0;
        xJam.azimuthHalfWidthRad = 30.0 * kDeg;
        xJam.elevationHalfWidthRad = 20.0 * kDeg;
        REQUIRE(growler.submit(xJam).accepted());
        world.step(40);
        CHECK(reportValue(c130.payloadReport(wxIdx), "jam_degradation") > 15.0);
        CHECK(c130.contacts(wxIdx).size() < cleanWx.size());
    }
}
