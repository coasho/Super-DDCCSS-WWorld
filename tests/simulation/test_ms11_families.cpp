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
