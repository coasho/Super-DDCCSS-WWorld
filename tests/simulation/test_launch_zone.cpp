// SIM-M04 (MS-8): Flyout-derived DLZ/WEZ unit, table-population, native 6-DOF
// cross-check, and Agent permission tests.

#include "fsim/Agent.h"
#include "fsim/World.h"
#include "foundation/io/AssetResolver.h"
#include "simulation/mission_systems/Cards.h"
#include "simulation/mission_systems/weapon/LaunchZone.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using Catch::Approx;
using namespace fsim;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr double kEarthR = 6378137.0;

std::string readBytes(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

ms::weapon::LaunchZoneQueryV1 makeQuery(double altM,
                                        double launchMach,
                                        double dhM,
                                        double targetMach,
                                        double aspectDeg,
                                        double rangeM,
                                        double offBoresightDeg = 0.0) {
    ms::weapon::LaunchZoneQueryV1 q;
    const double vL = launchMach * ms::weapon::isaSoundSpeedMs(altM);
    const double tAlt = std::max(500.0, altM + dhM);
    const double vT = targetMach * ms::weapon::isaSoundSpeedMs(tAlt);

    q.launchPosEcef[0] = kEarthR + altM;
    q.launchPosEcef[1] = 0.0;
    q.launchPosEcef[2] = 0.0;
    q.launchVelEcef[0] = 0.0;
    q.launchVelEcef[1] = 0.0;
    q.launchVelEcef[2] = vL;
    q.launchForwardEcef[0] = 0.0;
    q.launchForwardEcef[1] = 0.0;
    q.launchForwardEcef[2] = 1.0;
    q.launchAltitudeM = altM;
    q.launchAirspeedMs = vL;

    const double az = offBoresightDeg * kDeg;
    const double horiz = std::sqrt(std::max(0.0, rangeM * rangeM - dhM * dhM));
    const double ux = 0.0;
    const double uy = std::sin(az);
    const double uz = std::cos(az);
    q.targetPosEcef[0] = kEarthR + tAlt + ux;
    q.targetPosEcef[1] = horiz * uy;
    q.targetPosEcef[2] = horiz * uz;

    // Target velocity at aspectDeg relative to LOS (u = (0, uy, uz)):
    // aspect 0 => -u, aspect 90 => perpendicular in y-z plane.
    const double asp = aspectDeg * kDeg;
    const double py = uz;
    const double pz = -uy;
    q.targetVelEcef[0] = 0.0;
    q.targetVelEcef[1] = vT * (-std::cos(asp) * uy + std::sin(asp) * py);
    q.targetVelEcef[2] = vT * (-std::cos(asp) * uz + std::sin(asp) * pz);
    q.targetAltitudeM = tAlt;
    return q;
}

ms::weapon::LaunchZoneTableV1 makeSyntheticTable() {
    ms::weapon::LaunchZoneTableV1 t;
    t.schema = "fsim.ms.dlz.v1";
    t.card = "synthetic-aam";
    t.model = "syntheticaam";
    t.seeker = "active_radar";
    t.basis = "simulated";
    t.tableSha256 = std::string(64, 'a');
    t.modelXmlSha256 = std::string(64, 'b');
    t.cardSha256 = std::string(64, 'c');
    t.armingS = 1.0;
    t.lifeS = 90.0;
    t.activeRangeM = 17000.0;
    t.maxOffBoresightRad = 60.0 * kDeg;
    t.escapeTurnG = 7.0;
    t.minTerminalSpeedMs = 120.0;

    t.launchAltitudeM = {1000.0, 12000.0};
    t.launchMach = {0.8, 1.4};
    t.targetAltitudeDeltaM = {-2000.0, 0.0, 2000.0};
    t.targetMach = {0.8, 1.2};
    t.aspectDeg = {0.0, 90.0, 180.0};
    t.offBoresightDeg = {0.0, 30.0, 60.0};
    t.offBoresightFactor = {1.0, 0.90, 0.70};
    t.uncRMax = 0.15;
    t.uncRNe = 0.18;
    t.uncRMin = 0.10;
    t.uncTof = 0.12;

    const std::size_t n = t.gridSize();
    t.rMaxM.resize(n);
    t.rNeM.resize(n);
    t.rMinM.resize(n);
    t.tofMaxS.resize(n);
    t.tofNeS.resize(n);
    t.tofMinS.resize(n);

    std::size_t idx = 0;
    for (double alt : t.launchAltitudeM) {
        for (double lm : t.launchMach) {
            for (double dh : t.targetAltitudeDeltaM) {
                for (double tm : t.targetMach) {
                    for (double asp : t.aspectDeg) {
                        const double aspScale = 1.0 - 0.55 * (asp / 180.0);
                        const double rMax = (25000.0 + 3.0 * alt + 15000.0 * lm - 1.5 * dh + 4000.0 * tm) * aspScale;
                        t.rMaxM[idx] = rMax;
                        t.rNeM[idx] = 0.45 * rMax;
                        t.rMinM[idx] = 1500.0;
                        t.tofMaxS[idx] = rMax / 700.0;
                        t.tofNeS[idx] = (0.45 * rMax) / 750.0;
                        t.tofMinS[idx] = 1.8;
                        ++idx;
                    }
                }
            }
        }
    }
    return t;
}

std::uint16_t payloadOf(const Vehicle& v, ms::Family family) {
    for (const auto& p : v.payloads()) {
        if (p.family == family) return p.index;
    }
    return 0xFFFF;
}

void operatePayload(World& world, Agent& agent, std::uint16_t payload) {
    REQUIRE(payload != 0xFFFF);
    const auto res = agent.submit(ms::StateCommand{payload, ms::PayloadState::Operate});
    REQUIRE((res.accepted() || res.reason == ms::Reason::AlreadyInState));
    for (unsigned i = 0; i < 1200; ++i) {
        for (const auto& st : agent.payloads()) {
            if (st.index == payload && st.state == ms::PayloadState::Operate) return;
        }
        world.step();
    }
    FAIL("payload did not enter Operate state");
}

} // namespace

TEST_CASE("launch zone: SHA-256 and ISA speed of sound", "[sim-m04][dlz]") {
    CHECK(ms::weapon::computeSha256Hex("") ==
          "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
    CHECK(ms::weapon::computeSha256Hex("abc") ==
          "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    CHECK(ms::weapon::isaSoundSpeedMs(0.0) == Approx(340.294).margin(0.05));
    CHECK(ms::weapon::isaSoundSpeedMs(11000.0) == Approx(295.07).margin(0.05));
    CHECK(ms::weapon::isaSoundSpeedMs(18000.0) == Approx(295.07).margin(0.05));
}

TEST_CASE("launch zone: 5-D multilinear interpolation, off-boresight, uncertainty, and WEZ", "[sim-m04][dlz]") {
    const auto table = makeSyntheticTable();

    // Exact knot reproduction at (12000 m, M1.4, dh=0 m, M1.2, 0 deg)
    const auto qKnot = makeQuery(12000.0, 1.4, 0.0, 1.2, 0.0, 30000.0, 0.0);
    const auto zKnot = ms::weapon::evaluateLaunchZoneV2(table, qKnot);
    REQUIRE(zKnot.valid);
    CHECK(zKnot.flyoutDerived);
    CHECK(zKnot.inEnvelope);
    CHECK(zKnot.basis == "simulated");
    CHECK(zKnot.tableSha256 == table.tableSha256);

    const double expectedMax = 25000.0 + 3.0 * 12000.0 + 15000.0 * 1.4 + 4000.0 * 1.2;
    CHECK(zKnot.maximumM == Approx(expectedMax).margin(1.0));
    CHECK(zKnot.probabilityInterceptM == Approx(expectedMax).margin(1.0));
    CHECK(zKnot.noEscapeM == Approx(0.45 * expectedMax).margin(1.0));
    CHECK(zKnot.minimumM == Approx(1500.0).margin(1.0));
    CHECK(zKnot.maximumUncertaintyM == Approx(0.15 * zKnot.maximumM).margin(1.0));
    CHECK(zKnot.noEscapeUncertaintyM == Approx(0.18 * zKnot.noEscapeM).margin(1.0));
    CHECK(zKnot.minimumUncertaintyM == Approx(0.10 * zKnot.minimumM).margin(1.0));
    CHECK(std::isfinite(zKnot.timeOfFlightS));
    CHECK(zKnot.timeOfFlightS > 0.0);
    CHECK(std::isfinite(zKnot.timeOfFlightUncertaintyS));
    CHECK(zKnot.timeOfFlightUncertaintyS > 0.0);
    CHECK(std::isfinite(zKnot.fPoleM));
    CHECK(std::isfinite(zKnot.aPoleM));
    CHECK(zKnot.inRange);
    CHECK(zKnot.inNoEscape);

    // Off-boresight attenuation at 30 deg reduces R_pi and R_ne by 0.90x while preserving boresight R_aero.
    const auto qOff = makeQuery(12000.0, 1.4, 0.0, 1.2, 0.0, 30000.0, 30.0);
    const auto zOff = ms::weapon::evaluateLaunchZoneV2(table, qOff);
    REQUIRE(zOff.valid);
    CHECK(zOff.maximumM == Approx(zKnot.maximumM).margin(10.0));
    CHECK(zOff.probabilityInterceptM == Approx(0.90 * zKnot.maximumM).margin(10.0));
    CHECK(zOff.noEscapeM == Approx(0.90 * zKnot.noEscapeM).margin(10.0));

    // Out-of-envelope query clamps smoothly and sets inEnvelope=false
    const auto qOut = makeQuery(25000.0, 2.2, 0.0, 1.0, 0.0, 30000.0, 0.0);
    const auto zOut = ms::weapon::evaluateLaunchZoneV2(table, qOut);
    REQUIRE(zOut.valid);
    CHECK_FALSE(zOut.inEnvelope);
    CHECK(zOut.maximumM > 0.0);
    CHECK(zOut.minimumM <= zOut.noEscapeM);
    CHECK(zOut.noEscapeM <= zOut.maximumM);

    // Aspect-polar WEZ contour
    const auto wez = ms::weapon::evaluateLaunchZoneWezV1(table, qKnot);
    REQUIRE(wez.valid);
    CHECK(wez.flyoutDerived);
    CHECK(wez.inEnvelope);
    REQUIRE(wez.aspectContour.size() >= 3);
    CHECK(wez.aspectContour.front().aspectRad == Approx(0.0).margin(1e-6));
    CHECK(wez.aspectContour.back().aspectRad == Approx(kPi).margin(1e-4));
    CHECK(wez.aspectContour.front().maximumM > wez.aspectContour.back().maximumM);
    for (const auto& sample : wez.aspectContour) {
        CHECK(sample.minimumM >= 0.0);
        CHECK(sample.noEscapeM <= sample.maximumM + 1e-6);
        CHECK(sample.maximumUncertaintyM > 0.0);
    }
}

TEST_CASE("launch zone: table validator rejects malformed files cleanly", "[sim-m04][dlz]") {
    const auto tmpDir = std::filesystem::temp_directory_path() / "fsim_test_dlz_invalid";
    std::filesystem::create_directories(tmpDir);
    const auto badPath = tmpDir / "bad.dlz-v1.json";
    {
        std::ofstream out(badPath, std::ios::binary | std::ios::trunc);
        out << R"({"schema":"wrong.schema.v0","card":"x"})";
    }
    ms::weapon::LaunchZoneTableV1 tbl = makeSyntheticTable();
    const std::string origCard = tbl.card;
    std::string err;
    CHECK_FALSE(ms::weapon::loadLaunchZoneTableV1(badPath, tbl, err));
    CHECK_FALSE(err.empty());
    CHECK(tbl.card == origCard); // unchanged on failure
    std::filesystem::remove_all(tmpDir);
}

TEST_CASE("launch zone: all 24 maintained AAM .dlz-v1.json tables load with valid provenance and monotonic physics", "[sim-m04][dlz]") {
    const std::filesystem::path storeDir(FSIM_TEST_STORE_DIR);
    const std::filesystem::path aircraftDir(FSIM_TEST_AIRCRAFT_DIR);

    std::vector<std::string> errors;
    const auto library = ms::readStoreLibrary(storeDir, errors);
    REQUIRE_FALSE(library.empty());

    int aamCount = 0;
    double amraamNeRatio = 0.0;
    double meteorNeRatio = 0.0;

    for (const auto& [id, card] : library) {
        if (!card || card->storeClass != "aam" || card->model.rfind("jsbsim:", 0) != 0) continue;
        ++aamCount;
        const std::string model = card->model.substr(7);
        const auto dlzPath = aircraftDir / "stores" / model / (model + ".dlz-v1.json");
        const auto xmlPath = aircraftDir / "stores" / model / (model + ".xml");
        const auto cardPath = storeDir / (id + ".json");
        INFO("AAM card=" << id << " path=" << dlzPath.string());
        REQUIRE(std::filesystem::is_regular_file(dlzPath));

        ms::weapon::LaunchZoneTableV1 table;
        std::string err;
        REQUIRE(ms::weapon::loadLaunchZoneTableV1(dlzPath, table, err));
        CHECK(table.card == id);
        CHECK(table.model == model);
        CHECK(table.basis == "simulated");
        CHECK_FALSE(table.equipmentCalibrated);
        CHECK(table.tableSha256.size() == 64);
        CHECK(table.modelXmlSha256 == ms::weapon::computeSha256Hex(readBytes(xmlPath)));
        CHECK(table.cardSha256 == ms::weapon::computeSha256Hex(readBytes(cardPath)));
        CHECK(table.uncRMax >= 0.05);
        CHECK(table.uncRNe >= 0.05);

        const auto zHighHead = ms::weapon::evaluateLaunchZoneV2(table, makeQuery(12000.0, 1.2, 0.0, 0.9, 0.0, 15000.0));
        const auto zLowHead = ms::weapon::evaluateLaunchZoneV2(table, makeQuery(1000.0, 0.9, 0.0, 0.9, 0.0, 8000.0));
        const auto zMidTail = ms::weapon::evaluateLaunchZoneV2(table, makeQuery(6000.0, 0.9, 0.0, 0.9, 180.0, 8000.0));
        REQUIRE(zHighHead.valid);
        REQUIRE(zLowHead.valid);
        REQUIRE(zMidTail.valid);
        CHECK(zHighHead.maximumM > 5000.0);
        CHECK(zHighHead.minimumM <= zHighHead.noEscapeM + 1e-6);
        CHECK(zHighHead.noEscapeM <= zHighHead.maximumM + 1e-6);
        CHECK(zHighHead.maximumM > zLowHead.maximumM);
        CHECK(zHighHead.maximumM > zMidTail.maximumM);

        if (id == "aim-120c") amraamNeRatio = zHighHead.noEscapeM / zHighHead.maximumM;
        if (id == "meteor") meteorNeRatio = zHighHead.noEscapeM / zHighHead.maximumM;
    }

    CHECK(aamCount == 24);
    // Ducted ramjet sustained thrust naturally yields a higher NEZ/R_max fraction than solid rocket.
    CHECK(meteorNeRatio > amraamNeRatio);
}

TEST_CASE("launch zone: World and Agent APIs expose flyout-derived DLZ/WEZ and enforce own designations", "[sim-m04][dlz][native]") {
    WorldOptions opts;
    opts.name = "sim-m04-dlz-native";
    opts.publish = false;
    opts.workers = 1;
    opts.pinWorkers = false;
    opts.jsbsimRoot = FSIM_TEST_JSBSIM_ROOT;
    opts.seed = 42;

    World world(opts);

    VehicleSpec blueSpec;
    blueSpec.name = "blue";
    blueSpec.type = "jsbsim:f16c";
    blueSpec.initial.latitudeDeg = 40.0;
    blueSpec.initial.longitudeDeg = 0.0;
    blueSpec.initial.altitudeMslM = 10000.0;
    blueSpec.initial.headingDeg = 0.0;
    blueSpec.initial.airspeedTrueMs = 300.0;

    VehicleSpec redSpec = blueSpec;
    redSpec.name = "red";
    redSpec.initial.latitudeDeg = 40.0 + 20.0 / 111.2; // ~20 km north, head-on
    redSpec.initial.headingDeg = 180.0;
    redSpec.initial.airspeedTrueMs = 265.0;

    Vehicle blue = world.createVehicle(blueSpec);
    Vehicle red = world.createVehicle(redSpec);
    REQUIRE(blue.loadStore(payloadOf(blue, ms::Family::StoreStation), 1, "aim-120c") == ms::Reason::None);

    // World Range query returns flyout-derived LaunchZoneV2 and WEZ.
    const auto z2 = world.launchZoneV2(blue.id(), "aim-120c", red.id());
    REQUIRE(z2.valid);
    CHECK(z2.version == 2u);
    CHECK(z2.flyoutDerived);
    CHECK(z2.basis == "simulated");
    CHECK(z2.tableSha256.size() == 64);
    CHECK(z2.maximumM > 40000.0);
    CHECK(z2.noEscapeM > 15000.0);
    CHECK(z2.noEscapeM < z2.maximumM);
    CHECK(z2.minimumM < z2.noEscapeM);
    CHECK(z2.maximumUncertaintyM > 0.0);
    CHECK(z2.inRange);
    CHECK(z2.inNoEscape);
    CHECK(std::isfinite(z2.timeOfFlightS));
    CHECK(z2.timeOfFlightS > 5.0);

    // Legacy World::launchZone delegates to LaunchZoneV2.
    const auto z1 = world.launchZone(blue.id(), "aim-120c", red.id());
    CHECK(z1.maximumM == Approx(z2.maximumM));
    CHECK(z1.noEscapeM == Approx(z2.noEscapeM));
    CHECK(z1.minimumM == Approx(z2.minimumM));

    // WEZ contour
    const auto wez = world.launchZoneWezV1(blue.id(), "aim-120c", red.id());
    REQUIRE(wez.valid);
    CHECK(wez.flyoutDerived);
    CHECK(wez.tableSha256 == z2.tableSha256);
    REQUIRE(wez.aspectContour.size() >= 5);
    CHECK(wez.aspectContour.front().maximumM > wez.aspectContour.back().maximumM);

    // Agent boundary enforcement: refuse raw entity IDs and foreign designations.
    Agent blueAgent = world.createAgent(blue, "blue-agent");
    Agent redAgent = world.createAgent(red, "red-agent");
    CHECK_FALSE(blueAgent.launchZoneV2("aim-120c", red.id()).has_value());
    CHECK_FALSE(blueAgent.launchZoneWezV1("aim-120c", red.id()).has_value());

    const auto blueRadar = payloadOf(blue, ms::Family::AirRadar);
    operatePayload(world, blueAgent, blueRadar);
    ms::AmtiCommand search;
    search.payload = blueRadar;
    search.azimuthHalfWidthRad = 10.0 * kDeg;
    search.bars = 1;
    search.rangeScaleM = 60000.0;
    REQUIRE(blueAgent.submit(search).accepted());

    std::uint64_t trackId = 0;
    for (unsigned step = 0; step < 600 && !trackId; ++step) {
        world.step();
        for (const auto& tr : blueAgent.amtiTracks(blueRadar)) {
            if (tr.updates >= 3) {
                trackId = tr.id;
                break;
            }
        }
    }
    REQUIRE(trackId != 0);
    const auto desig = blueAgent.designate(blueRadar, trackId);
    REQUIRE(desig != 0);

    const auto agentZ2 = blueAgent.launchZoneV2("aim-120c", desig);
    REQUIRE(agentZ2.has_value());
    REQUIRE(agentZ2->valid);
    CHECK(agentZ2->flyoutDerived);
    CHECK(agentZ2->tableSha256 == z2.tableSha256);
    CHECK_FALSE(redAgent.launchZoneV2("aim-120c", desig).has_value());

    const auto agentWez = blueAgent.launchZoneWezV1("aim-120c", desig);
    REQUIRE(agentWez.has_value());
    REQUIRE(agentWez->valid);
    CHECK(agentWez->flyoutDerived);
    CHECK_FALSE(redAgent.launchZoneWezV1("aim-120c", desig).has_value());

    // Lapsed designation is rejected.
    REQUIRE(blueAgent.undesignate(desig));
    CHECK_FALSE(blueAgent.launchZoneV2("aim-120c", desig).has_value());
    CHECK_FALSE(blueAgent.launchZoneWezV1("aim-120c", desig).has_value());
}

TEST_CASE("launch zone: native 6-DOF JSBSim AIM-120C launch inside R_ne intercepts target", "[sim-m04][dlz][native]") {
    WorldOptions opts;
    opts.name = "sim-m04-dlz-crosscheck";
    opts.publish = false;
    opts.workers = 1;
    opts.pinWorkers = false;
    opts.jsbsimRoot = FSIM_TEST_JSBSIM_ROOT;
    opts.seed = 99;

    World world(opts);

    VehicleSpec blueSpec;
    blueSpec.name = "blue";
    blueSpec.type = "jsbsim:f16c";
    blueSpec.initial.latitudeDeg = 40.0;
    blueSpec.initial.longitudeDeg = 0.0;
    blueSpec.initial.altitudeMslM = 10000.0;
    blueSpec.initial.headingDeg = 0.0;
    blueSpec.initial.airspeedTrueMs = 300.0;

    VehicleSpec redSpec = blueSpec;
    redSpec.name = "red";
    redSpec.initial.latitudeDeg = 40.0 + 22.0 / 111.2; // 22 km head-on, well inside R_ne
    redSpec.initial.headingDeg = 180.0;
    redSpec.initial.airspeedTrueMs = 265.0;

    Vehicle blue = world.createVehicle(blueSpec);
    Vehicle red = world.createVehicle(redSpec);
    const auto stations = payloadOf(blue, ms::Family::StoreStation);
    const auto radar = payloadOf(blue, ms::Family::AirRadar);
    REQUIRE(blue.loadStore(stations, 1, "aim-120c") == ms::Reason::None);

    Agent blueAgent = world.createAgent(blue, "blue-agent");
    Agent redAgent = world.createAgent(red, "red-agent");

    control::HsaCommand blueHold;
    blueHold.headingRad = 0.0;
    blueHold.speed = 300.0;
    blueHold.altitudeM = 10000.0;
    REQUIRE(blueAgent.submit(blueHold).accepted());

    control::HsaCommand redHold;
    redHold.headingRad = 180.0 * kDeg;
    redHold.speed = 265.0;
    redHold.altitudeM = 10000.0;
    REQUIRE(redAgent.submit(redHold).accepted());

    operatePayload(world, blueAgent, stations);
    operatePayload(world, blueAgent, radar);

    ms::AmtiCommand search;
    search.payload = radar;
    search.azimuthHalfWidthRad = 10.0 * kDeg;
    search.bars = 1;
    search.rangeScaleM = 60000.0;
    REQUIRE(blueAgent.submit(search).accepted());

    std::uint64_t trackId = 0;
    for (unsigned step = 0; step < 600 && !trackId; ++step) {
        world.step();
        for (const auto& tr : blueAgent.amtiTracks(radar)) {
            if (tr.updates >= 3) {
                trackId = tr.id;
                break;
            }
        }
    }
    REQUIRE(trackId != 0);
    const auto desig = blueAgent.designate(radar, trackId);
    REQUIRE(desig != 0);

    const auto dlz = blueAgent.launchZoneV2("aim-120c", desig);
    REQUIRE(dlz.has_value());
    REQUIRE(dlz->valid);
    REQUIRE(dlz->flyoutDerived);
    REQUIRE(dlz->inRange);
    REQUIRE(dlz->inNoEscape);

    ms::StrikeCommand strike;
    strike.payload = stations;
    strike.store = "aim-120c";
    strike.quantity = 1;
    strike.target = desig;
    strike.consent = true;
    REQUIRE(blue.submit(strike).accepted());

    bool detonated = false;
    double missM = std::numeric_limits<double>::infinity();
    const unsigned maxSteps = static_cast<unsigned>(std::ceil(45.0 / world.stepSeconds()));
    for (unsigned step = 0; step < maxSteps && !detonated; ++step) {
        world.step();
        for (const auto& w : world.weapons()) {
            if (w.end == ms::WeaponEnd::Detonated) {
                detonated = true;
                missM = w.missM;
                break;
            }
        }
    }
    CHECK(detonated);
    CHECK(std::isfinite(missM));
    CHECK(missM < 25.0);
}

TEST_CASE("launch zone: open-source alignment with ICAO ISA, Zarchan F-pole/A-pole kinematics, and propulsion scaling", "[sim-m04][dlz][alignment]") {
    // 1) ICAO / US Standard Atmosphere 1976 speed-of-sound cross-check (0..20 km)
    CHECK(ms::weapon::isaSoundSpeedMs(0.0) == Approx(340.294).margin(0.05));
    CHECK(ms::weapon::isaSoundSpeedMs(5000.0) == Approx(320.545).margin(0.05));
    CHECK(ms::weapon::isaSoundSpeedMs(11000.0) == Approx(295.070).margin(0.05));
    CHECK(ms::weapon::isaSoundSpeedMs(15000.0) == Approx(295.070).margin(0.05));
    CHECK(ms::weapon::isaSoundSpeedMs(20000.0) == Approx(295.070).margin(0.05));

    // 2) Exact kinematic F-pole and A-pole closure alignment (Zarchan / BVR geometry)
    const std::filesystem::path aircraftDir(FSIM_TEST_AIRCRAFT_DIR);
    ms::weapon::LaunchZoneTableV1 amraam, aim9x, meteor;
    std::string err;
    REQUIRE(ms::weapon::loadLaunchZoneTableV1(aircraftDir / "stores" / "aim120c" / "aim120c.dlz-v1.json", amraam, err));
    REQUIRE(ms::weapon::loadLaunchZoneTableV1(aircraftDir / "stores" / "aim9x" / "aim9x.dlz-v1.json", aim9x, err));
    REQUIRE(ms::weapon::loadLaunchZoneTableV1(aircraftDir / "stores" / "meteor" / "meteor.dlz-v1.json", meteor, err));

    const auto qBvr = makeQuery(12000.0, 1.2, 0.0, 0.9, 0.0, 85000.0, 0.0);
    const auto zAmraam = ms::weapon::evaluateLaunchZoneV2(amraam, qBvr);
    REQUIRE(zAmraam.valid);
    REQUIRE(zAmraam.inRange);
    REQUIRE(std::isfinite(zAmraam.timeOfFlightS));
    // Pre-merge head-on closure: separation shrinks by (vL + vT) * t_tof for F-pole, and (vL + vT) * t_act for A-pole.
    const double vClose = zAmraam.closingSpeedMs;
    const double expectedFPole = std::abs(zAmraam.slantRangeM - vClose * zAmraam.timeOfFlightS);
    CHECK(zAmraam.fPoleM == Approx(expectedFPole).margin(5.0));
    const double tActive = zAmraam.timeOfFlightS * (1.0 - amraam.activeRangeM / zAmraam.slantRangeM);
    const double expectedAPole = std::abs(zAmraam.slantRangeM - vClose * tActive);
    CHECK(zAmraam.aPoleM == Approx(expectedAPole).margin(5.0));
    CHECK(zAmraam.aPoleM > zAmraam.fPoleM); // Pre-merge: pitbull occurs strictly before intercept

    // IR missile (no active radar seeker) must report NaN A-pole
    const auto qWvr = makeQuery(6000.0, 0.9, 0.0, 0.9, 0.0, 12000.0, 0.0);
    const auto zAim9x = ms::weapon::evaluateLaunchZoneV2(aim9x, qWvr);
    REQUIRE(zAim9x.valid);
    REQUIRE(zAim9x.inRange);
    CHECK(std::isfinite(zAim9x.fPoleM));
    CHECK(std::isnan(zAim9x.aPoleM));

    // 3) Propulsion-class kinematic ordering: Meteor ducted ramjet sustains terminal energy better than solid-rocket AMRAAM
    const auto zMeteor = ms::weapon::evaluateLaunchZoneV2(meteor, qBvr);
    REQUIRE(zMeteor.valid);
    CHECK(zMeteor.noEscapeM / zMeteor.maximumM > zAmraam.noEscapeM / zAmraam.maximumM + 0.05);
    // Higher launch altitude lowers atmospheric drag, monotonically increasing R_max
    const auto zAmraamLow = ms::weapon::evaluateLaunchZoneV2(amraam, makeQuery(1000.0, 1.2, 0.0, 0.9, 0.0, 45000.0, 0.0));
    CHECK(zAmraam.maximumM > zAmraamLow.maximumM);
}
