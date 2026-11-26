// SIM-M06 (MS-10): Passive & Active Optical/IR Sensors (IRST, EOTS/FLIR,
// Spherical DAS/MAWS, Solar-Blind UV MWS, Flare Signatures, and Laser Rangefinder).

#include "fsim/Agent.h"
#include "fsim/World.h"
#include "simulation/mission_systems/Cards.h"
#include "simulation/mission_systems/Model.h"
#include "simulation/mission_systems/optical/OpticalSensor.h"

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

fsim::WorldOptions makeOptions(const char* name, std::uint64_t seed = 51) {
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

TEST_CASE("SIM-M06 open-source radiometric alignment: Planck integral, kinetic heating, Beer-Lambert/Kruse transmittance, and LRF link", "[sim-m06][optical][alignment]") {
    using namespace fsim::ms::optical;

    // 1) Exact dimensionless Planck blackbody band fraction (Wien / Stefan-Boltzmann limits)
    CHECK(planckBandFraction(1000.0, 0.1, 1000.0) == Catch::Approx(1.0).margin(1e-3));
    const double f300Mwir = planckBandFraction(300.0, 3.0, 5.0);
    const double f900Mwir = planckBandFraction(900.0, 3.0, 5.0);
    const double f1800Mwir = planckBandFraction(1800.0, 3.0, 5.0);
    const double f300Lwir = planckBandFraction(300.0, 8.0, 12.0);
    CHECK(f300Mwir > 0.005);
    CHECK(f300Mwir < 0.03);
    CHECK(f300Lwir > 0.25); // 300 K blackbody peaks at 9.66 um (LWIR)
    CHECK(f900Mwir > 0.20); // 900 K blackbody peaks at 3.22 um (MWIR)
    CHECK(f1800Mwir > f300Mwir * 10.0);

    // 2) ICAO ISA temperature & aerodynamic kinetic-heating skin recovery temperature
    CHECK(isaTemperatureK(0.0) == Catch::Approx(288.15).margin(1e-6));
    CHECK(isaTemperatureK(11000.0) == Catch::Approx(216.65).margin(1e-6));
    CHECK(kineticSkinTemperatureK(216.65, 0.8, 0.85) == Catch::Approx(216.65 * (1.0 + 0.85 * 0.2 * 0.64)).margin(1e-9));
    CHECK(kineticSkinTemperatureK(216.65, 1.8, 0.85) > 330.0); // Strong supercruise skin heating

    // 3) Altitude-stratified Beer-Lambert + Kruse visibility + cloud opacity
    fsim::EnvironmentState clearEnv;
    clearEnv.visibilityM = 50000.0;
    clearEnv.humidity = 0.2;
    clearEnv.cloudCover = 0.0;
    clearEnv.precipitation = 0.0;

    const double tauHigh = atmosphericTransmittance(30000.0, 11000.0, 11000.0, 3.0, 5.0, clearEnv);
    const double tauLow = atmosphericTransmittance(30000.0, 500.0, 500.0, 3.0, 5.0, clearEnv);
    CHECK(tauHigh > tauLow); // Thinner upper atmosphere transmits significantly better
    CHECK(tauHigh > 0.45);
    CHECK(tauLow > 0.02);

    // Beer-Lambert exact exponential range law at constant altitude
    const double tau10 = atmosphericTransmittance(10000.0, 6000.0, 6000.0, 3.0, 5.0, clearEnv);
    const double tau20 = atmosphericTransmittance(20000.0, 6000.0, 6000.0, 3.0, 5.0, clearEnv);
    CHECK(tau20 == Catch::Approx(tau10 * tau10).margin(1e-9));

    // Overcast cloud layer between sensor (8 km) and target (1 km) strongly blocks EO/IR
    fsim::EnvironmentState cloudEnv = clearEnv;
    cloudEnv.cloudBaseM = 2500.0;
    cloudEnv.cloudCover = 1.0;
    const double tauCloud = atmosphericTransmittance(15000.0, 8000.0, 1000.0, 3.0, 5.0, cloudEnv);
    CHECK(tauCloud < 1e-3);

    // 4) Aircraft & weapon physical IR signatures (aspect, afterburner, Mach, UV)
    fsim::sim::VehicleState jet{};
    jet.positionEcef[0] = 6377000.0;
    jet.altitudeMslM = 10000.0;
    jet.mach = 0.85;
    jet.engineCount = 1;
    jet.throttlePosition[0] = 0.85;
    jet.afterburner[0] = 0.0;
    // Body +x points along +Y ECEF
    jet.rotationBodyToEcef[3] = 1.0;
    jet.rotationBodyToEcef[1] = -1.0;
    jet.rotationBodyToEcef[8] = 1.0;

    const double noseSensor[3] = {6377000.0, 20000.0, 0.0};
    const double tailSensor[3] = {6377000.0, -20000.0, 0.0};
    const double iNoseDry = targetRadiantIntensityWSr(jet, fsim::ms::Role::Aircraft, false, noseSensor, 3.0, 5.0, 5.0);
    const double iTailDry = targetRadiantIntensityWSr(jet, fsim::ms::Role::Aircraft, false, tailSensor, 3.0, 5.0, 5.0);
    CHECK(iTailDry > iNoseDry * 2.0); // Tailpipe + plume dominates rear hemisphere

    jet.afterburner[0] = 1.0;
    const double iTailAb = targetRadiantIntensityWSr(jet, fsim::ms::Role::Aircraft, false, tailSensor, 3.0, 5.0, 5.0);
    CHECK(iTailAb > iTailDry * 3.0); // Afterburner surge

    jet.afterburner[0] = 0.0;
    jet.mach = 1.8;
    const double iNoseSuper = targetRadiantIntensityWSr(jet, fsim::ms::Role::Aircraft, false, noseSensor, 3.0, 5.0, 5.0);
    CHECK(iNoseSuper > iNoseDry * 1.4); // Supersonic aerodynamic skin heating boosts head-on MWIR

    // Solar-blind UV: aircraft == 0, burning rocket > 0, coasting rocket == 0
    CHECK(targetRadiantIntensityWSr(jet, fsim::ms::Role::Aircraft, false, tailSensor, 0.25, 0.28, 5.0) == 0.0);
    CHECK(targetRadiantIntensityWSr(jet, fsim::ms::Role::Weapon, true, noseSensor, 0.25, 0.28, 1.0) > 50.0);
    CHECK(targetRadiantIntensityWSr(jet, fsim::ms::Role::Weapon, false, noseSensor, 0.25, 0.28, 1.0) == 0.0);

    // 5) Two-way Lambertian laser rangefinder link equation (R^-2 * tau^2)
    const double snr10 = laserReturnSnr(10000.0, 8000.0, 8000.0, 5e5, 0.12, 1.064, 0.2, 2.5e-9, clearEnv);
    const double snr5 = laserReturnSnr(5000.0, 8000.0, 8000.0, 5e5, 0.12, 1.064, 0.2, 2.5e-9, clearEnv);
    CHECK(snr10 > 10.0);
    CHECK(snr5 > snr10 * 4.0); // Strictly faster than R^-2 due to two-way atmospheric attenuation
    const double snrCloud = laserReturnSnr(10000.0, 8000.0, 1000.0, 5e5, 0.12, 1.064, 0.2, 2.5e-9, cloudEnv);
    CHECK(snrCloud < 1e-3);
}

TEST_CASE("SIM-M06 fleet coverage: all 19 passive_optical payload cards construct with honest LRF discrimination", "[sim-m06][optical][cards]") {
    std::vector<std::string> errors;
    std::vector<std::shared_ptr<const fsim::ms::Card>> cards;
    for (const auto& entry : std::filesystem::directory_iterator(std::filesystem::path(FSIM_TEST_PAYLOAD_DIR))) {
        if (!entry.is_regular_file() || entry.path().extension() != ".json") continue;
        if (auto c = fsim::ms::readCard(entry.path(), errors)) cards.push_back(std::move(c));
    }
    REQUIRE(errors.empty());

    const std::set<std::string> lrfCards = {
        "an-aaq-40", "eots-86", "h6k-eots", "oeps-27", "oeps-29", "osf", "101ks-atoll", "klen-ps"};

    int poCount = 0;
    for (const auto& card : cards) {
        if (!card || card->family != fsim::ms::Family::PassiveOptical) continue;
        const std::string& id = card->id;
        ++poCount;
        auto model = fsim::ms::makeModel(*card, 1);
        INFO("passive_optical card=" << id);
        REQUIRE(model != nullptr);
        std::vector<fsim::ms::PayloadValue> rep;
        model->report(rep);
        const double expectedLrf = lrfCards.count(id) ? 1.0 : 0.0;
        CHECK(reportValue(rep, "laser_capable") == expectedLrf);
        CHECK(reportValue(rep, "wavelength_min") > 0.0);
        CHECK(reportValue(rep, "wavelength_max") > reportValue(rep, "wavelength_min"));
        CHECK(reportValue(rep, "nei") > 0.0);
    }
    CHECK(poCount == 19);
}

TEST_CASE("SIM-M06 fighter IRST search, passive-range honesty, cloud occultation, and active LRF track ranging", "[sim-m06][optical][lrf]") {
    fsim::World world(makeOptions("sim_m06_irst_lrf", 61));
    // F-35A (an-aaq-40 EOTS with LRF) trailing a target F-16C at 12 km (tail aspect, inside 25 km EOTS LRF range)
    auto f35 = createAircraft(world, "f35a", "f35a", 0.0, 0.0, 7000.0, 90.0, 230.0);
    auto target = createAircraft(world, "target", "f16c", 12.0, 0.0, 7000.0, 90.0, 230.0);

    const auto eotsIdx = findPayload(f35, "eots");
    powerToOperate(world, f35, eotsIdx);

    // Reject out-of-FOR search elevation (+75 deg on under-nose EOTS limited to +35 deg)
    fsim::ms::PayloadTask badEl;
    badEl.payload = eotsIdx;
    badEl.kind = fsim::ms::TaskKind::Search;
    badEl.elevationCentreRad = 75.0 * kDeg;
    const auto badRes = f35.submit(badEl);
    CHECK(!badRes.accepted());
    CHECK(badRes.reason == fsim::ms::Reason::OutOfReach);

    // Valid forward volume search
    fsim::ms::PayloadTask search;
    search.payload = eotsIdx;
    search.kind = fsim::ms::TaskKind::Search;
    search.azimuthCentreRad = 0.0;
    search.elevationCentreRad = 0.0;
    search.azimuthHalfWidthRad = 30.0 * kDeg;
    search.elevationHalfWidthRad = 20.0 * kDeg;
    REQUIRE(f35.submit(search).accepted());

    world.step(30); // 1 s
    const auto contacts = f35.contacts(eotsIdx);
    REQUIRE(!contacts.empty());
    const auto& cSearch = contacts.front();
    CHECK(cSearch.kind == fsim::ms::ContactKind::Infrared);
    CHECK(cSearch.truth == target.id());
    CHECK(std::abs(cSearch.azimuthRad) < 5.0 * kDeg);
    // Passive search MUST NOT fabricate range or 3-D geodetic position
    CHECK(std::isnan(cSearch.rangeM));
    CHECK(std::isnan(cSearch.latitudeRad));
    CHECK(std::isnan(cSearch.longitudeRad));
    CHECK(reportValue(f35.payloadReport(eotsIdx), "laser_ranged") == 0.0);

    // Command TaskKind::Track on the held IR contact -> EOTS activates LRF and measures range + 3-D coordinates
    fsim::ms::PayloadTask track;
    track.payload = eotsIdx;
    track.kind = fsim::ms::TaskKind::Track;
    track.contact = cSearch.id;
    const auto trackAct = f35.submit(track);
    REQUIRE(trackAct.accepted());

    world.step(30); // 1 s
    const auto trackedContacts = f35.contacts(eotsIdx);
    REQUIRE(!trackedContacts.empty());
    const auto& cTrack = trackedContacts.front();
    CHECK(cTrack.id == cSearch.id);
    REQUIRE(std::isfinite(cTrack.rangeM));
    CHECK(cTrack.rangeM == Catch::Approx(12000.0).margin(500.0));
    CHECK(std::isfinite(cTrack.latitudeRad));
    CHECK(std::isfinite(cTrack.longitudeRad));
    CHECK(std::isfinite(cTrack.heightM));

    const auto rep = f35.payloadReport(eotsIdx);
    CHECK(reportValue(rep, "tracking") == 1.0);
    CHECK(reportValue(rep, "laser_active") == 1.0);
    CHECK(reportValue(rep, "laser_ranged") == 1.0);
    CHECK(reportValue(rep, "laser_range") == Catch::Approx(cTrack.rangeM).margin(1.0));

    // Passive-only IRST (Typhoon PIRATE) tracks angularly without ever fabricating laser range
    auto typhoon = createAircraft(world, "typhoon", "typhoon", 0.0, 2.0, 7000.0, 90.0, 230.0);
    const auto pirateIdx = findPayload(typhoon, "irst");
    powerToOperate(world, typhoon, pirateIdx);
    fsim::ms::PayloadTask pSearch = search;
    pSearch.payload = pirateIdx;
    REQUIRE(typhoon.submit(pSearch).accepted());
    world.step(30);
    const auto pContacts = typhoon.contacts(pirateIdx);
    REQUIRE(!pContacts.empty());
    fsim::ms::PayloadTask pTrack;
    pTrack.payload = pirateIdx;
    pTrack.kind = fsim::ms::TaskKind::Track;
    pTrack.contact = pContacts.front().id;
    REQUIRE(typhoon.submit(pTrack).accepted());
    world.step(30);
    const auto pTracked = typhoon.contacts(pirateIdx);
    REQUIRE(!pTracked.empty());
    CHECK(std::isnan(pTracked.front().rangeM));
    CHECK(reportValue(typhoon.payloadReport(pirateIdx), "laser_capable") == 0.0);
    CHECK(reportValue(typhoon.payloadReport(pirateIdx), "laser_ranged") == 0.0);

    // Overcast cloud layer placed at 7 km altitude blinds optical sensors (track lost after memoryS)
    fsim::EnvironmentState cloudy = world.environment().state();
    cloudy.cloudBaseM = 6200.0;
    cloudy.cloudCover = 1.0;
    world.environment().set(cloudy);
    world.step(150); // 5 s (> 3 s track memory)
    const auto act = f35.missionActivity(trackAct.activity);
    REQUIRE(act.has_value());
    CHECK(act->state == fsim::control::ActivityState::Failed);
    CHECK(act->reason == fsim::ms::Reason::ContactLost);
}

TEST_CASE("SIM-M06 spherical DAS continuous 360-deg coverage, flare visibility, UV MWS discrimination, and Agent boundaries", "[sim-m06][optical][das][agent]") {
    fsim::World world(makeOptions("sim_m06_das_uv", 71));
    // F-35A (an-aaq-37 DAS) flying east; target F-16C 10 km BEHIND it (rear hemisphere) dispensing flares;
    // KC-46A (an-aar-54 solar-blind UV MWS) nearby to confirm zero aircraft false alarms.
    auto f35 = createAircraft(world, "f35a", "f35a", 10.0, 0.0, 6000.0, 90.0, 220.0);
    auto rearJet = createAircraft(world, "rear", "f16c", 0.0, 0.0, 6000.0, 90.0, 220.0);
    auto tanker = createAircraft(world, "kc46", "kc46a", 5.0, 5.0, 6000.0, 90.0, 190.0);

    const auto dasIdx = findPayload(f35, "das");
    const auto cmdsIdx = findPayload(rearJet, "cmds");
    const auto mwsIdx = findPayload(tanker, "mws");
    powerToOperate(world, f35, dasIdx);
    powerToOperate(world, rearJet, cmdsIdx);
    powerToOperate(world, tanker, mwsIdx);

    // DAS operates continuously on Operate without any Search task and sees the rear-hemisphere jet
    world.step(30); // 1 s
    const auto dasContacts = f35.contacts(dasIdx);
    bool sawRearJet = false;
    for (const auto& c : dasContacts) {
        if (c.truth == rearJet.id()) {
            sawRearJet = true;
            CHECK(c.kind == fsim::ms::ContactKind::Infrared);
            CHECK(std::abs(c.azimuthRad) > 150.0 * kDeg); // Directly behind F-35A
            CHECK(std::isnan(c.rangeM));
            CHECK(!c.warning); // Aircraft, not an incoming missile
        }
    }
    CHECK(sawRearJet);

    // Solar-blind UV MWS on KC-46A must NOT false-alarm on non-burning jet aircraft
    CHECK(tanker.contacts(mwsIdx).empty());

    // Rear jet dispenses flares -> F-35A DAS detects bright pyrotechnic flare decoy in MWIR
    REQUIRE(world.dispense(rearJet.id(), true) == fsim::ms::Reason::None);
    world.step(15); // 0.5 s
    bool sawFlare = false;
    for (const auto& c : f35.contacts(dasIdx)) {
        if (c.truth >= fsim::ms::kDecoyIds) {
            sawFlare = true;
            CHECK(c.kind == fsim::ms::ContactKind::Infrared);
        }
    }
    CHECK(sawFlare);

    // Agent permission boundary: Agent contacts redact truth to 0, Agent can search/track own EOTS,
    // and foreign Agent cannot cancel its optical task.
    const auto eotsIdx = findPayload(f35, "eots");
    powerToOperate(world, f35, eotsIdx);
    auto f35Agent = world.createAgent(f35, "f35-agent");
    auto rearAgent = world.createAgent(rearJet, "rear-agent");
    REQUIRE(f35Agent.valid());
    REQUIRE(rearAgent.valid());

    const auto agentDas = f35Agent.contacts(dasIdx);
    REQUIRE(!agentDas.empty());
    for (const auto& c : agentDas) {
        CHECK(c.truth == 0);
    }

    fsim::ms::PayloadTask agSearch;
    agSearch.payload = eotsIdx;
    agSearch.kind = fsim::ms::TaskKind::Search;
    agSearch.azimuthHalfWidthRad = 45.0 * kDeg;
    const auto act = f35Agent.submit(agSearch);
    REQUIRE(act.accepted());
    CHECK(!rearAgent.cancelMission(act.activity).accepted());
    CHECK(reportValue(f35Agent.payloadReport(eotsIdx), "searching") == 1.0);
}
