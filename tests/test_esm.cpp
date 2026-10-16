// The warning receiver (ADR-30, MS-9a; docs/sdk/mission-systems.md): the power it takes from an emission - main
// beam, side lobe, pulsed or not - and, through the World, what an F-16C's receiver holds of another's radar: a
// search heard, a lock and an illumination a warning, its own radar blanked, an emitter gone forgotten; the same
// from the same seed.

#include "mode_flights.h"

#include "simulation/mission_systems/Model.h"
#include "simulation/mission_systems/esm/Receiver.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <string>
#include <vector>

using namespace fsim;
using namespace fsim::modes;

namespace {

/// A second F-16C `eastKm` east of the first, at its height, flying west at it.
std::uint32_t facing(session::World& w, double eastKm) {
    session::VehicleSpec s;
    s.name = "emitter";
    s.type = "jsbsim:f16c";
    s.initial.altitudeMslM = 5000.0;
    s.initial.headingDeg = 270.0;
    s.initial.airspeedTrueMs = 200.0;
    s.initial.longitudeDeg += eastKm * 1e3 / (kEarthM * std::cos(s.initial.latitudeDeg * kDeg)) / kDeg;
    const auto id = w.createVehicle(s);
    REQUIRE(id != 0);
    return id;
}

void operate(session::World& w, std::uint32_t id, std::uint16_t payload) {
    REQUIRE(w.submit(id, ms::StateCommand{payload, ms::PayloadState::Operate}).accepted());
    for (int i = 0; i < 600 && w.missionSystems(id)->payloads()[payload].state != ms::PayloadState::Operate; ++i) w.step();
    REQUIRE(w.missionSystems(id)->payloads()[payload].state == ms::PayloadState::Operate);
}

/// The receiver's one contact after an emitter 40 km ahead has searched for `seconds`.
ms::Contact heard(const char* name, double seconds) {
    session::World w(options(name));
    const auto own = wing(w, "f16c", 5000.0, 200.0);
    const auto emitter = facing(w, 40.0);
    operate(w, emitter, 0);
    operate(w, own, 1);
    ms::AmtiCommand search;
    search.azimuthHalfWidthRad = 30 * kDeg;
    REQUIRE(w.submit(emitter, search).accepted());
    w.step(stepsFor(w, seconds));
    const auto list = w.contacts(own, 1);
    REQUIRE(list.size() == 1);
    return list[0];
}

} // namespace

TEST_CASE("warning receiver: the power it takes from an emission - main beam, side lobe, pulsed or not", "[ms][esm]") {
    // an emitter and a receiver 50 km apart along the emitter's nose (its body axes: x at the receiver, y, z)
    sim::VehicleState tx{}, rx{};
    tx.positionEcef[0] = 6.4e6, rx.positionEcef[0] = 6.4e6, rx.positionEcef[1] = 50e3;
    const double axes[9] = {0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0}; // (its columns: the body's x, y and z in ECEF)
    for (int k = 0; k < 9; ++k) tx.rotationBodyToEcef[k] = axes[k];
    ms::Emission e;
    e.frequencyHz = 10e9;
    e.powerW = 1000.0, e.peakPowerW = 10000.0;
    e.peakGainDbi = 33.0, e.sidelobeDbi = -5.0;
    e.azimuthHalfWidthRad = 30 * kDeg, e.elevationHalfWidthRad = 5 * kDeg;
    e.state = &tx;
    const double loss = 20.0 * std::log10(4.0 * kPi * 50e3 * 10e9 / 299792458.0); // (146.4 dB)
    CHECK(std::abs(e.gainTowardsDbi(rx.positionEcef) - 33.0) < 1e-12);
    // 10 kW pulses, 70 dBm, through 33 dBi and the path: -43.4 dBm
    CHECK(std::abs(ms::esm::receivedDbm(e, rx, loss, 0.0) - (70.0 + 33.0 - loss)) < 1e-9);
    CHECK(std::abs(ms::esm::receivedDbm(e, rx, loss, 3.0) - (70.0 + 33.0 - loss + 3.0)) < 1e-9);
    // its scan elsewhere: a side lobe, 38 dB less
    e.azimuthCentreRad = 90 * kDeg;
    CHECK(std::abs(e.gainTowardsDbi(rx.positionEcef) + 5.0) < 1e-12);
    CHECK(std::abs(ms::esm::receivedDbm(e, rx, loss, 0.0) - (70.0 - 5.0 - loss)) < 1e-9);
    // all round in azimuth (a rotodome): its main beam again; but not above its beam's height
    e.allRound = true;
    CHECK(std::abs(e.gainTowardsDbi(rx.positionEcef) - 33.0) < 1e-12);
    e.elevationCentreRad = 20 * kDeg;
    CHECK(std::abs(e.gainTowardsDbi(rx.positionEcef) + 5.0) < 1e-12);
    // one that does not pulse: its average power
    e.elevationCentreRad = 0.0, e.peakPowerW = 0.0;
    CHECK(std::abs(ms::esm::receivedDbm(e, rx, loss, 0.0) - (60.0 + 33.0 - loss)) < 1e-9);
}

TEST_CASE("warning receiver through the World: a search heard, a lock and an illumination a warning, gone with the emitter", "[ms][esm]") {
    session::World w(options("esm"));
    const auto own = wing(w, "f16c", 5000.0, 200.0);
    const auto emitter = facing(w, 40.0);
    const ms::Host* host = w.missionSystems(own);
    REQUIRE(host != nullptr);
    REQUIRE(std::string(host->payloads()[1].name) == "rwr");
    const int esm = host->find("fsim.ms.esm:rwr");
    REQUIRE(esm >= 0);
    CHECK(host->capabilities()[static_cast<std::size_t>(esm)].support == control::Support::Supported);

    // the other's radar searching towards it; its own receiver off: nothing held
    operate(w, emitter, 0);
    ms::AmtiCommand search;
    search.azimuthHalfWidthRad = 30 * kDeg;
    REQUIRE(w.submit(emitter, search).accepted());
    w.step(stepsFor(w, 2.0));
    CHECK(w.contacts(own, 1).empty());

    // on: the emitter, ahead, searching - no warning
    operate(w, own, 1);
    w.step(stepsFor(w, 1.0));
    auto list = w.contacts(own, 1);
    REQUIRE(list.size() == 1);
    {
        const ms::Contact& c = list[0];
        CHECK(c.kind == ms::ContactKind::Emitter);
        CHECK(c.mode == ms::EmitterMode::Search);
        CHECK(!c.warning);
        CHECK(c.truth == emitter);
        CHECK(c.payload == 1);
        CHECK((c.id >> 48) == 1);
        CHECK(std::abs(c.azimuthRad) < 40 * kDeg);                // (ahead, within four of its 10 deg)
        CHECK(std::abs(c.bearingSigmaRad - 10 * kDeg) < 1e-12);
        CHECK(std::isnan(c.rangeM));                              // (a receiver measures none)
        CHECK(c.frequencyHz == w.radarStatus(emitter, 0).frequencyHz);
        // its main beam at 40 km: the pulses' 70 dBm and its 30-odd dBi less 144.5 dB of path
        CHECK(c.strengthDb > -50.0);
        CHECK(c.strengthDb < -30.0);
        CHECK(c.firstTimeS <= c.timeS);
    }
    CHECK(w.contacts(own, 0).empty()); // (a radar holds no such contacts)

    // its own radar on too: blanked, not a contact
    operate(w, own, 0);
    REQUIRE(w.submit(own, search).accepted());
    w.step(stepsFor(w, 1.0));
    list = w.contacts(own, 1);
    REQUIRE(list.size() == 1);
    CHECK(list[0].truth == emitter);
    const std::uint64_t first = list[0].id;

    // the emitter takes it in single-target track: a lock, a warning; lighting it: an illumination
    std::vector<ms::AmtiTrack> tracks;
    for (int i = 0; i < 90 && tracks.empty(); ++i) {
        w.step(stepsFor(w, 1.0));
        tracks = w.amtiTracks(emitter, 0);
    }
    REQUIRE(!tracks.empty());
    ms::AmtiCommand track;
    track.task = ms::AmtiTask::Track;
    track.mode = ms::AmtiMode::SingleTargetTrack;
    track.track = tracks[0].id;
    REQUIRE(w.submit(emitter, track).accepted());
    w.step(stepsFor(w, 2.0));
    list = w.contacts(own, 1);
    REQUIRE(list.size() == 1);
    CHECK(list[0].id == first); // (the same emitter: the same contact)
    CHECK(list[0].mode == ms::EmitterMode::Lock);
    CHECK(list[0].warning);
    track.illuminate = true;
    REQUIRE(w.submit(emitter, track).accepted());
    w.step(stepsFor(w, 2.0));
    list = w.contacts(own, 1);
    REQUIRE(list.size() == 1);
    CHECK(list[0].mode == ms::EmitterMode::Illumination);
    CHECK(list[0].warning);

    // the emitter off: forgotten once its memory (3 s) has gone by
    REQUIRE(w.submit(emitter, ms::StateCommand{0, ms::PayloadState::Standby}).accepted());
    w.step(stepsFor(w, 1.0));
    CHECK(w.contacts(own, 1).size() == 1);
    w.step(stepsFor(w, 3.0));
    CHECK(w.contacts(own, 1).empty());

    // its own receiver to standby and back: what it held is gone, and found again
    operate(w, emitter, 0);
    REQUIRE(w.submit(emitter, search).accepted());
    w.step(stepsFor(w, 1.0));
    REQUIRE(w.contacts(own, 1).size() == 1);
    REQUIRE(w.submit(own, ms::StateCommand{1, ms::PayloadState::Standby}).accepted());
    CHECK(w.contacts(own, 1).empty());
    operate(w, own, 1);
    w.step(stepsFor(w, 1.0));
    list = w.contacts(own, 1);
    REQUIRE(list.size() == 1);
    CHECK(list[0].id != first);
}

TEST_CASE("warning receiver: the same bearings from the same seed", "[ms][esm]") {
    const ms::Contact a = heard("esm-a", 5.0), b = heard("esm-b", 5.0);
    CHECK(a.azimuthRad == b.azimuthRad);
    CHECK(a.elevationRad == b.elevationRad);
    CHECK(a.strengthDb == b.strengthDb);
    CHECK(a.azimuthRad != 0.0); // (a measurement, with its error)
}
