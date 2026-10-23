// Stores and weapon employment (ADR-30, MS-7a): an F-16C's stations from its
// store-station card, its default loadout (four Mk 82s), loads the stations
// cannot take refused, and A-GRA's Weapon Employment sequences as SDK calls -
// strike with release consent (asked for, given with the command, refused),
// releases a ripple interval apart as weapon entities at their stations,
// abort, the master arm, standard and emergency jettison.
#include "mode_flights.h"

#include "simulation/mission_systems/Cards.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <set>
#include <string>
#include <vector>

using namespace fsim;
using namespace fsim::modes;
using Catch::Matchers::WithinAbs;

namespace {

namespace fs = std::filesystem;
const fs::path kSource = fs::path(FSIM_TEST_JSBSIM_ROOT).parent_path().parent_path();

/// What a loadout hangs on the aircraft, kg: each store's mass, a tank's with the fuel it is loaded with.
double loadoutKg(const ms::Fit& fit, const ms::Loadout& l) {
    double kg = 0.0;
    for (const auto& it : l.items)
        if (const auto card = fit.stores.find(it.store); card != fit.stores.end())
            kg += it.count * (card->second->value("mass_kg", 0.0) + card->second->value("fuel_kg", 0.0));
    return kg;
}

std::uint16_t stationsOf(const session::World& w, std::uint32_t id) {
    const ms::Host* host = w.missionSystems(id);
    REQUIRE(host != nullptr);
    for (const auto& p : host->payloads())
        if (p.family == ms::Family::StoreStation) return p.index;
    FAIL("no store station payload");
    return 0;
}

void power(session::World& w, std::uint32_t id, std::uint16_t payload, ms::PayloadState state) {
    ms::StateCommand c;
    c.payload = payload;
    c.state = state;
    const auto r = w.submit(id, c);
    REQUIRE((r.accepted() || r.reason == ms::Reason::AlreadyInState));
    for (int i = 0; i < 20 && w.missionSystems(id)->payloads()[payload].state != state; ++i) w.step();
    REQUIRE(w.missionSystems(id)->payloads()[payload].state == state);
}

int count(const std::vector<ms::StoreInfo>& stores, ms::StoreStatus status) {
    return static_cast<int>(std::count_if(stores.begin(), stores.end(), [&](const ms::StoreInfo& s) { return s.status == status; }));
}

/// A real target near the launcher's east-facing nose, without advancing either flight model.
std::uint32_t launchTarget(session::World& w, std::uint32_t launcher, double angleDeg) {
    const auto* own = w.vehicleState(launcher);
    REQUIRE(own != nullptr);
    session::VehicleSpec target;
    target.name = "launch-target-" + std::to_string(angleDeg);
    target.type = "jsbsim:f16c";
    target.initial.latitudeDeg = own->latitudeRad / kDeg + 1000.0 * std::sin(angleDeg * kDeg) / kEarthM / kDeg;
    target.initial.longitudeDeg = own->longitudeRad / kDeg +
        1000.0 * std::cos(angleDeg * kDeg) / (kEarthM * std::cos(own->latitudeRad)) / kDeg;
    target.initial.altitudeMslM = own->altitudeMslM;
    target.initial.headingDeg = 90.0;
    target.initial.airspeedTrueMs = 220.0;
    const auto id = w.createVehicle(target);
    REQUIRE(id != 0);
    return id;
}

/// A refused launch must not start preparation, create an activity, or publish combat events.
void launchRejectedUnchanged(session::World& w, std::uint32_t launcher, const ms::StrikeCommand& strike,
                             ms::Reason expected) {
    const auto* host = w.missionSystems(launcher);
    REQUIRE(host != nullptr);
    const auto before = w.stores(launcher);
    const auto revision = host->storeRevision();
    REQUIRE(host->activities().empty());
    REQUIRE_FALSE(host->releasing());
    REQUIRE(w.weapons().empty());
    REQUIRE(w.engagementEvents().empty());
    REQUIRE(w.takeEngagementEvents().empty());
    ms::CommandOptions command;
    command.commandId = 713;
    const auto result = w.submit(launcher, strike, command);
    CHECK(result.status == control::CommandStatus::Rejected);
    CHECK(result.reason == expected);
    CHECK(result.activity == 0);
    CHECK(result.commandId == command.commandId);
    CHECK(host->activities().empty());
    CHECK_FALSE(host->releasing());
    CHECK(host->storeRevision() == revision);
    CHECK(w.weapons().empty());
    CHECK(w.engagementEvents().empty());
    CHECK(w.takeEngagementEvents().empty());
    const auto after = w.stores(launcher);
    REQUIRE(after.size() == before.size());
    for (std::size_t i = 0; i < after.size(); ++i) {
        CHECK(after[i].index == before[i].index);
        CHECK(after[i].payload == before[i].payload);
        CHECK(after[i].station == before[i].station);
        CHECK(std::string(after[i].store) == before[i].store);
        CHECK(after[i].status == before[i].status);
        CHECK(after[i].massKg == before[i].massKg);
        CHECK(after[i].fuelKg == before[i].fuelKg);
        CHECK(after[i].weapon == before[i].weapon);
        CHECK(after[i].bayDoors == before[i].bayDoors);
    }
}

} // namespace

TEST_CASE("stores: the F-16C's stations, its store cards and loadouts read clean", "[ms][stores]") {
    std::vector<std::string> errors;
    const auto fit = ms::readFit(kSource / "aircraft" / "f16c" / "payloads.json", kSource / "assets" / "payloads", errors);
    for (const auto& e : errors) UNSCOPED_INFO(e);
    CHECK(errors.empty());
    REQUIRE(fit);
    const auto card = fit->cards.find("f16-store-station");
    REQUIRE(card != fit->cards.end());
    REQUIRE(card->second->stations.size() == 9);
    for (int n = 1; n <= 9; ++n) {
        const ms::Station* s = card->second->station(n);
        REQUIRE(s != nullptr);
        CHECK(s->positionM[1] * (n - 5) >= 0.0); // (1 to 4 left, 5 the centreline, 6 to 9 right)
        CHECK(s->maxKg > 0.0);
    }
    REQUIRE(fit->stores.count("mk82"));
    CHECK(fit->stores.at("mk82")->model == "jsbsim:mk82");
    REQUIRE(fit->loadout("mk82 x4"));
    CHECK(fit->loadout("mk82 x4")->items.size() == 4);
    REQUIRE(fit->loadout("air superiority"));
    CHECK(fit->loadout("air superiority")->items.size() == 6);

    // every store card in the library reads clean
    std::vector<std::string> storeErrors;
    const auto library = ms::readStoreLibrary(kSource / "assets" / "stores", storeErrors);
    for (const auto& e : storeErrors) UNSCOPED_INFO(e);
    CHECK(storeErrors.empty());
    CHECK(!library.empty());
}

TEST_CASE("stores: every aircraft with stations loads each of its loadouts - its stores as the loadout says", "[ms][stores]") {
    // MS-7b: each fit's loadouts load into a vehicle of its type with nothing refused, the inventory the loadout's
    // (each store on its station) and the flight model's mass up by the stores' (each station's point mass mapped)
    session::World w(options("stores-fleet"));
    int aircraft = 0, loadouts = 0;
    for (const auto& dir : fs::directory_iterator(kSource / "aircraft")) {
        std::vector<std::string> errors;
        const auto fit = ms::readFit(dir.path() / "payloads.json", kSource / "assets" / "payloads", errors);
        if (!fit || fit->loadouts.empty()) continue;
        const std::string type = dir.path().filename().string();
        INFO(type);
        for (const auto& e : errors) UNSCOPED_INFO(e);
        CHECK(errors.empty());
        // (with the fuel that leaves room for its heaviest loadout under its maximum take-off weight: an A-10C's three
        // ferry tanks and a C-17A's eighteen pallets are refused with the tanks full)
        double heaviest = 0.0;
        for (const auto& l : fit->loadouts) heaviest = std::max(heaviest, loadoutKg(*fit, l));
        const auto id = wing(w, type, 5000.0, 150.0, aircraft, fuelWithRoomFor(w, type, heaviest));
        REQUIRE(id != 0);
        ++aircraft;
        for (const auto& l : fit->loadouts) {
            INFO(l.name);
            // (each loadout onto empty stations; its mass against theirs, the flight model's after a step)
            for (const auto& s : w.stores(id)) REQUIRE(w.loadStore(id, s.payload, s.station, "") == ms::Reason::None);
            w.step();
            const double emptyKg = w.massKg(id);
            CHECK(w.loadLoadout(id, l.name) == ms::Reason::None);
            w.step();
            std::size_t expected = 0;
            double expectedKg = 0.0;
            for (const auto& it : l.items) {
                REQUIRE(fit->stores.count(it.store));
                expected += static_cast<std::size_t>(it.count);
                const auto& card = *fit->stores.at(it.store);
                expectedKg += it.count * (card.value("mass_kg", 0.0) + card.value("fuel_kg", 0.0)); // (a tank loaded full)
            }
            const auto stores = w.stores(id);
            CHECK(stores.size() == expected);
            for (const auto& s : stores) { // (each station holds the loadout's store)
                const auto item = std::find_if(l.items.begin(), l.items.end(), [&](const auto& it) { return it.station == s.station; });
                REQUIRE(item != l.items.end());
                CHECK(item->store == s.store);
            }
            // (the stations' point masses carry them: the flight model's mass up by theirs, 1 kg for a step's fuel)
            CHECK_THAT(w.massKg(id) - emptyKg, WithinAbs(expectedKg, 1.0));
            ++loadouts;
        }
    }
    CHECK(aircraft >= 17);
    CHECK(loadouts >= 18);
}

TEST_CASE("stores: an F-16C's inventory - empty as created, its loadout, loads its stations cannot take refused", "[ms][stores]") {
    session::World w(options("stores-inventory"));
    const auto f16 = wing(w, "f16c", 5000.0, 220.0);
    const std::uint16_t st = stationsOf(w, f16);
    CHECK(w.stores(f16).empty()); // (an aircraft starts with its stations empty)
    CHECK(w.loadLoadout(f16, "no such loadout") == ms::Reason::InvalidParameter);
    REQUIRE(w.loadLoadout(f16, "mk82 x4") == ms::Reason::None);
    auto stores = w.stores(f16);
    REQUIRE(stores.size() == 4);
    std::vector<int> stations;
    for (const auto& s : stores) {
        stations.push_back(s.station);
        CHECK(std::string(s.store) == "mk82");
        CHECK(std::string(s.carriage) == "ejector");
        CHECK(s.flyable);
        CHECK(s.status == ms::StoreStatus::Loaded);
        CHECK_THAT(s.massKg, WithinAbs(226.8, 0.1));
    }
    CHECK(stations == std::vector<int>{3, 4, 6, 7});
    // Native engagements and advisory launch zones exist; calibrated full-fidelity acceptance remains partial.
    const ms::Host* host = w.missionSystems(f16);
    int strike = -1;
    for (const auto& c : host->capabilities())
        if (c.payload == st && c.type == ms::CapabilityType::Strike) strike = c.index;
    REQUIRE(strike >= 0);
    CHECK(host->capabilities()[static_cast<std::size_t>(strike)].support == control::Support::Partial);

    CHECK(w.loadStore(f16, st, 1, "mk82") == ms::Reason::StationCannotTake); // (a wingtip rail takes missiles)
    CHECK(w.loadStore(f16, st, 3, "mk82", 2) == ms::Reason::StationCannotTake); // (one store a station)
    CHECK(w.loadStore(f16, st, 3, "no-such-store") == ms::Reason::InvalidParameter);
    CHECK(w.loadStore(f16, st, 12, "mk82") == ms::Reason::InvalidParameter);
    CHECK(w.loadStore(f16, st, 5, "mk82") == ms::Reason::None); // (the centreline takes a bomb)
    CHECK(w.stores(f16).size() == 5);
    CHECK(w.loadStore(f16, st, 5, "") == ms::Reason::None);     // (unloaded)
    CHECK(w.stores(f16).size() == 4);

    // A missile on a wingtip rail has its native flight model and can enter release preparation.
    REQUIRE(w.loadStore(f16, st, 1, "aim-120c") == ms::Reason::None);
    REQUIRE(w.loadStore(f16, st, 9, "aim-9x") == ms::Reason::None);
    CHECK(w.loadStore(f16, st, 4, "aim-120c") == ms::Reason::StationCannotTake); // (an inboard station: bombs and tanks)
    stores = w.stores(f16);
    REQUIRE(stores.size() == 6);
    CHECK(stores.front().station == 1);
    CHECK(std::string(stores.front().carriage) == "rail");
    CHECK(std::string(stores.front().storeClass) == "aam");
    CHECK(stores.front().flyable);
    CHECK_THAT(stores.front().massKg, WithinAbs(161.5, 0.1));
    w.step();
    ms::StateCommand arm;
    arm.payload = st;
    arm.state = ms::PayloadState::Operate;
    REQUIRE(w.submit(f16, arm).accepted());
    w.step();
    ms::StrikeCommand missile;
    missile.payload = st;
    missile.store = "aim-120c";
    CHECK(w.submit(f16, missile).accepted());
}

TEST_CASE("stores: launch target and FOV refusals precede host admission without changing preparation",
          "[ms][stores][world][launch-policy]") {
    session::World w(options("stores-launch-policy"));
    const auto launcher = wing(w, "f16c", 5000.0, 220.0);
    const auto stations = stationsOf(w, launcher);
    REQUIRE(w.loadStore(launcher, stations, 1, "aim-120c") == ms::Reason::None);
    REQUIRE(w.loadStore(launcher, stations, 9, "r-60m") == ms::Reason::None);
    const auto betweenDefaults = launchTarget(w, launcher, 40.0);
    const auto behind = launchTarget(w, launcher, 180.0);
    ms::StrikeCommand strike;
    strike.payload = stations;
    strike.target = betweenDefaults;

    // The R-60M's 20-degree field cannot see the 40-degree target; the radar
    // default is 55. (The AIM-9X's published 90-degree field admits it.)
    strike.store = "r-60m";
    launchRejectedUnchanged(w, launcher, strike, ms::Reason::OutOfReach);
    strike.store = "aim-120c";
    launchRejectedUnchanged(w, launcher, strike, ms::Reason::PayloadOff);

    // Explicit indices override the named type. Every selected AAM must meet
    // its own envelope, including a later IR store in a mixed selection.
    strike.stores = {1};
    launchRejectedUnchanged(w, launcher, strike, ms::Reason::OutOfReach);
    strike.store = "r-60m";
    strike.stores = {1, 0};
    launchRejectedUnchanged(w, launcher, strike, ms::Reason::OutOfReach);
    strike.stores.clear();
    strike.store.clear();
    strike.quantity = 2;
    launchRejectedUnchanged(w, launcher, strike, ms::Reason::OutOfReach);

    strike.store = "aim-120c";
    strike.quantity = 1; // The following refusal cases select the one installed AIM-120C.
    strike.target = 0xfffffffeu;
    launchRejectedUnchanged(w, launcher, strike, ms::Reason::UnknownTrack);
    strike.target = behind;
    launchRejectedUnchanged(w, launcher, strike, ms::Reason::OutOfReach);
    strike.target = launcher; // coincident: acos(0 / max(norm, 1)) is 90 degrees
    launchRejectedUnchanged(w, launcher, strike, ms::Reason::OutOfReach);
    strike.target = 0; // no target, or no selected AAM: host admission remains the next owner
    launchRejectedUnchanged(w, launcher, strike, ms::Reason::PayloadOff);
    strike.target = 0xfffffffeu;
    strike.store = "no-selected-store";
    launchRejectedUnchanged(w, launcher, strike, ms::Reason::PayloadOff);
}

TEST_CASE("stores: SAR launch FOV precedes illumination refusal without starting preparation",
          "[ms][stores][world][launch-policy]") {
    session::World w(options("stores-launch-illumination"));
    const auto launcher = wing(w, "fa18c", 5000.0, 220.0);
    const auto stations = stationsOf(w, launcher);
    REQUIRE(w.loadStore(launcher, stations, 4, "aim-7m") == ms::Reason::None);
    ms::StrikeCommand strike;
    strike.payload = stations;
    strike.store = "aim-7m";
    strike.target = launchTarget(w, launcher, 0.0); // a real entity supplies no radar designation
    launchRejectedUnchanged(w, launcher, strike, ms::Reason::UnknownTrack);
    strike.target = launchTarget(w, launcher, 180.0);
    launchRejectedUnchanged(w, launcher, strike, ms::Reason::OutOfReach);
    strike.target = 0xfffffffeu;
    launchRejectedUnchanged(w, launcher, strike, ms::Reason::UnknownTrack);
}

TEST_CASE("stores: the stores' mass is the flight model's - four Mk 82s on, then jettisoned", "[ms][stores]") {
    // two F-16Cs side by side, one with its stations emptied: the loaded one heavier by the four Mk 82s (both burning
    // the same fuel), then as light once they are jettisoned
    session::World w(options("stores-mass"));
    const auto loaded = wing(w, "f16c", 5000.0, 220.0);
    const auto clean = wing(w, "f16c", 5000.0, 220.0, 1);
    const std::uint16_t st = stationsOf(w, clean);
    REQUIRE(w.loadLoadout(loaded, "mk82 x4") == ms::Reason::None);
    w.step(stepsFor(w, 1.0));
    CHECK_THAT(w.massKg(loaded) - w.massKg(clean), WithinAbs(4 * 226.8, 1.0));
    ms::JettisonCommand all;
    all.payload = st;
    all.emergency = true;
    REQUIRE(w.submit(loaded, all).accepted());
    w.step(stepsFor(w, 1.0));
    CHECK_THAT(w.massKg(loaded) - w.massKg(clean), WithinAbs(0.0, 1.0));
}

TEST_CASE("stores: a strike asks for release consent, then releases its stores a ripple apart as weapons at their stations", "[ms][stores]") {
    session::World w(options("stores-strike"));
    const auto f16 = wing(w, "f16c", 5000.0, 220.0);
    const std::uint16_t st = stationsOf(w, f16);
    REQUIRE(w.loadLoadout(f16, "mk82 x4") == ms::Reason::None);
    w.step(stepsFor(w, 1.0));

    ms::StrikeCommand strike;
    strike.payload = st;
    strike.store = "mk82";
    strike.quantity = 2;
    strike.rippleS = 0.5;
    // the master arm off: refused
    CHECK(w.submit(f16, strike).reason == ms::Reason::PayloadOff);
    power(w, f16, st, ms::PayloadState::Operate);
    const auto r = w.submit(f16, strike);
    REQUIRE(r.accepted());
    // it waits for consent (nothing leaves)
    w.step(stepsFor(w, 2.0));
    auto status = w.strikeStatus(r.activity);
    REQUIRE(status);
    CHECK(status->phase == ms::StrikePhase::AwaitingConsent);
    CHECK(w.weapons().empty());
    CHECK(count(w.stores(f16), ms::StoreStatus::Ready) == 2);

    const sim::VehicleState before = *w.vehicleState(f16);
    REQUIRE(w.strikeConsent(r.activity, true) == ms::Reason::None);
    CHECK(w.strikeConsent(r.activity, true) == ms::Reason::NotAwaitingConsent);
    w.step(stepsFor(w, 2.0));
    const auto weapons = w.weapons();
    REQUIRE(weapons.size() == 2);
    CHECK(weapons[0].store == "mk82");
    CHECK(weapons[0].activity == r.activity);
    CHECK_FALSE(weapons[0].jettisoned);
    CHECK_THAT(weapons[1].launchTimeS - weapons[0].launchTimeS, WithinAbs(0.5, 0.05)); // (the ripple)
    CHECK(weapons[0].station == 3);
    CHECK(weapons[1].station == 4);
    status = w.strikeStatus(r.activity);
    REQUIRE(status);
    CHECK(status->weapons.size() == 2);
    const auto* a = w.missionSystems(f16)->activity(r.activity);
    REQUIRE(a != nullptr);
    CHECK(a->state == control::ActivityState::Completed);
    CHECK(count(w.stores(f16), ms::StoreStatus::Released) == 2);
    CHECK(count(w.stores(f16), ms::StoreStatus::Loaded) == 2);
    (void)before;

    // the next strike takes the two left; a third has none
    strike.consent = true; // (given with the command)
    const auto r2 = w.submit(f16, strike);
    REQUIRE(r2.accepted());
    w.step(stepsFor(w, 2.0));
    CHECK(w.weapons().size() == 4);
    strike.quantity = 1;
    CHECK(w.submit(f16, strike).reason == ms::Reason::NoStore);
}

TEST_CASE("stores: abort, consent refused, and the master arm off end a strike with its stores still on", "[ms][stores]") {
    session::World w(options("stores-abort"));
    const auto f16 = wing(w, "f16c", 5000.0, 220.0);
    const std::uint16_t st = stationsOf(w, f16);
    REQUIRE(w.loadLoadout(f16, "mk82 x4") == ms::Reason::None);
    power(w, f16, st, ms::PayloadState::Operate);
    ms::StrikeCommand strike;
    strike.payload = st;
    strike.quantity = 2;

    // abort: the activity canceled before its release
    auto r = w.submit(f16, strike);
    REQUIRE(r.accepted());
    w.step(stepsFor(w, 0.5));
    CHECK(w.cancelMission(f16, control::Caller{control::Source::Policy, 0}, r.activity).status == control::CommandStatus::Canceled);
    w.step(stepsFor(w, 1.0));
    CHECK(w.weapons().empty());
    CHECK(count(w.stores(f16), ms::StoreStatus::Loaded) == 4);

    // consent refused: it fails
    r = w.submit(f16, strike);
    REQUIRE(r.accepted());
    w.step();
    CHECK(w.strikeConsent(r.activity, false) == ms::Reason::None);
    const auto* a = w.missionSystems(f16)->activity(r.activity);
    REQUIRE(a != nullptr);
    CHECK(a->state == control::ActivityState::Failed);
    CHECK(a->reason == ms::Reason::ConsentDenied);
    CHECK(count(w.stores(f16), ms::StoreStatus::Loaded) == 4);

    // the master arm off: canceled
    r = w.submit(f16, strike);
    REQUIRE(r.accepted());
    w.step();
    power(w, f16, st, ms::PayloadState::Standby);
    a = w.missionSystems(f16)->activity(r.activity);
    REQUIRE(a != nullptr);
    CHECK(a->state == control::ActivityState::Canceled);
    CHECK(a->reason == ms::Reason::PayloadStandby);
    CHECK(w.weapons().empty());
}

TEST_CASE("stores: jettison - standard with the stations powered, emergency in any state - releases them unarmed", "[ms][stores]") {
    session::World w(options("stores-jettison"));
    const auto f16 = wing(w, "f16c", 5000.0, 220.0);
    const std::uint16_t st = stationsOf(w, f16);
    REQUIRE(w.loadLoadout(f16, "mk82 x4") == ms::Reason::None);
    ms::JettisonCommand j;
    j.payload = st;
    j.stores = {0};
    // standard: needs the stations powered
    CHECK(w.submit(f16, j).reason == ms::Reason::PayloadOff);
    power(w, f16, st, ms::PayloadState::Standby);
    auto r = w.submit(f16, j);
    REQUIRE(r.accepted());
    w.step(stepsFor(w, 1.0));
    auto weapons = w.weapons();
    REQUIRE(weapons.size() == 1);
    CHECK(weapons[0].jettisoned);
    CHECK(weapons[0].station == 3);
    CHECK(w.stores(f16)[0].status == ms::StoreStatus::Jettisoned);

    // emergency, the stations off: every store left goes together
    power(w, f16, st, ms::PayloadState::Off);
    ms::JettisonCommand all;
    all.payload = st;
    all.emergency = true;
    r = w.submit(f16, all);
    REQUIRE(r.accepted());
    w.step(stepsFor(w, 0.5));
    weapons = w.weapons();
    REQUIRE(weapons.size() == 4);
    for (const auto& wr : weapons) CHECK(wr.jettisoned);
    CHECK_THAT(weapons[3].launchTimeS - weapons[1].launchTimeS, WithinAbs(0.0, 1e-9)); // (together)
    CHECK(count(w.stores(f16), ms::StoreStatus::Jettisoned) == 4);
    CHECK(w.submit(f16, all).reason == ms::Reason::NoStore);
    // the weapons fall away below the aircraft
    w.step(stepsFor(w, 2.0));
    const sim::VehicleState& own = *w.vehicleState(f16);
    for (const auto& wr : w.weapons())
        if (const sim::VehicleState* s = w.vehicleState(wr.id)) CHECK(s->altitudeMslM < own.altitudeMslM - 5.0);
}

TEST_CASE("stores: a weapon bay's doors open before its store leaves, and shut after it", "[ms][stores]") {
    std::vector<std::string> errors;
    const auto read = ms::readFit(kSource / "aircraft" / "f22a" / "payloads.json", kSource / "assets" / "payloads", errors);
    REQUIRE(read);
    auto fit = std::make_shared<ms::Fit>(*read);
    auto card = std::make_shared<ms::StoreCard>(*fit->stores.at("aim-120c"));
    card->model = "jsbsim:mk82";
    fit->stores["aim-120c"] = card;
    ms::Host host;
    host.bind(1, fit);
    std::uint16_t st = 0;
    while (st < host.payloads().size() && host.payloads()[st].family != ms::Family::StoreStation) ++st;
    REQUIRE(st < host.payloads().size());
    REQUIRE(host.loadLoadout("air superiority") == ms::Reason::None);
    const ms::CommandOptions o;
    double t = 0.0;
    REQUIRE(host.submit(ms::StateCommand{st, ms::PayloadState::Operate}, o, t).accepted());
    for (; t < 30.0 && host.payloads()[st].state != ms::PayloadState::Operate; t += 0.01) host.step(t);
    const auto doors = [&](int station) {
        for (const auto& s : host.stores())
            if (s.station == station) return s.bayDoors;
        return -1.0;
    };
    CHECK(doors(5) == 0.0); // main bay 2's (shut)
    ms::StrikeCommand strike;
    strike.payload = st;
    strike.store = "aim-120c";
    strike.quantity = 1;
    strike.consent = true;
    const auto r = host.submit(strike, o, t);
    REQUIRE(r.accepted());
    // it powers up, its bay's doors open over half a second, and only then does it leave
    double opened = -1.0, left = -1.0;
    int station = 0;
    for (; t < 60.0 && left < 0.0; t += 0.01) {
        host.step(t);
        for (const auto& s : host.stores())
            if (s.status == ms::StoreStatus::Ready) station = s.station;
        if (opened < 0.0 && station && doors(station) > 0.0) opened = t;
        const auto out = host.takeReleases();
        if (!out.empty()) {
            left = t;
            CHECK(doors(station) == 1.0);
            host.released(out[0].store, 99, t);
        }
    }
    REQUIRE(opened > 0.0);
    REQUIRE(left > 0.0);
    INFO("the doors from " << opened << " s, the missile at " << left << " s, from station " << station);
    CHECK(left - opened >= 0.45); // (their half second: not before)
    // the bay's other stations' doors are the same pair; another bay's stay shut
    for (const auto& s : host.stores())
        if (s.station >= 4 && s.station <= 9) CHECK(s.bayDoors == 1.0);
    CHECK(doors(3) == 0.0);
    // held open a moment, then shut
    for (const double until = t + 0.5; t < until; t += 0.01) host.step(t);
    CHECK(doors(station) == 1.0);
    for (const double until = t + 2.0; t < until; t += 0.01) host.step(t);
    CHECK(doors(station) == 0.0);
}

TEST_CASE("stores: everything a loadout carries, jettisoned, flies on as a vehicle of its own - as heavy as it was on its station",
          "[ms][stores][world]") {
    // docs/hangar.md, Mass accounting: what an aircraft carries beside its people, its built-in equipment and its small
    // items is a store with a JSBSim model of its own - a missile, a bomb, a tank, a pod, a pallet of cargo. Each loads,
    // is weighed on its station, and leaves as that model with the store's mass (a tank's fuel with it); the aircraft is
    // lighter by what left
    int aircraft = 0, flown = 0;
    std::set<std::string> classes;
    for (const auto& dir : fs::directory_iterator(kSource / "aircraft")) {
        std::vector<std::string> errors;
        const auto fit = ms::readFit(dir.path() / "payloads.json", kSource / "assets" / "payloads", errors);
        if (!fit || fit->loadouts.empty()) continue;
        const std::string type = dir.path().filename().string();
        INFO(type);
        for (const auto& e : errors) UNSCOPED_INFO(e);
        CHECK(errors.empty());
        session::World w(options(("stores-jettison-" + type).c_str()));
        int index = 0;
        for (const auto& l : fit->loadouts) {
            INFO(type << ": " << l.name);
            const auto id = wing(w, type, 5000.0, 200.0, index++, fuelWithRoomFor(w, type, loadoutKg(*fit, l)));
            REQUIRE(id != 0);
            ++aircraft;
            w.step();
            const double spawned = w.massKg(id);
            REQUIRE(w.loadLoadout(id, l.name) == ms::Reason::None);
            w.step();
            std::vector<double> carried;
            double loaded = 0.0;
            for (const auto& s : w.stores(id)) {
                const auto& card = *fit->stores.at(s.store);
                INFO(s.store);
                CHECK(!card.model.empty());                // (it has the model it leaves as)
                CHECK(s.jettisonable);
                CHECK_THAT(s.massKg, WithinAbs(card.value("mass_kg", 0.0) + card.value("fuel_kg", 0.0), card.storeClass == "tank" ? 1.0 : 1e-6)); // (a tank: less the step's draw)
                classes.insert(card.storeClass);
                carried.push_back(s.massKg);
                loaded += s.massKg;
            }
            REQUIRE(!carried.empty());
            CHECK_THAT(w.massKg(id) - spawned, WithinAbs(loaded, 1.0)); // (on: the flight model has what the stations carry)
            ms::JettisonCommand all;
            all.payload = stationsOf(w, id);
            all.emergency = true;
            const double before = w.massKg(id);
            REQUIRE(w.submit(id, all).accepted());
            w.step(stepsFor(w, 4.0)); // (a weapon bay's doors, a hold's ramp, open first, and its stores leave one after another)
            const auto stores = w.stores(id);
            REQUIRE(stores.size() == carried.size());
            double gone = 0.0;
            for (std::size_t k = 0; k < stores.size(); ++k) {
                INFO(stores[k].store);
                CHECK(stores[k].status == ms::StoreStatus::Jettisoned);
                REQUIRE(stores[k].weapon != 0);
                const double kg = w.massKg(stores[k].weapon);
                CHECK_THAT(kg, WithinAbs(carried[k], 0.1)); // (the vehicle it is now: as heavy as it was on its station)
                CHECK_FALSE(w.vehicleState(stores[k].weapon)->diverged);
                gone += kg;
                ++flown;
            }
            CHECK_THAT(before - w.massKg(id), WithinAbs(gone, 6.0)); // (and four seconds' burn)
            CHECK_THAT(w.massKg(id), WithinAbs(spawned, 8.0));       // (off: as it spawned)
        }
    }
    CHECK(aircraft >= 40);
    CHECK(flown >= 180);
    for (const char* c : {"aam", "tank", "pod", "cargo", "bomb"}) CHECK(classes.count(c) == 1);
}

TEST_CASE("stores: the Growler's jamming pods are stores - not part of its airframe, loaded on their stations beside its tanks", "[ms][stores]") {
    // (they were drawn and weighed in its design: a fixed load that could not leave it, and kept its stations from use)
    session::World w(options("stores-growler"));
    const auto id = wing(w, "ea18g", 5000.0, 200.0);
    REQUIRE(id != 0);
    w.step();
    const double spawned = w.massKg(id);
    CHECK(w.stores(id).empty());
    REQUIRE(w.loadLoadout(id, "jamming pods") == ms::Reason::None);
    REQUIRE(w.loadLoadout(id, "external tanks") == ms::Reason::None);
    w.step();
    std::string on;
    double pods = 0.0, tanks = 0.0;
    for (const auto& s : w.stores(id)) {
        on += std::to_string(s.station) + ":" + s.store + " ";
        (std::string(s.store) == "an-alq-99" ? pods : tanks) += s.massKg;
    }
    CHECK(on == "3:an-alq-99 4:fpu-11 6:an-alq-99 8:fpu-11 9:an-alq-99 ");
    CHECK_THAT(pods, WithinAbs(3 * 435.0, 1e-6));
    CHECK_THAT(w.massKg(id) - spawned, WithinAbs(pods + tanks, 1.0));
    // a pod's station takes a pod, and one store at a time
    CHECK(w.loadStore(id, stationsOf(w, id), 3, "fpu-11") == ms::Reason::StationCannotTake);
    // its drag while it is carried is the station's, not the airframe's: the clean aircraft's drag area less by three pods'
    std::vector<ms::Host::DragPoint> drag;
    w.missionSystems(id)->carriageDrag(drag);
    double area = 0.0;
    for (const auto& d : drag) area += d.areaM2;
    CHECK(area > 3 * 0.05);
}

TEST_CASE("stores: a transport's cargo is stores in its hold - released aft over the ramp, each a body of its own", "[ms][stores][world]") {
    session::World w(options("stores-cargo"));
    const auto id = wing(w, "c130j", 3000.0, 110.0);
    REQUIRE(id != 0);
    w.step();
    const double spawned = w.massKg(id);
    REQUIRE(w.loadLoadout(id, "airdrop") == ms::Reason::None);
    w.step();
    REQUIRE(w.stores(id).size() == 6);
    CHECK_THAT(w.massKg(id) - spawned, WithinAbs(6 * 998.0, 1.0));
    std::vector<ms::Host::DragPoint> drag;
    w.missionSystems(id)->carriageDrag(drag);
    CHECK(drag.empty()); // (in its hold: no drag of its own)
    ms::JettisonCommand drop;
    drop.payload = stationsOf(w, id);
    drop.emergency = true;
    REQUIRE(w.submit(id, drop).accepted());
    w.step(stepsFor(w, 3.0));
    const sim::VehicleState& carrier = *w.vehicleState(id);
    const double heading = carrier.eulerRad[2];
    int out = 0;
    for (const auto& s : w.stores(id)) {
        REQUIRE(s.weapon != 0);
        const sim::VehicleState& b = *w.vehicleState(s.weapon);
        CHECK_THAT(w.massKg(s.weapon), WithinAbs(998.0, 0.1));
        // behind the aircraft and below it: north and east from it, along its heading
        const double north = (b.latitudeRad - carrier.latitudeRad) * 6371000.0;
        const double east = (b.longitudeRad - carrier.longitudeRad) * 6371000.0 * std::cos(carrier.latitudeRad);
        const double along = north * std::cos(heading) + east * std::sin(heading);
        INFO("station " << s.station << ": " << along << " m along, " << b.altitudeMslM - carrier.altitudeMslM << " m up");
        CHECK(along < -15.0);
        CHECK(b.altitudeMslM < carrier.altitudeMslM - 5.0);
        CHECK_FALSE(b.diverged);
        ++out;
    }
    CHECK(out == 6);
    CHECK_THAT(w.massKg(id), WithinAbs(spawned, 3.0));
}
