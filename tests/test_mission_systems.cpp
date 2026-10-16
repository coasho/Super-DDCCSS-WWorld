// Mission Systems (ADR-30, MS-1): the payload cards and the aircraft's fits -
// every card and fit in the source tree read clean, each parameter with its
// basis and source - a vehicle's payloads and capabilities from its fit,
// honest discovery (what is carried but not modelled yet is unavailable,
// not implemented, with its stage), the payloads' states, warm-ups and
// built-in tests as activities, and their grants through the control calls.
#include "mode_flights.h"

#include "simulation/mission_systems/Cards.h"
#include "simulation/mission_systems/Host.h"

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace fsim;
using namespace fsim::modes;

namespace {

namespace fs = std::filesystem;

const fs::path kSource = fs::path(FSIM_TEST_JSBSIM_ROOT).parent_path().parent_path(); // (third_party/jsbsim -> the source tree)

/// A fit of one payload whose card has the warm-up and BIT times given.
std::shared_ptr<const ms::Fit> testFit(double warmUpS, double bitS) {
    auto card = std::make_shared<ms::Card>();
    card->id = "test-radar";
    card->designation = "TEST/APG-0";
    card->family = ms::Family::AirRadar;
    card->sources["s"] = ms::Source{"a test", "", "", ""};
    card->parameters["warm_up_s"] = ms::Parameter{warmUpS, "s", 0.0, ms::Basis::Assumed, "", "a test"};
    card->parameters["bit_s"] = ms::Parameter{bitS, "s", 0.0, ms::Basis::Assumed, "", "a test"};
    auto fit = std::make_shared<ms::Fit>();
    fit->aircraft = "test";
    fit->sources["s"] = ms::Source{"a test", "", "", ""};
    fit->payloads.push_back(ms::FitEntry{"radar", "test-radar", 1, "s", ""});
    fit->cards["test-radar"] = card;
    return fit;
}

const ms::ActivityRecord& record(const ms::Host& host, ms::ActivityId id) {
    const ms::ActivityRecord* a = host.activity(id);
    REQUIRE(a != nullptr);
    return *a;
}

} // namespace

TEST_CASE("mission systems: every payload card and every aircraft's fit in the source tree read clean", "[ms]") {
    const fs::path library = kSource / "assets" / "payloads";
    REQUIRE(fs::is_directory(library));
    std::size_t cards = 0;
    for (const auto& file : fs::directory_iterator(library)) {
        if (file.path().extension() != ".json") continue;
        std::vector<std::string> errors;
        const auto card = ms::readCard(file.path(), errors);
        INFO(file.path().string());
        for (const auto& e : errors) INFO(e);
        CHECK(card != nullptr);
        CHECK(errors.empty());
        ++cards;
    }
    CHECK(cards > 0);
    // every aircraft has a fit: its payloads, or a note saying why it carries none
    std::size_t fits = 0, aircraft = 0;
    for (const auto& dir : fs::directory_iterator(kSource / "aircraft")) {
        if (!dir.is_directory() || !fs::exists(dir.path() / (dir.path().filename().string() + ".xml"))) continue;
        ++aircraft;
        const fs::path file = dir.path() / "payloads.json";
        INFO(dir.path().string());
        CHECK(fs::is_regular_file(file));
        if (!fs::is_regular_file(file)) continue;
        std::vector<std::string> errors;
        const auto fit = ms::readFit(file, library, errors);
        INFO(file.string());
        for (const auto& e : errors) INFO(e);
        CHECK(fit != nullptr);
        CHECK(errors.empty());
        if (fit) {
            CHECK(fit->aircraft == dir.path().filename().string());
            CHECK(!fit->note.empty());
            for (const auto& p : fit->payloads) CHECK(!p.source.empty());
        }
        ++fits;
    }
    CHECK(fits > 0);
    CHECK(fits == aircraft);
}

TEST_CASE("mission systems: a card's parameters need a basis, an uncertainty and, unless assumed, a source", "[ms]") {
    const fs::path dir = fs::temp_directory_path() / "fsim_ms_cards";
    fs::create_directories(dir);
    const auto write = [&](const char* name, const char* text) {
        std::ofstream(dir / name) << text;
        std::vector<std::string> errors;
        ms::readCard(dir / name, errors);
        return errors;
    };
    CHECK(write("good.json", R"({"id":"good","designation":"G","family":"esm","sources":{"a":{"title":"t"}},
        "parameters":{"band_low_hz":{"value":2e9,"unit":"Hz","uncertainty":0,"basis":"published","source":"a"},
                      "warm_up_s":{"value":60,"unit":"s","uncertainty":30,"basis":"assumed","note":"no published figure"}}})")
              .empty());
    CHECK(!write("nosource.json", R"({"id":"nosource","designation":"N","family":"esm","sources":{"a":{"title":"t"}},
        "parameters":{"x":{"value":1,"unit":"s","uncertainty":0,"basis":"published","source":"b"}}})")
               .empty());
    CHECK(!write("nobasis.json", R"({"id":"nobasis","designation":"N","family":"esm","sources":{"a":{"title":"t"}},
        "parameters":{"x":{"value":1,"unit":"s","uncertainty":0}}})")
               .empty());
    CHECK(!write("nonote.json", R"({"id":"nonote","designation":"N","family":"esm","sources":{"a":{"title":"t"}},
        "parameters":{"x":{"value":1,"unit":"s","uncertainty":1,"basis":"assumed"}}})")
               .empty());
    CHECK(!write("family.json", R"({"id":"family","designation":"F","family":"radio","sources":{"a":{"title":"t"}}})").empty());
    CHECK(!write("wrongid.json", R"({"id":"other","designation":"W","family":"esm","sources":{"a":{"title":"t"}}})").empty());
    fs::remove_all(dir);
}

TEST_CASE("mission systems: the F-16C's payloads from its fit, discovered honestly", "[ms]") {
    session::World w(options("ms-f16c"));
    const auto v = wing(w, "f16c", 3000.0, 200.0);
    const ms::Host* host = w.missionSystems(v);
    REQUIRE(host != nullptr);
    const auto& payloads = host->payloads();
    REQUIRE(payloads.size() == 6);
    CHECK(std::string(payloads[0].name) == "radar");
    CHECK(std::string(payloads[0].designation) == "AN/APG-68(V)9");
    CHECK(payloads[0].family == ms::Family::AirRadar);
    CHECK(payloads[0].state == ms::PayloadState::Off);
    CHECK(std::string(payloads[4].name) == "stations");
    CHECK(payloads[4].count == 9);
    CHECK(std::string(payloads[5].designation) == "AN/ALQ-213"); // (its EW suite, from its own article)
    for (const auto& p : payloads) CHECK(std::string(p.source).size() > 0);

    // each payload's status capability first, then its family's
    const auto& caps = host->capabilities();
    const int status = host->find("fsim.ms.status:radar"), amti = host->find("fsim.ms.amti:radar");
    REQUIRE(status >= 0);
    REQUIRE(amti == status + 1);
    CHECK(caps[static_cast<std::size_t>(status)].support == control::Support::Supported);
    CHECK(caps[static_cast<std::size_t>(amti)].support == control::Support::Supported); // (its card describes the radar: MS-4)
    CHECK(caps[static_cast<std::size_t>(amti)].stage == 0);
    CHECK(host->find("fsim.ms.strike:stations") >= 0);
    const int esm = host->find("fsim.ms.esm:rwr");
    REQUIRE(esm >= 0);
    CHECK(caps[static_cast<std::size_t>(esm)].support == control::Support::Supported); // (its warning receiver: MS-9a)
    CHECK(caps[static_cast<std::size_t>(esm)].stage == 0);
    const int ea = host->find("fsim.ms.ea:cmds");
    REQUIRE(ea >= 0);
    CHECK(caps[static_cast<std::size_t>(ea)].support == control::Support::NotImplemented);
    CHECK(caps[static_cast<std::size_t>(ea)].stage == 9);

    // availability: the radar off; not implemented yet; a type the aircraft does not carry; nothing of the kind
    ms::CapabilityStatus s = w.missionCapabilityStatus(v, "fsim.ms.amti:radar");
    CHECK(s.availability == control::Availability::TemporarilyUnavailable);
    CHECK(s.reason == ms::Reason::PayloadOff);
    s = w.missionCapabilityStatus(v, "fsim.ms.ea:cmds");
    CHECK(s.availability == control::Availability::Unavailable);
    CHECK(s.reason == ms::Reason::NotImplemented);
    s = w.missionCapabilityStatus(v, "fsim.ms.sar:radar");
    CHECK(s.reason == ms::Reason::NotSupported);
    s = w.missionCapabilityStatus(v, "fsim.ms.teleport:radar");
    CHECK(s.reason == ms::Reason::UnknownCapability);
    // its status capability: available (it commands the payload's state)
    CHECK(w.missionCapabilityStatus(v, "fsim.ms.status:radar").availability == control::Availability::Available);

    // an aircraft without a fit carries nothing
    const auto c = wing(w, "c172x", 1500.0, 55.0, 1);
    CHECK(w.missionSystems(c)->payloads().empty());
    CHECK(w.missionCapabilityStatus(c, "fsim.ms.amti:radar").reason == ms::Reason::NotSupported);
}

TEST_CASE("mission systems: a payload warms up, operates, stands by, tests and fails, each change an activity", "[ms]") {
    ms::Host host;
    host.bind(7, testFit(3.0, 2.0));
    const ms::CommandOptions o;
    // off -> operate: through its warm-up
    ms::CommandResult r = host.submit(ms::StateCommand{0, ms::PayloadState::Operate}, o, 10.0);
    REQUIRE(r.accepted());
    CHECK(ms::isMissionActivity(r.activity));
    CHECK(static_cast<std::uint32_t>(r.activity >> 32) == 7);
    CHECK(host.payloads()[0].state == ms::PayloadState::Starting);
    // (its AMTI capability is carried, not modelled yet: unavailable for that first, whatever the payload's state)
    CHECK(host.status(1).reason == ms::Reason::NotImplemented);
    CHECK(host.status(0).availability == control::Availability::Available);
    host.step(11.5);
    CHECK(record(host, r.activity).percent == 50.0);
    host.step(13.0);
    CHECK(host.payloads()[0].state == ms::PayloadState::Operate);
    CHECK(record(host, r.activity).state == control::ActivityState::Completed);
    CHECK(record(host, r.activity).reason == ms::Reason::GoalReached);
    // the same state again: refused
    CHECK(host.submit(ms::StateCommand{0, ms::PayloadState::Operate}, o, 13.0).reason == ms::Reason::AlreadyInState);
    // standby at once
    r = host.submit(ms::StateCommand{0, ms::PayloadState::Standby}, o, 14.0);
    REQUIRE(r.accepted());
    CHECK(host.payloads()[0].state == ms::PayloadState::Standby);
    CHECK(record(host, r.activity).state == control::ActivityState::Completed);
    // a BIT: unavailable meanwhile, back to standby after it
    r = host.submit(ms::BitCommand{0}, o, 20.0);
    REQUIRE(r.accepted());
    CHECK(host.payloads()[0].state == ms::PayloadState::Bit);
    CHECK(host.submit(ms::BitCommand{0}, o, 20.5).reason == ms::Reason::BitInProgress);
    host.step(22.0);
    CHECK(host.payloads()[0].state == ms::PayloadState::Standby);
    CHECK(record(host, r.activity).reason == ms::Reason::GoalReached);
    // a warm-up canceled: off again
    host.submit(ms::StateCommand{0, ms::PayloadState::Off}, o, 30.0);
    r = host.submit(ms::StateCommand{0, ms::PayloadState::Operate}, o, 31.0);
    REQUIRE(host.cancel({}, r.activity, 32.0).status == control::CommandStatus::Canceled);
    CHECK(host.payloads()[0].state == ms::PayloadState::Off);
    CHECK(record(host, r.activity).reason == ms::Reason::Requested);
    // a newer state command supersedes a warm-up
    const auto first = host.submit(ms::StateCommand{0, ms::PayloadState::Standby}, o, 40.0).activity;
    const auto second = host.submit(ms::StateCommand{0, ms::PayloadState::Operate}, o, 41.0).activity;
    CHECK(record(host, first).reason == ms::Reason::Superseded);
    host.step(43.0);
    CHECK(host.payloads()[0].state == ms::PayloadState::Operate);
    CHECK(record(host, second).state == control::ActivityState::Completed);
    // a failure: faulted, its live activity failed; repaired, off
    r = host.submit(ms::BitCommand{0}, o, 50.0);
    REQUIRE(host.setFailed(0, true, 50.5) == ms::Reason::None);
    CHECK(record(host, r.activity).state == control::ActivityState::Failed);
    CHECK(host.status(0).availability == control::Availability::Faulted);
    CHECK(host.submit(ms::StateCommand{0, ms::PayloadState::Operate}, o, 51.0).reason == ms::Reason::PayloadFailed);
    host.setFailed(0, false, 52.0);
    CHECK(host.payloads()[0].state == ms::PayloadState::Off);
    // unknown payloads and states
    CHECK(host.submit(ms::StateCommand{3, ms::PayloadState::Operate}, o, 60.0).reason == ms::Reason::UnknownPayload);
    CHECK(host.submit(ms::StateCommand{0, ms::PayloadState::Bit}, o, 60.0).reason == ms::Reason::InvalidParameter);
}

TEST_CASE("mission systems: grants through the control calls, by a capability's instance id", "[ms]") {
    session::World w(options("ms-grants"));
    const auto v = wing(w, "f16c", 3000.0, 200.0);
    const char* status = "fsim.ms.status:radar";
    const ms::StateCommand on{0, ms::PayloadState::Standby};
    REQUIRE(w.setControlMode(v, control::ControlMode::Granted) == control::Reason::None);
    CHECK(w.submit(v, on).reason == ms::Reason::NotGranted);
    CHECK(w.requestControl(v, status, 2) == control::Reason::None);
    CHECK(w.controlStatus(v, status).granted);
    CHECK(w.controlStatus(v, status).holder == 2);
    CHECK(w.requestControl(v, status, 3) == control::Reason::AuthorityHeld);
    ms::CommandOptions o;
    o.controller = 3;
    CHECK(w.submit(v, on, o).reason == ms::Reason::AuthorityHeld);
    o.controller = 2;
    REQUIRE(w.submit(v, on, o).accepted());
    CHECK(w.missionSystems(v)->payloads()[0].state == ms::PayloadState::Standby);
    // a capability carried but not modelled: no grant for it yet
    CHECK(w.requestControl(v, "fsim.ms.ea:cmds", 2) == control::Reason::NotImplemented);
    CHECK(w.requestControl(v, "fsim.ms.sar:radar", 2) == control::Reason::NotSupported);
    // released: the grant ends
    CHECK(w.releaseControl(v, status, 2) == control::Reason::None);
    CHECK(!w.controlStatus(v, status).granted);
    // the platform's restriction
    REQUIRE(w.setAvailability(v, status, control::Availability::Disabled) == control::Reason::None);
    CHECK(w.missionCapabilityStatus(v, status).availability == control::Availability::Disabled);
    o.source = control::Source::Override;
    CHECK(w.submit(v, ms::StateCommand{0, ms::PayloadState::Off}, o).reason == ms::Reason::Restricted);
}
