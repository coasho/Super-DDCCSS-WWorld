// Equipment parity and permission-scoped reports through the public C++ SDK.
#include "fsim/Agent.h"
#include "fsim/MAService.h"
#include "fsim/World.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <string>
#include <utility>

using namespace fsim;

namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;

WorldOptions options(const char* name) {
    WorldOptions out;
    out.name = name;
    out.publish = false;
    out.workers = 1;
    out.pinWorkers = false;
    out.seed = 17;
    return out;
}

Vehicle aircraft(World& world, const char* name, double northKm = 0.0, double headingDeg = 0.0,
                 const char* type = "jsbsim:f16c") {
    VehicleSpec spec;
    spec.name = name;
    spec.type = type;
    spec.initial.latitudeDeg = 40.0 + northKm / 111.2;
    spec.initial.longitudeDeg = 0.0;
    spec.initial.altitudeMslM = 6000.0;
    spec.initial.headingDeg = headingDeg;
    spec.initial.airspeedTrueMs = 230.0;
    return world.createVehicle(spec);
}

std::uint16_t payloadOf(const Vehicle& vehicle, ms::Family family) {
    for (const auto& payload : vehicle.payloads())
        if (payload.family == family) return payload.index;
    return 0xFFFF;
}

void advance(World& world, double seconds) {
    world.step(static_cast<unsigned>(std::ceil(seconds / world.stepSeconds())));
}

void operate(World& world, Agent& agent, std::uint16_t payload) {
    REQUIRE(payload != 0xFFFF);
    const auto result = agent.submit(ms::StateCommand{payload, ms::PayloadState::Operate});
    REQUIRE((result.accepted() || result.reason == ms::Reason::AlreadyInState));
    for (unsigned step = 0; step < 1200; ++step) {
        for (const auto& state : agent.payloads())
            if (state.index == payload && state.state == ms::PayloadState::Operate) return;
        world.step();
    }
    FAIL("payload did not finish native startup");
}

void ownMeta(const ma::ReportMeta& meta, double now) {
    CHECK(meta.source == ma::Source::OwnPlatform);
    CHECK(meta.validity == ma::Validity::Valid);
    CHECK(meta.sampleTimeS == now);
    CHECK(meta.receiveTimeS == now);
    CHECK(meta.ageS == 0.0);
}

} // namespace

TEST_CASE("ma equipment: countermeasure parity preserves native inventory and shared cooldown", "[ma][equipment]") {
    World world(options("ma-equipment-countermeasures"));
    Vehicle platform = aircraft(world, "platform");
    Agent agent = world.createAgent(platform, "equipment");
    const auto countermeasurePayload = payloadOf(platform, ms::Family::Countermeasures);
    const auto off = agent.submit(ms::StateCommand{countermeasurePayload, ms::PayloadState::Off});
    REQUIRE((off.accepted() || off.reason == ms::Reason::AlreadyInState));
    world.step();
    CHECK(agent.dispense() == ms::Reason::PayloadOff);
    const auto initial = world.countermeasures(platform.id());
    CHECK(agent.countermeasures().chaff == initial.chaff);
    CHECK(agent.countermeasures().flare == initial.flare);
    const auto first = agent.reportCountermeasures();
    REQUIRE(first.availability == ma::Availability::Available);
    ownMeta(first.meta, world.time());
    CHECK(first.value.chaff == initial.chaff);
    CHECK(first.value.flare == initial.flare);

    operate(world, agent, countermeasurePayload);
    REQUIRE(agent.dispense() == ms::Reason::None);
    CHECK(world.dispense(platform.id(), true) == ms::Reason::StoreBusy);
    CHECK(agent.dispense(true) == ms::Reason::StoreBusy);
    CHECK(agent.countermeasures().chaff + 1 == initial.chaff);
    CHECK(agent.countermeasures().flare == initial.flare);
    advance(world, 0.36);
    REQUIRE(agent.dispense(true) == ms::Reason::None);
    const auto native = world.countermeasures(platform.id());
    const auto current = agent.reportCountermeasures();
    REQUIRE(current.availability == ma::Availability::Available);
    CHECK(current.value.chaff == native.chaff);
    CHECK(current.value.flare == native.flare);
    CHECK(current.value.chaff + 1 == initial.chaff);
    CHECK(current.value.flare + 1 == initial.flare);
    ownMeta(current.meta, world.time());
    CHECK(agent.reportCountermeasures().value.chaff == current.value.chaff);
    CHECK(agent.reportCountermeasures().value.flare == current.value.flare);
}

TEST_CASE("ma equipment: launch zones use live own designations and preserve native advisory values", "[ma][equipment]") {
    World world(options("ma-equipment-launch-zone"));
    Vehicle platform = aircraft(world, "platform");
    Vehicle opponent = aircraft(world, "opponent", 5.0, 180.0);
    Agent own = world.createAgent(platform, "own");
    Agent other = world.createAgent(opponent, "other");
    REQUIRE(platform.loadStore(payloadOf(platform, ms::Family::StoreStation), 1, "aim-120c") == ms::Reason::None);
    for (auto* agent : {&own, &other}) {
        control::HsaCommand hold;
        hold.headingRad = agent == &own ? 0.0 : 180.0 * kDeg;
        hold.speed = 230.0;
        hold.altitudeM = 6000.0;
        REQUIRE(agent->submit(hold).accepted());
    }
    const auto ownRadar = payloadOf(platform, ms::Family::AirRadar);
    const auto otherRadar = payloadOf(opponent, ms::Family::AirRadar);
    operate(world, own, ownRadar);
    operate(world, other, otherRadar);
    for (const auto pair : {std::pair<Agent*, std::uint16_t>{&own, ownRadar}, {&other, otherRadar}}) {
        ms::AmtiCommand search;
        search.payload = pair.second;
        search.azimuthHalfWidthRad = 5.0 * kDeg;
        search.bars = 1;
        search.rangeScaleM = 60000.0;
        REQUIRE(pair.first->submit(search).accepted());
    }
    std::uint64_t ownTrack = 0, otherTrack = 0;
    for (unsigned step = 0; step < 600 && (!ownTrack || !otherTrack); ++step) {
        world.step();
        for (const auto& track : own.amtiTracks(ownRadar))
            if (track.updates >= 3) ownTrack = track.id;
        for (const auto& track : other.amtiTracks(otherRadar))
            if (track.updates >= 3) otherTrack = track.id;
    }
    REQUIRE(ownTrack != 0);
    REQUIRE(otherTrack != 0);
    const auto target = own.designate(ownRadar, ownTrack);
    const auto foreign = other.designate(otherRadar, otherTrack);
    REQUIRE(target != 0);
    REQUIRE(foreign != 0);
    CHECK_FALSE(own.launchZone("aim-120c", opponent.id()));
    CHECK_FALSE(own.launchZone("aim-120c", foreign));
    CHECK_FALSE(own.launchZone("aim-120c", 0));
    const auto emptyStore = own.launchZone("", target);
    REQUIRE(emptyStore);
    CHECK(emptyStore->maximumM == world.launchZone(platform.id(), "", target).maximumM);
    const auto reported = own.launchZone("aim-120c", target);
    REQUIRE(reported);
    const auto native = world.launchZone(platform.id(), "aim-120c", target);
    CHECK(reported->maximumM == native.maximumM);
    CHECK(reported->noEscapeM == native.noEscapeM);
    CHECK(reported->minimumM == native.minimumM);
    CHECK(reported->maximumM > 0.0);
    // The legacy query returns its default for an unknown store. Parity must not add release eligibility checks.
    const auto unknown = own.launchZone("no-such-store", target);
    REQUIRE(unknown);
    CHECK(unknown->maximumM == world.launchZone(platform.id(), "no-such-store", target).maximumM);
    CHECK(unknown->minimumM == world.launchZone(platform.id(), "no-such-store", target).minimumM);
    REQUIRE(own.undesignate(target));
    CHECK_FALSE(own.launchZone("aim-120c", target));
}

TEST_CASE("ma equipment: store and strike reports preserve inventory and redact range truth targets", "[ma][equipment]") {
    World world(options("ma-equipment-own-reports"));
    Vehicle platform = aircraft(world, "platform");
    Vehicle opponent = aircraft(world, "opponent", 20.0);
    REQUIRE(platform.loadLoadout("mk82 x4") == ms::Reason::None);
    Agent agent = world.createAgent(platform, "equipment");
    const auto stores = agent.reportStores();
    const auto native = platform.stores();
    REQUIRE(stores.availability == ma::Availability::Available);
    REQUIRE(stores.items.size() == native.size());
    REQUIRE_FALSE(stores.items.empty());
    for (std::size_t index = 0; index < native.size(); ++index) {
        ownMeta(stores.items[index].meta, world.time());
        CHECK(stores.items[index].value.index == native[index].index);
        CHECK(stores.items[index].value.payload == native[index].payload);
        CHECK(stores.items[index].value.station == native[index].station);
        CHECK(std::string(stores.items[index].value.store) == native[index].store);
        CHECK(stores.items[index].value.massKg == native[index].massKg);
    }
    operate(world, agent, payloadOf(platform, ms::Family::StoreStation));
    ms::StrikeCommand command;
    command.payload = payloadOf(platform, ms::Family::StoreStation);
    command.store = "mk82";
    command.target = opponent.id(); // A range command may name truth; a legal actor report must not reveal it.
    const auto strike = platform.submit(command);
    REQUIRE(strike.accepted());
    world.step();
    REQUIRE(platform.strikeStatus(strike.activity));
    CHECK(platform.strikeStatus(strike.activity)->target == opponent.id());
    const auto first = agent.reportStrikes();
    REQUIRE(first.availability == ma::Availability::Available);
    REQUIRE(first.items.size() == 1);
    ownMeta(first.items[0].meta, world.time());
    CHECK(first.items[0].value.activity == strike.activity);
    CHECK(first.items[0].value.target == 0);
    CHECK(first.items[0].value.phase == platform.strikeStatus(strike.activity)->phase);
    CHECK(first.items[0].value.stores == platform.strikeStatus(strike.activity)->stores);
    CHECK(first.items[0].value.weapons == platform.strikeStatus(strike.activity)->weapons);
    CHECK(agent.reportStrikes().items[0].value.activity == strike.activity);
    CHECK(world.createAgent(opponent, "observer").reportStrikes().items.empty());
}

TEST_CASE("ma equipment: platform report access grants no authority to cancel another service strike", "[ma][equipment]") {
    World world(options("ma-equipment-service-authority"));
    Vehicle platform = aircraft(world, "platform");
    REQUIRE(platform.loadLoadout("mk82 x4") == ms::Reason::None);
    Agent owner = world.createAgent(platform, "owner");
    Agent reader = world.createAgent(platform, "reader");
    REQUIRE(bindMAService(world, owner, 1, {"*"}, true, true));
    REQUIRE(bindMAService(world, reader, 2, {}, false, false));
    operate(world, owner, payloadOf(platform, ms::Family::StoreStation));
    ms::StrikeCommand command;
    command.payload = payloadOf(platform, ms::Family::StoreStation);
    command.store = "mk82";
    const auto strike = owner.submit(command);
    REQUIRE(strike.accepted());
    world.step();
    const auto seen = reader.reportStrikes();
    REQUIRE(seen.availability == ma::Availability::Available);
    REQUIRE(seen.items.size() == 1);
    CHECK(seen.items[0].value.activity == strike.activity);
    CHECK(reader.cancelMission(strike.activity).reason == ms::Reason::AuthorityHeld);
    REQUIRE(owner.missionActivity(strike.activity));
    CHECK(owner.missionActivity(strike.activity)->live());
    CHECK(owner.cancelMission(strike.activity).status == control::CommandStatus::Canceled);
}

TEST_CASE("ma equipment: delivered consent reports retain provenance and respect configured ingress", "[ma][equipment]") {
    World world(options("ma-equipment-consent-ingress"));
    Vehicle platform = aircraft(world, "platform");
    REQUIRE(platform.loadLoadout("mk82 x4") == ms::Reason::None);
    comm::C2NodeSpec node;
    node.name = "approver";
    node.latitudeRad = (40.0 - 40.0 / 111.2) * kDeg;
    node.altitudeMslM = 200.0;
    const auto c2 = world.createC2Node(node);
    const auto net = world.createNet({});
    REQUIRE(world.joinNet(platform.id(), net));
    REQUIRE(world.joinNet(c2, net));
    Agent launcher = world.createAgent(platform, "launcher");
    Agent approver = world.createAgent(c2, "approver");
    Agent observer = world.createAgent(c2, "observer");
    REQUIRE(bindMAService(world, approver, 1, {}, true, true));
    REQUIRE(bindMAService(world, observer, 2, {}, false, false));
    REQUIRE(approver.declareRole(2, "APPROVER"));
    advance(world, 2.0);
    REQUIRE(approver.reportConsents().items.empty());
    operate(world, launcher, payloadOf(platform, ms::Family::StoreStation));
    ms::StrikeCommand command;
    command.payload = payloadOf(platform, ms::Family::StoreStation);
    command.store = "mk82";
    const auto strike = launcher.submit(command);
    REQUIRE(strike.accepted());
    CHECK(approver.reportConsents().items.empty()); // Sending and delivery require a native step.
    advance(world, 3.0);
    const auto reports = approver.reportConsents();
    REQUIRE(reports.availability == ma::Availability::Available);
    REQUIRE(reports.items.size() == 1);
    const auto& consent = reports.items.front();
    CHECK(consent.meta.source == ma::Source::Dms);
    CHECK(consent.meta.system == launcher.system());
    CHECK(consent.meta.net == 0);
    CHECK(consent.meta.sampleTimeS == consent.value.request.timeS);
    CHECK(consent.meta.receiveTimeS == consent.value.receivedS);
    CHECK(consent.meta.receiveTimeS >= consent.meta.sampleTimeS);
    CHECK(consent.meta.ageS == world.time() - consent.value.request.timeS);
    CHECK(consent.value.from == launcher.system());
    CHECK(consent.value.request.strike == strike.activity);
    CHECK_FALSE(consent.value.answered);
    CHECK(approver.reportConsents().items[0].value.request.requestId == consent.value.request.requestId);
    ma::ReportOptions strict;
    strict.staleAfterS = 0.0;
    CHECK(approver.reportConsents(strict).items[0].meta.validity == ma::Validity::Stale);
    const auto denied = observer.reportConsents();
    CHECK(denied.availability == ma::Availability::NotOwner);
    CHECK(denied.items.empty());
    CHECK(observer.consentRequests().size() == 1); // The existing platform-shared read retains its legacy behavior.
    REQUIRE(approver.answerConsent(consent.value.from, consent.value.request.requestId, true));
    CHECK(approver.reportConsents().items[0].value.answered);
    CHECK(launcher.strikeStatus(strike.activity)->phase == ms::StrikePhase::AwaitingConsent);
    advance(world, 3.0); // Native DMS delivery, authorization and station release run in the SDK.
    const auto completed = launcher.strikeStatus(strike.activity);
    REQUIRE(completed);
    REQUIRE(completed->phase == ms::StrikePhase::Done);
    REQUIRE(completed->weapons.size() == 1);
    REQUIRE(completed->stores.size() == 1);
    REQUIRE(completed->weapons[0] != 0);
    const auto released = launcher.reportStrikes();
    REQUIRE(released.items.size() == 1);
    CHECK(released.items[0].value.activity == strike.activity);
    CHECK(released.items[0].value.phase == completed->phase);
    CHECK(released.items[0].value.stores == completed->stores);
    CHECK(released.items[0].value.weapons == completed->weapons);
    bool matchedInventory = false;
    std::uint16_t remainingStore = 0xFFFF;
    for (const auto& item : launcher.reportStores().items) {
        if (item.value.index == completed->stores[0]) {
            matchedInventory = true;
            CHECK(item.value.status == ms::StoreStatus::Released);
            CHECK(item.value.weapon == completed->weapons[0]);
        } else if (item.value.status == ms::StoreStatus::Loaded) {
            remainingStore = item.value.index;
        }
    }
    REQUIRE(matchedInventory);
    bool matchedRelease = false;
    for (const auto& record : world.referee().weapons())
        if (record.id == completed->weapons[0]) {
            matchedRelease = true;
            CHECK(record.launcher == platform.id());
            CHECK(record.activity == strike.activity);
            CHECK_FALSE(record.jettisoned);
        }
    REQUIRE(matchedRelease); // Referee evidence checks this fixture; it is never added to actor reports.

    REQUIRE(remainingStore != 0xFFFF);
    ms::JettisonCommand jettison;
    jettison.payload = payloadOf(platform, ms::Family::StoreStation);
    jettison.stores = {remainingStore};
    jettison.emergency = true;
    const auto dropped = launcher.submit(jettison);
    REQUIRE(dropped.accepted());
    advance(world, 0.5);
    const auto droppedStatus = launcher.strikeStatus(dropped.activity);
    REQUIRE(droppedStatus);
    REQUIRE(droppedStatus->phase == ms::StrikePhase::Done);
    REQUIRE(droppedStatus->weapons.size() == 1);
    const auto afterJettison = launcher.reportStrikes();
    REQUIRE(afterJettison.items.size() == 2);
    // Native activity discovery lists the newer jettison first; reports retain that order.
    CHECK(afterJettison.items[0].value.activity == dropped.activity);
    CHECK(afterJettison.items[1].value.activity == strike.activity);
    CHECK(afterJettison.items[0].value.jettison);
    CHECK(afterJettison.items[0].value.stores == droppedStatus->stores);
    CHECK(afterJettison.items[0].value.weapons == droppedStatus->weapons);
}

TEST_CASE("ma equipment: absent endpoints, C2 nodes and removed platforms keep explicit availability", "[ma][equipment]") {
    const Agent empty;
    CHECK(empty.reportStores().availability == ma::Availability::Unavailable);
    CHECK(empty.reportStrikes().availability == ma::Availability::Unavailable);
    CHECK(empty.reportCountermeasures().availability == ma::Availability::Unavailable);
    CHECK(empty.reportConsents().availability == ma::Availability::Unavailable);
    CHECK_FALSE(empty.launchZone("aim-120c", 0));
    Agent absent;
    CHECK(absent.dispense() == ms::Reason::UnknownVehicle);

    World world(options("ma-equipment-unavailable"));
    comm::C2NodeSpec spec;
    spec.name = "node";
    const auto node = world.createC2Node(spec);
    Agent c2 = world.createAgent(node, "node");
    CHECK(c2.reportStores().availability == ma::Availability::NotApplicable);
    CHECK(c2.reportStrikes().availability == ma::Availability::NotApplicable);
    CHECK(c2.reportCountermeasures().availability == ma::Availability::NotApplicable);
    CHECK(c2.reportConsents().availability == ma::Availability::Available);
    CHECK(c2.reportConsents().items.empty());
    Vehicle civilian = aircraft(world, "civilian", 0.0, 0.0, "jsbsim:c172");
    Agent unarmed = world.createAgent(civilian, "unarmed");
    CHECK(unarmed.reportCountermeasures().availability == ma::Availability::NotApplicable);
    CHECK(unarmed.reportStores().availability == ma::Availability::Available);
    CHECK(unarmed.reportStores().items.empty());
    CHECK(unarmed.dispense() == ms::Reason::NotSupported);
    REQUIRE(civilian.remove());
    CHECK(unarmed.reportStores().availability == ma::Availability::Unavailable);
    CHECK(unarmed.reportStrikes().availability == ma::Availability::Unavailable);
    CHECK(unarmed.reportCountermeasures().availability == ma::Availability::Unavailable);
    CHECK(unarmed.reportConsents().availability == ma::Availability::Unavailable);
    CHECK(unarmed.reportStores().items.empty());
    CHECK(unarmed.reportStrikes().items.empty());
    REQUIRE(world.removeC2Node(node));
    CHECK(c2.reportConsents().availability == ma::Availability::Unavailable);
}
