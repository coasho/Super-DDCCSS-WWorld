// Explicit MA-service binding (docs/sdk/mission-training.md, section 9; fsim/MAService.h), through the public C++ SDK.
// A platform may carry several agents - several MA services. Where the range binds them, the bindings say who owns
// each command capability and each side of the platform's DMS, whatever order the agents were made in; a binding
// that overlaps another, or names a controller that is not one of the platform's own, is refused; a platform the
// range binds nothing on behaves as it always did.
#include "fsim/Agent.h"
#include "fsim/MAService.h"
#include "fsim/World.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <map>
#include <optional>
#include <string>
#include <vector>

using namespace fsim;

namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;
const char* const kHsa = "fsim.guidance.hsa";
const char* const kRoute = "fsim.guidance.route";

WorldOptions options(const char* name) {
    WorldOptions o;
    o.name = name;
    o.publish = false;
    o.workers = 1;
    o.pinWorkers = false;
    return o;
}

Vehicle aircraft(World& w, const std::string& name, double northKm) {
    VehicleSpec s;
    s.name = name;
    s.type = "jsbsim:f16c";
    s.initial.latitudeDeg = 40.0 + northKm / 111.2;
    s.initial.longitudeDeg = 0.0;
    s.initial.altitudeMslM = 6000.0;
    s.initial.headingDeg = 0.0;
    s.initial.airspeedTrueMs = 230.0;
    return w.createVehicle(s);
}

comm::C2NodeSpec c2At(const char* name, double northKm) {
    comm::C2NodeSpec s;
    s.name = name;
    s.latitudeRad = (40.0 + northKm / 111.2) * kDeg, s.longitudeRad = 0.0, s.altitudeMslM = 200.0;
    return s;
}

control::HsaCommand hsa(double headingDeg, double altitudeM = 6000.0) {
    control::HsaCommand c;
    c.headingRad = headingDeg * kDeg;
    c.speed = 230.0;
    c.altitudeM = altitudeM;
    return c;
}

/// The air radar's payload index, and its capability of a type by its discovery id.
std::uint16_t radarOf(const Vehicle& v) {
    for (const ms::PayloadInfo& p : v.payloads())
        if (p.family == ms::Family::AirRadar) return p.index;
    return 0xFFFF;
}
std::string radarCapability(const Vehicle& v, ms::CapabilityType type) {
    for (const ms::CapabilityInfo& c : v.missionCapabilities())
        if (c.type == type && c.payload == radarOf(v)) return c.id;
    return {};
}
std::string amtiOf(const Vehicle& v) { return radarCapability(v, ms::CapabilityType::Amti); }

MAServiceBindingV1 binding(std::uint64_t service, const Agent& agent, std::vector<std::string> capabilities, bool ingress, bool egress) {
    return {service, agent.vehicle(), agent.controller(), std::move(capabilities), ingress, egress};
}

std::optional<comm::CommandStatus> statusOf(World& w, const Agent& c2, std::uint64_t command, comm::Topic topic) {
    for (int k = 0; k < 60; ++k) {
        for (const auto& e : c2.commandStatuses())
            if (e.status.commandId == command && e.status.topic == topic) return e.status;
        w.step();
    }
    return std::nullopt;
}

/// The service a controller is, as the bindings have it (0: none).
std::uint64_t serviceOf(World& w, control::ControllerId controller) {
    for (const MAServiceBindingV1& b : maServiceBindings(w))
        if (b.controller == controller) return b.serviceId;
    return 0;
}

/// Two services on one aircraft - 1 flies, 2 works the radar and owns the inbox - and a C2 node that designates
/// control; the aircraft's two agents made in one order or the other. What comes of it, by service id alone.
struct Outcome {
    std::map<std::string, std::uint64_t> owner;  ///< each capability's bound service
    std::map<std::string, std::uint64_t> holder; ///< the service holding its grant after C2's designation
    std::uint64_t ingress = 0, egress = 0;
    bool flightFlies = false, sensorsFly = false, sensorsSearch = false, flightSearches = false;
    control::Reason sensorsAskHsa = control::Reason::None, flightAsksAmti = control::Reason::None;
    bool operator==(const Outcome& o) const {
        return owner == o.owner && holder == o.holder && ingress == o.ingress && egress == o.egress && flightFlies == o.flightFlies &&
               sensorsFly == o.sensorsFly && sensorsSearch == o.sensorsSearch && flightSearches == o.flightSearches &&
               sensorsAskHsa == o.sensorsAskHsa && flightAsksAmti == o.flightAsksAmti;
    }
};

Outcome configured(bool flightFirst) {
    World w(options(flightFirst ? "ma-binding-order-a" : "ma-binding-order-b"));
    Vehicle a = aircraft(w, "viper", 0.0);
    const std::uint32_t crc = w.createC2Node(c2At("crc", -40.0));
    const comm::NetId net = w.createNet({});
    REQUIRE(w.joinNet(a.id(), net));
    REQUIRE(w.joinNet(crc, net));
    // the same two services, their agents made in either order: only their controllers' numbers differ
    std::optional<Agent> flight, sensors;
    if (flightFirst) flight = w.createAgent(a, "flight"), sensors = w.createAgent(a, "sensors");
    else sensors = w.createAgent(a, "sensors"), flight = w.createAgent(a, "flight");
    Agent boss = w.createAgent(crc, "crc");
    const std::string amti = amtiOf(a);
    REQUIRE_FALSE(amti.empty());
    std::string error;
    // (and bound in either order too)
    const std::string power = radarCapability(a, ms::CapabilityType::Status);
    REQUIRE_FALSE(power.empty());
    const MAServiceBindingV1 one = binding(1, *flight, {kHsa, kRoute}, false, true), two = binding(2, *sensors, {amti, power}, true, false);
    REQUIRE(bindMAService(w, flightFirst ? one : two, &error));
    REQUIRE(bindMAService(w, flightFirst ? two : one, &error));
    REQUIRE(a.controlMode() == control::ControlMode::Granted);

    Outcome out;
    for (const char* c : {kHsa, kRoute}) out.owner[c] = serviceOf(w, maCapabilityOwner(w, a.id(), c));
    out.owner[amti] = serviceOf(w, maCapabilityOwner(w, a.id(), amti));
    const auto all = maServiceBindings(w);
    REQUIRE(all.size() == 2);
    REQUIRE(all[0].serviceId == 1); // (ascending service id, whichever was bound first)
    for (const MAServiceBindingV1& b : all) {
        if (b.ownsDmsIngress) out.ingress = b.serviceId;
        if (b.ownsDmsEgress) out.egress = b.serviceId;
    }
    // C2 designates every flight mode: the grants go to the capabilities' owners
    REQUIRE(boss.declareRole(3, "AVC-1"));
    w.step(10);
    const std::uint64_t d = boss.designateControl(flight->system(), 0xFFFFFFFFu);
    const auto st = statusOf(w, boss, d, comm::topic::kControlDesignation);
    REQUIRE(st);
    REQUIRE(st->state == comm::CommandStatus::Accepted);
    for (const char* c : {kHsa, kRoute}) {
        const control::ControlStatus s = a.controlStatus(c);
        out.holder[c] = s.granted ? serviceOf(w, s.holder) : 0;
    }
    out.holder[amti] = a.controlStatus(amti).granted ? serviceOf(w, a.controlStatus(amti).holder) : 0;
    out.sensorsAskHsa = sensors->requestControl(kHsa);
    out.flightAsksAmti = flight->requestControl(amti);
    out.flightFlies = flight->submit(hsa(90.0)).accepted();
    out.sensorsFly = sensors->submit(hsa(270.0)).accepted();
    ms::StateCommand on;
    on.payload = radarOf(a);
    on.state = ms::PayloadState::Operate;
    CHECK_FALSE(flight->submit(on).accepted()); // (the radar's power is the sensors service's too)
    REQUIRE(sensors->submit(on).accepted());
    w.step(static_cast<unsigned>(std::lround(60.0 / w.stepSeconds())));
    ms::AmtiCommand search;
    search.payload = on.payload;
    const ms::CommandResult mine = sensors->submit(search);
    INFO(ms::reasonName(mine.reason));
    out.sensorsSearch = mine.accepted();
    out.flightSearches = flight->submit(search).accepted();
    return out;
}

} // namespace

TEST_CASE("ma service binding: a platform with no binding keeps the legacy behaviour", "[ma][binding]") {
    World w(options("ma-binding-legacy"));
    Vehicle a = aircraft(w, "viper", 0.0);
    const std::uint32_t crc = w.createC2Node(c2At("crc", -40.0));
    const comm::NetId net = w.createNet({});
    REQUIRE(w.joinNet(a.id(), net));
    REQUIRE(w.joinNet(crc, net));
    Agent first = w.createAgent(a, "first");
    Agent last = w.createAgent(a, "last");
    Agent boss = w.createAgent(crc, "crc");
    CHECK(maServiceBindings(w).empty());
    CHECK(maCapabilityOwner(w, a.id(), kHsa) == 0);
    CHECK(a.controlMode() == control::ControlMode::Open);
    // open: either agent flies, either sends, either reads the inbox
    CHECK(first.submit(hsa(90.0)).accepted());
    CHECK(last.submit(hsa(180.0)).accepted());
    comm::DmsMessage m;
    m.topic = comm::topic::kFreeForm, m.bytes = {1, 2, 3};
    CHECK(first.send(m) != 0);
    CHECK(last.send(m) != 0);
    CHECK(first.subscribe(comm::topic::kFreeForm));
    // C2's designation: granted to the platform's agent made last, as before
    REQUIRE(boss.declareRole(3, "AVC-1"));
    w.step(10);
    const std::uint64_t d = boss.designateControl(first.system(), 0xFFFFFFFFu);
    const auto st = statusOf(w, boss, d, comm::topic::kControlDesignation);
    REQUIRE(st);
    REQUIRE(st->state == comm::CommandStatus::Accepted);
    const control::ControlStatus s = a.controlStatus(kHsa);
    CHECK(s.granted);
    CHECK(s.holder == last.controller());
    CHECK(maServiceBindings(w).empty()); // (a designation binds nothing)
}

TEST_CASE("ma service binding: ownership is the bindings', whatever order the agents were made in", "[ma][binding]") {
    const Outcome a = configured(true), b = configured(false);
    CHECK(a == b);
    for (const Outcome& o : {a, b}) {
        CHECK(o.owner.at(kHsa) == 1);
        CHECK(o.owner.at(kRoute) == 1);
        CHECK(o.holder.at(kHsa) == 1);
        CHECK(o.holder.at(kRoute) == 1);
        CHECK(o.ingress == 2);
        CHECK(o.egress == 1);
        CHECK(o.flightFlies);
        CHECK_FALSE(o.sensorsFly);
        CHECK(o.sensorsSearch);
        CHECK_FALSE(o.flightSearches);
        CHECK(o.sensorsAskHsa == control::Reason::NotAllowed);
        CHECK(o.flightAsksAmti == control::Reason::NotAllowed);
        for (const auto& [capability, service] : o.owner) {
            INFO(capability);
            CHECK(o.holder.at(capability) == service);
        }
    }
}

TEST_CASE("ma service binding: a duplicate, a foreign controller and an unknown capability are refused, and nothing changes", "[ma][binding]") {
    World w(options("ma-binding-refusals"));
    Vehicle a = aircraft(w, "viper", 0.0), b = aircraft(w, "wing", 2.0);
    const std::uint32_t crc = w.createC2Node(c2At("crc", -40.0));
    Agent one = w.createAgent(a, "one"), two = w.createAgent(a, "two"), three = w.createAgent(a, "three");
    Agent other = w.createAgent(b, "other");
    Agent boss = w.createAgent(crc, "crc");
    std::string error;
    const auto refused = [&](MAServiceBindingV1 x) {
        error.clear();
        const std::size_t before = maServiceBindings(w).size();
        const bool bound = bindMAService(w, x, &error);
        INFO(error);
        CHECK_FALSE(bound);
        CHECK_FALSE(error.empty());
        CHECK(maServiceBindings(w).size() == before);
        return !bound;
    };
    // before anything is bound: identities
    MAServiceBindingV1 x = binding(7, one, {kHsa}, true, true);
    x.serviceId = 0;
    CHECK(refused(x));
    x = binding(7, one, {kHsa}, true, true);
    x.controller = 0x40000099u; // (never minted)
    CHECK(refused(x));
    x.controller = 0; // (the default policy's: not an agent's)
    CHECK(refused(x));
    x.controller = 7; // (a program's own controller number)
    CHECK(refused(x));
    x = binding(7, other, {kHsa}, true, true);
    x.platform = a.id(); // (another platform's agent)
    CHECK(refused(x));
    x = binding(7, one, {kHsa}, true, true);
    x.platform = b.id();
    CHECK(refused(x));
    CHECK(refused(binding(7, one, {"fsim.guidance.no_such"}, true, true)));
    CHECK(refused(binding(7, one, {"fsim.ms.amti:no-such-payload"}, true, true)));
    CHECK(refused(binding(7, one, {kHsa, kHsa}, true, true)));
    CHECK(refused(binding(7, one, {"*", kHsa}, true, true)));
    CHECK(refused(binding(7, one, {""}, true, true)));
    CHECK(refused(binding(7, boss, {kHsa}, true, true))); // (a C2 node has no command capability)
    CHECK(refused(binding(7, boss, {"*"}, true, true)));
    CHECK(a.controlMode() == control::ControlMode::Open); // (a refused binding configured nothing)
    CHECK(one.submit(hsa(90.0)).accepted());

    REQUIRE(bindMAService(w, binding(7, one, {kHsa}, true, false), &error));
    CHECK(error.empty());
    // exclusive ownership, each kind
    CHECK(refused(binding(7, two, {kRoute}, false, false)));     // the service id
    CHECK(refused(binding(8, one, {kRoute}, false, false)));     // the controller
    CHECK(refused(binding(8, two, {kHsa}, false, false)));       // the capability
    CHECK(refused(binding(8, two, {kRoute, kHsa}, false, false)));
    CHECK(refused(binding(8, two, {"*"}, false, false)));        // every capability, one of them owned
    CHECK(refused(binding(8, two, {kRoute}, true, false)));      // the ingress
    REQUIRE(bindMAService(w, binding(8, two, {kRoute}, false, true), &error));
    CHECK(refused(binding(9, three, {}, false, true)));          // the egress
    CHECK(refused(binding(9, three, {}, true, false)));
    REQUIRE(bindMAService(w, binding(9, three, {}, false, false), &error)); // (a service that only reads)
    // "*" on a platform of its own, then nothing else there
    REQUIRE(bindMAService(w, binding(20, other, {"*"}, true, true), &error));
    Agent late = w.createAgent(b, "late");
    CHECK(refused(binding(21, late, {kHsa}, false, false)));
    REQUIRE(bindMAService(w, binding(21, late, {}, false, false), &error));
    // a C2 node: its DMS sides alone
    REQUIRE(bindMAService(w, binding(30, boss, {}, true, true), &error));
    CHECK(maServiceBindings(w).size() == 6);
    CHECK(maCapabilityOwner(w, b.id(), amtiOf(b)) == other.controller());
    CHECK(maCapabilityOwner(w, a.id(), amtiOf(a)) == 0); // (bound to no service there: nobody's)
}

TEST_CASE("ma service binding: the inbox is its ingress owner's and the sends its egress owner's; reports are shared", "[ma][binding]") {
    World w(options("ma-binding-dms"));
    Vehicle lead = aircraft(w, "lead", 0.0), wing = aircraft(w, "wing", 2.0);
    const std::uint32_t crc = w.createC2Node(c2At("crc", -40.0));
    const comm::NetId net = w.createNet({});
    for (const std::uint32_t e : {lead.id(), wing.id(), crc}) REQUIRE(w.joinNet(e, net));
    comm::DisseminationPlan plan;
    plan.positionPeriodS = 1.0;
    REQUIRE(w.setDissemination(lead.id(), plan));
    Agent talker = w.createAgent(wing, "talker"), listener = w.createAgent(wing, "listener"), reader = w.createAgent(wing, "reader");
    Agent peer = w.createAgent(lead, "lead-ma");
    Agent boss = w.createAgent(crc, "crc");
    std::string error;
    REQUIRE(bindMAService(w, binding(1, talker, {"*"}, false, true), &error));
    REQUIRE(bindMAService(w, binding(2, listener, {}, true, false), &error));
    REQUIRE(bindMAService(w, binding(3, reader, {}, false, false), &error));

    comm::DmsMessage m;
    m.topic = comm::topic::kFreeForm + 1, m.bytes = {0xFF, 0xFE, 0x00, 0x80};
    m.destinations = {peer.system()};
    comm::DmsReason why = comm::DmsReason::None;
    CHECK(talker.send(m, &why) != 0);
    for (Agent* other : {&listener, &reader}) {
        why = comm::DmsReason::None;
        CHECK(other->send(m, &why) == 0);
        CHECK(why == comm::DmsReason::NotOwner);
        CHECK(other->publishPosition({}, &why) == 0);
        CHECK(why == comm::DmsReason::NotOwner);
        CHECK(other->publishTracks(0xFFFF, {}, &why).empty());
        CHECK_FALSE(other->declareRole(5, "nobody", {}, &why));
        CHECK(why == comm::DmsReason::NotOwner);
    }
    CHECK(talker.publishPosition() != 0);
    // what reaches the wing waits for its ingress owner alone
    comm::DmsMessage back;
    back.topic = comm::topic::kFreeForm + 2, back.bytes = {7};
    back.destinations = {talker.system()};
    REQUIRE(peer.send(back) != 0);
    w.step(static_cast<unsigned>(std::lround(3.0 / w.stepSeconds())));
    CHECK(talker.inbox().empty());
    CHECK(talker.receive().empty());
    CHECK(reader.receive().empty());
    CHECK_FALSE(talker.subscribe(comm::topic::kFreeForm + 2));
    CHECK_FALSE(reader.subscribeAll(false));
    const auto waiting = listener.inbox();
    REQUIRE_FALSE(waiting.empty());
    CHECK(listener.subscribe(comm::topic::kFreeForm + 2));
    const auto taken = listener.receive();
    CHECK(taken.size() == waiting.size());
    bool found = false;
    for (const comm::DmsReceived& r : taken) found = found || (r.topic == comm::topic::kFreeForm + 2 && r.source == peer.system());
    CHECK(found);
    CHECK(listener.receive().empty()); // (taken once)
    // the platform's picture is every service's alike: sharing it grants nothing
    REQUIRE_FALSE(talker.peers().empty());
    CHECK(talker.peers().size() == listener.peers().size());
    CHECK(reader.peers().size() == listener.peers().size());
    CHECK(reader.ownship().timeS == talker.ownship().timeS);
    CHECK(reader.system() == talker.system());

    // C2's task: every service reads it, the egress owner answers it
    REQUIRE(boss.declareRole(2, "CRC"));
    w.step(10);
    comm::TaskOrder order;
    order.type = comm::TaskOrder::Patrol;
    const std::uint64_t task = boss.sendTask(talker.system(), order);
    REQUIRE(task != 0);
    w.step(static_cast<unsigned>(std::lround(2.0 / w.stepSeconds())));
    REQUIRE(listener.c2Tasks().size() == 1);
    REQUIRE(reader.c2Tasks().size() == 1);
    CHECK_FALSE(listener.acceptTask(boss.system(), task));
    CHECK_FALSE(reader.rejectTask(boss.system(), task, 3));
    CHECK(reader.c2Tasks()[0].state == comm::ReceivedTask::Received);
    CHECK(talker.acceptTask(boss.system(), task));
    CHECK(listener.c2Tasks()[0].state == comm::ReceivedTask::Accepted);
    CHECK(reader.authority().operators.size() == talker.authority().operators.size());
    // the C2 node bound too: its own sends its egress owner's
    Agent clerk = w.createAgent(crc, "clerk");
    REQUIRE(bindMAService(w, binding(40, boss, {}, true, true), &error));
    REQUIRE(bindMAService(w, binding(41, clerk, {}, false, false), &error));
    CHECK(clerk.sendTask(talker.system(), order, &why) == 0);
    CHECK(why == comm::DmsReason::NotOwner);
    CHECK(boss.sendTask(talker.system(), order) != 0);
}

TEST_CASE("ma service binding: an activity is its owner's; C2's override reaches the owner's flight, not the agent made last", "[ma][binding]") {
    World w(options("ma-binding-activities"));
    Vehicle a = aircraft(w, "viper", 0.0);
    const std::uint32_t crc = w.createC2Node(c2At("crc", -40.0));
    const comm::NetId net = w.createNet({});
    REQUIRE(w.joinNet(a.id(), net));
    REQUIRE(w.joinNet(crc, net));
    Agent flight = w.createAgent(a, "flight");
    Agent aux = w.createAgent(a, "aux"); // (made last: the legacy path would have chosen it)
    Agent boss = w.createAgent(crc, "crc");
    std::string error;
    REQUIRE(bindMAService(w, binding(1, flight, {"*"}, true, true), &error));
    REQUIRE(bindMAService(w, binding(2, aux, {}, false, false), &error));
    const control::CommandResult own = flight.submit(hsa(0.0));
    REQUIRE(own.accepted());
    // another service may read it, not change it
    REQUIRE(aux.activity(own.activity));
    const control::CommandResult changed = aux.update(own.activity, hsa(90.0));
    CHECK_FALSE(changed.accepted());
    CHECK(changed.reason == control::Reason::AuthorityHeld);
    const control::CommandResult ended = aux.cancel(own.activity);
    CHECK_FALSE(ended.accepted());
    CHECK(ended.reason == control::Reason::AuthorityHeld);
    CHECK(flight.activity(own.activity)->live());
    const control::CommandResult stolen = aux.submit(hsa(180.0));
    CHECK_FALSE(stolen.accepted());
    CHECK(stolen.reason == control::Reason::NotGranted);
    // C2's altitude override: the owner's live HSA flies it
    REQUIRE(boss.declareRole(3, "AVC"));
    w.step(10);
    comm::OverrideCommand high;
    high.kind = comm::OverrideCommand::Altitude, high.value = 7500.0;
    const std::uint64_t o = boss.setOverride(flight.system(), high);
    const auto st = statusOf(w, boss, o, comm::topic::kOverride);
    REQUIRE(st);
    REQUIRE(st->state == comm::CommandStatus::Accepted);
    const auto sp = flight.activitySetpoint(own.activity);
    REQUIRE(sp);
    CHECK(std::get<control::HsaCommand>(std::get<control::Command>(sp->command)).altitudeM == 7500.0);
    CHECK(flight.update(own.activity, hsa(45.0)).accepted());
    CHECK(flight.cancel(own.activity).status == control::CommandStatus::Canceled);
    CHECK_FALSE(flight.activity(own.activity)->live());
}

TEST_CASE("ma service binding: the lost-communication plan is flown as the route capability's owner", "[ma][binding]") {
    World w(options("ma-binding-lost-comms"));
    Vehicle a = aircraft(w, "viper", 0.0);
    const std::uint32_t crc = w.createC2Node(c2At("crc", -40.0));
    const comm::NetId net = w.createNet({});
    REQUIRE(w.joinNet(a.id(), net));
    REQUIRE(w.joinNet(crc, net));
    comm::DisseminationPlan heartbeat;
    heartbeat.statusPeriodS = 2.0;
    REQUIRE(w.setDissemination(crc, heartbeat));
    Agent flight = w.createAgent(a, "flight");
    Agent aux = w.createAgent(a, "aux"); // (made last)
    Agent boss = w.createAgent(crc, "crc");
    std::string error;
    REQUIRE(bindMAService(w, binding(1, flight, {"*"}, true, true), &error));
    REQUIRE(bindMAService(w, binding(2, aux, {}, false, false), &error));
    control::RoutePlan rtb;
    rtb.id = 9, rtb.version = 1;
    for (const double north : {-20.0, -60.0}) {
        control::Waypoint wp;
        wp.latitudeRad = (40.0 + north / 111.2) * kDeg, wp.longitudeRad = 0.0, wp.altitudeM = 6000.0;
        rtb.waypoints.push_back(wp);
    }
    REQUIRE(flight.planCommand(9, control::PlanCommand::PrepareForUpload).completed);
    REQUIRE(flight.publishPlan(rtb) == control::Reason::None);
    REQUIRE(flight.planCommand(9, control::PlanCommand::Upload).completed);
    REQUIRE(boss.declareRole(3, "AVC"));
    w.step(10);
    const std::uint64_t set = boss.setLostComms(flight.system(), 10.0, 9);
    const auto st = statusOf(w, boss, set, comm::topic::kLostComms);
    REQUIRE(st);
    REQUIRE(st->state == comm::CommandStatus::Accepted);
    REQUIRE(w.leaveNet(crc, net));
    w.step(static_cast<unsigned>(std::lround(12.0 / w.stepSeconds())));
    CHECK(aux.authority().lost); // (every service reads the authority alike)
    const auto plan = flight.planStatus(9);
    REQUIRE(plan);
    CHECK(plan->state == control::PlanState::Activated);
    bool flown = false;
    for (const control::ActivityRecord& r : flight.activities())
        if (r.live() && r.source == control::Source::Policy) flown = flown || r.controller == flight.controller();
    CHECK(flown);
    for (const control::ActivityRecord& r : flight.activities()) CHECK(r.controller != aux.controller());
}

TEST_CASE("ma service binding: a reset keeps the bindings; a removed platform's end, and a new one binds afresh", "[ma][binding][lifecycle]") {
    World w(options("ma-binding-lifecycle"));
    Vehicle a = aircraft(w, "viper", 0.0);
    Agent flight = w.createAgent(a, "flight"), aux = w.createAgent(a, "aux");
    std::string error;
    REQUIRE(bindMAService(w, binding(1, flight, {kHsa}, true, true), &error));
    REQUIRE(bindMAService(w, binding(2, aux, {kRoute}, false, false), &error));
    REQUIRE(flight.submit(hsa(90.0)).accepted());
    w.step(30);
    // the legacy vehicle reset: authority, and with it the bindings, stay
    a.reset();
    w.step(5);
    CHECK(maServiceBindings(w).size() == 2);
    CHECK(a.controlMode() == control::ControlMode::Granted);
    CHECK(maCapabilityOwner(w, a.id(), kHsa) == flight.controller());
    CHECK(aux.requestControl(kHsa) == control::Reason::NotAllowed);
    CHECK_FALSE(aux.submit(hsa(0.0)).accepted());
    if (flight.controlStatus(kHsa).granted) CHECK(flight.controlStatus(kHsa).holder == flight.controller());
    else CHECK(flight.requestControl(kHsa) == control::Reason::None); // (its owner's to ask for again)
    CHECK(flight.submit(hsa(0.0)).accepted());
    // removal: its services' ownership ends with it, and their agents answer nothing more
    const std::uint32_t gone = a.id();
    REQUIRE(a.remove());
    CHECK(maServiceBindings(w).empty());
    CHECK(maCapabilityOwner(w, gone, kHsa) == 0);
    CHECK_FALSE(flight.valid());
    CHECK_FALSE(flight.submit(hsa(0.0)).accepted());
    CHECK_FALSE(bindMAService(w, binding(1, flight, {kHsa}, true, true), &error)); // (the platform is gone)
    // a new platform, new agents: the same service ids bind afresh
    Vehicle b = aircraft(w, "viper-2", 0.0);
    CHECK(b.controlMode() == control::ControlMode::Open);
    Agent again = w.createAgent(b, "flight");
    CHECK(again.controller() != flight.controller());
    REQUIRE(bindMAService(w, binding(1, again, {"*"}, true, true), &error));
    CHECK(maServiceBindings(w).size() == 1);
    CHECK(again.submit(hsa(90.0)).accepted());
}

// --- What stood before a binding ------------------------------------------------------------------------------------
//
// A binding is the range configuring who owns what. Whatever grant or policy activity stood in its way is taken back by
// the platform as it takes anything back (revoked), and after it every grant is its capability's owner's or no one's.

namespace {

/// A radar switched on while the platform is open, so that a search can be commanded on it.
void operate(World& w, Vehicle& v) {
    REQUIRE(v.submit(ms::StateCommand{radarOf(v), ms::PayloadState::Operate}).accepted());
    w.step(60);
    for (const ms::PayloadInfo& p : v.payloads())
        if (p.index == radarOf(v)) REQUIRE(p.state == ms::PayloadState::Operate);
}

ms::AmtiCommand search(const Vehicle& v) {
    ms::AmtiCommand c;
    c.payload = radarOf(v);
    return c;
}

bool endedAs(const std::optional<control::ActivityRecord>& r, control::Reason why) {
    return r && !r->live() && r->state == control::ActivityState::Canceled && r->reason == why;
}

} // namespace

TEST_CASE("ma service binding: a grant another agent held is taken back, its activities end, and the owner is granted", "[ma][binding][transition]") {
    for (const bool granted : {false, true}) { // (the platform open, or under grants already)
        CAPTURE(granted);
        World w(options(granted ? "ma-binding-held-granted" : "ma-binding-held-open"));
        Vehicle a = aircraft(w, "viper", 0.0);
        Agent legacy = w.createAgent(a, "legacy");
        Agent owner = w.createAgent(a, "owner");
        if (granted) REQUIRE(a.setControlMode(control::ControlMode::Granted) == control::Reason::None);
        REQUIRE(legacy.requestControl(kHsa) == control::Reason::None);
        REQUIRE(legacy.requestControl(kRoute) == control::Reason::None); // (a capability the binding leaves unowned)
        const control::CommandResult flown = legacy.submit(hsa(90.0));
        REQUIRE(flown.accepted());
        control::CommandOptions later;
        later.window.startNotBefore = w.time() + 600.0;
        const control::CommandResult waits = legacy.submit(hsa(180.0), later);
        REQUIRE(waits.accepted());
        w.step(5);
        REQUIRE(legacy.activity(flown.activity)->live());
        REQUIRE(legacy.activity(waits.activity)->live());
        REQUIRE(legacy.activity(waits.activity)->waiting != control::ActivityWait::None);

        std::string error;
        REQUIRE(bindMAService(w, binding(1, owner, {kHsa}, true, true), &error));
        CHECK(a.controlMode() == control::ControlMode::Granted);
        // the grant is the owner's, as the binding says
        CHECK(maCapabilityOwner(w, a.id(), kHsa) == owner.controller());
        CHECK(a.controlStatus(kHsa).granted);
        CHECK(a.controlStatus(kHsa).holder == owner.controller());
        // what the other flew and what it had waiting ended, as a revoked grant's activities do
        CHECK(endedAs(legacy.activity(flown.activity), control::Reason::Revoked));
        CHECK(endedAs(legacy.activity(waits.activity), control::Reason::Revoked));
        // an unowned capability is no one's
        CHECK(maCapabilityOwner(w, a.id(), kRoute) == 0);
        CHECK_FALSE(a.controlStatus(kRoute).granted);
        // and from now on: the owner commands, the other does not and cannot ask
        const control::CommandResult stolen = legacy.submit(hsa(0.0));
        CHECK_FALSE(stolen.accepted());
        CHECK(stolen.reason == control::Reason::NotGranted);
        CHECK(legacy.requestControl(kHsa) == control::Reason::NotAllowed);
        CHECK(legacy.requestControl(kRoute) == control::Reason::NotAllowed);
        const control::CommandResult own = owner.submit(hsa(45.0));
        REQUIRE(own.accepted());
        CHECK(legacy.update(own.activity, hsa(0.0)).reason == control::Reason::AuthorityHeld);
        CHECK(legacy.cancel(own.activity).reason == control::Reason::AuthorityHeld);
        CHECK(owner.activity(own.activity)->live());
        // a later binding takes nothing from the first
        REQUIRE(bindMAService(w, binding(2, legacy, {kRoute}, false, false), &error));
        CHECK(a.controlStatus(kHsa).holder == owner.controller());
        CHECK(owner.activity(own.activity)->live());
        CHECK(a.controlStatus(kRoute).granted);
        CHECK(a.controlStatus(kRoute).holder == legacy.controller());
        CHECK(owner.requestControl(kRoute) == control::Reason::NotAllowed);
    }
}

TEST_CASE("ma service binding: what an open platform's agents flew without a grant - the owner's goes on, another's ends", "[ma][binding][transition]") {
    World w(options("ma-binding-open-activities"));
    Vehicle a = aircraft(w, "viper", 0.0);
    Agent owner = w.createAgent(a, "owner");
    Agent other = w.createAgent(a, "other");
    operate(w, a);
    // open, no grant asked for: the owner-to-be flies, the other searches with the radar
    REQUIRE(a.controlMode() == control::ControlMode::Open);
    const control::CommandResult flown = owner.submit(hsa(90.0));
    REQUIRE(flown.accepted());
    const ms::CommandResult searching = other.submit(search(a));
    REQUIRE(searching.accepted());
    w.step(5);
    REQUIRE(owner.activity(flown.activity)->live());
    REQUIRE(other.missionActivity(searching.activity)->live());
    const std::string amti = amtiOf(a);
    REQUIRE_FALSE(a.controlStatus(kHsa).granted);

    std::string error;
    REQUIRE(bindMAService(w, binding(1, owner, {kHsa}, true, true), &error)); // (the radar's search left unowned)
    CHECK(a.controlMode() == control::ControlMode::Granted);
    // the owner's own flight goes on, now under its grant, and is its to change
    CHECK(a.controlStatus(kHsa).holder == owner.controller());
    REQUIRE(owner.activity(flown.activity));
    CHECK(owner.activity(flown.activity)->live());
    CHECK(owner.update(flown.activity, hsa(45.0)).accepted());
    // the other's search is of a capability no service owns: ended, and no one's to command
    const auto ended = other.missionActivity(searching.activity);
    REQUIRE(ended);
    CHECK_FALSE(ended->live());
    CHECK(ended->state == control::ActivityState::Canceled);
    CHECK(ended->reason == ms::Reason::Revoked);
    CHECK_FALSE(a.controlStatus(amti).granted);
    CHECK_FALSE(other.submit(search(a)).accepted());
    CHECK(other.requestControl(amti) == control::Reason::NotAllowed);
    CHECK_FALSE(owner.submit(search(a)).accepted()); // (not the flight service's either)
}

TEST_CASE("ma service binding: a Mission Systems grant and its live activity follow the binding", "[ma][binding][transition]") {
    World w(options("ma-binding-ms-grant"));
    Vehicle a = aircraft(w, "viper", 0.0);
    Agent legacy = w.createAgent(a, "legacy");
    Agent owner = w.createAgent(a, "owner");
    operate(w, a);
    const std::string amti = amtiOf(a);
    REQUIRE(a.setControlMode(control::ControlMode::Granted) == control::Reason::None);
    REQUIRE(legacy.requestControl(amti) == control::Reason::None);
    const ms::CommandResult searching = legacy.submit(search(a));
    REQUIRE(searching.accepted());
    w.step(5);
    REQUIRE(legacy.missionActivity(searching.activity)->live());
    REQUIRE(a.controlStatus(amti).holder == legacy.controller());

    std::string error;
    REQUIRE(bindMAService(w, binding(1, owner, {amti}, true, true), &error));
    CHECK(maCapabilityOwner(w, a.id(), amti) == owner.controller());
    CHECK(a.controlStatus(amti).granted);
    CHECK(a.controlStatus(amti).holder == owner.controller());
    const auto ended = legacy.missionActivity(searching.activity);
    REQUIRE(ended);
    CHECK_FALSE(ended->live());
    CHECK(ended->reason == ms::Reason::Revoked);
    CHECK_FALSE(legacy.submit(search(a)).accepted());
    CHECK(legacy.requestControl(amti) == control::Reason::NotAllowed);
    const ms::CommandResult mine = owner.submit(search(a));
    REQUIRE(mine.accepted());
    CHECK(legacy.cancelMission(mine.activity).reason == ms::Reason::AuthorityHeld);
    CHECK(owner.missionActivity(mine.activity)->live());
}

TEST_CASE("ma service binding: the default policy holds no grant on a bound platform, and the platform stays under grants", "[ma][binding][transition]") {
    World w(options("ma-binding-controller-0"));
    Vehicle a = aircraft(w, "viper", 0.0);
    Agent owner = w.createAgent(a, "owner");
    // the range's own policy (controller 0) holds the grants and flies, as a range script may
    REQUIRE(a.setControlMode(control::ControlMode::Granted) == control::Reason::None);
    REQUIRE(a.requestControl(kHsa) == control::Reason::None);
    REQUIRE(a.requestControl(kRoute) == control::Reason::None);
    const control::CommandResult flown = a.submit(hsa(90.0));
    REQUIRE(flown.accepted());
    w.step(5);
    REQUIRE(a.controlStatus(kHsa).granted);
    REQUIRE(a.controlStatus(kHsa).holder == 0);

    std::string error;
    REQUIRE(bindMAService(w, binding(1, owner, {kHsa}, true, true), &error));
    CHECK(a.controlStatus(kHsa).granted);
    CHECK(a.controlStatus(kHsa).holder == owner.controller());
    CHECK_FALSE(a.controlStatus(kRoute).granted); // (unowned: no one's, the default policy's neither)
    CHECK(endedAs(w.activity(flown.activity), control::Reason::Revoked));
    CHECK_FALSE(a.submit(hsa(0.0)).accepted());
    // no controller but a capability's owner is granted it: not the default policy, not a number of the range's choosing
    CHECK(a.requestControl(kHsa) == control::Reason::NotAllowed);
    CHECK(a.requestControl(kRoute) == control::Reason::NotAllowed);
    CHECK(a.requestControl(kHsa, 7) == control::Reason::NotAllowed);
    CHECK(a.requestControl(kRoute, 7) == control::Reason::NotAllowed);
    CHECK(a.controlStatus(kHsa).holder == owner.controller());
    // the platform takes a grant back as before; only its owner has it again
    const control::CommandResult own = owner.submit(hsa(45.0));
    REQUIRE(own.accepted());
    REQUIRE(a.revokeControl(kHsa) == control::Reason::None);
    CHECK_FALSE(a.controlStatus(kHsa).granted);
    CHECK(endedAs(owner.activity(own.activity), control::Reason::Revoked));
    CHECK(a.requestControl(kHsa) == control::Reason::NotAllowed);
    CHECK(owner.requestControl(kHsa) == control::Reason::None);
    CHECK(owner.submit(hsa(45.0)).accepted());
    // and it cannot be opened again while it is bound: an open platform would answer every controller
    CHECK(a.setControlMode(control::ControlMode::Open) == control::Reason::NotAllowed);
    CHECK(a.controlMode() == control::ControlMode::Granted);
    CHECK(a.setControlMode(control::ControlMode::Granted) == control::Reason::None);
    // the platform's own sources command as ever: they need no grant
    control::CommandOptions platform;
    platform.source = control::Source::Override;
    CHECK(a.submit(hsa(10.0), platform).accepted());

    // a platform nothing is bound on: its modes and its grants as before
    Vehicle b = aircraft(w, "wing", 2.0);
    CHECK(b.setControlMode(control::ControlMode::Granted) == control::Reason::None);
    CHECK(b.requestControl(kHsa) == control::Reason::None);
    CHECK(b.requestControl(kRoute, 7) == control::Reason::None);
    CHECK(b.setControlMode(control::ControlMode::Open) == control::Reason::None);
    CHECK(b.controlMode() == control::ControlMode::Open);
    CHECK(b.controlStatus(kHsa).granted); // (a grant outlives the mode, as it always did)
    // a capability no platform has is refused for what it is, bound or not: not as an ownership matter
    CHECK(b.requestControl("fsim.guidance.no_such") != control::Reason::None);
    CHECK(a.requestControl("fsim.guidance.no_such") == b.requestControl("fsim.guidance.no_such"));
    CHECK(owner.requestControl("fsim.guidance.no_such") == b.requestControl("fsim.guidance.no_such"));
    CHECK(a.requestControl("fsim.ms.no_such:radar") == b.requestControl("fsim.ms.no_such:radar"));
}

TEST_CASE("ma service binding: a refused binding leaves grants, mode and activities as they were", "[ma][binding][transition]") {
    World w(options("ma-binding-refused-transition"));
    Vehicle a = aircraft(w, "viper", 0.0);
    Vehicle b = aircraft(w, "wing", 2.0);
    Agent legacy = w.createAgent(a, "legacy");
    Agent owner = w.createAgent(a, "owner");
    Agent stranger = w.createAgent(b, "stranger");
    REQUIRE(legacy.requestControl(kHsa) == control::Reason::None);
    const control::CommandResult flown = legacy.submit(hsa(90.0));
    REQUIRE(flown.accepted());
    w.step(5);
    std::string error;
    MAServiceBindingV1 foreign = binding(1, stranger, {kHsa}, true, true);
    foreign.platform = a.id();
    CHECK_FALSE(bindMAService(w, foreign, &error));
    CHECK_FALSE(bindMAService(w, binding(1, owner, {kHsa, "fsim.guidance.no_such"}, true, true), &error));
    CHECK_FALSE(bindMAService(w, binding(0, owner, {kHsa}, true, true), &error));
    CHECK(maServiceBindings(w).empty());
    CHECK(a.controlMode() == control::ControlMode::Open);
    CHECK(a.controlStatus(kHsa).granted);
    CHECK(a.controlStatus(kHsa).holder == legacy.controller());
    CHECK(legacy.activity(flown.activity)->live());
    CHECK(legacy.update(flown.activity, hsa(45.0)).accepted());
    CHECK(a.setControlMode(control::ControlMode::Granted) == control::Reason::None); // (unbound: the range's to set)
    CHECK(a.setControlMode(control::ControlMode::Open) == control::Reason::None);
}

TEST_CASE("ma service binding: an agent of another world is refused, though its numbers are this world's too", "[ma][binding]") {
    World one(options("ma-binding-world-one"));
    World two(options("ma-binding-world-two"));
    Vehicle a1 = aircraft(one, "viper", 0.0);
    Vehicle a2 = aircraft(two, "viper", 0.0);
    Agent mine = one.createAgent(a1, "mine");
    Agent theirs = two.createAgent(a2, "theirs");
    // each world minted the same ids: by number, the other world's agent is indistinguishable from this one's
    REQUIRE(a1.id() == a2.id());
    REQUIRE(mine.controller() == theirs.controller());
    std::string error;
    CHECK_FALSE(bindMAService(one, theirs, 1, {kHsa}, true, true, &error));
    CHECK(error.find("MA service 1") != std::string::npos);
    CHECK_FALSE(bindMAService(one, Agent{}, 1, {kHsa}, true, true, &error));
    CHECK_FALSE(bindMAService(one, mine, 0, {kHsa}, true, true, &error));                 // (and what the binding itself refuses)
    CHECK_FALSE(bindMAService(one, mine, 1, {"fsim.guidance.no_such"}, true, true, &error));
    // nothing changed, in either world
    CHECK(maServiceBindings(one).empty());
    CHECK(maServiceBindings(two).empty());
    CHECK(a1.controlMode() == control::ControlMode::Open);
    CHECK_FALSE(a1.controlStatus(kHsa).granted);
    CHECK(maCapabilityOwner(one, a1.id(), kHsa) == 0);
    CHECK(mine.submit(hsa(90.0)).accepted()); // (open still)
    // its own agent binds, each world its own
    REQUIRE(bindMAService(one, mine, 1, {kHsa}, true, true, &error));
    CHECK(error.empty());
    REQUIRE(bindMAService(two, theirs, 1, {"*"}, true, true));
    REQUIRE(maServiceBindings(one).size() == 1);
    CHECK(maServiceBindings(one)[0].platform == a1.id());
    CHECK(maServiceBindings(one)[0].controller == mine.controller());
    CHECK(maServiceBindings(one)[0].commandCapabilities == std::vector<std::string>{kHsa});
    CHECK(a1.controlStatus(kHsa).holder == mine.controller());
    CHECK(maServiceBindings(two).size() == 1);
    CHECK(a2.controlMode() == control::ControlMode::Granted);
}
