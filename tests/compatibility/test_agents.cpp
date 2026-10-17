// Agents and the referee (ADR-31, AB-2a; docs/agent-boundary.md): the range mints an agent per aircraft; an agent
// commands as itself (its source and controller its own: L2), addresses its own vehicle's activities alone, names no
// other aircraft by its entity id (L4) - checked by the world too, whoever passes an agent's controller - reads its
// radar without the entity behind a track (L3, the referee's), and sends and reads messages on its own node (L6).
// AB-2b: it designates a track as a target - flown on the track's estimate, never the aircraft behind it.
// AB-2c: mission data is each MA's own, or the range's common: no MA sees another's.
// AB-2d: every call an agent makes is counted in its audit, refused or not - the referee's to read.
#include "fsim/Agent.h"
#include "fsim/World.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

using namespace fsim;

namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;

WorldOptions options(const char* name) {
    WorldOptions o;
    o.name = name;
    o.publish = false;
    o.workers = 1;
    o.pinWorkers = false;
    return o;
}

/// An aircraft `northKm` and `eastKm` from 40 N 0 E.
Vehicle aircraft(World& w, const std::string& name, const char* type, double northKm, double eastKm, double altitudeM, double headingDeg,
                 double tasMs) {
    VehicleSpec s;
    s.name = name;
    s.type = std::string("jsbsim:") + type;
    s.initial.latitudeDeg = 40.0 + northKm / 111.2;
    s.initial.longitudeDeg = eastKm / (111.2 * std::cos(40.0 * kDeg));
    s.initial.altitudeMslM = altitudeM;
    s.initial.headingDeg = headingDeg;
    s.initial.airspeedTrueMs = tasMs;
    return w.createVehicle(s);
}

control::HsaCommand hsa(double headingDeg) {
    control::HsaCommand c;
    c.headingRad = headingDeg * kDeg;
    c.speed = 230.0;
    c.altitudeM = 6000.0;
    return c;
}

} // namespace

TEST_CASE("agents: the range mints one per live aircraft, each a controller of its own", "[agents]") {
    World w(options("agents-mint"));
    Vehicle a = aircraft(w, "a", "f16c", 0.0, 0.0, 6000.0, 0.0, 230.0);
    Vehicle b = aircraft(w, "b", "f16c", 20.0, 0.0, 6000.0, 180.0, 230.0);
    Agent blue = w.createAgent(a, "blue-1");
    Agent red = w.createAgent(b, "red-1");
    CHECK(blue.valid());
    CHECK(blue.vehicle() == a.id());
    CHECK(blue.name() == "blue-1");
    CHECK(blue.type() == "jsbsim:f16c");
    CHECK(blue.controller() > 0x40000000u);
    CHECK(red.controller() != blue.controller());
    CHECK_THROWS_AS(w.createAgent(w.vehicle(999u), "nobody"), Error);
    const Referee ref = w.referee();
    REQUIRE(ref.agents().size() == 2);
    CHECK(ref.agents()[0].vehicle == a.id());
    CHECK(ref.agents()[1].controller == red.controller());
    CHECK(ref.state(b.id()) != nullptr);
    CHECK(ref.vehicles().size() == 2);
}

TEST_CASE("agents: an agent commands as itself - another source or controller is refused (L2)", "[agents]") {
    World w(options("agents-identity"));
    Vehicle a = aircraft(w, "a", "f16c", 0.0, 0.0, 6000.0, 0.0, 230.0);
    Agent blue = w.createAgent(a, "blue-1");
    const control::CommandResult r = blue.submit(hsa(30.0));
    REQUIRE(r.accepted());
    const auto record = blue.activity(r.activity);
    REQUIRE(record);
    CHECK(record->controller == blue.controller());
    control::CommandOptions override;
    override.source = control::Source::Override;
    CHECK(blue.submit(hsa(40.0), override).reason == control::Reason::NotAllowed);
    control::CommandOptions other;
    other.controller = 7;
    CHECK(blue.submit(hsa(40.0), other).reason == control::Reason::NotAllowed);
    control::CommandOptions own;
    own.controller = blue.controller();
    CHECK(blue.submit(hsa(40.0), own).accepted());
    // its own ship as its navigation reports it (the sensed state, without what no platform reports)
    const OwnShip ship = blue.ownship();
    CHECK(ship.altitudeMslM > 5000.0);
    CHECK(std::abs(ship.latitudeRad - a.state().latitudeRad) < 1e-6);
    CHECK(ship.airspeedTrueMs > 150.0);
    CHECK((ship.throttle >= 0.0 && ship.throttle <= 1.0)); // (AB-3: a batch observation reads it)
    ms::CommandOptions msOverride;
    msOverride.source = control::Source::Override;
    CHECK(blue.submit(ms::StateCommand{0, ms::PayloadState::Standby}, msOverride).reason == ms::Reason::NotAllowed);
    // a grant is its own controller's
    a.setControlMode(control::ControlMode::Granted);
    CHECK(blue.requestControl("fsim.guidance.hsa") == control::Reason::None);
    CHECK(a.controlStatus("fsim.guidance.hsa").holder == blue.controller());
    CHECK(blue.releaseControl("fsim.guidance.hsa") == control::Reason::None);
}

TEST_CASE("agents: its own vehicle's activities alone", "[agents]") {
    World w(options("agents-scope"));
    Vehicle a = aircraft(w, "a", "f16c", 0.0, 0.0, 6000.0, 0.0, 230.0);
    Vehicle b = aircraft(w, "b", "f16c", 20.0, 0.0, 6000.0, 180.0, 230.0);
    Agent blue = w.createAgent(a, "blue-1");
    const control::CommandResult theirs = b.submit(hsa(90.0));
    REQUIRE(theirs.accepted());
    CHECK_FALSE(blue.activity(theirs.activity));
    CHECK_FALSE(blue.activitySetpoint(theirs.activity));
    CHECK(blue.endPoints(theirs.activity).empty());
    CHECK(blue.update(theirs.activity, hsa(120.0)).reason == control::Reason::NotAllowed);
    CHECK(blue.cancel(theirs.activity).reason == control::Reason::NotAllowed);
    CHECK(blue.activityCommand(theirs.activity, control::ActivityCommand::Disable).reason == control::Reason::NotAllowed);
    w.step();
    CHECK(w.activity(theirs.activity)->state == control::ActivityState::Active); // (untouched)
    const control::CommandResult mine = blue.submit(hsa(10.0));
    REQUIRE(mine.accepted());
    w.step();
    CHECK(blue.update(mine.activity, hsa(20.0)).accepted());
    const control::CommandResult cancel = blue.cancel(mine.activity);
    INFO(control::reasonName(cancel.reason));
    CHECK(cancel.status == control::CommandStatus::Canceled);
}

TEST_CASE("agents: no other aircraft by its entity id (L4) - the agent's calls and the world's own check", "[agents]") {
    World w(options("agents-entities"));
    Vehicle a = aircraft(w, "a", "f16c", 0.0, 0.0, 6000.0, 0.0, 230.0);
    Vehicle b = aircraft(w, "b", "f16c", 20.0, 0.0, 6000.0, 180.0, 230.0);
    Agent blue = w.createAgent(a, "blue-1");
    control::BehaviorCommand pursuit;
    pursuit.id = "pursuit";
    pursuit.target = b.id();
    CHECK(blue.submit(pursuit).reason == control::Reason::NotAllowed);
    // whoever passes an agent's controller: the world itself refuses it
    control::CommandOptions asBlue;
    asBlue.controller = blue.controller();
    CHECK(a.submit(pursuit, asBlue).reason == control::Reason::NotAllowed);
    // the range's own scripts may still fly on truth
    CHECK(a.submit(pursuit).accepted());
    control::MustFlyCommand mustFly;
    mustFly.location = static_cast<double>(control::MustFlyLocation::Entity);
    mustFly.target = static_cast<double>(b.id());
    CHECK(blue.submit(mustFly).reason == control::Reason::NotAllowed);
    // a strike names no entity, and asks for consent
    ms::StrikeCommand strike;
    strike.target = b.id();
    CHECK(blue.submit(strike).reason == ms::Reason::NotAllowed);
    ms::StrikeCommand consented;
    consented.consent = true;
    CHECK(blue.submit(consented).reason == ms::Reason::NotAllowed);
    // a frame following another aircraft is its true pose; one following its own is its own
    control::FrameSpec theirs;
    theirs.origin = control::FrameOrigin::Vehicle, theirs.vehicle = b.id();
    control::FrameSpec own;
    own.origin = control::FrameOrigin::Vehicle, own.vehicle = a.id();
    const control::FrameId t = w.createFrame(theirs), o = w.createFrame(own);
    REQUIRE((t != 0 && o != 0));
    CHECK_FALSE(blue.framePoint(t, control::FrameOffset{}, std::nan("")));
    CHECK(blue.framePoint(o, control::FrameOffset{}, std::nan("")));
    CHECK(w.framePoint(t, control::FrameOffset{}));
}

TEST_CASE("agents: a separate check - a command as an agent's controller on another vehicle is refused", "[agents]") {
    World w(options("agents-elsewhere"));
    Vehicle a = aircraft(w, "a", "f16c", 0.0, 0.0, 6000.0, 0.0, 230.0);
    Vehicle b = aircraft(w, "b", "f16c", 20.0, 0.0, 6000.0, 180.0, 230.0);
    Agent blue = w.createAgent(a, "blue-1");
    control::CommandOptions asBlue;
    asBlue.controller = blue.controller();
    control::Command c = hsa(90.0);
    CHECK(b.submit(c, asBlue).reason == control::Reason::NotAllowed);
}

TEST_CASE("agents: its radar's tracks without the entity behind them - the referee has it (L3)", "[agents][ms]") {
    World w(options("agents-tracks"));
    Vehicle a = aircraft(w, "a", "f16c", 0.0, 0.0, 6000.0, 90.0, 230.0);
    Vehicle b = aircraft(w, "b", "f16c", 3.0, 55.0, 7000.0, 270.0, 230.0);
    (void)b;
    Agent blue = w.createAgent(a, "blue-1");
    REQUIRE(blue.submit(ms::StateCommand{0, ms::PayloadState::Operate}).accepted());
    w.step();
    ms::AmtiCommand search;
    search.elevationCentreRad = -1.0 * kDeg;
    search.bars = 4;
    REQUIRE(blue.submit(search).accepted());
    w.step(static_cast<unsigned>(std::lround(45.0 / w.stepSeconds())));
    const auto tracks = blue.amtiTracks(0);
    const auto truthful = a.amtiTracks(0);
    REQUIRE(!tracks.empty());
    REQUIRE(tracks.size() == truthful.size());
    const Referee ref = w.referee();
    std::size_t labelled = 0;
    for (std::size_t i = 0; i < tracks.size(); ++i) {
        CHECK(tracks[i].truth == 0);
        CHECK(tracks[i].id == truthful[i].id);
        CHECK(tracks[i].latitudeRad == truthful[i].latitudeRad); // (the estimate itself is the agent's)
        CHECK(ref.trackTruth(a.id(), 0, tracks[i].id) == truthful[i].truth);
        labelled += truthful[i].truth != 0 ? 1u : 0u;
    }
    CHECK(labelled > 0);
    for (const auto& d : blue.amtiDetections(0)) CHECK(d.truth == 0);
}

namespace {
comm::DmsMessage freeForm(comm::Topic topic, std::vector<std::uint8_t> bytes, std::vector<comm::SystemId> to = {}) {
    comm::DmsMessage m;
    m.topic = topic, m.bytes = std::move(bytes), m.destinations = std::move(to);
    return m;
}
/// A C2 node `northKm` and `eastKm` from 40 N 0 E.
comm::C2NodeSpec c2At(const char* name, double northKm, double eastKm, double altitudeM) {
    comm::C2NodeSpec n;
    n.name = name;
    n.latitudeRad = (40.0 + northKm / 111.2) * kDeg;
    n.longitudeRad = eastKm / (111.2 * std::cos(40.0 * kDeg)) * kDeg;
    n.altitudeMslM = altitudeM;
    return n;
}
} // namespace

TEST_CASE("agents: links (AB-4a) - systems by SystemID, sends stamped by the DMS, its own inbox", "[agents][comm]") {
    World w(options("agents-dms"));
    Vehicle a = aircraft(w, "a", "f16c", 0.0, 0.0, 6000.0, 0.0, 230.0);
    Vehicle b = aircraft(w, "b", "f16c", 20.0, 0.0, 6000.0, 180.0, 230.0);
    const std::uint32_t c2 = w.createC2Node(c2At("awacs-cell", -30.0, 0.0, 150.0));
    CHECK(c2 > 0x60000000u); // (clear of the vehicles' ids)
    Agent blue = w.createAgent(a, "blue-1");
    Agent red = w.createAgent(b, "blue-2");
    Agent boss = w.createAgent(c2, "c2");
    // on no net: no system, nothing sent
    comm::DmsReason why = comm::DmsReason::None;
    CHECK(blue.system() == comm::kNoSystem);
    CHECK(blue.send(freeForm(comm::topic::kFreeForm, {1}), &why) == 0);
    CHECK(why == comm::DmsReason::NoSystem);

    comm::NetSpec spec;
    spec.name = "l16-a";
    const comm::NetId net = w.createNet(spec);
    CHECK(w.assignSystem(a.id(), 0x1001, "AF-0101") == 0x1001);
    CHECK(w.assignSystem(b.id(), 0x1001) == comm::kNoSystem); // (another's)
    REQUIRE(w.joinNet(a.id(), net));
    REQUIRE(w.joinNet(b.id(), net));
    REQUIRE(w.joinNet(c2, net));
    CHECK(blue.system() == 0x1001);
    CHECK(red.system() != comm::kNoSystem);
    CHECK(red.system() != b.id()); // (never the entity id)
    REQUIRE(blue.nets().size() == 1);
    CHECK(blue.nets()[0].name == "l16-a");

    // the native types are the MA core's; payloads within the net's size; destinations it can reach
    CHECK(blue.send(freeForm(comm::topic::kPositionReport, {1}), &why) == 0);
    CHECK(why == comm::DmsReason::ReservedTopic);
    CHECK(blue.send(freeForm(comm::topic::kFreeForm, std::vector<std::uint8_t>(spec.maxPayloadBytes + 1)), &why) == 0);
    CHECK(why == comm::DmsReason::TooLarge);
    CHECK(blue.send(freeForm(comm::topic::kFreeForm, {1}, {0x7777}), &why) == 0);
    CHECK(why == comm::DmsReason::UnknownSystem);

    // published: whoever is subscribed, stamped with its sender's SystemID
    const std::uint64_t m1 = blue.send(freeForm(1001, {1, 2, 3}));
    REQUIRE(m1 != 0);
    std::vector<comm::DmsReceived> atRed, atBoss;
    for (int k = 0; k < 15 && (atRed.empty() || atBoss.empty()); ++k) {
        w.step();
        CHECK(blue.inbox().empty()); // (not its own)
        for (auto& r : red.receive()) atRed.push_back(r);
        for (auto& r : boss.receive()) atBoss.push_back(r);
    }
    REQUIRE(atRed.size() == 1);
    REQUIRE(atBoss.size() == 1);
    CHECK(atRed[0].source == 0x1001);
    CHECK(atRed[0].topic == 1001);
    CHECK(atRed[0].net == net);
    CHECK(atRed[0].bytes == std::vector<std::uint8_t>{1, 2, 3});
    CHECK(atRed[0].timeDelivered >= atRed[0].timeSent + spec.latencyS - spec.jitterS);
    const auto t1 = blue.transmission(m1);
    REQUIRE(t1);
    CHECK(t1->published);
    CHECK(t1->reached == 2);

    // reliable, to one system: its status per destination; the C2 node's agent answers its sender by its SystemID
    const std::uint64_t m2 = boss.send(freeForm(1002, {9}, {atBoss[0].source}));
    REQUIRE(m2 != 0);
    bool got = false;
    for (int k = 0; k < 15 && !got; ++k) {
        w.step();
        CHECK(red.inbox().empty());
        for (auto& r : blue.receive()) got = got || (r.topic == 1002 && r.source == boss.system());
    }
    CHECK(got);
    const auto t2 = boss.transmission(m2);
    REQUIRE(t2);
    REQUIRE(t2->destinations.size() == 1);
    CHECK(t2->destinations[0].first == 0x1001);
    CHECK(t2->destinations[0].second == comm::Delivery::Delivered);

    // the referee maps SystemIDs to entities and counts the traffic
    const Referee ref = w.referee();
    CHECK(ref.systemEntity(0x1001) == a.id());
    CHECK(ref.entitySystem(c2) == boss.system());
    CHECK(ref.tailNumber(0x1001) == "AF-0101");
    CHECK(ref.netStats(net).delivered == 3);
    REQUIRE(ref.c2Nodes().size() == 1);
    CHECK(ref.c2Node(c2)->name == "awacs-cell");
    // a C2 node's agent has no flight to command
    CHECK_FALSE(boss.capabilities().size() > 0);
}

TEST_CASE("agents: links (AB-4a) - range, the earth's curvature, reliable retries and their timeout", "[agents][comm]") {
    World w(options("agents-dms-sight"));
    // two low aircraft 400 km apart: within the net's range, below each other's radio horizon (~2 x 130 km at 1 km)
    Vehicle a = aircraft(w, "a", "f16c", 0.0, 0.0, 1000.0, 0.0, 200.0);
    Vehicle b = aircraft(w, "b", "f16c", 400.0, 0.0, 1000.0, 0.0, 200.0);
    Vehicle c = aircraft(w, "c", "f16c", 50.0, 0.0, 1000.0, 0.0, 200.0);
    Agent blue = w.createAgent(a, "blue-1");
    Agent far = w.createAgent(b, "blue-2");
    comm::NetSpec spec;
    spec.reliableTimeoutS = 2.0;
    const comm::NetId net = w.createNet(spec);
    for (const Vehicle* v : {&a, &b, &c}) REQUIRE(w.joinNet(v->id(), net));
    const std::uint64_t m = blue.send(freeForm(1001, {1}, {far.system()}));
    REQUIRE(m != 0);
    for (int k = 0; k < 90; ++k) {
        w.step();
        CHECK(far.inbox().empty());
    }
    const auto t = blue.transmission(m);
    REQUIRE(t);
    CHECK(t->destinations[0].second == comm::Delivery::Failed);
    const auto stats = w.referee().netStats(net);
    CHECK(stats.masked >= 3);  // (retried each half second, the path refreshed each second)
    CHECK(stats.failed == 1);
    CHECK(stats.delivered == 0);

    // out of range: a net reaching 30 km
    comm::NetSpec shortSpec;
    shortSpec.rangeM = 30000.0;
    shortSpec.lineOfSight = false;
    const comm::NetId shortNet = w.createNet(shortSpec);
    REQUIRE(w.joinNet(a.id(), shortNet));
    REQUIRE(w.joinNet(c.id(), shortNet));
    Agent near = w.createAgent(c, "blue-3");
    const std::uint64_t m2 = blue.send(freeForm(1001, {2}, {near.system()}));
    REQUIRE(m2 != 0);
    for (int k = 0; k < 15; ++k) w.step();
    // (the long net delivers it: 50 km is within its radio horizon; the short one counts it out of range)
    CHECK(blue.transmission(m2)->destinations[0].second == comm::Delivery::Delivered);
    CHECK(w.referee().netStats(shortNet).outOfRange >= 1);
}

TEST_CASE("agents: links (AB-4a) - subscriptions, two nets delivering once, a sender's share of the rate", "[agents][comm]") {
    World w(options("agents-dms-share"));
    Vehicle a = aircraft(w, "a", "f16c", 0.0, 0.0, 6000.0, 0.0, 230.0);
    Vehicle b = aircraft(w, "b", "f16c", 10.0, 0.0, 6000.0, 0.0, 230.0);
    Agent blue = w.createAgent(a, "blue-1");
    Agent red = w.createAgent(b, "blue-2");
    const comm::NetId n1 = w.createNet({});
    const comm::NetId n2 = w.createNet({});
    for (const comm::NetId n : {n1, n2}) {
        REQUIRE(w.joinNet(a.id(), n));
        REQUIRE(w.joinNet(b.id(), n));
    }
    REQUIRE(red.subscribeAll(false));
    REQUIRE(red.subscribe(1001));
    CHECK(red.subscriptions() == std::vector<comm::Topic>{1001});
    REQUIRE(blue.send(freeForm(1001, {1})) != 0);
    REQUIRE(blue.send(freeForm(1002, {2})) != 0);           // (published, not subscribed: not received)
    REQUIRE(blue.send(freeForm(1003, {3}, {red.system()})) != 0); // (sent to it: received)
    // (what arrives waits until taken: read once, after the steps)
    for (int k = 0; k < 15; ++k) w.step();
    CHECK(red.inbox().size() == 2);
    std::vector<comm::Topic> topics;
    for (const auto& r : red.receive()) topics.push_back(r.topic);
    CHECK(red.inbox().empty());
    std::sort(topics.begin(), topics.end());
    CHECK(topics == std::vector<comm::Topic>{1001, 1003}); // (each once, though both nets carried it)
    const Referee ref = w.referee();
    CHECK(ref.netStats(n1).delivered + ref.netStats(n2).delivered == 2);
    CHECK(ref.netStats(n1).duplicates + ref.netStats(n2).duplicates == 2);

    // a slow net: 2 members share 2,000 bit/s - a 1,000-byte message (+64 header) takes ~8.5 s of a sender's share;
    // the queue takes 2 s - the second waits too long and is dropped
    comm::NetSpec slow;
    slow.dataRateBps = 2000.0;
    const comm::NetId n3 = w.createNet(slow);
    Vehicle c = aircraft(w, "c", "f16c", 0.0, 10.0, 6000.0, 0.0, 230.0);
    Vehicle d = aircraft(w, "d", "f16c", 10.0, 10.0, 6000.0, 0.0, 230.0);
    REQUIRE(w.joinNet(c.id(), n3));
    REQUIRE(w.joinNet(d.id(), n3));
    Agent cA = w.createAgent(c, "c");
    const std::uint64_t s1 = cA.send(freeForm(1001, std::vector<std::uint8_t>(1000)));
    const std::uint64_t s2 = cA.send(freeForm(1001, std::vector<std::uint8_t>(1000)));
    for (int k = 0; k < 400; ++k) w.step();
    CHECK(cA.transmission(s1)->reached == 1);
    CHECK_FALSE(cA.transmission(s1)->queueDropped);
    CHECK(cA.transmission(s2)->queueDropped);
    CHECK(ref.netStats(n3).queueDropped == 1);
}

TEST_CASE("agents: links (AB-4b) - position reports, heartbeats and tracks shared; a received track designated", "[agents][comm]") {
    World w(options("agents-dms-picture"));
    Vehicle a = aircraft(w, "lead", "f16c", 0.0, 0.0, 6000.0, 90.0, 230.0);
    Vehicle b = aircraft(w, "bandit", "f16c", 3.0, 55.0, 7000.0, 270.0, 230.0);
    Vehicle c = aircraft(w, "wing", "f16c", -10.0, 0.0, 6000.0, 90.0, 230.0);
    Agent lead = w.createAgent(a, "blue-1");
    Agent wing = w.createAgent(c, "blue-2");
    const comm::NetId net = w.createNet({});
    REQUIRE(w.assignSystem(a.id(), 0x4001, "VIPER11") == 0x4001);
    REQUIRE(w.joinNet(a.id(), net));
    REQUIRE(w.joinNet(c.id(), net));
    // the lead's plan: a position report each second, a heartbeat every five, its radar's tracks every two
    comm::DisseminationPlan plan;
    plan.positionPeriodS = 1.0, plan.statusPeriodS = 5.0, plan.tracksPeriodS = 2.0;
    REQUIRE(w.setDissemination(a.id(), plan));
    REQUIRE(lead.submit(ms::StateCommand{0, ms::PayloadState::Operate}).accepted());
    w.step();
    ms::AmtiCommand search;
    search.elevationCentreRad = -1.0 * kDeg;
    search.bars = 4;
    REQUIRE(lead.submit(search).accepted());
    // a native type's message is the MA core's alone, from the platform's reports
    CHECK(wing.send(freeForm(comm::topic::kEntity, std::vector<std::uint8_t>(sizeof(comm::EntityReport)))) == 0);
    w.step(static_cast<unsigned>(std::lround(45.0 / w.stepSeconds())));

    // the wingman's picture: the lead's last position report (its navigation's) and heartbeat
    const auto peers = wing.peers();
    REQUIRE(peers.size() == 1);
    CHECK(peers[0].system == 0x4001);
    CHECK(peers[0].tailNumber == "VIPER11");
    REQUIRE(peers[0].hasPosition);
    const VehicleState* truth = w.referee().state(a.id());
    const double dNorth = (peers[0].position.latitudeRad - truth->latitudeRad) * 6.371e6;
    CHECK(std::abs(dNorth) < 400.0); // (at most a second old, plus the latency)
    CHECK(w.time() - peers[0].receivedS < 1.2);
    CHECK(lead.peers().empty()); // (the wingman publishes nothing)

    // the lead's tracks, received: the bandit's among them - its numbering, never the entity
    const Referee ref = w.referee();
    const auto received = wing.linkTracks();
    REQUIRE(!received.empty());
    const comm::LinkTrack* bandit = nullptr;
    for (const auto& t : received)
        if (ref.linkTrackTruth(t.source, static_cast<std::uint16_t>(t.report.payload), t.report.track) == b.id()) bandit = &t;
    REQUIRE(bandit != nullptr);
    CHECK(bandit->source == 0x4001);
    CHECK(wing.amtiTracks(0).empty()); // (its own radar is off: it knows the bandit by the link alone)

    // designated: a target by a network track, flown on its last report carried forward
    CHECK(wing.designateLink(0x4001, 0, 987654321u) == 0);
    const std::uint32_t target = wing.designateLink(bandit->source, static_cast<std::uint16_t>(bandit->report.payload), bandit->report.track);
    REQUIRE(target > 0x80000000u);
    CHECK(ref.designationTruth(target) == b.id());
    REQUIRE(wing.designations().size() == 1);
    CHECK(wing.designations()[0].source == 0x4001);
    control::BehaviorCommand pursuit;
    pursuit.id = "pursuit";
    pursuit.target = target;
    const auto chase = wing.submit(pursuit);
    REQUIRE(chase.accepted());
    w.step(static_cast<unsigned>(std::lround(5.0 / w.stepSeconds())));
    CHECK(w.activity(chase.activity)->state == control::ActivityState::Active);
    const VehicleState* estimate = ref.state(target);
    REQUIRE(estimate != nullptr);
    const VehicleState* bTruth = ref.state(b.id());
    const double dLat = (estimate->latitudeRad - bTruth->latitudeRad) * 6.371e6;
    const double dLon = (estimate->longitudeRad - bTruth->longitudeRad) * 6.371e6 * std::cos(bTruth->latitudeRad);
    CHECK(std::hypot(dLat, dLon) < 3000.0);

    // the link lost (the lead leaves the net): the received track ages out and the designation lapses with it
    REQUIRE(w.leaveNet(a.id(), net));
    w.step(static_cast<unsigned>(std::lround((plan.trackTimeoutS + 1.0) / w.stepSeconds())));
    CHECK(wing.linkTracks().empty());
    CHECK_FALSE(wing.designations()[0].valid);
    CHECK(ref.state(target) == nullptr);
}

namespace {
/// Its command's last status from `ma`, after stepping up to two seconds for it.
std::optional<comm::CommandStatus> statusOf(World& w, const Agent& c2, std::uint64_t command, comm::Topic topic,
                                            std::uint32_t wantState = 0xFFFFFFFFu) {
    for (int k = 0; k < 60; ++k) {
        for (const auto& e : c2.commandStatuses())
            if (e.status.commandId == command && e.status.topic == topic && (wantState == 0xFFFFFFFFu || e.status.state == wantState))
                return e.status;
        w.step();
    }
    return std::nullopt;
}
} // namespace

TEST_CASE("agents: C2 authority (AB-5a) - roles declared, authorized or not; a direct command held against the agent", "[agents][c2]") {
    World w(options("agents-c2"));
    Vehicle a = aircraft(w, "viper", "f16c", 0.0, 0.0, 6000.0, 0.0, 230.0);
    const std::uint32_t crc = w.createC2Node(c2At("crc", -40.0, 0.0, 200.0));
    const std::uint32_t watch = w.createC2Node(c2At("watch", -45.0, 0.0, 200.0));
    const comm::NetId net = w.createNet({});
    for (const std::uint32_t e : {a.id(), crc, watch}) REQUIRE(w.joinNet(e, net));
    Agent blue = w.createAgent(a, "blue-1");
    Agent boss = w.createAgent(crc, "crc");
    Agent observer = w.createAgent(watch, "watch");
    CHECK(w.rbac(a.id()).roles.size() == 5); // (A-GRA's defaults)

    control::HsaCommand south;
    south.headingRad = 180.0 * kDeg, south.speed = 230.0, south.altitudeM = 6500.0;
    // no role declared: unauthorized, not acted on
    const std::uint64_t early = boss.commandHsa(blue.system(), south);
    REQUIRE(early != 0);
    auto st = statusOf(w, boss, early, comm::topic::kFlightCommand);
    REQUIRE(st);
    CHECK(st->state == comm::CommandStatus::Unauthorized);
    CHECK(blue.authority().commands.empty());

    // roles: an unknown one rejected; QB and Observer declared; the MA's agent reads who is who
    REQUIRE(boss.declareRole(99, "nobody", {blue.system()}));
    st = statusOf(w, boss, 0, comm::topic::kOperator);
    REQUIRE(st);
    CHECK(st->state == comm::CommandStatus::Rejected);
    REQUIRE(boss.declareRole(2, "CRC LEAD"));
    REQUIRE(observer.declareRole(5, "WATCH"));
    w.step(15);
    const C2Authority before = blue.authority();
    REQUIRE(before.operators.size() == 2);
    for (const auto& o : before.operators) {
        if (o.system == boss.system()) CHECK((o.role == "QB" && o.callsign == "CRC LEAD"));
        if (o.system == observer.system()) CHECK(o.role == "Observer");
    }
    // an Observer is read-only
    const std::uint64_t peek = observer.commandHsa(blue.system(), south);
    st = statusOf(w, observer, peek, comm::topic::kFlightCommand);
    REQUIRE(st);
    CHECK(st->state == comm::CommandStatus::Unauthorized);

    // the QB's direct command: flown as C2's, under MA's secondary control
    const std::uint64_t order = boss.commandHsa(blue.system(), south);
    st = statusOf(w, boss, order, comm::topic::kFlightCommand);
    REQUIRE(st);
    REQUIRE(st->state == comm::CommandStatus::Accepted);
    const control::ActivityId c2Activity = st->activity;
    const auto held = blue.authority().commands;
    REQUIRE(held.size() == 1);
    CHECK((held[0].activity == c2Activity && held[0].system == boss.system() && held[0].commandId == order));
    // the agent's own on its axes: authority_held, naming C2's activity; C2's activity is not the agent's to change
    control::HsaCommand north;
    north.headingRad = 0.0, north.speed = 230.0, north.altitudeM = 6000.0;
    const auto mine = blue.submit(north);
    CHECK(mine.reason == control::Reason::AuthorityHeld);
    CHECK(mine.other == c2Activity);
    CHECK_FALSE(blue.cancel(c2Activity).accepted());
    CHECK(w.activity(c2Activity)->state == control::ActivityState::Active);

    // C2 cancels it: then its end reported, and the agent flies again
    REQUIRE(boss.cancelCommand(blue.system(), order) == order);
    st = statusOf(w, boss, order, comm::topic::kFlightCommand, comm::CommandStatus::Canceled);
    REQUIRE(st);
    CHECK(blue.authority().commands.empty());
    CHECK(blue.submit(north).accepted());
}

TEST_CASE("agents: C2 authority (AB-5a) - a control designation: modes allowed, the rest revoked, for MA and C2 alike", "[agents][c2]") {
    World w(options("agents-c2-designation"));
    Vehicle a = aircraft(w, "viper", "f16c", 0.0, 0.0, 6000.0, 0.0, 230.0);
    const std::uint32_t crc = w.createC2Node(c2At("crc", -40.0, 0.0, 200.0));
    const comm::NetId net = w.createNet({});
    REQUIRE(w.joinNet(a.id(), net));
    REQUIRE(w.joinNet(crc, net));
    Agent blue = w.createAgent(a, "blue-1");
    Agent boss = w.createAgent(crc, "crc");
    // an AVC: flight and control, not weapons
    REQUIRE(boss.declareRole(3, "AVC-1"));
    w.step(10);
    const std::uint32_t hsaOnly = 1u << static_cast<unsigned>(control::FlightMode::HsaCsa);
    const std::uint64_t d = boss.designateControl(blue.system(), hsaOnly);
    auto st = statusOf(w, boss, d, comm::topic::kControlDesignation);
    REQUIRE(st);
    REQUIRE(st->state == comm::CommandStatus::Accepted);
    const C2Authority auth = blue.authority();
    CHECK(auth.designated);
    CHECK(auth.designatedModes == hsaOnly);
    CHECK(auth.designatedBy == boss.system());
    CHECK(a.controlMode() == control::ControlMode::Granted);
    // the agent: HSA granted to it; waypoints revoked
    control::HsaCommand east;
    east.headingRad = 90.0 * kDeg, east.speed = 230.0, east.altitudeM = 6000.0;
    const auto eastResult = blue.submit(east);
    INFO(control::reasonName(eastResult.reason));
    CHECK(eastResult.accepted());
    control::Waypoint wp;
    wp.latitudeRad = 40.3 * kDeg, wp.longitudeRad = 0.0, wp.altitudeM = 6000.0;
    const auto route = blue.submit(control::RouteCommand{}, Span<const control::Waypoint>(&wp, 1));
    CHECK_FALSE(route.accepted());
    // C2's own route on a mode it revoked: rejected
    const std::uint64_t r = boss.commandRoute(blue.system(), Span<const control::Waypoint>(&wp, 1));
    st = statusOf(w, boss, r, comm::topic::kFlightCommand);
    REQUIRE(st);
    CHECK(st->state == comm::CommandStatus::Rejected);
    CHECK(st->reason == static_cast<std::uint32_t>(control::Reason::NotAllowed));
}

TEST_CASE("agents: C2 authority (AB-5b) - a task decided by the agent, its answers back to C2", "[agents][c2]") {
    World w(options("agents-c2-tasks"));
    Vehicle a = aircraft(w, "viper", "f16c", 0.0, 0.0, 6000.0, 0.0, 230.0);
    const std::uint32_t crc = w.createC2Node(c2At("crc", -40.0, 0.0, 200.0));
    const std::uint32_t avc = w.createC2Node(c2At("avc", -45.0, 0.0, 200.0));
    const comm::NetId net = w.createNet({});
    for (const std::uint32_t e : {a.id(), crc, avc}) REQUIRE(w.joinNet(e, net));
    Agent blue = w.createAgent(a, "blue-1");
    Agent boss = w.createAgent(crc, "crc");
    Agent controller = w.createAgent(avc, "avc");
    REQUIRE(boss.declareRole(2, "CRC"));
    REQUIRE(controller.declareRole(3, "AVC"));
    w.step(10);
    // a patrol: the agent's to decide; its acceptance, progress and completion go back
    comm::TaskOrder patrol;
    patrol.type = comm::TaskOrder::Patrol;
    patrol.latitudeRad = 40.5 * kDeg, patrol.longitudeRad = 0.0, patrol.altitudeM = 7000.0, patrol.radiusM = 20000.0;
    std::strcpy(patrol.text, "CAP north");
    const std::uint64_t t1 = boss.sendTask(blue.system(), patrol);
    REQUIRE(t1 != 0);
    w.step(15);
    auto tasks = blue.c2Tasks();
    REQUIRE(tasks.size() == 1);
    CHECK(tasks[0].from == boss.system());
    CHECK(tasks[0].state == comm::ReceivedTask::Received);
    CHECK(std::string(tasks[0].order.text) == "CAP north");
    CHECK_FALSE(blue.reportTask(boss.system(), t1, comm::ReceivedTask::Completed, 100.0)); // (not accepted yet)
    REQUIRE(blue.acceptTask(boss.system(), t1));
    auto st = statusOf(w, boss, t1, comm::topic::kTask, comm::CommandStatus::Accepted);
    REQUIRE(st);
    REQUIRE(blue.reportTask(boss.system(), t1, comm::ReceivedTask::Accepted, 40.0));
    w.step(15); // (the report over the link)
    st = statusOf(w, boss, t1, comm::topic::kTask);
    REQUIRE(st);
    CHECK(st->percent == 40.0);
    REQUIRE(blue.reportTask(boss.system(), t1, comm::ReceivedTask::Completed, 100.0));
    st = statusOf(w, boss, t1, comm::topic::kTask, comm::CommandStatus::Completed);
    REQUIRE(st);
    CHECK_FALSE(blue.acceptTask(boss.system(), t1)); // (ended)
    // a strike task from an AVC: weapons are not its role's - unauthorized, never the agent's
    comm::TaskOrder strike;
    strike.type = comm::TaskOrder::Strike;
    const std::uint64_t t2 = controller.sendTask(blue.system(), strike);
    st = statusOf(w, controller, t2, comm::topic::kTask);
    REQUIRE(st);
    CHECK(st->state == comm::CommandStatus::Unauthorized);
    CHECK(blue.c2Tasks().size() == 1);
    // rejected, with the agent's reason
    const std::uint64_t t3 = controller.sendTask(blue.system(), patrol);
    w.step(15);
    REQUIRE(blue.rejectTask(controller.system(), t3, 7));
    st = statusOf(w, controller, t3, comm::topic::kTask, comm::CommandStatus::Rejected);
    REQUIRE(st);
    CHECK(st->reason == 7);
}

TEST_CASE("agents: MA removal retains history and settles C2, overrides and lost communication at their existing steps",
          "[agents][c2][lifecycle]") {
    World w(options("agents-ma-removal"));
    Vehicle a = aircraft(w, "viper", "f16c", 0.0, 0.0, 6000.0, 0.0, 230.0);
    const auto node = w.createC2Node(c2At("boss", -40.0, 0.0, 200.0));
    const auto net = w.createNet({});
    REQUIRE(w.joinNet(a.id(), net));
    REQUIRE(w.joinNet(node, net));
    Agent blue = w.createAgent(a, "blue");
    Agent boss = w.createAgent(node, "boss");
    REQUIRE(boss.declareRole(2, "QB"));
    w.step(10);

    comm::TaskOrder patrol;
    patrol.type = comm::TaskOrder::Patrol;
    std::strcpy(patrol.text, "retained patrol");
    const auto completed = boss.sendTask(blue.system(), patrol);
    const auto pending = boss.sendTask(blue.system(), patrol);
    REQUIRE(completed != 0);
    REQUIRE(pending != 0);
    w.step(15);
    REQUIRE(blue.acceptTask(boss.system(), completed));
    REQUIRE(blue.reportTask(boss.system(), completed, comm::ReceivedTask::Completed, 100.0));
    REQUIRE(statusOf(w, boss, completed, comm::topic::kTask, comm::CommandStatus::Completed));

    comm::OverrideCommand finite;
    finite.kind = comm::OverrideCommand::Altitude;
    finite.value = 7000.0;
    finite.expiryS = w.time() + 4.0;
    REQUIRE(statusOf(w, boss, boss.setOverride(blue.system(), finite), comm::topic::kOverride));
    comm::OverrideCommand indefinite;
    indefinite.kind = comm::OverrideCommand::Speed;
    indefinite.value = 240.0; // NaN expiry: retained until canceled.
    REQUIRE(statusOf(w, boss, boss.setOverride(blue.system(), indefinite), comm::topic::kOverride));
    const auto lostSettings = boss.setLostComms(blue.system(), 5.0);
    REQUIRE(statusOf(w, boss, lostSettings, comm::topic::kLostComms));
    const auto direct = boss.commandHsa(blue.system(), hsa(30.0));
    const auto accepted = statusOf(w, boss, direct, comm::topic::kFlightCommand, comm::CommandStatus::Accepted);
    REQUIRE(accepted);
    const auto system = blue.system();
    const auto before = blue.authority();
    REQUIRE(before.commands.size() == 1);
    REQUIRE(blue.overrides().size() == 2);
    REQUIRE_FALSE(before.lost);
    const auto tasksBefore = blue.c2Tasks();
    REQUIRE(tasksBefore.size() == 2);

    REQUIRE(a.remove());
    CHECK(blue.system() == system);
    CHECK(blue.authority().commands.size() == 1); // C2 clearing remains deferred to MA's next step.
    CHECK(blue.authority().operators.size() == 1);
    CHECK(blue.c2Tasks().size() == 2);
    CHECK(blue.overrides().size() == 2);
    CHECK_FALSE(w.joinNet(a.id(), net));
    REQUIRE(blue.acceptTask(boss.system(), pending)); // Local history still changes although its reply cannot send.
    CHECK_FALSE(blue.acceptTask(boss.system(), completed));
    CHECK(blue.c2Tasks()[0].state == comm::ReceivedTask::Completed);
    CHECK(blue.c2Tasks()[1].state == comm::ReceivedTask::Accepted);
    CHECK(blue.c2Tasks()[0].percent == 100.0);

    w.step();
    CHECK(blue.authority().commands.empty());
    CHECK(blue.overrides().size() == 2);
    CHECK_FALSE(blue.authority().lost);
    for (const auto& status : boss.commandStatuses())
        if (status.status.commandId == direct && status.status.topic == comm::topic::kFlightCommand)
            CHECK(status.status.state == comm::CommandStatus::Accepted); // A removed sender cannot send its final status.

    while (w.time() + w.stepSeconds() < finite.expiryS) w.step();
    REQUIRE(blue.overrides().size() == 2);
    w.step();
    REQUIRE(blue.overrides().size() == 1);
    CHECK(std::isnan(blue.overrides()[0].expiryS));
    while (!blue.authority().lost && w.time() < before.lostSinceS + 20.0) w.step();
    const auto lost = blue.authority();
    CHECK(lost.lost);
    CHECK(lost.alerts == before.alerts + 1);
    CHECK(w.time() - lost.lostSinceS > 5.0);
    CHECK(w.time() - w.stepSeconds() - lost.lostSinceS <= 5.0);
    w.step(10);
    CHECK(blue.authority().alerts == lost.alerts);
    CHECK(blue.authority().lostSinceS == lost.lostSinceS);

    Vehicle replacement = aircraft(w, "replacement", "f16c", 0.0, 0.0, 6000.0, 0.0, 230.0);
    CHECK(replacement.id() != a.id());
    REQUIRE(w.joinNet(replacement.id(), net));
    Agent fresh = w.createAgent(replacement, "fresh");
    CHECK(fresh.c2Tasks().empty());
    CHECK(fresh.overrides().empty());
    CHECK(fresh.authority().operators.empty());
    CHECK(blue.c2Tasks().size() == 2);
    CHECK(w.removeC2Node(node));
    CHECK(boss.system() != comm::kNoSystem);
    CHECK_FALSE(boss.commandStatuses().empty());
    comm::DmsReason why = comm::DmsReason::None;
    CHECK(boss.sendTask(fresh.system(), patrol, &why) == 0);
    CHECK(why == comm::DmsReason::NoSystem);
}

TEST_CASE("agents: reset preserves MA authority, tasks, overrides and C2 ownership", "[agents][c2][lifecycle]") {
    World w(options("agents-ma-reset"));
    Vehicle a = aircraft(w, "viper", "f16c", 0.0, 0.0, 6000.0, 0.0, 230.0);
    const auto node = w.createC2Node(c2At("boss", -40.0, 0.0, 200.0));
    const auto net = w.createNet({});
    REQUIRE(w.joinNet(a.id(), net));
    REQUIRE(w.joinNet(node, net));
    Agent blue = w.createAgent(a, "blue");
    Agent boss = w.createAgent(node, "boss");
    REQUIRE(boss.declareRole(2, "QB"));
    w.step(10);
    const auto modes = 1u << static_cast<unsigned>(control::FlightMode::HsaCsa);
    REQUIRE(statusOf(w, boss, boss.designateControl(blue.system(), modes), comm::topic::kControlDesignation));
    comm::TaskOrder patrol;
    REQUIRE(boss.sendTask(blue.system(), patrol) != 0);
    comm::OverrideCommand high;
    high.kind = comm::OverrideCommand::Altitude;
    high.value = 7000.0;
    REQUIRE(statusOf(w, boss, boss.setOverride(blue.system(), high), comm::topic::kOverride));
    REQUIRE(statusOf(w, boss, boss.setLostComms(blue.system(), 30.0), comm::topic::kLostComms));
    const auto direct = boss.commandHsa(blue.system(), hsa(20.0));
    const auto status = statusOf(w, boss, direct, comm::topic::kFlightCommand, comm::CommandStatus::Accepted);
    REQUIRE(status);
    REQUIRE(blue.c2Tasks().size() == 1);
    const auto before = blue.authority();
    REQUIRE(a.reset());
    const auto after = blue.authority();
    CHECK(after.designatedModes == modes);
    CHECK(after.designatedBy == before.designatedBy);
    CHECK(after.operators.size() == before.operators.size());
    CHECK(after.lostCommsSet);
    CHECK(after.alerts == before.alerts);
    REQUIRE(after.commands.size() == 1);
    CHECK(after.commands[0].activity == status->activity);
    CHECK(blue.c2Tasks()[0].state == comm::ReceivedTask::Received);
    CHECK(blue.overrides()[0].value == 7000.0);
    REQUIRE(w.activity(status->activity));
    CHECK(w.activity(status->activity)->state == control::ActivityState::Pending);
    CHECK(blue.cancel(status->activity).reason == control::Reason::NotAllowed);
    w.step();
    CHECK(w.activity(status->activity)->live());
    CHECK(blue.authority().commands.size() == 1);
}

TEST_CASE("agents: consent keeps first-answer and late unanswered reply status", "[agents][c2][lifecycle]") {
    World w(options("agents-ma-consent-history"));
    Vehicle a = aircraft(w, "viper", "f16c", 0.0, 0.0, 6000.0, 0.0, 230.0);
    REQUIRE(a.loadLoadout("mk82 x4") == ms::Reason::None);
    const auto firstNode = w.createC2Node(c2At("first", -40.0, 0.0, 200.0));
    const auto secondNode = w.createC2Node(c2At("second", -45.0, 0.0, 200.0));
    const auto net = w.createNet({});
    for (const auto entity : {a.id(), firstNode, secondNode}) REQUIRE(w.joinNet(entity, net));
    Agent blue = w.createAgent(a, "blue");
    Agent first = w.createAgent(firstNode, "first");
    Agent second = w.createAgent(secondNode, "second");
    REQUIRE(first.declareRole(2, "QB1"));
    REQUIRE(second.declareRole(2, "QB2"));
    w.step(10);
    REQUIRE(blue.submit(ms::StateCommand{4, ms::PayloadState::Operate}).accepted());
    w.step();
    ms::StrikeCommand strike;
    strike.payload = 4;
    strike.store = "mk82";
    strike.quantity = 1;
    const auto r = blue.submit(strike);
    REQUIRE(r.accepted());
    w.step(static_cast<unsigned>(std::lround(3.0 / w.stepSeconds())));
    REQUIRE(first.consentRequests().size() == 1);
    REQUIRE(second.consentRequests().size() == 1);
    const auto request = first.consentRequests()[0].request.requestId;
    CHECK(second.consentRequests()[0].request.requestId == request);
    REQUIRE(first.answerConsent(blue.system(), request, true));
    const auto accepted = statusOf(w, first, request, comm::topic::kConsentAnswer, comm::CommandStatus::Accepted);
    REQUIRE(accepted);
    REQUIRE(second.answerConsent(blue.system(), request, false)); // The first answer has arrived despite link jitter.
    const auto duplicate = statusOf(w, second, request, comm::topic::kConsentAnswer, comm::CommandStatus::Unauthorized);
    REQUIRE(duplicate);
    CHECK(duplicate->reason == static_cast<std::uint32_t>(control::Reason::NotAllowed));
    CHECK_FALSE(first.answerConsent(blue.system(), request, true));
    CHECK(first.consentRequests()[0].answered);
    CHECK(second.consentRequests()[0].answered);

    const auto lateStrike = blue.submit(strike);
    REQUIRE(lateStrike.accepted());
    w.step(static_cast<unsigned>(std::lround(3.0 / w.stepSeconds())));
    REQUIRE(first.consentRequests().size() == 2);
    const auto late = first.consentRequests()[1].request.requestId;
    REQUIRE(blue.cancelMission(lateStrike.activity).status == control::CommandStatus::Canceled);
    w.step(2); // The unanswered request must remain addressable after its strike ends.
    REQUIRE(first.answerConsent(blue.system(), late, true));
    const auto rejected = statusOf(w, first, late, comm::topic::kConsentAnswer, comm::CommandStatus::Rejected);
    REQUIRE(rejected);
    CHECK(rejected->reason == static_cast<std::uint32_t>(ms::Reason::ActivityEnded));
    CHECK(rejected->activity == lateStrike.activity);
    CHECK(first.consentRequests().size() == 2);
    CHECK(second.consentRequests().size() == 2);
    CHECK_FALSE(second.consentRequests()[1].answered);
}

TEST_CASE("agents: C2 authority (AB-5b) - overrides on the agent's flight; release consent over the link", "[agents][c2]") {
    World w(options("agents-c2-consent"));
    Vehicle a = aircraft(w, "viper", "f16c", 0.0, 0.0, 6000.0, 0.0, 230.0);
    REQUIRE(a.loadLoadout("mk82 x4") == ms::Reason::None);
    const std::uint32_t crc = w.createC2Node(c2At("crc", -40.0, 0.0, 200.0));
    const std::uint32_t avc = w.createC2Node(c2At("avc", -45.0, 0.0, 200.0));
    const comm::NetId net = w.createNet({});
    for (const std::uint32_t e : {a.id(), crc, avc}) REQUIRE(w.joinNet(e, net));
    Agent blue = w.createAgent(a, "blue-1");
    Agent boss = w.createAgent(crc, "crc");
    Agent controller = w.createAgent(avc, "avc");
    REQUIRE(boss.declareRole(2, "CRC"));
    REQUIRE(controller.declareRole(3, "AVC"));
    w.step(10);

    // an altitude override: the agent's live HSA and its next fly it
    control::HsaCommand north;
    north.headingRad = 0.0, north.speed = 230.0, north.altitudeM = 6000.0;
    const auto own = blue.submit(north);
    REQUIRE(own.accepted());
    comm::OverrideCommand high;
    high.kind = comm::OverrideCommand::Altitude, high.value = 7500.0, high.expiryS = w.time() + 30.0;
    const std::uint64_t o1 = controller.setOverride(blue.system(), high);
    auto st = statusOf(w, controller, o1, comm::topic::kOverride);
    REQUIRE(st);
    REQUIRE(st->state == comm::CommandStatus::Accepted);
    REQUIRE(blue.overrides().size() == 1);
    CHECK(blue.overrides()[0].value == 7500.0);
    auto altitudeOf = [&](control::ActivityId id) {
        const auto sp = blue.activitySetpoint(id);
        return sp ? std::get<control::HsaCommand>(std::get<control::Command>(sp->command)).altitudeM : std::nan("");
    };
    CHECK(altitudeOf(own.activity) == 7500.0);
    const auto next = blue.submit(north);
    REQUIRE(next.accepted());
    CHECK(altitudeOf(next.activity) == 7500.0);
    w.step(static_cast<unsigned>(std::lround(31.0 / w.stepSeconds())));
    CHECK(blue.overrides().empty()); // (expired)

    // a strike asks the approving role (the QB) for release consent: an AVC may not answer; the QB grants it
    REQUIRE(blue.submit(ms::StateCommand{4, ms::PayloadState::Operate}).accepted());
    w.step();
    ms::StrikeCommand strike;
    strike.payload = 4, strike.store = "mk82", strike.quantity = 1;
    const ms::CommandResult r = blue.submit(strike);
    REQUIRE(r.accepted());
    w.step(static_cast<unsigned>(std::lround(3.0 / w.stepSeconds())));
    CHECK(blue.strikeStatus(r.activity)->phase == ms::StrikePhase::AwaitingConsent);
    CHECK(controller.consentRequests().empty()); // (not an approver: not asked)
    const auto asked = boss.consentRequests();
    REQUIRE(asked.size() == 1);
    CHECK(asked[0].from == blue.system());
    CHECK(asked[0].request.strike == r.activity);
    CHECK(std::string(asked[0].request.store) == "mk82");
    CHECK(asked[0].request.quantity == 1);
    CHECK_FALSE(controller.answerConsent(blue.system(), asked[0].request.requestId, true)); // (it holds no such request)
    REQUIRE(boss.answerConsent(blue.system(), asked[0].request.requestId, true));
    w.step(static_cast<unsigned>(std::lround(3.0 / w.stepSeconds())));
    CHECK(blue.strikeStatus(r.activity)->phase != ms::StrikePhase::AwaitingConsent);
    CHECK(!w.referee().weapons().empty());

    // the policy: AUTO_REJECT - the MA core refuses itself, nobody asked
    comm::RbacConfig rejecting = w.rbac(a.id());
    rejecting.strikePolicy = comm::StrikePolicy::AutoReject;
    w.setRbac(a.id(), rejecting);
    const ms::CommandResult r2 = blue.submit(strike);
    REQUIRE(r2.accepted());
    w.step(static_cast<unsigned>(std::lround(3.0 / w.stepSeconds())));
    CHECK(boss.consentRequests().size() == 1);
    const auto ended = blue.missionActivity(r2.activity);
    REQUIRE(ended);
    CHECK(ended->reason == ms::Reason::ConsentDenied);
}

TEST_CASE("agents: C2 authority (AB-5c) - lost communication: its alert, its plan activated; heard again, armed again", "[agents][c2]") {
    World w(options("agents-c2-lost"));
    Vehicle a = aircraft(w, "viper", "f16c", 0.0, 0.0, 6000.0, 0.0, 230.0);
    const std::uint32_t crc = w.createC2Node(c2At("crc", -40.0, 0.0, 200.0));
    const comm::NetId net = w.createNet({});
    REQUIRE(w.joinNet(a.id(), net));
    REQUIRE(w.joinNet(crc, net));
    comm::DisseminationPlan heartbeat;
    heartbeat.statusPeriodS = 2.0;
    REQUIRE(w.setDissemination(crc, heartbeat)); // (C2's heartbeats: what the MA hears of it)
    Agent blue = w.createAgent(a, "blue-1");
    Agent boss = w.createAgent(crc, "crc");
    // the MA's own return-to-base plan, uploaded beforehand
    control::RoutePlan rtb;
    rtb.id = 9, rtb.version = 1;
    for (const double north : {-20.0, -60.0}) {
        control::Waypoint wp;
        wp.latitudeRad = (40.0 + north / 111.2) * kDeg, wp.longitudeRad = 0.0, wp.altitudeM = 6000.0;
        rtb.waypoints.push_back(wp);
    }
    REQUIRE(blue.planCommand(9, control::PlanCommand::PrepareForUpload).completed);
    REQUIRE(blue.publishPlan(rtb) == control::Reason::None);
    REQUIRE(blue.planCommand(9, control::PlanCommand::Upload).completed);
    REQUIRE(boss.declareRole(3, "AVC")); // (settings: an AVC's)
    w.step(10);
    const std::uint64_t set = boss.setLostComms(blue.system(), 10.0, 9);
    auto st = statusOf(w, boss, set, comm::topic::kLostComms);
    REQUIRE(st);
    REQUIRE(st->state == comm::CommandStatus::Accepted);
    C2Authority auth = blue.authority();
    CHECK((auth.lostCommsSet && auth.lostCommsThresholdS == 10.0 && auth.lostCommsPlan == 9 && !auth.lost));
    w.step(static_cast<unsigned>(std::lround(15.0 / w.stepSeconds())));
    CHECK_FALSE(blue.authority().lost); // (its heartbeats heard)
    // C2 off the net: nothing heard for 10 s - the contingency raised, the plan flown
    REQUIRE(w.leaveNet(crc, net));
    w.step(static_cast<unsigned>(std::lround(12.0 / w.stepSeconds())));
    auth = blue.authority();
    CHECK(auth.lost);
    CHECK(auth.alerts == 1);
    const auto plan = blue.planStatus(9);
    REQUIRE(plan);
    CHECK(plan->state == control::PlanState::Activated);
    // back on the net: heard again - and armed for the next loss
    REQUIRE(w.joinNet(crc, net));
    w.step(static_cast<unsigned>(std::lround(4.0 / w.stepSeconds())));
    CHECK_FALSE(blue.authority().lost);
}

TEST_CASE("agents: C2 authority (AB-5c) - a team: its members accept its leader's commands as C2's, until dissolved", "[agents][c2]") {
    World w(options("agents-c2-team"));
    Vehicle a = aircraft(w, "lead", "f16c", 0.0, 0.0, 6000.0, 0.0, 230.0);
    Vehicle b = aircraft(w, "wing", "f16c", -5.0, 0.0, 6000.0, 0.0, 230.0);
    Vehicle c = aircraft(w, "stranger", "f16c", -10.0, 0.0, 6000.0, 0.0, 230.0);
    const std::uint32_t crc = w.createC2Node(c2At("crc", -40.0, 0.0, 200.0));
    const comm::NetId net = w.createNet({});
    for (const std::uint32_t e : {a.id(), b.id(), c.id(), crc}) REQUIRE(w.joinNet(e, net));
    Agent lead = w.createAgent(a, "blue-1");
    Agent wing = w.createAgent(b, "blue-2");
    Agent stranger = w.createAgent(c, "blue-3");
    Agent boss = w.createAgent(crc, "crc");
    REQUIRE(boss.declareRole(3, "AVC"));
    w.step(10);
    control::HsaCommand west;
    west.headingRad = 270.0 * kDeg, west.speed = 230.0, west.altitudeM = 6000.0;
    // before a team: a peer is no C2 of the wingman's
    std::uint64_t order = lead.commandHsa(wing.system(), west);
    auto st = statusOf(w, lead, order, comm::topic::kFlightCommand);
    REQUIRE(st);
    CHECK(st->state == comm::CommandStatus::Unauthorized);
    const std::uint64_t team = boss.formPackage(7, lead.system(), {lead.system(), wing.system()});
    REQUIRE(team != 0);
    st = statusOf(w, boss, team, comm::topic::kPackage);
    REQUIRE(st);
    REQUIRE(st->state == comm::CommandStatus::Accepted);
    const C2Authority auth = wing.authority();
    CHECK((auth.packageId == 7 && auth.leader == lead.system() && auth.members.size() == 2));
    // its leader relays: accepted as C2's; a stranger's is not
    order = lead.commandHsa(wing.system(), west);
    st = statusOf(w, lead, order, comm::topic::kFlightCommand);
    REQUIRE(st);
    CHECK(st->state == comm::CommandStatus::Accepted);
    REQUIRE(wing.authority().commands.size() == 1);
    CHECK(wing.authority().commands[0].system == lead.system());
    const std::uint64_t theirs = stranger.commandHsa(wing.system(), west);
    st = statusOf(w, stranger, theirs, comm::topic::kFlightCommand);
    REQUIRE(st);
    CHECK(st->state == comm::CommandStatus::Unauthorized);
    // a leader does not remake its team; C2 dissolves it
    const std::uint64_t remake = lead.formPackage(8, lead.system(), {wing.system()});
    st = statusOf(w, lead, remake, comm::topic::kPackage);
    REQUIRE(st);
    CHECK(st->state == comm::CommandStatus::Unauthorized);
    REQUIRE(boss.dissolvePackage(7, {lead.system(), wing.system()}) != 0);
    w.step(15);
    CHECK(wing.authority().packageId == 0);
    order = lead.commandHsa(wing.system(), west);
    st = statusOf(w, lead, order, comm::topic::kFlightCommand);
    REQUIRE(st);
    CHECK(st->state == comm::CommandStatus::Unauthorized);
}

TEST_CASE("agents: a target by track (AB-2b) - designated, flown on its estimate, struck, and gone with it", "[agents][ms]") {
    World w(options("agents-designate"));
    Vehicle a = aircraft(w, "a", "f16c", 0.0, 0.0, 6000.0, 90.0, 230.0);
    Vehicle b = aircraft(w, "b", "f16c", 3.0, 55.0, 7000.0, 270.0, 230.0);
    REQUIRE(a.loadLoadout("mk82 x4") == ms::Reason::None);
    Agent blue = w.createAgent(a, "blue-1");
    REQUIRE(blue.submit(ms::StateCommand{0, ms::PayloadState::Operate}).accepted());
    REQUIRE(blue.submit(ms::StateCommand{4, ms::PayloadState::Operate}).accepted()); // (the stations: the master arm)
    w.step();
    ms::AmtiCommand search;
    search.elevationCentreRad = -1.0 * kDeg;
    search.bars = 4;
    REQUIRE(blue.submit(search).accepted());
    w.step(static_cast<unsigned>(std::lround(45.0 / w.stepSeconds())));
    const auto tracks = blue.amtiTracks(0);
    REQUIRE(!tracks.empty());
    const Referee ref = w.referee();
    std::uint64_t track = 0;
    for (const auto& t : tracks)
        if (ref.trackTruth(a.id(), 0, t.id) == b.id()) track = t.id;
    REQUIRE(track != 0);

    CHECK(blue.designate(0, 987654321u) == 0); // (a track it does not hold)
    const std::uint32_t target = blue.designate(0, track);
    REQUIRE(target > 0x80000000u);
    REQUIRE(blue.designations().size() == 1);
    CHECK(blue.designations()[0].valid);
    CHECK(ref.designationTruth(target) == b.id());

    // the designation reads as the track's estimate - near the bandit, not the bandit's own state
    w.step();
    const Referee::AgentInfo me = ref.agents().front();
    (void)me;
    const VehicleState* estimate = ref.state(target);
    REQUIRE(estimate != nullptr);
    const VehicleState* truth = ref.state(b.id());
    const double dLat = (estimate->latitudeRad - truth->latitudeRad) * 6.371e6;
    const double dLon = (estimate->longitudeRad - truth->longitudeRad) * 6.371e6 * std::cos(truth->latitudeRad);
    CHECK(std::hypot(dLat, dLon) < 2000.0);
    CHECK(std::hypot(dLat, dLon) > 0.0); // (an estimate: never the truth to the bit)

    // a pursuit and a must fly at its own designation: accepted; at another vehicle's: refused
    control::BehaviorCommand pursuit;
    pursuit.id = "pursuit";
    pursuit.target = target;
    const auto chase = blue.submit(pursuit);
    CHECK(chase.accepted());
    control::MustFlyCommand over;
    over.location = static_cast<double>(control::MustFlyLocation::Entity);
    over.target = static_cast<double>(target);
    Vehicle c = aircraft(w, "c", "f16c", -20.0, 0.0, 6000.0, 90.0, 230.0);
    REQUIRE(c.submit(ms::StateCommand{0, ms::PayloadState::Operate}).accepted());
    Agent green = w.createAgent(c, "green-1");
    control::BehaviorCommand theirs = pursuit;
    CHECK(green.submit(theirs).reason == control::Reason::NotAllowed); // (blue's designation, not green's)
    CHECK(green.submit(over).reason == control::Reason::NotAllowed);
    w.step(static_cast<unsigned>(std::lround(5.0 / w.stepSeconds())));
    CHECK(w.activity(chase.activity)->state == control::ActivityState::Active);
    const auto overIt = blue.submit(over); // (a must fly over its own designation: the track's estimate, flown over)
    INFO(control::reasonName(overIt.reason));
    CHECK(overIt.accepted());

    // a strike at it: its weapons carry the designation, not the entity
    ms::StrikeCommand strike;
    strike.payload = 4;
    strike.store = "mk82";
    strike.quantity = 1;
    strike.target = target;
    const ms::CommandResult r = blue.submit(strike);
    REQUIRE(r.accepted());
    w.step(static_cast<unsigned>(std::lround(2.0 / w.stepSeconds())));
    REQUIRE(a.strikeConsent(r.activity, true) == ms::Reason::None); // (the range, standing in for C2)
    w.step(static_cast<unsigned>(std::lround(2.0 / w.stepSeconds())));
    const auto weapons = ref.weapons();
    REQUIRE(!weapons.empty());
    CHECK(weapons.front().target == target);

    // undesignated, it is gone: the pursuit has no target
    CHECK(blue.undesignate(target));
    CHECK_FALSE(blue.undesignate(target));
    CHECK(ref.state(target) == nullptr);
    CHECK_FALSE(blue.designations()[0].valid);
}

namespace {

/// A circle of `radiusM` about a place `northKm` and `eastKm` from 40 N 0 E.
control::OpZone circle(control::OpZoneId id, double northKm, double eastKm, double radiusM) {
    control::OpZone z;
    z.id = id;
    z.shape = static_cast<double>(control::ZoneShape::Ellipse);
    z.latitudeRad = (40.0 + northKm / 111.2) * kDeg;
    z.longitudeRad = (eastKm / (111.2 * std::cos(40.0 * kDeg))) * kDeg;
    z.semiMajorM = z.semiMinorM = radiusM;
    return z;
}

} // namespace

TEST_CASE("agents: mission data is each MA's own or the range's common - no MA sees another's (AB-2c, L11)", "[agents]") {
    World w(options("agents-mission-data"));
    Vehicle a = aircraft(w, "a", "f16c", 0.0, 0.0, 6000.0, 0.0, 230.0);
    Vehicle b = aircraft(w, "b", "f16c", 20.0, 0.0, 6000.0, 180.0, 230.0);
    Agent blue = w.createAgent(a, "blue-1");
    Agent red = w.createAgent(b, "red-1");
    // the range's common mission data: every MA reads it; the range alone changes it
    REQUIRE(w.setOpZone(circle(1, 30.0, 0.0, 2000.0)) == control::Reason::None);
    // each MA's own, the same id: two zones, neither seen by the other
    REQUIRE(blue.setOpZone(circle(2, 10.0, 0.0, 1000.0)) == control::Reason::None);
    REQUIRE(red.setOpZone(circle(2, 10.0, 20.0, 3000.0)) == control::Reason::None);
    REQUIRE(blue.setOpZone(circle(3, 5.0, 5.0, 500.0)) == control::Reason::None);
    CHECK(w.opZones() == std::vector<control::OpZoneId>{1});
    CHECK(blue.opZones() == std::vector<control::OpZoneId>{1, 2, 3});
    CHECK(red.opZones() == std::vector<control::OpZoneId>{1, 2});
    REQUIRE(blue.opZone(2));
    REQUIRE(red.opZone(2));
    CHECK(blue.opZone(2)->semiMajorM == 1000.0);
    CHECK(red.opZone(2)->semiMajorM == 3000.0);
    CHECK_FALSE(red.opZone(3));
    CHECK(red.opZone(1)->semiMajorM == 2000.0);
    // its own shadows the common one of its id; removing its own leaves the common
    REQUIRE(blue.setOpZone(circle(1, 30.0, 0.0, 600.0)) == control::Reason::None);
    CHECK(blue.opZone(1)->semiMajorM == 600.0);
    CHECK(red.opZone(1)->semiMajorM == 2000.0);
    CHECK(blue.removeOpZone(1));
    CHECK_FALSE(blue.removeOpZone(1)); // (the common one is not its to remove)
    CHECK(blue.opZone(1)->semiMajorM == 2000.0);
    // its commands name what it sees: its own zone 3, the common 1; red has no zone 3
    control::MustFlyCommand into;
    into.location = static_cast<double>(control::MustFlyLocation::OpZone);
    into.target = 3.0;
    const auto mine = blue.submit(into);
    INFO(control::reasonName(mine.reason));
    CHECK(mine.accepted());
    CHECK_FALSE(red.submit(into).accepted());
    into.target = 1.0;
    CHECK(red.submit(into).accepted());
    // frames: its own may follow itself, not another aircraft; the range's following another is not there for it
    control::FrameSpec followRed;
    followRed.origin = control::FrameOrigin::Vehicle, followRed.vehicle = b.id();
    control::FrameSpec followSelf = followRed;
    followSelf.vehicle = a.id();
    CHECK(blue.createFrame(followRed) == 0);
    const control::FrameId own = blue.createFrame(followSelf);
    REQUIRE(own != 0);
    CHECK(blue.frame(own));
    CHECK_FALSE(red.frame(own));
    const control::FrameId ranges = w.createFrame(followRed);
    REQUIRE(ranges != 0);
    CHECK_FALSE(blue.frame(ranges));
    CHECK_FALSE(blue.framePoint(ranges, control::FrameOffset{}, std::nan("")));
    const auto seen = blue.frames();
    CHECK(std::find(seen.begin(), seen.end(), ranges) == seen.end());
    CHECK(std::find(seen.begin(), seen.end(), own) != seen.end());
    CHECK(w.frame(ranges)); // (the range's own)
    CHECK(blue.removeFrame(own));
    CHECK_FALSE(blue.removeFrame(ranges));
}

TEST_CASE("agents: the audit counts every call an agent makes, and those refused it (AB-2d)", "[agents]") {
    World w(options("agents-audit"));
    Vehicle a = aircraft(w, "a", "f16c", 0.0, 0.0, 6000.0, 0.0, 230.0);
    Vehicle b = aircraft(w, "b", "f16c", 20.0, 0.0, 6000.0, 180.0, 230.0);
    Agent blue = w.createAgent(a, "blue-1");
    Agent red = w.createAgent(b, "red-1");
    REQUIRE(blue.submit(hsa(30.0)).accepted());
    REQUIRE(blue.submit(hsa(40.0)).accepted());
    control::CommandOptions override;
    override.source = control::Source::Override;
    CHECK(blue.submit(hsa(50.0), override).reason == control::Reason::NotAllowed);
    const auto theirs = b.submit(hsa(90.0));
    CHECK(blue.cancel(theirs.activity).reason == control::Reason::NotAllowed);
    (void)blue.ownship();
    (void)red.ownship();
    const auto audit = w.referee().audit();
    REQUIRE(audit.size() == 2);
    CHECK(audit[0].agent.name == "blue-1");
    auto find = [](const Referee::AgentAudit& x, const std::string& call) {
        for (const auto& c : x.calls)
            if (c.call == call) return c;
        return Referee::AuditCall{call, 0, 0};
    };
    CHECK(find(audit[0], "fsim::Agent::submit").calls == 3);
    CHECK(find(audit[0], "fsim::Agent::submit").refused == 1);
    CHECK(find(audit[0], "fsim::Agent::cancel").calls == 1);
    CHECK(find(audit[0], "fsim::Agent::cancel").refused == 1);
    CHECK(find(audit[0], "fsim::Agent::ownship").calls == 1);
    CHECK(find(audit[1], "fsim::Agent::ownship").calls == 1);
    CHECK(find(audit[1], "fsim::Agent::submit").calls == 0);
}
