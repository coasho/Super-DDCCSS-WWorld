// Legal reports with their provenance (docs/sdk/mission-training.md, section 10; fsim/MAReports.h), through the public
// C++ SDK. Each report is what the agent's own read gives - nothing more - with where it came from, when, and how old
// it is: an unknown stays unknown, a stale report says so, a report a link has not delivered is not there, and nothing
// the referee alone knows changes what an agent reads.
#include "fsim/Agent.h"
#include "fsim/BuiltinEffects.h"
#include "fsim/MAReports.h"
#include "fsim/World.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstring>
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
    o.seed = 3;
    return o;
}

Vehicle aircraft(World& w, const std::string& name, double northKm, double eastKm, double altitudeM, double headingDeg, double tasMs = 230.0) {
    VehicleSpec s;
    s.name = name;
    s.type = "jsbsim:f16c";
    s.initial.latitudeDeg = 40.0 + northKm / 111.2;
    s.initial.longitudeDeg = eastKm / (111.2 * std::cos(40.0 * kDeg));
    s.initial.altitudeMslM = altitudeM;
    s.initial.headingDeg = headingDeg;
    s.initial.airspeedTrueMs = tasMs;
    return w.createVehicle(s);
}

std::uint16_t payloadOf(const Vehicle& v, ms::Family family) {
    for (const ms::PayloadInfo& p : v.payloads())
        if (p.family == family) return p.index;
    return 0xFFFF;
}

unsigned steps(const World& w, double seconds) { return static_cast<unsigned>(std::lround(seconds / w.stepSeconds())); }

bool unknown(double x) { return std::isnan(x); }

/// Byte equality of two trivially copyable reports.
template <typename T>
bool same(const T& a, const T& b) {
    return std::memcmp(&a, &b, sizeof(T)) == 0;
}

/// The same time, an unknown one equal to an unknown one.
bool sameTime(double a, double b) { return a == b || (std::isnan(a) && std::isnan(b)); }

/// The same report with the same provenance.
template <typename T>
bool same(const ma::Reported<T>& a, const ma::Reported<T>& b) {
    const ma::ReportMeta& m = a.meta;
    const ma::ReportMeta& n = b.meta;
    return same(a.value, b.value) && m.source == n.source && m.validity == n.validity && m.system == n.system && m.net == n.net &&
           m.payload == n.payload && sameTime(m.sampleTimeS, n.sampleTimeS) && sameTime(m.receiveTimeS, n.receiveTimeS) &&
           sameTime(m.ageS, n.ageS);
}

/// A lead searching with its radar and sharing its tracks, a wingman with its radar off, a bandit ahead of them.
struct TwoShip {
    World w;
    Vehicle lead, bandit, wing;
    Agent leadMa, wingMa;
    comm::NetId net = 0;
    comm::DisseminationPlan plan;
    explicit TwoShip(const char* name)
        : w(options(name)), lead(aircraft(w, "lead", 0.0, 0.0, 6000.0, 90.0)), bandit(aircraft(w, "bandit", 3.0, 55.0, 7000.0, 270.0)),
          wing(aircraft(w, "wing", -10.0, 0.0, 6000.0, 90.0)), leadMa(w.createAgent(lead, "blue-1")), wingMa(w.createAgent(wing, "blue-2")) {
        net = w.createNet({});
        REQUIRE(w.assignSystem(lead.id(), 0x4001, "VIPER11") == 0x4001);
        REQUIRE(w.joinNet(lead.id(), net));
        REQUIRE(w.joinNet(wing.id(), net));
        plan.positionPeriodS = 1.0, plan.tracksPeriodS = 2.0;
        REQUIRE(w.setDissemination(lead.id(), plan));
        REQUIRE(leadMa.submit(ms::StateCommand{payloadOf(lead, ms::Family::AirRadar), ms::PayloadState::Operate}).accepted());
        w.step();
        ms::AmtiCommand search;
        search.payload = payloadOf(lead, ms::Family::AirRadar);
        search.elevationCentreRad = -1.0 * kDeg;
        search.bars = 4;
        REQUIRE(leadMa.submit(search).accepted());
        w.step(steps(w, 45.0));
    }
};

} // namespace

TEST_CASE("ma reports: an own ship and a radar's tracks, each with where it came from and how old it is", "[ma][reports]") {
    TwoShip s("ma-reports-own");
    const double now = s.w.time();
    const ma::Report<ma::Navigation> nav = s.leadMa.reportNavigation();
    REQUIRE(nav.availability == ma::Availability::Available);
    CHECK(nav.meta.source == ma::Source::OwnNavigation);
    CHECK(nav.meta.validity == ma::Validity::Valid);
    CHECK(nav.meta.system == comm::kNoSystem);
    CHECK(nav.meta.payload == ma::kNoPayload);
    CHECK(nav.meta.sampleTimeS == nav.value.ownShip.timeS);
    CHECK(nav.meta.receiveTimeS == now);
    CHECK(nav.meta.ageS == now - nav.value.ownShip.timeS);
    CHECK(nav.meta.ageS >= 0.0);
    // the value is the agent's own read, no more
    const OwnShip own = s.leadMa.ownship();
    CHECK(nav.value.ownShip.latitudeRad == own.latitudeRad);
    CHECK(nav.value.ownShip.airspeedTrueMs == own.airspeedTrueMs);
    CHECK(nav.value.endurance.fuelKg == s.leadMa.navigationReport().fuelKg);

    const std::uint16_t radar = payloadOf(s.lead, ms::Family::AirRadar);
    const std::vector<ms::AmtiTrack> tracks = s.leadMa.amtiTracks(radar);
    const ma::Collection<ms::AmtiTrack> reported = s.leadMa.reportTracks();
    REQUIRE(reported.availability == ma::Availability::Available);
    REQUIRE_FALSE(tracks.empty());
    REQUIRE(reported.items.size() == tracks.size());
    for (std::size_t i = 0; i < tracks.size(); ++i) {
        const ma::Reported<ms::AmtiTrack>& r = reported.items[i];
        CHECK(same(r.value, tracks[i]));
        CHECK(r.value.truth == 0); // (the entity behind it is the referee's)
        CHECK(r.meta.source == ma::Source::OwnSensor);
        CHECK(r.meta.payload == radar);
        CHECK(r.meta.validity == ma::Validity::Valid);
        CHECK(r.meta.sampleTimeS == tracks[i].lastUpdateS);
        CHECK(r.meta.receiveTimeS == now);
        CHECK(r.meta.ageS == now - tracks[i].lastUpdateS);
    }
    // the range's own read of the same radar does carry the entity: the report did not come from there
    bool labelled = false;
    for (const ms::AmtiTrack& t : s.lead.amtiTracks(radar)) labelled = labelled || t.truth == s.bandit.id();
    CHECK(labelled);
    // reading is not consuming: again, the same
    const ma::Collection<ms::AmtiTrack> again = s.leadMa.reportTracks();
    REQUIRE(again.items.size() == reported.items.size());
    for (std::size_t i = 0; i < again.items.size(); ++i) CHECK(same(again.items[i], reported.items[i]));
    // stale is the caller's threshold on a held report's age, and only that
    ma::ReportOptions strict;
    strict.staleAfterS = 1.0e9;
    for (const auto& r : s.leadMa.reportTracks(strict).items) CHECK(r.meta.validity == ma::Validity::Valid);
    strict.staleAfterS = -1.0;
    for (const auto& r : s.leadMa.reportTracks(strict).items) CHECK(r.meta.validity == ma::Validity::Stale);
    CHECK(s.leadMa.reportNavigation(strict).meta.validity == ma::Validity::Stale);
    // the wingman's radar is off: its own tracks are an available, empty collection - not the lead's, not truth
    const ma::Collection<ms::AmtiTrack> none = s.wingMa.reportTracks();
    CHECK(none.availability == ma::Availability::Available);
    CHECK(none.items.empty());
}

TEST_CASE("ma reports: navigation its own sensors call invalid stays as reported, marked invalid", "[ma][reports]") {
    World w(options("ma-reports-invalid"));
    Vehicle a = aircraft(w, "viper", 0.0, 0.0, 6000.0, 90.0);
    Agent blue = w.createAgent(a, "blue-1");
    w.step(10);
    REQUIRE(blue.reportNavigation().meta.validity == ma::Validity::Valid);
    auto* outage = a.addEffect<effects::GnssDegradation>();
    REQUIRE(outage != nullptr);
    outage->lossProbabilityPerS = 1.0e9, outage->outageS = 1.0e6;
    w.step(steps(w, 2.0));
    const ma::Report<ma::Navigation> nav = blue.reportNavigation();
    REQUIRE(nav.availability == ma::Availability::Available);
    REQUIRE_FALSE(nav.value.ownShip.gnssValid);
    CHECK(nav.meta.validity == ma::Validity::Invalid);
    // what the platform reports is passed on as it is: not replaced by truth, not blanked
    CHECK(nav.value.ownShip.latitudeRad == blue.ownship().latitudeRad);
    CHECK(nav.meta.sampleTimeS == nav.value.ownShip.timeS);
}

TEST_CASE("ma reports: a link track is there only once delivered, as its source said it, until its picture drops it", "[ma][reports]") {
    TwoShip s("ma-reports-link");
    const Referee ref = s.w.referee();
    // wait for a delivery, so that the next is the tracks period away
    const auto newest = [&] {
        double t = -1.0;
        for (const comm::LinkTrack& l : s.wingMa.linkTracks()) t = std::max(t, l.receivedS);
        return t;
    };
    const double before = newest();
    REQUIRE(before > 0.0);
    int guard = 0;
    while (newest() == before && guard++ < 400) s.w.step();
    REQUIRE(newest() > before);

    const double t1 = s.w.time();
    const std::vector<comm::LinkTrack> held = s.wingMa.linkTracks();
    const ma::Collection<comm::LinkTrack> first = s.wingMa.reportLinkTracks();
    REQUIRE(first.availability == ma::Availability::Available);
    REQUIRE(first.items.size() == held.size());
    REQUIRE_FALSE(held.empty());
    const comm::LinkTrack* banditTrack = nullptr;
    for (std::size_t i = 0; i < held.size(); ++i) {
        const ma::Reported<comm::LinkTrack>& r = first.items[i];
        CHECK(same(r.value, held[i]));
        CHECK(r.meta.source == ma::Source::Link);
        CHECK(r.meta.system == 0x4001u);
        CHECK(r.meta.net == s.net);
        CHECK(r.meta.payload == held[i].report.payload);
        CHECK(r.meta.sampleTimeS == held[i].report.timeS);
        CHECK(r.meta.receiveTimeS == held[i].receivedS);
        CHECK(r.meta.ageS == t1 - held[i].report.timeS);
        CHECK(r.meta.receiveTimeS >= r.meta.sampleTimeS); // (a link delivers after its source spoke)
        CHECK(r.meta.validity == ma::Validity::Valid);
        if (ref.linkTrackTruth(held[i].source, static_cast<std::uint16_t>(held[i].report.payload), held[i].report.track) == s.bandit.id())
            banditTrack = &held[i];
    }
    REQUIRE(banditTrack != nullptr);
    const std::uint64_t banditNumber = banditTrack->report.track;
    const comm::EntityReport said = banditTrack->report;
    const VehicleState truthThen = *ref.state(s.bandit.id());

    // the bandit flies on; no report is delivered meanwhile: what the wingman holds of it does not move
    s.w.step(steps(s.w, 1.0));
    const double t2 = s.w.time();
    const VehicleState* truthNow = ref.state(s.bandit.id());
    REQUIRE(truthNow != nullptr);
    CHECK(truthNow->longitudeRad != truthThen.longitudeRad); // (hidden truth changed)
    const ma::Collection<comm::LinkTrack> second = s.wingMa.reportLinkTracks();
    REQUIRE(second.items.size() == first.items.size());
    for (std::size_t i = 0; i < second.items.size(); ++i) {
        CHECK(same(second.items[i].value, first.items[i].value));
        CHECK(second.items[i].meta.sampleTimeS == first.items[i].meta.sampleTimeS);
        CHECK(second.items[i].meta.receiveTimeS == first.items[i].meta.receiveTimeS);
        CHECK(std::abs(second.items[i].meta.ageS - (first.items[i].meta.ageS + (t2 - t1))) < 1e-9); // (only its age grew)
    }
    // the range moves an aircraft no sensor holds, and removes it: no actor report changes
    Vehicle ghost = aircraft(s.w, "ghost", -600.0, 0.0, 9000.0, 0.0);
    const ma::Collection<ms::AmtiTrack> leadBefore = s.leadMa.reportTracks();
    InitialConditions elsewhere;
    elsewhere.latitudeDeg = 30.0, elsewhere.longitudeDeg = 10.0, elsewhere.altitudeMslM = 3000.0, elsewhere.airspeedTrueMs = 200.0;
    REQUIRE(ghost.reset(elsewhere));
    REQUIRE(ghost.remove());
    const ma::Collection<ms::AmtiTrack> leadAfter = s.leadMa.reportTracks();
    REQUIRE(leadAfter.items.size() == leadBefore.items.size());
    for (std::size_t i = 0; i < leadAfter.items.size(); ++i) CHECK(same(leadAfter.items[i], leadBefore.items[i]));
    const ma::Collection<comm::LinkTrack> third = s.wingMa.reportLinkTracks();
    REQUIRE(third.items.size() == second.items.size());
    for (std::size_t i = 0; i < third.items.size(); ++i) CHECK(same(third.items[i], second.items[i]));

    // the next delivery: now the report changes, and says when it was sampled and when it arrived
    guard = 0;
    comm::LinkTrack updated;
    const auto held2 = [&] {
        for (const comm::LinkTrack& l : s.wingMa.linkTracks())
            if (l.report.track == banditNumber) return updated = l, true;
        return false;
    };
    bool there = held2();
    while (there && updated.report.timeS == said.timeS && guard++ < 400) s.w.step(), there = held2();
    REQUIRE(there);
    CHECK(updated.report.timeS > said.timeS);
    CHECK(updated.receivedS > t1);
    CHECK(updated.report.longitudeRad != said.longitudeRad);

    // a caller's threshold marks an old held report stale; with none, nothing is
    ma::ReportOptions strict;
    strict.staleAfterS = 0.0;
    for (const auto& r : s.wingMa.reportLinkTracks(strict).items) CHECK(r.meta.validity == ma::Validity::Stale);
    for (const auto& r : s.wingMa.reportLinkTracks().items) CHECK(r.meta.validity == ma::Validity::Valid);

    // peers: the lead, by its position reports - sampled when the lead reported, received when the link delivered
    const ma::Collection<comm::PeerReport> peers = s.wingMa.reportPeers();
    REQUIRE(peers.availability == ma::Availability::Available);
    REQUIRE(peers.items.size() == 1);
    REQUIRE(peers.items[0].value.hasPosition);
    CHECK(peers.items[0].meta.source == ma::Source::Link);
    CHECK(peers.items[0].meta.system == 0x4001u);
    CHECK(peers.items[0].meta.sampleTimeS == peers.items[0].value.position.timeS);
    CHECK(peers.items[0].meta.receiveTimeS == peers.items[0].value.receivedS);
    CHECK(s.leadMa.reportPeers().items.empty()); // (the wingman publishes nothing: the lead heard no one)

    // the link lost: after its picture's timeout the track is gone - dropped, not kept and marked
    REQUIRE(s.w.leaveNet(s.lead.id(), s.net));
    s.w.step(steps(s.w, s.plan.trackTimeoutS + 1.0));
    const ma::Collection<comm::LinkTrack> lost = s.wingMa.reportLinkTracks();
    CHECK(lost.availability == ma::Availability::Available);
    CHECK(lost.items.empty());
}

TEST_CASE("ma reports: a C2 task and the authority over a platform; a C2 node has neither navigation nor sensors", "[ma][reports]") {
    World w(options("ma-reports-c2"));
    Vehicle a = aircraft(w, "viper", 0.0, 0.0, 6000.0, 0.0);
    comm::C2NodeSpec spec;
    spec.name = "crc", spec.latitudeRad = (40.0 - 40.0 / 111.2) * kDeg, spec.longitudeRad = 0.0, spec.altitudeMslM = 200.0;
    const std::uint32_t crc = w.createC2Node(spec);
    const comm::NetId net = w.createNet({});
    REQUIRE(w.joinNet(a.id(), net));
    REQUIRE(w.joinNet(crc, net));
    comm::DisseminationPlan heartbeat;
    heartbeat.statusPeriodS = 2.0;
    REQUIRE(w.setDissemination(crc, heartbeat));
    Agent blue = w.createAgent(a, "blue-1");
    Agent boss = w.createAgent(crc, "crc");
    REQUIRE(boss.declareRole(2, "CRC LEAD"));
    w.step(steps(w, 3.0));

    const ma::Report<C2Authority> authority = blue.reportAuthority();
    REQUIRE(authority.availability == ma::Availability::Available);
    CHECK(authority.meta.source == ma::Source::MaCore);
    CHECK(authority.meta.validity == ma::Validity::Valid);
    CHECK(authority.meta.sampleTimeS == w.time());
    REQUIRE(authority.value.operators.size() == 1);
    CHECK(authority.value.operators[0].system == boss.system());
    CHECK(authority.value.operators[0].callsign == "CRC LEAD");

    // a task is not there until the link delivers it
    comm::TaskOrder order;
    order.type = comm::TaskOrder::Patrol;
    order.latitudeRad = 40.3 * kDeg, order.longitudeRad = 0.0; // (its altitude, radius and window left out: unknown)
    const double sent = w.time();
    const std::uint64_t task = boss.sendTask(blue.system(), order);
    REQUIRE(task != 0);
    CHECK(blue.reportTasks().availability == ma::Availability::Available);
    CHECK(blue.reportTasks().items.empty());
    w.step(steps(w, 2.0));
    const ma::Collection<comm::ReceivedTask> tasks = blue.reportTasks();
    REQUIRE(tasks.items.size() == 1);
    const ma::Reported<comm::ReceivedTask>& held = tasks.items[0];
    CHECK(held.meta.source == ma::Source::Link);
    CHECK(held.meta.system == boss.system());
    CHECK(held.meta.validity == ma::Validity::Valid);
    CHECK(held.meta.receiveTimeS == held.value.receivedS);
    CHECK(held.meta.receiveTimeS > sent);
    CHECK(unknown(held.meta.sampleTimeS)); // (an order states no time of its own)
    CHECK(unknown(held.meta.ageS));
    ma::ReportOptions strict;
    strict.staleAfterS = 0.0;
    CHECK(blue.reportTasks(strict).items[0].meta.validity == ma::Validity::Valid); // (no age: never stale)
    CHECK(held.value.order.taskId == task);
    CHECK(held.value.state == comm::ReceivedTask::Received);
    CHECK(unknown(held.value.order.altitudeM)); // (unknown stays unknown: not zero)
    CHECK(unknown(held.value.percent));

    // a peer heard by its heartbeat alone: held, its sample time unknown
    const ma::Collection<comm::PeerReport> peers = blue.reportPeers();
    REQUIRE(peers.items.size() == 1);
    CHECK_FALSE(peers.items[0].value.hasPosition);
    CHECK(unknown(peers.items[0].meta.sampleTimeS));
    CHECK(unknown(peers.items[0].meta.ageS));
    CHECK(peers.items[0].meta.receiveTimeS == peers.items[0].value.receivedS);
    CHECK(peers.items[0].meta.validity == ma::Validity::Valid);

    // the C2 node: no navigation, no sensors, no tasks, no authority over it - not applicable, none of it zero
    CHECK(boss.reportNavigation().availability == ma::Availability::NotApplicable);
    CHECK(boss.reportNavigation().meta.validity == ma::Validity::Unknown);
    CHECK(unknown(boss.reportNavigation().meta.sampleTimeS));
    CHECK(boss.reportTracks().availability == ma::Availability::NotApplicable);
    CHECK(boss.reportContacts().availability == ma::Availability::NotApplicable);
    CHECK(boss.reportTasks().availability == ma::Availability::NotApplicable);
    CHECK(boss.reportAuthority().availability == ma::Availability::NotApplicable);
    CHECK(boss.reportLinkTracks().availability == ma::Availability::Available);
    CHECK(boss.reportPeers().availability == ma::Availability::Available);

    // a platform gone: unavailable, and nothing listed
    REQUIRE(a.remove());
    CHECK(blue.reportNavigation().availability == ma::Availability::Unavailable);
    CHECK(blue.reportTracks().availability == ma::Availability::Unavailable);
    CHECK(blue.reportTasks().availability == ma::Availability::Unavailable);
    CHECK(blue.reportTasks().items.empty());
    CHECK(blue.reportAuthority().availability == ma::Availability::Unavailable);
    const Agent nobody;
    CHECK(nobody.reportNavigation().availability == ma::Availability::Unavailable);
    CHECK(nobody.reportPeers().items.empty());
    CHECK(nobody.contacts(0).empty());
}

TEST_CASE("ma reports: a passive contact, as its receiver measured it - no range, no entity", "[ma][reports]") {
    World w(options("ma-reports-contacts"));
    Vehicle own = aircraft(w, "viper", 0.0, 0.0, 5000.0, 90.0, 200.0);
    Vehicle other = aircraft(w, "bandit", 0.0, 40.0, 5000.0, 270.0, 200.0);
    Agent blue = w.createAgent(own, "blue-1");
    const std::uint16_t rwr = payloadOf(own, ms::Family::Esm);
    REQUIRE(rwr != 0xFFFF);
    REQUIRE(other.submit(ms::StateCommand{payloadOf(other, ms::Family::AirRadar), ms::PayloadState::Operate}).accepted());
    w.step();
    ms::AmtiCommand search;
    search.payload = payloadOf(other, ms::Family::AirRadar);
    search.azimuthHalfWidthRad = 30.0 * kDeg;
    REQUIRE(other.submit(search).accepted());
    w.step(60);
    // its receiver off: an available, empty collection
    CHECK(blue.contacts(rwr).empty());
    CHECK(blue.reportContacts().availability == ma::Availability::Available);
    CHECK(blue.reportContacts().items.empty());
    REQUIRE(blue.submit(ms::StateCommand{rwr, ms::PayloadState::Operate}).accepted());
    w.step(30);
    const std::vector<ms::Contact> mine = blue.contacts(rwr);
    REQUIRE(mine.size() == 1);
    CHECK(mine[0].truth == 0);                       // (the agent's read: the entity behind it is the referee's)
    REQUIRE(own.contacts(rwr).size() == 1);
    CHECK(own.contacts(rwr)[0].truth == other.id()); // (the range's read of the same receiver has it)
    CHECK(mine[0].kind == ms::ContactKind::Emitter);
    CHECK(unknown(mine[0].rangeM));                  // (a receiver measures none: unknown, not zero)
    CHECK(unknown(mine[0].latitudeRad));
    const ma::Collection<ms::Contact> reported = blue.reportContacts();
    REQUIRE(reported.availability == ma::Availability::Available);
    REQUIRE(reported.items.size() == 1);
    const ma::Reported<ms::Contact>& r = reported.items[0];
    CHECK(r.value.id == mine[0].id);
    CHECK(r.value.truth == 0);
    CHECK(unknown(r.value.rangeM));
    CHECK(r.meta.source == ma::Source::OwnSensor);
    CHECK(r.meta.payload == rwr);
    CHECK(r.meta.sampleTimeS == mine[0].timeS);
    CHECK(r.meta.receiveTimeS == w.time());
    CHECK(r.meta.ageS == w.time() - mine[0].timeS);
    CHECK(r.meta.validity == ma::Validity::Valid);
    CHECK(mine[0].firstTimeS <= mine[0].timeS);
    // the names are the contract's
    CHECK(std::string(ma::availabilityName(ma::Availability::NotApplicable)) == "not_applicable");
    CHECK(std::string(ma::validityName(ma::Validity::Stale)) == "stale");
    CHECK(std::string(ma::sourceName(ma::Source::OwnNavigation)) == "own_navigation");
    CHECK(std::string(ma::sourceName(ma::Source::Link)) == "link");
}
