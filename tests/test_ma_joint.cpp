// Joint acceptance, generic part (M8; docs/sdk/ma-training-validation.md): the recorded joint chains replayed through
// the public C++ SDK alone, and what each case requires checked on what the endpoints returned - J2 P2P perception,
// J4 typed C2 tasks and an unauthorized role, J6 control ownership, J7 link aging, lost link and package changes, the
// message part of J3 on a lossy link, and the lifecycle part of J8: a reset equals a fresh episode, and independent
// episodes do not interfere. The installed C++, C99 and Python consumers replay the same chains and must write the
// same trace (mission_package_consumer).
#include "MissionTestSupport.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

using namespace fsim;
using namespace mission_test;

namespace {

constexpr std::uint64_t kLead = 1, kSensors = 2, kObs = 5, kWingMa = 9007199254740995ull; // (kCrc: MissionTestSupport.h's)
const std::string kLeadSys = "1392508929", kWingSys = "1392508930", kCrcSys = "1392508931";

std::string jointPath(const std::string& name) { return std::string(FSIM_TEST_SOURCE_DIR) + "/tests/fixtures/mission_training/joint/" + name; }

/// A recorded chain (tools/ma_contract/joint_chain.py): its configuration, its ticks and its submission lines.
struct Chain {
    std::string config;
    std::uint64_t ticks = 0;
    struct Line {
        std::uint64_t tick, service;
        std::string requests;
    };
    std::vector<Line> lines;
    std::vector<std::uint64_t> services;

    explicit Chain(const std::string& name) {
        std::istringstream in(readFile(jointPath(name)));
        std::string line;
        while (std::getline(in, line)) {
            if (!line.empty() && line.back() == '\r') line.pop_back();
            if (line.empty() || line[0] == '#') continue;
            if (line.rfind("config ", 0) == 0) {
                config = replaced(readFile(jointPath(line.substr(7))), "\"@JSBSIM_ROOT@\"", jsonString(FSIM_TEST_JSBSIM_ROOT));
            } else if (line.rfind("ticks ", 0) == 0) {
                ticks = std::stoull(line.substr(6));
            } else {
                const std::size_t a = line.find(' '), b = line.find(' ', a + 1);
                lines.push_back({std::stoull(line.substr(0, a)), std::stoull(line.substr(a + 1, b - a - 1)), line.substr(b + 1)});
            }
        }
        const core::Json document = parse(config);
        for (const core::Json& s : at(document, "services").asArray()) services.push_back(std::stoull(str(s, "service_id")));
        std::sort(services.begin(), services.end());
    }
    /// The tick of a service's first line holding `operation` ("kind\",\"operation\":\"op").
    std::uint64_t tickOf(std::uint64_t service, const std::string& kind, const std::string& operation) const {
        const std::string needle = "\"kind\":\"" + kind + "\",\"operation\":\"" + operation + "\"";
        for (const Line& l : lines)
            if (l.service == service && l.requests.find(needle) != std::string::npos) return l.tick;
        throw std::runtime_error("the chain has no " + kind + "." + operation + " of service " + std::to_string(service));
    }
};

/// A replay's trace: the same lines every consumer writes, and each snapshot and acknowledgment by tick and service.
struct Trace {
    std::vector<std::string> lines;
    std::map<std::pair<std::uint64_t, std::uint64_t>, std::string> obs, ack;
    std::string truth, costs; ///< the referee's, after the last tick (not part of an actor's trace)

    /// The parsed document, kept: what the checks read stays valid as long as the trace.
    const core::Json& snapshot(std::uint64_t tick, std::uint64_t service) const { return parsed(parsedObs, obs, tick, service); }
    const core::Json& acknowledgment(std::uint64_t tick, std::uint64_t service) const { return parsed(parsedAck, ack, tick, service); }

private:
    using Key = std::pair<std::uint64_t, std::uint64_t>;
    mutable std::map<Key, core::Json> parsedObs, parsedAck;
    static const core::Json& parsed(std::map<Key, core::Json>& cache, const std::map<Key, std::string>& text, std::uint64_t tick,
                                    std::uint64_t service) {
        const Key key{tick, service};
        auto it = cache.find(key);
        if (it == cache.end()) it = cache.emplace(key, parse(text.at(key))).first;
        return it->second;
    }
};

/// The chain replayed on an episode just reset, up to `until` ticks (the chain's own by default).
Trace replay(MissionEpisode& episode, const Chain& chain, std::uint64_t until = ~std::uint64_t{0}) {
    Trace t;
    std::size_t k = 0;
    const std::uint64_t last = std::min(until, chain.ticks);
    for (std::uint64_t tick = 0; tick <= last; ++tick) {
        std::map<std::uint64_t, std::string> keys;
        for (const std::uint64_t s : chain.services) {
            const std::string o = episode.endpoint(s).observationJsonV1();
            t.obs[{tick, s}] = o;
            t.lines.push_back("obs " + std::to_string(tick) + " " + std::to_string(s) + " " + o);
            keys[s] = at(parse(o), "episode").dump();
        }
        for (; k < chain.lines.size() && chain.lines[k].tick == tick; ++k) {
            const Chain::Line& l = chain.lines[k];
            const std::string a = episode.endpoint(l.service).submitJsonV1("{\"schema\":\"ma.actions.v1\",\"episode\":" + keys[l.service] +
                                                                           ",\"service_id\":\"" + std::to_string(l.service) + "\",\"tick\":\"" +
                                                                           std::to_string(tick) + "\",\"requests\":" + l.requests + "}");
            t.ack[{tick, l.service}] = a;
            t.lines.push_back("ack " + std::to_string(tick) + " " + std::to_string(l.service) + " " + a);
        }
        if (tick < last) episode.step();
    }
    t.truth = episode.referee().truthJsonV1();
    t.costs = episode.referee().costsJsonV1();
    return t;
}

Trace replayFresh(const Chain& chain) {
    MissionEpisode episode(chain.config);
    episode.reset(0);
    return replay(episode, chain);
}

const core::Json* receiptOf(const core::Json& snapshot, const std::string& requestId) {
    for (const core::Json& r : at(snapshot, "receipts").asArray())
        if (str(r, "request_id") == requestId) return &r;
    return nullptr;
}
std::string native(const core::Json& receipt, const char* field) {
    const core::Json& n = at(receipt, "native");
    if (n.isNull()) return "";
    const core::Json* v = n.find(field);
    return v && v->isString() ? v->asString() : "";
}
std::string nativeId(const core::Json& receipt, const char* id) {
    const core::Json& n = at(receipt, "native");
    if (n.isNull()) return "";
    const core::Json* v = at(n, "ids").find(id);
    return v && v->isString() ? v->asString() : "";
}
const core::Json& items(const core::Json& snapshot, const char* report) { return at(at(at(snapshot, "reports"), report), "items"); }
double timeOf(const core::Json& snapshot) { return at(snapshot, "time_s").asNumber(); }

/// Every generation's identity, the same: a reset's snapshots compared with a fresh episode's.
std::string sameGeneration(std::string text) {
    for (std::size_t at = text.find("\"episode_generation\":\""); at != std::string::npos; at = text.find("\"episode_generation\":\"", at + 1)) {
        const std::size_t start = at + 22, end = text.find('"', start);
        text.replace(start, end - start, "1");
    }
    return text;
}

} // namespace

TEST_CASE("joint chains replay to the same trace every time", "[joint][determinism]") {
    for (const char* name : {"chain.joint.txt", "chain.lossy.txt"}) {
        INFO(name);
        const Chain chain(name);
        CHECK(chain.ticks > 0);
        CHECK(replayFresh(chain).lines == replayFresh(chain).lines);
    }
}

TEST_CASE("J2 P2P perception: the radar-off wing holds the lead's track only after its delivery, then designates it",
          "[joint][J2]") {
    const Chain chain("chain.joint.txt");
    const Trace t = replayFresh(chain);
    const std::uint64_t shared = chain.tickOf(kLead, "dms", "publish_native"), designated = chain.tickOf(kWingMa, "sensor", "designate_link");
    for (std::uint64_t tick = 0; tick <= chain.ticks; ++tick) { // (the wing's radar stays off: no track of its own, ever)
        const core::Json wing = t.snapshot(tick, kWingMa);
        CHECK(items(wing, "tracks").asArray().empty());
        CHECK(str(items(wing, "sensors").asArray().at(0), "state") == "off");
    }
    const core::Json lead = t.snapshot(shared, kSensors);
    REQUIRE_FALSE(items(lead, "tracks").asArray().empty());
    const std::string track = str(items(lead, "tracks").asArray().at(0), "track_id");
    for (std::uint64_t tick = 0; tick <= shared; ++tick) CHECK(items(t.snapshot(tick, kWingMa), "link_tracks").asArray().empty());
    const core::Json delivered = t.snapshot(shared + 1, kWingMa);
    REQUIRE(items(delivered, "link_tracks").asArray().size() == 1);
    const core::Json& link = items(delivered, "link_tracks").asArray().at(0);
    CHECK(str(link, "source_system_id") == kLeadSys);
    CHECK(str(link, "track_id") == track);
    const double received = at(at(link, "meta"), "receive_time_s").asNumber();
    CHECK(received > timeOf(t.snapshot(shared, kLead))); // (after the modelled link carried it)
    CHECK(received <= timeOf(delivered));
    const core::Json* publication = receiptOf(t.snapshot(shared + 1, kLead), "1");
    REQUIRE(publication);
    CHECK(native(*publication, "status") == "published");
    const core::Json after = t.snapshot(designated + 1, kWingMa);
    const core::Json* designation = receiptOf(after, "3");
    REQUIRE(designation);
    CHECK(native(*designation, "status") == "designated");
    const std::string target = nativeId(*designation, "target_id");
    CHECK_FALSE(target.empty());
    bool held = false;
    for (const core::Json& d : items(after, "designations").asArray())
        held = held || (str(d, "target_id") == target && str(d, "source_system_id") == kLeadSys && str(d, "track_id") == track);
    CHECK(held);
}

TEST_CASE("J4 typed C2 task: accepted, progress and completion reported; an unauthorized role has no effect", "[joint][J4]") {
    const Chain chain("chain.joint.txt");
    const Trace t = replayFresh(chain);
    const std::uint64_t sent = chain.tickOf(kCrc, "c2", "send_task");
    // the role that may not task: answered unauthorized natively, and the wing never holds its task
    const core::Json* refused = receiptOf(t.snapshot(chain.ticks, kObs), "2");
    REQUIRE(refused);
    CHECK(str(*refused, "state") == "rejected");
    CHECK(native(*refused, "status") == "unauthorized");
    std::set<std::pair<std::string, std::string>> tasks;
    for (std::uint64_t tick = 0; tick <= chain.ticks; ++tick)
        for (const core::Json& task : items(t.snapshot(tick, kWingMa), "tasks").asArray()) tasks.insert({str(task, "from_system_id"), str(task, "task_id")});
    CHECK(tasks == std::set<std::pair<std::string, std::string>>{{kCrcSys, "1"}}); // (one task: the retry below made none)
    // J3's task part: the same request again is answered as the first, and sends nothing
    const core::Json retry = t.acknowledgment(sent + 1, kCrc);
    CHECK(str(at(at(retry, "submission"), "requests").asArray().at(0), "status") == "duplicate");
    // the authorised task's receipt follows the MA's answers: sent, then executing, then completed
    std::vector<std::string> states;
    for (std::uint64_t tick = sent + 1; tick <= chain.ticks; ++tick)
        if (const core::Json* r = receiptOf(t.snapshot(tick, kCrc), "2"); r && (states.empty() || states.back() != str(*r, "state")))
            states.push_back(str(*r, "state"));
    CHECK(states == std::vector<std::string>{"pending", "executing", "completed"});
    bool progress = false;
    for (std::uint64_t tick = 0; tick <= chain.ticks; ++tick)
        for (const core::Json& task : items(t.snapshot(tick, kWingMa), "tasks").asArray())
            progress = progress || (str(task, "state") == "accepted" && at(task, "percent").isNumber() && at(task, "percent").asNumber() == 50.0);
    CHECK(progress);
    CHECK(str(items(t.snapshot(chain.ticks, kWingMa), "tasks").asArray().at(0), "state") == "completed");
}

TEST_CASE("J6 control ownership: C2's command holds the flight axes for every component; only C2 ends it", "[joint][J6]") {
    const Chain chain("chain.joint.txt");
    const Trace t = replayFresh(chain);
    const std::uint64_t commanded = chain.tickOf(kCrc, "c2", "command_hsa"), tried = chain.tickOf(kWingMa, "flight", "new"),
                        canceled = chain.tickOf(kCrc, "c2", "cancel_command");
    const core::Json& held = t.snapshot(tried, kWingMa);
    REQUIRE(at(at(at(held, "reports"), "authority"), "commands").asArray().size() == 1);
    const core::Json& command = at(at(at(held, "reports"), "authority"), "commands").asArray().at(0);
    CHECK(str(command, "system_id") == kCrcSys);
    const std::string activity = str(command, "activity_id");
    CHECK(commanded < tried);
    const core::Json& answered = t.snapshot(tried + 1, kWingMa);
    for (const char* id : {"4", "5"}) { // (components 1 and 2: the same native refusal for each)
        INFO(id);
        const core::Json* r = receiptOf(answered, id);
        REQUIRE(r);
        CHECK(str(*r, "state") == "rejected");
        CHECK(native(*r, "reason") == "authority_held");
        CHECK(nativeId(*r, "other_activity_id") == activity);
        CHECK(str(at(*r, "component"), "component_id") == (std::string(id) == "4" ? "1" : "2"));
    }
    const core::Json* cancel = receiptOf(answered, "6"); // (the MA may not cancel C2's activity)
    REQUIRE(cancel);
    CHECK(str(*cancel, "state") == "rejected");
    CHECK(at(at(at(t.snapshot(tried + 1, kWingMa), "reports"), "authority"), "commands").asArray().size() == 1);
    const core::Json* control = receiptOf(t.snapshot(chain.ticks, kObs), "3"); // (LRE may not designate control)
    REQUIRE(control);
    CHECK(native(*control, "status") == "unauthorized");
    // C2 cancels its own command: the axes are the MA's again
    std::uint64_t again = 0;
    for (const Chain::Line& l : chain.lines)
        if (l.service == kWingMa && l.tick > canceled && l.requests.find("\"operation\":\"new\"") != std::string::npos) again = l.tick;
    REQUIRE(again > canceled);
    CHECK(at(at(at(t.snapshot(again, kWingMa), "reports"), "authority"), "commands").asArray().empty());
    const core::Json* flying = receiptOf(t.snapshot(again + 1, kWingMa), "7");
    REQUIRE(flying);
    CHECK(str(*flying, "state") == "executing");
    bool owned = false;
    for (const core::Json& a : items(t.snapshot(again + 1, kWingMa), "activities").asArray())
        owned = owned || (str(a, "activity_id") == nativeId(*flying, "activity_id") && str(a, "owner_service_id") == std::to_string(kWingMa));
    CHECK(owned);
}

TEST_CASE("J7 link and package change: reports age, the link is lost and restored in simulated time; C2 forms the proposed package",
          "[joint][J7]") {
    const Chain chain("chain.joint.txt");
    const Trace t = replayFresh(chain);
    INFO("report aging: the shared track's age is simulated time since its report; stale past 10 s; dropped after 20 s");
    const std::uint64_t shared = chain.tickOf(kLead, "dms", "publish_native");
    double sample = -1.0;
    bool stale = false, dropped = false;
    for (std::uint64_t tick = shared + 1; tick <= chain.ticks; ++tick) {
        const core::Json wing = t.snapshot(tick, kWingMa);
        const auto& links = items(wing, "link_tracks").asArray();
        if (links.empty()) {
            CHECK(timeOf(wing) - sample >= 20.0 - 1e-6); // (dropped by the native timeout, in simulated time)
            dropped = true;
            break;
        }
        const core::Json& meta = at(links.at(0), "meta");
        sample = at(meta, "sample_time_s").asNumber();
        const double age = at(meta, "age_s").asNumber();
        CHECK(std::fabs(age - (timeOf(wing) - sample)) < 1e-9);
        CHECK(str(meta, "validity") == (age > 10.0 ? "stale" : "valid"));
        stale = stale || age > 10.0;
    }
    CHECK(stale);
    CHECK(dropped);
    INFO("lost link: raised when C2 has been silent for the threshold, cleared when it is heard again");
    const std::uint64_t configured = chain.tickOf(kCrc, "c2", "set_lost_comms"), heard = chain.tickOf(kCrc, "dms", "publish_native");
    std::uint64_t lostAt = 0, restoredAt = 0;
    for (std::uint64_t tick = configured + 1; tick <= chain.ticks; ++tick) {
        const core::Json& lost = at(at(at(t.snapshot(tick, kWingMa), "reports"), "authority"), "lost_comms");
        if (!lostAt && at(lost, "lost").asBool()) {
            lostAt = tick;
            const double since = at(lost, "lost_since_s").asNumber();
            CHECK(timeOf(t.snapshot(tick, kWingMa)) - since >= 4.0 - 1e-6);
            CHECK(timeOf(t.snapshot(tick - 1, kWingMa)) - since < 4.0); // (raised at the first tick past the threshold)
        }
        if (lostAt && !restoredAt && !at(lost, "lost").asBool()) restoredAt = tick;
    }
    CHECK(lostAt > configured);
    CHECK(lostAt <= heard);
    CHECK(restoredAt > heard);
    REQUIRE(restoredAt > 0);
    // (one alert for that loss; C2 silent again afterwards raises the next one later, as simulated time goes on)
    CHECK(at(at(at(at(t.snapshot(restoredAt, kWingMa), "reports"), "authority"), "lost_comms"), "alerts").asNumber() == 1.0);
    INFO("the proposal: delivered to C2 once; the package the authorised role forms; the other role's refused");
    std::vector<std::uint64_t> proposals;
    for (std::uint64_t tick = 0; tick <= chain.ticks; ++tick)
        for (const core::Json& m : at(at(t.snapshot(tick, kCrc), "messages"), "items").asArray())
            if (str(m, "topic") == "1001") {
                proposals.push_back(tick);
                CHECK(str(m, "source_system_id") == kWingSys);
                CHECK(str(m, "data_b64") == "bGVhZGVyOjEzOTI1MDg5Mjk="); // ("leader:1392508929")
            }
    CHECK(proposals.size() == 1);
    REQUIRE_FALSE(proposals.empty());
    CHECK(proposals[0] <= chain.tickOf(kCrc, "c2", "form_package"));
    for (const std::uint64_t platform : {kWingMa, kLead}) {
        const core::Json& package = at(at(at(t.snapshot(chain.ticks, platform), "reports"), "authority"), "package");
        REQUIRE_FALSE(package.isNull());
        CHECK(str(package, "package_id") == "7");
        CHECK(str(package, "leader_system_id") == kLeadSys);
    }
    const core::Json obs = t.snapshot(chain.ticks, kObs);
    const std::string obsCommand = nativeId(*receiptOf(obs, "4"), "command_id");
    std::set<std::string> refusedBy;
    for (const core::Json& s : items(obs, "command_statuses").asArray())
        if (str(s, "command_id") == obsCommand && str(s, "state") == "unauthorized") refusedBy.insert(str(s, "from_system_id"));
    CHECK(refusedBy == std::set<std::string>{kLeadSys, kWingSys});
}

TEST_CASE("J3 messages on a lossy link: retransmitted until delivered, each delivered once; a resubmission sends nothing",
          "[joint][J3]") {
    const Chain chain("chain.lossy.txt");
    const Trace t = replayFresh(chain);
    std::size_t proposals = 0, acknowledgments = 0;
    for (std::uint64_t tick = 0; tick <= chain.ticks; ++tick) {
        for (const core::Json& m : at(at(t.snapshot(tick, kCrc), "messages"), "items").asArray()) proposals += str(m, "topic") == "1001";
        for (const core::Json& m : at(at(t.snapshot(tick, kWingMa), "messages"), "items").asArray()) acknowledgments += str(m, "topic") == "1002";
    }
    CHECK(proposals == 1);
    CHECK(acknowledgments == 1);
    const core::Json truth = parse(t.truth);
    const core::Json& net = at(truth, "nets").asArray().at(0);
    INFO(net.dump());
    CHECK(std::stoull(str(net, "lost")) > 0); // (attempts were lost and sent again)
    CHECK(std::stoull(str(net, "transmissions")) > std::stoull(str(net, "delivered")));
    std::vector<std::string> resubmitted;
    for (const Chain::Line& l : chain.lines)
        if (l.service == kWingMa && l.tick > 0) {
            const core::Json& r = at(at(t.acknowledgment(l.tick, kWingMa), "submission"), "requests").asArray().at(0);
            resubmitted.push_back(str(r, "status") + "/" + (at(r, "reason").isNull() ? "" : str(r, "reason")));
        }
    // the same request: its first acknowledgment again; its id with another body: refused
    CHECK(resubmitted == std::vector<std::string>{"duplicate/", "rejected/request_id_reuse"});
    const core::Json costs = parse(t.costs);
    for (const core::Json& s : at(costs, "services").asArray())
        if (str(s, "service_id") == std::to_string(kWingMa)) CHECK(at(at(s, "total"), "dms_messages").asNumber() == 1.0);
}

TEST_CASE("J8 lifecycle: a reset after the chain stopped midway - messages, tasks, commands pending - equals a fresh episode",
          "[joint][J8]") {
    const Chain chain("chain.joint.txt");
    const Trace fresh = replayFresh(chain);
    MissionEpisode episode(chain.config);
    episode.reset(0);
    // stopped where the proposal, the package commands and C2's status are on the link and the task is open
    const std::uint64_t stop = chain.tickOf(kCrc, "c2", "form_package");
    const Trace partial = replay(episode, chain, stop);
    CHECK_FALSE(items(partial.snapshot(stop, kWingMa), "tasks").asArray().empty());
    episode.reset(0); // (the same local episode: its next generation)
    const Trace again = replay(episode, chain);
    REQUIRE(again.lines.size() == fresh.lines.size());
    for (std::size_t i = 0; i < fresh.lines.size(); ++i) {
        INFO(i);
        CHECK(sameGeneration(again.lines[i]) == fresh.lines[i]);
    }
    CHECK(sameGeneration(again.truth) == fresh.truth);
    CHECK(sameGeneration(again.costs) == fresh.costs);
}

TEST_CASE("J8 lifecycle: independent episodes stepped together, one reset midway, each keep their own trace", "[joint][J8]") {
    const Chain joint("chain.joint.txt"), lossy("chain.lossy.txt");
    const Trace soloJoint = replayFresh(joint), soloLossy = replayFresh(lossy);
    MissionEpisode a(joint.config), b(lossy.config);
    a.reset(0);
    b.reset(0);
    // b runs half its chain, is reset, and runs it whole - all while a runs its own, tick for tick
    Trace ta, tb;
    std::size_t ka = 0, kb = 0;
    const auto tickOf = [](MissionEpisode& e, const Chain& c, Trace& t, std::size_t& k, std::uint64_t tick) {
        std::map<std::uint64_t, std::string> keys;
        for (const std::uint64_t s : c.services) {
            const std::string o = e.endpoint(s).observationJsonV1();
            t.lines.push_back("obs " + std::to_string(tick) + " " + std::to_string(s) + " " + o);
            keys[s] = at(parse(o), "episode").dump();
        }
        for (; k < c.lines.size() && c.lines[k].tick == tick; ++k)
            t.lines.push_back("ack " + std::to_string(tick) + " " + std::to_string(c.lines[k].service) + " " +
                              e.endpoint(c.lines[k].service)
                                  .submitJsonV1("{\"schema\":\"ma.actions.v1\",\"episode\":" + keys[c.lines[k].service] + ",\"service_id\":\"" +
                                                std::to_string(c.lines[k].service) + "\",\"tick\":\"" + std::to_string(tick) +
                                                "\",\"requests\":" + c.lines[k].requests + "}"));
        if (tick < c.ticks) e.step();
    };
    std::uint64_t tickB = 0;
    bool restarted = false;
    for (std::uint64_t tick = 0; tick <= joint.ticks; ++tick) {
        tickOf(a, joint, ta, ka, tick);
        if (tickB <= lossy.ticks) {
            tickOf(b, lossy, tb, kb, tickB++);
            if (!restarted && tickB == lossy.ticks / 2) {
                b.reset(0), tb = Trace{}, kb = 0, tickB = 0, restarted = true;
            }
        }
    }
    CHECK(restarted);
    CHECK(ta.lines == soloJoint.lines);
    REQUIRE(tb.lines.size() == soloLossy.lines.size());
    for (std::size_t i = 0; i < tb.lines.size(); ++i) CHECK(sameGeneration(tb.lines[i]) == soloLossy.lines[i]);
}
