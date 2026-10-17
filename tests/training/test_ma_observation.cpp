// The mission gateway's snapshots and referee documents, through the public C++ SDK (docs/sdk/mission-training.md,
// sections 10, 12 and 13; M5 core): one immutable legal snapshot per service per decision tick, read any number of
// times without consuming anything; each delivered message in exactly one snapshot of the ingress owner; referee truth
// and outcomes on the referee's handle alone; a fresh reset equal to a fresh episode.
#include "MissionTestSupport.h"

#include <catch2/catch_test_macros.hpp>

#include <set>
#include <string>
#include <vector>

using namespace fsim;
using namespace mission_test;

namespace {

std::set<std::string> keys(const core::Json& object) {
    std::set<std::string> out;
    for (const auto& [k, v] : object.asObject()) out.insert(k);
    return out;
}

const core::Json& part(const core::Json& observation, const char* name) { return at(at(observation, "reports"), name); }

/// Messages of one snapshot from a source system on a topic.
int messages(const core::Json& observation, const std::string& source, const std::string& topic) {
    int n = 0;
    for (const core::Json& m : at(at(observation, "messages"), "items").asArray())
        if (str(m, "source_system_id") == source && str(m, "topic") == topic) ++n;
    return n;
}

} // namespace

TEST_CASE("each service reads one immutable legal snapshot per decision tick", "[mission][observation]") {
    MissionEpisode episode(config());
    episode.reset(41);
    MissionEndpoint lead = episode.endpoint(kLeadFlight), sensors = episode.endpoint(kLeadSensors), crc = episode.endpoint(kCrc),
                    wing = episode.endpoint(kWing);

    const std::string first = wing.observationJsonV1();
    for (int k = 0; k < 5; ++k) CHECK(wing.observationJsonV1() == first);
    const core::Json o = parse(first);
    CHECK(keys(o) == std::set<std::string>{"schema", "episode", "service", "tick", "time_s", "reports", "messages", "receipts"});
    CHECK(str(o, "schema") == "ma.observation.v1");
    CHECK(str(o, "tick") == "0");
    CHECK(at(o, "time_s").asNumber() == 0.0);
    const core::Json& service = at(o, "service");
    CHECK(str(service, "service_id") == "9007199254740995");
    CHECK(str(service, "name") == "wing-ma");
    CHECK(str(service, "platform_kind") == "vehicle");
    CHECK(str(service, "system_id") == "1392508930");
    CHECK(at(service, "alive").asBool());
    INFO("controllers are minted in ascending service id, whatever the array's order");
    CHECK(str(at(parse(lead.observationJsonV1()), "service"), "controller_id") == "1073741825");
    CHECK(str(at(parse(sensors.observationJsonV1()), "service"), "controller_id") == "1073741826");
    CHECK(str(at(parse(crc.observationJsonV1()), "service"), "controller_id") == "1073741827");
    CHECK(str(service, "controller_id") == "1073741828");

    CHECK(str(part(o, "navigation"), "availability") == "available");
    CHECK(str(at(part(o, "navigation"), "meta"), "source") == "own_navigation");
    const core::Json c = parse(crc.observationJsonV1());
    CHECK(str(at(c, "service"), "platform_kind") == "c2_node");
    CHECK(str(part(c, "navigation"), "availability") == "not_applicable");
    CHECK(at(part(c, "navigation"), "own_ship").isNull());
    CHECK(str(at(part(c, "navigation"), "meta"), "validity") == "unknown");

    INFO("capabilities name their configured owner; the shared reports are every service's of the platform alike");
    const core::Json s = parse(sensors.observationJsonV1());
    for (const core::Json& cap : at(part(s, "capabilities"), "items").asArray()) {
        const std::string id = str(cap, "capability");
        const std::string owner = at(cap, "owner_service_id").isNull() ? std::string() : str(cap, "owner_service_id");
        if (id == "fsim.guidance.hsa" || id == "fsim.guidance.route") CHECK(owner == "1");
        if (id == "fsim.ms.amti:radar" || id == "fsim.ms.status:radar") CHECK(owner == "2");
    }
    CHECK(str(at(s, "messages"), "availability") == "not_owner");
    CHECK(str(part(s, "transmissions"), "availability") == "not_owner");
    CHECK(str(at(parse(lead.observationJsonV1()), "messages"), "availability") == "available");

    INFO("the actor's snapshot carries no truth, outcome or referee member anywhere");
    for (const std::string& text : {first, crc.observationJsonV1(), sensors.observationJsonV1()})
        for (const char* banned : {"\"entities\"", "\"truth\"", "\"episode_done\"", "\"criteria\"", "\"roles\"", "\"reward\""}) {
            INFO(banned);
            CHECK(text.find(banned) == std::string::npos);
        }

    episode.step();
    const std::string next = wing.observationJsonV1();
    CHECK(next != first);
    CHECK(str(parse(next), "tick") == "1");
    CHECK(at(parse(next), "time_s").asNumber() > 0.0);
    CHECK(wing.observationJsonV1() == next);
}

TEST_CASE("a delivered message is in exactly one snapshot of its platform's ingress owner", "[mission][messages]") {
    MissionEpisode episode(config());
    const std::string key = episode.reset(5);
    MissionEndpoint wing = episode.endpoint(kWing), crc = episode.endpoint(kCrc);
    MissionReferee referee = episode.referee();
    const std::string send = actions(key, kWing, 0,
                                     request(1, "dms", "send", "{\"topic\":\"1001\",\"destination_system_ids\":[\"1392508931\"],\"byte_length\":3,\"data_b64\":\"AQID\"}"));
    wing.submitJsonV1(send);
    CHECK(submitted(parse(wing.submitJsonV1(send)), 1) == "duplicate"); // (an identical retry sends nothing again)
    int seen = 0;
    std::string delivered;
    for (int tick = 1; tick <= 4; ++tick) {
        episode.step();
        const std::string text = crc.observationJsonV1();
        CHECK(crc.observationJsonV1() == text); // (reading it again consumes nothing)
        const int here = messages(parse(text), "1392508930", "1001");
        if (here) delivered = text;
        seen += here;
    }
    CHECK(seen == 1);
    REQUIRE(!delivered.empty());
    const core::Json deliveredSnapshot = parse(delivered);
    for (const core::Json& m : at(at(deliveredSnapshot, "messages"), "items").asArray())
        if (str(m, "topic") == "1001") {
            CHECK(str(m, "data_b64") == "AQID");
            CHECK(at(m, "byte_length").asNumber() == 3.0);
        }
    const core::Json w = parse(wing.observationJsonV1());
    const core::Json* sent = receipt(w, 1);
    REQUIRE(sent != nullptr);
    CHECK(str(*sent, "state") == "completed");
    CHECK(str(at(*sent, "native"), "status") == "delivered");
    INFO("a native exchange's completion is not a mission's success: the referee alone judges, by its criteria");
    CHECK(at(parse(referee.outcomeJsonV1()), "reason").isNull());
    bool listed = false;
    for (const core::Json& t : at(part(w, "transmissions"), "items").asArray())
        listed = listed || str(t, "request_id") == "1";
    CHECK(listed);
}

TEST_CASE("referee documents are the range's alone, and judge the configured criteria", "[mission][referee]") {
    MissionEpisode episode(replaced(config(), "\"max_ticks\": 600", "\"max_ticks\": 2"));
    episode.reset(9);
    MissionReferee referee = episode.referee();
    const std::string outcome = referee.outcomeJsonV1();
    CHECK(referee.outcomeJsonV1() == outcome);
    const core::Json o = parse(outcome);
    CHECK(str(o, "schema") == "ma.outcome.v1");
    CHECK(at(o, "episode_done").asBool() == false);
    CHECK(at(o, "reason").isNull());
    CHECK(at(o, "criteria").asArray().size() == 3);
    CHECK(at(o, "roles").asArray().size() == 4);
    for (const core::Json& role : at(o, "roles").asArray()) CHECK(at(role, "active").asBool());

    const core::Json t = parse(referee.truthJsonV1());
    CHECK(str(t, "schema") == "ma.truth.v1");
    CHECK(at(t, "entities").asArray().size() == 3);
    CHECK(at(t, "services").asArray().size() == 4);
    for (const core::Json& e : at(t, "entities").asArray())
        if (str(e, "kind") == "c2_node") CHECK(at(e, "state").isNull());

    episode.step();
    episode.step();
    const core::Json done = parse(referee.outcomeJsonV1());
    CHECK(at(done, "episode_done").asBool());
    CHECK(str(done, "reason") == "truncated");
    for (const core::Json& role : at(done, "roles").asArray()) {
        CHECK_FALSE(at(role, "active").asBool());
        CHECK(at(role, "truncated").asBool());
        CHECK(at(role, "bootstrap").asBool());
        CHECK(at(role, "final_observation").asBool());
    }
    for (const core::Json& c : at(done, "criteria").asArray())
        if (str(c, "id") == "time") CHECK(str(c, "satisfied_tick") == "2");
}

TEST_CASE("arrival order decides nothing: services dispatch in service id, then request id", "[mission][order]") {
    const auto run = [](bool wingFirst) {
        MissionEpisode episode(config());
        const std::string key = episode.reset(23);
        MissionEndpoint wing = episode.endpoint(kWing), crc = episode.endpoint(kCrc), lead = episode.endpoint(kLeadFlight);
        const std::string w = actions(key, kWing, 0, request(5, "flight", "new", "{\"mode\":\"hsa\",\"heading_rad\":0.3}") + "," +
                                                         request(6, "dms", "send", "{\"topic\":\"1001\",\"byte_length\":1,\"data_b64\":\"AA==\"}"));
        const std::string c = actions(key, kCrc, 0, request(2, "dms", "send", "{\"topic\":\"1002\",\"byte_length\":1,\"data_b64\":\"AQ==\"}"));
        const std::string l = actions(key, kLeadFlight, 0, request(9, "flight", "new", "{\"mode\":\"hsa\",\"heading_rad\":0.1}"));
        if (wingFirst) wing.submitJsonV1(w), crc.submitJsonV1(c), lead.submitJsonV1(l);
        else lead.submitJsonV1(l), crc.submitJsonV1(c), wing.submitJsonV1(w);
        std::vector<std::string> out;
        for (int k = 0; k < 2; ++k) {
            episode.step();
            out.push_back(wing.observationJsonV1() + crc.observationJsonV1() + lead.observationJsonV1() + episode.referee().truthJsonV1());
        }
        return out;
    };
    CHECK(run(true) == run(false));
}

TEST_CASE("a later cancellation is its own one-shot request; the activity's receipt shows its end", "[mission][cancel]") {
    MissionEpisode episode(config());
    const std::string key = episode.reset(29);
    MissionEndpoint wing = episode.endpoint(kWing);
    wing.submitJsonV1(actions(key, kWing, 0, request(1, "flight", "new", "{\"mode\":\"hsa\",\"heading_rad\":0.3}")));
    episode.step();
    episode.step();
    const core::Json running = parse(wing.observationJsonV1());
    REQUIRE(receipt(running, 1) != nullptr);
    CHECK(str(*receipt(running, 1), "state") == "executing");
    const std::string activity = str(at(at(*receipt(running, 1), "native"), "ids"), "activity_id");
    wing.submitJsonV1(actions(key, kWing, 2, request(2, "flight", "cancel", "{\"activity_id\":\"" + activity + "\"}")));
    episode.step();
    const core::Json ended = parse(wing.observationJsonV1());
    CHECK(str(*receipt(ended, 2), "state") == "accepted");
    CHECK(at(*receipt(ended, 2), "closed").asBool());
    CHECK(str(*receipt(ended, 1), "state") == "canceled");
    CHECK(at(*receipt(ended, 1), "closed").asBool());
    CHECK(str(*receipt(ended, 1), "updated_tick") == "3");
}

TEST_CASE("a reset to the same local episode equals a fresh episode, snapshot for snapshot", "[mission][reset]") {
    const auto play = [](MissionEpisode& episode, const std::string& key) {
        MissionEndpoint wing = episode.endpoint(kWing), crc = episode.endpoint(kCrc);
        std::vector<std::string> out;
        wing.submitJsonV1(actions(key, kWing, 0,
                                  request(1, "flight", "new", "{\"mode\":\"hsa\",\"heading_rad\":0.4,\"altitude_m\":7200.0}") + "," +
                                      request(2, "dms", "send", "{\"topic\":\"1001\",\"byte_length\":2,\"data_b64\":\"AAE=\"}")));
        crc.submitJsonV1(actions(key, kCrc, 0, request(1, "c2", "declare_role", "{\"role_id\":\"3\",\"callsign\":\"CRC\"}")));
        for (int k = 0; k < 3; ++k) {
            episode.step();
            out.push_back(wing.observationJsonV1());
            out.push_back(crc.observationJsonV1());
        }
        out.push_back(episode.referee().truthJsonV1());
        return out;
    };
    MissionEpisode reused(config());
    play(reused, reused.reset(17));
    const std::string key = reused.reset(17);
    const std::vector<std::string> again = play(reused, key);

    MissionEpisode fresh(config());
    const std::vector<std::string> reference = play(fresh, fresh.reset(17));
    REQUIRE(again.size() == reference.size());
    for (std::size_t i = 0; i < again.size(); ++i) {
        INFO(i);
        // (the generation is the one field normalised: lifetime, never state)
        CHECK(replaced(again[i], "\"episode_generation\":\"2\"", "\"episode_generation\":\"1\"") == reference[i]);
    }
}
