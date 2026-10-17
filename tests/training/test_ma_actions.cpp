// The mission gateway's requests, through the public C++ SDK (docs/sdk/mission-training.md, sections 4, 5, 11 and 12;
// M5 core): a configuration validated whole, handles bound to their generation, a submission checked whole and then
// request by request, identical retries answered from the record, native refusals transported as the SDK gives them,
// and the service the endpoint authenticates - never one a document names.
#include "MissionTestSupport.h"

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <functional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace fsim;
using namespace mission_test;

namespace {

std::string withLimits(const std::string& limits) {
    return replaced(config(),
                    "\"completed_request_retention_ticks\": 64,\n    \"max_in_flight_requests\": 256,\n    \"max_requests_per_document\": 64,\n"
                    "    \"max_document_bytes\": 1048576,\n    \"max_message_bytes\": 1024",
                    limits);
}

const char* const kHsaNew = "{\"mode\":\"hsa\",\"heading_rad\":0.5,\"speed\":230.0,\"altitude_m\":7000.0}";

/// The state of a request's receipt, after a submit or in a snapshot.
std::string stateOf(const core::Json& document, std::uint64_t requestId) {
    const core::Json* r = receipt(document, requestId);
    return r ? str(*r, "state") : std::string("(none)");
}
std::string gatewayReason(const core::Json& document, std::uint64_t requestId) {
    const core::Json* r = receipt(document, requestId);
    return r && !at(*r, "gateway_reason").isNull() ? str(*r, "gateway_reason") : std::string();
}

bool threwRuntime(const std::function<void()>& f) {
    try {
        f();
    } catch (const std::invalid_argument&) {
        return false;
    } catch (const std::runtime_error&) {
        return true;
    }
    return false;
}
bool threwInvalid(const std::function<void()>& f) {
    try {
        f();
    } catch (const std::invalid_argument&) {
        return true;
    } catch (...) {
        return false;
    }
    return false;
}

} // namespace

TEST_CASE("a configuration is validated whole and builds no World", "[mission][config]") {
    CHECK_NOTHROW(MissionEpisode(config()));
    for (const char* name : {"invalid.config.capability-twice.json", "invalid.config.controller-named.json", "invalid.config.duplicate-service.json",
                             "invalid.config.missing-timing.json", "invalid.config.no-egress-owner.json",
                             "invalid.config.publish-without-diagnostic.json", "invalid.config.scenario-owns-seed.json",
                             "invalid.config.seed-as-number.json", "invalid.config.two-ingress-owners.json", "invalid.config.unknown-platform.json"}) {
        INFO(name);
        CHECK(threwInvalid([&] { MissionEpisode refused(fixture(name)); }));
    }
    const std::string good = config();
    INFO("not strict JSON, or a member the version does not define, or an extension nothing installed registered");
    CHECK(threwInvalid([&] { MissionEpisode refused("\xEF\xBB\xBF" + good); }));
    CHECK(threwInvalid([&] { MissionEpisode refused(replaced(good, "\"schema\": \"ma.config.v1\",", "\"schema\": \"ma.config.v1\", // c\n")); }));
    CHECK(threwInvalid([&] { MissionEpisode refused(replaced(good, "\"env_id\": \"9007199254740993\",", "\"env_id\": \"9007199254740993\", \"env_id\": \"1\",")); }));
    CHECK(threwInvalid([&] { MissionEpisode refused(replaced(good, "\"terrain\": false,", "\"terrain\": false,,")); }));
    CHECK(threwInvalid([&] { MissionEpisode refused(replaced(good, "\"dt_s\": 0.008333333333333333", "\"dt_s\": 1e400")); }));
    CHECK(threwInvalid([&] { MissionEpisode refused(replaced(good, "\"workers\": 1,", "\"workers\": 1, \"seed\": 3,")); }));
    CHECK(threwInvalid([&] { MissionEpisode refused(replaced(good, "\"execution\": {", "\"extensions\": {\"training.combat.v1\": {}},\n  \"execution\": {")); }));
    CHECK(threwInvalid([&] { MissionEpisode refused(replaced(good, "\"vehicles\": [", "\"vecenv\": {\"num_envs\": 2},\n    \"vehicles\": [")); }));
    CHECK(threwInvalid([&] { MissionEpisode refused(replaced(good, "\"name\": \"lead-sensors\"", "\"name\": \"lead-flight\"")); }));
}

TEST_CASE("handles answer only in their generation, and a moved or ended episode closes them", "[mission][lifetime]") {
    MissionEpisode episode(config());
    CHECK(threwRuntime([&] { episode.endpoint(kWing); }));
    CHECK(threwRuntime([&] { episode.referee(); }));
    CHECK(threwRuntime([&] { episode.step(); }));

    const std::string key1 = episode.reset(41);
    const core::Json k1 = parse(key1);
    CHECK(str(k1, "env_id") == "9007199254740993");
    CHECK(str(k1, "episode_generation") == "1");
    CHECK(str(k1, "local_episode_index") == "41");
    CHECK(threwInvalid([&] { episode.endpoint(3); }));

    MissionEndpoint first = episode.endpoint(kWing);
    MissionReferee firstReferee = episode.referee();
    const std::string observation = first.observationJsonV1();
    CHECK(str(at(parse(observation), "episode"), "episode_generation") == "1");

    const std::string key2 = episode.reset(41);
    CHECK(str(parse(key2), "episode_generation") == "2");
    INFO("every earlier handle is stale: each call fails and touches nothing");
    CHECK(threwRuntime([&] { first.observationJsonV1(); }));
    CHECK(threwRuntime([&] { first.receiptsJsonV1(); }));
    CHECK(threwRuntime([&] { first.submitJsonV1(actions(key2, kWing, 0, "")); }));
    CHECK(threwRuntime([&] { firstReferee.outcomeJsonV1(); }));
    CHECK(threwRuntime([&] { firstReferee.truthJsonV1(); }));

    MissionEndpoint second = episode.endpoint(kWing);
    INFO("a well-formed document of an earlier generation is answered, not thrown");
    const std::string stale = second.submitJsonV1(actions(key1, kWing, 0, request(1, "flight", "new", kHsaNew)));
    const core::Json s = parse(stale);
    CHECK(at(at(s, "submission"), "accepted").asBool() == false);
    CHECK(str(at(s, "submission"), "reason") == "stale_generation");
    CHECK(at(at(s, "submission"), "requests").asArray().empty());
    CHECK(at(s, "receipts").asArray().empty());
    CHECK(second.receiptsJsonV1() == stale);

    INFO("moving the parent keeps its children; a moved-from episode is closed");
    MissionEpisode moved(std::move(episode));
    CHECK(second.observationJsonV1() == second.observationJsonV1());
    CHECK(threwRuntime([&] { episode.step(); }));
    CHECK(threwRuntime([&] { episode.reset(1); }));
    CHECK_NOTHROW(moved.step());
    CHECK(str(parse(second.observationJsonV1()), "tick") == "1");

    MissionEndpoint third(std::move(second));
    CHECK(threwRuntime([&] { second.observationJsonV1(); }));
    CHECK_NOTHROW(third.observationJsonV1());

    MissionReferee referee = moved.referee();
    {
        MissionEpisode ending(std::move(moved));
    }
    INFO("the parent's end closes every child before its World goes");
    CHECK(threwRuntime([&] { third.observationJsonV1(); }));
    CHECK(threwRuntime([&] { referee.outcomeJsonV1(); }));
}

TEST_CASE("a submission is checked whole, against the endpoint's generation, tick and service", "[mission][submit]") {
    MissionEpisode episode(config());
    const std::string key = episode.reset(7);
    MissionEndpoint lead = episode.endpoint(kLeadFlight);
    const std::string before = lead.receiptsJsonV1();
    CHECK(at(parse(before), "submission").isNull());

    const auto refused = [&](const std::string& document) {
        const std::string text = lead.submitJsonV1(document);
        const core::Json r = parse(text);
        CHECK(lead.receiptsJsonV1() == text);
        CHECK(at(r, "receipts").asArray().empty()); // (nothing queued)
        return at(at(r, "submission"), "accepted").asBool() ? std::string("accepted") : str(at(r, "submission"), "reason");
    };
    const std::string requests = request(5, "flight", "new", kHsaNew);
    CHECK(refused(actions(key, kLeadFlight, 1, requests)) == "future_tick");
    CHECK(refused(actions(key, kWing, 0, requests)) == "wrong_service");
    CHECK(refused(actions(replaced(key, "\"episode_generation\":\"1\"", "\"episode_generation\":\"2\""), kLeadFlight, 0, requests)) == "stale_generation");
    CHECK(refused(actions(replaced(key, "\"local_episode_index\":\"7\"", "\"local_episode_index\":\"8\""), kLeadFlight, 0, requests)) == "stale_generation");
    CHECK(refused(actions(replaced(key, "\"env_id\":\"9007199254740993\"", "\"env_id\":\"1\""), kLeadFlight, 0, requests)) == "stale_generation");
    episode.step();
    CHECK(refused(actions(key, kLeadFlight, 0, requests)) == "late_tick");
    CHECK(stateOf(parse(lead.observationJsonV1()), 5) == "(none)");

    INFO("a malformed document is an error of the call, and changes nothing");
    const std::string last = lead.receiptsJsonV1();
    for (const char* name : {"invalid.actions.base64-alphabet.json", "invalid.actions.base64-length.json", "invalid.actions.base64-noncanonical.json",
                             "invalid.actions.id-above-u64.json", "invalid.actions.id-as-number.json", "invalid.actions.id-leading-zero.json",
                             "invalid.actions.id-negative.json", "invalid.actions.ids-not-increasing.json", "invalid.actions.missing-parameter.json",
                             "invalid.actions.override-without-value.json", "invalid.actions.unknown-field.json",
                             "invalid.actions.unknown-operation.json", "invalid.actions.unknown-parameter.json",
                             "invalid.actions.update-behavior-target.json", "invalid.actions.update-without-activity.json",
                             "refused.actions.byte-order-mark.json", "refused.actions.comment.json", "refused.actions.duplicate-key.json",
                             "refused.actions.not-utf8.json", "refused.actions.overflow-number.json", "refused.actions.trailing-comma.json"}) {
        INFO(name);
        CHECK(threwInvalid([&] { lead.submitJsonV1(fixture(name)); }));
    }
    const std::string wellFormed = actions(key, kLeadFlight, 1, request(9, "flight", "new", kHsaNew));
    CHECK(threwInvalid([&] { lead.submitJsonV1(wellFormed + " x"); }));
    CHECK(threwInvalid([&] { lead.submitJsonV1(wellFormed + std::string(1048576, ' ')); }));
    std::string many;
    for (int i = 1; i <= 65; ++i) many += (i > 1 ? "," : "") + request(static_cast<std::uint64_t>(100 + i), "dms", "subscribe", "{\"topic\":\"1001\"}");
    CHECK(threwInvalid([&] { lead.submitJsonV1(actions(key, kLeadFlight, 1, many)); }));
    INFO("the endpoint authenticates the service: a request member naming a sender, controller or platform is unknown");
    CHECK(threwInvalid([&] {
        lead.submitJsonV1(actions(key, kLeadFlight, 1, request(9, "dms", "send", "{\"topic\":\"1001\",\"byte_length\":0,\"data_b64\":\"\",\"source_system_id\":\"1392508930\"}")));
    }));
    CHECK(threwInvalid([&] { lead.submitJsonV1(actions(key, kLeadFlight, 1, request(9, "flight", "new", "{\"mode\":\"hsa\",\"controller_id\":\"1073741828\"}"))); }));
    CHECK(threwInvalid([&] {
        lead.submitJsonV1("{\"schema\":\"ma.actions.v1\",\"episode\":" + key + ",\"service_id\":\"1\",\"sender\":\"4\",\"tick\":\"1\",\"requests\":[]}");
    }));
    CHECK(lead.receiptsJsonV1() == last);
}

TEST_CASE("identical retries return the record, a changed body is refused, and old ids expire", "[mission][dedup]") {
    MissionEpisode episode(withLimits("\"completed_request_retention_ticks\": 1"));
    const std::string key = episode.reset(3);
    MissionEndpoint wing = episode.endpoint(kWing);

    const std::string first = actions(key, kWing, 0, request(10, "flight", "new", kHsaNew));
    const core::Json a = parse(wing.submitJsonV1(first));
    CHECK(submitted(a, 10) == "queued");
    CHECK(stateOf(a, 10) == "queued");
    CHECK(wing.receiptsJsonV1() == wing.receiptsJsonV1());

    const core::Json again = parse(wing.submitJsonV1(first));
    CHECK(submitted(again, 10) == "duplicate");
    std::string why;
    const core::Json changed = parse(wing.submitJsonV1(actions(key, kWing, 0, request(10, "flight", "new", "{\"mode\":\"hsa\",\"heading_rad\":0.6}"))));
    CHECK(submitted(changed, 10, &why) == "rejected");
    CHECK(why == "request_id_reuse");
    const core::Json older = parse(wing.submitJsonV1(actions(key, kWing, 0, request(9, "flight", "new", kHsaNew))));
    CHECK(submitted(older, 9, &why) == "rejected");
    CHECK(why == "retry_expired");
    CHECK(receipt(older, 9) == nullptr);

    episode.step();
    const core::Json tick1 = parse(wing.observationJsonV1());
    const core::Json* r = receipt(tick1, 10);
    REQUIRE(r != nullptr);
    CHECK(str(*r, "state") == "executing");
    CHECK(str(at(*r, "native"), "domain") == "vi");
    CHECK(str(at(at(*r, "native"), "ids"), "command_id") == "10");
    const std::string activity = str(at(at(*r, "native"), "ids"), "activity_id");
    const auto owned = [&](const core::Json& observation) {
        int n = 0;
        for (const core::Json& item : at(at(at(observation, "reports"), "activities"), "items").asArray())
            if (!at(item, "owner_service_id").isNull() && str(item, "owner_service_id") == std::to_string(kWing)) ++n;
        return n;
    };
    CHECK(owned(tick1) == 1);

    INFO("an open receipt is never forgotten: the identical request at a later tick is its duplicate and repeats nothing");
    episode.step();
    episode.step();
    const core::Json late = parse(wing.submitJsonV1(actions(key, kWing, 3, request(10, "flight", "new", kHsaNew))));
    CHECK(submitted(late, 10) == "duplicate");
    episode.step();
    CHECK(owned(parse(wing.observationJsonV1())) == 1);

    INFO("gateway.close stops following it in its last known state; the activity flies on natively");
    wing.submitJsonV1(actions(key, kWing, 4, request(11, "gateway", "close", "{\"request_id\":\"10\"}") + "," +
                                                   request(12, "gateway", "close", "{\"request_id\":\"4242\"}")));
    episode.step();
    const core::Json closed = parse(wing.observationJsonV1());
    CHECK(at(*receipt(closed, 10), "closed").asBool());
    CHECK(str(*receipt(closed, 10), "state") == "executing");
    CHECK(stateOf(closed, 11) == "accepted");
    CHECK(stateOf(closed, 12) == "rejected");
    CHECK(gatewayReason(closed, 12) == "unknown_request");
    bool live = false;
    for (const core::Json& item : at(at(at(closed, "reports"), "activities"), "items").asArray())
        live = live || str(item, "activity_id") == activity;
    CHECK(live);

    INFO("a closed receipt is kept for the configured ticks, then its id is expired - never re-executed");
    episode.step();
    episode.step();
    const core::Json expired = parse(wing.submitJsonV1(actions(key, kWing, 7, request(10, "flight", "new", kHsaNew))));
    CHECK(submitted(expired, 10, &why) == "rejected");
    CHECK(why == "retry_expired");
    CHECK(receipt(expired, 10) == nullptr);
}

TEST_CASE("capacity and message size refuse one request, with a closed receipt of its own", "[mission][limits]") {
    MissionEpisode episode(withLimits("\"max_in_flight_requests\": 1, \"max_message_bytes\": 4"));
    const std::string key = episode.reset(3);
    MissionEndpoint wing = episode.endpoint(kWing);
    const std::string document = actions(key, kWing, 0,
                                         request(20, "flight", "new", kHsaNew) + "," + request(21, "flight", "new", kHsaNew) + "," +
                                             request(22, "dms", "send", "{\"topic\":\"1001\",\"byte_length\":7,\"data_b64\":\"//4AgMMoQQ==\"}") + "," +
                                             request(23, "strike", "new", "{}"));
    const core::Json r = parse(wing.submitJsonV1(document));
    std::string why;
    CHECK(submitted(r, 20) == "queued");
    CHECK(submitted(r, 21, &why) == "rejected");
    CHECK(why == "capacity");
    CHECK(submitted(r, 22, &why) == "rejected");
    CHECK(why == "message_too_large");
    CHECK(submitted(r, 23, &why) == "rejected");
    CHECK(why == "unsupported_kind");
    for (const std::uint64_t id : {21u, 22u, 23u}) {
        INFO(id);
        CHECK(stateOf(r, id) == "rejected");
        CHECK(at(*receipt(r, id), "closed").asBool());
        CHECK(at(*receipt(r, id), "native").isNull());
    }
    CHECK(gatewayReason(r, 21) == "capacity");
    CHECK(submitted(parse(wing.submitJsonV1(document)), 21) == "duplicate");
}

TEST_CASE("dispatch checks ownership as the service, and transports native refusals unchanged", "[mission][dispatch]") {
    MissionEpisode episode(config());
    const std::string key = episode.reset(11);
    MissionEndpoint flight = episode.endpoint(kLeadFlight), sensors = episode.endpoint(kLeadSensors), crc = episode.endpoint(kCrc),
                    wing = episode.endpoint(kWing);

    sensors.submitJsonV1(actions(key, kLeadSensors, 0,
                                 request(1, "flight", "new", kHsaNew) + "," +
                                     request(2, "dms", "send", "{\"topic\":\"1001\",\"byte_length\":3,\"data_b64\":\"AQID\"}") + "," +
                                     request(3, "dms", "subscribe", "{\"topic\":\"1001\"}") + "," +
                                     request(4, "sensor", "set_state", "{\"payload\":\"0\",\"state\":\"operate\"}")));
    crc.submitJsonV1(actions(key, kCrc, 0,
                             request(1, "flight", "new", kHsaNew) + "," + request(2, "sensor", "set_state", "{\"payload\":\"0\",\"state\":\"operate\"}") +
                                 "," + request(3, "c2", "declare_role", "{\"role_id\":\"2\",\"callsign\":\"CRC LEAD\"}")));
    flight.submitJsonV1(actions(key, kLeadFlight, 0,
                                request(1, "flight", "update", "{\"mode\":\"hsa\",\"activity_id\":\"12345\",\"heading_rad\":0.2}") + "," +
                                    request(2, "sensor", "set_state", "{\"payload\":\"0\",\"state\":\"operate\"}") + "," +
                                    request(3, "flight", "cancel", "{\"activity_id\":\"4294968295\"}")));
    wing.submitJsonV1(actions(key, kWing, 0, request(1, "sensor", "designate_link", "{\"source_system_id\":\"1392508929\",\"payload\":\"0\",\"track_id\":\"1\"}")));
    episode.step();

    const core::Json s = parse(sensors.observationJsonV1());
    CHECK(gatewayReason(s, 1) == "capability_not_bound");
    CHECK(gatewayReason(s, 2) == "dms_egress_not_owned");
    CHECK(gatewayReason(s, 3) == "dms_ingress_not_owned");
    CHECK(gatewayReason(s, 4).empty());
    REQUIRE(receipt(s, 4) != nullptr);
    CHECK(str(at(*receipt(s, 4), "native"), "domain") == "ms");
    CHECK(str(*receipt(s, 4), "state") != "rejected");

    const core::Json c = parse(crc.observationJsonV1());
    CHECK(gatewayReason(c, 1) == "not_applicable");
    CHECK(gatewayReason(c, 2) == "not_applicable");
    CHECK(stateOf(c, 3) == "accepted");
    CHECK(str(at(*receipt(c, 3), "native"), "status") == "sent");

    const core::Json f = parse(flight.observationJsonV1());
    REQUIRE(receipt(f, 1) != nullptr);
    CHECK(stateOf(f, 1) == "rejected");
    CHECK(gatewayReason(f, 1).empty());
    CHECK(str(at(*receipt(f, 1), "native"), "domain") == "vi");
    INFO("another vehicle's activity: the Agent's own refusal (fsim/Agent.h), transported unchanged");
    CHECK(str(at(*receipt(f, 1), "native"), "reason") == "not_allowed");
    CHECK(gatewayReason(f, 2) == "capability_not_bound");
    INFO("an activity of its own vehicle (lead, entity 1) that does not exist");
    CHECK(stateOf(f, 3) == "rejected");
    CHECK(str(at(*receipt(f, 3), "native"), "reason") == "unknown_activity");

    const core::Json w = parse(wing.observationJsonV1());
    CHECK(stateOf(w, 1) == "rejected");
    CHECK(str(at(*receipt(w, 1), "native"), "domain") == "designation");
    CHECK(str(at(*receipt(w, 1), "native"), "status") == "not_held");
}

TEST_CASE("a C2 task travels the modelled link and its receipt follows the MA's answers", "[mission][c2]") {
    MissionEpisode episode(config());
    const std::string key = episode.reset(12);
    MissionEndpoint crc = episode.endpoint(kCrc), wing = episode.endpoint(kWing);
    crc.submitJsonV1(actions(key, kCrc, 0, request(1, "c2", "declare_role", "{\"role_id\":\"2\",\"callsign\":\"CRC LEAD\"}")));
    episode.step(); // (the role first: a task that overtook it on the link would be unauthorized)
    crc.submitJsonV1(actions(key, kCrc, 1,
                             request(2, "c2", "send_task",
                                     "{\"ma_system_id\":\"1392508930\",\"type\":\"patrol\",\"priority\":1,\"radius_m\":3000.0,\"text\":\"north\"}")));
    episode.step();
    const core::Json sent = parse(crc.observationJsonV1());
    CHECK(stateOf(sent, 2) == "pending");
    const std::string task = str(at(at(*receipt(sent, 2), "native"), "ids"), "task_id");

    std::string fromSystem;
    for (int k = 0; k < 6 && fromSystem.empty(); ++k) {
        const core::Json o = parse(wing.observationJsonV1());
        for (const core::Json& t : at(at(at(o, "reports"), "tasks"), "items").asArray())
            if (str(t, "task_id") == task) fromSystem = str(t, "from_system_id");
        if (fromSystem.empty()) episode.step();
    }
    REQUIRE(fromSystem == "1392508931");
    const std::uint64_t tick = std::stoull(str(parse(wing.observationJsonV1()), "tick"));
    wing.submitJsonV1(actions(key, kWing, tick, request(1, "task", "accept", "{\"from_system_id\":\"1392508931\",\"task_id\":\"" + task + "\"}")));
    episode.step();
    CHECK(stateOf(parse(wing.observationJsonV1()), 1) == "accepted");
    std::string state;
    for (int k = 0; k < 6 && state != "executing"; ++k) {
        episode.step();
        state = stateOf(parse(crc.observationJsonV1()), 2);
    }
    CHECK(state == "executing");
}

TEST_CASE("a reset refused as busy changes nothing of the reset that holds the episode", "[mission][lifetime][reset]") {
    // The public reset is one operation under the episode's boundary: its generation's metadata, the native rebuild and
    // the episode key it returns. Another reset meanwhile fails busy and touches none of them; nor can a call after the
    // rebuild change or fail the key the first one returns.
    MissionEpisode episode(loadedConfig(36));
    episode.reset(0);
    MissionEndpoint earlier = episode.endpoint(kWing);
    std::string key;
    std::atomic<bool> done{false};
    std::thread first([&] {
        try {
            key = episode.reset(11);
        } catch (const std::exception& e) {
            key = std::string("threw: ") + e.what();
        }
        done = true;
    });
    bool busySeen = false, secondRefused = false, secondReturned = false;
    const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    while (!done && std::chrono::steady_clock::now() < until && !busySeen) {
        try {
            earlier.observationJsonV1();
        } catch (const std::runtime_error& e) {
            busySeen = std::string(e.what()).find("busy") != std::string::npos;
        }
        if (!busySeen) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (busySeen) {
        try {
            episode.reset(22);
            secondReturned = true;
        } catch (const std::runtime_error& e) {
            secondRefused = std::string(e.what()).find("busy") != std::string::npos;
        }
    }
    first.join();
    REQUIRE(busySeen); // (the first reset was met while it held the episode)
    CHECK(secondRefused);
    CHECK_FALSE(secondReturned);
    CHECK(key == "{\"env_id\":\"9007199254740993\",\"episode_generation\":\"2\",\"local_episode_index\":\"11\"}");
    const core::Json snapshot = parse(episode.endpoint(kWing).observationJsonV1());
    CHECK(str(at(snapshot, "episode"), "local_episode_index") == "11");
    CHECK(str(at(snapshot, "episode"), "episode_generation") == "2");
}
