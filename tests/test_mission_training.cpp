// Batches of mission episodes for training, through the public C++ SDK (docs/sdk/mission-training.md, section 18;
// M7 core): independent slots with their own topology, roles that end apart from their episode, autoreset that never
// lets an ended generation act, final documents kept apart by principal, the referee's costs, and actor views that no
// hidden truth can change.
#include "MissionTestSupport.h"

#include "fsim/MissionBatch.h"

#include <catch2/catch_test_macros.hpp>

#include <functional>
#include <set>
#include <string>
#include <vector>

using namespace fsim;
using namespace mission_test;

namespace {

/// The two-ship configuration with another env_id and tick limit.
std::string slotConfig(std::uint64_t envId, int maxTicks = 600) {
    return replaced(replaced(config(), "\"env_id\": \"9007199254740993\"", "\"env_id\": \"" + std::to_string(envId) + "\""), "\"max_ticks\": 600",
                    "\"max_ticks\": " + std::to_string(maxTicks));
}

/// Another topology: the wing and the C2 node alone, two services, no criteria.
std::string wingAndC2(std::uint64_t envId) {
    return "{\"schema\":\"ma.config.v1\",\"env_id\":\"" + std::to_string(envId) +
           "\",\"base_seed\":\"5\",\"timing\":{\"dt_s\":0.008333333333333333,\"frame_skip\":4,\"decision_steps\":30},"
           "\"scenario\":{\"world\":{\"name\":\"wing-c2\",\"capacity\":4,\"jsbsim_root\":" +
           jsonString(FSIM_TEST_JSBSIM_ROOT) +
           "},\"vehicles\":[{\"name\":\"wing\",\"type\":\"jsbsim:f16c\",\"initial\":{\"lat_deg\":35.99,\"lon_deg\":-115.01,"
           "\"alt_msl_m\":7000,\"heading_deg\":0,\"airspeed_ms\":230}}]},"
           "\"c2_nodes\":[{\"name\":\"crc\",\"latitude_rad\":0.6283185307179586,\"longitude_rad\":-2.007128639793479,\"altitude_msl_m\":600.0}],"
           "\"nets\":[{\"name\":\"l16\",\"members\":[\"wing\",\"crc\"]}],"
           "\"services\":[{\"service_id\":\"7\",\"name\":\"wing\",\"platform\":\"wing\",\"command_capabilities\":[\"*\"],"
           "\"owns_dms_ingress\":true,\"owns_dms_egress\":true},{\"service_id\":\"8\",\"name\":\"c2\",\"platform\":\"crc\","
           "\"command_capabilities\":[],\"owns_dms_ingress\":true,\"owns_dms_egress\":true}]}";
}

const core::Json* role(const core::Json& outcome, std::uint64_t service) {
    for (const core::Json& r : at(outcome, "roles").asArray())
        if (str(r, "service_id") == std::to_string(service)) return &r;
    return nullptr;
}
bool flag(const core::Json& outcome, std::uint64_t service, const char* name) { return at(*role(outcome, service), name).asBool(); }

bool threwInvalid(const std::function<void()>& f) {
    try {
        f();
    } catch (const std::invalid_argument&) {
        return true;
    } catch (...) {
    }
    return false;
}

} // namespace

TEST_CASE("slots are independent episodes, each with its own topology", "[training][batch]") {
    MissionBatch batch({slotConfig(11), wingAndC2(12)}, MissionBatchOptions{MissionAutoreset::NextStep, 1, 40});
    CHECK(batch.size() == 2);
    CHECK(threwInvalid([] { MissionBatch duplicate({slotConfig(11), slotConfig(11)}, MissionBatchOptions{}); })); // (env_id unique)
    CHECK(threwInvalid([] { MissionBatch none({}, MissionBatchOptions{}); }));
    batch.reset();
    for (std::size_t s = 0; s < 2; ++s) {
        const MissionSlotStatus st = batch.status(s);
        CHECK(st.generation == 1);
        CHECK(st.localEpisodeIndex == 40);
        CHECK(st.tick == 0);
        CHECK(st.reset);
    }
    INFO("batch, service and actor counts are unrelated: four services in one slot, two in the other");
    CHECK(at(parse(batch.referee(0).outcomeJsonV1()), "roles").asArray().size() == 4);
    CHECK(at(parse(batch.referee(1).outcomeJsonV1()), "roles").asArray().size() == 2);
    CHECK(threwInvalid([&] { batch.endpoint(1, kWing); }));
    CHECK(threwInvalid([&] { batch.endpoint(2, 7); }));
    CHECK(str(at(parse(batch.endpoint(1, 8).observationJsonV1()), "service"), "platform_kind") == "c2_node");
    batch.step();
    CHECK(batch.status(0).tick == 1);
    CHECK(batch.status(1).tick == 1);
    CHECK_FALSE(batch.status(0).reset);
    batch.resetSlot(1);
    CHECK(batch.status(1).generation == 2);
    CHECK(batch.status(1).localEpisodeIndex == 41); // (its own count)
    CHECK(batch.status(0).generation == 1);
    CHECK(batch.status(0).tick == 1);
    MissionBatch moved(std::move(batch));
    CHECK(moved.size() == 2);
    bool movedFrom = false;
    try {
        batch.step();
    } catch (const std::runtime_error&) {
        movedFrom = true;
    }
    CHECK(movedFrom);
}

TEST_CASE("the worker count decides nothing, under every reset schedule", "[training][batch][determinism]") {
    // Slots of different lengths (2 and 3 ticks, and one without an end) so that resets fall on different steps.
    const auto run = [](unsigned workers, MissionAutoreset autoreset) {
        MissionBatch batch({slotConfig(21, 2), slotConfig(22, 3), wingAndC2(23), slotConfig(24, 2)}, MissionBatchOptions{autoreset, workers, 3});
        batch.reset();
        std::vector<std::string> out;
        for (int t = 0; t < 7; ++t) {
            if (!batch.status(0).episodeDone) {
                MissionEndpoint wing = batch.endpoint(0, kWing);
                const core::Json snapshot = parse(wing.observationJsonV1());
                wing.submitJsonV1(actions(at(snapshot, "episode").dump(), kWing, std::stoull(str(snapshot, "tick")),
                                          request(static_cast<std::uint64_t>(t + 1), "flight", "new", "{\"mode\":\"hsa\",\"heading_rad\":0.3}")));
            }
            batch.step();
            for (std::size_t s = 0; s < batch.size(); ++s) {
                const MissionSlotStatus st = batch.status(s);
                out.push_back(std::to_string(st.generation) + "/" + std::to_string(st.localEpisodeIndex) + "/" + std::to_string(st.tick) + "/" +
                              std::to_string(st.ended) + std::to_string(st.reset) + std::to_string(st.episodeDone));
                out.push_back(batch.referee(s).truthJsonV1());
                out.push_back(batch.finalOutcomeJsonV1(s));
            }
            out.push_back(batch.endpoint(0, kWing).observationJsonV1());
        }
        return out;
    };
    for (const MissionAutoreset autoreset : {MissionAutoreset::NextStep, MissionAutoreset::SameStep, MissionAutoreset::Disabled}) {
        INFO(static_cast<int>(autoreset));
        const std::vector<std::string> one = run(1, autoreset);
        CHECK(run(2, autoreset) == one);
        CHECK(run(4, autoreset) == one);
        if (autoreset != MissionAutoreset::Disabled) {
            std::size_t resets = 0;
            for (const std::string& line : one) resets += line.size() < 32 && line.find('/') != std::string::npos && line[line.size() - 2] == '1';
            CHECK(resets >= 4); // (the schedule did reset slots, several times)
        }
    }
}

TEST_CASE("a lost member ends its role while its teammates and the episode go on", "[training][roles]") {
    MissionBatch batch({slotConfig(31)}, MissionBatchOptions{MissionAutoreset::NextStep, 1, 0});
    batch.reset();
    batch.step();
    CHECK(batch.removePlatform(0, "wing"));
    CHECK_FALSE(batch.removePlatform(0, "wing")); // (already gone)
    CHECK(threwInvalid([&] { batch.removePlatform(0, "nobody"); }));
    batch.step();
    const core::Json outcome = parse(batch.referee(0).outcomeJsonV1());
    CHECK_FALSE(at(outcome, "episode_done").asBool());
    CHECK(flag(outcome, kWing, "terminated"));
    CHECK(flag(outcome, kWing, "final_observation"));
    CHECK_FALSE(flag(outcome, kWing, "active"));
    for (const std::uint64_t teammate : {kLeadFlight, kLeadSensors, kCrc}) {
        INFO(teammate);
        CHECK(flag(outcome, teammate, "active"));
        CHECK_FALSE(flag(outcome, teammate, "terminated"));
    }
    CHECK_FALSE(batch.status(0).ended);
    CHECK_FALSE(at(at(parse(batch.endpoint(0, kWing).observationJsonV1()), "service"), "alive").asBool());
}

TEST_CASE("a team episode's end closes every role, the C2 role without an aircraft among them", "[training][roles]") {
    MissionBatch batch({slotConfig(41, 2)}, MissionBatchOptions{MissionAutoreset::Disabled, 1, 0});
    batch.reset();
    batch.step();
    CHECK_FALSE(batch.status(0).ended);
    batch.step();
    CHECK(batch.status(0).ended);
    CHECK(batch.status(0).episodeDone);
    CHECK_FALSE(batch.status(0).reset);
    const core::Json outcome = parse(batch.referee(0).outcomeJsonV1());
    CHECK(str(outcome, "reason") == "truncated");
    for (const std::uint64_t service : {kLeadFlight, kLeadSensors, kCrc, kWing}) {
        INFO(service);
        CHECK(flag(outcome, service, "truncated"));
        CHECK(flag(outcome, service, "bootstrap"));
        CHECK(flag(outcome, service, "final_observation"));
        CHECK_FALSE(flag(outcome, service, "active"));
    }
    INFO("Disabled: the slot holds its ended generation until the range resets it - no request of it is dispatched, "
         "its World does not step; one task instance per episode");
    const std::string truth = batch.referee(0).truthJsonV1();
    MissionEndpoint wing = batch.endpoint(0, kWing);
    const core::Json snapshot = parse(wing.observationJsonV1());
    wing.submitJsonV1(actions(at(snapshot, "episode").dump(), kWing, 2, request(9, "flight", "new", "{\"mode\":\"hsa\",\"heading_rad\":0.3}")));
    const std::string receipts = wing.receiptsJsonV1(), observation = wing.observationJsonV1();
    batch.step();
    CHECK(batch.status(0).generation == 1);
    CHECK(batch.status(0).tick == 2);
    CHECK_FALSE(batch.status(0).ended); // (it ended once)
    CHECK(batch.status(0).episodeDone);
    CHECK(batch.referee(0).truthJsonV1() == truth);
    CHECK(wing.receiptsJsonV1() == receipts); // (the request was never dispatched: no tick closed)
    CHECK(wing.observationJsonV1() == observation);
    batch.resetSlot(0);
    const core::Json fresh = parse(batch.referee(0).outcomeJsonV1());
    CHECK_FALSE(at(fresh, "episode_done").asBool());
    for (const core::Json& c : at(fresh, "criteria").asArray()) CHECK_FALSE(at(c, "satisfied").asBool());
}

TEST_CASE("NextStep: the reset comes before any request of the ended generation", "[training][autoreset]") {
    MissionBatch batch({slotConfig(51, 2)}, MissionBatchOptions{MissionAutoreset::NextStep, 1, 0});
    batch.reset();
    batch.step();
    batch.step();
    REQUIRE(batch.status(0).ended);
    CHECK_FALSE(batch.status(0).reset);
    INFO("the ended generation's snapshot is still its final observation; requests submitted to it are queued");
    MissionEndpoint wing = batch.endpoint(0, kWing);
    const core::Json last = parse(wing.observationJsonV1());
    CHECK(str(last, "tick") == "2");
    const std::string key = at(last, "episode").dump();
    const core::Json ack = parse(wing.submitJsonV1(actions(key, kWing, 2,
        request(1, "dms", "send", "{\"topic\":\"1001\",\"destination_system_ids\":[\"1392508931\"],\"byte_length\":1,\"data_b64\":\"AA==\"}") + "," +
            request(2, "flight", "new", "{\"mode\":\"hsa\",\"heading_rad\":1.0}"))));
    CHECK(submitted(ack, 1) == "queued");
    batch.step();
    const MissionSlotStatus st = batch.status(0);
    CHECK(st.reset);
    CHECK_FALSE(st.ended);
    CHECK(st.generation == 2);
    CHECK(st.tick == 0);
    INFO("nothing of the old requests happened: no step, no dispatch, no message, no activity");
    const core::Json truth = parse(batch.referee(0).truthJsonV1());
    CHECK(at(truth, "time_s").asNumber() == 0.0);
    for (const core::Json& n : at(truth, "nets").asArray()) CHECK(str(n, "transmissions") == "0");
    const core::Json wingNow = parse(batch.endpoint(0, kWing).observationJsonV1());
    CHECK(at(wingNow, "receipts").asArray().empty());
    CHECK(at(at(at(wingNow, "reports"), "activities"), "items").asArray().empty());
    CHECK(at(at(parse(batch.endpoint(0, kCrc).observationJsonV1()), "messages"), "items").asArray().empty());
    bool stale = false;
    try {
        wing.observationJsonV1();
    } catch (const std::runtime_error&) {
        stale = true;
    }
    CHECK(stale);
}

TEST_CASE("SameStep: final results stay apart from the new generation's first snapshot", "[training][autoreset]") {
    MissionBatch batch({slotConfig(61, 2), wingAndC2(62)}, MissionBatchOptions{MissionAutoreset::SameStep, 2, 0});
    batch.reset();
    CHECK(batch.finalObservationJsonV1(0, kWing).empty());
    batch.step();
    batch.step();
    const MissionSlotStatus st = batch.status(0);
    CHECK(st.ended);
    CHECK(st.reset);
    CHECK(st.generation == 2);
    CHECK(st.tick == 0);
    CHECK_FALSE(batch.status(1).ended); // (the other slot has no criteria: it goes on)
    const core::Json final = parse(batch.finalObservationJsonV1(0, kWing));
    CHECK(str(final, "tick") == "2");
    CHECK(str(at(final, "episode"), "episode_generation") == "1");
    const core::Json first = parse(batch.endpoint(0, kWing).observationJsonV1());
    CHECK(str(first, "tick") == "0");
    CHECK(str(at(first, "episode"), "episode_generation") == "2");
    const core::Json finalOutcome = parse(batch.finalOutcomeJsonV1(0));
    CHECK(at(finalOutcome, "episode_done").asBool());
    CHECK(str(at(finalOutcome, "episode"), "episode_generation") == "1");
    CHECK_FALSE(at(parse(batch.referee(0).outcomeJsonV1()), "episode_done").asBool());
    CHECK(str(parse(batch.finalCostsJsonV1(0)), "schema") == "ma.costs.v1");
    CHECK(str(at(parse(batch.finalTruthJsonV1(0)), "episode"), "episode_generation") == "1");
    CHECK(threwInvalid([&] { batch.finalObservationJsonV1(0, 99); }));
}

TEST_CASE("costs are the referee's: requests and communication per service, never in a snapshot", "[training][costs]") {
    MissionEpisode episode(slotConfig(71));
    const std::string key = episode.reset(0);
    MissionEndpoint wing = episode.endpoint(kWing), sensors = episode.endpoint(kLeadSensors);
    wing.submitJsonV1(actions(key, kWing, 0,
        request(1, "dms", "send", "{\"topic\":\"1001\",\"destination_system_ids\":[\"1392508931\"],\"byte_length\":3,\"data_b64\":\"AQID\"}") + "," +
            request(2, "flight", "new", "{\"mode\":\"hsa\",\"heading_rad\":0.4}") + "," + request(3, "strike", "new", "{}")));
    sensors.submitJsonV1(actions(key, kLeadSensors, 0, request(1, "dms", "send", "{\"topic\":\"1001\",\"byte_length\":1,\"data_b64\":\"AA==\"}")));
    episode.step();
    const core::Json costs = parse(episode.referee().costsJsonV1());
    CHECK(str(costs, "schema") == "ma.costs.v1");
    CHECK(str(costs, "tick") == "1");
    std::set<std::string> services;
    for (const core::Json& s : at(costs, "services").asArray()) {
        services.insert(str(s, "service_id"));
        const core::Json& tick = at(s, "tick");
        if (str(s, "service_id") == std::to_string(kWing)) {
            CHECK(at(tick, "requests").asNumber() == 3.0);
            CHECK(at(tick, "refused").asNumber() == 1.0);    // (unsupported_kind)
            CHECK(at(tick, "dispatched").asNumber() == 2.0);
            CHECK(at(tick, "dms_messages").asNumber() == 1.0);
            CHECK(at(tick, "dms_bytes").asNumber() == 3.0);
        }
        if (str(s, "service_id") == std::to_string(kLeadSensors)) {
            CHECK(at(tick, "refused").asNumber() == 1.0);    // (dms_egress_not_owned)
            CHECK(at(tick, "dms_messages").asNumber() == 0.0);
        }
    }
    CHECK(services.size() == 4);
    episode.step();
    const core::Json later = parse(episode.referee().costsJsonV1());
    for (const core::Json& s : at(later, "services").asArray())
        if (str(s, "service_id") == std::to_string(kWing)) {
            CHECK(at(at(s, "tick"), "requests").asNumber() == 0.0);
            CHECK(at(at(s, "total"), "requests").asNumber() == 3.0);
        }
    CHECK(wing.observationJsonV1().find("ma.costs") == std::string::npos);
    CHECK(wing.observationJsonV1().find("dms_bytes") == std::string::npos);
}

TEST_CASE("no hidden truth changes what an actor sees", "[training][critic]") {
    // A third aircraft far beyond every sensor and link, on no net: two episodes differ in its true state alone.
    const auto withBogey = [](double latDeg) {
        return replaced(slotConfig(81), "\"vehicles\": [",
                        "\"vehicles\": [{\"name\": \"bogey\", \"type\": \"jsbsim:f16c\", \"initial\": {\"lat_deg\": " + std::to_string(latDeg) +
                            ", \"lon_deg\": 20.0, \"alt_msl_m\": 9000, \"heading_deg\": 90, \"airspeed_ms\": 250}},");
    };
    const auto run = [](const std::string& configText) {
        MissionEpisode episode(replaced(configText, "\"capacity\": 8", "\"capacity\": 9"));
        episode.reset(0);
        std::vector<std::string> actor, critic;
        for (int t = 0; t < 4; ++t) {
            for (const std::uint64_t s : {kLeadFlight, kLeadSensors, kCrc, kWing}) actor.push_back(episode.endpoint(s).observationJsonV1());
            critic.push_back(episode.referee().truthJsonV1());
            episode.step();
        }
        return std::make_pair(actor, critic);
    };
    const auto a = run(withBogey(-30.0)), b = run(withBogey(-31.5));
    CHECK(a.first == b.first);   // (every actor snapshot, byte for byte)
    CHECK(a.second != b.second); // (the critic's truth tells them apart)
}
