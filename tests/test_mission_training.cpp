// Batches of mission episodes for training, through the public C++ SDK (docs/sdk/mission-training.md, section 18;
// M7 core): independent slots with their own topology, roles that end apart from their episode, autoreset that never
// lets an ended generation act, final documents kept apart by principal, the referee's costs, and actor views that no
// hidden truth can change.
#include "MissionTestSupport.h"

#include "fsim/Comm.h"
#include "fsim/MissionBatch.h"

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <exception>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <thread>
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

/// Whether f failed busy - another call held the batch (std::runtime_error, "busy"); any other failure is f's own.
bool threwBusy(const std::function<void()>& f) {
    try {
        f();
    } catch (const std::runtime_error& e) {
        if (std::string(e.what()).find("busy") == std::string::npos) throw;
        return true;
    }
    return false;
}

/// f(), retried while it fails busy: a read of the range's while another thread's call holds the batch.
template <typename F>
auto unbusy(F&& f) {
    for (;;) {
        try {
            return f();
        } catch (const std::runtime_error& e) {
            if (std::string(e.what()).find("busy") == std::string::npos) throw;
        }
    }
}

/// An actor's thread against a one-tick batch's steps: it reads and submits through an endpoint and reads a referee
/// opened before each step - reopened when its generation ended - so that its calls meet the step's transitions. The
/// range steps until `wholeSteps` steps ran whole; every step meets it or not, and either runs whole or fails busy with
/// nothing changed. Returns the actor's busy refusals.
std::uint64_t preopenedAgainst(MissionAutoreset mode, std::uint64_t envId, int wholeSteps) {
    MissionBatch batch({slotConfig(envId, 1)}, MissionBatchOptions{mode, 1, 0});
    batch.reset();
    std::atomic<bool> stop{false};
    std::atomic<std::uint64_t> calls{0}, refusals{0};
    std::exception_ptr failure;
    // (opened before any step, here: the actor's first handles are preopened ones)
    auto firstWing = std::make_unique<MissionEndpoint>(batch.endpoint(0, kWing));
    auto firstReferee = std::make_unique<MissionReferee>(batch.referee(0));
    std::thread actor([&] {
        try {
            std::unique_ptr<MissionEndpoint> wing = std::move(firstWing);
            std::unique_ptr<MissionReferee> referee = std::move(firstReferee);
            std::uint64_t requestId = 0;
            while (!stop) {
                try {
                    if (!wing || !referee) { // (both, or neither: either opening may find the batch held)
                        auto openedWing = std::make_unique<MissionEndpoint>(batch.endpoint(0, kWing));
                        auto openedReferee = std::make_unique<MissionReferee>(batch.referee(0));
                        wing = std::move(openedWing), referee = std::move(openedReferee);
                    }
                    const core::Json snapshot = parse(wing->observationJsonV1());
                    ++calls;
                    wing->submitJsonV1(actions(at(snapshot, "episode").dump(), kWing, std::stoull(str(snapshot, "tick")),
                                               mission_test::request(++requestId, "flight", "new", "{\"mode\":\"hsa\",\"heading_rad\":0.2}")));
                    (void)wing->receiptsJsonV1();
                    (void)referee->outcomeJsonV1();
                    (void)referee->costsJsonV1();
                } catch (const std::runtime_error& e) {
                    const std::string what = e.what();
                    if (what.find("busy") != std::string::npos) ++refusals;
                    else if (what.find("earlier generation") != std::string::npos) wing.reset(), referee.reset();
                    else throw;
                }
            }
        } catch (...) {
            failure = std::current_exception();
        }
    });
    int whole = 0, busy = 0;
    std::vector<std::string> problems;
    const auto note = [&](bool ok, const std::string& what) {
        if (!ok) problems.push_back(what);
    };
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(180);
    while (whole < wholeSteps && std::chrono::steady_clock::now() < deadline) {
        const MissionSlotStatus before = unbusy([&] { return batch.status(0); });
        const std::string finalBefore = unbusy([&] { return batch.finalOutcomeJsonV1(0); });
        try {
            batch.step();
        } catch (const std::runtime_error& e) {
            if (std::string(e.what()).find("busy") == std::string::npos) throw;
            ++busy;
            // refused whole: nothing of the slot changed
            const MissionSlotStatus after = unbusy([&] { return batch.status(0); });
            note(after.generation == before.generation && after.tick == before.tick && after.episodeDone == before.episodeDone &&
                     after.ended == before.ended && after.reset == before.reset,
                 "a busy step changed the status");
            note(unbusy([&] { return batch.finalOutcomeJsonV1(0); }) == finalBefore, "a busy step changed the final outcome");
            continue;
        }
        ++whole;
        const MissionSlotStatus after = unbusy([&] { return batch.status(0); });
        const std::string step = "step " + std::to_string(whole) + ": ";
        const bool resets = mode == MissionAutoreset::NextStep && before.episodeDone;
        if (resets) { // NextStep, after the end: a reset alone - no tick, no request of the ended generation
            note(after.generation == before.generation + 1 && after.tick == 0 && after.reset && !after.ended && !after.episodeDone,
                 step + "NextStep did not reset alone");
            const core::Json truth = parse(unbusy([&] { return batch.referee(0).truthJsonV1(); }));
            note(at(truth, "time_s").asNumber() == 0.0, step + "the new generation is not at its start");
            continue;
        }
        // the one tick ends the episode: its final documents are the ended generation's, whole
        const core::Json outcome = parse(unbusy([&] { return batch.finalOutcomeJsonV1(0); }));
        const core::Json observation = parse(unbusy([&] { return batch.finalObservationJsonV1(0, kWing); }));
        note(at(outcome, "episode_done").asBool() && str(outcome, "tick") == "1" && str(outcome, "reason") == "truncated", step + "final outcome");
        note(str(at(outcome, "episode"), "episode_generation") == std::to_string(before.generation), step + "final outcome of another generation");
        note(str(observation, "tick") == "1" && str(at(observation, "episode"), "episode_generation") == std::to_string(before.generation),
             step + "final observation");
        note(!parse(unbusy([&] { return batch.finalTruthJsonV1(0); })).isNull() &&
                 !parse(unbusy([&] { return batch.finalCostsJsonV1(0); })).isNull(),
             step + "final truth and costs");
        if (mode == MissionAutoreset::SameStep)
            note(after.generation == before.generation + 1 && after.tick == 0 && after.ended && after.reset && !after.episodeDone,
                 step + "SameStep did not end and reset");
        else
            note(after.generation == before.generation && after.tick == 1 && after.ended && after.episodeDone && !after.reset,
                 step + "NextStep did not end");
    }
    stop = true;
    actor.join();
    if (failure) std::rethrow_exception(failure);
    INFO("whole steps " << whole << ", refused " << busy << "; the actor's calls " << calls << ", refused " << refusals);
    CHECK(whole == wholeSteps);
    for (const std::string& p : problems) FAIL_CHECK(p);
    INFO("uncontended afterwards: the slot goes on");
    const MissionSlotStatus last = batch.status(0);
    batch.step();
    batch.step();
    const MissionSlotStatus later = batch.status(0);
    CHECK(later.generation == last.generation + (mode == MissionAutoreset::SameStep ? 2u : 1u));
    CHECK(calls > 0);
    return refusals;
}

/// A service's costs in a costs document: its "tick" or "total".
const core::Json& spent(const core::Json& costs, std::uint64_t service, const char* which) {
    for (const core::Json& s : at(costs, "services").asArray())
        if (str(s, "service_id") == std::to_string(service)) return at(s, which);
    throw std::logic_error("no costs of service " + std::to_string(service));
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

TEST_CASE("one call at a time holds the whole batch: a conflicting call fails busy and changes nothing", "[training][batch][concurrency]") {
    // A one-tick episode under SameStep: every whole step closes its tick, ends the generation and begins the next,
    // every whole reset begins one - so the generation counts every call that ran, and every status anyone can read
    // between whole calls is a fresh generation's (tick 0, not done). A step that ran in another's middle would break
    // the count, and a status read there would show the ended generation (tick 1, done).
    MissionBatch batch({slotConfig(81, 1)}, MissionBatchOptions{MissionAutoreset::SameStep, 1, 0});
    batch.reset();
    std::atomic<bool> done{false};
    std::exception_ptr failure;
    std::thread worker([&] {
        try {
            while (threwBusy([&] { batch.step(); })) { // (its one step, as soon as the batch is free)
            }
        } catch (...) {
            failure = std::current_exception();
        }
        done = true;
    });
    std::uint64_t ran = 1, busy = 0, reads = 0; // (ran: the first reset; the worker's step is added below)
    std::vector<std::string> torn;
    const auto attempt = [&](const std::function<void()>& f) -> std::uint64_t {
        if (threwBusy(f)) {
            ++busy;
            return 0;
        }
        return 1;
    };
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(120);
    while (!done && std::chrono::steady_clock::now() < deadline) {
        reads += attempt([&] {
            const MissionSlotStatus st = batch.status(0);
            if (st.tick != 0 || st.episodeDone || !st.reset) torn.push_back(std::to_string(st.generation) + "/" + std::to_string(st.tick));
        });
        attempt([&] { (void)batch.finalOutcomeJsonV1(0); });
        attempt([&] { (void)batch.finalObservationJsonV1(0, kWing); });
        attempt([&] { (void)batch.referee(0); });
        attempt([&] { (void)batch.endpoint(0, kWing); });
        if (ran < 3) { // (a few whole calls of its own; then the worker must find the batch free)
            ran += attempt([&] { batch.step(); });
            ran += attempt([&] { batch.resetSlot(0); });
            ran += attempt([&] { batch.reset(); });
        }
    }
    worker.join(); // (the worker ends whatever the deadline: its step runs once the batch is free)
    if (failure) std::rethrow_exception(failure);
    ++ran;
    INFO("whole calls " << ran << ", busy refusals " << busy << ", status reads " << reads);
    CHECK(busy > 0); // (the calls did meet: some found the batch held)
    CHECK(torn.empty());
    const MissionSlotStatus st = batch.status(0);
    CHECK(st.generation == ran);
    CHECK(st.localEpisodeIndex == ran - 1);
    CHECK(st.tick == 0);
    CHECK_FALSE(st.episodeDone);
}

TEST_CASE("a step refused busy steps no slot", "[training][batch][concurrency]") {
    // While one slot's reset holds the batch, every step tried is refused whole: slot 0 has ticked exactly as many times as
    // steps ran whole (before the reset took the batch, or after it let go), and the reset slot at most as many.
    MissionBatch batch({slotConfig(82), slotConfig(83)}, MissionBatchOptions{MissionAutoreset::NextStep, 2, 0});
    batch.reset();
    const std::string before = batch.referee(0).truthJsonV1();
    std::atomic<bool> done{false};
    std::exception_ptr failure;
    std::uint64_t refused = 0, whole = 0;
    std::thread worker([&] {
        try {
            while (threwBusy([&] { batch.resetSlot(1); })) { // (a World rebuilt: long enough to meet; retried while a step holds)
            }
        } catch (...) {
            failure = std::current_exception();
        }
        done = true;
    });
    while (!done)
        (threwBusy([&] { batch.step(); }) ? refused : whole) += 1;
    worker.join();
    if (failure) std::rethrow_exception(failure);
    const MissionSlotStatus s0 = batch.status(0), s1 = batch.status(1);
    INFO("refused steps " << refused << ", whole steps " << whole << "; slot 0 at tick " << s0.tick << ", slot 1 at " << s1.tick);
    CHECK(refused > 0);
    CHECK(s0.tick == whole); // (a refused step stepped nothing)
    CHECK(s1.generation == 2);
    CHECK(s1.tick <= whole);
    if (whole == 0) CHECK(batch.referee(0).truthJsonV1() == before);
}

TEST_CASE("native publication costs the bytes it sends: one encoded report per message", "[training][costs]") {
    MissionEpisode episode(slotConfig(72));
    const std::string key = episode.reset(0);
    MissionEndpoint wing = episode.endpoint(kWing), crc = episode.endpoint(kCrc);
    const std::string position = request(1, "dms", "publish_native", "{\"report\":\"position\",\"destination_system_ids\":[\"1392508931\"]}");
    wing.submitJsonV1(actions(key, kWing, 0,
        position + "," + request(2, "dms", "publish_native", "{\"report\":\"status\",\"destination_system_ids\":[\"1392508931\"]}") + "," +
            request(3, "dms", "publish_native", "{\"report\":\"tracks\",\"destination_system_ids\":[\"1392508931\"]}") + "," +
            request(4, "dms", "publish_native", "{\"report\":\"position\",\"destination_system_ids\":[\"99\"]}")));
    episode.step();
    // what each request published, as its native receipt says
    std::uint64_t positions = 0, statuses = 0, tracks = 0, refusedPublications = 0;
    std::string positionMessage;
    const core::Json receipts = parse(wing.receiptsJsonV1());
    for (const core::Json& r : at(receipts, "receipts").asArray()) {
        const core::Json& n = at(r, "native");
        if (str(n, "status") != "published") {
            ++refusedPublications;
            continue;
        }
        const core::Json* count = at(n, "ids").find("message_count");
        const std::uint64_t messages = count ? static_cast<std::uint64_t>(count->asNumber()) : 0;
        const std::string id = str(r, "request_id");
        (id == "1" ? positions : id == "2" ? statuses : tracks) += messages;
        if (id == "1") positionMessage = str(at(n, "ids"), "message_id");
    }
    CHECK(positions == 1);
    CHECK(statuses == 1);
    CHECK(refusedPublications == 1); // (request 4: no such system - no message, no bytes)
    const std::uint64_t bytes = positions * sizeof(comm::PositionReport) + statuses * sizeof(comm::SystemStatusReport) +
                                tracks * sizeof(comm::EntityReport);
    const core::Json costs = parse(episode.referee().costsJsonV1());
    const core::Json& tick = spent(costs, kWing, "tick");
    CHECK(at(tick, "dms_messages").asNumber() == static_cast<double>(positions + statuses + tracks));
    CHECK(at(tick, "dms_bytes").asNumber() == static_cast<double>(bytes));
    INFO("the bytes counted are the bytes delivered: the C2 node's snapshot holds the position report");
    bool delivered = false;
    for (int t = 0; t < 3 && !delivered; ++t) {
        const core::Json snapshot = parse(crc.observationJsonV1());
        for (const core::Json& m : at(at(snapshot, "messages"), "items").asArray())
            if (str(m, "source_system_id") == "1392508930" && str(m, "message_id") == positionMessage) {
                CHECK(at(m, "byte_length").asNumber() == static_cast<double>(sizeof(comm::PositionReport)));
                delivered = true;
            }
        if (!delivered) episode.step();
    }
    CHECK(delivered);
    INFO("an identical retry is the original acknowledgment: nothing sent again, nothing counted again");
    const std::uint64_t now = std::stoull(str(parse(wing.observationJsonV1()), "tick"));
    const double before = at(spent(parse(episode.referee().costsJsonV1()), kWing, "total"), "dms_bytes").asNumber();
    wing.submitJsonV1(actions(key, kWing, now, position));
    episode.step();
    const core::Json after = parse(episode.referee().costsJsonV1());
    CHECK(at(spent(after, kWing, "tick"), "dms_messages").asNumber() == 0.0);
    CHECK(at(spent(after, kWing, "total"), "dms_bytes").asNumber() == before);
}

TEST_CASE("a platform's own dissemination is no service's cost", "[training][costs]") {
    // The fixture's Data Dissemination Plans publish positions every second on their own; no request sent them.
    MissionEpisode episode(slotConfig(74));
    episode.reset(0);
    MissionEndpoint crc = episode.endpoint(kCrc);
    std::size_t heard = 0;
    for (int t = 0; t < 4; ++t) {
        episode.step();
        heard += at(at(parse(crc.observationJsonV1()), "messages"), "items").asArray().size();
    }
    CHECK(heard > 0);
    const core::Json costs = parse(episode.referee().costsJsonV1());
    for (const core::Json& s : at(costs, "services").asArray()) {
        CHECK(at(at(s, "total"), "dms_messages").asNumber() == 0.0);
        CHECK(at(at(s, "total"), "dms_bytes").asNumber() == 0.0);
    }
}

TEST_CASE("SameStep under an actor's preopened calls: every step whole - final documents, the next generation, nothing stuck",
          "[training][autoreset][concurrency]") {
    CHECK(preopenedAgainst(MissionAutoreset::SameStep, 93, 200) > 0); // (the actor's calls did meet the steps)
}

TEST_CASE("NextStep under an actor's preopened calls: the end, then a reset alone - no request of the ended generation",
          "[training][autoreset][concurrency]") {
    CHECK(preopenedAgainst(MissionAutoreset::NextStep, 94, 200) > 0);
}
