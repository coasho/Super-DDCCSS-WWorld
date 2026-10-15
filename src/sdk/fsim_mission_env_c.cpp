// Mission episodes through the C ABI (fsim/fsim_mission_env.h; docs/sdk/mission-training.md, sections 3 to 5; M5 core).
//
// Three opaque handle kinds, each the public C++ object behind a kind tag: a call given a live handle of another kind
// answers FSIM_NOT_ALLOWED and touches nothing. Status mapping is section 5's: a malformed document, an unconfigured
// service, a handle of an earlier generation or ended episode, an episode not reset, a null pointer or a buffer too
// small - FSIM_INVALID_ARGUMENT; the boundary held by another call - FSIM_ERROR.

#include "fsim/fsim_mission_env.h"

#include "fsim/MissionEpisode.h"
#include "sdk/CAbiSupport.h"
#include "sdk/MissionGateway.h"

#include <cstring>
#include <new>
#include <utility>

namespace {

enum Kind : std::uint32_t {
    kEpisodeKind = 0x4D45504Bu,  // "MEPK"
    kEndpointKind = 0x4D454E44u, // "MEND"
    kRefereeKind = 0x4D524546u,  // "MREF"
    kDestroyed = 0,
};

/// Every mission handle starts with its kind.
struct MissionHandle {
    std::uint32_t kind;
};

} // namespace

struct fsim_mission_episode : MissionHandle {
    explicit fsim_mission_episode(fsim::MissionEpisode e) : MissionHandle{kEpisodeKind}, episode(std::move(e)) {}
    fsim::MissionEpisode episode;
};
struct fsim_mission_endpoint : MissionHandle {
    explicit fsim_mission_endpoint(fsim::MissionEndpoint e) : MissionHandle{kEndpointKind}, endpoint(std::move(e)) {}
    fsim::MissionEndpoint endpoint;
};
struct fsim_mission_referee : MissionHandle {
    explicit fsim_mission_referee(fsim::MissionReferee r) : MissionHandle{kRefereeKind}, referee(std::move(r)) {}
    fsim::MissionReferee referee;
};

namespace {

/// The handle's kind check: OK, FSIM_INVALID_ARGUMENT for null, FSIM_NOT_ALLOWED for a live handle of another kind.
int kindOf(const void* handle, std::uint32_t expected, const char* call) noexcept {
    if (!handle) return fail(FSIM_INVALID_ARGUMENT, std::string(call) + ": null handle");
    const std::uint32_t kind = static_cast<const MissionHandle*>(handle)->kind;
    if (kind == expected) return FSIM_OK;
    if (kind == kEpisodeKind || kind == kEndpointKind || kind == kRefereeKind)
        return fail(FSIM_NOT_ALLOWED, std::string(call) + ": a mission handle of another kind than this call takes");
    return fail(FSIM_INVALID_ARGUMENT, std::string(call) + ": not a live mission handle");
}

/// Section 5's mapping of the C++ errors.
int mission(const char* call, const std::function<int()>& fn) noexcept {
    try {
        const int r = fn();
        if (r == FSIM_OK) fsim::sdk::lastError().clear();
        return r;
    } catch (const std::invalid_argument& e) {
        return fail(FSIM_INVALID_ARGUMENT, std::string(call) + ": " + e.what());
    } catch (const fsim::mission::Busy& e) {
        return fail(FSIM_ERROR, std::string(call) + ": " + e.what());
    } catch (const std::runtime_error& e) {
        return fail(FSIM_INVALID_ARGUMENT, std::string(call) + ": " + e.what());
    } catch (const std::exception& e) {
        return fail(FSIM_ERROR, std::string(call) + ": " + e.what());
    } catch (...) {
        return fail(FSIM_ERROR, std::string(call) + ": unknown error");
    }
}

/// The two-call read: the size with its terminator; nothing written unless all of it fits.
int copyDocument(const std::string& text, char* buffer, std::size_t capacity, std::size_t* needed, const char* call) {
    *needed = text.size() + 1;
    if (!buffer || capacity == 0) return FSIM_OK;
    if (capacity < *needed) return fail(FSIM_INVALID_ARGUMENT, std::string(call) + ": buffer too small for the document");
    std::memcpy(buffer, text.c_str(), *needed);
    return FSIM_OK;
}

} // namespace

extern "C" {

FSIM_API int fsim_mission_episode_create(const char* config_json, size_t config_bytes, fsim_mission_episode** episode) {
    if (!episode || (!config_json && config_bytes)) return fail(FSIM_INVALID_ARGUMENT, "fsim_mission_episode_create: null argument");
    *episode = nullptr;
    if (!config_json) return fail(FSIM_INVALID_ARGUMENT, "fsim_mission_episode_create: null configuration");
    return mission("fsim_mission_episode_create", [&] {
        *episode = new fsim_mission_episode(fsim::MissionEpisode(std::string_view(config_json, config_bytes)));
        return FSIM_OK;
    });
}

FSIM_API void fsim_mission_episode_destroy(fsim_mission_episode* episode) {
    if (!episode || episode->kind != kEpisodeKind) return;
    episode->kind = kDestroyed;
    delete episode;
}

FSIM_API int fsim_mission_episode_reset(fsim_mission_episode* episode, uint64_t local_episode_index) {
    if (const int k = kindOf(episode, kEpisodeKind, "fsim_mission_episode_reset")) return k;
    return mission("fsim_mission_episode_reset", [&] {
        episode->episode.reset(local_episode_index);
        return FSIM_OK;
    });
}

FSIM_API int fsim_mission_episode_step(fsim_mission_episode* episode) {
    if (const int k = kindOf(episode, kEpisodeKind, "fsim_mission_episode_step")) return k;
    return mission("fsim_mission_episode_step", [&] {
        episode->episode.step();
        return FSIM_OK;
    });
}

FSIM_API int fsim_mission_endpoint_open(fsim_mission_episode* episode, uint64_t service_id, fsim_mission_endpoint** endpoint) {
    if (const int k = kindOf(episode, kEpisodeKind, "fsim_mission_endpoint_open")) return k;
    if (!endpoint) return fail(FSIM_INVALID_ARGUMENT, "fsim_mission_endpoint_open: null output");
    *endpoint = nullptr;
    return mission("fsim_mission_endpoint_open", [&] {
        *endpoint = new fsim_mission_endpoint(episode->episode.endpoint(service_id));
        return FSIM_OK;
    });
}

FSIM_API int fsim_mission_referee_open(fsim_mission_episode* episode, fsim_mission_referee** referee) {
    if (const int k = kindOf(episode, kEpisodeKind, "fsim_mission_referee_open")) return k;
    if (!referee) return fail(FSIM_INVALID_ARGUMENT, "fsim_mission_referee_open: null output");
    *referee = nullptr;
    return mission("fsim_mission_referee_open", [&] {
        *referee = new fsim_mission_referee(episode->episode.referee());
        return FSIM_OK;
    });
}

FSIM_API void fsim_mission_endpoint_destroy(fsim_mission_endpoint* endpoint) {
    if (!endpoint || endpoint->kind != kEndpointKind) return;
    endpoint->kind = kDestroyed;
    delete endpoint;
}

FSIM_API int fsim_mission_endpoint_submit(fsim_mission_endpoint* endpoint, const char* actions_json, size_t actions_bytes) {
    if (const int k = kindOf(endpoint, kEndpointKind, "fsim_mission_endpoint_submit")) return k;
    if (!actions_json) return fail(FSIM_INVALID_ARGUMENT, "fsim_mission_endpoint_submit: null document");
    return mission("fsim_mission_endpoint_submit", [&] {
        endpoint->endpoint.submitJsonV1(std::string_view(actions_json, actions_bytes)); // (kept: _receipts_json reads it)
        return FSIM_OK;
    });
}

FSIM_API int fsim_mission_endpoint_observation_json(fsim_mission_endpoint* endpoint, char* buffer, size_t capacity, size_t* needed) {
    if (const int k = kindOf(endpoint, kEndpointKind, "fsim_mission_endpoint_observation_json")) return k;
    if (!needed) return fail(FSIM_INVALID_ARGUMENT, "fsim_mission_endpoint_observation_json: null size output");
    return mission("fsim_mission_endpoint_observation_json", [&] {
        return copyDocument(endpoint->endpoint.observationJsonV1(), buffer, capacity, needed, "fsim_mission_endpoint_observation_json");
    });
}

FSIM_API int fsim_mission_endpoint_receipts_json(fsim_mission_endpoint* endpoint, char* buffer, size_t capacity, size_t* needed) {
    if (const int k = kindOf(endpoint, kEndpointKind, "fsim_mission_endpoint_receipts_json")) return k;
    if (!needed) return fail(FSIM_INVALID_ARGUMENT, "fsim_mission_endpoint_receipts_json: null size output");
    return mission("fsim_mission_endpoint_receipts_json", [&] {
        return copyDocument(endpoint->endpoint.receiptsJsonV1(), buffer, capacity, needed, "fsim_mission_endpoint_receipts_json");
    });
}

FSIM_API void fsim_mission_referee_destroy(fsim_mission_referee* referee) {
    if (!referee || referee->kind != kRefereeKind) return;
    referee->kind = kDestroyed;
    delete referee;
}

FSIM_API int fsim_mission_referee_outcome_json(fsim_mission_referee* referee, char* buffer, size_t capacity, size_t* needed) {
    if (const int k = kindOf(referee, kRefereeKind, "fsim_mission_referee_outcome_json")) return k;
    if (!needed) return fail(FSIM_INVALID_ARGUMENT, "fsim_mission_referee_outcome_json: null size output");
    return mission("fsim_mission_referee_outcome_json", [&] {
        return copyDocument(referee->referee.outcomeJsonV1(), buffer, capacity, needed, "fsim_mission_referee_outcome_json");
    });
}

FSIM_API int fsim_mission_referee_truth_json(fsim_mission_referee* referee, char* buffer, size_t capacity, size_t* needed) {
    if (const int k = kindOf(referee, kRefereeKind, "fsim_mission_referee_truth_json")) return k;
    if (!needed) return fail(FSIM_INVALID_ARGUMENT, "fsim_mission_referee_truth_json: null size output");
    return mission("fsim_mission_referee_truth_json", [&] {
        return copyDocument(referee->referee.truthJsonV1(), buffer, capacity, needed, "fsim_mission_referee_truth_json");
    });
}

} // extern "C"
