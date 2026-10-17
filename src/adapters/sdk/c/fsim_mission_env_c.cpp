// Mission episodes through the C ABI (fsim/fsim_mission_env.h; docs/sdk/mission-training.md, sections 3 to 5; M5 core).
//
// Three opaque handle kinds, each the public C++ object behind a kind tag: a call given a live handle of another kind
// answers FSIM_NOT_ALLOWED and touches nothing. Status mapping is section 5's: a malformed document, an unconfigured
// service, a handle of an earlier generation or ended episode, an episode not reset, a null pointer or a buffer too
// small - FSIM_INVALID_ARGUMENT; the boundary held by another call - FSIM_ERROR.

#include "fsim/fsim_mission_env.h"

#include "fsim/MissionBatch.h"
#include "fsim/MissionEpisode.h"
#include "CAbiSupport.h"
#include "training/native/mission/MissionGateway.h"

#include <algorithm>
#include <cstring>
#include <new>
#include <string>
#include <utility>
#include <vector>

namespace {

enum Kind : std::uint32_t {
    kEpisodeKind = 0x4D45504Bu,  // "MEPK"
    kEndpointKind = 0x4D454E44u, // "MEND"
    kRefereeKind = 0x4D524546u,  // "MREF"
    kBatchKind = 0x4D424154u,    // "MBAT"
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
struct fsim_mission_batch : MissionHandle {
    explicit fsim_mission_batch(fsim::MissionBatch b) : MissionHandle{kBatchKind}, batch(std::move(b)) {}
    fsim::MissionBatch batch;
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
    if (kind == kEpisodeKind || kind == kEndpointKind || kind == kRefereeKind || kind == kBatchKind)
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


// --- ABI 1.77 (M7 core) -------------------------------------------------------------------------------------------------

extern "C" {

FSIM_API int fsim_mission_episode_remove_platform(fsim_mission_episode* episode, const char* platform, size_t platform_bytes, int* removed) {
    if (const int k = kindOf(episode, kEpisodeKind, "fsim_mission_episode_remove_platform")) return k;
    if (!platform) return fail(FSIM_INVALID_ARGUMENT, "fsim_mission_episode_remove_platform: null platform");
    return mission("fsim_mission_episode_remove_platform", [&] {
        const bool done = episode->episode.removePlatform(std::string_view(platform, platform_bytes));
        if (removed) *removed = done ? 1 : 0;
        return FSIM_OK;
    });
}

FSIM_API int fsim_mission_referee_costs_json(fsim_mission_referee* referee, char* buffer, size_t capacity, size_t* needed) {
    if (const int k = kindOf(referee, kRefereeKind, "fsim_mission_referee_costs_json")) return k;
    if (!needed) return fail(FSIM_INVALID_ARGUMENT, "fsim_mission_referee_costs_json: null size output");
    return mission("fsim_mission_referee_costs_json", [&] {
        return copyDocument(referee->referee.costsJsonV1(), buffer, capacity, needed, "fsim_mission_referee_costs_json");
    });
}

FSIM_API void fsim_mission_batch_options_init(fsim_mission_batch_options* options) {
    if (!options) return;
    std::memset(options, 0, sizeof *options);
    options->struct_size = sizeof *options;
    options->autoreset = FSIM_MISSION_AUTORESET_NEXT_STEP;
    options->workers = 1;
}

FSIM_API int fsim_mission_batch_create(const char* const* configs, const size_t* config_bytes, size_t count,
                                       const fsim_mission_batch_options* options, fsim_mission_batch** batch) {
    if (!batch || (count && (!configs || !config_bytes))) return fail(FSIM_INVALID_ARGUMENT, "fsim_mission_batch_create: null argument");
    *batch = nullptr;
    fsim_mission_batch_options o;
    fsim_mission_batch_options_init(&o);
    if (options) {
        if (options->struct_size < sizeof(uint32_t)) return fail(FSIM_INVALID_ARGUMENT, "fsim_mission_batch_create: options without struct_size");
        std::memcpy(&o, options, std::min<size_t>(options->struct_size, sizeof o));
    }
    for (size_t i = 0; i < count; ++i)
        if (!configs[i]) return fail(FSIM_INVALID_ARGUMENT, "fsim_mission_batch_create: null configuration");
    return mission("fsim_mission_batch_create", [&] {
        std::vector<std::string> texts;
        for (size_t i = 0; i < count; ++i) texts.emplace_back(configs[i], config_bytes[i]);
        fsim::MissionBatchOptions bo;
        if (o.autoreset > FSIM_MISSION_AUTORESET_SAME_STEP) throw std::invalid_argument("an autoreset mode the batch does not have");
        bo.autoreset = static_cast<fsim::MissionAutoreset>(o.autoreset);
        bo.workers = o.workers;
        bo.firstLocalEpisodeIndex = o.first_local_episode_index;
        *batch = new fsim_mission_batch(fsim::MissionBatch(texts, bo));
        return FSIM_OK;
    });
}

FSIM_API void fsim_mission_batch_destroy(fsim_mission_batch* batch) {
    if (!batch || batch->kind != kBatchKind) return;
    batch->kind = kDestroyed;
    delete batch;
}

FSIM_API int fsim_mission_batch_size(fsim_mission_batch* batch, size_t* size) {
    if (const int k = kindOf(batch, kBatchKind, "fsim_mission_batch_size")) return k;
    if (!size) return fail(FSIM_INVALID_ARGUMENT, "fsim_mission_batch_size: null output");
    return mission("fsim_mission_batch_size", [&] {
        *size = batch->batch.size();
        return FSIM_OK;
    });
}

FSIM_API int fsim_mission_batch_reset(fsim_mission_batch* batch) {
    if (const int k = kindOf(batch, kBatchKind, "fsim_mission_batch_reset")) return k;
    return mission("fsim_mission_batch_reset", [&] {
        batch->batch.reset();
        return FSIM_OK;
    });
}

FSIM_API int fsim_mission_batch_reset_slot(fsim_mission_batch* batch, size_t slot) {
    if (const int k = kindOf(batch, kBatchKind, "fsim_mission_batch_reset_slot")) return k;
    return mission("fsim_mission_batch_reset_slot", [&] {
        batch->batch.resetSlot(slot);
        return FSIM_OK;
    });
}

FSIM_API int fsim_mission_batch_step(fsim_mission_batch* batch) {
    if (const int k = kindOf(batch, kBatchKind, "fsim_mission_batch_step")) return k;
    return mission("fsim_mission_batch_step", [&] {
        batch->batch.step();
        return FSIM_OK;
    });
}

FSIM_API int fsim_mission_batch_status(fsim_mission_batch* batch, size_t slot, fsim_mission_slot_status* status) {
    if (const int k = kindOf(batch, kBatchKind, "fsim_mission_batch_status")) return k;
    if (!status || status->struct_size < sizeof(uint32_t)) return fail(FSIM_INVALID_ARGUMENT, "fsim_mission_batch_status: null output or no struct_size");
    return mission("fsim_mission_batch_status", [&] {
        const fsim::MissionSlotStatus st = batch->batch.status(slot);
        fsim_mission_slot_status local;
        std::memset(&local, 0, sizeof local);
        local.struct_size = status->struct_size;
        local.flags = (st.episodeDone ? uint32_t{FSIM_MISSION_SLOT_EPISODE_DONE} : 0u) | (st.ended ? uint32_t{FSIM_MISSION_SLOT_ENDED} : 0u) |
                      (st.reset ? uint32_t{FSIM_MISSION_SLOT_RESET} : 0u);
        local.generation = st.generation, local.local_episode_index = st.localEpisodeIndex, local.tick = st.tick;
        copyOut(local, status);
        return FSIM_OK;
    });
}

FSIM_API int fsim_mission_batch_endpoint_open(fsim_mission_batch* batch, size_t slot, uint64_t service_id, fsim_mission_endpoint** endpoint) {
    if (const int k = kindOf(batch, kBatchKind, "fsim_mission_batch_endpoint_open")) return k;
    if (!endpoint) return fail(FSIM_INVALID_ARGUMENT, "fsim_mission_batch_endpoint_open: null output");
    *endpoint = nullptr;
    return mission("fsim_mission_batch_endpoint_open", [&] {
        *endpoint = new fsim_mission_endpoint(batch->batch.endpoint(slot, service_id));
        return FSIM_OK;
    });
}

FSIM_API int fsim_mission_batch_referee_open(fsim_mission_batch* batch, size_t slot, fsim_mission_referee** referee) {
    if (const int k = kindOf(batch, kBatchKind, "fsim_mission_batch_referee_open")) return k;
    if (!referee) return fail(FSIM_INVALID_ARGUMENT, "fsim_mission_batch_referee_open: null output");
    *referee = nullptr;
    return mission("fsim_mission_batch_referee_open", [&] {
        *referee = new fsim_mission_referee(batch->batch.referee(slot));
        return FSIM_OK;
    });
}

FSIM_API int fsim_mission_batch_final_observation_json(fsim_mission_batch* batch, size_t slot, uint64_t service_id, char* buffer,
                                                       size_t capacity, size_t* needed) {
    if (const int k = kindOf(batch, kBatchKind, "fsim_mission_batch_final_observation_json")) return k;
    if (!needed) return fail(FSIM_INVALID_ARGUMENT, "fsim_mission_batch_final_observation_json: null size output");
    return mission("fsim_mission_batch_final_observation_json", [&] {
        return copyDocument(batch->batch.finalObservationJsonV1(slot, service_id), buffer, capacity, needed,
                            "fsim_mission_batch_final_observation_json");
    });
}

FSIM_API int fsim_mission_batch_final_outcome_json(fsim_mission_batch* batch, size_t slot, char* buffer, size_t capacity, size_t* needed) {
    if (const int k = kindOf(batch, kBatchKind, "fsim_mission_batch_final_outcome_json")) return k;
    if (!needed) return fail(FSIM_INVALID_ARGUMENT, "fsim_mission_batch_final_outcome_json: null size output");
    return mission("fsim_mission_batch_final_outcome_json", [&] {
        return copyDocument(batch->batch.finalOutcomeJsonV1(slot), buffer, capacity, needed, "fsim_mission_batch_final_outcome_json");
    });
}

FSIM_API int fsim_mission_batch_final_truth_json(fsim_mission_batch* batch, size_t slot, char* buffer, size_t capacity, size_t* needed) {
    if (const int k = kindOf(batch, kBatchKind, "fsim_mission_batch_final_truth_json")) return k;
    if (!needed) return fail(FSIM_INVALID_ARGUMENT, "fsim_mission_batch_final_truth_json: null size output");
    return mission("fsim_mission_batch_final_truth_json", [&] {
        return copyDocument(batch->batch.finalTruthJsonV1(slot), buffer, capacity, needed, "fsim_mission_batch_final_truth_json");
    });
}

FSIM_API int fsim_mission_batch_final_costs_json(fsim_mission_batch* batch, size_t slot, char* buffer, size_t capacity, size_t* needed) {
    if (const int k = kindOf(batch, kBatchKind, "fsim_mission_batch_final_costs_json")) return k;
    if (!needed) return fail(FSIM_INVALID_ARGUMENT, "fsim_mission_batch_final_costs_json: null size output");
    return mission("fsim_mission_batch_final_costs_json", [&] {
        return copyDocument(batch->batch.finalCostsJsonV1(slot), buffer, capacity, needed, "fsim_mission_batch_final_costs_json");
    });
}

FSIM_API int fsim_mission_batch_remove_platform(fsim_mission_batch* batch, size_t slot, const char* platform, size_t platform_bytes, int* removed) {
    if (const int k = kindOf(batch, kBatchKind, "fsim_mission_batch_remove_platform")) return k;
    if (!platform) return fail(FSIM_INVALID_ARGUMENT, "fsim_mission_batch_remove_platform: null platform");
    return mission("fsim_mission_batch_remove_platform", [&] {
        const bool done = batch->batch.removePlatform(slot, std::string_view(platform, platform_bytes));
        if (removed) *removed = done ? 1 : 0;
        return FSIM_OK;
    });
}

} // extern "C"
