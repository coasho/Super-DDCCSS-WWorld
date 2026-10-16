/* fsim SDK: mission episodes for MA training through the C ABI (docs/sdk/mission-training.md; M5 core).
 *
 * The same episodes, endpoints and referees as fsim/MissionEpisode.h, as three opaque handle types - none of them an
 * fsim_world. int results are enum fsim_status values of fsim_c.h:
 *   FSIM_INVALID_ARGUMENT  a malformed document, a configuration the contract refuses, a service the configuration does
 *                          not bind, a null handle or output, a buffer too small; an episode not yet reset, a handle of
 *                          an earlier generation or of an ended episode - nothing changed, fsim_last_error() says why
 *   FSIM_ERROR             another call holds the episode's boundary - nothing changed; retry is the caller's
 *   FSIM_NOT_ALLOWED       a live handle of another kind than the call takes
 * A document passed in is a pointer and a byte count, not NUL-terminated. A document read reports in *needed its size
 * including the NUL terminator: with a null buffer or capacity 0 that is a size query (FSIM_OK); with a smaller
 * capacity FSIM_INVALID_ARGUMENT and nothing written; otherwise the whole document. No read consumes anything.
 * _destroy accepts NULL; a destroyed handle must not be used again; children stay safe after their parent's end. */
#ifndef FSIM_MISSION_ENV_H
#define FSIM_MISSION_ENV_H

#include "fsim/fsim_c.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct fsim_mission_episode fsim_mission_episode;
typedef struct fsim_mission_endpoint fsim_mission_endpoint;
typedef struct fsim_mission_referee fsim_mission_referee;

/* The range's. */
FSIM_API int fsim_mission_episode_create(const char* config_json, size_t config_bytes, fsim_mission_episode** episode);
FSIM_API void fsim_mission_episode_destroy(fsim_mission_episode* episode);
FSIM_API int fsim_mission_episode_reset(fsim_mission_episode* episode, uint64_t local_episode_index);
FSIM_API int fsim_mission_episode_step(fsim_mission_episode* episode);
FSIM_API int fsim_mission_endpoint_open(fsim_mission_episode* episode, uint64_t service_id, fsim_mission_endpoint** endpoint);
FSIM_API int fsim_mission_referee_open(fsim_mission_episode* episode, fsim_mission_referee** referee);

/* An actor's: its endpoint alone. */
FSIM_API void fsim_mission_endpoint_destroy(fsim_mission_endpoint* endpoint);
FSIM_API int fsim_mission_endpoint_submit(fsim_mission_endpoint* endpoint, const char* actions_json, size_t actions_bytes);
FSIM_API int fsim_mission_endpoint_observation_json(fsim_mission_endpoint* endpoint, char* buffer, size_t capacity, size_t* needed);
FSIM_API int fsim_mission_endpoint_receipts_json(fsim_mission_endpoint* endpoint, char* buffer, size_t capacity, size_t* needed);

/* The referee's. */
FSIM_API void fsim_mission_referee_destroy(fsim_mission_referee* referee);
FSIM_API int fsim_mission_referee_outcome_json(fsim_mission_referee* referee, char* buffer, size_t capacity, size_t* needed);
FSIM_API int fsim_mission_referee_truth_json(fsim_mission_referee* referee, char* buffer, size_t capacity, size_t* needed);

/* ABI 1.77 (M7 core), additive. The range removes a platform by name (its roles end at the next tick): *removed (when
 * not null) 1, or 0 when it was already gone; FSIM_INVALID_ARGUMENT for a name the configuration does not have. The
 * referee reads its costs (ma.costs.v1). */
FSIM_API int fsim_mission_episode_remove_platform(fsim_mission_episode* episode, const char* platform, size_t platform_bytes, int* removed);
FSIM_API int fsim_mission_referee_costs_json(fsim_mission_referee* referee, char* buffer, size_t capacity, size_t* needed);

/* Batches (fsim/MissionBatch.h; ABI 1.77): one independent episode per slot, configurations given as pointer and byte
 * count each. A fourth handle kind, the range's; its endpoints and referees are the ordinary ones. */
typedef struct fsim_mission_batch fsim_mission_batch;

enum fsim_mission_autoreset { FSIM_MISSION_AUTORESET_DISABLED = 0, FSIM_MISSION_AUTORESET_NEXT_STEP = 1, FSIM_MISSION_AUTORESET_SAME_STEP = 2 };

typedef struct fsim_mission_batch_options {
    uint32_t struct_size;
    uint32_t autoreset; /* enum fsim_mission_autoreset */
    uint32_t workers;
    uint32_t reserved;
    uint64_t first_local_episode_index;
} fsim_mission_batch_options;
FSIM_API void fsim_mission_batch_options_init(fsim_mission_batch_options* options);

enum fsim_mission_slot_flag { FSIM_MISSION_SLOT_EPISODE_DONE = 1, FSIM_MISSION_SLOT_ENDED = 2, FSIM_MISSION_SLOT_RESET = 4 };
typedef struct fsim_mission_slot_status {
    uint32_t struct_size;
    uint32_t flags; /* enum fsim_mission_slot_flag bits */
    uint64_t generation;
    uint64_t local_episode_index;
    uint64_t tick;
} fsim_mission_slot_status;

FSIM_API int fsim_mission_batch_create(const char* const* configs, const size_t* config_bytes, size_t count,
                                       const fsim_mission_batch_options* options, fsim_mission_batch** batch);
FSIM_API void fsim_mission_batch_destroy(fsim_mission_batch* batch);
FSIM_API int fsim_mission_batch_size(fsim_mission_batch* batch, size_t* size);
FSIM_API int fsim_mission_batch_reset(fsim_mission_batch* batch);
FSIM_API int fsim_mission_batch_reset_slot(fsim_mission_batch* batch, size_t slot);
FSIM_API int fsim_mission_batch_step(fsim_mission_batch* batch);
/* status->struct_size set by the caller (sizeof); filled up to it. */
FSIM_API int fsim_mission_batch_status(fsim_mission_batch* batch, size_t slot, fsim_mission_slot_status* status);
FSIM_API int fsim_mission_batch_endpoint_open(fsim_mission_batch* batch, size_t slot, uint64_t service_id, fsim_mission_endpoint** endpoint);
FSIM_API int fsim_mission_batch_referee_open(fsim_mission_batch* batch, size_t slot, fsim_mission_referee** referee);
FSIM_API int fsim_mission_batch_final_observation_json(fsim_mission_batch* batch, size_t slot, uint64_t service_id, char* buffer,
                                                       size_t capacity, size_t* needed);
FSIM_API int fsim_mission_batch_final_outcome_json(fsim_mission_batch* batch, size_t slot, char* buffer, size_t capacity, size_t* needed);
FSIM_API int fsim_mission_batch_final_truth_json(fsim_mission_batch* batch, size_t slot, char* buffer, size_t capacity, size_t* needed);
FSIM_API int fsim_mission_batch_final_costs_json(fsim_mission_batch* batch, size_t slot, char* buffer, size_t capacity, size_t* needed);
FSIM_API int fsim_mission_batch_remove_platform(fsim_mission_batch* batch, size_t slot, const char* platform, size_t platform_bytes, int* removed);

#ifdef __cplusplus
}
#endif

#endif /* FSIM_MISSION_ENV_H */
