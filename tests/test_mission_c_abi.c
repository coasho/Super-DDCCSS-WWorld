/* Mission episodes through the C ABI, compiled as C99 (docs/sdk/mission-training.md, sections 3 to 5; M5 core): three
 * opaque handle kinds that never stand in for one another, two-call document reads that consume nothing, and the
 * contract's status for every failure - through fsim/fsim_mission_env.h alone.
 * Arguments: the contract's fixture directory, then this build's JSBSim data directory. */
#include "fsim/fsim_mission_env.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

#define CHECK(cond)                                                                                  \
    do {                                                                                             \
        if (!(cond)) {                                                                               \
            fprintf(stderr, "%s:%d: check failed: %s (%s)\n", __FILE__, __LINE__, #cond, fsim_last_error()); \
            return 1;                                                                                \
        }                                                                                            \
    } while (0)

static char* read_file(const char* dir, const char* name, size_t* size) {
    char path[4096];
    FILE* f;
    char* data;
    long n;
    snprintf(path, sizeof path, "%s/%s", dir, name);
    f = fopen(path, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    n = ftell(f);
    fseek(f, 0, SEEK_SET);
    data = (char*)malloc((size_t)n + 1);
    if (data && fread(data, 1, (size_t)n, f) != (size_t)n) {
        free(data);
        data = NULL;
    }
    fclose(f);
    if (data) data[n] = '\0';
    if (size) *size = (size_t)n;
    return data;
}

/* The fixture's configuration with this build's JSBSim data in its scenario's world section. */
static char* make_config(const char* fixtures, const char* jsbsim_root) {
    const char* marker = "\"terrain\": false";
    size_t size = 0;
    char* text = read_file(fixtures, "config.two-ship-c2.json", &size);
    char* at;
    char* out;
    size_t head, extra = strlen(jsbsim_root) * 2 + 64, i, j;
    if (!text || !(at = strstr(text, marker))) return NULL;
    head = (size_t)(at - text) + strlen(marker);
    out = (char*)malloc(size + extra);
    if (!out) return NULL;
    memcpy(out, text, head);
    j = head;
    j += (size_t)sprintf(out + j, ", \"jsbsim_root\": \"");
    for (i = 0; jsbsim_root[i]; ++i) {
        if (jsbsim_root[i] == '\\' || jsbsim_root[i] == '"') out[j++] = '\\';
        out[j++] = jsbsim_root[i];
    }
    out[j++] = '"';
    memcpy(out + j, text + head, size - head + 1);
    free(text);
    return out;
}

/* A whole document by the two-call protocol: size, then the read. */
typedef int (*endpoint_read)(fsim_mission_endpoint*, char*, size_t, size_t*);
static char* read_endpoint(endpoint_read read, fsim_mission_endpoint* endpoint) {
    size_t needed = 0;
    char* buffer;
    if (read(endpoint, NULL, 0, &needed) != FSIM_OK || needed == 0) return NULL;
    buffer = (char*)malloc(needed);
    if (!buffer || read(endpoint, buffer, needed, &needed) != FSIM_OK) {
        free(buffer);
        return NULL;
    }
    return buffer;
}

/* The episode key a reset gave, from its first observation: the text of its "episode" member. */
static int episode_key(const char* observation, char* out, size_t capacity) {
    const char* start = strstr(observation, "\"episode\":{");
    const char* end;
    if (!start) return 0;
    start += strlen("\"episode\":");
    end = strchr(start, '}');
    if (!end || (size_t)(end - start + 2) > capacity) return 0;
    memcpy(out, start, (size_t)(end - start + 1));
    out[end - start + 1] = '\0';
    return 1;
}

#ifdef _WIN32
/* The configuration with `extra` light aircraft more, so that a reset lasts long enough to be met by another thread. */
static char* loaded_config(const char* config, int extra) {
    const char* marker = "\"vehicles\": [";
    const char* at = strstr(config, marker);
    const char* capacity = strstr(config, "\"capacity\": 8");
    size_t head, size = strlen(config) + (size_t)extra * 200 + 64, j;
    char* out;
    int i;
    if (!at || !capacity || capacity > at) return NULL;
    out = (char*)malloc(size);
    if (!out) return NULL;
    j = (size_t)(capacity - config);
    memcpy(out, config, j);
    j += (size_t)sprintf(out + j, "\"capacity\": %d", extra + 8);
    head = (size_t)(at - config) + strlen(marker);
    memcpy(out + j, capacity + strlen("\"capacity\": 8"), head - (size_t)(capacity - config) - strlen("\"capacity\": 8"));
    j += head - (size_t)(capacity - config) - strlen("\"capacity\": 8");
    for (i = 0; i < extra; ++i)
        j += (size_t)sprintf(out + j, "{\"name\": \"load-%d\", \"type\": \"jsbsim:c172x\", \"initial\": {\"lat_deg\": 36.1, "
                                      "\"lon_deg\": -115.1, \"alt_msl_m\": 3000, \"heading_deg\": 0, \"airspeed_ms\": 50}},", i);
    strcpy(out + j, config + head);
    return out;
}

typedef struct {
    fsim_mission_episode* episode;
    volatile LONG done;
    int status;
} reset_job;

static DWORD WINAPI run_reset(LPVOID p) {
    reset_job* job = (reset_job*)p;
    job->status = fsim_mission_episode_reset(job->episode, 11);
    InterlockedExchange(&job->done, 1);
    return 0;
}

/* A reset refused as busy (FSIM_ERROR) changes nothing of the reset that holds the episode. */
static int busy_reset(const char* config) {
    char* loaded = loaded_config(config, 36);
    fsim_mission_episode* episode = NULL;
    fsim_mission_endpoint* earlier = NULL;
    fsim_mission_endpoint* current = NULL;
    reset_job job;
    HANDLE thread;
    size_t needed = 0;
    int busy = 0, second = FSIM_OK;
    DWORD start;
    char* observation;
    CHECK(loaded != NULL);
    CHECK(fsim_mission_episode_create(loaded, strlen(loaded), &episode) == FSIM_OK);
    CHECK(fsim_mission_episode_reset(episode, 0) == FSIM_OK);
    CHECK(fsim_mission_endpoint_open(episode, 9007199254740995ull, &earlier) == FSIM_OK);
    job.episode = episode, job.done = 0, job.status = -100;
    thread = CreateThread(NULL, 0, run_reset, &job, 0, NULL);
    CHECK(thread != NULL);
    start = GetTickCount();
    while (!job.done && !busy && GetTickCount() - start < 30000) {
        busy = fsim_mission_endpoint_observation_json(earlier, NULL, 0, &needed) == FSIM_ERROR;
        if (!busy) Sleep(1);
    }
    if (busy) second = fsim_mission_episode_reset(episode, 22);
    WaitForSingleObject(thread, INFINITE);
    CloseHandle(thread);
    CHECK(busy);                   /* (the first reset was met while it held the episode) */
    CHECK(second == FSIM_ERROR);   /* busy: refused at once, nothing changed */
    CHECK(job.status == FSIM_OK);
    CHECK(fsim_mission_endpoint_open(episode, 9007199254740995ull, &current) == FSIM_OK);
    observation = read_endpoint(fsim_mission_endpoint_observation_json, current);
    CHECK(observation != NULL);
    CHECK(strstr(observation, "\"episode_generation\":\"2\",\"local_episode_index\":\"11\"") != NULL);
    free(observation);
    fsim_mission_endpoint_destroy(current);
    fsim_mission_endpoint_destroy(earlier);
    fsim_mission_episode_destroy(episode);
    free(loaded);
    return 0;
}
#endif

/* A batch (ABI 1.77): two slots of the same configuration under other env_ids, SameStep autoreset after two ticks. */
static char* with_env(const char* config, const char* env_id, int max_ticks) {
    const char* env = "\"env_id\": \"9007199254740993\"";
    const char* ticks = "\"max_ticks\": 600";
    const char* at_env = strstr(config, env);
    const char* at_ticks = strstr(config, ticks);
    size_t j = 0;
    char* out = (char*)malloc(strlen(config) + 64);
    if (!out || !at_env || !at_ticks || at_ticks < at_env) {
        free(out);
        return NULL;
    }
    memcpy(out, config, (size_t)(at_env - config));
    j = (size_t)(at_env - config);
    j += (size_t)sprintf(out + j, "\"env_id\": \"%s\"", env_id);
    memcpy(out + j, at_env + strlen(env), (size_t)(at_ticks - at_env) - strlen(env));
    j += (size_t)(at_ticks - at_env) - strlen(env);
    j += (size_t)sprintf(out + j, "\"max_ticks\": %d", max_ticks);
    strcpy(out + j, at_ticks + strlen(ticks));
    return out;
}

static int batch_cases(const char* config) {
    char* a = with_env(config, "101", 2);
    char* b = with_env(config, "102", 600);
    const char* configs[2];
    size_t sizes[2], size = 0, needed = 0, first = 0;
    int removed;
    fsim_mission_batch_options options;
    fsim_mission_slot_status status;
    fsim_mission_batch* batch = NULL;
    fsim_mission_endpoint* wing = NULL;
    fsim_mission_referee* referee = NULL;
    char* text;
    CHECK(a && b);
    configs[0] = a, configs[1] = b, sizes[0] = strlen(a), sizes[1] = strlen(b);
    fsim_mission_batch_options_init(&options);
    CHECK(options.struct_size == sizeof options && options.autoreset == FSIM_MISSION_AUTORESET_NEXT_STEP && options.workers == 1);
    options.autoreset = FSIM_MISSION_AUTORESET_SAME_STEP;
    options.workers = 2;
    CHECK(fsim_mission_batch_create(configs, sizes, 2, NULL, &batch) == FSIM_OK); /* (null options: the defaults) */
    fsim_mission_batch_destroy(batch);
    batch = NULL;
    CHECK(fsim_mission_batch_create(configs, sizes, 0, &options, &batch) == FSIM_INVALID_ARGUMENT);
    configs[1] = a, sizes[1] = sizes[0];
    CHECK(fsim_mission_batch_create(configs, sizes, 2, &options, &batch) == FSIM_INVALID_ARGUMENT); /* (an env_id twice) */
    configs[1] = b, sizes[1] = strlen(b);
    CHECK(fsim_mission_batch_create(configs, sizes, 2, &options, &batch) == FSIM_OK);
    CHECK(fsim_mission_batch_size(batch, &size) == FSIM_OK && size == 2);
    CHECK(fsim_mission_batch_step(batch) == FSIM_INVALID_ARGUMENT); /* (not reset) */
    CHECK(fsim_mission_batch_reset(batch) == FSIM_OK);
    status.struct_size = sizeof status;
    CHECK(fsim_mission_batch_status(batch, 0, &status) == FSIM_OK);
    CHECK(status.generation == 1 && status.tick == 0 && (status.flags & FSIM_MISSION_SLOT_RESET));
    CHECK(fsim_mission_batch_status(batch, 2, &status) == FSIM_INVALID_ARGUMENT);
    CHECK(fsim_mission_batch_final_outcome_json(batch, 0, NULL, 0, &first) == FSIM_OK && first == 1); /* (none yet: "") */
    /* handle kinds: a batch is none of the other three, nor they a batch */
    CHECK(fsim_mission_batch_endpoint_open(batch, 0, 9007199254740995ull, &wing) == FSIM_OK);
    CHECK(fsim_mission_batch_step((fsim_mission_batch*)wing) == FSIM_NOT_ALLOWED);
    CHECK(fsim_mission_episode_step((fsim_mission_episode*)batch) == FSIM_NOT_ALLOWED);
    CHECK(fsim_mission_batch_step(batch) == FSIM_OK);
    CHECK(fsim_mission_batch_step(batch) == FSIM_OK);
    CHECK(fsim_mission_batch_status(batch, 0, &status) == FSIM_OK);
    CHECK((status.flags & FSIM_MISSION_SLOT_ENDED) && (status.flags & FSIM_MISSION_SLOT_RESET) && status.generation == 2 && status.tick == 0);
    CHECK(fsim_mission_batch_status(batch, 1, &status) == FSIM_OK && !(status.flags & FSIM_MISSION_SLOT_ENDED) && status.tick == 2);
    CHECK(fsim_mission_endpoint_observation_json(wing, NULL, 0, &needed) == FSIM_INVALID_ARGUMENT); /* (its generation ended) */
    /* the ended generation's final documents, by the two-call protocol */
    CHECK(fsim_mission_batch_final_observation_json(batch, 0, 9007199254740995ull, NULL, 0, &needed) == FSIM_OK && needed > 1);
    text = (char*)malloc(needed);
    CHECK(text && fsim_mission_batch_final_observation_json(batch, 0, 9007199254740995ull, text, needed, &needed) == FSIM_OK);
    CHECK(strstr(text, "\"tick\":\"2\"") && strstr(text, "\"episode_generation\":\"1\""));
    free(text);
    CHECK(fsim_mission_batch_final_outcome_json(batch, 0, NULL, 0, &needed) == FSIM_OK && needed > 1);
    CHECK(fsim_mission_batch_final_costs_json(batch, 0, NULL, 0, &needed) == FSIM_OK && needed > 1);
    CHECK(fsim_mission_batch_final_truth_json(batch, 0, NULL, 0, &needed) == FSIM_OK && needed > 1);
    CHECK(fsim_mission_batch_referee_open(batch, 0, &referee) == FSIM_OK);
    CHECK(fsim_mission_referee_costs_json(referee, NULL, 0, &needed) == FSIM_OK && needed > 1);
    removed = -1;
    CHECK(fsim_mission_batch_remove_platform(batch, 1, "wing", 4, &removed) == FSIM_OK && removed == 1);
    CHECK(fsim_mission_batch_remove_platform(batch, 1, "wing", 4, &removed) == FSIM_OK && removed == 0);
    CHECK(fsim_mission_batch_remove_platform(batch, 1, "nobody", 6, NULL) == FSIM_INVALID_ARGUMENT);
    fsim_mission_endpoint_destroy(wing);
    fsim_mission_referee_destroy(referee);
    fsim_mission_batch_destroy(batch);
    fsim_mission_batch_destroy(NULL);
    free(a);
    free(b);
    return 0;
}

int main(int argc, char** argv) {
    fsim_mission_episode* episode = NULL;
    fsim_mission_endpoint* wing = NULL;
    fsim_mission_endpoint* stale = NULL;
    fsim_mission_referee* referee = NULL;
    char* config;
    char* observation;
    char* again;
    char* receipts;
    char key[256], document[1024];
    char small[8];
    size_t needed = 0, first = 0;
    const uint64_t kWing = 9007199254740995ull;
    if (argc < 3) {
        fprintf(stderr, "usage: %s <contract fixtures> <jsbsim root>\n", argv[0]);
        return 2;
    }
    config = make_config(argv[1], argv[2]);
    CHECK(config != NULL);

    /* a malformed configuration: nothing made */
    CHECK(fsim_mission_episode_create("{}", 2, &episode) == FSIM_INVALID_ARGUMENT);
    CHECK(episode == NULL);
    CHECK(fsim_mission_episode_create(NULL, 0, &episode) == FSIM_INVALID_ARGUMENT);
    CHECK(fsim_mission_episode_create(config, strlen(config), NULL) == FSIM_INVALID_ARGUMENT);
    CHECK(fsim_mission_episode_create(config, strlen(config), &episode) == FSIM_OK);
    CHECK(episode != NULL);

    /* before the first reset */
    CHECK(fsim_mission_episode_step(episode) == FSIM_INVALID_ARGUMENT);
    CHECK(fsim_mission_endpoint_open(episode, kWing, &wing) == FSIM_INVALID_ARGUMENT);
    CHECK(wing == NULL);
    CHECK(fsim_mission_referee_open(episode, &referee) == FSIM_INVALID_ARGUMENT);

    CHECK(fsim_mission_episode_reset(episode, 41) == FSIM_OK);
    CHECK(fsim_mission_endpoint_open(episode, 3, &wing) == FSIM_INVALID_ARGUMENT); /* not configured */
    CHECK(fsim_mission_endpoint_open(episode, kWing, &wing) == FSIM_OK);
    CHECK(fsim_mission_referee_open(episode, &referee) == FSIM_OK);

    /* two-call reads: a size query, a too-small buffer writes nothing, the whole document, again identical */
    CHECK(fsim_mission_endpoint_observation_json(wing, NULL, 0, &first) == FSIM_OK);
    CHECK(first > 1);
    memset(small, 'x', sizeof small);
    CHECK(fsim_mission_endpoint_observation_json(wing, small, sizeof small, &needed) == FSIM_INVALID_ARGUMENT);
    CHECK(needed == first);
    CHECK(small[0] == 'x');
    CHECK(fsim_mission_endpoint_observation_json(wing, small, 0, &needed) == FSIM_OK);
    CHECK(fsim_mission_endpoint_observation_json(wing, NULL, 0, NULL) == FSIM_INVALID_ARGUMENT);
    observation = read_endpoint(fsim_mission_endpoint_observation_json, wing);
    again = read_endpoint(fsim_mission_endpoint_observation_json, wing);
    CHECK(observation && again);
    CHECK(strlen(observation) + 1 == first);
    CHECK(strcmp(observation, again) == 0);
    CHECK(strstr(observation, "\"schema\":\"ma.observation.v1\"") != NULL);
    CHECK(strstr(observation, "\"episode_generation\":\"1\"") != NULL);
    CHECK(episode_key(observation, key, sizeof key));

    /* a live handle of another kind: refused, nothing done */
    CHECK(fsim_mission_episode_step((fsim_mission_episode*)wing) == FSIM_NOT_ALLOWED);
    CHECK(fsim_mission_episode_reset((fsim_mission_episode*)referee, 1) == FSIM_NOT_ALLOWED);
    CHECK(fsim_mission_endpoint_open((fsim_mission_episode*)wing, kWing, &stale) == FSIM_NOT_ALLOWED);
    CHECK(fsim_mission_endpoint_observation_json((fsim_mission_endpoint*)referee, NULL, 0, &needed) == FSIM_NOT_ALLOWED);
    CHECK(fsim_mission_endpoint_observation_json((fsim_mission_endpoint*)episode, NULL, 0, &needed) == FSIM_NOT_ALLOWED);
    CHECK(fsim_mission_endpoint_submit((fsim_mission_endpoint*)episode, "{}", 2) == FSIM_NOT_ALLOWED);
    CHECK(fsim_mission_referee_outcome_json((fsim_mission_referee*)wing, NULL, 0, &needed) == FSIM_NOT_ALLOWED);
    CHECK(fsim_mission_referee_truth_json((fsim_mission_referee*)episode, NULL, 0, &needed) == FSIM_NOT_ALLOWED);
    CHECK(fsim_mission_endpoint_observation_json(NULL, NULL, 0, &needed) == FSIM_INVALID_ARGUMENT);

    /* a submission: a malformed document is an error, a well-formed one for another tick an answer */
    CHECK(fsim_mission_endpoint_submit(wing, "{\"schema\":\"ma.actions.v1\"", 25) == FSIM_INVALID_ARGUMENT);
    CHECK(fsim_mission_endpoint_submit(wing, NULL, 0) == FSIM_INVALID_ARGUMENT);
    snprintf(document, sizeof document,
             "{\"schema\":\"ma.actions.v1\",\"episode\":%s,\"service_id\":\"9007199254740995\",\"tick\":\"5\",\"requests\":[]}", key);
    CHECK(fsim_mission_endpoint_submit(wing, document, strlen(document)) == FSIM_OK);
    receipts = read_endpoint(fsim_mission_endpoint_receipts_json, wing);
    CHECK(receipts != NULL);
    CHECK(strstr(receipts, "\"reason\":\"future_tick\"") != NULL);
    free(receipts);
    snprintf(document, sizeof document,
             "{\"schema\":\"ma.actions.v1\",\"episode\":%s,\"service_id\":\"9007199254740995\",\"tick\":\"0\",\"requests\":["
             "{\"request_id\":\"1\",\"kind\":\"flight\",\"operation\":\"new\",\"parameters\":{\"mode\":\"hsa\",\"heading_rad\":0.5}}]}",
             key);
    CHECK(fsim_mission_endpoint_submit(wing, document, strlen(document)) == FSIM_OK);
    receipts = read_endpoint(fsim_mission_endpoint_receipts_json, wing);
    CHECK(receipts != NULL);
    CHECK(strstr(receipts, "\"status\":\"queued\"") != NULL);
    free(receipts);

    CHECK(fsim_mission_episode_step(episode) == FSIM_OK);
    free(observation);
    observation = read_endpoint(fsim_mission_endpoint_observation_json, wing);
    CHECK(observation != NULL);
    CHECK(strstr(observation, "\"tick\":\"1\"") != NULL);
    CHECK(strstr(observation, "\"state\":\"executing\"") != NULL);
    CHECK(fsim_mission_referee_outcome_json(referee, NULL, 0, &needed) == FSIM_OK);
    CHECK(fsim_mission_referee_truth_json(referee, NULL, 0, &needed) == FSIM_OK);

    /* a reset makes every earlier handle stale */
    stale = wing;
    wing = NULL;
    CHECK(fsim_mission_episode_reset(episode, 41) == FSIM_OK);
    CHECK(fsim_mission_endpoint_observation_json(stale, NULL, 0, &needed) == FSIM_INVALID_ARGUMENT);
    CHECK(fsim_mission_endpoint_submit(stale, document, strlen(document)) == FSIM_INVALID_ARGUMENT);
    CHECK(fsim_mission_referee_outcome_json(referee, NULL, 0, &needed) == FSIM_INVALID_ARGUMENT);
    CHECK(fsim_mission_endpoint_open(episode, kWing, &wing) == FSIM_OK);
    /* the earlier generation's document: answered, nothing queued */
    CHECK(fsim_mission_endpoint_submit(wing, document, strlen(document)) == FSIM_OK);
    receipts = read_endpoint(fsim_mission_endpoint_receipts_json, wing);
    CHECK(receipts != NULL);
    CHECK(strstr(receipts, "\"reason\":\"stale_generation\"") != NULL);
    free(receipts);

    /* the parent's end: its children stay safe and answer no more */
    fsim_mission_episode_destroy(episode);
    CHECK(fsim_mission_endpoint_observation_json(wing, NULL, 0, &needed) == FSIM_INVALID_ARGUMENT);
    fsim_mission_endpoint_destroy(wing);
    fsim_mission_endpoint_destroy(stale);
    fsim_mission_referee_destroy(referee);
    fsim_mission_endpoint_destroy(NULL);
    fsim_mission_referee_destroy(NULL);
    fsim_mission_episode_destroy(NULL);
    free(observation);
    free(again);
#ifdef _WIN32
    if (busy_reset(config) != 0) return 1;
#endif
    if (batch_cases(config) != 0) return 1;
    free(config);
    printf("mission C ABI: ok\n");
    return 0;
}
