/* Mission episodes through the C ABI, compiled as C99 (docs/sdk/mission-training.md, sections 3 to 5; M5 core): three
 * opaque handle kinds that never stand in for one another, two-call document reads that consume nothing, and the
 * contract's status for every failure - through fsim/fsim_mission_env.h alone.
 * Arguments: the contract's fixture directory, then this build's JSBSim data directory. */
#include "fsim/fsim_mission_env.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
    free(config);
    printf("mission C ABI: ok\n");
    return 0;
}
