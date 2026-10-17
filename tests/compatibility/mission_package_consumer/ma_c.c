/* MA-service binding, legal reports and mission episodes from an installed SDK through its C ABI, as C99: fsim_c.h,
 * fsim_mission_env.h and the library, nothing else (docs/sdk/mission-training.md, sections 3 to 14). */
#include <fsim/fsim_c.h>
#include <fsim/fsim_mission_env.h>

#include <stdlib.h>

#include <math.h>
#include <stdio.h>
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

static const char* const kHsa = "fsim.guidance.hsa";
static const char* const kRoute = "fsim.guidance.route";

#ifdef _WIN32
/* Whether two paths are one file. */
static int same_file(const char* a, const char* b) {
    BY_HANDLE_FILE_INFORMATION ia, ib;
    const HANDLE ha = CreateFileA(a, 0, FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    const HANDLE hb = CreateFileA(b, 0, FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    int same = 0;
    if (ha != INVALID_HANDLE_VALUE && hb != INVALID_HANDLE_VALUE && GetFileInformationByHandle(ha, &ia) && GetFileInformationByHandle(hb, &ib))
        same = ia.dwVolumeSerialNumber == ib.dwVolumeSerialNumber && ia.nFileIndexHigh == ib.nFileIndexHigh && ia.nFileIndexLow == ib.nFileIndexLow;
    if (ha != INVALID_HANDLE_VALUE) CloseHandle(ha);
    if (hb != INVALID_HANDLE_VALUE) CloseHandle(hb);
    return same;
}
#endif

/* The library this process loaded is the installed prefix's. */
static int installed_library(const char* prefix) {
#ifdef _WIN32
    char loaded[4096], expected[4096];
    const HMODULE module = GetModuleHandleA("libfsim.dll");
    DWORD size;
    CHECK(module != NULL);
    size = GetModuleFileNameA(module, loaded, sizeof loaded);
    CHECK(size != 0 && size < sizeof loaded);
    CHECK(strlen(prefix) + 32 < sizeof expected);
    sprintf(expected, "%s/bin/libfsim.dll", prefix);
    CHECK(same_file(loaded, expected));
    printf("installed DLL: %s\n", loaded);
    return 0;
#else
    (void)prefix;
    fprintf(stderr, "this package gate requires the Windows UCRT64 runtime\n");
    return 1;
#endif
}

static fsim_world* make_world(const char* name, const char* jsbsim_root) {
    fsim_world_options wo;
    fsim_world* world = NULL;
    fsim_world_options_init(&wo);
    wo.name = name;
    wo.publish = 0;
    wo.workers = 1;
    wo.pin_workers = 0;
    wo.jsbsim_root = jsbsim_root;
    return fsim_world_create(&wo, &world) == FSIM_OK ? world : NULL;
}

static uint32_t make_aircraft(fsim_world* world, const char* name, double north_deg) {
    fsim_vehicle_spec spec;
    uint32_t id = 0;
    fsim_vehicle_spec_init(&spec);
    spec.name = name;
    spec.type = "jsbsim:f16c";
    spec.latitude_deg = 40.0 + north_deg;
    spec.longitude_deg = 0.0;
    spec.altitude_msl_m = 6000.0;
    spec.airspeed_ms = 230.0;
    return fsim_world_create_vehicle(world, &spec, &id) == FSIM_OK ? id : 0;
}

static uint32_t controller_of(const fsim_world* agent) {
    uint32_t vehicle = 0, controller = 0;
    const char* name = NULL;
    return fsim_agent_identity(agent, &vehicle, &controller, &name) == 1 ? controller : 0;
}

static int bind(fsim_world* world, uint64_t service, uint32_t platform, const fsim_world* agent, const char* const* capabilities,
                uint32_t count, int ingress, int egress) {
    fsim_ma_service_binding b;
    fsim_ma_service_binding_init(&b);
    b.service_id = service;
    b.platform = platform;
    b.controller = controller_of(agent);
    b.command_capabilities = capabilities;
    b.command_capability_count = count;
    b.owns_dms_ingress = ingress;
    b.owns_dms_egress = egress;
    return fsim_world_bind_ma_service(world, &b);
}

static int fly(fsim_world* agent, uint32_t id, fsim_command_result* cr) {
    double hsa[6];
    fsim_command_options co;
    fsim_command_options_init(&co);
    hsa[0] = 1.0, hsa[1] = fsim_hold(), hsa[2] = 230.0, hsa[3] = FSIM_SPEED_TRUE_AIRSPEED, hsa[4] = 6000.0, hsa[5] = FSIM_ALTITUDE_MSL;
    memset(cr, 0, sizeof *cr);
    return fsim_vehicle_submit_mode(agent, id, FSIM_MODE_HSA, hsa, 6, &co, cr);
}

/* Who owns what with two services on one platform, the agents made in one order or the other: owners[] the services
 * of HSA, the route, the DMS ingress and the DMS egress. */
static int ownership(const char* jsbsim_root, int flight_first, uint64_t owners[4]) {
    fsim_world* world = make_world(flight_first ? "installed-c-order-a" : "installed-c-order-b", jsbsim_root);
    fsim_world* first = NULL;
    fsim_world* second = NULL;
    fsim_world* other = NULL;
    fsim_world* flight;
    fsim_world* comms;
    fsim_ma_service_binding read;
    fsim_command_result cr;
    const char* guidance[2];
    const char* hsa[1];
    const char* all[1];
    fsim_ma_service_binding by_handle;
    const unsigned char raw[1] = {0x01};
    uint32_t viper, wing, i, dest = 1, holder = 0;
    uint64_t msg = 0;
    int32_t reason = -1, granted = 0, mode = -1;
    guidance[0] = kHsa, guidance[1] = kRoute, hsa[0] = kHsa, all[0] = "*";
    CHECK(world != NULL);
    viper = make_aircraft(world, "viper", 0.0);
    wing = make_aircraft(world, "wing", 0.02);
    CHECK(viper != 0 && wing != 0);
    CHECK(fsim_agent_create(world, viper, flight_first ? "flight" : "comms", &first) == FSIM_OK &&
          fsim_agent_create(world, viper, flight_first ? "comms" : "flight", &second) == FSIM_OK &&
          fsim_agent_create(world, wing, "other", &other) == FSIM_OK);
    flight = flight_first ? first : second;
    comms = flight_first ? second : first;
    /* before anything is bound, the other service holds the grant and flies: the binding, not that, decides */
    CHECK(fsim_vehicle_request_control_by(comms, viper, kHsa, 0, &reason) == FSIM_OK && reason == 0);
    CHECK(fly(comms, viper, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
    /* (the comms service by its agent's handle, the flight service by its numbers) */
    fsim_ma_service_binding_init(&by_handle);
    by_handle.service_id = 2, by_handle.owns_dms_ingress = 1, by_handle.owns_dms_egress = 1;
    CHECK(fsim_world_bind_ma_service_agent(world, world, &by_handle) == FSIM_INVALID_ARGUMENT); /* (not an agent's handle) */
    if (flight_first) {
        CHECK(bind(world, 1, viper, flight, guidance, 2, 0, 0) == FSIM_OK && fsim_world_bind_ma_service_agent(world, comms, &by_handle) == FSIM_OK);
    } else {
        CHECK(fsim_world_bind_ma_service_agent(world, comms, &by_handle) == FSIM_OK && bind(world, 1, viper, flight, guidance, 2, 0, 0) == FSIM_OK);
    }
    /* what stood before the bindings gave way to them */
    CHECK(fsim_vehicle_control_holder(world, viper, kHsa, &granted, &holder) == FSIM_OK && granted == 1 && holder == controller_of(flight));
    CHECK(fsim_vehicle_request_control_by(comms, viper, kHsa, 0, &reason) == FSIM_OK && strcmp(fsim_reason_name(reason), "not_allowed") == 0);
    CHECK(fsim_vehicle_request_control(world, viper, kHsa, &reason) == FSIM_OK && strcmp(fsim_reason_name(reason), "not_allowed") == 0);
    CHECK(fsim_vehicle_set_control_mode(world, viper, FSIM_CONTROL_OPEN) == FSIM_NOT_ALLOWED);
    CHECK(fsim_vehicle_control_mode(world, viper, &mode) == FSIM_OK && mode == FSIM_CONTROL_GRANTED);
    /* an exclusive ownership twice, another platform's controller: refused, nothing changed */
    CHECK(bind(world, 3, viper, comms, hsa, 1, 0, 0) == FSIM_INVALID_ARGUMENT && strlen(fsim_last_error()) > 0);
    CHECK(bind(world, 4, viper, other, all, 1, 0, 0) == FSIM_INVALID_ARGUMENT && strlen(fsim_last_error()) > 0);
    CHECK(fsim_world_ma_service_count(world) == 2);
    /* its capability its owner's to command, and no one else's */
    CHECK(fly(flight, viper, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
    CHECK(fly(comms, viper, &cr) != FSIM_OK || cr.status != FSIM_COMMAND_ACCEPTED);
    CHECK(fsim_dms_send(flight, viper, 1001, raw, sizeof raw, &dest, 1, &msg, &reason) == FSIM_NOT_ALLOWED && reason == FSIM_DMS_NOT_OWNER);
    owners[0] = owners[1] = owners[2] = owners[3] = 0;
    for (i = 0; i < fsim_world_ma_service_count(world); ++i) {
        fsim_ma_service_binding_init(&read);
        CHECK(fsim_world_ma_service(world, i, &read) == FSIM_OK);
        if (read.controller == fsim_world_ma_capability_owner(world, viper, kHsa)) owners[0] = read.service_id;
        if (read.controller == fsim_world_ma_capability_owner(world, viper, kRoute)) owners[1] = read.service_id;
        if (read.owns_dms_ingress) owners[2] = read.service_id;
        if (read.owns_dms_egress) owners[3] = read.service_id;
    }
    fsim_world_destroy(other);
    fsim_world_destroy(second);
    fsim_world_destroy(first);
    fsim_world_destroy(world);
    return 0;
}

/* Two worlds mint the same numbers: an agent's handle binds only in the world that made it. */
static int worlds(const char* jsbsim_root) {
    fsim_world* one = make_world("installed-c-world-one", jsbsim_root);
    fsim_world* two = make_world("installed-c-world-two", jsbsim_root);
    fsim_world* mine = NULL;
    fsim_world* theirs = NULL;
    fsim_ma_service_binding b;
    const char* hsa[1];
    uint32_t a1, a2;
    int32_t mode = -1;
    hsa[0] = kHsa;
    CHECK(one != NULL && two != NULL);
    a1 = make_aircraft(one, "viper", 0.0);
    a2 = make_aircraft(two, "viper", 0.0);
    CHECK(a1 != 0 && fsim_agent_create(one, a1, "mine", &mine) == FSIM_OK && fsim_agent_create(two, a2, "theirs", &theirs) == FSIM_OK);
    CHECK(a1 == a2 && controller_of(mine) == controller_of(theirs));
    fsim_ma_service_binding_init(&b);
    b.service_id = 1, b.command_capabilities = hsa, b.command_capability_count = 1, b.owns_dms_ingress = 1, b.owns_dms_egress = 1;
    CHECK(fsim_world_bind_ma_service_agent(one, theirs, &b) == FSIM_INVALID_ARGUMENT && strlen(fsim_last_error()) > 0);
    CHECK(fsim_world_ma_service_count(one) == 0 && fsim_vehicle_control_mode(one, a1, &mode) == FSIM_OK && mode == FSIM_CONTROL_OPEN);
    CHECK(fsim_world_bind_ma_service_agent(one, mine, &b) == FSIM_OK && fsim_world_ma_service_count(one) == 1 &&
          fsim_world_ma_capability_owner(one, a1, kHsa) == controller_of(mine));
    fsim_world_destroy(theirs);
    fsim_world_destroy(mine);
    fsim_world_destroy(two);
    fsim_world_destroy(one);
    return 0;
}

static int reports(const char* jsbsim_root) {
    fsim_world* world = make_world("installed-c-reports", jsbsim_root);
    fsim_world* blue = NULL;
    fsim_world* boss = NULL;
    fsim_net_spec ns;
    fsim_c2_node_spec cs;
    fsim_ownship own;
    fsim_c2_task order, held;
    fsim_report_meta meta;
    uint32_t viper, crc = 0, net = 0, count = 99, mine, c2;
    uint64_t task = 0;
    int32_t availability = -1, reason = -1;
    double sent;
    int kind;
    CHECK(world != NULL);
    viper = make_aircraft(world, "viper", 0.0);
    fsim_c2_node_spec_init(&cs);
    cs.name = "crc", cs.latitude_rad = 39.64 * 0.017453292519943295, cs.longitude_rad = 0.0, cs.altitude_msl_m = 200.0;
    fsim_net_spec_init(&ns);
    CHECK(viper != 0 && fsim_world_create_c2_node(world, &cs, &crc) == FSIM_OK && fsim_world_create_net(world, &ns, &net) == FSIM_OK);
    CHECK(fsim_world_join_net(world, viper, net) == FSIM_OK && fsim_world_join_net(world, crc, net) == FSIM_OK);
    CHECK(fsim_agent_create(world, viper, "blue-1", &blue) == FSIM_OK && fsim_agent_create(world, crc, "crc", &boss) == FSIM_OK);
    mine = fsim_dms_system(blue, viper);
    c2 = fsim_dms_system(boss, crc);
    CHECK(mine != 0 && c2 != 0 && fsim_c2_declare_role(boss, crc, 2, "CRC LEAD", NULL, 0, &reason) == FSIM_OK); /* (it may task) */
    CHECK(fsim_world_step(world, 60) == FSIM_OK);

    fsim_ownship_init(&own);
    fsim_report_meta_init(&meta);
    CHECK(fsim_vehicle_ownship(blue, viper, &own) == FSIM_OK);
    CHECK(fsim_agent_report(blue, viper, FSIM_REPORT_NAVIGATION, NAN, &availability, &meta, 1, &count) == FSIM_OK);
    CHECK(availability == FSIM_REPORT_AVAILABLE && count == 1 && meta.source == FSIM_REPORT_OWN_NAVIGATION && meta.validity == FSIM_REPORT_VALID);
    CHECK(meta.sample_time_s == own.time_s && meta.receive_time_s == fsim_world_time(world) && meta.payload == 0xFFFF);
    CHECK(fsim_agent_report(blue, viper, FSIM_REPORT_NAVIGATION, -1.0, &availability, &meta, 1, &count) == FSIM_OK &&
          meta.validity == FSIM_REPORT_STALE);
    for (kind = FSIM_REPORT_TRACKS; kind <= FSIM_REPORT_LINK_TRACKS; ++kind)
        CHECK(fsim_agent_report(blue, viper, kind, NAN, &availability, NULL, 0, &count) == FSIM_OK && availability == FSIM_REPORT_AVAILABLE &&
              count == 0);

    /* a task is there once the link delivered it, and not before */
    fsim_c2_task_init(&order);
    order.type = FSIM_C2_TASK_PATROL;
    order.latitude_rad = 40.3 * 0.017453292519943295, order.longitude_rad = 0.0;
    sent = fsim_world_time(world);
    CHECK(fsim_c2_send_task(boss, crc, mine, &order, &task, &reason) == FSIM_OK && task != 0);
    CHECK(fsim_agent_report(blue, viper, FSIM_REPORT_TASKS, NAN, &availability, NULL, 0, &count) == FSIM_OK && count == 0);
    CHECK(fsim_world_step(world, 120) == FSIM_OK);
    CHECK(fsim_agent_report(blue, viper, FSIM_REPORT_TASKS, NAN, &availability, &meta, 1, &count) == FSIM_OK && count == 1);
    fsim_c2_task_init(&held);
    CHECK(fsim_agent_c2_task_get(blue, viper, 0, &held) == FSIM_OK && held.task_id == task);
    CHECK(meta.source == FSIM_REPORT_LINK && meta.system == c2 && meta.receive_time_s == held.received_s && meta.receive_time_s > sent);
    CHECK(isnan(meta.sample_time_s) && isnan(meta.age_s) && isnan(held.altitude_m)); /* (unknown stays unknown) */
    CHECK(fsim_agent_report(blue, viper, FSIM_REPORT_TASKS, NAN, &availability, NULL, 0, &count) == FSIM_OK && count == 1 &&
          fsim_agent_c2_task_count(blue, viper) == 1); /* (reading took nothing) */
    CHECK(fsim_agent_report(blue, viper, FSIM_REPORT_AUTHORITY, NAN, &availability, &meta, 1, &count) == FSIM_OK && count == 1 &&
          meta.source == FSIM_REPORT_MA_CORE && strcmp(fsim_agent_operator_callsign(blue, viper, 0), "CRC LEAD") == 0);

    /* a C2 node has links, and neither navigation nor sensors; another platform's reports are not its to read */
    CHECK(fsim_agent_report(boss, crc, FSIM_REPORT_NAVIGATION, NAN, &availability, NULL, 0, &count) == FSIM_OK &&
          availability == FSIM_REPORT_NOT_APPLICABLE && count == 0);
    CHECK(fsim_agent_report(boss, crc, FSIM_REPORT_PEERS, NAN, &availability, NULL, 0, &count) == FSIM_OK && availability == FSIM_REPORT_AVAILABLE);
    /* a peer's heartbeat has its own provenance, beside the peers index for index */
    CHECK(fsim_agent_report(blue, viper, FSIM_REPORT_PEER_HEARTBEATS, NAN, &availability, NULL, 0, &count) == FSIM_OK &&
          availability == FSIM_REPORT_AVAILABLE && count == fsim_dms_peer_count(blue, viper));
    CHECK(fsim_agent_report(boss, viper, FSIM_REPORT_NAVIGATION, NAN, &availability, NULL, 0, &count) == FSIM_NOT_ALLOWED);
    CHECK(fsim_world_remove_vehicle(world, viper) == FSIM_OK);
    CHECK(fsim_agent_report(blue, viper, FSIM_REPORT_NAVIGATION, NAN, &availability, NULL, 0, &count) == FSIM_OK &&
          availability == FSIM_REPORT_UNAVAILABLE && count == 0);
    CHECK(strcmp(fsim_report_availability_name(FSIM_REPORT_NOT_APPLICABLE), "not_applicable") == 0 &&
          strcmp(fsim_report_validity_name(FSIM_REPORT_STALE), "stale") == 0 && strcmp(fsim_report_source_name(FSIM_REPORT_LINK), "link") == 0);
    fsim_world_destroy(boss);
    fsim_world_destroy(blue);
    fsim_world_destroy(world);
    return 0;
}

/* A whole document by the two-call protocol. */
static char* endpoint_document(fsim_mission_endpoint* e, int (*read)(fsim_mission_endpoint*, char*, size_t, size_t*)) {
    size_t needed = 0;
    char* text;
    if (read(e, NULL, 0, &needed) != FSIM_OK) return NULL;
    text = (char*)malloc(needed);
    if (text && read(e, text, needed, &needed) != FSIM_OK) {
        free(text);
        text = NULL;
    }
    return text;
}

/* The installed episode's configuration (env_id 7), its JSBSim root already JSON text. */
static void mission_config(char* out, size_t size, const char* root) {
    snprintf(out, size,
             "{\"schema\":\"ma.config.v1\",\"env_id\":\"7\",\"base_seed\":\"18446744073709551615\", "
             "\"timing\":{\"dt_s\":0.008333333333333333,\"frame_skip\":4,\"decision_steps\":30}, "
             "\"scenario\":{\"world\":{\"name\":\"installed-mission\",\"capacity\":4,\"jsbsim_root\":\"%s\"}, "
             " \"vehicles\":[{\"name\":\"lead\",\"type\":\"jsbsim:f16c\",\"initial\":{\"lat_deg\":36.0,\"lon_deg\":-115.0,\"alt_msl_m\":7000,\"heading_deg\":0,\"airspeed_ms\":230}}]}, "
             "\"c2_nodes\":[{\"name\":\"crc\",\"latitude_rad\":0.6283185307179586,\"longitude_rad\":-2.007128639793479,\"altitude_msl_m\":600.0}], "
             "\"systems\":[{\"platform\":\"lead\",\"system_id\":\"1392508929\"},{\"platform\":\"crc\",\"system_id\":\"1392508931\"}], "
             "\"nets\":[{\"name\":\"l16\",\"members\":[\"lead\",\"crc\"]}], "
             "\"services\":[{\"service_id\":\"9007199254740995\",\"name\":\"lead-ma\",\"platform\":\"lead\",\"command_capabilities\":[\"*\"],\"owns_dms_ingress\":true,\"owns_dms_egress\":true}, "
             " {\"service_id\":\"4\",\"name\":\"crc-c2\",\"platform\":\"crc\",\"command_capabilities\":[],\"owns_dms_ingress\":true,\"owns_dms_egress\":true}], "
             "\"task\":{\"criteria\":[{\"id\":\"time\",\"kind\":\"tick_limit\",\"outcome\":\"truncated\",\"max_ticks\":3}]}}",
             root);
}

static int mission_episode(const char* jsbsim_root) {
    char config[4096], document[1024], root[2048];
    size_t i, j = 0;
    fsim_mission_episode* episode = NULL;
    fsim_mission_endpoint *lead = NULL, *stale = NULL;
    fsim_mission_referee* referee = NULL;
    char *first, *again;
    size_t needed = 0;
    int executing;
    for (i = 0; jsbsim_root[i] && j + 2 < sizeof root; ++i) { /* (its path as a JSON string's text) */
        if (jsbsim_root[i] == '\\' || jsbsim_root[i] == '"') root[j++] = '\\';
        root[j++] = jsbsim_root[i];
    }
    root[j] = '\0';
    mission_config(config, sizeof config, root);
    CHECK(fsim_mission_episode_create(config, strlen(config), &episode) == FSIM_OK);
    CHECK(fsim_mission_endpoint_open(episode, 9007199254740995ull, &lead) == FSIM_INVALID_ARGUMENT); /* (before a reset) */
    CHECK(fsim_mission_episode_reset(episode, 41) == FSIM_OK);
    CHECK(fsim_mission_endpoint_open(episode, 9007199254740995ull, &lead) == FSIM_OK);
    CHECK(fsim_mission_referee_open(episode, &referee) == FSIM_OK);
    first = endpoint_document(lead, fsim_mission_endpoint_observation_json);
    again = endpoint_document(lead, fsim_mission_endpoint_observation_json);
    CHECK(first && again && strcmp(first, again) == 0);
    CHECK(strstr(first, "\"episode\":{\"env_id\":\"7\",\"episode_generation\":\"1\",\"local_episode_index\":\"41\"}") != NULL);
    CHECK(fsim_mission_episode_step((fsim_mission_episode*)lead) == FSIM_NOT_ALLOWED);
    CHECK(fsim_mission_referee_truth_json((fsim_mission_referee*)lead, NULL, 0, &needed) == FSIM_NOT_ALLOWED);
    snprintf(document, sizeof document,
             "{\"schema\":\"ma.actions.v1\",\"episode\":{\"env_id\":\"7\",\"episode_generation\":\"1\",\"local_episode_index\":\"41\"},"
             "\"service_id\":\"9007199254740995\",\"tick\":\"0\",\"requests\":[{\"request_id\":\"1\",\"kind\":\"flight\","
             "\"operation\":\"new\",\"parameters\":{\"mode\":\"hsa\",\"heading_rad\":0.5}}]}");
    CHECK(fsim_mission_endpoint_submit(lead, document, strlen(document)) == FSIM_OK);
    CHECK(fsim_mission_episode_step(episode) == FSIM_OK);
    free(first);
    first = endpoint_document(lead, fsim_mission_endpoint_observation_json);
    CHECK(first != NULL);
    executing = strstr(first, "\"state\":\"executing\"") != NULL;
    CHECK(executing);
    CHECK(fsim_mission_referee_outcome_json(referee, NULL, 0, &needed) == FSIM_OK && needed > 1);
    stale = lead;
    CHECK(fsim_mission_episode_reset(episode, 41) == FSIM_OK);
    CHECK(fsim_mission_endpoint_observation_json(stale, NULL, 0, &needed) == FSIM_INVALID_ARGUMENT);
    fsim_mission_episode_destroy(episode);
    CHECK(fsim_mission_referee_outcome_json(referee, NULL, 0, &needed) == FSIM_INVALID_ARGUMENT);
    fsim_mission_endpoint_destroy(stale);
    fsim_mission_referee_destroy(referee);
    free(first);
    free(again);
    return 0;
}

/* A batch of two slots through fsim_mission_env.h (ABI 1.77): the episode's configuration under env_ids 7 and 8. */
static int mission_batch(const char* jsbsim_root) {
    char a[4096], b[4096], root[2048];
    const char* configs[2];
    size_t sizes[2], needed = 0, i, j = 0;
    fsim_mission_batch_options options;
    fsim_mission_slot_status status;
    fsim_mission_batch* batch = NULL;
    char* env;
    int t;
    for (i = 0; jsbsim_root[i] && j + 2 < sizeof root; ++i) {
        if (jsbsim_root[i] == '\\' || jsbsim_root[i] == '"') root[j++] = '\\';
        root[j++] = jsbsim_root[i];
    }
    root[j] = '\0';
    mission_config(a, sizeof a, root);
    strcpy(b, a);
    env = strstr(b, "\"env_id\":\"7\"");
    CHECK(env != NULL);
    env[10] = '8';
    configs[0] = a, configs[1] = b, sizes[0] = strlen(a), sizes[1] = strlen(b);
    fsim_mission_batch_options_init(&options);
    options.autoreset = FSIM_MISSION_AUTORESET_SAME_STEP;
    CHECK(fsim_mission_batch_create(configs, sizes, 2, &options, &batch) == FSIM_OK);
    CHECK(fsim_mission_batch_reset(batch) == FSIM_OK);
    for (t = 0; t < 3; ++t) CHECK(fsim_mission_batch_step(batch) == FSIM_OK);
    status.struct_size = sizeof status;
    CHECK(fsim_mission_batch_status(batch, 0, &status) == FSIM_OK);
    CHECK((status.flags & FSIM_MISSION_SLOT_ENDED) && (status.flags & FSIM_MISSION_SLOT_RESET) && status.generation == 2);
    CHECK(fsim_mission_batch_final_outcome_json(batch, 0, NULL, 0, &needed) == FSIM_OK && needed > 1);
    CHECK(fsim_mission_episode_step((fsim_mission_episode*)batch) == FSIM_NOT_ALLOWED);
    fsim_mission_batch_destroy(batch);
    return 0;
}

/* A whole file, terminated (NULL if it cannot be read). */
static char* read_all(const char* path) {
    FILE* f = fopen(path, "rb");
    long n;
    char* text = NULL;
    if (!f) return NULL;
    if (fseek(f, 0, SEEK_END) == 0 && (n = ftell(f)) >= 0 && fseek(f, 0, SEEK_SET) == 0 && (text = (char*)malloc((size_t)n + 1)) != NULL) {
        if (fread(text, 1, (size_t)n, f) != (size_t)n) {
            free(text);
            text = NULL;
        } else {
            text[n] = '\0';
        }
    }
    fclose(f);
    return text;
}

typedef struct chain_line {
    unsigned long long tick, service;
    const char* requests;
} chain_line;

/* A joint chain (M8; tools/validation/ma_contract/joint_chain.py) replayed through fsim_mission_env.h alone: its trace - each
 * tick's snapshots, then its acknowledgments (the receipts document a submission answers with) - written as every
 * installed consumer writes it, to be compared byte for byte with the C++ and Python consumers' traces. */
static int mission_chain(const char* jsbsim_root, const char* chain_path, const char* trace_path) {
    static chain_line lines[512];
    static const char token[] = "\"@JSBSIM_ROOT@\"";
    char path[4096], root[2048], key[64][256];
    char *chain, *text, *line, *next, *config_file = NULL, *config = NULL, *at, *document;
    unsigned long long services[64], tick, ticks = 0, t;
    size_t nlines = 0, nservices = 0, i, j, k = 0, dir;
    fsim_mission_episode* episode = NULL;
    fsim_mission_endpoint* endpoint = NULL;
    FILE* out;
    for (i = 0, j = 0; jsbsim_root[i] && j + 2 < sizeof root; ++i) {
        if (jsbsim_root[i] == '\\' || jsbsim_root[i] == '"') root[j++] = '\\';
        root[j++] = jsbsim_root[i];
    }
    root[j] = '\0';
    chain = read_all(chain_path);
    CHECK(chain != NULL);
    for (line = chain; line && *line; line = next) {
        next = strchr(line, '\n');
        if (next) *next++ = '\0';
        if (*line && line[strlen(line) - 1] == '\r') line[strlen(line) - 1] = '\0';
        if (*line == '#' || *line == '\0') continue;
        if (strncmp(line, "config ", 7) == 0) {
            dir = (size_t)(strrchr(chain_path, '/') ? strrchr(chain_path, '/') - chain_path : 0);
            CHECK(dir + strlen(line) < sizeof path);
            snprintf(path, sizeof path, "%.*s/%s", (int)dir, chain_path, line + 7);
            config_file = read_all(path);
            CHECK(config_file != NULL);
        } else if (strncmp(line, "ticks ", 6) == 0) {
            ticks = strtoull(line + 6, NULL, 10);
        } else {
            CHECK(nlines < sizeof lines / sizeof lines[0]);
            lines[nlines].tick = strtoull(line, &at, 10);
            lines[nlines].service = strtoull(at, &at, 10);
            lines[nlines].requests = at + 1;
            ++nlines;
        }
    }
    CHECK(config_file != NULL && ticks > 0);
    /* the JSBSim root as JSON text in place of its token */
    at = strstr(config_file, token);
    CHECK(at != NULL);
    config = (char*)malloc(strlen(config_file) + strlen(root) + 3);
    CHECK(config != NULL);
    sprintf(config, "%.*s\"%s\"%s", (int)(at - config_file), config_file, root, at + strlen(token));
    /* its services, ascending */
    for (at = strstr(config, "\"service_id\": \""); at; at = strstr(at + 1, "\"service_id\": \"")) {
        CHECK(nservices < 64);
        services[nservices++] = strtoull(at + 15, NULL, 10);
    }
    for (i = 1; i < nservices; ++i)
        for (j = i; j > 0 && services[j - 1] > services[j]; --j) t = services[j], services[j] = services[j - 1], services[j - 1] = t;
    CHECK(fsim_mission_episode_create(config, strlen(config), &episode) == FSIM_OK);
    CHECK(fsim_mission_episode_reset(episode, 0) == FSIM_OK);
    out = fopen(trace_path, "wb");
    CHECK(out != NULL);
    for (tick = 0; tick <= ticks; ++tick) {
        for (i = 0; i < nservices; ++i) {
            CHECK(fsim_mission_endpoint_open(episode, services[i], &endpoint) == FSIM_OK);
            text = endpoint_document(endpoint, fsim_mission_endpoint_observation_json);
            CHECK(text != NULL);
            fprintf(out, "obs %llu %llu %s\n", tick, services[i], text);
            at = strstr(text, "\"episode\":");
            CHECK(at && strchr(at, '}') && (size_t)(strchr(at, '}') - at) < sizeof key[i] - 1);
            sprintf(key[i], "%.*s", (int)(strchr(at, '}') - at - 9), at + 10);
            free(text);
            fsim_mission_endpoint_destroy(endpoint);
        }
        for (; k < nlines && lines[k].tick == tick; ++k) {
            for (i = 0; i < nservices && services[i] != lines[k].service; ++i) {
            }
            CHECK(i < nservices);
            document = (char*)malloc(strlen(lines[k].requests) + 512);
            CHECK(document != NULL);
            sprintf(document, "{\"schema\":\"ma.actions.v1\",\"episode\":%s,\"service_id\":\"%llu\",\"tick\":\"%llu\",\"requests\":%s}", key[i],
                    lines[k].service, tick, lines[k].requests);
            CHECK(fsim_mission_endpoint_open(episode, lines[k].service, &endpoint) == FSIM_OK);
            CHECK(fsim_mission_endpoint_submit(endpoint, document, strlen(document)) == FSIM_OK);
            text = endpoint_document(endpoint, fsim_mission_endpoint_receipts_json); /* (what the submission answered) */
            CHECK(text != NULL);
            fprintf(out, "ack %llu %llu %s\n", tick, lines[k].service, text);
            free(text);
            free(document);
            fsim_mission_endpoint_destroy(endpoint);
        }
        if (tick < ticks) CHECK(fsim_mission_episode_step(episode) == FSIM_OK);
    }
    CHECK(fclose(out) == 0);
    fsim_mission_episode_destroy(episode);
    free(config);
    free(config_file);
    free(chain);
    return 0;
}

int main(int argc, char** argv) {
    char jsbsim_root[4096];
    uint64_t a[4], b[4];
    int i;
    if (argc < 2 || argc % 2 || strlen(argv[1]) + 32 >= sizeof jsbsim_root) {
        fprintf(stderr, "usage: ma_consumer_c <installed-prefix> [<joint chain> <trace>]...\n");
        return 1;
    }
    if (installed_library(argv[1]) != 0) return 1;
    sprintf(jsbsim_root, "%s/share/flightsim/jsbsim", argv[1]);
    if (ownership(jsbsim_root, 1, a) != 0 || ownership(jsbsim_root, 0, b) != 0) return 1;
    CHECK(memcmp(a, b, sizeof a) == 0); /* (the order the agents were made in decides nothing) */
    CHECK(a[0] == 1 && a[1] == 1 && a[2] == 2 && a[3] == 2);
    if (worlds(jsbsim_root) != 0) return 1;
    if (reports(jsbsim_root) != 0) return 1;
    if (mission_episode(jsbsim_root) != 0) return 1;
    if (mission_batch(jsbsim_root) != 0) return 1;
    for (i = 2; i + 1 < argc; i += 2)
        if (mission_chain(jsbsim_root, argv[i], argv[i + 1]) != 0) return 1;
    printf("fsim ABI %u installed C99 consumer: the same ownership in either creation order; legal reports with provenance; "
           "a mission episode and a mission batch through fsim_mission_env.h; %d joint chain(s) replayed\n",
           (unsigned)fsim_abi_version(), (argc - 2) / 2);
    return 0;
}
