/* MA-service binding and legal reports from an installed SDK through its C ABI, as C99: fsim_c.h and the library,
 * nothing else (docs/sdk/mission-training.md, sections 9 and 10). */
#include <fsim/fsim_c.h>

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
    const unsigned char raw[1] = {0x01};
    uint32_t viper, wing, i, dest = 1;
    uint64_t msg = 0;
    int32_t reason = -1;
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
    if (flight_first) {
        CHECK(bind(world, 1, viper, flight, guidance, 2, 0, 0) == FSIM_OK && bind(world, 2, viper, comms, NULL, 0, 1, 1) == FSIM_OK);
    } else {
        CHECK(bind(world, 2, viper, comms, NULL, 0, 1, 1) == FSIM_OK && bind(world, 1, viper, flight, guidance, 2, 0, 0) == FSIM_OK);
    }
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

int main(int argc, char** argv) {
    char jsbsim_root[4096];
    uint64_t a[4], b[4];
    if (argc != 2 || strlen(argv[1]) + 32 >= sizeof jsbsim_root) {
        fprintf(stderr, "usage: ma_consumer_c <installed-prefix>\n");
        return 1;
    }
    if (installed_library(argv[1]) != 0) return 1;
    sprintf(jsbsim_root, "%s/share/flightsim/jsbsim", argv[1]);
    if (ownership(jsbsim_root, 1, a) != 0 || ownership(jsbsim_root, 0, b) != 0) return 1;
    CHECK(memcmp(a, b, sizeof a) == 0); /* (the order the agents were made in decides nothing) */
    CHECK(a[0] == 1 && a[1] == 1 && a[2] == 2 && a[3] == 2);
    if (reports(jsbsim_root) != 0) return 1;
    printf("fsim ABI %u installed C99 consumer: the same ownership in either creation order; legal reports with provenance\n",
           (unsigned)fsim_abi_version());
    return 0;
}
