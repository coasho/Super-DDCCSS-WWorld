/* Own-platform equipment reports through the installed C99 interface alone. */
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

#define CHECK(condition) do { \
    if (!(condition)) { \
        fprintf(stderr, "%s:%d: %s (%s)\n", __FILE__, __LINE__, #condition, fsim_last_error()); \
        return 1; \
    } \
} while (0)

#ifdef _WIN32
static int same_file(const char* a, const char* b) {
    BY_HANDLE_FILE_INFORMATION ia, ib;
    HANDLE ha = CreateFileA(a, 0, FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    HANDLE hb = CreateFileA(b, 0, FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    int same = 0;
    if (ha != INVALID_HANDLE_VALUE && hb != INVALID_HANDLE_VALUE && GetFileInformationByHandle(ha, &ia) && GetFileInformationByHandle(hb, &ib))
        same = ia.dwVolumeSerialNumber == ib.dwVolumeSerialNumber && ia.nFileIndexHigh == ib.nFileIndexHigh && ia.nFileIndexLow == ib.nFileIndexLow;
    if (ha != INVALID_HANDLE_VALUE) CloseHandle(ha);
    if (hb != INVALID_HANDLE_VALUE) CloseHandle(hb);
    return same;
}
#endif

static int installed_library(const char* prefix, const char* name) {
#ifdef _WIN32
    char loaded[32768], expected[32768];
    HMODULE module = GetModuleHandleA(name);
    DWORD size;
    CHECK(module != NULL);
    size = GetModuleFileNameA(module, loaded, sizeof loaded);
    CHECK(size > 0 && size < sizeof loaded);
    CHECK(strlen(prefix) + strlen(name) + 6 < sizeof expected);
    sprintf(expected, "%s/bin/%s", prefix, name);
    CHECK(same_file(loaded, expected));
    printf("installed DLL: %s\n", loaded);
    return 0;
#else
    (void)prefix;
    (void)name;
    return 1;
#endif
}

int main(int argc, char** argv) {
    fsim_world_options options;
    fsim_vehicle_spec spec;
    fsim_world* world = NULL;
    fsim_world* agent = NULL;
    fsim_report_meta meta;
    fsim_store_info store;
    fsim_payload_info payload;
    fsim_ms_options command_options;
    fsim_ms_result command_result;
    fsim_strike_status strike;
    fsim_consent_request consent;
    fsim_launch_zone zone;
    char jsbsim_root[32768];
    uint32_t vehicle = 0, count = 0, chaff = 0, flare = 0;
    uint32_t dispenser = UINT32_MAX, index, step;
    uint64_t activity = 0;
    int32_t reason = -1, availability = -1;
    CHECK(argc == 2);
    CHECK(installed_library(argv[1], "libfsim.dll") == 0 && installed_library(argv[1], "libJSBSim.dll") == 0);
    CHECK(strlen(argv[1]) + 24 < sizeof jsbsim_root);
    sprintf(jsbsim_root, "%s/share/flightsim/jsbsim", argv[1]);
    fsim_world_options_init(&options);
    options.name = "installed-equipment-c";
    options.publish = 0;
    options.workers = 1;
    options.pin_workers = 0;
    options.jsbsim_root = jsbsim_root;
    CHECK(fsim_world_create(&options, &world) == FSIM_OK);
    fsim_vehicle_spec_init(&spec);
    spec.name = "viper";
    spec.type = "jsbsim:f16c";
    spec.latitude_deg = 40.0;
    spec.altitude_msl_m = 6000.0;
    spec.heading_deg = 90.0;
    spec.airspeed_ms = 230.0;
    CHECK(fsim_world_create_vehicle(world, &spec, &vehicle) == FSIM_OK && vehicle != 0);
    CHECK(fsim_vehicle_load_loadout(world, vehicle, "mk82 x4", &reason) == FSIM_OK && reason == 0);
    CHECK(fsim_agent_create(world, vehicle, "equipment", &agent) == FSIM_OK);
    for (index = 0; index < fsim_vehicle_payload_count(agent, vehicle); ++index) {
        fsim_payload_info_init(&payload);
        CHECK(fsim_vehicle_payload(agent, vehicle, index, &payload) == FSIM_OK);
        if (payload.family == FSIM_PAYLOAD_COUNTERMEASURES) dispenser = payload.index;
    }
    CHECK(dispenser != UINT32_MAX);
    fsim_ms_options_init(&command_options);
    fsim_ms_result_init(&command_result);
    CHECK(fsim_vehicle_payload_state(agent, vehicle, dispenser, FSIM_PAYLOAD_OPERATE, &command_options, &command_result) == FSIM_OK);
    CHECK(command_result.reason == 0);
    for (step = 0; step < 1200; ++step) {
        CHECK(fsim_world_step(world, 1) == FSIM_OK);
        fsim_payload_info_init(&payload);
        CHECK(fsim_vehicle_payload(agent, vehicle, dispenser, &payload) == FSIM_OK);
        if (payload.state == FSIM_PAYLOAD_OPERATE) break;
    }
    CHECK(payload.state == FSIM_PAYLOAD_OPERATE);
    CHECK(fsim_world_countermeasures(agent, vehicle, &chaff, &flare) == FSIM_OK && chaff == 30 && flare == 15);
    fsim_report_meta_init(&meta);
    CHECK(fsim_agent_report(agent, vehicle, FSIM_REPORT_COUNTERMEASURES, NAN, &availability, &meta, 1, &count) == FSIM_OK);
    CHECK(availability == FSIM_REPORT_AVAILABLE && count == 1 && meta.source == FSIM_REPORT_OWN_PLATFORM && meta.validity == FSIM_REPORT_VALID);
    CHECK(meta.sample_time_s == fsim_world_time(world) && meta.receive_time_s == meta.sample_time_s && meta.age_s == 0.0);
    CHECK(fsim_world_dispense(agent, vehicle, 0) == FSIM_OK);
    CHECK(fsim_world_countermeasures(agent, vehicle, &chaff, &flare) == FSIM_OK && chaff == 29 && flare == 15);
    CHECK(fsim_world_dispense(agent, vehicle, 1) != FSIM_OK);
    CHECK(fsim_world_countermeasures(agent, vehicle, &chaff, &flare) == FSIM_OK && chaff == 29 && flare == 15);
    CHECK(fsim_agent_report(agent, vehicle, FSIM_REPORT_COUNTERMEASURES, -1.0, &availability, &meta, 1, &count) == FSIM_OK);
    CHECK(count == 1 && meta.validity == FSIM_REPORT_STALE);
    CHECK(fsim_agent_report(agent, vehicle, FSIM_REPORT_STORES, NAN, &availability, NULL, 0, &count) == FSIM_OK);
    CHECK(availability == FSIM_REPORT_AVAILABLE && count == 4 && fsim_vehicle_store_count(agent, vehicle) == count);
    fsim_store_info_init(&store);
    CHECK(fsim_vehicle_store(agent, vehicle, 0, &store) == FSIM_OK && strcmp(store.store, "mk82") == 0 && store.weapon == 0);
    CHECK(fsim_agent_report(agent, vehicle, FSIM_REPORT_STRIKES, NAN, &availability, NULL, 0, &count) == FSIM_OK);
    CHECK(availability == FSIM_REPORT_AVAILABLE && count == 0);
    fsim_strike_status_init(&strike);
    CHECK(fsim_agent_strike_report_get(agent, vehicle, 0, &activity, &strike, NULL, 0, NULL, 0) == FSIM_INVALID_ARGUMENT);
    CHECK(fsim_agent_report(agent, vehicle, FSIM_REPORT_CONSENTS, NAN, &availability, NULL, 0, &count) == FSIM_OK && count == 0);
    fsim_consent_request_init(&consent);
    CHECK(fsim_agent_consent_report_get(agent, vehicle, 0, &consent) == FSIM_INVALID_ARGUMENT);
    fsim_launch_zone_init(&zone);
    CHECK(fsim_world_launch_zone(agent, vehicle, "mk82", 0, &zone) == FSIM_NOT_ALLOWED);
    printf("installed C99 equipment: chaff=%u flare=%u stores=4 strikes=0 consents=0\n", chaff, flare);
    fsim_world_destroy(agent);
    fsim_world_destroy(world);
    return 0;
}
