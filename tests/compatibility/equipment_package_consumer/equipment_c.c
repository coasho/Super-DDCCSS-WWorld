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

static int guided_controls(const fsim_world_options* base_options) {
    const double deg = 3.14159265358979323846 / 180.0;
    fsim_world_options options = *base_options;
    fsim_world *world = NULL, *launcher = NULL, *approver = NULL;
    fsim_vehicle_spec spec;
    fsim_c2_node_spec node;
    fsim_net_spec net_spec;
    fsim_payload_info payload;
    fsim_ms_options command_options;
    fsim_ms_result result;
    fsim_command_result flight;
    fsim_amti_command scan;
    fsim_amti_track tracks[16];
    fsim_strike_command strike;
    fsim_consent_request consent;
    fsim_weapon_info weapon;
    fsim_weapon_controls_v1 sample;
    fsim_weapon_fin_v1 fins[32], short_rows[2];
    uint32_t carrier = 0, target = 0, c2 = 0, net = 0, designation = 0;
    uint32_t radar = UINT32_MAX, stations = UINT32_MAX, i, j, n, ready = 0, required = 0;
    uint64_t track = 0, activity = 0;
    int32_t reason = -1;
    int answered = 0, moved = 0;
    double hold[4] = {250, 0, 0, NAN}, jam_angle;
    char jam_property[128];
    const fsim_vehicle_state* own;
    options.name = "installed-fin-c"; options.terrain = 0;
    CHECK(fsim_world_create(&options, &world) == FSIM_OK);
    fsim_vehicle_spec_init(&spec);
    spec.name = "launcher"; spec.type = "jsbsim:f16c";
    spec.latitude_deg = 40; spec.longitude_deg = 0; spec.altitude_msl_m = 6000; spec.heading_deg = 90; spec.airspeed_ms = 250;
    CHECK(fsim_world_create_vehicle(world, &spec, &carrier) == FSIM_OK);
    CHECK(fsim_vehicle_load_loadout(world, carrier, "air superiority", &reason) == FSIM_OK && reason == 0);
    CHECK(fsim_agent_create(world, carrier, "launcher", &launcher) == FSIM_OK);
    hold[2] = 90 * deg;
    CHECK(fsim_vehicle_submit(launcher, carrier, FSIM_LEVEL_VELOCITY, hold, 4, NULL, &flight) == FSIM_OK && flight.reason == 0);
    fsim_c2_node_spec_init(&node);
    node.name = "approver"; node.latitude_rad = 39.9 * deg; node.altitude_msl_m = 200;
    CHECK(fsim_world_create_c2_node(world, &node, &c2) == FSIM_OK);
    fsim_net_spec_init(&net_spec);
    CHECK(fsim_world_create_net(world, &net_spec, &net) == FSIM_OK);
    CHECK(fsim_world_join_net(world, carrier, net) == FSIM_OK && fsim_world_join_net(world, c2, net) == FSIM_OK);
    CHECK(fsim_agent_create(world, c2, "approver", &approver) == FSIM_OK);
    CHECK(fsim_c2_declare_role(approver, c2, 2, "APPROVER", NULL, 0, &reason) == FSIM_OK && reason == 0);
    for (i = 0; i < fsim_vehicle_payload_count(launcher, carrier); ++i) {
        fsim_payload_info_init(&payload);
        CHECK(fsim_vehicle_payload(launcher, carrier, i, &payload) == FSIM_OK);
        if (payload.family == FSIM_PAYLOAD_AIR_RADAR) radar = payload.index;
        if (payload.family == FSIM_PAYLOAD_STORE_STATION) stations = payload.index;
    }
    CHECK(radar != UINT32_MAX && stations != UINT32_MAX);
    fsim_ms_options_init(&command_options); fsim_ms_result_init(&result);
    CHECK(fsim_vehicle_payload_state(launcher, carrier, radar, FSIM_PAYLOAD_OPERATE, &command_options, &result) == FSIM_OK && result.reason == 0);
    CHECK(fsim_vehicle_payload_state(launcher, carrier, stations, FSIM_PAYLOAD_OPERATE, &command_options, &result) == FSIM_OK && result.reason == 0);
    for (i = 0; i < 1200 && ready != 2; ++i) {
        CHECK(fsim_world_step(world, 1) == FSIM_OK);
        ready = 0;
        for (j = 0; j < 2; ++j) {
            fsim_payload_info_init(&payload);
            CHECK(fsim_vehicle_payload(launcher, carrier, j ? stations : radar, &payload) == FSIM_OK);
            if (payload.state == FSIM_PAYLOAD_OPERATE) ++ready;
        }
    }
    CHECK(ready == 2);
    own = fsim_vehicle_state_ptr(world, carrier); CHECK(own != NULL);
    spec.name = "opponent"; spec.latitude_deg = own->latitude_rad / deg + .01; spec.longitude_deg = own->longitude_rad / deg + .15;
    spec.heading_deg = 270;
    CHECK(fsim_world_create_vehicle(world, &spec, &target) == FSIM_OK);
    hold[2] = 270 * deg;
    CHECK(fsim_vehicle_submit(world, target, FSIM_LEVEL_VELOCITY, hold, 4, NULL, &flight) == FSIM_OK && flight.reason == 0);
    fsim_amti_command_init(&scan);
    scan.payload = radar; scan.bars = 1; scan.azimuth_half_width_rad = 15 * deg; scan.range_scale_m = 60000;
    CHECK(fsim_vehicle_amti_search(launcher, carrier, &scan, &command_options, &result) == FSIM_OK && result.reason == 0);
    for (i = 0; i < 600 && !track; ++i) {
        CHECK(fsim_world_step(world, 1) == FSIM_OK);
        for (j = 0; j < 16; ++j) fsim_amti_track_init(&tracks[j]);
        CHECK(fsim_vehicle_amti_tracks(launcher, carrier, radar, tracks, 16, &n) == FSIM_OK);
        for (j = 0; j < n && j < 16; ++j) if (tracks[j].updates >= 3) { track = tracks[j].id; break; }
    }
    CHECK(track != 0);
    CHECK(fsim_vehicle_designate(launcher, carrier, radar, track, &designation) == FSIM_OK && designation != 0);
    scan.task = FSIM_AMTI_TRACK; scan.mode = FSIM_AMTI_SINGLE_TARGET_TRACK; scan.track = track;
    CHECK(fsim_vehicle_amti_search(launcher, carrier, &scan, &command_options, &result) == FSIM_OK && result.reason == 0);
    fsim_strike_command_init(&strike);
    strike.payload = stations; strike.store = "aim-120c"; strike.target = designation;
    CHECK(fsim_vehicle_strike(launcher, carrier, &strike, &command_options, &result) == FSIM_OK && result.reason == 0);
    activity = result.activity;
    CHECK(fsim_world_weapon_count(world) == 0);
    for (i = 0; i < 1200 && !answered; ++i) {
        CHECK(fsim_world_step(world, 1) == FSIM_OK);
        for (j = 0; j < fsim_c2_consent_count(approver, c2); ++j) {
            fsim_consent_request_init(&consent);
            CHECK(fsim_c2_consent_get(approver, c2, j, &consent) == FSIM_OK);
            if (consent.strike == activity) {
                CHECK(fsim_world_weapon_count(world) == 0 && consent.received_s >= consent.time_s);
                CHECK(fsim_c2_answer_consent(approver, c2, consent.from, consent.request_id, 1, &reason) == FSIM_OK && reason == 0);
                answered = 1; break;
            }
        }
    }
    CHECK(answered);
    for (i = 0; i < 600 && fsim_world_weapon_count(world) == 0; ++i) CHECK(fsim_world_step(world, 1) == FSIM_OK);
    CHECK(fsim_world_weapon_count(world) == 1);
    fsim_weapon_info_init(&weapon); CHECK(fsim_world_weapon(world, 0, &weapon) == FSIM_OK && !weapon.jettisoned);
    fsim_weapon_controls_v1_init(&sample);
    CHECK(fsim_world_weapon_controls_v1(launcher, weapon.id, &sample, NULL, 0, &required) == FSIM_NOT_ALLOWED);
    for (i = 0; i < 20; ++i) {
        CHECK(fsim_world_step(world, 1) == FSIM_OK);
        CHECK(fsim_world_weapon_controls_v1(world, weapon.id, &sample, NULL, 0, &required) == FSIM_OK && required == 4);
        CHECK(sample.available && sample.controller_available && sample.sample_time_s == fsim_world_time(world));
        CHECK(sample.controller_time_s <= sample.sample_time_s);
        for (j = 0; j < 32; ++j) fsim_weapon_fin_v1_init(&fins[j]);
        CHECK(fsim_world_weapon_controls_v1(world, weapon.id, &sample, fins, 32, &required) == FSIM_OK && required == 4);
        for (j = 0; j < required; ++j) {
            CHECK(fins[j].valid && fins[j].id && fins[j].actual_rad >= fins[j].minimum_rad && fins[j].actual_rad <= fins[j].maximum_rad);
            if (fabs(fins[j].actual_rad) > 1e-8) moved = 1;
        }
    }
    CHECK(moved);
    fsim_weapon_fin_v1_init(&short_rows[0]); fsim_weapon_fin_v1_init(&short_rows[1]); short_rows[1].id = 0xdeadbeef;
    CHECK(fsim_world_weapon_controls_v1(world, weapon.id, &sample, short_rows, 1, &required) == FSIM_OK && required == 4);
    CHECK(short_rows[1].id == 0xdeadbeef && short_rows[0].id == fins[0].id && short_rows[0].actual_rad == fins[0].actual_rad);
    jam_angle = fins[0].actual_rad;
    sprintf(jam_property, "fcs/fin-%u-actuator/malfunction/fail_stuck", fins[0].native_index);
    CHECK(fsim_vehicle_set_property(world, weapon.id, jam_property, 1) == FSIM_OK);
    CHECK(fsim_world_step(world, 5) == FSIM_OK);
    CHECK(fsim_world_weapon_controls_v1(world, weapon.id, &sample, fins, 32, &required) == FSIM_OK);
    CHECK(fins[0].failed_stuck && fins[0].actual_rad == jam_angle);
    printf("installed C99 guided controls: armed=1 delivered_consent=1 fins=4 jam=1 bounded_buffer=1 actor_refusal=1\n");
    fsim_world_destroy(approver); fsim_world_destroy(launcher); fsim_world_destroy(world);
    return 0;
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
    CHECK(guided_controls(&options) == 0);
    return 0;
}
