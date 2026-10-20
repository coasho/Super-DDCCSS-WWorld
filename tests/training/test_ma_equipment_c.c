/* Permission-scoped equipment reports through the installed-compatible C99 ABI. */
#include "fsim/fsim_c.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "%s:%d: %s (%s)\n", __FILE__, __LINE__, #condition, fsim_last_error()); return 1; \
} } while (0)

static uint32_t aircraft(fsim_world* world, const char* name, double latitude) {
    fsim_vehicle_spec spec;
    uint32_t id = 0;
    fsim_vehicle_spec_init(&spec);
    spec.name = name;
    spec.type = "jsbsim:f16c";
    spec.latitude_deg = latitude;
    spec.longitude_deg = 0.0;
    spec.altitude_msl_m = 6000.0;
    spec.airspeed_ms = 230.0;
    return fsim_world_create_vehicle(world, &spec, &id) == FSIM_OK ? id : 0;
}

static uint32_t payload_of(const fsim_world* world, uint32_t id, int family) {
    uint32_t i;
    for (i = 0; i < fsim_vehicle_payload_count(world, id); ++i) {
        fsim_payload_info payload;
        fsim_payload_info_init(&payload);
        if (fsim_vehicle_payload(world, id, i, &payload) == FSIM_OK && payload.family == family)
            return payload.index;
    }
    return UINT32_MAX;
}

int main(int argc, char** argv) {
    fsim_world_options options;
    fsim_world* world = NULL;
    fsim_world *blue = NULL, *red = NULL, *boss = NULL, *observer = NULL;
    fsim_c2_node_spec node;
    fsim_net_spec net_spec;
    fsim_ma_service_binding binding;
    fsim_ms_result result;
    fsim_strike_command command;
    fsim_strike_status status;
    fsim_consent_request consent;
    fsim_report_meta meta;
    fsim_launch_zone zone;
    uint32_t a, b, crc = 0, net = 0, payload, count = 0, before_chaff, before_flare, chaff, flare;
    uint32_t store_indices[2] = {UINT32_MAX, UINT32_MAX}, weapon_indices[2] = {UINT32_MAX, UINT32_MAX};
    uint64_t activity = 0;
    int32_t availability = -1, reason = -1;
    CHECK(FSIM_STORE_LOST == 7);
    CHECK(strcmp(fsim_store_status_name(FSIM_STORE_LOST), "lost") == 0);
    CHECK(strcmp(fsim_store_status_name(FSIM_STORE_LOST + 1), "") == 0);
    fsim_world_options_init(&options);
    options.name = "c-ma-equipment";
    options.publish = 0;
    options.workers = 1;
    options.pin_workers = 0;
    if (argc > 1) options.jsbsim_root = argv[1];
    CHECK(fsim_world_create(&options, &world) == FSIM_OK);
    a = aircraft(world, "blue", 40.0);
    b = aircraft(world, "red", 40.1);
    CHECK(a && b);
    CHECK(fsim_vehicle_load_loadout(world, a, "mk82 x4", &reason) == FSIM_OK && reason == 0);
    payload = payload_of(world, a, FSIM_PAYLOAD_STORE_STATION);
    CHECK(payload != UINT32_MAX);
    CHECK(fsim_agent_create(world, a, "blue", &blue) == FSIM_OK);
    CHECK(fsim_agent_create(world, b, "red", &red) == FSIM_OK);

    CHECK(fsim_agent_report(blue, a, FSIM_REPORT_STORES, NAN, &availability, NULL, 0, &count) == FSIM_OK);
    CHECK(availability == FSIM_REPORT_AVAILABLE && count == fsim_vehicle_store_count(blue, a) && count == 4);
    fsim_report_meta_init(&meta);
    CHECK(fsim_agent_report(blue, a, FSIM_REPORT_STORES, NAN, &availability, &meta, 1, &count) == FSIM_OK);
    CHECK(meta.source == FSIM_REPORT_OWN_PLATFORM && meta.validity == FSIM_REPORT_VALID);
    CHECK(fsim_agent_report(red, a, FSIM_REPORT_STORES, NAN, &availability, NULL, 0, &count) == FSIM_NOT_ALLOWED);
    CHECK(fsim_world_countermeasures(world, a, &before_chaff, &before_flare) == FSIM_OK && before_chaff > 0);
    CHECK(fsim_world_countermeasures(blue, a, &chaff, &flare) == FSIM_OK);
    CHECK(chaff == before_chaff && flare == before_flare);
    CHECK(fsim_agent_report(blue, a, FSIM_REPORT_COUNTERMEASURES, NAN, &availability, NULL, 0, &count) == FSIM_OK);
    CHECK(availability == FSIM_REPORT_AVAILABLE && count == 1);
    CHECK(fsim_world_dispense(blue, a, 0) != FSIM_OK);
    CHECK(fsim_world_countermeasures(blue, a, &chaff, &flare) == FSIM_OK && chaff == before_chaff && flare == before_flare);
    fsim_ms_result_init(&result);
    CHECK(fsim_vehicle_payload_state(blue, a, payload_of(world, a, FSIM_PAYLOAD_COUNTERMEASURES), FSIM_PAYLOAD_OPERATE,
                                     NULL, &result) == FSIM_OK && result.status == FSIM_COMMAND_ACCEPTED);
    CHECK(fsim_world_step(world, 1) == FSIM_OK);
    CHECK(fsim_world_dispense(blue, a, 0) == FSIM_OK);
    CHECK(fsim_world_countermeasures(blue, a, &chaff, &flare) == FSIM_OK && chaff + 1 == before_chaff && flare == before_flare);
    CHECK(fsim_world_dispense(blue, b, 0) == FSIM_NOT_ALLOWED);
    fsim_launch_zone_init(&zone);
    CHECK(fsim_world_launch_zone(blue, a, "mk82", b, &zone) == FSIM_NOT_ALLOWED);
    CHECK(fsim_world_weapon_count(blue) == 0);

    fsim_c2_node_spec_init(&node);
    node.name = "crc";
    node.latitude_rad = 39.7 * 3.14159265358979323846 / 180.0;
    node.altitude_msl_m = 200.0;
    fsim_net_spec_init(&net_spec);
    CHECK(fsim_world_create_c2_node(world, &node, &crc) == FSIM_OK);
    CHECK(fsim_world_create_net(world, &net_spec, &net) == FSIM_OK);
    CHECK(fsim_world_join_net(world, a, net) == FSIM_OK && fsim_world_join_net(world, crc, net) == FSIM_OK);
    CHECK(fsim_agent_create(world, crc, "ingress", &boss) == FSIM_OK);
    CHECK(fsim_agent_create(world, crc, "observer", &observer) == FSIM_OK);
    fsim_ma_service_binding_init(&binding);
    binding.service_id = 1;
    binding.command_capabilities = NULL;
    binding.command_capability_count = 0;
    binding.owns_dms_ingress = binding.owns_dms_egress = 1;
    CHECK(fsim_world_bind_ma_service_agent(world, boss, &binding) == FSIM_OK);
    binding.service_id = 2;
    binding.owns_dms_ingress = binding.owns_dms_egress = 0;
    CHECK(fsim_world_bind_ma_service_agent(world, observer, &binding) == FSIM_OK);
    CHECK(fsim_c2_declare_role(boss, crc, 2, "CRC", NULL, 0, &reason) == FSIM_OK);
    CHECK(fsim_world_step(world, 30) == FSIM_OK);
    fsim_ms_result_init(&result);
    CHECK(fsim_vehicle_payload_state(blue, a, payload, FSIM_PAYLOAD_OPERATE, NULL, &result) == FSIM_OK);
    CHECK(result.status == FSIM_COMMAND_ACCEPTED);
    CHECK(fsim_world_step(world, 1) == FSIM_OK);
    fsim_strike_command_init(&command);
    command.payload = payload;
    command.store = "mk82";
    command.quantity = 2;
    fsim_ms_result_init(&result);
    CHECK(fsim_vehicle_strike(blue, a, &command, NULL, &result) == FSIM_OK && result.status == FSIM_COMMAND_ACCEPTED);
    CHECK(fsim_agent_report(boss, crc, FSIM_REPORT_CONSENTS, NAN, &availability, NULL, 0, &count) == FSIM_OK && count == 0);
    CHECK(fsim_world_step(world, 360) == FSIM_OK);
    CHECK(fsim_agent_report(blue, a, FSIM_REPORT_STRIKES, NAN, &availability, NULL, 0, &count) == FSIM_OK && count == 1);
    fsim_strike_status_init(&status);
    CHECK(fsim_agent_strike_report_get(blue, a, 0, &activity, &status, store_indices, 1, weapon_indices, 1) == FSIM_OK);
    CHECK(activity == result.activity && status.phase == FSIM_STRIKE_AWAITING_CONSENT && status.store_count == 2);
    CHECK(store_indices[0] != UINT32_MAX && store_indices[1] == UINT32_MAX && weapon_indices[1] == UINT32_MAX);
    {
        struct { uint32_t struct_size; int32_t phase; uint32_t sentinel; } short_status;
        short_status.struct_size = (uint32_t)offsetof(fsim_strike_status, jettison);
        short_status.phase = -1;
        short_status.sentinel = UINT32_C(0x12345678);
        CHECK(fsim_agent_strike_report_get(blue, a, 0, &activity, (fsim_strike_status*)&short_status, NULL, 0, NULL, 0) == FSIM_OK);
        CHECK(short_status.phase == status.phase && short_status.sentinel == UINT32_C(0x12345678));
    }
    CHECK(fsim_agent_strike_report_get(red, a, 0, &activity, &status, NULL, 0, NULL, 0) == FSIM_NOT_ALLOWED);
    CHECK(fsim_agent_strike_report_get(blue, a, 99, &activity, &status, NULL, 0, NULL, 0) == FSIM_INVALID_ARGUMENT);
    CHECK(fsim_agent_report(boss, crc, FSIM_REPORT_CONSENTS, NAN, &availability, NULL, 0, &count) == FSIM_OK && count == 1);
    fsim_consent_request_init(&consent);
    CHECK(fsim_agent_consent_report_get(boss, crc, 0, &consent) == FSIM_OK);
    CHECK(consent.from == fsim_dms_system(blue, a) && consent.strike == result.activity && consent.quantity == 2);
    CHECK(fsim_agent_consent_report_get(boss, crc, 0, &consent) == FSIM_OK && !consent.answered);
    CHECK(fsim_agent_report(observer, crc, FSIM_REPORT_CONSENTS, NAN, &availability, NULL, 0, &count) == FSIM_OK);
    CHECK(availability == FSIM_REPORT_NOT_OWNER && count == 0);
    CHECK(fsim_agent_consent_report_get(observer, crc, 0, &consent) == FSIM_NOT_ALLOWED);
    CHECK(fsim_c2_consent_count(observer, crc) == 1); /* The legacy getter retains its established semantics. */
    CHECK(fsim_agent_consent_report_get(boss, crc, 0, &consent) == FSIM_OK);
    CHECK(fsim_c2_answer_consent(boss, crc, consent.from, consent.request_id, 1, &reason) == FSIM_OK);
    CHECK(fsim_world_step(world, 360) == FSIM_OK);
    fsim_strike_status_init(&status);
    CHECK(fsim_agent_strike_report_get(blue, a, 0, &activity, &status, store_indices, 2, weapon_indices, 2) == FSIM_OK);
    CHECK(activity == result.activity && status.phase == FSIM_STRIKE_DONE && status.weapon_count == 2);
    {
        uint32_t released[2];
        fsim_strike_status native;
        fsim_strike_status_init(&native);
        CHECK(fsim_strike_status_get(world, activity, &native, NULL, 0, released, 2) == FSIM_OK && native.weapon_count == 2);
        CHECK(weapon_indices[0] == released[0] && weapon_indices[1] == released[1]);
    }
    CHECK(fsim_agent_report(boss, crc, FSIM_REPORT_STORES, NAN, &availability, NULL, 0, &count) == FSIM_OK);
    CHECK(availability == FSIM_REPORT_NOT_APPLICABLE && count == 0);
    command.quantity = 1;
    command.target = b;
    command.consent = 1;
    CHECK(fsim_vehicle_strike(world, a, &command, NULL, &result) == FSIM_OK && result.status == FSIM_COMMAND_ACCEPTED);
    fsim_strike_status_init(&status);
    CHECK(fsim_strike_status_get(world, result.activity, &status, NULL, 0, NULL, 0) == FSIM_OK && status.target == b);
    {
        fsim_strike_status visible;
        uint32_t native_store = UINT32_MAX, visible_store = UINT32_MAX;
        fsim_strike_status_init(&visible);
        CHECK(fsim_strike_status_get(world, result.activity, &status, &native_store, 1, NULL, 0) == FSIM_OK);
        CHECK(fsim_strike_status_get(blue, result.activity, &visible, &visible_store, 1, NULL, 0) == FSIM_OK);
        CHECK(visible.target == 0);
        CHECK(visible.phase == status.phase && visible.jettison == status.jettison);
        CHECK(visible.store_count == status.store_count && visible.weapon_count == status.weapon_count);
        CHECK(visible_store == native_store);
        CHECK(fsim_strike_status_get(blue, result.activity, &visible, NULL, 0, NULL, 0) == FSIM_OK && visible.target == 0);
        CHECK(fsim_strike_status_get(red, result.activity, &visible, NULL, 0, NULL, 0) == FSIM_NOT_ALLOWED);
        CHECK(fsim_strike_status_get(world, result.activity, &status, NULL, 0, NULL, 0) == FSIM_OK && status.target == b);
        {
            struct { uint32_t struct_size; int32_t phase; uint32_t sentinel; } short_status;
            short_status.struct_size = (uint32_t)offsetof(fsim_strike_status, jettison);
            short_status.phase = -1;
            short_status.sentinel = UINT32_C(0x12345678);
            CHECK(fsim_strike_status_get(blue, result.activity, (fsim_strike_status*)&short_status,
                                         NULL, 0, NULL, 0) == FSIM_OK);
            CHECK(short_status.phase == status.phase && short_status.sentinel == UINT32_C(0x12345678));
        }
    }
    CHECK(fsim_agent_report(blue, a, FSIM_REPORT_STRIKES, NAN, &availability, NULL, 0, &count) == FSIM_OK);
    {
        uint32_t i;
        int found = 0;
        for (i = 0; i < count; ++i) {
            CHECK(fsim_agent_strike_report_get(blue, a, i, &activity, &status, NULL, 0, NULL, 0) == FSIM_OK);
            if (activity == result.activity) { CHECK(status.target == 0); found = 1; }
        }
        CHECK(found);
    }
    fsim_world_destroy(observer);
    fsim_world_destroy(boss);
    fsim_world_destroy(red);
    fsim_world_destroy(blue);
    fsim_world_destroy(world);
    puts("MA equipment C99 reports passed");
    return 0;
}
