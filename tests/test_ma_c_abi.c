/* MA services and legal reports through the C ABI, compiled as C99 (docs/sdk/mission-training.md, sections 9 and 10):
 * what the C++ and Python tests hold - configured ownership whatever order the agents were made in, refused overlaps and
 * foreign controllers, each DMS side its owner's - through fsim_c.h alone. */
#include "fsim/fsim_c.h"

#include <math.h>
#include <stddef.h>
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

static const char* const kHsa = "fsim.guidance.hsa";
static const char* const kRoute = "fsim.guidance.route";

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

static int fly(fsim_world* agent, uint32_t id, double heading_rad, fsim_command_result* cr) {
    double hsa[6];
    fsim_command_options co;
    fsim_command_options_init(&co);
    hsa[0] = heading_rad, hsa[1] = fsim_hold(), hsa[2] = 230.0, hsa[3] = FSIM_SPEED_TRUE_AIRSPEED, hsa[4] = 6000.0, hsa[5] = FSIM_ALTITUDE_MSL;
    memset(cr, 0, sizeof *cr);
    return fsim_vehicle_submit_mode(agent, id, FSIM_MODE_HSA, hsa, 6, &co, cr);
}

/* The service of a controller, as the bindings have it (0: none). */
static uint64_t service_of(fsim_world* world, uint32_t controller) {
    uint32_t i, n = fsim_world_ma_service_count(world);
    for (i = 0; i < n; ++i) {
        fsim_ma_service_binding b;
        fsim_ma_service_binding_init(&b);
        if (fsim_world_ma_service(world, i, &b) == FSIM_OK && b.controller == controller) return b.service_id;
    }
    return 0;
}

/* Two services on one aircraft, their agents made in one order or the other: what the bindings give, by service id. */
static int configured(const char* jsbsim_root, int flight_first, uint64_t owners[2], uint64_t holders[2], int flown[2]) {
    fsim_world* world = make_world(flight_first ? "c-ma-order-a" : "c-ma-order-b", jsbsim_root);
    fsim_world* flight = NULL;
    fsim_world* sensors = NULL;
    const char* flying[2];
    uint32_t a, holder = 0;
    int32_t granted = 0, reason = 0, mode = -1;
    fsim_command_result cr;
    flying[0] = kHsa, flying[1] = kRoute;
    CHECK(world != NULL);
    a = make_aircraft(world, "viper", 0.0);
    CHECK(a != 0);
    if (flight_first) {
        CHECK(fsim_agent_create(world, a, "flight", &flight) == FSIM_OK && fsim_agent_create(world, a, "sensors", &sensors) == FSIM_OK);
    } else {
        CHECK(fsim_agent_create(world, a, "sensors", &sensors) == FSIM_OK && fsim_agent_create(world, a, "flight", &flight) == FSIM_OK);
    }
    CHECK(fsim_vehicle_control_mode(world, a, &mode) == FSIM_OK && mode == FSIM_CONTROL_OPEN);
    if (flight_first) {
        CHECK(bind(world, 1, a, flight, flying, 2, 0, 1) == FSIM_OK && bind(world, 2, a, sensors, NULL, 0, 1, 0) == FSIM_OK);
    } else {
        CHECK(bind(world, 2, a, sensors, NULL, 0, 1, 0) == FSIM_OK && bind(world, 1, a, flight, flying, 2, 0, 1) == FSIM_OK);
    }
    CHECK(fsim_vehicle_control_mode(world, a, &mode) == FSIM_OK && mode == FSIM_CONTROL_GRANTED);
    owners[0] = service_of(world, fsim_world_ma_capability_owner(world, a, kHsa));
    owners[1] = service_of(world, fsim_world_ma_capability_owner(world, a, kRoute));
    CHECK(fsim_vehicle_control_holder(world, a, kHsa, &granted, &holder) == FSIM_OK && granted);
    holders[0] = service_of(world, holder);
    CHECK(fsim_vehicle_control_holder(world, a, kRoute, &granted, &holder) == FSIM_OK && granted);
    holders[1] = service_of(world, holder);
    /* only its owner asks for it, and only its owner flies it */
    /* (the call answers in *reason: FSIM_OK with "not_allowed", as every refused grant) */
    CHECK(fsim_vehicle_request_control_by(sensors, a, kHsa, 0, &reason) == FSIM_OK && strcmp(fsim_reason_name(reason), "not_allowed") == 0);
    CHECK(fsim_vehicle_request_control(sensors, a, kRoute, &reason) == FSIM_OK && strcmp(fsim_reason_name(reason), "not_allowed") == 0);
    CHECK(fsim_vehicle_request_control_by(flight, a, kHsa, 0, &reason) == FSIM_OK && reason == 0);
    flown[0] = fly(flight, a, 1.0, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED;
    flown[1] = fly(sensors, a, 2.0, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED;
    /* the bindings read back in ascending service id, whichever was bound first; an agent's handle reads none */
    {
        fsim_ma_service_binding b;
        fsim_ma_service_binding_init(&b);
        CHECK(fsim_world_ma_service_count(world) == 2);
        CHECK(fsim_world_ma_service(world, 0, &b) == FSIM_OK && b.service_id == 1 && b.platform == a && b.command_capability_count == 2 &&
              strcmp(b.command_capabilities[0], kHsa) == 0 && strcmp(b.command_capabilities[1], kRoute) == 0 && !b.owns_dms_ingress &&
              b.owns_dms_egress && b.controller == controller_of(flight));
        CHECK(fsim_world_ma_service(world, 1, &b) == FSIM_OK && b.service_id == 2 && b.command_capability_count == 0 && b.owns_dms_ingress);
        CHECK(fsim_world_ma_service(world, 2, &b) == FSIM_INVALID_ARGUMENT);
        CHECK(fsim_world_ma_service_count(flight) == 0 && fsim_world_ma_service(flight, 0, &b) == FSIM_NOT_ALLOWED);
        CHECK(fsim_world_ma_capability_owner(flight, a, kHsa) == 0);
        CHECK(bind(flight, 9, a, sensors, NULL, 0, 0, 0) == FSIM_NOT_ALLOWED);
    }
    fsim_world_destroy(sensors);
    fsim_world_destroy(flight);
    fsim_world_destroy(world);
    return 0;
}

int main(int argc, char** argv) {
    const char* jsbsim_root = argc > 1 ? argv[1] : NULL;

    /* --- M1: explicit MA-service binding -------------------------------------------------------------------------- */
    {
        uint64_t owners_a[2], owners_b[2], holders_a[2], holders_b[2];
        int flown_a[2], flown_b[2];
        CHECK(configured(jsbsim_root, 1, owners_a, holders_a, flown_a) == 0);
        CHECK(configured(jsbsim_root, 0, owners_b, holders_b, flown_b) == 0);
        CHECK(owners_a[0] == 1 && owners_a[1] == 1 && holders_a[0] == 1 && holders_a[1] == 1 && flown_a[0] && !flown_a[1]);
        CHECK(memcmp(owners_a, owners_b, sizeof owners_a) == 0 && memcmp(holders_a, holders_b, sizeof holders_a) == 0 &&
              memcmp(flown_a, flown_b, sizeof flown_a) == 0);
    }
    {
        /* refused: a duplicate, a foreign or unminted controller, an unknown capability - and nothing changed */
        fsim_world* world = make_world("c-ma-refusals", jsbsim_root);
        fsim_world* one = NULL;
        fsim_world* two = NULL;
        fsim_world* other = NULL;
        const char* hsa[1];
        const char* route[1];
        const char* twice[2];
        const char* all[1];
        const char* unknown[1];
        fsim_ma_service_binding b;
        uint32_t a, w2;
        int32_t mode = -1;
        hsa[0] = kHsa, route[0] = kRoute, twice[0] = kHsa, twice[1] = kHsa, all[0] = "*", unknown[0] = "fsim.guidance.no_such";
        CHECK(world != NULL);
        a = make_aircraft(world, "viper", 0.0);
        w2 = make_aircraft(world, "wing", 0.02);
        CHECK(a && w2 && fsim_agent_create(world, a, "one", &one) == FSIM_OK && fsim_agent_create(world, a, "two", &two) == FSIM_OK &&
              fsim_agent_create(world, w2, "other", &other) == FSIM_OK);
        fsim_ma_service_binding_init(&b);
        CHECK(b.struct_size == sizeof b && b.service_id == 0 && b.command_capabilities == NULL);
        CHECK(fsim_world_bind_ma_service(world, NULL) == FSIM_INVALID_ARGUMENT);
        b.service_id = 7, b.platform = a, b.controller = 0x40000099u; /* (never minted) */
        CHECK(fsim_world_bind_ma_service(world, &b) == FSIM_INVALID_ARGUMENT && strstr(fsim_last_error(), "not one this world minted") != NULL);
        b.controller = controller_of(other); /* (another platform's agent) */
        CHECK(fsim_world_bind_ma_service(world, &b) == FSIM_INVALID_ARGUMENT && strstr(fsim_last_error(), "minted for entity") != NULL);
        b.controller = controller_of(one);
        b.struct_size = 8; /* (a header that does not reach its fields) */
        CHECK(fsim_world_bind_ma_service(world, &b) == FSIM_INVALID_ARGUMENT);
        CHECK(bind(world, 0, a, one, hsa, 1, 1, 1) == FSIM_INVALID_ARGUMENT);
        CHECK(bind(world, 7, a, one, unknown, 1, 1, 1) == FSIM_INVALID_ARGUMENT && strstr(fsim_last_error(), "not a command capability") != NULL);
        CHECK(bind(world, 7, a, one, twice, 2, 1, 1) == FSIM_INVALID_ARGUMENT);
        CHECK(fsim_world_ma_service_count(world) == 0);
        CHECK(fsim_vehicle_control_mode(world, a, &mode) == FSIM_OK && mode == FSIM_CONTROL_OPEN);
        CHECK(bind(world, 7, a, one, hsa, 1, 1, 0) == FSIM_OK && fsim_last_error()[0] == '\0');
        CHECK(bind(world, 7, a, two, route, 1, 0, 0) == FSIM_INVALID_ARGUMENT); /* the service id */
        CHECK(bind(world, 8, a, one, route, 1, 0, 0) == FSIM_INVALID_ARGUMENT); /* the controller */
        CHECK(bind(world, 8, a, two, hsa, 1, 0, 0) == FSIM_INVALID_ARGUMENT);   /* the capability */
        CHECK(bind(world, 8, a, two, all, 1, 0, 0) == FSIM_INVALID_ARGUMENT);   /* every capability, one of them owned */
        CHECK(bind(world, 8, a, two, route, 1, 1, 0) == FSIM_INVALID_ARGUMENT && strstr(fsim_last_error(), "DMS ingress") != NULL);
        CHECK(fsim_world_ma_service_count(world) == 1);
        CHECK(bind(world, 8, a, two, route, 1, 0, 1) == FSIM_OK);
        CHECK(bind(world, 20, w2, other, all, 1, 1, 1) == FSIM_OK);
        CHECK(fsim_world_ma_service_count(world) == 3);
        CHECK(fsim_world_ma_capability_owner(world, a, kRoute) == controller_of(two));
        CHECK(fsim_world_ma_capability_owner(world, w2, kRoute) == controller_of(other));
        CHECK(fsim_world_ma_capability_owner(world, a, "fsim.guidance.pursuit") == 0 && fsim_world_ma_capability_owner(world, a, NULL) == 0);
        /* removal ends a platform's bindings */
        CHECK(fsim_world_remove_vehicle(world, w2) == FSIM_OK && fsim_world_ma_service_count(world) == 2);
        fsim_world_destroy(other);
        fsim_world_destroy(two);
        fsim_world_destroy(one);
        fsim_world_destroy(world);
    }
    {
        /* each DMS side its owner's; the platform's picture every service's alike */
        fsim_world* world = make_world("c-ma-dms", jsbsim_root);
        fsim_world* talker = NULL;
        fsim_world* listener = NULL;
        fsim_world* peer = NULL;
        fsim_net_spec ns;
        fsim_dms_received rx;
        const char* all[1];
        const unsigned char raw[4] = {0xFF, 0xFE, 0x00, 0x80};
        uint32_t a, b2, net = 0, dest, mine, sent = 0;
        uint64_t msg = 0, first = 0;
        int32_t reason = -1;
        int step, got = 0;
        all[0] = "*";
        memset(&rx, 0, sizeof rx);
        CHECK(world != NULL);
        a = make_aircraft(world, "wing", 0.0);
        b2 = make_aircraft(world, "lead", 0.02);
        fsim_net_spec_init(&ns);
        ns.name = "l16";
        CHECK(a && b2 && fsim_world_create_net(world, &ns, &net) == FSIM_OK && fsim_world_join_net(world, a, net) == FSIM_OK &&
              fsim_world_join_net(world, b2, net) == FSIM_OK);
        CHECK(fsim_agent_create(world, a, "talker", &talker) == FSIM_OK && fsim_agent_create(world, a, "listener", &listener) == FSIM_OK &&
              fsim_agent_create(world, b2, "peer", &peer) == FSIM_OK);
        dest = fsim_dms_system(peer, b2);
        mine = fsim_dms_system(talker, a);
        CHECK(dest != 0 && mine != 0 && fsim_dms_system(listener, a) == mine);
        /* unbound: either sends */
        CHECK(fsim_dms_send(listener, a, 1001, raw, sizeof raw, &dest, 1, &msg, &reason) == FSIM_OK);
        CHECK(bind(world, 1, a, talker, all, 1, 0, 1) == FSIM_OK && bind(world, 2, a, listener, NULL, 0, 1, 0) == FSIM_OK);
        reason = -1;
        CHECK(fsim_dms_send(listener, a, 1001, raw, sizeof raw, &dest, 1, &msg, &reason) == FSIM_NOT_ALLOWED && reason == FSIM_DMS_NOT_OWNER &&
              strstr(fsim_last_error(), "DMS egress") != NULL);
        CHECK(fsim_dms_send(talker, a, 1001, raw, sizeof raw, &dest, 1, &msg, &reason) == FSIM_OK && msg != 0);
        reason = -1;
        CHECK(fsim_dms_publish(listener, a, FSIM_TOPIC_POSITION_REPORT, 0, NULL, 0, &first, &sent, &reason) == FSIM_NOT_ALLOWED &&
              reason == FSIM_DMS_NOT_OWNER);
        CHECK(fsim_dms_publish(talker, a, FSIM_TOPIC_POSITION_REPORT, 0, NULL, 0, &first, &sent, &reason) == FSIM_OK && sent == 1);
        CHECK(fsim_dms_subscribe(talker, a, 1002, 1) == FSIM_NOT_ALLOWED && strstr(fsim_last_error(), "DMS ingress") != NULL);
        CHECK(fsim_dms_subscribe_all(talker, a, 0) == FSIM_NOT_ALLOWED);
        CHECK(fsim_dms_subscribe(listener, a, 1002, 1) == FSIM_OK);
        CHECK(fsim_dms_send(peer, b2, 1002, raw, sizeof raw, &mine, 1, &msg, &reason) == FSIM_OK);
        for (step = 0; step < 30 && !got; ++step) {
            CHECK(fsim_world_step(world, 1) == FSIM_OK);
            CHECK(fsim_dms_inbox_count(talker, a) == 0); /* (not its side) */
            got = fsim_dms_inbox_count(listener, a) >= 1;
        }
        CHECK(got);
        CHECK(fsim_dms_inbox_get(talker, a, 0, &rx) == FSIM_NOT_ALLOWED && fsim_dms_inbox_take(talker, a, 1) == FSIM_NOT_ALLOWED);
        CHECK(fsim_dms_inbox_count(listener, a) >= 1); /* (a refused take took nothing) */
        CHECK(fsim_dms_inbox_get(listener, a, fsim_dms_inbox_count(listener, a) - 1, &rx) == FSIM_OK && rx.source == dest && rx.topic == 1002 &&
              rx.length == sizeof raw && memcmp(rx.bytes, raw, sizeof raw) == 0);
        CHECK(fsim_dms_inbox_take(listener, a, fsim_dms_inbox_count(listener, a)) == FSIM_OK && fsim_dms_inbox_count(listener, a) == 0);
        /* the picture: every service's alike */
        CHECK(fsim_dms_peer_count(talker, a) == fsim_dms_peer_count(listener, a));
        /* the range's own handle is no service: it still reads and sends as before */
        CHECK(fsim_dms_send(world, a, 1001, raw, sizeof raw, &dest, 1, &msg, &reason) == FSIM_OK);
        fsim_world_destroy(peer);
        fsim_world_destroy(listener);
        fsim_world_destroy(talker);
        fsim_world_destroy(world);
    }

    {
        /* legal reports with their provenance: what the getters give, each with where it came from and when */
        fsim_world* world = make_world("c-ma-reports", jsbsim_root);
        fsim_world* blue = NULL;
        fsim_world* boss = NULL;
        fsim_world* stranger = NULL;
        fsim_net_spec ns;
        fsim_c2_node_spec cs;
        fsim_dissemination_plan plan;
        fsim_ownship own;
        fsim_c2_task order, held;
        fsim_peer_report peer;
        fsim_report_meta meta;
        struct { fsim_report_meta meta; double beyond; } wide[2]; /* (a newer caller's wider record) */
        union { fsim_report_meta meta; unsigned char bytes[sizeof(fsim_report_meta)]; } small; /* (an older caller's, shorter) */
        uint32_t a, other, crc = 0, net = 0, count = 99, n, mine, c2;
        uint64_t task = 0;
        int32_t availability = -1, reason = -1;
        double now, sent;
        int kind;
        CHECK(world != NULL);
        a = make_aircraft(world, "viper", 0.0);
        other = make_aircraft(world, "other", 0.3);
        fsim_ownship_init(&own);
        CHECK(a != 0 && other != 0 && fsim_vehicle_ownship(world, a, &own) == FSIM_OK);
        fsim_c2_node_spec_init(&cs); /* (some 40 km south of it) */
        cs.name = "crc", cs.latitude_rad = own.latitude_rad - 0.0063, cs.longitude_rad = own.longitude_rad, cs.altitude_msl_m = 200.0;
        fsim_net_spec_init(&ns);
        fsim_dissemination_plan_init(&plan);
        plan.status_period_s = 2.0;
        CHECK(fsim_world_create_c2_node(world, &cs, &crc) == FSIM_OK && fsim_world_create_net(world, &ns, &net) == FSIM_OK);
        CHECK(fsim_world_join_net(world, a, net) == FSIM_OK && fsim_world_join_net(world, crc, net) == FSIM_OK &&
              fsim_world_set_dissemination(world, crc, &plan) == FSIM_OK);
        CHECK(fsim_agent_create(world, a, "blue-1", &blue) == FSIM_OK && fsim_agent_create(world, crc, "crc", &boss) == FSIM_OK &&
              fsim_agent_create(world, other, "red-1", &stranger) == FSIM_OK);
        mine = fsim_dms_system(blue, a);
        c2 = fsim_dms_system(boss, crc);
        CHECK(mine != 0 && c2 != 0);
        CHECK(fsim_c2_declare_role(boss, crc, 2, "CRC LEAD", NULL, 0, &reason) == FSIM_OK);
        CHECK(fsim_world_step(world, 360) == FSIM_OK);
        now = fsim_world_time(world);

        /* navigation: one, its own, sampled when its sensors said */
        fsim_ownship_init(&own);
        fsim_report_meta_init(&meta);
        CHECK(meta.struct_size == sizeof meta);
        CHECK(fsim_vehicle_ownship(blue, a, &own) == FSIM_OK);
        CHECK(fsim_agent_report(blue, a, FSIM_REPORT_NAVIGATION, NAN, &availability, NULL, 0, &count) == FSIM_OK);
        CHECK(availability == FSIM_REPORT_AVAILABLE && count == 1); /* (the count alone) */
        CHECK(fsim_agent_report(blue, a, FSIM_REPORT_NAVIGATION, NAN, &availability, &meta, 1, &count) == FSIM_OK && count == 1);
        CHECK(meta.struct_size == sizeof meta && meta.source == FSIM_REPORT_OWN_NAVIGATION && meta.validity == FSIM_REPORT_VALID);
        CHECK(meta.system == 0 && meta.net == 0 && meta.payload == 0xFFFF);
        CHECK(meta.sample_time_s == own.time_s && meta.receive_time_s == now && meta.age_s == now - own.time_s);
        CHECK(fsim_agent_report(blue, a, FSIM_REPORT_NAVIGATION, -1.0, &availability, &meta, 1, &count) == FSIM_OK &&
              meta.validity == FSIM_REPORT_STALE);
        /* its sensors off, its picture empty: available, and none - not an error, not zero-filled */
        for (kind = FSIM_REPORT_TRACKS; kind <= FSIM_REPORT_LINK_TRACKS; ++kind) {
            count = 99, availability = -1;
            CHECK(fsim_agent_report(blue, a, kind, NAN, &availability, NULL, 0, &count) == FSIM_OK);
            CHECK(availability == FSIM_REPORT_AVAILABLE && count == 0);
        }

        /* a task: not there until delivered; then from C2's system, received when it arrived, its sample time unknown */
        fsim_c2_task_init(&order);
        order.type = FSIM_C2_TASK_PATROL;
        order.latitude_rad = own.latitude_rad + 0.005, order.longitude_rad = own.longitude_rad;
        sent = fsim_world_time(world);
        CHECK(fsim_c2_send_task(boss, crc, mine, &order, &task, &reason) == FSIM_OK && task != 0);
        CHECK(fsim_agent_report(blue, a, FSIM_REPORT_TASKS, NAN, &availability, NULL, 0, &count) == FSIM_OK && count == 0);
        CHECK(fsim_world_step(world, 240) == FSIM_OK);
        CHECK(fsim_agent_report(blue, a, FSIM_REPORT_TASKS, 0.0, &availability, &meta, 1, &count) == FSIM_OK);
        CHECK(availability == FSIM_REPORT_AVAILABLE && count == 1 && count == fsim_agent_c2_task_count(blue, a));
        fsim_c2_task_init(&held);
        CHECK(fsim_agent_c2_task_get(blue, a, 0, &held) == FSIM_OK && held.task_id == task && held.state == FSIM_C2_TASK_RECEIVED);
        CHECK(meta.source == FSIM_REPORT_LINK && meta.system == c2 && meta.system == held.from);
        CHECK(meta.receive_time_s == held.received_s && meta.receive_time_s > sent);
        CHECK(isnan(meta.sample_time_s) && isnan(meta.age_s) && meta.validity == FSIM_REPORT_VALID); /* (no age: never stale) */
        CHECK(isnan(held.altitude_m) && isnan(held.radius_m)); /* (unknown stays unknown) */
        /* reading takes nothing: the same again, and the task still there */
        for (n = 0; n < 3; ++n) {
            fsim_report_meta again;
            fsim_report_meta_init(&again);
            CHECK(fsim_agent_report(blue, a, FSIM_REPORT_TASKS, 0.0, &availability, &again, 1, &count) == FSIM_OK && count == 1);
            CHECK(again.system == meta.system && again.receive_time_s == meta.receive_time_s && isnan(again.sample_time_s));
        }
        CHECK(fsim_agent_c2_task_count(blue, a) == 1);

        /* the peer it heard, by its heartbeat alone; the records as far apart as the caller's struct_size says */
        fsim_peer_report_init(&peer);
        CHECK(fsim_dms_peer_count(blue, a) == 1 && fsim_dms_peer_get(blue, a, 0, &peer) == FSIM_OK && !peer.has_position);
        memset(wide, 0, sizeof wide);
        wide[0].meta.struct_size = sizeof wide[0];
        wide[0].beyond = wide[1].beyond = 7.0;
        CHECK(fsim_agent_report(blue, a, FSIM_REPORT_PEERS, NAN, &availability, &wide[0].meta, 2, &count) == FSIM_OK && count == 1);
        CHECK(wide[0].meta.source == FSIM_REPORT_LINK && wide[0].meta.system == peer.system && wide[0].meta.net == net);
        CHECK(wide[0].meta.receive_time_s == peer.received_s && isnan(wide[0].meta.sample_time_s) && isnan(wide[0].meta.age_s));
        CHECK(wide[0].beyond == 7.0 && wide[1].beyond == 7.0 && wide[1].meta.struct_size == 0); /* (nothing written past it) */
        memset(&small, 0xAB, sizeof small);
        small.meta.struct_size = 24; /* (as far as its payload) */
        CHECK(fsim_agent_report(blue, a, FSIM_REPORT_PEERS, NAN, &availability, &small.meta, 1, &count) == FSIM_OK);
        CHECK(small.meta.struct_size == 24 && small.meta.source == FSIM_REPORT_LINK && small.meta.system == peer.system);
        CHECK(small.bytes[24] == 0xAB && small.bytes[sizeof small - 1] == 0xAB); /* (nothing written past its size) */
        /* that heartbeat's own provenance, beside the peer index for index: its net, its delivery, the time it states */
        fsim_report_meta_init(&meta);
        CHECK(fsim_agent_report(blue, a, FSIM_REPORT_PEER_HEARTBEATS, NAN, &availability, &meta, 1, &count) == FSIM_OK);
        CHECK(availability == FSIM_REPORT_AVAILABLE && count == 1 && count == fsim_dms_peer_count(blue, a));
        CHECK(meta.source == FSIM_REPORT_LINK && meta.system == peer.system && meta.net == net && meta.validity == FSIM_REPORT_VALID);
        CHECK(meta.receive_time_s == peer.received_s && meta.sample_time_s <= meta.receive_time_s &&
              meta.age_s == fsim_world_time(world) - meta.sample_time_s);
        /* a smaller buffer than it has: what fits, and the whole count */
        CHECK(fsim_agent_report(blue, a, FSIM_REPORT_AUTHORITY, NAN, &availability, NULL, 0, &count) == FSIM_OK && count == 1);
        CHECK(fsim_agent_report(blue, a, FSIM_REPORT_AUTHORITY, NAN, &availability, &meta, 1, &count) == FSIM_OK &&
              meta.source == FSIM_REPORT_MA_CORE && meta.sample_time_s == fsim_world_time(world));
        CHECK(strcmp(fsim_agent_operator_callsign(blue, a, 0), "CRC LEAD") == 0 && strcmp(fsim_agent_operator_callsign(blue, a, 9), "") == 0);

        /* a C2 node: no navigation, no sensors - not applicable; its links' picture as any platform's */
        CHECK(fsim_agent_report(boss, crc, FSIM_REPORT_NAVIGATION, NAN, &availability, NULL, 0, &count) == FSIM_OK &&
              availability == FSIM_REPORT_NOT_APPLICABLE && count == 0);
        CHECK(fsim_agent_report(boss, crc, FSIM_REPORT_CONTACTS, NAN, &availability, NULL, 0, &count) == FSIM_OK &&
              availability == FSIM_REPORT_NOT_APPLICABLE);
        CHECK(fsim_agent_report(boss, crc, FSIM_REPORT_PEERS, NAN, &availability, NULL, 0, &count) == FSIM_OK &&
              availability == FSIM_REPORT_AVAILABLE);
        /* another platform's: refused, nothing written */
        count = 99, availability = -7;
        CHECK(fsim_agent_report(stranger, a, FSIM_REPORT_NAVIGATION, NAN, &availability, NULL, 0, &count) == FSIM_NOT_ALLOWED);
        CHECK(fsim_agent_report(blue, other, FSIM_REPORT_TASKS, NAN, &availability, NULL, 0, &count) == FSIM_NOT_ALLOWED);
        CHECK(count == 99 && availability == -7);
        CHECK(strcmp(fsim_agent_operator_callsign(stranger, a, 0), "") == 0);
        /* a kind there is none of, a buffer with no size */
        CHECK(fsim_agent_report(blue, a, FSIM_REPORT_PEER_HEARTBEATS + 1, NAN, &availability, NULL, 0, &count) == FSIM_INVALID_ARGUMENT);
        CHECK(fsim_agent_report(blue, a, -1, NAN, &availability, NULL, 0, &count) == FSIM_INVALID_ARGUMENT);
        CHECK(fsim_agent_report(blue, a, FSIM_REPORT_PEERS, NAN, &availability, NULL, 1, &count) == FSIM_INVALID_ARGUMENT);
        CHECK(fsim_agent_report(blue, a, FSIM_REPORT_PEERS, NAN, &availability, NULL, 0, NULL) == FSIM_INVALID_ARGUMENT);
        /* its platform gone: unavailable */
        CHECK(fsim_world_remove_vehicle(world, a) == FSIM_OK);
        CHECK(fsim_agent_report(blue, a, FSIM_REPORT_NAVIGATION, NAN, &availability, NULL, 0, &count) == FSIM_OK &&
              availability == FSIM_REPORT_UNAVAILABLE && count == 0);
        CHECK(fsim_agent_report(blue, a, FSIM_REPORT_TASKS, NAN, &availability, NULL, 0, &count) == FSIM_OK &&
              availability == FSIM_REPORT_UNAVAILABLE && count == 0);
        /* the names are the contract's */
        CHECK(strcmp(fsim_report_availability_name(FSIM_REPORT_NOT_APPLICABLE), "not_applicable") == 0 &&
              strcmp(fsim_report_availability_name(FSIM_REPORT_NOT_OWNER), "not_owner") == 0 && strcmp(fsim_report_availability_name(9), "") == 0);
        CHECK(strcmp(fsim_report_validity_name(FSIM_REPORT_STALE), "stale") == 0 && strcmp(fsim_report_validity_name(FSIM_REPORT_UNKNOWN), "unknown") == 0);
        CHECK(strcmp(fsim_report_source_name(FSIM_REPORT_OWN_NAVIGATION), "own_navigation") == 0 &&
              strcmp(fsim_report_source_name(FSIM_REPORT_LINK), "link") == 0 && strcmp(fsim_report_source_name(-1), "") == 0);
        fsim_world_destroy(stranger);
        fsim_world_destroy(boss);
        fsim_world_destroy(blue);
        fsim_world_destroy(world);
    }

    {
        /* two worlds mint the same numbers: a binding by an agent's handle takes only an agent of the world it is made in */
        fsim_world* one = make_world("c-ma-world-one", jsbsim_root);
        fsim_world* two = make_world("c-ma-world-two", jsbsim_root);
        fsim_world* mine = NULL;
        fsim_world* theirs = NULL;
        fsim_ma_service_binding b;
        const char* hsa[1];
        uint32_t a1, a2, holder = 0;
        int32_t mode = -1, granted = -1;
        hsa[0] = kHsa;
        CHECK(one != NULL && two != NULL);
        a1 = make_aircraft(one, "viper", 0.0);
        a2 = make_aircraft(two, "viper", 0.0);
        CHECK(a1 != 0 && fsim_agent_create(one, a1, "mine", &mine) == FSIM_OK && fsim_agent_create(two, a2, "theirs", &theirs) == FSIM_OK);
        CHECK(a1 == a2 && controller_of(mine) == controller_of(theirs)); /* (indistinguishable by number) */
        fsim_ma_service_binding_init(&b);
        b.service_id = 1, b.command_capabilities = hsa, b.command_capability_count = 1, b.owns_dms_ingress = 1, b.owns_dms_egress = 1;
        CHECK(fsim_world_bind_ma_service_agent(one, theirs, &b) == FSIM_INVALID_ARGUMENT && strstr(fsim_last_error(), "another world") != NULL);
        CHECK(fsim_world_bind_ma_service_agent(one, one, &b) == FSIM_INVALID_ARGUMENT && strstr(fsim_last_error(), "not an agent") != NULL);
        CHECK(fsim_world_bind_ma_service_agent(one, two, &b) == FSIM_INVALID_ARGUMENT);
        CHECK(fsim_world_bind_ma_service_agent(one, NULL, &b) == FSIM_INVALID_ARGUMENT);
        CHECK(fsim_world_bind_ma_service_agent(mine, mine, &b) == FSIM_NOT_ALLOWED); /* (the range's to make, not an agent's) */
        b.controller = controller_of(mine) + 1;
        CHECK(fsim_world_bind_ma_service_agent(one, mine, &b) == FSIM_INVALID_ARGUMENT); /* (not the handle's own) */
        b.controller = 0, b.platform = a1 + 1;
        CHECK(fsim_world_bind_ma_service_agent(one, mine, &b) == FSIM_INVALID_ARGUMENT);
        /* nothing changed in either world */
        CHECK(fsim_world_ma_service_count(one) == 0 && fsim_world_ma_service_count(two) == 0);
        CHECK(fsim_vehicle_control_mode(one, a1, &mode) == FSIM_OK && mode == FSIM_CONTROL_OPEN);
        CHECK(fsim_vehicle_control_holder(one, a1, kHsa, &granted, &holder) == FSIM_OK && granted == 0);
        /* its own agent binds, by the handle alone or with its own numbers said again */
        b.platform = 0;
        CHECK(fsim_world_bind_ma_service_agent(one, mine, &b) == FSIM_OK);
        b.platform = a2, b.controller = controller_of(theirs);
        CHECK(fsim_world_bind_ma_service_agent(two, theirs, &b) == FSIM_OK);
        CHECK(fsim_world_ma_service_count(one) == 1 && fsim_world_ma_service_count(two) == 1);
        CHECK(fsim_world_ma_capability_owner(one, a1, kHsa) == controller_of(mine));
        CHECK(fsim_vehicle_control_holder(one, a1, kHsa, &granted, &holder) == FSIM_OK && granted == 1 && holder == controller_of(mine));
        fsim_world_destroy(theirs);
        fsim_world_destroy(mine);
        fsim_world_destroy(two);
        fsim_world_destroy(one);
    }
    {
        /* what stood before a binding: another agent's grant is taken back, the owner granted, the platform kept under grants */
        fsim_world* world = make_world("c-ma-transition", jsbsim_root);
        fsim_world* legacy = NULL;
        fsim_world* owner = NULL;
        fsim_command_result cr;
        const char* hsa[1];
        uint32_t a, holder = 0;
        int32_t reason = -1, granted = 0, mode = -1;
        hsa[0] = kHsa;
        CHECK(world != NULL);
        a = make_aircraft(world, "viper", 0.0);
        CHECK(a != 0 && fsim_agent_create(world, a, "legacy", &legacy) == FSIM_OK && fsim_agent_create(world, a, "owner", &owner) == FSIM_OK);
        CHECK(fsim_vehicle_request_control_by(legacy, a, kHsa, 0, &reason) == FSIM_OK && reason == 0);
        CHECK(fsim_vehicle_request_control_by(legacy, a, kRoute, 0, &reason) == FSIM_OK && reason == 0); /* (left unowned) */
        CHECK(fly(legacy, a, 1.0, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
        CHECK(fsim_vehicle_control_holder(world, a, kHsa, &granted, &holder) == FSIM_OK && granted == 1 && holder == controller_of(legacy));
        CHECK(bind(world, 1, a, owner, hsa, 1, 1, 1) == FSIM_OK);
        CHECK(fsim_world_ma_capability_owner(world, a, kHsa) == controller_of(owner));
        CHECK(fsim_vehicle_control_holder(world, a, kHsa, &granted, &holder) == FSIM_OK && granted == 1 && holder == controller_of(owner));
        CHECK(fsim_vehicle_control_holder(world, a, kRoute, &granted, &holder) == FSIM_OK && granted == 0 && holder == 0);
        CHECK(fly(legacy, a, 2.0, &cr) != FSIM_OK || cr.status != FSIM_COMMAND_ACCEPTED);
        CHECK(fsim_vehicle_request_control_by(legacy, a, kHsa, 0, &reason) == FSIM_OK && strcmp(fsim_reason_name(reason), "not_allowed") == 0);
        CHECK(fsim_vehicle_request_control_by(legacy, a, kRoute, 0, &reason) == FSIM_OK && strcmp(fsim_reason_name(reason), "not_allowed") == 0);
        CHECK(fly(owner, a, 1.0, &cr) == FSIM_OK && cr.status == FSIM_COMMAND_ACCEPTED);
        /* not the default policy either, nor a controller of the range's choosing; and the platform stays under grants */
        CHECK(fsim_vehicle_request_control(world, a, kHsa, &reason) == FSIM_OK && strcmp(fsim_reason_name(reason), "not_allowed") == 0);
        CHECK(fsim_vehicle_request_control_by(world, a, kRoute, 7, &reason) == FSIM_OK && strcmp(fsim_reason_name(reason), "not_allowed") == 0);
        CHECK(fsim_vehicle_set_control_mode(world, a, FSIM_CONTROL_OPEN) == FSIM_NOT_ALLOWED);
        CHECK(fsim_vehicle_control_mode(world, a, &mode) == FSIM_OK && mode == FSIM_CONTROL_GRANTED);
        CHECK(fsim_vehicle_set_control_mode(world, a, FSIM_CONTROL_GRANTED) == FSIM_OK);
        CHECK(fsim_vehicle_control_holder(world, a, kHsa, &granted, &holder) == FSIM_OK && granted == 1 && holder == controller_of(owner));
        fsim_world_destroy(owner);
        fsim_world_destroy(legacy);
        fsim_world_destroy(world);
    }

    printf("ma c abi ok\n");
    return 0;
}
