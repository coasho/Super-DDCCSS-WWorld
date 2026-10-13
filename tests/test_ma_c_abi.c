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

    printf("ma c abi ok\n");
    return 0;
}
