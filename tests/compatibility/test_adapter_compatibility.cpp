// A3: public C control and Mission adapter behavior before source extraction.
#include "fsim/fsim_c.h"

#include <cstring>
#include <iostream>
#include <string>

namespace {

#define CHECK(condition) do { if (!(condition)) { \
    std::cerr << __FILE__ << ':' << __LINE__ << ": " #condition \
              << " (" << fsim_last_error() << ")\n"; return 1; \
} } while (false)

struct World {
    fsim_world* value = nullptr;
    ~World() { fsim_world_destroy(value); }
};

int run() {
    World world;
    fsim_world_options options;
    fsim_world_options_init(&options);
    options.name = "adapter-compatibility";
    options.publish = 0;
    options.terrain = 0;
    options.workers = 1;
    options.pin_workers = 0;
    CHECK(fsim_world_create(&options, &world.value) == FSIM_OK);

    fsim_vehicle_spec spec;
    fsim_vehicle_spec_init(&spec);
    spec.name = "viper";
    spec.type = "jsbsim:f16c";
    spec.latitude_deg = 40.0;
    spec.altitude_msl_m = 3000.0;
    spec.airspeed_ms = 160.0;
    uint32_t id = 0;
    CHECK(fsim_world_create_vehicle(world.value, &spec, &id) == FSIM_OK);
    CHECK(id != 0);
    CHECK(fsim_vehicle_payload_count(world.value, id) == 6);

    fsim_command_options control_options;
    fsim_command_options_init(&control_options);
    control_options.command_id = 7101;
    const double actuator[8] = {0.0, 0.0, 0.0, 0.5, 0.0, 1.0, 0.0, 0.0};
    fsim_command_result control{};
    CHECK(fsim_vehicle_submit(world.value, id, FSIM_LEVEL_ACTUATOR, actuator, 8, &control_options, &control) == FSIM_OK);
    fsim_command_detail detail;
    fsim_command_detail_init(&detail);
    CHECK(fsim_last_command_detail(world.value, &detail) == FSIM_OK);
    CHECK(detail.command_id == 7101);
    const int32_t original_reason = detail.reason;

    // A missing control output is rejected before submission, leaving the last answer intact.
    control_options.command_id = 7102;
    CHECK(fsim_vehicle_submit(world.value, id, FSIM_LEVEL_ACTUATOR, actuator, 8, &control_options, nullptr) == FSIM_INVALID_ARGUMENT);
    fsim_command_detail_init(&detail);
    CHECK(fsim_last_command_detail(world.value, &detail) == FSIM_OK);
    CHECK(detail.command_id == 7101 && detail.reason == original_reason);

    fsim_ms_options mission_options;
    fsim_ms_options_init(&mission_options);
    mission_options.command_id = 8201;
    const uint32_t before = fsim_vehicle_ms_activity_count(world.value, id);
    CHECK(before == 0);
    CHECK(fsim_vehicle_payload_state(world.value, id, 0, FSIM_PAYLOAD_STANDBY, &mission_options, nullptr) == FSIM_OK);
    CHECK(fsim_vehicle_ms_activity_count(world.value, id) == before + 1);
    fsim_ms_activity_info mission_activity;
    fsim_ms_activity_info_init(&mission_activity);
    CHECK(fsim_vehicle_ms_activity(world.value, id, 0, &mission_activity) == FSIM_OK);
    CHECK(mission_activity.command_id == 8201);
    const fsim_activity_id first_mission_activity = mission_activity.id;
    fsim_command_detail_init(&detail);
    CHECK(fsim_last_command_detail(world.value, &detail) == FSIM_OK);
    CHECK(detail.command_id == 7101); // Mission does not overwrite control detail.

    // A too-short result is diagnosed after Mission submits its command.
    mission_options.command_id = 8202;
    fsim_ms_result short_result{};
    short_result.struct_size = 1;
    CHECK(fsim_vehicle_payload_state(world.value, id, 0, FSIM_PAYLOAD_OFF, &mission_options, &short_result) == FSIM_INVALID_ARGUMENT);
    CHECK(fsim_vehicle_ms_activity_count(world.value, id) == before + 2);
    fsim_ms_activity_info_init(&mission_activity);
    CHECK(fsim_vehicle_ms_activity(world.value, id, 0, &mission_activity) == FSIM_OK);
    CHECK(mission_activity.command_id == 8202);
    fsim_ms_activity_info_init(&mission_activity);
    CHECK(fsim_vehicle_ms_activity(world.value, id, 1, &mission_activity) == FSIM_OK);
    CHECK(mission_activity.id == first_mission_activity && mission_activity.command_id == 8201);
    fsim_command_detail_init(&detail);
    CHECK(fsim_last_command_detail(world.value, &detail) == FSIM_OK);
    CHECK(detail.command_id == 7101);

    // Direct control success preserves a previous LastError; guarded success clears it.
    CHECK(fsim_vehicle_submit(world.value, id, FSIM_LEVEL_ACTUATOR, actuator, 8, &control_options, nullptr) == FSIM_INVALID_ARGUMENT);
    const std::string previous_error = fsim_last_error();
    CHECK(!previous_error.empty());
    const fsim_actuator_command direct{0.0, 0.0, 0.0, 0.5, 0.0, 1.0, 0.0, 0.0};
    CHECK(fsim_vehicle_command_actuator(world.value, id, &direct) == FSIM_OK);
    CHECK(std::strcmp(fsim_last_error(), previous_error.c_str()) == 0);
    const fsim_behavior_command guarded{"hold", 0, nullptr, nullptr, 0, nullptr, 0};
    CHECK(fsim_vehicle_submit_behavior(world.value, id, &guarded, &control_options, &control) == FSIM_OK);
    CHECK(std::strlen(fsim_last_error()) == 0);

    fsim_world* agent = nullptr;
    CHECK(fsim_agent_create(world.value, id, "compat-agent", &agent) == FSIM_OK);
    World agent_owner{agent};
    CHECK(fsim_vehicle_payload_count(agent, id) == 6);
    CHECK(fsim_vehicle_payload_count(agent, id + 1) == 0);
    uint32_t agent_vehicle = 0, agent_controller = 0;
    const char* agent_name = nullptr;
    CHECK(fsim_agent_identity(agent, &agent_vehicle, &agent_controller, &agent_name) == 1);
    CHECK(agent_vehicle == id && agent_controller != 0 && std::strcmp(agent_name, "compat-agent") == 0);
    CHECK(fsim_vehicle_submit(agent, id, FSIM_LEVEL_ACTUATOR, actuator, 8, &control_options, &control) == FSIM_OK);
    fsim_command_options forged_control = control_options;
    forged_control.source = FSIM_SOURCE_OVERRIDE;
    CHECK(fsim_vehicle_submit(agent, id, FSIM_LEVEL_ACTUATOR, actuator, 8, &forged_control, &control) == FSIM_NOT_ALLOWED);
    fsim_ms_options forged_mission;
    fsim_ms_options_init(&forged_mission);
    forged_mission.source = FSIM_SOURCE_AUTOPILOT;
    fsim_ms_result denied_mission;
    fsim_ms_result_init(&denied_mission);
    CHECK(fsim_vehicle_payload_state(agent, id, 0, FSIM_PAYLOAD_STANDBY, &forged_mission, &denied_mission) == FSIM_NOT_ALLOWED);
    CHECK(fsim_vehicle_payload_fail(agent, id, 0, 1) == FSIM_NOT_ALLOWED);
    CHECK(fsim_last_command_detail(agent, &detail) == FSIM_OK);
    return 0;
}

} // namespace

int main() { return run(); }
