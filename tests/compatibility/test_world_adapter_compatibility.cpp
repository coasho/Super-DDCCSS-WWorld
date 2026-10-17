// A4: public World, Scenario and Environment contracts before adapter extraction.
#include "fsim/fsim_c.h"

#include <cmath>
#include <cstddef>
#include <cstring>
#include <iostream>
#include <limits>
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

struct Scenario {
    fsim_scenario* value = nullptr;
    ~Scenario() { fsim_scenario_destroy(value); }
};

int run() {
    const char* json = R"({
        "world": {"name": "world-adapter-scenario", "publish": false,
                  "workers": 1, "pin_workers": false, "terrain": false,
                  "jsbsim_root": "borrowed/root", "terrain_url": "borrowed/tiles",
                  "record_path": "borrowed/record", "terrain_offline": true},
        "environment": {"wind": {"direction_deg": 180, "speed_ms": 5}},
        "vehicles": [{"name": "pair", "count": 2,
                      "initial": {"alt_msl_m": 1200, "airspeed_ms": 60},
                      "command": {"level": "attitude", "pitch_deg": 3}}]
    })";
    Scenario scenario;
    CHECK(fsim_scenario_parse(json, "compatibility.json", &scenario.value) == FSIM_OK);
    CHECK(fsim_scenario_vehicle_count(scenario.value) == 2);
    CHECK(fsim_scenario_vehicle_count(nullptr) == 0);

    fsim_world_options borrowed;
    fsim_world_options_init(&borrowed);
    CHECK(fsim_scenario_world_options(scenario.value, &borrowed) == FSIM_OK);
    const char* name = borrowed.name;
    const char* root = borrowed.jsbsim_root;
    const char* tiles = borrowed.terrain_url;
    const char* record = borrowed.record_path;
    Scenario other;
    CHECK(fsim_scenario_parse("{}", "other.json", &other.value) == FSIM_OK);
    fsim_world_options other_options;
    fsim_world_options_init(&other_options);
    CHECK(fsim_scenario_world_options(other.value, &other_options) == FSIM_OK);
    CHECK(std::strcmp(name, "world-adapter-scenario") == 0);
    CHECK(std::strcmp(root, "borrowed/root") == 0);
    CHECK(std::strcmp(tiles, "borrowed/tiles") == 0);
    CHECK(std::strcmp(record, "borrowed/record") == 0);

    // Older callers receive only the option fields their declared size reaches.
    fsim_world_options versioned;
    fsim_world_options_init(&versioned);
    versioned.struct_size = static_cast<uint32_t>(offsetof(fsim_world_options, terrain_offline));
    versioned.terrain_offline = 71;
    versioned.store_drag = 72;
    CHECK(fsim_scenario_world_options(scenario.value, &versioned) == FSIM_OK);
    CHECK(versioned.struct_size == offsetof(fsim_world_options, terrain_offline));
    CHECK(std::strcmp(versioned.name, name) == 0 && versioned.terrain_offline == 71 && versioned.store_drag == 72);
    versioned.struct_size = static_cast<uint32_t>(offsetof(fsim_world_options, terrain_offline) + sizeof(int32_t));
    CHECK(fsim_scenario_world_options(scenario.value, &versioned) == FSIM_OK);
    CHECK(versioned.terrain_offline == 1 && versioned.store_drag == 72);
    versioned.struct_size = sizeof(versioned);
    CHECK(fsim_scenario_world_options(scenario.value, &versioned) == FSIM_OK);
    CHECK(versioned.store_drag == 1);
    versioned.struct_size = static_cast<uint32_t>(offsetof(fsim_world_options, record_interval_s));
    versioned.name = "untouched";
    CHECK(fsim_scenario_world_options(scenario.value, &versioned) == FSIM_INVALID_ARGUMENT);
    CHECK(std::strcmp(versioned.name, "untouched") == 0);

    World world;
    fsim_world_options options;
    fsim_world_options_init(&options);
    options.name = "world-adapter-compatibility";
    options.publish = 0;
    options.terrain = 0;
    options.workers = 1;
    options.pin_workers = 0;
    CHECK(fsim_world_create(&options, &world.value) == FSIM_OK);

    // Direct Environment success and short-struct failures preserve LastError.
    CHECK(fsim_scenario_parse(nullptr, nullptr, &other.value) == FSIM_INVALID_ARGUMENT);
    const std::string previous_error = fsim_last_error();
    CHECK(!previous_error.empty());
    fsim_environment environment;
    fsim_environment_init(&environment);
    CHECK(environment.struct_size == sizeof(environment));
    CHECK(environment.time_factor == 1.0 && environment.temperature_sl_k == 288.15);
    CHECK(environment.pressure_sl_pa == 101325.0 && environment.visibility_m == 50000.0);
    CHECK(fsim_world_get_environment(world.value, &environment) == FSIM_OK);
    environment.wind_speed_ms = 11.0;
    CHECK(fsim_world_set_environment(world.value, &environment) == FSIM_OK);
    environment.struct_size = sizeof(environment) - 1;
    environment.wind_speed_ms = 99.0;
    CHECK(fsim_world_get_environment(world.value, &environment) == FSIM_INVALID_ARGUMENT);
    CHECK(environment.wind_speed_ms == 99.0);
    CHECK(fsim_world_set_environment(world.value, &environment) == FSIM_INVALID_ARGUMENT);
    fsim_environment_init(&environment);
    CHECK(fsim_world_get_environment(world.value, &environment) == FSIM_OK);
    CHECK(environment.wind_speed_ms == 11.0);
    CHECK(std::strcmp(fsim_last_error(), previous_error.c_str()) == 0);

    // Validation precedes count clearing; a successful truncated copy reports all creations.
    size_t count = 71;
    CHECK(fsim_scenario_apply(nullptr, scenario.value, nullptr, 0, &count) == FSIM_INVALID_ARGUMENT);
    CHECK(count == 71);
    uint32_t ids[2] = {0, 0x12345678u};
    CHECK(fsim_scenario_apply(world.value, scenario.value, ids, 1, &count) == FSIM_OK);
    CHECK(count == 2 && ids[0] != 0 && ids[1] == 0x12345678u);
    CHECK(std::strlen(fsim_last_error()) == 0);
    CHECK(fsim_world_vehicle_count(world.value) == 2);
    CHECK(fsim_world_find_vehicle(world.value, "pair-1") == ids[0]);
    CHECK(fsim_world_find_vehicle(world.value, "pair-2") != 0);
    CHECK(fsim_vehicle_active_level(world.value, ids[0]) == FSIM_LEVEL_ATTITUDE);
    CHECK(fsim_world_get_environment(world.value, &environment) == FSIM_OK);
    CHECK(environment.wind_direction_deg == 180.0 && environment.wind_speed_ms == 5.0);
    CHECK(std::strcmp(name, "world-adapter-scenario") == 0);

    CHECK(fsim_world_terrain(world.value, 0, nullptr, nullptr, nullptr) == 0);
    double lat = 0.1, lon = 0.2, height = -1.0;
    CHECK(fsim_world_terrain(world.value, 1, nullptr, &lon, &height) == FSIM_INVALID_ARGUMENT);
    CHECK(height == -1.0);
    CHECK(fsim_world_terrain(world.value, 1, &lat, &lon, &height) == 1 && height == 0.0);
    World offline;
    options.name = "world-adapter-offline";
    options.terrain = 1;
    options.terrain_offline = 1;
    options.terrain_url = "https://world-adapter.invalid/t/{z}/{x}/{y}.png";
    CHECK(fsim_world_create(&options, &offline.value) == FSIM_OK);
    CHECK(fsim_world_terrain(offline.value, 1, &lat, &lon, &height) == 0 && std::isnan(height));

    fsim_magnetic_field field;
    fsim_magnetic_field_init(&field);
    CHECK(field.struct_size == sizeof(field) && field.reserved == 0);
    CHECK(std::isnan(field.north_nt) && std::isnan(field.inclination_rad));
    field.struct_size = static_cast<uint32_t>(offsetof(fsim_magnetic_field, east_nt));
    field.east_nt = 71.0;
    CHECK(fsim_magnetic_field_at(0.1, 0.2, 0.0, 2025.0, &field) == FSIM_OK);
    CHECK(field.struct_size == offsetof(fsim_magnetic_field, east_nt));
    CHECK(std::isfinite(field.north_nt) && field.east_nt == 71.0);
    const double north = field.north_nt;
    CHECK(fsim_magnetic_field_at(std::numeric_limits<double>::quiet_NaN(), 0.2, 0.0, 2025.0, &field) == FSIM_INVALID_ARGUMENT);
    CHECK(field.north_nt == north && field.east_nt == 71.0);
    field.struct_size = sizeof(uint32_t) - 1;
    CHECK(fsim_magnetic_field_at(0.1, 0.2, 0.0, 2025.0, &field) == FSIM_INVALID_ARGUMENT);
    CHECK(fsim_world_magnetic_year(world.value) == 2025.0);
    CHECK(std::isnan(fsim_world_magnetic_year(nullptr)));

    World agent;
    CHECK(fsim_agent_create(world.value, ids[0], "world-adapter-agent", &agent.value) == FSIM_OK);
    uint32_t vehicle = 0, controller = 0;
    const char* agent_name = nullptr;
    CHECK(fsim_agent_identity(agent.value, &vehicle, &controller, &agent_name) == 1);
    CHECK(vehicle == ids[0] && controller != 0 && std::strcmp(agent_name, "world-adapter-agent") == 0);
    CHECK(fsim_world_get_environment(agent.value, nullptr) == FSIM_NOT_ALLOWED);
    CHECK(fsim_world_set_environment(agent.value, nullptr) == FSIM_NOT_ALLOWED);
    count = 72;
    CHECK(fsim_scenario_apply(agent.value, nullptr, nullptr, 0, &count) == FSIM_NOT_ALLOWED);
    CHECK(count == 72);
    CHECK(std::isnan(fsim_world_magnetic_year(agent.value)));
    CHECK(std::strstr(fsim_last_error(), "fsim_world_magnetic_year") != nullptr);
    CHECK(fsim_world_terrain(agent.value, 0, nullptr, nullptr, nullptr) == 0);
    return 0;
}

} // namespace

int main() { return run(); }
