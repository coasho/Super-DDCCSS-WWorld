// C ABI Scenario adapter over session::World.

#include "fsim/Scenario.h"
#include "fsim/fsim_c.h"

#include "sdk/CAbiSupport.h"
#include "sdk/Handles.h"
#include "session/Scenario.h"

#include <cstddef>
#include <cstdint>

// --- Scenarios -------------------------------------------------------------------------

struct fsim_scenario {
    fsim::Scenario scenario;
};

FSIM_API int fsim_scenario_load(const char* path, fsim_scenario** out) {
    if (!path || !out) return fail(FSIM_INVALID_ARGUMENT, "fsim_scenario_load: bad arguments");
    *out = nullptr;
    return guard("fsim_scenario_load", [&] {
        *out = new fsim_scenario{fsim::loadScenario(path)};
        return FSIM_OK;
    });
}

FSIM_API int fsim_scenario_parse(const char* json, const char* source_name, fsim_scenario** out) {
    if (!json || !out) return fail(FSIM_INVALID_ARGUMENT, "fsim_scenario_parse: bad arguments");
    *out = nullptr;
    return guard("fsim_scenario_parse", [&] {
        *out = new fsim_scenario{fsim::parseScenario(json, source_name ? source_name : "scenario")};
        return FSIM_OK;
    });
}

FSIM_API void fsim_scenario_destroy(fsim_scenario* scenario) { delete scenario; }

FSIM_API int fsim_scenario_world_options(const fsim_scenario* scenario, fsim_world_options* out) {
    constexpr std::size_t kBeforeOffline = offsetof(fsim_world_options, record_interval_s) + sizeof(double);
    if (!scenario || !out || out->struct_size < kBeforeOffline) return fail(FSIM_INVALID_ARGUMENT, "fsim_scenario_world_options: bad arguments");
    const auto& o = scenario->scenario.world;
    out->name = o.name.c_str();
    out->dt = o.dt;
    out->frame_skip = o.frameSkip;
    out->workers = o.workers;
    out->pin_workers = o.pinWorkers ? 1 : 0;
    out->seed = o.seed;
    out->capacity = o.capacity;
    out->publish = o.publish ? 1 : 0;
    out->publish_interval_s = o.publishIntervalSeconds;
    out->jsbsim_root = o.jsbsimRoot.empty() ? nullptr : o.jsbsimRoot.c_str();
    out->terrain = o.terrain ? 1 : 0;
    out->terrain_url = o.terrainUrl.empty() ? nullptr : o.terrainUrl.c_str();
    out->terrain_zoom = o.terrainZoom;
    out->record_path = o.recordPath.empty() ? nullptr : o.recordPath.c_str();
    out->record_interval_s = o.recordIntervalSeconds;
    if (out->struct_size >= offsetof(fsim_world_options, terrain_offline) + sizeof(int32_t)) out->terrain_offline = o.terrainOffline ? 1 : 0;
    if (out->struct_size >= offsetof(fsim_world_options, store_drag) + sizeof(int32_t)) out->store_drag = o.storeDrag ? 1 : 0;
    return FSIM_OK;
}

FSIM_API uint32_t fsim_scenario_vehicle_count(const fsim_scenario* scenario) {
    if (!scenario) return 0;
    uint32_t n = 0;
    for (const auto& v : scenario->scenario.vehicles) n += v.count;
    return n;
}

FSIM_API int fsim_scenario_apply(fsim_world* world, const fsim_scenario* scenario, uint32_t* ids, size_t capacity, size_t* count) {
    FSIM_NOT_AGENT(world, FSIM_NOT_ALLOWED);
    if (!world || !scenario) return fail(FSIM_INVALID_ARGUMENT, "fsim_scenario_apply: bad arguments");
    if (count) *count = 0;
    return guard("fsim_scenario_apply", [&] {
        const auto created = fsim::session::applyScenario(world->world, scenario->scenario);
        if (ids)
            for (size_t i = 0; i < created.size() && i < capacity; ++i) ids[i] = created[i];
        if (count) *count = created.size();
        return FSIM_OK;
    });
}
