// C ABI adapter for the vectorized environment.
#include "sdk/VecEnvHandles.h"
#include "core/Log.h"
#include "sdk/LastError.h"
#include <cstring>
#include <exception>
#include <functional>
#include <stdexcept>
#include <string>

namespace {
void setError(const std::string& message) {
    fsim::sdk::lastError() = message;
    LOG_ERROR("sdk") << message;
}

int guard(const char* what, const std::function<void()>& fn) noexcept {
    try {
        fn();
        fsim::sdk::lastError().clear();
        return FSIM_OK;
    } catch (const std::invalid_argument& e) {
        setError(std::string(what) + ": " + e.what());
        return FSIM_INVALID_ARGUMENT;
    } catch (const std::exception& e) {
        setError(std::string(what) + ": " + e.what());
        return FSIM_ERROR;
    } catch (...) {
        setError(std::string(what) + ": unknown error");
        return FSIM_ERROR;
    }
}

fsim::env::Scenario toScenario(const fsim_options& o) {
    fsim::env::Scenario s;
    if (o.aircraft) s.aircraft = o.aircraft;
    if (o.jsbsim_root) s.jsbsimRoot = o.jsbsim_root;
    s.vehiclesPerEnv = o.vehicles_per_env;
    s.dt = o.dt;
    s.frameSkip = o.frame_skip;
    s.maxEpisodeSteps = o.max_episode_steps;
    if (o.task) s.task = o.task;
    if (o.observation) s.observation = o.observation;
    if (o.action) s.action = o.action;
    auto& ic = s.initial;
    ic.centre.latitudeDeg = o.latitude_deg;
    ic.centre.longitudeDeg = o.longitude_deg;
    ic.centre.altitudeMslM = o.altitude_m;
    ic.centre.headingDeg = o.heading_deg;
    ic.centre.airspeedTrueMs = o.airspeed_ms;
    ic.latitudeJitterDeg = o.latitude_jitter_deg;
    ic.longitudeJitterDeg = o.longitude_jitter_deg;
    ic.altitudeJitterM = o.altitude_jitter_m;
    ic.headingJitterDeg = o.heading_jitter_deg;
    ic.airspeedJitterMs = o.airspeed_jitter_ms;
    s.targetAltitudeDeltaM = o.target_altitude_delta_m;
    s.targetHeadingDeltaDeg = o.target_heading_delta_deg;
    if (o.world_name) s.worldName = o.world_name;
    return s;
}

} // namespace

extern "C" {
FSIM_API void fsim_options_init(fsim_options* o) {
    if (!o) return;
    std::memset(o, 0, sizeof(*o));
    o->struct_size = sizeof(fsim_options);
    o->num_envs = 1;
    o->vehicles_per_env = 1;
    o->aircraft = "c172x";
    o->task = "altitude_heading_hold";
    o->observation = "state";
    o->action = "surfaces";
    o->dt = 1.0 / 120.0;
    o->frame_skip = 4;
    o->max_episode_steps = 2000;
    o->latitude_deg = 37.6188;
    o->longitude_deg = -122.375;
    o->altitude_m = 1500.0;
    o->airspeed_ms = 60.0;
    o->latitude_jitter_deg = 0.02;
    o->longitude_jitter_deg = 0.02;
    o->altitude_jitter_m = 150.0;
    o->heading_jitter_deg = 180.0;
    o->airspeed_jitter_ms = 5.0;
    o->target_altitude_delta_m = 300.0;
    o->target_heading_delta_deg = 60.0;
    o->world_name = "vecenv";
    o->publish = 1;
}

FSIM_API int fsim_vecenv_set_action_ranges(fsim_vecenv* env, const char* mode) {
    if (!env || !mode) return FSIM_INVALID_ARGUMENT;
    if (!env->env.setActionRanges(mode)) {
        setError(std::string("fsim_vecenv_set_action_ranges: \"fixed\" or \"aircraft\", not '") + mode + "'");
        return FSIM_INVALID_ARGUMENT;
    }
    return FSIM_OK;
}

FSIM_API int fsim_vecenv_create(const fsim_options* options, fsim_vecenv** out) {
    if (!options || !out || options->struct_size < sizeof(fsim_options)) {
        setError("fsim_vecenv_create: bad arguments (call fsim_options_init first)");
        return FSIM_INVALID_ARGUMENT;
    }
    *out = nullptr;
    return guard("fsim_vecenv_create", [&] {
        fsim::env::VecEnv::Options o;
        o.numEnvs = options->num_envs;
        o.seed = options->seed;
        o.workers = options->workers;
        o.publish = options->publish != 0;
        o.terrain = options->terrain != 0;
        if (options->scenario_path) o.scenarioPath = options->scenario_path;
        auto* h = new fsim_vecenv(toScenario(*options), o);
        h->last = h->env.reset();
        *out = h;
    });
}

FSIM_API void fsim_vecenv_destroy(fsim_vecenv* env) { delete env; }

FSIM_API int fsim_vecenv_reset(fsim_vecenv* env, uint64_t seed) {
    if (!env) return FSIM_INVALID_ARGUMENT;
    return guard("fsim_vecenv_reset", [&] { env->last = env->env.reset(seed); });
}

FSIM_API int fsim_vecenv_step(fsim_vecenv* env, const float* actions, size_t count) {
    if (!env || !actions) return FSIM_INVALID_ARGUMENT;
    return guard("fsim_vecenv_step", [&] { env->last = env->env.step(fsim::Span<const float>(actions, count)); });
}

FSIM_API int fsim_vecenv_buffers(const fsim_vecenv* env, fsim_buffers* out) {
    if (!env || !out || out->struct_size < sizeof(fsim_buffers)) return FSIM_INVALID_ARGUMENT;
    out->num_envs = env->env.numEnvs();
    out->vehicles_per_env = env->env.vehiclesPerEnv();
    out->observation_size = static_cast<uint32_t>(env->env.observationSize());
    out->action_size = static_cast<uint32_t>(env->env.actionSize());
    out->observations = env->last.observations.data();
    out->rewards = env->last.rewards.data();
    out->terminated = env->last.terminated.data();
    out->truncated = env->last.truncated.data();
    out->final_observations = env->env.finalObservations().data();
    out->episode_steps = env->last.episodeSteps.data();
    out->agent_step_seconds = env->env.agentStepSeconds();
    return FSIM_OK;
}

FSIM_API const char* fsim_vecenv_observation_name(const fsim_vecenv* env, uint32_t i) {
    if (!env || i >= env->env.observationNames().size()) return "";
    return env->env.observationNames()[i].c_str();
}

FSIM_API const char* fsim_vecenv_action_name(const fsim_vecenv* env, uint32_t i) {
    if (!env || i >= env->env.actionNames().size()) return "";
    return env->env.actionNames()[i].c_str();
}

FSIM_API uint64_t fsim_vecenv_vehicle_steps(const fsim_vecenv* env) { return env ? env->env.vehicleSteps() : 0; }

FSIM_API int fsim_vecenv_set_autoreset(fsim_vecenv* env, int mode) {
    if (!env || (mode != FSIM_AUTORESET_NEXT_STEP && mode != FSIM_AUTORESET_SAME_STEP)) {
        setError("fsim_vecenv_set_autoreset: mode must be FSIM_AUTORESET_NEXT_STEP or FSIM_AUTORESET_SAME_STEP");
        return FSIM_INVALID_ARGUMENT;
    }
    env->env.setAutoReset(mode == FSIM_AUTORESET_SAME_STEP ? fsim::env::VecEnv::AutoReset::SameStep
                                                           : fsim::env::VecEnv::AutoReset::NextStep);
    return FSIM_OK;
}

FSIM_API int fsim_vecenv_autoreset(const fsim_vecenv* env) {
    return env && env->env.autoReset() == fsim::env::VecEnv::AutoReset::SameStep ? FSIM_AUTORESET_SAME_STEP
                                                                                 : FSIM_AUTORESET_NEXT_STEP;
}

FSIM_API uint32_t fsim_vecenv_vehicle_ids(const fsim_vecenv* env, uint32_t* ids, uint32_t capacity) {
    if (!env) return 0;
    const std::size_t n = env->env.numVehicles();
    if (ids)
        for (std::size_t i = 0; i < n && i < capacity; ++i) ids[i] = env->env.vehicleId(i);
    return static_cast<uint32_t>(n);
}

} // extern "C"

namespace fsim {
extern "C" FSIM_API fsim_world* fsim_vecenv_world(fsim_vecenv* env) {
    if (!env) return nullptr;
    if (!env->world) env->world = std::make_unique<fsim_world>(env->env.world());
    return env->world.get();
}

} // namespace fsim
