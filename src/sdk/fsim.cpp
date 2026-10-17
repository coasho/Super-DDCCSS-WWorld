// fsim.dll: C ABI + C++ SDK over env::VecEnv (design 4.4, 9.1, 9.2).

#include "fsim/VecEnv.h"
#include "fsim/World.h"
#include "fsim/fsim_c.h"

#include "foundation/core/Log.h"
#include "sdk/VecEnvHandles.h"
#include "sdk/LastError.h"
#include "training/native/vecenv/Registry.h"
#include "training/native/vecenv/VecEnv.h"

#include <cstring>
#include <exception>
#include <functional>
#include <stdexcept>
#include <string>

#ifndef FSIM_VERSION_STRING
#    define FSIM_VERSION_STRING "0.0.0"
#endif

// ---------------------------------------------------------------------------
// Shared implementation: a C handle *is* the environment object.
// ---------------------------------------------------------------------------
namespace fsim::sdk {
std::string& lastError() noexcept {
    thread_local std::string t_lastError;
    return t_lastError;
}
} // namespace fsim::sdk

namespace {

fsim_options toOptions(const fsim::VecEnvOptions& o) {
    fsim_options c;
    fsim_options_init(&c);
    c.num_envs = o.numEnvs;
    c.vehicles_per_env = o.vehiclesPerEnv;
    c.workers = o.workers;
    c.seed = o.seed;
    c.aircraft = o.aircraft.c_str();
    c.jsbsim_root = o.jsbsimRoot.empty() ? nullptr : o.jsbsimRoot.c_str();
    c.task = o.task.c_str();
    c.observation = o.observation.c_str();
    c.action = o.action.c_str();
    c.dt = o.dt;
    c.frame_skip = o.frameSkip;
    c.max_episode_steps = o.maxEpisodeSteps;
    c.latitude_deg = o.latitudeDeg; c.longitude_deg = o.longitudeDeg; c.altitude_m = o.altitudeM;
    c.heading_deg = o.headingDeg; c.airspeed_ms = o.airspeedMs;
    c.latitude_jitter_deg = o.latitudeJitterDeg; c.longitude_jitter_deg = o.longitudeJitterDeg;
    c.altitude_jitter_m = o.altitudeJitterM; c.heading_jitter_deg = o.headingJitterDeg; c.airspeed_jitter_ms = o.airspeedJitterMs;
    c.target_altitude_delta_m = o.targetAltitudeDeltaM; c.target_heading_delta_deg = o.targetHeadingDeltaDeg;
    c.world_name = o.worldName.c_str();
    c.publish = o.publish ? 1 : 0;
    c.terrain = o.terrain ? 1 : 0;
    c.scenario_path = o.scenarioPath.empty() ? nullptr : o.scenarioPath.c_str();
    return c;
}

template <typename T>
fsim::ConstSpan<T> span(fsim::Span<const T> s) { return fsim::ConstSpan<T>{s.data(), s.size()}; }

fsim::StepResult toResult(const fsim_vecenv& h) {
    fsim::StepResult r;
    r.observations = span(h.last.observations);
    r.rewards = span(h.last.rewards);
    r.terminated = span(h.last.terminated);
    r.truncated = span(h.last.truncated);
    r.finalObservations = span(h.env.finalObservations());
    r.episodeSteps = span(h.last.episodeSteps);
    return r;
}

} // namespace

// ---------------------------------------------------------------------------
// C ABI
// ---------------------------------------------------------------------------
extern "C" {

FSIM_API uint32_t fsim_abi_version(void) { return FSIM_ABI_VERSION; }
FSIM_API const char* fsim_version(void) { return FSIM_VERSION_STRING; }
FSIM_API const char* fsim_last_error(void) { return fsim::sdk::lastError().c_str(); }

FSIM_API void fsim_set_log_level(int level) {
    if (level < FSIM_LOG_TRACE) level = FSIM_LOG_TRACE;
    if (level > FSIM_LOG_OFF) level = FSIM_LOG_OFF;
    fsim::log::setLevel(static_cast<fsim::log::Level>(level));
}

FSIM_API int fsim_log_level(void) { return static_cast<int>(fsim::log::level()); }

FSIM_API const char* fsim_registered_id(int registry, uint32_t index) {
    // Registration is process-wide and may change between calls; the list is
    // taken fresh each time and kept per thread so the pointer outlives it.
    thread_local std::vector<std::string> ids;
    const auto& r = fsim::env::PluginRegistry::instance();
    switch (registry) {
    case FSIM_REGISTRY_TASK: ids = r.taskIds(); break;
    case FSIM_REGISTRY_OBSERVATION: ids = r.observationIds(); break;
    case FSIM_REGISTRY_ACTION: ids = r.actionIds(); break;
    default: return "";
    }
    return index < ids.size() ? ids[index].c_str() : "";
}

} // extern "C"

// ---------------------------------------------------------------------------
// C++ SDK
// ---------------------------------------------------------------------------
namespace fsim {

struct VecEnv::Impl {
    fsim_vecenv* handle = nullptr;
    ~Impl() { fsim_vecenv_destroy(handle); }
};

VecEnv::VecEnv(const VecEnvOptions& options) : impl_(std::make_unique<Impl>()) {
    const fsim_options c = toOptions(options);
    if (fsim_vecenv_create(&c, &impl_->handle) != FSIM_OK) throw std::runtime_error(fsim_last_error());
    fsim_vecenv_set_autoreset(impl_->handle, options.autoReset == AutoReset::SameStep ? FSIM_AUTORESET_SAME_STEP
                                                                                       : FSIM_AUTORESET_NEXT_STEP);
    if (fsim_vecenv_set_action_ranges(impl_->handle, options.actionRanges.c_str()) != FSIM_OK) throw std::runtime_error(fsim_last_error());
}

VecEnv::~VecEnv() = default;

StepResult VecEnv::reset(std::uint64_t seed) {
    if (fsim_vecenv_reset(impl_->handle, seed) != FSIM_OK) throw std::runtime_error(fsim_last_error());
    return toResult(*impl_->handle);
}

StepResult VecEnv::step(const float* actions, std::size_t count) {
    if (fsim_vecenv_step(impl_->handle, actions, count) != FSIM_OK) throw std::runtime_error(fsim_last_error());
    return toResult(*impl_->handle);
}

unsigned VecEnv::numEnvs() const noexcept { return impl_->handle->env.numEnvs(); }
unsigned VecEnv::vehiclesPerEnv() const noexcept { return impl_->handle->env.vehiclesPerEnv(); }
std::size_t VecEnv::numVehicles() const noexcept { return impl_->handle->env.numVehicles(); }
std::size_t VecEnv::observationSize() const noexcept { return impl_->handle->env.observationSize(); }
std::size_t VecEnv::actionSize() const noexcept { return impl_->handle->env.actionSize(); }
const std::vector<std::string>& VecEnv::observationNames() const noexcept { return impl_->handle->env.observationNames(); }
const std::vector<std::string>& VecEnv::actionNames() const noexcept { return impl_->handle->env.actionNames(); }
double VecEnv::agentStepSeconds() const noexcept { return impl_->handle->env.agentStepSeconds(); }
std::uint64_t VecEnv::vehicleSteps() const noexcept { return impl_->handle->env.vehicleSteps(); }
fsim_vecenv* VecEnv::handle() noexcept { return impl_->handle; }

World& VecEnv::world() { return *fsim_world_object(fsim_vecenv_world(impl_->handle)); }

const char* version() noexcept { return FSIM_VERSION_STRING; }

} // namespace fsim
