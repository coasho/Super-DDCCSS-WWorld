// fsim.dll: the C ABI's core entry points (design 4.4, 9.2) - version, the per-thread error string every C adapter
// writes, the log level and the plug-in registry listing.

#include "fsim/fsim_c.h"

#include "foundation/core/Log.h"
#include "LastError.h"
#include "training/native/vecenv/Registry.h"

#include <string>
#include <vector>

#ifndef FSIM_VERSION_STRING
#    define FSIM_VERSION_STRING "0.0.0"
#endif

namespace fsim::sdk {
std::string& lastError() noexcept {
    thread_local std::string t_lastError;
    return t_lastError;
}
} // namespace fsim::sdk

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
