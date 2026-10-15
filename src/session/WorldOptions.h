#pragma once

// Internal construction adapter. Public C++ Worlds and mission episodes use the same mapping; the C ABI keeps its
// separate size-versioned adapter, including the defaults of callers whose options predate an appended field.

namespace fsim {
struct WorldOptions;

namespace session {
struct WorldOptions;

/// Map every public construction option without changing it. Native-only terrainPrefetchRadiusM keeps its native
/// default. The implementation deliberately decomposes both option aggregates by exact arity: adding a field requires
/// an explicit mapping/default decision here and an admission-policy decision for mission episodes.
WorldOptions makeWorldOptions(const fsim::WorldOptions& options);

} // namespace session
} // namespace fsim
