#pragma once

// Same-build test support, not an SDK header or an episode owner. Returned objects own IDs/data only.
#include "fsim/World.h"

#include <cstdint>
#include <map>
#include <string>

namespace fsim::test::native_reset {

struct Scenario {
    std::uint32_t launcher = 0, target = 0, victim = 0, removed = 0, c2 = 0;
    comm::NetId net = 0;
    std::uint16_t stations = 0, countermeasures = 0;
};

struct DirtyState {
    bool crewLost = false, activeFire = false;
    unsigned crewLossEvents = 0, fireEvents = 0;
    std::uint32_t liveWeapon = 0, liveDecoy = 0, removedEntity = 0;
    std::uint64_t releasedActivity = 0, waitingActivity = 0, pendingTask = 0, pendingConsent = 0, queuedMessage = 0;
    std::uint16_t consumedStore = 0;
    comm::SystemId launcherSystem = 0, c2System = 0;
};

using Snapshot = std::map<std::string, std::string>;

/// Uses stock models, fixed 120 Hz / four-step timing, seed 17, no publication/recording and an explicit UTC epoch.
WorldOptions options(const std::string& name, unsigned workers = 1);
/// Deterministic scenario recipe for an empty World. No handles escape; apply this same recipe after reconstruction.
Scenario configure(World& world);
/// Creates configured Agents locally, exercises existing native paths and leaves all required states coexisting.
/// Throws fsim::Error on any refused operation, timeout or missing native outcome. Never injects release consent.
DirtyState contaminate(World& world, const Scenario& scenario);
/// Revalidates simultaneous live state/correlations and queued send status without consuming or advancing time.
void requireDirty(World& world, const Scenario& scenario, const DirtyState& dirty);
/// After native delivery, checks the exact queued marker bytes, identities and timestamps without consuming it.
void requireDeliveredMarker(World& world, const Scenario& scenario, const DirtyState& dirty);
/// Owned fieldwise native observations for same-build fresh/replay comparison. No padding or pointer comparisons;
/// no normalization of native IDs, serials, time, outcomes, inventory or RNG-dependent state.
Snapshot capture(World& world);

} // namespace fsim::test::native_reset
