#pragma once

// One batch of vehicle snapshots, as the simulation's runner hands it to an in-process reader and a world mirror hands
// it to the viewer's interpolation (design 6.3). A neutral contract (src/contracts, ARCH-02): the producer's triple
// buffer (sim/SnapshotBuffer.h) and the transport's mirror both include it, neither through the other. Its name and
// namespace are those it had in sim/SnapshotBuffer.h.

#include "fsim/VehicleState.h"

#include <cstdint>
#include <vector>

namespace fsim::sim {

/// One published batch of vehicle snapshots.
struct SnapshotBatch {
    double simTime = 0.0;            ///< simulation time of the batch, seconds
    std::int64_t wallNs = 0;         ///< platform::Clock::nanoseconds() at publish
    std::uint64_t sequence = 0;      ///< increments per publish
    std::vector<VehicleState> states;
};

} // namespace fsim::sim
