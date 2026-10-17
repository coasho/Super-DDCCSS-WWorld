#pragma once

// The viewer's built-in demo (ARCH-06): a native simulation of a few aircraft flown by a small autopilot on an
// internal simulation thread, whose snapshots the viewer draws. This is demo composition, owned by the demo
// application; the live/replay viewer reads only these snapshots and owns no flight model or stepping.

#include "contracts/world_stream/SnapshotBatch.h"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

namespace fsim::io {
class AssetResolver;
class TerrainTiles;
}

namespace fsim::app {

/// The demo scenario (the viewer's --demo options).
struct DemoOptions {
    std::string aircraft = "c172x";
    std::filesystem::path jsbsimRoot;
    unsigned vehicles = 8;
    unsigned workers = 0;
    double dt = 1.0 / 120.0;
    int frameSkip = 2;
    std::uint64_t seed = 1;
    double throttle = 0.65;
    double speedMs = 0.0;
    double timeFactor = 1.0;
    double latitudeDeg = 37.6188, longitudeDeg = -122.375, altitudeM = 1500.0, spreadDeg = 0.03;
    bool onGround = false;
};

class DemoSimulation {
public:
    /// The demo's vehicles loaded and its simulation thread prepared (not started); nullptr, after logging why, when
    /// the JSBSim data or a vehicle cannot be loaded. `terrain` (may be null: flat ground) is the physics' ground.
    static std::unique_ptr<DemoSimulation> create(const DemoOptions& options, io::AssetResolver& assets,
                                                  const std::shared_ptr<io::TerrainTiles>& terrain);
    ~DemoSimulation();
    DemoSimulation(const DemoSimulation&) = delete;
    DemoSimulation& operator=(const DemoSimulation&) = delete;

    std::size_t vehicles() const noexcept;
    /// Vehicle i's CG's place from its model's origin, body axes (3 values).
    void cgFromOriginBodyM(std::size_t i, double* out) const;

    void start();
    void stop();
    void setPaused(bool paused) noexcept;
    void setTimeFactor(double factor) noexcept;
    void singleStep() noexcept;
    /// The newest snapshot since the last call, or nullptr (as the simulation's snapshot buffer hands it out).
    const sim::SnapshotBatch* acquire() noexcept;
    double throughput() const noexcept;

private:
    DemoSimulation();
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace fsim::app
