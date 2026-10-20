#pragma once

// The viewer's --demo (ARCH-08, HC-16): the demo publisher process (flightsim-demo, applications/demo) launched beside
// the viewer, its world attached read-only (foundation/transport/WorldMirror.h) and its simulation thread driven through
// the demo control block (contracts/world_stream/DemoControl.h) - pause, single step, time factor - as the viewer drove
// the in-process demo. The viewer composes no simulation: the demo's is the publisher's, in its own process.

#include "contracts/world_stream/DemoControl.h"
#include "contracts/world_stream/SnapshotBatch.h"
#include "foundation/platform/SharedMemory.h"
#include "foundation/transport/WorldMirror.h"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace fsim::app {

class DemoLink {
public:
    /// Launch `program` with `arguments` (its demo and terrain options; --world, --control and --parent-pid are added),
    /// its simulation starting at `timeFactor`, held from its first state when `paused`, and wait - up to
    /// `timeoutSeconds` - until its world has published a snapshot. nullptr, after logging why, when it cannot be
    /// launched, exits first or does not publish in time.
    static std::unique_ptr<DemoLink> launch(const std::filesystem::path& program, std::vector<std::string> arguments,
                                            double timeFactor, bool paused, double timeoutSeconds);
    ~DemoLink();
    DemoLink(const DemoLink&) = delete;
    DemoLink& operator=(const DemoLink&) = delete;

    std::size_t vehicles() const noexcept;
    /// Vehicle i's CG's place from its model's origin, body axes (3 values), as the publisher read it before its
    /// simulation's thread ran.
    void cgFromOriginBodyM(std::size_t i, double* out) const;

    void setPaused(bool paused) noexcept;
    void setTimeFactor(double factor) noexcept;
    void singleStep() noexcept;
    /// The single steps the publisher has taken, each a published step (DemoControl::stepsTaken): when it equals the
    /// steps asked for, they are all in the world stream.
    std::uint64_t stepsTaken() const noexcept;
    /// The newest published snapshot since the last call, or nullptr.
    const sim::SnapshotBatch* acquire() noexcept;
    const std::vector<ms::WeaponControlSnapshotV1>& weaponControls() const noexcept { return mirror_.weaponControls(); }
    double throughput() const noexcept;
    /// Ask the publisher to stop and wait for it to exit (ended by force after a few seconds): whether it exited on
    /// request.
    bool stop() noexcept;

private:
    DemoLink();
    struct Process;
    std::unique_ptr<Process> process_;
    platform::SharedMemory controlMemory_;
    ipc::DemoControl* control_ = nullptr;
    ipc::WorldMirror mirror_;
    sim::SnapshotBatch batch_;
    bool fresh_ = false;
};

} // namespace fsim::app
