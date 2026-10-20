#pragma once

#include "contracts/world_stream/WorldLayout.h"
#include "foundation/platform/SharedMemory.h"
#include "foundation/transport/WeaponControlsSegment.h"
#include "contracts/world_stream/SnapshotBatch.h"

#include <cstdint>
#include <string>
#include <vector>

namespace fsim::ipc {

/// A vehicle as the mirror sees it (decoded VehicleRecord).
struct MirroredVehicle {
    std::uint32_t slot = 0;
    std::uint32_t id = 0;
    std::uint64_t generation = 0;
    bool alive = false;
    std::uint8_t controlLevel = 0;
    std::string name, type, model;
    double initialLatitudeDeg = 0.0, initialLongitudeDeg = 0.0, initialAltitudeMslM = 0.0, initialHeadingDeg = 0.0;
};

/// Reader side of the world segment (design 9.7): read-only, never blocks the
/// publisher, tolerates a publisher that stalls, restarts or dies. The viewer
/// polls it once per frame.
class WorldMirror {
public:
    WorldMirror() = default;
    ~WorldMirror();
    WorldMirror(const WorldMirror&) = delete;
    WorldMirror& operator=(const WorldMirror&) = delete;

    /// Attach to a published world by name. Returns false if it does not exist
    /// or has an incompatible layout.
    bool open(const std::string& name);
    void close();
    bool valid() const noexcept { return header_ != nullptr; }

    const std::string& name() const noexcept { return name_; }
    std::uint32_t capacity() const noexcept { return header_ ? header_->capacity : 0; }
    double dt() const noexcept { return header_ ? header_->dt : 0.0; }
    int frameSkip() const noexcept { return header_ ? header_->frameSkip : 1; }
    std::uint64_t publisherPid() const noexcept { return header_ ? header_->publisherPid : 0; }

    /// Publisher still alive (flag + process check)?
    bool publisherAlive() const noexcept;
    /// Another world now publishes under this name: the segment was re-initialised (WorldHeader::incarnation) - every
    /// counter this mirror has seen is the old world's. Close and open again.
    bool replaced() const noexcept;
    /// Seconds since the last publish (wall clock).
    double ageSeconds() const noexcept;
    std::uint64_t vehicleSteps() const noexcept { return header_ ? header_->vehicleSteps.load(std::memory_order_relaxed) : 0; }
    std::uint64_t worldSteps() const noexcept { return header_ ? header_->worldSteps.load(std::memory_order_relaxed) : 0; }
    /// The publisher's requests that viewers follow a vehicle: a count bumped by each, and the vehicle id the last named.
    std::uint32_t focusSequence() const noexcept { return header_ ? header_->focusSequence.load(std::memory_order_acquire) : 0; }
    std::uint32_t focusVehicle() const noexcept { return header_ ? header_->focusVehicle.load(std::memory_order_relaxed) : 0; }

    /// Re-read the vehicle table if it changed. Returns true when it did;
    /// `vehicles()` then reflects the new table (one entry per slot).
    bool pollTable();
    const std::vector<MirroredVehicle>& vehicles() const noexcept { return vehicles_; }
    std::uint64_t tableGeneration() const noexcept { return tableGeneration_; }

    /// Copy the newest snapshot into `out` if it is newer than the last one
    /// returned. `out.states` has `capacity()` entries; `inputs()` matches.
    bool pollSnapshot(sim::SnapshotBatch& out);
    const std::vector<sim::ControlInputs>& inputs() const noexcept { return inputs_; }
    /// Each vehicle's flight status with that snapshot (WorldLayout.h, Derived): all 0 from a publisher that gives none.
    const std::vector<FlightStatus>& status() const noexcept { return status_; }
    const std::vector<ms::WeaponControlSnapshotV1>& weaponControls() const noexcept { return weaponControls_; }
    std::int64_t lastWallNs() const noexcept { return lastWallNs_; }
    /// The engagement's events written since the last call, appended to `out` in their order (the first call: those
    /// still in the ring - a viewer that attaches to a fire sees it); how many.
    std::size_t pollEvents(std::vector<EngagementEvent>& out);

    /// Latest environment (seqlock copy).
    sim::EnvironmentState environment() const noexcept;

    /// A vehicle slot's Mission Systems record (seqlock copy) if its revision is not `known`; false if unchanged,
    /// torn eight times running, or out of range.
    bool mission(std::uint32_t slot, MissionRecord& out, std::uint64_t known = ~0ull) const noexcept;

private:
    const VehicleRecord* row(std::uint32_t index) const noexcept;
    const StateSlotHeader* slot(std::uint32_t index) const noexcept;
    const VehicleSample* samples(std::uint32_t index) const noexcept;

    platform::SharedMemory memory_;
    WeaponControlsSegment finSegment_;
    std::vector<ms::WeaponControlSnapshotV1> weaponControls_;
    const WorldHeader* header_ = nullptr;
    const unsigned char* base_ = nullptr;
    std::string name_;
    std::vector<MirroredVehicle> vehicles_;
    std::vector<sim::ControlInputs> inputs_;
    std::vector<FlightStatus> status_;
    std::uint64_t eventsSeen_ = 0;
    std::uint64_t tableGeneration_ = ~0ull;
    std::uint64_t lastSequence_ = 0;
    std::int64_t lastWallNs_ = 0;
    std::uint32_t incarnation_ = 0;
    bool attached_ = false;
};

} // namespace fsim::ipc
