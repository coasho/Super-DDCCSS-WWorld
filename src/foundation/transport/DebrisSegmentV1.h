#pragma once

#include "contracts/world_stream/DebrisLayoutV1.h"
#include "foundation/platform/SharedMemory.h"

#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

namespace fsim::ipc {

struct DebrisFrameV1 {
    double simTime = 0.0;
    std::vector<DebrisBodyWireV1> bodies;
    std::vector<VehicleDestructionWireV1> vehicles;
};

/// Optional wait-free triple-buffered debris & destruction sidecar segment.
/// WorldLayout v9 remains byte-for-byte unchanged.
class DebrisSegmentV1 {
public:
    ~DebrisSegmentV1();
    bool create(const std::string& world, std::uint32_t debrisCapacity, std::uint32_t vehicleCapacity,
                std::uint64_t pid, std::uint32_t incarnation);
    bool open(const std::string& world, std::uint64_t pid, std::uint32_t incarnation);
    void close() noexcept;
    bool publish(std::uint64_t sequence, const DebrisFrameV1& frame) noexcept;
    bool read(std::uint64_t sequence, DebrisFrameV1& out) const;

private:
    DebrisSlotV1* slot(std::uint64_t sequence) const noexcept;
    platform::SharedMemory memory_;
    DebrisHeaderV1* header_ = nullptr;
    std::uint64_t pid_ = 0;
    std::uint32_t incarnation_ = 0;
    std::uint32_t debrisCapacity_ = 0;
    std::uint32_t vehicleCapacity_ = 0;
    bool owner_ = false;
};

/// Companion .debris sidecar recorder for deterministic replay (leaves fsrec v8 untouched).
class DebrisRecorderV1 {
public:
    DebrisRecorderV1() = default;
    ~DebrisRecorderV1();
    DebrisRecorderV1(const DebrisRecorderV1&) = delete;
    DebrisRecorderV1& operator=(const DebrisRecorderV1&) = delete;

    bool open(const std::filesystem::path& path);
    bool isOpen() const noexcept { return file_ != nullptr; }
    bool write(const DebrisFrameV1& frame);
    void close();

private:
    std::FILE* file_ = nullptr;
};

/// Companion .debris sidecar reader with exact and interpolated replay queries.
class DebrisReplayV1 {
public:
    bool load(const std::filesystem::path& path, std::string* error = nullptr);
    void clear() noexcept { frames_.clear(); }
    bool empty() const noexcept { return frames_.empty(); }
    std::size_t frameCount() const noexcept { return frames_.size(); }
    const std::vector<DebrisFrameV1>& frames() const noexcept { return frames_; }

    /// Populate `out` at simulation time `simTimeS`. When `interpolate` is true,
    /// matching body IDs are linearly interpolated in position/velocity/thermal
    /// state and slerped in orientation between bracketing frames.
    bool atTime(double simTimeS, DebrisFrameV1& out, bool interpolate = true) const;

private:
    std::vector<DebrisFrameV1> frames_;
};

} // namespace fsim::ipc
