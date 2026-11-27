#pragma once

#include "contracts/world_stream/CombatVfxLayoutV1.h"
#include "foundation/platform/SharedMemory.h"

#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

namespace fsim::ipc {

struct CombatVfxFrameV1 {
    double simTime = 0.0;
    std::vector<ProjectileWireV1> projectiles;
    std::vector<DecoyWireV1> decoys;
    std::vector<CombatVfxEventWireV1> events;
};

/// Optional wait-free triple-buffered combat VFX sidecar segment (VIS-M13).
/// WorldLayout v9 and DebrisLayoutV1 remain byte-for-byte unchanged.
class CombatVfxSegmentV1 {
public:
    ~CombatVfxSegmentV1();
    bool create(const std::string& world, std::uint32_t projectileCapacity, std::uint32_t decoyCapacity,
                std::uint32_t eventCapacity, std::uint64_t pid, std::uint32_t incarnation);
    bool open(const std::string& world, std::uint64_t pid, std::uint32_t incarnation);
    void close() noexcept;
    bool publish(std::uint64_t sequence, const CombatVfxFrameV1& frame) noexcept;
    bool read(std::uint64_t sequence, CombatVfxFrameV1& out) const;

private:
    CombatVfxSlotV1* slot(std::uint64_t sequence) const noexcept;
    platform::SharedMemory memory_;
    CombatVfxHeaderV1* header_ = nullptr;
    std::uint64_t pid_ = 0;
    std::uint32_t incarnation_ = 0;
    std::uint32_t projectileCapacity_ = 0;
    std::uint32_t decoyCapacity_ = 0;
    std::uint32_t eventCapacity_ = 0;
    bool owner_ = false;
};

/// Companion .vfx sidecar recorder for deterministic combat VFX replay (leaves fsrec v8 untouched).
class CombatVfxRecorderV1 {
public:
    CombatVfxRecorderV1() = default;
    ~CombatVfxRecorderV1();
    CombatVfxRecorderV1(const CombatVfxRecorderV1&) = delete;
    CombatVfxRecorderV1& operator=(const CombatVfxRecorderV1&) = delete;

    bool open(const std::filesystem::path& path);
    bool isOpen() const noexcept { return file_ != nullptr; }
    bool write(const CombatVfxFrameV1& frame);
    void close();

private:
    std::FILE* file_ = nullptr;
};

/// Companion .vfx sidecar reader with smooth projectile/decoy interpolation and deduplicated event queries.
class CombatVfxReplayV1 {
public:
    bool load(const std::filesystem::path& path, std::string* error = nullptr);
    void clear() noexcept { frames_.clear(); }
    bool empty() const noexcept { return frames_.empty(); }
    std::size_t frameCount() const noexcept { return frames_.size(); }
    const std::vector<CombatVfxFrameV1>& frames() const noexcept { return frames_; }

    /// Populate `out` at simulation time `simTimeS`. When `interpolate` is true,
    /// matching-ID projectiles and decoys are linearly interpolated between bracketing frames,
    /// and discrete events in `[simTimeS - 0.12, simTimeS]` are gathered deduplicated by serial.
    bool atTime(double simTimeS, CombatVfxFrameV1& out, bool interpolate = true) const;

private:
    std::vector<CombatVfxFrameV1> frames_;
};

} // namespace fsim::ipc
