#pragma once

#include "contracts/world_stream/TacticalOverlayLayoutV1.h"
#include "foundation/platform/SharedMemory.h"

#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

namespace fsim::ipc {

struct TacticalOverlayFrameV1 {
    double simTime = 0.0;
    std::vector<TacticalRadarVolumeWireV1> radars;
    std::vector<TacticalJammerLobeWireV1> jammers;
    std::vector<TacticalEsmRayWireV1> esmRays;
    std::vector<TacticalLinkPulseWireV1> links;
    std::vector<TacticalWeaponCueWireV1> weapons;
    std::vector<TacticalSwathWireV1> swaths;
    std::vector<TacticalPropulsionExtWireV1> propulsion;
    std::vector<TacticalRadarTrackWireV1> radarTracks; ///< Additive own-radar tracks & electronic false targets

    bool empty() const noexcept {
        return radars.empty() && jammers.empty() && esmRays.empty() && links.empty() &&
               weapons.empty() && swaths.empty() && propulsion.empty() && radarTracks.empty();
    }
};

/// Optional wait-free triple-buffered Tactical & EMSO overlay sidecar segment (VIS-M14).
/// Preserves WorldLayout v9, VehicleState, DebrisLayoutV1, and CombatVfxLayoutV1 byte-for-byte.
class TacticalOverlaySegmentV1 {
public:
    ~TacticalOverlaySegmentV1();
    bool create(const std::string& world, std::uint64_t pid, std::uint32_t incarnation);
    bool open(const std::string& world, std::uint64_t pid, std::uint32_t incarnation);
    void close() noexcept;
    bool publish(std::uint64_t sequence, const TacticalOverlayFrameV1& frame) noexcept;
    bool read(std::uint64_t sequence, TacticalOverlayFrameV1& out) const;

private:
    TacticalOverlaySlotV1* slot(std::uint64_t sequence) const noexcept;
    platform::SharedMemory memory_;
    TacticalOverlayHeaderV1* header_ = nullptr;
    std::uint64_t pid_ = 0;
    std::uint32_t incarnation_ = 0;
    std::uint32_t radarCap_ = 0;
    std::uint32_t jammerCap_ = 0;
    std::uint32_t esmCap_ = 0;
    std::uint32_t linkCap_ = 0;
    std::uint32_t weaponCap_ = 0;
    std::uint32_t swathCap_ = 0;
    std::uint32_t propCap_ = 0;
    bool owner_ = false;
};

/// Companion .tact sidecar recorder for deterministic tactical/EMSO overlay replay (leaves fsrec v8 untouched).
class TacticalOverlayRecorderV1 {
public:
    TacticalOverlayRecorderV1() = default;
    ~TacticalOverlayRecorderV1();
    TacticalOverlayRecorderV1(const TacticalOverlayRecorderV1&) = delete;
    TacticalOverlayRecorderV1& operator=(const TacticalOverlayRecorderV1&) = delete;

    bool open(const std::filesystem::path& path);
    bool isOpen() const noexcept { return file_ != nullptr; }
    bool write(const TacticalOverlayFrameV1& frame);
    void close();

private:
    std::FILE* file_ = nullptr;
};

/// Companion .tact sidecar reader for deterministic replay seeking.
class TacticalOverlayReplayV1 {
public:
    bool load(const std::filesystem::path& path, std::string* error = nullptr);
    void clear() noexcept { frames_.clear(); }
    bool empty() const noexcept { return frames_.empty(); }
    std::size_t frameCount() const noexcept { return frames_.size(); }
    const std::vector<TacticalOverlayFrameV1>& frames() const noexcept { return frames_; }
    bool atTime(double simTimeS, TacticalOverlayFrameV1& out) const;

private:
    std::vector<TacticalOverlayFrameV1> frames_;
};

} // namespace fsim::ipc
