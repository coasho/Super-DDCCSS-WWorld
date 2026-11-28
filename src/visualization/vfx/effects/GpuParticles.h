#pragma once

// Unified compute-driven Vulkan GPU particle & direct combat ribbon/core renderer (VIS-M13).
// Consumes strictly read-only native SDK/IPC combat VFX & debris frames (gun projectiles,
// muzzle flashes, airframe/surface bullet hits, IR flares, chaff dipole clouds, detonation
// shockwaves, and burning debris chunks); never simulates combat authority.

#include "contracts/world_stream/CombatVfxLayoutV1.h"
#include "foundation/core/Span.h"
#include "foundation/transport/CombatVfxSegmentV1.h"
#include "foundation/transport/DebrisSegmentV1.h"
#include "foundation/transport/TacticalOverlaySegmentV1.h"
#include "fsim/VehicleState.h"
#include "visualization/scene/VehicleVisuals.h"
#include "visualization/vfx/effects/DebrisVisuals.h"

#include <vsg/all.h>

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace fsim::world {

class GpuParticles {
public:
    static constexpr std::uint32_t kMaxParticles = 65536u;
    static constexpr std::uint32_t kMaxSpawnsPerFrame = 2048u;
    static constexpr std::uint32_t kMaxRibbons = 1024u;

    GpuParticles(vsg::ref_ptr<const vsg::Options> options, const std::filesystem::path& shaders);
    ~GpuParticles() = default;

    bool valid() const noexcept { return valid_; }
    bool active() const noexcept { return active_; }

    /// Build compute simulation + indirect draw + direct ribbon passes (call before Viewer::setScene,
    /// passing the Vulkan device, Viewer::sceneDepth(), and optionally FlowVolumes::halfFrontImage()).
    /// Wire computeNode() into `betweenPass` and drawNode() into `overScene`.
    bool build(vsg::ref_ptr<vsg::Device> device, vsg::ref_ptr<vsg::ImageInfo> sceneDepth,
               vsg::ref_ptr<vsg::ImageInfo> flowFront = {});

    /// Call once after Viewer::setScene() has compiled pipelines so idle switches start disabled (B-62 warmup).
    void compiled();

    /// Compute spawn + physics simulation + frustum compaction pass (`betweenPass`).
    vsg::ref_ptr<vsg::Node> computeNode() const { return computeRoot_; }
    /// Indirect GPU particles + direct tracers/flares/muzzle-flashes/shockwaves (`overScene`).
    vsg::ref_ptr<vsg::Node> drawNode() const { return drawRoot_; }

    /// Trigger an expanding supersonic detonation shockwave directly from EngagementKind::Detonation
    /// (deduplicated against CombatVfxEventKindV1::DetonationShockwave so legacy replays benefit too).
    void addDetonationShockwave(const vsg::dvec3& posEcef, const vsg::dvec3& velEcef, double tntKg, double simTime);

    /// Emit B-59 rotorcraft low-hover downwash dust/water-mist rings and blade-tip/propeller-tip condensation helices.
    void addRotorEffects(Span<const sim::VehicleState> states, const std::vector<unsigned char>& alive,
                         const VehicleVisuals& visuals, const ipc::TacticalOverlayFrameV1* tactical,
                         double humidity, double simTime);

    /// Clear all GPU particles, active ribbons, and event deduplication state (on replay seek/rewind).
    void clear();

    /// Ingest read-only native combat VFX & debris frames and update compute/render buffers.
    void update(const ipc::CombatVfxFrameV1* vfxFrame, const ipc::DebrisFrameV1* debrisFrame,
                const vsg::dmat4& viewMatrix, const vsg::dmat4& projection,
                std::uint32_t screenWidth, std::uint32_t screenHeight,
                const vsg::dvec3& eyeEcef, const vsg::dvec3& cameraRight, const vsg::dvec3& cameraUp,
                const vsg::dvec3& sunDirEcef, float daylight, double simTime,
                const DebrisVisuals::FlowInterleave* flowRef = nullptr);

    std::uint32_t lastSpawnCount() const noexcept { return lastSpawnCount_; }
    std::uint32_t lastRibbonCount() const noexcept { return lastRibbonCount_; }

private:
    struct GpuParticleCpu {
        vsg::vec4 posAge;
        vsg::vec4 velLife;
        vsg::vec4 colorSize;
        vsg::vec4 params;
    };
    struct RibbonCpu {
        vsg::vec4 posRadius;
        vsg::vec4 dirLength;
        vsg::vec4 colorIntensity;
        vsg::vec4 params;
    };
    struct ActiveMuzzleFlash {
        std::uint32_t shooterId = 0;
        vsg::dvec3 posEcef;
        vsg::dvec3 velEcef;
        vsg::dvec3 dirEcef;
        double startTime = 0.0;
        double duration = 0.075;
        float calibreM = 0.02f;
        float seed = 0.0f;
    };
    struct ActiveShockwave {
        vsg::dvec3 posEcef;
        vsg::dvec3 velEcef;
        double startTime = 0.0;
        double duration = 0.42;
        float maxRadiusM = 30.0f;
        float intensity = 1.0f;
        float seed = 0.0f;
    };
    struct DecoyTrailState {
        vsg::dvec3 lastPosEcef;
        double lastEmitTime = -1e9;
        bool initialized = false;
    };
    struct DebrisEmberState {
        double lastEmitTime = -1e9;
    };
    struct RotorVfxState {
        double lastEmitTime = -1e9;
        double phaseRad = 0.0;
    };

    void queueParticle(const vsg::dvec3& posEcef, const vsg::dvec3& velEcef, float maxLifeS,
                       const vsg::vec3& color, float baseRadiusM, float kind, float dragPerS, float extra);
    float nextRand() noexcept;
    vsg::dvec3 randomUnitVec() noexcept;

    bool valid_ = false;
    bool built_ = false;
    bool active_ = false;
    std::filesystem::path shaders_;

    vsg::ref_ptr<vsg::Group> computeRoot_;
    vsg::ref_ptr<vsg::Group> drawRoot_;
    vsg::ref_ptr<vsg::Switch> computeSwitch_;
    vsg::ref_ptr<vsg::Switch> spawnSwitch_;
    vsg::ref_ptr<vsg::Switch> drawSwitch_;
    vsg::ref_ptr<vsg::Switch> ribbonSwitch_;
    vsg::ref_ptr<vsg::MatrixTransform> drawTransform_;
    vsg::ref_ptr<vsg::MatrixTransform> ribbonTransform_;

    vsg::ref_ptr<vsg::Dispatch> spawnDispatch_;
    vsg::ref_ptr<vsg::Dispatch> simDispatch_;
    vsg::ref_ptr<vsg::Draw> ribbonDraw_;

    vsg::ref_ptr<vsg::vec4Array> spawnStaging_;
    vsg::ref_ptr<vsg::vec4Array> simUbo_;
    vsg::ref_ptr<vsg::vec4Array> renderUbo_;
    vsg::ref_ptr<vsg::vec4Array> ribbonStaging_;
    std::shared_ptr<std::atomic<bool>> resetFlag_;

    vsg::dvec3 originEcef_{0.0, 0.0, 0.0};
    bool hasOrigin_ = false;
    double lastClock_ = -1e9;
    double particleIdleUntil_ = -1e9;
    std::uint32_t rngState_ = 0x9E3779B9u;
    std::uint32_t lastSpawnCount_ = 0;
    std::uint32_t lastRibbonCount_ = 0;

    std::vector<GpuParticleCpu> pendingSpawns_;
    std::unordered_set<std::uint32_t> seenEventSerials_;
    std::unordered_map<std::uint32_t, DecoyTrailState> decoyTrails_;
    std::unordered_map<std::uint32_t, DebrisEmberState> debrisEmbers_;
    std::unordered_map<std::uint64_t, RotorVfxState> rotorStates_;
    std::vector<ActiveMuzzleFlash> muzzleFlashes_;
    std::vector<ActiveShockwave> shockwaves_;
};

} // namespace fsim::world
