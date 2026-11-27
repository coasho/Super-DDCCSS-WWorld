#pragma once

// Real GLB aircraft section breakup + manifold3d Voronoi fractured shards + scene-depth
// soft-particle volumetric smoke ribbons, ocean water geysers, foam wakes, ground dust
// clouds, scorched skid tracks, flame cores, and incandescent sparks (VIS-M11 v2).
// Pure presentation renderer: consumes read-only ipc::DebrisFrameV1 and compiled
// VehicleVisuals GLB section subgraphs; never simulates physics.

#include "contracts/world_stream/DebrisLayoutV1.h"
#include "foundation/transport/DebrisSegmentV1.h"
#include "visualization/scene/VehicleVisuals.h"

#include <vsg/all.h>

#include <array>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <unordered_map>
#include <vector>

namespace fsim::world {

class DebrisVisuals {
public:
    static constexpr std::uint32_t kMaxSolidInstances = ipc::kMaxDebrisV1;
    static constexpr std::uint32_t kMaxTrailQuads = 6144;
    static constexpr std::uint32_t kMaxSectionPool = 32;
    static constexpr std::uint32_t kVoronoiBuckets = 6;

    DebrisVisuals(vsg::ref_ptr<const vsg::Options> options, const std::filesystem::path& shaders);
    ~DebrisVisuals() = default;

    bool valid() const noexcept { return valid_; }

    /// Optional active NVIDIA Flow volume reference for exact 360-degree 3-D depth interleaving
    /// between water spray / ground dust / smoke / sparks and Flow fireballs.
    struct FlowInterleave {
        bool active = false;
        vsg::dvec3 centerEcef{0.0, 0.0, 0.0};
        vsg::vec4 laid{0.0f, 0.0f, -1.0f, -1.0f};
        float scale = 2.0f;
    };

    /// Build the scene-depth soft-particle translucent passes (call before Viewer::setScene, passing Viewer::sceneDepth()
    /// and optionally FlowVolumes::halfFrontImage(); wire trailNode() BEFORE FlowVolumes::compositeNode() and
    /// frontTrailNode() AFTER FlowVolumes::compositeNode() in `overScene` for exact 3-D volumetric interleaving).
    bool buildTrails(vsg::ref_ptr<vsg::ImageInfo> sceneDepth, vsg::ref_ptr<vsg::ImageInfo> flowFront = {});
    /// Call once after Viewer::setScene() has compiled pipelines so idle switches start disabled.
    void compiled();

    /// Opaque sunlit 3-D real GLB sections & manifold3d Voronoi shards (main scene pass).
    vsg::ref_ptr<vsg::Node> solidNode() const { return solidRoot_; }
    /// Back translucent pass: surface decals (scorch/foam) + particles behind active Flow volumes (`overScene`).
    vsg::ref_ptr<vsg::Node> trailNode() const { return trailRoot_; }
    /// Front translucent pass: water spray, dust, smoke & sparks in front of active Flow volumes (`overScene`).
    vsg::ref_ptr<vsg::Node> frontTrailNode() const { return frontTrailRoot_; }

    /// Reset presentation trail history (called on replay seek/rewind or world replacement).
    void clear();

    /// Update real GLB section transforms, manifold3d Voronoi shards, and soft-depth trail quads.
    void update(const ipc::DebrisFrameV1& frame, const VehicleVisuals& visuals,
                const std::unordered_map<std::uint32_t, std::size_t>& slotByVehicleId,
                const vsg::dmat4& projection, std::uint32_t screenWidth, std::uint32_t screenHeight,
                const vsg::dvec3& eyeEcef, const vsg::dvec3& cameraRight, const vsg::dvec3& cameraUp,
                const vsg::dvec3& sunDirEcef, float daylight, double simTime,
                const FlowInterleave* flowRef = nullptr);

    std::uint32_t solidCount() const noexcept { return solidCount_; }
    std::uint32_t trailQuadCount() const noexcept { return trailCount_; }

private:
    struct SmokeSample {
        vsg::dvec3 posEcef;
        vsg::vec3 velEcef{0.0f, 0.0f, 0.0f};
        vsg::vec3 driftEcef{0.0f, 0.0f, 0.0f}; ///< 3D turbulent vortex curl + buoyancy velocity (breaks straight-tube artifacts!)
        double birthTime = 0.0;
        float baseRadius = 1.0f;
        float tempK = 300.0f;
        float opacity = 0.45f;
        float seed = 0.0f;
        float spin = 1.0f;
    };
    struct TrailHistory {
        std::deque<SmokeSample> samples;
        vsg::dvec3 lastPosEcef{};
        double lastSampleTime = -1e9;
        bool hasLastPos = false;
    };

    bool valid_ = false;
    std::filesystem::path shaders_;
    vsg::ref_ptr<vsg::Group> solidRoot_;
    vsg::ref_ptr<vsg::Group> trailRoot_;
    vsg::ref_ptr<vsg::Group> frontTrailRoot_;
    vsg::ref_ptr<vsg::Switch> solidSwitch_;
    vsg::ref_ptr<vsg::Switch> sectionPoolSwitch_;
    vsg::ref_ptr<vsg::Switch> trailSwitch_;
    vsg::ref_ptr<vsg::Switch> frontTrailSwitch_;
    vsg::ref_ptr<vsg::MatrixTransform> solidTransform_;
    vsg::ref_ptr<vsg::MatrixTransform> sectionPoolTransform_;
    vsg::ref_ptr<vsg::MatrixTransform> trailTransform_;
    vsg::ref_ptr<vsg::MatrixTransform> frontTrailTransform_;

    struct SectionSlot {
        vsg::ref_ptr<vsg::Switch> sw;
        vsg::ref_ptr<vsg::MatrixTransform> xf;
        vsg::ref_ptr<vsg::Node> boundChild;
    };
    std::array<SectionSlot, kMaxSectionPool> sectionSlots_;

    vsg::ref_ptr<vsg::vec4Array> frameUbo_;
    vsg::ref_ptr<vsg::vec4Array> solidInstances_;
    vsg::ref_ptr<vsg::vec4Array> trailQuads_;
    std::array<vsg::ref_ptr<vsg::VertexDraw>, kVoronoiBuckets> solidDraws_;
    vsg::ref_ptr<vsg::Draw> trailDraw_;
    vsg::ref_ptr<vsg::Draw> frontTrailDraw_;

    std::unordered_map<std::uint32_t, TrailHistory> smokeTrails_;
    double lastClock_ = -1e9;
    std::uint32_t solidCount_ = 0;
    std::uint32_t trailCount_ = 0;
};

} // namespace fsim::world
