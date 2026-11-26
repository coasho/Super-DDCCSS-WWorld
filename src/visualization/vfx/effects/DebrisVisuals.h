#pragma once

// Instanced 6-DOF tumbling debris, glowing incandescent sparks, trailing smoke
// ribbons, and water splash plumes (VIS-M11). Pure presentation renderer:
// consumes read-only ipc::DebrisFrameV1 from live shared memory or .debris
// companion replay sidecars; never simulates physics.

#include "contracts/world_stream/DebrisLayoutV1.h"
#include "foundation/transport/DebrisSegmentV1.h"

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
    static constexpr std::uint32_t kMaxTrailQuads = 4096;

    DebrisVisuals(vsg::ref_ptr<const vsg::Options> options, const std::filesystem::path& shaders);
    ~DebrisVisuals() = default;

    bool valid() const noexcept { return valid_; }

    /// Opaque sunlit 3-D tumbling shards (rendered in main scene pass, depth-tested & depth-written).
    vsg::ref_ptr<vsg::Node> solidNode() const { return solidRoot_; }
    /// Translucent sparks, burning smoke trails, flame cores & water splashes (depth-tested, premultiplied).
    vsg::ref_ptr<vsg::Node> trailNode() const { return trailRoot_; }

    /// Reset presentation trail history (called on replay seek/rewind or world replacement).
    void clear();

    /// Update GPU instance buffers for the current frame from authoritative DebrisFrameV1.
    void update(const ipc::DebrisFrameV1& frame, const vsg::dvec3& eyeEcef, const vsg::dvec3& cameraRight,
                const vsg::dvec3& cameraUp, const vsg::dvec3& sunDirEcef, float daylight, double simTime);

    std::uint32_t solidCount() const noexcept { return solidCount_; }
    std::uint32_t trailQuadCount() const noexcept { return trailCount_; }

private:
    struct SmokeSample {
        vsg::dvec3 posEcef;
        vsg::vec3 velEcef{0.0f, 0.0f, 0.0f};
        double birthTime = 0.0;
        float baseRadius = 1.0f;
        float tempK = 300.0f;
        float opacity = 0.5f;
        float seed = 0.0f;
        float stretch = 1.0f;
    };
    struct TrailHistory {
        std::deque<SmokeSample> samples;
        vsg::dvec3 lastPosEcef{};
        double lastSampleTime = -1e9;
        bool hasLastPos = false;
    };

    bool valid_ = false;
    vsg::ref_ptr<vsg::Group> solidRoot_;
    vsg::ref_ptr<vsg::Group> trailRoot_;
    vsg::ref_ptr<vsg::Switch> solidSwitch_;
    vsg::ref_ptr<vsg::Switch> trailSwitch_;
    vsg::ref_ptr<vsg::MatrixTransform> solidTransform_;
    vsg::ref_ptr<vsg::MatrixTransform> trailTransform_;

    vsg::ref_ptr<vsg::vec4Array> frameUbo_;
    vsg::ref_ptr<vsg::vec4Array> solidInstances_;
    vsg::ref_ptr<vsg::vec4Array> trailQuads_;
    std::array<vsg::ref_ptr<vsg::VertexDraw>, 4> solidDraws_;
    vsg::ref_ptr<vsg::Draw> trailDraw_;

    std::unordered_map<std::uint32_t, TrailHistory> smokeTrails_;
    double lastClock_ = -1e9;
    std::uint32_t solidCount_ = 0;
    std::uint32_t trailCount_ = 0;
};

} // namespace fsim::world
