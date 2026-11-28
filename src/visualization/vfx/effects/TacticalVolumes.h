#pragma once

// VIS-M14 3-D Depth-Tested Volumetric Tactical & EMSO Overlay Renderer.
// Renders strictly read-only native SDK TacticalOverlayFrameV1 telemetry as smooth
// 3-D Fresnel volumetric shells, dual-zone burn-through domes, animated EA jamming
// wavefronts, tapered ESM DF lobes / CRLB ellipsoids, color-zoned 5-D DLZ decks,
// animated Link 16 energy arcs, and SAR/weather footprints with soft reverse-Z
// terrain occlusion and terrain-intersection contours. Zero GPU cost when OFF.

#include "contracts/world_stream/TacticalOverlayLayoutV1.h"
#include "foundation/core/Span.h"
#include "foundation/transport/TacticalOverlaySegmentV1.h"
#include "fsim/VehicleState.h"

#include <vsg/all.h>

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace fsim::world {

class TacticalVolumes {
public:
    static constexpr std::uint32_t kMaxVertices = 65536u;

    // Visual affiliation tags inferred purely from vehicle display metadata (presentation only).
    enum class Side : std::uint8_t { Neutral = 0, Blue = 1, Red = 2 };

    // Overlay bitmask constants matching ui::kOverlay*.
    static constexpr std::uint32_t kMaskRadar   = 1u << 0;
    static constexpr std::uint32_t kMaskJammer  = 1u << 1;
    static constexpr std::uint32_t kMaskEsm     = 1u << 2;
    static constexpr std::uint32_t kMaskLinks   = 1u << 3;
    static constexpr std::uint32_t kMaskWeapons = 1u << 4;
    static constexpr std::uint32_t kMaskSwaths  = 1u << 5;

    TacticalVolumes(vsg::ref_ptr<const vsg::Options> options, const std::filesystem::path& shaders);
    ~TacticalVolumes() = default;

    bool valid() const noexcept { return valid_; }

    /// Build Vulkan overPass pipeline (call before Viewer::setScene, wire node() into overScene).
    bool build(vsg::ref_ptr<vsg::ImageInfo> sceneDepth);

    /// Park switch disabled after Viewer::setScene() pipeline compilation (0.0 ms when OFF).
    void compiled();

    vsg::ref_ptr<vsg::Node> node() const { return root_; }

    /// Infer visual force affiliation purely from callsign string for Blue vs. Red distinction.
    static Side inferSide(const std::string& callsign) noexcept;

    /// Rebuild 3-D eye-relative volumetric geometry from native read-only tactical overlay data.
    void update(std::uint32_t overlayMask,
                const ipc::TacticalOverlayFrameV1* frame,
                Span<const sim::VehicleState> states,
                const std::vector<unsigned char>& alive,
                const std::vector<Side>& sides,
                int selectedSlot,
                const vsg::dmat4& projection,
                const vsg::dvec3& eyeEcef,
                std::uint32_t screenWidth,
                std::uint32_t screenHeight,
                float daylight,
                double simTime,
                double wallTime,
                const vsg::dvec3& sunDirEcef = vsg::dvec3(0.0, 0.0, 0.0));

    std::uint32_t lastVertexCount() const noexcept { return lastVertexCount_; }

private:
    void pushVert(const vsg::dvec3& worldEcef, float rNorm,
                  const vsg::dvec3& normalEcef, float uCoord,
                  const vsg::vec4& color, const vsg::vec4& params) noexcept;
    void pushTri(const vsg::dvec3& p0, float r0, const vsg::dvec3& n0, float u0,
                 const vsg::dvec3& p1, float r1, const vsg::dvec3& n1, float u1,
                 const vsg::dvec3& p2, float r2, const vsg::dvec3& n2, float u2,
                 const vsg::vec4& color, const vsg::vec4& params) noexcept;
    void pushQuad(const vsg::dvec3& p00, float r00, const vsg::dvec3& n00, float u00,
                  const vsg::dvec3& p10, float r10, const vsg::dvec3& n10, float u10,
                  const vsg::dvec3& p11, float r11, const vsg::dvec3& n11, float u11,
                  const vsg::dvec3& p01, float r01, const vsg::dvec3& n01, float u01,
                  const vsg::vec4& color, const vsg::vec4& params) noexcept;
    void pushBeamRibbon(const vsg::dvec3& a, const vsg::dvec3& b, double halfWidthM,
                        const vsg::vec4& color, float style, float aux1, float aux2) noexcept;
    void pushRingRibbon(const vsg::dvec3& center, const vsg::dvec3& axisU, const vsg::dvec3& axisV,
                        double radiusU, double radiusV, double halfWidthM, int segments,
                        const vsg::vec4& color, float dashFreq, float dashSpeed) noexcept;

    void emitRadars(const ipc::TacticalOverlayFrameV1& frame, Span<const sim::VehicleState> states,
                    const std::vector<unsigned char>& alive, const std::vector<Side>& sides, int selectedSlot);
    void emitJammers(const ipc::TacticalOverlayFrameV1& frame, Span<const sim::VehicleState> states,
                     const std::vector<unsigned char>& alive, const std::vector<Side>& sides, int selectedSlot);
    void emitEsm(const ipc::TacticalOverlayFrameV1& frame, const std::vector<Side>& sides, int selectedSlot);
    void emitWeapons(const ipc::TacticalOverlayFrameV1& frame, Span<const sim::VehicleState> states,
                     const std::vector<unsigned char>& alive, int selectedSlot);
    void emitLinks(const ipc::TacticalOverlayFrameV1& frame, const std::vector<Side>& sides, int selectedSlot);
    void emitSwaths(const ipc::TacticalOverlayFrameV1& frame, int selectedSlot);

    std::filesystem::path shaders_;
    bool valid_ = false;
    bool built_ = false;
    vsg::dvec3 eyeEcef_{0.0, 0.0, 0.0};

    vsg::ref_ptr<vsg::Group> root_;
    vsg::ref_ptr<vsg::Switch> switch_;
    vsg::ref_ptr<vsg::MatrixTransform> transform_;
    vsg::ref_ptr<vsg::Draw> draw_;
    vsg::ref_ptr<vsg::vec4Array> vertexStaging_;
    vsg::ref_ptr<vsg::vec4Array> frameUbo_;

    std::uint32_t cursor_ = 0u;
    std::uint32_t lastVertexCount_ = 0u;
};

} // namespace fsim::world
