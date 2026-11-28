#pragma once

// VIS-M14: Clean, Anti-Overlap Tactical & EMSO Annotation Micro-Pills & Target Markers.
// Pairs with world::TacticalVolumes (which renders all 3-D depth-tested volumetric shells,
// jammer wavefronts, burn-through domes, DLZ decks, ESM lobes/ellipsoids, and link arcs in Vulkan).
// Strictly read-only presentation of native TacticalOverlayFrameV1; zero CPU cost when OFF.

#include "foundation/core/Span.h"
#include "foundation/transport/TacticalOverlaySegmentV1.h"
#include "fsim/VehicleState.h"
#include "visualization/ui/MonitorGui.h"
#include "visualization/ui/ViewerControls.h"

#include <vsg/all.h>

#include <cstdint>
#include <memory>
#include <vector>

namespace fsim::ui {

class TacticalOverlays : public vsg::Inherit<vsg::Command, TacticalOverlays> {
public:
    explicit TacticalOverlays(std::shared_ptr<ViewerControls> controls);

    /// Per-frame snapshot update on the render thread before recording ImGui.
    void update(const ipc::TacticalOverlayFrameV1* frame,
                Span<const sim::VehicleState> states,
                const std::vector<unsigned char>& alive,
                const std::vector<MonitorGui::VehicleMeta>& meta,
                int selectedSlot,
                const vsg::dmat4& viewMatrix,
                const vsg::dmat4& projMatrix,
                const vsg::dvec3& eyeEcef,
                std::uint32_t screenWidth,
                std::uint32_t screenHeight,
                double simTime);

    void record(vsg::CommandBuffer& cb) const override;

    /// Last frame's overlay CPU annotation time (ms) for --stats reporting (0.0 when OFF).
    double lastDrawMs() const noexcept { return lastDrawMs_; }

private:
    struct PlacedBox {
        float x0 = 0.0f, y0 = 0.0f, x1 = 0.0f, y1 = 0.0f;
    };

    struct ZoneColor {
        int r = 145, g = 206, b = 232;
    };

    bool projectPoint(const vsg::dvec3& ecef, float& sx, float& sy) const noexcept;
    /// Pick the most readable on-screen anchor among 3-D candidate points inside a zone,
    /// adapting automatically to camera orbit/zoom while avoiding HUD panels and overlaps.
    bool pickAdaptiveAnchor(const std::vector<vsg::dvec3>& candidates,
                            const std::vector<PlacedBox>& reservedPanels,
                            float scale,
                            float& outX, float& outY) const noexcept;
    void drawPill(void* dl, float anchorX, float anchorY, const char* text, ZoneColor zoneColor,
                  std::vector<PlacedBox>& placed, float scale, bool drawLeader = false, bool centered = false) const;

    std::shared_ptr<ViewerControls> controls_;
    ipc::TacticalOverlayFrameV1 frame_;
    bool hasFrame_ = false;
    std::vector<sim::VehicleState> states_;
    std::vector<unsigned char> alive_;
    std::vector<MonitorGui::VehicleMeta> meta_;
    int selectedSlot_ = -1;

    vsg::dmat4 view_{};
    vsg::dmat4 proj_{};
    vsg::dvec3 eye_{};
    std::uint32_t width_ = 1920;
    std::uint32_t height_ = 1080;
    double simTime_ = 0.0;
    mutable double lastDrawMs_ = 0.0;
};

} // namespace fsim::ui
