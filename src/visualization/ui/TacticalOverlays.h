#pragma once

// VIS-M14: Smooth, Quiet, Borderless 3-D Region Annotations & Target Markers.
// Pairs with world::TacticalVolumes (3-D stereoscopic radar corridors, burn-through
// portals, jammer cones, upright DLZ range gates, ESM/CRLB ellipsoids, link arcs).
// Uses per-label hysteresis & critically-damped screen smoothing so leader lines track
// 3-D zones instantaneously while text annotations remain rock-steady (zero flickering).

#include "foundation/core/Span.h"
#include "foundation/transport/TacticalOverlaySegmentV1.h"
#include "fsim/VehicleState.h"
#include "visualization/ui/MonitorGui.h"
#include "visualization/ui/ViewerControls.h"

#include <vsg/all.h>

#include <cstdint>
#include <memory>
#include <unordered_map>
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

    struct SmoothLabelState {
        float textX = 0.0f;
        float textY = 0.0f;
        int candIdx = -1;
        double lastWallS = 0.0;
        bool valid = false;
    };

    bool projectPoint(const vsg::dvec3& ecef, float& sx, float& sy) const noexcept;
    bool pickStableAnchor(std::uint64_t labelKey,
                          const std::vector<vsg::dvec3>& candidates,
                          const std::vector<PlacedBox>& reservedPanels,
                          float scale,
                          float& outAnchorX, float& outAnchorY) const noexcept;
    void drawZoneTag(void* dl, std::uint64_t labelKey,
                     float anchorX, float anchorY,
                     float preferOffsetX, float preferOffsetY,
                     const char* text, ZoneColor zoneColor,
                     std::vector<PlacedBox>& placed,
                     float scale, double nowWallS, float dtS,
                     bool drawLeader = true) const;

    std::shared_ptr<ViewerControls> controls_;
    ipc::TacticalOverlayFrameV1 frame_;
    bool hasFrame_ = false;
    std::vector<sim::VehicleState> states_;
    std::vector<unsigned char> alive_;
    std::vector<MonitorGui::VehicleMeta> meta_;
    int selectedSlot_ = -1;
    int lastSelectedSlot_ = -1;

    vsg::dmat4 view_{};
    vsg::dmat4 proj_{};
    vsg::dvec3 eye_{};
    std::uint32_t width_ = 1920;
    std::uint32_t height_ = 1080;
    double simTime_ = 0.0;
    mutable double lastDrawMs_ = 0.0;
    mutable double lastRecordWallS_ = 0.0;
    mutable std::unordered_map<std::uint64_t, SmoothLabelState> labelStates_;
};

} // namespace fsim::ui
