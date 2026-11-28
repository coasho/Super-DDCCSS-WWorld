#pragma once

// VIS-M14: Toggleable 3-D Tactical & EMSO Overlays, Mil-Spec Fighter Cockpit HUD,
// and Real-Time Vulkan Picture-in-Picture (PiP) Inset Panel.
// Strictly read-only presentation of native SDK TacticalOverlayFrameV1 snapshots;
// zero simulation math is duplicated here. Zero CPU/GPU draw cost when disabled.

#include "foundation/core/Span.h"
#include "foundation/transport/TacticalOverlaySegmentV1.h"
#include "fsim/VehicleState.h"
#include "visualization/scene/CameraController.h"
#include "visualization/ui/MonitorGui.h"
#include "visualization/ui/ViewerControls.h"

#include <vsg/all.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace fsim::ui {

class TacticalOverlays : public vsg::Inherit<vsg::Command, TacticalOverlays> {
public:
    TacticalOverlays(std::shared_ptr<ViewerControls> controls, std::uint32_t deviceID);

    /// Bind the offscreen Vulkan PiP descriptor set created by render::Viewer::enablePip().
    void setPipSource(vsg::ref_ptr<vsg::DescriptorSet> descriptorSet, std::uint32_t width, std::uint32_t height);

    /// Per-frame snapshot update on the render thread before recording ImGui.
    void update(const ipc::TacticalOverlayFrameV1* frame,
                Span<const sim::VehicleState> states,
                const std::vector<unsigned char>& alive,
                const std::vector<MonitorGui::VehicleMeta>& meta,
                int selectedSlot,
                world::CameraController::Mode cameraMode,
                const vsg::dmat4& viewMatrix,
                const vsg::dmat4& projMatrix,
                const vsg::dvec3& eyeEcef,
                std::uint32_t screenWidth,
                std::uint32_t screenHeight,
                double simTime,
                bool pipActive,
                std::string pipTitle,
                std::string pipSubtitle);

    void record(vsg::CommandBuffer& cb) const override;

    /// Last frame's overlay CPU draw time (ms) for --stats reporting (0.0 when disabled).
    double lastDrawMs() const noexcept { return lastDrawMs_; }

private:
    bool projectPoint(const vsg::dvec3& ecef, float& sx, float& sy) const noexcept;
    void drawLine3D(void* drawList, const vsg::dvec3& a, const vsg::dvec3& b, std::uint32_t col, float thickness) const;
    void drawDashedLine3D(void* drawList, const vsg::dvec3& a, const vsg::dvec3& b, std::uint32_t col, float thickness, int segments) const;
    void drawRing3D(void* drawList, const vsg::dvec3& center, const vsg::dvec3& axisU, const vsg::dvec3& axisV,
                    double radiusU, double radiusV, int segments, std::uint32_t col, float thickness, bool dashed = false) const;
    void drawArc3D(void* drawList, const vsg::dvec3& origin, const vsg::dvec3& fwd, const vsg::dvec3& right,
                   double radiusM, double azStart, double azEnd, int segments, std::uint32_t col, float thickness, bool dashed = false) const;

    void drawRadarVolumes(void* dl) const;
    void drawJammerLobes(void* dl) const;
    void drawEsmRays(void* dl) const;
    void drawLinkPulses(void* dl) const;
    void drawWeaponCues(void* dl) const;
    void drawSwaths(void* dl) const;
    void drawCockpitHud(void* dl) const;
    void drawPipPanel() const;

    std::shared_ptr<ViewerControls> controls_;
    std::uint32_t deviceID_ = 0;

    vsg::ref_ptr<vsg::DescriptorSet> pipDescriptorSet_;
    std::uint32_t pipWidth_ = 480;
    std::uint32_t pipHeight_ = 270;

    ipc::TacticalOverlayFrameV1 frame_;
    bool hasFrame_ = false;
    std::vector<sim::VehicleState> states_;
    std::vector<unsigned char> alive_;
    std::vector<MonitorGui::VehicleMeta> meta_;
    int selectedSlot_ = -1;
    world::CameraController::Mode cameraMode_ = world::CameraController::Mode::Orbit;

    vsg::dmat4 view_{};
    vsg::dmat4 proj_{};
    vsg::dmat4 viewProj_{};
    vsg::dvec3 eye_{};
    std::uint32_t width_ = 1920;
    std::uint32_t height_ = 1080;
    double simTime_ = 0.0;

    bool pipActive_ = false;
    std::string pipTitle_;
    std::string pipSubtitle_;
    mutable double lastDrawMs_ = 0.0;
};

} // namespace fsim::ui
